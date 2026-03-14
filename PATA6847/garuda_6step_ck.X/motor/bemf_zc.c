/**
 * @file bemf_zc.c
 * @brief Digital BEMF zero-crossing detection using ATA6847 comparators.
 *
 * The ATA6847 has built-in BEMF comparators with digital outputs on
 * RC6 (Phase A), RC7 (Phase B), RD10 (Phase C). GDUCR1.BEMFEN=1
 * enables these comparators.
 *
 * Detection logic:
 *   1. After commutation, blank for ZC_BLANKING_PERCENT of step period
 *   2. Read the digital comparator output for the floating phase
 *   3. Detect transition (rising or falling per commutation table polarity)
 *   4. Filter: require ZC_FILTER_THRESHOLD consecutive matching reads
 *   5. On confirmed ZC, schedule next commutation at ZC + half step period
 *   6. Track sync: after ZC_SYNC_THRESHOLD good ZCs, motor is synchronized
 *
 * Called from ADC ISR at 20 kHz (every PWM cycle).
 */

#include "bemf_zc.h"
#include "commutation.h"
#include "../garuda_config.h"
#include "../hal/port_config.h"
#if FEATURE_IC_ZC
#include "../hal/hal_ic.h"
#endif

/**
 * @brief Read the digital BEMF comparator output for a given phase.
 * @param phase 0=A, 1=B, 2=C
 * @return 1 if comparator output is high, 0 if low
 */
static inline uint8_t ReadBEMFComparator(uint8_t phase)
{
    switch (phase)
    {
        case 0: return BEMF_A_GetValue() ? 1 : 0;
        case 1: return BEMF_B_GetValue() ? 1 : 0;
        case 2: return BEMF_C_GetValue() ? 1 : 0;
        default: return 0;
    }
}

void BEMF_ZC_Init(volatile GARUDA_DATA_T *pData)
{
    pData->bemf.zeroCrossDetected = false;
    pData->bemf.cmpPrev = 0xFF;  /* Unknown */
    pData->bemf.cmpExpected = 0;
    pData->bemf.filterCount = 0;

#if FEATURE_IC_ZC
    pData->icZc.phase = IC_ZC_BLANKING;
    pData->icZc.blankingEndTick = 0;
    pData->icZc.captureTick = 0;
    pData->icZc.captureTimer1 = 0;
    pData->icZc.guardCountdown = 0;
    pData->icZc.activeChannel = 0xFF;
    pData->icZc.diagAccepted = 0;
    pData->icZc.diagChatter = 0;
    pData->icZc.diagCaptures = 0;
    pData->icZc.diagFalseZc = 0;
    HAL_IC_DisableAll();
#endif

    pData->timing.stepPeriod = INITIAL_STEP_PERIOD;
    pData->timing.lastCommTick = 0;
    pData->timing.lastZcTick = 0;
    pData->timing.prevZcTick = 0;
    pData->timing.zcInterval = 0;
    pData->timing.prevZcInterval = 0;
    pData->timing.commDeadline = 0;
    pData->timing.forcedCountdown = 0;
    pData->timing.goodZcCount = 0;
    pData->timing.consecutiveMissedSteps = 0;
    pData->timing.stepsSinceLastZc = 0;
    pData->timing.zcSynced = false;
    pData->timing.deadlineActive = false;
    pData->timing.hasPrevZc = false;
}

/**
 * @brief Called after each commutation step to prepare for next ZC detection.
 * Resets filter, sets up expected comparator transition direction,
 * and starts the blanking window.
 */
void BEMF_ZC_OnCommutation(volatile GARUDA_DATA_T *pData)
{
    const COMMUTATION_STEP_T *step = &commutationTable[pData->currentStep];

    pData->bemf.zeroCrossDetected = false;
    pData->bemf.filterCount = 0;
    pData->bemf.cmpPrev = 0xFF;  /* Force re-read after blanking */

    /* Expected post-ZC comparator state:
     * Rising ZC (+1): comparator goes 0→1, so we expect 1
     * Falling ZC (-1): comparator goes 1→0, so we expect 0 */
    pData->bemf.cmpExpected = (step->zcPolarity > 0) ? 1 : 0;

    pData->timing.lastCommTick = pData->timer1Tick;
    pData->timing.stepsSinceLastZc++;
    pData->timing.deadlineActive = false;

    /* Set forced commutation timeout: ZC_TIMEOUT_MULT * stepPeriod */
    pData->timing.forcedCountdown =
        (uint16_t)(pData->timing.stepPeriod * ZC_TIMEOUT_MULT);

#if FEATURE_IC_ZC
    /* IC only active during CL — during OL_RAMP, polling handles ZC.
     * BEMF is too weak during ramp for clean IC edges; polling's 3-sample
     * filter rejects PWM noise that IC would capture as false ZCs. */
    if (pData->state == ESC_CLOSED_LOOP)
    {
        if (pData->icZc.activeChannel < 3)
            HAL_IC_Disable(pData->icZc.activeChannel);

        pData->icZc.phase = IC_ZC_BLANKING;
        pData->icZc.activeChannel = step->floatingPhase;
        pData->icZc.guardCountdown = 0;

        uint16_t blankingTicks = (uint16_t)(
            (uint32_t)pData->timing.stepPeriod * ZC_BLANKING_PERCENT / 100);
        if (blankingTicks < 3) blankingTicks = 3;
        pData->icZc.blankingEndTick = pData->timer1Tick + blankingTicks;
    }
#endif
}

/**
 * @brief Poll the BEMF comparator for zero-crossing.
 * Called from ADC ISR at 20 kHz. Handles blanking and digital filtering.
 * Used during OL_RAMP (always) and CL (when FEATURE_IC_ZC=0).
 *
 * @return true when a valid zero-crossing is detected
 */
bool BEMF_ZC_Poll(volatile GARUDA_DATA_T *pData)
{
    if (pData->bemf.zeroCrossDetected)
        return false;  /* Already detected this step */

    /* Blanking: skip ZC_BLANKING_PERCENT of step period after commutation.
     * Both Timer1 and ADC ISR run at 20 kHz (50 µs). */
    uint16_t ticksSinceComm = pData->timer1Tick - pData->timing.lastCommTick;
    uint16_t blankingTicks = (uint16_t)(
        (uint32_t)pData->timing.stepPeriod * ZC_BLANKING_PERCENT / 100);
    if (blankingTicks < 1) blankingTicks = 1;

    if (ticksSinceComm < blankingTicks)
        return false;  /* Still in blanking window */

    /* Read the comparator for the floating phase */
    const COMMUTATION_STEP_T *step = &commutationTable[pData->currentStep];
    uint8_t cmpNow = ReadBEMFComparator(step->floatingPhase);

    /* First read after blanking — initialize previous state */
    if (pData->bemf.cmpPrev == 0xFF)
    {
        pData->bemf.cmpPrev = cmpNow;

        /* If comparator is already in expected state after blanking,
         * the ZC transition occurred DURING the blanking window.
         * Start the filter immediately so we don't deadlock waiting
         * for a transition that already happened. */
        if (cmpNow == pData->bemf.cmpExpected)
            pData->bemf.filterCount = 1;

        return false;
    }

    /* Check for transition toward expected state */
    if (cmpNow == pData->bemf.cmpExpected && cmpNow != pData->bemf.cmpPrev)
    {
        /* Transition detected — start filtering */
        pData->bemf.filterCount = 1;
    }
    else if (cmpNow == pData->bemf.cmpExpected && pData->bemf.filterCount > 0)
    {
        /* Consistent with expected state — accumulate filter */
        pData->bemf.filterCount++;
    }
    else if (cmpNow != pData->bemf.cmpExpected)
    {
        /* Bounce tolerance: decrement instead of reset.
         * A single noise spike costs 2 samples (one lost + one to recover)
         * instead of restarting the entire filter. This handles BEMF
         * comparator noise at mid-speed where PWM switching creates
         * transients near the zero-crossing threshold. */
        if (pData->bemf.filterCount > 0)
            pData->bemf.filterCount--;
    }

    pData->bemf.cmpPrev = cmpNow;

    /* ZC confirmed when filter threshold is met */
    if (pData->bemf.filterCount >= ZC_FILTER_THRESHOLD)
    {
        pData->bemf.zeroCrossDetected = true;

        /* Record ZC timing */
        pData->timing.prevZcTick = pData->timing.lastZcTick;
        pData->timing.lastZcTick = pData->timer1Tick;
        pData->timing.stepsSinceLastZc = 0;

        /* Compute ZC interval for step period estimation */
        if (pData->timing.hasPrevZc)
        {
            pData->timing.prevZcInterval = pData->timing.zcInterval;
            pData->timing.zcInterval =
                pData->timing.lastZcTick - pData->timing.prevZcTick;

            /* 2-step averaging cancels the alternating short/long pattern
             * caused by BEMF comparator threshold offset from true neutral.
             * Rising-edge ZCs fire early, falling-edge late (or vice versa),
             * creating a period-2 oscillation that a per-step IIR can't filter.
             * Averaging consecutive pairs eliminates this perfectly.
             * Then IIR smooths remaining noise: 3/4 old + 1/4 new. */
            if (pData->timing.zcInterval > 0 &&
                pData->timing.zcInterval < 10000 &&
                pData->timing.prevZcInterval > 0)
            {
                uint16_t avgInterval =
                    (pData->timing.zcInterval + pData->timing.prevZcInterval) / 2;
                pData->timing.stepPeriod =
                    (pData->timing.stepPeriod * 3 + avgInterval) >> 2;
            }
            else if (pData->timing.zcInterval > 0 &&
                     pData->timing.zcInterval < 10000)
            {
                /* First valid interval — no pair yet, use single value */
                pData->timing.stepPeriod =
                    (pData->timing.stepPeriod * 3 + pData->timing.zcInterval) >> 2;
            }
        }
        pData->timing.hasPrevZc = true;

        /* Count consecutive good ZC events */
        pData->timing.goodZcCount++;
        pData->timing.consecutiveMissedSteps = 0;

        if (pData->timing.goodZcCount >= ZC_SYNC_THRESHOLD)
            pData->timing.zcSynced = true;

        return true;
    }

    return false;
}

/**
 * @brief Check for ZC timeout and handle forced commutation.
 * Called from Timer1 ISR. Decrements forced countdown.
 *
 * @return ZC_TIMEOUT_NONE, ZC_TIMEOUT_FORCE_STEP, or ZC_TIMEOUT_DESYNC
 */
ZC_TIMEOUT_RESULT_T BEMF_ZC_CheckTimeout(volatile GARUDA_DATA_T *pData)
{
    if (pData->bemf.zeroCrossDetected)
        return ZC_TIMEOUT_NONE;

    if (pData->timing.deadlineActive)
        return ZC_TIMEOUT_NONE;  /* Waiting for scheduled commutation */

    /* Decrement the timeout counter.
     * This is called from Timer1 ISR at 20 kHz (same rate as forcedCountdown units). */
    if (pData->timing.forcedCountdown > 0)
    {
        pData->timing.forcedCountdown--;
        if (pData->timing.forcedCountdown == 0)
        {
            /* Timeout — no ZC detected within allowed window.
             * Soft penalty: halve goodZcCount instead of zeroing.
             * A single missed step shouldn't destroy sync confidence
             * and slam duty to idle — that causes worse roughness
             * than the missed step itself. Only clear zcSynced after
             * multiple consecutive misses (ZC_DESYNC_THRESH). */
            pData->timing.consecutiveMissedSteps++;
            pData->timing.goodZcCount >>= 1;  /* Halve, don't zero */

            if (pData->timing.consecutiveMissedSteps >= ZC_DESYNC_THRESH)
                pData->timing.zcSynced = false;

            if (pData->timing.consecutiveMissedSteps >= ZC_MISS_LIMIT)
                return ZC_TIMEOUT_DESYNC;

            return ZC_TIMEOUT_FORCE_STEP;
        }
    }

    return ZC_TIMEOUT_NONE;
}

/**
 * @brief Schedule the next commutation based on ZC timing.
 * Called after BEMF_ZC_Poll returns true.
 *
 * Uses the IIR-averaged stepPeriod for scheduling. This gives consistent
 * commutation timing even when individual ZC intervals alternate due to
 * comparator threshold offset from true neutral. The 2-step averaging
 * in the step period IIR cancels the alternation, so stepPeriod tracks
 * the true mean period.
 *
 * Timing advance: linear by eRPM (matches dspic33AKESC 6-step).
 *   0° below TIMING_ADVANCE_START_ERPM, linearly ramps to
 *   TIMING_ADVANCE_MAX_DEG at MAX_CLOSED_LOOP_ERPM.
 *   Delay = stepPeriod * (30 - advDeg) / 60.
 */
void BEMF_ZC_ScheduleCommutation(volatile GARUDA_DATA_T *pData)
{
    uint16_t sp = pData->timing.stepPeriod;

    /* Compute eRPM for timing advance lookup.
     * eRPM = TIMER1_FREQ_HZ * 10 / stepPeriod */
    #define ERPM_CONST_SCHED ((uint32_t)TIMER1_FREQ_HZ * 10UL)
    uint32_t eRPM = ERPM_CONST_SCHED / sp;

    /* Linear timing advance by eRPM:
     *   advDeg = MIN at eRPM <= TIMING_ADVANCE_START_ERPM
     *   advDeg = MAX at eRPM >= MAX_CLOSED_LOOP_ERPM
     *   linear interpolation in between */
    uint16_t advDeg = TIMING_ADVANCE_MIN_DEG;
    if (eRPM >= MAX_CLOSED_LOOP_ERPM)
    {
        advDeg = TIMING_ADVANCE_MAX_DEG;
    }
    else if (eRPM > TIMING_ADVANCE_START_ERPM)
    {
        uint32_t range = MAX_CLOSED_LOOP_ERPM - TIMING_ADVANCE_START_ERPM;
        uint32_t pos = eRPM - TIMING_ADVANCE_START_ERPM;
        advDeg = TIMING_ADVANCE_MIN_DEG +
            (uint16_t)((uint32_t)(TIMING_ADVANCE_MAX_DEG - TIMING_ADVANCE_MIN_DEG)
                       * pos / range);
    }

    /* Delay = sp * (30 - advDeg) / 60.
     * At 0° advance: delay = sp/2 (standard 30° ZC-to-commutation).
     * At 8° advance: delay = sp * 22/60. */
    uint16_t delay = (uint16_t)((uint32_t)sp * (30 - advDeg) / 60);
    if (delay < 1) delay = 1;

    pData->timing.commDeadline = pData->timing.lastZcTick + delay;
    pData->timing.deadlineActive = true;
}

/* ── SCCP Input Capture ZC Detection ──────────────────────────────── */
#if FEATURE_IC_ZC

/**
 * @brief Record ZC timing and update step period IIR.
 * @param pData ESC data
 * @param zcTick Timer1 tick at which ZC occurred
 */
static void RecordZcTiming(volatile GARUDA_DATA_T *pData, uint16_t zcTick)
{
    if (pData->timing.hasPrevZc)
    {
        uint16_t interval = zcTick - pData->timing.lastZcTick;

        /* Reject impossibly short intervals — PWM noise, not real BEMF.
         * A real motor can't accelerate more than ~2x in one step.
         * Absolute floor of 8 ticks (400µs = 25000 eRPM) for safety.
         * CRITICAL: don't update lastZcTick on rejection — keeps the
         * baseline anchored to the last good ZC so noise can't creep
         * the timestamp forward and eventually pass the check. */
        uint16_t minInterval = pData->timing.stepPeriod / 2;
        if (minInterval < 8) minInterval = 8;
        if (interval < minInterval)
        {
            pData->icZc.diagFalseZc++;
            pData->bemf.zeroCrossDetected = false;
            return;
        }

        /* Interval valid — commit timestamps */
        pData->timing.prevZcTick = pData->timing.lastZcTick;
        pData->timing.lastZcTick = zcTick;
        pData->timing.stepsSinceLastZc = 0;

        pData->timing.prevZcInterval = pData->timing.zcInterval;
        pData->timing.zcInterval = interval;

        /* 2-step averaging + IIR: same as polling path */
        if (interval > 0 && interval < 10000 &&
            pData->timing.prevZcInterval > 0)
        {
            uint16_t avgInterval =
                (interval + pData->timing.prevZcInterval) / 2;
            pData->timing.stepPeriod =
                (pData->timing.stepPeriod * 3 + avgInterval) >> 2;
        }
        else if (interval > 0 && interval < 10000)
        {
            pData->timing.stepPeriod =
                (pData->timing.stepPeriod * 3 + interval) >> 2;
        }
    }
    else
    {
        /* First ZC — just record the timestamp */
        pData->timing.lastZcTick = zcTick;
        pData->timing.stepsSinceLastZc = 0;
    }
    pData->timing.hasPrevZc = true;

    pData->timing.goodZcCount++;
    pData->timing.consecutiveMissedSteps = 0;

    if (pData->timing.goodZcCount >= ZC_SYNC_THRESHOLD)
        pData->timing.zcSynced = true;
}

/**
 * @brief Handle an IC capture event — AM32-style immediate validation.
 *
 * On each edge capture, immediately poll the comparator in a tight loop
 * (filter_level times). If ANY read doesn't match the expected post-ZC
 * state, reject and return (IC stays armed for next edge/bounce).
 * If all reads match, accept as real ZC.
 *
 * This approach embraces the ATA6847's ~32 bounces per ZC: early bounce
 * ISRs fail the polling check (comparator still oscillating), but after
 * the bounce settles (~60µs), the final ISR sees consistent state → accept.
 *
 * PWM noise: brief transient (<1µs), comparator returns to original state
 * before tight loop completes → mismatch → rejected.
 *
 * Called from SCCP ISR (priority 4).
 */
void BEMF_ZC_IC_OnCapture(volatile GARUDA_DATA_T *pData,
                           uint8_t channel, uint16_t captureTick)
{
    /* Ignore captures from non-active channels */
    if (channel != pData->icZc.activeChannel)
        return;

    if (pData->icZc.phase != IC_ZC_ARMED)
        return;

    pData->icZc.diagCaptures++;

    /* AM32-style tight polling: read comparator filter_level times.
     * At 100 MHz FCY, each read ~100ns. 24 reads ≈ 2.4µs.
     * If any read doesn't match expected state → noise, reject. */
    const COMMUTATION_STEP_T *step = &commutationTable[pData->currentStep];
    uint8_t expected = pData->bemf.cmpExpected;
    uint8_t phase = step->floatingPhase;
    uint8_t filterN = IC_ZC_FILTER_LEVEL;

    uint8_t i;
    for (i = 0; i < filterN; i++)
    {
        if (ReadBEMFComparator(phase) != expected)
        {
            /* Mismatch — noise or bounce. Return without disabling IC.
             * Next edge/bounce will trigger another ISR attempt. */
            pData->icZc.diagChatter++;
            return;
        }
    }

    /* All reads matched — real ZC confirmed.
     * Disable IC to stop further chatter ISRs this step. */
    HAL_IC_Disable(pData->icZc.activeChannel);
    pData->icZc.phase = IC_ZC_DONE;
    pData->bemf.zeroCrossDetected = true;
    pData->icZc.diagAccepted++;

    RecordZcTiming(pData, pData->timer1Tick);
}

/**
 * @brief Timer1 tick handler for IC ZC: blanking release only.
 *
 * ZC confirmation now happens immediately in OnCapture (AM32 approach).
 * Timer1 only manages blanking → arm transition.
 *
 * @return true if ZC was confirmed by IC ISR since last call
 */
bool BEMF_ZC_IC_TimerTick(volatile GARUDA_DATA_T *pData)
{
    if (pData->icZc.phase == IC_ZC_BLANKING)
    {
        int16_t dt = (int16_t)(pData->timer1Tick - pData->icZc.blankingEndTick);
        if (dt >= 0)
        {
            /* Blanking done — arm IC for the floating phase */
            const COMMUTATION_STEP_T *step =
                &commutationTable[pData->currentStep];
            bool rising = (step->zcPolarity > 0);
            HAL_IC_Arm(pData->icZc.activeChannel, rising);
            pData->icZc.phase = IC_ZC_ARMED;
        }
        return false;
    }

    /* ZC confirmed by IC ISR — schedule commutation */
    if (pData->icZc.phase == IC_ZC_DONE && pData->bemf.zeroCrossDetected)
    {
        return true;
    }

    return false;
}

#endif /* FEATURE_IC_ZC */
