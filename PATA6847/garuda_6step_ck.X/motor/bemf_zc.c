/**
 * @file bemf_zc.c
 * @brief Digital BEMF zero-crossing detection using ATA6847 comparators.
 *
 * The ATA6847 has built-in BEMF comparators with digital outputs on
 * RC6 (Phase A), RC7 (Phase B), RD10 (Phase C). GDUCR1.BEMFEN=1
 * enables these comparators.
 *
 * Detection modes:
 *   FEATURE_IC_ZC=1: SCCP1 fast poll timer (100kHz) with adaptive deglitch.
 *     Primary ZC detection during closed-loop. SCCP4-based blanking (640ns).
 *     ADC ISR 20kHz poll as backup. Timer1 handles timeout/forced steps.
 *   FEATURE_IC_ZC=0: ADC ISR polling at 20kHz (every PWM cycle).
 *
 * During OL_RAMP, always uses ADC ISR polling (BEMF too weak for fast poll,
 * forced commutation handles timing).
 */

#include "bemf_zc.h"
#include "commutation.h"
#include "../garuda_config.h"
#include "../hal/port_config.h"
#if FEATURE_IC_ZC
#include "../hal/hal_ic.h"
#include "../hal/hal_com_timer.h"
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
    pData->icZc.activeChannel = 0xFF;
    pData->icZc.pollFilter = 0;
    pData->icZc.filterLevel = ZC_DEGLITCH_MAX;
    pData->icZc.blankingEndHR = 0;
    pData->icZc.lastCommHR = 0;
    pData->icZc.zcCandidateHR = 0;
    pData->icZc.zcCandidateT1 = 0;
    pData->icZc.diagAccepted = 0;
    pData->icZc.diagLcoutAccepted = 0;
    pData->icZc.diagFalseZc = 0;
    pData->icZc.diagPollCycles = 0;
    HAL_ZcTimer_Stop();
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

#if FEATURE_IC_ZC
    pData->timing.lastZcTickHR = 0;
    pData->timing.prevZcTickHR = 0;
    pData->timing.zcIntervalHR = 0;
    pData->timing.prevZcIntervalHR = 0;
    pData->timing.stepPeriodHR = 0;
    pData->timing.hasPrevZcHR = false;
#endif
}

#if FEATURE_IC_ZC
/**
 * @brief Compute adaptive deglitch filter level from step period.
 * Scales linearly: ZC_DEGLITCH_MIN at Tp <= FAST, ZC_DEGLITCH_MAX at Tp >= SLOW.
 */
static inline uint8_t ComputeFilterLevel(uint16_t stepPeriod)
{
    if (stepPeriod <= ZC_DEGLITCH_FAST_TP)
        return ZC_DEGLITCH_MIN;
    if (stepPeriod >= ZC_DEGLITCH_SLOW_TP)
        return ZC_DEGLITCH_MAX;
    return ZC_DEGLITCH_MIN +
        (uint8_t)((uint16_t)(stepPeriod - ZC_DEGLITCH_FAST_TP) *
                  (ZC_DEGLITCH_MAX - ZC_DEGLITCH_MIN) /
                  (ZC_DEGLITCH_SLOW_TP - ZC_DEGLITCH_FAST_TP));
}
#endif

/**
 * @brief Called after each commutation step to prepare for next ZC detection.
 * Resets filter, sets up expected comparator transition direction,
 * and configures blanking.
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

#if FEATURE_IC_ZC
    /* Cancel any pending hardware commutation timer (forced step preempts) */
    HAL_ComTimer_Cancel();
#endif

    /* Set forced commutation timeout: ZC_TIMEOUT_MULT * stepPeriod.
     * When stepPeriod is clamped at MIN_CL_STEP_PERIOD (Timer1 quantization
     * limit), use stepPeriodHR to derive a more accurate timeout.
     * HR ticks → Timer1 ticks: t1 = hrTicks × 8 / 625 (640ns → 50µs). */
    {
        uint16_t timeoutT1;
#if FEATURE_IC_ZC
        if (pData->state == ESC_CLOSED_LOOP &&
            pData->timing.stepPeriodHR > 0 &&
            pData->timing.hasPrevZcHR)
        {
            /* Convert HR step period to Timer1 ticks, then apply multiplier.
             * This gives accurate timeout even when Tp is clamped. */
            uint32_t spT1 = ((uint32_t)pData->timing.stepPeriodHR *
                             COM_TIMER_T1_DENOM + COM_TIMER_T1_NUMER / 2) /
                            COM_TIMER_T1_NUMER;
            if (spT1 < 1) spT1 = 1;
            uint32_t tout = spT1 * ZC_TIMEOUT_MULT;
            timeoutT1 = (tout > 65535U) ? 65535U : (uint16_t)tout;
        }
        else
#endif
        {
            timeoutT1 = (uint16_t)(pData->timing.stepPeriod * ZC_TIMEOUT_MULT);
        }
        if (timeoutT1 < 2) timeoutT1 = 2;
        pData->timing.forcedCountdown = timeoutT1;
    }

#if FEATURE_IC_ZC
    if (pData->state == ESC_CLOSED_LOOP)
    {
        /* Record commutation time in SCCP4 ticks for HR blanking */
        pData->icZc.lastCommHR = HAL_ComTimer_ReadTimer();

        /* Compute blanking end in SCCP4 ticks (640 ns resolution).
         * blanking = stepPeriodHR * ZC_BLANKING_PERCENT / 100.
         * Falls back to Timer1-derived estimate if HR not yet valid. */
        uint16_t blankHR;
        if (pData->timing.stepPeriodHR > 0 &&
            pData->timing.stepPeriod < HR_MAX_STEP_PERIOD)
        {
            blankHR = (uint16_t)(
                (uint32_t)pData->timing.stepPeriodHR * ZC_BLANKING_PERCENT / 100);
        }
        else
        {
            /* Convert Timer1 blanking to SCCP4 ticks */
            uint16_t blankT1 = (uint16_t)(
                (uint32_t)pData->timing.stepPeriod * ZC_BLANKING_PERCENT / 100);
            if (blankT1 < 2) blankT1 = 2;
            blankHR = (uint16_t)((uint32_t)blankT1 *
                                  COM_TIMER_T1_NUMER / COM_TIMER_T1_DENOM);
        }
        if (blankHR < 39) blankHR = 39;  /* Minimum ~25µs blanking (was 50µs).
                                          * 25µs > half a PWM cycle (25µs at 20kHz).
                                          * At TpHR=232 (67k eRPM, 148µs step), old
                                          * 50µs floor ate 34% of the step leaving
                                          * only 2-3 polls before ZC. Now 20% blanking
                                          * applies correctly at high speed. */

        pData->icZc.blankingEndHR = pData->icZc.lastCommHR + blankHR;
        pData->icZc.activeChannel = step->floatingPhase;
        pData->icZc.pollFilter = 0;
        /* Use the larger of IIR stepPeriod and latest ZC interval for FL.
         * During decel, IIR lags — raw interval is more current and ensures
         * FL re-increases promptly instead of staying stuck at the min. */
        {
            uint16_t flTp = pData->timing.stepPeriod;
            if (pData->timing.zcInterval > flTp)
                flTp = pData->timing.zcInterval;
            pData->icZc.filterLevel = ComputeFilterLevel(flTp);
        }
        pData->icZc.phase = IC_ZC_BLANKING;
    }
#endif
}

/**
 * @brief Poll the BEMF comparator for zero-crossing.
 * Called from ADC ISR at 20 kHz. Handles blanking and digital filtering.
 * Used during OL_RAMP (always) and CL (as backup when FEATURE_IC_ZC=1).
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
         * instead of restarting the entire filter. */
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

            /* REJECT multi-step intervals from IIR (AM32-inspired). */
            uint16_t maxInterval = pData->timing.stepPeriod +
                                   (pData->timing.stepPeriod >> 1); /* 1.5× */
            if (maxInterval < 12) maxInterval = 12;

            if (pData->timing.zcInterval > 0 &&
                pData->timing.zcInterval <= maxInterval &&
                pData->timing.zcInterval < 10000)
            {
                if (pData->timing.prevZcInterval > 0 &&
                    pData->timing.prevZcInterval <= maxInterval)
                {
                    uint16_t avgInterval =
                        (pData->timing.zcInterval +
                         pData->timing.prevZcInterval) / 2;
                    pData->timing.stepPeriod =
                        (pData->timing.stepPeriod * 3 + avgInterval) >> 2;
                }
                else
                {
                    pData->timing.stepPeriod =
                        (pData->timing.stepPeriod * 3 +
                         pData->timing.zcInterval) >> 2;
                }
            }
            /* Floor: prevent runaway IIR */
            if (pData->timing.stepPeriod < MIN_CL_STEP_PERIOD)
                pData->timing.stepPeriod = MIN_CL_STEP_PERIOD;
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

    if (pData->timing.forcedCountdown > 0)
    {
        pData->timing.forcedCountdown--;
        if (pData->timing.forcedCountdown == 0)
        {
            /* Timeout — no ZC detected within allowed window.
             * Soft penalty: halve goodZcCount instead of zeroing. */
            pData->timing.consecutiveMissedSteps++;
            pData->timing.goodZcCount >>= 1;

            if (pData->timing.consecutiveMissedSteps >= ZC_DESYNC_THRESH)
                pData->timing.zcSynced = false;

            if (pData->timing.consecutiveMissedSteps >= ZC_MISS_LIMIT)
                return ZC_TIMEOUT_DESYNC;

            /* Decelerate forced commutation when desynced.
             * Without this, forced steps keep firing at the stale high-speed
             * stepPeriod while the motor decelerates, producing false ZCs
             * that prevent recovery. Increase stepPeriod by 12.5% per forced
             * step so forced comm slows down to match actual motor speed. */
            if (!pData->timing.zcSynced)
            {
                uint16_t inc = pData->timing.stepPeriod >> 3;
                if (inc < 1) inc = 1;
                pData->timing.stepPeriod += inc;
                if (pData->timing.stepPeriod > INITIAL_STEP_PERIOD)
                    pData->timing.stepPeriod = INITIAL_STEP_PERIOD;
            }

            return ZC_TIMEOUT_FORCE_STEP;
        }
    }

    return ZC_TIMEOUT_NONE;
}

/**
 * @brief Schedule the next commutation based on ZC timing.
 * Called after ZC detection (fast poll, ADC poll, or backup).
 *
 * Uses the IIR-averaged stepPeriod for scheduling. This gives consistent
 * commutation timing even when individual ZC intervals alternate due to
 * comparator threshold offset from true neutral.
 *
 * Timing advance: linear by eRPM.
 */
void BEMF_ZC_ScheduleCommutation(volatile GARUDA_DATA_T *pData)
{
    /* Guard: don't re-schedule if a commutation is already pending. */
    if (pData->timing.deadlineActive)
        return;

    uint16_t sp = pData->timing.stepPeriod;

    /* Acceleration tracking: use minimum of IIR and latest 2-step average. */
    if (pData->timing.zcInterval > 0 && pData->timing.prevZcInterval > 0)
    {
        uint16_t avgInterval =
            (pData->timing.zcInterval + pData->timing.prevZcInterval) / 2;
        if (avgInterval > 0 && avgInterval < sp)
            sp = avgInterval;
    }

    /* Compute eRPM for timing advance lookup.
     * Prefer HR when available (78x better resolution than Timer1). */
    uint32_t eRPM;
#if FEATURE_IC_ZC
    if (pData->timing.stepPeriodHR > 0 && pData->timing.hasPrevZcHR)
    {
        /* eRPM = 15625000 / stepPeriodHR (from 640ns tick base) */
        eRPM = 15625000UL / pData->timing.stepPeriodHR;
    }
    else
#endif
    {
        #define ERPM_CONST_SCHED ((uint32_t)TIMER1_FREQ_HZ * 10UL)
        eRPM = ERPM_CONST_SCHED / sp;
    }

    /* Linear timing advance by eRPM */
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

    /* Delay = sp * (30 - advDeg) / 60 */
    uint16_t delay = (uint16_t)((uint32_t)sp * (30 - advDeg) / 60);
    if (delay < 1) delay = 1;

#if FEATURE_IC_ZC
    /* Hardware com timer with high-resolution timing. */
    if (pData->state == ESC_CLOSED_LOOP)
    {
        if (pData->timing.hasPrevZcHR &&
            pData->timing.stepPeriodHR > 0 &&
            pData->timing.stepPeriod < HR_MAX_STEP_PERIOD)
        {
            /* True HR path: delay computed directly in SCCP4 ticks. */
            uint16_t spHR = pData->timing.stepPeriodHR;
            if (pData->timing.zcIntervalHR > 0 &&
                pData->timing.prevZcIntervalHR > 0)
            {
                uint16_t avgHR =
                    (pData->timing.zcIntervalHR +
                     pData->timing.prevZcIntervalHR) / 2;
                if (avgHR > 0 && avgHR < spHR)
                    spHR = avgHR;
            }

            uint16_t delayHR = (uint16_t)(
                (uint32_t)spHR * (30 - advDeg) / 60);
            if (delayHR < 32) delayHR = 32;  /* Min ~20µs (fast poll, no CLC latency) */

            uint16_t targetHR = pData->timing.lastZcTickHR + delayHR;

            /* Check if target is in the past.
             * At high speed (TpHR < 200), filter + compute latency can
             * exceed the ZC-to-commutation delay. If we miss the window,
             * commutate ASAP via hardware timer rather than falling through
             * to the imprecise Timer1 path (50µs jitter at Tp:2). */
            int16_t margin = (int16_t)(targetHR - HAL_ComTimer_ReadTimer());
            if (margin <= 0)
                targetHR = HAL_ComTimer_ReadTimer() + 2;  /* Fire ASAP */
            HAL_ComTimer_ScheduleAbsolute(targetHR);
            pData->timing.commDeadline = pData->timing.lastZcTick + delay;
            pData->timing.deadlineActive = true;
            return;
        }
        else
        {
            /* HR not yet valid — use Timer1→SCCP4 conversion (coarse). */
            uint16_t elapsed = pData->timer1Tick - pData->timing.lastZcTick;
            uint16_t remaining = (delay > elapsed) ? (delay - elapsed) : 1;
            uint32_t delayCT = (uint32_t)remaining *
                               COM_TIMER_T1_NUMER / COM_TIMER_T1_DENOM;
            if (delayCT > 0 && delayCT <= 65535U)
            {
                uint16_t targetHR = HAL_ComTimer_ReadTimer() + (uint16_t)delayCT;
                HAL_ComTimer_ScheduleAbsolute(targetHR);
                pData->timing.commDeadline = pData->timing.lastZcTick + delay;
                pData->timing.deadlineActive = true;
                return;
            }
        }
    }
#endif

    pData->timing.commDeadline = pData->timing.lastZcTick + delay;
    pData->timing.deadlineActive = true;
}

/* ── SCCP1 Fast Poll ZC Detection ──────────────────────────────────── */
#if FEATURE_IC_ZC

/**
 * @brief Record ZC timing and update step period IIR.
 * Tracks both Timer1 (50µs) and SCCP4 HR (640ns) timestamps.
 */
static void RecordZcTiming(volatile GARUDA_DATA_T *pData,
                           uint16_t zcTick, uint16_t hrTick)
{
    if (pData->timing.hasPrevZc)
    {
        uint16_t interval = zcTick - pData->timing.lastZcTick;

        /* Reject impossibly short intervals — PWM noise, not real BEMF.
         * Floor must be below stepPeriod to accept valid ZCs at high speed.
         * At Tp:5, Timer1 intervals are 4-5 ticks. Beyond Tp:5 (67k eRPM),
         * intervals drop to 2-3 ticks. Floor of 2 accepts all valid ZCs
         * while the deglitch filter handles noise rejection. */
        uint16_t minInterval = pData->timing.stepPeriod / 2;
        if (minInterval < 2) minInterval = 2;
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

        /* REJECT multi-step intervals from IIR — AM32-inspired. */
        uint16_t maxInterval = pData->timing.stepPeriod +
                               (pData->timing.stepPeriod >> 1); /* 1.5× */
        if (maxInterval < 12) maxInterval = 12;

        if (interval <= maxInterval)
        {
            if (interval > 0 && interval < 10000 &&
                pData->timing.prevZcInterval > 0 &&
                pData->timing.prevZcInterval <= maxInterval)
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

        /* Floor: prevent step period from going below physical limit.
         * If IIR tracks below MIN_CL_STEP_PERIOD, it's accepting false
         * ZCs at a rate the motor can't physically achieve. */
        if (pData->timing.stepPeriod < MIN_CL_STEP_PERIOD)
            pData->timing.stepPeriod = MIN_CL_STEP_PERIOD;

        /* High-resolution step period tracking via SCCP4 timer. */
        if (pData->timing.stepPeriod < HR_MAX_STEP_PERIOD &&
            pData->timing.hasPrevZcHR)
        {
            uint16_t intervalHR = hrTick - pData->timing.lastZcTickHR;

            pData->timing.prevZcTickHR = pData->timing.lastZcTickHR;
            pData->timing.lastZcTickHR = hrTick;
            pData->timing.prevZcIntervalHR = pData->timing.zcIntervalHR;
            pData->timing.zcIntervalHR = intervalHR;

            uint16_t maxHR = pData->timing.stepPeriodHR +
                             (pData->timing.stepPeriodHR >> 1);
            if (maxHR < 100) maxHR = 100;

            if (intervalHR <= maxHR)
            {
                if (intervalHR > 0 && pData->timing.prevZcIntervalHR > 0 &&
                    pData->timing.prevZcIntervalHR <= maxHR)
                {
                    uint16_t avgHR =
                        (intervalHR + pData->timing.prevZcIntervalHR) / 2;
                    pData->timing.stepPeriodHR = (uint16_t)(
                        ((uint32_t)pData->timing.stepPeriodHR * 3 + avgHR) >> 2);
                }
                else if (intervalHR > 0)
                {
                    pData->timing.stepPeriodHR = (uint16_t)(
                        ((uint32_t)pData->timing.stepPeriodHR * 3 + intervalHR) >> 2);
                }
            }
        }
        else
        {
            pData->timing.lastZcTickHR = hrTick;
        }
        pData->timing.hasPrevZcHR = true;
    }
    else
    {
        /* First ZC — just record timestamps */
        pData->timing.lastZcTick = zcTick;
        pData->timing.lastZcTickHR = hrTick;
        pData->timing.stepsSinceLastZc = 0;
        pData->timing.hasPrevZcHR = false;
    }
    pData->timing.hasPrevZc = true;

    pData->timing.goodZcCount++;
    pData->timing.consecutiveMissedSteps = 0;

    if (pData->timing.goodZcCount >= ZC_SYNC_THRESHOLD)
        pData->timing.zcSynced = true;
}

/**
 * @brief Fast poll ZC detection — called from SCCP1 timer ISR at 100kHz.
 *
 * State machine:
 *   BLANKING: check SCCP4 timestamp against blankingEndHR → ARMED
 *   ARMED:    read comparator, adaptive deglitch filter → DONE
 *   DONE:     idle (ZC accepted, wait for next commutation)
 *
 * Deglitch: consecutive matching reads in expected state. First match
 * records the SCCP4 timestamp as ZC candidate (eliminates filter delay
 * from timing). On mismatch, filter resets to 0.
 *
 * @return true when ZC confirmed (caller schedules commutation)
 */
bool BEMF_ZC_FastPoll(volatile GARUDA_DATA_T *pData)
{
    if (pData->bemf.zeroCrossDetected)
        return false;

    pData->icZc.diagPollCycles++;

    /* BLANKING → ARMED transition using SCCP4 HR timestamps (640ns) */
    if (pData->icZc.phase == IC_ZC_BLANKING)
    {
        int16_t dt = (int16_t)(HAL_ComTimer_ReadTimer() -
                               pData->icZc.blankingEndHR);
        if (dt < 0)
            return false;  /* Still blanking */

        pData->icZc.phase = IC_ZC_ARMED;
        pData->icZc.pollFilter = 0;
    }

    if (pData->icZc.phase != IC_ZC_ARMED)
        return false;

    /* Read comparator for the floating phase */
    uint8_t cmp = ReadBEMFComparator(pData->icZc.activeChannel);

    if (cmp == pData->bemf.cmpExpected)
    {
        if (pData->icZc.pollFilter == 0)
        {
            /* First matching read — record candidate ZC timestamp.
             * This is the closest approximation of the actual ZC time.
             * Using this instead of the confirmation time eliminates
             * the deglitch filter delay from commutation scheduling. */
            pData->icZc.zcCandidateHR = HAL_ComTimer_ReadTimer();
            pData->icZc.zcCandidateT1 = pData->timer1Tick;
        }
        pData->icZc.pollFilter++;

        if (pData->icZc.pollFilter >= pData->icZc.filterLevel)
        {
            /* ZC confirmed! */
            pData->bemf.zeroCrossDetected = true;
            RecordZcTiming(pData, pData->icZc.zcCandidateT1,
                           pData->icZc.zcCandidateHR);

            if (pData->bemf.zeroCrossDetected)
            {
                pData->icZc.diagAccepted++;
                pData->icZc.phase = IC_ZC_DONE;
                return true;
            }
            /* RecordZcTiming rejected (too-short interval) — reset */
            pData->icZc.pollFilter = 0;
        }
    }
    else
    {
        /* Mismatch — bounce-tolerant decrement instead of hard reset.
         * A single noise spike costs 2 polls (one lost + one to recover)
         * instead of throwing away the entire filter. Critical at Tp:5
         * where only ~20 polls are available after blanking. */
        if (pData->icZc.pollFilter > 0)
            pData->icZc.pollFilter--;
    }

    return false;
}

#endif /* FEATURE_IC_ZC */
