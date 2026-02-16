/**
 * @file bemf_zc.c
 *
 * @brief BEMF zero-crossing detection via ADC software threshold.
 * All functions called exclusively from ADC ISR (24kHz).
 *
 * Design: Read floating phase voltage as 12-bit ADC value, compare
 * against Vbus/2 with deadband, per-phase gain/offset correction.
 * Detect edge transitions, filter for noise rejection, compute
 * commutation deadline at 30 degrees after confirmed ZC.
 *
 * Component: BEMF_ZC
 */

#include "../garuda_config.h"

#if FEATURE_BEMF_CLOSED_LOOP

#include "bemf_zc.h"
#include "commutation.h"
#include "../garuda_calc_params.h"

/* Per-phase Q15 gain + signed offset correction (shared by Poll and integration fast path) */
static const uint16_t phaseGain[3] = {
    ZC_PHASE_GAIN_A, ZC_PHASE_GAIN_B, ZC_PHASE_GAIN_C
};
static const int16_t phaseOffset[3] = {
    ZC_PHASE_OFFSET_A, ZC_PHASE_OFFSET_B, ZC_PHASE_OFFSET_C
};

/**
 * @brief Apply per-phase gain/offset correction and clamp to [0, 4095].
 * Shared by normal Poll path and integration-only fast path.
 */
static inline int32_t computeVCorrected(uint16_t vFloat, uint8_t floatPhase)
{
    int32_t v = (int32_t)(((uint32_t)vFloat * phaseGain[floatPhase]) >> 15)
              + phaseOffset[floatPhase];
    if (v < 0) v = 0;
    if (v > 4095) v = 4095;
    return v;
}

/**
 * @brief Compute blanking ticks for current step.
 * Shared by normal Poll path and integration-only fast path.
 */
static inline uint16_t computeBlankTicks(volatile GARUDA_DATA_T *pData)
{
    uint16_t sp = pData->timing.stepPeriod;
#if FEATURE_DYNAMIC_BLANKING
    uint16_t blankBase = (uint16_t)((uint32_t)sp * ZC_BLANKING_PERCENT / 100);
    uint8_t dutyPct = (uint8_t)((uint32_t)pData->duty * 100 / LOOPTIME_TCY);
    uint16_t blankDutyBoost = 0;
    if (dutyPct > ZC_DEMAG_DUTY_THRESH)
    {
        blankDutyBoost = (uint16_t)((uint32_t)sp * ZC_DEMAG_BLANK_EXTRA_PERCENT
                         * (dutyPct - ZC_DEMAG_DUTY_THRESH)
                         / (100 * (100 - ZC_DEMAG_DUTY_THRESH)));
    }
    uint16_t bt = blankBase + blankDutyBoost;
    uint16_t blankMax = sp / 4;
    if (bt > blankMax) bt = blankMax;
#else
    uint16_t bt = (uint16_t)((uint32_t)sp * ZC_BLANKING_PERCENT / 100);
#endif
    if (bt < 1) bt = 1;
    return bt;
}

/**
 * @brief Initialize ZC detection state for closed-loop entry.
 * Called once when ADC ISR first detects ESC_CLOSED_LOOP state.
 *
 * @param pData       Pointer to global ESC data
 * @param initialStepPeriod  Step period in adcIsrTick units (from OL ramp handoff)
 */
void BEMF_ZC_Init(volatile GARUDA_DATA_T *pData, uint16_t initialStepPeriod)
{
    pData->timing.stepPeriod = initialStepPeriod;
    pData->timing.lastCommTick = 0;
    pData->timing.lastZcTick = 0;
    pData->timing.prevZcTick = 0;
    pData->timing.zcInterval = 0;
    pData->timing.commDeadline = 0;
    pData->timing.forcedCountdown = initialStepPeriod;
    pData->timing.goodZcCount = 0;
    pData->timing.consecutiveMissedSteps = 0;
    pData->timing.stepsSinceLastZc = 0;
    for (uint8_t s = 0; s < 6; s++)
        pData->timing.stepMissCount[s] = 0;
    pData->timing.risingZcWorks = false;
    pData->timing.fallingZcWorks = false;
    pData->timing.zcSynced = false;
    pData->timing.deadlineActive = false;
    pData->timing.hasPrevZc = false;

    pData->bemf.zeroCrossDetected = false;
    pData->bemf.cmpPrev = 0xFF;
    pData->bemf.cmpExpected = 0;
    pData->bemf.filterCount = 0;
    pData->bemf.ad2SettleCount = 0;
    pData->bemf.bemfSampleValid = true;

    pData->zcDiag.zcConfirmedCount = 0;
    pData->zcDiag.zcTimeoutForceCount = 0;
    pData->zcDiag.zcDesyncCount = 0;
    pData->zcDiag.forcedStepPresyncCount = 0;
    pData->zcDiag.zcPollTotal = 0;
    pData->zcDiag.zcAboveTotal = 0;
    pData->zcDiag.blankSkipTotal = 0;
    pData->zcDiag.invalidSampleTotal = 0;
    pData->zcDiag.deadbandHoldTotal = 0;
    pData->zcDiag.blankTransitionTotal = 0;
    pData->zcDiag.wrongEdgeTotal = 0;
    pData->zcDiag.lastFloatAdc = 0;
    pData->zcDiag.lastZcThreshold = 0;
    pData->zcDiag.lastStateNow = 0;
    pData->zcDiag.lastCmpExpected = 0;
    pData->zcDiag.lastFloatPhase = 0;
    pData->zcDiag.lastEdgeCount = 0;
    for (uint8_t i = 0; i < 3; i++) {
        pData->zcDiag.floatAdcMin[i] = 4095;
        pData->zcDiag.floatAdcMax[i] = 0;
    }
    pData->zcDiag.zcThresholdMin = 4095;
    pData->zcDiag.zcThresholdMax = 0;
    for (uint8_t j = 0; j < 6; j++)
        pData->zcDiag.zcPerStep[j] = 0;

#if FEATURE_BEMF_INTEGRATION
    pData->integ.integral = 0;
    pData->integ.intThreshold = 1;
    pData->integ.stepDevMax = 0;
    pData->integ.shadowFireTick = 0;
    pData->integ.shadowFired = false;
    pData->integ.bemfPeakSmooth = 0;
    pData->integ.shadowVsActual = 0;
    pData->integ.shadowHitCount = 0;
    pData->integ.shadowMissCount = 0;
    pData->integ.shadowNoFireCount = 0;
    pData->integ.shadowAbsErrorSum = 0;
    pData->integ.shadowErrorSum = 0;
    pData->integ.shadowSampleCount = 0;
    pData->integ.shadowSkipCount = 0;
    pData->integ.shadowUnseededSkip = 0;
    pData->integ.shadowStepPeriodSum = 0;
    pData->timing.deadlineIsZc = false;
#endif
}

/**
 * @brief Reset ZC detection state after each commutation step.
 * Called immediately after every COMMUTATION_AdvanceStep().
 *
 * @param pData  Pointer to global ESC data
 * @param now    Current adcIsrTick value
 */
void BEMF_ZC_OnCommutation(volatile GARUDA_DATA_T *pData, uint16_t now)
{
    pData->timing.lastCommTick = now;
    pData->timing.deadlineActive = false;
    pData->timing.stepsSinceLastZc++;

    pData->bemf.zeroCrossDetected = false;
    pData->bemf.cmpPrev = 0xFF;  /* Unknown — first read will initialize */
    pData->bemf.filterCount = 0;

    /* Compute expected comparator state after ZC based on polarity:
     * Rising ZC (zcPolarity=+1): BEMF crosses Vbus/2 going up → after ZC, CMPSTAT=1
     * Falling ZC (zcPolarity=-1): BEMF crosses Vbus/2 going down → after ZC, CMPSTAT=0 */
    int8_t pol = commutationTable[pData->currentStep].zcPolarity;
    pData->bemf.cmpExpected = (pol > 0) ? 1 : 0;

#if FEATURE_BEMF_INTEGRATION
    /* IIR smooth: 7/8 old + 1/8 new. Seed on first nonzero. */
    if (pData->integ.stepDevMax > 0)
    {
        if (pData->integ.bemfPeakSmooth == 0)
            pData->integ.bemfPeakSmooth = pData->integ.stepDevMax;
        else
            pData->integ.bemfPeakSmooth = (uint16_t)(
                ((uint32_t)pData->integ.bemfPeakSmooth * 7
                 + pData->integ.stepDevMax) >> 3);
    }

    /* Reset per-step state */
    pData->integ.integral = 0;
    pData->integ.stepDevMax = 0;
    pData->integ.shadowFired = false;

    /* Threshold = peak * delayTicks / 2 * gain.
     * delayTicks = actual ZC-to-commutation window, accounting for timing
     * advance. Without advance: delayTicks = stepPeriod/2 (30 deg).
     * With advance: delayTicks = stepPeriod * (30 - advDeg) / 60.
     * This fixes the x5max noFire problem: at 18k eRPM with 13 deg advance,
     * delay shrinks from stepPeriod/2 to ~4 ticks, and the old stepPeriod/4
     * formula set the threshold too high for the available window. */
    {
        int32_t peak = (int32_t)pData->integ.bemfPeakSmooth;
        if (peak < 10) peak = 10;  /* Floor */

        uint16_t sp = pData->timing.stepPeriod;
#if FEATURE_TIMING_ADVANCE
        uint32_t eRPM = ERPM_FROM_ADC_STEP_NUM / sp;
        uint16_t advDeg;
        if (eRPM <= RAMP_TARGET_ERPM)
            advDeg = TIMING_ADVANCE_MIN_DEG;
        else if (eRPM >= MAX_CLOSED_LOOP_ERPM)
            advDeg = TIMING_ADVANCE_MAX_DEG;
        else
        {
            uint32_t range = MAX_CLOSED_LOOP_ERPM - RAMP_TARGET_ERPM;
            uint32_t pos = eRPM - RAMP_TARGET_ERPM;
            advDeg = TIMING_ADVANCE_MIN_DEG +
                (uint16_t)((uint32_t)(TIMING_ADVANCE_MAX_DEG - TIMING_ADVANCE_MIN_DEG)
                           * pos / range);
        }
        uint16_t delayTicks = (uint16_t)((uint32_t)sp * (30 - advDeg) / 60);
#else
        uint16_t delayTicks = sp / 2;
#endif
        if (delayTicks < 1) delayTicks = 1;

        int32_t rawThreshold = peak * (int32_t)delayTicks / 2;
        pData->integ.intThreshold = (rawThreshold * INTEG_THRESHOLD_GAIN) >> 8;
        if (pData->integ.intThreshold < 1)
            pData->integ.intThreshold = 1;
    }
#endif
}

/**
 * @brief Poll ADC for zero-crossing detection via software threshold.
 *
 * Blanking-aware design: during blanking, cmpPrev is tracked and edges
 * are detected + filtered, but ZC is NOT confirmed. This ensures the
 * crossing event is captured even if it occurs during the blanking window
 * (e.g. body diode recovery). After blanking, if the filter count has
 * reached the threshold, ZC is confirmed immediately.
 *
 * Processing order: validity → threshold → cmpPrev init → edge/filter
 * → (if past blanking) ZC confirmation.
 *
 * @param pData  Pointer to global ESC data
 * @param now    Current adcIsrTick value
 * @return true if ZC was confirmed this call
 */
bool BEMF_ZC_Poll(volatile GARUDA_DATA_T *pData, uint16_t now)
{
    /* One ZC per step guard — prevent multiple noisy edges from counting */
    if (pData->bemf.zeroCrossDetected)
    {
#if FEATURE_BEMF_INTEGRATION
        /* Post-ZC integration-only fast path.
         * Computes vCorrected and runs integration accumulation.
         * Skips ALL diagnostic counters, blanking, and edge detection. */
        if (pData->timing.zcSynced && pData->bemf.bemfSampleValid)
        {
            uint16_t elapsed = (uint16_t)(now - pData->timing.lastCommTick);
            uint16_t blankTicks = computeBlankTicks(pData);
            uint8_t floatPhase = commutationTable[pData->currentStep].floatingPhase;
            uint16_t vFloat = pData->bemf.bemfRaw;
            uint16_t zcThresh = pData->bemf.zcThreshold;
            int32_t vCorrected = computeVCorrected(vFloat, floatPhase);

            if (elapsed >= blankTicks)
            {
                int32_t dev = vCorrected - (int32_t)zcThresh;
                int8_t pol = commutationTable[pData->currentStep].zcPolarity;
                int32_t sample = (pol > 0) ? dev : -dev;
                if (sample < 0) sample = 0;

                if ((uint16_t)sample > pData->integ.stepDevMax)
                    pData->integ.stepDevMax = (uint16_t)sample;

                if (!pData->integ.shadowFired)
                {
                    pData->integ.integral += sample;
                    if (pData->integ.integral > INTEG_CLAMP)
                        pData->integ.integral = INTEG_CLAMP;
                    if (pData->integ.integral >= pData->integ.intThreshold)
                    {
                        pData->integ.shadowFired = true;
                        pData->integ.shadowFireTick = now;
                    }
                }
            }
        }
#endif
        return false;
    }

    /* Compute blanking window — edge tracking runs inside, confirmation only outside */
    uint16_t elapsed = (uint16_t)(now - pData->timing.lastCommTick);
    uint16_t blankTicks = computeBlankTicks(pData);
    bool inBlanking = (elapsed < blankTicks);

    /* Skip if sample is invalid (stale AD2 data after mux switch) */
    if (!pData->bemf.bemfSampleValid)
    {
        pData->zcDiag.invalidSampleTotal++;
        return false;
    }

    /* Read floating phase ADC value and compute binary state via threshold */
    uint8_t floatPhase = commutationTable[pData->currentStep].floatingPhase;
    uint16_t vFloat = pData->bemf.bemfRaw;
    uint16_t zcThresh = pData->bemf.zcThreshold;

    /* Per-phase gain + offset correction (shared helper) */
    int32_t vCorrected = computeVCorrected(vFloat, floatPhase);

    uint8_t cmpNow;
    if ((uint16_t)vCorrected > zcThresh + ZC_ADC_DEADBAND)
        cmpNow = 1;
    else if ((uint16_t)vCorrected < (zcThresh > ZC_ADC_DEADBAND ? zcThresh - ZC_ADC_DEADBAND : 0))
        cmpNow = 0;
    else
    {
        cmpNow = (pData->bemf.cmpPrev != 0xFF) ? pData->bemf.cmpPrev : 0;
        pData->zcDiag.deadbandHoldTotal++;
    }

    /* Diagnostic tracking (runs during AND after blanking) */
    pData->zcDiag.zcPollTotal++;
    if (cmpNow) pData->zcDiag.zcAboveTotal++;
    pData->zcDiag.lastStateNow = cmpNow;
    pData->zcDiag.lastCmpExpected = pData->bemf.cmpExpected;
    pData->zcDiag.lastFloatPhase = floatPhase;
    pData->zcDiag.lastFloatAdc = vFloat;
    pData->zcDiag.lastZcThreshold = zcThresh;

    /* Per-phase min/max */
    if (vFloat < pData->zcDiag.floatAdcMin[floatPhase])
        pData->zcDiag.floatAdcMin[floatPhase] = vFloat;
    if (vFloat > pData->zcDiag.floatAdcMax[floatPhase])
        pData->zcDiag.floatAdcMax[floatPhase] = vFloat;
    if (zcThresh < pData->zcDiag.zcThresholdMin)
        pData->zcDiag.zcThresholdMin = zcThresh;
    if (zcThresh > pData->zcDiag.zcThresholdMax)
        pData->zcDiag.zcThresholdMax = zcThresh;

#if FEATURE_BEMF_INTEGRATION
    /* Pre-ZC integration accumulation (normal path, before zeroCrossDetected is set).
     * Gated on zcSynced (no pre-sync training) AND past blanking (no noise contamination). */
    if (pData->timing.zcSynced && elapsed >= blankTicks)
    {
        int32_t dev = (int32_t)vCorrected - (int32_t)zcThresh;
        int8_t pol = commutationTable[pData->currentStep].zcPolarity;
        int32_t sample = (pol > 0) ? dev : -dev;
        if (sample < 0) sample = 0;  /* Pre-ZC: clamp to 0 */

        if ((uint16_t)sample > pData->integ.stepDevMax)
            pData->integ.stepDevMax = (uint16_t)sample;

        if (!pData->integ.shadowFired)
        {
            pData->integ.integral += sample;
            if (pData->integ.integral > INTEG_CLAMP)
                pData->integ.integral = INTEG_CLAMP;
            if (pData->integ.integral >= pData->integ.intThreshold)
            {
                pData->integ.shadowFired = true;
                pData->integ.shadowFireTick = now;
            }
        }
    }
#endif

    /* First read after commutation — initialize cmpPrev, no edge possible yet */
    if (pData->bemf.cmpPrev == 0xFF)
    {
        pData->bemf.cmpPrev = cmpNow;
        if (inBlanking) pData->zcDiag.blankSkipTotal++;
        return false;
    }

    /* Edge detection — runs during AND after blanking.
     * During blanking: edges start the filter count.
     * After blanking: edges also start the filter count.
     * ZC confirmation is gated by inBlanking below. */
    uint8_t expected = pData->bemf.cmpExpected;

    if (pData->bemf.filterCount == 0)
    {
        /* Looking for the initial transition edge */
        bool edgeDetected = false;
        if (expected == 1 && pData->bemf.cmpPrev == 0 && cmpNow == 1)
            edgeDetected = true;  /* Rising edge */
        else if (expected == 0 && pData->bemf.cmpPrev == 1 && cmpNow == 0)
            edgeDetected = true;  /* Falling edge */
        else if (pData->bemf.cmpPrev != cmpNow)
            pData->zcDiag.wrongEdgeTotal++;  /* Transition opposite to expected */

        if (edgeDetected)
        {
            pData->bemf.filterCount = 1;
            if (inBlanking) pData->zcDiag.blankTransitionTotal++;
        }
    }
    else
    {
        /* Already filtering — confirm edge holds */
        if (cmpNow == expected)
            pData->bemf.filterCount++;
        else if (!inBlanking)
        {
            /* Post-blanking non-match: genuine noise rejection, reset filter */
            pData->zcDiag.lastEdgeCount = pData->bemf.filterCount;
            pData->bemf.filterCount = 0;
        }
        /* During blanking: hold filter count on non-match.  Blanking
         * noise (body diode transients) should not wipe accumulated
         * evidence.  After blanking, the first non-match will reset. */
    }

    /* Update previous reading (tracked during blanking too) */
    pData->bemf.cmpPrev = cmpNow;

    /* During blanking: state tracked, edges detected, but ZC NOT confirmed */
    if (inBlanking)
    {
        pData->zcDiag.blankSkipTotal++;
        return false;
    }

    /* === Past blanking — ZC confirmation allowed === */

    /* Determine filter threshold */
#if ZC_ADAPTIVE_FILTER
    uint8_t threshold = (pData->timing.stepPeriod <= ZC_FILTER_SPEED_THRESH)
                        ? ZC_FILTER_MIN : ZC_FILTER_MAX;
#else
    uint8_t threshold = ZC_FILTER_THRESHOLD;
#endif

    /* Check if ZC is confirmed (ONLY after blanking ends) */
    if (pData->bemf.filterCount >= threshold)
    {
        /* ZC confirmed */
        if (pData->timing.goodZcCount < (uint16_t)ZC_SYNC_THRESHOLD)
            pData->timing.goodZcCount++;

        pData->timing.consecutiveMissedSteps = 0;
        pData->timing.stepMissCount[pData->currentStep] = 0;
        pData->bemf.zeroCrossDetected = true;
        pData->zcDiag.zcConfirmedCount++;
        pData->zcDiag.zcPerStep[pData->currentStep]++;

        /* Track which polarities produce detectable ZC */
        if (commutationTable[pData->currentStep].zcPolarity > 0)
            pData->timing.risingZcWorks = true;
        else
            pData->timing.fallingZcWorks = true;

        /* Post-sync: update timing and set commutation deadline */
        if (pData->timing.zcSynced)
        {
            pData->timing.prevZcTick = pData->timing.lastZcTick;
            pData->timing.lastZcTick = now;

            if (pData->timing.hasPrevZc)
            {
                pData->timing.zcInterval = (uint16_t)(now - pData->timing.prevZcTick);

                /* Only update stepPeriod from consecutive ZC-to-ZC intervals
                 * (spacing == 1).  When spacing > 1, forced timeout steps
                 * occurred between the two ZCs, inflating zcInterval.  The
                 * division-by-spacing approach cannot fully compensate because
                 * the forced step ran at 2× stepPeriod, not the real motor
                 * period.  Skipping the update breaks the death spiral where
                 * inflated intervals push stepPeriod up → more misses → repeat. */
                uint8_t spacing = pData->timing.stepsSinceLastZc;
                if (spacing == 1)
                {
                    uint16_t effectiveInterval = pData->timing.zcInterval;

#if ZC_ADAPTIVE_PERIOD
                    /* Phase 2B: IIR smoothing */
                    pData->timing.stepPeriod = (uint16_t)(
                        ((uint32_t)pData->timing.stepPeriod * 3 + effectiveInterval) >> 2);
#else
                    /* Phase 2A: direct assignment */
                    pData->timing.stepPeriod = effectiveInterval;
#endif
                }
                /* spacing > 1: keep current stepPeriod unchanged */

                /* Clamp step period (use CL limit, decoupled from ramp) */
                if (pData->timing.stepPeriod < MIN_CL_ADC_STEP_PERIOD)
                    pData->timing.stepPeriod = MIN_CL_ADC_STEP_PERIOD;
                if (pData->timing.stepPeriod > INITIAL_ADC_STEP_PERIOD)
                    pData->timing.stepPeriod = INITIAL_ADC_STEP_PERIOD;
            }
            else
            {
                /* First ZC after sync — no valid interval yet, keep inherited stepPeriod */
                pData->timing.hasPrevZc = true;
            }

            /* Commutation delay after ZC.
             * Default: 30° = stepPeriod/2.
             * With timing advance: linear by eRPM (not stepPeriod).
             * eRPM = ERPM_FROM_ADC_STEP_NUM / stepPeriod. */
#if FEATURE_TIMING_ADVANCE
            uint16_t sp = pData->timing.stepPeriod;
            uint32_t eRPM = ERPM_FROM_ADC_STEP_NUM / sp;
            uint16_t advDeg;
            if (eRPM <= RAMP_TARGET_ERPM)
                advDeg = TIMING_ADVANCE_MIN_DEG;
            else if (eRPM >= MAX_CLOSED_LOOP_ERPM)
                advDeg = TIMING_ADVANCE_MAX_DEG;
            else
            {
                uint32_t range = MAX_CLOSED_LOOP_ERPM - RAMP_TARGET_ERPM;
                uint32_t pos = eRPM - RAMP_TARGET_ERPM;
                advDeg = TIMING_ADVANCE_MIN_DEG +
                    (uint16_t)((uint32_t)(TIMING_ADVANCE_MAX_DEG - TIMING_ADVANCE_MIN_DEG)
                               * pos / range);
            }
            uint16_t delay = (uint16_t)((uint32_t)sp * (30 - advDeg) / 60);
            pData->zcDiag.lastAdvanceDeg = advDeg;
#else
            uint16_t delay = pData->timing.stepPeriod / 2;
#endif
            pData->timing.commDeadline = (uint16_t)(now + delay);
            pData->timing.deadlineActive = true;
#if FEATURE_BEMF_INTEGRATION
            pData->timing.deadlineIsZc = true;
#endif
        }

        /* Reset step spacing counter for next interval measurement */
        pData->timing.stepsSinceLastZc = 0;

        return true;
    }

    return false;
}

/**
 * @brief Check if commutation deadline has been reached.
 *
 * @param pData  Pointer to global ESC data
 * @param now    Current adcIsrTick value
 * @return true if deadline reached (time to commutate)
 */
bool BEMF_ZC_CheckDeadline(volatile GARUDA_DATA_T *pData, uint16_t now)
{
    if (pData->timing.deadlineActive)
    {
        /* Unsigned subtraction handles 16-bit wrap correctly:
         * if (now - deadline) < 0x8000, deadline has passed.
         * INVARIANT: deadlines must be < 32768 ticks in the future.
         * Enforced by static assert in garuda_calc_params.h. */
        if ((uint16_t)(now - pData->timing.commDeadline) < 0x8000u)
        {
            pData->timing.deadlineActive = false;
            return true;
        }
    }
    return false;
}

/**
 * @brief Check for ZC timeout (missed zero-crossing).
 * Called every ADC ISR in post-sync mode. Returns action to take.
 * Re-arms internally to prevent firing every tick.
 *
 * @param pData  Pointer to global ESC data
 * @param now    Current adcIsrTick value
 * @return ZC_TIMEOUT_NONE, ZC_TIMEOUT_FORCE_STEP, or ZC_TIMEOUT_DESYNC
 */
ZC_TIMEOUT_RESULT_T BEMF_ZC_CheckTimeout(volatile GARUDA_DATA_T *pData, uint16_t now)
{
    /* If a ZC was detected and deadline is pending, don't fight it */
    if (pData->timing.deadlineActive)
        return ZC_TIMEOUT_NONE;

    uint16_t elapsed = (uint16_t)(now - pData->timing.lastCommTick);
    uint32_t timeoutTicks = (uint32_t)pData->timing.stepPeriod * ZC_TIMEOUT_MULT;

    if ((uint32_t)elapsed > timeoutTicks)
    {
        /* ZC missed for this step */
        pData->timing.consecutiveMissedSteps++;

        if (pData->timing.goodZcCount > 0)
            pData->timing.goodZcCount--;

        /* Re-arm: set lastCommTick = now to prevent re-firing every tick */
        pData->timing.lastCommTick = now;

        if (pData->timing.consecutiveMissedSteps >= ZC_MISS_LIMIT)
            return ZC_TIMEOUT_DESYNC;
        else
            return ZC_TIMEOUT_FORCE_STEP;
    }

    return ZC_TIMEOUT_NONE;
}

/**
 * @brief Schedule timing-based fallback for steps with a history of ZC misses.
 *
 * Called after every commutation in post-sync mode.  If the current step
 * has missed ZC detection on its last ZC_STEP_MISS_LIMIT consecutive
 * occurrences, a fallback deadline is set at lastCommTick + stepPeriod.
 *
 * Unlike the previous polarity-based approach, Poll still runs — if ZC
 * is confirmed, it overwrites the fallback deadline with a shorter
 * 30-degree delay.  This allows automatic recovery when operating
 * conditions change (e.g. duty increases).
 *
 * @param pData  Pointer to global ESC data
 * @param now    Current adcIsrTick value (same as lastCommTick after OnCommutation)
 */
void BEMF_ZC_HandleUndetectableStep(volatile GARUDA_DATA_T *pData, uint16_t now)
{
    if (pData->timing.stepMissCount[pData->currentStep] >= ZC_STEP_MISS_LIMIT)
    {
        /* This step has a history of misses — schedule a timing-based
         * fallback at 1× stepPeriod.  Poll + Timeout remain active:
         * - If Poll confirms ZC → overwrites deadline (shorter delay)
         * - If not → fallback fires, avoiding the 2× timeout jerk */
        pData->timing.commDeadline = (uint16_t)(now + pData->timing.stepPeriod);
        pData->timing.deadlineActive = true;
#if FEATURE_BEMF_INTEGRATION
        pData->timing.deadlineIsZc = false;
#endif
        /* zeroCrossDetected is NOT set — Poll can still detect and override */
    }
}

#endif /* FEATURE_BEMF_CLOSED_LOOP */
