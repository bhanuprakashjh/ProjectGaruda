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
        return false;

    /* Compute blanking window — edge tracking runs inside, confirmation only outside */
    uint16_t elapsed = (uint16_t)(now - pData->timing.lastCommTick);
    uint16_t blankTicks = (uint16_t)((uint32_t)pData->timing.stepPeriod * ZC_BLANKING_PERCENT / 100);
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

    /* Per-phase Q15 gain + signed offset correction */
    static const uint16_t phaseGain[3] = {
        ZC_PHASE_GAIN_A, ZC_PHASE_GAIN_B, ZC_PHASE_GAIN_C
    };
    static const int16_t phaseOffset[3] = {
        ZC_PHASE_OFFSET_A, ZC_PHASE_OFFSET_B, ZC_PHASE_OFFSET_C
    };
    int32_t vCorrected = (int32_t)(((uint32_t)vFloat * phaseGain[floatPhase]) >> 15)
                       + phaseOffset[floatPhase];
    if (vCorrected < 0) vCorrected = 0;
    if (vCorrected > 4095) vCorrected = 4095;

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

            /* 30-degree delay: commutate at stepPeriod/2 after ZC */
            uint16_t delay = pData->timing.stepPeriod / 2;
            pData->timing.commDeadline = (uint16_t)(now + delay);
            pData->timing.deadlineActive = true;
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
        /* zeroCrossDetected is NOT set — Poll can still detect and override */
    }
}

#endif /* FEATURE_BEMF_CLOSED_LOOP */
