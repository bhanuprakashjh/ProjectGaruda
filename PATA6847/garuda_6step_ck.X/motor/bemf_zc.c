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
#if FEATURE_IC_DMA_SHADOW
#include "../hal/hal_ic_dma.h"
#include "../hal/hal_dma_burst.h"
#endif
#if FEATURE_CLC_BLANKING
#include "../hal/hal_clc.h"
#endif

/**
 * @brief Read the digital BEMF comparator output for a given phase.
 * @param phase 0=A, 1=B, 2=C
 * @return 1 if comparator output is high, 0 if low
 */
static inline uint8_t ReadBEMFComparator(uint8_t phase)
{
#if FEATURE_CLC_BLANKING
    /* CLC D-FF output: sampled at mid-PWM, held between cycles.
     * Eliminates switching noise aliasing (step-2 vector fix). */
    return HAL_CLC_ReadOutput(phase);
#else
    switch (phase)
    {
        case 0: return BEMF_A_GetValue() ? 1 : 0;
        case 1: return BEMF_B_GetValue() ? 1 : 0;
        case 2: return BEMF_C_GetValue() ? 1 : 0;
        default: return 0;
    }
#endif
}

/**
 * @brief Read the RAW BEMF comparator GPIO (bypasses CLC D-FF).
 * Used for raw corroboration: sanity-check that CLC D-FF isn't stale.
 */
static inline uint8_t ReadRawBEMFComparator(uint8_t phase)
{
    switch (phase)
    {
        case 0: return BEMF_A_GetValue() ? 1 : 0;
        case 1: return BEMF_B_GetValue() ? 1 : 0;
        case 2: return BEMF_C_GetValue() ? 1 : 0;
        default: return 0;
    }
}

/* ── ZC V2 Mode Helpers (Phase 2) ─────────────────────────────────── */

static void ZcEnterAcquire(volatile GARUDA_DATA_T *pData)
{
    pData->zcCtrl.mode = ZC_MODE_ACQUIRE;
    pData->zcCtrl.acquireGoodCount = 0;
    /* Phase 7: ACQUIRE uses conservative timing.
     * The refIntervalHR is kept as-is — it was valid when we left
     * RECOVER, and the motor should be near that speed. */
}

static void ZcEnterTrack(volatile GARUDA_DATA_T *pData)
{
    pData->zcCtrl.mode = ZC_MODE_TRACK;
    pData->zcCtrl.recoverAttempts = 0;  /* reset on successful lock */
}

static void ZcEnterRecover(volatile GARUDA_DATA_T *pData)
{
    pData->zcCtrl.mode = ZC_MODE_RECOVER;
    pData->zcCtrl.recoverGoodCount = 0;
    if (pData->zcCtrl.recoverAttempts < 255u)
        pData->zcCtrl.recoverAttempts++;
    /* Phase 7: RECOVER expands refIntervalHR by 12.5% to give more
     * time for ZC detection. The motor may be decelerating and the
     * estimator needs room to find the real speed. */
    {
        uint16_t ref = pData->zcCtrl.refIntervalHR;
        uint16_t inc = ref >> 3;  /* +12.5% */
        if (inc < 1) inc = 1;
        ref += inc;
        pData->zcCtrl.refIntervalHR = ref;
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
    pData->icZc.rawCoro = 0;
    pData->icZc.hasFirstClcMatch = false;
    pData->icZc.firstClcMatchHR = 0;
    pData->icZc.hasRawFirstMatch = false;
    pData->icZc.rawFirstMatchHR = 0;
    pData->icZc.rawFirstMatchT1 = 0;
    pData->icZc.diagRawVeto = 0;
    pData->icZc.diagIcAgeReject = 0;
    pData->icZc.diagTrackFallback = 0;
    pData->icZc.diagRawStableBlock = 0;
    pData->icZc.diagTsFromIc = 0;
    pData->icZc.diagTsFromRaw = 0;
    pData->icZc.diagTsFromClc = 0;
    pData->icZc.diagTsFromPoll = 0;
    pData->icZc.diagIcLeadReject = 0;
    pData->icZc.lastCmpState = 0xFF;
    {
        uint8_t i;
        for (i = 0; i < 6; i++) {
            pData->icZc.stepFlips[i] = 0;
            pData->icZc.stepPolls[i] = 0;
        }
    }
    HAL_ZcTimer_Stop();
#if FEATURE_IC_ZC_CAPTURE
    pData->icZc.icCommStamp = 0;
    pData->icZc.icArmed = false;
    pData->icZc.icCandidateValid = false;
    pData->icZc.diagIcAccepted = 0;
    pData->icZc.diagIcBounce = 0;
    HAL_ZcIC_Disarm();
#endif
#endif

    pData->zcDiag.zcLatencyPct = 0;
    pData->zcDiag.lastBlankingHR = 0;
    pData->zcDiag.diagBypassAccepted = 0;
    pData->zcDiag.diagRisingZcCount = 0;
    pData->zcDiag.diagFallingZcCount = 0;
    pData->zcDiag.diagIntervalReject = 0;
    pData->zcDiag.actualForcedComm = 0;
    pData->zcDiag.zcTimeoutCount = 0;
    pData->zcDiag.diagRisingTimeouts = 0;
    pData->zcDiag.diagFallingTimeouts = 0;
    pData->zcDiag.diagRisingRejects = 0;
    pData->zcDiag.diagFallingRejects = 0;
    pData->zcDiag.diagTargetPast = 0;
    pData->zcDiag.lastScheduleMarginHR = 0;
    {
        uint8_t i;
        for (i = 0; i < 6; i++) {
            pData->zcDiag.stepAccepted[i] = 0;
            pData->zcDiag.stepTimeouts[i] = 0;
        }
    }

    pData->zcCtrl.mode = ZC_MODE_ACQUIRE;
    pData->zcCtrl.acquireGoodCount = 0;
    pData->zcCtrl.recoverGoodCount = 0;
    pData->zcCtrl.recoverAttempts = 0;
    pData->zcCtrl.refIntervalHR = 0;
    pData->zcCtrl.rawIntervalHR = 0;
    pData->zcCtrl.refIntervalT1 = 0;
    pData->zcCtrl.rawIntervalT1 = 0;
    pData->zcCtrl.originalTimeoutHR = 0;
    pData->zcCtrl.demagMetric = 0;

    pData->timing.stepPeriod = INITIAL_STEP_PERIOD;
    pData->timing.lastCommTick = 0;
    pData->timing.lastZcTick = 0;
    pData->timing.prevZcTick = 0;
    pData->timing.zcInterval = 0;
    pData->timing.prevZcInterval = 0;
    pData->timing.commDeadline = 0;
    pData->timing.forcedCountdown = 0;
    pData->timing.goodZcCount = 0;
    pData->timing.checkpointStepPeriod = INITIAL_STEP_PERIOD;
#if FEATURE_IC_ZC
    pData->timing.checkpointStepPeriodHR = 0;
#endif
    pData->timing.revStepCount = 0;
    pData->timing.consecutiveMissedSteps = 0;
    pData->timing.stepsSinceLastZc = 0;
    pData->timing.zcSynced = false;
    pData->timing.deadlineActive = false;
    pData->timing.hasPrevZc = false;
    pData->timing.bypassSuppressed = false;

#if FEATURE_IC_ZC
    pData->timing.lastZcTickHR = 0;
    pData->timing.prevZcTickHR = 0;
    pData->timing.zcIntervalHR = 0;
    pData->timing.prevZcIntervalHR = 0;
    pData->timing.stepPeriodHR = 0;
    pData->timing.hasPrevZcHR = false;

    /* PLL predictor init (Step 1: shadow/telemetry mode) */
    pData->zcPred.predStepHR = 0;
    pData->zcPred.predNextCommHR = 0;
    pData->zcPred.predZcHR = 0;
    pData->zcPred.predZcOffsetHR = 0;
    pData->zcPred.phaseErrHR = 0;
    pData->zcPred.locked = false;
    pData->zcPred.missCount = 0;
    pData->zcPred.lastCommHR = 0;
    pData->zcPred.scanOpenHR = 0;
    pData->zcPred.scanCloseHR = 0;
    pData->zcPred.diagPredCommCount = 0;
    pData->zcPred.diagPhaseErrAccum = 0;
    pData->zcPred.diagZcInWindow = 0;
    pData->zcPred.diagZcOutWindow = 0;
    pData->zcPred.diagMinMarginHR = 32767;
    pData->zcPred.gateActive = false;
    pData->zcPred.gateArmCount = 0;
    pData->zcPred.gateRevRejects = 0;
    pData->zcPred.diagWinCandInGated = 0;
    pData->zcPred.diagWinCandOutGated = 0;
    pData->zcPred.diagWinOutEarly = 0;
    pData->zcPred.diagWinOutLate = 0;
    pData->zcPred.diagWindowReject = 0;
    pData->zcPred.diagWindowRecovered = 0;
    pData->zcPred.predictiveMode = false;
    pData->zcPred.handoffPending = false;
    pData->zcPred.lastPredCommHR = 0;
    pData->zcPred.pendingPredCommHR = 0;
    pData->zcPred.pendingPredValid = false;
    pData->zcPred.predVsReactiveDelta = 0;
    pData->zcPred.deltaOkCount = 0;
    pData->zcPred.entryScore = 0;
    pData->zcPred.diagPredCommOwned = 0;
    pData->zcPred.diagPredEnter = 0;
    pData->zcPred.diagPredEntryLate = 0;
    pData->zcPred.diagPredIsrFired = 0;
    pData->zcPred.diagPredIsrEntries = 0;
    pData->zcPred.diagPredExitRed = 0;
    pData->zcPred.diagPredExitMiss = 0;
    pData->zcPred.diagPredExitYellow = 0;
    pData->zcPred.diagPredExitPhaseErr = 0;
    pData->zcPred.diagPredExitTimeout = 0;
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

/**
 * @brief Speed-adaptive timing advance level.
 *
 * The static TIMING_ADVANCE_LEVEL works fine at low/mid speed but at
 * high eRPM the delayHR budget = (spHR/2 - spHR*TAL/8) shrinks below
 * the detection-pipeline latency (~16 µs FastPoll + rawStable + accept
 * overhead), so the schedule margin goes negative and the wall hits.
 *
 * Measured at 100k eRPM with TAL=3:  delayHR = 13 µs, latency ≈ 16 µs,
 *                                    margin ≈ −3 µs (matches CSV).
 * With TAL=2 at the same speed:      delayHR = 25 µs, margin ≈ +9 µs.
 *
 * Curve (with hysteresis to avoid thrashing at the boundary):
 *   eRPM <  65k → TAL = compile-time default (typically 3)
 *   eRPM >  75k → TAL = max(default-1, 1)
 *   eRPM > 115k → TAL = max(default-2, 1)   (only if default >= 3)
 *
 * AM32 / BLHeli / ESCape32 all back off advance at high speed for
 * the same reason.
 */
static inline uint8_t AdaptiveTimingAdvance(uint32_t eRPM)
{
    static uint8_t level = TIMING_ADVANCE_LEVEL;

    /* High band: drop to TAL-2 above 115k, recover below 105k */
    if (TIMING_ADVANCE_LEVEL >= 3)
    {
        if (level >= (TIMING_ADVANCE_LEVEL - 1) && eRPM > 115000U)
            level = (TIMING_ADVANCE_LEVEL > 2U) ? (TIMING_ADVANCE_LEVEL - 2U) : 1U;
        else if (level <  (TIMING_ADVANCE_LEVEL - 1) && eRPM < 105000U)
            level = TIMING_ADVANCE_LEVEL - 1U;
    }

    /* Mid band: drop to TAL-1 above 75k, recover below 65k */
    if (level >= TIMING_ADVANCE_LEVEL && eRPM > 75000U)
        level = (TIMING_ADVANCE_LEVEL > 1U) ? (TIMING_ADVANCE_LEVEL - 1U) : 1U;
    else if (level <  TIMING_ADVANCE_LEVEL && level >= (TIMING_ADVANCE_LEVEL - 1)
             && eRPM < 65000U)
        level = TIMING_ADVANCE_LEVEL;

    return level;
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

    /* ── Per-revolution desync check (ESCape32/AM32-inspired) ──────────
     * Every 6 steps (one electrical revolution), compare current step period
     * against the checkpoint. If changed by >2:1, false-ZC cascade detected.
     *
     * At high speed (Tp<=10), uses HR timer (640ns, 78x resolution) instead
     * of Timer1 (50µs) to avoid quantization-induced false positives.
     * Timer1 Tp=3 has 33% quantization error; HR Tp=234 has 0.4% error. */
    pData->timing.revStepCount++;
    if (pData->timing.revStepCount >= 6)
    {
        pData->timing.revStepCount = 0;
        /* Reset per-revolution gate reject counter.
         * If 2+ rejects in one revolution, disable gate. */
        if (pData->zcPred.gateRevRejects >= 2)
            pData->zcPred.gateActive = false;
        pData->zcPred.gateRevRejects = 0;
        bool desyncDetected = false;

#if FEATURE_IC_ZC
        /* Prefer HR timer for desync check when available (high speed).
         * Check if step period changed by >40% in one revolution. */
        if (pData->timing.hasPrevZcHR &&
            pData->timing.stepPeriodHR > 0 &&
            pData->timing.checkpointStepPeriodHR > 0)
        {
            uint16_t cpHR = pData->timing.checkpointStepPeriodHR;
            uint16_t spHR = pData->timing.stepPeriodHR;
            uint16_t threshold = (uint16_t)((uint32_t)cpHR * ZC_REV_DESYNC_RATIO_PCT / 100);
            uint16_t diff = (spHR > cpHR) ? (spHR - cpHR) : (cpHR - spHR);
            if (diff > threshold)
            {
                desyncDetected = true;
            }
            pData->timing.checkpointStepPeriodHR = spHR;
        }
        else
#endif
        {
            /* Fallback to Timer1 for low speed or no HR data */
            uint16_t cp = pData->timing.checkpointStepPeriod;
            uint16_t sp = pData->timing.stepPeriod;
            if (cp > 0 && sp > 0)
            {
                uint16_t threshold = (uint16_t)((uint32_t)cp * ZC_REV_DESYNC_RATIO_PCT / 100);
                uint16_t diff = (sp > cp) ? (sp - cp) : (cp - sp);
                if (diff > threshold)
                {
                    desyncDetected = true;
                }
            }
        }
        pData->timing.checkpointStepPeriod = pData->timing.stepPeriod;

        if (desyncDetected && pData->timing.zcSynced)
        {
            /* False-ZC cascade — reset to safe state */
            pData->timing.zcSynced = false;
            pData->timing.goodZcCount = 0;
            pData->timing.stepPeriod = pData->timing.checkpointStepPeriod;
            pData->timing.hasPrevZc = false;
#if FEATURE_IC_ZC
            pData->timing.hasPrevZcHR = false;
            pData->timing.checkpointStepPeriodHR = 0;
#endif
            /* ZC V2: enter RECOVER on per-revolution desync detection */
            if (pData->zcCtrl.mode == ZC_MODE_TRACK)
                ZcEnterRecover(pData);
        }
    }

    pData->bemf.zeroCrossDetected = false;
    pData->bemf.filterCount = 0;
    pData->bemf.cmpPrev = 0xFF;  /* Force re-read after blanking */

    /* Expected post-ZC comparator state:
     * Rising ZC (+1): comparator goes 0→1, so we expect 1
     * Falling ZC (-1): comparator goes 1→0, so we expect 0 */
    pData->bemf.cmpExpected = (step->zcPolarity > 0) ? 1 : 0;

    pData->timing.lastCommTick = pData->timer1Tick;
    pData->timing.stepsSinceLastZc++;
    /* Don't clear deadlineActive in predictive mode — the predictor
     * already programmed the next compare and is relying on this flag
     * to gate the next _CCP4Interrupt. */
#if FEATURE_IC_ZC
    if (!pData->zcPred.predictiveMode)
    {
        pData->timing.deadlineActive = false;
        HAL_ComTimer_Cancel();
    }
#else
    pData->timing.deadlineActive = false;
#endif

    /* Set forced commutation timeout: ZC_TIMEOUT_MULT * period.
     * Phase 4: use refIntervalHR when available for accurate timeout
     * at high speed where Timer1 quantization (50µs) is too coarse.
     * At Tp=3 Timer1, HR gives 234 ticks vs Timer1's 3 — 78× better. */
    {
        uint16_t timeoutT1;
#if FEATURE_IC_ZC
        if (pData->state == ESC_CLOSED_LOOP &&
            pData->zcCtrl.refIntervalHR > 0)
        {
            /* Use protected refIntervalHR for timeout.
             * Convert to Timer1 for the countdown mechanism. */
            uint32_t refT1 = ((uint32_t)pData->zcCtrl.refIntervalHR *
                             COM_TIMER_T1_DENOM + COM_TIMER_T1_NUMER / 2) /
                            COM_TIMER_T1_NUMER;
            if (refT1 < 1) refT1 = 1;
            uint32_t tout = refT1 * ZC_TIMEOUT_MULT;
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
        /* Record commutation time in SCCP4 ticks for interval rejection */
        pData->icZc.lastCommHR = HAL_ComTimer_ReadTimer();

        /* PRODUCTION BLANKING: fixed minimum + 50% interval rejection.
         *
         * Instead of percentage-based blanking (which requires per-motor
         * tuning), use the approach from ESCape32/VESC/AM32:
         *
         * Layer 1: Fixed minimum blanking (~25µs) — rejects immediate
         *   switching transients after commutation. This is hardware-
         *   related (FET switching + ringing), not motor-dependent.
         *
         * Layer 2: 50% interval rejection — any ZC candidate arriving
         *   before 50% of the last ZC interval is rejected. This is
         *   checked in FastPoll (not here). It automatically handles:
         *   - Demagnetization (completes within ~25-35% of step)
         *   - Switching noise at start of step
         *   - Self-adapts to any motor speed without configuration
         *
         * The combination of fixed minimum (Layer 1) + 50% interval
         * (Layer 2) + consecutive-read filter (Layer 3) gives three
         * independent noise rejection layers that work for ANY motor
         * at ANY speed without per-motor tuning.
         */
        /* Layer 1 blanking: percentage of step period from commutation.
         * Combined with Layer 2 (50% interval rejection in FastPoll).
         *
         * When FEATURE_IC_ZC_ADAPTIVE=0: fixed 12%, floor 25µs (proven).
         * When FEATURE_IC_ZC_ADAPTIVE=1: speed-adaptive percentage that
         * DECREASES at high speed to preserve detection window.
         * Bench data (2810): fixed 12% + 25µs floor = 25% at 100k eRPM
         * → ZcLat=0-2% (over-blanked). Adaptive reduces to ~10%. */
        uint16_t blankHR;
        {
#if FEATURE_IC_ZC_ADAPTIVE
            /* Speed-adaptive: lerp from SLOW% at sp>=200 to FAST% at sp<20.
             * Interpolation over sp range [20..200]. */
            uint16_t sp = pData->timing.stepPeriod;
            uint16_t blankPct;
            if (sp >= 200u)
                blankPct = ZC_BLANK_PCT_SLOW;
            else if (sp >= 20u)
                blankPct = ZC_BLANK_PCT_FAST +
                    (uint16_t)(((uint32_t)(ZC_BLANK_PCT_SLOW - ZC_BLANK_PCT_FAST)
                                * (sp - 20u)) / 180u);
            else
                blankPct = ZC_BLANK_PCT_FAST;
#else
            uint16_t blankPct = 12u;  /* Fixed 12% — original behavior */
#endif

            if (pData->timing.stepPeriodHR > 0 &&
                pData->timing.stepPeriod < HR_MAX_STEP_PERIOD)
            {
                blankHR = (uint16_t)((uint32_t)pData->timing.stepPeriodHR
                                     * blankPct / 100u);
            }
            else
            {
                uint16_t blankT1 = (uint16_t)((uint32_t)pData->timing.stepPeriod
                                               * blankPct / 100u);
                if (blankT1 < 1u) blankT1 = 1u;
                blankHR = (uint16_t)((uint32_t)blankT1 * COM_TIMER_T1_NUMER
                                     / COM_TIMER_T1_DENOM);
            }
        }
        if (blankHR < ZC_BLANK_FLOOR_HR) blankHR = ZC_BLANK_FLOOR_HR;

        pData->zcDiag.lastBlankingHR = blankHR;
        /* Capture original timeout in HR ticks at commutation time —
         * used for accurate zcLatencyPct computation in RecordZcTiming.
         * forcedCountdown is in Timer1 ticks and gets decremented during
         * the step, so using it later overstates lateness. */
        {
            uint32_t toutHR = (uint32_t)pData->timing.forcedCountdown *
                              COM_TIMER_T1_NUMER / COM_TIMER_T1_DENOM;
            pData->zcCtrl.originalTimeoutHR =
                (toutHR <= 65535u) ? (uint16_t)toutHR : 65535u;
        }
        pData->icZc.blankingEndHR = pData->icZc.lastCommHR + blankHR;
        pData->icZc.activeChannel = step->floatingPhase;
        pData->icZc.pollFilter = 0;
        pData->icZc.rawCoro = 0;
        pData->icZc.hasFirstClcMatch = false;
        pData->icZc.hasRawFirstMatch = false;
        pData->icZc.lastCmpState = 0xFF;  /* force re-read on first poll */

#if FEATURE_IC_ZC_CAPTURE
        /* Configure SCCP2 IC for this step's floating phase and polarity.
         * Disarms IC, re-routes PPS, sets edge. Armed later after blanking. */
        {
            static const uint16_t rpPins[3] = { BEMF_A_RP, BEMF_B_RP, BEMF_C_RP };
            HAL_ZcIC_Configure(rpPins[step->floatingPhase],
                               step->zcPolarity > 0);
            pData->icZc.icCommStamp = HAL_ZcIC_ReadTimer();
            pData->icZc.icArmed = false;
            pData->icZc.icCandidateValid = false;
        }
#endif

#if FEATURE_DMA_BURST_CAPTURE
        /* Research capture: MUST run BEFORE HAL_ZcDma_OnCommutation
         * below so that the prior step's edges can still be dumped
         * using the old commHead marker. Closes the previously-open
         * slot (spanning the full step including post-poll tail) and
         * opens a new slot for this step. */
        HAL_DmaBurst_OnCommutation(pData->icZc.lastCommHR,
                                   pData->currentStep,
                                   step->zcPolarity > 0);
#endif

#if FEATURE_IC_DMA_SHADOW
        /* DMA shadow: route floating-phase RP to both CCP2 and CCP5,
         * update commHead markers for bounded scan at probe time.
         * This does NOT affect live detection. */
        {
            static const uint16_t rpPins[3] = { BEMF_A_RP, BEMF_B_RP, BEMF_C_RP };
            HAL_ZcDma_OnCommutation(rpPins[step->floatingPhase],
                                    step->zcPolarity > 0);
        }
#endif

#if FEATURE_CLC_BLANKING
        /* Configure CLC AND filter for this step.
         * Routes floating phase BEMF and active PWM H to CLC1. */
        {
            /* Find which phase is PWM-active */
            uint8_t pwmPhase = 0;
            if (step->phaseA == PHASE_PWM_ACTIVE) pwmPhase = 0;
            else if (step->phaseB == PHASE_PWM_ACTIVE) pwmPhase = 1;
            else pwmPhase = 2;
            HAL_CLC_ConfigureStep(step->floatingPhase, pwmPhase);
        }
        HAL_CLC_ForcePreZcState(step->floatingPhase,
                                step->zcPolarity > 0);
#endif
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

        /* ── PLL Predictor: shadow computation (Step 1) ──────────────
         * Compute predicted next commutation and ZC times without
         * changing actual scheduling. Runs in parallel with reactive
         * path to verify tracking quality before enabling. */
        if (pData->state == ESC_CLOSED_LOOP &&
            pData->zcCtrl.refIntervalHR > 0 &&
            pData->timing.hasPrevZcHR)
        {
            /* Use exact OC fire time captured in _CCP4Interrupt,
             * not HAL_ComTimer_ReadTimer() which includes ISR latency.
             * lastCommHR is set to CCP4RA in the commutation ISR. */
            uint16_t commHR = pData->zcPred.lastCommHR;
            uint16_t spHR = pData->zcCtrl.refIntervalHR;

            /* Seed predictor from refInterval on first valid step */
            if (pData->zcPred.predStepHR == 0)
                pData->zcPred.predStepHR = spHR;

            /* Seed phase offset from measured delay on first valid step.
             * After seeding, predZcOffsetHR is IIR-updated from accepted
             * ZCs in RecordZcTiming. This replaces the fixed half+advance
             * formula which doesn't match the actual reactive phase. */
            if (pData->zcPred.predZcOffsetHR == 0 &&
                pData->zcPred.lastRealZcDelayHR > 0)
            {
                pData->zcPred.predZcOffsetHR =
                    pData->zcPred.lastRealZcDelayHR;
            }

            /* Predict next commutation: this comm + one step period */
            pData->zcPred.predNextCommHR = commHR + pData->zcPred.predStepHR;

            /* Predict where ZC should land in this sector.
             * Uses adaptive phase offset instead of fixed half+advance.
             * The offset converges to the actual comm-to-ZC relationship
             * and will adapt as predictor scheduling changes the phase. */
            pData->zcPred.predZcHR = commHR + pData->zcPred.predZcOffsetHR;

            /* Scan window: +/- 25% of step period around predicted ZC. */
            uint16_t windowHR = pData->zcPred.predStepHR >> 2;
            pData->zcPred.scanOpenHR = pData->zcPred.predZcHR - windowHR;
            pData->zcPred.scanCloseHR = pData->zcPred.predZcHR + windowHR;

            /* Predictor scheduling margin: how far ahead is predNextComm
             * from now? This is a real measurement, not a nominal calc. */
            {
                int16_t predMargin = (int16_t)(pData->zcPred.predNextCommHR
                                               - HAL_ComTimer_ReadTimer());
                if (predMargin < pData->zcPred.diagMinMarginHR)
                    pData->zcPred.diagMinMarginHR = predMargin;
            }

            pData->zcPred.diagPredCommCount++;

            /* Track miss count — incremented here, cleared when ZC
             * correction arrives in RecordZcTiming.
             *
             * At high speed with reactive scheduling, ZC detection is
             * inherently late (target-past), so missCount reaches 2-3
             * normally between telemetry samples. Use tiered thresholds:
             * >2: soft unlock (stop trusting phase for lock decisions)
             * >4: hard exit (stop predictive scheduling + cancel handoff) */
            pData->zcPred.missCount++;
            if (pData->zcPred.missCount > 4)
            {
                pData->zcPred.locked = false;
                /* Only clear entry-related state if predictive mode
                 * is active. Otherwise let the entry path complete. */
                if (pData->zcPred.predictiveMode)
                {
                    pData->zcPred.predictiveMode = false;
                    pData->zcPred.pendingPredValid = false;
                    pData->zcPred.deltaOkCount = 0;
                    pData->zcPred.entryScore = 0;
                    pData->zcPred.handoffPending = false;
                    pData->zcPred.diagPredExitMiss++;
                }
            }
            else if (pData->zcPred.missCount > 2)
            {
                pData->zcPred.locked = false;
            }

            /* Predictive mode entry is now in RecordZcTiming, evaluated
             * after the current ZC's phaseErr/offset/locked are refreshed.
             * This avoids using stale state from the previous step. */
        }
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
            pData->zcDiag.zcLatencyPct = 0xFFu;  /* 0xFF = timeout */
            pData->zcDiag.actualForcedComm++;
            pData->zcDiag.zcTimeoutCount++;
            /* Disarm gate and exit predictive mode on timeout */
            pData->zcPred.gateActive = false;
            pData->zcPred.gateArmCount = 0;
            pData->zcPred.deltaOkCount = 0;
            pData->zcPred.entryScore = 0;
            pData->zcPred.handoffPending = false;
            if (pData->zcPred.predictiveMode)
            {
                pData->zcPred.predictiveMode = false;
                pData->zcPred.pendingPredValid = false;
                pData->zcPred.diagPredExitTimeout++;
            }
            /* Per-polarity and per-step timeout tracking */
            if (commutationTable[pData->currentStep].zcPolarity > 0)
                pData->zcDiag.diagRisingTimeouts++;
            else
                pData->zcDiag.diagFallingTimeouts++;
            if (pData->currentStep < 6)
                pData->zcDiag.stepTimeouts[pData->currentStep]++;
            pData->timing.consecutiveMissedSteps++;
            pData->timing.goodZcCount >>= 1;

            if (pData->timing.consecutiveMissedSteps >= ZC_DESYNC_THRESH)
                pData->timing.zcSynced = false;

            if (pData->timing.consecutiveMissedSteps >= ZC_MISS_LIMIT)
                return ZC_TIMEOUT_DESYNC;

            /* ZC V2 Phase 7: mode-specific timeout behavior. */
            switch (pData->zcCtrl.mode)
            {
                case ZC_MODE_TRACK:
                    /* First timeout in TRACK → enter RECOVER.
                     * RECOVER expands refIntervalHR to give more detection window. */
                    ZcEnterRecover(pData);
                    break;

                case ZC_MODE_ACQUIRE:
                case ZC_MODE_RECOVER:
                    /* Already recovering. Expand refIntervalHR further
                     * to slow down commutation and let motor decelerate
                     * to a speed where ZC can be detected reliably. */
                    {
                        uint16_t ref = pData->zcCtrl.refIntervalHR;
                        uint16_t inc = ref >> 3;  /* +12.5% per timeout */
                        if (inc < 1) inc = 1;
                        ref += inc;
                        if (ref > 5000) ref = 5000;  /* cap ~3.2ms = ~5k eRPM */
                        pData->zcCtrl.refIntervalHR = ref;
                    }
                    /* Reset good-ZC counters */
                    pData->zcCtrl.acquireGoodCount = 0;
                    pData->zcCtrl.recoverGoodCount = 0;
                    /* Enforce max recovery attempts */
                    if (pData->zcCtrl.recoverAttempts >= ZC_RECOVER_MAX_ATTEMPTS)
                        return ZC_TIMEOUT_DESYNC;
                    break;
            }

            /* Also expand Timer1 stepPeriod for backup path consistency */
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

    /* Step 3: In predictive mode, commutation is already scheduled
     * by _CCP4Interrupt. ZC detection only updates the predictor. */
#if FEATURE_IC_ZC
    if (pData->zcPred.predictiveMode)
        return;
#endif

    /* Phase 4: schedule from protected refInterval, not min-of-recent.
     * The old min(IIR, 2-step-avg) shortcut amplified false short
     * intervals — one bad ZC immediately tightened the next schedule.
     * refIntervalHR has independent shrink/growth clamps. */
    uint16_t sp = pData->timing.stepPeriod;  /* Timer1 fallback */

    /* Compute eRPM for timing advance lookup.
     * Use protected refIntervalHR — consistent with scheduling. */
    uint32_t eRPM;
#if FEATURE_IC_ZC
    if (pData->zcCtrl.refIntervalHR > 0)
    {
        eRPM = 15625000UL / pData->zcCtrl.refIntervalHR;
    }
    else
#endif
    {
        #define ERPM_CONST_SCHED ((uint32_t)TIMER1_FREQ_HZ * 10UL)
        eRPM = ERPM_CONST_SCHED / sp;
    }

    /* AM32-style timing advance: fixed fraction of commutation interval.
     * advance = (interval / 8) * advance_level
     * waitTime = interval / 2 - advance
     *
     * advance_level: 0=0°, 1=7.5°, 2=15°, 3=22.5°
     *
     * Speed-adaptive: AdaptiveTimingAdvance() backs off TAL at high
     * eRPM so the delayHR budget stays larger than the detection
     * pipeline latency. See AdaptiveTimingAdvance() comment for the
     * curve and the measurement that motivated it. */
#if FEATURE_IC_ZC
    uint8_t tal = AdaptiveTimingAdvance(eRPM);
#else
    uint8_t tal = TIMING_ADVANCE_LEVEL;
#endif
    uint16_t halfPeriod = sp >> 1;
    uint16_t advance = (sp >> 3) * tal;
    uint16_t delay = (halfPeriod > advance) ? (halfPeriod - advance) : 1;

#if FEATURE_IC_ZC
    /* Hardware com timer with high-resolution timing. */
    if (pData->state == ESC_CLOSED_LOOP)
    {
        if (pData->timing.hasPrevZcHR &&
            pData->timing.stepPeriodHR > 0 &&
            pData->timing.stepPeriod < HR_MAX_STEP_PERIOD)
        {
            /* Phase 4: schedule from protected refIntervalHR.
             * No min-of-recent shortcut — acceleration comes from
             * speed governor, not from aggressive scheduling. */
            uint16_t spHR = (pData->zcCtrl.refIntervalHR > 0)
                          ? pData->zcCtrl.refIntervalHR
                          : pData->timing.stepPeriodHR;

            /* AM32-style: half interval minus advance fraction.
             * Uses the same speed-adaptive `tal` computed above so
             * the HR scheduling path tracks the Timer1 path. */
            uint16_t halfHR = spHR >> 1;
            uint16_t advHR = (spHR >> 3) * tal;
            uint16_t delayHR = (halfHR > advHR) ? (halfHR - advHR) : 2;

            uint16_t targetHR = pData->timing.lastZcTickHR + delayHR;

            /* Shadow delta: what would the predictor schedule vs reactive?
             * Predictor formula: lastZcHR + (predStepHR - predZcOffsetHR)
             * This is the ZC-anchored next-comm from learned states. */
            if (pData->zcPred.predStepHR > 0 &&
                pData->zcPred.predZcOffsetHR > 0 &&
                pData->zcPred.predStepHR > pData->zcPred.predZcOffsetHR)
            {
                uint16_t predTargetHR = pData->timing.lastZcTickHR +
                    (pData->zcPred.predStepHR - pData->zcPred.predZcOffsetHR);
                int16_t delta = (int16_t)(predTargetHR - targetHR);
                pData->zcPred.predVsReactiveDelta = delta;

                /* Track consecutive small deltas for entry validation */
                int16_t absDelta = (delta >= 0) ? delta : -delta;
                if (absDelta <= 10)  /* ~6.4µs */
                {
                    if (pData->zcPred.deltaOkCount < 255)
                        pData->zcPred.deltaOkCount++;
                }
                else
                {
                    pData->zcPred.deltaOkCount = 0;
                }
            }

            /* Check if target is in the past.
             * At high speed (TpHR < 200), filter + compute latency can
             * exceed the ZC-to-commutation delay. If we miss the window,
             * commutate ASAP via hardware timer rather than falling through
             * to the imprecise Timer1 path (50µs jitter at Tp:2). */
            int16_t margin = (int16_t)(targetHR - HAL_ComTimer_ReadTimer());
            pData->zcDiag.lastScheduleMarginHR = margin;
            if (margin <= 0)
            {
                targetHR = HAL_ComTimer_ReadTimer() + 2;  /* Fire ASAP */
                pData->zcDiag.diagTargetPast++;
            }
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
void RecordZcTiming(volatile GARUDA_DATA_T *pData,
                    uint16_t zcTick, uint16_t hrTick)
{
#if FEATURE_DMA_ZC_DIRECT
    /* ── Hybrid DMA-direct ZC timestamp substitution ───────────────────
     * Above a speed threshold the poll path's latency eats a significant
     * fraction of the step period. Replace hrTick with the hardware
     * DMA-captured edge closest to it (within a sanity window).
     * Below the threshold, or if DMA has no matching capture, use poll
     * unchanged — worst case equals today's behavior. */
    if (pData->timing.hasPrevZcHR &&
        pData->timing.stepPeriodHR > 0 &&
        pData->timing.stepPeriodHR < DMA_ZC_DIRECT_THRESHOLD_HR)
    {
        bool risingZc = (commutationTable[pData->currentStep].zcPolarity > 0);
        int16_t correction = 0;
        uint16_t refined = HAL_ZcDma_RefineTimestamp(
            hrTick,
            DMA_ZC_DIRECT_MAX_CORRECTION_HR,
            risingZc,
            &correction);

        if (refined != hrTick)
        {
            /* Successfully substituted — hrTick is now the hardware edge */
            hrTick = refined;
            pData->dmaShadow.substituteCount++;
            pData->dmaShadow.lastCorrectionHR = correction;
            if (correction < pData->dmaShadow.minCorrectionHR)
                pData->dmaShadow.minCorrectionHR = correction;
            if (correction > pData->dmaShadow.maxCorrectionHR)
                pData->dmaShadow.maxCorrectionHR = correction;
        }
        else
        {
            /* DMA had no match in range — use poll */
            pData->dmaShadow.substituteSkipRange++;
        }
    }
    else if (pData->timing.hasPrevZcHR)
    {
        /* Below speed threshold — poll path is better at low speed */
        pData->dmaShadow.substituteSkipGated++;
    }
#endif /* FEATURE_DMA_ZC_DIRECT */

    if (pData->timing.hasPrevZc)
    {
        uint16_t interval = zcTick - pData->timing.lastZcTick;

        /* Reject impossibly short intervals — PWM noise, not real BEMF.
         * Floor must be below stepPeriod to accept valid ZCs at high speed.
         * At Tp:5, Timer1 intervals are 4-5 ticks. Beyond Tp:5 (67k eRPM),
         * intervals drop to 2-3 ticks. Floor of 2 accepts all valid ZCs
         * while the deglitch filter handles noise rejection. */
        /* Reject impossibly short intervals using HR timer for precision.
         * At Tp=3 Timer1, the HR equivalent is ~234 ticks — 0.4% error vs 33%.
         * Uses HR checkpoint (per-revolution validated) so a corrupted IIR
         * can't weaken this check. */
        if (interval < ZC_ABSOLUTE_MIN_INTERVAL)
        {
            pData->icZc.diagFalseZc++;
            if (commutationTable[pData->currentStep].zcPolarity > 0)
                pData->zcDiag.diagRisingRejects++;
            else
                pData->zcDiag.diagFallingRejects++;
            pData->bemf.zeroCrossDetected = false;
            return;
        }
        /* HR-based 50% rejection when available */
        {
            uint16_t hrElapsed = hrTick - pData->timing.lastZcTickHR;
            uint16_t hrHalf = pData->timing.checkpointStepPeriodHR > 0
                ? pData->timing.checkpointStepPeriodHR / 2
                : pData->timing.stepPeriodHR / 2;
            if (hrHalf > 0 && hrElapsed < hrHalf)
            {
                pData->icZc.diagFalseZc++;
                if (commutationTable[pData->currentStep].zcPolarity > 0)
                    pData->zcDiag.diagRisingRejects++;
                else
                    pData->zcDiag.diagFallingRejects++;
                pData->bemf.zeroCrossDetected = false;
                return;
            }
        }
        /* Fallback: Timer1-based 50% check */
        uint16_t minInterval = pData->timing.checkpointStepPeriod / 2;
        if (minInterval < 2) minInterval = 2;
        if (interval < minInterval)
        {
            pData->icZc.diagFalseZc++;
            if (commutationTable[pData->currentStep].zcPolarity > 0)
                pData->zcDiag.diagRisingRejects++;
            else
                pData->zcDiag.diagFallingRejects++;
            pData->bemf.zeroCrossDetected = false;
            return;
        }

        /* Interval valid — commit timestamps */
        pData->timing.prevZcTick = pData->timing.lastZcTick;
        pData->timing.lastZcTick = zcTick;
        pData->timing.stepsSinceLastZc = 0;

        /* Diagnostic: ZC position within detection window (0-255).
         * Window = originalTimeoutHR (captured at commutation) - blanking.
         * Latency = ZC HR tick - blanking end HR tick.
         * Using originalTimeoutHR instead of current forcedCountdown
         * gives an accurate percentage that doesn't shift as the step
         * progresses. */
        {
            uint16_t timeoutHR = pData->zcCtrl.originalTimeoutHR;
            uint16_t blkHR = pData->zcDiag.lastBlankingHR;
            if (timeoutHR > blkHR)
            {
                uint16_t wnd = (uint16_t)(timeoutHR - blkHR);
                uint16_t latency = hrTick - pData->icZc.blankingEndHR;
                uint32_t pct = ((uint32_t)latency * 255u) / wnd;
                pData->zcDiag.zcLatencyPct =
                    (pct > 255u) ? 255u : (uint8_t)pct;
            }
            else
            {
                pData->zcDiag.zcLatencyPct = 0u;
            }
        }

        pData->timing.prevZcInterval = pData->timing.zcInterval;
        pData->timing.zcInterval = interval;

        /* REJECT multi-step intervals from IIR — AM32-inspired. */
        uint16_t maxInterval = pData->timing.stepPeriod +
                               (pData->timing.stepPeriod >> 1); /* 1.5× */
        if (maxInterval < 12) maxInterval = 12;

        if (interval <= maxInterval)
        {
            uint16_t oldPeriod = pData->timing.stepPeriod;

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

            /* IIR shrink clamp — AM32-inspired max 25% decrease per update.
             * Prevents a single false short ZC from halving the period.
             * The 3:1 IIR alone allows ~25% change, but when the measured
             * interval is ~50% of real (false ZC via bypass), the IIR
             * output drops ~12.5% per step. Over several steps this
             * compounds to the 300→140 collapse seen in bench data.
             * Clamping at 75% of old value limits the damage.
             * Growth (deceleration) is unclamped — motor must be able
             * to slow down freely for safety. */
            uint16_t minPeriod = oldPeriod - (oldPeriod >> 2);  /* 75% */
            if (minPeriod < MIN_CL_STEP_PERIOD)
                minPeriod = MIN_CL_STEP_PERIOD;
            if (pData->timing.stepPeriod < minPeriod)
                pData->timing.stepPeriod = minPeriod;
        }

        /* Floor: prevent step period from going below physical limit. */
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
                             (pData->timing.stepPeriodHR >> 1);  /* 1.5× */
            if (maxHR < 100) maxHR = 100;

            if (intervalHR <= maxHR)
            {
                uint16_t oldHR = pData->timing.stepPeriodHR;

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

                /* HR IIR shrink clamp — max 25% decrease per update */
                uint16_t minPeriodHR = oldHR - (oldHR >> 2);
                if (minPeriodHR < 10) minPeriodHR = 10;
                if (pData->timing.stepPeriodHR < minPeriodHR)
                    pData->timing.stepPeriodHR = minPeriodHR;
            }

            /* Phase 4: Update protected reference interval.
             * refIntervalHR is what the scheduler uses. It tracks
             * stepPeriodHR but with independent shrink/growth clamps.
             * This prevents one bad interval from tightening the
             * schedule even if the IIR moves slightly. */
            {
                uint16_t ref = pData->zcCtrl.refIntervalHR;
                uint16_t raw = pData->timing.stepPeriodHR;
                if (ref == 0) {
                    ref = raw;  /* first ZC — seed directly */
                } else {
                    /* Shrink clamp: max 25% per update */
                    uint16_t minRef = ref - (ref >> 2);
                    if (minRef < 10) minRef = 10;
                    if (raw < minRef) raw = minRef;
                    /* Growth clamp: max 50% per update */
                    uint16_t maxRef = ref + (ref >> 1);
                    if (raw > maxRef) raw = maxRef;
                    /* IIR */
                    ref = (uint16_t)(((uint32_t)ref * 3 + raw) >> 2);
                }
                pData->zcCtrl.refIntervalHR = ref;
                pData->zcCtrl.rawIntervalHR = pData->timing.zcIntervalHR;
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

#if FEATURE_IC_DMA_SHADOW
    /* DMA shadow probe — compare the hardware-precise DMA ring against
     * the poll-accepted timestamp and the predictor's expected ZC.
     * Results are logged into dmaShadow for telemetry. Shadow-only:
     * no effect on live scheduling. */
    if (pData->timing.hasPrevZcHR)
    {
        uint16_t prevHR       = pData->timing.prevZcTickHR;
        uint16_t prevInterval = pData->timing.prevZcIntervalHR;
        if (prevInterval == 0) prevInterval = pData->timing.zcIntervalHR;
        uint16_t halfInterval = prevInterval >> 1;
        uint16_t windowOpenHR = prevHR + halfInterval;
        uint16_t expectedHR   = prevHR + prevInterval;
        bool risingZc = (commutationTable[pData->currentStep].zcPolarity > 0);

        HAL_ZcDma_Result probe;
        HAL_ZcDma_Probe(windowOpenHR, expectedHR, hrTick, risingZc, &probe);

        pData->dmaShadow.stepCount++;
        pData->dmaShadow.edgesInWindowSum += probe.edgeCount;
        pData->dmaShadow.lastEdgeCount = probe.edgeCount;
        pData->dmaShadow.lastFound = probe.found;
        if (probe.ringWrappedSinceMark)
            pData->dmaShadow.ringOverflowCount++;
        if (probe.found)
        {
            pData->dmaShadow.matchCount++;
            pData->dmaShadow.lastEarliestVsPoll     = probe.earliestVsPoll;
            pData->dmaShadow.lastEarliestVsExpected = probe.earliestVsExpected;
            pData->dmaShadow.lastClosestVsExpected  = probe.closestVsExpected;
        }
        pData->dmaShadow.lastPollVsExpected =
            (int16_t)(hrTick - expectedHR);

#if FEATURE_DMA_BURST_CAPTURE
        /* Snapshot this step for the burst capture (research tool). */
        HAL_DmaBurst_OnZc(hrTick, expectedHR, false);
#endif
    }
#endif

    /* Per-polarity and per-step diagnostic counters */
    if (commutationTable[pData->currentStep].zcPolarity > 0)
        pData->zcDiag.diagRisingZcCount++;
    else
        pData->zcDiag.diagFallingZcCount++;
    if (pData->currentStep < 6)
        pData->zcDiag.stepAccepted[pData->currentStep]++;

    if (pData->timing.goodZcCount >= ZC_SYNC_THRESHOLD)
        pData->timing.zcSynced = true;

    /* First confirmed CL ZC clears bypass suppression */
    pData->timing.bypassSuppressed = false;

    /* ── PLL Predictor: phase error computation (Step 1 shadow) ──────
     * Compare actual ZC against TWO models to isolate error source:
     * Model A (nominal): predZcHR = comm + half + advance
     * Model B (reactive): comm + lastRealZcDelay (empirical)
     *
     * Positive phaseErr = ZC arrived later than predicted.
     * Negative phaseErr = ZC arrived earlier than predicted. */
    if (pData->zcPred.predZcHR != 0 && pData->zcPred.predStepHR > 0)
    {
        /* Model A: nominal predictor */
        int16_t phaseErr = (int16_t)(hrTick - pData->zcPred.predZcHR);
        pData->zcPred.phaseErrHR = phaseErr;

        /* Model B: reactive (comm + last observed ZC delay) */
        if (pData->zcPred.lastRealZcDelayHR > 0)
        {
            uint16_t reactiveZcHR = pData->zcPred.lastCommHR
                                    + pData->zcPred.lastRealZcDelayHR;
            pData->zcPred.phaseErrReactiveHR =
                (int16_t)(hrTick - reactiveZcHR);
        }

        /* Record actual comm-to-ZC delay */
        uint16_t actualDelayHR =
            (uint16_t)(hrTick - pData->zcPred.lastCommHR);
        pData->zcPred.lastRealZcDelayHR = actualDelayHR;

        /* Update adaptive phase offset via IIR: 7/8 old + 1/8 measured.
         * Always update when predictor has valid state — the predictor
         * must keep tracking to converge, even when not locked/gated.
         * The 25%-75% clamp protects against outlier corruption.
         * RED zone ZCs still update (they're accepted, just not trusted
         * for predictive scheduling). */
        if (pData->zcPred.predZcOffsetHR > 0)
        {
            uint16_t minOff = pData->zcPred.predStepHR >> 2;  /* 25% */
            uint16_t maxOff = pData->zcPred.predStepHR -
                              (pData->zcPred.predStepHR >> 2);  /* 75% */
            uint16_t clampedDelay = actualDelayHR;
            if (clampedDelay < minOff) clampedDelay = minOff;
            if (clampedDelay > maxOff) clampedDelay = maxOff;
            pData->zcPred.predZcOffsetHR = (uint16_t)(
                ((uint32_t)pData->zcPred.predZcOffsetHR * 7
                 + clampedDelay) >> 3);
        }
        else if (pData->zcPred.predZcOffsetHR == 0 &&
                 actualDelayHR > 0)
        {
            /* Re-seed after RED zone disarm */
            pData->zcPred.predZcOffsetHR = actualDelayHR;
        }

        /* Accumulate |phaseErr| for telemetry averaging */
        uint16_t absErr = (phaseErr >= 0) ? (uint16_t)phaseErr
                                          : (uint16_t)(-phaseErr);
        pData->zcPred.diagPhaseErrAccum += absErr;

        /* Check if ZC fell within scan window */
        int16_t sinceOpen = (int16_t)(hrTick - pData->zcPred.scanOpenHR);
        int16_t sinceClose = (int16_t)(hrTick - pData->zcPred.scanCloseHR);
        if (sinceOpen >= 0 && sinceClose <= 0)
            pData->zcPred.diagZcInWindow++;
        else
            pData->zcPred.diagZcOutWindow++;

        /* Frequency correction: adjust predStepHR toward actual interval.
         * Small IIR: 7/8 old + 1/8 correction. */
        if (pData->timing.hasPrevZcHR && pData->timing.zcIntervalHR > 0)
        {
            uint16_t actualStepHR = pData->timing.zcIntervalHR;
            uint16_t minStep = pData->zcPred.predStepHR >> 1;
            uint16_t maxStep = pData->zcPred.predStepHR +
                               (pData->zcPred.predStepHR >> 1);
            if (actualStepHR >= minStep && actualStepHR <= maxStep)
            {
                pData->zcPred.predStepHR = (uint16_t)(
                    ((uint32_t)pData->zcPred.predStepHR * 7 + actualStepHR)
                    >> 3);
            }
        }

        /* Mark predictor as locked if phase error is small enough
         * (within 12.5% of step period) */
        if (absErr < (pData->zcPred.predStepHR >> 3))
        {
            pData->zcPred.locked = true;
            pData->zcPred.missCount = 0;
        }
        else if (pData->zcPred.predictiveMode)
        {
            /* In predictive mode, ANY accepted ZC is a correction
             * arriving — reset missCount even if not "locked".
             * The lock check is for entry decisions, not for proving
             * the predictor is still receiving feedback. */
            pData->zcPred.missCount = 0;
        }

        /* Compute zone classification for entryScore.
         * Uses the scan window set in OnCommutation. */
        bool inGreen = false;
        bool inRed = false;
        {
            int16_t sinceOpen = (int16_t)(hrTick - pData->zcPred.scanOpenHR);
            int16_t sinceClose = (int16_t)(hrTick - pData->zcPred.scanCloseHR);
            inGreen = (sinceOpen >= 0 && sinceClose <= 0);
            if (!inGreen)
            {
                uint16_t outerHR = pData->zcPred.predStepHR >> 1;
                uint16_t outerOpen = pData->zcPred.predZcHR - outerHR;
                uint16_t outerClose = pData->zcPred.predZcHR + outerHR;
                int16_t sinceOO = (int16_t)(hrTick - outerOpen);
                int16_t sinceOC = (int16_t)(hrTick - outerClose);
                inRed = !(sinceOO >= 0 && sinceOC <= 0);
            }
        }

        /* ── entryScore: predictive scheduling readiness ──────────
         * Evaluated HERE with the CURRENT ZC's refreshed data
         * (phaseErr, locked, offset, stepHR all just updated).
         *
         * Scoring:
         *   GREEN + small |phaseErr| (< stepHR/6): +4
         *   GREEN + moderate |phaseErr|:           +1
         *   YELLOW:                                -1
         *   RED / large |phaseErr| (> stepHR/4):   clear to 0
         *
         * Entry threshold: >= 24 (~1 revolution of good ZCs).
         * Decay-based: occasional YELLOW doesn't kill progress. */
        {
            uint16_t smallErr = pData->zcPred.predStepHR / 6;
            uint16_t largeErr = pData->zcPred.predStepHR >> 2;

            if (inGreen && absErr < smallErr)
            {
                if (pData->zcPred.entryScore <= 251)
                    pData->zcPred.entryScore += 4;
                else
                    pData->zcPred.entryScore = 255;
            }
            else if (inGreen)
            {
                if (pData->zcPred.entryScore < 255)
                    pData->zcPred.entryScore++;
            }
            else if (inRed || absErr > largeErr)
            {
                pData->zcPred.entryScore = 0;
            }
            else /* YELLOW */
            {
                if (pData->zcPred.entryScore > 0)
                    pData->zcPred.entryScore--;
            }
        }

        /* ── Predictive mode entry (post-ZC, current state) ──────
         * Arm handoff when:
         * - entryScore high enough (predictor quality proven)
         * - predictor has valid state
         * - reactive path is under pressure (recent low margin
         *   or recent targetPast — not just high speed)
         * - not already in predictive mode or pending handoff
         *
         * The actual scheduling takeover happens in _CCP4Interrupt
         * on the NEXT commutation after handoffPending is set. */
        /* Predictive entry DISABLED — validating baseline reactive
         * SCCP3 scheduler first. Once IsrE climbs cleanly across the
         * entire speed range with no plateaus or stalls, this can be
         * re-enabled (but the ISR's `thisCommHR = CCP4RA` was wrong
         * for SCCP3 anyway and now reads CCP4TMRL — predictor handoff
         * math will need to be revisited before re-enabling). */
#if 0
        if (!pData->zcPred.predictiveMode &&
            !pData->zcPred.handoffPending &&
            pData->zcPred.entryScore >= 24 &&
            pData->zcPred.predStepHR > 0 &&
            pData->zcDiag.lastScheduleMarginHR < 10)
        {
            pData->zcPred.handoffPending = true;
        }
#endif
    }

    /* ZC V2 mode transitions on good ZC */
    switch (pData->zcCtrl.mode)
    {
        case ZC_MODE_ACQUIRE:
            pData->zcCtrl.acquireGoodCount++;
            if (pData->zcCtrl.acquireGoodCount >= ZC_ACQUIRE_GOOD_ZC)
                ZcEnterTrack(pData);
            break;

        case ZC_MODE_RECOVER:
            pData->zcCtrl.recoverGoodCount++;
            if (pData->zcCtrl.recoverGoodCount >= ZC_RECOVER_GOOD_ZC)
                ZcEnterAcquire(pData);
            break;

        case ZC_MODE_TRACK:
            /* Normal running — stay in TRACK */
            break;
    }
}

/**
 * @brief Fast poll ZC detection — called from SCCP1 timer ISR at 200kHz.
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

    /* BLANKING → ARMED transition.
     * Two-layer rejection:
     *   Layer 1: Fixed minimum blanking (~25µs) — clears switching transients
     *   Layer 2: 50% interval rejection — rejects demag and early noise */
    if (pData->icZc.phase == IC_ZC_BLANKING)
    {
        /* Layer 1: Fixed minimum blanking (hardware transient settling) */
        int16_t dt = (int16_t)(HAL_ComTimer_ReadTimer() -
                               pData->icZc.blankingEndHR);
        if (dt < 0)
            return false;  /* Still in minimum blanking */

        pData->icZc.phase = IC_ZC_ARMED;
        pData->icZc.pollFilter = 0;
#if FEATURE_IC_ZC_CAPTURE
        /* Arm SCCP2 IC now that blanking has expired.
         * One-shot: ISR disables itself on first capture. */
        if (!pData->icZc.icArmed)
        {
            HAL_ZcIC_Arm();
            pData->icZc.icArmed = true;
        }
#endif
    }

    if (pData->icZc.phase != IC_ZC_ARMED)
        return false;

    /* Layer 2: 50% interval rejection (ESCape32/VESC approach).
     * Reject any ZC candidate that arrives before 50% of the last
     * ZC-to-ZC interval. This is the production-grade blanking that
     * works for ANY motor without per-motor tuning.
     *
     * Why 50%: demagnetization completes within ~25-35% of step period.
     * True ZC occurs at ~50% (30 electrical degrees). Everything before
     * 50% is noise, demag, or switching artifacts. */
    {
        uint16_t elapsed = HAL_ComTimer_ReadTimer() - pData->timing.lastZcTickHR;
        /* Phase 4: use protected refIntervalHR for half-interval rejection.
         * The raw stepPeriodHR can be corrupted by aggressive IIR;
         * refIntervalHR has independent shrink/growth clamps. */
        uint16_t ref = pData->zcCtrl.refIntervalHR;
        if (ref == 0) ref = pData->timing.stepPeriodHR;  /* fallback */
        uint16_t halfInterval = ref >> 1;
        if (halfInterval > 0 && (int16_t)(elapsed - halfInterval) < 0)
            return false;  /* Too early — reject (demag/noise) */
    }

    /* Read comparator for the floating phase. */
    uint8_t cmp = ReadBEMFComparator(pData->icZc.activeChannel);

    /* Raw comparator edge trace — count transitions per step */
    {
        uint8_t step = pData->currentStep;
        if (step < 6)
        {
            pData->icZc.stepPolls[step]++;
            if (cmp != pData->icZc.lastCmpState && pData->icZc.lastCmpState != 0xFF)
                pData->icZc.stepFlips[step]++;
            pData->icZc.lastCmpState = cmp;
        }
    }

    /* ZC V2 Phase 3: strict polarity in all modes.
     * Never accept wrong-polarity comparator state. */
    uint8_t expected = pData->bemf.cmpExpected;

    /* Read raw GPIO in parallel with CLC for corroboration (TRACK only).
     * Raw is NOT the primary detector — it's a sanity check to catch
     * stale CLC D-FF holds and noise-coincidence fast accepts. */
    uint8_t rawCmp = ReadRawBEMFComparator(pData->icZc.activeChannel);

    if (cmp == expected)
    {
        /* Record first CLC match timestamp for candidateAge fallback */
        if (!pData->icZc.hasFirstClcMatch)
        {
            pData->icZc.firstClcMatchHR = HAL_ComTimer_ReadTimer();
            pData->icZc.hasFirstClcMatch = true;
        }

        /* Raw corroboration & stability (candidate-local, TRACK only).
         * rawCoro: saturating 0..2 counter of consecutive raw matches.
         * rawFirstMatchHR: scoped to CLC candidate — only starts when
         * CLC==expected AND raw==expected. Resets on ANY raw mismatch
         * (not just CLC mismatch) to prevent spanning chatter. */
        if (pData->zcCtrl.mode == ZC_MODE_TRACK)
        {
            if (rawCmp == expected)
            {
                if (pData->icZc.rawCoro < 2u)
                    pData->icZc.rawCoro++;
                if (!pData->icZc.hasRawFirstMatch)
                {
                    pData->icZc.rawFirstMatchHR = HAL_ComTimer_ReadTimer();
                    pData->icZc.rawFirstMatchT1 = pData->timer1Tick;
                    pData->icZc.hasRawFirstMatch = true;
                }
            }
            else
            {
                /* Raw mismatch — reset raw stability entirely */
                pData->icZc.rawCoro = 0;
                pData->icZc.hasRawFirstMatch = false;
            }
        }

        if (pData->icZc.pollFilter == 0)
        {
#if FEATURE_IC_ZC_CAPTURE
            /* PWM-aware IC timestamp age validation (no speed gate).
             * IC may catch a bounce; if stale, use poll time.
             *
             * NOTE: previous attempts to make this adaptive
             * (proportional to step period) and to re-arm on
             * age-reject both caused regressions in the 60–70 k
             * eRPM band (Mrgn went negative). Reverted to the
             * known-working fixed threshold. The IC contribution
             * is low (~1–3% of timestamps) but the system runs
             * cleanly on raw poll and the schedule margin stays
             * positive across the working speed range. */
            if (pData->icZc.icCandidateValid)
            {
                uint16_t now = HAL_ComTimer_ReadTimer();
                uint16_t icAge = now - pData->icZc.zcCandidateHR;
                if (icAge > IC_AGE_MAX_HR)
                {
                    pData->icZc.zcCandidateHR = now;
                    pData->icZc.zcCandidateT1 = pData->timer1Tick;
                    pData->icZc.icCandidateValid = false;
                    pData->icZc.diagIcAgeReject++;
                }
            }
            else if (pData->icZc.icArmed)
#else
            if (true)
#endif
            {
                pData->icZc.zcCandidateHR = HAL_ComTimer_ReadTimer();
                pData->icZc.zcCandidateT1 = pData->timer1Tick;
            }
        }
        pData->icZc.pollFilter++;

        /* ── TRACK acceptance: detector agreement, no speed gate ──────
         *
         * Fast accept: pollFilter >= 1 && rawStable
         *   rawStable = rawCoro >= 2 (primary: two consecutive raw matches)
         *            OR (hasRawFirstMatch && rawAge >= RAW_STABLE_AGE_HR)
         *               (jitter insurance: raw stable for >= 1 poll period,
         *                covers case where rawCoro is exactly 1 but raw
         *                has been consistent long enough)
         *
         * Safe fallback: pollFilter >= 2 && candidateAge >= 1 PWM cycle
         *   CLC persisted through a fresh D-FF update (not stale).
         *
         * ACQUIRE/RECOVER: full adaptive FL (3-8). */
        bool accept = false;
        if (pData->zcCtrl.mode == ZC_MODE_TRACK)
        {
            bool rawStable = false;
            if (pData->icZc.rawCoro >= 2u)
            {
                rawStable = true;  /* primary: two consecutive raw matches */
            }
            else if (pData->icZc.hasRawFirstMatch && pData->icZc.rawCoro >= 1u)
            {
                uint16_t rawAge = HAL_ComTimer_ReadTimer()
                                  - pData->icZc.rawFirstMatchHR;
                if (rawAge >= RAW_STABLE_AGE_HR)
                    rawStable = true;  /* jitter insurance: stable for 1 poll */
            }

            if (pData->icZc.pollFilter >= 1u && rawStable)
            {
                accept = true;  /* fast path: CLC + raw stably agree */
            }
            else if (pData->icZc.pollFilter >= 2u)
            {
                uint16_t candidateAge = HAL_ComTimer_ReadTimer()
                                        - pData->icZc.firstClcMatchHR;
                if (candidateAge >= PWM_PERIOD_HR)
                {
                    accept = true;  /* fallback: CLC survived a D-FF refresh */
                    pData->icZc.diagTrackFallback++;
                }
                else if (pData->icZc.rawCoro >= 1u)
                {
                    /* Raw was present but not stable enough for fast path */
                    pData->icZc.diagRawStableBlock++;
                }
                else
                {
                    pData->icZc.diagRawVeto++;
                }
            }
        }
        else
        {
            /* ACQUIRE/RECOVER: use full adaptive filter */
            if (pData->icZc.pollFilter >= pData->icZc.filterLevel)
                accept = true;
        }

        if (accept)
        {
            /* ── Timestamp selection by confidence ────────────────────
             * Select best ZC timestamp AFTER acceptance is decided.
             * IC lead check is a timestamp downgrade only — never
             * affects whether the ZC is accepted.
             *
             * Preferred: IC (if valid and doesn't lead raw too much)
             * Fallback 1: rawFirstMatchHR (first stable raw observation)
             * Fallback 2: firstClcMatchHR (first CLC match)
             * Fallback 3: current poll time */
            uint16_t tsHR = pData->icZc.zcCandidateHR;
            uint16_t tsT1 = pData->icZc.zcCandidateT1;

#if FEATURE_IC_ZC_CAPTURE
            if (pData->icZc.icCandidateValid && pData->icZc.hasRawFirstMatch)
            {
                /* Check IC lead vs raw stability */
                int16_t icLead = (int16_t)(pData->icZc.rawFirstMatchHR
                                           - pData->icZc.zcCandidateHR);
                if (icLead > (int16_t)IC_LEAD_MAX_HR)
                {
                    /* IC fired too far ahead of raw — bounce, downgrade */
                    tsHR = pData->icZc.rawFirstMatchHR;
                    tsT1 = pData->icZc.rawFirstMatchT1;
                    pData->icZc.diagIcLeadReject++;
                    pData->icZc.diagTsFromRaw++;
                }
                else
                {
                    /* IC is fresh and consistent with raw — best source */
                    pData->icZc.diagTsFromIc++;
                }
            }
            else if (pData->icZc.icCandidateValid)
            {
                /* IC valid but no raw reference — trust IC */
                pData->icZc.diagTsFromIc++;
            }
            else
#endif
            if (pData->icZc.hasRawFirstMatch)
            {
                /* No valid IC — use raw first match */
                tsHR = pData->icZc.rawFirstMatchHR;
                tsT1 = pData->icZc.rawFirstMatchT1;
                pData->icZc.diagTsFromRaw++;
            }
            else if (pData->icZc.hasFirstClcMatch)
            {
                /* No raw either — use CLC first match */
                tsHR = pData->icZc.firstClcMatchHR;
                tsT1 = pData->icZc.zcCandidateT1;  /* best T1 available */
                pData->icZc.diagTsFromClc++;
            }
            else
            {
                /* Nothing — use current poll time (already in tsHR/tsT1) */
                pData->icZc.diagTsFromPoll++;
            }

            /* ── Step 2: 3-zone confidence model ─────────────────────
             * Window supervises the predictor, does NOT gate acceptance.
             * ZC is always accepted if Phase 2 says valid.
             *
             * GREEN: inside ±25% → normal predictor update, trust high
             * YELLOW: outside ±25% but inside ±50% → accept ZC, skip
             *         predictor correction, count warning
             * RED: outside ±50% → accept ZC, disarm predictor trust,
             *      do not use for predictor update */
            if (pData->zcCtrl.mode == ZC_MODE_TRACK &&
                pData->zcPred.predZcOffsetHR > 0)
            {
                int16_t sinceOpen = (int16_t)(tsHR - pData->zcPred.scanOpenHR);
                int16_t sinceClose = (int16_t)(tsHR - pData->zcPred.scanCloseHR);
                bool inGreen = (sinceOpen >= 0 && sinceClose <= 0);

                /* Outer guard band: ±50% of step period */
                uint16_t outerHR = pData->zcPred.predStepHR >> 1;
                uint16_t outerOpen = pData->zcPred.predZcHR - outerHR;
                uint16_t outerClose = pData->zcPred.predZcHR + outerHR;
                int16_t sinceOuterOpen = (int16_t)(tsHR - outerOpen);
                int16_t sinceOuterClose = (int16_t)(tsHR - outerClose);
                bool inYellow = !inGreen &&
                    (sinceOuterOpen >= 0 && sinceOuterClose <= 0);
                bool inRed = !inGreen && !inYellow;

                /* Telemetry */
                if (pData->zcPred.gateActive)
                {
                    if (inGreen)
                        pData->zcPred.diagWinCandInGated++;
                    else
                        pData->zcPred.diagWinCandOutGated++;
                }

                if (!inGreen)
                {
                    if (sinceOpen < 0)
                        pData->zcPred.diagWinOutEarly++;
                    else
                        pData->zcPred.diagWinOutLate++;
                }

                /* Predictor update weighting based on zone.
                 * Set a flag that RecordZcTiming checks before
                 * updating predZcOffsetHR. */
                if (inRed)
                {
                    /* RED: disarm gate but DON'T touch handoffPending
                     * or predictiveMode. The handoff race is too tight
                     * to allow RED events to cancel pending entries. */
                    pData->zcPred.gateActive = false;
                    pData->zcPred.gateArmCount = 0;
                    pData->zcPred.diagWindowReject++;
                    pData->zcPred.deltaOkCount = 0;
                    /* RED zone disarms gate but does NOT exit predictive
                     * mode. Predictive exits are only via missCount > 4
                     * (no ZC corrections) or timeout (genuine desync).
                     * RED during predictive is expected at handoff when
                     * phase shifts — the predictor needs time to adapt. */
                    if (pData->zcPred.predictiveMode)
                        pData->zcPred.diagPredExitRed++;  /* count but don't exit */
                }
                else if (inYellow)
                {
                    /* YELLOW: accept, count warning, reduce confidence */
                    pData->zcPred.gateRevRejects++;
                    pData->zcPred.diagWinCandOutGated++;
                }
                /* GREEN: normal — predictor update proceeds in RecordZcTiming */

                /* gateActive hysteresis (Option C: stability-based):
                 * Arm after 12 consecutive locked+GREEN ZCs.
                 * Disarm on RED or timeout (timeout handled elsewhere).
                 * Disarm on 2+ YELLOW in one revolution. */
                if (pData->zcPred.locked && inGreen)
                {
                    if (pData->zcPred.gateArmCount < 255)
                        pData->zcPred.gateArmCount++;
                    if (pData->zcPred.gateArmCount >= 12 &&
                        !pData->zcPred.gateActive)
                        pData->zcPred.gateActive = true;
                }
                else if (!inGreen)
                {
                    pData->zcPred.gateArmCount = 0;
                }
            }

            if (accept)
            {
                pData->bemf.zeroCrossDetected = true;
                RecordZcTiming(pData, tsT1, tsHR);

                if (pData->bemf.zeroCrossDetected)
                {
                    pData->icZc.diagAccepted++;
                    pData->icZc.phase = IC_ZC_DONE;
                    return true;
                }
                /* RecordZcTiming rejected (too-short interval) — reset */
                pData->icZc.pollFilter = 0;
                pData->icZc.rawCoro = 0;
                pData->icZc.hasFirstClcMatch = false;
                pData->icZc.hasRawFirstMatch = false;
            }
        }
    }
    else
    {
        /* CLC mismatch — reset entire candidate state.
         * Bounce-tolerant decrement for pollFilter, but raw corroboration,
         * raw stability, and first-match are candidate-local so they reset. */
        if (pData->icZc.pollFilter > 0)
            pData->icZc.pollFilter--;
        pData->icZc.rawCoro = 0;
        pData->icZc.hasFirstClcMatch = false;
        pData->icZc.hasRawFirstMatch = false;
    }

    return false;
}

#endif /* FEATURE_IC_ZC */
