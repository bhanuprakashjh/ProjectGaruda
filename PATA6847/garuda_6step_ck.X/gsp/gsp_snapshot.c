/**
 * @file gsp_snapshot.c
 * @brief CK board snapshot capture — 48 bytes of 6-step telemetry.
 *
 * Most fields are 8/16-bit (atomic on dsPIC33CK). 32-bit fields
 * may tear but this is acceptable for telemetry display.
 */

#include "../garuda_config.h"

#if FEATURE_GSP && !FEATURE_V4_SECTOR_PI

#include <string.h>
#include "gsp_snapshot.h"
#include "../garuda_types.h"
#include "../garuda_service.h"

void GSP_CaptureSnapshot(GSP_CK_SNAPSHOT_T *dst)
{
    volatile GARUDA_DATA_T *src = &gData;

    memset(dst, 0, sizeof(GSP_CK_SNAPSHOT_T));

    /* Core state */
    dst->state       = (uint8_t)src->state;
    dst->faultCode   = (uint8_t)src->faultCode;
    dst->currentStep = src->currentStep;
    dst->ataStatus   = src->ataLastSIR1;
    dst->potRaw      = src->potRaw;
    dst->dutyPct     = (uint8_t)((uint32_t)src->duty * 100UL / LOOPTIME_TCY);
    dst->zcSynced    = src->timing.zcSynced ? 1 : 0;

    /* Electrical */
    dst->vbusRaw     = src->vbusRaw;
    dst->iaRaw       = src->iaRaw;
    dst->ibRaw       = src->ibRaw;
    dst->ibusRaw     = src->ibusRaw;
    dst->duty        = (uint16_t)src->duty;

    /* Speed/timing */
    dst->stepPeriod   = src->timing.stepPeriod;
    dst->goodZcCount  = src->timing.goodZcCount;
    dst->zcInterval   = src->timing.zcInterval;
    dst->prevZcInterval = src->timing.prevZcInterval;

#if FEATURE_IC_ZC
    dst->stepPeriodHR = src->timing.stepPeriodHR;
#endif

    /* Compute eRPM — only when motor is actually running (OL_RAMP or CL) */
    {
        uint32_t erpm = 0;
        if (src->state >= ESC_OL_RAMP && src->state <= ESC_CLOSED_LOOP)
        {
#if FEATURE_IC_ZC
            if (src->timing.stepPeriodHR > 0 && src->timing.hasPrevZcHR)
                erpm = 15625000UL / src->timing.stepPeriodHR;
            else
#endif
            if (src->timing.stepPeriod > 0)
                erpm = (uint32_t)TIMER1_FREQ_HZ * 10UL / src->timing.stepPeriod;
        }
        dst->eRpm = erpm;
    }

    /* ZC diagnostics */
#if FEATURE_IC_ZC
    dst->icAccepted   = src->icZc.diagAccepted;
    dst->icFalse      = src->icZc.diagFalseZc;
    dst->filterLevel  = src->icZc.filterLevel;
#endif
    dst->missedSteps  = src->timing.consecutiveMissedSteps;
    dst->forcedSteps  = src->timing.stepsSinceLastZc;
    dst->ilimActive   = src->ataIlimActive ? 1 : 0;

    /* System */
    dst->systemTick = src->systemTick;
    dst->uptimeSec  = src->systemTick / 1000;

    /* ZC diagnostics */
    dst->zcLatencyPct  = src->zcDiag.zcLatencyPct;
    if (src->timing.stepPeriodHR > 0)
        dst->zcBlankPct = (uint8_t)((uint32_t)src->zcDiag.lastBlankingHR * 100
                                     / src->timing.stepPeriodHR);
    else
        dst->zcBlankPct = 0;
    dst->zcBypassCount = src->zcDiag.diagBypassAccepted;

    /* ZC V2 diagnostics */
    dst->zcMode = (uint8_t)src->zcCtrl.mode;
    {
        uint16_t fc = src->zcDiag.actualForcedComm;
        dst->actualForcedComm = (fc > 255u) ? 255u : (uint8_t)fc;
    }
    dst->zcTimeoutCount  = src->zcDiag.zcTimeoutCount;
    dst->risingZcCount   = src->zcDiag.diagRisingZcCount;
    dst->fallingZcCount  = src->zcDiag.diagFallingZcCount;
    dst->risingTimeouts  = src->zcDiag.diagRisingTimeouts;
    dst->fallingTimeouts = src->zcDiag.diagFallingTimeouts;

    /* Per-step 0..5 counters */
    {
        uint8_t i;
        for (i = 0; i < 6; i++) {
            dst->stepAccepted[i] = src->zcDiag.stepAccepted[i];
            dst->stepTimeouts[i] = src->zcDiag.stepTimeouts[i];
        }
    }

    /* Raw comparator edge trace */
#if FEATURE_IC_ZC
    {
        uint8_t i;
        for (i = 0; i < 6; i++) {
            dst->stepFlips[i] = src->icZc.stepFlips[i];
            dst->stepPolls[i] = src->icZc.stepPolls[i];
        }
    }

    /* Raw corroboration & IC age diagnostics */
    dst->rawVetoCount      = src->icZc.diagRawVeto;
    dst->icAgeRejectCount  = src->icZc.diagIcAgeReject;
    dst->trackFallbackCount = src->icZc.diagTrackFallback;

    /* Phase 2: raw stability & timestamp source */
    dst->rawStableBlock    = src->icZc.diagRawStableBlock;
    dst->tsFromIc          = src->icZc.diagTsFromIc;
    dst->tsFromRaw         = src->icZc.diagTsFromRaw;
    dst->tsFromClc         = src->icZc.diagTsFromClc;
    dst->tsFromPoll        = src->icZc.diagTsFromPoll;
    dst->icLeadReject      = src->icZc.diagIcLeadReject;

    /* Scheduler margin */
    dst->targetPastCount   = src->zcDiag.diagTargetPast;
    dst->schedMarginHR     = src->zcDiag.lastScheduleMarginHR;

    /* PLL predictor telemetry */
    dst->predPhaseErrHR    = src->zcPred.phaseErrHR;
    dst->predPhaseErrRxHR  = src->zcPred.phaseErrReactiveHR;
    dst->predStepHR        = src->zcPred.predStepHR;
    dst->predZcInWindow    = src->zcPred.diagZcInWindow;
    dst->predZcOutWindow   = src->zcPred.diagZcOutWindow;
    dst->predLocked        = src->zcPred.locked ? 1 : 0;
    dst->predMissCount     = src->zcPred.missCount;
    dst->predMinMarginHR   = src->zcPred.diagMinMarginHR;
    dst->predRealZcDelayHR = src->zcPred.lastRealZcDelayHR;
    dst->predZcOffsetHR    = src->zcPred.predZcOffsetHR;

    /* Gate readiness fields removed — reclaimed for sector PI */

    /* Step 3: predictive scheduling */
    dst->predCommOwned     = src->zcPred.diagPredCommOwned;
    dst->predictiveMode    = src->zcPred.predictiveMode ? 1 : 0;
    dst->handoffPending    = src->zcPred.handoffPending ? 1 : 0;
    dst->predExitMiss      = src->zcPred.diagPredExitMiss;
    dst->predExitTimeout   = src->zcPred.diagPredExitTimeout;
    dst->predEnter         = src->zcPred.diagPredEnter;
    dst->predEntryLate     = src->zcPred.diagPredEntryLate;
    dst->predVsReactiveDelta = src->zcPred.predVsReactiveDelta;
    dst->deltaOkCount      = src->zcPred.deltaOkCount;
    dst->entryScore        = src->zcPred.entryScore;
    dst->predIsrFired      = src->zcPred.diagPredIsrFired;
    dst->predIsrEntries    = src->zcPred.diagPredIsrEntries;

    /* DPLL state */
#if FEATURE_6STEP_DPLL
    dst->dpllPhaseBiasHR    = src->zcPred.phaseBiasHR;
    dst->dpllErrHR          = src->zcPred.dpllErrHR;
    dst->dmaMeasUsed        = src->zcPred.dmaMeasUsedCount;
    dst->dmaMeasReject      = src->zcPred.dmaMeasRejectCount;
    dst->predCloseAgree     = src->zcPred.predCloseAgreeCount;
    dst->predCloseDisagree  = src->zcPred.predCloseDisagreeCount;
    dst->dpllFallbackReason = src->zcPred.fallbackReason;
#else
    dst->dpllPhaseBiasHR    = 0;
    dst->dpllErrHR          = 0;
    dst->dmaMeasUsed        = 0;
    dst->dmaMeasReject      = 0;
    dst->predCloseAgree     = 0;
    dst->predCloseDisagree  = 0;
    dst->dpllFallbackReason = 0;
#endif

#if FEATURE_IC_DMA_SHADOW
    dst->measSource         = src->dmaShadow.measSource;
#else
    dst->measSource         = 0;
#endif

    /* Sector PI synchronizer telemetry */
#if FEATURE_SECTOR_PI
    dst->syncErrHR          = src->zcSync.syncErrHR;
    dst->syncT_hatHR        = src->zcSync.T_hatHR;
    dst->syncVsReactive     = (int16_t)src->zcSync.capValueHR;  /* DEBUG: repurposed for capValueHR */
    dst->syncMode           = src->zcSync.mode;
    dst->syncGoodStreak     = src->zcSync.goodStreak;
    dst->syncMissStreak     = src->zcSync.missStreak;
    dst->syncClusterCount   = src->zcSync.clusterCount;
    dst->syncAccepts        = src->zcSync.diagSyncAccepts;
    dst->syncMisses         = src->zcSync.diagSyncMisses;
#else
    dst->syncErrHR          = 0;
    dst->syncT_hatHR        = 0;
    dst->syncVsReactive     = 0;
    dst->syncMode           = 0;
    dst->syncGoodStreak     = 0;
    dst->syncMissStreak     = 0;
    dst->syncClusterCount   = 0;
    dst->syncAccepts        = 0;
    dst->syncMisses         = 0;
#endif

    /* IC bounce: SCCP2 captures rejected by the 50% interval gate.
     * Gated on FEATURE_IC_ZC_CAPTURE because the field only exists
     * in that configuration. When FEATURE_IC_DMA_SHADOW takes over
     * CCP2 exclusively, diagIcBounce is replaced by DMA shadow
     * telemetry. */
#if FEATURE_IC_ZC_CAPTURE
    dst->icBounce          = src->icZc.diagIcBounce;
#else
    dst->icBounce          = 0;
#endif

    /* DMA shadow telemetry */
#if FEATURE_IC_DMA_SHADOW
    dst->dmaStepCount          = src->dmaShadow.stepCount;
    dst->dmaMatchCount         = src->dmaShadow.matchCount;
    dst->dmaRingOverflow       = src->dmaShadow.ringOverflowCount;
    /* edges × 16 / stepCount, saturated at uint16 max */
    if (src->dmaShadow.stepCount > 0) {
        uint32_t avgX16 =
            (src->dmaShadow.edgesInWindowSum * 16UL)
            / src->dmaShadow.stepCount;
        dst->dmaEdgesAvgX16 =
            (avgX16 > 0xFFFFUL) ? 0xFFFFU : (uint16_t)avgX16;
    } else {
        dst->dmaEdgesAvgX16 = 0;
    }
    dst->dmaLastEarliestVsPoll = src->dmaShadow.lastEarliestVsPoll;
    dst->dmaLastEarliestVsExp  = src->dmaShadow.lastEarliestVsExpected;
    dst->dmaLastClosestVsExp   = src->dmaShadow.lastClosestVsExpected;
    dst->dmaLastPollVsExp      = src->dmaShadow.lastPollVsExpected;
    dst->dmaLastEdgeCount      = src->dmaShadow.lastEdgeCount;
    dst->dmaLastFound          = src->dmaShadow.lastFound ? 1u : 0u;

    /* DMA-direct substitution telemetry */
    dst->dmaSubCount          = src->dmaShadow.substituteCount;
    dst->dmaSubSkipGated      = src->dmaShadow.substituteSkipGated;
    dst->dmaSubSkipRange      = src->dmaShadow.substituteSkipRange;
    dst->dmaLastCorrectionHR  = src->dmaShadow.lastCorrectionHR;
    dst->dmaMinCorrectionHR   = src->dmaShadow.minCorrectionHR;
    dst->dmaMaxCorrectionHR   = src->dmaShadow.maxCorrectionHR;
#else
    dst->dmaStepCount          = 0;
    dst->dmaMatchCount         = 0;
    dst->dmaRingOverflow       = 0;
    dst->dmaEdgesAvgX16        = 0;
    dst->dmaLastEarliestVsPoll = 0;
    dst->dmaLastEarliestVsExp  = 0;
    dst->dmaLastClosestVsExp   = 0;
    dst->dmaLastPollVsExp      = 0;
    dst->dmaLastEdgeCount      = 0;
    dst->dmaLastFound          = 0;
    dst->dmaSubCount           = 0;
    dst->dmaSubSkipGated       = 0;
    dst->dmaSubSkipRange       = 0;
    dst->dmaLastCorrectionHR   = 0;
    dst->dmaMinCorrectionHR    = 0;
    dst->dmaMaxCorrectionHR    = 0;
#endif
#endif
}

#endif /* FEATURE_GSP */
