/**
 * @file gsp_snapshot.c
 * @brief CK board snapshot capture — 48 bytes of 6-step telemetry.
 *
 * Most fields are 8/16-bit (atomic on dsPIC33CK). 32-bit fields
 * may tear but this is acceptable for telemetry display.
 */

#include "../garuda_config.h"

#if FEATURE_GSP

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

    /* Step 2: gate readiness */
    dst->winCandInGated    = src->zcPred.diagWinCandInGated;
    dst->winCandOutGated   = src->zcPred.diagWinCandOutGated;
    dst->winOutEarly       = src->zcPred.diagWinOutEarly;
    dst->winOutLate        = src->zcPred.diagWinOutLate;
    dst->gateActive        = src->zcPred.gateActive ? 1 : 0;
    dst->_pad1             = 0;
    dst->windowReject      = src->zcPred.diagWindowReject;
    dst->windowRecovered   = src->zcPred.diagWindowRecovered;

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
#endif
}

#endif /* FEATURE_GSP */
