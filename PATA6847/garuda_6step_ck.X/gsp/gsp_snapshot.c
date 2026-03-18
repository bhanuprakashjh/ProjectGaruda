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

    /* Compute eRPM (cap at 65535 for uint16_t) */
    {
        uint32_t erpm = 0;
#if FEATURE_IC_ZC
        if (src->timing.stepPeriodHR > 0 && src->timing.hasPrevZcHR)
            erpm = 15625000UL / src->timing.stepPeriodHR;
        else
#endif
        if (src->timing.stepPeriod > 0)
            erpm = (uint32_t)TIMER1_FREQ_HZ * 10UL / src->timing.stepPeriod;
        dst->eRpm = (erpm > 65535UL) ? 65535U : (uint16_t)erpm;
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
}

#endif /* FEATURE_GSP */
