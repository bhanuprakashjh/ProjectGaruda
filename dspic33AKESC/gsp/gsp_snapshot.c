/**
 * @file gsp_snapshot.c
 *
 * @brief Tear-safe snapshot capture from garudaData.
 *
 * Strategy:
 * - Most fields are 8-bit or 16-bit (atomic on dsPIC33AK)
 * - 32-bit fields (duty, systemTick, trip counters) may tear but
 *   this is acceptable for telemetry — worst case is off by 1
 * - Prio-7 ISR fields (HWZC counters, stepPeriodHR) use:
 *   - Seqlock for stepPeriodHR (existing hwzc.writeSeq)
 *   - Double-read with bounded retry for 32-bit counters
 * - Feature-gated fields are zeroed when feature is disabled
 *
 * Component: GSP
 */

#include "garuda_config.h"

#if FEATURE_GSP

#include <xc.h>
#include <string.h>
#include "gsp_snapshot.h"
#include "garuda_calc_params.h"
#include "garuda_types.h"
#include "garuda_service.h"

#define SNAPSHOT_RETRY_MAX  4

/* Read a volatile uint32_t with double-read consistency check.
 * Returns v1 if all retries torn (acceptable for telemetry). */
static uint32_t ReadU32Consistent(volatile uint32_t *src)
{
    uint32_t v1, v2;
    uint8_t retry = SNAPSHOT_RETRY_MAX;
    do {
        v1 = *src;
        v2 = *src;
        if (v1 == v2)
            return v1;
    } while (--retry);
    return v1;
}

void GSP_CaptureSnapshot(GSP_SNAPSHOT_T *dst)
{
    volatile GARUDA_DATA_T *src = &garudaData;

    /* Zero the entire struct first (handles disabled-feature fields) */
    memset(dst, 0, sizeof(GSP_SNAPSHOT_T));

    /* Core state (all 8/16-bit: atomic on dsPIC33AK) */
    dst->state       = (uint8_t)src->state;
    dst->faultCode   = (uint8_t)src->faultCode;
    dst->currentStep = src->currentStep;
    dst->direction   = src->direction;
    dst->throttle    = src->throttle;
    dst->dutyPct     = (uint8_t)((uint64_t)src->duty * 100 / LOOPTIME_TCY);

    /* Bus */
    dst->vbusRaw     = src->vbusRaw;
#if FEATURE_HW_OVERCURRENT
    dst->ibusRaw     = src->ibusRaw;
    dst->ibusMax     = src->ibusMax;
#endif

    /* BEMF/ZC */
    dst->bemfRaw     = src->bemf.bemfRaw;
    dst->zcThreshold = src->bemf.zcThreshold;
    dst->stepPeriod  = src->timing.stepPeriod;
    dst->goodZcCount = src->timing.goodZcCount;

    /* ZC flags */
    dst->risingZcWorks  = src->timing.risingZcWorks  ? 1 : 0;
    dst->fallingZcWorks = src->timing.fallingZcWorks ? 1 : 0;
    dst->zcSynced       = src->timing.zcSynced       ? 1 : 0;

    /* ZC diagnostics */
#if FEATURE_BEMF_CLOSED_LOOP
    dst->zcConfirmedCount    = src->zcDiag.zcConfirmedCount;
    dst->zcTimeoutForceCount = src->zcDiag.zcTimeoutForceCount;
#endif

    /* HWZC — non-prio-7 fields (8/16-bit: atomic) */
#if FEATURE_ADC_CMP_ZC
    dst->hwzcEnabled         = src->hwzc.enabled ? 1 : 0;
    dst->hwzcPhase           = (uint8_t)src->hwzc.phase;
    dst->hwzcDbgLatchDisable = src->hwzc.dbgLatchDisable ? 1 : 0;
#endif

    /* Morph */
#if FEATURE_SINE_STARTUP
    dst->morphSubPhase = (uint8_t)src->morph.subPhase;
    dst->morphStep     = src->morph.morphStep;
    dst->morphZcCount  = src->morph.morphZcCount;
    dst->morphAlpha    = src->morph.alpha;
#endif

    /* Overcurrent (32-bit counters: may tear, acceptable for telemetry) */
#if FEATURE_HW_OVERCURRENT
    dst->clpciTripCount = src->clpciTripCount;
    dst->fpciTripCount  = src->fpciTripCount;
#endif

    /* System (32-bit: may tear by 1ms, acceptable) */
    dst->systemTick = src->systemTick;
    dst->uptimeSec  = src->systemTick / 1000;

    /* Prio-7 fields: use consistency techniques */
#if FEATURE_ADC_CMP_ZC
    /* stepPeriodHR: seqlock */
    {
        uint16_t s1, s2;
        uint32_t val;
        uint8_t retry = SNAPSHOT_RETRY_MAX;
        do {
            s1  = src->hwzc.writeSeq;
            val = src->hwzc.stepPeriodHR;
            s2  = src->hwzc.writeSeq;
        } while ((s1 != s2 || (s1 & 1)) && --retry);
        dst->hwzcStepPeriodHR = val;
    }

    /* 32-bit counters: double-read */
    dst->hwzcTotalZcCount   = ReadU32Consistent(&src->hwzc.totalZcCount);
    dst->hwzcTotalMissCount = ReadU32Consistent(&src->hwzc.totalMissCount);
#endif
}

#endif /* FEATURE_GSP */
