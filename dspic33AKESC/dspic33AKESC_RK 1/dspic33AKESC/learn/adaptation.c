/**
 * @file adaptation.c
 *
 * @brief Bounded adaptation policy implementation.
 *
 * Safety rules:
 * - All params clamped to [min, max] envelope
 * - Only one param changed per evaluation cycle
 * - Auto-rollback if confidence drops below ADAPT_MIN_CONFIDENCE/2
 *   within 5 seconds of a change
 * - Lock after ADAPT_MAX_CONSECUTIVE_FAIL failures
 *
 * Component: LEARN / ADAPTATION
 */

#include "adaptation.h"

#if FEATURE_LEARN_MODULES

/* Timing advance step size (degrees per adjustment) */
#define TIMING_STEP_DEG     1

/* Startup duty step size (percent per adjustment) */
#define STARTUP_DUTY_STEP   0.5f

/* Ramp accel step size (eRPM/s per adjustment) */
#define RAMP_ACCEL_STEP     500

/* Low-throttle threshold for safe boundary */
#define LOW_THROTTLE_THRESHOLD  200

void ADAPT_Init(ADAPT_PARAMS_T *adapt, const GARUDA_CONFIG_T *config)
{
    adapt->timingAdvanceDeg = 0;
    adapt->startupDutyPercent = (float)config->alignDutyPercent;
    adapt->rampAccelErpmPerS = config->rampAccelErpmPerS;
    adapt->alignTimeMs = config->alignTimeMs;

#ifdef TIMING_ADVANCE_MIN_DEG
    adapt->timingAdvanceMin = TIMING_ADVANCE_MIN_DEG;
    adapt->timingAdvanceMax = TIMING_ADVANCE_MAX_DEG;
#else
    adapt->timingAdvanceMin = 0;
    adapt->timingAdvanceMax = 15;
#endif
    adapt->startupDutyMin = 2.0f;
    adapt->startupDutyMax = 15.0f;

    adapt->lkgTimingAdvanceDeg = adapt->timingAdvanceDeg;
    adapt->lkgStartupDutyPercent = adapt->startupDutyPercent;
    adapt->lkgRampAccelErpmPerS = adapt->rampAccelErpmPerS;
    adapt->lkgAlignTimeMs = adapt->alignTimeMs;

    adapt->rollbackCount = 0;
    adapt->consecutiveFailures = 0;
    adapt->adaptationLocked = false;
}

ADAPT_ACTION_T ADAPT_Evaluate(ADAPT_PARAMS_T *adapt,
                              const QUALITY_METRICS_T *quality)
{
    if (adapt->adaptationLocked)
        return ADAPT_NO_CHANGE;

    /* Auto-rollback trigger: confidence crashed */
    if (quality->confidenceScore < (ADAPT_MIN_CONFIDENCE / 2))
    {
        adapt->consecutiveFailures++;
        if (adapt->consecutiveFailures >= ADAPT_MAX_CONSECUTIVE_FAIL)
        {
            adapt->adaptationLocked = true;
        }
        return ADAPT_ROLLBACK;
    }

    /* Need sufficient data to make decisions */
    if (quality->confidenceScore < ADAPT_MIN_CONFIDENCE)
        return ADAPT_NO_CHANGE;

    /* Reset failure counter on good confidence */
    adapt->consecutiveFailures = 0;

    /* Priority 1: Timing advance optimization based on ZC miss rate */
    if (quality->zcMissRate > 0.25f)
    {
        if (adapt->timingAdvanceDeg > adapt->timingAdvanceMin)
            return ADAPT_TIMING_DECREASE;
    }
    else if (quality->zcMissRate < 0.05f &&
             quality->confidenceScore > 220)
    {
        if (adapt->timingAdvanceDeg < adapt->timingAdvanceMax)
            return ADAPT_TIMING_INCREASE;
    }

    /* Priority 2: Startup duty adjustment based on desync recoveries */
    if (quality->desyncRecoveries > 2)
    {
        return ADAPT_STARTUP_DUTY_UP;
    }

    return ADAPT_NO_CHANGE;
}

static uint8_t ClampU8(int16_t val, uint8_t min, uint8_t max)
{
    if (val < (int16_t)min) return min;
    if (val > (int16_t)max) return max;
    return (uint8_t)val;
}

static uint16_t ClampU16(int32_t val, uint16_t min, uint16_t max)
{
    if (val < (int32_t)min) return min;
    if (val > (int32_t)max) return max;
    return (uint16_t)val;
}

static float ClampF(float val, float min, float max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

bool ADAPT_Apply(ADAPT_PARAMS_T *adapt, ADAPT_ACTION_T action)
{
    if (adapt->adaptationLocked || action == ADAPT_NO_CHANGE)
        return false;

    if (action == ADAPT_ROLLBACK)
        return ADAPT_Rollback(adapt);

    /* Snapshot before changing */
    ADAPT_SnapshotLKG(adapt);

    switch (action)
    {
        case ADAPT_TIMING_INCREASE:
            adapt->timingAdvanceDeg = ClampU8(
                (int16_t)adapt->timingAdvanceDeg + TIMING_STEP_DEG,
                adapt->timingAdvanceMin, adapt->timingAdvanceMax);
            break;

        case ADAPT_TIMING_DECREASE:
            adapt->timingAdvanceDeg = ClampU8(
                (int16_t)adapt->timingAdvanceDeg - TIMING_STEP_DEG,
                adapt->timingAdvanceMin, adapt->timingAdvanceMax);
            break;

        case ADAPT_STARTUP_DUTY_UP:
            adapt->startupDutyPercent = ClampF(
                adapt->startupDutyPercent + STARTUP_DUTY_STEP,
                adapt->startupDutyMin, adapt->startupDutyMax);
            break;

        case ADAPT_STARTUP_DUTY_DOWN:
            adapt->startupDutyPercent = ClampF(
                adapt->startupDutyPercent - STARTUP_DUTY_STEP,
                adapt->startupDutyMin, adapt->startupDutyMax);
            break;

        case ADAPT_RAMP_ACCEL_UP:
            adapt->rampAccelErpmPerS = ClampU16(
                (int32_t)adapt->rampAccelErpmPerS + RAMP_ACCEL_STEP,
                1000, 20000);
            break;

        case ADAPT_RAMP_ACCEL_DOWN:
            adapt->rampAccelErpmPerS = ClampU16(
                (int32_t)adapt->rampAccelErpmPerS - RAMP_ACCEL_STEP,
                1000, 20000);
            break;

        default:
            return false;
    }

    return true;
}

void ADAPT_SnapshotLKG(ADAPT_PARAMS_T *adapt)
{
    adapt->lkgTimingAdvanceDeg = adapt->timingAdvanceDeg;
    adapt->lkgStartupDutyPercent = adapt->startupDutyPercent;
    adapt->lkgRampAccelErpmPerS = adapt->rampAccelErpmPerS;
    adapt->lkgAlignTimeMs = adapt->alignTimeMs;
}

bool ADAPT_Rollback(ADAPT_PARAMS_T *adapt)
{
    if (adapt->rollbackCount >= ADAPT_MAX_ROLLBACK_TOTAL)
    {
        adapt->adaptationLocked = true;
        return false;
    }

    adapt->timingAdvanceDeg = adapt->lkgTimingAdvanceDeg;
    adapt->startupDutyPercent = adapt->lkgStartupDutyPercent;
    adapt->rampAccelErpmPerS = adapt->lkgRampAccelErpmPerS;
    adapt->alignTimeMs = adapt->lkgAlignTimeMs;
    adapt->rollbackCount++;
    return true;
}

bool ADAPT_IsSafeBoundary(ESC_STATE_T state, uint16_t throttle)
{
    if (state == ESC_IDLE || state == ESC_ARMED)
        return true;
    if (state == ESC_CLOSED_LOOP && throttle < LOW_THROTTLE_THRESHOLD)
        return true;
    return false;
}

#endif /* FEATURE_LEARN_MODULES */
