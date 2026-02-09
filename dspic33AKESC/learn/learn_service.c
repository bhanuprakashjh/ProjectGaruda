/**
 * @file learn_service.c
 *
 * @brief Top-level dispatcher for learning modules.
 *
 * Runs in the main loop at 1ms resolution. Uses divider counters to
 * dispatch quality (1ms), health (10ms), and adaptation (100ms) at
 * their configured rates.
 *
 * Component: LEARN / SERVICE
 */

#include "learn_service.h"

#if FEATURE_LEARN_MODULES

#include "quality.h"
#include "health.h"
#include "adaptation.h"

/* Global ring buffer instance */
TELEM_RING_T telemRing;

/* Rate divider counters */
static uint32_t lastQualityTick;
static uint32_t lastHealthTick;
static uint32_t lastAdaptTick;

/* Pending adaptation action (evaluated at ADAPT rate, applied at safe boundary) */
static ADAPT_ACTION_T pendingAction;

void LEARN_ServiceInit(volatile GARUDA_DATA_T *pData)
{
    RingBuffer_Init(&telemRing);
    QUALITY_Init(&pData->quality);
    HEALTH_Init(&pData->health, NULL);

#if FEATURE_ADAPTATION
    {
        /* Need a non-volatile copy to pass to ADAPT_Init */
        GARUDA_CONFIG_T cfg;
        cfg.alignDutyPercent = ALIGN_DUTY_PERCENT;
        cfg.rampAccelErpmPerS = RAMP_ACCEL_ERPM_PER_S;
        cfg.alignTimeMs = ALIGN_TIME_MS;
        ADAPT_Init(&pData->adapt, &cfg);
    }
#endif

    lastQualityTick = 0;
    lastHealthTick = 0;
    lastAdaptTick = 0;
    pendingAction = ADAPT_NO_CHANGE;
}

void LEARN_Service(volatile GARUDA_DATA_T *pData, uint32_t now)
{
    /* Quality update — every QUALITY_UPDATE_DIVIDER ms */
    if ((now - lastQualityTick) >= QUALITY_UPDATE_DIVIDER)
    {
        lastQualityTick = now;
        QUALITY_Update(&pData->quality, &telemRing, now);
    }

    /* Health update — every HEALTH_UPDATE_DIVIDER ms */
    if ((now - lastHealthTick) >= HEALTH_UPDATE_DIVIDER)
    {
        lastHealthTick = now;
        HEALTH_Update(&pData->health, &pData->quality, pData);
    }

#if FEATURE_ADAPTATION
    /* Adaptation evaluation — every ADAPT_EVAL_DIVIDER ms */
    if ((now - lastAdaptTick) >= ADAPT_EVAL_DIVIDER)
    {
        lastAdaptTick = now;
        pendingAction = ADAPT_Evaluate(&pData->adapt, &pData->quality);
    }

    /* Apply pending action only at safe boundary */
    if (pendingAction != ADAPT_NO_CHANGE)
    {
        if (ADAPT_IsSafeBoundary(pData->state, pData->throttle))
        {
            ADAPT_Apply(&pData->adapt, pendingAction);
            pendingAction = ADAPT_NO_CHANGE;
        }
    }
#endif
}

#endif /* FEATURE_LEARN_MODULES */
