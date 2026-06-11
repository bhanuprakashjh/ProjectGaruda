/**
 * @file quality.c
 *
 * @brief ZC quality metrics estimator implementation.
 *
 * Uses Welford's online algorithm for incremental mean + variance
 * computation (no sample storage needed). Window auto-resets after
 * QUALITY_WINDOW_MS milliseconds.
 *
 * Component: LEARN / QUALITY
 */

#include "quality.h"

#if FEATURE_LEARN_MODULES

#include <math.h>

/* Welford's online algorithm for incremental mean + variance */
static uint32_t welfordCount;
static float    welfordMean;
static float    welfordM2;

static void Welford_Reset(void)
{
    welfordCount = 0;
    welfordMean = 0.0f;
    welfordM2 = 0.0f;
}

static void Welford_AddSample(uint16_t latency)
{
    float val = (float)latency;
    welfordCount++;
    float delta = val - welfordMean;
    welfordMean += delta / (float)welfordCount;
    float delta2 = val - welfordMean;
    welfordM2 += delta * delta2;
}

static float Welford_GetStdDev(void)
{
    if (welfordCount < 2)
        return 0.0f;
    float variance = welfordM2 / (float)(welfordCount - 1);
    return sqrtf(variance);
}

void QUALITY_Init(QUALITY_METRICS_T *metrics)
{
    metrics->zcMissCount = 0;
    metrics->zcTotalCount = 0;
    metrics->zcTimeoutCount = 0;
    metrics->falseCrossCount = 0;
    metrics->desyncRecoveries = 0;
    metrics->zcMissRate = 0.0f;
    metrics->falseRate = 0.0f;
    metrics->timeoutRate = 0.0f;
    metrics->zcJitter = 0.0f;
    metrics->confidenceScore = 0;
    metrics->lastWindowTotal = 0;
    metrics->windowStartTick = 0;
    metrics->windowSizeTicks = QUALITY_WINDOW_MS;
    Welford_Reset();
}

void QUALITY_ResetWindow(QUALITY_METRICS_T *metrics, uint32_t now)
{
    metrics->zcMissCount = 0;
    metrics->zcTotalCount = 0;
    metrics->zcTimeoutCount = 0;
    metrics->falseCrossCount = 0;
    metrics->desyncRecoveries = 0;
    metrics->windowStartTick = now;
    Welford_Reset();
}

static void ComputeRatesAndConfidence(QUALITY_METRICS_T *metrics)
{
    float total = (float)metrics->zcTotalCount;
    if (total < 1.0f)
    {
        metrics->zcMissRate = 0.0f;
        metrics->falseRate = 0.0f;
        metrics->timeoutRate = 0.0f;
        metrics->confidenceScore = 0;
        return;
    }

    metrics->zcMissRate = (float)metrics->zcMissCount / total;
    metrics->falseRate = (float)metrics->falseCrossCount / total;
    metrics->timeoutRate = (float)metrics->zcTimeoutCount / total;
    metrics->zcJitter = Welford_GetStdDev();

    /* Confidence = 255 - penalties, clamped [0,255] */
    float conf = 255.0f;
    conf -= metrics->zcMissRate * 127.5f;               /* missRate / 2 scaled to 255 */
    conf -= metrics->falseRate * 127.5f;                 /* falseRate / 2 scaled to 255 */
    conf -= metrics->timeoutRate * 255.0f;               /* timeoutRate scaled to 255 */
    conf -= (float)metrics->desyncRecoveries * 50.0f;    /* heavy penalty */

    if (conf < 0.0f) conf = 0.0f;
    if (conf > 255.0f) conf = 255.0f;
    metrics->confidenceScore = (uint8_t)conf;
}

void QUALITY_Update(QUALITY_METRICS_T *metrics, TELEM_RING_T *ring, uint32_t now)
{
    /* Drain all available samples from ring buffer */
    TELEM_SAMPLE_T sample;
    while (RingBuffer_Read(ring, &sample))
    {
        metrics->zcTotalCount++;

        if (sample.faultFlags & TELEM_FLAG_ZC_MISSED)
            metrics->zcMissCount++;
        if (sample.faultFlags & TELEM_FLAG_ZC_TIMEOUT)
            metrics->zcTimeoutCount++;
        if (sample.faultFlags & TELEM_FLAG_DESYNC_RECOVERY)
            metrics->desyncRecoveries++;
        if (sample.faultFlags & TELEM_FLAG_FALSE_CROSS)
            metrics->falseCrossCount++;

        /* Add valid ZC latency to jitter estimator */
        if (sample.zcPolarity != 0 && !(sample.faultFlags & TELEM_FLAG_ZC_MISSED))
        {
            Welford_AddSample(sample.zcLatencyTicks);
        }
    }

    /* Auto-reset window: compute final rates, snapshot total, then reset */
    if ((now - metrics->windowStartTick) >= metrics->windowSizeTicks)
    {
        ComputeRatesAndConfidence(metrics);
        metrics->lastWindowTotal = metrics->zcTotalCount;
        QUALITY_ResetWindow(metrics, now);
    }
}

bool QUALITY_IsSufficientForAdaptation(const QUALITY_METRICS_T *metrics)
{
    /* confidenceScore and lastWindowTotal persist across window resets,
     * so this check works even immediately after a reset. */
    return (metrics->confidenceScore >= ADAPT_MIN_CONFIDENCE) &&
           (metrics->lastWindowTotal >= 100);
}

#endif /* FEATURE_LEARN_MODULES */
