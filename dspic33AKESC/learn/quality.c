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

/* Welford's state for ZC latency jitter (internal) */
static uint32_t welfordCount;
static int32_t  welfordMeanQ8;      /* Q8.8 fixed-point */
static int32_t  welfordM2Q16;       /* Q16.16 for variance accumulator */

static void Welford_Reset(void)
{
    welfordCount = 0;
    welfordMeanQ8 = 0;
    welfordM2Q16 = 0;
}

static void Welford_AddSample(uint16_t latency)
{
    int32_t valQ8 = (int32_t)latency << 8;
    welfordCount++;
    int32_t delta = valQ8 - welfordMeanQ8;
    welfordMeanQ8 += delta / (int32_t)welfordCount;
    int32_t delta2 = valQ8 - welfordMeanQ8;
    welfordM2Q16 += (delta >> 4) * (delta2 >> 4);  /* scale to avoid overflow */
}

static uint16_t Welford_GetStdDevQ8(void)
{
    if (welfordCount < 2)
        return 0;
    /* Variance = M2 / (count-1), but M2 is scaled by 2^-8 from the shift */
    uint32_t varianceScaled = (uint32_t)(welfordM2Q16 / (int32_t)(welfordCount - 1));
    /* Integer square root approximation for std dev */
    uint32_t x = varianceScaled;
    uint32_t result = 0;
    uint32_t bit = 1UL << 30;
    while (bit > x)
        bit >>= 2;
    while (bit != 0)
    {
        if (x >= result + bit)
        {
            x -= result + bit;
            result = (result >> 1) + bit;
        }
        else
        {
            result >>= 1;
        }
        bit >>= 2;
    }
    return (uint16_t)result;
}

void QUALITY_Init(QUALITY_METRICS_T *metrics)
{
    metrics->zcMissCount = 0;
    metrics->zcTotalCount = 0;
    metrics->zcTimeoutCount = 0;
    metrics->falseCrossCount = 0;
    metrics->desyncRecoveries = 0;
    metrics->zcMissRateQ8 = 0;
    metrics->falseRateQ8 = 0;
    metrics->timeoutRateQ8 = 0;
    metrics->zcJitterQ8 = 0;
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
    uint16_t total = metrics->zcTotalCount;
    if (total == 0)
    {
        metrics->zcMissRateQ8 = 0;
        metrics->falseRateQ8 = 0;
        metrics->timeoutRateQ8 = 0;
        metrics->confidenceScore = 0;
        return;
    }

    /* Q8.8 rates: (count * 256) / total */
    metrics->zcMissRateQ8 = (uint16_t)(((uint32_t)metrics->zcMissCount << 8) / total);
    metrics->falseRateQ8 = (uint16_t)(((uint32_t)metrics->falseCrossCount << 8) / total);
    metrics->timeoutRateQ8 = (uint16_t)(((uint32_t)metrics->zcTimeoutCount << 8) / total);
    metrics->zcJitterQ8 = Welford_GetStdDevQ8();

    /* Confidence = 255 - penalties, clamped [0,255] */
    int16_t conf = 255;
    conf -= (int16_t)(metrics->zcMissRateQ8 >> 1);     /* missRate / 2 */
    conf -= (int16_t)(metrics->falseRateQ8 >> 1);       /* falseRate / 2 */
    conf -= (int16_t)metrics->timeoutRateQ8;             /* timeoutRate */
    conf -= (int16_t)metrics->desyncRecoveries * 50;     /* heavy penalty */

    if (conf < 0) conf = 0;
    if (conf > 255) conf = 255;
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
