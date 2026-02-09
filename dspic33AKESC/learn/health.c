/**
 * @file health.c
 *
 * @brief Explainable health scoring implementation.
 *
 * Each sub-score starts at 255 (perfect) and degrades proportionally
 * to how far the measured metric has drifted from its baseline.
 * The composite score is a weighted average using shift-divide (>>8).
 *
 * Component: LEARN / HEALTH
 */

#include "health.h"

#if FEATURE_LEARN_MODULES

/* Scale a metric ratio to a 0-255 score.
 * score = 255 - clamp(255 * (current - baseline) / baseline, 0, 255)
 * When current <= baseline, score = 255 (perfect).
 * When current >= 2*baseline, score = 0 (worst). */
static uint8_t ScoreFromDrift(uint16_t current, uint16_t baseline)
{
    if (baseline == 0 || current <= baseline)
        return 255;

    uint32_t drift = (uint32_t)(current - baseline) * 255 / baseline;
    if (drift >= 255)
        return 0;
    return (uint8_t)(255 - drift);
}

void HEALTH_Init(HEALTH_STATE_T *health, const LEARNED_PARAMS_T *learned)
{
    health->bearingScore = 255;
    health->balanceScore = 255;
    health->connectionScore = 255;
    health->thermalScore = 255;
    health->electricalScore = 255;
    health->compositeHealth = 255;
    health->trend = 0;
    health->operatingHours = 0;
    health->baselineJitterQ8 = 0;
    health->baselineAsymmetryQ8 = 0;
    health->baselineResistanceMilliOhm = 0;

    if (learned != NULL)
    {
        health->baselineResistanceMilliOhm = learned->phaseResistanceMilliOhm;
    }
}

void HEALTH_SetBaseline(HEALTH_STATE_T *health, const QUALITY_METRICS_T *quality)
{
    health->baselineJitterQ8 = quality->zcJitterQ8;
    /* Asymmetry baseline: placeholder — Phase 2 will provide step timing */
    health->baselineAsymmetryQ8 = 0;
}

void HEALTH_Update(HEALTH_STATE_T *health, const QUALITY_METRICS_T *quality,
                   const volatile GARUDA_DATA_T *pData)
{
    uint8_t prevComposite = health->compositeHealth;
    (void)pData;    /* used in Phase 2+ for thermal/electrical sensing */

    /* Bearing: ZC jitter vs baseline */
    if (health->baselineJitterQ8 > 0)
        health->bearingScore = ScoreFromDrift(quality->zcJitterQ8,
                                              health->baselineJitterQ8);

    /* Balance: step timing asymmetry vs baseline */
    /* Phase 2 will populate asymmetry data; for now hold at 255 */

    /* Connection: resistance drift from commissioned baseline */
    /* Phase 2 will provide live resistance measurement; for now hold at 255 */

    /* Thermal: duty-to-speed ratio anomaly */
    /* Phase 2 will provide ERPM for this calculation; for now hold at 255 */

    /* Electrical: Vbus ripple / current anomalies */
    /* Phase 2 will provide current sensing; for now hold at 255 */

    /* Compute weighted composite (weights sum to 256 for >>8 divide) */
    uint16_t composite =
        (uint16_t)health->bearingScore    * HEALTH_WEIGHT_BEARING +
        (uint16_t)health->balanceScore    * HEALTH_WEIGHT_BALANCE +
        (uint16_t)health->connectionScore * HEALTH_WEIGHT_CONNECTION +
        (uint16_t)health->thermalScore    * HEALTH_WEIGHT_THERMAL +
        (uint16_t)health->electricalScore * HEALTH_WEIGHT_ELECTRICAL;
    health->compositeHealth = (uint8_t)(composite >> 8);

    /* Trend detection */
    if (health->compositeHealth > prevComposite + 2)
        health->trend = +1;
    else if (health->compositeHealth + 2 < prevComposite)
        health->trend = -1;
    else
        health->trend = 0;
}

bool HEALTH_IsDegraded(const HEALTH_STATE_T *health)
{
    return (health->bearingScore    < HEALTH_DEGRADED_THRESHOLD) ||
           (health->balanceScore    < HEALTH_DEGRADED_THRESHOLD) ||
           (health->connectionScore < HEALTH_DEGRADED_THRESHOLD) ||
           (health->thermalScore    < HEALTH_DEGRADED_THRESHOLD) ||
           (health->electricalScore < HEALTH_DEGRADED_THRESHOLD);
}

bool HEALTH_IsCritical(const HEALTH_STATE_T *health)
{
    return (health->bearingScore    < HEALTH_CRITICAL_THRESHOLD) ||
           (health->balanceScore    < HEALTH_CRITICAL_THRESHOLD) ||
           (health->connectionScore < HEALTH_CRITICAL_THRESHOLD) ||
           (health->thermalScore    < HEALTH_CRITICAL_THRESHOLD) ||
           (health->electricalScore < HEALTH_CRITICAL_THRESHOLD);
}

uint8_t HEALTH_GetWorstScore(const HEALTH_STATE_T *health, const char **category)
{
    uint8_t worst = health->bearingScore;
    const char *cat = "bearing";

    if (health->balanceScore < worst)
    {
        worst = health->balanceScore;
        cat = "balance";
    }
    if (health->connectionScore < worst)
    {
        worst = health->connectionScore;
        cat = "connection";
    }
    if (health->thermalScore < worst)
    {
        worst = health->thermalScore;
        cat = "thermal";
    }
    if (health->electricalScore < worst)
    {
        worst = health->electricalScore;
        cat = "electrical";
    }

    if (category != NULL)
        *category = cat;
    return worst;
}

#endif /* FEATURE_LEARN_MODULES */
