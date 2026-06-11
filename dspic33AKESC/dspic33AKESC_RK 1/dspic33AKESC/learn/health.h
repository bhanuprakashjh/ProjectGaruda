/**
 * @file health.h
 *
 * @brief Explainable health scoring with 5 sub-scores.
 *
 * Five sub-scores (0-255 each), weighted composite (weights sum to 256
 * for shift-divide):
 *   bearing    (25%=64):  ZC jitter vs baseline (vibration indicator)
 *   balance    (25%=64):  step-to-step timing asymmetry
 *   connection (20%=51):  resistance drift from commissioned baseline
 *   thermal    (20%=51):  temperature trend / duty-to-speed ratio
 *   electrical (10%=26):  Vbus ripple / current anomalies
 *
 * Component: LEARN / HEALTH
 */

#ifndef HEALTH_H
#define HEALTH_H

#include <stdint.h>
#include <stdbool.h>
#include "../garuda_types.h"
#include "../garuda_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#if FEATURE_LEARN_MODULES

/* Health score weight constants (sum = 256 for >> 8 divide) */
#define HEALTH_WEIGHT_BEARING       64
#define HEALTH_WEIGHT_BALANCE       64
#define HEALTH_WEIGHT_CONNECTION    51
#define HEALTH_WEIGHT_THERMAL       51
#define HEALTH_WEIGHT_ELECTRICAL    26

/* Degradation thresholds */
#define HEALTH_DEGRADED_THRESHOLD   128
#define HEALTH_CRITICAL_THRESHOLD   64

/**
 * @brief Initialize health state from learned baselines.
 * @param health   Pointer to health state
 * @param learned  Pointer to learned motor params (for baselines), or NULL
 */
void HEALTH_Init(HEALTH_STATE_T *health, const LEARNED_PARAMS_T *learned);

/**
 * @brief Update health scores from quality metrics and ESC data.
 *
 * Call at HEALTH_UPDATE_DIVIDER rate (every 10ms).
 *
 * @param health   Pointer to health state
 * @param quality  Pointer to current quality metrics
 * @param pData    Pointer to ESC runtime data
 */
void HEALTH_Update(HEALTH_STATE_T *health, const QUALITY_METRICS_T *quality,
                   const volatile GARUDA_DATA_T *pData);

/**
 * @brief Set baselines from current quality data (after commissioning).
 * @param health   Pointer to health state
 * @param quality  Pointer to current quality metrics
 */
void HEALTH_SetBaseline(HEALTH_STATE_T *health, const QUALITY_METRICS_T *quality);

/**
 * @brief Check if any sub-score is below degraded threshold.
 * @param health  Pointer to health state
 * @return true if any sub-score < 128
 */
bool HEALTH_IsDegraded(const HEALTH_STATE_T *health);

/**
 * @brief Check if any sub-score is below critical threshold.
 * @param health  Pointer to health state
 * @return true if any sub-score < 64
 */
bool HEALTH_IsCritical(const HEALTH_STATE_T *health);

/**
 * @brief Get the worst (lowest) sub-score and its category name.
 * @param health    Pointer to health state
 * @param category  Output: pointer to category name string
 * @return The lowest sub-score value (0-255)
 */
uint8_t HEALTH_GetWorstScore(const HEALTH_STATE_T *health, const char **category);

#endif /* FEATURE_LEARN_MODULES */

#ifdef __cplusplus
}
#endif

#endif /* HEALTH_H */
