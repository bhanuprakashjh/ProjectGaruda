/**
 * @file quality.h
 *
 * @brief ZC quality metrics estimator.
 *
 * Drains the telemetry ring buffer in batches and computes per-window
 * quality metrics: miss rate, false cross rate, timeout rate, ZC jitter
 * (via Welford's online algorithm), and a composite confidence score.
 *
 * Component: LEARN / QUALITY
 */

#ifndef QUALITY_H
#define QUALITY_H

#include <stdint.h>
#include <stdbool.h>
#include "../garuda_types.h"
#include "../garuda_config.h"
#include "ring_buffer.h"

#ifdef __cplusplus
extern "C" {
#endif

#if FEATURE_LEARN_MODULES

/**
 * @brief Initialize quality metrics to zero/defaults.
 * @param metrics  Pointer to quality metrics state
 */
void QUALITY_Init(QUALITY_METRICS_T *metrics);

/**
 * @brief Drain ring buffer and update quality metrics.
 *
 * Call at QUALITY_UPDATE_DIVIDER rate (every 1ms).
 * Auto-resets window after QUALITY_WINDOW_MS.
 *
 * @param metrics  Pointer to quality metrics state
 * @param ring     Pointer to telemetry ring buffer
 * @param now      Current systemTick (1ms)
 */
void QUALITY_Update(QUALITY_METRICS_T *metrics, TELEM_RING_T *ring, uint32_t now);

/**
 * @brief Force-reset the quality window and start fresh.
 * @param metrics  Pointer to quality metrics state
 * @param now      Current systemTick (1ms)
 */
void QUALITY_ResetWindow(QUALITY_METRICS_T *metrics, uint32_t now);

/**
 * @brief Check if quality data is sufficient for adaptation decisions.
 * @param metrics  Pointer to quality metrics state
 * @return true if confidence >= ADAPT_MIN_CONFIDENCE and enough samples
 */
bool QUALITY_IsSufficientForAdaptation(const QUALITY_METRICS_T *metrics);

#endif /* FEATURE_LEARN_MODULES */

#ifdef __cplusplus
}
#endif

#endif /* QUALITY_H */
