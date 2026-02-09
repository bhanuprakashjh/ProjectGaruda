/**
 * @file adaptation.h
 *
 * @brief Bounded adaptation policy.
 *
 * All parameter changes are clamped to a compile-time safety envelope.
 * Only one parameter changed per evaluation cycle. Auto-rollback if
 * confidence drops rapidly after a change. Lock after consecutive
 * failures.
 *
 * Component: LEARN / ADAPTATION
 */

#ifndef ADAPTATION_H
#define ADAPTATION_H

#include <stdint.h>
#include <stdbool.h>
#include "../garuda_types.h"
#include "../garuda_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#if FEATURE_LEARN_MODULES

/**
 * @brief Initialize adaptation parameters from config defaults.
 * @param adapt   Pointer to adaptation params
 * @param config  Pointer to user config (for initial values)
 */
void ADAPT_Init(ADAPT_PARAMS_T *adapt, const GARUDA_CONFIG_T *config);

/**
 * @brief Evaluate quality metrics and decide what to change.
 *
 * Call at ADAPT_EVAL_DIVIDER rate (every 100ms).
 * Returns an action but does NOT apply it.
 *
 * @param adapt    Pointer to adaptation params
 * @param quality  Pointer to current quality metrics
 * @return Action to take (or ADAPT_NO_CHANGE)
 */
ADAPT_ACTION_T ADAPT_Evaluate(ADAPT_PARAMS_T *adapt,
                              const QUALITY_METRICS_T *quality);

/**
 * @brief Apply a previously evaluated action.
 *
 * Should only be called at a safe boundary (idle, arming, low throttle).
 * Clamps all values to [min, max] envelope.
 *
 * @param adapt   Pointer to adaptation params
 * @param action  Action from ADAPT_Evaluate()
 * @return true if param was changed, false if locked/no-op
 */
bool ADAPT_Apply(ADAPT_PARAMS_T *adapt, ADAPT_ACTION_T action);

/**
 * @brief Snapshot current params as last-known-good.
 * @param adapt  Pointer to adaptation params
 */
void ADAPT_SnapshotLKG(ADAPT_PARAMS_T *adapt);

/**
 * @brief Rollback to last-known-good params.
 * @param adapt  Pointer to adaptation params
 * @return true if rollback applied, false if max rollbacks exceeded
 */
bool ADAPT_Rollback(ADAPT_PARAMS_T *adapt);

/**
 * @brief Check if current ESC state is a safe boundary for applying changes.
 * @param state     Current ESC state
 * @param throttle  Current throttle value (0-2000)
 * @return true if safe to apply parameter changes
 */
bool ADAPT_IsSafeBoundary(ESC_STATE_T state, uint16_t throttle);

#endif /* FEATURE_LEARN_MODULES */

#ifdef __cplusplus
}
#endif

#endif /* ADAPTATION_H */
