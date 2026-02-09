/**
 * @file learn_service.h
 *
 * @brief Top-level dispatcher for learning modules.
 *
 * Dispatches at configured rates:
 *   QUALITY_Update()           every QUALITY_UPDATE_DIVIDER ms
 *   HEALTH_Update()            every HEALTH_UPDATE_DIVIDER ms
 *   ADAPT_Evaluate()+Apply()   every ADAPT_EVAL_DIVIDER ms
 *
 * Component: LEARN / SERVICE
 */

#ifndef LEARN_SERVICE_H
#define LEARN_SERVICE_H

#include <stdint.h>
#include "../garuda_types.h"
#include "../garuda_config.h"
#include "ring_buffer.h"

#ifdef __cplusplus
extern "C" {
#endif

#if FEATURE_LEARN_MODULES

/* Global ring buffer instance (written by ISR, read by learn service) */
extern TELEM_RING_T telemRing;

/**
 * @brief Initialize learning subsystem.
 * @param pData  Pointer to ESC runtime data
 */
void LEARN_ServiceInit(volatile GARUDA_DATA_T *pData);

/**
 * @brief Run learning modules at their configured rates.
 *
 * Call from main loop every 1ms (tied to systemTick).
 *
 * @param pData  Pointer to ESC runtime data
 * @param now    Current systemTick (1ms)
 */
void LEARN_Service(volatile GARUDA_DATA_T *pData, uint32_t now);

#endif /* FEATURE_LEARN_MODULES */

#ifdef __cplusplus
}
#endif

#endif /* LEARN_SERVICE_H */
