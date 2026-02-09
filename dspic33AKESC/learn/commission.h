/**
 * @file commission.h
 *
 * @brief Self-commissioning state machine for motor parameter detection.
 *
 * Staged sequence:
 *   IDLE -> STATIC_R -> STATIC_L -> DYNAMIC_KV -> DYNAMIC_POLES
 *   -> DYNAMIC_TIMING -> VALIDATE -> COMMIT -> COMPLETE
 *
 * Static tests (R, L): motor stopped, DC injection / voltage step response.
 * Dynamic tests (Kv, poles, timing): protected low-speed spin, reads
 * telemetry ring buffer. Each state has a timeout + target sample count.
 *
 * Component: LEARN / COMMISSION
 */

#ifndef COMMISSION_H
#define COMMISSION_H

#include <stdint.h>
#include <stdbool.h>
#include "../garuda_types.h"
#include "../garuda_config.h"
#include "ring_buffer.h"

#ifdef __cplusplus
extern "C" {
#endif

#if FEATURE_COMMISSION

/**
 * @brief Initialize commissioning data to idle state.
 * @param comm  Pointer to commissioning data
 */
void COMMISSION_Init(COMMISSION_DATA_T *comm);

/**
 * @brief Start the commissioning sequence.
 * @param comm   Pointer to commissioning data
 * @param state  Current ESC state (must be ESC_IDLE)
 * @param now    Current systemTick (1ms) for timeout tracking
 * @return true if started, false if wrong state
 */
bool COMMISSION_Start(COMMISSION_DATA_T *comm, ESC_STATE_T state, uint32_t now);

/**
 * @brief Update the commissioning state machine (call from main loop).
 *
 * Drives the staged commissioning sequence. Each state collects
 * samples and transitions when target count or timeout is reached.
 *
 * @param comm   Pointer to commissioning data
 * @param pData  Pointer to ESC runtime data
 * @param ring   Pointer to telemetry ring buffer
 * @param now    Current systemTick (1ms)
 * @return true if commissioning is still in progress, false when done/error
 */
bool COMMISSION_Update(COMMISSION_DATA_T *comm, volatile GARUDA_DATA_T *pData,
                       TELEM_RING_T *ring, uint32_t now);

/**
 * @brief Abort commissioning and return to safe state.
 * @param comm   Pointer to commissioning data
 * @param pData  Pointer to ESC runtime data (to disable outputs)
 */
void COMMISSION_Abort(COMMISSION_DATA_T *comm, volatile GARUDA_DATA_T *pData);

/**
 * @brief Get commissioning progress as 0-100 percentage.
 * @param comm  Pointer to commissioning data
 * @return Progress percentage
 */
uint8_t COMMISSION_GetProgress(const COMMISSION_DATA_T *comm);

/**
 * @brief Apply commissioning results to config and adaptation params.
 * @param comm    Pointer to commissioning data (must be COMM_COMPLETE)
 * @param config  Pointer to user config to update
 * @param adapt   Pointer to adaptation params to initialize
 */
void COMMISSION_ApplyResults(const COMMISSION_DATA_T *comm,
                             GARUDA_CONFIG_T *config, ADAPT_PARAMS_T *adapt);

#endif /* FEATURE_COMMISSION */

#ifdef __cplusplus
}
#endif

#endif /* COMMISSION_H */
