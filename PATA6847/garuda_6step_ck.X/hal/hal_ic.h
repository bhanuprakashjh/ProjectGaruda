/**
 * @file hal_ic.h
 * @brief SCCP Input Capture HAL for BEMF zero-crossing detection.
 *
 * Uses SCCP1/2/3 mapped via PPS to ATA6847 digital BEMF comparator outputs:
 *   SCCP1 ← RC6/RP54 (BEMF_A)
 *   SCCP2 ← RC7/RP55 (BEMF_B)
 *   SCCP3 ← RD10/RP74 (BEMF_C)
 *
 * Only the floating phase's SCCP is active at any time.
 * Timer: 1:64 prescaler → 640 ns/tick, 41.9 ms 16-bit range.
 *
 * Enabled by FEATURE_IC_ZC=1 in garuda_config.h.
 */

#ifndef HAL_IC_H
#define HAL_IC_H

#include "../garuda_config.h"

#if FEATURE_IC_ZC

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize SCCP1/2/3 for Input Capture mode.
 * Configures timers, prescaler, and IC mode but leaves all channels disabled.
 * PPS mapping must be done in SetupGPIOPorts() before calling this.
 */
void HAL_IC_Init(void);

/**
 * @brief Arm a specific IC channel for ZC detection.
 * Sets the edge mode (rising/falling) and clears ICDIS to enable capture.
 * @param channel 0=SCCP1(A), 1=SCCP2(B), 2=SCCP3(C)
 * @param risingEdge true for rising ZC, false for falling ZC
 */
void HAL_IC_Arm(uint8_t channel, bool risingEdge);

/**
 * @brief Disable capture on a specific IC channel.
 * Sets MOD=0 to prevent further capture events.
 * @param channel 0=SCCP1(A), 1=SCCP2(B), 2=SCCP3(C)
 */
void HAL_IC_Disable(uint8_t channel);

/**
 * @brief Disable all IC channels. Called at motor stop.
 */
void HAL_IC_DisableAll(void);

#endif /* FEATURE_IC_ZC */
#endif /* HAL_IC_H */
