/**
 * @file hal_ic.h
 * @brief SCCP1 ZC polling timer HAL for BEMF zero-crossing detection.
 *
 * Replaces edge-triggered Input Capture with periodic timer polling.
 * SCCP1 fires at ZC_POLL_FREQ_HZ (default 100 kHz). The ISR reads
 * the ATA6847 BEMF comparator for the floating phase and applies an
 * adaptive deglitch filter (2-8 consecutive matching reads).
 *
 * SCCP4 free-running timer provides high-resolution timestamps (640 ns)
 * for both blanking and commutation scheduling.
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
 * @brief Initialize SCCP1 as periodic ZC polling timer.
 * Configures timer at ZC_POLL_FREQ_HZ but leaves interrupt disabled.
 * Timer starts running immediately (free-running).
 */
void HAL_ZcTimer_Init(void);

/**
 * @brief Start ZC polling (enable SCCP1 timer interrupt).
 * Call at CL entry.
 */
void HAL_ZcTimer_Start(void);

/**
 * @brief Stop ZC polling (disable SCCP1 timer interrupt).
 * Call at motor stop, fault, or recovery.
 */
void HAL_ZcTimer_Stop(void);

#endif /* FEATURE_IC_ZC */
#endif /* HAL_IC_H */
