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

/* BEMF comparator RP pin numbers for PPS routing.
 * RC6 = RP54 (0x36), RC7 = RP55 (0x37), RD10 = RP74 (0x4A).
 * Board hardware constants — used by multiple PPS consumers
 * (IC capture, DMA shadow), so declared at FEATURE_IC_ZC scope. */
#define BEMF_A_RP   0x0036U
#define BEMF_B_RP   0x0037U
#define BEMF_C_RP   0x004AU

/* ── SCCP2 Input Capture for hardware-precise ZC timestamps ────────── */
#if FEATURE_IC_ZC_CAPTURE

/**
 * @brief Initialize SCCP2 as Input Capture for ZC edge detection.
 * Timer runs at same prescaler as SCCP4 (Fp/64 = 640ns/tick).
 * Interrupt disabled until armed after blanking.
 */
void HAL_ZcIC_Init(void);

/**
 * @brief Route a BEMF comparator pin to SCCP2 IC input and set edge.
 * Call at each commutation to select the floating phase.
 * @param rpPin RP pin number (BEMF_A_RP, BEMF_B_RP, or BEMF_C_RP)
 * @param risingEdge true=capture rising edge, false=capture falling
 */
void HAL_ZcIC_Configure(uint16_t rpPin, bool risingEdge);

/**
 * @brief Arm SCCP2 IC for one-shot capture.
 * Call after blanking expires. Clears any stale flag and enables interrupt.
 */
void HAL_ZcIC_Arm(void);

/**
 * @brief Disarm SCCP2 IC (disable interrupt).
 * Called in IC ISR (one-shot) or at commutation/stop.
 */
static inline void HAL_ZcIC_Disarm(void)
{
    _CCP2IE = 0;
    _CCP2IF = 0;
}

/**
 * @brief Read the captured SCCP2 timer value (latched at edge).
 */
static inline uint16_t HAL_ZcIC_ReadCapture(void)
{
    return CCP2BUFL;
}

/**
 * @brief Read the current SCCP2 timer value (for backdate calculation).
 */
static inline uint16_t HAL_ZcIC_ReadTimer(void)
{
    return CCP2TMRL;
}

#endif /* FEATURE_IC_ZC_CAPTURE */

#endif /* FEATURE_IC_ZC */
#endif /* HAL_IC_H */
