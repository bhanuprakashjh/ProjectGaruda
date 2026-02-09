/**
 * @file hal_comparator.h
 *
 * @brief Comparator/DAC module interface for BEMF zero-crossing detection.
 * Adapted from AN1292 reference — initializes all 3 CMPs for ZC instead
 * of just CMP3 for overcurrent.
 *
 * CMP1: INSEL=1 (B input) -> CMP1B=RA4 (Phase A BEMF)
 * CMP2: INSEL=1 (B input) -> CMP2B=RB2 (Phase B BEMF)
 * CMP3: INSEL=1 (B input) -> CMP3B=RB5 (Phase C BEMF)
 * DAC1/2/3 reference = Vbus/2
 *
 * Component: COMPARATOR
 */

#ifndef _HAL_COMPARATOR_H
#define _HAL_COMPARATOR_H

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void InitializeCMPs(void);
void HAL_CMP_EnableFloatingPhase(uint8_t phase);
void HAL_CMP_SetReference(uint16_t vbusHalf);

#ifdef __cplusplus
}
#endif

#endif /* _HAL_COMPARATOR_H */
