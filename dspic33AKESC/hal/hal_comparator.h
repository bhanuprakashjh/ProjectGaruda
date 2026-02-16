/**
 * @file hal_comparator.h
 *
 * @brief Comparator/DAC module interface.
 *
 * NOTE: CMP is NOT used for BEMF zero-crossing detection on the MCLV-48V-300W
 * board (EV68M17A DIM). Phase voltage pins (RB9/RB8/RA10) are not routable to
 * any CMP input. ZC detection uses ADC software threshold instead (see bemf_zc.c).
 *
 * This module is retained for future use as an overcurrent PCI fault source
 * (CMP3 → PCI PSS=0b11101). InitializeCMPs() is NOT called at startup.
 * Re-add the call in HAL_InitPeripherals() if PCI fault is switched to CMP3.
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
uint8_t HAL_CMP_ReadStatus(uint8_t phase);

#ifdef __cplusplus
}
#endif

#endif /* _HAL_COMPARATOR_H */
