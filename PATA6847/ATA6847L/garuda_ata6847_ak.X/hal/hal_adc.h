/**
 * @file hal_adc.h
 * @brief ADC HAL for dsPIC33AK128MC106 on EV92R69A + EV68M17A.
 *
 * Channel mapping (per DS70005527 DIM info sheet):
 *   AD1CH1 (PINSEL=10, AN10/RA11/DIM 28) — Potentiometer → AD1CH1DATA
 *   AD1CH4 (PINSEL=6,  AN6/RA7/DIM 39)   — Vbus voltage  → AD1CH4DATA
 *
 * Phase current sensing (ATA6847L CSA) is reserved for a future port
 * extension once we re-map the ATA6847L CSA outputs to AK ADC inputs.
 * BEMF ZC is GPIO-read from ATA6847L digital comparators, not ADC.
 */
#ifndef HAL_ADC_H
#define HAL_ADC_H

#include <stdint.h>

void HAL_ADC_Init(void);
void HAL_ADC_Enable(void);
void HAL_ADC_Disable(void);
void HAL_ADC_InterruptEnable(void);
void HAL_ADC_InterruptDisable(void);
void HAL_ADC_PollPotVbus(uint16_t *pot, uint16_t *vbus);

#endif
