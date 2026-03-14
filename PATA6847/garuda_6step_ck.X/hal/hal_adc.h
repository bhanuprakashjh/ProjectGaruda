/**
 * @file hal_adc.h
 * @brief ADC HAL for dsPIC33CK on EV43F54A.
 *
 * Channel mapping:
 *   AN6  = Potentiometer (speed reference)  → ADCBUF6
 *   AN9  = Vbus voltage                     → ADCBUF9
 *   AN0  = Phase C current (ATA6847 CSA)    → ADCBUF0
 *   AN1  = Phase A current (ATA6847 CSA)    → ADCBUF1
 *   AN4  = Phase B current (ATA6847 CSA)    → ADCBUF4
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
