/**
 * @file hal_adc.h
 *
 * @brief ADC module definitions for phase voltage sensing, Vbus, and potentiometer.
 * Adapted from AN1292 reference:
 *   - Keeps: Vbus (AD1CH4), Pot (AD1CH1), PWM trigger source
 *   - Removes: Current sense channels (IA, IB, IBUS)
 *   - Adds: Phase B on AD1CH0 (RB8), Phase A/C on AD2CH0 (RB9/RA10, muxed)
 *
 * Definitions in this file are for dsPIC33AK128MC106
 *
 * Component: ADC
 */

#ifndef _HAL_ADC_H
#define _HAL_ADC_H

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "../garuda_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum count in 12-bit ADC */
#define MAX_ADC_COUNT       4096.0f
#define HALF_ADC_COUNT      2048

/* ADC buffer read macros
 * AD1CH0 = Phase B voltage (RB8/AD1AN11) — interrupt source, always sampled
 * AD2CH0 = Phase A or C voltage (muxed per commutation step) */
#define ADCBUF_PHASE_B      (uint16_t)AD1CH0DATA
#define ADCBUF_PHASE_AC     (uint16_t)AD2CH0DATA
#define ADCBUF_POT          (uint16_t)AD1CH1DATA
#define ADCBUF_VBUS         (uint16_t)AD1CH4DATA

#if FEATURE_HW_OVERCURRENT
#define ADCBUF_IBUS         (uint16_t)AD1CH2DATA
#endif

/* Phase B completion (AD1CH0) is the ADC interrupt source */
#define GARUDA_EnableADCInterrupt()     _AD1CH0IE = 1
#define GARUDA_DisableADCInterrupt()    _AD1CH0IE = 0
#define GARUDA_ADC_INTERRUPT            _AD1CH0Interrupt
#define GARUDA_ClearADCIF()             _AD1CH0IF = 0

/* Floating phase identifiers for BEMF mux selection */
#define FLOATING_PHASE_A    0
#define FLOATING_PHASE_B    1
#define FLOATING_PHASE_C    2

void InitializeADCs(void);
bool HAL_ADC_SelectBEMFChannel(uint8_t floatingPhase);

#if FEATURE_ADC_CMP_ZC
void HAL_ADC_InitHighSpeedBEMF(void);
void HAL_ADC_ConfigComparator(uint8_t adcCore, uint16_t threshold, bool risingZc);
void HAL_ADC_EnableComparatorIE(uint8_t adcCore);
void HAL_ADC_DisableComparatorIE(uint8_t adcCore);
void HAL_ADC_ClearComparatorFlag(uint8_t adcCore);
void HAL_ADC_SetHighSpeedPinsel(uint8_t pinsel);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _HAL_ADC_H */
