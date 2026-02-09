/**
 * @file hal_adc.h
 *
 * @brief ADC module definitions for BEMF sensing, Vbus, and potentiometer.
 * Adapted from AN1292 reference:
 *   - Keeps: Vbus (AD1CH4), Pot (AD1CH1), PWM trigger source
 *   - Removes: Current sense channels (IA, IB, IBUS)
 *   - Adds: BEMF_A (AD1CH0), BEMF_B/C on AD2CH0 (muxed)
 *
 * Definitions in this file are for dsPIC33AK128MC106
 *
 * Component: ADC
 */

#ifndef _HAL_ADC_H
#define _HAL_ADC_H

#include <xc.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum count in 12-bit ADC */
#define MAX_ADC_COUNT       4096.0f
#define HALF_ADC_COUNT      2048

/* ADC buffer read macros */
#define ADCBUF_BEMF_A       (int16_t)AD1CH0DATA
#define ADCBUF_BEMF_BC      (int16_t)AD2CH0DATA     /* BEMF_B or BEMF_C depending on mux */
#define ADCBUF_POT          (int16_t)AD1CH1DATA
#define ADCBUF_VBUS         (int16_t)AD1CH4DATA

/* BEMF_A completion (AD1CH0) is the ADC interrupt source */
#define GARUDA_EnableADCInterrupt()     _AD1CH0IE = 1
#define GARUDA_DisableADCInterrupt()    _AD1CH0IE = 0
#define GARUDA_ADC_INTERRUPT            _AD1CH0Interrupt
#define GARUDA_ClearADCIF()             _AD1CH0IF = 0

/* Floating phase identifiers for BEMF mux selection */
#define FLOATING_PHASE_A    0
#define FLOATING_PHASE_B    1
#define FLOATING_PHASE_C    2

void InitializeADCs(void);
void HAL_ADC_SelectBEMFChannel(uint8_t floatingPhase);

#ifdef __cplusplus
}
#endif

#endif /* _HAL_ADC_H */
