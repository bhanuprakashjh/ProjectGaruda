/**
 * @file hal_adc.h
 *
 * @brief ADC module definitions for FOC 3-phase current sensing, Vbus, and POT.
 * FOC configuration:
 *   - AD1CH0: Phase A current (OA1 output, RA2/AD1AN0, PINSEL=0)
 *   - AD2CH0: Phase B current (OA2 output, RB0/AD2AN1, PINSEL=1)
 *   - AD1CH2: Phase C current (OA3 output, RA5/AD1AN3, PINSEL=3)
 *   - AD1CH1: POT (RA11/AD1AN10, PINSEL=10) — unchanged
 *   - AD1CH4: VBUS (RA7/AD1AN6, PINSEL=6)  — unchanged
 *   - Interrupt: AD1CH0 completion (Phase A current)
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

/* ADC buffer read macros — FOC 3-phase current sensing */
#define ADCBUF_IA           (uint16_t)AD1CH0DATA   /* Phase A current (OA1 out, RA2/AD1AN0) */
#define ADCBUF_IB           (uint16_t)AD2CH0DATA   /* Phase B current (OA2 out, RB0/AD2AN1) */
#define ADCBUF_IC           (uint16_t)AD1CH2DATA   /* Phase C current (OA3 out, RA5/AD1AN3) */
#define ADCBUF_POT          (uint16_t)AD1CH1DATA   /* Potentiometer (RA11/AD1AN10) */
#define ADCBUF_VBUS         (uint16_t)AD1CH4DATA   /* DC bus voltage (RA7/AD1AN6) */

/* Phase A current completion (AD1CH0) is the ADC interrupt source */
#define GARUDA_EnableADCInterrupt()     _AD1CH0IE = 1
#define GARUDA_DisableADCInterrupt()    _AD1CH0IE = 0
#define GARUDA_ADC_INTERRUPT            _AD1CH0Interrupt
#define GARUDA_ClearADCIF()             _AD1CH0IF = 0

void InitializeADCs(void);


#ifdef __cplusplus
}
#endif

#endif /* _HAL_ADC_H */
