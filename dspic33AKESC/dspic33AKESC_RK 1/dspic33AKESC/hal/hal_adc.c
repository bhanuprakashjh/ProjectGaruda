/**
 * @file hal_adc.c
 *
 * @brief ADC module configuration for FOC 3-phase current sensing.
 * FOC configuration:
 *   - AD1CH0: Phase A current (OA1 output, RA2/AD1AN0, PINSEL=0) — interrupt source
 *   - AD2CH0: Phase B current (OA2 output, RB0/AD2AN1, PINSEL=1)
 *   - AD1CH2: Phase C current (OA3 output, RA5/AD1AN3, PINSEL=3) — unconditional
 *   - AD1CH1: POT (RA11/AD1AN10, PINSEL=10) — unchanged
 *   - AD1CH4: VBUS (RA7/AD1AN6, PINSEL=6)  — unchanged
 *
 * Definitions in this file are for dsPIC33AK128MC106
 *
 * Component: ADC
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "hal_adc.h"
#include "../garuda_config.h"

/**
 * @brief Initialize and enable both ADC cores for FOC 3-phase current sensing.
 * All channels triggered by PWM1 (TRG1SRC=4) for synchronized sampling.
 */
void InitializeADCs(void)
{
    /* Phase A current on AD1CH0: RA2 = OA1OUT = AD1AN0, PINSEL=0 — interrupt source */
    AD1CH0CONbits.PINSEL = 0;
    AD1CH0CONbits.SAMC = 3;        /* Low Z from op-amp output — fast sample */
    AD1CH0CONbits.LEFT = 0;
    AD1CH0CONbits.DIFF = 0;

    /* Phase B current on AD2CH0: RB0 = OA2OUT = AD2AN1, PINSEL=1 */
    AD2CH0CONbits.PINSEL = 1;
    AD2CH0CONbits.SAMC = 3;
    AD2CH0CONbits.LEFT = 0;
    AD2CH0CONbits.DIFF = 0;

    /* Phase C current on AD1CH2: RA5 = OA3OUT = AD1AN3, PINSEL=3 */
    AD1CH2CONbits.PINSEL = 3;
    AD1CH2CONbits.SAMC = 3;
    AD1CH2CONbits.LEFT = 0;
    AD1CH2CONbits.DIFF = 0;

    /* POT on AD1CH1: RA11 = AD1AN10, PINSEL=10 */
    AD1CH1CONbits.PINSEL = 10;
    AD1CH1CONbits.SAMC = 5;
    AD1CH1CONbits.LEFT = 0;
    AD1CH1CONbits.DIFF = 0;

    /* VBUS on AD1CH4: RA7 = AD1AN6, PINSEL=6 */
    AD1CH4CONbits.PINSEL = 6;
    AD1CH4CONbits.SAMC = 5;
    AD1CH4CONbits.LEFT = 0;
    AD1CH4CONbits.DIFF = 0;

    /* Turn on ADC Core 1 */
    AD1CONbits.ON = 1;
    while (AD1CONbits.ADRDY == 0);

    /* Turn on ADC Core 2 */
    AD2CONbits.ON = 1;
    while (AD2CONbits.ADRDY == 0);

    /* ADC interrupt on Phase A current completion (AD1CH0) */
    _AD1CH0IP = 7;         /* Highest priority */
    _AD1CH0IF = 0;
    _AD1CH0IE = 0;         /* Disabled until service init */

    /* All channels triggered by PWM1 ADC Trigger 1 (TRG1SRC=4, bottom of counter) */
    AD1CH0CONbits.TRG1SRC = 4;     /* Phase A current from PWM1 trigger */
    AD2CH0CONbits.TRG1SRC = 4;     /* Phase B current from PWM1 trigger */
    AD1CH2CONbits.TRG1SRC = 4;     /* Phase C current from PWM1 trigger */
    AD1CH1CONbits.TRG1SRC = 4;     /* POT from PWM1 trigger */
    AD1CH4CONbits.TRG1SRC = 4;     /* VBUS from PWM1 trigger */
}
