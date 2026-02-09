/**
 * @file hal_adc.c
 *
 * @brief ADC module configuration for BEMF sensing.
 * Adapted from AN1292 reference:
 *   - Keeps: AD1/AD2 core enable, Vbus (AD1CH4, RA7, PINSEL=6),
 *            Pot (AD1CH1, RA11, PINSEL=10), PWM trigger source
 *   - Removes: Current sense channels (IA on AD1CH0, IB on AD2CH0, IBUS)
 *   - Adds: BEMF_A on AD1CH0 (RA4, PINSEL=1),
 *           BEMF_B on AD2CH0 (RB2, PINSEL=4),
 *           BEMF_C on AD2CH0 (RB5, PINSEL=2 — muxed with BEMF_B)
 *
 * Definitions in this file are for dsPIC33AK128MC106
 *
 * Component: ADC
 */

#include <xc.h>
#include <stdint.h>

#include "hal_adc.h"

/**
 * @brief Initialize and enable both ADC cores with BEMF, Vbus, and Pot channels.
 */
void InitializeADCs(void)
{
    /* BEMF_A on AD1CH0: RA4 = AD1AN1, PINSEL=1 */
    AD1CH0CONbits.PINSEL = 1;
    AD1CH0CONbits.SAMC = 3;
    AD1CH0CONbits.LEFT = 0;
    AD1CH0CONbits.DIFF = 0;

    /* BEMF_B on AD2CH0: RB2 = AD2AN4, PINSEL=4 (default; muxed with BEMF_C) */
    AD2CH0CONbits.PINSEL = 4;
    AD2CH0CONbits.SAMC = 3;
    AD2CH0CONbits.LEFT = 0;
    AD2CH0CONbits.DIFF = 0;

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

    /* ADC interrupt on BEMF_A completion (AD1CH0) */
    _AD1CH0IP = 7;         /* Highest priority */
    _AD1CH0IF = 0;
    _AD1CH0IE = 0;         /* Disabled until service init */

    /* Trigger sources: PWM1 ADC Trigger 1 (TRG1SRC=4) */
    AD1CH0CONbits.TRG1SRC = 4;     /* BEMF_A from PWM1 trigger */
    AD2CH0CONbits.TRG1SRC = 4;     /* BEMF_B/C from PWM1 trigger */
    AD1CH1CONbits.TRG1SRC = 4;     /* POT from PWM1 trigger */
    AD1CH4CONbits.TRG1SRC = 4;     /* VBUS from PWM1 trigger */
}

/**
 * @brief Select which BEMF channel to sample on AD2CH0 for the current
 * floating phase. BEMF_B and BEMF_C share AD2, so we mux PINSEL.
 *
 * @param floatingPhase 0=A (read from AD1CH0), 1=B (AD2CH0 PINSEL=4),
 *                      2=C (AD2CH0 PINSEL=2)
 */
void HAL_ADC_SelectBEMFChannel(uint8_t floatingPhase)
{
    switch (floatingPhase)
    {
        case FLOATING_PHASE_A:
            /* BEMF_A is on AD1CH0 — no mux change needed for AD2 */
            break;
        case FLOATING_PHASE_B:
            /* BEMF_B: RB2 = AD2AN4, PINSEL=4 */
            AD2CH0CONbits.PINSEL = 4;
            break;
        case FLOATING_PHASE_C:
            /* BEMF_C: RB5 = AD2AN2, PINSEL=2 */
            AD2CH0CONbits.PINSEL = 2;
            break;
    }
}
