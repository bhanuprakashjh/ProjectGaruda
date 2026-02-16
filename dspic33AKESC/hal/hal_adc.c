/**
 * @file hal_adc.c
 *
 * @brief ADC module configuration for phase voltage sensing.
 * Adapted from AN1292 reference:
 *   - Keeps: AD1/AD2 core enable, Vbus (AD1CH4, RA7, PINSEL=6),
 *            Pot (AD1CH1, RA11, PINSEL=10), PWM trigger source
 *   - Removes: Current sense channels (IA on AD1CH0, IB on AD2CH0, IBUS)
 *   - Adds: Phase B on AD1CH0 (RB8, PINSEL=11) — interrupt source,
 *           Phase A on AD2CH0 (RB9, PINSEL=10) — default mux,
 *           Phase C on AD2CH0 (RA10, PINSEL=7) — muxed with Phase A
 *
 * Definitions in this file are for dsPIC33AK128MC106
 *
 * Component: ADC
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "hal_adc.h"

/**
 * @brief Initialize and enable both ADC cores with phase voltage, Vbus, and Pot channels.
 */
void InitializeADCs(void)
{
    /* Phase B on AD1CH0: RB8 = AD1AN11, PINSEL=11 — interrupt source, always sampled */
    AD1CH0CONbits.PINSEL = 11;
    AD1CH0CONbits.SAMC = 5;        /* Increased for divider impedance */
    AD1CH0CONbits.LEFT = 0;
    AD1CH0CONbits.DIFF = 0;

    /* Phase A (default) on AD2CH0: RB9 = AD2AN10, PINSEL=10 (muxed with Phase C) */
    AD2CH0CONbits.PINSEL = 10;
    AD2CH0CONbits.SAMC = 5;        /* Increased for divider impedance */
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

    /* ADC interrupt on Phase B completion (AD1CH0) */
    _AD1CH0IP = 7;         /* Highest priority */
    _AD1CH0IF = 0;
    _AD1CH0IE = 0;         /* Disabled until service init */

    /* Trigger sources: PWM1 ADC Trigger 1 (TRG1SRC=4) */
    AD1CH0CONbits.TRG1SRC = 4;     /* Phase B from PWM1 trigger */
    AD2CH0CONbits.TRG1SRC = 4;     /* Phase A/C from PWM1 trigger */
    AD1CH1CONbits.TRG1SRC = 4;     /* POT from PWM1 trigger */
    AD1CH4CONbits.TRG1SRC = 4;     /* VBUS from PWM1 trigger */
}

/**
 * @brief Select which phase voltage to sample on AD2CH0 for the current
 * floating phase. Phase A and C share AD2, so we mux PINSEL.
 * Phase B is on AD1CH0 (always sampled).
 *
 * @param floatingPhase 0=A (AD2CH0 PINSEL=10), 1=B (AD1CH0, no mux change),
 *                      2=C (AD2CH0 PINSEL=7)
 * @return true if AD2CH0 PINSEL was changed (caller must discard next sample)
 */
bool HAL_ADC_SelectBEMFChannel(uint8_t floatingPhase)
{
    switch (floatingPhase)
    {
        case FLOATING_PHASE_A:
            if (AD2CH0CONbits.PINSEL != 10) {
                AD2CH0CONbits.PINSEL = 10;  /* M1_VA: RB9 = AD2AN10 */
                return true;
            }
            return false;
        case FLOATING_PHASE_B:
            return false;  /* VB on AD1CH0 — no mux change */
        case FLOATING_PHASE_C:
            if (AD2CH0CONbits.PINSEL != 7) {
                AD2CH0CONbits.PINSEL = 7;   /* M1_VC: RA10 = AD2AN7 */
                return true;
            }
            return false;
        default:
            return false;
    }
}
