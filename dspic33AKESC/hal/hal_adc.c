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
#include "../garuda_config.h"

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

#if FEATURE_HW_OVERCURRENT
    /* Bus current on AD1CH2: RA5 = OA3OUT = AD1AN3, PINSEL=3 */
    AD1CH2CONbits.PINSEL = 3;
    AD1CH2CONbits.SAMC = 3;        /* Fast sample (low Z from OA3 output) */
    AD1CH2CONbits.LEFT = 0;
    AD1CH2CONbits.DIFF = 0;
    AD1CH2CONbits.TRG1SRC = 4;     /* PWM1 trigger (24kHz, midpoint sampling) */
#endif

#if FEATURE_ADC_CMP_ZC
    /* High-speed BEMF channels configured BEFORE ADC ON.
     * Datasheet warns configuring channels when ADON=1 is unpredictable
     * for non-PWM trigger sources. TRG1SRC=14 (SCCP3 Trigger out).
     * Comparator disabled initially (CMPMOD=0). */

    /* AD1CH5: Phase B high-speed (RB8 = AD1AN11) */
    AD1CH5CONbits.PINSEL = 11;
    AD1CH5CONbits.SAMC = HWZC_SAMC;
    AD1CH5CONbits.LEFT = 0;
    AD1CH5CONbits.DIFF = 0;
    AD1CH5CONbits.TRG1SRC = 14;  /* SCCP3 Trigger out */
    AD1CH5CONbits.TRG2SRC = 2;   /* Immediate re-trigger for oversampling repeats */
    AD1CH5CONbits.MODE = 0b11;   /* Oversampling mode */
    AD1CH5CONbits.ACCNUM = 0b00; /* 4 samples, result right-shifted by 2 bits */
    AD1CH5CONbits.ACCBRST = 1;   /* Non-interruptible burst (prevent 24kHz split) */
    AD1CH5CONbits.CMPMOD = 0;    /* Comparator disabled initially */
    AD1CH5CMPLO = 0;
    AD1CH5CMPHI = 0;

    /* AD2CH1: Phase A/C high-speed (initial: RB9 = AD2AN10) */
    AD2CH1CONbits.PINSEL = 10;
    AD2CH1CONbits.SAMC = HWZC_SAMC;
    AD2CH1CONbits.LEFT = 0;
    AD2CH1CONbits.DIFF = 0;
    AD2CH1CONbits.TRG1SRC = 14;  /* SCCP3 Trigger out */
    AD2CH1CONbits.TRG2SRC = 2;   /* Immediate re-trigger for oversampling repeats */
    AD2CH1CONbits.MODE = 0b11;   /* Oversampling mode */
    AD2CH1CONbits.ACCNUM = 0b00; /* 4 samples, result right-shifted by 2 bits */
    AD2CH1CONbits.ACCBRST = 1;   /* Non-interruptible burst (prevent 24kHz split) */
    AD2CH1CONbits.CMPMOD = 0;    /* Comparator disabled initially */
    AD2CH1CMPLO = 0;
    AD2CH1CMPHI = 0;
#endif

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

#if FEATURE_ADC_CMP_ZC

/**
 * @brief Post-ON setup for high-speed BEMF comparator channels.
 * Channel configuration (PINSEL, SAMC, TRG1SRC=14) is done in
 * InitializeADCs() BEFORE ADC ON — required for non-PWM trigger
 * sources per datasheet.  This function just clears interrupt
 * flags and leaves comparator interrupts disabled.
 *
 * Trigger: TRG1SRC=14 (SCCP3 Trigger out) at HWZC_ADC_SAMPLE_HZ.
 * Mode: 4x oversampling (MODE=11, ACCNUM=00, TRG2SRC=2).
 * Each SCCP3 trigger produces a 4-sample averaged result (+6 dB SNR).
 * Comparator fires on the averaged result; threshold scale unchanged.
 */
void HAL_ADC_InitHighSpeedBEMF(void)
{
    /* Comparator interrupts: clear and leave disabled */
    _AD1CMP5IF = 0;
    _AD1CMP5IE = 0;
    _AD2CMP1IF = 0;
    _AD2CMP1IE = 0;
}

/**
 * @brief Configure comparator mode and threshold on a high-speed channel.
 * @param adcCore  1=AD1CH5 (Phase B), 2=AD2CH1 (Phase A/C)
 * @param threshold  ADC count threshold for CMPLO register
 * @param risingZc  true=rising ZC (greater-than), false=falling (less-or-equal)
 */
void HAL_ADC_ConfigComparator(uint8_t adcCore, uint16_t threshold, bool risingZc)
{
    if (adcCore == 1)
    {
        AD1CH5CONbits.CMPMOD = risingZc ? 0b011 : 0b100;
        AD1CH5CMPLO = threshold;
    }
    else
    {
        AD2CH1CONbits.CMPMOD = risingZc ? 0b011 : 0b100;
        AD2CH1CMPLO = threshold;
    }
}

/**
 * @brief Enable comparator interrupt on a high-speed channel.
 * Clears flag before enabling to prevent stale triggers.
 * @param adcCore  1=AD1CH5, 2=AD2CH1
 */
void HAL_ADC_EnableComparatorIE(uint8_t adcCore)
{
    if (adcCore == 1)
    {
        _AD1CMP5IF = 0;
        _AD1CMP5IE = 1;
    }
    else
    {
        _AD2CMP1IF = 0;
        _AD2CMP1IE = 1;
    }
}

/**
 * @brief Disable comparator interrupt on a high-speed channel.
 * @param adcCore  1=AD1CH5, 2=AD2CH1
 */
void HAL_ADC_DisableComparatorIE(uint8_t adcCore)
{
    if (adcCore == 1)
        _AD1CMP5IE = 0;
    else
        _AD2CMP1IE = 0;
}

/**
 * @brief Clear comparator status flag and interrupt flag.
 * @param adcCore  1=AD1CH5, 2=AD2CH1
 */
void HAL_ADC_ClearComparatorFlag(uint8_t adcCore)
{
    if (adcCore == 1)
    {
        AD1CMPSTATbits.CH5CMP = 0;
        _AD1CMP5IF = 0;
    }
    else
    {
        AD2CMPSTATbits.CH1CMP = 0;
        _AD2CMP1IF = 0;
    }
}

/**
 * @brief Set PINSEL for AD2CH1 high-speed channel (Phase A vs Phase C mux).
 * @param pinsel  10=Phase A (RB9), 7=Phase C (RA10)
 */
void HAL_ADC_SetHighSpeedPinsel(uint8_t pinsel)
{
    AD2CH1CONbits.PINSEL = pinsel;
}

#endif /* FEATURE_ADC_CMP_ZC */
