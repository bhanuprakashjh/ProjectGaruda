/**
 * @file hal_ic.c
 * @brief SCCP Input Capture HAL for BEMF zero-crossing detection.
 *
 * Configures SCCP1/2/3 in Input Capture mode with 1:64 prescaler
 * (640 ns/tick, 41.9 ms range). Each channel maps to one BEMF phase
 * via PPS. Only the floating phase is armed during each commutation step.
 *
 * IC ISRs are in garuda_service.c alongside the other motor ISRs.
 */

#include "hal_ic.h"

#if FEATURE_IC_ZC

#include <xc.h>

void HAL_IC_Init(void)
{
    /* ── SCCP1: BEMF_A (RC6/RP54) ─────────────────────────────── */
    CCP1CON1L = 0;
    CCP1CON1H = 0;
    CCP1CON2L = 0;
    CCP1CON2H = 0;

    CCP1CON1Lbits.CCSEL  = 1;              /* Input Capture mode */
    CCP1CON1Lbits.T32    = 0;              /* 16-bit timer */
    CCP1CON1Lbits.CLKSEL = 0b000;          /* Fp (Fosc/2 = 100 MHz) */
    CCP1CON1Lbits.TMRPS  = IC_ZC_PRESCALER; /* 1:64 → 640 ns/tick */
    CCP1CON1Lbits.MOD    = 0b0000;         /* Disabled initially */

    CCP1CON2Hbits.ICS    = 0b000;          /* IC source = PPS pin */
    CCP1CON2Hbits.ICGSM  = 0b00;           /* No hardware gating (V1) */

    CCP1CON1Lbits.CCPON  = 1;              /* Enable module (timer runs) */

    _CCP1IP = IC_ZC_ISR_PRIORITY;
    _CCP1IF = 0;
    _CCP1IE = 1;                            /* Enable interrupt */

    /* ── SCCP2: BEMF_B (RC7/RP55) ─────────────────────────────── */
    CCP2CON1L = 0;
    CCP2CON1H = 0;
    CCP2CON2L = 0;
    CCP2CON2H = 0;

    CCP2CON1Lbits.CCSEL  = 1;
    CCP2CON1Lbits.T32    = 0;
    CCP2CON1Lbits.CLKSEL = 0b000;
    CCP2CON1Lbits.TMRPS  = IC_ZC_PRESCALER;
    CCP2CON1Lbits.MOD    = 0b0000;

    CCP2CON2Hbits.ICS    = 0b000;
    CCP2CON2Hbits.ICGSM  = 0b00;

    CCP2CON1Lbits.CCPON  = 1;

    _CCP2IP = IC_ZC_ISR_PRIORITY;
    _CCP2IF = 0;
    _CCP2IE = 1;

    /* ── SCCP3: BEMF_C (RD10/RP74) ────────────────────────────── */
    CCP3CON1L = 0;
    CCP3CON1H = 0;
    CCP3CON2L = 0;
    CCP3CON2H = 0;

    CCP3CON1Lbits.CCSEL  = 1;
    CCP3CON1Lbits.T32    = 0;
    CCP3CON1Lbits.CLKSEL = 0b000;
    CCP3CON1Lbits.TMRPS  = IC_ZC_PRESCALER;
    CCP3CON1Lbits.MOD    = 0b0000;

    CCP3CON2Hbits.ICS    = 0b000;
    CCP3CON2Hbits.ICGSM  = 0b00;

    CCP3CON1Lbits.CCPON  = 1;

    _CCP3IP = IC_ZC_ISR_PRIORITY;
    _CCP3IF = 0;
    _CCP3IE = 1;
}

void HAL_IC_Arm(uint8_t channel, bool risingEdge)
{
    uint16_t mod = risingEdge ? 0b0001 : 0b0010;

    /* Sequence: ensure MOD=0, drain FIFO, clear IF, THEN set MOD.
     * Setting MOD before drain risks capturing a new edge during drain,
     * and the ISR could fire on stale data. */
    switch (channel)
    {
        case 0:
            CCP1CON1Lbits.MOD = 0;
            while (CCP1STATbits.ICBNE)
                (void)CCP1BUFL;
            _CCP1IF = 0;
            CCP1CON1Lbits.MOD = mod;
            break;
        case 1:
            CCP2CON1Lbits.MOD = 0;
            while (CCP2STATbits.ICBNE)
                (void)CCP2BUFL;
            _CCP2IF = 0;
            CCP2CON1Lbits.MOD = mod;
            break;
        case 2:
            CCP3CON1Lbits.MOD = 0;
            while (CCP3STATbits.ICBNE)
                (void)CCP3BUFL;
            _CCP3IF = 0;
            CCP3CON1Lbits.MOD = mod;
            break;
    }
}

void HAL_IC_Disable(uint8_t channel)
{
    switch (channel)
    {
        case 0:
            CCP1CON1Lbits.MOD = 0;
            while (CCP1STATbits.ICBNE) (void)CCP1BUFL;
            _CCP1IF = 0;
            break;
        case 1:
            CCP2CON1Lbits.MOD = 0;
            while (CCP2STATbits.ICBNE) (void)CCP2BUFL;
            _CCP2IF = 0;
            break;
        case 2:
            CCP3CON1Lbits.MOD = 0;
            while (CCP3STATbits.ICBNE) (void)CCP3BUFL;
            _CCP3IF = 0;
            break;
    }
}

void HAL_IC_DisableAll(void)
{
    CCP1CON1Lbits.MOD = 0;
    CCP2CON1Lbits.MOD = 0;
    CCP3CON1Lbits.MOD = 0;
}

#endif /* FEATURE_IC_ZC */
