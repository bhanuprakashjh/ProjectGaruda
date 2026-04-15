/**
 * @file hal_ic.c
 * @brief SCCP1 ZC polling timer for BEMF zero-crossing detection.
 *
 * Configures SCCP1 as a periodic timer at ZC_POLL_FREQ_HZ (100 kHz default).
 * Each period match fires CCT1IF → _CCT1Interrupt ISR in garuda_service.c.
 *
 * Previous approach used SCCP1/2/3 as Input Capture to detect edges on
 * ATA6847 digital BEMF outputs. This failed at high speed because IC
 * captured every PWM-induced comparator bounce (~32 per ZC), overwhelming
 * the ISR with 14000+ chatter events and failing the deglitch filter.
 *
 * New approach: periodic timer polls comparator state directly. No edge
 * detection, no bounce amplification. Deglitch is inherent in the
 * consecutive-matching-reads approach. Step 0 experiment confirmed
 * ATA6847 comparator is PWM-independent (valid during both ON and OFF).
 *
 * SCCP2/3 are not used — only SCCP1 needed (polls the active floating phase).
 */

#include "hal_ic.h"

#if FEATURE_IC_ZC && !FEATURE_V4_SECTOR_PI

#include <xc.h>

void HAL_ZcTimer_Init(void)
{
    /* ── SCCP1: periodic timer for ZC polling ──────────────────────── */
    CCP1CON1L = 0;
    CCP1CON1H = 0;
    CCP1CON2L = 0;
    CCP1CON2H = 0;

    CCP1CON1Lbits.CCSEL  = 0;              /* Timer mode (NOT Input Capture) */
    CCP1CON1Lbits.T32    = 0;              /* 16-bit timer */
    CCP1CON1Lbits.CLKSEL = 0b000;          /* Fp (Fosc/2 = 100 MHz) */
    CCP1CON1Lbits.TMRPS  = 0b00;           /* 1:1 prescaler (10 ns/tick) */
    CCP1CON1Lbits.MOD    = 0b0000;         /* Edge-aligned timer, auto-reload */

    CCP1PRL = ZC_POLL_PERIOD - 1;           /* Period for desired frequency */

    /* Timer period interrupt (CCT1IF) — NOT capture/compare (CCP1IF).
     * In timer mode MOD=0, period match sets CCT1IF and resets timer. */
    _CCT1IP = ZC_POLL_ISR_PRIORITY;
    _CCT1IF = 0;
    _CCT1IE = 0;                            /* Enabled at CL entry */

    /* Disable capture/compare interrupt (not used in timer mode) */
    _CCP1IP = 1;
    _CCP1IF = 0;
    _CCP1IE = 0;

    /* Start timer — runs continuously, ISR gated by CCT1IE */
    CCP1CON1Lbits.CCPON = 1;

}

void HAL_ZcTimer_Start(void)
{
    CCP1TMRL = 0;           /* Reset timer for clean first period */
    _CCT1IF = 0;
    _CCT1IE = 1;            /* Enable periodic interrupt */
}

void HAL_ZcTimer_Stop(void)
{
    _CCT1IE = 0;            /* Disable interrupt — timer keeps running */
    _CCT1IF = 0;
#if FEATURE_IC_ZC_CAPTURE
    HAL_ZcIC_Disarm();      /* Also disarm IC capture */
#endif
}

/* ── SCCP2 Input Capture for hardware-precise ZC timestamps ────────── */
#if FEATURE_IC_ZC_CAPTURE

void HAL_ZcIC_Init(void)
{
    /* SCCP2: Input Capture mode, same clock as SCCP4 (Fp/64 = 640ns/tick).
     * Captures CCP2TMRL into CCP2BUFL on selected edge of ICM2 input.
     * Timer runs free (PRL=0xFFFF). ISR disabled until armed after blanking. */
    CCP2CON1L = 0;
    CCP2CON1H = 0;
    CCP2CON2L = 0;
    CCP2CON2H = 0;

    CCP2CON1Lbits.CCSEL  = 1;          /* Input Capture mode */
    CCP2CON1Lbits.T32    = 0;          /* 16-bit timer */
    CCP2CON1Lbits.CLKSEL = 0b000;     /* Fp (Fosc/2 = 100 MHz) */
    CCP2CON1Lbits.TMRPS  = 0b11;      /* 1:64 prescaler → 640 ns/tick (same as SCCP4) */
    /* In IC mode (CCSEL=1), MOD selects capture edge (DS70005349 Table 22-3):
     * 0001 = every rising edge
     * 0010 = every falling edge
     * ATA6847 comparator is inverted: rising ZC = falling comp edge.
     * Single-polarity capture avoids ISR storm from noise edges.
     * If IC misses (comp settled before capture), poll path catches it. */
    CCP2CON1Lbits.MOD    = 0b0010;    /* Falling edge default (rising ZC) */

    CCP2PRL = 0xFFFF;                  /* Free-running (no period reset) */

    /* Interrupt: disabled until armed. Priority same as poll. */
    _CCP2IP = ZC_IC_ISR_PRIORITY;
    _CCP2IF = 0;
    _CCP2IE = 0;

    /* Timer period interrupt not used */
    _CCT2IP = 1;
    _CCT2IF = 0;
    _CCT2IE = 0;

    /* Module OFF until CL entry. Configure() enables it.
     * Prevents FIFO filling with stale captures during OL_RAMP. */
    CCP2CON1Lbits.CCPON = 0;
}

void HAL_ZcIC_Configure(uint16_t rpPin, bool risingEdge)
{
    /* Disarm during reconfiguration */
    _CCP2IE = 0;
    _CCP2IF = 0;

    /* PPS: route the selected BEMF comparator pin to SCCP2 IC input.
     * Note: PPS lock must be opened briefly for runtime re-routing.
     * On dsPIC33CK, writing RPCON=0 unlocks, 0x0800 locks. */
    __builtin_write_RPCON(0x0000);      /* unlock PPS */
    RPINR4bits.ICM2R = (uint8_t)rpPin;  /* route pin to ICM2 */
    __builtin_write_RPCON(0x0800);      /* lock PPS */

    /* Reset module, set correct polarity, re-enable.
     * ATA6847 inverted: rising ZC = falling comp edge, and vice versa.
     * Single-polarity capture: one ISR per step (no noise storm). */
    CCP2CON1Lbits.CCPON = 0;   /* Disable: clears timer and FIFO */
    if (risingEdge)
        CCP2CON1Lbits.MOD = 0b0010;    /* Capture falling edge (rising ZC) */
    else
        CCP2CON1Lbits.MOD = 0b0001;    /* Capture rising edge (falling ZC) */
    CCP2CON1Lbits.CCPON = 1;   /* Re-enable with clean state */
}

void HAL_ZcIC_Arm(void)
{
    /* Drain the 4-deep capture FIFO completely */
    (void)CCP2BUFL;
    (void)CCP2BUFL;
    (void)CCP2BUFL;
    (void)CCP2BUFL;
    _CCP2IF = 0;
    _CCP2IE = 1;
}

#endif /* FEATURE_IC_ZC_CAPTURE */

#endif /* FEATURE_IC_ZC */
