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

#if FEATURE_IC_ZC

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

    /* SCCP2/3: not used in fast-poll mode. Leave at power-on defaults
     * (CCPON=0, interrupts disabled). */
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
}

#endif /* FEATURE_IC_ZC */
