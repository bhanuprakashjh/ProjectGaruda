/**
 * @file hal_sector_timer.c
 * @brief V4 Sector Timer HAL — SCCP3 periodic + SCCP4 HR timestamp.
 */

#include "hal_sector_timer.h"

#if FEATURE_V4_SECTOR_PI

#include <xc.h>

void HAL_SectorTimer_Init(void)
{
    /* ── SCCP4: free-running HR timestamp (unchanged from V3) ──────── */
    CCP4CON1L = 0;
    CCP4CON1H = 0;
    CCP4CON2L = 0;
    CCP4CON2H = 0;
    CCP4CON3H = 0;

    CCP4CON1Lbits.CCSEL  = 0;          /* Timer mode */
    CCP4CON1Lbits.T32    = 0;          /* 16-bit */
    CCP4CON1Lbits.CLKSEL = 0b000;      /* Fp (100 MHz) */
    CCP4CON1Lbits.TMRPS  = 0b11;       /* 1:64 → 640 ns/tick */
    CCP4CON1Lbits.MOD    = 0b0000;     /* Time Base, free-running */

    CCP4PRL = 0xFFFF;
    _CCT4IE = 0; _CCT4IF = 0;
    _CCP4IE = 0; _CCP4IF = 0;
    CCP4CON1Lbits.CCPON = 1;           /* Start counting immediately */

    /* ── SCCP3: periodic sector timer ─────────────────────────────── */
    CCP3CON1L = 0;
    CCP3CON1H = 0;
    CCP3CON2L = 0;
    CCP3CON2H = 0;
    CCP3CON3H = 0;

    CCP3CON1Lbits.CCSEL  = 0;          /* Timer mode (NOT IC) */
    CCP3CON1Lbits.T32    = 0;          /* 16-bit */
    CCP3CON1Lbits.CLKSEL = 0b000;      /* Fp — same as SCCP4 */
    CCP3CON1Lbits.TMRPS  = 0b11;       /* 1:64 — 640 ns/tick */
    CCP3CON1Lbits.MOD    = 0b0000;     /* Time Base — auto-reload on PRL match */

    CCP3PRL = 0xFFFF;                   /* Won't fire until SetPeriod() */
    CCP3TMRL = 0;

    _CCT3IP = V4_SECTOR_ISR_PRIORITY;
    _CCT3IF = 0;
    _CCT3IE = 0;                        /* Enabled in Start() */

    _CCP3IP = 1;
    _CCP3IF = 0;
    _CCP3IE = 0;

    CCP3CON1Lbits.CCPON = 0;           /* Off until motor start */
}

void HAL_SectorTimer_Start(void)
{
    /* PRL must be set while CCPON=0 on dsPIC33CK SCCP.
     * Read current PRL, disable, re-write, enable. */
    uint16_t prl = CCP3PRL;
    CCP3CON1Lbits.CCPON = 0;
    CCP3PRL = prl;
    CCP3TMRL = 0;
    _CCT3IF = 0;
    _CCT3IE = 1;
    CCP3CON1Lbits.CCPON = 1;
}

void HAL_SectorTimer_Stop(void)
{
    _CCT3IE = 0;
    CCP3CON1Lbits.CCPON = 0;
    _CCT3IF = 0;
}

#endif /* FEATURE_V4_SECTOR_PI */
