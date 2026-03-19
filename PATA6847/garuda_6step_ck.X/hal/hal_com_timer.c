/**
 * @file hal_com_timer.c
 * @brief SCCP4 free-running timer + output compare for commutation.
 *
 * Dual-purpose SCCP4:
 *   1. Free-running 16-bit timer at 640 ns/tick (wraps every 41.9 ms).
 *      IC ISRs call HAL_ComTimer_ReadTimer() for HR ZC timestamps.
 *   2. Output compare: after ZC, ScheduleAbsolute(target) sets CCP4RA.
 *      When CCP4TMRL matches CCP4RA, CCP4IF fires → commutation ISR.
 *
 * OC mode (MOD=0001): compare match sets CCP4IF (capture/compare interrupt),
 * NOT CCT4IF (timer period interrupt). Timer does NOT reset on match —
 * it continues counting through PRL=0xFFFF, acting as free-running.
 *
 * One-shot behavior: ISR disables CCP4IE after firing, preventing
 * re-fire on the next period wrap.
 */

#include "hal_com_timer.h"

#if FEATURE_IC_ZC

#include <xc.h>

void HAL_ComTimer_Init(void)
{
    /* Clear all control registers */
    CCP4CON1L = 0;
    CCP4CON1H = 0;
    CCP4CON2L = 0;
    CCP4CON2H = 0;
    CCP4CON3H = 0;

    CCP4CON1Lbits.CCSEL  = 0;          /* Timer/OC mode (not IC) */
    CCP4CON1Lbits.T32    = 0;          /* 16-bit timer */
    CCP4CON1Lbits.CLKSEL = 0b000;     /* Fp (Fosc/2 = 100 MHz) */
    CCP4CON1Lbits.TMRPS  = 0b11;      /* 1:64 prescaler -> 640 ns/tick */
    CCP4CON1Lbits.MOD    = 0b0001;    /* Single edge OC — compare match fires CCP4IF */

    /* No output pin needed — we only use the compare interrupt */
    CCP4CON2Hbits.OCAEN  = 0;

    CCP4PRL = 0xFFFF;                  /* Full 16-bit range (free-running) */
    CCP4RA  = 0xFFFF;                  /* Default compare value (won't match soon) */

    /* Disable timer period interrupt — not needed in OC mode */
    _CCT4IE = 0;
    _CCT4IF = 0;

    /* Configure compare match interrupt (CCP4, NOT CCT4).
     * In OC mode, CCP4TMRL == CCP4RA sets CCP4IF. */
    _CCP4IP = COM_TIMER_ISR_PRIORITY;
    _CCP4IF = 0;
    _CCP4IE = 0;                       /* Enabled only when scheduled */

    /* Start timer — runs continuously as free-running timebase */
    CCP4CON1Lbits.CCPON = 1;
}

void HAL_ComTimer_ScheduleAbsolute(uint16_t targetTick)
{
    /* Set compare match target */
    CCP4RA = targetTick;

    /* Clear any stale interrupt flag, then enable */
    _CCP4IF = 0;
    _CCP4IE = 1;
}

void HAL_ComTimer_Cancel(void)
{
    /* Disable compare interrupt — timer keeps running */
    _CCP4IE = 0;
    _CCP4IF = 0;
}

#endif /* FEATURE_IC_ZC */
