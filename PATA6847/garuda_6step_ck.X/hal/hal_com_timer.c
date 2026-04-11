/**
 * @file hal_com_timer.c
 * @brief SCCP4 free-running HR timer + SCCP3 one-shot commutation scheduler.
 *
 * Two cooperating SCCP modules:
 *
 *   SCCP4 — Free-running 16-bit timer at 640 ns/tick (wraps every 41.9 ms).
 *           Provides HR timestamps via HAL_ComTimer_ReadTimer() for ZC
 *           interval measurement and predictor state. NEVER fires an ISR.
 *
 *   SCCP3 — One-shot scheduler in Time Base mode (MOD=0000). Each
 *           HAL_ComTimer_ScheduleAbsolute() resets CCP3TMRL to 0 and
 *           sets CCP3PRL = (target - now), so when CCP3TMRL reaches
 *           PRL the timer period match fires _CCT3Interrupt exactly
 *           once (one-shot via CCT3IE clear in the ISR).
 *
 * Why two modules: SCCP4 OC mode (MOD=0001) does NOT raise CCP4IF on
 * hardware compare match on this device — only manual `_CCP4IF=1`
 * works. Empirically tested with OCAEN=0 and OCAEN=1; neither
 * generates the interrupt. The Time Base CCTxIF path is the proven
 * working pattern (SCCP1 fast poll uses it at 100kHz reliably), so
 * SCCP3 in that mode is the cleanest hardware-precise scheduler.
 *
 * Both modules share the same Fp/64 = 1.5625 MHz clock, so their
 * tick units are identical and HR timestamps map 1:1 onto schedule
 * delays.
 */

#include "hal_com_timer.h"

#if FEATURE_IC_ZC

#include <xc.h>

void HAL_ComTimer_Init(void)
{
    /* ── SCCP4: free-running HR timestamp source ─────────────────── */
    CCP4CON1L = 0;
    CCP4CON1H = 0;
    CCP4CON2L = 0;
    CCP4CON2H = 0;
    CCP4CON3H = 0;

    CCP4CON1Lbits.CCSEL  = 0;          /* Timer mode */
    CCP4CON1Lbits.T32    = 0;          /* 16-bit */
    CCP4CON1Lbits.CLKSEL = 0b000;     /* Fp (Fosc/2 = 100 MHz) */
    CCP4CON1Lbits.TMRPS  = 0b11;      /* 1:64 prescaler → 640 ns/tick */
    CCP4CON1Lbits.MOD    = 0b0000;    /* Time Base only — free-running */

    CCP4PRL = 0xFFFF;                  /* Full 16-bit range */

    /* No SCCP4 interrupts — purely a counter source */
    _CCT4IE = 0; _CCT4IF = 0;
    _CCP4IE = 0; _CCP4IF = 0;

    CCP4CON1Lbits.CCPON = 1;           /* Start counting */

    /* ── SCCP3: one-shot scheduler (Time Base, fires CCT3IF) ─────── */
    CCP3CON1L = 0;
    CCP3CON1H = 0;
    CCP3CON2L = 0;
    CCP3CON2H = 0;
    CCP3CON3H = 0;

    CCP3CON1Lbits.CCSEL  = 0;          /* Timer mode (NOT IC) */
    CCP3CON1Lbits.T32    = 0;          /* 16-bit */
    CCP3CON1Lbits.CLKSEL = 0b000;     /* Fp — same source as SCCP4 */
    CCP3CON1Lbits.TMRPS  = 0b11;      /* 1:64 — same 640 ns/tick as SCCP4 */
    CCP3CON1Lbits.MOD    = 0b0000;    /* Time Base — period match → CCT3IF */

    CCP3PRL = 0xFFFF;                  /* Free-running — won't match until updated */

    /* Period interrupt (CCT3IF) — fires on TMRL == PRL match */
    _CCT3IP = COM_TIMER_ISR_PRIORITY;
    _CCT3IF = 0;
    _CCT3IE = 0;                       /* Enabled per-schedule */

    /* Capture/compare interrupt unused */
    _CCP3IP = 1;
    _CCP3IF = 0;
    _CCP3IE = 0;

    /* Always-on free-running (PRL=0xFFFF) — no CCPON cycling per
     * schedule. Each ScheduleAbsolute updates CCP3PRL relative to
     * current CCP3TMRL; CCT3IF fires on the new period match. This
     * eliminates the ~500ns CCPON enable overhead that was burning
     * the schedule margin at 80k+ eRPM. */
    CCP3CON1Lbits.CCPON = 1;
}

void HAL_ComTimer_ScheduleAbsolute(uint16_t targetTick)
{
    /* Disarm previous schedule */
    _CCT3IE = 0;

    /* Compute schedule margin in SCCP4 domain (signed). Negative or
     * tiny means the target is already in the past or too close to
     * fire via the hardware compare path. */
    uint16_t now4 = CCP4TMRL;
    int16_t margin = (int16_t)(targetTick - now4);

    if (margin < 4)
    {
        /* ASAP path: bypass the CCP3 hardware count entirely.
         * Setting CCT3IF manually fires _CCT3Interrupt on the next
         * instruction (IPL 6 preempts the calling FastPoll IPL 5).
         * Total latency: ~50 ns vs ~1.5 µs for the count-from-zero
         * path. Critical at 80k+ eRPM where the schedule margin
         * routinely goes negative due to detector latency.
         *
         * PRL stays at 0xFFFF so the free-running timer doesn't
         * generate a stray match in the next tick. */
        CCP3PRL = 0xFFFF;
        _CCT3IF = 1;
        _CCT3IE = 1;
        return;
    }

    /* Hardware compare path: SCCP3 is free-running. Compute the
     * SCCP3 PRL value that corresponds to firing `margin` ticks
     * from now. SCCP3 and SCCP4 share the same Fp/64 clock so
     * deltas are 1:1 between domains. */
    uint16_t now3   = CCP3TMRL;
    uint16_t target3 = now3 + (uint16_t)margin;
    CCP3PRL = target3;
    _CCT3IF = 0;

    /* Race check: between reading now3 and writing PRL, the timer
     * may have advanced past the new target. If so, fire ASAP. */
    if ((int16_t)(target3 - CCP3TMRL) <= 0)
    {
        CCP3PRL = 0xFFFF;
        _CCT3IF = 1;
    }
    _CCT3IE = 1;
}

void HAL_ComTimer_Cancel(void)
{
    _CCT3IE = 0;
    _CCT3IF = 0;
    CCP3PRL = 0xFFFF;   /* Push next match ~42 ms out — effectively idle */
}

#endif /* FEATURE_IC_ZC */
