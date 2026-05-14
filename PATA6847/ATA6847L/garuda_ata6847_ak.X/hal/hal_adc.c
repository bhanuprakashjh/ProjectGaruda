/**
 * @file hal_adc.c
 * @brief ADC HAL for dsPIC33AK128MC106 on EV92R69A + EV68M17A (AK port).
 *
 * Channels:
 *   POT   on AD1CH1 (PINSEL=10, AN10/RA11)            — throttle
 *   VBUS  on AD1CH4 (PINSEL=6,  AN6/RA7)              — DC bus monitor
 *   Ia    on AD1CH0 (PINSEL=0,  OA1OUT/AN0/RA2)       — Phase A current
 *   Ibus  on AD1CH2 (PINSEL=3,  OA3OUT/AN3/RA5)       — DC bus current
 *   Ib    on AD2CH0 (PINSEL=1,  OA2OUT/AN1/RB0)       — Phase B current
 *
 * The three current channels are driven by the dsPIC's INTERNAL op-amps
 * (AMP1/AMP2/AMP3) which amplify the raw shunt voltages routed in over
 * DIM pins 13/15, 21/23 and 29/31 (per AN6285 §3 and the AK DIM info
 * sheet DS70005527 §2 + Table 3-1).
 *
 * Why dsPIC op-amps and not the ATA6847L's internal CSAs?
 *  - 6-step BEMF detection needs J5/J7/J9 in BEMF position on the
 *    EV92R69A board, which disables the ATA6847L's first op-amp output
 *    (shared with BEMF pins).  Using the dsPIC's op-amps keeps BEMF
 *    available on the ATA6847L side.
 *  - Default DIM jumpers (R6/R25/R11 populated, R17/R30/R9 not) wire
 *    raw shunts into the dsPIC's op-amp inputs.  No DIM rework needed.
 *
 * Trigger:  PG1TRIGA fires all five channels at the PWM trigger position
 * (currently mid-OFF / period boundary per the recent CK-parity change).
 * ISR fires on AD1CH4 (VBUS) completion — last in AD1's trigger group.
 * AD2CH0 (Ib) runs on the separate AD2 core in parallel and is finished
 * well before AD1CH4 by the time the ISR enters.
 *
 * Reference: dspic33AKESC RK1 hal_adc.c + port_config.c (same MCU,
 * different motherboard).
 */

#include <xc.h>
#include <stdint.h>
#include "hal_adc.h"
#include "../garuda_config.h"

/**
 * @brief Power up the three internal op-amps that amplify the shunt
 * signals routed in from the EV92R69A board.  Must be called once,
 * before HAL_ADC_Init() arms the ADC channels — op-amp outputs need
 * ~10 µs to settle and the first conversion after enabling will be
 * dirty otherwise.
 *
 * Settings mirror dspic33AKESC RK1 (port_config.c HAL_OA3_Init):
 *   HPEN=1     high-power / high-bandwidth (we sample at 60 kHz with
 *              short SAMC, op-amp needs the BW)
 *   UGE=0      external resistor gain (DIM has 24.95× gain network)
 *   DIFFCON=0  both differential pairs active
 *   OMONEN=1   output-monitor enabled (lets ADC sample the op-amp output
 *              internally)
 *   AMPEN=1    enable amplifier core
 */
void HAL_OpAmp_Init(void)
{
    /* OA1 — amplifies Phase A shunt (DIM 13/15 → AN0/RA2) */
    AMP1CON1 = 0x0000;
    AMP1CON1bits.HPEN    = 1;
    AMP1CON1bits.DIFFCON = 0;
    AMP1CON1bits.OMONEN  = 1;
    AMP1CON1bits.AMPEN   = 1;

    /* OA2 — amplifies Phase B shunt (DIM 21/23 → AN1/RB0) */
    AMP2CON1 = 0x0000;
    AMP2CON1bits.HPEN    = 1;
    AMP2CON1bits.DIFFCON = 0;
    AMP2CON1bits.OMONEN  = 1;
    AMP2CON1bits.AMPEN   = 1;

    /* OA3 — amplifies DC-bus shunt (DIM 29/31 → AN3/RA5) */
    AMP3CON1 = 0x0000;
    AMP3CON1bits.HPEN    = 1;
    AMP3CON1bits.DIFFCON = 0;
    AMP3CON1bits.OMONEN  = 1;
    AMP3CON1bits.AMPEN   = 1;

    /* Settle (~10 µs per datasheet).  200 NOPs at FCY=200 MHz ≈ 1 µs;
     * 4000 NOPs gives ~20 µs comfortable margin. */
    for (uint16_t i = 0; i < 4000U; i++) { __builtin_nop(); }
}

void HAL_ADC_Init(void)
{
    /* Op-amps must be live before the ADC samples their outputs. */
    HAL_OpAmp_Init();

    /* ── Ia on AD1CH0 — OA1OUT = RA2 = AD1AN0 ──────────────────── */
    /* SAMC=1 (was 3): op-amp output drives this pin via OMONEN, so source
     * impedance is the op-amp itself (~tens of Ω) — well under what a
     * single-cycle S/H window can settle.  Each unit of SAMC at AK's ADC
     * clock is ~100 ns; trimming to 1 saves ~200 ns per channel × 3 current
     * channels = ~600 ns earlier ADC-ISR fire, claws back the BEMF read
     * timing that the extra channels pushed late. */
    AD1CH0CONbits.PINSEL  = 0;
    AD1CH0CONbits.SAMC    = 1;
    AD1CH0CONbits.LEFT    = 0;
    AD1CH0CONbits.DIFF    = 0;
    AD1CH0CONbits.TRG1SRC = 4;       /* PWM1 trigger — same as VBUS */
    AD1CH0CONbits.MODE    = 0;
    AD1CH0CONbits.CMPMOD  = 0;

    /* ── POT on AD1CH1 — RA11 / AD1AN10 ────────────────────────── */
    AD1CH1CONbits.PINSEL  = 10;
    AD1CH1CONbits.SAMC    = 5;       /* high-Z pot divider needs longer S/H */
    AD1CH1CONbits.LEFT    = 0;
    AD1CH1CONbits.DIFF    = 0;
    AD1CH1CONbits.TRG1SRC = 4;
    AD1CH1CONbits.MODE    = 0;
    AD1CH1CONbits.CMPMOD  = 0;

    /* ── Ibus on AD1CH2 — OA3OUT = RA5 = AD1AN3 ────────────────── */
    AD1CH2CONbits.PINSEL  = 3;
    AD1CH2CONbits.SAMC    = 1;       /* see SAMC note on AD1CH0 above */
    AD1CH2CONbits.LEFT    = 0;
    AD1CH2CONbits.DIFF    = 0;
    AD1CH2CONbits.TRG1SRC = 4;
    AD1CH2CONbits.MODE    = 0;
    AD1CH2CONbits.CMPMOD  = 0;

    /* ── VBUS on AD1CH4 — RA7 / AD1AN6 ─────────────────────────── */
    AD1CH4CONbits.PINSEL  = 6;
    AD1CH4CONbits.SAMC    = 5;
    AD1CH4CONbits.LEFT    = 0;
    AD1CH4CONbits.DIFF    = 0;
    AD1CH4CONbits.TRG1SRC = 4;
    AD1CH4CONbits.MODE    = 0;
    AD1CH4CONbits.CMPMOD  = 0;

    /* ── Ib on AD2CH0 — OA2OUT = RB0 = AD2AN1 ──────────────────── */
    /* AD2CH0 is the only channel on AD2 core so its SAMC doesn't affect
     * ISR latency — still trimmed to 1 for consistency. */
    AD2CH0CONbits.PINSEL  = 1;
    AD2CH0CONbits.SAMC    = 1;
    AD2CH0CONbits.LEFT    = 0;
    AD2CH0CONbits.DIFF    = 0;
    AD2CH0CONbits.TRG1SRC = 4;
    AD2CH0CONbits.MODE    = 0;
    AD2CH0CONbits.CMPMOD  = 0;

    /* Turn on both ADC cores. */
    AD1CONbits.ON = 1;
    while (AD1CONbits.ADRDY == 0);
    AD2CONbits.ON = 1;
    while (AD2CONbits.ADRDY == 0);

    /* ADC ISR fires on AD1CH4 (VBUS) completion — last AD1 channel in
     * the trigger group.  By the time we enter the ISR, AD1CH0/CH1/CH2
     * are all done on AD1, and AD2CH0 (single-channel on AD2) finished
     * long before.  Priority 3 sits below Timer1(4)/CCP(5,6) so capture
     * preempts. */
    _AD1CH4IP = 3;
    _AD1CH4IF = 0;
    _AD1CH4IE = 0;                   /* enabled at motor start */
}

void HAL_ADC_Enable(void)
{
    AD1CONbits.ON = 1;
    AD2CONbits.ON = 1;
}

void HAL_ADC_Disable(void)
{
    AD1CONbits.ON = 0;
    AD2CONbits.ON = 0;
}

void HAL_ADC_InterruptEnable(void)
{
    _AD1CH4IF = 0;
    _AD1CH4IE = 1;
}

void HAL_ADC_InterruptDisable(void)
{
    _AD1CH4IE = 0;
}

/**
 * @brief Software-triggered read of pot + Vbus.  Used during IDLE when
 * PWM triggers aren't running.  Returns whatever the data registers
 * currently hold (last conversion result).
 */
void HAL_ADC_PollPotVbus(uint16_t *pot, uint16_t *vbus)
{
    *pot  = (uint16_t)AD1CH1DATA;
    *vbus = (uint16_t)AD1CH4DATA;
}
