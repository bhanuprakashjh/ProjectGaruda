/**
 * @file hal_adc.c
 * @brief ADC HAL for dsPIC33AK128MC106 on EV92R69A + EV68M17A (AK port).
 *
 * Rewritten from CK source. dsPIC33AK ADC organisation is per-channel
 * (AD1CHnCONbits with PINSEL, SAMC, LEFT, DIFF, TRG1SRC, MODE), not the
 * shared-core + dedicated-core arrangement on dsPIC33CK.  Reference:
 * `../../dspic33AKESC/hal/hal_adc.c` (AK FOC sibling — same MCU).
 *
 * 6-step needs:
 *   POT  on AD1CH1 (PINSEL=10, AN10/RA11) — throttle reference
 *   VBUS on AD1CH4 (PINSEL=6,  AN6/RA7)   — DC bus monitor + UV/OV fault
 *
 * BEMF zero-crossing is read as a GPIO from the ATA6847L digital
 * comparator outputs — NOT analog ADC sampling.  So the ADC HAL only
 * needs POT + VBUS channels for the 6-step milestone.
 *
 * Trigger: PG1TRIGA fires the ADC conversion at PWM midpoint (TRG1SRC=4).
 * ISR fires on AD1CH4 (VBUS) completion — last in the trigger group.
 * The ISR body in `garuda_service.c` reads POT, VBUS and the BEMF GPIO.
 *
 * [AK PORT] TRG1SRC=4 selects the AK PWM1 trigger source — verify
 * against DS70005539 Table 27-4 (ADC Trigger Source Selection) once
 * the datasheet table is at hand.
 */

#include <xc.h>
#include <stdint.h>
#include "hal_adc.h"
#include "../garuda_config.h"

void HAL_ADC_Init(void)
{
    /* ── POT on AD1CH1 — RA11 / AD1AN10 ────────────────────────── */
    AD1CH1CONbits.PINSEL  = 10;
    AD1CH1CONbits.SAMC    = 5;       /* Increased for pot divider Z */
    AD1CH1CONbits.LEFT    = 0;
    AD1CH1CONbits.DIFF    = 0;
    AD1CH1CONbits.TRG1SRC = 4;       /* PWM1 trigger (PG1TRIGA, midpoint) */
    AD1CH1CONbits.MODE    = 0;       /* Single sample per trigger */
    AD1CH1CONbits.CMPMOD  = 0;       /* Comparator disabled */

    /* ── VBUS on AD1CH4 — RA7 / AD1AN6 ─────────────────────────── */
    AD1CH4CONbits.PINSEL  = 6;
    AD1CH4CONbits.SAMC    = 5;
    AD1CH4CONbits.LEFT    = 0;
    AD1CH4CONbits.DIFF    = 0;
    AD1CH4CONbits.TRG1SRC = 4;       /* PWM1 trigger — same as POT */
    AD1CH4CONbits.MODE    = 0;
    AD1CH4CONbits.CMPMOD  = 0;

    /* Turn on ADC Core 1 */
    AD1CONbits.ON = 1;
    while (AD1CONbits.ADRDY == 0);

    /* ADC ISR: fire on AD1CH4 (VBUS) completion.
     *
     * [AK PORT NOTE] DS70005539 §15 does NOT guarantee that same-trigger
     * channels complete in channel-number order. AD1CH1 and AD1CH4 share
     * the PG1TRIGA trigger; the AD1 core scheduler decides the conversion
     * order internally. We assume CH4 ≥ CH1 completion time because the
     * scheduler typically processes channels in priority/index order,
     * but that's a working assumption — verify on scope at bring-up.
     * Worst case: VBUS data is one PWM cycle stale (16 µs at 60 kHz),
     * which is fine for OV/UV fault checking. POT is always fresh
     * because it's the first conversion.
     *
     * Priority 3 sits below Timer1(4) and CCP(5/6) so capture preempts. */
    _AD1CH4IP = 3;
    _AD1CH4IF = 0;
    _AD1CH4IE = 0;                   /* Enabled at motor start. In IDLE we
                                      * poll AD1CHnDATA directly from the
                                      * main loop — keeps the ISR cost off
                                      * the gate-driver / capture path. */
}

void HAL_ADC_Enable(void)
{
    AD1CONbits.ON = 1;
}

void HAL_ADC_Disable(void)
{
    AD1CONbits.ON = 0;
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
 * PWM triggers aren't running. Call from main loop, NOT from ISR.
 *
 * AK doesn't have CK's `CNVCHSEL`/`CNVRTCH` software-trigger SFRs.
 * Trigger one-shot via TRG1SRC=1 (software trigger) momentarily, or
 * simply read the last-converted data registers.  We use the latter:
 * with the ADC enabled and triggers connected to PG1TRIGA, the data
 * registers hold the most recent sample.  During IDLE the PWM is
 * still running (override low/float, but trigger source unaffected),
 * so the data is fresh.
 */
void HAL_ADC_PollPotVbus(uint16_t *pot, uint16_t *vbus)
{
    *pot  = (uint16_t)AD1CH1DATA;
    *vbus = (uint16_t)AD1CH4DATA;
}
