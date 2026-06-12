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
/* FEATURE_IF_STARTUP (M1 bring-up): configure the ADC EXACTLY like the FOC
 * build — CH0s on the OA1/OA2 current op-amps, set BEFORE ADC ON (the runtime
 * PINSEL repurpose at IF entry read OA2 as railed garbage; channel config with
 * ADON=1 is unreliable per datasheet). The 6-step BEMF machinery runs inert
 * against these channels during the I-f ramp; M2 will do the proper handoff. */
#if FEATURE_FOC || FEATURE_FOC_V2 || FEATURE_FOC_V3 || FEATURE_FOC_AN1078 || FEATURE_IF_STARTUP
#if GARUDA_TARGET_AK512
    /* Ia on AD1CH0: OA1OUT = RA2 = AD1AN0, PINSEL=0 (AN957) */
    AD1CH0CON1bits.PINSEL = 0;
    AD1CH0CON1bits.SAMC = 3;       /* Low Z from OA1 output */
    AD1CH0CON1bits.FRAC = 0;       /* right-aligned (LEFT equivalent) */
    AD1CH0CON1bits.DIFF = 0;

    /* Ib on AD2CH0: OA2OUT = RB0 = AD2AN0, PINSEL=0 (AN957; the 106 had
     * OA2OUT on AD2AN1 — MC510 bonds it to AD2AN0) */
    AD2CH0CON1bits.PINSEL = 0;
    AD2CH0CON1bits.SAMC = 3;       /* Low Z from OA2 output */
    AD2CH0CON1bits.FRAC = 0;       /* right-aligned (LEFT equivalent) */
    AD2CH0CON1bits.DIFF = 0;
#else
    /* Ia on AD1CH0: OA1OUT = RA2 = AD1AN0, PINSEL=0 */
    AD1CH0CONbits.PINSEL = 0;
    AD1CH0CONbits.SAMC = 3;        /* Low Z from OA1 output */
    AD1CH0CONbits.LEFT = 0;
    AD1CH0CONbits.DIFF = 0;

    /* Ib on AD2CH0: OA2OUT = RB0 = AD2AN1, PINSEL=1 */
    AD2CH0CONbits.PINSEL = 1;
    AD2CH0CONbits.SAMC = 3;        /* Low Z from OA2 output */
    AD2CH0CONbits.LEFT = 0;
    AD2CH0CONbits.DIFF = 0;
#endif /* GARUDA_TARGET_AK512 */
#else
#if GARUDA_TARGET_AK512
    /* MC510 6-step: every BEMF phase owns a dedicated PWM-synced channel
     * (PG1TRIGA sampling instant — same trigger event as the 106's CH0s).
     * Phase B on AD1CH3: RA10 = AD1AN4 — interrupt source, always sampled */
    AD1CH3CON1bits.PINSEL = 4;
    AD1CH3CON1bits.SAMC = 5;       /* Increased for divider impedance */
    AD1CH3CON1bits.FRAC = 0;       /* right-aligned (LEFT equivalent) */
    AD1CH3CON1bits.DIFF = 0;

    /* Phase A on AD1CH4: RA9 = AD1AN3 (dedicated — no A/C mux on MC510) */
    AD1CH4CON1bits.PINSEL = 3;
    AD1CH4CON1bits.SAMC = 5;       /* Increased for divider impedance */
    AD1CH4CON1bits.FRAC = 0;       /* right-aligned (LEFT equivalent) */
    AD1CH4CON1bits.DIFF = 0;

    /* Phase C on AD2CH3: RB8 = AD2AN4 (dedicated) */
    AD2CH3CON1bits.PINSEL = 4;
    AD2CH3CON1bits.SAMC = 5;       /* Increased for divider impedance */
    AD2CH3CON1bits.FRAC = 0;       /* right-aligned (LEFT equivalent) */
    AD2CH3CON1bits.DIFF = 0;
#else
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
#endif /* GARUDA_TARGET_AK512 */
#endif

#if GARUDA_TARGET_AK512
    /* POT on AD2CH1: RB15 = AD2AN5, PINSEL=5 (AN957) */
    AD2CH1CON1bits.PINSEL = 5;
    AD2CH1CON1bits.SAMC = 5;
    AD2CH1CON1bits.FRAC = 0;       /* right-aligned (LEFT equivalent) */
    AD2CH1CON1bits.DIFF = 0;

    /* VBUS on AD3CH2: RF0 = AD3AN4, PINSEL=4 (AN957) */
    AD3CH2CON1bits.PINSEL = 4;
    AD3CH2CON1bits.SAMC = 5;
    AD3CH2CON1bits.FRAC = 0;       /* right-aligned (LEFT equivalent) */
    AD3CH2CON1bits.DIFF = 0;
#else
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
#endif /* GARUDA_TARGET_AK512 */

#if FEATURE_HW_OVERCURRENT
#if GARUDA_TARGET_AK512
    /* Bus current on AD3CH1: RA5 = OA3OUT = AD3AN0, PINSEL=0 (AN957) */
    AD3CH1CON1bits.PINSEL = 0;
    AD3CH1CON1bits.SAMC = 3;       /* Fast sample (low Z from OA3 output) */
    AD3CH1CON1bits.FRAC = 0;       /* right-aligned (LEFT equivalent) */
    AD3CH1CON1bits.DIFF = 0;
    AD3CH1CON1bits.TRG1SRC = 4;    /* PWM1 trigger (24kHz, midpoint sampling) */
#else
    /* Bus current on AD1CH2: RA5 = OA3OUT = AD1AN3, PINSEL=3 */
    AD1CH2CONbits.PINSEL = 3;
    AD1CH2CONbits.SAMC = 3;        /* Fast sample (low Z from OA3 output) */
    AD1CH2CONbits.LEFT = 0;
    AD1CH2CONbits.DIFF = 0;
    AD1CH2CONbits.TRG1SRC = 4;     /* PWM1 trigger (24kHz, midpoint sampling) */
#endif /* GARUDA_TARGET_AK512 */
#endif

#if FEATURE_ADC_CMP_ZC && !FEATURE_IF_STARTUP
    /* High-speed BEMF channels configured BEFORE ADC ON.
     * Datasheet warns configuring channels when ADON=1 is unpredictable
     * for non-PWM trigger sources. TRG1SRC=14 (SCCP3 Trigger out).
     * Comparator disabled initially (CMPMOD=0). */

    /* AD1CH5/AD2CH1 sample at 1 MHz via SCCP3 (free-running). Phantom-ZC
     * rejection is done in SOFTWARE inside HWZC_OnZcDetected(): any ZC
     * interrupt that fires while the active phase's PWMxH output is LOW
     * (i.e. during OFF-time) is treated as switching noise and dropped.
     * This keeps worst-case detection latency at ~1 µs while filtering
     * out the 76 %-of-cycle OFF-time samples at high duty / 24 V where
     * the floating-phase voltage is dominated by freewheel-diode
     * conduction and switching ringing, not clean BEMF. */
#if GARUDA_TARGET_AK512
    /* MC510: one dedicated high-speed channel PER PHASE (no A/C PINSEL mux).
     * VA = AD1CH1, VB = AD1CH2, VC = AD2CH2 (AN957 channel map). Same
     * trigger event source as the 106's AD1CH5/AD2CH1: SCCP3 Trigger out —
     * MC510 TRG1SRC encoding is 34 (0b100010, DS70005591 Table 17-4; the
     * 106 used 14). All three convert continuously; the 40 MSPS MC510 core
     * schedules two 1 MHz 4×-burst channels on AD1 with ample headroom
     * (the 106's "second 1 MHz channel starves" limit does not apply). */
    AD1CH1CON1bits.PINSEL = 3;   /* VA: RA9 = AD1AN3 */
    AD1CH1CON1bits.SAMC = HWZC_SAMC;
    AD1CH1CON1bits.FRAC = 0;     /* right-aligned (LEFT equivalent) */
    AD1CH1CON1bits.DIFF = 0;
    AD1CH1CON1bits.TRG1SRC = 0;  /* gated OFF at init — SelectBemfPhase
                                  * triggers ONLY the active phase (two
                                  * simultaneous 1MHz bursts on one core
                                  * starve the PWM-triggered channels —
                                  * bench-proven on bring-up day 1) */ /* SCCP3 Trigger out (1 MHz free-running) */
    AD1CH1CON1bits.TRG2SRC = 0;  /* AK512 capture-rate exp: no retrigger */  /* Immediate re-trigger for oversampling repeats */
    AD1CH1CON1bits.MODE = 0;     /* AK512 capture-rate exp: single sample (was 4x oversample burst) */  /* Oversampling mode */
    AD1CH1CON1bits.ACCNUM = 0b00; /* 4 samples, result right-shifted by 2 bits */
    AD1CH1CON2bits.ACCBRST = 0;  /* Non-interruptible burst (prevent 24kHz split) */
    AD1CH1CON2bits.CMPMOD = 0;   /* Comparator disabled initially */
    AD1CH1CMPLO = 0;
    AD1CH1CMPHI = 0;

    AD1CH2CON1bits.PINSEL = 4;   /* VB: RA10 = AD1AN4 */
    AD1CH2CON1bits.SAMC = HWZC_SAMC;
    AD1CH2CON1bits.FRAC = 0;
    AD1CH2CON1bits.DIFF = 0;
    AD1CH2CON1bits.TRG1SRC = 0;  /* gated OFF at init — SelectBemfPhase
                                  * triggers ONLY the active phase (two
                                  * simultaneous 1MHz bursts on one core
                                  * starve the PWM-triggered channels —
                                  * bench-proven on bring-up day 1) */
    AD1CH2CON1bits.TRG2SRC = 0;  /* AK512 capture-rate exp: no retrigger */
    AD1CH2CON1bits.MODE = 0;     /* AK512 capture-rate exp: single sample (was 4x oversample burst) */
    AD1CH2CON1bits.ACCNUM = 0b00;
    AD1CH2CON2bits.ACCBRST = 0;
    AD1CH2CON2bits.CMPMOD = 0;
    AD1CH2CMPLO = 0;
    AD1CH2CMPHI = 0;

    AD2CH2CON1bits.PINSEL = 4;   /* VC: RB8 = AD2AN4 */
    AD2CH2CON1bits.SAMC = HWZC_SAMC;
    AD2CH2CON1bits.FRAC = 0;
    AD2CH2CON1bits.DIFF = 0;
    AD2CH2CON1bits.TRG1SRC = 0;  /* gated OFF at init — SelectBemfPhase
                                  * triggers ONLY the active phase (two
                                  * simultaneous 1MHz bursts on one core
                                  * starve the PWM-triggered channels —
                                  * bench-proven on bring-up day 1) */
    AD2CH2CON1bits.TRG2SRC = 0;  /* AK512 capture-rate exp: no retrigger */
    AD2CH2CON1bits.MODE = 0;     /* AK512 capture-rate exp: single sample (was 4x oversample burst) */
    AD2CH2CON1bits.ACCNUM = 0b00;
    AD2CH2CON2bits.ACCBRST = 0;
    AD2CH2CON2bits.CMPMOD = 0;
    AD2CH2CMPLO = 0;
    AD2CH2CMPHI = 0;
#else
    AD1CH5CONbits.PINSEL = 11;
    AD1CH5CONbits.SAMC = HWZC_SAMC;
    AD1CH5CONbits.LEFT = 0;
    AD1CH5CONbits.DIFF = 0;
    AD1CH5CONbits.TRG1SRC = 14;  /* SCCP3 Trigger out (1 MHz free-running) */
    AD1CH5CONbits.TRG2SRC = 2;   /* Immediate re-trigger for oversampling repeats */
    AD1CH5CONbits.MODE = 0b11;   /* Oversampling mode */
    AD1CH5CONbits.ACCNUM = 0b00; /* 4 samples, result right-shifted by 2 bits */
    AD1CH5CONbits.ACCBRST = 1;   /* Non-interruptible burst (prevent 24kHz split) */
    AD1CH5CONbits.CMPMOD = 0;    /* Comparator disabled initially */
    AD1CH5CMPLO = 0;
    AD1CH5CMPHI = 0;

    AD2CH1CONbits.PINSEL = 10;
    AD2CH1CONbits.SAMC = HWZC_SAMC;
    AD2CH1CONbits.LEFT = 0;
    AD2CH1CONbits.DIFF = 0;
    AD2CH1CONbits.TRG1SRC = 14;  /* SCCP3 Trigger out (1 MHz free-running) */
    AD2CH1CONbits.TRG2SRC = 2;
    AD2CH1CONbits.MODE = 0b11;
    AD2CH1CONbits.ACCNUM = 0b00;
    AD2CH1CONbits.ACCBRST = 1;
    AD2CH1CONbits.CMPMOD = 0;
    AD2CH1CMPLO = 0;
    AD2CH1CMPHI = 0;
#endif /* GARUDA_TARGET_AK512 */
#endif

#if !FEATURE_FOC && !FEATURE_FOC_V2 && !FEATURE_FOC_V3 && !FEATURE_FOC_AN1078 && !FEATURE_IF_STARTUP
    /* 6-step Phase-current monitor channels — diagnostic peak tracking.
     *
     * AD1CH3 samples OA1OUT (phase A low-side shunt amp) at 1 MHz via
     * SCCP3. AD2CH2 samples OA2OUT (phase B shunt amp). Both op-amp pins
     * (RA2 = AD1AN0, RB0 = AD2AN1) are otherwise unused in 6-step mode
     * (those ADC channels get repurposed for BEMF voltage sensing at CH0).
     *
     * The 24 kHz ADC ISR reads AD1CH3DATA / AD2CH2DATA every tick and
     * tracks running max/min — the hardware is always converting at
     * 1 MHz, so the data register holds the most recent 1 µs-window
     * sample. We can't see 100 ns Qrr spikes at 42 µs polling cadence,
     * but sustained ON-time currents and multi-µs commutation transients
     * will register. Purpose: empirically confirm whether peak phase
     * current crosses the U25B 22 A trip threshold at high eRPM.
     *
     * Shunt = 3 mΩ, op-amp gain = 24.95 default, VREF = 1.65 V bias.
     * Scale: ~93 ADC counts / amp, bias ~2048. 22 A → ~4094 / 2.
     */
    /* Trigger on PG1TRIGA (24 kHz, mid-ON valley) instead of SCCP3 (1 MHz).
     * Reason: AD2CH1 (HWZC Phase A/C high-speed) runs 4-sample oversample
     * bursts at 1 MHz which fully occupy the AD2 core. A second 1 MHz
     * channel on AD2 never gets scheduled — Ib read-out starves to 0. At
     * 24 kHz PG1TRIGA we sample once per PWM cycle, interleaved with the
     * HWZC burst, and both conversions complete. Trade-off: we only see
     * the mid-ON current, not 100 ns Qrr spikes during the commutation
     * dead-time — but that's sufficient to validate the sustained-ON
     * operating current (the "real" ~20 A we've inferred from 50 Hz
     * telemetry). */
#if GARUDA_TARGET_AK512
    /* MC510: the monitor channels are the AN957 IA/IB channels themselves
     * (AD1CH0/AD2CH0, PG1TRIGA-triggered) — same sampling instant as the
     * 106's AD1CH3/AD2CH2 monitors. AD2CH2 is VC high-speed here. */
    AD1CH0CON1bits.PINSEL = 0;      /* OA1OUT = RA2 = AD1AN0 (Ia) */
    AD1CH0CON1bits.SAMC = 3;
    AD1CH0CON1bits.FRAC = 0;        /* right-aligned (LEFT equivalent) */
    AD1CH0CON1bits.DIFF = 0;
    AD1CH0CON1bits.TRG1SRC = 4;     /* PG1TRIGA (24 kHz mid-ON valley) */
    AD1CH0CON1bits.MODE = 0;        /* Single sample, one per trigger */
    AD1CH0CON2bits.CMPMOD = 0;

    AD2CH0CON1bits.PINSEL = 0;      /* OA2OUT = RB0 = AD2AN0 (Ib) */
    AD2CH0CON1bits.SAMC = 3;
    AD2CH0CON1bits.FRAC = 0;        /* right-aligned (LEFT equivalent) */
    AD2CH0CON1bits.DIFF = 0;
    AD2CH0CON1bits.TRG1SRC = 4;     /* PG1TRIGA (24 kHz mid-ON valley) */
    AD2CH0CON1bits.MODE = 0;
    AD2CH0CON2bits.CMPMOD = 0;
#else
    AD1CH3CONbits.PINSEL = 0;       /* OA1OUT = RA2 = AD1AN0 (Ia) */
    AD1CH3CONbits.SAMC = 3;
    AD1CH3CONbits.LEFT = 0;
    AD1CH3CONbits.DIFF = 0;
    AD1CH3CONbits.TRG1SRC = 4;      /* PG1TRIGA (24 kHz mid-ON valley) */
    AD1CH3CONbits.MODE = 0;         /* Single sample, one per trigger */
    AD1CH3CONbits.CMPMOD = 0;

    AD2CH2CONbits.PINSEL = 1;       /* OA2OUT = RB0 = AD2AN1 (Ib) */
    AD2CH2CONbits.SAMC = 3;
    AD2CH2CONbits.LEFT = 0;
    AD2CH2CONbits.DIFF = 0;
    AD2CH2CONbits.TRG1SRC = 4;      /* PG1TRIGA (24 kHz mid-ON valley) */
    AD2CH2CONbits.MODE = 0;
    AD2CH2CONbits.CMPMOD = 0;
#endif /* GARUDA_TARGET_AK512 */
#endif

#if GARUDA_TARGET_AK512
    /* Turn on ADC Core 1 */
    AD1CONbits.ON = 1;
    while (AD1CONbits.ADRDY == 0);

    /* Turn on ADC Core 2 */
    AD2CONbits.ON = 1;
    while (AD2CONbits.ADRDY == 0);

    /* Turn on ADC Core 3 (IBUS + VBUS live on AD3 on the MC510) */
    AD3CONbits.ON = 1;
    while (AD3CONbits.ADRDY == 0);

    /* Control-loop interrupt source: VB PWM-synced channel (AD1CH3) in the
     * 6-step build; IA (AD1CH0) in FOC/I-f builds — same signal as the
     * 106's AD1CH0 in each mode. GARUDA_ADC_IP from hal_adc.h. */
    GARUDA_ADC_IP = 7;     /* Highest priority */
    GARUDA_ClearADCIF();
    GARUDA_DisableADCInterrupt();  /* Disabled until service init */

    /* Trigger sources: PWM1 ADC Trigger 1 (TRG1SRC=4 — same encoding as
     * the 106, confirmed by AN957 adc.c). IA/IB (CH0s) and IBUS already
     * have TRG1SRC=4 from their config blocks above. */
    AD2CH1CON1bits.TRG1SRC = 4;    /* POT from PWM1 trigger */
    AD3CH2CON1bits.TRG1SRC = 4;    /* VBUS from PWM1 trigger */
#if !FEATURE_FOC && !FEATURE_FOC_V2 && !FEATURE_FOC_V3 && !FEATURE_FOC_AN1078 && !FEATURE_IF_STARTUP
    AD1CH3CON1bits.TRG1SRC = 4;    /* Phase B (VB) from PWM1 trigger */
    AD1CH4CON1bits.TRG1SRC = 4;    /* Phase A (VA) from PWM1 trigger */
    AD2CH3CON1bits.TRG1SRC = 4;    /* Phase C (VC) from PWM1 trigger */
#else
    AD1CH0CON1bits.TRG1SRC = 4;    /* IA from PWM1 trigger */
    AD2CH0CON1bits.TRG1SRC = 4;    /* IB from PWM1 trigger */
#endif
#else
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
#endif /* GARUDA_TARGET_AK512 */
}

#if !FEATURE_FOC && !FEATURE_FOC_V2 && !FEATURE_FOC_V3 && !FEATURE_FOC_AN1078

#if GARUDA_TARGET_AK512 && !FEATURE_IF_STARTUP
/* Active floating phase for the PWM-synced A/C read (set per commutation by
 * HAL_ADC_SelectBEMFChannel, consumed by HAL_ADC_ReadPhaseAC in the ISR). */
static volatile uint8_t halBemfAcPhase = FLOATING_PHASE_A;

/**
 * @brief Return the PWM-synced sample of the active A-or-C floating phase.
 * Reads BOTH dedicated channels (clears data-ready on both — required on
 * dsPIC33AK), returns the one selected at the last commutation.
 */
uint16_t HAL_ADC_ReadPhaseAC(void)
{
    uint16_t va = (uint16_t)AD1CH4DATA;
    uint16_t vc = (uint16_t)AD2CH3DATA;
    return (halBemfAcPhase == FLOATING_PHASE_C) ? vc : va;
}
#endif

/**
 * @brief Select which phase voltage to sample on AD2CH0 for the current
 * floating phase. Phase A and C share AD2, so we mux PINSEL.
 * Phase B is on AD1CH0 (always sampled).
 *
 * AK512: every phase has a dedicated PWM-synced channel — PINSEL is never
 * touched; this just latches which channel HAL_ADC_ReadPhaseAC returns.
 * Always returns false (no mux settling, no sample discard needed).
 *
 * @param floatingPhase 0=A (AD2CH0 PINSEL=10), 1=B (AD1CH0, no mux change),
 *                      2=C (AD2CH0 PINSEL=7)
 * @return true if AD2CH0 PINSEL was changed (caller must discard next sample)
 */
bool HAL_ADC_SelectBEMFChannel(uint8_t floatingPhase)
{
#if FEATURE_IF_STARTUP
    /* M1 bring-up: AD2CH0 belongs to the OA2 current op-amp (PINSEL=1, set
     * before ADC ON). Never let the BEMF mux steal it mid-run. */
    return false;
#elif GARUDA_TARGET_AK512
    if (floatingPhase == FLOATING_PHASE_A || floatingPhase == FLOATING_PHASE_C)
        halBemfAcPhase = floatingPhase;
    return false;   /* dedicated channels — nothing to settle */
#else
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
#endif /* GARUDA_TARGET_AK512 */
}
#endif /* !FEATURE_FOC && !FEATURE_FOC_V2 */

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
#if GARUDA_TARGET_AK512
    /* Comparator interrupts (one per dedicated BEMF channel):
     * VA = AD1CH1 → AD1CMP1, VB = AD1CH2 → AD1CMP2, VC = AD2CH2 → AD2CMP2.
     * Clear and leave disabled. */
    _AD1CMP1IF = 0;
    _AD1CMP1IE = 0;
    _AD1CMP2IF = 0;
    _AD1CMP2IE = 0;
    _AD2CMP2IF = 0;
    _AD2CMP2IE = 0;
#else
    /* Comparator interrupts: clear and leave disabled */
    _AD1CMP5IF = 0;
    _AD1CMP5IE = 0;
    _AD2CMP1IF = 0;
    _AD2CMP1IE = 0;
#endif
}

/**
 * @brief Select the high-speed BEMF channel/comparator for the floating
 * phase. Returns the comparator handle hwzc stores in hwzc.activeCore.
 *
 * AK128: VERBATIM move of the selection block from HWZC_OnCommutation —
 * core 1 = AD1CH5 (Phase B, PINSEL fixed at 11), core 2 = AD2CH1 with the
 * Phase A/C PINSEL mux (10 = RB9/VA, 7 = RA10/VC). Identical semantics:
 * caller has already disabled both comparator IEs (Rule 2).
 *
 * AK512: dedicated channel per phase — handle IS the floating phase
 * (0 = VA/AD1CH1, 1 = VB/AD1CH2, 2 = VC/AD2CH2); no PINSEL writes ever.
 */
#if GARUDA_TARGET_AK512
/* Silence all three high-speed BEMF burst channels. MUST be called whenever
 * HWZC shuts down: a leftover TRG1SRC=34 channel keeps 4x-bursting at 1 MHz
 * and permanently starves the PWM-triggered control channel on the same AD
 * core (AD1CH3 = the 45 kHz ISR source) -> board runs exactly once per
 * reset, every later start is silent/inert. Found on bench 2026-06-12. */
void HAL_ADC_BemfBurstOff(void)
{
    AD1CH1CON1bits.TRG1SRC = 0;
    AD1CH2CON1bits.TRG1SRC = 0;
    AD2CH2CON1bits.TRG1SRC = 0;
}
#endif

uint8_t HAL_ADC_SelectBemfPhase(uint8_t floatPhase)
{
#if GARUDA_TARGET_AK512
    uint8_t ph = (floatPhase <= FLOATING_PHASE_C) ? floatPhase : FLOATING_PHASE_B;
    /* Trigger ONLY the active phase's high-speed channel (SCCP3 = 34);
     * silence the other two. Keeps each AD core at <=1 burst channel —
     * same effective load as the 106 — so the PWM-triggered channels
     * (AD1CH3/CH4, AD2CH3) never starve. */
    AD1CH1CON1bits.TRG1SRC = (ph == FLOATING_PHASE_A) ? 34 : 0;
    AD1CH2CON1bits.TRG1SRC = (ph == FLOATING_PHASE_B) ? 34 : 0;
    AD2CH2CON1bits.TRG1SRC = (ph == FLOATING_PHASE_C) ? 34 : 0;
    return ph;
#else
    uint8_t core;
    if (floatPhase == FLOATING_PHASE_B)
    {
        core = 1;  /* AD1CH5, PINSEL fixed at 11 */
    }
    else
    {
        core = 2;  /* AD2CH1 */
        uint8_t pinsel = (floatPhase == FLOATING_PHASE_A) ? 10 : 7;
        HAL_ADC_SetHighSpeedPinsel(pinsel);
    }
    return core;
#endif
}

/**
 * @brief Configure comparator mode and threshold on a high-speed channel.
 * @param adcCore  1=AD1CH5 (Phase B), 2=AD2CH1 (Phase A/C)
 * @param threshold  ADC count threshold for CMPLO register
 * @param risingZc  true=rising ZC (greater-than), false=falling (less-or-equal)
 */
void HAL_ADC_ConfigComparator(uint8_t adcCore, uint16_t threshold, bool risingZc)
{
#if GARUDA_TARGET_AK512
    /* Per-phase channels; same CMPMOD encodings as the 106 (DS70005591
     * Table: 0b011 = greater-than CMPLO → rising ZC, 0b100 = less-or-equal
     * CMPLO → falling ZC; only CMPLO is used). */
    switch (adcCore)
    {
        case FLOATING_PHASE_A:
            AD1CH1CON2bits.CMPMOD = risingZc ? 0b011 : 0b100;
            AD1CH1CMPLO = threshold;
            break;
        case FLOATING_PHASE_B:
            AD1CH2CON2bits.CMPMOD = risingZc ? 0b011 : 0b100;
            AD1CH2CMPLO = threshold;
            break;
        default: /* FLOATING_PHASE_C */
            AD2CH2CON2bits.CMPMOD = risingZc ? 0b011 : 0b100;
            AD2CH2CMPLO = threshold;
            break;
    }
#else
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
#endif /* GARUDA_TARGET_AK512 */
}

/**
 * @brief Live CMPLO refresh — updates threshold without touching CMPMOD.
 * Single SFR write, atomic. Safe to call from the 24 kHz ADC ISR while
 * the comparator is armed; the new value takes effect on the next
 * ADC conversion (comparator re-evaluates at SCCP3 trigger rate).
 *
 * Use this to track Vbus/duty changes mid-sector. HAL_ADC_ConfigComparator
 * is still used per-commutation to set CMPMOD for the new sector polarity.
 *
 * @param adcCore  1=AD1CH5, 2=AD2CH1
 * @param threshold  New CMPLO value (deadband already applied by caller)
 */
void HAL_ADC_UpdateComparatorThreshold(uint8_t adcCore, uint16_t threshold)
{
#if GARUDA_TARGET_AK512
    switch (adcCore)
    {
        case FLOATING_PHASE_A: AD1CH1CMPLO = threshold; break;
        case FLOATING_PHASE_B: AD1CH2CMPLO = threshold; break;
        default:               AD2CH2CMPLO = threshold; break;
    }
#else
    if (adcCore == 1)
        AD1CH5CMPLO = threshold;
    else
        AD2CH1CMPLO = threshold;
#endif /* GARUDA_TARGET_AK512 */
}

/**
 * @brief Enable comparator interrupt on a high-speed channel.
 * Clears flag before enabling to prevent stale triggers.
 * @param adcCore  1=AD1CH5, 2=AD2CH1
 */
void HAL_ADC_EnableComparatorIE(uint8_t adcCore)
{
#if GARUDA_TARGET_AK512
    switch (adcCore)
    {
        case FLOATING_PHASE_A: _AD1CMP1IF = 0; _AD1CMP1IE = 1; break;
        case FLOATING_PHASE_B: _AD1CMP2IF = 0; _AD1CMP2IE = 1; break;
        default:               _AD2CMP2IF = 0; _AD2CMP2IE = 1; break;
    }
#else
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
#endif /* GARUDA_TARGET_AK512 */
}

/**
 * @brief Disable comparator interrupt on a high-speed channel.
 * @param adcCore  1=AD1CH5, 2=AD2CH1
 */
void HAL_ADC_DisableComparatorIE(uint8_t adcCore)
{
#if GARUDA_TARGET_AK512
    switch (adcCore)
    {
        case FLOATING_PHASE_A: _AD1CMP1IE = 0; break;
        case FLOATING_PHASE_B: _AD1CMP2IE = 0; break;
        default:               _AD2CMP2IE = 0; break;
    }
#else
    if (adcCore == 1)
        _AD1CMP5IE = 0;
    else
        _AD2CMP1IE = 0;
#endif /* GARUDA_TARGET_AK512 */
}

/**
 * @brief Clear comparator status flag and interrupt flag.
 * @param adcCore  1=AD1CH5, 2=AD2CH1
 */
void HAL_ADC_ClearComparatorFlag(uint8_t adcCore)
{
#if GARUDA_TARGET_AK512
    /* Same clear semantics as the 106: drop the per-channel CMPSTAT flag
     * first, then the interrupt flag. */
    switch (adcCore)
    {
        case FLOATING_PHASE_A:
            AD1CMPSTATbits.CH1FLG = 0;
            _AD1CMP1IF = 0;
            break;
        case FLOATING_PHASE_B:
            AD1CMPSTATbits.CH2FLG = 0;
            _AD1CMP2IF = 0;
            break;
        default: /* FLOATING_PHASE_C */
            AD2CMPSTATbits.CH2FLG = 0;
            _AD2CMP2IF = 0;
            break;
    }
#else
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
#endif /* GARUDA_TARGET_AK512 */
}

/**
 * @brief Set PINSEL for AD2CH1 high-speed channel (Phase A vs Phase C mux).
 * @param pinsel  10=Phase A (RB9), 7=Phase C (RA10)
 */
void HAL_ADC_SetHighSpeedPinsel(uint8_t pinsel)
{
#if GARUDA_TARGET_AK512
    /* No-op on the MC510: every BEMF phase owns a dedicated channel and
     * AD2CH1 carries the POT. Never rewrite PINSEL at runtime here. */
    (void)pinsel;
#else
    AD2CH1CONbits.PINSEL = pinsel;
#endif
}

#endif /* FEATURE_ADC_CMP_ZC */
