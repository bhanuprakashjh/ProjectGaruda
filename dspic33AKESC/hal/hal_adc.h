/**
 * @file hal_adc.h
 *
 * @brief ADC module definitions for phase voltage sensing, Vbus, and potentiometer.
 * Adapted from AN1292 reference:
 *   - Keeps: Vbus (AD1CH4), Pot (AD1CH1), PWM trigger source
 *   - Removes: Current sense channels (IA, IB, IBUS)
 *   - Adds: Phase B on AD1CH0 (RB8), Phase A/C on AD2CH0 (RB9/RA10, muxed)
 *
 * Definitions in this file are for dsPIC33AK128MC106.
 *
 * GARUDA_TARGET_AK512 (dsPIC33AK512MC510, AN957 channel map): every signal
 * owns a dedicated channel — no runtime PINSEL muxing anywhere:
 *   IA   = AD1CH0 PINSEL 0 (OA1OUT/AD1AN0)   IB   = AD2CH0 PINSEL 0 (OA2OUT/AD2AN0)
 *   IBUS = AD3CH1 PINSEL 0 (OA3OUT/AD3AN0)   VBUS = AD3CH2 PINSEL 4 (AD3AN4)
 *   POT  = AD2CH1 PINSEL 5 (AD2AN5)
 *   BEMF high-speed (SCCP3 1 MHz + digital comparator, replaces AD1CH5/AD2CH1):
 *   VA   = AD1CH1 PINSEL 3 (AD1AN3)          VB   = AD1CH2 PINSEL 4 (AD1AN4)
 *   VC   = AD2CH2 PINSEL 4 (AD2AN4)
 *   BEMF PWM-synced samples (PG1TRIGA, replaces 106's AD1CH0/AD2CH0 role):
 *   VB   = AD1CH3 PINSEL 4 (ISR source)      VA   = AD1CH4 PINSEL 3
 *   VC   = AD2CH3 PINSEL 4
 *
 * Component: ADC
 */

#ifndef _HAL_ADC_H
#define _HAL_ADC_H

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "../garuda_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum count in 12-bit ADC */
#define MAX_ADC_COUNT       4096.0f
#define HALF_ADC_COUNT      2048

/* Floating phase identifiers for BEMF mux selection */
#define FLOATING_PHASE_A    0
#define FLOATING_PHASE_B    1
#define FLOATING_PHASE_C    2

#if GARUDA_TARGET_AK512
/* ============================ AK512 (MC510) ============================= */

/* ADC buffer read macros — MUST read to clear data-ready condition */
#if FEATURE_FOC || FEATURE_FOC_V2 || FEATURE_FOC_V3 || FEATURE_FOC_AN1078
#define ADCBUF_IA           (uint16_t)AD1CH0DATA
#define ADCBUF_IB           (uint16_t)AD2CH0DATA
#elif FEATURE_IF_STARTUP
/* I-f bring-up: same aliasing trick as the 106 — the "phase voltage" reads
 * deliver the OA1/OA2 currents (CH0s carry IA/IB on the MC510 anyway). */
#define ADCBUF_PHASE_B      (uint16_t)AD1CH0DATA
#define ADCBUF_PHASE_AC     (uint16_t)AD2CH0DATA
#else
/* 6-step phase voltage buffers — PWM-synced (PG1TRIGA) dedicated channels.
 * PHASE_B  = VB on AD1CH3 (ISR source).
 * PHASE_AC = active floating phase A or C; both VA (AD1CH4) and VC (AD2CH3)
 *            are read every tick (clears data-ready on both), the helper
 *            returns the one selected by HAL_ADC_SelectBEMFChannel(). */
#define ADCBUF_PHASE_B      (uint16_t)AD1CH3DATA
#define ADCBUF_PHASE_AC     HAL_ADC_ReadPhaseAC()
uint16_t HAL_ADC_ReadPhaseAC(void);
/* All three PWM-synced phase voltages, same PG1TRIGA instant — the raw
 * ingredients of the measured virtual neutral (VA+VB+VC)/3. */
#define ADCBUF_BEMF_VA      (uint16_t)AD1CH4DATA
#define ADCBUF_BEMF_VB      (uint16_t)AD1CH3DATA
#define ADCBUF_BEMF_VC      (uint16_t)AD2CH3DATA
#endif
#define ADCBUF_POT          (uint16_t)AD2CH1DATA
#define ADCBUF_VBUS         (uint16_t)AD3CH2DATA

#if FEATURE_HW_OVERCURRENT
#define ADCBUF_IBUS         (uint16_t)AD3CH1DATA
#endif

#if !FEATURE_FOC && !FEATURE_FOC_V2 && !FEATURE_FOC_V3 && !FEATURE_FOC_AN1078
/* 6-step diagnostic phase-current monitors. On the MC510 the AN957 map puts
 * IA/IB on the CH0s permanently (PG1TRIGA-triggered) — same sampling instant
 * as the 106's AD1CH3/AD2CH2 monitor channels. */
#define ADCBUF_IA_MON       (uint16_t)AD1CH0DATA
#define ADCBUF_IB_MON       (uint16_t)AD2CH0DATA
#endif

/* Control-loop ISR source: the channel carrying the SAME SIGNAL as the 106's
 * AD1CH0 — VB (AD1CH3) in the 6-step build, IA (AD1CH0) in FOC/I-f builds. */
#if FEATURE_FOC || FEATURE_FOC_V2 || FEATURE_FOC_V3 || FEATURE_FOC_AN1078 || FEATURE_IF_STARTUP
#define GARUDA_EnableADCInterrupt()     _AD1CH0IE = 1
#define GARUDA_DisableADCInterrupt()    _AD1CH0IE = 0
#define GARUDA_ADC_INTERRUPT            _AD1CH0Interrupt
#define GARUDA_ClearADCIF()             _AD1CH0IF = 0
#define GARUDA_ADC_IP                   _AD1CH0IP
#else
#define GARUDA_EnableADCInterrupt()     _AD1CH3IE = 1
#define GARUDA_DisableADCInterrupt()    _AD1CH3IE = 0
#define GARUDA_ADC_INTERRUPT            _AD1CH3Interrupt
#define GARUDA_ClearADCIF()             _AD1CH3IF = 0
#define GARUDA_ADC_IP                   _AD1CH3IP
#endif

#else /* !GARUDA_TARGET_AK512 — original dsPIC33AK128MC106 map */

/* ADC buffer read macros — MUST read to clear data-ready condition */
#if FEATURE_FOC || FEATURE_FOC_V2 || FEATURE_FOC_V3 || FEATURE_FOC_AN1078
/* FOC current sense buffers — raw unsigned, offset subtracted at runtime.
 * RK1 convention: runtime calibration during IDLE/ARMED averages 1024 samples
 * to find the zero-current midpoint, then subtracts per-tick in the ISR. */
#define ADCBUF_IA           (uint16_t)AD1CH0DATA
#define ADCBUF_IB           (uint16_t)AD2CH0DATA
#else
/* 6-step phase voltage buffers */
#define ADCBUF_PHASE_B      (uint16_t)AD1CH0DATA
#define ADCBUF_PHASE_AC     (uint16_t)AD2CH0DATA
#endif
#define ADCBUF_POT          (uint16_t)AD1CH1DATA
#define ADCBUF_VBUS         (uint16_t)AD1CH4DATA

#if FEATURE_HW_OVERCURRENT
#define ADCBUF_IBUS         (uint16_t)AD1CH2DATA
#endif

#if !FEATURE_FOC && !FEATURE_FOC_V2 && !FEATURE_FOC_V3 && !FEATURE_FOC_AN1078
/* 6-step diagnostic phase-current monitors (AD1CH3 = Ia, AD2CH2 = Ib).
 * 1 MHz SCCP3 sampling, read every 24 kHz ADC ISR to track peak phase
 * currents for empirical validation of the U25B / 22 A trip hypothesis. */
#define ADCBUF_IA_MON       (uint16_t)AD1CH3DATA
#define ADCBUF_IB_MON       (uint16_t)AD2CH2DATA
#endif

/* Phase B completion (AD1CH0) is the ADC interrupt source */
#define GARUDA_EnableADCInterrupt()     _AD1CH0IE = 1
#define GARUDA_DisableADCInterrupt()    _AD1CH0IE = 0
#define GARUDA_ADC_INTERRUPT            _AD1CH0Interrupt
#define GARUDA_ClearADCIF()             _AD1CH0IF = 0
#define GARUDA_ADC_IP                   _AD1CH0IP

#endif /* GARUDA_TARGET_AK512 */

void InitializeADCs(void);

#if FEATURE_FOC || FEATURE_FOC_V2 || FEATURE_FOC_V3 || FEATURE_FOC_AN1078
/* No-op: FOC does not use BEMF mux, but commutation.c references this symbol */
static inline bool HAL_ADC_SelectBEMFChannel(uint8_t floatingPhase)
{ (void)floatingPhase; return false; }
#else
bool HAL_ADC_SelectBEMFChannel(uint8_t floatingPhase);
#endif

#if FEATURE_ADC_CMP_ZC
void HAL_ADC_InitHighSpeedBEMF(void);
void HAL_ADC_ConfigComparator(uint8_t adcCore, uint16_t threshold, bool risingZc);
void HAL_ADC_EnableComparatorIE(uint8_t adcCore);
void HAL_ADC_DisableComparatorIE(uint8_t adcCore);
void HAL_ADC_ClearComparatorFlag(uint8_t adcCore);
void HAL_ADC_SetHighSpeedPinsel(uint8_t pinsel);
/* Live CMPLO update (no CMPMOD change) — safe to call from ADC ISR
 * while the comparator is armed. Atomic SFR write. Takes effect on
 * the next ADC conversion (~1 µs at SCCP3 1 MHz, or ~4 µs at 4×
 * oversample). */
void HAL_ADC_UpdateComparatorThreshold(uint8_t adcCore, uint16_t threshold);

/* Select the high-speed BEMF detection channel for the floating phase and
 * return the comparator handle that hwzc stores in hwzc.activeCore and
 * passes back into the comparator functions above.
 *   AK128: handle = ADC core (1 = AD1CH5 fixed Phase B, 2 = AD2CH1 with the
 *          A/C PINSEL mux applied here — verbatim the old hwzc.c block).
 *   AK512: handle = floating phase (0=VA/AD1CH1, 1=VB/AD1CH2, 2=VC/AD2CH2);
 *          dedicated channels, PINSEL is never touched at runtime. */
uint8_t HAL_ADC_SelectBemfPhase(uint8_t floatPhase);
#if GARUDA_TARGET_AK512
void HAL_ADC_BemfBurstOff(void);
#endif

#if GARUDA_TARGET_AK512
/* Raw data of a high-speed BEMF channel by comparator handle (verify reads,
 * timeout diagnostics). AK128 code reads AD1CH5DATA/AD2CH1DATA directly. */
static inline uint16_t HAL_ADC_BemfChannelData(uint8_t handle)
{
    if (handle == FLOATING_PHASE_A) return (uint16_t)AD1CH1DATA;
    if (handle == FLOATING_PHASE_B) return (uint16_t)AD1CH2DATA;
    return (uint16_t)AD2CH2DATA;
}
#endif
#endif

#ifdef __cplusplus
}
#endif

#endif /* _HAL_ADC_H */
