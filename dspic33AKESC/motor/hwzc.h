/**
 * @file hwzc.h
 *
 * @brief Hardware ZC detection module using ADC digital comparators and SCCP timers.
 * State machine: BLANKING -> WATCHING -> COMM_PENDING -> (commutate) -> BLANKING.
 *
 * Component: HWZC
 */

#ifndef HWZC_H
#define HWZC_H

#include "../garuda_types.h"

#if FEATURE_ADC_CMP_ZC

#ifdef __cplusplus
extern "C" {
#endif

void HWZC_Init(volatile GARUDA_DATA_T *pData);
void HWZC_Enable(volatile GARUDA_DATA_T *pData);
void HWZC_Disable(volatile GARUDA_DATA_T *pData);
void HWZC_OnCommutation(volatile GARUDA_DATA_T *pData);
void HWZC_OnZcDetected(volatile GARUDA_DATA_T *pData);
void HWZC_OnBlankingExpired(volatile GARUDA_DATA_T *pData);
void HWZC_OnCommDeadline(volatile GARUDA_DATA_T *pData);
void HWZC_OnTimeout(volatile GARUDA_DATA_T *pData);
#if FEATURE_HWZC_SECTOR_PI
void HWZC_OnPiPeriodExpired(volatile GARUDA_DATA_T *pData);
#endif
#if HWZC_USE_SW_COMPARE
void HWZC_OnSoftwareSample(volatile GARUDA_DATA_T *pData);
#endif

/**
 * @brief Apply RC-filter phase-lag compensation to a CMPLO value.
 *
 * Shifts the threshold away from neutral so the HW comparator fires at the
 * TRUE (pre-filter) BEMF zero-crossing instead of the filter-delayed one.
 * Shared by HWZC_OnCommutation (initial CMPLO set) and the 24 kHz ADC ISR's
 * live CMPLO refresh, to keep behavior consistent between the two sites.
 *
 * Math + sign convention documented at FEATURE_HWZC_FILTER_COMP in
 * garuda_config.h. Writes the applied offset to pData->hwzc.dbgFilterOffset
 * for bench validation. Returns input unchanged when the feature is off.
 */
static inline uint16_t HWZC_ApplyFilterComp(volatile GARUDA_DATA_T *pData,
                                            uint16_t thresh,
                                            bool risingZc)
{
#if FEATURE_HWZC_FILTER_COMP
    /* Source for ω·τ: prefer the SLOW-IIR period when PI mode is active,
     * to break the positive-feedback loop (PI shrinks period → ω·τ grows →
     * offset grows → CMPLO shifts → comparator earlier → PI shrinks more).
     * In reactive mode (PI off), fall back to stepPeriodHR. */
#if FEATURE_HWZC_SECTOR_PI
    uint32_t stepP = pData->hwzc.stepPeriodForFilterComp;
    if (stepP == 0) stepP = pData->hwzc.stepPeriodHR;
#else
    uint32_t stepP = pData->hwzc.stepPeriodHR;
#endif
    if (stepP == 0) {
        pData->hwzc.dbgFilterOffset = 0;
        return thresh;
    }

    /* ω·τ_Q15 = K / stepPeriodHR. Cap before multiplying by amp so the
     * intermediate (amp × ω·τ) fits inside uint32_t. */
    uint32_t omegaTauQ15 = HWZC_FILTER_K_Q15 / stepP;
    if (omegaTauQ15 > HWZC_FILTER_MAX_OMEGA_Q15)
        omegaTauQ15 = HWZC_FILTER_MAX_OMEGA_Q15;

    /* offset magnitude uses zcAmpForFilterComp (slow IIR ~46 ms), NOT the
     * fast-tracking thresh. This decouples the offset from duty steps —
     * when duty bumps, the comparator's neutral (CMPLO baseline = thresh)
     * tracks the new DC level instantly, but the offset magnitude stays
     * matched to the real BEMF amplitude which lags by rotor inertia.
     *
     * Fallback to thresh on first tick if amp not yet seeded — same as
     * the old behavior, avoids a zero offset before the IIR converges. */
    uint16_t amp = pData->bemf.zcAmpForFilterComp;
    if (amp == 0) amp = thresh;

    /* offset = (amp × ω·τ) >> 15  ×  AMP_PCT / 100   (staged for no overflow).
     * Per-polarity: rising uses HWZC_FILTER_AMP_PCT (bench-proven to 232k),
     * falling uses HWZC_FILTER_AMP_PCT_FALLING (the masked branch — see config).
     * AMP_PCT_FALLING=0 → falling offset is 0 → falling returns bare thresh. */
    uint32_t ampPct = risingZc ? HWZC_FILTER_AMP_PCT : HWZC_FILTER_AMP_PCT_FALLING;
    uint32_t offset = ((uint32_t)amp * omegaTauQ15) >> 15;
    offset = (offset * ampPct) / 100UL;
    if (offset > HWZC_FILTER_MAX_OFFSET) offset = HWZC_FILTER_MAX_OFFSET;
    pData->hwzc.dbgFilterOffset = (uint16_t)offset;

    if (risingZc)
        return (thresh > offset) ? (uint16_t)(thresh - offset) : 0;
    else
        return (thresh + offset < 4095u) ? (uint16_t)(thresh + offset) : 4095u;
#else
    (void)pData; (void)risingZc;
    return thresh;
#endif
}

#ifdef __cplusplus
}
#endif

#endif /* FEATURE_ADC_CMP_ZC */
#endif /* HWZC_H */
