/**
 * @file hwzc.c
 *
 * @brief Hardware ZC detection via ADC digital comparators + SCCP timers.
 *
 * State machine per commutation step:
 *   BLANKING -> WATCHING -> COMM_PENDING -> commutate -> BLANKING
 *
 * All functions called from priority-7 ISRs (comparator or SCCP1) except
 * HWZC_Init (called from ServiceInit) and HWZC_Enable (called from ADC ISR
 * at the moment of crossover handoff).
 *
 * Binding rules enforced:
 *   Rule 2: Comparator IE disabled before PINSEL changes
 *   Rule 3: hwzc.phase written BEFORE starting SCCP1 timer
 *   Rule 4: Threshold from garudaData.bemf.zcThreshold
 *   Rule 10: Comparator ISR trusts ADnCMPSTAT, no re-read
 *   Rule 13: stepPeriodHR protected by seqlock (writeSeq)
 *
 * Component: HWZC
 */

#include "../garuda_config.h"

#if FEATURE_ADC_CMP_ZC

#include "hwzc.h"
#include "commutation.h"
#include "../garuda_calc_params.h"
#include "../hal/hal_adc.h"
#include "../hal/hal_timer.h"
#include <xc.h>

/**
 * @brief Initialize HWZC state to idle defaults.
 * Called from GARUDA_ServiceInit().
 */
void HWZC_Init(volatile GARUDA_DATA_T *pData)
{
    pData->hwzc.phase = HWZC_IDLE;
    pData->hwzc.enabled = false;
    pData->hwzc.enablePending = false;
    pData->hwzc.activeCore = 0;
    pData->hwzc.lastZcStamp = 0;
    pData->hwzc.lastCommStamp = 0;
    pData->hwzc.stepPeriodHR = 0;
    pData->hwzc.writeSeq = 0;
    pData->hwzc.commSeq = 0;
    pData->hwzc.goodZcCount = 0;
    pData->hwzc.missCount = 0;
    pData->hwzc.fallbackPending = false;
    pData->hwzc.disableRequested = false;
    pData->hwzc.dbgLatchDisable = false;
    pData->hwzc.totalZcCount = 0;
    pData->hwzc.totalMissCount = 0;
    pData->hwzc.totalCommCount = 0;
    pData->hwzc.dbgTimeoutAdcVal = 0;
    pData->hwzc.dbgTimeoutThresh = 0;
    pData->hwzc.dbgTimeoutCmpmod = 0;
    pData->hwzc.dbgTimeoutCmpstat = 0;
    pData->hwzc.dbgTimeoutCore = 0;
    pData->hwzc.dbgTimeoutStep = 0;
    pData->hwzc.dbgTimeoutBemfThresh = 0;
    pData->hwzc.dbgEnableStepPeriod = 0;
    pData->hwzc.dbgAdvanceDeg = 0;
#if FEATURE_BEMF_INTEGRATION
    pData->hwzc.shadowHwzcSkipCount = 0;
    pData->hwzc.obsCommSeq = 0;
    pData->hwzc.obsLastCommTick = 0;
#endif

    HAL_ADC_DisableComparatorIE(1);
    HAL_ADC_DisableComparatorIE(2);
}

/**
 * @brief Activate hardware ZC detection. Called from ADC ISR at the
 * post-CheckDeadline commutation hook when enablePending && wasZcDeadline.
 * The new step has just been applied by COMMUTATION_AdvanceStep().
 */
void HWZC_Enable(volatile GARUDA_DATA_T *pData)
{
    pData->hwzc.enabled = true;
    pData->hwzc.enablePending = false;
    pData->hwzc.disableRequested = false;
    pData->hwzc.goodZcCount = 0;
    pData->hwzc.missCount = 0;

    /* Seed stepPeriodHR from software ZC step period */
    pData->hwzc.dbgEnableStepPeriod = pData->timing.stepPeriod;
    pData->hwzc.stepPeriodHR = HWZC_ADC_TO_SCCP2(pData->timing.stepPeriod);

    /* Initialize timestamps */
    uint32_t now = HAL_SCCP2_ReadTimestamp();
    pData->hwzc.lastZcStamp = now;
    pData->hwzc.lastCommStamp = now;

    /* Enable SCCP1 interrupt (priority already set by ServiceInit) */
    _CCT1IE = 1;

    /* Arm hardware for the current (new) step */
    HWZC_OnCommutation(pData);
}

/**
 * @brief Deactivate hardware ZC and request fallback to software ZC.
 * Called from SCCP1 ISR (on miss-limit) or when eRPM drops below crossover.
 * Sets fallbackPending for the ADC ISR to perform the actual re-seed.
 */
void HWZC_Disable(volatile GARUDA_DATA_T *pData)
{
    pData->hwzc.enabled = false;

    /* Stop timer and disable all HW ZC interrupts */
    HAL_SCCP1_Stop();
    _CCT1IE = 0;
    HAL_ADC_DisableComparatorIE(1);
    HAL_ADC_DisableComparatorIE(2);

    pData->hwzc.phase = HWZC_IDLE;
    pData->hwzc.fallbackPending = true;
}

/**
 * @brief Set up ZC detection for the current commutation step.
 * Configures comparator channel, PINSEL, threshold, and starts blanking timer.
 * Called from SCCP1 ISR after commutation fires, and from HWZC_Enable().
 */
void HWZC_OnCommutation(volatile GARUDA_DATA_T *pData)
{
    uint32_t now = HAL_SCCP2_ReadTimestamp();
    pData->hwzc.lastCommStamp = now;

    /* Determine floating phase and comparator for this step */
    uint8_t step = pData->currentStep;
    uint8_t floatPhase = commutationTable[step].floatingPhase;
    int8_t pol = commutationTable[step].zcPolarity;
    bool risingZc = (pol > 0);

    /* Disable both comparator IEs before any mux change (Rule 2) */
    HAL_ADC_DisableComparatorIE(1);
    HAL_ADC_DisableComparatorIE(2);

    /* Select active ADC core and PINSEL */
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
    pData->hwzc.activeCore = core;

    /* Compute threshold with deadband (Rule 4) */
    uint16_t thresh = pData->bemf.zcThreshold;
    if (risingZc)
        thresh = (thresh + HWZC_CMP_DEADBAND < 4095) ? thresh + HWZC_CMP_DEADBAND : 4095;
    else
        thresh = (thresh > HWZC_CMP_DEADBAND) ? thresh - HWZC_CMP_DEADBAND : 0;

    HAL_ADC_ConfigComparator(core, thresh, risingZc);

    /* Calculate blanking delay from step period */
    uint32_t blankTicks = pData->hwzc.stepPeriodHR * HWZC_BLANKING_PERCENT / 100;
    if (blankTicks < 100) blankTicks = 100;  /* Minimum 1us blanking */

    /* Set state BEFORE starting timer (Rule 3) */
    pData->hwzc.phase = HWZC_BLANKING;

    HAL_SCCP1_StartOneShot(blankTicks);
}

/**
 * @brief Blanking period expired — enable comparator and start timeout timer.
 * Called from SCCP1 ISR when phase == HWZC_BLANKING.
 */
void HWZC_OnBlankingExpired(volatile GARUDA_DATA_T *pData)
{
    uint8_t core = pData->hwzc.activeCore;

    /* Clear any stale comparator events, then enable interrupt */
    HAL_ADC_ClearComparatorFlag(core);
    HAL_ADC_EnableComparatorIE(core);

    /* Timeout = 2x step period (same as software ZC) */
    uint32_t timeoutTicks = pData->hwzc.stepPeriodHR * 2;

    /* Set state BEFORE starting timer (Rule 3) */
    pData->hwzc.phase = HWZC_WATCHING;

    HAL_SCCP1_StartOneShot(timeoutTicks);
}

/**
 * @brief ZC detected by ADC comparator.
 * Comparator IE already disabled and CMPSTAT cleared by ISR stub.
 * Records timestamp, updates step period via seqlock, schedules commutation.
 */
void HWZC_OnZcDetected(volatile GARUDA_DATA_T *pData)
{
    uint32_t zcStamp = HAL_SCCP2_ReadTimestamp();

    /* Cancel the timeout timer */
    HAL_SCCP1_Stop();

    /* Update stepPeriodHR with seqlock (Rule 13) */
    uint32_t interval = zcStamp - pData->hwzc.lastZcStamp;
    pData->hwzc.writeSeq++;  /* odd = write in progress */
    uint32_t newPeriod = (3 * pData->hwzc.stepPeriodHR + interval) / 4;
    if (newPeriod < HWZC_MIN_STEP_TICKS)
        newPeriod = HWZC_MIN_STEP_TICKS;
    pData->hwzc.stepPeriodHR = newPeriod;
    pData->hwzc.writeSeq++;  /* even = stable */

    pData->hwzc.lastZcStamp = zcStamp;

    /* Track consecutive good ZCs */
    pData->hwzc.goodZcCount++;
    pData->hwzc.missCount = 0;
    pData->hwzc.totalZcCount++;

    /* Calculate commutation delay: half step period (30 degrees).
     * With timing advance, the delay is reduced proportionally. */
    uint32_t commDelay;
#if FEATURE_TIMING_ADVANCE
    {
        uint32_t eRPM = HWZC_TICKS_TO_ERPM(pData->hwzc.stepPeriodHR);
        uint16_t advDeg;
        if (eRPM <= RAMP_TARGET_ERPM)
            advDeg = TIMING_ADVANCE_MIN_DEG;
        else if (eRPM >= MAX_CLOSED_LOOP_ERPM)
            advDeg = TIMING_ADVANCE_MAX_DEG;
        else
        {
            uint32_t range = MAX_CLOSED_LOOP_ERPM - RAMP_TARGET_ERPM;
            uint32_t pos = eRPM - RAMP_TARGET_ERPM;
            advDeg = TIMING_ADVANCE_MIN_DEG +
                (uint16_t)((uint32_t)(TIMING_ADVANCE_MAX_DEG - TIMING_ADVANCE_MIN_DEG)
                           * pos / range);
        }
        commDelay = pData->hwzc.stepPeriodHR * (30 - advDeg) / 60;
        pData->hwzc.dbgAdvanceDeg = advDeg;
    }
#else
    commDelay = pData->hwzc.stepPeriodHR / 2;
#endif
    if (commDelay < 10) commDelay = 10;  /* Minimum 100ns */

    /* Set state BEFORE starting timer (Rule 3) */
    pData->hwzc.phase = HWZC_COMM_PENDING;

    HAL_SCCP1_StartOneShot(commDelay);
}

/**
 * @brief Commutation deadline reached — advance to next step.
 * Called from SCCP1 ISR when phase == HWZC_COMM_PENDING.
 */
void HWZC_OnCommDeadline(volatile GARUDA_DATA_T *pData)
{
    COMMUTATION_AdvanceStep(pData);
    pData->hwzc.totalCommCount++;
    pData->hwzc.commSeq++;  /* 16-bit, atomic for observer (Rule 13) */

    /* Set up ZC detection for the new step */
    HWZC_OnCommutation(pData);
}

/**
 * @brief ZC timeout — no crossing detected within 2x step period.
 * Called from SCCP1 ISR when phase == HWZC_WATCHING.
 * Either forces a step or falls back to software ZC on miss-limit.
 */
void HWZC_OnTimeout(volatile GARUDA_DATA_T *pData)
{
    /* Capture diagnostics: what did the ADC channel actually see? */
    uint8_t core = pData->hwzc.activeCore;
    if (core == 1)
    {
        pData->hwzc.dbgTimeoutAdcVal = AD1CH5DATA;
        pData->hwzc.dbgTimeoutThresh = AD1CH5CMPLO;
        pData->hwzc.dbgTimeoutCmpmod = AD1CH5CONbits.CMPMOD;
        pData->hwzc.dbgTimeoutCmpstat = AD1CMPSTATbits.CH5CMP;
    }
    else
    {
        pData->hwzc.dbgTimeoutAdcVal = AD2CH1DATA;
        pData->hwzc.dbgTimeoutThresh = AD2CH1CMPLO;
        pData->hwzc.dbgTimeoutCmpmod = AD2CH1CONbits.CMPMOD;
        pData->hwzc.dbgTimeoutCmpstat = AD2CMPSTATbits.CH1CMP;
    }
    pData->hwzc.dbgTimeoutCore = core;
    pData->hwzc.dbgTimeoutStep = pData->currentStep;
    pData->hwzc.dbgTimeoutBemfThresh = pData->bemf.zcThreshold;

    pData->hwzc.missCount++;
    pData->hwzc.totalMissCount++;

    if (pData->hwzc.goodZcCount > 0)
        pData->hwzc.goodZcCount--;

    if (pData->hwzc.missCount >= HWZC_MISS_LIMIT)
    {
        /* Too many misses — fall back to software ZC.
         * Latch disable to prevent re-enable cycling (preserves diagnostics). */
        pData->hwzc.dbgLatchDisable = true;
        HWZC_Disable(pData);
        return;
    }

    /* Forced step: advance commutation and try next step */
    COMMUTATION_AdvanceStep(pData);
    pData->hwzc.totalCommCount++;
    pData->hwzc.commSeq++;

    HWZC_OnCommutation(pData);
}

#endif /* FEATURE_ADC_CMP_ZC */
