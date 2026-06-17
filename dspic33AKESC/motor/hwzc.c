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
#include "speed_pi.h"
#include "commutation.h"
#include "../garuda_calc_params.h"
#include "../hal/hal_adc.h"
#include "../hal/hal_timer.h"
#if (FEATURE_HWZC_PI_FLOAT && HWZC_PI_FF_ENABLE) || FEATURE_HWZC_ABS_FLOOR
#include "../garuda_foc_params.h"   /* VBUS_SCALE_V_PER_COUNT, focKeUvSRad scaling */
#endif
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
    pData->hwzc.firstZcAfterEnable = false;
    pData->hwzc.dbgLatchDisable = false;
    pData->hwzc.stepsSinceLastHwZc = 0;
    pData->hwzc.totalZcCount = 0;
    pData->hwzc.totalMissCount = 0;
    pData->hwzc.totalCommCount = 0;
    pData->hwzc.rejectsThisStep = 0;
    pData->hwzc.dbgTimeoutAdcVal = 0;
    pData->hwzc.dbgTimeoutThresh = 0;
    pData->hwzc.dbgTimeoutCmpmod = 0;
    pData->hwzc.dbgTimeoutCmpstat = 0;
    pData->hwzc.dbgTimeoutCore = 0;
    pData->hwzc.dbgTimeoutStep = 0;
    pData->hwzc.dbgTimeoutBemfThresh = 0;
    pData->hwzc.dbgEnableStepPeriod = 0;
    pData->hwzc.dbgAdvanceDeg = 0;
    pData->hwzc.dbgFilterOffset = 0;
#if FEATURE_HWZC_SECTOR_PI
    pData->hwzc.timerPeriod = 0;
    pData->hwzc.integrator = 0;
#if FEATURE_HWZC_PI_FLOAT
    pData->hwzc.integratorF = 0.0f;
#endif
#if FEATURE_HWZC_PI_DEFENSIVE
    pData->hwzc.piDefMissStreak = 0;
    pData->hwzc.piDefGoodStreak = 0;
    pData->hwzc.piDefActive = 0;
    pData->hwzc.piDefEntryCount = 0;
#endif
    pData->hwzc.lastCaptureHR = 0;
    pData->hwzc.prevCommHR = 0;
    pData->hwzc.captureValid = false;
    pData->hwzc.pllStartActive = 0;
    pData->hwzc.pllStartGood = 0;
    pData->hwzc.pllPrevCap = 0;
    pData->hwzc.stepPeriodForFilterComp = 0;
    pData->hwzc.dbgPiDelta = 0;
    pData->hwzc.dbgPiCaptureCount = 0;
    pData->hwzc.dbgPiMissCount = 0;
    for (uint8_t s = 0; s < 6; s++) {
        pData->hwzc.dbgPiMissBySector[s] = 0;
        pData->hwzc.dbgPiCapBySector[s] = 0;
    }
#endif
#if FEATURE_BEMF_INTEGRATION
    pData->hwzc.shadowHwzcSkipCount = 0;
    pData->hwzc.obsCommSeq = 0;
    pData->hwzc.obsLastCommTick = 0;
#endif

#if GARUDA_TARGET_AK512
    /* Per-phase comparator handles: disable all three BEMF channels */
    HAL_ADC_DisableComparatorIE(FLOATING_PHASE_A);
    HAL_ADC_DisableComparatorIE(FLOATING_PHASE_B);
    HAL_ADC_DisableComparatorIE(FLOATING_PHASE_C);
#else
    HAL_ADC_DisableComparatorIE(1);
    HAL_ADC_DisableComparatorIE(2);
#endif
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
    pData->hwzc.goodZcCount = 0;
    pData->hwzc.missCount = 0;
    pData->hwzc.noiseRejectCount = 0;
    pData->hwzc.stepsSinceLastHwZc = 1;  /* First step: IIR valid with seeded lastZcStamp */
    pData->hwzc.firstZcAfterEnable = true; /* Bypass plausibility gate + IIR on 1st ZC */

    /* Seed stepPeriodHR from software ZC step period */
    pData->hwzc.dbgEnableStepPeriod = pData->timing.stepPeriod;
    pData->hwzc.stepPeriodHR = HWZC_ADC_TO_SCCP2(pData->timing.stepPeriod);

    /* Initialize timestamps. Seed lastZcStamp to half a step period before
     * now so the first measured interval (first real ZC - lastZcStamp) is
     * approximately one full step, not half a step. Without this, the first
     * IIR update biases stepPeriodHR low, causing an early commutation. */
    uint32_t now = HAL_SCCP2_ReadTimestamp();
    pData->hwzc.lastZcStamp = now - pData->hwzc.stepPeriodHR / 2;
    pData->hwzc.lastCommStamp = now;

#if FEATURE_HWZC_SECTOR_PI
    /* Sector PI seed: start timerPeriod at the IIR-seeded stepPeriodHR
     * (the same value the reactive path uses for its first commutation
     * scheduling). integrator tracks timerPeriod 1:1. prevCommHR matches
     * lastCommStamp so the first capValue (computed at next PI fire)
     * uses a sensible base. captureValid stays false until first ZC. */
    pData->hwzc.timerPeriod   = pData->hwzc.stepPeriodHR;
    pData->hwzc.integrator    = pData->hwzc.stepPeriodHR;
    pData->hwzc.prevCommHR    = now;
    pData->hwzc.lastCaptureHR = now - pData->hwzc.stepPeriodHR / 2;
    pData->hwzc.captureValid  = false;
    pData->hwzc.stepPeriodForFilterComp = pData->hwzc.stepPeriodHR;
#if FEATURE_HWZC_PI_FLOAT
    /* integratorF = period (same semantics as integer integrator). */
    pData->hwzc.integratorF = (float)pData->hwzc.stepPeriodHR;
  #if HWZC_PI_FF_ENABLE
    /* Phase 2 feedforward as a ONE-SHOT SEED ONLY: if the no-load
     * physics formula predicts a tighter period than the SW handoff
     * value, jump-start the integrator to the physics prediction.
     * Only safe to apply when motor is already moving (duty > 3%)
     * AND the prediction would SLOW the controller (P_ff > stepPeriodHR).
     * Speeding it up at handoff is what stalled the motor on the
     * first FF attempt — don't do that. */
    {
        float dutyFrac = (float)pData->duty / (float)LOOPTIME_TCY;
        float vbus_v   = (float)pData->vbusRaw * VBUS_SCALE_V_PER_COUNT;
        float lambdaPm = (float)gspParams.focKeUvSRad;
        if (dutyFrac > 0.03f && vbus_v > 6.0f && lambdaPm > 0.0f) {
            float P_ff = (181.380f * lambdaPm) / (vbus_v * dutyFrac);
            /* Only adopt FF if it predicts a SLOWER controller
             * (longer period) than the SW handoff. Faster predictions
             * at handoff over-commit the bridge before the rotor has
             * caught up. */
            if (P_ff > pData->hwzc.integratorF)
                pData->hwzc.integratorF = P_ff;
        }
    }
  #endif
#endif
#endif

    /* Enable SCCP1 interrupt (priority already set by ServiceInit).
     * Clear any stale flag first to prevent immediate spurious ISR entry. */
    _CCT1IF = 0;
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

    /* Stop timer and disable all HW ZC interrupts.
     * Clear _CCT1IF to prevent stale event on next HWZC_Enable(). */
    HAL_SCCP1_Stop();
    _CCT1IE = 0;
    _CCT1IF = 0;
#if GARUDA_TARGET_AK512
    HAL_ADC_DisableComparatorIE(FLOATING_PHASE_A);
    HAL_ADC_DisableComparatorIE(FLOATING_PHASE_B);
    HAL_ADC_DisableComparatorIE(FLOATING_PHASE_C);
    HAL_ADC_BemfBurstOff();   /* leftover 1 MHz burst starves AD1CH3 ISR */
#else
    HAL_ADC_DisableComparatorIE(1);
    HAL_ADC_DisableComparatorIE(2);
#endif

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
#if FEATURE_HWZC_SECTOR_PI
    /* Save previous commutation timestamp before overwriting — capValue is
     * computed as (lastCaptureHR - prevCommHR) at next OnPiPeriodExpired. */
    pData->hwzc.prevCommHR = pData->hwzc.lastCommStamp;
#endif
    pData->hwzc.lastCommStamp = now;
    pData->hwzc.rejectsThisStep = 0;

    /* Determine floating phase and comparator for this step */
    uint8_t step = pData->currentStep;
    uint8_t floatPhase = commutationTable[step].floatingPhase;
    int8_t pol = commutationTable[step].zcPolarity;
    bool risingZc = (pol > 0);

    /* Disable both comparator IEs before any mux change (Rule 2) */
#if GARUDA_TARGET_AK512
    HAL_ADC_DisableComparatorIE(FLOATING_PHASE_A);
    HAL_ADC_DisableComparatorIE(FLOATING_PHASE_B);
    HAL_ADC_DisableComparatorIE(FLOATING_PHASE_C);
#else
    HAL_ADC_DisableComparatorIE(1);
    HAL_ADC_DisableComparatorIE(2);
#endif

    /* Select active detection channel/comparator for the floating phase.
     * AK128: HAL reproduces the old in-place block verbatim (core 1 =
     * AD1CH5 Phase B; core 2 = AD2CH1 + A/C PINSEL mux). AK512: handle =
     * floating phase, dedicated channel, no runtime PINSEL. */
    uint8_t core = HAL_ADC_SelectBemfPhase(floatPhase);
    pData->hwzc.activeCore = core;

    /* Compute threshold with deadband (Rule 4).
     * Filter compensation (if enabled) shifts the threshold AWAY from neutral
     * in the direction OPPOSITE the deadband, so the comparator fires earlier
     * by an amount equal to the PCB RC filter's phase lag at this stepPeriodHR.
     * No-op when FEATURE_HWZC_FILTER_COMP=0. */
    uint16_t thresh = pData->bemf.zcThreshold;
    thresh = HWZC_ApplyFilterComp(pData, thresh, risingZc);
    if (risingZc)
        thresh = (thresh + HWZC_CMP_DEADBAND < 4095) ? thresh + HWZC_CMP_DEADBAND : 4095;
    else
        thresh = (thresh > HWZC_CMP_DEADBAND) ? thresh - HWZC_CMP_DEADBAND : 0;

    HAL_ADC_ConfigComparator(core, thresh, risingZc);

    /* Calculate blanking delay from step period.
     * In sector PI mode, use timerPeriod (the PI-controlled commutation
     * interval) as the basis so blanking scales with the actual closed-loop
     * period, not the IIR-tracked stepPeriodHR (which is paused in PI mode). */
#if FEATURE_HWZC_SECTOR_PI
    uint32_t periodForBlank = pData->hwzc.timerPeriod;
    if (periodForBlank < pData->hwzc.stepPeriodHR)
        periodForBlank = pData->hwzc.stepPeriodHR;   /* startup safety */
#else
    uint32_t periodForBlank = pData->hwzc.stepPeriodHR;
#endif
    uint32_t blankTicks = periodForBlank * HWZC_BLANKING_PERCENT / 100;
    if (blankTicks < 100) blankTicks = 100;  /* Minimum 1us blanking */

    /* Set state BEFORE starting timer (Rule 3) */
    pData->hwzc.phase = HWZC_BLANKING;

    HAL_SCCP1_StartOneShot(blankTicks);
}

/**
 * @brief Compute commDelay from current stepPeriodHR + advance schedule.
 * Pulled out so OnBlankingExpired can pre-compute (saving ~1.5µs in the
 * latency-critical OnZcDetected ISR). Uses stepPeriodHR (IIR-filtered), so
 * pre-computing during BLANKING gives the same result as computing in
 * OnZcDetected except for the most-recent IIR update (~25% influence from
 * the just-measured interval). Acceptable tradeoff for latency.
 */
static inline uint32_t ComputeCommDelay(uint32_t stepPeriodHR,
                                        volatile uint16_t *dbgAdvanceDegOut)
{
    uint32_t commDelay;
#if FEATURE_TIMING_ADVANCE
    uint32_t eRPM = HWZC_TICKS_TO_ERPM(stepPeriodHR);
    uint16_t advDeg;
    if (eRPM <= RT_RAMP_TARGET_ERPM)
        advDeg = TIMING_ADVANCE_MIN_DEG;
    else if (eRPM >= RT_TIMING_ADV_FULL_ERPM)
        advDeg = RT_TIMING_ADV_MAX_DEG;
    else
    {
        uint32_t range = RT_TIMING_ADV_FULL_ERPM - RT_RAMP_TARGET_ERPM;
        uint32_t pos = eRPM - RT_RAMP_TARGET_ERPM;
        advDeg = TIMING_ADVANCE_MIN_DEG +
            (uint16_t)((uint32_t)(RT_TIMING_ADV_MAX_DEG - TIMING_ADVANCE_MIN_DEG)
                       * pos / range);
    }
    commDelay = stepPeriodHR * (30 - advDeg) / 60;
    if (dbgAdvanceDegOut) *dbgAdvanceDegOut = advDeg;
#else
    commDelay = stepPeriodHR / 2;
    (void)dbgAdvanceDegOut;
#endif
    if (commDelay < 10) commDelay = 10;
    return commDelay;
}

/**
 * @brief Blanking period expired — enable comparator and start timeout timer.
 * Called from SCCP1 ISR when phase == HWZC_BLANKING.
 */
void HWZC_OnBlankingExpired(volatile GARUDA_DATA_T *pData)
{
#if !HWZC_USE_SW_COMPARE
    uint8_t core = pData->hwzc.activeCore;

#if FEATURE_HWZC_FALLING_SW
    /* Hybrid per-polarity: arm the HW comparator ONLY for rising sectors.
     * Falling sectors are detected via the OFF-center SW compare in the ADC ISR
     * (the HW comparator is silent on falling at speed). Leaving its IE off for
     * falling avoids phantom ON-time fires competing with the SW capture. */
    if (commutationTable[pData->currentStep].zcPolarity > 0)
    {
        HAL_ADC_ClearComparatorFlag(core);
        HAL_ADC_EnableComparatorIE(core);
    }
#else
    /* Clear any stale comparator events, then enable interrupt */
    HAL_ADC_ClearComparatorFlag(core);
    HAL_ADC_EnableComparatorIE(core);
#endif
#else
    /* SW compare mode: leave the HW digital comparator IE disabled.
     * ZC detection runs in the 24 kHz ADC ISR via HWZC_OnSoftwareSample(). */
#endif

#if FEATURE_HWZC_SECTOR_PI
    /* Sector PI mode: schedule SCCP1 to fire at the autonomous commutation
     * deadline (lastCommStamp + timerPeriod). ZC events that arrive between
     * now and that deadline just timestamp lastCaptureHR; the PI math runs
     * in HWZC_OnPiPeriodExpired (dispatched from _CCT1Interrupt at phase
     * WATCHING when in PI mode). No separate timeout — a missed ZC is
     * absorbed by the PI as a non-update (timerPeriod unchanged). */
    uint32_t blankTicks = pData->hwzc.timerPeriod * HWZC_BLANKING_PERCENT / 100;
    if (blankTicks < 100) blankTicks = 100;
    uint32_t remaining = (pData->hwzc.timerPeriod > blankTicks)
                         ? (pData->hwzc.timerPeriod - blankTicks)
                         : 100;
    /* Set state BEFORE starting timer (Rule 3) */
    pData->hwzc.phase = HWZC_WATCHING;
    HAL_SCCP1_StartOneShot(remaining);
#else
    /* Reactive mode: SCCP1 fires as a timeout (no ZC) at 2× stepPeriodHR.
     * On real ZC, OnZcDetected restarts SCCP1 with commDelay for the
     * commutation phase. */
    uint32_t timeoutTicks = pData->hwzc.stepPeriodHR * 2;

    /* Pre-compute commDelay so the latency-critical OnZcDetected ISR can
     * skip the ~1.5µs of division/interpolation. Result is essentially the
     * same — OnZcDetected would only see a slightly different stepPeriodHR
     * after one more IIR update (25% influence). At 100k eRPM that's ~1°
     * of advance equivalent recovered. */
    pData->hwzc.cachedCommDelay = ComputeCommDelay(pData->hwzc.stepPeriodHR,
                                                    &pData->hwzc.dbgAdvanceDeg);

    /* Set state BEFORE starting timer (Rule 3) */
    pData->hwzc.phase = HWZC_WATCHING;

    HAL_SCCP1_StartOneShot(timeoutTicks);
#endif
}

/**
 * @brief ZC detected by ADC comparator.
 * Comparator IE already disabled and CMPSTAT cleared by ISR stub.
 * Records timestamp, updates step period via seqlock, schedules commutation.
 */
void HWZC_OnZcDetected(volatile GARUDA_DATA_T *pData)
{
    /* Off-time gate (PWMxH-LOW rejection) was REMOVED (2026-04-20).
     *
     * The gate added up to ~42 µs of detection latency (one full PWM cycle
     * at 24 kHz) when a real ZC happened to cross the threshold during
     * PWM OFF-time. At 118 k eRPM (sector = 85 µs) that's ~50 % of a
     * sector — the 22° TIMING_ADVANCE_MAX_DEG couldn't compensate it, and
     * the motor topped out around 88 k instead of the 118 k milestone
     * recorded at commit 868b2ff.
     *
     * The plausibility gate below (reject interval < 70 % of stepPeriodHR)
     * is sufficient alone for phantom rejection — it was what the 118 k
     * run used, and phantoms whose spacing happens to land above 70 % of
     * the expected sector are rare enough that the IIR absorbs them.
     */
    uint32_t zcStamp = HAL_SCCP2_ReadTimestamp();

    /* First ZC after HWZC_Enable: bypass plausibility gate AND skip IIR.
     * The seeded lastZcStamp is a half-step guess based on where morph exit
     * happened to fall in the rotor cycle. The first measured interval is
     * therefore rotor-position-dependent and not representative — it can be
     * anywhere from 10% to 190% of a real step period. Accepting it blindly
     * risks (a) false reject by plausibility gate → timeout cascade → HWZC
     * disable → stall, or (b) IIR getting dragged off stepPeriodHR. Just
     * anchor lastZcStamp; the SECOND ZC gives a clean, full-step interval. */
    if (pData->hwzc.firstZcAfterEnable)
    {
        pData->hwzc.firstZcAfterEnable = false;
#if FEATURE_HWZC_SECTOR_PI
        /* PI mode: don't reschedule SCCP1 — the autonomous period timer
         * set up in OnBlankingExpired stays active. Just timestamp this
         * first capture so the next OnPiPeriodExpired can compute capValue
         * against prevCommHR. */
        pData->hwzc.lastCaptureHR = zcStamp;
        pData->hwzc.captureValid  = true;
        pData->hwzc.lastZcStamp   = zcStamp;
        pData->hwzc.stepsSinceLastHwZc = 0;
        if (pData->hwzc.goodZcCount < 0xFFFE)
            pData->hwzc.goodZcCount++;
        pData->hwzc.missCount = 0;
        pData->hwzc.totalZcCount++;
        HAL_ADC_DisableComparatorIE(pData->hwzc.activeCore);
        /* phase stays WATCHING; SCCP1 already armed for end-of-period */
        return;
#else
        HAL_SCCP1_Stop();
        pData->hwzc.lastZcStamp = zcStamp;
        pData->hwzc.stepsSinceLastHwZc = 0;
        if (pData->hwzc.goodZcCount < 0xFFFE)
            pData->hwzc.goodZcCount++;
        pData->hwzc.missCount = 0;
        pData->hwzc.totalZcCount++;

        /* First ZC: use freshly computed commDelay since OnBlankingExpired's
         * cached value reflects the seeded stepPeriodHR which is correct here.
         * (One inline call to ComputeCommDelay — couldn't pre-compute yet.) */
        uint32_t commDelay = ComputeCommDelay(pData->hwzc.stepPeriodHR,
                                              &pData->hwzc.dbgAdvanceDeg);
        /* Gate comparator OFF during COMM_PENDING — see comment in main path */
        HAL_ADC_DisableComparatorIE(pData->hwzc.activeCore);
        pData->hwzc.phase = HWZC_COMM_PENDING;
        HAL_SCCP1_StartOneShot(commDelay);
        return;
#endif
    }

#if FEATURE_HWZC_PWM_GATE
    /* PWM-state gate: reject captures during PWM OFF time. At BEMF crossover
     * (Vapp ≈ Vbemf), the comparator fires on different signal regimes in
     * ON vs OFF windows — ON-time crossings around motor neutral (duty×Vbus/2),
     * OFF-time pseudo-crossings around 0V (all driven phases grounded, only
     * BEMF on floating phase). PI ingesting both produces ambiguous rotor
     * angle estimates → occasional 60° slip → 22A phase spike → UV cascade.
     *
     * Reads the H-side GPIO of the currently PWMing phase. In complementary
     * mode, H-side HIGH = PWM ON, H-side LOW = PWM OFF. Worst-case latency
     * at 45 kHz × 79% duty = 4.6 µs OFF time ≈ 3° at 200k eRPM. */
    {
        const COMMUTATION_STEP_T *cs = &commutationTable[pData->currentStep];
        bool pwmOn;
        if      (cs->phaseA == PHASE_PWM_ACTIVE) pwmOn = (PORTD & (1u << 2)) != 0; /* RD2 = PWM1H */
        else if (cs->phaseB == PHASE_PWM_ACTIVE) pwmOn = (PORTD & (1u << 0)) != 0; /* RD0 = PWM2H */
        else                                     pwmOn = (PORTC & (1u << 3)) != 0; /* RC3 = PWM3H */
        if (!pwmOn) {
            pData->hwzc.noiseRejectCount++;
            pData->hwzc.rejectsThisStep++;
            uint8_t core = pData->hwzc.activeCore;
            HAL_ADC_ClearComparatorFlag(core);
            HAL_ADC_EnableComparatorIE(core);
            return;
        }
    }
#endif

    /* INTERVAL CHECK FIRST (cheap, ~500 ns) — reorder C: kills early-arriving
     * noise before spending 3 µs on verify reads. Most PWM-ripple-driven
     * comparator re-fires happen within microseconds of the last accept,
     * so this catches them before the verify loop runs. */
    uint32_t interval = zcStamp - pData->hwzc.lastZcStamp;
    uint32_t minInterval = pData->hwzc.stepPeriodHR
                           * HWZC_MIN_INTERVAL_PCT / 100;
    if (minInterval < RT_HWZC_MIN_STEP_TICKS)
        minInterval = RT_HWZC_MIN_STEP_TICKS;

    if (interval < minInterval)
    {
        /* Noise — re-arm comparator, keep timeout running */
        pData->hwzc.noiseRejectCount++;
        pData->hwzc.rejectsThisStep++;
        uint8_t core = pData->hwzc.activeCore;
        HAL_ADC_ClearComparatorFlag(core);
        HAL_ADC_EnableComparatorIE(core);
        return;
    }

#if FEATURE_HWZC_VERIFY_READS
    /* Signal-coherence verification (post-203k restructure 2026-05-26):
     * The comparator-trigger ADC sample already passed the threshold (else the
     * IRQ wouldn't have fired) — re-reading AD1CH5DATA immediately would just
     * see that same sample. Useful verification requires WAITING for the next
     * ADC conversion (SCCP3 = 1 MHz → ~1 µs cadence) then reading a fresh
     * value. A real BEMF crossing stays on the crossed side; a phantom driven
     * by PWM ripple flips back within one sample period.
     *
     * Order matters: wait FIRST, then read. With HWZC_VERIFY_READS=1 we make
     * exactly one meaningful verification at ~1 µs cost (~7° at 200k). With
     * N>1 each additional iteration adds another 1 µs and one more read.
     *
     * Active across the full RPM range — HWZC_VERIFY_SKIP_ERPM=999999 keeps
     * this loop enabled at high RPM where it's most needed (PWM ripple at the
     * BEMF passband becomes harder to distinguish from real crossings). */
    if (pData->hwzc.stepPeriodHR > HWZC_VERIFY_SKIP_TICKS) {
        uint8_t v_core = pData->hwzc.activeCore;
        int8_t  v_pol  = commutationTable[pData->currentStep].zcPolarity;
        bool    v_rising = (v_pol > 0);
        uint16_t v_thresh = pData->bemf.zcThreshold;
        if (v_rising)
            v_thresh = (v_thresh + HWZC_CMP_DEADBAND < 4095)
                       ? v_thresh + HWZC_CMP_DEADBAND : 4095;
        else
            v_thresh = (v_thresh > HWZC_CMP_DEADBAND)
                       ? v_thresh - HWZC_CMP_DEADBAND : 0;

        for (uint8_t i = 0; i < HWZC_VERIFY_READS; i++) {
            /* Wait one ADC conversion period FIRST so the read sees a fresh
             * sample (not the comparator-trigger sample). 80 NOPs ≈ 800 ns
             * + read/branch ≈ 1 µs total at SCCP3 = 1 MHz. */
            for (uint8_t j = 0; j < 80; j++) __asm__ volatile("nop");

#if GARUDA_TARGET_AK512
            uint16_t adc_now = HAL_ADC_BemfChannelData(v_core);
#else
            uint16_t adc_now = (v_core == 1) ? AD1CH5DATA : AD2CH1DATA;
#endif
            bool crossed = v_rising ? (adc_now > v_thresh) : (adc_now < v_thresh);
            if (!crossed) {
                /* Phantom — BEMF returned to original side within one sample
                 * period. Re-arm comparator, keep timeout running. */
                pData->hwzc.noiseRejectCount++;
                pData->hwzc.rejectsThisStep++;
                HAL_ADC_ClearComparatorFlag(v_core);
                HAL_ADC_EnableComparatorIE(v_core);
                return;
            }
        }
    }
#endif

#if FEATURE_HWZC_SECTOR_PI
    /* Sector PI mode: ZC has passed noise rejection. Just record the
     * timestamp and let the autonomous SCCP1 timer (set in OnBlankingExpired
     * to fire at lastCommStamp + timerPeriod) drive commutation.
     * HWZC_OnPiPeriodExpired will run the PI math using this capture.
     *
     * Comparator IE is disabled — this is the "one accept per sector" rule
     * (matches ATA's ProcessBemfSample one-shot semantic). Further ZC IRQs
     * in this sector would be phantoms or chatter; let them sleep until
     * the next OnBlankingExpired re-enables. */
    pData->hwzc.lastCaptureHR = zcStamp;
    pData->hwzc.captureValid  = true;
    pData->hwzc.lastZcStamp   = zcStamp;   /* keep legacy field for diag */
    if (pData->hwzc.goodZcCount < 0xFFFE)
        pData->hwzc.goodZcCount++;
    pData->hwzc.missCount = 0;
    pData->hwzc.totalZcCount++;
    HAL_ADC_DisableComparatorIE(pData->hwzc.activeCore);
    return;
#endif

    /* Cancel the timeout timer */
    HAL_SCCP1_Stop();

    /* Update stepPeriodHR with seqlock (Rule 13).
     * Guard 1: only update IIR when this ZC follows exactly one commutation
     * (consecutive detection). Multi-step intervals include timeout periods,
     * not real motor periods — they contaminate the estimator.
     * Guard 2: freeze IIR for the first N ZCs after HWZC_Enable. A single
     * phantom or mistimed ZC during morph handoff used to pull stepPeriodHR
     * rapidly toward floor (observed in ~50% of morph attempts). Holding the
     * seeded stepPeriodHR for a few cycles lets the motor lock in at the
     * morph-exit rate before the IIR starts adapting. */
    if (pData->hwzc.stepsSinceLastHwZc == 1
        && pData->hwzc.goodZcCount >= HWZC_IIR_FREEZE_ZC_COUNT)
    {
        pData->hwzc.writeSeq++;  /* odd = write in progress */
        uint32_t newPeriod = (3 * pData->hwzc.stepPeriodHR + interval) / 4;
        if (newPeriod < RT_HWZC_MIN_STEP_TICKS)
            newPeriod = RT_HWZC_MIN_STEP_TICKS;
        pData->hwzc.stepPeriodHR = newPeriod;
        pData->hwzc.writeSeq++;  /* even = stable */
    }
    pData->hwzc.stepsSinceLastHwZc = 0;

    pData->hwzc.lastZcStamp = zcStamp;

    /* Track consecutive good ZCs (saturate to prevent stale re-seed) */
    if (pData->hwzc.goodZcCount < 0xFFFE)
        pData->hwzc.goodZcCount++;
    pData->hwzc.missCount = 0;
    pData->hwzc.totalZcCount++;

    /* Optimization B: use commDelay pre-computed by OnBlankingExpired.
     * Saves ~1.5 µs of division/interpolation inside the latency-critical ISR.
     * The cached value reflects stepPeriodHR at blanking-expiry, which is the
     * same IIR-filtered value used here except for the one IIR update above
     * (25% weight on the just-measured interval — negligible at steady state). */
    uint32_t commDelay = pData->hwzc.cachedCommDelay;
    if (commDelay < 10) commDelay = 10;

    /* Gate comparator OFF during COMM_PENDING. After accepting a real ZC, the
     * BEMF stays on the crossed side until the next commutation — any further
     * comparator IRQs in this window come from PWM ripple oscillating the
     * signal back across threshold, body-diode recovery, or cross-coupling,
     * and waste ~5µs of ISR time each (verify reads + interval check + re-arm)
     * for no useful purpose. At high RPM these spurious ISRs starve the 24kHz
     * ADC ISR, delaying threshold updates and degrading the NEXT sector's
     * detection. Re-enabled in OnBlankingExpired of next step. */
    HAL_ADC_DisableComparatorIE(pData->hwzc.activeCore);

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
    pData->hwzc.stepsSinceLastHwZc++;

    /* Set up ZC detection for the new step */
    HWZC_OnCommutation(pData);
}

#if FEATURE_HWZC_SECTOR_PI
/**
 * @brief Sector PI period expired — run PI math and commutate.
 *
 * Called from SCCP1 ISR (via _CCT1Interrupt) when phase == HWZC_WATCHING
 * AND FEATURE_HWZC_SECTOR_PI=1. In reactive mode this slot is HWZC_OnTimeout
 * (a missed-ZC handler); in PI mode it is the AUTONOMOUS commutation event.
 *
 * Algorithm (ported from PATA6847 sector_pi.c at the 225k milestone):
 *   1. Compute setValue from current advance schedule and timerPeriod.
 *      setValue = (advancePlus30Fp8 × timerPeriod) >> 8
 *      At lock, capValue ≈ setValue. With filter comp already aligning
 *      capValue with the true rotor ZC, the formula assumes:
 *          ZC happens at (advance + 30°)/60° of the inter-commutation
 *          interval, since commutation fires `advance°` early relative
 *          to the next rotor sector boundary.
 *   2. delta = capValue − setValue (signed phase error)
 *   3. delta = clamp(delta, ±timerPeriod >> CLAMP_SHIFT)
 *   4. integrator += delta >> KI_SHIFT
 *   5. timerPeriod = integrator + (delta >> KP_SHIFT)
 *   6. Advance commutation step, re-arm next sector.
 *
 * On capture-miss: skip PI update (timerPeriod unchanged), still commutate.
 * The motor is more robust to a missed sample than to a wrong correction.
 */
#if FEATURE_PLL_STARTUP
/* PLL-from-align blind startup tick (task #10, twin-prototyped).
 * Runs INSTEAD of the PI while pllStartActive: commutates on an
 * accelerating commanded schedule, consumes captures (discarding them
 * below the BEMF floor — phantom-proof), and hands over to the normal
 * PI after PLL_START_SYNC_CAPS consecutive plausible captures. */
static void HWZC_PllStartTick(volatile GARUDA_DATA_T *pData)
{
    uint32_t T = pData->hwzc.timerPeriod;
    uint32_t eCmd = (T > 0) ? HWZC_TICKS_TO_ERPM(T) : PLL_START_ERPM0;

    /* Capture floor: never trust captures below the active profile's HWZC
     * crossover (per-motor BEMF detectability), whichever is higher. */
    uint32_t pllFloorErpm = PLL_START_CAPTURE_FLOOR_ERPM;
    if (RT_HWZC_CROSSOVER_ERPM > pllFloorErpm)
        pllFloorErpm = RT_HWZC_CROSSOVER_ERPM;
    if (pData->hwzc.captureValid) {
        if (eCmd >= pllFloorErpm) {
            uint32_t cap = pData->hwzc.lastCaptureHR
                         - pData->hwzc.lastCommStamp;   /* modular */
            /* CONSISTENCY gate (twin iteration #5). A locked rotor puts the
             * ZC at the SAME sector fraction every sector; phantoms land
             * randomly. The old absolute window (0.30T..0.75T) was fragile:
             * load angle / timing advance / RC filter lag all shift the
             * true crossing (twin saw a fully-locked rotor pinned at the
             * blanking edge ~0.14T and never synced). Gate instead on
             * sector-plausible (T/8..7T/8, outside blanking, inside sector)
             * AND stable vs the previous capture (|Δ| < T/4 — wide enough
             * for the rising/falling polarity lag split, narrow enough that
             * a 6-streak of random phantoms is ~3% likely). */
            uint32_t prevCap = pData->hwzc.pllPrevCap;
            pData->hwzc.pllPrevCap = cap;
            if (cap > (T >> 3) && cap < (T - (T >> 3))) {
                uint32_t d = (cap > prevCap) ? cap - prevCap : prevCap - cap;
                if (prevCap != 0u && d < (T >> 2)) {
                    if (++pData->hwzc.pllStartGood >= PLL_START_SYNC_CAPS) {
                        /* HANDOVER: declared synced; leave captureValid set
                         * so the normal PI consumes THIS capture next event. */
                        pData->hwzc.pllStartActive = 0;
                        pData->timing.zcSynced = true;
                        pData->timing.goodZcCount = (uint16_t)RT_ZC_SYNC_THRESHOLD;
                        goto pll_commutate;
                    }
                } else {
                    pData->hwzc.pllStartGood = 1;   /* plausible, streak restarts */
                }
            } else {
                pData->hwzc.pllStartGood = 0;
                pData->hwzc.pllPrevCap = 0;
            }
        }
        pData->hwzc.captureValid = false;   /* consume in blind phase */
    } else if (eCmd >= pllFloorErpm
               && pData->hwzc.pllStartGood > 0) {
        pData->hwzc.pllStartGood = 0;       /* silence resets the streak */
        pData->hwzc.pllPrevCap = 0;
    }

    /* blind accel schedule toward the target */
    if (eCmd < PLL_START_TARGET_ERPM) {
        /* Sector time in seconds = T / SCCP_CLOCK_HZ (HR tick = 10 ns).
         * NOT /1e9 — that unit slip made the schedule accelerate exactly
         * 10× slower than commanded (twin iteration #4 root cause). */
        uint32_t eNew = eCmd + (uint32_t)(((uint64_t)PLL_START_ACCEL_ERPM_PER_S
                                           * T) / (uint64_t)SCCP_CLOCK_HZ);
        if (eNew <= eCmd) eNew = eCmd + 1u;
        if (eNew > PLL_START_TARGET_ERPM) eNew = PLL_START_TARGET_ERPM;
        T = HWZC_ERPM_TO_TICKS(eNew);       /* symmetric: 1e9/e */
        pData->hwzc.timerPeriod = T;
        pData->hwzc.integrator  = (int32_t)T;
#if FEATURE_HWZC_PI_FLOAT
        pData->hwzc.integratorF = (float)T;
#endif
        pData->hwzc.writeSeq++;             /* Rule 13 seqlock */
        pData->hwzc.stepPeriodHR = T;
        pData->hwzc.writeSeq++;
    }

pll_commutate:
    COMMUTATION_AdvanceStep(pData);
    pData->hwzc.totalCommCount++;
    pData->hwzc.commSeq++;
    pData->hwzc.stepsSinceLastHwZc = 0;
    HWZC_OnCommutation(pData);
}
#endif /* FEATURE_PLL_STARTUP */

void HWZC_OnPiPeriodExpired(volatile GARUDA_DATA_T *pData)
{
#if FEATURE_PLL_STARTUP
    if (pData->hwzc.pllStartActive) { HWZC_PllStartTick(pData); return; }
#endif
    /* Compute current torque-advance (interpolated from RT_TIMING_ADV_MAX_DEG)
     * — same schedule the reactive path uses, just consumed differently. */
    uint16_t advDeg;
    uint32_t T = pData->hwzc.timerPeriod;
#if FEATURE_TIMING_ADVANCE
    uint32_t eRPM = (T > 0) ? HWZC_TICKS_TO_ERPM(T) : 0;
    if (eRPM <= RT_RAMP_TARGET_ERPM)
        advDeg = TIMING_ADVANCE_MIN_DEG;
    else if (eRPM >= RT_TIMING_ADV_FULL_ERPM)
        advDeg = RT_TIMING_ADV_MAX_DEG;
    else {
        uint32_t range = RT_TIMING_ADV_FULL_ERPM - RT_RAMP_TARGET_ERPM;
        uint32_t pos = eRPM - RT_RAMP_TARGET_ERPM;
        advDeg = TIMING_ADVANCE_MIN_DEG +
            (uint16_t)((uint32_t)(RT_TIMING_ADV_MAX_DEG - TIMING_ADVANCE_MIN_DEG)
                       * pos / range);
    }
#else
    advDeg = 0;
#endif
    pData->hwzc.dbgAdvanceDeg = advDeg;

    /* advancePlus30Fp8 = (30 + advDeg) × 256 / 60. Max value at advDeg=30 is 256;
     * at advDeg=0 is 128. Fits comfortably in uint16. */
    uint16_t advFp8 = (uint16_t)(((30u + (uint32_t)advDeg) * 256u) / 60u);

#if FEATURE_HWZC_PI_DEFENSIVE
    /* Track whether THIS event has a capture (BEFORE the gate below
     * consumes captureValid). Used by the defensive-mode streak counters
     * regardless of whether the capture passes the 0<cap<T range check.
     * A rejected capture (cap > T) is NOT silence — only "captureValid==
     * false at entry" counts toward the defensive trigger. */
    bool hadCaptureThisEvent = pData->hwzc.captureValid;
    if (!hadCaptureThisEvent)
        pData->hwzc.dbgPiNoCap++;        /* bring-up diag: silent PI event */
#endif

    /* Run PI if we have a fresh capture from this sector */
    if (pData->hwzc.captureValid) {
        /* capValue = elapsed HR ticks FROM THE START OF THIS SECTOR (i.e. the
         * most recent commutation = lastCommStamp) TO the captured ZC.
         *
         * Timing: OnPiPeriodExpired fires BEFORE OnCommutation runs. So at
         * this moment, lastCommStamp still holds T_N (start of the sector
         * that's ending), and lastCaptureHR ≈ T_N + T/2 (ZC at midpoint).
         * After PI math, OnCommutation shifts prevCommHR ← lastCommStamp
         * and lastCommStamp ← now. Modular subtraction handles SCCP2 wrap
         * (every ~42 s at 100 MHz). */
        uint32_t capValue = pData->hwzc.lastCaptureHR - pData->hwzc.lastCommStamp;
        uint32_t setValue = ((uint32_t)advFp8 * T) >> 8;

        /* bring-up diag: capture position in sector, even if rejected —
         * split by ZC polarity (even sectors 0/2/4 = rising comparator path,
         * odd 1/3/5 = falling RC-filtered SW path). */
        if (T > 0) {
            uint16_t pm = (uint16_t)(((uint64_t)capValue * 1000u) / T);
            if (pData->currentStep & 1)
                pData->hwzc.dbgPiCrossSector = pm;   /* falling-sector cap */
            else
                pData->hwzc.dbgLastCapPm = pm;       /* rising-sector cap */
        }

        /* Cross-sector reject (same gate in both float and int paths). */
        if (capValue > T) {
            pData->hwzc.dbgPiMissCount++;
            pData->hwzc.captureValid = false;
            goto pi_commutate;
        }

        int32_t delta = (int32_t)capValue - (int32_t)setValue;

        /* Per-capture delta clamp — ASYMMETRIC at high RPM. */
        int32_t posDeltaCap, negDeltaCap;
        if (T < HWZC_PI_CLAMP_HIGH_TICKS) {
            posDeltaCap = (int32_t)(T >> HWZC_PI_DELTA_CLAMP_SHIFT_HIGH);
            negDeltaCap = (int32_t)(T >> HWZC_PI_NEG_DELTA_CLAMP_SHIFT_HIGH);
        } else {
            posDeltaCap = (int32_t)(T >> HWZC_PI_DELTA_CLAMP_SHIFT);
            negDeltaCap = posDeltaCap;
        }
#if FEATURE_HWZC_HANDOFF_DAMP
        /* Hand-off damp: during the first window after CL entry, hard-limit how
         * much a single capture can SHRINK the period (negative delta only), so a
         * too-early first capture can't collapse to the half-period phantom. Legit
         * acceleration is gradual and proceeds at the damped rate; the runaway is
         * blocked. negative delta = capture earlier than setpoint = period shrink. */
        if (pData->hwzc.handoffDamp > 0) {
            pData->hwzc.handoffDamp--;
            int32_t damped = (int32_t)(T >> HWZC_HANDOFF_NEG_SHIFT);
            if (damped < 1) damped = 1;
            if (negDeltaCap > damped) negDeltaCap = damped;
        }
#endif
        if (delta >  posDeltaCap) delta =  posDeltaCap;
        if (delta < -negDeltaCap) delta = -negDeltaCap;
        pData->hwzc.dbgPiDelta = (int16_t)(delta > 32767 ? 32767
                                          : delta < -32768 ? -32768
                                          : delta);

#if FEATURE_HWZC_PI_FLOAT
        /* --- FLOAT PI (Phase 1) --- */
        float deltaF = (float)delta;
        (void)T;

  #if FEATURE_HWZC_PI_DEFENSIVE
        /* In defensive mode, ignore the capture's delta — captures during
         * a silence-recovery window can be misleading. The slow-down
         * logic below the if/else block walks T larger. */
        if (!pData->hwzc.piDefActive) {
  #endif
            pData->hwzc.integratorF += HWZC_PI_KI_FLOAT * deltaF;
            if (pData->hwzc.integratorF < (float)RT_HWZC_MIN_STEP_TICKS)
                pData->hwzc.integratorF = (float)RT_HWZC_MIN_STEP_TICKS;

            float newPerF = pData->hwzc.integratorF + HWZC_PI_KP_FLOAT * deltaF;
            if (newPerF < (float)RT_HWZC_MIN_STEP_TICKS)
                newPerF = (float)RT_HWZC_MIN_STEP_TICKS;

            pData->hwzc.timerPeriod = (uint32_t)newPerF;
            pData->hwzc.integrator  = (uint32_t)pData->hwzc.integratorF;
  #if FEATURE_HWZC_PI_DEFENSIVE
        }
  #endif
#else
        /* --- ORIGINAL INTEGER BIT-SHIFT PI --- */
        /* Integrator update — slow drift correction. */
        int32_t newInt = (int32_t)pData->hwzc.integrator + (delta >> HWZC_PI_KI_SHIFT);
        if (newInt < (int32_t)RT_HWZC_MIN_STEP_TICKS)
            newInt = (int32_t)RT_HWZC_MIN_STEP_TICKS;
        if (newInt < 0) newInt = (int32_t)RT_HWZC_MIN_STEP_TICKS;
        pData->hwzc.integrator = (uint32_t)newInt;

        /* Period update — fast dynamics. */
        int32_t newPer = (int32_t)pData->hwzc.integrator + (delta >> HWZC_PI_KP_SHIFT);
        if (newPer < (int32_t)RT_HWZC_MIN_STEP_TICKS)
            newPer = (int32_t)RT_HWZC_MIN_STEP_TICKS;
        if (newPer < 0) newPer = (int32_t)RT_HWZC_MIN_STEP_TICKS;
        pData->hwzc.timerPeriod = (uint32_t)newPer;
#endif /* FEATURE_HWZC_PI_FLOAT */

#if FEATURE_HWZC_ABS_FLOOR
        /* Absolute operating-point floor — block the harmonic false-lock that
         * the relative gates can't (cumulative ratchet → phantom). The rotor
         * cannot spin faster than the no-load physics for the present duty/Vbus,
         * so the commanded period cannot be shorter than P_ff/(overspeed margin).
         * Reuses the FF-seed no-load-period formula (see HWZC_Enable). */
        {
            float dutyFrac = (float)pData->duty / (float)LOOPTIME_TCY;
            float vbus_v   = (float)pData->vbusRaw * VBUS_SCALE_V_PER_COUNT;
            float lambdaPm = (float)gspParams.focKeUvSRad;
            if (dutyFrac > HWZC_ABS_FLOOR_MIN_DUTYFRAC && vbus_v > 6.0f
                && lambdaPm > 0.0f) {
                float P_ff = (181.380f * lambdaPm) / (vbus_v * dutyFrac);
                uint32_t floorP = (uint32_t)(P_ff *
                    (100.0f / (float)HWZC_ABS_FLOOR_OVERSPEED_PCT));
                /* SHRINK-ONLY: clamp only when the PI is DRIVING the period
                 * below the floor (commutating FASTER — the phantom). A period
                 * already below the floor but GROWING (T_new > T_entry) is a
                 * legitimate high-speed decel coasting down through the floor —
                 * the rotor really is that fast — so let it track. The rotor
                 * cannot ACCELERATE at idle duty, so a shrink below floor is
                 * provably a false-lock. */
                if (pData->hwzc.timerPeriod < floorP
                    && pData->hwzc.timerPeriod < T) {
                    pData->hwzc.timerPeriod = floorP;
  #if FEATURE_HWZC_PI_FLOAT
                    if (pData->hwzc.integratorF < (float)floorP)
                        pData->hwzc.integratorF = (float)floorP;
  #endif
                    if (pData->hwzc.integrator < floorP)
                        pData->hwzc.integrator = floorP;
                }
            }
        }
#endif

        /* Mirror to stepPeriodHR so eRPM telemetry (and the reactive
         * path's diagnostics) reflect the PI-controlled rotor period. */
        pData->hwzc.writeSeq++;
        pData->hwzc.stepPeriodHR = pData->hwzc.timerPeriod;
        pData->hwzc.writeSeq++;

        pData->hwzc.captureValid = false;
        pData->hwzc.dbgPiCaptureCount++;
        {
            uint8_t s = pData->currentStep;
            if (s < 6) pData->hwzc.dbgPiCapBySector[s]++;
        }
        if (pData->hwzc.goodZcCount < 0xFFFE) /* keep goodZc growing for telem */
            pData->hwzc.goodZcCount++;
    } else {
        /* No capture this sector — period unchanged. Robust by design.
         * Track which sector is missing so we can confirm AD1 vs AD2 split. */
        pData->hwzc.dbgPiMissCount++;
        pData->hwzc.missCount++;
        pData->hwzc.totalMissCount++;
        {
            uint8_t s = pData->currentStep;
            if (s < 6) pData->hwzc.dbgPiMissBySector[s]++;
        }
    }

#if FEATURE_HWZC_PI_DEFENSIVE
    /* --- DEFENSIVE PI streak tracking + slow-down ---
     * Only true silence (no capture event at all) counts toward the
     * defensive trigger. A capture that arrived but was rejected by the
     * range gate (cap > T) is NOT silence — those are transient
     * mismatches the PI handles normally. */
    if (hadCaptureThisEvent) {
        pData->hwzc.piDefMissStreak = 0;
        if (pData->hwzc.piDefGoodStreak < 0xFFFF)
            pData->hwzc.piDefGoodStreak++;
    } else {
        if (pData->hwzc.piDefMissStreak < 0xFFFF)
            pData->hwzc.piDefMissStreak++;
        pData->hwzc.piDefGoodStreak = 0;
    }

    /* Enter / exit defensive mode (hysteresis: 6 in / 2 out) */
    if (!pData->hwzc.piDefActive &&
        pData->hwzc.piDefMissStreak >= HWZC_PI_DEFENSIVE_TRIGGER) {
        pData->hwzc.piDefActive = 1;
        if (pData->hwzc.piDefEntryCount < 0xFFFFFFFF)
            pData->hwzc.piDefEntryCount++;
    } else if (pData->hwzc.piDefActive &&
               pData->hwzc.piDefGoodStreak >= HWZC_PI_DEFENSIVE_EXIT) {
        pData->hwzc.piDefActive = 0;
    }

    /* While defensive, walk integratorF (and timerPeriod) larger to
     * deliberately slow the bridge so the rotor can catch up. */
    if (pData->hwzc.piDefActive) {
  #if FEATURE_HWZC_PI_FLOAT
        pData->hwzc.integratorF *= (1.0f + (float)HWZC_PI_DEFENSIVE_GROW_PCT * 0.01f);
        pData->hwzc.timerPeriod = (uint32_t)pData->hwzc.integratorF;
        pData->hwzc.integrator  = (uint32_t)pData->hwzc.integratorF;
  #else
        /* Integer-PI fallback: scale by (100 + grow) / 100 */
        uint32_t scaled = (pData->hwzc.integrator
                           * (100UL + HWZC_PI_DEFENSIVE_GROW_PCT)) / 100UL;
        pData->hwzc.integrator  = scaled;
        pData->hwzc.timerPeriod = scaled;
  #endif
        /* Mirror to stepPeriodHR so telemetry stays consistent */
        pData->hwzc.writeSeq++;
        pData->hwzc.stepPeriodHR = pData->hwzc.timerPeriod;
        pData->hwzc.writeSeq++;
    }
#endif /* FEATURE_HWZC_PI_DEFENSIVE */

pi_commutate:
    /* Slow-IIR update of stepPeriodForFilterComp — this is the period used
     * in ω·τ for HWZC_ApplyFilterComp. Decoupled from timerPeriod to break
     * the positive-feedback loop where a single PI period-shrink → larger
     * ω·τ → larger CMPLO offset → earlier comparator fire → more PI shrink.
     *
     * Shift 11 → 1/2048 per sector ≈ 11 ms time constant at 190k eRPM.
     * Slow vs PI per-sector dynamics (~1 ms) → loop gain << 1.
     * Fast vs real motor accel (~50 ms) → tracks real changes adequately. */
    if (pData->hwzc.stepPeriodForFilterComp == 0) {
        pData->hwzc.stepPeriodForFilterComp = pData->hwzc.timerPeriod;
    } else {
        int32_t pDelta = (int32_t)pData->hwzc.timerPeriod
                       - (int32_t)pData->hwzc.stepPeriodForFilterComp;
        pData->hwzc.stepPeriodForFilterComp =
            (uint32_t)((int32_t)pData->hwzc.stepPeriodForFilterComp
                       + (pDelta >> 11));
    }

    /* Advance to next sector — autonomous timer drives this regardless
     * of capture state. OnCommutation re-arms BLANKING + comparator. */
    COMMUTATION_AdvanceStep(pData);
    pData->hwzc.totalCommCount++;
    pData->hwzc.commSeq++;
    pData->hwzc.stepsSinceLastHwZc = 0;
    HWZC_OnCommutation(pData);

#if FEATURE_SPEED_PI
    /* Speed PI runs after the timing PI has updated stepPeriodHR.
     * Per-ZC interval-based: this fires every sector (variable rate
     * scaling with rotor speed). No-op until SPEED_PI_Enable() called
     * by CL state entry. */
    SPEED_PI_OnZcEvent(pData);
#endif
}
#endif /* FEATURE_HWZC_SECTOR_PI */

/**
 * @brief ZC timeout — no crossing detected within 2x step period.
 * Called from SCCP1 ISR when phase == HWZC_WATCHING.
 * Either forces a step or falls back to software ZC on miss-limit.
 */
void HWZC_OnTimeout(volatile GARUDA_DATA_T *pData)
{
    /* Capture diagnostics: what did the ADC channel actually see? */
    uint8_t core = pData->hwzc.activeCore;
#if GARUDA_TARGET_AK512
    /* Per-phase channels: VA = AD1CH1, VB = AD1CH2, VC = AD2CH2 */
    if (core == FLOATING_PHASE_A)
    {
        pData->hwzc.dbgTimeoutAdcVal = AD1CH1DATA;
        pData->hwzc.dbgTimeoutThresh = AD1CH1CMPLO;
        pData->hwzc.dbgTimeoutCmpmod = AD1CH1CON2bits.CMPMOD;
        pData->hwzc.dbgTimeoutCmpstat = AD1CMPSTATbits.CH1FLG;
    }
    else if (core == FLOATING_PHASE_B)
    {
        pData->hwzc.dbgTimeoutAdcVal = AD1CH2DATA;
        pData->hwzc.dbgTimeoutThresh = AD1CH2CMPLO;
        pData->hwzc.dbgTimeoutCmpmod = AD1CH2CON2bits.CMPMOD;
        pData->hwzc.dbgTimeoutCmpstat = AD1CMPSTATbits.CH2FLG;
    }
    else
    {
        pData->hwzc.dbgTimeoutAdcVal = AD2CH2DATA;
        pData->hwzc.dbgTimeoutThresh = AD2CH2CMPLO;
        pData->hwzc.dbgTimeoutCmpmod = AD2CH2CON2bits.CMPMOD;
        pData->hwzc.dbgTimeoutCmpstat = AD2CMPSTATbits.CH2FLG;
    }
#else
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
#endif /* GARUDA_TARGET_AK512 */
    pData->hwzc.dbgTimeoutCore = core;
    pData->hwzc.dbgTimeoutStep = pData->currentStep;
    pData->hwzc.dbgTimeoutBemfThresh = pData->bemf.zcThreshold;

    pData->hwzc.missCount++;
    pData->hwzc.totalMissCount++;
    pData->hwzc.stepsSinceLastHwZc++;

    if (pData->hwzc.goodZcCount > 0)
        pData->hwzc.goodZcCount--;

    if (pData->hwzc.missCount >= HWZC_MISS_LIMIT)
    {
        /* Too many misses — fall back to software ZC.
         * Clear goodZcCount to prevent stale synced re-seed.
         * Latch disable to prevent re-enable cycling (preserves diagnostics). */
        pData->hwzc.goodZcCount = 0;
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

#if HWZC_USE_SW_COMPARE
/**
 * @brief Software ZC detection on a mid-ON ADC sample.
 * Called from the 24 kHz ADC ISR after bemfRaw is populated. Replaces the
 * 1 MHz hardware digital comparator path — mid-ON sampling avoids the
 * 48 kHz phantom-crossing issue where PWM ripple through the board's
 * 5.5 kHz RC filter constantly crosses the threshold.
 *
 * Runs the exact same accept logic as HWZC_OnZcDetected (plausibility gate,
 * IIR update, commDelay scheduling) minus the HW comparator re-arm. State
 * machine (BLANKING → WATCHING → COMM_PENDING) is identical.
 *
 * No-op unless enabled && phase == HWZC_WATCHING && sample is valid.
 */
void HWZC_OnSoftwareSample(volatile GARUDA_DATA_T *pData)
{
    if (!pData->hwzc.enabled) return;
    if (pData->hwzc.phase != HWZC_WATCHING) return;
    if (!pData->bemf.bemfSampleValid) return;   /* AD2 pin-mux settling */

    uint8_t step = pData->currentStep;
    int8_t  pol = commutationTable[step].zcPolarity;
    bool    risingZc = (pol > 0);

    /* Apply deadband/hysteresis matching the HW comparator path */
    uint16_t thresh = pData->bemf.zcThreshold;
    if (risingZc)
        thresh = (thresh + HWZC_CMP_DEADBAND < 4095) ? thresh + HWZC_CMP_DEADBAND : 4095;
    else
        thresh = (thresh > HWZC_CMP_DEADBAND) ? thresh - HWZC_CMP_DEADBAND : 0;

    uint16_t bemfRaw = pData->bemf.bemfRaw;
    bool crossed = risingZc ? (bemfRaw > thresh) : (bemfRaw < thresh);
    if (!crossed) return;

    uint32_t zcStamp = HAL_SCCP2_ReadTimestamp();

    /* First ZC after HWZC_Enable: bypass plausibility gate + skip IIR.
     * See HWZC_OnZcDetected for rationale. */
    if (pData->hwzc.firstZcAfterEnable) {
        pData->hwzc.firstZcAfterEnable = false;
        HAL_SCCP1_Stop();
        pData->hwzc.lastZcStamp = zcStamp;
        pData->hwzc.stepsSinceLastHwZc = 0;
        if (pData->hwzc.goodZcCount < 0xFFFE)
            pData->hwzc.goodZcCount++;
        pData->hwzc.missCount = 0;
        pData->hwzc.totalZcCount++;

        uint32_t commDelay;
#if FEATURE_TIMING_ADVANCE
        {
            uint32_t eRPM = HWZC_TICKS_TO_ERPM(pData->hwzc.stepPeriodHR);
            uint16_t advDeg;
            if (eRPM <= RT_RAMP_TARGET_ERPM)
                advDeg = TIMING_ADVANCE_MIN_DEG;
            else if (eRPM >= RT_TIMING_ADV_FULL_ERPM)
                advDeg = RT_TIMING_ADV_MAX_DEG;
            else {
                uint32_t range = RT_TIMING_ADV_FULL_ERPM - RT_RAMP_TARGET_ERPM;
                uint32_t pos   = eRPM - RT_RAMP_TARGET_ERPM;
                advDeg = TIMING_ADVANCE_MIN_DEG +
                    (uint16_t)((uint32_t)(RT_TIMING_ADV_MAX_DEG - TIMING_ADVANCE_MIN_DEG)
                               * pos / range);
            }
            commDelay = pData->hwzc.stepPeriodHR * (30 - advDeg) / 60;
            pData->hwzc.dbgAdvanceDeg = advDeg;
        }
#else
        commDelay = pData->hwzc.stepPeriodHR / 2;
#endif
        if (commDelay < 10) commDelay = 10;
        pData->hwzc.phase = HWZC_COMM_PENDING;
        HAL_SCCP1_StartOneShot(commDelay);
        return;
    }

    /* Plausibility gate: reject intervals shorter than 70% of stepPeriodHR,
     * with RT_HWZC_MIN_STEP_TICKS as hard floor (physical max eRPM). */
    uint32_t interval = zcStamp - pData->hwzc.lastZcStamp;
    uint32_t minInterval = pData->hwzc.stepPeriodHR
                           * HWZC_MIN_INTERVAL_PCT / 100;
    if (minInterval < RT_HWZC_MIN_STEP_TICKS)
        minInterval = RT_HWZC_MIN_STEP_TICKS;

    if (interval < minInterval) {
        pData->hwzc.noiseRejectCount++;
        pData->hwzc.rejectsThisStep++;
        return;  /* stay in WATCHING; timeout still armed */
    }

    /* Cancel timeout */
    HAL_SCCP1_Stop();

    /* Update IIR: see HWZC_OnZcDetected for the freeze-first-N rationale. */
    if (pData->hwzc.stepsSinceLastHwZc == 1
        && pData->hwzc.goodZcCount >= HWZC_IIR_FREEZE_ZC_COUNT) {
        pData->hwzc.writeSeq++;
        uint32_t newPeriod = (3 * pData->hwzc.stepPeriodHR + interval) / 4;
        if (newPeriod < RT_HWZC_MIN_STEP_TICKS)
            newPeriod = RT_HWZC_MIN_STEP_TICKS;
        pData->hwzc.stepPeriodHR = newPeriod;
        pData->hwzc.writeSeq++;
    }
    pData->hwzc.stepsSinceLastHwZc = 0;
    pData->hwzc.lastZcStamp = zcStamp;

    if (pData->hwzc.goodZcCount < 0xFFFE)
        pData->hwzc.goodZcCount++;
    pData->hwzc.missCount = 0;
    pData->hwzc.totalZcCount++;

    /* Schedule commutation (same advance/delay math as HW path) */
    uint32_t commDelay;
#if FEATURE_TIMING_ADVANCE
    {
        uint32_t eRPM = HWZC_TICKS_TO_ERPM(pData->hwzc.stepPeriodHR);
        uint16_t advDeg;
        if (eRPM <= RT_RAMP_TARGET_ERPM)
            advDeg = TIMING_ADVANCE_MIN_DEG;
        else if (eRPM >= RT_TIMING_ADV_FULL_ERPM)
            advDeg = RT_TIMING_ADV_MAX_DEG;
        else {
            uint32_t range = RT_TIMING_ADV_FULL_ERPM - RT_RAMP_TARGET_ERPM;
            uint32_t pos   = eRPM - RT_RAMP_TARGET_ERPM;
            advDeg = TIMING_ADVANCE_MIN_DEG +
                (uint16_t)((uint32_t)(RT_TIMING_ADV_MAX_DEG - TIMING_ADVANCE_MIN_DEG)
                           * pos / range);
        }
        commDelay = pData->hwzc.stepPeriodHR * (30 - advDeg) / 60;
        pData->hwzc.dbgAdvanceDeg = advDeg;
    }
#else
    commDelay = pData->hwzc.stepPeriodHR / 2;
#endif
    if (commDelay < 10) commDelay = 10;

    pData->hwzc.phase = HWZC_COMM_PENDING;
    HAL_SCCP1_StartOneShot(commDelay);
}
#endif /* HWZC_USE_SW_COMPARE */

#endif /* FEATURE_ADC_CMP_ZC */
