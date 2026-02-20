/**
 * @file garuda_service.c
 *
 * @brief ESC state machine and ADC ISR.
 *
 * State machine (driven from ADC ISR at PWM rate):
 *   IDLE → ARMED (throttle=0 for 500ms) → ALIGN → OL_RAMP
 *   → (Phase 2: CLOSED_LOOP)
 *
 * Timer1 ISR: 100us tick for heartbeat, board service, and commutation timing.
 *
 * Component: GARUDA SERVICE
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "garuda_service.h"
#include "garuda_types.h"
#include "garuda_config.h"
#include "garuda_calc_params.h"
#include "hal/hal_adc.h"
#include "hal/hal_pwm.h"
#include "hal/board_service.h"
#include "hal/port_config.h"
#include "motor/commutation.h"
#include "motor/startup.h"
#if FEATURE_BEMF_CLOSED_LOOP
#include "motor/bemf_zc.h"
#endif
#if FEATURE_ADC_CMP_ZC
#include "motor/hwzc.h"
#include "hal/hal_timer.h"
#endif

#if FEATURE_LEARN_MODULES
#include "learn/learn_service.h"
#endif

#include "x2cscope/diagnostics.h"

/* Global ESC runtime data — volatile: shared between ISRs and main loop */
volatile GARUDA_DATA_T garudaData;

/* Heartbeat LED counter */
static uint16_t heartbeatCounter = 0;
/* Sub-counter for 1ms system tick from 100us Timer1 */
static uint8_t msSubCounter = 0;

#if FEATURE_BEMF_CLOSED_LOOP
/* File-scope statics for ZC — only accessed from ADC ISR, NOT Timer1 ISR */
static uint16_t adcIsrTick = 0;
static ESC_STATE_T prevAdcState = ESC_IDLE;
#endif

/**
 * @brief Initialize ESC service data to safe defaults.
 */
void GARUDA_ServiceInit(void)
{
    garudaData.state = ESC_IDLE;
    garudaData.throttle = 0;
    garudaData.currentStep = 0;
    garudaData.direction = DIRECTION_DEFAULT;
    garudaData.duty = 0;
    garudaData.vbusRaw = 0;
    garudaData.potRaw = 0;
    garudaData.faultCode = FAULT_NONE;
    garudaData.alignCounter = 0;
    garudaData.rampStepPeriod = INITIAL_STEP_PERIOD;
    garudaData.rampCounter = 0;
    garudaData.systemTick = 0;
    garudaData.armCounter = 0;
    garudaData.runCommandActive = false;
    garudaData.desyncRestartAttempts = 0;
    garudaData.recoveryCounter = 0;

#if FEATURE_HW_OVERCURRENT
    garudaData.ibusRaw = 0;
    garudaData.ibusMax = 0;
    garudaData.clpciTripCount = 0;
#endif

    garudaData.bemf.bemfRaw = 0;
    garudaData.bemf.zcThreshold = 0;
    garudaData.bemf.zeroCrossDetected = false;
    garudaData.bemf.cmpPrev = 0xFF;
    garudaData.bemf.cmpExpected = 0;
    garudaData.bemf.filterCount = 0;
    garudaData.bemf.ad2SettleCount = 0;
    garudaData.bemf.bemfSampleValid = true;

    garudaData.timing.stepPeriod = 0;
    garudaData.timing.lastCommTick = 0;
    garudaData.timing.lastZcTick = 0;
    garudaData.timing.prevZcTick = 0;
    garudaData.timing.zcInterval = 0;
    garudaData.timing.commDeadline = 0;
    garudaData.timing.forcedCountdown = 0;
    garudaData.timing.goodZcCount = 0;
    garudaData.timing.consecutiveMissedSteps = 0;
    garudaData.timing.stepsSinceLastZc = 0;
    for (uint8_t i = 0; i < 6; i++)
        garudaData.timing.stepMissCount[i] = 0;
    garudaData.timing.risingZcWorks = false;
    garudaData.timing.fallingZcWorks = false;
    garudaData.timing.zcSynced = false;
    garudaData.timing.deadlineActive = false;
    garudaData.timing.hasPrevZc = false;
#if FEATURE_BEMF_INTEGRATION || FEATURE_ADC_CMP_ZC
    garudaData.timing.deadlineIsZc = false;
#endif

    /* ISR priority setup */
#if FEATURE_ADC_CMP_ZC
    _AD1CH0IP = 6;      /* ADC ISR lowered from 7 to 6 when HW ZC available */
    _AD1CMP5IP = 7;     /* AD1 comparator CH5: highest priority */
    _AD2CMP1IP = 7;     /* AD2 comparator CH1: highest priority */
    _CCT1IP = 7;        /* SCCP1 timer: highest priority */
#endif

    /* Enable ADC interrupt to start the control loop */
    GARUDA_ClearADCIF();
    GARUDA_EnableADCInterrupt();

#if FEATURE_ADC_CMP_ZC
    HWZC_Init(&garudaData);
    HAL_ADC_InitHighSpeedBEMF();
    HAL_SCCP1_Init();
    HAL_SCCP2_Init();
    HAL_SCCP3_InitPeriodic(HWZC_SCCP3_PERIOD);  /* Start high-speed ADC trigger */
#endif

#if FEATURE_LEARN_MODULES
    LEARN_ServiceInit(&garudaData);
#endif
}

/**
 * @brief ADC ISR — runs at PWM rate (24kHz).
 * Reads BEMF/Vbus, runs state machine, updates PWM.
 * Phase 2: sole commutation owner for CLOSED_LOOP state
 * (bypassed when DIAGNOSTIC_MANUAL_STEP=1).
 */
void __attribute__((__interrupt__, no_auto_psv)) GARUDA_ADC_INTERRUPT(void)
{
    /* Read all ADC buffers. MUST read AD1CH0DATA first — interrupt source.
     * AD1CH0 = Phase B voltage (RB8/AD1AN11, always sampled)
     * AD2CH0 = Phase A or C voltage (muxed per commutation step)
     * AD2CH0DATA is safe: same PWM trigger, same SAMC, ISR latency guarantees done. */
    uint16_t phaseB_val = ADCBUF_PHASE_B;
    uint16_t phaseAC_val = ADCBUF_PHASE_AC;
    garudaData.vbusRaw = ADCBUF_VBUS;
    garudaData.potRaw = ADCBUF_POT;

    garudaData.throttle = garudaData.potRaw;

#if FEATURE_BEMF_CLOSED_LOOP
    /* Capture entry state for transition detection. Must be saved before
     * any state changes (morph→CL etc.) so the NEXT tick sees the transition. */
    ESC_STATE_T entryState = garudaData.state;

    /* Duty-proportional ZC threshold = virtual neutral point.
     * neutral = Vbus * duty / (2 * LOOPTIME_TCY)
     * Approximated as (vbusRaw * duty) >> 18 for 24kHz PWM.
     *
     * Asymmetric IIR smoothing prevents threshold-BEMF mismatch during
     * rapid duty transients.  Threshold tracks UP fast (motor accelerating,
     * BEMF rising) but DOWN slowly (motor coasting, BEMF stays high).
     * Rise: tau ~0.33ms (8 ticks).  Fall: tau ~10.7ms (256 ticks). */
    {
        static uint16_t zcThreshSmooth = 0;
        uint16_t rawThresh = (uint16_t)(
            ((uint32_t)garudaData.vbusRaw * garudaData.duty) >> ZC_DUTY_THRESHOLD_SHIFT);

        if (garudaData.state == ESC_CLOSED_LOOP
#if FEATURE_SINE_STARTUP
            || (garudaData.state == ESC_MORPH
                && garudaData.morph.subPhase == MORPH_HIZ)
#endif
           )
        {
            if (prevAdcState != ESC_CLOSED_LOOP
#if FEATURE_SINE_STARTUP
                && !(garudaData.state == ESC_MORPH
                     && garudaData.morph.subPhase == MORPH_HIZ
                     && prevAdcState == ESC_MORPH)
#endif
               )
            {
                zcThreshSmooth = rawThresh;  /* Seed on CL entry */
            }
            else if (rawThresh >= zcThreshSmooth)
            {
                /* Rising — track fast (tau ~8 ticks = 0.33ms) */
                zcThreshSmooth += ((uint16_t)(rawThresh - zcThreshSmooth) + 4) >> 3;
            }
            else
            {
                /* Falling — track slow (tau ~256 ticks = 10.7ms) */
                uint16_t decay = ((uint16_t)(zcThreshSmooth - rawThresh) + 128) >> 8;
                if (decay == 0) decay = 1;  /* Guarantee convergence */
                zcThreshSmooth -= decay;
            }
            garudaData.bemf.zcThreshold = zcThreshSmooth;
        }
        else
        {
            zcThreshSmooth = rawThresh;  /* Non-CL: instant tracking */
            garudaData.bemf.zcThreshold = rawThresh;
        }
    }
#else
    garudaData.bemf.zcThreshold = garudaData.vbusRaw >> 1;
#endif

#if FEATURE_BEMF_CLOSED_LOOP
    /* Store floating phase ADC value in bemfRaw with validity tracking */
    {
        uint8_t fp = commutationTable[garudaData.currentStep].floatingPhase;
        if (fp == FLOATING_PHASE_B)
        {
            garudaData.bemf.bemfRaw = phaseB_val;
            garudaData.bemf.bemfSampleValid = true;
        }
        else if (garudaData.bemf.ad2SettleCount > 0)
        {
            garudaData.bemf.bemfRaw = phaseAC_val;
            garudaData.bemf.bemfSampleValid = false;
            garudaData.bemf.ad2SettleCount--;
        }
        else
        {
            garudaData.bemf.bemfRaw = phaseAC_val;
            garudaData.bemf.bemfSampleValid = true;
        }
    }
    adcIsrTick++;
#else
    garudaData.bemf.bemfRaw = phaseB_val;
    (void)phaseAC_val;  /* AD2CH0DATA must be read; suppress unused warning */
#endif

    /* Bus voltage fault enforcement (OV/UV) */
#if FEATURE_VBUS_FAULT
    {
        static uint8_t vbusOvCount = 0, vbusUvCount = 0;

        if (garudaData.state >= ESC_ALIGN && garudaData.state <= ESC_CLOSED_LOOP)
        {
            if (garudaData.vbusRaw > VBUS_OVERVOLTAGE_ADC)
            {
                if (++vbusOvCount >= VBUS_FAULT_FILTER)
                {
#if FEATURE_ADC_CMP_ZC
                    if (garudaData.hwzc.enabled)
                        HWZC_Disable(&garudaData);
                    garudaData.hwzc.fallbackPending = false;
#endif
#if FEATURE_HW_OVERCURRENT
                    HAL_CMP3_SetThreshold(OC_CMP3_STARTUP_DAC);
#endif
                    HAL_MC1PWMDisableOutputs();
                    garudaData.state = ESC_FAULT;
                    garudaData.faultCode = FAULT_OVERVOLTAGE;
                    garudaData.runCommandActive = false;
                    LED2 = 0;
                }
            }
            else { vbusOvCount = 0; }

            {
                /* Determine UV threshold: relaxed during pre-sync startup to
                 * tolerate bus sag from CC-limited bench supply. Normal
                 * threshold resumes after ZC sync is achieved. */
                uint16_t uvThreshold = VBUS_UNDERVOLTAGE_ADC;
#if FEATURE_PRESYNC_RAMP
                if (garudaData.state <= ESC_OL_RAMP
                    || (garudaData.state == ESC_CLOSED_LOOP
                        && !garudaData.timing.zcSynced))
                    uvThreshold = VBUS_UV_STARTUP_ADC;
#endif

            if (garudaData.vbusRaw < uvThreshold)
            {
                if (++vbusUvCount >= VBUS_FAULT_FILTER)
                {
#if FEATURE_ADC_CMP_ZC
                    if (garudaData.hwzc.enabled)
                        HWZC_Disable(&garudaData);
                    garudaData.hwzc.fallbackPending = false;
#endif
#if FEATURE_HW_OVERCURRENT
                    HAL_CMP3_SetThreshold(OC_CMP3_STARTUP_DAC);
#endif
                    HAL_MC1PWMDisableOutputs();
                    garudaData.state = ESC_FAULT;
                    garudaData.faultCode = FAULT_UNDERVOLTAGE;
                    garudaData.runCommandActive = false;
                    LED2 = 0;
                }
            }
            else { vbusUvCount = 0; }
            } /* uvThreshold scope */
        }
        else
        {
            vbusOvCount = 0;
            vbusUvCount = 0;
        }
    }
#endif

    /* Bus current sensing and overcurrent protection */
#if FEATURE_HW_OVERCURRENT
    /* Read AD1CH2DATA — clears data-ready (mandatory on dsPIC33AK) */
    garudaData.ibusRaw = ADCBUF_IBUS;

    /* Track peak for diagnostics */
    if (garudaData.ibusRaw > garudaData.ibusMax)
        garudaData.ibusMax = garudaData.ibusRaw;

    /* Count CLPCI activity via CLEVT latched event flags.
     * Poll all 3 generators — active PWM phase rotates with commutation.
     * Coarse counter: one 41.7us ADC tick may collapse multiple chop events. */
#if OC_PROTECT_MODE == 0 || OC_PROTECT_MODE == 2
    if (PCI_CLIMIT_EVT_PG1 || PCI_CLIMIT_EVT_PG2 || PCI_CLIMIT_EVT_PG3)
    {
        garudaData.clpciTripCount++;
        /* Clear W1C event flags via direct register write (not bitfield RMW)
         * to avoid accidentally clearing other W1C bits in PGxSTAT. */
        PG1STAT = PCI_CLIMIT_EVT_MASK;
        PG2STAT = PCI_CLIMIT_EVT_MASK;
        PG3STAT = PCI_CLIMIT_EVT_MASK;
    }
#endif

    /* Software hard fault — immediate shutdown (Mode 2 only) */
#if OC_PROTECT_MODE == 2
    if (garudaData.ibusRaw > OC_FAULT_ADC_VAL
        && garudaData.state >= ESC_ALIGN
        && garudaData.state <= ESC_CLOSED_LOOP)
    {
#if FEATURE_ADC_CMP_ZC
        if (garudaData.hwzc.enabled)
            HWZC_Disable(&garudaData);
        garudaData.hwzc.fallbackPending = false;
#endif
        HAL_MC1PWMDisableOutputs();
        garudaData.state = ESC_FAULT;
        garudaData.faultCode = FAULT_OVERCURRENT;
        garudaData.runCommandActive = false;
        LED2 = 0;
    }
#endif
#endif /* FEATURE_HW_OVERCURRENT */

    /* State machine */
    switch (garudaData.state)
    {
        case ESC_IDLE:
        case ESC_ARMED:
            break;

#if FEATURE_SINE_STARTUP
        case ESC_ALIGN:
        case ESC_OL_RAMP:
            if (garudaData.sine.active)
            {
                uint32_t dA, dB, dC;
                STARTUP_SineComputeDuties(&garudaData, &dA, &dB, &dC);
                HAL_PWM_SetDutyCycle3Phase(dA, dB, dC);
            }
            break;

        case ESC_MORPH:
        {
            if (garudaData.morph.subPhase == MORPH_CONVERGE)
            {
                /* Sub-phase A: blended duties, all 3 phases driven */
                uint32_t dA, dB, dC;
                STARTUP_MorphComputeDuties(&garudaData, &dA, &dB, &dC);

#if FEATURE_HW_OVERCURRENT
                /* Current-proportional duty reduction during convergence.
                 * Same logic as CL SW OC limiter but applied to all 3 phases. */
                if (garudaData.ibusRaw > OC_SW_LIMIT_ADC)
                {
                    uint16_t excess = garudaData.ibusRaw - OC_SW_LIMIT_ADC;
                    uint16_t range = OC_CMP3_DAC_VAL - OC_SW_LIMIT_ADC;
                    if (range == 0) range = 1;
                    /* Scale factor: 256 = no reduction, 0 = full cut */
                    uint32_t scale = 256;
                    uint32_t reduction = ((uint32_t)excess * 256u) / range;
                    if (reduction >= scale)
                        scale = 0;
                    else
                        scale -= reduction;
                    dA = (dA * scale) >> 8;
                    dB = (dB * scale) >> 8;
                    dC = (dC * scale) >> 8;
                    if (dA < MIN_DUTY) dA = MIN_DUTY;
                    if (dB < MIN_DUTY) dB = MIN_DUTY;
                    if (dC < MIN_DUTY) dC = MIN_DUTY;
                }
#endif
                HAL_PWM_SetDutyCycle3Phase(dA, dB, dC);

                if (STARTUP_MorphCheckSectorBoundary(&garudaData))
                {
                    if (garudaData.morph.alpha >= 256)
                    {
                        /* Convergence complete → enter Hi-Z sub-phase */
                        garudaData.morph.alpha = 256;
                        garudaData.morph.subPhase = MORPH_HIZ;
                        garudaData.morph.sectorCount = 0;
                        garudaData.sine.active = false;

                        /* Apply 6-step overrides */
                        COMMUTATION_ApplyStep(&garudaData,
                                              garudaData.morph.morphStep);

                        /* Set trap duty */
                        uint32_t td = ((uint32_t)garudaData.sine.amplitude
                            * SINE_TRAP_DUTY_NUM + SINE_TRAP_DUTY_DEN / 2)
                            / SINE_TRAP_DUTY_DEN;
                        if (td < MIN_DUTY) td = MIN_DUTY;
                        if (td > RAMP_DUTY_CAP) td = RAMP_DUTY_CAP;
                        garudaData.duty = td;
                        HAL_PWM_SetDutyCycle(garudaData.duty);

                        /* Initialize timing for forced commutation in Hi-Z.
                         * Dynamic bounds from actual rampStepPeriod. */
                        uint16_t handoff =
                            TIMER1_TO_ADC_TICKS(garudaData.rampStepPeriod);
                        uint16_t initPeriod = handoff;
                        garudaData.timing.stepPeriod = initPeriod;
                        garudaData.timing.forcedCountdown = initPeriod;

                        /* Initialize ZC detection */
                        BEMF_ZC_Init(&garudaData, initPeriod);
                        BEMF_ZC_OnCommutation(&garudaData, adcIsrTick);
                        garudaData.bemf.ad2SettleCount = ZC_AD2_SETTLE_SAMPLES;
                    }
                }
            }
            else /* MORPH_HIZ */
            {
                /* Sub-phase B: real 6-step, BEMF on float phase */
                bool wasDetected = garudaData.bemf.zeroCrossDetected;
                uint8_t spacing = garudaData.timing.stepsSinceLastZc;
                BEMF_ZC_Poll(&garudaData, adcIsrTick);

                /* Latch "ZC confirmed this tick" BEFORE any forced
                 * commutation runs — immune to stepsSinceLastZc race. */
                bool newZcThisTick = (!wasDetected
                    && garudaData.bemf.zeroCrossDetected);

                /* Track ZC timestamps for IIR period adaptation.
                 * Guard: only trust single-step intervals (spacing==1). */
                if (newZcThisTick)
                {
                    garudaData.morph.morphZcCount++;
                    if (garudaData.morph.morphZcCount >= 2 && spacing == 1)
                    {
                        uint16_t measured = (uint16_t)(adcIsrTick
                            - garudaData.morph.lastZcTick);
                        uint16_t mHandoff =
                            TIMER1_TO_ADC_TICKS(garudaData.rampStepPeriod);
                        /* Clamp to [handoff, 1.5× handoff] */
                        if (measured < mHandoff)
                            measured = mHandoff;
                        uint16_t maxMeasured = mHandoff + (mHandoff >> 1);
                        if (measured > maxMeasured)
                            measured = maxMeasured;
                        /* IIR: 7/8 old + 1/8 measured */
                        int32_t delta = (int32_t)measured
                            - (int32_t)garudaData.timing.stepPeriod;
                        garudaData.timing.stepPeriod = (uint16_t)(
                            (int32_t)garudaData.timing.stepPeriod
                            + (delta >> 3));
                    }
                    garudaData.morph.lastZcTick = adcIsrTick;
                }

                /* Forced commutation (open-loop timing) */
                bool forcedStepThisTick = false;
                if (garudaData.timing.forcedCountdown > 0)
                    garudaData.timing.forcedCountdown--;

                if (garudaData.timing.forcedCountdown == 0)
                {
                    COMMUTATION_AdvanceStep(&garudaData);
                    BEMF_ZC_OnCommutation(&garudaData, adcIsrTick);
                    garudaData.timing.forcedCountdown =
                        garudaData.timing.stepPeriod;
                    garudaData.morph.sectorCount++;
                    forcedStepThisTick = true;
                }

                /* Staleness decay: mirror pre-sync pattern */
                if (garudaData.timing.stepsSinceLastZc > ZC_STALENESS_LIMIT)
                {
                    garudaData.timing.goodZcCount = 0;
                    garudaData.timing.risingZcWorks = false;
                    garudaData.timing.fallingZcWorks = false;
                }

                /* ZC confidence gate → exit morph */
                if (garudaData.timing.goodZcCount >= MORPH_ZC_THRESHOLD
                    && garudaData.timing.risingZcWorks
                    && garudaData.timing.fallingZcWorks
                    && newZcThisTick)
                {
                    garudaData.timing.zcSynced = true;

                    /* Seed first post-sync commutation deadline */
                    if (forcedStepThisTick)
                    {
                        garudaData.timing.commDeadline = (uint16_t)(
                            adcIsrTick + garudaData.timing.stepPeriod);
                    }
                    else if (garudaData.morph.morphZcCount >= 1)
                    {
                        garudaData.timing.commDeadline = (uint16_t)(
                            garudaData.morph.lastZcTick
                            + garudaData.timing.stepPeriod / 2);
                    }
                    else
                    {
                        garudaData.timing.commDeadline = (uint16_t)(
                            adcIsrTick + garudaData.timing.stepPeriod);
                    }
                    garudaData.timing.deadlineActive = true;
#if FEATURE_BEMF_INTEGRATION || FEATURE_ADC_CMP_ZC
                    /* Only mark as ZC-timed when deadline is anchored
                     * to an actual ZC event (not a forced-step fallback).
                     * Downstream HWZC handoff gating uses this flag. */
                    garudaData.timing.deadlineIsZc = !forcedStepThisTick;
#endif

                    garudaData.state = ESC_CLOSED_LOOP;
                    break;
                }

                /* Sector timeout */
                if (garudaData.morph.sectorCount >= MORPH_HIZ_MAX_SECTORS)
                {
                    if (garudaData.timing.goodZcCount >= 3)
                    {
                        /* Partial lock — let CL pre-sync finish.
                         * Sync rampStepPeriod from IIR-adapted stepPeriod
                         * so CL entry re-init doesn't discard morph's
                         * speed tracking and jump back to ramp-end rate.
                         * Inverse of TIMER1_TO_ADC_TICKS: t1 = adc * 5/12 */
                        garudaData.rampStepPeriod = (uint16_t)(
                            ((uint32_t)garudaData.timing.stepPeriod * 5 + 6) / 12);
                        garudaData.state = ESC_CLOSED_LOOP;
                    }
                    else
                    {
                        HAL_MC1PWMDisableOutputs();
#if FEATURE_HW_OVERCURRENT
                        HAL_CMP3_SetThreshold(OC_CMP3_STARTUP_DAC);
#endif
                        garudaData.state = ESC_FAULT;
                        garudaData.faultCode = FAULT_MORPH_TIMEOUT;
                        garudaData.runCommandActive = false;
                        LED2 = 0;
                    }
                }


#if FEATURE_HW_OVERCURRENT
                /* SW OC soft limiter — same as CL (proportional duty cut) */
                if (garudaData.ibusRaw > OC_SW_LIMIT_ADC)
                {
                    uint16_t excess = garudaData.ibusRaw - OC_SW_LIMIT_ADC;
                    uint16_t range = OC_CMP3_DAC_VAL - OC_SW_LIMIT_ADC;
                    if (range == 0) range = 1;
                    uint32_t reduction = ((uint32_t)excess
                        * (garudaData.duty - MIN_DUTY)) / range;
                    if (reduction >= garudaData.duty - MIN_DUTY)
                        garudaData.duty = MIN_DUTY;
                    else
                        garudaData.duty -= reduction;
                }
#endif
                HAL_PWM_SetDutyCycle(garudaData.duty);
            }

            /* Absolute timeout — covers BOTH sub-phases.
             * If motor stalls during CONVERGE (e.g. prop overload),
             * this is the only exit path. */
            if ((garudaData.systemTick
                - garudaData.morph.entryTick) > MORPH_TIMEOUT_MS)
            {
                HAL_MC1PWMDisableOutputs();
#if FEATURE_HW_OVERCURRENT
                HAL_CMP3_SetThreshold(OC_CMP3_STARTUP_DAC);
#endif
                garudaData.state = ESC_FAULT;
                garudaData.faultCode = FAULT_MORPH_TIMEOUT;
                garudaData.runCommandActive = false;
                LED2 = 0;
            }
            break;
        }
#else
        case ESC_ALIGN:
        case ESC_OL_RAMP:
            break;
#endif

#if DIAGNOSTIC_MANUAL_STEP
        case ESC_CLOSED_LOOP:
            /* Manual step mode: ADC ISR just holds duty. No automatic commutation. */
            HAL_PWM_SetDutyCycle(garudaData.duty);
            break;
#elif FEATURE_BEMF_CLOSED_LOOP
        case ESC_CLOSED_LOOP:
        {
            /* Throttle-zero shutdown: if pot returns to zero after being raised,
             * gracefully stop. Don't wait for desync — at low duty the HW ZC
             * comparator can trigger on noise indefinitely, keeping the motor
             * in a stalled-but-"running" state (audible buzz).
             *
             * hasSeenThrottle prevents false shutdown at CL entry — the arming
             * gate requires pot=0, so pot is still at zero when CL starts.
             * Only arm the shutdown after the user has raised the pot once. */
            {
                static bool hasSeenThrottle = false;
                static uint16_t zeroThrottleCount = 0;

                if (prevAdcState != ESC_CLOSED_LOOP)
                {
                    hasSeenThrottle = false;
                    zeroThrottleCount = 0;
                }

                if (garudaData.throttle >= ARM_THROTTLE_ZERO_ADC)
                    hasSeenThrottle = true;

                if (hasSeenThrottle && garudaData.throttle < ARM_THROTTLE_ZERO_ADC)
                {
                    if (++zeroThrottleCount >= (PWMFREQUENCY_HZ / 20))  /* 50ms */
                    {
#if FEATURE_ADC_CMP_ZC
                        if (garudaData.hwzc.enabled)
                            HWZC_Disable(&garudaData);
                        garudaData.hwzc.fallbackPending = false;
#endif
#if FEATURE_HW_OVERCURRENT
                        HAL_CMP3_SetThreshold(OC_CMP3_STARTUP_DAC);
#endif
                        HAL_MC1PWMDisableOutputs();
                        garudaData.runCommandActive = false;
                        garudaData.state = ESC_IDLE;
                        LED2 = 0;
                        zeroThrottleCount = 0;
                        break;
                    }
                }
                else
                {
                    zeroThrottleCount = 0;
                }
            }

            /* Detect first entry into CLOSED_LOOP (state transition) */
            if (prevAdcState != ESC_CLOSED_LOOP)
            {
#if FEATURE_SINE_STARTUP
                /* Morph hot handoff: ZC already locked, skip re-init.
                 * Gate on prevAdcState == ESC_MORPH (not just zcSynced)
                 * to prevent stale state from a previous run triggering
                 * the hot-handoff path from OL_RAMP. */
                if (prevAdcState == ESC_MORPH
                    && garudaData.timing.zcSynced)
                {
                    garudaData.desyncRestartAttempts = 0;
#if FEATURE_HW_OVERCURRENT
                    HAL_CMP3_SetThreshold(OC_CMP3_DAC_VAL);
#endif
#if FEATURE_ADC_CMP_ZC
                    garudaData.hwzc.fallbackPending = false;
                    garudaData.hwzc.enablePending = false;
                    garudaData.hwzc.dbgLatchDisable = false;
#endif
                }
                else
#endif
                {
                    /* Normal CL entry (from OL_RAMP or morph without lock) */
                    uint16_t initPeriod = TIMER1_TO_ADC_TICKS(garudaData.rampStepPeriod);
                    if (initPeriod < MIN_ADC_STEP_PERIOD)
                        initPeriod = MIN_ADC_STEP_PERIOD;
                    if (initPeriod > INITIAL_ADC_STEP_PERIOD)
                        initPeriod = INITIAL_ADC_STEP_PERIOD;
                    BEMF_ZC_Init(&garudaData, initPeriod);
                    BEMF_ZC_OnCommutation(&garudaData, adcIsrTick);
                    garudaData.bemf.ad2SettleCount = ZC_AD2_SETTLE_SAMPLES;
#if FEATURE_ADC_CMP_ZC
                    /* Clear stale HWZC state from any prior run */
                    garudaData.hwzc.fallbackPending = false;
                    garudaData.hwzc.enablePending = false;
                    garudaData.hwzc.dbgLatchDisable = false;

#if FEATURE_PRESYNC_RAMP
                    /* Defensively disable HWZC: stale hwzc.enabled=true from a
                     * prior run (e.g. desync recovery) would skip the entire SW
                     * pre-sync block (line ~479 gates on !hwzc.enabled). Force
                     * HWZC off so pre-sync runs. Clear fallbackPending to prevent
                     * stale HWZC state from re-seeding SW ZC. */
                    if (garudaData.hwzc.enabled)
                        HWZC_Disable(&garudaData);
                    garudaData.hwzc.fallbackPending = false;
#else
                    /* Immediate HWZC enable for motors with reliable OL ramp
                     * delivery speed (non-presync-ramp path). */
                    {
                        uint32_t curErpm = ERPM_FROM_ADC_STEP_NUM / initPeriod;
                        if (curErpm >= HWZC_CROSSOVER_ERPM)
                        {
                            HWZC_Enable(&garudaData);
                        }
                    }
#endif
#endif
                }
            }

#if FEATURE_ADC_CMP_ZC
            /* HW->SW fallback re-seed (Rule 9) */
            if (garudaData.hwzc.fallbackPending)
            {
                garudaData.hwzc.fallbackPending = false;
                garudaData.hwzc.enablePending = false;
                uint16_t swPeriod = HWZC_SCCP2_TO_ADC(
                    garudaData.hwzc.stepPeriodHR);
                if (swPeriod < MIN_ADC_STEP_PERIOD)
                    swPeriod = MIN_ADC_STEP_PERIOD;
                if (swPeriod > INITIAL_ADC_STEP_PERIOD)
                    swPeriod = INITIAL_ADC_STEP_PERIOD;
                BEMF_ZC_Init(&garudaData, swPeriod);
                BEMF_ZC_OnCommutation(&garudaData, adcIsrTick);
                garudaData.bemf.ad2SettleCount = ZC_AD2_SETTLE_SAMPLES;

                if (garudaData.hwzc.goodZcCount >= ZC_SYNC_THRESHOLD)
                {
                    /* HW ZC had good lock — seed as synced to avoid
                     * pre-sync forced-commutation jerk at transition */
                    garudaData.timing.zcSynced = true;
                    garudaData.timing.goodZcCount = ZC_SYNC_THRESHOLD;
                    garudaData.timing.forcedCountdown =
                        swPeriod * ZC_TIMEOUT_MULT;
#if FEATURE_HW_OVERCURRENT
                    /* Lower CMP3 from startup to operational threshold */
                    HAL_CMP3_SetThreshold(OC_CMP3_DAC_VAL);
#endif
                }
                else
                {
                    /* HW ZC was struggling — conservative pre-sync */
                    garudaData.timing.zcSynced = false;
                    garudaData.timing.forcedCountdown = swPeriod;
                }
            }

            if (!garudaData.hwzc.enabled)
            {
#endif /* FEATURE_ADC_CMP_ZC */

            if (!garudaData.timing.zcSynced)
            {
                /* === PRE-SYNC: Forced commutation + passive ZC detection === */

                if (garudaData.timing.forcedCountdown > 0)
                    garudaData.timing.forcedCountdown--;

                if (garudaData.timing.forcedCountdown == 0)
                {
                    COMMUTATION_AdvanceStep(&garudaData);
                    BEMF_ZC_OnCommutation(&garudaData, adcIsrTick);

#if FEATURE_PRESYNC_RAMP
                    /* Feedback-gated pre-sync ramp: accelerate forced commutation
                     * only when ZC evidence is strong. Requires goodZcCount >= 3
                     * AND risingZcWorks — not a single noisy edge. If motor is
                     * stale (no recent ZC), hold current speed until motor catches
                     * up and ZC resumes. Same eRPM formula as OL_RAMP. */
                    if (garudaData.timing.stepPeriod > MIN_ADC_STEP_PERIOD
                        && garudaData.timing.goodZcCount >= 3
                        && garudaData.timing.risingZcWorks
                        && garudaData.timing.stepsSinceLastZc <= ZC_STALENESS_LIMIT)
                    {
                        uint32_t sp = garudaData.timing.stepPeriod;
                        uint32_t curErpm = ERPM_FROM_ADC_STEP_NUM / sp;
                        uint32_t deltaErpm = ((uint32_t)RAMP_ACCEL_ERPM_PER_S * sp)
                                             / PWMFREQUENCY_HZ;
                        if (deltaErpm < 1) deltaErpm = 1;
                        uint32_t newErpm = curErpm + deltaErpm;
                        uint32_t newPeriod = ERPM_FROM_ADC_STEP_NUM / newErpm;
                        if (newPeriod < MIN_ADC_STEP_PERIOD)
                            newPeriod = MIN_ADC_STEP_PERIOD;
                        garudaData.timing.stepPeriod = (uint16_t)newPeriod;
                    }
#endif

                    garudaData.timing.forcedCountdown = garudaData.timing.stepPeriod;
                    garudaData.zcDiag.forcedStepPresyncCount++;
                }

#if FEATURE_PRESYNC_RAMP
                /* Pre-sync timeout: fault if ZC never achieved */
                {
                    static uint32_t presyncEntryTick = 0;
                    if (prevAdcState != ESC_CLOSED_LOOP)
                        presyncEntryTick = garudaData.systemTick;

                    if ((garudaData.systemTick - presyncEntryTick) > PRESYNC_TIMEOUT_MS)
                    {
#if FEATURE_ADC_CMP_ZC
                        if (garudaData.hwzc.enabled)
                            HWZC_Disable(&garudaData);
                        garudaData.hwzc.fallbackPending = false;
#endif
#if FEATURE_HW_OVERCURRENT
                        HAL_CMP3_SetThreshold(OC_CMP3_STARTUP_DAC);
#endif
                        HAL_MC1PWMDisableOutputs();
                        garudaData.state = ESC_FAULT;
                        garudaData.faultCode = FAULT_STARTUP_TIMEOUT;
                        garudaData.runCommandActive = false;
                        LED2 = 0;
                    }
                }
#endif

                /* Passive ZC detection (builds goodZcCount, no commutation trigger) */
                BEMF_ZC_Poll(&garudaData, adcIsrTick);

                if (garudaData.timing.stepsSinceLastZc > ZC_STALENESS_LIMIT)
                    garudaData.timing.goodZcCount = 0;

                if (garudaData.timing.goodZcCount >= (uint16_t)ZC_SYNC_THRESHOLD
                    && garudaData.timing.risingZcWorks)
                {
                    garudaData.timing.zcSynced = true;
                    garudaData.desyncRestartAttempts = 0;
#if FEATURE_HW_OVERCURRENT
                    /* Lower CMP3 from startup to operational threshold */
                    HAL_CMP3_SetThreshold(OC_CMP3_DAC_VAL);
#endif
#if FEATURE_BEMF_INTEGRATION
                    garudaData.integ.bemfPeakSmooth = 0;
                    garudaData.integ.integral = 0;
                    garudaData.integ.stepDevMax = 0;
                    garudaData.integ.shadowFired = false;
#endif
                    if (garudaData.bemf.zeroCrossDetected)
                    {
#if FEATURE_TIMING_ADVANCE
                        uint16_t sp0 = garudaData.timing.stepPeriod;
                        uint32_t eRPM0 = ERPM_FROM_ADC_STEP_NUM / sp0;
                        uint16_t adv0;
                        if (eRPM0 <= RAMP_TARGET_ERPM)
                            adv0 = TIMING_ADVANCE_MIN_DEG;
                        else if (eRPM0 >= MAX_CLOSED_LOOP_ERPM)
                            adv0 = TIMING_ADVANCE_MAX_DEG;
                        else
                        {
                            uint32_t r0 = MAX_CLOSED_LOOP_ERPM - RAMP_TARGET_ERPM;
                            uint32_t p0 = eRPM0 - RAMP_TARGET_ERPM;
                            adv0 = TIMING_ADVANCE_MIN_DEG +
                                (uint16_t)((uint32_t)(TIMING_ADVANCE_MAX_DEG - TIMING_ADVANCE_MIN_DEG)
                                           * p0 / r0);
                        }
                        uint16_t d0 = (uint16_t)((uint32_t)sp0 * (30 - adv0) / 60);
                        garudaData.timing.commDeadline = (uint16_t)(adcIsrTick + d0);
#else
                        garudaData.timing.commDeadline = (uint16_t)(
                            adcIsrTick + garudaData.timing.stepPeriod / 2);
#endif
                        garudaData.timing.deadlineActive = true;
#if FEATURE_BEMF_INTEGRATION || FEATURE_ADC_CMP_ZC
                        garudaData.timing.deadlineIsZc = true;
#endif
                    }
                    else
                    {
                        BEMF_ZC_HandleUndetectableStep(&garudaData, adcIsrTick);
                    }
                }
            }
            else
            {
                /* === POST-SYNC: ZC-driven commutation === */

#if FEATURE_ADC_CMP_ZC
                /* Crossover check + enablePending management (Rule 12).
                 * dbgLatchDisable: after first HW ZC failure, permanently
                 * block re-enable so motor stays on SW ZC and diagnostics
                 * are preserved for debugger reading. */
                if (!garudaData.hwzc.dbgLatchDisable)
                {
                    uint32_t curErpm = ERPM_FROM_ADC_STEP_NUM /
                        garudaData.timing.stepPeriod;
                    if (curErpm >= HWZC_CROSSOVER_ERPM)
                        garudaData.hwzc.enablePending = true;
                    else if (garudaData.hwzc.enablePending
                             && curErpm < (HWZC_CROSSOVER_ERPM
                                           - HWZC_HYSTERESIS_ERPM))
                        garudaData.hwzc.enablePending = false;
                }
#endif

                /* Poll for ZC */
                BEMF_ZC_Poll(&garudaData, adcIsrTick);

                /* Check commutation deadline */
                if (BEMF_ZC_CheckDeadline(&garudaData, adcIsrTick))
                {
#if FEATURE_BEMF_INTEGRATION
                    if (!garudaData.timing.deadlineIsZc)
                    {
                        garudaData.integ.shadowSkipCount++;
                    }
                    else if (garudaData.integ.bemfPeakSmooth == 0)
                    {
                        garudaData.integ.shadowUnseededSkip++;
                    }
                    else
                    {
                        uint16_t tol = garudaData.timing.stepPeriod / INTEG_HIT_DIVISOR;
                        if (tol < 1) tol = 1;

                        garudaData.integ.shadowSampleCount++;
                        garudaData.integ.shadowStepPeriodSum +=
                            garudaData.timing.stepPeriod;

                        if (garudaData.integ.shadowFired)
                        {
                            int16_t diff = (int16_t)(garudaData.integ.shadowFireTick
                                                   - adcIsrTick);
                            garudaData.integ.shadowVsActual = diff;

                            {
                                int64_t wide = (int64_t)garudaData.integ.shadowErrorSum + diff;
                                if (wide < INT32_MIN) wide = INT32_MIN;
                                else if (wide > INT32_MAX) wide = INT32_MAX;
                                garudaData.integ.shadowErrorSum = (int32_t)wide;
                            }

                            int16_t absDiff = (diff < 0) ? -diff : diff;
                            garudaData.integ.shadowAbsErrorSum += (uint32_t)absDiff;

                            if (absDiff <= (int16_t)tol)
                                garudaData.integ.shadowHitCount++;
                            else
                                garudaData.integ.shadowMissCount++;
                        }
                        else
                        {
                            garudaData.integ.shadowNoFireCount++;
                            garudaData.integ.shadowMissCount++;
                            garudaData.integ.shadowVsActual = SHADOW_NO_FIRE_SENTINEL;
                        }
                    }
#endif
#if FEATURE_ADC_CMP_ZC
                    /* Snapshot deadlineIsZc BEFORE AdvanceStep clears it (Rule 14) */
                    bool wasZcDeadline = garudaData.timing.deadlineIsZc;
#endif
                    COMMUTATION_AdvanceStep(&garudaData);
                    BEMF_ZC_OnCommutation(&garudaData, adcIsrTick);
                    BEMF_ZC_HandleUndetectableStep(&garudaData, adcIsrTick);

#if FEATURE_ADC_CMP_ZC
                    /* enablePending hook: hand off to hardware ZC at true-ZC
                     * commutation boundary only (Rule 12) */
                    if (garudaData.hwzc.enablePending && wasZcDeadline)
                    {
                        HWZC_Enable(&garudaData);
                        goto cl_duty_control;
                    }
#endif
                }

                /* Timeout watchdog */
                ZC_TIMEOUT_RESULT_T toResult = BEMF_ZC_CheckTimeout(&garudaData, adcIsrTick);
                if (toResult == ZC_TIMEOUT_DESYNC)
                {
                    garudaData.zcDiag.zcDesyncCount++;
#if FEATURE_HW_OVERCURRENT
                    HAL_CMP3_SetThreshold(OC_CMP3_STARTUP_DAC);
#endif
#if FEATURE_DESYNC_RECOVERY
                    if (garudaData.runCommandActive)
                    {
                        HAL_MC1PWMDisableOutputs();
                        garudaData.state = ESC_RECOVERY;
                        garudaData.recoveryCounter = DESYNC_COAST_COUNTS;
                        LED2 = 0;
                    }
                    else
                    {
                        HAL_MC1PWMDisableOutputs();
                        garudaData.desyncRestartAttempts = 0;
                        garudaData.state = ESC_IDLE;
                        LED2 = 0;
                    }
#else
                    garudaData.state = ESC_FAULT;
                    garudaData.faultCode = FAULT_DESYNC;
                    HAL_MC1PWMDisableOutputs();
                    LED2 = 0;
#endif
                }
                else if (toResult == ZC_TIMEOUT_FORCE_STEP)
                {
#if FEATURE_BEMF_INTEGRATION
                    garudaData.integ.shadowSkipCount++;
#endif
                    garudaData.zcDiag.zcTimeoutForceCount++;
                    {
                        uint8_t s = garudaData.currentStep;
                        if (garudaData.timing.stepMissCount[s] < 255)
                            garudaData.timing.stepMissCount[s]++;
                    }
                    COMMUTATION_AdvanceStep(&garudaData);
                    BEMF_ZC_OnCommutation(&garudaData, adcIsrTick);
                    BEMF_ZC_HandleUndetectableStep(&garudaData, adcIsrTick);

                    if (garudaData.timing.goodZcCount == 0)
                    {
                        garudaData.timing.zcSynced = false;
                        garudaData.timing.hasPrevZc = false;
                        uint16_t initPeriod = TIMER1_TO_ADC_TICKS(garudaData.rampStepPeriod);
                        if (initPeriod < MIN_ADC_STEP_PERIOD)
                            initPeriod = MIN_ADC_STEP_PERIOD;
                        garudaData.timing.stepPeriod = initPeriod;
                        garudaData.timing.forcedCountdown = initPeriod;
#if FEATURE_PRESYNC_RAMP
                        /* Reset duty to ramp level to prevent ratchet:
                         * post-sync CL_IDLE floor inflates duty, and pre-sync
                         * holds mappedDuty=garudaData.duty. Without reset,
                         * each sync→unsync cycle pumps duty higher, driving
                         * massive current through low-R motor. */
                        garudaData.duty = RAMP_DUTY_CAP;
#endif
                    }
                }
            }

#if FEATURE_ADC_CMP_ZC
            }
            else
            {
                /* hwzc.enabled=true: software ZC block skipped.
                 * SCCP1 ISR + comparator ISR handle commutation. */

                /* No speed-drop fallback: once HW ZC activates, it stays
                 * active at all speeds. The ADC comparator works fine at low
                 * eRPM (24kHz samples → plenty per step). Only miss-limit
                 * triggers fallback (error condition). This eliminates the
                 * HW→SW transition jerk during normal deceleration. */

                /* Promote zcSynced once HWZC has confirmed good ZCs.
                 * This unlocks pot-mapped duty (CL_IDLE_DUTY floor)
                 * and MAX_DUTY cap instead of RAMP_DUTY_CAP. */
                if (!garudaData.timing.zcSynced
                    && garudaData.hwzc.goodZcCount >= ZC_SYNC_THRESHOLD)
                {
                    garudaData.timing.zcSynced = true;
                    garudaData.desyncRestartAttempts = 0;
#if FEATURE_HW_OVERCURRENT
                    HAL_CMP3_SetThreshold(OC_CMP3_DAC_VAL);
#endif
                }

#if FEATURE_BEMF_INTEGRATION
                /* Read-only observer: keep shadow integration warm (Rule 11) */
                {
                    bool newComm = (garudaData.hwzc.commSeq
                                    != garudaData.hwzc.obsCommSeq);
                    if (newComm)
                    {
                        garudaData.hwzc.obsCommSeq =
                            garudaData.hwzc.commSeq;
                        garudaData.hwzc.obsLastCommTick = adcIsrTick;
                        /* Seqlock read of stepPeriodHR (Rule 13) */
                        uint32_t sp;
                        uint16_t s1, s2;
                        do {
                            s1 = garudaData.hwzc.writeSeq;
                            sp = garudaData.hwzc.stepPeriodHR;
                            s2 = garudaData.hwzc.writeSeq;
                        } while (s1 != s2 || (s1 & 1));
                        uint16_t obsPeriod = HWZC_SCCP2_TO_ADC(sp);
                        if (obsPeriod < 1) obsPeriod = 1;
                        BEMF_INTEG_ObserverOnComm(&garudaData, obsPeriod);
                        garudaData.hwzc.shadowHwzcSkipCount++;
                    }
                    BEMF_INTEG_ObserverTick(&garudaData, adcIsrTick,
                        garudaData.hwzc.obsLastCommTick);
                }
#endif /* FEATURE_BEMF_INTEGRATION */
            }
#endif /* FEATURE_ADC_CMP_ZC */

#if FEATURE_ADC_CMP_ZC
        cl_duty_control:
#endif

            /* Duty cycle control. */
            {
                uint32_t cap = garudaData.timing.zcSynced ? MAX_DUTY : RAMP_DUTY_CAP;
                uint32_t mappedDuty;

#if FEATURE_DUTY_SLEW
                static uint16_t postSyncCounter = 0;
#endif

                if (!garudaData.timing.zcSynced)
                {
                    /* Pre-sync: hold duty at CL entry level.
                     * Don't let pot changes affect duty while forced
                     * commutation is trying to lock ZC — changing duty
                     * shifts the threshold and confuses detection. */
                    mappedDuty = garudaData.duty;
#if FEATURE_DUTY_SLEW
                    postSyncCounter = 0;
#endif
                }
                else
                {
#if FEATURE_DUTY_SLEW
                    if (postSyncCounter < POST_SYNC_SETTLE_TICKS)
                        postSyncCounter++;
#endif
                    mappedDuty = CL_IDLE_DUTY +
                        ((uint32_t)garudaData.potRaw * (cap - CL_IDLE_DUTY)) / 4096;
                }
                if (mappedDuty < MIN_DUTY) mappedDuty = MIN_DUTY;
                if (mappedDuty > cap) mappedDuty = cap;

#if FEATURE_DUTY_SLEW
                {
                    static uint32_t prevDuty = 0;
                    if (prevAdcState != ESC_CLOSED_LOOP)
                        prevDuty = garudaData.duty;

                    /* Post-sync settle: use reduced slew-up rate for
                     * POST_SYNC_SETTLE_MS after ZC lock. This prevents
                     * the motor from accelerating faster than the
                     * stepPeriod IIR can track on low-inertia motors.
                     * Normal rate: 2%/ms. Settle rate: 0.25%/ms (1/8). */
                    uint32_t upRate = DUTY_SLEW_UP_RATE;
                    if (postSyncCounter < POST_SYNC_SETTLE_TICKS)
                        upRate = DUTY_SLEW_UP_RATE / POST_SYNC_SLEW_DIVISOR;

                    int32_t delta = (int32_t)mappedDuty - (int32_t)prevDuty;
                    if (delta > 0)
                    {
                        if ((uint32_t)delta > upRate)
                            mappedDuty = prevDuty + upRate;
                    }
                    else if (delta < 0)
                    {
                        if ((uint32_t)(-delta) > DUTY_SLEW_DOWN_RATE)
                            mappedDuty = prevDuty - DUTY_SLEW_DOWN_RATE;
                    }
                    prevDuty = mappedDuty;
                }
#endif
                if (garudaData.timing.stepPeriod <= MIN_CL_ADC_STEP_PERIOD
                    && mappedDuty > garudaData.duty)
                {
                    mappedDuty = garudaData.duty;
                }

#if FEATURE_VBUS_SAG_LIMIT
                {
                    static uint16_t vbusFiltered = 0;
                    static bool vbusSagActive = false;

                    if (prevAdcState != ESC_CLOSED_LOOP)
                    {
                        vbusFiltered = garudaData.vbusRaw;
                        vbusSagActive = false;
                    }

                    /* IIR filter always runs (keeps filter warm during pre-sync) */
                    vbusFiltered = (uint16_t)(
                        ((uint32_t)vbusFiltered * 7 + garudaData.vbusRaw) >> 3);

#if FEATURE_PRESYNC_RAMP
                    /* Bypass sag limiter during pre-sync: at 12V CC-limited supply,
                     * vbus sags to ~636 ADC (8.9V) under startup load. With
                     * VBUS_SAG_THRESHOLD_ADC=900, the sag limiter would reduce
                     * RAMP_DUTY_CAP by ~26%, killing startup torque. Allow full
                     * startup duty until ZC sync is achieved. */
                    if (garudaData.timing.zcSynced)
                    {
#endif
                    if (!vbusSagActive && vbusFiltered < VBUS_SAG_THRESHOLD_ADC)
                        vbusSagActive = true;
                    else if (vbusSagActive && vbusFiltered > VBUS_SAG_RECOVERY_ADC)
                        vbusSagActive = false;

                    if (vbusSagActive)
                    {
                        uint32_t sagDepth = (vbusFiltered < VBUS_SAG_THRESHOLD_ADC) ?
                            (VBUS_SAG_THRESHOLD_ADC - vbusFiltered) : 0;
                        uint32_t reduction = (sagDepth * VBUS_SAG_GAIN) >> 4;
                        if (reduction >= mappedDuty)
                            mappedDuty = MIN_DUTY;
                        else
                            mappedDuty -= reduction;
                    }
#if FEATURE_PRESYNC_RAMP
                    }
                    else
                    {
                        vbusSagActive = false;  /* Reset so it re-evaluates on sync */
                    }
#endif

                    if (mappedDuty < MIN_DUTY)
                        mappedDuty = MIN_DUTY;
                }
#endif

#if FEATURE_HW_OVERCURRENT
                /* Software bus current soft limiter — proportional duty reduction.
                 * Ramps down duty smoothly BEFORE CMP3/CLPCI hardware trips. */
                if (garudaData.ibusRaw > OC_SW_LIMIT_ADC)
                {
                    uint16_t excess = garudaData.ibusRaw - OC_SW_LIMIT_ADC;
                    uint16_t range = OC_CMP3_DAC_VAL - OC_SW_LIMIT_ADC;
                    if (range == 0) range = 1;
                    uint32_t reduction = ((uint32_t)excess * (mappedDuty - MIN_DUTY)) / range;
                    if (reduction >= mappedDuty - MIN_DUTY)
                        mappedDuty = MIN_DUTY;
                    else
                        mappedDuty -= reduction;
                }
#endif

                garudaData.duty = mappedDuty;
            }
            HAL_PWM_SetDutyCycle(garudaData.duty);
            break;
        }
#else  /* !FEATURE_BEMF_CLOSED_LOOP — Phase 1 open-loop path */
        case ESC_CLOSED_LOOP:
            HAL_PWM_SetDutyCycle(garudaData.duty);
            break;
#endif

        case ESC_BRAKING:
            break;

        case ESC_RECOVERY:
            /* PWM already disabled on entry — coast-down handled in Timer1 */
            break;

        case ESC_FAULT:
            HAL_MC1PWMDisableOutputs();
            break;
    }

#if FEATURE_BEMF_CLOSED_LOOP
    /* Track state for transition detection (must be last before flag clear).
     * Use entryState (not garudaData.state) so mid-ISR state changes
     * (e.g. morph→CL) are visible on the NEXT tick. */
    prevAdcState = entryState;
#endif

    /* X2CScope: sample variables at ADC rate (24kHz) */
#ifdef ENABLE_DIAGNOSTICS
    DiagnosticsStepIsr();
#endif

    /* Clear interrupt flag AFTER reading all buffers (matches reference) */
    GARUDA_ClearADCIF();
}

/**
 * @brief Timer1 ISR — 100us tick.
 * Handles: heartbeat LED, board service, commutation timing for
 * align/ramp states, 1ms system tick.
 */
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
    /* Heartbeat LED — toggle at ~2Hz (250ms) */
    heartbeatCounter++;
    if (heartbeatCounter >= HEART_BEAT_LED_COUNT)
    {
        heartbeatCounter = 0;
        if (LED1 == 1)
            LED1 = 0;
        else
            LED1 = 1;
    }

    /* Board service step (drives button debounce at 1ms) */
    BoardServiceStepIsr();

    /* 1ms system tick sub-counter (10 x 100us = 1ms) */
    msSubCounter++;
    if (msSubCounter >= 10)
    {
        msSubCounter = 0;
        garudaData.systemTick++;
    }

    /* State-specific processing at 100us rate */
    switch (garudaData.state)
    {
        case ESC_IDLE:
            /* Nothing — waiting for button press in main loop */
            break;

        case ESC_ARMED:
            /* Verify throttle stays at zero for ARM_TIME */
            if (garudaData.throttle < ARM_THROTTLE_ZERO_ADC)
            {
                garudaData.armCounter++;
                if (garudaData.armCounter >= ARM_TIME_COUNTS)
                {
                    /* Armed successfully — transition to ALIGN */
                    garudaData.state = ESC_ALIGN;
                    STARTUP_Init(&garudaData);
                    LED2 = 1; /* Motor running indicator */
                }
            }
            else
            {
                garudaData.armCounter = 0; /* Reset if throttle not zero */
            }
            break;

        case ESC_ALIGN:
#if FEATURE_SINE_STARTUP
            if (STARTUP_SineAlign(&garudaData))
                garudaData.state = ESC_OL_RAMP;
#else
            if (STARTUP_Align(&garudaData))
            {
                garudaData.state = ESC_OL_RAMP;
                garudaData.rampCounter = garudaData.rampStepPeriod;
            }
#endif
            break;

#if DIAGNOSTIC_MANUAL_STEP
        case ESC_OL_RAMP:
            garudaData.state = ESC_CLOSED_LOOP;
            break;
#elif FEATURE_SINE_STARTUP
        case ESC_OL_RAMP:
            if (STARTUP_SineRamp(&garudaData))
            {
                /* Sine ramp complete → enter waveform morph.
                 * rampStepPeriod synced from sine eRPM in MorphInit. */
                STARTUP_MorphInit(&garudaData);
#if FEATURE_HW_OVERCURRENT
                /* Lower CMP3 from startup (22A) to operational (12A).
                 * Morph convergence can spike current above the board PCI
                 * threshold (~15A) but below CMP3 startup. Without this,
                 * CMP3 never trips and the board PCI hard-faults. */
                HAL_CMP3_SetThreshold(OC_CMP3_DAC_VAL);
#endif
                garudaData.state = ESC_MORPH;
            }
            break;

        case ESC_MORPH:
            /* All morph logic runs in ADC ISR (24kHz). Timer1 is idle. */
            break;
#else
        case ESC_OL_RAMP:
#if FEATURE_PRESYNC_RAMP
            /* Skip blind forced ramp. Enter CL directly with slow initial
             * step period. ADC ISR pre-sync handles feedback-gated
             * acceleration with ZC detection in parallel. */
            garudaData.duty = RAMP_DUTY_CAP;
            garudaData.state = ESC_CLOSED_LOOP;
#else
            if (STARTUP_OpenLoopRamp(&garudaData))
            {
                garudaData.state = ESC_CLOSED_LOOP;
            }
#endif
            break;
#endif

#if DIAGNOSTIC_MANUAL_STEP
        case ESC_CLOSED_LOOP:
            /* Manual step mode: hold current step. SW2 advances from main loop. */
            break;
#elif FEATURE_BEMF_CLOSED_LOOP
        case ESC_CLOSED_LOOP:
            /* Phase 2: All commutation handled in ADC ISR. Timer1 does nothing. */
            break;
#else
        case ESC_CLOSED_LOOP:
            /* Phase 1: keep forced commutation at final ramp speed. */
            if (garudaData.rampCounter > 0)
            {
                garudaData.rampCounter--;
            }
            if (garudaData.rampCounter == 0)
            {
                COMMUTATION_AdvanceStep(&garudaData);
                garudaData.rampCounter = garudaData.rampStepPeriod;
            }
            break;
#endif

        case ESC_BRAKING:
            break;

        case ESC_RECOVERY:
#if FEATURE_DESYNC_RECOVERY
            if (garudaData.recoveryCounter > 0)
            {
                garudaData.recoveryCounter--;
            }
            else
            {
                if (garudaData.runCommandActive &&
                    garudaData.desyncRestartAttempts < DESYNC_MAX_RESTARTS &&
                    garudaData.throttle >= ARM_THROTTLE_ZERO_ADC)
                {
                    garudaData.desyncRestartAttempts++;
                    garudaData.state = ESC_ALIGN;
                    STARTUP_Init(&garudaData);
                    LED2 = 1;
                }
                else if (garudaData.throttle < ARM_THROTTLE_ZERO_ADC)
                {
                    /* Throttle is zero — user wants motor stopped.
                     * Don't restart, go to IDLE. */
                    garudaData.runCommandActive = false;
                    garudaData.desyncRestartAttempts = 0;
                    garudaData.state = ESC_IDLE;
                    LED2 = 0;
                }
                else if (!garudaData.runCommandActive)
                {
                    /* User pressed stop during coast — graceful idle */
                    garudaData.desyncRestartAttempts = 0;
                    garudaData.state = ESC_IDLE;
                    LED2 = 0;
                }
                else
                {
                    /* Max restarts exhausted — permanent fault */
                    garudaData.runCommandActive = false;
                    garudaData.state = ESC_FAULT;
                    garudaData.faultCode = FAULT_DESYNC;
                    LED2 = 0;
                }
            }
#endif
            break;

        case ESC_FAULT:
            break;
    }

    TIMER1_InterruptFlagClear();
}

/**
 * @brief PWM Fault ISR — handles PCI fault events.
 */
void __attribute__((__interrupt__, no_auto_psv)) _PWMInterrupt(void)
{
    if (PCI_FAULT_ACTIVE_STATUS)
    {
        /* Board-level FPCI fault via PCI8R (RP28/DIM040).
         * Source: U25A (overvoltage) + U25B (overcurrent) → U27 AND gate.
         * This is a combined OC+OV signal — cannot distinguish which triggered. */
#if FEATURE_ADC_CMP_ZC
        if (garudaData.hwzc.enabled)
            HWZC_Disable(&garudaData);
#endif
#if FEATURE_HW_OVERCURRENT
        HAL_CMP3_SetThreshold(OC_CMP3_STARTUP_DAC);
#endif
        HAL_MC1ClearPWMPCIFault();
        HAL_MC1PWMDisableOutputs();
        garudaData.state = ESC_FAULT;
        garudaData.faultCode = FAULT_BOARD_PCI;
        garudaData.runCommandActive = false;
#if FEATURE_ADC_CMP_ZC
        garudaData.hwzc.fallbackPending = false;
#endif
        LED2 = 0;
    }
    ClearPWMIF();
}

#if FEATURE_ADC_CMP_ZC
/**
 * @brief ADC1 Comparator CH5 ISR — ZC detected on Phase B.
 * Do NOT re-read AD1CH5DATA for validation (Rule 10).
 */
void __attribute__((__interrupt__, no_auto_psv)) _AD1CMP5Interrupt(void)
{
    _AD1CMP5IE = 0;              /* Disable immediately to prevent re-trigger */
    AD1CMPSTATbits.CH5CMP = 0;  /* Clear comparator status */
    if (garudaData.hwzc.enabled)
        HWZC_OnZcDetected(&garudaData);
    _AD1CMP5IF = 0;
}

/**
 * @brief ADC2 Comparator CH1 ISR — ZC detected on Phase A or C.
 */
void __attribute__((__interrupt__, no_auto_psv)) _AD2CMP1Interrupt(void)
{
    _AD2CMP1IE = 0;
    AD2CMPSTATbits.CH1CMP = 0;
    if (garudaData.hwzc.enabled)
        HWZC_OnZcDetected(&garudaData);
    _AD2CMP1IF = 0;
}

/**
 * @brief SCCP1 Timer ISR — blanking expired, commutation deadline, or timeout.
 * Action determined by hwzc.phase (Rule 3: phase is set BEFORE timer starts).
 */
void __attribute__((__interrupt__, no_auto_psv)) _CCT1Interrupt(void)
{
    if (!garudaData.hwzc.enabled)
    {
        /* HWZC was disabled (fault, button stop, throttle-zero shutdown)
         * while a timer event was pending. Discard — do not commutate. */
        _CCT1IF = 0;
        return;
    }

    switch (garudaData.hwzc.phase)
    {
        case HWZC_BLANKING:
            HWZC_OnBlankingExpired(&garudaData);
            break;
        case HWZC_WATCHING:
            HWZC_OnTimeout(&garudaData);
            break;
        case HWZC_COMM_PENDING:
            HWZC_OnCommDeadline(&garudaData);
            break;
        default:
            break;
    }
    _CCT1IF = 0;
}
#endif /* FEATURE_ADC_CMP_ZC */
