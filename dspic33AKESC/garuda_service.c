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

#if FEATURE_LEARN_MODULES
#include "learn/learn_service.h"
#endif

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

    /* Enable ADC interrupt to start the control loop */
    GARUDA_ClearADCIF();
    GARUDA_EnableADCInterrupt();

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
    /* Duty-proportional ZC threshold = virtual neutral point.
     * neutral = Vbus * duty / (2 * LOOPTIME_TCY)
     * Approximated as (vbusRaw * duty) >> 18 for 24kHz PWM.
     * At 8% duty: ~52 counts.  At 40% duty: ~261 counts. */
    garudaData.bemf.zcThreshold = (uint16_t)(
        ((uint32_t)garudaData.vbusRaw * garudaData.duty) >> ZC_DUTY_THRESHOLD_SHIFT);
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

    /* State machine */
    switch (garudaData.state)
    {
        case ESC_IDLE:
        case ESC_ARMED:
        case ESC_ALIGN:
        case ESC_OL_RAMP:
            /* Handled in Timer1 ISR */
            break;

#if DIAGNOSTIC_MANUAL_STEP
        case ESC_CLOSED_LOOP:
            /* Manual step mode: ADC ISR just holds duty. No automatic commutation. */
            HAL_PWM_SetDutyCycle(garudaData.duty);
            break;
#elif FEATURE_BEMF_CLOSED_LOOP
        case ESC_CLOSED_LOOP:
        {
            /* Detect first entry into CLOSED_LOOP (state transition) */
            if (prevAdcState != ESC_CLOSED_LOOP)
            {
                uint16_t initPeriod = TIMER1_TO_ADC_TICKS(garudaData.rampStepPeriod);
                if (initPeriod < MIN_ADC_STEP_PERIOD)
                    initPeriod = MIN_ADC_STEP_PERIOD;
                if (initPeriod > INITIAL_ADC_STEP_PERIOD)
                    initPeriod = INITIAL_ADC_STEP_PERIOD;
                BEMF_ZC_Init(&garudaData, initPeriod);
                BEMF_ZC_OnCommutation(&garudaData, adcIsrTick);
            }

            if (!garudaData.timing.zcSynced)
            {
                /* === PRE-SYNC: Forced commutation + passive ZC detection === */

                if (garudaData.timing.forcedCountdown > 0)
                    garudaData.timing.forcedCountdown--;

                if (garudaData.timing.forcedCountdown == 0)
                {
                    /* No penalty for missed ZC in pre-sync.
                     * goodZcCount only increments (inside BEMF_ZC_Poll on
                     * confirmed ZC).  This allows sync lock even when only
                     * rising-ZC steps produce detectable crossings (50% hit
                     * rate).  Noise-lock is prevented by the ZC_FILTER_THRESHOLD
                     * (3 consecutive confirming samples) inside Poll. */

                    COMMUTATION_AdvanceStep(&garudaData);
                    BEMF_ZC_OnCommutation(&garudaData, adcIsrTick);
                    garudaData.timing.forcedCountdown = garudaData.timing.stepPeriod;
                    garudaData.zcDiag.forcedStepPresyncCount++;
                }

                /* Passive ZC detection (builds goodZcCount, no commutation trigger) */
                BEMF_ZC_Poll(&garudaData, adcIsrTick);

                /* Staleness guard: if too many forced steps pass without any
                 * ZC confirmation, the accumulated goodZcCount is stale.
                 * At 50% hit rate (rising-only), stepsSinceLastZc normally
                 * peaks at 2.  12 = two full electrical cycles of silence. */
                if (garudaData.timing.stepsSinceLastZc > ZC_STALENESS_LIMIT)
                    garudaData.timing.goodZcCount = 0;

                /* Check for ZC lock:
                 * 1. goodZcCount >= threshold (enough recent confirmed ZCs)
                 * 2. At least one polarity confirmed (risingZcWorks) */
                if (garudaData.timing.goodZcCount >= (uint16_t)ZC_SYNC_THRESHOLD
                    && garudaData.timing.risingZcWorks)
                {
                    garudaData.timing.zcSynced = true;
                    /* Seed post-sync: if ZC was just confirmed on this step,
                     * set a 30-degree deadline.  Otherwise check if this step
                     * is undetectable and schedule timing-based commutation. */
                    if (garudaData.bemf.zeroCrossDetected)
                    {
                        garudaData.timing.commDeadline = (uint16_t)(
                            adcIsrTick + garudaData.timing.stepPeriod / 2);
                        garudaData.timing.deadlineActive = true;
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

                /* Poll for ZC (skipped on undetectable steps via zeroCrossDetected guard) */
                BEMF_ZC_Poll(&garudaData, adcIsrTick);

                /* Check commutation deadline (handles both ZC-based and timing-based) */
                if (BEMF_ZC_CheckDeadline(&garudaData, adcIsrTick))
                {
                    COMMUTATION_AdvanceStep(&garudaData);
                    BEMF_ZC_OnCommutation(&garudaData, adcIsrTick);
                    /* If new step can't detect ZC, schedule timing-based comm */
                    BEMF_ZC_HandleUndetectableStep(&garudaData, adcIsrTick);
                }

                /* Timeout watchdog (skipped on undetectable steps via deadlineActive guard) */
                ZC_TIMEOUT_RESULT_T toResult = BEMF_ZC_CheckTimeout(&garudaData, adcIsrTick);
                if (toResult == ZC_TIMEOUT_DESYNC)
                {
                    garudaData.zcDiag.zcDesyncCount++;
                    garudaData.state = ESC_FAULT;
                    garudaData.faultCode = FAULT_DESYNC;
                    HAL_MC1PWMDisableOutputs();
                    LED2 = 0;
                }
                else if (toResult == ZC_TIMEOUT_FORCE_STEP)
                {
                    garudaData.zcDiag.zcTimeoutForceCount++;
                    /* Increment per-step miss counter (before AdvanceStep
                     * changes currentStep).  Saturate at 255. */
                    {
                        uint8_t s = garudaData.currentStep;
                        if (garudaData.timing.stepMissCount[s] < 255)
                            garudaData.timing.stepMissCount[s]++;
                    }
                    COMMUTATION_AdvanceStep(&garudaData);
                    BEMF_ZC_OnCommutation(&garudaData, adcIsrTick);
                    /* If new step can't detect ZC, schedule timing-based comm */
                    BEMF_ZC_HandleUndetectableStep(&garudaData, adcIsrTick);

                    /* If goodZcCount dropped to 0, lose sync → revert to forced */
                    if (garudaData.timing.goodZcCount == 0)
                    {
                        garudaData.timing.zcSynced = false;
                        garudaData.timing.hasPrevZc = false;
                        /* Reset stepPeriod to OL ramp value.  Without this,
                         * the inflated post-sync stepPeriod persists, causing
                         * the motor to run far below ramp target speed and
                         * making ZC detection harder (weaker BEMF). */
                        uint16_t initPeriod = TIMER1_TO_ADC_TICKS(garudaData.rampStepPeriod);
                        if (initPeriod < MIN_ADC_STEP_PERIOD)
                            initPeriod = MIN_ADC_STEP_PERIOD;
                        garudaData.timing.stepPeriod = initPeriod;
                        garudaData.timing.forcedCountdown = initPeriod;
                    }
                }
            }

            /* Duty cycle control.  Pot controls duty in both modes.
             * Pre-sync: capped at RAMP_DUTY_CAP (safety — forced commutation).
             * Post-sync: full range up to MAX_DUTY. */
            {
                uint32_t cap = garudaData.timing.zcSynced ? MAX_DUTY : RAMP_DUTY_CAP;
                uint32_t mappedDuty = MIN_DUTY +
                    ((uint32_t)garudaData.potRaw * (cap - MIN_DUTY)) / 4096;
                if (mappedDuty < MIN_DUTY) mappedDuty = MIN_DUTY;
                if (mappedDuty > cap) mappedDuty = cap;
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

        case ESC_FAULT:
            HAL_MC1PWMDisableOutputs();
            break;
    }

#if FEATURE_BEMF_CLOSED_LOOP
    /* Track state for transition detection (must be last before flag clear) */
    prevAdcState = garudaData.state;
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
            if (garudaData.throttle < 50) /* Near-zero threshold */
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
            if (STARTUP_Align(&garudaData))
            {
                /* Alignment complete — start open-loop ramp */
                garudaData.state = ESC_OL_RAMP;
                garudaData.rampCounter = garudaData.rampStepPeriod;
            }
            break;

#if DIAGNOSTIC_MANUAL_STEP
        case ESC_OL_RAMP:
            /* Manual step mode: hold current step, SW2 advances.
             * Immediately transition to CLOSED_LOOP (which also holds). */
            garudaData.state = ESC_CLOSED_LOOP;
            break;
#else
        case ESC_OL_RAMP:
            if (STARTUP_OpenLoopRamp(&garudaData))
            {
                /* Ramp complete — hold at current speed/duty.
                 * Phase 2 will transition to ESC_CLOSED_LOOP here. */
                garudaData.state = ESC_CLOSED_LOOP;
            }
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
        /* Fault detected — disable outputs and set fault state */
        HAL_MC1ClearPWMPCIFault();
        HAL_MC1PWMDisableOutputs();
        garudaData.state = ESC_FAULT;
        garudaData.faultCode = FAULT_OVERCURRENT;
        LED2 = 0;
    }
    ClearPWMIF();
}
