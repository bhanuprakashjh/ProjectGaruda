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
#include "hal/hal_comparator.h"
#include "hal/board_service.h"
#include "hal/port_config.h"
#include "motor/commutation.h"
#include "motor/startup.h"

/* Global ESC runtime data — volatile: shared between ISRs and main loop */
volatile GARUDA_DATA_T garudaData;

/* Heartbeat LED counter */
static uint16_t heartbeatCounter = 0;
/* Sub-counter for 1ms system tick from 100us Timer1 */
static uint8_t msSubCounter = 0;

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
    garudaData.bemf.vbusHalf = 0;
    garudaData.bemf.zeroCrossDetected = false;

    garudaData.timing.stepPeriod = INITIAL_STEP_PERIOD;
    garudaData.timing.lastZcTimestamp = 0;
    garudaData.timing.zcInterval = 0;
    garudaData.timing.filterCount = 0;

    /* Enable ADC interrupt to start the control loop */
    GARUDA_ClearADCIF();
    GARUDA_EnableADCInterrupt();
}

/**
 * @brief ADC ISR — runs at PWM rate (24kHz).
 * Reads BEMF/Vbus, runs state machine, updates PWM.
 */
void __attribute__((__interrupt__, auto_psv)) GARUDA_ADC_INTERRUPT(void)
{
    GARUDA_ClearADCIF();

    /* Read ADC values */
    garudaData.vbusRaw = ADCBUF_VBUS;
    garudaData.potRaw = ADCBUF_POT;

    /* Use pot as throttle input for Phase 1 (no DShot yet) */
    garudaData.throttle = garudaData.potRaw;

    /* Update Vbus/2 for comparator reference */
    garudaData.bemf.vbusHalf = garudaData.vbusRaw >> 1;

    /* State machine */
    switch (garudaData.state)
    {
        case ESC_IDLE:
            /* Waiting for button press — handled in main loop */
            break;

        case ESC_ARMED:
            /* Armed state — already handled in Timer1 ISR */
            break;

        case ESC_ALIGN:
            /* Alignment — handled in Timer1 ISR */
            break;

        case ESC_OL_RAMP:
            /* Open-loop ramp — handled in Timer1 ISR */
            break;

        case ESC_CLOSED_LOOP:
            /* Phase 1: hold at current duty (open-loop steady state).
             * Phase 2 will add BEMF ZC commutation here. */
            HAL_PWM_SetDutyCycle(garudaData.duty);
            break;

        case ESC_BRAKING:
            break;

        case ESC_FAULT:
            HAL_MC1PWMDisableOutputs();
            break;
    }
}

/**
 * @brief Timer1 ISR — 100us tick.
 * Handles: heartbeat LED, board service, commutation timing for
 * align/ramp states, 1ms system tick.
 */
void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void)
{
    TIMER1_InterruptFlagClear();

    /* Heartbeat LED — toggle at ~2Hz (250ms) */
    heartbeatCounter++;
    if (heartbeatCounter >= HEART_BEAT_LED_COUNT)
    {
        heartbeatCounter = 0;
        LED1 ^= 1;
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

        case ESC_OL_RAMP:
            if (STARTUP_OpenLoopRamp(&garudaData))
            {
                /* Ramp complete — hold at current speed/duty.
                 * Phase 2 will transition to ESC_CLOSED_LOOP here. */
                garudaData.state = ESC_CLOSED_LOOP;
            }
            break;

        case ESC_CLOSED_LOOP:
            /* Phase 1: keep forced commutation at final ramp speed.
             * Phase 2 will replace this with BEMF ZC-driven commutation. */
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

        case ESC_BRAKING:
            break;

        case ESC_FAULT:
            break;
    }
}

/**
 * @brief PWM Fault ISR — handles PCI fault events.
 */
void __attribute__((__interrupt__, auto_psv)) _PWMInterrupt(void)
{
    ClearPWMIF();

    if (PCI_FAULT_ACTIVE_STATUS)
    {
        /* Fault detected — disable outputs and set fault state */
        HAL_MC1PWMDisableOutputs();
        garudaData.state = ESC_FAULT;
        garudaData.faultCode = FAULT_OVERCURRENT;
        LED2 = 0;
    }
}
