/**
 * @file main.c
 *
 * @brief Project Garuda ESC firmware entry point.
 *
 * Initialization sequence:
 *   1. InitOscillator() — 200MHz system clock, 400MHz PWM, 100MHz ADC
 *   2. SetupGPIOPorts() — PWM, BEMF, LED, button, UART, DShot pins
 *   3. HAL_InitPeripherals() — ADC, PWM, Timer1
 *   4. GARUDA_ServiceInit() — state machine data, enable ADC ISR
 *   5. Main loop — button polling, board service (all real work in ISRs)
 *
 * Component: MAIN
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "garuda_types.h"
#include "garuda_config.h"
#include "garuda_service.h"
#include "hal/clock.h"
#include "hal/port_config.h"
#include "hal/board_service.h"
#include "hal/hal_pwm.h"
#include "motor/startup.h"
#include "motor/commutation.h"
#if FEATURE_ADC_CMP_ZC
#include "motor/hwzc.h"
#endif

#if FEATURE_LEARN_MODULES
#include "learn/learn_service.h"
#endif

#include "x2cscope/diagnostics.h"
#if FEATURE_COMMISSION
#include "learn/commission.h"
#endif
#if FEATURE_ADAPTATION
#include "learn/adaptation.h"
#endif

int main(void)
{
    /* Initialize oscillator / PLL */
    InitOscillator();

    /* Configure GPIO pins */
    SetupGPIOPorts();

    /* Initialize all peripherals */
    HAL_InitPeripherals();

    /* Initialize board service (buttons, counters) */
    BoardServiceInit();

    /* Initialize ESC state machine and enable ADC interrupt */
    GARUDA_ServiceInit();

    /* Initialize X2CScope diagnostics (UART1 at 115200 baud via PKoB4) */
#ifdef ENABLE_DIAGNOSTICS
    DiagnosticsInit();
#endif

    /* Main loop — all real work happens in ISRs */
    while (1)
    {
#ifdef ENABLE_DIAGNOSTICS
        DiagnosticsStepMain();  /* X2CScope serial communication */
#endif

        /* Board service — button debounce at 1ms rate */
        BoardService();

        /* Button 1 (SW1) — Start/Stop motor */
        if (IsPressed_Button1())
        {
            if (garudaData.state == ESC_IDLE)
            {
                /* Enter arming — Timer1 ESC_ARMED handler verifies throttle=0
                 * for ARM_TIME_MS, then transitions to ESC_ALIGN.
                 * Init before state change: Timer1 ISR (prio 5) can
                 * preempt main between writes. */
                garudaData.runCommandActive = true;
                garudaData.desyncRestartAttempts = 0;
                garudaData.armCounter = 0;
                garudaData.state = ESC_ARMED;
#if FEATURE_ADAPTATION
                if (ADAPT_IsSafeBoundary(ESC_ARMED, garudaData.throttle))
                {
                    /* Adaptation params already evaluated; applied here */
                }
#endif
            }
            else if (garudaData.state == ESC_FAULT)
            {
                /* Clear fault and return to idle.
                 * State first: ADC ISR (prio 6) sees IDLE immediately,
                 * skips CL case, so HWZC_Disable's fallbackPending=true
                 * is never consumed by the fallback re-seed path. */
                garudaData.state = ESC_IDLE;
                garudaData.runCommandActive = false;
                garudaData.desyncRestartAttempts = 0;
                garudaData.faultCode = FAULT_NONE;
#if FEATURE_ADC_CMP_ZC
                if (garudaData.hwzc.enabled)
                    HWZC_Disable(&garudaData);
                garudaData.hwzc.fallbackPending = false;
#endif
                HAL_MC1ClearPWMPCIFault();
                HAL_MC1PWMDisableOutputs();
                LED2 = 0;
            }
            else
            {
                /* Stop motor (any running state including ESC_RECOVERY).
                 * State first: same preemption safety as fault-clear. */
                garudaData.state = ESC_IDLE;
                garudaData.runCommandActive = false;
                garudaData.desyncRestartAttempts = 0;
#if FEATURE_ADC_CMP_ZC
                if (garudaData.hwzc.enabled)
                    HWZC_Disable(&garudaData);
                garudaData.hwzc.fallbackPending = false;
#endif
                HAL_MC1PWMDisableOutputs();
                LED2 = 0;
            }
        }

#if DIAGNOSTIC_MANUAL_STEP
        /* DIAGNOSTIC: SW2 manually advances one commutation step.
         * Only active when motor is running (ALIGN/OL_RAMP/CLOSED_LOOP). */
        if (IsPressed_Button2())
        {
            if (garudaData.state == ESC_OL_RAMP ||
                garudaData.state == ESC_CLOSED_LOOP)
            {
                COMMUTATION_AdvanceStep(&garudaData);
                HAL_PWM_SetDutyCycle(garudaData.duty);
                LED2 ^= 1;  /* Toggle LED2 as visual step indicator */
            }
        }
#else
        /* Button 2 (SW2) — Change direction (IDLE only — safe) */
        if (IsPressed_Button2())
        {
            if (garudaData.state == ESC_IDLE)
            {
                garudaData.direction ^= 1;
            }
        }
#endif

#if FEATURE_LEARN_MODULES
        /* Learning modules dispatcher (quality/health/adaptation) */
        LEARN_Service(&garudaData, garudaData.systemTick);
#endif

#if FEATURE_COMMISSION
        /* Self-commissioning state machine (when active) */
        if (garudaData.commission.state > COMM_IDLE &&
            garudaData.commission.state < COMM_COMPLETE)
        {
            COMMISSION_Update(&garudaData.commission, &garudaData,
                              &telemRing, garudaData.systemTick);
        }
#endif
    }

    return 0;
}
