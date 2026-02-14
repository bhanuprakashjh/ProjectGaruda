/**
 * @file main.c
 *
 * @brief Project Garuda ESC firmware entry point.
 *
 * Initialization sequence:
 *   1. InitOscillator() — 200MHz system clock, 400MHz PWM, 100MHz ADC
 *   2. SetupGPIOPorts() — PWM, BEMF, LED, button, UART, DShot pins
 *   3. HAL_InitPeripherals() — ADC, CMP, PWM, Timer1
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

#if FEATURE_LEARN_MODULES
#include "learn/learn_service.h"
#endif
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

    /* Main loop — all real work happens in ISRs */
    while (1)
    {
        /* Board service — button debounce at 1ms rate */
        BoardService();

        /* Button 1 (SW1) — Start/Stop motor */
        if (IsPressed_Button1())
        {
            if (garudaData.state == ESC_IDLE)
            {
                /* Skip arming — go directly to ALIGN for initial motor testing.
                 * TODO: Restore ESC_ARMED with throttle-zero check for production. */
                garudaData.state = ESC_ALIGN;
                STARTUP_Init(&garudaData);
                LED2 = 1;
                garudaData.armCounter = 0;
#if FEATURE_ADAPTATION
                /* Apply any pending adaptation at the IDLE→ARMED boundary */
                if (ADAPT_IsSafeBoundary(ESC_ARMED, garudaData.throttle))
                {
                    /* Adaptation params already evaluated; applied here */
                }
#endif
            }
            else if (garudaData.state == ESC_FAULT)
            {
                /* Clear fault and return to idle */
                garudaData.faultCode = FAULT_NONE;
                HAL_MC1ClearPWMPCIFault();
                HAL_MC1PWMDisableOutputs();
                garudaData.state = ESC_IDLE;
                LED2 = 0;
            }
            else
            {
                /* Stop motor */
                HAL_MC1PWMDisableOutputs();
                garudaData.state = ESC_IDLE;
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
        /* Button 2 (SW2) — Change direction.
         * For initial testing: allowed in any state so we can find
         * the correct direction without reflashing.
         * TODO: Restrict to IDLE only for production. */
        if (IsPressed_Button2())
        {
            garudaData.direction ^= 1;
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
