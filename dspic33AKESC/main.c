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
                /* Start arming sequence */
                garudaData.state = ESC_ARMED;
                garudaData.armCounter = 0;
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

        /* Button 2 (SW2) — Change direction (only when idle) */
        if (IsPressed_Button2())
        {
            if (garudaData.state == ESC_IDLE)
            {
                garudaData.direction ^= 1;
            }
        }
    }

    return 0;
}
