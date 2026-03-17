/**
 * @file hal_trap.c
 * @brief Trap/exception handlers for dsPIC33CK.
 *
 * Without these handlers, any hardware trap (address error, stack overflow,
 * math error, etc.) causes the MCU to silently hang — no fault LED, no
 * UART output, no heartbeat. These handlers kill PWM immediately and
 * provide diagnostic output.
 *
 * Trap sources that can hang the MCU:
 *   - Address error: invalid memory access (bad pointer, misaligned word)
 *   - Stack error: stack overflow from deep ISR nesting
 *   - Math error: integer divide by zero
 *   - Oscillator fail: PLL lost lock
 *   - DMA error: invalid DMA transfer
 *   - Default: any unhandled interrupt vector
 */

#include <xc.h>
#include "hal_pwm.h"
#include "hal_uart.h"
#include "port_config.h"
#include "../garuda_config.h"

#if FEATURE_IC_ZC
#include "hal_ic.h"
#include "hal_com_timer.h"
#endif

/* Kill all outputs immediately — safe from any context */
static void TrapSafeStop(void)
{
    /* Disable PWM outputs — highest priority */
    HAL_PWM_DisableOutputs();

#if FEATURE_IC_ZC
    /* Stop ZC polling and commutation timer */
    _CCT1IE = 0;
    _CCP4IE = 0;
#endif

    /* Fault LED on, run LED off */
    LED_FAULT = 1;
    LED_RUN = 0;
}

/* Print trap name and hang with blinking fault LED.
 * We don't reset because the user needs to see which trap fired. */
static void TrapReport(const char *name)
{
    TrapSafeStop();

    HAL_UART_WriteString("\r\n!!! TRAP: ");
    HAL_UART_WriteString(name);
    HAL_UART_WriteString(" !!!\r\n");

    /* Blink fault LED forever — distinguishable from normal fault (solid) */
    volatile uint32_t i;
    while (1)
    {
        for (i = 0; i < 500000UL; i++) { }
        LED_FAULT ^= 1;
    }
}

/* ── Trap Handlers ─────────────────────────────────────────────────── */

void __attribute__((interrupt, no_auto_psv)) _OscillatorFail(void)
{
    INTCON1bits.OSCFAIL = 0;
    TrapReport("OSCFAIL");
}

void __attribute__((interrupt, no_auto_psv)) _AddressError(void)
{
    INTCON1bits.ADDRERR = 0;
    TrapReport("ADDRERR");
}

void __attribute__((interrupt, no_auto_psv)) _StackError(void)
{
    INTCON1bits.STKERR = 0;
    TrapReport("STKERR");
}

void __attribute__((interrupt, no_auto_psv)) _MathError(void)
{
    INTCON1bits.MATHERR = 0;
    TrapReport("MATHERR");
}

/* Catch-all for any unhandled interrupt vector */
void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void)
{
    TrapReport("DEFAULT_INT");
}
