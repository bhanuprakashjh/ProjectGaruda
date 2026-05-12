/**
 * @file hal_trap.c
 * @brief Trap/exception handlers for dsPIC33AK128MC106.
 *
 * Forked from CK `../../garuda_6step_ck.X/hal/hal_trap.c`. Trap vector
 * names differ on dsPIC33AK — XC-DSC uses `_BusErrorTrap`,
 * `_IllegalInstructionTrap`, `_AddressErrorTrap`, `_StackErrorTrap`,
 * `_MathErrorTrap`, `_GeneralTrap`, `_DefaultInterrupt`. The legacy
 * dsPIC33CK vector `_OscillatorFail` is replaced by clock-loss handling
 * inside `_BusErrorTrap` on AK.
 *
 * Without these, any CPU trap silently resets the MCU — looks like a
 * mysterious "hard fault" with no diagnostic info.  Each handler kills
 * PWM, reports the trap on UART, then blinks LED_FAULT forever.
 */

#include <xc.h>
#include "hal_pwm.h"
#include "hal_uart.h"
#include "port_config.h"
#include "../garuda_config.h"

/* Kill all outputs immediately — safe from any context */
static void TrapSafeStop(void)
{
    HAL_PWM_DisableOutputs();
    LED_RUN = 0;
    LED_FAULT = 0;
}

/* Blink N times fast, then long pause. Repeat forever.
 * Count = trap ID per the table below:
 *   1 = BusError
 *   2 = IllegalInstruction
 *   3 = AddressError
 *   4 = StackError
 *   5 = MathError
 *   6 = GeneralTrap
 *   7 = DefaultInterrupt (unhandled IRQ)
 */
static void TrapBlinkLoop(uint8_t blinks, const char *name)
{
    TrapSafeStop();

    /* Mask all interrupts.  XC-DSC doesn't support __builtin_disi on AK,
     * so use the generic disable-interrupts builtin instead. */
    __builtin_disable_interrupts();

    HAL_UART_WriteString("\r\n!!! TRAP: ");
    HAL_UART_WriteString(name);
    HAL_UART_WriteString(" !!!\r\n");

    /* At Fcy=200 MHz, one iteration of the inner loop is ~3-4 cycles.
     * 10,000,000 iterations ≈ 30M cycles ≈ 150 ms — visibly slow. */
    volatile uint32_t i;
    while (1)
    {
        for (uint8_t b = 0; b < blinks; b++)
        {
            LED_FAULT = 1;
            for (i = 0; i < 10000000UL; i++) { }   /* ~150 ms ON */
            LED_FAULT = 0;
            for (i = 0; i < 10000000UL; i++) { }   /* ~150 ms OFF */
        }
        /* Long pause between bursts — ~1.5 seconds */
        for (i = 0; i < 100000000UL; i++) { }
    }
}

/* ── Trap Handlers (dsPIC33AK) ─────────────────────────────────────── */

void __attribute__((__interrupt__, no_auto_psv)) _BusErrorTrap(void)
{
    TrapBlinkLoop(1, "BUS_ERR");
}

void __attribute__((__interrupt__, no_auto_psv)) _IllegalInstructionTrap(void)
{
    TrapBlinkLoop(2, "ILLEGAL");
}

void __attribute__((__interrupt__, no_auto_psv)) _AddressErrorTrap(void)
{
    INTCON1bits.ADDRERR = 0;
    TrapBlinkLoop(3, "ADDR_ERR");
}

void __attribute__((__interrupt__, no_auto_psv)) _StackErrorTrap(void)
{
    INTCON1bits.STKERR = 0;
    TrapBlinkLoop(4, "STK_ERR");
}

void __attribute__((__interrupt__, no_auto_psv)) _MathErrorTrap(void)
{
    TrapBlinkLoop(5, "MATH_ERR");
}

void __attribute__((__interrupt__, no_auto_psv)) _GeneralTrap(void)
{
    TrapBlinkLoop(6, "GENERAL");
}

/* Catch-all for any unhandled interrupt vector */
void __attribute__((__interrupt__, no_auto_psv)) _DefaultInterrupt(void)
{
    TrapBlinkLoop(7, "DEFAULT_INT");
}
