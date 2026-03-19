/**
 * @file hal_timer1.c
 * @brief Timer1 initialization for 50 µs periodic interrupt on dsPIC33CK.
 *
 * Timer1 runs at FCY/8 = 12.5 MHz.
 * PR1 = 12500000/20000 - 1 = 624.
 * This gives a 50 µs period (20 kHz), matching the ADC ISR rate
 * for consistent ZC timing resolution.
 *
 * The Timer1 ISR is in garuda_service.c.
 */

#include <xc.h>
#include "hal_timer1.h"
#include "../garuda_config.h"

void HAL_Timer1_Init(void)
{
    T1CONbits.TON = 0;      /* Stop timer */
    T1CONbits.TCKPS = 0b01; /* Prescaler 1:8 */
    T1CONbits.TCS = 0;      /* Internal clock (FCY) */

    TMR1 = 0;
    PR1 = TIMER1_PR;        /* 100 µs period */

    IFS0bits.T1IF = 0;      /* Clear flag */
    IPC0bits.T1IP = 4;      /* Priority 4 — below IC(5) so hardware
                             * ZC captures preempt LCOUT fallback */
    IEC0bits.T1IE = 1;      /* Enable interrupt */
}

void HAL_Timer1_Start(void)
{
    TMR1 = 0;
    T1CONbits.TON = 1;
}

void HAL_Timer1_Stop(void)
{
    T1CONbits.TON = 0;
}
