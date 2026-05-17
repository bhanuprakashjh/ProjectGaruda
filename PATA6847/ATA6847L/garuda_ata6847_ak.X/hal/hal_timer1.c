/**
 * @file hal_timer1.c
 * @brief Timer1 init for 50 µs (20 kHz) periodic interrupt on dsPIC33AK.
 *
 * Clock chain:
 *   FCY = 200 MHz, Timer1 input = FPB (TCS=0) = 200 MHz.
 *   TCKPS=1:8 → 25 MHz Timer1 clock.
 *   PR1 = TIMER1_PR = 25M / 20k - 1 = 1249 (auto from TIMER1_PR macro).
 *
 * The Timer1 ISR is in garuda_service.c.
 */

#include <xc.h>
#include "hal_timer1.h"
#include "../garuda_config.h"

void HAL_Timer1_Init(void)
{
    T1CONbits.ON = 0;       /* Stop timer */
    T1CONbits.SIDL = 0;     /* Continue in idle */
    T1CONbits.TGATE = 0;
    T1CONbits.TCKPS = 0b01; /* Prescaler 1:8 */
    T1CONbits.TCS = 0;      /* Internal peripheral clock */
    T1CONbits.TSYNC = 0;

    TMR1 = 0;
    PR1 = TIMER1_PR;        /* 50 µs period — auto-scaled by FCY */

    _T1IF = 0;
    _T1IP = 4;              /* Priority 4 — below IC(5) */
    _T1IE = 1;
}

void HAL_Timer1_Start(void)
{
    TMR1 = 0;
    T1CONbits.ON = 1;
}

void HAL_Timer1_Stop(void)
{
    T1CONbits.ON = 0;
}
