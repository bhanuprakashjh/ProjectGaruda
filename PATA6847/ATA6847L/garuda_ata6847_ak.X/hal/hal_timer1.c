/**
 * @file hal_timer1.c
 * @brief Timer1 init for 50 µs (20 kHz) periodic interrupt on dsPIC33AK.
 *
 * Forked from CK `../../garuda_6step_ck.X/hal/hal_timer1.c`. AK uses the
 * same T1CON layout for prescaler / clock-source bits, but the ON bit
 * is named `T1CONbits.ON` (not `T1CONbits.TON` like CK).  TCKPS enum
 * matches CK: 0b01 = 1:8 prescaler.
 *
 * Clock chain on AK:
 *   FCY = 200 MHz, Timer1 input = FPB (FCY in TCS=0 mode) = 200 MHz.
 *   With TCKPS=1:8 → 25 MHz Timer1 clock.
 *   PR1 = TIMER1_PR = 25M / 20k - 1 = 1249  (auto from TIMER1_PR macro).
 *
 * Note CK has FCY=100 MHz → 12.5 MHz Timer1 clock → PR1=624. Same
 * 50 µs tick produced on both MCUs because the TIMER1_PR macro derives
 * from FCY.
 *
 * The Timer1 ISR is in garuda_service.c.
 */

#include <xc.h>
#include "hal_timer1.h"
#include "../garuda_config.h"

void HAL_Timer1_Init(void)
{
    T1CONbits.ON = 0;       /* Stop timer (AK: ON, CK: TON) */
    T1CONbits.SIDL = 0;     /* Continue in idle */
    T1CONbits.TGATE = 0;
    T1CONbits.TCKPS = 0b01; /* Prescaler 1:8 (matches CK enum) */
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
