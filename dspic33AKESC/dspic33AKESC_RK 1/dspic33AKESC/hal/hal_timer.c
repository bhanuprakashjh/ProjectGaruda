/**
 * @file hal_timer.c
 *
 * @brief SCCP timer implementation for hardware ZC detection.
 * SCCP1: 32-bit one-shot for blanking delays and commutation scheduling.
 * SCCP2: 32-bit free-running for high-resolution timestamps (100 MHz).
 *
 * dsPIC33AK SCCP registers: CCP1PR and CCP1TMR are 32-bit (no L/H split).
 *
 * Priority and IE for SCCP1 are managed by GARUDA_ServiceInit() and
 * HWZC_Enable()/HWZC_Disable() — NOT set here (Rule 7).
 *
 * Component: HAL_TIMER
 */

#include "../garuda_config.h"

#if FEATURE_ADC_CMP_ZC

#include <xc.h>
#include "hal_timer.h"

/**
 * @brief Initialize SCCP1 as a 32-bit one-shot timer at FCY (100 MHz).
 * Timer remains idle until HAL_SCCP1_StartOneShot() triggers it.
 */
void HAL_SCCP1_Init(void)
{
    CCP1CON1 = 0;
    CCP1CON1bits.T32 = 1;        /* 32-bit mode */
    CCP1CON1bits.MOD = 0b0000;   /* Timer mode (output disabled) */
    CCP1CON1bits.TMRPS = 0b00;   /* Prescaler 1:1 */
    CCP1CON1bits.CLKSEL = 0b000; /* FCY clock (100 MHz) */
    CCP1CON1bits.ONESHOT = 1;    /* One-shot mode */
    CCP1CON1bits.TRIGEN = 1;     /* Trigger operation enabled */

    _CCT1IF = 0;                 /* Clear flag; IE=0, IP set by ServiceInit */

    CCP1CON1bits.ON = 1;         /* Enable module (timer idle until TRSET) */
}

/**
 * @brief Load period and start a one-shot count on SCCP1.
 * @param ticks  Number of FCY ticks (10ns each) until interrupt fires.
 */
void HAL_SCCP1_StartOneShot(uint32_t ticks)
{
    CCP1PR = ticks;
    CCP1TMR = 0;
    _CCT1IF = 0;
    CCP1STATbits.TRSET = 1;     /* Software trigger -> start counting */
}

/**
 * @brief Stop SCCP1 timer (cancel any pending one-shot).
 */
void HAL_SCCP1_Stop(void)
{
    CCP1STATbits.TRCLR = 1;
}

/**
 * @brief Initialize SCCP2 as a 32-bit free-running timer at FCY (100 MHz).
 * Wraps every ~43 seconds. No interrupt needed.
 */
void HAL_SCCP2_Init(void)
{
    CCP2CON1 = 0;
    CCP2CON1bits.T32 = 1;        /* 32-bit mode */
    CCP2CON1bits.MOD = 0b0000;   /* Timer mode */
    CCP2CON1bits.TMRPS = 0b00;   /* Prescaler 1:1 */
    CCP2CON1bits.CLKSEL = 0b000; /* FCY clock (100 MHz) */
    /* Free-running: max period */
    CCP2PR = 0xFFFFFFFF;
    CCP2CON1bits.ON = 1;
}

/**
 * @brief Read SCCP2 timestamp. CCP2TMR is 32-bit on dsPIC33AK.
 * @return Current timer count (10ns per tick).
 */
uint32_t HAL_SCCP2_ReadTimestamp(void)
{
    return CCP2TMR;
}

/**
 * @brief Initialize SCCP3 as a periodic timer for high-speed ADC triggering.
 * Output drives TRG1SRC=14 (SCCP3 Trigger out) on ADC channels.
 * @param periodTicks  FCY ticks per trigger (e.g. 100 = 1 MHz at 100 MHz FCY).
 */
void HAL_SCCP3_InitPeriodic(uint32_t periodTicks)
{
    CCP3CON1 = 0;
    CCP3CON1bits.T32 = 1;        /* 32-bit mode */
    CCP3CON1bits.MOD = 0b0000;   /* Timer mode (output disabled) */
    CCP3CON1bits.TMRPS = 0b00;   /* Prescaler 1:1 */
    CCP3CON1bits.CLKSEL = 0b000; /* FCY clock (100 MHz) */
    CCP3PR = periodTicks;
    CCP3TMR = 0;
    CCP3CON1bits.ON = 1;
}

/**
 * @brief Stop SCCP3 periodic timer.
 */
void HAL_SCCP3_Stop(void)
{
    CCP3CON1bits.ON = 0;
}

#endif /* FEATURE_ADC_CMP_ZC */
