/**
 * @file hal_com_timer.h
 * @brief SCCP4 free-running HR timer + SCCP3 commutation scheduler.
 *
 * Current architecture (V4 sector PI):
 *   SCCP4 — Free-running 16-bit timer at 1.5625 MHz (640 ns/tick).
 *           Provides timestamps to the ADC ISR midpoint sampler and
 *           the sector PI scheduler.
 *   SCCP3 — One-shot timer; HAL_ComTimer_ScheduleAbsolute writes
 *           CCP3PR = (target - now), and CCT3IF fires when CCP3TMR
 *           reaches the period. ISR triggers commutation.
 *
 * SCCP CLKSEL=000 routes Standard Speed Peripheral Clock (FPB/2 =
 * 100 MHz on AK; FPB = 100 MHz on CK — identical SCCP input rate
 * on both MCUs). TMRPS=0b11 (1:64) gives 640 ns tick.
 *
 * Enabled by FEATURE_V4_SECTOR_PI=1 (or legacy FEATURE_IC_ZC).
 */

#ifndef HAL_COM_TIMER_H
#define HAL_COM_TIMER_H

#include "../garuda_config.h"

#if FEATURE_IC_ZC || FEATURE_V4_SECTOR_PI

#include <stdint.h>
#include <xc.h>

/* Conversion: Timer1 ticks (50 us) to com timer ticks (640 ns).
 * 50000 ns / 640 ns = 78.125 = 625/8.
 * Usage: delay_ct = (uint32_t)delay_t1 * COM_TIMER_T1_NUMER / COM_TIMER_T1_DENOM */
#define COM_TIMER_T1_NUMER  625U
#define COM_TIMER_T1_DENOM  8U

/* ISR priority: 6 — above IC(5) and Timer1(4). Commutation timing
 * is the most time-critical interrupt in the system. */
#define COM_TIMER_ISR_PRIORITY  6

/* HR step period above this Timer1 threshold overflows uint16_t.
 * 65535 / 78.125 ≈ 839. Use HR only when stepPeriod < 800. */
#define HR_MAX_STEP_PERIOD  800U  /* HR scheduling for Tp < 800. Required for
                                    * mid-speed stability (Test B2 confirmed desync
                                    * without HR at Tp:13). Does NOT fix ZcI
                                    * alternation — that's comparator offset. */

/**
 * @brief Initialize SCCP4 as free-running timer + output compare.
 * Timer starts running immediately. Compare interrupt disabled until scheduled.
 */
void HAL_ComTimer_Init(void);

/**
 * @brief Read current SCCP4 timer value (640 ns/tick).
 * Use in IC ISRs for high-resolution ZC timestamps.
 */
static inline uint16_t HAL_ComTimer_ReadTimer(void)
{
    return CCP4TMR;
}

/**
 * @brief Schedule commutation at an absolute timer tick.
 * Sets CCP4RA and enables compare match interrupt. ISR fires when
 * CCP4TMR reaches targetTick.
 * @param targetTick Absolute SCCP4 tick for commutation.
 */
void HAL_ComTimer_ScheduleAbsolute(uint16_t targetTick);

/**
 * @brief Cancel any pending commutation timer.
 * Disables compare interrupt. Timer keeps running (free-running).
 */
void HAL_ComTimer_Cancel(void);

#endif /* FEATURE_IC_ZC || FEATURE_V4_SECTOR_PI */
#endif /* HAL_COM_TIMER_H */
