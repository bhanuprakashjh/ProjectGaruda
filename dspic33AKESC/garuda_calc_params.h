/**
 * @file garuda_calc_params.h
 *
 * @brief Derived constants computed from garuda_config.h values.
 * Do not edit these directly — change garuda_config.h instead.
 *
 * Component: CALCULATED PARAMETERS
 */

#ifndef GARUDA_CALC_PARAMS_H
#define GARUDA_CALC_PARAMS_H

#include "garuda_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* PWM clock is 400 MHz (from clock.c CLK5 config) */
#define PWM_CLOCK_MHZ               400

/* Loop time (one PWM period) in microseconds */
#define LOOPTIME_MICROSEC           (1000000.0f / PWMFREQUENCY_HZ)

/* Loop time in PWM clock counts (center-aligned: period register value)
 * Formula from reference: (LOOPTIME_MICROSEC * 8 * PWM_CLOCK_MHZ) - 16
 * 8x multiplier due to PCLKCON DIVSEL=0 (1:2) and center-aligned (2x) */
#define LOOPTIME_TCY                (uint32_t)((LOOPTIME_MICROSEC * 8 * PWM_CLOCK_MHZ) - 16)

/* Dead time in PWM clock counts
 * Formula from reference: DEADTIME_NS/1000 * 16 * PWM_CLOCK_MHZ
 * 16x multiplier for dead-time register resolution */
#define DEADTIME_COUNTS             (uint32_t)((DEADTIME_NS / 1000.0f) * 16 * PWM_CLOCK_MHZ)

/* Duty cycle limits */
#define MIN_DUTY                    (uint32_t)(DEADTIME_COUNTS + DEADTIME_COUNTS)
#define MAX_DUTY                    (LOOPTIME_TCY - (uint32_t)(DEADTIME_COUNTS + DEADTIME_COUNTS))

/* Alignment duty cycle in PWM counts */
#define ALIGN_DUTY                  (uint32_t)((ALIGN_DUTY_PERCENT / 100.0f) * LOOPTIME_TCY)

/* Ramp duty cap in PWM counts */
#define RAMP_DUTY_CAP               (uint32_t)((RAMP_DUTY_PERCENT / 100.0f) * LOOPTIME_TCY)

/* Timer1 period = 100us, so alignment time in Timer1 ticks */
#define ALIGN_TIME_COUNTS           (uint32_t)(ALIGN_TIME_MS * 10)

/* Arming time in Timer1 ticks (100us per tick) */
#define ARM_TIME_COUNTS             (uint32_t)(ARM_TIME_MS * 10)

/* eRPM to commutation step period conversion
 * eRPM = (60 * 1e6) / (stepPeriod_us * 6)
 * stepPeriod_us = (60 * 1e6) / (eRPM * 6) = 10000000 / eRPM
 * Timer1 runs at 100us ticks, so:
 * stepPeriod_ticks = 10000000 / (eRPM * 100) = 100000 / eRPM */
#define ERPM_TO_STEP_TICKS(erpm)    (uint32_t)(100000UL / (erpm))

/* Initial forced commutation step period (Timer1 ticks) */
#define INITIAL_STEP_PERIOD         ERPM_TO_STEP_TICKS(INITIAL_ERPM)

/* Minimum step period (fastest commutation = ramp target) */
#define MIN_STEP_PERIOD             ERPM_TO_STEP_TICKS(RAMP_TARGET_ERPM)

/* Step period decrement per Timer1 tick during ramp
 * Acceleration in eRPM/s → need to decrease step period over time
 * Computed in startup.c at runtime for better precision */

/* Bootstrap charging parameters */
#define BOOTSTRAP_CHARGING_TIME_SECS    0.015f
#define BOOTSTRAP_CHARGING_COUNTS       (uint32_t)(BOOTSTRAP_CHARGING_TIME_SECS * PWMFREQUENCY_HZ)
#define TICKLE_CHARGE_TIME_MICROSEC     1.0f
#define TICKLE_CHARGE_DUTY              (LOOPTIME_TCY - (uint32_t)(TICKLE_CHARGE_TIME_MICROSEC * 16 * PWM_CLOCK_MHZ))

/* ADC sampling point (trigger position within PWM period) */
#define ADC_SAMPLING_POINT              0

#ifdef __cplusplus
}
#endif

#endif /* GARUDA_CALC_PARAMS_H */
