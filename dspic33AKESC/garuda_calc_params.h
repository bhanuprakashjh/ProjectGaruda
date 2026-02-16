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

/* Phase 2: Timer1-tick to ADC ISR tick conversion.
 * Timer1 tick = 100us. ADC ISR tick = 1/24000 s = 41.67us.
 * Ratio: 100/41.67 = 2.4. adcIsrTicks = Timer1_ticks * 12/5 */
#if FEATURE_BEMF_CLOSED_LOOP
#define TIMER1_TO_ADC_TICKS(t1)     (uint16_t)(((uint32_t)(t1) * 12) / 5)

/* Step periods in adcIsrTick units */
#define INITIAL_ADC_STEP_PERIOD     TIMER1_TO_ADC_TICKS(INITIAL_STEP_PERIOD)   /* ~800 ticks at 300 eRPM */
#define MIN_ADC_STEP_PERIOD         TIMER1_TO_ADC_TICKS(MIN_STEP_PERIOD)       /* ~120 ticks at 2000 eRPM (ramp handoff) */
/* Closed-loop speed limit — decoupled from ramp target */
#define MAX_CL_STEP_PERIOD_T1      ERPM_TO_STEP_TICKS(MAX_CLOSED_LOOP_ERPM)
#define MIN_CL_ADC_STEP_PERIOD     TIMER1_TO_ADC_TICKS(MAX_CL_STEP_PERIOD_T1) /* ~24 ticks at 10000 eRPM */

/* Compile-time config sanity checks */
_Static_assert(MIN_CL_ADC_STEP_PERIOD > 0,    "MIN_CL_ADC_STEP_PERIOD must be > 0");
_Static_assert(MIN_CL_ADC_STEP_PERIOD <= MIN_ADC_STEP_PERIOD,
               "CL min step period must be <= ramp min step period");
_Static_assert(ZC_FILTER_THRESHOLD >= 1,       "ZC_FILTER_THRESHOLD must be >= 1");
_Static_assert(ZC_TIMEOUT_MULT >= 1,           "ZC_TIMEOUT_MULT must be >= 1");
_Static_assert(ZC_SYNC_THRESHOLD >= 1,         "ZC_SYNC_THRESHOLD must be >= 1");
_Static_assert(ZC_BLANKING_PERCENT < 100,      "ZC_BLANKING_PERCENT must be < 100");
_Static_assert(ZC_ADC_DEADBAND < 512,          "ZC_ADC_DEADBAND must be < 512");
_Static_assert(ZC_STALENESS_LIMIT >= 6,        "ZC_STALENESS_LIMIT must be >= 6 (one e-cycle)");
_Static_assert(ZC_STEP_MISS_LIMIT >= 1,        "ZC_STEP_MISS_LIMIT must be >= 1");
/* Duty-proportional threshold: (vbusRaw * duty) >> SHIFT approximates
 * Vbus * D / 2.  Verify 2*LOOPTIME_TCY is in the right power-of-2 range. */
_Static_assert(2UL * LOOPTIME_TCY >= (1UL << ZC_DUTY_THRESHOLD_SHIFT)
            && 2UL * LOOPTIME_TCY <  (1UL << (ZC_DUTY_THRESHOLD_SHIFT + 1)),
               "ZC_DUTY_THRESHOLD_SHIFT doesn't match 2*LOOPTIME_TCY range");
_Static_assert(ZC_AD2_SETTLE_SAMPLES >= 1 && ZC_AD2_SETTLE_SAMPLES <= 4,
               "ZC_AD2_SETTLE_SAMPLES must be 1-4");
_Static_assert(ZC_PHASE_GAIN_A > 0 && ZC_PHASE_GAIN_A < 65536, "Gain A out of Q15 range");
_Static_assert(ZC_PHASE_GAIN_B > 0 && ZC_PHASE_GAIN_B < 65536, "Gain B out of Q15 range");
_Static_assert(ZC_PHASE_GAIN_C > 0 && ZC_PHASE_GAIN_C < 65536, "Gain C out of Q15 range");
/* Wrap-safety for uint16_t half-range compares in BEMF_ZC_CheckDeadline and
 * BEMF_ZC_CheckTimeout: (now - target) < 0x8000 is correct only when the
 * target is less than 32768 ticks in the future.
 * Largest future target = timeout = stepPeriod * ZC_TIMEOUT_MULT.
 * At slowest speed, stepPeriod = INITIAL_ADC_STEP_PERIOD.
 * Enforce < 16384 to leave margin for ISR jitter. */
_Static_assert((uint32_t)INITIAL_ADC_STEP_PERIOD * ZC_TIMEOUT_MULT < 16384,
               "Max step timeout exceeds uint16 half-range wrap safety margin");
#if ZC_ADAPTIVE_FILTER
_Static_assert(ZC_FILTER_MIN >= 1,             "ZC_FILTER_MIN must be >= 1");
_Static_assert(ZC_FILTER_MAX >= ZC_FILTER_MIN, "ZC_FILTER_MAX must be >= ZC_FILTER_MIN");
#endif
#endif /* FEATURE_BEMF_CLOSED_LOOP */

#ifdef __cplusplus
}
#endif

#endif /* GARUDA_CALC_PARAMS_H */
