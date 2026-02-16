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

/* eRPM from adcIsrTick stepPeriod: eRPM = 60 / (stepPeriod * 6 / PWMFREQUENCY_HZ)
 * = PWMFREQUENCY_HZ * 10 / stepPeriod.  Precomputed numerator: */
#define ERPM_FROM_ADC_STEP_NUM      (uint32_t)(PWMFREQUENCY_HZ * 10UL)

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

/* Phase B1: Duty slew rate limits (frequency-independent) */
#if FEATURE_DUTY_SLEW
/* Per ADC ISR tick: MAX_DUTY * percent / 100 / ticks_per_ms
 * ticks_per_ms = PWMFREQUENCY_HZ / 1000 = 24 */
#define DUTY_SLEW_UP_RATE   (uint32_t)( \
    (uint64_t)MAX_DUTY * DUTY_SLEW_UP_PERCENT_PER_MS / 100 \
    / (PWMFREQUENCY_HZ / 1000))
#define DUTY_SLEW_DOWN_RATE (uint32_t)( \
    (uint64_t)MAX_DUTY * DUTY_SLEW_DOWN_PERCENT_PER_MS / 100 \
    / (PWMFREQUENCY_HZ / 1000))
#endif

/* Phase B2: Desync recovery coast-down counts (Timer1 = 100us ticks) */
#if FEATURE_DESYNC_RECOVERY
#define DESYNC_COAST_COUNTS     (uint32_t)(DESYNC_COAST_MS * 10)
#endif

/* Phase B3: Timing advance static asserts */
#if FEATURE_TIMING_ADVANCE
_Static_assert(TIMING_ADVANCE_MAX_DEG < 30,  "Must leave positive delay");
_Static_assert(TIMING_ADVANCE_MIN_DEG <= TIMING_ADVANCE_MAX_DEG, "Min <= Max");
_Static_assert(TIMING_ADVANCE_MAX_DEG <= 25, "Advance > 25 deg risks desync");
_Static_assert(MIN_ADC_STEP_PERIOD > MIN_CL_ADC_STEP_PERIOD,
               "Timing advance interpolation range must be non-zero");
#endif

/* Feature dependency guards */
#if FEATURE_TIMING_ADVANCE && !FEATURE_BEMF_CLOSED_LOOP
#error "Timing advance requires FEATURE_BEMF_CLOSED_LOOP"
#endif
#if FEATURE_DESYNC_RECOVERY && !FEATURE_BEMF_CLOSED_LOOP
#error "Desync recovery requires FEATURE_BEMF_CLOSED_LOOP"
#endif
#if FEATURE_DUTY_SLEW && !FEATURE_BEMF_CLOSED_LOOP
#error "Duty slew requires FEATURE_BEMF_CLOSED_LOOP"
#endif
#if FEATURE_DYNAMIC_BLANKING && !FEATURE_BEMF_CLOSED_LOOP
#error "Dynamic blanking requires FEATURE_BEMF_CLOSED_LOOP"
#endif
#if FEATURE_VBUS_SAG_LIMIT && !FEATURE_BEMF_CLOSED_LOOP
#error "Vbus sag limiting requires FEATURE_BEMF_CLOSED_LOOP"
#endif
#if FEATURE_BEMF_INTEGRATION && !FEATURE_BEMF_CLOSED_LOOP
#error "BEMF integration requires FEATURE_BEMF_CLOSED_LOOP"
#endif
#if FEATURE_BEMF_INTEGRATION
_Static_assert(INTEG_THRESHOLD_GAIN > 0 && INTEG_THRESHOLD_GAIN < 1024,
               "INTEG_THRESHOLD_GAIN out of useful range");
_Static_assert(INTEG_HIT_DIVISOR >= 2 && INTEG_HIT_DIVISOR <= 16,
               "INTEG_HIT_DIVISOR out of range");
#endif

/* Phase D: Sine startup dependency guards and derived constants */
#if FEATURE_SINE_STARTUP && !FEATURE_BEMF_CLOSED_LOOP
#error "Sine startup requires FEATURE_BEMF_CLOSED_LOOP for transition to trap"
#endif
#if FEATURE_SINE_STARTUP && DIAGNOSTIC_MANUAL_STEP
#error "Sine startup and DIAGNOSTIC_MANUAL_STEP cannot be enabled simultaneously"
#endif

#if FEATURE_SINE_STARTUP
/* Center duty for sine (50% of period) */
#define SINE_CENTER_DUTY   ((uint32_t)(LOOPTIME_TCY / 2))

/* Amplitude limits in PWM counts */
#define SINE_MIN_AMPLITUDE ((uint32_t)(LOOPTIME_TCY * SINE_ALIGN_MODULATION_PCT / 200))
#define SINE_MAX_AMPLITUDE ((uint32_t)(LOOPTIME_TCY * SINE_RAMP_MODULATION_PCT / 200))

/* eRPM -> angleIncrement conversion (Q16 fixed-point multiplier).
 *
 * CRITICAL: INITIAL_ERPM and RAMP_TARGET_ERPM are already ELECTRICAL RPM.
 * No MOTOR_POLE_PAIRS factor. See Binding Rule 1.
 *
 * freq_Hz = eRPM / 60
 * angleInc = freq_Hz * 65536 / PWMFREQUENCY_HZ
 *          = eRPM * 65536 / (60 * PWMFREQUENCY_HZ)
 *
 * Q16: angleIncrement = (eRPM * SINE_ERPM_TO_ANGLE_Q16) >> 16 */
#define SINE_ERPM_TO_ANGLE_Q16 \
    ((uint32_t)((uint64_t)65536UL * 65536UL / (60UL * PWMFREQUENCY_HZ)))

/* eRPM ramp rate per Timer1 tick (Q16 fractional).
 * Each Timer1 tick = 100us, so 10000 ticks/sec.
 * erpmFrac += RATE per tick; actual eRPM delta = erpmFrac >> 16 */
#define SINE_ERPM_RAMP_RATE_Q16 \
    ((uint32_t)((uint64_t)RAMP_ACCEL_ERPM_PER_S * 65536UL / 10000UL))

/* Phase offset for sector->step mapping (Q16 angle units) */
#define SINE_PHASE_OFFSET_Q16 \
    ((uint16_t)((uint32_t)SINE_PHASE_OFFSET_DEG * 65536UL / 360UL))

/* Alignment angle: 90 deg = Phase A peak (d-axis toward A) = 16384 Q16 */
#define SINE_ALIGN_ANGLE_Q16   ((uint16_t)16384)

_Static_assert(RAMP_TARGET_ERPM > INITIAL_ERPM,
               "RAMP_TARGET_ERPM must be > INITIAL_ERPM (sine V/f ramp denominator)");
_Static_assert(SINE_ALIGN_MODULATION_PCT >= 5 && SINE_ALIGN_MODULATION_PCT <= 50,
               "SINE_ALIGN_MODULATION_PCT out of range");
_Static_assert(SINE_RAMP_MODULATION_PCT >= 10 && SINE_RAMP_MODULATION_PCT <= 80,
               "SINE_RAMP_MODULATION_PCT out of range");
_Static_assert(SINE_PHASE_OFFSET_DEG < 360,
               "SINE_PHASE_OFFSET_DEG must be 0-359");
#endif

#ifdef __cplusplus
}
#endif

#endif /* GARUDA_CALC_PARAMS_H */
