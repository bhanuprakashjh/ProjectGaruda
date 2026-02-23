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

/* Closed-loop idle duty floor (minimum duty at pot=0 to maintain ZC-capable speed) */
#if CL_IDLE_DUTY_PERCENT > 0
#define CL_IDLE_DUTY                (uint32_t)((CL_IDLE_DUTY_PERCENT / 100.0f) * LOOPTIME_TCY)
#else
#define CL_IDLE_DUTY                MIN_DUTY
#endif

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
#define MAX_CL_STEP_PERIOD_T1  ((ERPM_TO_STEP_TICKS(MAX_CLOSED_LOOP_ERPM) > 0) ? \
                                 ERPM_TO_STEP_TICKS(MAX_CLOSED_LOOP_ERPM) : 1)
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
/* Post-sync duty settle period in ADC ISR ticks */
#define POST_SYNC_SETTLE_TICKS ((uint16_t)(POST_SYNC_SETTLE_MS * PWMFREQUENCY_HZ / 1000))
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

/* UART1 mutual exclusion: X2CScope and GSP cannot coexist */
#if FEATURE_X2CSCOPE && FEATURE_GSP
#error "FEATURE_X2CSCOPE and FEATURE_GSP are mutually exclusive (both use UART1)"
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

_Static_assert(MORPH_ZC_THRESHOLD <= ZC_SYNC_THRESHOLD,
               "MORPH_ZC_THRESHOLD cannot exceed goodZcCount cap (ZC_SYNC_THRESHOLD)");
_Static_assert(RAMP_TARGET_ERPM > INITIAL_ERPM,
               "RAMP_TARGET_ERPM must be > INITIAL_ERPM (sine V/f ramp denominator)");
_Static_assert(SINE_ALIGN_MODULATION_PCT >= 2 && SINE_ALIGN_MODULATION_PCT <= 50,
               "SINE_ALIGN_MODULATION_PCT out of range");
_Static_assert(SINE_RAMP_MODULATION_PCT >= 5 && SINE_RAMP_MODULATION_PCT <= 80,
               "SINE_RAMP_MODULATION_PCT out of range");
_Static_assert(SINE_PHASE_OFFSET_DEG < 360,
               "SINE_PHASE_OFFSET_DEG must be 0-359");

/* Window schedule validation */
_Static_assert(MORPH_WINDOW_SECTORS >= 1,
               "Need at least 1 windowed sector");
_Static_assert(MORPH_WINDOW_SECTORS < MORPH_HIZ_MAX_SECTORS,
               "Window sectors must leave room for HIZ confidence gate");

/* Verify schedule array length matches MORPH_WINDOW_SECTORS */
_Static_assert(sizeof((const uint8_t[])MORPH_WINDOW_SCHEDULE)
               == MORPH_WINDOW_SECTORS,
               "MORPH_WINDOW_SCHEDULE length must match MORPH_WINDOW_SECTORS");

/* Final schedule entry must be 100% (full Hi-Z) — validated via named macro */
_Static_assert(MORPH_WINDOW_PCT_4 == 100,
               "Last MORPH_WINDOW_PCT entry must be 100 (full Hi-Z)");

/* Min window must leave room for sensing after overhead */
_Static_assert(MORPH_WINDOW_MIN_TICKS >= 5,
               "Min window too small for settle+init+open-skip overhead");
#endif

/* Phase F: ADC Comparator ZC dependency guard and derived constants */
#if FEATURE_ADC_CMP_ZC && !FEATURE_BEMF_CLOSED_LOOP
#error "ADC comparator ZC requires FEATURE_BEMF_CLOSED_LOOP"
#endif

#if FEATURE_ADC_CMP_ZC
/* SCCP2 clock: FCY = 100 MHz -> 10ns per tick -> 1e8 ticks/sec */
#define HWZC_TIMER_HZ           100000000UL

/* eRPM <-> SCCP2 step period conversion.
 * Step freq = eRPM / 10 Hz -> step period = 10 / eRPM seconds.
 * In SCCP2 ticks: (10 / eRPM) * 1e8 = 1e9 / eRPM. */
#define HWZC_ERPM_TO_TICKS(erpm)  (1000000000UL / (erpm))
#define HWZC_TICKS_TO_ERPM(ticks) (1000000000UL / (ticks))

/* ADC ISR tick <-> SCCP2 tick conversion.
 * 1 ADC tick = 1/PWMFREQUENCY_HZ seconds = 1e8/PWMFREQUENCY_HZ SCCP2 ticks.
 * At 24kHz: 1 ADC tick = 100000000/24000 = 4166 SCCP2 ticks.
 * CAUTION: HWZC_SCCP2_TO_ADC truncates to uint16_t via integer division.
 * For stepPeriodHR < 4166, result is 0. Callers MUST clamp to >= 1 before
 * using as a divisor. */
#define HWZC_ADC_TO_SCCP2(t)   ((uint32_t)(t) * (HWZC_TIMER_HZ / PWMFREQUENCY_HZ))
#define HWZC_SCCP2_TO_ADC(t)   ((uint16_t)((t) / (HWZC_TIMER_HZ / PWMFREQUENCY_HZ)))

/* Minimum step period in SCCP2 ticks at MAX_CLOSED_LOOP_ERPM */
#define HWZC_MIN_STEP_TICKS     HWZC_ERPM_TO_TICKS(MAX_CLOSED_LOOP_ERPM)

/* Noise rejection stall limit: if noiseRejectCount reaches this value since
 * HWZC_Enable, the ZC events are dominated by PWM switching noise (stalled
 * motor). Normal operation produces 0 rejects. Stall produces ~17% reject
 * rate at ~3k events/sec → 500 rejects in ~1s. Value must be large enough
 * to survive the morph→CL startup transient (~600 commutations). */
#define HWZC_NOISE_REJECT_LIMIT 500

/* Absolute floor for interval rejection filter (SCCP2 ticks).
 * Prevents IIR cascade: even if stepPeriodHR converges to HWZC_MIN_STEP_TICKS,
 * the interval filter never drops below this floor. Must be between PWM noise
 * interval (4167 ticks at 24kHz) and HWZC_MIN_STEP_TICKS. 2/3 of min step
 * rejects PWM noise while still accepting ZCs at MAX_CLOSED_LOOP_ERPM. */
#define HWZC_NOISE_FLOOR_TICKS  (HWZC_MIN_STEP_TICKS * 2 / 3)

/* Crossover step period threshold (SCCP2 ticks) */
#define HWZC_CROSSOVER_TICKS    HWZC_ERPM_TO_TICKS(HWZC_CROSSOVER_ERPM)

/* SCCP3 period for high-speed ADC triggering.
 * FCY / sample_rate = ticks per trigger pulse.
 * At 1 MHz: 100000000 / 1000000 = 100 ticks = 1us between conversions. */
#define HWZC_SCCP3_PERIOD       (HWZC_TIMER_HZ / HWZC_ADC_SAMPLE_HZ)

/* Stall plausibility: duty limit and debounce in ADC ISR ticks */
#define HWZC_STALL_DUTY_LIMIT   (uint32_t)((uint64_t)MAX_DUTY * HWZC_STALL_DUTY_PCT / 100)
#define HWZC_STALL_DEBOUNCE_TICKS (uint16_t)(HWZC_STALL_DEBOUNCE_MS * PWMFREQUENCY_HZ / 1000)

_Static_assert(HWZC_MIN_INTERVAL_PCT >= 1 && HWZC_MIN_INTERVAL_PCT <= 99,
               "HWZC_MIN_INTERVAL_PCT must be 1..99");

_Static_assert(HWZC_SCCP3_PERIOD >= 21,
               "SCCP3 period too short (ADC needs ~205ns = 21 ticks at 100MHz)");
_Static_assert(HWZC_CROSSOVER_ERPM > 0,
               "HW ZC crossover must be positive");
_Static_assert(HWZC_CROSSOVER_ERPM <= MAX_CLOSED_LOOP_ERPM,
               "HW ZC crossover must be within CL range");
_Static_assert(HWZC_BLANKING_PERCENT >= 1 && HWZC_BLANKING_PERCENT <= 20,
               "HWZC_BLANKING_PERCENT out of range");
#endif

/* Phase G: Hardware Overcurrent Protection derived constants */
#if FEATURE_HW_OVERCURRENT

/* Integer threshold math — milliamp inputs (no float, XC-DSC safe for _Static_assert):
 * V_trip_mV = VREF_mV + (mA * SHUNT_mOHM * GAIN_x100 / 100000)
 * DAC_counts = V_trip_mV * 4096 / VADC_mV
 * Max safe input: 22000 mA (22A) → 22000*3*2495 = 164.67M < 2^32 */
#define OC_TRIP_MV(ma)      (OC_VREF_MV + \
    ((uint32_t)(ma) * OC_SHUNT_MOHM * OC_GAIN_X100 / 100000))
#define OC_MV_TO_COUNTS(mv) ((uint16_t)((uint32_t)(mv) * 4096 / OC_VADC_MV))
#define OC_BIAS_COUNTS       OC_MV_TO_COUNTS(OC_VREF_MV)

/* Leading-edge blanking register value for PGxLEBbits.LEB bitfield.
 * Unlike DTH (full 16-bit register, bits 3:0 unused → formula has *16),
 * LEB is a 12-bit bitfield (bits 15:4) — compiler positions it, so the
 * value IS the PWM clock count directly. Max 4095 = 10.2us @ 400MHz. */
#define OC_LEB_COUNTS       (uint16_t)(OC_LEB_BLANKING_NS / 1000.0f * PWM_CLOCK_MHZ)

/* CMP3/DAC3 operational threshold — active after ZC sync.
 * Mode only changes which PCI target CMP3 drives (CLPCI vs FPCI). */
#define OC_CMP3_DAC_VAL     OC_MV_TO_COUNTS(OC_TRIP_MV(OC_LIMIT_MA))

/* CMP3/DAC3 startup threshold — used during alignment + ramp.
 * High to survive PWM switching noise/ringing. Lowered to OC_CMP3_DAC_VAL
 * when ZC sync is achieved. Raised back on fault/recovery/idle. */
#define OC_CMP3_STARTUP_DAC OC_MV_TO_COUNTS(OC_TRIP_MV(OC_STARTUP_MA))

/* Software thresholds (ADC readback comparison): */
#define OC_SW_LIMIT_ADC     OC_MV_TO_COUNTS(OC_TRIP_MV(OC_SW_LIMIT_MA))
#define OC_FAULT_ADC_VAL    OC_MV_TO_COUNTS(OC_TRIP_MV(OC_FAULT_MA))

/* Static asserts — all integer constant expressions */
_Static_assert(OC_CMP3_DAC_VAL < 4096, "CMP3 operational threshold exceeds 12-bit DAC range");
_Static_assert(OC_CMP3_STARTUP_DAC < 4096, "CMP3 startup threshold exceeds 12-bit DAC range");
_Static_assert(OC_FAULT_ADC_VAL < 4096, "Software fault threshold exceeds ADC range");
_Static_assert(OC_SW_LIMIT_ADC < OC_CMP3_DAC_VAL,
               "Software soft limit must be below CMP3 operational threshold");
_Static_assert(OC_CMP3_DAC_VAL <= OC_CMP3_STARTUP_DAC,
               "Operational threshold must be <= startup threshold");
#if OC_PROTECT_MODE == 2
_Static_assert(OC_CMP3_DAC_VAL < OC_FAULT_ADC_VAL,
               "Mode 2: CMP3 threshold must be below software fault threshold");
#endif

/* Numeric verification (integer truncation at each division):
 * OC_TRIP_MV(1500)  = 1650 + 1500*3*2495/100000 = 1762 mV -> 2187 counts (soft limit)
 * OC_TRIP_MV(1800)  = 1650 + 1800*3*2495/100000 = 1784 mV -> 2214 counts (CLPCI)
 * OC_TRIP_MV(3000)  = 1650 + 3000*3*2495/100000 = 1874 mV -> 2326 counts (SW fault)
 * OC_TRIP_MV(18000) = 1650 + 18000*3*2495/100000 = 2997 mV -> 3719 counts (startup)
 * OC_TRIP_MV(5000)  = 1650 + 5000*3*2495/100000  = 2024 mV -> 2512 counts (ramp gate) */

/* Current-gated ramp threshold (0 = disabled, >0 = hold accel when ibus exceeds) */
#if RAMP_CURRENT_GATE_MA > 0
#define RAMP_CURRENT_GATE_ADC   OC_MV_TO_COUNTS(OC_TRIP_MV(RAMP_CURRENT_GATE_MA))
_Static_assert(RAMP_CURRENT_GATE_ADC < OC_CMP3_DAC_VAL,
               "Ramp current gate must be below CMP3 operational threshold");
#endif

#endif /* FEATURE_HW_OVERCURRENT */

/* ── Runtime parameter macros ────────────────────────────────────────── *
 * When FEATURE_GSP=1, these read from the GSP runtime param system
 * (gspParams for raw values, gspDerived for precomputed ISR values).
 * When FEATURE_GSP=0, they resolve to compile-time constants.
 * All raw param reads are 8/16-bit: atomic on dsPIC33AK (no lock needed). */
#if FEATURE_GSP
  #include "gsp/gsp_params.h"
  /* Raw param reads (ISR-safe: 8/16-bit atomic on dsPIC33AK) */
  #define RT_RAMP_TARGET_ERPM         gspParams.rampTargetErpm
  #define RT_RAMP_ACCEL_ERPM_PER_S    gspParams.rampAccelErpmPerS
  #define RT_TIMING_ADV_MAX_DEG       gspParams.timingAdvMaxDeg
  #define RT_HWZC_CROSSOVER_ERPM      gspParams.hwzcCrossoverErpm
  /* Precomputed derived (set from main context only) */
  #define RT_RAMP_DUTY_CAP            gspDerived.rampDutyCap
  #define RT_CL_IDLE_DUTY             gspDerived.clIdleDuty
  #define RT_SINE_ERPM_RAMP_RATE_Q16  gspDerived.sineErpmRampRateQ16
  #define RT_MIN_STEP_PERIOD          gspDerived.minStepPeriod
  #define RT_MIN_ADC_STEP_PERIOD      gspDerived.minAdcStepPeriod
  #define RT_OC_SW_LIMIT_ADC          gspDerived.ocSwLimitAdc
  #define RT_OC_FAULT_ADC_VAL         gspDerived.ocFaultAdcVal
#else
  #define RT_RAMP_TARGET_ERPM         RAMP_TARGET_ERPM
  #define RT_RAMP_ACCEL_ERPM_PER_S    RAMP_ACCEL_ERPM_PER_S
  #define RT_TIMING_ADV_MAX_DEG       TIMING_ADVANCE_MAX_DEG
  #define RT_HWZC_CROSSOVER_ERPM      HWZC_CROSSOVER_ERPM
  #define RT_RAMP_DUTY_CAP            RAMP_DUTY_CAP
  #define RT_CL_IDLE_DUTY             CL_IDLE_DUTY
  #define RT_SINE_ERPM_RAMP_RATE_Q16  SINE_ERPM_RAMP_RATE_Q16
  #define RT_MIN_STEP_PERIOD          MIN_STEP_PERIOD
  #define RT_MIN_ADC_STEP_PERIOD      MIN_ADC_STEP_PERIOD
  #define RT_OC_SW_LIMIT_ADC          OC_SW_LIMIT_ADC
  #define RT_OC_FAULT_ADC_VAL         OC_FAULT_ADC_VAL
#endif

#ifdef __cplusplus
}
#endif

#endif /* GARUDA_CALC_PARAMS_H */
