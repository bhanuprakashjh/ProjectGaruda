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

/* SCCP peripheral bus clock (FCY = 100 MHz) — shared by all SCCP timers */
#define SCCP_CLOCK_HZ  100000000UL

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
#if FEATURE_CL_LOW_IDLE
/* Lower CL-idle-only floor (< MIN_DUTY). Deadtime unchanged; H-pulse shorter. */
#define CL_LOW_IDLE_FLOOR           (uint32_t)((DEADTIME_COUNTS * CL_LOW_IDLE_DT_PCT) / 100)
_Static_assert(CL_LOW_IDLE_DT_PCT >= 100, "CL idle floor must exceed 1x deadtime (else no H-pulse)");
#endif
#if FEATURE_CL_DIFF_IDLE
/* Differential-low CL idle floor — EFFECTIVE duty (line-line volts fraction).
 * Applied as active = MIN_DUTY + duty, low = MIN_DUTY (complementary), so the
 * physical H-pulses always exceed the deadtime swallow. */
#if FEATURE_CL_ENTRY_GLIDE
/* Glide mode: steady-state diff idle NEUTRALIZED — floor pinned to MIN_DUTY
 * makes the CL idle mapping numerically identical to the baseline (pot-0
 * target = MIN_DUTY → 10.4k equilibrium, never re-enters diff at idle). The
 * diff waveform only runs during the post-coast entry glide. */
#define CL_DIFF_IDLE_FLOOR          ((uint32_t)MIN_DUTY)
#else
#define CL_DIFF_IDLE_FLOOR          (uint32_t)(((uint32_t)LOOPTIME_TCY * CL_DIFF_IDLE_PCT_X10) / 1000UL)
_Static_assert(CL_DIFF_IDLE_PCT_X10 >= 1, "diff idle floor must be > 0 (0V idle would stall)");
#endif
#endif
/* Raised to 99% (2026-05-26): old formula reserved 2×deadtime of L-side on-time
 * (max 94.6% at 45kHz × 300ns DT). Past ~94% the L-FET is going to be suppressed
 * anyway (PWM peripheral handles deadtime internally — when requested H-pulse
 * leaves no room for L+2×DT, L just doesn't fire). So allow up to 99% and let
 * the module take over at the top. Effectively this gives BC-like behavior
 * above ~94% without an explicit BC mode. */
#define MAX_DUTY                    ((uint32_t)((LOOPTIME_TCY * 99UL) / 100UL))

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

/* ADC sampling point (PG1TRIGA position within the PWM period). This is WHERE
 * in the cycle the mid-ON BEMF/current sample is taken AND when the 45 kHz
 * control ISR fires — so it anchors the SCCP2 ZC timestamp for the high-speed
 * mid-ON ZC path (HWZC_OnSoftwareSample).
 *
 * The value is PWM-MODULE-SPECIFIC, not portable. The 106 maps count 0 to the
 * center of the driven ON pulse, so 0 = mid-ON there. On the MC510's PWM the
 * center-aligned ON pulse centers at LOOPTIME_TCY/2 (AN957 samples phase
 * current mid-ON at exactly LOOPTIME_TCY/2) — so 0 here samples at the period
 * boundary (mid-OFF/freewheel), ~half a PWM period (~11 us) away from mid-ON.
 * That fixed timestamp offset becomes a commutation phase error of omega*11us,
 * which GROWS LINEARLY with eRPM: bench 2026-06-13 showed current rising ~linearly
 * to the 22 A OC chop and desync ~174k (vs the 106's 232k). Re-derive per module. */
#if GARUDA_TARGET_AK512
/* Sample at the OFF-CENTER (freewheel) point, NOT the ON-pulse center.
 * EMPIRICAL (4 bench points, fire-time within the 22.2us PWM period):
 *   t=0       (TRIGA0  /CA0) -> 174k
 *   t=5.55us  (MPER2   /CA0) -> 215k  <-- PEAK
 *   t=11.1us  (TRIGA0  /CA1, pulse center) -> 174k+OC   (refutes "center")
 *   t=16.7us  (MPER2   /CA1) -> 115k
 * Quality peaks at t=5.55us = the MIDDLE of cycle-1's freewheel window (at low/
 * mid duty the ON pulse sits in the LAST ~5-50% of the cycle, so MPER/2 lands
 * in the clean freewheel where the floating phase shows BEMF undistorted by
 * driven-phase coupling — matches the firmware's OFF-center detection design).
 * The pulse CENTER (boundary) is NOISY for this scheme, not optimal.
 * KNOWN LIMIT: this is a FIXED point; above ~50% duty the ON pulse swallows
 * MPER/2 -> sample no longer OFF-center -> current climbs to OC near the top.
 * The 106 reached 232k by using the 1 MHz ADC comparator at high speed instead
 * of this fixed mid-sample (see study notes / next step). */
#define ADC_SAMPLING_POINT              ((uint32_t)(LOOPTIME_TCY / 2))
#else
#define ADC_SAMPLING_POINT              0
#endif

/* Phase 2: Timer1-tick to ADC ISR tick conversion.
 * Timer1 tick = 100 us. The ADC ISR is PG1TRIGA-triggered once per PWM
 * period, so its tick = 1/PWMFREQUENCY_HZ (22.22 us at 45 kHz) and the
 * ratio = 100 us * PWMFREQUENCY_HZ / 1e6 = 4.5 -- derived from the PWM
 * frequency so it can never rot again.
 * BUG FIXED 2026-06-11: this was hardcoded *12/5 (the 24 kHz factor) ever
 * since the 24->45 kHz migration -- every Timer1-seeded period (morph
 * handoff, CL entry seed, HWZC enable seed) was 1.875x too fast: the
 * "5696 eRPM" hand-off from a 3,000 eRPM rotor, and the morph IIR clamp
 * [seed, 1.5*seed] = [79,118] that FORBADE the true 150-tick period. */
#if FEATURE_BEMF_CLOSED_LOOP
#define TIMER1_TO_ADC_TICKS(t1)     (uint16_t)(((uint32_t)(t1) * PWMFREQUENCY_HZ) / 10000UL)

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
/* P0: Exact divisor for duty-proportional threshold. Replaces >>18 shift
 * which had +1.7% bias (2^18=262144 vs 2*LOOPTIME_TCY=266636).
 * neutral = Vbus * duty / (2 * LOOPTIME_TCY).
 * Compile-time constant — compiler strength-reduces to reciprocal multiply. */
/* 2026-06-17 PER-PROFILE: high-KV micros (VEX prof 6, 1407 prof 7/8 @10V) lower the
 * Vdc*D/2 threshold ~23% (2.6 vs 2.0) to pull detection onto neutral on their tiny BEMF.
 * 2810 and everything else use the exact Vdc*D/2 neutral (2.0). MOTOR_PROFILE comes from
 * garuda_config.h, which is included before this header. */
#if MOTOR_PROFILE == 6 || MOTOR_PROFILE == 7 || MOTOR_PROFILE == 8
#define ZC_DUTY_DIVISOR  ((26UL * LOOPTIME_TCY) / 10UL)
#else
#define ZC_DUTY_DIVISOR  (2UL * LOOPTIME_TCY)
#endif
_Static_assert(ZC_DUTY_DIVISOR > 0 && ZC_DUTY_DIVISOR < 0x100000UL,
               "ZC_DUTY_DIVISOR out of expected range");
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
#if FEATURE_CL_SOFT_ENTRY
/* Soft CL-entry window in ADC ISR ticks */
#define CL_SOFT_ENTRY_TICKS ((uint16_t)(CL_SOFT_ENTRY_MS * PWMFREQUENCY_HZ / 1000))
_Static_assert(CL_SOFT_ENTRY_DIVISOR >= 1, "CL_SOFT_ENTRY_DIVISOR must be >= 1");
#endif
#else /* !FEATURE_DUTY_SLEW */
#if FEATURE_CL_SOFT_ENTRY
#error "FEATURE_CL_SOFT_ENTRY requires FEATURE_DUTY_SLEW=1"
#endif
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

/* FOC mutual exclusion guards */
#if FEATURE_FOC && FEATURE_FOC_V2
#error "FEATURE_FOC (v1) and FEATURE_FOC_V2 are mutually exclusive"
#endif
#if FEATURE_FOC && FEATURE_BEMF_CLOSED_LOOP
#error "FEATURE_FOC and FEATURE_BEMF_CLOSED_LOOP are mutually exclusive"
#endif
#if FEATURE_FOC_V2 && FEATURE_BEMF_CLOSED_LOOP
#error "FEATURE_FOC_V2 and FEATURE_BEMF_CLOSED_LOOP are mutually exclusive"
#endif
#if FEATURE_FOC && FEATURE_SINE_STARTUP
#error "FEATURE_FOC and FEATURE_SINE_STARTUP are mutually exclusive"
#endif
#if FEATURE_FOC_V2 && FEATURE_SINE_STARTUP
#error "FEATURE_FOC_V2 and FEATURE_SINE_STARTUP are mutually exclusive"
#endif
#if FEATURE_FOC && FEATURE_ADC_CMP_ZC
#error "FEATURE_FOC and FEATURE_ADC_CMP_ZC are mutually exclusive"
#endif
#if FEATURE_FOC_V2 && FEATURE_ADC_CMP_ZC
#error "FEATURE_FOC_V2 and FEATURE_ADC_CMP_ZC are mutually exclusive"
#endif
#if FEATURE_PLL_STARTUP && (!FEATURE_ADC_CMP_ZC || !FEATURE_HWZC_SECTOR_PI)
#error "FEATURE_PLL_STARTUP requires ADC_CMP_ZC + HWZC_SECTOR_PI (the PI/SCCP1 machinery)"
#endif
#if FEATURE_AM32_STARTUP && (!FEATURE_ADC_CMP_ZC || !FEATURE_HWZC_SECTOR_PI)
#error "FEATURE_AM32_STARTUP requires ADC_CMP_ZC + HWZC_SECTOR_PI (the listener)"
#endif
#if FEATURE_FOC && DIAGNOSTIC_MANUAL_STEP
#error "FEATURE_FOC and DIAGNOSTIC_MANUAL_STEP are mutually exclusive"
#endif
#if FEATURE_FOC_V2 && DIAGNOSTIC_MANUAL_STEP
#error "FEATURE_FOC_V2 and DIAGNOSTIC_MANUAL_STEP are mutually exclusive"
#endif

/* MXLEMMING requires FOC v1 */
#if FEATURE_MXLEMMING && !FEATURE_FOC
#error "FEATURE_MXLEMMING requires FEATURE_FOC"
#endif

/* FOC SVM switches all 3 phases at different instants — LEB only covers one
 * edge, so CMP3 false-trips on switching ringing. Force CLPCI off; software
 * overcurrent (OC_PROTECT_MODE 1 or 2 software path) still protects. */
#if (FEATURE_FOC || FEATURE_FOC_V2 || FEATURE_FOC_V3 || FEATURE_FOC_AN1078) && OC_CLPCI_ENABLE
#undef OC_CLPCI_ENABLE
#define OC_CLPCI_ENABLE 0
#endif

/* 6-step feature dependency guards (N/A when FOC active) */
#if !FEATURE_FOC && !FEATURE_FOC_V2 && !FEATURE_FOC_V3 && !FEATURE_FOC_AN1078
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
#endif /* !FEATURE_FOC && !FEATURE_FOC_V2 — end of 6-step dependency guards */

#if FEATURE_ADC_CMP_ZC
/* SCCP2 clock = SCCP peripheral bus = 100 MHz */
#define HWZC_TIMER_HZ           SCCP_CLOCK_HZ

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

/* Step period threshold for skipping verify reads (high-RPM latency optimization).
 * Below this many ticks, motor is spinning fast enough that the ~3µs verify
 * overhead matters more than the noise immunity. */
#define HWZC_VERIFY_SKIP_TICKS  HWZC_ERPM_TO_TICKS(HWZC_VERIFY_SKIP_ERPM)

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
#define HWZC_NO_CAPTURE_TICKS     (uint16_t)(HWZC_NO_CAPTURE_MS    * PWMFREQUENCY_HZ / 1000)

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

/* Option D: I-f current-limited hand-off bridge thresholds (frequency-independent).
 * IF_BRIDGE_LIMIT_ADC uses the same bias+scale as OC_SW_LIMIT_ADC, so it's
 * directly comparable to garudaData.ibusRaw. */
#if FEATURE_IF_BRIDGE
#define IF_BRIDGE_LIMIT_ADC  OC_MV_TO_COUNTS(OC_TRIP_MV(IF_BRIDGE_LIMIT_MA))
#define IF_BRIDGE_TICKS      ((uint16_t)((uint32_t)IF_BRIDGE_MS * PWMFREQUENCY_HZ / 1000))
#define IF_BRIDGE_UP_RATE    (uint32_t)((uint64_t)MAX_DUTY * IF_BRIDGE_UP_PCT_PER_MS \
                                        / 100 / (PWMFREQUENCY_HZ / 1000))
#define IF_BRIDGE_DOWN_RATE  (uint32_t)((uint64_t)MAX_DUTY * IF_BRIDGE_DOWN_PCT_PER_MS \
                                        / 100 / (PWMFREQUENCY_HZ / 1000))
/* Cap expressed as a MAGNITUDE delta from the zero-current bias, so the limiter
 * backs off on BOTH motoring (+) and regen (-) excursions. During the unlocked
 * hand-off the bus current swings strongly negative (phase-mismatch slosh) — a
 * SIGNED compare is fooled into "current is low" and ramps duty up. */
#define IF_BRIDGE_LIMIT_DELTA ((int32_t)IF_BRIDGE_LIMIT_ADC - (int32_t)OC_BIAS_COUNTS)
_Static_assert(IF_BRIDGE_LIMIT_ADC < OC_FAULT_ADC_VAL,
               "IF_BRIDGE_LIMIT must be below the OC fault threshold");
_Static_assert(IF_BRIDGE_LIMIT_DELTA > 0, "IF_BRIDGE_LIMIT must exceed the zero bias");
#endif

/* Hand-off CMP3 chop: a LOW cycle-by-cycle current threshold held for a window
 * at CL entry. Same DAC scale as OC_CMP3_DAC_VAL / OC_CMP3_STARTUP_DAC. */
#if FEATURE_HANDOFF_CHOP
#define OC_CMP3_HANDOFF_DAC  OC_MV_TO_COUNTS(OC_TRIP_MV(OC_CMP3_HANDOFF_MA))
#define HANDOFF_CHOP_TICKS   ((uint16_t)((uint32_t)HANDOFF_CHOP_MS * PWMFREQUENCY_HZ / 1000))
_Static_assert(OC_CMP3_HANDOFF_DAC < OC_CMP3_STARTUP_DAC,
               "hand-off chop must be below the startup chop threshold");
_Static_assert(OC_CMP3_HANDOFF_DAC > OC_BIAS_COUNTS,
               "hand-off chop must be above the zero-current bias");
#endif

/* CL-entry soft-start derived constants (see FEATURE_CL_ENTRY_SOFTSTART). */
#if FEATURE_CL_ENTRY_SOFTSTART
#define CL_ENTRY_START_DUTY  (uint32_t)((CL_ENTRY_START_PCT / 100.0f) * LOOPTIME_TCY)
#define CL_ENTRY_RAMP_TICKS  ((uint16_t)((uint32_t)CL_ENTRY_RAMP_MS * PWMFREQUENCY_HZ / 1000))
_Static_assert(CL_ENTRY_RAMP_TICKS > 0, "CL-entry ramp must span >=1 tick");
#endif

/* I-f spin-up derived constants (the control loop runs in the 24kHz ADC ISR). */
#if FEATURE_IF_STARTUP
#define IF_ERPM_TO_RAD_S(e)  ((float)(e) * 0.104719755f)   /* eRPM → elec rad/s */
#define IF_HANDOFF_RAD_S     IF_ERPM_TO_RAD_S(IF_HANDOFF_ERPM)
#define IF_ALIGN_TICKS       ((uint16_t)((uint32_t)IF_ALIGN_MS * 24u))  /* 24kHz */
#define IF_DT_S              (1.0f / 24000.0f)
#define IF_INV_SQRT3         0.57735027f
#endif

/* Static asserts — all integer constant expressions */
_Static_assert(OC_CMP3_DAC_VAL < 4096, "CMP3 operational threshold exceeds 12-bit DAC range");
_Static_assert(OC_CMP3_STARTUP_DAC < 4096, "CMP3 startup threshold exceeds 12-bit DAC range");
_Static_assert(OC_FAULT_ADC_VAL < 4096, "Software fault threshold exceeds ADC range");
/* 2026-06-13: temporarily relaxed for the HW-chop isolation test — the SW soft
 * limit is deliberately set ABOVE the CMP3 chop so the hardware limiter acts
 * alone. Restore this assert when the OC thresholds go back to normal order.
 * _Static_assert(OC_SW_LIMIT_ADC < OC_CMP3_DAC_VAL,
 *                "Software soft limit must be below CMP3 operational threshold"); */
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

/* ── FOC compile-time constants ─────────────────────────────────────── */
#if FEATURE_FOC || FEATURE_FOC_V2
/* FOC current/voltage scaling now in garuda_foc_params.h:
 * CURRENT_SCALE_A_PER_COUNT, VBUS_SCALE_V_PER_COUNT, ADC_MIDPOINT.
 * These are derived from the board shunt (3 mohm) and DIM diff-amp gain (24.95x).
 * OA3 (bus current for FEATURE_HW_OVERCURRENT) uses the same shunt/gain path.
 *
 * Legacy AN1292 defines (VBUS_ADC_TO_VOLTS, ADC_CURRENT_SCALE) removed;
 * garuda_foc_params.h provides the canonical values. */
#endif /* FEATURE_FOC || FEATURE_FOC_V2 */

/* ── Runtime parameter macros ────────────────────────────────────────── *
 * When FEATURE_GSP=1, these read from the GSP runtime param system
 * (gspParams for raw values, gspDerived for precomputed ISR values).
 * When FEATURE_GSP=0, they resolve to compile-time constants.
 * All raw param reads are 8/16-bit: atomic on dsPIC33AK (no lock needed). */
#if FEATURE_GSP
  #include "gsp/gsp_params.h"
  /* Raw param reads (ISR-safe: 8/16-bit atomic on dsPIC33AK) */
  #define RT_RAMP_TARGET_ERPM             gspParams.rampTargetErpm
  #define RT_RAMP_ACCEL_ERPM_PER_S        gspParams.rampAccelErpmPerS
  #define RT_TIMING_ADV_MAX_DEG           gspParams.timingAdvMaxDeg
  #define RT_HWZC_CROSSOVER_ERPM          gspParams.hwzcCrossoverErpm
  #define RT_MOTOR_POLE_PAIRS             gspParams.motorPolePairs
  #define RT_INITIAL_ERPM                 gspParams.initialErpm
  #define RT_MAX_CLOSED_LOOP_ERPM         gspParams.maxClosedLoopErpm
  #define RT_ZC_BLANKING_PERCENT          gspParams.zcBlankingPercent
  #define RT_ZC_ADC_DEADBAND              gspParams.zcAdcDeadband
  #define RT_ZC_FILTER_THRESHOLD          gspParams.zcFilterThreshold
  #define RT_ZC_SYNC_THRESHOLD            gspParams.zcSyncThreshold
  #define RT_ZC_DEMAG_DUTY_THRESH         gspParams.zcDemagDutyThresh
  #define RT_ZC_DEMAG_BLANK_EXTRA_PERCENT gspParams.zcDemagBlankExtraPct
  #define RT_POST_SYNC_SLEW_DIVISOR       gspParams.postSyncSlewDivisor
  #define RT_DESYNC_MAX_RESTARTS          gspParams.desyncMaxRestarts
  #define RT_MORPH_LOCK_ZC_COUNT          gspParams.morphLockZcCount
  #define RT_MORPH_LOCK_TOL_PCT           gspParams.morphLockTolPct
  #define RT_IF_CURRENT_CA                gspParams.ifCurrentCa
  #define RT_IF_RAMP_ERPM_PER_S           gspParams.ifRampErpmPerS
  #define RT_VBUS_OVERVOLTAGE_ADC         gspParams.vbusOvAdc
  #define RT_VBUS_UNDERVOLTAGE_ADC        gspParams.vbusUvAdc
  /* Precomputed derived (set from main context only) */
  #define RT_RAMP_DUTY_CAP                gspDerived.rampDutyCap
  #define RT_CL_IDLE_DUTY                 gspDerived.clIdleDuty
  #define RT_SINE_ERPM_RAMP_RATE_Q16      gspDerived.sineErpmRampRateQ16
  #define RT_MIN_STEP_PERIOD              gspDerived.minStepPeriod
  #define RT_MIN_ADC_STEP_PERIOD          gspDerived.minAdcStepPeriod
  #define RT_OC_SW_LIMIT_ADC              gspDerived.ocSwLimitAdc
  #define RT_OC_FAULT_ADC_VAL             gspDerived.ocFaultAdcVal
  #define RT_ALIGN_DUTY                   gspDerived.alignDuty
  #define RT_INITIAL_STEP_PERIOD          gspDerived.initialStepPeriod
  #define RT_INITIAL_ADC_STEP_PERIOD      gspDerived.initialAdcStepPeriod
  #define RT_MIN_CL_ADC_STEP_PERIOD       gspDerived.minClAdcStepPeriod
  #define RT_SINE_MIN_AMPLITUDE           gspDerived.sineMinAmplitude
  #define RT_SINE_MAX_AMPLITUDE           gspDerived.sineMaxAmplitude
  #define RT_DUTY_SLEW_UP_RATE            gspDerived.dutySlewUpRate
  #define RT_DUTY_SLEW_DOWN_RATE          gspDerived.dutySlewDownRate
  #define RT_POST_SYNC_SETTLE_TICKS       gspDerived.postSyncSettleTicks
  #define RT_OC_CMP3_DAC_VAL              gspDerived.ocCmp3DacVal
  #define RT_OC_CMP3_STARTUP_DAC          gspDerived.ocCmp3StartupDac
  #define RT_RAMP_CURRENT_GATE_ADC        gspDerived.rampCurrentGateAdc
  #define RT_DESYNC_COAST_COUNTS          gspDerived.desyncCoastCounts
  #define RT_HWZC_MIN_STEP_TICKS          gspDerived.hwzcMinStepTicks
  #define RT_HWZC_NOISE_FLOOR_TICKS       gspDerived.hwzcNoiseFloorTicks
  #define RT_VBUS_UV_STARTUP_ADC          gspDerived.vbusUvStartupAdc
#else
  #define RT_RAMP_TARGET_ERPM             RAMP_TARGET_ERPM
  #define RT_RAMP_ACCEL_ERPM_PER_S        RAMP_ACCEL_ERPM_PER_S
  #define RT_TIMING_ADV_MAX_DEG           TIMING_ADVANCE_MAX_DEG
  #define RT_HWZC_CROSSOVER_ERPM          HWZC_CROSSOVER_ERPM
  #define RT_MOTOR_POLE_PAIRS             MOTOR_POLE_PAIRS
  #define RT_INITIAL_ERPM                 INITIAL_ERPM
  #define RT_MAX_CLOSED_LOOP_ERPM         MAX_CLOSED_LOOP_ERPM
  #define RT_ZC_BLANKING_PERCENT          ZC_BLANKING_PERCENT
  #define RT_ZC_ADC_DEADBAND              ZC_ADC_DEADBAND
  #define RT_ZC_FILTER_THRESHOLD          ZC_FILTER_THRESHOLD
  #define RT_ZC_SYNC_THRESHOLD            ZC_SYNC_THRESHOLD
  #define RT_ZC_DEMAG_DUTY_THRESH         ZC_DEMAG_DUTY_THRESH
  #define RT_ZC_DEMAG_BLANK_EXTRA_PERCENT ZC_DEMAG_BLANK_EXTRA_PERCENT
  #define RT_POST_SYNC_SLEW_DIVISOR       POST_SYNC_SLEW_DIVISOR
  #define RT_DESYNC_MAX_RESTARTS          DESYNC_MAX_RESTARTS
  #define RT_MORPH_LOCK_ZC_COUNT          MORPH_LOCK_ZC_COUNT
  #define RT_MORPH_LOCK_TOL_PCT           MORPH_LOCK_TOL_PCT
  #define RT_IF_CURRENT_CA                600
  #define RT_IF_RAMP_ERPM_PER_S           12000
  #define RT_VBUS_OVERVOLTAGE_ADC         VBUS_OVERVOLTAGE_ADC
  #define RT_VBUS_UNDERVOLTAGE_ADC        VBUS_UNDERVOLTAGE_ADC
  #define RT_RAMP_DUTY_CAP                RAMP_DUTY_CAP
  #define RT_CL_IDLE_DUTY                 CL_IDLE_DUTY
  #define RT_SINE_ERPM_RAMP_RATE_Q16      SINE_ERPM_RAMP_RATE_Q16
  #define RT_MIN_STEP_PERIOD              MIN_STEP_PERIOD
  #define RT_MIN_ADC_STEP_PERIOD          MIN_ADC_STEP_PERIOD
  #define RT_OC_SW_LIMIT_ADC              OC_SW_LIMIT_ADC
  #define RT_OC_FAULT_ADC_VAL             OC_FAULT_ADC_VAL
  #define RT_ALIGN_DUTY                   ALIGN_DUTY
  #define RT_INITIAL_STEP_PERIOD          INITIAL_STEP_PERIOD
  #define RT_INITIAL_ADC_STEP_PERIOD      INITIAL_ADC_STEP_PERIOD
  #define RT_MIN_CL_ADC_STEP_PERIOD       MIN_CL_ADC_STEP_PERIOD
  #define RT_SINE_MIN_AMPLITUDE           SINE_MIN_AMPLITUDE
  #define RT_SINE_MAX_AMPLITUDE           SINE_MAX_AMPLITUDE
  #define RT_DUTY_SLEW_UP_RATE            DUTY_SLEW_UP_RATE
  #define RT_DUTY_SLEW_DOWN_RATE          DUTY_SLEW_DOWN_RATE
  #define RT_POST_SYNC_SETTLE_TICKS       POST_SYNC_SETTLE_TICKS
  #define RT_OC_CMP3_DAC_VAL              OC_CMP3_DAC_VAL
  #define RT_OC_CMP3_STARTUP_DAC          OC_CMP3_STARTUP_DAC
  #if RAMP_CURRENT_GATE_MA > 0
    #define RT_RAMP_CURRENT_GATE_ADC      RAMP_CURRENT_GATE_ADC
  #else
    #define RT_RAMP_CURRENT_GATE_ADC      0
  #endif
  #define RT_DESYNC_COAST_COUNTS          DESYNC_COAST_COUNTS
  #define RT_HWZC_MIN_STEP_TICKS          HWZC_MIN_STEP_TICKS
  #define RT_HWZC_NOISE_FLOOR_TICKS       HWZC_NOISE_FLOOR_TICKS
  #define RT_VBUS_UV_STARTUP_ADC          VBUS_UV_STARTUP_ADC
#endif

/* Timing-advance ramp ENDPOINT — the eRPM at which the advance schedule reaches
 * timingAdvMaxDeg. Historically this was MAX_CLOSED_LOOP_ERPM, which starves
 * advance for any motor that OPERATES well below its top-speed cap (e.g. a
 * 4000KV micro motor running at 40k on a 260k cap gets only ~3°). Decoupled so
 * advance reaches full value at the actual operating speed. Define
 * HWZC_ADV_FULL_ERPM to override; else falls back to the speed cap (no change). */
#ifdef HWZC_ADV_FULL_ERPM
#define RT_TIMING_ADV_FULL_ERPM  HWZC_ADV_FULL_ERPM
#else
#define RT_TIMING_ADV_FULL_ERPM  RT_MAX_CLOSED_LOOP_ERPM
#endif

#ifdef __cplusplus
}
#endif

#endif /* GARUDA_CALC_PARAMS_H */
