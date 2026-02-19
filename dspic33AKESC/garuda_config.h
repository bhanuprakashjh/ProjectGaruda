/**
 * @file garuda_config.h
 *
 * @brief User-configurable parameters for Project Garuda ESC firmware.
 * These are the "knobs" — change these to tune the ESC for your motor/setup.
 *
 * Component: CONFIGURATION
 */

#ifndef GARUDA_CONFIG_H
#define GARUDA_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Feature Flags (0=disabled, 1=enabled) */
#define FEATURE_BEMF_CLOSED_LOOP 1  /* Phase 2: BEMF ZC detection (0=Phase 1 open-loop only) */
#define FEATURE_VBUS_FAULT       1  /* Phase A4: Bus voltage OV/UV fault enforcement */
#define FEATURE_DESYNC_RECOVERY  1  /* Phase B2: Controlled restart-on-desync (ESC_RECOVERY) */
#define FEATURE_DUTY_SLEW        1  /* Phase B1: Asymmetric duty slew rate limiter */
#define FEATURE_TIMING_ADVANCE   1  /* Phase B3: Linear timing advance by RPM (0-15 deg linear by speed) */
#define FEATURE_DYNAMIC_BLANKING 1  /* Phase C1: Speed+duty-aware blanking (extra blank at high duty/demag) */
#define FEATURE_VBUS_SAG_LIMIT   1  /* Phase C2: Bus voltage sag power limiting (reduce duty on Vbus dip) */
#define FEATURE_BEMF_INTEGRATION 1  /* Phase E: Shadow integration estimator (shadow-only, no control) */
#define FEATURE_SINE_STARTUP     0  /* Disabled for A2212: sine->trap transition unreliable */
#define FEATURE_ADC_CMP_ZC       1  /* Phase F: ADC comparator-based high-speed ZC */
#define FEATURE_HW_OVERCURRENT  1  /* Phase G: Hardware overcurrent protection via CMP3+OA3 */
#define FEATURE_LEARN_MODULES    0  /* master: ring buffer + quality + health */
#define FEATURE_ADAPTATION       0  /* requires FEATURE_LEARN_MODULES */
#define FEATURE_COMMISSION       0  /* requires FEATURE_LEARN_MODULES */
#define FEATURE_EEPROM_V2        0  /* requires at least one above */

/* Diagnostic: Manual step mode (1=enabled)
 * SW1: Start motor → align to step 0
 * SW2: Manually advance one commutation step
 * LED2: Toggles on each step advance
 * No automatic ramp — user controls step timing.
 * Set to 0 for normal auto-ramp operation. */
#define DIAGNOSTIC_MANUAL_STEP  0

/* Learn module tuning (only compiled when FEATURE_LEARN_MODULES=1) */
#if FEATURE_LEARN_MODULES
#define TELEM_RING_SIZE             64      /* power of 2, 64*13=832B RAM */
#define TELEM_RING_MASK             (TELEM_RING_SIZE - 1)
#define QUALITY_WINDOW_MS           1000
#define QUALITY_UPDATE_DIVIDER      1       /* every 1ms */
#define HEALTH_UPDATE_DIVIDER       10      /* every 10ms */
#define ADAPT_EVAL_DIVIDER          100     /* every 100ms */
#define ADAPT_MIN_CONFIDENCE        180
#define ADAPT_MAX_CONSECUTIVE_FAIL  3
#define ADAPT_MAX_ROLLBACK_TOTAL    10
#endif

/* PWM Configuration */
#define PWMFREQUENCY_HZ            24000       /* PWM switching frequency */

/*──────────────────────────────────────────────────────────────────────────
 * Motor Profile Selection
 * 0 = Hurst DMB0224C10002 (10-pole, 24V, 4.03 ohm, 7.24 Vpk/KRPM)
 * 1 = A2212 1400KV (14-pole, 12V, 0.065 ohm, 1400 KV)
 *
 * All motor-dependent parameters are grouped here for easy swapping.
 * Board-specific and feature-tuning parameters are below.
 *──────────────────────────────────────────────────────────────────────────*/
#define MOTOR_PROFILE  1

#if MOTOR_PROFILE == 0
/* === Hurst DMB0224C10002 (dev bench motor) ===
 * 10 poles, 24VDC, 1.0A rated, L-L 4.03 ohm / 4.60 mH
 * No-load 3125 RPM @ 24V, Ke 7.24 Vpk/KRPM */
#define MOTOR_POLE_PAIRS             5
#define DEADTIME_NS                750
#define ALIGN_DUTY_PERCENT          20
#define RAMP_DUTY_PERCENT           40
#define INITIAL_ERPM               300     /* ~5 steps/sec — slow start */
#define RAMP_TARGET_ERPM          2000     /* Step period ~5ms >> L/R=1.14ms */
#define MAX_CLOSED_LOOP_ERPM     20000     /* No-load ~15625 eRPM */
#define RAMP_ACCEL_ERPM_PER_S     1000
#define SINE_ALIGN_MODULATION_PCT   15
#define SINE_RAMP_MODULATION_PCT    35
#define ZC_DEMAG_DUTY_THRESH        70     /* % duty above which extra blanking applies */
#define ZC_DEMAG_BLANK_EXTRA_PERCENT 12    /* Extra blanking % at 100% duty */
#define HWZC_CROSSOVER_ERPM       5000     /* HW ZC activates above this eRPM */
#define CL_IDLE_DUTY_PERCENT         0     /* No idle floor for Hurst */
#define SINE_PHASE_OFFSET_DEG       60     /* Sine-to-trap transition offset (Hurst: 60 works) */
#define OC_LIMIT_MA               1800     /* CMP3 CLPCI chopping (1.8A) */
#define OC_STARTUP_MA            18000     /* Mode 1 only: high CMP3 during startup */
#define OC_FAULT_MA               3000     /* Software hard fault (3.0A, mode 2) */
#define OC_SW_LIMIT_MA            1500     /* Software soft limit (1.5A) */
#define RAMP_CURRENT_GATE_MA         0     /* 0=disabled: Hurst starts easily without gating */
#define FEATURE_PRESYNC_RAMP       0       /* Hurst: standard forced OL_RAMP */

#elif MOTOR_PROFILE == 1
/* === A2212 1400KV (drone motor) ===
 * 14 poles, 12V, 0.065 ohm, ~30 uH
 * 1400 KV => ~16800 RPM @ 12V, 117600 eRPM */
#define MOTOR_POLE_PAIRS             7
#define DEADTIME_NS                500     /* Less distortion on low-L motor */
#define ALIGN_DUTY_PERCENT           8     /* 12V*8%/0.065=14.8A stall (supply CC limits) */
#define RAMP_DUTY_PERCENT           10     /* More torque to push prop through ramp */
#define INITIAL_ERPM               200     /* Very slow start: 50ms per step — prop can follow */
#define RAMP_TARGET_ERPM          2000     /* Low target for loaded start, HWZC takes over */
#define MAX_CLOSED_LOOP_ERPM    120000     /* 1400KV * 12V * 7pp */
#define RAMP_ACCEL_ERPM_PER_S      300     /* Gentle ramp — each step must physically complete */
#define SINE_ALIGN_MODULATION_PCT    4     /* Limit alignment current */
#define SINE_RAMP_MODULATION_PCT     8     /* Limit ramp current */
#define ZC_DEMAG_DUTY_THRESH        40     /* Low-L = more demag */
#define ZC_DEMAG_BLANK_EXTRA_PERCENT 18    /* Aggressive demag blanking */
#define HWZC_CROSSOVER_ERPM       1500     /* HWZC activates after zcSynced when eRPM exceeds this */
#define CL_IDLE_DUTY_PERCENT        20     /* Min CL duty at pot=0; sustains prop at idle speed */
#define SINE_PHASE_OFFSET_DEG       60     /* Sine-to-trap offset (unused when sine disabled) */
#define OC_LIMIT_MA              12000     /* CMP3 CLPCI chopping (12A) */
#define OC_STARTUP_MA            22000     /* High: let 10A supply CC be the limiter, not CMP3 */
#define OC_FAULT_MA              18000     /* Software hard fault (18A, mode 2) */
#define OC_SW_LIMIT_MA            8000     /* Software soft limit (8A, 2A below 10A supply CC) */
#define RAMP_CURRENT_GATE_MA      5000     /* Hold ramp accel when ibus > 5A (prevents overdrive stall).
                                            * At 12V/10% duty the 0.065-ohm A2212 draws ~10A stall.
                                            * 5A gate pauses acceleration when rotor lags, resumes
                                            * when motor catches up and current drops. 10V works
                                            * because lower V/L means slower current rise → less
                                            * braking torque during wrong commutation. */
#define FEATURE_PRESYNC_RAMP       0       /* Disabled: standard forced OL_RAMP (reliable no-prop) */

#else
#error "Unknown MOTOR_PROFILE — see garuda_config.h"
#endif

/* Motor Configuration (shared across profiles) */
#define DIRECTION_DEFAULT          0           /* 0=CW, 1=CCW */

/* Startup / Alignment */
#define ALIGN_TIME_MS              500         /* Time to hold alignment position */

/* Arming */
#define ARM_TIME_MS                500         /* Throttle must be zero for this long to arm */
#define ARM_THROTTLE_ZERO_ADC      200         /* Max pot ADC to consider "zero" (~4.9% of 4095) */

/* Bus Voltage */
#define VBUS_OVERVOLTAGE_ADC       3600        /* ~52V with typical divider (tunable) */
#define VBUS_UNDERVOLTAGE_ADC      500         /* ~7V with typical divider (tunable) */
#define VBUS_FAULT_FILTER          3           /* Consecutive ADC samples to confirm fault (3 = ~125us) */
#define VBUS_UV_STARTUP_ADC        400         /* ~5.6V: relaxed UV during pre-sync startup.
                                                * Below ~4.5V, bootstrap caps can't charge and
                                                * gate drive fails. 400 ADC (~5.6V) gives margin
                                                * above brownout while staying well below the
                                                * CC-sag floor (~636 ADC = 8.9V at 10A).
                                                * Normal UV threshold resumes after zcSynced. */
#define PRESYNC_TIMEOUT_MS         5000        /* Max time in pre-sync before FAULT_STARTUP_TIMEOUT.
                                                * At 200 eRPM / 7pp = ~2.5 mech revolutions.
                                                * If BEMF too weak for 3 ZC confirmations in this
                                                * time, motor/prop combination can't start. */

/* Duty Slew Rate (Phase B1) */
#if FEATURE_DUTY_SLEW
#define DUTY_SLEW_UP_PERCENT_PER_MS     2   /* Max duty increase: 2%/ms (~50ms full scale) */
#define DUTY_SLEW_DOWN_PERCENT_PER_MS   5   /* Max duty decrease: 5%/ms (~20ms full scale) */
#define POST_SYNC_SETTLE_MS            500  /* Reduced slew-up rate period after ZC sync (ms).
                                             * During this window, duty ramps at 1/DIVISOR of
                                             * normal rate to prevent over-acceleration on
                                             * low-inertia motors. Was 2000 for Hurst; A2212
                                             * decelerates too fast at 2000ms. */
#define POST_SYNC_SLEW_DIVISOR           2  /* Slew-up rate divisor during settle (2 = 1%/ms).
                                             * Was 8 for Hurst; A2212 needs faster response. */
#endif

/* Desync Recovery (Phase B2) */
#if FEATURE_DESYNC_RECOVERY
#define DESYNC_COAST_MS             200     /* Coast-down time before restart attempt */
#define DESYNC_MAX_RESTARTS         3       /* Max restart attempts before permanent fault */
#endif

/* Timing Advance (Phase B3) */
#if FEATURE_TIMING_ADVANCE
#define TIMING_ADVANCE_MIN_DEG      0       /* Degrees advance at low speed */
#define TIMING_ADVANCE_MAX_DEG      15      /* Degrees advance at max speed */
#endif

/* Dynamic Blanking (Phase C1) — motor-specific params in motor profile above */

/* Bus Voltage Sag Limiting (Phase C2) */
#if FEATURE_VBUS_SAG_LIMIT
#define VBUS_SAG_THRESHOLD_ADC   900      /* Vbus below this → reduce duty (~13V with typical divider) */
#define VBUS_SAG_RECOVERY_ADC    1000     /* Vbus above this → release limit (hysteresis band = 100) */
#define VBUS_SAG_GAIN            8        /* Duty reduction proportional to sag depth: (depth * gain) >> 4 */
#endif

/* BEMF Integration Shadow Estimator (Phase E) */
#if FEATURE_BEMF_INTEGRATION
#define INTEG_THRESHOLD_GAIN  256   /* Q8.8: 256=1.0x */
#define INTEG_HIT_DIVISOR     8     /* Tolerance = stepPeriod / 8 (~7.5 deg elec) */
#define INTEG_CLAMP           0x7FFFFF
#define SHADOW_NO_FIRE_SENTINEL  ((int16_t)0x7FFF)  /* shadowVsActual when shadow didn't fire */
#endif

/* Sine Startup (Phase D) — modulation params in motor profile above */
#if FEATURE_SINE_STARTUP
/* SINE_PHASE_OFFSET_DEG is now per-motor-profile (above) */
#define SINE_COAST_GAP_MS            2  /* De-energization gap before sine->trap transition (ms).
                                         * Motor coasts on inertia while winding currents decay.
                                         * Hurst L/R=1.14ms, A2212 L/R=0.46ms.
                                         * 2ms = 4.3 tau for A2212 (98.6% decay). */
#define SINE_TRAP_DUTY_NUM           6  /* Sine->trap duty scale factor numerator. */
#define SINE_TRAP_DUTY_DEN           5  /* Sine->trap duty scale factor denominator.
                                         * 6/5 = 1.2x compensates for the sine-to-trap L-L
                                         * voltage conversion at RAMP_TARGET_ERPM.
                                         * Tune: increase NUM if motor brakes at transition,
                                         *        decrease NUM if motor surges forward. */
#endif

/* ADC Comparator ZC (Phase F) — crossover eRPM in motor profile above */
#if FEATURE_ADC_CMP_ZC
#define HWZC_BLANKING_PERCENT    5   /* Blanking as % of step period (after commutation) */
#define HWZC_HYSTERESIS_ERPM   500   /* Hysteresis band for crossover (prevents oscillation) */
#define HWZC_SYNC_THRESHOLD      6   /* Consecutive HW ZCs to declare sync */
#define HWZC_MISS_LIMIT          3   /* Missed HW ZCs before fallback to software ZC (low for debug) */
#define HWZC_CMP_DEADBAND        4   /* ADC counts deadband for comparator sanity check */
#define HWZC_SAMC               3    /* Sample time for high-speed channels (~205ns conversion) */
#define HWZC_ADC_SAMPLE_HZ   1000000  /* High-speed ADC trigger rate (SCCP3). Max ~4.9 MHz. */
#endif

/* Hardware Overcurrent Protection (Phase G) — current thresholds in motor profile above */
#if FEATURE_HW_OVERCURRENT

/* Current limit mode:
 * 0 = CLPCI cycle-by-cycle chopping only (CMP3, motor keeps running)
 * 1 = FPCI hard fault only via CMP3 (motor stops, needs SW1 restart)
 * 2 = CLPCI chopping (CMP3, OC_LIMIT_MA) + software hard fault (ADC,
 *     OC_FAULT_MA) + board FPCI backup (PCI8R, fixed HW threshold) */
#define OC_PROTECT_MODE          2

/* Board-specific amplifier parameters (MCLV-48V-300W).
 * Integer representation — no float in compile-time constants. */
#define OC_SHUNT_MOHM            3       /* 0.003 ohm = 3 milliohms */
#define OC_GAIN_X100          2495       /* 24.95 x 100 */
#define OC_VREF_MV            1650       /* 1.65V bias in millivolts */
#define OC_VADC_MV            3300       /* 3.3V ADC reference in millivolts */

/* CMP3 settings */
#define OC_CMP_HYSTERESIS       0b11  /* 45mV hysteresis (max, for noise immunity) */
#define OC_CMP_FILTER_EN        1     /* 1=enable CMP3 digital filter, 0=disable */

#endif /* FEATURE_HW_OVERCURRENT */

/* Comparator DAC reference for overcurrent fault (from reference) */
#define CMP_REF_DCBUS_FAULT        2048        /* Default DAC reference, midscale */

/* Phase 2: BEMF Closed-Loop ZC Detection Parameters (ADC threshold method) */
#if FEATURE_BEMF_CLOSED_LOOP
/* Core ZC detection */
#define ZC_BLANKING_PERCENT     3       /* Ignore ZC for first 3% of step period after commutation */
#define ZC_FILTER_THRESHOLD     2       /* Reduced from 3: deadband is primary noise gate */
#define ZC_SYNC_THRESHOLD       6       /* Confirmed ZCs to declare lock (6 = two e-cycles of rising-only) */
#define ZC_MISS_LIMIT           12      /* Missed steps before FAULT_DESYNC (two e-cycles) */
#define ZC_STALENESS_LIMIT      12      /* Max forced steps without a ZC before resetting goodZcCount */
#define ZC_STEP_MISS_LIMIT      2       /* Per-step misses before fallback deadline (timing-based) */
#define ZC_TIMEOUT_MULT         2       /* Timeout = ZC_TIMEOUT_MULT * stepPeriod (in adcIsrTick) */

/* ADC threshold parameters.
 * Threshold tracks the virtual neutral: Vbus * duty / (2 * LOOPTIME_TCY).
 * Approximated as (vbusRaw * duty) >> ZC_DUTY_THRESHOLD_SHIFT.
 * Shift 18 is accurate for 24kHz PWM (2*LOOPTIME_TCY ≈ 266K ≈ 2^18). */
#define ZC_DUTY_THRESHOLD_SHIFT 18
#define ZC_ADC_DEADBAND         4       /* Reduced from 10: tighter deadband for faster ZC crossing */
#define ZC_AD2_SETTLE_SAMPLES   2       /* Increased from 1: extra settle for AD2 mux (steps 1,3,4) */

/* Per-phase signed offset correction (ADC counts). Default 0. */
#define ZC_PHASE_OFFSET_A       0
#define ZC_PHASE_OFFSET_B       0
#define ZC_PHASE_OFFSET_C       0

/* Per-phase Q15 gain correction. 32768 = 1.0 (unity). */
#define ZC_PHASE_GAIN_A         32768
#define ZC_PHASE_GAIN_B         32768
#define ZC_PHASE_GAIN_C         32768

/* Phase 2B: Adaptive refinements (disabled for initial bring-up) */
#define ZC_ADAPTIVE_FILTER      1       /* Speed-dependent filter: drops to ZC_FILTER_MIN at high eRPM */
#define ZC_ADAPTIVE_PERIOD      1       /* IIR smoothing on stepPeriod: 3/4 old + 1/4 new */

#if ZC_ADAPTIVE_FILTER
#define ZC_FILTER_MIN           1
#define ZC_FILTER_MAX           3
#define ZC_FILTER_SPEED_THRESH  16
#endif
#endif /* FEATURE_BEMF_CLOSED_LOOP */

#ifdef __cplusplus
}
#endif

#endif /* GARUDA_CONFIG_H */
