/**
 * @file garuda_config.h
 * @brief Configuration for 6-step BLDC on dsPIC33CK + ATA6847.
 *
 * Target: EV43F54A board (dsPIC33CK64MP205 + ATA6847)
 *
 * Motor profiles:
 *   0 = Hurst DMB2424B10002 (5PP, 24V, 149KV, low-speed bench motor)
 *   1 = A2212 1400KV         (7PP, 12V, 1400KV, drone motor)
 */

#ifndef GARUDA_CONFIG_H
#define GARUDA_CONFIG_H

/* ── Motor Profile Selection ──────────────────────────────────────── */
#ifndef MOTOR_PROFILE
#define MOTOR_PROFILE   2   /* 0=Hurst, 1=A2212, 2=2810 */
#endif

/* ── Clock ─────────────────────────────────────────────────────────── */
#define FOSC                200000000UL  /* 200 MHz */
#define FCY                 100000000UL  /* 100 MHz instruction clock */
#define FOSC_PWM_MHZ        400U         /* PWM timing base (CK: 2x Fosc) */

/* ── PWM ───────────────────────────────────────────────────────────── */
#define PWMFREQUENCY_HZ     40000U       /* 40 kHz — zero timeouts at 100k with fixes.
                                          * 15kHz gave 19x fewer ZC timeouts
                                          * but was audible. 24kHz should give
                                          * cleaner ZC than 20kHz while staying
                                          * above hearing. Aliasing at 72k eRPM
                                          * (was 60k at 20kHz). */
#define LOOPTIME_MICROSEC   (uint16_t)(1000000UL / PWMFREQUENCY_HZ)  /* 50 us */
#define LOOPTIME_TCY        (uint16_t)((uint32_t)LOOPTIME_MICROSEC * FOSC_PWM_MHZ / 2 - 1)
/* PWM drive mode: complementary vs unipolar.
 * 0 = Complementary (H+L alternate): active braking, more switching noise
 * 1 = Unipolar (H-PWM, L-OFF): no braking, half switching noise.
 *     34x fewer ZC timeouts, 99.96% success rate.
 *     Needs lower MAX_DUTY (25%) since no braking. */
#ifndef PWM_DRIVE_UNIPOLAR
#define PWM_DRIVE_UNIPOLAR  0   /* Complementary + CLC AND filter.
                                 * CLC gates BEMF with PWM-ON window →
                                 * switching noise hardware-blocked.
                                 * Proper braking + clean ZC. */
#endif

#if PWM_DRIVE_UNIPOLAR
#define MAX_DUTY            (LOOPTIME_TCY - 200U)  /* Full range. Previous 97% duty test
                                                    * had zero ZC timeouts at 100k eRPM.
                                                    * Governor (DUTY_RAMP_ERPM) limits
                                                    * acceleration. No-load self-limits
                                                    * via back-EMF. Prop needs full range. */
#else
#define MAX_DUTY            (LOOPTIME_TCY - 200U)  /* ~100% for complementary */
#endif
#if PWM_DRIVE_UNIPOLAR
#define MIN_DUTY            30U          /* Unipolar: minimum for 1% idle */
#else
#define MIN_DUTY            200U         /* Complementary: normal min */
#endif
/* Dead time: ATA6847 handles 700ns CCPT internally, so dsPIC dead time
 * is minimal. Reference firmware uses 0x14 = 20 counts = 50ns. */
#define DEADTIME_TCY        0x14U

/* ── Current Sensing (EV43F54A inverter board) ─────────────────────── */
/* Shunt: RS1/RS2/RS3 = 3mΩ (0.003R) on each phase + DC bus return.
 * Phase A (IS1): dsPIC OA2, Gt = 1 + R46/(R57+R58) = 1+12k/800 = 16
 * Phase B (IS2): dsPIC OA3, same topology, Gt = 16
 * DC Bus (IBus): ATA6847 OPO3 Gt=8 → dsPIC OA1
 * ADC: 12-bit signed fractional, 3.3V ref → ±1.65V range after offset.
 * ADC is 12-bit signed fractional (left-justified in 16-bit: /65536 not /4096)
 * Phase current: V = raw × 3.3 / 65536, I = V / (0.003 × 16)
 *   → I_mA = raw × 3300 / 65536 / 0.048 = raw × 1.049
 * Bus current:   V = raw × 3.3 / 65536, I = V / (0.003 × 8)
 *   → I_mA = raw × 3300 / 65536 / 0.024 = raw × 2.098 */
#define CURRENT_SHUNT_MOHM      3U      /* 3mΩ shunt resistors */
#define CURRENT_PHASE_GAIN      16U     /* OA2/OA3 gain for phase currents */
#define CURRENT_IBUS_GAIN       16U     /* ATA6847 CSA3 gain (CSCR.GAIN=0b01=Gain_16)
                                         * Note: dsPIC OA1 is NOT used (OA1IN- shares
                                         * pin with AN9/Vbus). AN0 reads CSA3 output
                                         * directly through RC filter. */

/* ── Timer1 (50 µs tick) ───────────────────────────────────────────── */
#define TIMER1_PRESCALE     8U
#define TIMER1_FREQ_HZ      20000U       /* 50 µs period — matches ADC ISR rate */
#define TIMER1_PR           (uint16_t)(FCY / TIMER1_PRESCALE / TIMER1_FREQ_HZ - 1)

/* ================================================================
 *          MOTOR-SPECIFIC PARAMETERS  (per-profile)
 * ================================================================ */

#if MOTOR_PROFILE == 0
/* ── Motor: Hurst Long (DMB2424B10002) ─────────────────────────────── */
/* 10 poles (5PP), 24VDC, 3.4A RMS rated, 2500 RPM nom, 3500 RPM max
 * Low-KV bench motor: high Rs, high Ls, strong BEMF at low speed */
#define MOTOR_POLE_PAIRS    5U
#define MOTOR_RS_MILLIOHM   534U         /* Phase resistance (measured) */
#define MOTOR_LS_MICROH     471U         /* Phase inductance (measured) */
#define MOTOR_KV            149U         /* RPM/V */

/* Startup */
#define ALIGN_TIME_MS       200U
#define ALIGN_DUTY          (LOOPTIME_TCY / 20)    /* ~5% duty (~2.2A on Hurst) */
#define INITIAL_STEP_PERIOD 1000U        /* Timer1 ticks (50 ms per step = ~200 eRPM) */
#define MIN_STEP_PERIOD     66U          /* Timer1 ticks (3.3 ms per step = ~3000 eRPM) */
#define RAMP_ACCEL_ERPM_S   1500U        /* eRPM/s acceleration */
#define RAMP_DUTY_CAP       (LOOPTIME_TCY / 6)     /* Max ~17% duty during OL ramp */

/* ZC Detection */
#define ZC_BLANKING_PERCENT 20U          /* % of step period to blank after commutation */
#define ZC_FILTER_THRESHOLD 3U           /* Net matching reads to confirm ZC (bounce-tolerant) */
/* Adaptive blanking knobs (used when FEATURE_IC_ZC_ADAPTIVE=1) */
#define ZC_BLANK_PCT_SLOW   12U         /* Blanking % at low speed (sp >= 200 T1 ticks) */
#define ZC_BLANK_PCT_FAST   10U         /* Blanking % at high speed (sp < 20 T1 ticks) */
#define ZC_BLANK_FLOOR_US   25U         /* Absolute minimum blanking (µs) */
#define ZC_BLANK_CAP_PCT    45U         /* Max blanking as % of step period */

/* Timing advance: uses shared TIMING_ADVANCE_LEVEL default (15°) */

/* Closed-Loop */
#define MAX_CLOSED_LOOP_ERPM 15000U      /* ~3000 mech RPM */
#define RAMP_TARGET_ERPM     3000U       /* OL→CL handoff speed */
/* CL_IDLE_DUTY_PERCENT: shared default below */

/* ATA6847 Hardware Current Limit — cycle-by-cycle chopping.
 * DAC formula: VILIM_TH = 3.3V × DAC / 128
 * Trip current: I = (VILIM_TH - 1.65V) / (Gain × Rshunt)
 *             = (3.3 × DAC / 128 - 1.65) / (16 × 0.003)
 * Hurst rated 3.4A, max 5A. DAC=80 → 8.6A trip (2.5× rated). */
#define ILIM_DAC            80U     /* 8.6A trip for Hurst */

/* Vbus Fault Thresholds (24V system) */
#define VBUS_OV_THRESHOLD   (3200U * 16U)  /* ~30V → 51200 in 16-bit */
#define VBUS_UV_THRESHOLD   (700U * 16U)   /* ~7V  → 11200 in 16-bit */

#elif MOTOR_PROFILE == 1
/* ── Motor: A2212 1400KV (drone motor) ─────────────────────────────── */
/* 12N14P, 14 poles (7PP), 12V (3S LiPo), no-load ~16800 RPM
 * High-KV drone motor: very low Rs (65mΩ), very low Ls (30µH),
 * weak BEMF at low speed, fast electrical frequency.
 *
 * 6-step challenges vs Hurst:
 *   - BEMF is ~10x weaker per RPM: ZC detection harder at low speed
 *   - 7PP vs 5PP: 40% more commutations per mech revolution
 *   - Low Rs: alignment current is high for given duty (I≈V/Rs)
 *   - Low Ls: fast current transients, less ZC blanking needed
 *   - Max eRPM = 16800 * 7 * 2 / 60 ≈ 117,600 — well beyond timer range
 *     but Vbus-limited to ~70,000 eRPM at 12V with losses */
#define MOTOR_POLE_PAIRS    7U
#define MOTOR_RS_MILLIOHM   65U          /* Phase resistance */
#define MOTOR_LS_MICROH     30U          /* Phase inductance */
#define MOTOR_KV            1400U        /* RPM/V */

/* Startup — A2212 needs less align duty (low Rs → high current per volt)
 * but SLOWER ramp than Hurst to ensure motor tracks forced commutation.
 * At RAMP_DUTY_CAP (17%), V = 0.17*12V = 2.04V → I = 2.04/0.065 = 31A peak.
 * This is safe for A2212 (rated 15-20A burst) during brief OL ramp.
 * Align duty: 2% → V = 0.02*12V = 0.24V → I = 0.24/0.065 = 3.7A (OK).
 *
 * CRITICAL: ramp must not outrun the motor. At 3000 eRPM/s the forced
 * rampStepPeriod reaches MIN_STEP before motor physically accelerates,
 * causing CL entry at low actual speed with weak BEMF → IC ZC fails.
 * Use 1500 eRPM/s (proven on Hurst) with higher duty cap for torque. */
#define ALIGN_TIME_MS       150U         /* Lighter rotor, less settling */
#define ALIGN_DUTY          (LOOPTIME_TCY / 40)    /* ~2.5% duty (~4.6A at 12V, must be > MIN_DUTY=200) */
#define INITIAL_STEP_PERIOD 800U         /* Timer1 ticks (40 ms = ~250 eRPM) */
#define MIN_STEP_PERIOD     50U          /* Timer1 ticks (2.5 ms = ~4000 eRPM) */
#define RAMP_ACCEL_ERPM_S   1500U        /* eRPM/s — same as Hurst (proven) */
#define RAMP_DUTY_CAP       (LOOPTIME_TCY / 6)     /* Max ~17% duty during OL ramp */

/* ZC Detection — A2212 BEMF comparator output is cleaner (lower Ls = less
 * ringing), but weaker signal at low speed. Same blanking %. */
#define ZC_BLANKING_PERCENT 20U          /* % of step period to blank after commutation */
#define ZC_FILTER_THRESHOLD 3U           /* Net matching reads to confirm ZC */
/* Adaptive blanking knobs (used when FEATURE_IC_ZC_ADAPTIVE=1) */
#define ZC_BLANK_PCT_SLOW   12U         /* Blanking % at low speed */
#define ZC_BLANK_PCT_FAST   10U         /* Blanking % at high speed */
#define ZC_BLANK_FLOOR_US   25U         /* Abs minimum blanking (µs). A2212 L/R=462µs */
#define ZC_BLANK_CAP_PCT    45U         /* Max blanking as % of step period */

/* Closed-Loop */
#define MAX_CLOSED_LOOP_ERPM 100000U
#define MIN_CL_STEP_PERIOD   2U          /* Tp floor — was 5, clamped entire pot range.
                                          * Tp:2 = 100k eRPM. Below Tp:2, Timer1 quantization
                                          * is too coarse; HR timing handles sub-tick precision. */
#define RAMP_TARGET_ERPM     4000U       /* OL→CL handoff speed (~571 mech RPM) */
/* ATA6847 Hardware Current Limit — cycle-by-cycle chopping.
 * TEST VALUE: DAC=75 → 5.3A peak trip. At ~50% duty, DC bus limit
 * ≈ 2.6A — easily triggered on bench with prop.
 * PRODUCTION: raise to DAC=95 (16.6A) for flight. */
#define ILIM_DAC            85U     /* TEST: calibrating actual trip point */

/* CL_IDLE_DUTY_PERCENT: shared default below */

/* Vbus Fault Thresholds (12V / 3S LiPo system)
 * EV43F54A Vbus divider: ~1211 raw/V (from 24V→29072 calibration).
 * Bench supply sags significantly under motor load (12V → 9V at full duty).
 * OV: 18V (headroom for regen spikes + 4S compatibility)
 * UV: 6V (only trips on actual supply disconnect, not load sag) */
#define VBUS_OV_THRESHOLD   (1817U * 16U)  /* ~24V → 29072 in 16-bit.
                                             * Raised from 18V — regen spikes
                                             * on fast decel reach 14-16V at 12V
                                             * supply, 18-20V at 14V supply. */
#define VBUS_UV_THRESHOLD   (454U * 16U)   /* ~6V  → 7264 in 16-bit */

#elif MOTOR_PROFILE == 2
/* ── Motor: 2810 1350KV (7-8" FPV/cine drone motor) ─────────────── */
/* 12N14P, 14 poles (7PP), 5-6S LiPo (18.5-25.2V)
 * Stator: 28mm dia × 10mm height. Similar to A2212 in KV/PP.
 * Rs: 43-61mΩ (varies by manufacturer: GEPRC=43, T-Motor=61)
 * Max power: ~1240W, peak current: ~61A
 * Recommended prop: 7-8 inch
 *
 * Refs: GEPRC EM2810, T-Motor F100, BrotherHobby Avenger 2810 */
#define MOTOR_POLE_PAIRS    7U
#define MOTOR_RS_MILLIOHM   50U          /* Phase resistance (avg of 43-61mΩ) */
#define MOTOR_LS_MICROH     25U          /* Phase inductance (estimated) */
#define MOTOR_KV            1350U        /* RPM/V */

/* Startup — gentle ramp proven to work on this motor.
 * Low Rs (50mΩ) means high current at moderate duty.
 * At 13V, 5% duty → I = 0.05 × 13 / 0.050 = 13A peak (brief ramp). */
#define ALIGN_TIME_MS       200U
#define ALIGN_DUTY          (LOOPTIME_TCY / 20)    /* ~5% for 40kHz (CCPT overhead) */
#define INITIAL_STEP_PERIOD 1000U        /* Timer1 ticks (50ms = ~200 eRPM) */
#define MIN_STEP_PERIOD     50U          /* Timer1 ticks (2.5ms = ~4000 eRPM) */
#define RAMP_ACCEL_ERPM_S   500U         /* eRPM/s — moderate, proven on this motor */
#define RAMP_DUTY_CAP       (LOOPTIME_TCY / 8)     /* ~12.5% for 40kHz */

/* ZC Detection */
#define ZC_BLANKING_PERCENT 20U          /* For OL_RAMP ADC backup poll only.
                                          * CL uses 12% + 50% interval rejection. */
#define ZC_FILTER_THRESHOLD 3U
/* Adaptive blanking knobs (FEATURE_IC_ZC_ADAPTIVE=1 for 2810) */
#define ZC_BLANK_PCT_SLOW   12U         /* Blanking % at low speed (same as fixed 12%) */
#define ZC_BLANK_PCT_FAST    8U         /* Blanking % at high speed. Bench data: 12%+25µs
                                         * floor = 25% at 100k eRPM → ZcLat=0-2% (over-
                                         * blanked). 8% + 15µs floor → ~10% at 100k. */
#define ZC_BLANK_FLOOR_US   15U         /* Reduced from 25µs. At 100k eRPM (Tp=100µs),
                                         * 25µs = 25% of step — too much. 15µs = 15%. */
#define ZC_BLANK_CAP_PCT    45U         /* Max blanking as % of step period */
#define FEATURE_IC_ZC_ADAPTIVE 1        /* Enable adaptive blanking for 2810 */

/* Timing advance: level 3 (22.5°) for 2810 at 24V.
 * Level 3 vs 2 bench test: speed wall at ~100k is detector-latency-
 * limited (target-past >75% above 90k), not advance-limited.
 * Level 3 kept for now — does not hurt sub-90k operation and
 * provides slightly more margin in the 60-90k band.
 * Next step: scan-window state machine for earlier accepted timestamps. */
#define TIMING_ADVANCE_LEVEL 3U

/* Closed-Loop
 * At 25.2V (6S full): 1350 × 25.2 = 34020 RPM = 238k eRPM theoretical.
 * Practical max ~130k eRPM with losses. */
#define MAX_CLOSED_LOOP_ERPM 150000U
#define MIN_CL_STEP_PERIOD   2U          /* HR timer provides sub-tick precision.
                                          * Per-rev desync check uses HR (0.4% error)
                                          * instead of Timer1 (33% error at Tp=2). */
#define RAMP_TARGET_ERPM     3000U       /* Lower than A2212 — gentler handoff */

/* Speed governance: max duty ramps linearly from CL_IDLE_DUTY at
 * 0 eRPM to MAX_DUTY at DUTY_RAMP_ERPM. Motor can only get more
 * duty as it proves it can commutate at the current speed.
 * Prevents desync from fast throttle increase under load. */
/* Speed PD: uses shared default (disabled) */

#define DUTY_RAMP_ERPM       60000U      /* full duty available at 60k eRPM.
                                         * With props at 24V, motor reaches
                                         * ~50-65k eRPM at full throttle.
                                         * 60k allows full range with margin. */

/* ATA6847 Hardware Current Limit */
#define ILIM_DAC            120U         /* ~30.6A peak. With props, commutation
                                         * transients reach 20-25A at mid speed.
                                         * 110 (24.7A) chops too aggressively under
                                         * load, limiting max speed to ~40k eRPM.
                                         * 120 allows prop operation while still
                                         * protecting during genuine desync (34A+). */

/* CL_IDLE_DUTY_PERCENT: shared default below */

/* Vbus Fault Thresholds
 * OV: 28V (margin above fully charged 6S)
 * UV: 10V (bench supply sags under desync current spikes;
 *     production should use 16V for LiPo protection) */
#define VBUS_OV_THRESHOLD   (48440U)       /* ~40V. Regen spikes at 24V supply
                                         * reach 32V+ during fast decel or rough
                                         * commutation. 40V gives headroom.
                                         * Board FETs rated 100V — 40V is safe. */
#define VBUS_UV_THRESHOLD   (12110U)       /* ~10V → 1211*10 */

#else
#error "Unknown MOTOR_PROFILE — select 0 (Hurst), 1 (A2212), or 2 (2810)"
#endif

/* ================================================================
 *          SHARED PARAMETERS  (all motor profiles)
 * ================================================================ */

#define ALIGN_TIME_COUNTS   ((uint16_t)((uint32_t)ALIGN_TIME_MS * TIMER1_FREQ_HZ / 1000))
#ifndef CL_IDLE_DUTY_PERCENT
#if PWM_DRIVE_UNIPOLAR
#define CL_IDLE_DUTY_PERCENT 6U    /* Unipolar: 3% had constant ZC timeouts
                                    * at idle. 6% sustains prop + clean ZC. */
#else
#define CL_IDLE_DUTY_PERCENT 10U   /* Complementary: braking needs higher idle */
#endif
#endif
#define CL_IDLE_DUTY         (((uint32_t)CL_IDLE_DUTY_PERCENT * LOOPTIME_TCY) / 100)

/* ── BEMF ZC Detection (shared) ───────────────────────────────────── */
#define ZC_SYNC_THRESHOLD   6U           /* Good ZC count to enter post-sync */
#define ZC_TIMEOUT_MULT     2U           /* Forced step at 2x step period.
                                          * Was 3x — at Tp:8 that's 1.2ms (180° elec),
                                          * forced step fires at completely wrong position.
                                          * 2x = 800µs (120° elec) — less position error,
                                          * faster desync detection. */
#define ZC_DESYNC_THRESH    3U           /* Consecutive misses before clearing zcSynced */
#define ZC_MISS_LIMIT       12U          /* Consecutive misses → desync fault */

/* Per-revolution desync detection (ESCape32/AM32-inspired).
 * Every 6 commutation steps, compare stepPeriod against checkpoint.
 * If changed by >50%, it's a false-ZC cascade — declare desync. */
#define ZC_REV_DESYNC_RATIO_PCT 40U      /* If stepPeriod changes by >40% in one revolution → desync.
                                          * ESCape32 uses 50%, but at high speed with Timer1
                                          * quantization, tighter is safer. 62k→112k = 45% change. */

/* Absolute minimum ZC interval in Timer1 ticks.
 * Any ZC faster than this is physically impossible — reject regardless of IIR.
 * At 100k eRPM (max practical for 7PP at 25V): Tp = 20000*10/100000 = 2 ticks.
 * Floor of 1 tick allows full speed; the per-rev check catches false cascades. */
#define ZC_ABSOLUTE_MIN_INTERVAL  1U

/* ── ZC V2 Mode Thresholds ────────────────────────────────────────── */
#define ZC_ACQUIRE_GOOD_ZC       20U   /* consecutive good ZCs to exit ACQUIRE → TRACK */
#define ZC_RECOVER_GOOD_ZC       10U   /* consecutive good ZCs to exit RECOVER → ACQUIRE */
#define ZC_RECOVER_MAX_ATTEMPTS   5U   /* RECOVER entries before top-level desync fault */

/* ── Adaptive Blanking (derived constants + defaults) ──────────────── */
/* HR floor: ZC_BLANK_FLOOR_US converted to SCCP4 ticks (640ns each).
 * 1 µs = 25/16 HR ticks (1000ns / 640ns = 1.5625). */
#define ZC_BLANK_FLOOR_HR   ((uint16_t)((uint32_t)ZC_BLANK_FLOOR_US * 25U / 16U))

/* Defaults if a profile doesn't define these */
#ifndef ZC_BLANK_PCT_SLOW
#define ZC_BLANK_PCT_SLOW   12U
#endif
#ifndef ZC_BLANK_PCT_FAST
#define ZC_BLANK_PCT_FAST   10U
#endif
#ifndef ZC_BLANK_CAP_PCT
#define ZC_BLANK_CAP_PCT    45U
#endif

/* Feature gate for adaptive blanking + bypass guard.
 * 0 = existing 12% blanking (Hurst/A2212 proven behavior).
 * 1 = speed-adaptive blanking (for 2810 — reduces blanking at high speed). */
#ifndef FEATURE_IC_ZC_ADAPTIVE
#define FEATURE_IC_ZC_ADAPTIVE  0
#endif

/* Speed-adaptive detector front end threshold.
 * Below this StpHR: switch from CLC D-FF to raw GPIO (inverted) for
 * BEMF reads. CLC updates once per PWM (42µs at 24kHz) — at high speed,
 * the CLC may show stale pre-ZC state if its update falls in blanking.
 * Raw GPIO updates every poll (4.75µs at 210.5kHz), 9x faster.
 * BEMF is strong at high speed → raw GPIO noise is manageable.
 * 250 HR = 160µs = ~62k eRPM. Below 62k: CLC. Above 62k: raw GPIO. */
#ifndef ZC_IC_DIRECT_THRESHOLD_HR
#define ZC_IC_DIRECT_THRESHOLD_HR  250U
#endif

/* Bypass guard bounds (used when FEATURE_IC_ZC_ADAPTIVE=1) */
#define ZC_BYPASS_LO_PCT    40U     /* Min % of stepPeriodHR for bypass acceptance */
#define ZC_BYPASS_HI_PCT   160U     /* Max % of stepPeriodHR for bypass acceptance */

/* ── Universal defaults (apply to ALL motors) ─────────────────────── */

/* AM32-style timing advance: fixed fraction of commutation interval.
 * advance = (interval/8) * level → 7.5° per level.
 * Level 2 = 15° works for all motors tested (Hurst, A2212, 2810).
 * Per-profile override possible but not needed. */
#ifndef TIMING_ADVANCE_LEVEL
#define TIMING_ADVANCE_LEVEL  2U
#endif

/* Speed PD disabled globally — direct pot→duty + duty governor.
 * Current eRPM-based 1ms PD causes duty oscillation at high speed.
 * Needs AM32-style per-ZC interval-based PID before re-enabling. */
#ifndef FEATURE_SPEED_PD
#define FEATURE_SPEED_PD     0
#endif

/* Duty governor: limits max duty based on measured eRPM.
 * Prevents fast-pot desync. Full duty at threshold. */
#ifndef DUTY_RAMP_ERPM
#if PWM_DRIVE_UNIPOLAR
#define DUTY_RAMP_ERPM  60000U  /* Unipolar: full duty at 60k. Governor prevents
                                 * fast-pot desync. With 50% MAX_DUTY, governor
                                 * effectively allows 50% at 60k eRPM. */
#else
#define DUTY_RAMP_ERPM  60000U  /* Complementary: braking assists at low speed */
#endif
#endif
#ifndef MIN_TARGET_ERPM
#define MIN_TARGET_ERPM   1000U
#endif
#ifndef MAX_TARGET_ERPM
#define MAX_TARGET_ERPM 100000U
#endif

/* ── Feature Flags ─────────────────────────────────────────────────── */
#ifndef FEATURE_IC_ZC
#define FEATURE_IC_ZC           1   /* 1 = fast poll timer ZC, 0 = ADC ISR polling only */
#endif

#ifndef FEATURE_CLC_BLANKING
#define FEATURE_CLC_BLANKING    1   /* CLC AND filter: BEMF gated by PWM-ON */   /* CLC experiment: reduced step-2 aliasing but
                                     * degraded overall detection at 20kHz PWM.
                                     * Needs 40kHz+ PWM or new detector architecture.
                                     * Disabled — raw GPIO poller is better at 20kHz. */
#endif

#ifndef FEATURE_IC_ZC_CAPTURE
#define FEATURE_IC_ZC_CAPTURE   1   /* Hybrid IC+poll: IC timestamp, poll validates */
#endif

#ifndef FEATURE_IC_DMA_SHADOW
#define FEATURE_IC_DMA_SHADOW   1   /* Dual-CCP + DMA shadow ring experiment.
                                     * When 1: CCP2 (rising ZCs) and CCP5 (falling ZCs)
                                     *  run as free-running input captures with DMA0/DMA1
                                     *  writing hardware-precise timestamps to circular
                                     *  RAM rings. A per-step probe compares DMA captures
                                     *  against the poll-accepted ZC and logs telemetry.
                                     *  Shadow-only: does NOT affect live scheduling.
                                     * When 0: no DMA, no CCP5, production path unchanged.
                                     * MUTUALLY EXCLUSIVE with FEATURE_IC_ZC_CAPTURE — under
                                     * the shadow flag the live CCP2 path is stubbed so
                                     * DMA owns CCP2BUFL drains exclusively. */
#endif

#if FEATURE_IC_DMA_SHADOW && FEATURE_IC_ZC_CAPTURE
#undef  FEATURE_IC_ZC_CAPTURE
#define FEATURE_IC_ZC_CAPTURE   0   /* Shadow takes exclusive ownership of CCP2 */
#endif

/* ── Sector PI synchronizer (FEATURE_SECTOR_PI) ────────────────────
 * New high-speed control path modeled on Microchip AVR high-speed
 * example. Uses hardware-captured ZC (DMA ring) + PI on phase error
 * + explicit fixed detector delay + explicit torque advance.
 *
 * Does NOT modify the reactive poll-timed path. Runs in shadow
 * first (mode=1), then takes ownership (mode=2) at high speed.
 * Reactive path stays as fallback.
 *
 * Requires FEATURE_IC_DMA_SHADOW. */
#ifndef FEATURE_SECTOR_PI
#define FEATURE_SECTOR_PI        1
#endif

/* Detector delay: sum of three fixed hardware pipeline components.
 * Precomputed to integer HR ticks (640 ns each). No float in hot path.
 *
 * ATA6847 comparator propagation:  ~2 µs → 3 HR ticks
 * RC filter on phase inputs:       ~0 µs → 0 HR ticks (CK board has CLC, no RC)
 * DMA cluster selector bias:       ~1 µs → 2 HR ticks (midpoint vs true edge) */
#ifndef ZC_EXTCMP_DELAY_HR
/* Detector delay components in HR ticks (640 ns each).
 *
 * Measured from CSV pot_capture_20260414_175759.csv:
 * T_hat gap is consistently -24 to -27 HR in the 26-60k band
 * where acceptance is 99% and the motor runs cleanly.
 * This gap = true detector delay not accounted for in the model.
 *
 * ATA6847 comparator propagation:       ~2 µs →  3 HR
 * Poll filter delay (multi-read):       ~10 µs → 16 HR
 * DMA cluster bias (midpoint vs edge):  ~1 µs →  2 HR
 * Poll-to-DMA offset residual:          ~2 µs →  4 HR
 * Total:                                ~15 µs   25 HR */
#define ZC_EXTCMP_DELAY_HR       3U
#endif
#ifndef ZC_RC_DELAY_HR
#define ZC_RC_DELAY_HR           0U
#endif
#ifndef ZC_CLUSTER_BIAS_HR
#define ZC_CLUSTER_BIAS_HR       2U
#endif
#ifndef ZC_POLL_FILTER_DELAY_HR
#define ZC_POLL_FILTER_DELAY_HR  20U   /* DMA captures ZC ~25 HR BEFORE poll.
                                        * This is SUBTRACTED from setValueHR
                                        * (not added) because the DMA measurement
                                        * is EARLY, unlike Microchip's RC filter
                                        * which makes measurements LATE. */
#endif
#define ZC_SYNC_DET_DELAY_HR     (ZC_EXTCMP_DELAY_HR + ZC_RC_DELAY_HR + ZC_CLUSTER_BIAS_HR + ZC_POLL_FILTER_DELAY_HR)

/* Torque advance: pure motor-dependent angle (electrical degrees).
 * 10° is the Microchip default for similar KV motors (A2207-2500KV).
 * NOT mixed with detector latency. Converted to HR ticks at runtime
 * using integer multiply: advHR = (deg * 256 / 60) * T_hatHR >> 8.
 * The constant below is the fixed-point multiplier (deg * 256 / 60). */
#ifndef ZC_SYNC_ADVANCE_DEG
#define ZC_SYNC_ADVANCE_DEG      10.0f
#endif
#define ZC_SYNC_ADVANCE_FP8      ((uint16_t)(ZC_SYNC_ADVANCE_DEG * 256.0f / 60.0f + 0.5f))

/* PI gains (bit shifts for division).
 * Kp = 1/4: fast transient response to speed changes.
 * Ki = 1/16: smooth long-term frequency tracking.
 * Matches Microchip example: motor.c lines 444 (>>4) and 448 (>>2). */
#ifndef ZC_SYNC_KP_SHIFT
#define ZC_SYNC_KP_SHIFT         2
#endif
#ifndef ZC_SYNC_KI_SHIFT
#define ZC_SYNC_KI_SHIFT         4
#endif

/* Speed thresholds with hysteresis.
 * Entry at 60k: above TAL transition zone, DMA has data.
 * Exit at 50k: below DMA threshold, poll is reliable. */
#ifndef ZC_SYNC_ENTRY_ERPM
#define ZC_SYNC_ENTRY_ERPM       26000UL  /* Lowered for debugging: get PI
                                           * measurements at lower speed where
                                           * reactive margin is positive and
                                           * motor runs cleanly. */
#endif
#ifndef ZC_SYNC_EXIT_ERPM
#define ZC_SYNC_EXIT_ERPM        20000UL
#endif

/* Corridor half-width for DMA cluster selection.
 * Clusters outside expectedHR ± corridor are rejected.
 * Default: T_hatHR/4 (±15° = ±25% of sector period). */
#ifndef ZC_SYNC_CORRIDOR_DIV
#define ZC_SYNC_CORRIDOR_DIV     4
#endif

#ifndef FEATURE_REACTIVE_LATENCY_SPLIT
#define FEATURE_REACTIVE_LATENCY_SPLIT  0  /* DISABLED — experiment proved that
                                            * splitting TAL in the reactive path
                                            * doesn't help without also changing
                                            * the measurement source. The sector
                                            * PI synchronizer (FEATURE_SECTOR_PI)
                                            * handles advance/delay separation
                                            * in a new control path instead. */
#endif

#ifndef REACTIVE_LATENCY_COMP_LEVELS
#define REACTIVE_LATENCY_COMP_LEVELS  1U   /* Only used when FEATURE_REACTIVE_LATENCY_SPLIT=1 */
#endif

#ifndef FEATURE_DMA_ZC_DIRECT
#define FEATURE_DMA_ZC_DIRECT   0   /* DISABLED — selector is validated but live
                                     * deployment not production-ready.
                                     *
                                     * The selector ("last ≥2-edge cluster before
                                     * poll, return midpoint") is sound: 1.3–7 µs
                                     * stdev, <3 µs polarity split, validated
                                     * offline across 25–90k eRPM with and without
                                     * prop. See memory/ck_bemf_signal_physics.md.
                                     *
                                     * The reactive scheduler now has an explicit
                                     * latency-compensation term, but live DMA
                                     * substitution is still off until bench data
                                     * confirms the split is calibrated well enough
                                     * to replace poll timing in the hot path. */
#endif

#if FEATURE_DMA_ZC_DIRECT && !FEATURE_IC_DMA_SHADOW
#error "FEATURE_DMA_ZC_DIRECT requires FEATURE_IC_DMA_SHADOW"
#endif

#ifndef FEATURE_DMA_BURST_CAPTURE
#define FEATURE_DMA_BURST_CAPTURE  1   /* Research tool: one-shot capture of
                                        * DMA raw edge sequences for 12
                                        * consecutive commutation steps.
                                        * Armed via GSP command; motor runs
                                        * unaffected. Cost: ~600 bytes .bss */
#endif

#if FEATURE_DMA_BURST_CAPTURE && !FEATURE_IC_DMA_SHADOW
#error "FEATURE_DMA_BURST_CAPTURE requires FEATURE_IC_DMA_SHADOW"
#endif

#ifndef FEATURE_6STEP_DPLL
#define FEATURE_6STEP_DPLL  1       /* 6-step event DPLL for commutation.
                                     * When 1: a digital PLL tracks rotor
                                     * phase from ZC events and owns
                                     * commutation scheduling at high speed.
                                     * Separates T_hat (period), phaseBiasHR
                                     * (measurement bias), and advanceCmdHR
                                     * (torque advance) — the reactive path's
                                     * delayHR = halfHR - advHR tangles them.
                                     *
                                     * V1: t_meas = pollHR (no DMA).
                                     * V2: t_meas = DMA cluster midpoint.
                                     *
                                     * Requires FEATURE_IC_ZC. */
#endif

/* DPLL handoff speed threshold. Predictive mode only engages above
 * this eRPM. Below this, reactive poll scheduling is sufficient. */
#ifndef DPLL_HANDOFF_ERPM
#define DPLL_HANDOFF_ERPM   200000UL  /* Shadow-only. Predictive ownership
                                       * causes limit cycle: T_hat drift →
                                       * phase error → exit → re-enter.
                                       * Need T_hat to track actual period
                                       * during ownership before enabling. */
#endif

/* DMA-primary ZC detection: above the speed threshold, FastPoll
 * queries the DMA ring for qualifying edge clusters instead of
 * reading the comparator GPIO with multi-read filtering.
 *
 * Eliminates ~16 µs of poll filter latency. The DMA ring already
 * has hardware-precise timestamps of every comparator edge.
 * Poll ISR becomes a DMA consumer, not a GPIO sampler.
 *
 * Window: open = max(blankingEnd, 50% gate), close = now.
 * Cluster: ≥2 edges within DMA_DETECT_GAP_HR (6.4 µs).
 * Fallback: if no qualifying cluster by late bound, fall through
 *           to the existing poll/raw path.
 *
 * Requires FEATURE_IC_DMA_SHADOW. */
#ifndef FEATURE_DMA_PRIMARY_ZC
#define FEATURE_DMA_PRIMARY_ZC   0    /* DISABLED: DMA timestamp is ~20 HR
                                       * earlier than poll. Reactive scheduling
                                       * now separates detector latency from
                                       * torque advance, but DMA-primary still
                                       * stays off until the new split is bench-
                                       * tuned and validated across the 50-100k
                                       * eRPM band. */
#endif

/* Speed threshold for DMA-primary ZC and DMA-direct substitution.
 * Expressed as stepPeriodHR (HR ticks per commutation step).
 * 15,625,000 HR ticks/sec ÷ 312 ≈ 50,080 eRPM.
 * Lower (smaller) stepPeriodHR means FASTER motor → gate opens. */
#ifndef DMA_ZC_DIRECT_THRESHOLD_HR
#define DMA_ZC_DIRECT_THRESHOLD_HR    312u   /* enable above ~50k eRPM */
#endif

/* Max absolute |refinedHR - pollHR| allowed. Outside this, the DMA
 * edge is considered unrelated and poll is used unchanged. 300 HR
 * ticks = ~192 μs — covers the worst observed poll latency. */
#ifndef DMA_ZC_DIRECT_MAX_CORRECTION_HR
#define DMA_ZC_DIRECT_MAX_CORRECTION_HR  300u
#endif

/* Speed-adaptive bias added to the refined timestamp AFTER
 * substitution. Compensates for the advance shift caused by moving
 * lastZcTickHR earlier. The measured backdate is speed-dependent:
 *   ~22 HR at 30-50k eRPM (stepHR ~312)
 *   ~24 HR at 50-60k       (stepHR ~260)
 *   ~27 HR at 60-70k       (stepHR ~220)
 *   ~29 HR at 70-90k       (stepHR ~195)
 * A linear fit:
 *   bias = BASE + (THRESHOLD - stepHR) / SLOPE
 * maps these points well. */
#ifndef DMA_ZC_DIRECT_BIAS_BASE_HR
#define DMA_ZC_DIRECT_BIAS_BASE_HR     22u   /* bias at threshold crossing */
#endif
#ifndef DMA_ZC_DIRECT_BIAS_SLOPE
#define DMA_ZC_DIRECT_BIAS_SLOPE       17u   /* HR ticks of stepHR per +1 bias */
#endif

#ifndef FEATURE_PRED_WINDOW_GATE
#define FEATURE_PRED_WINDOW_GATE 0  /* PLL predictor scan window gate.
                                     * When gateActive (12 consecutive locked+in-window),
                                     * veto ZC candidates outside the predicted scan window.
                                     * On veto: clear candidate, continue scanning same step.
                                     * Self-disarms on timeout or 2+ rejects/revolution. */
#endif

#ifndef FEATURE_PTG_ZC
#define FEATURE_PTG_ZC          0   /* PTG edge-relative BEMF sampling.
                                     * 0 = disabled (CLC D-FF only — proven working)
                                     * 1 = PTG supplements CLC: reads raw GPIO at
                                     *     fixed delay after switching edge.
                                     *     Fixes duty-dependent CLC clock position
                                     *     issue at 50%/80% duty. */
#endif
#if FEATURE_PTG_ZC
#define PTG_DELAY_US            7U  /* Delay after switching edge (µs).
                                     * Must be > ATA6847 comparator settling (~3µs)
                                     * and > edge blanking (EGBLT=3.75µs at max).
                                     * 7µs gives margin. Increase to 10µs at 24V
                                     * if ringing is still visible. */
#define PTG_BTE_BIT             9U  /* PTGBTE bit for PG1 ADC Trigger 2 (TRIGB).
                                     * Device-specific — verify against DS70005178
                                     * Table 2 for dsPIC33CK64MP205.
                                     * If PTG doesn't trigger: try 1, 2, or 8. */
#endif

#ifndef FEATURE_GSP
#define FEATURE_GSP             1   /* 1 = GSP binary protocol on UART1 (disables debug prints)
                                     * 0 = debug UART text output (default for development)
                                     * Set to 1 when using GUI or gsp_ck_test.py */
#endif

/* ── SCCP1 Fast ZC Polling Timer (FEATURE_IC_ZC=1) ────────────────── */
/* Replaces edge-triggered IC with periodic timer that polls ATA6847
 * BEMF comparator directly. Step 0 experiment confirmed comparator
 * output is PWM-independent — valid during both ON and OFF states.
 *
 * At 100 kHz (10 µs period), the ISR reads the floating phase comparator
 * and applies an adaptive deglitch filter (2-8 consecutive matching reads).
 * Blanking uses SCCP4 HR timestamps (640 ns resolution).
 *
 * CPU budget: ~15-20 instructions per ISR call = 150-200 ns.
 * At 100 kHz, worst-case idle CPU = 2%. ZC detection + scheduling
 * adds ~1-2 µs once per step (negligible). */
#if FEATURE_IC_ZC
#define ZC_POLL_FREQ_HZ         210526U     /* ~210.5 kHz — NOT an integer multiple of
                                              * 20kHz PWM. 200kHz/20kHz = exactly 10 polls
                                              * per PWM cycle → polls always hit the same
                                              * positions → aliasing at 60k/80k eRPM.
                                              * 210.5kHz/20kHz = 10.526 → polls drift
                                              * across PWM cycle, smearing switching noise.
                                              * Period = 475 ticks (vs 500 at 200kHz).
                                              * 100kHz desync'd 2810 at 36k eRPM (Tp=5).
                                              * 200kHz doubles polls per step — FL=3 at Tp=5
                                              * gets 50 polls (10 blanked) vs 25 at 100kHz.
                                              * False ZC rate managed by per-rev desync check. */
#define ZC_POLL_PERIOD          (FCY / ZC_POLL_FREQ_HZ)  /* 1000 ticks at 100MHz Fp */
#define ZC_POLL_ISR_PRIORITY    5           /* Above Timer1(4), below ComTimer(6) */
#define ZC_IC_ISR_PRIORITY      5           /* SCCP2 IC capture: same as poll */

/* Adaptive deglitch filter: consecutive matching reads to confirm ZC.
 * Fewer reads at high speed (tight timing margins), more at low speed
 * (more noise, ample time). Scales linearly with step period (Timer1 ticks).
 *
 * At 100kHz poll rate and ZC_DEGLITCH_MIN=3:
 *   Detection latency = 3 × 10µs = 30µs = 4.3° at 100k eRPM (Tp:2)
 * At ZC_DEGLITCH_MAX=8:
 *   Detection latency = 8 × 10µs = 80µs = 2.9° at 4k eRPM (Tp:50)
 *
 * FL=2 was too thin at 100k eRPM — 7.7% false IC rate. FL=3 requires
 * 3 consecutive matching reads (30µs), still fits in the ~80µs post-
 * blanking window at Tp:2. */
#define ZC_DEGLITCH_MIN         3U          /* FL=2 tested: accepts false ZCs at 62k eRPM.
                                             * At 200kHz, FL=3 = 15µs latency (vs 30µs at 100kHz).
                                             * Tp=2 at 200kHz: 100µs step, 50µs usable, 15µs FL = OK. */
#define ZC_DEGLITCH_MAX         8U          /* At Tp >= ZC_DEGLITCH_SLOW_TP */
#define ZC_DEGLITCH_FAST_TP     5U          /* Tp threshold for min filter (was 10) */
#define ZC_DEGLITCH_SLOW_TP     50U         /* Tp threshold for max filter */

/* PWM-aware IC age budget (replaces hard StpHR < 300 gate).
 * Maximum acceptable IC-to-poll delay: IC fires on comparator edge,
 * CLC D-FF updates on next PWM valley, poll reads CLC on next poll.
 * Budget = pwmPeriod + pollPeriod + margin.
 * In HR ticks (640ns): LOOPTIME_MICROSEC * 25/16 + pollPeriod_us * 25/16 + margin.
 * At 40kHz: 39 + 7 + 10 = 56 HR ticks (~36µs)
 * At 24kHz: 65 + 7 + 10 = 82 HR ticks (~53µs) */
#define IC_AGE_PWM_HR       ((uint16_t)((uint32_t)LOOPTIME_MICROSEC * 25u / 16u))
#define IC_AGE_POLL_HR      ((uint16_t)((1000000UL / ZC_POLL_FREQ_HZ) * 25u / 16u))
#define IC_AGE_MARGIN_HR    10u   /* ~6.4µs margin for ISR jitter */
#define IC_AGE_MAX_HR       (IC_AGE_PWM_HR + IC_AGE_POLL_HR + IC_AGE_MARGIN_HR)
/* PWM period in HR ticks — used for candidateAge fallback check */
#define PWM_PERIOD_HR       IC_AGE_PWM_HR
/* Phase 2: IC lead budget — max acceptable IC-ahead-of-raw gap.
 * If IC fires this far before raw stabilises, the IC caught a bounce.
 * Derived from poll period + margin, not a magic number.
 * At 210kHz: pollPeriodHR ≈ 7, margin 10 → 17 HR ticks (~11µs) */
#define IC_LEAD_MAX_HR      (IC_AGE_POLL_HR + IC_AGE_MARGIN_HR)
/* Phase 2: raw stability threshold for fast accept.
 * Main rule: rawCoro >= 2 (two consecutive raw matches).
 * Jitter insurance: raw has been stable for >= 1 poll period.
 * rawCoro >= 2 is the primary gate; age check is backup for cases
 * where rawCoro is exactly 1 but raw has been stable long enough
 * that the single-sample concern doesn't apply. */
#define RAW_STABLE_AGE_HR   IC_AGE_POLL_HR
#endif

/* ── Desync Recovery ───────────────────────────────────────────────── */
#define DESYNC_RESTART_MAX  10U   /* More restart attempts before fault.
                                    * Counter resets after 2s stable CL. */
#define RECOVERY_TIME_MS    50U    /* Shortened from 200ms — PWM is OFF during
                                    * recovery so no current risk. Faster restart. */
#define RECOVERY_COUNTS     ((uint16_t)((uint32_t)RECOVERY_TIME_MS * TIMER1_FREQ_HZ / 1000))

/* ── ARM ───────────────────────────────────────────────────────────── */
#define ARM_TIME_MS         200U
#define ARM_TIME_COUNTS     ((uint16_t)((uint32_t)ARM_TIME_MS * TIMER1_FREQ_HZ / 1000))

/* ── ADC Channel Mapping (dsPIC33CK on EV43F54A) ──────────────────── */
/* AN6 = Potentiometer (speed ref), AN9 = Vbus voltage */
#define ADCBUF_POT          ADCBUF6
#define ADCBUF_VBUS         ADCBUF9

#endif /* GARUDA_CONFIG_H */
