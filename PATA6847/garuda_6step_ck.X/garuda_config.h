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
#define PWMFREQUENCY_HZ     20000U       /* 20 kHz switching */
#define LOOPTIME_MICROSEC   (uint16_t)(1000000UL / PWMFREQUENCY_HZ)  /* 50 us */
#define LOOPTIME_TCY        (uint16_t)((uint32_t)LOOPTIME_MICROSEC * FOSC_PWM_MHZ / 2 - 1)
#define MAX_DUTY            (LOOPTIME_TCY - 200U)  /* Min off-time margin */
#define MIN_DUTY            200U
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

/* Timing Advance — Hurst has high inductance (471µH), moderate advance needed.
 * At max speed (Tp:11), total delay from real ZC = 3 + sp*(30-adv)/60.
 * With 8° max: 3 + 11*22/60 = 7 ticks. HalfStep = 5.5. OK margin. */
#define TIMING_ADVANCE_MIN_DEG      0U   /* Degrees advance at low speed */
#define TIMING_ADVANCE_MAX_DEG      8U   /* Degrees advance at max speed */
#define TIMING_ADVANCE_START_ERPM   6000U /* eRPM where advance begins (~1200 mech RPM) */

/* Closed-Loop */
#define MAX_CLOSED_LOOP_ERPM 15000U      /* ~3000 mech RPM */
#define RAMP_TARGET_ERPM     3000U       /* OL→CL handoff speed */
#define CL_IDLE_DUTY_PERCENT 8U          /* Minimum duty in CL (idle floor) */

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

/* Timing Advance — compensates commutation delay at high eRPM.
 *
 * Fast poll (100kHz) with SCCP4 HR timestamps (640ns) gives ~10µs
 * worst-case ZC timing error. Advance must ensure comm delay >
 * deglitch latency (30µs at min filter=3) so HR scheduling works.
 *
 * At Tp:6 (33k eRPM), advance = 20*(33-5)/(40-5) = 16°:
 *   delay = 300 × 14/60 = 70µs — comfortable
 * At Tp:5 (40k+ eRPM), advance = 20° (max):
 *   delay = 250 × 10/60 = 42µs — tight but HR-scheduled */
#define TIMING_ADVANCE_MIN_DEG      0U   /* Degrees advance at low speed */
#define TIMING_ADVANCE_MAX_DEG      20U  /* Degrees advance at max speed */
#define TIMING_ADVANCE_START_ERPM   5000U /* eRPM where advance begins */

/* Closed-Loop advance curve endpoint.
 * Advance ramps from 0° to 20° linearly over 5k-40k eRPM range.
 * Beyond 40k eRPM, advance stays at 20° (max). */
#define MAX_CLOSED_LOOP_ERPM 100000U     /* Advance curve endpoint — A2212 reaches 100k+ eRPM */
#define MIN_CL_STEP_PERIOD   2U          /* Tp floor — was 5, clamped entire pot range.
                                          * Tp:2 = 100k eRPM. Below Tp:2, Timer1 quantization
                                          * is too coarse; HR timing handles sub-tick precision. */
#define RAMP_TARGET_ERPM     4000U       /* OL→CL handoff speed (~571 mech RPM) */
/* ATA6847 Hardware Current Limit — cycle-by-cycle chopping.
 * TEST VALUE: DAC=75 → 5.3A peak trip. At ~50% duty, DC bus limit
 * ≈ 2.6A — easily triggered on bench with prop.
 * PRODUCTION: raise to DAC=95 (16.6A) for flight. */
#define ILIM_DAC            85U     /* TEST: calibrating actual trip point */

#define CL_IDLE_DUTY_PERCENT 10U         /* Higher idle for prop drag (10% of 12V) */

/* Vbus Fault Thresholds (12V / 3S LiPo system)
 * EV43F54A Vbus divider: ~1211 raw/V (from 24V→29072 calibration).
 * Bench supply sags significantly under motor load (12V → 9V at full duty).
 * OV: 18V (headroom for regen spikes + 4S compatibility)
 * UV: 6V (only trips on actual supply disconnect, not load sag) */
#define VBUS_OV_THRESHOLD   (1363U * 16U)  /* ~18V → 21808 in 16-bit */
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
#define ALIGN_DUTY          (LOOPTIME_TCY / 40)    /* ~2.5% duty */
#define INITIAL_STEP_PERIOD 1000U        /* Timer1 ticks (50ms = ~200 eRPM) */
#define MIN_STEP_PERIOD     50U          /* Timer1 ticks (2.5ms = ~4000 eRPM) */
#define RAMP_ACCEL_ERPM_S   500U         /* eRPM/s — moderate, proven on this motor */
#define RAMP_DUTY_CAP       (LOOPTIME_TCY / 15)    /* Max ~7% duty during OL ramp */

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

/* Timing Advance — 2810 has low Ls (25µH), BEMF rises fast.
 * Start advance earlier (3000 eRPM vs 5000) and ramp to 25° (vs 20°).
 * Low-Ls motors benefit from more advance at high speed because the
 * commutation delay (deglitch + scheduling) is a larger fraction of
 * the electrical period. */
#define TIMING_ADVANCE_MIN_DEG      2U    /* Small base advance even at low speed */
#define TIMING_ADVANCE_MAX_DEG      25U   /* More advance for low-Ls motor */
#define TIMING_ADVANCE_START_ERPM   3000U /* Start earlier — BEMF usable sooner */

/* Closed-Loop
 * At 25.2V (6S full): 1350 × 25.2 = 34020 RPM = 238k eRPM theoretical.
 * Practical max ~130k eRPM with losses. */
#define MAX_CLOSED_LOOP_ERPM 150000U
#define MIN_CL_STEP_PERIOD   2U          /* HR timer provides sub-tick precision.
                                          * Per-rev desync check uses HR (0.4% error)
                                          * instead of Timer1 (33% error at Tp=2). */
#define RAMP_TARGET_ERPM     3000U       /* Lower than A2212 — gentler handoff */

/* ATA6847 Hardware Current Limit */
#define ILIM_DAC            120U         /* ~30.6A peak. With props, commutation
                                         * transients reach 20-25A at mid speed.
                                         * 110 (24.7A) chops too aggressively under
                                         * load, limiting max speed to ~40k eRPM.
                                         * 120 allows prop operation while still
                                         * protecting during genuine desync (34A+). */

#define CL_IDLE_DUTY_PERCENT 10U

/* Vbus Fault Thresholds
 * OV: 28V (margin above fully charged 6S)
 * UV: 10V (bench supply sags under desync current spikes;
 *     production should use 16V for LiPo protection) */
#define VBUS_OV_THRESHOLD   (38752U)       /* ~32V. Was 28V — regen spikes during
                                         * fast deceleration reach 28V at 24V supply.
                                         * 32V gives margin for regen while still
                                         * protecting against real OV faults. */
#define VBUS_UV_THRESHOLD   (12110U)       /* ~10V → 1211*10 */

#else
#error "Unknown MOTOR_PROFILE — select 0 (Hurst), 1 (A2212), or 2 (2810)"
#endif

/* ================================================================
 *          SHARED PARAMETERS  (all motor profiles)
 * ================================================================ */

#define ALIGN_TIME_COUNTS   ((uint16_t)((uint32_t)ALIGN_TIME_MS * TIMER1_FREQ_HZ / 1000))
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

/* Bypass guard bounds (used when FEATURE_IC_ZC_ADAPTIVE=1) */
#define ZC_BYPASS_LO_PCT    40U     /* Min % of stepPeriodHR for bypass acceptance */
#define ZC_BYPASS_HI_PCT   160U     /* Max % of stepPeriodHR for bypass acceptance */

/* ── Feature Flags ─────────────────────────────────────────────────── */
#ifndef FEATURE_IC_ZC
#define FEATURE_IC_ZC           1   /* 1 = fast poll timer ZC, 0 = ADC ISR polling only */
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
#define ZC_POLL_FREQ_HZ         200000U     /* 200 kHz polling rate.
                                              * 100kHz desync'd 2810 at 36k eRPM (Tp=5).
                                              * 200kHz doubles polls per step — FL=3 at Tp=5
                                              * gets 50 polls (10 blanked) vs 25 at 100kHz.
                                              * False ZC rate managed by per-rev desync check. */
#define ZC_POLL_PERIOD          (FCY / ZC_POLL_FREQ_HZ)  /* 1000 ticks at 100MHz Fp */
#define ZC_POLL_ISR_PRIORITY    5           /* Above Timer1(4), below ComTimer(6) */

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
#endif

/* ── Desync Recovery ───────────────────────────────────────────────── */
#define DESYNC_RESTART_MAX  3U
#define RECOVERY_TIME_MS    200U
#define RECOVERY_COUNTS     ((uint16_t)((uint32_t)RECOVERY_TIME_MS * TIMER1_FREQ_HZ / 1000))

/* ── ARM ───────────────────────────────────────────────────────────── */
#define ARM_TIME_MS         200U
#define ARM_TIME_COUNTS     ((uint16_t)((uint32_t)ARM_TIME_MS * TIMER1_FREQ_HZ / 1000))

/* ── ADC Channel Mapping (dsPIC33CK on EV43F54A) ──────────────────── */
/* AN6 = Potentiometer (speed ref), AN9 = Vbus voltage */
#define ADCBUF_POT          ADCBUF6
#define ADCBUF_VBUS         ADCBUF9

#endif /* GARUDA_CONFIG_H */
