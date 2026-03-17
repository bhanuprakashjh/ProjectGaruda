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
#define MOTOR_PROFILE   1   /* 0=Hurst, 1=A2212 */
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
#define CL_IDLE_DUTY_PERCENT 10U         /* Higher idle for prop drag (10% of 12V) */

/* Vbus Fault Thresholds (12V / 3S LiPo system)
 * EV43F54A Vbus divider: ~1211 raw/V (from 24V→29072 calibration).
 * OV: 15V → 18165 raw. UV: 8V → 9690 raw.
 * Using 12-bit × 16 notation for consistency with Hurst profile. */
#define VBUS_OV_THRESHOLD   (1136U * 16U)  /* ~15V → 18176 in 16-bit */
#define VBUS_UV_THRESHOLD   (606U * 16U)   /* ~8V  → 9696 in 16-bit */

#else
#error "Unknown MOTOR_PROFILE — select 0 (Hurst) or 1 (A2212)"
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

/* ── Feature Flags ─────────────────────────────────────────────────── */
#ifndef FEATURE_IC_ZC
#define FEATURE_IC_ZC           1   /* 1 = fast poll timer ZC, 0 = ADC ISR polling only */
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
#define ZC_POLL_FREQ_HZ         100000U     /* 100 kHz polling rate */
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
#define ZC_DEGLITCH_MIN         3U          /* At Tp <= ZC_DEGLITCH_FAST_TP */
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
