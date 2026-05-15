/**
 * @file garuda_config.h
 * @brief Configuration for 6-step BLDC on dsPIC33AK + ATA6847L.
 *
 * Target: EV92R69A carrier (ATA6847L QFN48) + EV68M17A DIM (dsPIC33AK128MC106).
 * Forked from CK source-of-truth `../../garuda_6step_ck.X/garuda_config.h`
 * at commit 24703f6 (228k eRPM milestone). Bulk of file is hardware-
 * agnostic; only sections marked [AK PORT] differ from CK.
 *
 * Motor profiles:
 *   0 = Hurst DMB2424B10002 (5PP, 24V, 149KV, low-speed bench motor)
 *   1 = A2212 1400KV        (7PP, 12V, 1400KV, drone motor)
 *   2 = 2810 1350KV         (7PP, 24V, drone motor — 228k eRPM target)
 *   3 = HiZ1460             (7PP, 30V, high-Z bench)
 *
 * [AK PORT] open items — re-tune after Phase 0 bring-up:
 *   - FOSC_PWM_MHZ may need adjustment for AK CLK5 (currently 400 MHz to
 *     match CK; AK supports up to 500 MHz on CLK5 — verify in clock.c).
 *   - ADC channel mapping (bottom of file) — EV68M17A wires POT/Vbus to
 *     different AN pins than EV43F54A; update once DIM info sheet
 *     (DS70005527) cross-referenced with EV92R69A schematic.
 *   - CCP5 references in CK code use SCCP4 on AK (AK has SCCP1-4 only);
 *     see `motor/sector_pi.c` port notes.
 */

#ifndef GARUDA_CONFIG_H
#define GARUDA_CONFIG_H

/* ── Motor Profile Selection ──────────────────────────────────────── */
#ifndef MOTOR_PROFILE
#define MOTOR_PROFILE   2   /* 0=Hurst, 1=A2212, 2=2810, 3=HiZ1460 */
#endif

/* ── Clock — [AK PORT] ─────────────────────────────────────────────── */
/* dsPIC33AK128MC106 (verified against DS70005539 §12):
 *   Fosc = 200 MHz (PLL1 FOUT)
 *   Fcy  = 200 MHz — AK is 1-cycle-per-instruction, so Fcy == Fosc
 *                    (NOT Fosc/2 like CK). Datasheet line 130569 confirms:
 *                    "6 instruction cycles or 30 nS @ 200 MHz CPU clock"
 *                    → 5 ns/cycle → Fcy = 200 MHz.
 *   CLK5 = 400 MHz (PWM clock — VCO_DIV=800 MHz, INTDIV=1 → /2 → 400 MHz)
 *
 * MPER / MDC / MPHASE are 20-bit registers but only the upper 16 bits
 * are programmable in standard PWM mode (datasheet §14.3.6: bits 3:0
 * are Reserved). The extra 4 bits are exposed by AK's High-Resolution
 * PWM mode for sub-Tcy fractional pulse-width control — not used by
 * us since 6-step doesn't need sub-2.5 ns timing precision. */
#define FOSC                200000000UL  /* 200 MHz */
#define FCY                 200000000UL  /* 200 MHz instruction clock (AK: 1 cyc/instr) */
#define FOSC_PWM_MHZ        400U         /* CLK5 PWM clock (MHz) — see hal/clock.c */

/* ── PWM ───────────────────────────────────────────────────────────── */
/* Supported carriers: 20 kHz, 40 kHz, 60 kHz (compile-time selection).
 * LOOPTIME_TCY at FOSC_PWM=400 MHz fits in uint16_t for all three:
 *   20 kHz → 50 µs → MPER = 9999  ticks (DEADTIME 0x14 = 0.20% of period)
 *   40 kHz → 25 µs → MPER = 4999  ticks (DEADTIME 0x14 = 0.40% of period)
 *   60 kHz → 16 µs → MPER = 3199  ticks (DEADTIME 0x14 = 0.63% of period)
 *
 * CK-board bench data (228k eRPM milestone):
 *   - 40 kHz: peak 195k, walls early
 *   - 50 kHz: peak 220k (also with 2x ADC)
 *   - 60 kHz: peak 228k stable — 2026-04-29 milestone keeper
 *   - 20 kHz: quieter audible noise, more current ripple, lower ceiling
 *     (expected); useful for low-speed bench work + EMI debug.
 *
 * Win is current-ripple / BEMF cleanliness, not sampling rate (verified
 * 2026-04-29 with 2x ADC experiment at 50 kHz that gave no improvement).
 *
 * [AK PORT] 20 kHz has not been bench-validated on this firmware since
 * the V4 milestone — verify ADC ISR timing budget at 50 µs/tick still
 * leaves headroom for sector PI + GSP. */
#define PWMFREQUENCY_HZ     60000U   /* 20000U | 40000U | 60000U
                                      * 2026-05-15: tested 40 kHz — peak
                                      * dropped 226k→166k with desync,
                                      * currents *increased* at same speed.
                                      * Definitive: AK gap is commutation
                                      * accuracy, not switching loss. AK
                                      * needs the 60 kHz BEMF sample rate
                                      * to keep timing accurate at peak. */
#define LOOPTIME_MICROSEC   (uint16_t)(1000000UL / PWMFREQUENCY_HZ)  /* 50 us */

/* [AK PORT — CRITICAL] PWM period equation on dsPIC33AK is NOT the simple
 * CK model. Per DS70005539 Eq 14-1 (Center-Aligned, Push-Pull off):
 *     FPWM       = 8 × FPGx_clk / (PGxPER + 16)
 *     PGxPER     = (8 × FPGx_clk / FPWM) − 16
 * For PGx_clk = 400 MHz (CLKGEN5), PGxPER for 60 kHz = 53317 → overflows
 * uint16_t for 40/20 kHz. We instead route Std Speed Peripheral Clock
 * (100 MHz) to PWM via PCLKCON.MCLKSEL=0 (set in hal_pwm.c), so:
 *     20 kHz → PGxPER = 39984
 *     40 kHz → PGxPER = 19984
 *     60 kHz → PGxPER = 13317
 * All fit uint16_t. Trade-off: 10 ns PWM tick on AK vs 5 ns on CK; for
 * 6-step BLDC the >13-bit duty resolution at 60 kHz is still excessive. */
#define FPGX_CLK_HZ         100000000UL   /* PG clock — Std Speed Periph Clk */
#define LOOPTIME_TCY        (uint16_t)((8UL * FPGX_CLK_HZ / PWMFREQUENCY_HZ) - 16U)
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
/* Dead time: ATA6847L handles 700 ns CCPT internally, so MCU dead time
 * is minimal. [AK PORT] Per DS70005539 Eq 14-2:
 *     PGxDTy = 16 × FPGx_clk × DeadTime(s)
 * For 100 ns at FPGx_clk=100 MHz: 16 × 100e6 × 100e-9 = 160 ticks.
 * CK comment said "0x14 = 50 ns" but actual was 100 ns at 200 MHz CK
 * effective clock. Holding 100 ns on AK; raise if VDS transients leak. */
#define DEADTIME_TCY        ((uint16_t)((uint32_t)16U * FPGX_CLK_HZ / 1000000UL / 10U))  /* 100 ns = 160 */

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
/* On AK, Timer1 input is Standard Speed Peripheral Clock = FPB/2 = 100 MHz
 * (not FCY=200 MHz directly). This matches CK behaviour where FPB=FCY=100MHz.
 * Both MCUs produce the same Timer1 tick rate, so PR1 value is identical. */
#define TIMER1_INPUT_CLOCK  100000000UL  /* Hz — Std Speed Peripheral Clock */
#define TIMER1_PRESCALE     8U
#define TIMER1_FREQ_HZ      20000U       /* 50 µs period — matches ADC ISR rate */
#define TIMER1_PR           (uint16_t)(TIMER1_INPUT_CLOCK / TIMER1_PRESCALE / TIMER1_FREQ_HZ - 1)

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

/* Startup — aligned with profile 1's proven bench cadence on
 * 2026-04-29. The previous "gentle 2810" values (5% align, /8 duty
 * cap, 500 eRPM/s ramp) were never bench-validated and broke startup
 * (rotor didn't track the slow ramp → CL seeded with bad period →
 * PI runaway to phantom 700k eRPM). Profile 1's settings drive the
 * 2810 motor reliably to 235-238k eRPM, so adopting them here.
 * ZC tunables (blanking, advance) are kept profile-2-specific because
 * those are CL-stage params where the 2810's higher-speed regime
 * benefits from tighter blanking (8% vs 10%, 15 µs floor vs 25 µs). */
#define ALIGN_TIME_MS       150U
#define ALIGN_DUTY          (LOOPTIME_TCY / 40)    /* ~2.5% — matches profile 1 */
#define INITIAL_STEP_PERIOD 800U
#define MIN_STEP_PERIOD     50U          /* Timer1 ticks (2.5ms = ~4000 eRPM) */
/* OL ramp tuning 2026-05-13:
 * - Faster ramp (2500 vs 1500 eRPM/s) → 2.0s → 1.2s in forced mode.
 * - Duty cap stays at /6 (17%): bench-tested /10 (10%) and /8 (12.5%)
 *   both failed to break standstill on the 2810 (iaPk≈0.4A, rotor
 *   stationary, stl=41 at CL entry). 2810 cogging needs the full 17%.
 *   Startup roughness reduction comes from the shorter ramp time only. */
#define RAMP_ACCEL_ERPM_S   2500U        /* 1500 → 2500: ramp 2.0s → 1.2s */
#define RAMP_DUTY_CAP       (LOOPTIME_TCY / 6)     /* ~17% during OL ramp */

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

#elif MOTOR_PROFILE == 3
/* ── Motor: HiZ1460 (high-impedance 1460KV @ 30V) ────────────────
 * 7PP, 16Ω phase resistance, 15µH inductance, 1460 RPM/V, 30V max.
 * High-Z motor: max current capped by Rs at 30/16 = 1.87A. Cannot
 * overcurrent regardless of duty — runs very different operating
 * point from 2810/A2212 (low-Rs, high-current motors).
 *
 *   ALIGN: 50% duty × 30V = 15V → ~0.94A align current
 *   RAMP : 80% duty × 30V = 24V → ~1.50A peak ramp
 *   FULL : 100% duty × 30V = 30V → 1.87A (ohmic limit)
 * Theoretical no-load: 1460 × 30 = 43800 RPM = 306k eRPM. */
#define MOTOR_POLE_PAIRS    7U
#define MOTOR_RS_MILLIOHM   16000U       /* 16 Ω */
#define MOTOR_LS_MICROH     15U
#define MOTOR_KV            1460U

/* Startup — proven by colleague's bench testing on the actual HiZ
 * 16 Ω motor at commit 1956441. High-Rs motor has Rs naturally
 * limiting current, so startup needs SLOW ramp and modest duty —
 * not aggressive. Earlier draft (50% align / 80% cap) was wrong:
 * Rs limits current anyway, but the abrupt step was destabilizing. */
#define ALIGN_TIME_MS       200U
#define ALIGN_DUTY          (LOOPTIME_TCY / 20)    /* ~5% — soft align */
#define INITIAL_STEP_PERIOD 1000U
#define MIN_STEP_PERIOD     50U          /* 2.5 ms = ~4000 eRPM handoff */
#define RAMP_ACCEL_ERPM_S   500U         /* Slow ramp — high-Z motor needs time */
#define RAMP_DUTY_CAP       (LOOPTIME_TCY / 8)     /* ~12.5% — modest */

/* ZC Detection — same as 2810; low Ls means fast current settling. */
#define ZC_BLANKING_PERCENT 20U
#define ZC_FILTER_THRESHOLD 3U
#define ZC_BLANK_PCT_SLOW   12U
#define ZC_BLANK_PCT_FAST    8U
#define ZC_BLANK_FLOOR_US   15U
#define ZC_BLANK_CAP_PCT    45U
#define FEATURE_IC_ZC_ADAPTIVE 1
#define TIMING_ADVANCE_LEVEL 3U

/* Closed-Loop */
#define MAX_CLOSED_LOOP_ERPM 250000U     /* 30V × KV × PP headroom */
#define MIN_CL_STEP_PERIOD   2U
#define RAMP_TARGET_ERPM     3000U

/* Idle duty for high-Z motor — needs more % to make any torque.
 * 35% × 30V = 10.5V → 0.66 A continuous. */
#define CL_IDLE_DUTY_PERCENT 35U

/* V4_MIN_AMPLITUDE_PROFILE for HiZ1460 is set in the V4 startup block
 * below (50% Q15). High-Z motor desyncs below ~50% Q15 because BEMF is
 * too weak for the duty to maintain lock. */

/* ATA6847 ILIM — max possible current is 1.87A, far below any DAC
 * setting. Effectively disabled (current is naturally Rs-limited). */
#define ILIM_DAC            120U

#define VBUS_OV_THRESHOLD   (48440U)     /* ~40V — same as 2810 */
#define VBUS_UV_THRESHOLD   (12110U)     /* ~10V */

#else
#error "Unknown MOTOR_PROFILE — select 0 (Hurst), 1 (A2212), 2 (2810), or 3 (HiZ1460)"
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

/* ── V4 Sector PI Architecture ────────────────────────────────────────
 * Ground-up rewrite modeled on Microchip AVR high-speed motor control.
 * Two timers + one PI. No poll, no DMA, no reactive/predictive modes.
 * SCCP3 = sector timer (periodic, fires commutation ISR).
 * CCP2  = capture timer (IC mode, ISR drains FIFO, last edge wins).
 * PI always owns commutation scheduling from startup onward.
 *
 * When FEATURE_V4_SECTOR_PI=1, ALL V3 motor control code is gated off:
 * FEATURE_IC_ZC, FEATURE_CLC_BLANKING, FEATURE_IC_DMA_SHADOW, etc.
 * are ignored. Only V4 files compile. V3 code remains for fallback
 * (set FEATURE_V4_SECTOR_PI=0 to restore V3). */
/* Bring-up diagnostic: when set, the V4 ramp NEVER transitions to closed
 * loop. Motor stays in forced-commutation OL mode at MIN_STEP_PERIOD speed
 * with rampDuty = RAMP_DUTY_CAP. Lets the operator confirm the motor
 * physically rotates smoothly under pure 6-step commutation — if it does,
 * the bug is isolated to the BEMF/CL path. Revert to 0 for normal CL. */
#ifndef FEATURE_DEBUG_OL_ONLY
#define FEATURE_DEBUG_OL_ONLY   0
#endif

#ifndef FEATURE_V4_SECTOR_PI
#define FEATURE_V4_SECTOR_PI    1
#endif

/* AK port: CCP capture path is diag-only (motor runs on ADC-ISR
 * midpoint sampler). Default OFF — turn on once hal_capture.c is
 * re-mapped to AK SCCP3 (no SCCP5 on this MCU). */
#ifndef FEATURE_V4_CCP_DIAG
#define FEATURE_V4_CCP_DIAG     0
#endif

#if FEATURE_V4_SECTOR_PI

/* Motor phase advance (electrical degrees, 0..30).
 *
 * Default 15° = old TAL=2 value (used at low/mid speed in the original
 * piecewise scheduler).  This is conservative for startup since the
 * unified PI+scheduler now applies this value across all speeds.
 *
 * Bumped to 22.5° (= old TAL=3) briefly on 2026-04-28 to preserve
 * high-speed behavior, but that destabilized low-speed startup
 * (motor stuck around 28k eRPM, eTPm frozen, PI saturated).  Reverted
 * to 15° as safe starting point — tune up live via SET_PARAM 0xF0
 * once startup is stable:
 *   SET_PARAM 0xF0 200  → 20.0°  (compromise, recommended high-speed)
 *   SET_PARAM 0xF0 225  → 22.5°  (max benefit, may need bench validation)
 *
 * For motors that need more low-speed advance, drop further:
 *   SET_PARAM 0xF0 100  → 10.0°  (original Microchip A2207 default)
 *
 * Phase 3 PTG calibration sweep (2026-05-13):
 *   10.0f  →  194k peak, pR=85% at top
 *    5.0f  →  190k peak, pR=77% at top  (worse acceptance, slightly slower)
 *    7.5f  →  208k peak (was current)
 *   12.5f  →  ← TESTING (2026-05-15) — unexplored upward direction
 * The PTG path has lower latency than ADC ISR, so the bias-portion of this
 * constant should drop — but empirically 5° hurts more than 10°, suggesting
 * this value is partly real torque advance, not pure latency compensation. */
#define V4_PHASE_ADVANCE_DEG    12.5f

/* Board detector delay (µs). ATA6847 comparator propagation ~2 µs.
 * CK board has no RC filter on BEMF inputs (unlike Microchip MPPB
 * which has 10 µs RC delay). */
#define V4_RC_DELAY_US          2.0f

/* Pre-computed constants (compile-time, no float in hot path) */
#define V4_ADVANCE_PLUS_30_FP8  ((uint16_t)((V4_PHASE_ADVANCE_DEG + 30.0f) * 256.0f / 60.0f + 0.5f))
#define V4_RC_DELAY_HR          ((uint16_t)(V4_RC_DELAY_US * 1.5625f + 0.5f))

/* PI gains (bit shifts). Matches Microchip AVR motor.c:444,448. */
#define V4_KP_SHIFT             2       /* Kp = 1/4 */
#define V4_KI_SHIFT             4       /* Ki = 1/16 */

/* Startup + V4_MIN_AMPLITUDE + block-comm thresholds (per-profile).
 *
 * V4_MIN_AMPLITUDE_PROFILE  Q15 amplitude floor. Pot-at-zero target.
 *                           Must be high enough at the profile's Vbus to
 *                           push past the BEMF-blind low-speed regime
 *                           (~10k eRPM) so CL doesn't stall at idle.
 * V4_BLOCK_ENTER_ERPM       Block-comm engagement floor — should be just
 *                           below the duty-saturation eRPM at the
 *                           profile's Vbus (where motor naturally clips
 *                           the 96.9% duty cap).
 * V4_BLOCK_EXIT_ERPM        Re-entry hysteresis — typically 0.85× enter. */
#if MOTOR_PROFILE == 0   /* Hurst */
#define V4_STARTUP_SPEED_ERPM   3000UL
#define V4_STARTUP_CURRENT_MA   2000.0f
#define V4_ALIGN_DURATION_MS    200U
#define V4_MIN_AMPLITUDE_PROFILE 5000U     /* 15.3% Q15 — Hurst is high-Rs, idles fine */
#define V4_BLOCK_ENTER_ERPM     30000UL    /* Hurst peaks ~25-30k, mostly out of range */
#define V4_BLOCK_EXIT_ERPM      25000UL
#elif MOTOR_PROFILE == 1  /* A2212 @ 12V */
#define V4_STARTUP_SPEED_ERPM   500UL
#define V4_STARTUP_CURRENT_MA   3000.0f
#define V4_ALIGN_DURATION_MS    100U
#define V4_MIN_AMPLITUDE_PROFILE 5000U     /* 15.3% Q15 — bench-safe (12V/65mΩ).
                                            * Bumping to 30% on 2026-05-12 collapsed bench
                                            * supply to 5V (vbus 1857→807) — 12V × 30% duty
                                            * draws ~48A peak which exceeds typical bench
                                            * supply CC limit. Keep at 15% until validated
                                            * with proper power supply. */
#define V4_BLOCK_ENTER_ERPM     100000UL   /* A2212 @ 12V peaks ~123k */
#define V4_BLOCK_EXIT_ERPM      85000UL
#elif MOTOR_PROFILE == 2  /* 2810 @ 25V */
#define V4_STARTUP_SPEED_ERPM   500UL
#define V4_STARTUP_CURRENT_MA   3000.0f
#define V4_ALIGN_DURATION_MS    100U
#define V4_MIN_AMPLITUDE_PROFILE 5000U     /* 15.3% — proven on 2810 @ 25V */
#define V4_BLOCK_ENTER_ERPM     150000UL   /* 2810 @ 25V — matches CK GitHub
                                            * baseline (CK peaks 235k with this
                                            * threshold). 2026-05-15 reverted
                                            * from AK-specific 100k after the
                                            * PTG postscale experiment ruled out
                                            * sample rate as the gap source.
                                            * Testing whether the climb path
                                            * (PWM modulation up to 150k vs to
                                            * 100k on AK) accounts for the 9k
                                            * gap to CK. */
#define V4_BLOCK_EXIT_ERPM      130000UL   /* CK-matched hysteresis (~0.87×).
                                            * Narrower than AK's old 80k; the
                                            * wide-hysteresis rationale was
                                            * untested at 150k entry. */
#elif MOTOR_PROFILE == 3  /* HiZ1460 — Rs caps current at 1.87A */
#define V4_STARTUP_SPEED_ERPM   500UL
#define V4_STARTUP_CURRENT_MA   1500.0f
#define V4_ALIGN_DURATION_MS    200U       /* Matches profile 3 ALIGN_TIME_MS */
#define V4_MIN_AMPLITUDE_PROFILE 5000U     /* 15.3% Q15 — proven on colleague's bench
                                            * with the actual HiZ 16 Ω motor (commit
                                            * 1956441). 50% Q15 was over-tuned —
                                            * Rs limits current naturally so high
                                            * idle floor isn't needed and creates
                                            * a destabilizing duty-step at CL. */
#define V4_BLOCK_ENTER_ERPM     250000UL   /* HiZ1460 @ 30V theoretical 306k */
#define V4_BLOCK_EXIT_ERPM      220000UL
#endif

#define V4_STARTUP_TIME_MS      1000U   /* Forced ramp duration before commands accepted */
#define V4_STALL_THRESHOLD      200U    /* Consecutive no-capture sectors → stall.
                                         * At 3000 eRPM: 200 sectors ≈ 0.4s */
#define V4_MIN_PERIOD           10U     /* Timer period floor (~1.5M eRPM, safety).
                                         * Tested 10 → 60 on 2026-05-13: no effect
                                         * on top-end (motor never hits the clamp
                                         * during normal operation). Reverted. */

/* Sector timer clock: SCCP4 HR timer rate (640 ns/tick).
 * V4 algorithm — advance fractions, blanking, PI scaling — was tuned
 * against 1.5625 MHz HR clock on CK.
 *
 * Same rate on AK: SCCP `CLKSEL=000` selects Standard Speed Peripheral
 * Clock (FPB/2 = 100 MHz on AK; FPB = 100 MHz on CK so unchanged), and
 * `TMRPS=0b11` gives /64 → 1.5625 MHz on both MCUs. AK supports only
 * 1:1/1:4/1:16/1:64 prescalers (DS70005539 Table 24-2 §24.3) but the
 * /2 in the peripheral clock chain hides the FCY doubling. */
#define V4_TIMER_FREQ_HZ        1562500UL      /* 1.5625 MHz, 640 ns/tick */
#define V4_ERPM_TO_PERIOD(e)    (uint16_t)(60UL * V4_TIMER_FREQ_HZ / (6UL * (e)))

/* ISR priorities */
#define V4_SECTOR_ISR_PRIORITY  6       /* Commutation — highest motor ISR */
#define V4_CAPTURE_ISR_PRIORITY 5       /* ZC edge capture — below sector */

/* ZC detection method:
 * 0 = CCP edge capture + 3-read deglitch. Tested 2026-04-16: same 49%
 *     Cap% as Mode 1, but motor desynced at ~110k eRPM. The deglitch
 *     state-check convention in CCP2/CCP5 ISRs (garuda_service.c) is
 *     suspect — appears to be checking pre-edge state instead of
 *     post-edge state, which would mean the only Sets that fire are on
 *     comparator noise spikes rather than real ZCs.
 * 1 = PWM-midpoint sampling in ADC ISR. Reaches 196k eRPM stably with
 *     49% Cap% (rising sectors only — falling sectors past blanking
 *     never see a stable comp state, ATA6847 has no hysteresis and
 *     falling-sector BEMF hovers near neutral). PROVEN BASELINE.
 * 2 = Hybrid: ADC midpoint confirms ZC state, CCP provides accurate
 *     timestamp. Mask CCP after acceptance like AM32. Untested in V4. */
#define FEATURE_V4_MIDPOINT_ZC  1

/* ── V5 Symmetric Sensing (architecture gate, default off) ────────
 * V4 hits a wall because only rising sectors produce real captures.
 * V5 is the symmetric-sensing rewrite: both polarities feed the PI
 * with matching signal quality.
 *
 * FEATURE_V5_SYMMETRIC_SENSING=0 → byte-identical to V4 baseline.
 * FEATURE_V5_SYMMETRIC_SENSING=1 → V5 code paths active.
 *
 * V5.0 attempt 1 (2026-04-18): SCCP1 priority 2 → 5. Hypothesis was
 * that the OFF-mid diagnostic ISR was starved by CCP storm during
 * falling sectors. Bench result: NO measurable improvement in
 * offMidCapture / offMidMismatch counters between priority 2 and 5.
 * The bottleneck isn't CPU budget — SCCP1 was already firing ~10 kHz
 * at priority 2. The real issue is SCCP1's 24.96 µs clock aliasing
 * against the commutation cadence such that fires never land in
 * sector windows where currentRisingZc == false. Priority can't fix
 * that. The V5_SCCP1_ISR_PRIORITY knob is kept for reference only.
 *
 * V5.0 attempt 2 (active direction): PTG-based BEMF sampling. A
 * core-independent state machine fires ADC/GPIO samples at
 * programmable delays from the PWM trigger, one offset per sector
 * polarity, with hardware-exact timing and zero CPU jitter. This
 * is the right tool for per-sector sample timing — the sampling
 * problem can't be solved in software polling. */
#ifndef FEATURE_V5_SYMMETRIC_SENSING
#define FEATURE_V5_SYMMETRIC_SENSING  0
#endif

#if FEATURE_V5_SYMMETRIC_SENSING
#define V5_SCCP1_ISR_PRIORITY   5  /* archived: tied with CCP, no effect */
#else
#define V5_SCCP1_ISR_PRIORITY   2  /* V4 baseline: below ADC */
#endif

/* ── V5.0-PTG: Peripheral Trigger Generator BEMF sampling ────────
 * Core-independent state machine triggers per-sector BEMF reads at
 * hardware-exact offsets from the PWM valley event. This is the real
 * V5.0 approach — the SCCP1 priority experiment is archived.
 *
 * PTG command queue (FEATURE_V5_PTG_ZC=1):
 *   STEP0: PTGWHI | 0x1         wait for PWM trigger input
 *   STEP1: PTGCTRL| 0x8         start PTGT0, wait for timeout
 *   STEP2: PTGIRQ | 0x0         generate PTG0 interrupt (_PTG0Interrupt)
 *   STEP3: PTGJMP | 0x0         loop back to STEP0
 *
 * PTG clock = FCY / PTGDIV. On AK FCY=200 MHz so PTGDIV=0 → 200 MHz,
 * 5 ns/tick (was 10 ns/tick on CK). PTG_PEAK_DELAY uses LOOPTIME_TCY
 * which is in PWM-counter ticks (2.5 ns), so PTG ticks at 5 ns map to
 * LOOPTIME_TCY/2 not LOOPTIME_TCY/4 like on CK — update V5_PTG_PEAK_DELAY
 * before turning FEATURE_V5_PTG_ZC on. (PTG diag-only, off by default.)
 * PTGT0LIM sets the delay-from-PWM-trigger in PTG ticks.
 *
 * Per-sector sample offset (rewritten by Commutate ISR):
 *   rising sector → PTGT0LIM = 0         (sample at valley, ~existing behavior)
 *   falling sector → PTGT0LIM = MPER/2*N (sample at PWM peak / OFF-mid)
 *
 * When FEATURE_V5_PTG_ZC=0: byte-identical to V4 baseline.
 * When =1: PTG init runs at motor start, ISR counts samples (shadow
 *          only in V5.0-step1 — does NOT set v4_captureValid). */
#ifndef FEATURE_V5_PTG_ZC
#define FEATURE_V5_PTG_ZC  1   /* AK PTG phase-1 heartbeat (2026-05-13).
                                * hal_ptg.c provides a real implementation
                                * for the dsPIC33AK128MC106 — PTG fires
                                * _PTG0Interrupt at every PG1TRIGB match
                                * (mid-OFF), ISR increments v5_ptgFires.
                                * Flag flipped on so the existing
                                * snapshot path at sector_pi.c:1347
                                * exposes the heartbeat count to the host.
                                * NOTE: this does NOT compile any V5 BEMF
                                * logic on AK — there's no equivalent of
                                * the CK V5 ISR.  Phase 2 will add real
                                * BEMF + PI handling inside
                                * _PTG0Interrupt. */
#endif

/* PTG-driven BEMF detection master flag.
 *   0 — ADC ISR reads BEMF GPIO + runs V4 ZC detection
 *       (original/CK-style path; Phase 1 default).
 *   1 — PTG ISR reads BEMF + runs V4 ZC detection; ADC ISR keeps
 *       current/Vbus only (Phase 2).  Latency to BEMF read drops
 *       from ~1.5 µs to ~50 ns.
 *
 * If enabling Phase 2 and the motor walls below the previous milestone,
 * try LOWERING V4_PHASE_ADVANCE_DEG (currently 10°) — that constant
 * compensates for the old ADC-ISR latency and is now too large for the
 * PTG path.  Sweep 10 → 5 → 2 → 1 to find the new optimum. */
#ifndef FEATURE_BEMF_VIA_PTG
#define FEATURE_BEMF_VIA_PTG    1
#endif

/* B1 mid-ON diagnostic probe (2026-05-13).
 * When enabled, V4_ProcessBemfSample busy-waits ~half a PWM period after
 * its normal mid-OFF read, then takes a second comp read at mid-ON and
 * classifies it for FALLING sectors only. Counters expose via the
 * snapshot pF% slot (replaces post-ZC shadow when enabled).
 *
 * Tests whether mid-ON has a cleaner falling-sector BEMF signal than
 * mid-OFF. If pF% jumps from ~12% (current mid-OFF) to >70%, then a
 * proper PTG dual-trigger architecture (option A) is justified.
 *
 * COST: ~8µs busy-wait inside PTG ISR at 60kHz (~50% of PWM period
 * burned). Top-end performance will tank during the test. Run a low-
 * speed (60-80k) sweep with the prop off to read pF%. DISABLE for
 * normal operation by setting to 0. */
/* B1 busy-wait probe — rejected 2026-05-13. The 8µs delay-after-PG1TRIGB
 * sample didn't actually land at mid-ON; PG1TRIGB fires at counter==duty
 * (duty-edge), and 8µs later put us at the next switching edge or in
 * deep OFF region. Not a fair test of mid-ON. Disabled. */
#ifndef FEATURE_MIDON_DIAG_PROBE
#define FEATURE_MIDON_DIAG_PROBE   0
#endif

/* B2 dual-position probe (2026-05-13).
 * Alternates PG1TRIGB between MID-OFF (counter=1, just past period
 * boundary) and MID-ON (counter=LOOPTIME_TCY-1, just before peak) on
 * each PTG ISR fire. This puts the diagnostic sample at the actual
 * peak of the ON pulse — independent of duty cycle.
 *
 * PTG fires alternate at MID-OFF and MID-ON positions. ISR uses a
 * toggle state to know which position the current sample is at. Mid-OFF
 * fires run normal V4_ProcessBemfSample (feeds the PI). Mid-ON fires
 * only increment the diagnostic counters — no PI feed, no PWM-cycle
 * dependent behavior change.
 *
 * Expected ISR rate: same as current (one fire per PWM cycle) if
 * PG1TRIGB shadow-updates on period boundary, OR double rate if
 * immediate-update. Both are fine for diagnosis.
 *
 * Compare pF% (= midOn falling acc from FEATURE_MIDON_DIAG_PROBE
 * snapshot wiring) against pre-experiment pF% (~5% at midOff via
 * V5_POST_ZC_ACCEPT shadow). Big jump = mid-ON is the right position. */
#ifndef FEATURE_DUAL_POS_PROBE
#define FEATURE_DUAL_POS_PROBE     0   /* superseded by FEATURE_PER_SECTOR_PTG */
#endif

#define PTG_TRIG_MID_OFF_POS       1U
#define PTG_TRIG_MID_ON_POS        (LOOPTIME_TCY - 1U)

/* Duty-adaptive sample position (2026-05-15):
 * In center-aligned PWM, the OFF window width is (1-duty)*period and the
 * ON window is duty*period. The two are equal at duty=50%; below that the
 * OFF window is wider, above it the ON window is wider. Sample in
 * whichever is wider so the read point stays maximally far from the
 * switching edges and the 3-read deglitch doesn't straddle a transition.
 *
 * Bench evidence (2026-05-15 MID-OFF run):
 *   - Low-speed currents at MID-OFF were ~5A vs ~12A at MID-ON (idle).
 *   - pR/pF balance ~40/60 at MID-OFF vs 0-1/99 at MID-ON.
 *   - Above ~80% duty, MID-OFF desyncs (OFF window <20% of cycle, sample
 *     lands too near switching edge).
 *
 * 50% threshold with small hysteresis (avoid chatter at the boundary). */
#ifndef PTG_DUTY_ADAPT_THRESHOLD_PCT
#define PTG_DUTY_ADAPT_THRESHOLD_PCT   50U
#endif
#define PTG_DUTY_ADAPT_THRESHOLD \
    ((uint16_t)(((uint32_t)LOOPTIME_TCY * PTG_DUTY_ADAPT_THRESHOLD_PCT) / 100U))
/* Hysteresis: ±5% around threshold to avoid sample-position chatter when
 * commanded duty hovers at exactly 50% (PI hunting in the boundary band). */
#define PTG_DUTY_ADAPT_HYST \
    ((uint16_t)(((uint32_t)LOOPTIME_TCY * 5U) / 100U))

/* Phase 1 — Per-sector PTG trigger position (2026-05-13).
 * B2 result: mid-OFF works for rising sectors (pR=95%); mid-ON has real
 * signal for falling sectors (pF=47% raw, expected ~85% after blanking).
 *
 * Per-sector adaptive sampling: Commutate ISR writes PG1TRIGB based on
 * the next sector's polarity. Rising sectors fire PTG at mid-OFF (works
 * with the proven V4 detection path). Falling sectors fire PTG at mid-ON
 * (new path — falling-sector BEMF is readable there but not at mid-OFF).
 *
 * Phase 1 keeps piFeedPolarity=1 (rising-only PI feed) — observation
 * mode. Watch pF via the diagnostic; should climb from 47% toward >80%
 * once blanking gates the pre-ZC half of falling-sector samples. */
/* Phase 1 fix-pass 2026-05-13:
 *  (a) Commutate writes PG1TRIGB + sets PG1STATbits.UPDREQ → buffer
 *      transfers to active at next SOC (within ~1 PWM cycle), so the
 *      per-sector position takes effect before the sector ends even
 *      at 200k eRPM (3 PWM cycles per sector).
 *  (b) Legacy V4 detection polarity gate now reads v5_ptgExpectedComp
 *      instead of HAL_Capture_IsRisingZc() (stuck-true). With
 *      piFeedPolarity=1 falling sectors stay out of PI until we
 *      explicitly flip to piFeedPolarity=0 in Phase 2. */
#ifndef FEATURE_PER_SECTOR_PTG
#define FEATURE_PER_SECTOR_PTG     1
#endif

/* PTG ISR postscaler (2026-05-15 CK-rate experiment).
 * AK currently runs PTG at 60 kHz (every PWM cycle) while CK runs its
 * ADC ISR at ~15-20 kHz via PG1EVTL.ADTR1PS=0b00011 (1:4). CK peaks
 * 235k eRPM with that low rate; AK peaks 226k with 3-4x more data.
 * Hypothesis: extra samples either feed noise into the PI integrator,
 * or shift the elapsed-to-match distribution in a way the advance
 * constant can't fully absorb.
 *
 * N=1: every fire processed (current 60 kHz behaviour)
 * N=3: 1-in-3 fires processed → 20 kHz BEMF (closest to CK's
 *      source-comment claim)
 * N=4: 1-in-4 fires processed → 15 kHz BEMF (matches CK's actual
 *      postscaler math)
 *
 * Skipped ISR fires still execute the PG1TRIGB write so the trigger
 * stays armed — only V4_ProcessBemfSample() is gated. v5_ptgSkipped
 * counts gated fires for telemetry. */
#ifndef PTG_POSTSCALE_N
#define PTG_POSTSCALE_N            1U   /* 2026-05-15: tested N=3 (20kHz)
                                          * → peak 172k vs 226k baseline.
                                          * Lower rate actively hurts AK.
                                          * Restored to 1 (60kHz). */
#endif

/* Spin-loop iteration count to delay ~half a PWM period.
 * FCY=200MHz (5 ns/cycle). Volatile-counter loop ~6 cyc/iter on XC-DSC.
 * 60kHz PWM half-period = 8.33µs = 1666 cycles / 6 ≈ 278 iter. */
#ifndef MIDON_DIAG_LOOPS
#define MIDON_DIAG_LOOPS           280U
#endif

/* PTG ISR priority — above ADC(3), tied with Timer1(4), below CCP(5).
 * Started at 5 (tied with CCP) but bench showed peak speed dropped to
 * ~62k from V4's 107k — the PTG-CCP same-level queue was slowing CCP
 * servicing under high-speed comparator chatter. Priority 4 lets CCP
 * preempt PTG so CCP processing isn't held up; PTG still preempts ADC. */
#define V5_PTG_ISR_PRIORITY  4

/* ── V5.1 Symmetric ADC accept (post-ZC convention) ──────────────
 * The V4 ADC ISR uses `expected = isRising ? 1 : 0` — this is the
 * pre-ZC state for both polarities (rising sector pre-ZC has comp=1,
 * falling sector pre-ZC has comp=0 on the inverted ATA6847 comp).
 * V4 only ever accepts captures in rising sectors because the
 * pre-ZC sample for falling sectors lands too late within the sector
 * and gets rejected by the half-period filter in Commutate.
 *
 * V5.1 flips the convention to post-ZC sampling: accept when
 *   rising  sector → comp == 0
 *   falling sector → comp == 1
 * The PTG diagnostic data showed both polarities have ~67% post-ZC
 * accept rate at PWM valley sampling — strong, observable signal on
 * both. With the convention flipped, the ADC ISR should accept on
 * both polarities and feed the PI ~2× more captures, hopefully
 * collapsing the 2× equilibrium offset (eTP ≈ eRpm/2) to truth.
 *
 * V5.1-step1 (FEATURE_V5_POST_ZC_ACCEPT=1, FEATURE_V5_POST_ZC_OWN=0):
 *   Shadow only — V4 convention still drives v4_captureValid; new
 *   counters report what post-ZC accept would do.
 * V5.1-step2 (FEATURE_V5_POST_ZC_OWN=1):
 *   Post-ZC accept actually drives v4_captureValid; PI sees both
 *   polarities. Bench-validate PI stability before this. */
/* 2026-04-29 — Post-ZC ADC accept enabled.  Capture log on prop startup
 * showed the V4 pre-ZC convention feeding bimodal capValues (cluster A
 * around real ZC ~50% T, cluster B around first post-blanking sample
 * where pre-ZC state still held) → 2-step PI limit cycle → death
 * spiral. Post-ZC convention reads v5_ptgExpectedComp written cleanly
 * by Commutate, bypassing the stuck-TRUE HAL_Capture_IsRisingZc()
 * issue. Both polarities should now contribute single-cluster captures.
 * Verify high-speed (196k) baseline isn't regressed before declaring
 * this stable. */
#ifndef FEATURE_V5_POST_ZC_ACCEPT
#define FEATURE_V5_POST_ZC_ACCEPT  1
#endif

#ifndef FEATURE_V5_POST_ZC_OWN
#define FEATURE_V5_POST_ZC_OWN     0   /* 2026-05-14 reverted to V4 PRE-ZC after
                                        * git diff revealed local CK V5_OWN
                                        * changes are EXPERIMENTAL (commented
                                        * "Verify high-speed baseline isn't
                                        * regressed before declaring stable").
                                        * GitHub CK working at 235k uses
                                        * V5_POST_ZC_OWN=0 (V4 PRE-ZC). AK V5
                                        * without edge gate causes cR cascade.
                                        * Stay on legacy V4 path here too. */
#endif

#if FEATURE_V5_POST_ZC_OWN && !FEATURE_V5_POST_ZC_ACCEPT
#error "FEATURE_V5_POST_ZC_OWN requires FEATURE_V5_POST_ZC_ACCEPT"
#endif

/* ── V5.2 Measurement-based PI ─────────────────────────────────
 * V4's set-point PI converges to a wrong equilibrium (eTP ≈ eRpm/2)
 * and collapses to its floor when capture timing semantics change
 * (e.g., PTG sampling, hardware edge captures). Root cause: the
 * setValue formula models a specific capture convention (first
 * pre-ZC sample after blanking), so any sensor that delivers data
 * with different statistics breaks the equilibrium search.
 *
 * V5.2 fix: replace `timerPeriod ← integrator + delta/4` with a
 * simple exponential smoother on the measured commutation interval:
 *   v5_tMeasHR += (actualStepPeriodHR - v5_tMeasHR) >> alpha_shift
 *
 * This tracks T_real directly and doesn't care about the capture
 * sample moment. The reactive scheduler still uses capture
 * timestamps for *when* to commutate; the period estimate becomes
 * truly independent of what the ZC sensor delivers.
 *
 * V5.2-step1 (FEATURE_V5_MEAS_PI=1, FEATURE_V5_MEAS_PI_OWN=0):
 *   Shadow only — v5_tMeasHR updates in parallel with the set-point
 *   PI, exposed via telemetry (eTPm column). Compare eTPm to eRpm:
 *   if eTPm tracks eRpm while eTP (set-point PI) shows half, the
 *   measurement tracker is correct.
 * V5.2-step2 (FEATURE_V5_MEAS_PI_OWN=1):
 *   v5_tMeasHR actually drives timerPeriod. Set-point PI bypassed.
 *   Reactive scheduling and everything downstream reads the new
 *   value. With OWN=1 + PTG, we finally get symmetric captures on a
 *   PI that tracks truth.
 *
 * Alpha shift 2 (= 1/4) mirrors V4's Ki choice. Higher shifts
 * smooth more (slower response to speed changes); lower shifts
 * track tighter but are noisier. */
#ifndef FEATURE_V5_MEAS_PI
#define FEATURE_V5_MEAS_PI  1
#endif

#ifndef FEATURE_V5_MEAS_PI_OWN
#define FEATURE_V5_MEAS_PI_OWN  0   /* requires FEATURE_V5_MEAS_PI */
#endif

#if FEATURE_V5_MEAS_PI_OWN && !FEATURE_V5_MEAS_PI
#error "FEATURE_V5_MEAS_PI_OWN requires FEATURE_V5_MEAS_PI"
#endif

#ifndef V5_MEAS_PI_ALPHA_SHIFT
#define V5_MEAS_PI_ALPHA_SHIFT  2    /* α = 1/4, matches V4 Ki shift */
#endif

/* ── V5.3 scheduler rewrite (AM32-style) ────────────────────────
 * Root cause of everything V5.0/V5.1/V5.2 hit: the V4 set-point PI
 * scheduler fires Commutates in PAIRS — one reactive (ASAP, because
 * lastCaptureHR + delayHR is always slightly past) and one fallback
 * (2T later). Over each 2T wallclock, it runs 2 Commutates and
 * advances position by 2. Software state isn't 1:1 with the
 * physical rotor sector, which breaks any per-sector reasoning
 * (eTP=eRpm/2, PTG bucket skew, can't use captures on falling
 * sectors).
 *
 * V5.3 architecture:
 *   - One Commutate per physical sector (6 per elec rev).
 *   - Commutate fires at lastCaptureHR + (T_sector/2 - advance).
 *   - T_sector = thisCaptureHR - prevCaptureHR (capture-to-capture,
 *     one sector apart — not 2x like V4's measuredCommPeriod).
 *   - Both CCP2 (rising) and CCP5 (falling) feed captures. PTG
 *     provides the back-up polarity discriminator.
 *   - position advances once per Commutate, tracking physical
 *     rotor sectors 1:1.
 *   - No ASAP pair firing: scheduler guarantees target is always
 *     at least N HR in the future before writing CCP3PRL.
 *
 * Flag-gated so V4 stays the fallback. V5_SCHEDULER=0 → V4 path
 * (current working 109k baseline). V5_SCHEDULER=1 → V5.3 path. */
#ifndef FEATURE_V5_SCHEDULER
#define FEATURE_V5_SCHEDULER  0   /* WIP — motor runs chaotically, needs deeper
                                   * rework. V4 scheduler (flag=0) is the proven
                                   * 109k baseline. V5.3 implementation lives in
                                   * sector_pi.c:CommutateV5_3 + garuda_service.c
                                   * :V5_AcceptCapture — all gated by this flag. */
#endif

#if FEATURE_V5_SCHEDULER && !FEATURE_V4_SECTOR_PI
#error "FEATURE_V5_SCHEDULER builds on top of V4 scaffolding — keep V4 flag on"
#endif

/* Default PTGT0LIM values. [AK PORT] At FCY=200 MHz with PTGDIV=0,
 * 1 PTG tick = 5 ns (was 10 ns on CK).
 * LOOPTIME_TCY is in PWM counter ticks (5 ns each on this CK device —
 * FOSC_PWM 400 MHz with internal /2 divider on the PWM counter).
 *
 *   Valley sample: PWM trigger itself fires at counter=0. A tiny delay
 *     covers ATA6847 comparator propagation (~500 ns) and signal settle.
 *   Peak sample:  Counter triangle 0 → MPER → 0 each PWM period.
 *     Valley to peak = MPER/2 PWM-cycle time.
 *     At 40 kHz, MPER=4999 → ~25 µs cycle → ~12.5 µs valley→peak.
 *     12.5 µs = 1250 PTG ticks. MPER * 5 ns / 2 / 10 ns = MPER/4.
 */
#define V5_PTG_TICK_NS          10u
#define V5_PTG_VALLEY_DELAY     60u        /* 600 ns settle after trigger */
#define V5_PTG_PEAK_DELAY       ((uint16_t)(LOOPTIME_TCY / 4u))

#endif /* FEATURE_V4_SECTOR_PI */

#if !FEATURE_V4_SECTOR_PI
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

#endif /* !FEATURE_V4_SECTOR_PI — end of V3-only config */

/* ── ARM ───────────────────────────────────────────────────────────── */
#define ARM_TIME_MS         200U
#define ARM_TIME_COUNTS     ((uint16_t)((uint32_t)ARM_TIME_MS * TIMER1_FREQ_HZ / 1000))

/* ── ADC Channel Mapping — EV92R69A + EV68M17A (verified) ─────────── */
/* Verified against DS70005527 (EV68M17A DIM Info Sheet) §5 pin-map table:
 *   DIM 28 → AD1AN10 / RP12 / RA11  → Potentiometer
 *   DIM 39 → AD1AN6  / RP8  / RA7   → DC bus voltage
 * Both wired to ADC1. AK ADC is per-channel, not per-buffer:
 *   POT  on AD1CH1 (PINSEL=10) → read AD1CH1DATA
 *   VBUS on AD1CH4 (PINSEL=6)  → read AD1CH4DATA
 * Macros below preserve the CK API name so `garuda_service.c` reads
 * still look like `uint16_t v = ADCBUF_POT;`. */
#define ADCBUF_POT          AD1CH1DATA   /* AD1CH1.PINSEL=10 → AN10/RA11 */
#define ADCBUF_VBUS         AD1CH4DATA   /* AD1CH4.PINSEL=6  → AN6/RA7  */

/* Phase + DC bus currents — through the dsPIC's internal op-amps OA1/OA2/OA3
 * which amplify the raw shunt voltages routed in on DIM 13/15, 21/23 and
 * 29/31 (AN6285 §3 + DS70005527 §2).  J5/J7/J9 on the EV92R69A must be in
 * BEMF position (default) so the ATA6847L's BEMF comparators stay live;
 * that frees us to use the dsPIC op-amps for current sense.  Raw ADC is
 * unsigned 12-bit, zero current = ~2048; the ADC ISR subtracts the bias
 * to land a signed int16 in gV4IaRaw / gV4IbRaw / gV4IbusRaw. */
#define ADCBUF_IA           AD1CH0DATA   /* AD1CH0.PINSEL=0 → OA1OUT/AN0/RA2 */
#define ADCBUF_IB           AD2CH0DATA   /* AD2CH0.PINSEL=1 → OA2OUT/AN1/RB0 */
#define ADCBUF_IBUS         AD1CH2DATA   /* AD1CH2.PINSEL=3 → OA3OUT/AN3/RA5 */
#define ADC_CURRENT_BIAS    2048         /* mid-scale of 12-bit ADC = zero current */

/* ── Calibration: convert raw ADC counts to physical units ─────────────
 *
 * Current (Ia / Ib / Ibus):
 *   shunt R   = 3 mΩ      (EV92R69A BOM: RS1/RS2/RS3 = WSLF25123L000FEA)
 *   amp G     = 24.95×    (AK DIM differential gain: R_F/(R_IN1+R_IN2)
 *                          = 4.99 kΩ / 200 Ω, DS70005527 Table 2-1)
 *   ADC slope = 4096 / 3.3 V = 1241.21 counts/V
 *   → counts per amp = G × R × slope = 24.95 × 0.003 × 1241.21 ≈ 92.89
 *   → milliamps per count = 1000 / 92.89 ≈ 10.7654
 *   Q8 fixed-point: 10.7654 × 256 = 2756 (fits int16).
 *   Saturation check: count = ±2048 → mA = (2048 × 2756) >> 8 = ±22055 mA.
 *
 * Vbus:
 *   divider   = 16:1 (R47 36 kΩ / R49 2.4 kΩ, AN6285 Table 5-2)
 *   → millivolts per count = 3.3 V × 16 × 1000 / 4096 = 12.8906
 *   Q8 fixed-point: 12.8906 × 256 = 3300 (rounded).
 *   Saturation check: count = 4095 → mV = (4095 × 3300) >> 8 = 52777 mV.
 *
 * ── CURRENT CONFIG (NO PROP / NO-LOAD BENCH) ──────────────────────────
 * Stock AK DIM, R_F = 4.99 kΩ, G = 24.95×, range = ±22 A peak.  This is
 * adequate for no-load testing (observed peaks 18-22 A at 200k eRPM on
 * the 2810 motor).  WILL SATURATE under prop load.
 *
 * ── BEFORE PROP TESTING — DIM REWORK REQUIRED ─────────────────────────
 * The AK DIM (EV68M17A) has six 4.99 kΩ feedback resistors that set the
 * differential gain.  To raise the current measurement ceiling:
 *
 *   Replace ALL SIX (0603, 0.1% thin-film, matched reel — CMRR matters):
 *     OA1 (Ia):   R5,  R18
 *     OA2 (Ib):   R24, R31
 *     OA3 (Ibus): R10, R21
 *
 *   Target peak | new R_F | new G    | ADC_MA_PER_COUNT_Q8
 *   ±44 A       | 2.49 kΩ | 12.45×   |  5520
 *   ±55 A       | 2.00 kΩ | 10.00×   |  6872   (recommended for drone prop)
 *   ±73 A       | 1.50 kΩ |  7.50×   |  9162
 *   ±110 A      | 1.00 kΩ |  5.00×   | 13743
 *
 *   After rework: change ADC_MA_PER_COUNT_Q8 below to the matching value,
 *   rebuild, reflash.  NO OTHER FIRMWARE CHANGE NEEDED — the snapshot,
 *   peak tracker, and Python tool all read the calibrated globals.
 *
 *   Recommended part for ±55 A target: Panasonic ERA-3AEB2001V × 6
 *   (0.1% thin-film, AEC-Q200, same series as the parts being replaced).
 */
#define ADC_MA_PER_COUNT_Q8       2756U   /* G=24.95× (stock, ±22 A peak) */
#define ADC_VBUS_MV_PER_COUNT_Q8  3300U   /* 12.8906 mV/count × 256       */

#endif /* GARUDA_CONFIG_H */
