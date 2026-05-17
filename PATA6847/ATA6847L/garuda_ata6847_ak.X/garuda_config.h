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

#define MAX_DUTY            (LOOPTIME_TCY - 200U)  /* ~100% complementary */
#define MIN_DUTY            200U
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
#define CL_IDLE_DUTY_PERCENT 10U   /* Complementary: braking needs higher idle */
#endif
#define CL_IDLE_DUTY         (((uint32_t)CL_IDLE_DUTY_PERCENT * LOOPTIME_TCY) / 100)

/* ── Universal defaults (apply to ALL motors) ─────────────────────── */

/* AM32-style timing advance: fixed fraction of commutation interval.
 * advance = (interval/8) * level → 7.5° per level.
 * Level 2 = 15° works for all motors tested (Hurst, A2212, 2810).
 * Per-profile override possible but not needed. */
#ifndef TIMING_ADVANCE_LEVEL
#define TIMING_ADVANCE_LEVEL  2U
#endif

/* Duty governor: limits max duty based on measured eRPM.
 * Prevents fast-pot desync. Full duty at threshold. */
#ifndef DUTY_RAMP_ERPM
#define DUTY_RAMP_ERPM  60000U
#endif
#ifndef MIN_TARGET_ERPM
#define MIN_TARGET_ERPM   1000U
#endif
#ifndef MAX_TARGET_ERPM
#define MAX_TARGET_ERPM 100000U
#endif


#ifndef FEATURE_GSP
#define FEATURE_GSP             1   /* 1 = GSP binary protocol on UART1 (disables debug prints)
                                     * 0 = debug UART text output (default for development)
                                     * Set to 1 when using GUI or gsp_ck_test.py */
#endif

/* ── V4 Sector PI Architecture ────────────────────────────────────────
 * Ground-up rewrite modeled on Microchip AVR high-speed motor control.
 * SCCP3 = sector timer (periodic, fires commutation ISR).
 * SCCP4 = HR free-running timer (640 ns/tick) for timestamps.
 * PI always owns commutation scheduling from startup onward.
 * BEMF detection runs in the PTG ISR (mid-OFF / mid-ON per duty). */

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

/* ZC detection: PWM-midpoint sampling. PTG ISR runs V4_ProcessBemfSample,
 * reads the floating-phase comparator, classifies pre/post-ZC and feeds
 * the sector PI. Reaches 226k eRPM stably on 2810 @ 25V (rising sectors
 * contribute the captures, falling sectors return CAP_SENTINEL — ATA6847
 * has no hysteresis so falling-sector BEMF hovers near neutral). */

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

/* PTG ISR postscaler. N=1 (every fire processed) is the validated default;
 * dropping to N=3 (20 kHz BEMF) regressed peak to 172k vs 226k baseline. */
#ifndef PTG_POSTSCALE_N
#define PTG_POSTSCALE_N            1U
#endif

/* PTG ISR priority — above ADC(3), tied with Timer1(4), below CCP(5).
 * Started at 5 (tied with CCP) but bench showed peak speed dropped to
 * ~62k from V4's 107k — the PTG-CCP same-level queue was slowing CCP
 * servicing under high-speed comparator chatter. Priority 4 lets CCP
 * preempt PTG so CCP processing isn't held up; PTG still preempts ADC. */
#define V5_PTG_ISR_PRIORITY  4

/* Post-ZC shadow counters (rising/falling Acc/Rej) populated by
 * V4_ProcessBemfSample. Reported via GSP snapshot — diagnostic only,
 * does not drive motor control. */
#ifndef FEATURE_V5_POST_ZC_ACCEPT
#define FEATURE_V5_POST_ZC_ACCEPT  1
#endif

/* Measurement-domain shadow PI: smoothed period tracker v5_tMeasHRSmooth
 * computed in parallel with the set-point PI. Telemetry-only — exposed as
 * the eTPm column. α = 1/4 (matches set-point Ki). */
#ifndef FEATURE_V5_MEAS_PI
#define FEATURE_V5_MEAS_PI  1
#endif
#ifndef V5_MEAS_PI_ALPHA_SHIFT
#define V5_MEAS_PI_ALPHA_SHIFT  2
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
