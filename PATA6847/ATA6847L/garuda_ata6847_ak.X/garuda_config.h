/**
 * @file garuda_config.h
 * @brief Configuration for 6-step BLDC on dsPIC33AK + ATA6847L.
 *
 * Target: EV92R69A carrier (ATA6847L QFN48) + EV68M17A DIM (dsPIC33AK128MC106).
 *
 * Motor profiles:
 *   0 = Hurst DMB2424B10002 (5PP, 24V, 149KV, low-speed bench motor)
 *   1 = A2212 1400KV        (7PP, 12V, 1400KV, drone motor)
 *   2 = 2810 1350KV         (7PP, 24V, drone motor — 226k eRPM peak)
 *   3 = HiZ1460             (7PP, 30V, high-Z bench)
 */

#ifndef GARUDA_CONFIG_H
#define GARUDA_CONFIG_H

/* ── Mode select (needs to come BEFORE the PWM block, since
 *      PWMFREQUENCY_HZ is mode-dependent) ───────────────────────── */
#ifndef FEATURE_FOC_AN1078
#define FEATURE_FOC_AN1078  1   /* 2026-05-22: FOC mode for ESMO bench testing.
                                   Flip back to 0 to return to 6-step. */
#endif

/* ── 6-step BEMF detection method ──────────────────────────────────
 * 0 = PTG-triggered level sample (current, proven 225 k eRPM)
 * 1 = Interrupt-on-Change edge capture (experimental — see
 *     docs/ioc_bemf_detection_plan.md)
 *
 * IOC path is edge-true (no PWM-rate latency), uses two CN interrupt
 * vectors (Port A for BEMF_C/RA10, Port B for BEMF_A/RB9 + BEMF_B/RB8).
 * Per sector only ONE pin is armed; the other port's interrupt enable
 * is masked. Five-layer rejection: demag interval gate, static
 * blanking, 3-read consensus, polarity match, speed-adaptive widths.
 *
 * Only meaningful when FEATURE_FOC_AN1078=0 (FOC uses currents not
 * BEMF). Mutually exclusive with the PTG path. */
#ifndef FEATURE_IOC_BEMF
#define FEATURE_IOC_BEMF    0   /* 2026-05-22: disabled — FOC mode active.
                                   Flip back to 1 when returning to 6-step. */
#endif
#if FEATURE_IOC_BEMF && FEATURE_FOC_AN1078
#error "FEATURE_IOC_BEMF is for 6-step path only — disable FEATURE_FOC_AN1078"
#endif

/* FOC ESMO (Enhanced SMO): TI-C2000-style observer drop-in replacement
 * for AN1078 SMO. Adds adaptive Kp PLL, adaptive Kslide, closed-form
 * theta-offset (atan2(speedRef·offsetSF, Kslf)), Ed/|E| discriminator
 * with full-quadrant pull-in, and speed LPF. Default OFF — flip to 1
 * to compile in the ESMO path; AN_SMC functions still callable for
 * fallback. Mutually selected with AN1078 via OBS_* macros in
 * an1078_motor.c. See docs/ti_esmo_inventory_2026_05_22.md. */
#ifndef FEATURE_FOC_ESMO
#define FEATURE_FOC_ESMO    1   /* 2026-05-22: ESMO observer default ON. Bench-
                                   tuned config: Ed/|E| PLL + adaptive Kp (3×) +
                                   speed LPF. Side-by-side vs AN1078 baseline:
                                   top-end Id range ±13 (vs ±24), ω jitter 3× lower.
                                   Peak ~210k eRPM (within 2% of baseline 213k).
                                   Set to 0 to revert to AN1078 SMO baseline. */
#endif
#if FEATURE_FOC_ESMO && !FEATURE_FOC_AN1078
#error "FEATURE_FOC_ESMO requires FEATURE_FOC_AN1078=1 (ESMO is an observer alternative)"
#endif

/* FWC (Field Weakening Control): voltage-margin PI on stator current
 * angle.  Port of TI C2000 motor-control-sdk libraries/control/fwc/.
 * Default OFF until bench-validated.  Set to 1 to compile foc/fwc.{c,h}
 * and enable the FW path in an1078_motor.c.  See foc/fwc.h. */
#ifndef FEATURE_FWC
#define FEATURE_FWC         1   /* 2026-05-22: bench test of voltage-margin
                                   angle-PI FW.  Set to 0 for legacy id_ref_fw
                                   integrator (proven to ~210k but with Id
                                   excursions ±20A above 175k). */
#endif
#if FEATURE_FWC && !FEATURE_FOC_AN1078
#error "FEATURE_FWC requires FEATURE_FOC_AN1078=1 (FWC is for FOC only)"
#endif

/* FW depth — maximum field-weakening angle in radians.
 *   1.05f (60°)  → deep FW; Id can pull to -0.87·|Is_ref|, Iq scaled by 0.5.
 *   0.785f (45°) → moderate FW; Id ≤ -0.71·|Is_ref|.
 *   0.0f         → FW DISABLED.  Id_ref always 0, motor capped at the
 *                  natural base speed (Vbus/√3·frac / λ).
 * Effective only when FEATURE_FWC=1.  At runtime FWC's PI clamps to
 * [0, FWC_ANGLE_MAX_RAD] so setting this to 0 turns the FW off entirely. */
#ifndef FWC_ANGLE_MAX_RAD
#define FWC_ANGLE_MAX_RAD   0.0f
#endif

/* FOC feed-forward decoupling: cancel cross-coupling and BEMF terms in
 * the d/q current loop, so the PI only handles residual R·I + L·dI/dt
 * dynamics.  Bench evidence (2026-05-22): at 226k eRPM with FWC engaged
 * at max, Id swings -24 to +7 A with no commanded change — current PIs
 * are unstable because they're fighting ω·L·I cross-coupling and ω·λ
 * BEMF terms internally with no voltage headroom (Vs saturated at vmax).
 *
 *   Vd_ff = -ω·L·Iq            (cancel d-axis cross-coupling)
 *   Vq_ff =  ω·L·Id + ω·λ      (cancel q-axis cross-coupling + BEMF)
 *   Vd_total = Vd_ff + PI_d(Id_ref, Id_meas)
 *   Vq_total = Vq_ff + PI_q(Iq_ref, Iq_meas)
 *
 * PI then operates on a clean R+s·L plant, much more stable at speed. */
#ifndef FEATURE_FOC_FF_DECOUPLE
#define FEATURE_FOC_FF_DECOUPLE     1   /* Phase A: BEMF-only FF.
                                         *   Vq_ff = ω · λ_est   (no current dep)
                                         *   Vd_ff = 0
                                         * Frees the full ±vmax envelope for
                                         * the PIs above ~180k eRPM where ω·λ
                                         * approaches vmax and current control
                                         * gets rough.
                                         *
                                         * History — earlier attempt with full
                                         * cross-coupling FF using unfiltered
                                         * id_meas/iq_meas diverged at 110k.
                                         * Phase B (task #128) re-adds cross-
                                         * coupling with LPF on currents. */
#endif
#if FEATURE_FOC_FF_DECOUPLE && !FEATURE_FOC_AN1078
#error "FEATURE_FOC_FF_DECOUPLE requires FEATURE_FOC_AN1078=1"
#endif

/* Startup V2: proper forced-commutation spinup + lock-detect (2026-05-21).
 * Default OFF — preserves the existing ALIGN→CL-direct path so nothing
 * regresses on the bench. Set to 1 to enable the new startup which:
 *   ALIGN  →  OL_RAMP_V2 (forced comm, rate & duty ramp, captures observed)
 *          →  lock detect (N consecutive plausible captures)
 *          →  CL with PI seeded from observed Tp
 * Does NOT modify the IOC/PTG ZC detection path. */
#ifndef FEATURE_STARTUP_V2
#define FEATURE_STARTUP_V2  1   /* default ON 2026-05-21 for bench validation —
                                   flip back to 0 once tuned */
#endif

/* ── Motor Profile Selection ──────────────────────────────────────── */
#ifndef MOTOR_PROFILE
#define MOTOR_PROFILE   2   /* 0=Hurst, 1=A2212, 2=2810, 3=HiZ1460 */
#endif

/* ── Clock ─────────────────────────────────────────────────────────── */
/* dsPIC33AK128MC106 (DS70005539 §12):
 *   Fosc = 200 MHz, Fcy = 200 MHz (AK is 1-cycle-per-instruction).
 *   CLK5 = 400 MHz (PWM clock — VCO_DIV=800 MHz, INTDIV=1 → /2 → 400 MHz)
 */
#define FOSC                200000000UL  /* 200 MHz */
#define FCY                 200000000UL  /* 200 MHz instruction clock (AK: 1 cyc/instr) */
#define FOSC_PWM_MHZ        400U         /* CLK5 PWM clock (MHz) — see hal/clock.c */

/* ── PWM ───────────────────────────────────────────────────────────── */
/* Mode-specific PWM rates. 6-step has a narrow window of working PWM
 * frequencies because BEMF sampling is locked to PWM edges (40 kHz proven
 * dead-end on this hardware — see ak_40khz_floating_coupling memory note).
 * FOC's SMO is not coupled to PWM edges, so it can use whatever rate gives
 * the best observer resolution + acceptable switching losses.
 *
 * Override either at compile time:
 *   make MP_EXTRA_CC_PRE="-DPWMFREQUENCY_HZ_FOC=60000"
 */
#ifndef PWMFREQUENCY_HZ_6STEP
#define PWMFREQUENCY_HZ_6STEP   60000U   /* 6-step: 60 kHz proven for PTG path
                                          * at 225k eRPM milestone. 30 kHz also
                                          * works for PTG; 40 kHz dead-end.
                                          * (30 kHz tested with IOC path — was
                                          * counter-productive, see docs.) */
#endif
#ifndef PWMFREQUENCY_HZ_FOC
#define PWMFREQUENCY_HZ_FOC     30000U   /* FOC first-light at 30 kHz; bump
                                          * to 60000 to match source board's
                                          * 200k+ tuning (an1078_200k milestone). */
#endif

/* Active PWM rate selected by FEATURE_FOC_AN1078 — all downstream
 * derivations (LOOPTIME_TCY, MIN/MAX_DUTY, AN_FS_HZ, AN_TS, …) hang off
 * this single symbol so the two paths can't drift apart. */
#if FEATURE_FOC_AN1078
#define PWMFREQUENCY_HZ     PWMFREQUENCY_HZ_FOC
#else
#define PWMFREQUENCY_HZ     PWMFREQUENCY_HZ_6STEP
#endif

#define LOOPTIME_MICROSEC   (uint16_t)(1000000UL / PWMFREQUENCY_HZ)  /* 50 us @ 30 kHz */

/* PGxPER from DS70005539 Eq 14-1 (Center-Aligned): PGxPER = 8·FPGx/FPWM − 16.
 * PGx clock is Std Speed Peripheral Clock (100 MHz) via PCLKCON.MCLKSEL=0
 * in hal_pwm.c — routing 400 MHz CLK5 would overflow uint16_t below 60 kHz. */
#define FPGX_CLK_HZ         100000000UL   /* PG clock — Std Speed Periph Clk */
#define LOOPTIME_TCY        (uint16_t)((8UL * FPGX_CLK_HZ / PWMFREQUENCY_HZ) - 16U)

#define MAX_DUTY            (LOOPTIME_TCY - 200U)  /* ~100% complementary */
#define MIN_DUTY            200U
/* Dead time: ATA6847L handles 700 ns CCPT internally → 100 ns MCU dead
 * time is sufficient. PGxDTy = 16·FPGx·DeadTime per DS70005539 Eq 14-2. */
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
/* Timer1 input is Standard Speed Peripheral Clock = FPB/2 = 100 MHz. */
#define TIMER1_INPUT_CLOCK  100000000UL  /* Hz — Std Speed Peripheral Clock */
#define TIMER1_PRESCALE     8U
#define TIMER1_FREQ_HZ      20000U       /* 50 µs period — matches ADC ISR rate */
#define TIMER1_PR           (uint16_t)(TIMER1_INPUT_CLOCK / TIMER1_PRESCALE / TIMER1_FREQ_HZ - 1)

/* ================================================================
 *          MOTOR-SPECIFIC PARAMETERS  (per-profile)
 * ================================================================ */

#if MOTOR_PROFILE == 0
/* ── Hurst DMB2424B10002 — 10P (5PP), 24V, 3.4A, low-KV bench motor ── */
#define MOTOR_POLE_PAIRS    5U
#define MOTOR_RS_MILLIOHM   534U
#define MOTOR_LS_MICROH     471U
#define MOTOR_KV            149U

#define ALIGN_TIME_MS       200U
#define ALIGN_DUTY          (LOOPTIME_TCY / 20)
#define INITIAL_STEP_PERIOD 1000U
#define MIN_STEP_PERIOD     66U
#define RAMP_ACCEL_ERPM_S   1500U
#define RAMP_DUTY_CAP       (LOOPTIME_TCY / 6)

#define MAX_CLOSED_LOOP_ERPM 15000U
#define RAMP_TARGET_ERPM     3000U

/* ILIM DAC: VILIM_TH = 3.3·DAC/128, I = (VILIM_TH-1.65)/(16·0.003).
 * DAC=80 → 8.6A trip (2.5× Hurst's 3.4A rating). */
#define ILIM_DAC            80U

#define VBUS_OV_THRESHOLD   (3200U * 16U)  /* ~30V */
#define VBUS_UV_THRESHOLD   (700U * 16U)   /* ~7V  */

#elif MOTOR_PROFILE == 1
/* ── A2212 1400KV — 14P (7PP), 12V (3S LiPo) drone motor ──────────── */
#define MOTOR_POLE_PAIRS    7U
#define MOTOR_RS_MILLIOHM   65U
#define MOTOR_LS_MICROH     30U
#define MOTOR_KV            1400U

#define ALIGN_TIME_MS       150U
#define ALIGN_DUTY          (LOOPTIME_TCY / 40)    /* ~2.5% */
#define INITIAL_STEP_PERIOD 800U
#define MIN_STEP_PERIOD     50U
#define RAMP_ACCEL_ERPM_S   1500U
#define RAMP_DUTY_CAP       (LOOPTIME_TCY / 6)     /* ~17% */

#define MAX_CLOSED_LOOP_ERPM 100000U
#define MIN_CL_STEP_PERIOD   2U          /* Tp=2 ≈ 100k eRPM; HR timing handles sub-tick */
#define RAMP_TARGET_ERPM     4000U

#define ILIM_DAC            85U          /* ~6.3A trip */

#define VBUS_OV_THRESHOLD   (1817U * 16U)  /* ~24V — headroom for regen spikes */
#define VBUS_UV_THRESHOLD   (454U * 16U)   /* ~6V */

#elif MOTOR_PROFILE == 2
/* ── 2810 1350KV — 14P (7PP), 5-6S LiPo, 7-8" FPV drone motor ─────── */
#define MOTOR_POLE_PAIRS    7U
#define MOTOR_RS_MILLIOHM   50U
#define MOTOR_LS_MICROH     25U
#define MOTOR_KV            1350U

#define ALIGN_TIME_MS       150U
#define ALIGN_DUTY          (LOOPTIME_TCY / 40)
#define INITIAL_STEP_PERIOD 800U
#define MIN_STEP_PERIOD     50U
/* RAMP_DUTY_CAP /6 (~17%): /10 and /8 both failed to break standstill
 * on the 2810 (rotor stationary, stl=41 at CL entry). Cogging needs 17%. */
#define RAMP_ACCEL_ERPM_S   2500U
#define RAMP_DUTY_CAP       (LOOPTIME_TCY / 6)

#define TIMING_ADVANCE_LEVEL 3U          /* 22.5° */

#define MAX_CLOSED_LOOP_ERPM 150000U
#define MIN_CL_STEP_PERIOD   2U
#define RAMP_TARGET_ERPM     3000U

#define DUTY_RAMP_ERPM       60000U

/* ILIM 120 ≈ 30.6A: 110 (24.7A) chops too aggressively under prop
 * load, limiting speed to ~40k. 120 allows prop operation while still
 * tripping on genuine desync (>34A). */
#define ILIM_DAC            120U

/* VBUS_OV 40V: regen spikes at 24V supply reach 32V+ on fast decel.
 * Board FETs rated 100V → 40V is safe headroom. */
#define VBUS_OV_THRESHOLD   (48440U)
#define VBUS_UV_THRESHOLD   (12110U)     /* ~10V */

#elif MOTOR_PROFILE == 3
/* ── HiZ1460 — 7PP, 16Ω, 1460KV @ 30V, high-impedance bench motor ─── */
/* Rs caps current at 30/16 = 1.87A regardless of duty. */
#define MOTOR_POLE_PAIRS    7U
#define MOTOR_RS_MILLIOHM   16000U
#define MOTOR_LS_MICROH     15U
#define MOTOR_KV            1460U

#define ALIGN_TIME_MS       200U
#define ALIGN_DUTY          (LOOPTIME_TCY / 20)
#define INITIAL_STEP_PERIOD 1000U
#define MIN_STEP_PERIOD     50U
#define RAMP_ACCEL_ERPM_S   500U
#define RAMP_DUTY_CAP       (LOOPTIME_TCY / 8)

#define TIMING_ADVANCE_LEVEL 3U

#define MAX_CLOSED_LOOP_ERPM 250000U
#define MIN_CL_STEP_PERIOD   2U
#define RAMP_TARGET_ERPM     3000U

/* 35% idle: 35% × 30V = 10.5V → 0.66A — high-Z needs this much to make
 * any torque at idle. */
#define CL_IDLE_DUTY_PERCENT 35U

/* ILIM effectively disabled — Rs naturally limits current to 1.87A. */
#define ILIM_DAC            120U

#define VBUS_OV_THRESHOLD   (48440U)
#define VBUS_UV_THRESHOLD   (12110U)

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
#define FEATURE_GSP             1   /* Back to 1 for 6-step testing — re-enables pot_capture.py
                                     * and the GUI binary protocol on UART1. Set to 0 only when
                                     * doing FOC bring-up text debug (HAL_UART_WriteString
                                     * is stubbed when GSP=1, so the FOC debug prints don't
                                     * appear; that's fine for 6-step since it uses GSP).
                                     * 1 = GSP binary protocol on UART1 (disables debug prints)
                                     * 0 = debug UART text output */
#endif

/* ── Sector PI Architecture ────────────────────────────────────────
 * Ground-up rewrite modeled on Microchip AVR high-speed motor control.
 * SCCP3 = sector timer (periodic, fires commutation ISR).
 * SCCP4 = HR free-running timer (640 ns/tick) for timestamps.
 * PI always owns commutation scheduling from startup onward.
 * BEMF detection runs in the PTG ISR (mid-OFF / mid-ON per duty). */

/* Motor phase advance (electrical degrees, 0..30). Tunable live via
 * GSP SET_PARAM 0xF0. PTG sweep validated 12.5°; 5° hurt more than
 * 10°, so this is partly torque advance, not pure latency compensation. */
#define PHASE_ADVANCE_DEG    12.5f

/* ATA6847 comparator propagation. EV92R69A has no RC filter on BEMF. */
#define RC_DELAY_US          2.0f

/* Pre-computed constants (compile-time, no float in hot path) */
#define ADVANCE_PLUS_30_FP8  ((uint16_t)((PHASE_ADVANCE_DEG + 30.0f) * 256.0f / 60.0f + 0.5f))
#define RC_DELAY_HR          ((uint16_t)(RC_DELAY_US * 1.5625f + 0.5f))

/* PI gains (bit shifts). Matches Microchip AVR motor.c:444,448. */
#define PI_KP_SHIFT             2       /* Kp = 1/4 */
#define PI_KI_SHIFT             4       /* Ki = 1/16 */

/* Startup + MIN_AMPLITUDE + block-comm thresholds (per-profile).
 *   MIN_AMPLITUDE_PROFILE  Q15 idle floor — must push past BEMF-blind
 *                             low-speed regime so CL doesn't stall at idle.
 *   BLOCK_ENTER_ERPM       Just below duty-saturation eRPM at Vbus.
 *   BLOCK_EXIT_ERPM        Re-entry hysteresis (~0.87× enter). */
#if MOTOR_PROFILE == 0   /* Hurst */
#define STARTUP_SPEED_ERPM   3000UL
#define STARTUP_CURRENT_MA   2000.0f
#define ALIGN_DURATION_MS    200U
#define MIN_AMPLITUDE_PROFILE 5000U     /* 15.3% Q15 — Hurst is high-Rs, idles fine */
#define BLOCK_ENTER_ERPM     30000UL    /* Hurst peaks ~25-30k, mostly out of range */
#define BLOCK_EXIT_ERPM      25000UL
#elif MOTOR_PROFILE == 1  /* A2212 @ 12V */
#define STARTUP_SPEED_ERPM   500UL
#define STARTUP_CURRENT_MA   3000.0f
#define ALIGN_DURATION_MS    100U
#define MIN_AMPLITUDE_PROFILE 5000U     /* 15.3% Q15 — bench-safe (12V/65mΩ) */
#define BLOCK_ENTER_ERPM     100000UL
#define BLOCK_EXIT_ERPM      85000UL
#elif MOTOR_PROFILE == 2  /* 2810 @ 25V */
#define STARTUP_SPEED_ERPM   500UL
#define STARTUP_CURRENT_MA   3000.0f
#define ALIGN_DURATION_MS    100U
#define MIN_AMPLITUDE_PROFILE 5000U
#define BLOCK_ENTER_ERPM     150000UL
#define BLOCK_EXIT_ERPM      130000UL
#elif MOTOR_PROFILE == 3  /* HiZ1460 */
#define STARTUP_SPEED_ERPM   500UL
#define STARTUP_CURRENT_MA   1500.0f
#define ALIGN_DURATION_MS    200U
#define MIN_AMPLITUDE_PROFILE 5000U
#define BLOCK_ENTER_ERPM     250000UL
#define BLOCK_EXIT_ERPM      220000UL
#endif

#define STARTUP_TIME_MS      1000U
/* Stall threshold: weighted miss counter (+1/sector miss, -2/sector
 * accept). Stored as uint16_t in sector_pi.c. At 19 k eRPM (~1.9 kHz
 * commutation rate) with 20 % accept ratio, the counter climbs ~930/s
 * during desync. 200 tripped in ~210 ms — too eager when real ZCs are
 * intermittently absent during a brief lag. 1000 gives ~1 s recovery
 * window before ShutOff. Bumped 2026-05-21 after CNSTYLE=1 fix exposed
 * real ZC behavior. */
#define STALL_THRESHOLD      1000U
#define MIN_PERIOD_HR           10U     /* Timer period floor (~1.5M eRPM safety) */

/* SCCP4 HR timer: CLKSEL=000 (Std Speed Periph Clock = 100 MHz),
 * TMRPS=0b11 (/64) → 1.5625 MHz = 640 ns/tick. PI scaling is tuned
 * against this clock — change it and re-tune Kp/Ki shifts. */
#define SECTOR_TIMER_FREQ_HZ        1562500UL
#define ERPM_TO_PERIOD(e)    (uint16_t)(60UL * SECTOR_TIMER_FREQ_HZ / (6UL * (e)))

/* ISR priorities */
#define SECTOR_ISR_PRIORITY  6       /* Commutation — highest motor ISR */
#define CAPTURE_ISR_PRIORITY 5       /* ZC edge capture — below sector */

/* PWM-midpoint BEMF sampling. PTG ISR samples in whichever of the OFF
 * or ON window is wider (below/above 50% duty) so the 3-read deglitch
 * doesn't straddle a switching edge. Above ~80% duty MID-OFF desyncs
 * (window too narrow for ISR latency + 3-read fit). */

#define PTG_TRIG_MID_OFF_POS       1U
/* MID-ON sample position. AK center-aligned PWM is two back-to-back
 * up-counter cycles per pulse (DS70005539 §14.4.2.2.4). PGxTRIGy[31] =
 * CAHALF selects which cycle the trigger fires in. PG counter ticks
 * at 1.6 GHz (625 ps/tick) regardless of PWM frequency; MPER spans
 * half the PWM period.
 *
 * Prior: 3% of MPER past peak — scaled with PWM (250 ns @ 60kHz,
 * 375 ns @ 40 kHz). Hypothesis under test (2026-05-19): the absolute-
 * time settling envelope (motor L/R, gate driver CCPT) is fixed by
 * physics and doesn't scale with PWM, so scaling our sample position
 * proportionally lands at different points in that envelope at each
 * frequency.
 *
 * New: fixed 400 PG ticks past peak (= 250 ns absolute) regardless
 * of PWM frequency. Matches the working 60 kHz position exactly;
 * at 40 kHz this places the sample closer to peak than the prior
 * 3%-of-MPER setting did. If 40 kHz works (or works better) with
 * this, the proportional scaling was the issue. */
#define PTG_TRIG_MID_ON_POS \
    ((uint32_t)0x80000000UL | 400U)

/* Single-sample-per-PWM-cycle, duty-adaptive position.
 *
 * Reason: under prop load at low duty, freewheel current through the
 * low-side body diodes during OFF time couples large transient voltages
 * onto the active phases. That noise rides on the floating-phase pin
 * and the MID-OFF sample lands in this corrupted region — ProcessBemfSample
 * sees post-comm chatter (lE ≈ 19 HR) instead of the real BEMF ZC at ~T/2.
 * MID-ON sampling above ~25% duty avoids the freewheel ringing because
 * the active phase is hard-driven and the floating phase sees clean
 * BEMF + a stable level shift the comparator threshold tolerates.
 *
 * Below 25%-5% duty: MID-OFF (OFF window is wide enough, ON is too narrow).
 * Above 25%+5% duty: MID-ON  (clean signal away from freewheel ringing).
 * ±5% hysteresis prevents sample-position chatter when PI hunts at the
 * threshold. */
#ifndef PTG_DUTY_ADAPT_THRESHOLD_PCT
#define PTG_DUTY_ADAPT_THRESHOLD_PCT   25U
#endif
#define PTG_DUTY_ADAPT_THRESHOLD \
    ((uint16_t)(((uint32_t)LOOPTIME_TCY * PTG_DUTY_ADAPT_THRESHOLD_PCT) / 100U))
#define PTG_DUTY_ADAPT_HYST \
    ((uint16_t)(((uint32_t)LOOPTIME_TCY * 5U) / 100U))

/* Sample-mode enum, exported via telemetry. Only SINGLE_OFF and
 * SINGLE_ON are used at runtime now; DUAL (1) is reserved for hosts
 * that still parse it from older builds. */
#define PTG_SAMPLE_MODE_SINGLE_OFF  0U
#define PTG_SAMPLE_MODE_DUAL        1U
#define PTG_SAMPLE_MODE_SINGLE_ON   2U

/* PTG ISR postscaler. N=3 (20 kHz BEMF) regressed peak to 172k vs 226k. */
#ifndef PTG_POSTSCALE_N
#define PTG_POSTSCALE_N            1U
#endif

/* PTG ISR priority 4: above ADC(3), below CCP(5). Tying with CCP(5)
 * regressed peak 107k→62k — CCP servicing got queued behind PTG. */
#define PTG_ISR_PRIORITY  4

/* Post-ZC shadow counters (rising/falling Acc/Rej) populated by
 * ProcessBemfSample. Reported via GSP snapshot — diagnostic only,
 * does not drive motor control. */
#ifndef FEATURE_POST_ZC_ACCEPT
#define FEATURE_POST_ZC_ACCEPT  1
#endif

/* Measurement-domain shadow PI: smoothed period tracker tMeasHRSmooth
 * computed in parallel with the set-point PI. Telemetry-only — exposed as
 * the eTPm column. α = 1/4 (matches set-point Ki). */
#ifndef FEATURE_MEAS_PI
#define FEATURE_MEAS_PI  1
#endif
#ifndef MEAS_PI_ALPHA_SHIFT
#define MEAS_PI_ALPHA_SHIFT  2
#endif

/* (FEATURE_FOC_AN1078 default lives at the top of this file because
 *  PWMFREQUENCY_HZ depends on it. See the "Mode select" block.) */

/* Default PTGT0LIM values. At FCY=200 MHz with PTGDIV=0, 1 PTG tick = 5 ns.
 * Valley delay covers ATA6847 comparator propagation (~500 ns) + settle.
 * Peak delay = MPER/4 (PWM counter triangle: valley → peak = MPER/2 PWM-tick
 * = MPER/4 PTG-tick since PWM-tick is 10 ns and PTG-tick is 5 ns). */
#define PTG_TICK_NS          10u
#define PTG_VALLEY_DELAY     60u        /* 600 ns settle after trigger */
#define PTG_PEAK_DELAY       ((uint16_t)(LOOPTIME_TCY / 4u))



/* ── ARM ───────────────────────────────────────────────────────────── */
#define ARM_TIME_MS         200U
#define ARM_TIME_COUNTS     ((uint16_t)((uint32_t)ARM_TIME_MS * TIMER1_FREQ_HZ / 1000))

/* ── ADC Channel Mapping — EV92R69A + EV68M17A ────────────────────── */
/* DIM pin map per DS70005527 §5. AK ADC is per-channel:
 *   POT  on AD1CH1 (PINSEL=10, DIM 28 → AN10/RA11)
 *   VBUS on AD1CH4 (PINSEL=6,  DIM 39 → AN6/RA7)
 * Phase + bus currents via OA1/OA2/OA3. J5/J7/J9 must be in BEMF
 * position so ATA6847L comparators stay live; that frees the op-amps
 * for current sense. Raw ADC is unsigned 12-bit, zero current = 2048. */
#define ADCBUF_POT          AD1CH1DATA
#define ADCBUF_VBUS         AD1CH4DATA
#define ADCBUF_IA           AD1CH0DATA   /* OA1OUT/AN0/RA2 */
#define ADCBUF_IB           AD2CH0DATA   /* OA2OUT/AN1/RB0 */
#define ADCBUF_IBUS         AD1CH2DATA   /* OA3OUT/AN3/RA5 */
#define ADC_CURRENT_BIAS    2048

/* ── ADC → physical-unit calibration (Q8 fixed-point) ───────────────
 *
 * Current: 3mΩ shunt, OA G=24.95× (stock AK DIM, R_F=4.99kΩ/R_IN=200Ω,
 * DS70005527 Table 2-1), ADC=1241.21 counts/V → 10.7654 mA/count × 256.
 * Range ±22A — fine for no-load bench, WILL SATURATE under prop load.
 *
 * For prop testing, replace all six R_F (OA1: R5,R18; OA2: R24,R31;
 * OA3: R10,R21) with matched 0.1% thin-film and update Q8 below:
 *   2.49kΩ → ±44A (Q8=5520) | 2.00kΩ → ±55A (Q8=6872, recommended)
 *   1.50kΩ → ±73A (Q8=9162) | 1.00kΩ → ±110A (Q8=13743)
 * Recommended part for ±55A: Panasonic ERA-3AEB2001V × 6.
 *
 * Vbus: 16:1 divider (R47 36kΩ / R49 2.4kΩ, AN6285 Table 5-2)
 *       → 12.8906 mV/count × 256 = 3300. */
#define ADC_MA_PER_COUNT_Q8       2756U   /* stock ±22A */
#define ADC_VBUS_MV_PER_COUNT_Q8  3300U

#endif /* GARUDA_CONFIG_H */
