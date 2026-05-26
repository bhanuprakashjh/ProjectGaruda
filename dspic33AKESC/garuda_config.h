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
#define FEATURE_BEMF_CLOSED_LOOP 1  /* Phase 2: BEMF ZC detection — ENABLED for 6-step (2026-05-25) */
#define FEATURE_VBUS_FAULT       1  /* Phase A4: Bus voltage OV/UV fault enforcement */
#define FEATURE_DESYNC_RECOVERY  1  /* Phase B2: Controlled restart-on-desync (ESC_RECOVERY) */
#define FEATURE_DUTY_SLEW        1  /* Phase B1: Asymmetric duty slew rate limiter */
#define FEATURE_THROTTLE_ZERO_AUTO_DISARM 0  /* When 1, motor disarms to IDLE state if
                                              * throttle stays below ARM_THROTTLE_ZERO_ADC
                                              * for 50ms. When 0, motor keeps running at
                                              * CL_IDLE_DUTY when throttle is at zero —
                                              * which matches the user's expectation that
                                              * pot-zero = motor-idling, not motor-off.
                                              * Stopping the motor in this mode requires
                                              * GSP stop command or power cycle. */
#define FEATURE_TIMING_ADVANCE   1  /* Phase B3: Linear timing advance by RPM — RE-ENABLED 2026-05-26 to compensate detection-chain latency at high RPM. Original baseline schedule: 0° below 3k eRPM, linear ramp to 22° at MAX_CLOSED_LOOP_ERPM (70k for 2810), clamped 22° above. */
#define FEATURE_DYNAMIC_BLANKING 1  /* Phase C1: Speed+duty-aware blanking (extra blank at high duty/demag) */
#define FEATURE_VBUS_SAG_LIMIT   1  /* Phase C2: Bus voltage sag power limiting (reduce duty on Vbus dip) */
#define FEATURE_BEMF_INTEGRATION 1  /* Phase E: Shadow integration estimator (shadow-only, no control) */
#define FEATURE_SINE_STARTUP     0  /* Disabled (2026-04-20) — using straight forced
                                     * commutation for prop-start debug. When 0, ALIGN
                                     * applies ALIGN_DUTY_PERCENT directly; OL_RAMP
                                     * advances steps at increasing rate with RAMP_DUTY_CAP.
                                     * Much easier to reason about current and timing
                                     * than sine modulation. */
#define FEATURE_ADC_CMP_ZC       1  /* Phase F: ADC comparator-based high-speed ZC — required for 6-step (2026-05-25). */
#define FEATURE_HW_OVERCURRENT  1  /* Phase G: Hardware overcurrent protection via CMP3+OA3 */

/* FOC (Field-Oriented Control) — compile-time alternative to 6-step */
#define FEATURE_FOC              0  /* Phase I: OLD FOC v1 (reference, deprecated) */
#define FEATURE_FOC_V2           0  /* Phase I v2: closed-loop current control + MXLEMMING */
#define FEATURE_FOC_V3           0  /* Phase J: FOC v3 — SMO observer + PLL */
#define FEATURE_FOC_AN1078       0  /* 2026-05-25: switched to 6-step. Flip back to 1 to return to FOC AN1078. */
#define FEATURE_SMO              0  /* 0=PLL only, 1=PLL+SMO parallel (v1 only) */
#define FEATURE_MXLEMMING        0  /* 0=PLL chain, 1=MXLEMMING flux observer (v1 only) */
#define FEATURE_LEARN_MODULES    0  /* master: ring buffer + quality + health */
#define FEATURE_ADAPTATION       0  /* requires FEATURE_LEARN_MODULES */
#define FEATURE_COMMISSION       0  /* requires FEATURE_LEARN_MODULES */
#define FEATURE_EEPROM_V2        1  /* NVM persistent storage for GSP params */
#define FEATURE_X2CSCOPE         0  /* X2CScope via UART1 (bring-up debug) */
#define FEATURE_GSP              1  /* Garuda Serial Protocol via UART1 */

/* ADC pot: ON for MCLV-48V-300W bench, OFF for flight boards */
#define FEATURE_ADC_POT          1

/* Burst scope: 128-sample ring buffer at ISR rate, triggered readout */
#define FEATURE_BURST_SCOPE      1  /* Triggered high-rate burst scope (128×28B ring buffer) */

/* RX input features: default OFF (Phase H) */
#define FEATURE_RX_PWM           1  /* RC PWM capture (1000-2000us) */
#define FEATURE_RX_DSHOT         1  /* DShot digital protocol */
#define FEATURE_RX_AUTO          1  /* Auto-detect DShot vs PWM */

/* C.0 DMA gate test (Phase H, Milestone C.0) — default OFF */
#define C0_DMA_TEST              0

/* Diagnostic: Manual step mode (1=enabled)
 * SW1: Start motor → align to step 0
 * SW2: Manually advance one commutation step
 * LED2: Toggles on each step advance
 * No automatic ramp — user controls step timing.
 * Set to 0 for normal auto-ramp operation. */
#define DIAGNOSTIC_MANUAL_STEP  0

/* FOC diagnostic levels (requires FEATURE_FOC=1):
 * 0 = Normal FOC (production)
 * 1 = PWM-only: 50% duty, no FOC math (tests PWM hardware)
 * 2 = Open-loop voltage: applies fixed Vq, reads current (tests current sense)
 *     Check focIa/focIb/focSubState in watch. Expect:
 *       focSubState=98, Iq≈Vq/Rs (positive = correct polarity)
 * 3 = FOC with PI but no feedforward (tests PI only) */
#define FOC_DIAG_PWM_TEST       2

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
#define PWMFREQUENCY_HZ            45000       /* PWM switching frequency
                                                 * History:
                                                 *   24→40 kHz (2026-04-20): ripple/HWZC.
                                                 *   40→48 kHz (2026-04-25): SMC angle res.
                                                 *   48→60 kHz (2026-04-25): chasing 200k.
                                                 *   60→45 kHz (2026-04-26): production
                                                 *     compromise — 60 kHz pushed the
                                                 *     hardware past comfort.  PLL+FW
                                                 *     reach ~130k eRPM here, plenty for
                                                 *     drone use under prop load.
                                                 *
                                                 * Trade-off vs 60 kHz:
                                                 *   - Top no-load eRPM: 201k → ~130k (-35%)
                                                 *   - FET switching losses: ~25% lower
                                                 *   - Thermal margin: significant
                                                 *     improvement for sustained ops
                                                 *   - Per-tick angle res at 130k: ~21°
                                                 *     (similar to 60 kHz at 200k — same
                                                 *     observer wobble margin)
                                                 *
                                                 * AN_FS_HZ in an1078_params.h MUST match
                                                 * (F_PLANT/G_PLANT/KSLF_SCALE all derive
                                                 * from AN_TS = 1/AN_FS_HZ). */

/*──────────────────────────────────────────────────────────────────────────
 * Motor Profile Selection
 * 0 = Hurst DMB2424B10002 (long Hurst, 10-pole 5PP, 24V bench motor)
 * 1 = A2212 1400KV (14-pole 7PP, 12V drone motor)
 * 2 = 2810 1350KV (14-pole 7PP, 5-6S drone motor, ~226k eRPM ceiling at 24V)
 * 3 = 5055 ~580KV (14-pole 7PP, 4S, 0.05 ohm L-N, 17.5 uH L-N)
 *
 * All motor-dependent parameters are grouped here for easy swapping.
 * Board-specific and feature-tuning parameters are below.
 *──────────────────────────────────────────────────────────────────────────*/
#define MOTOR_PROFILE  2

#if MOTOR_PROFILE == 0
/* === Hurst DMB2424B10002 (long Hurst, MCLV-48V-300W bench motor) ===
 * 10 poles (5PP), 24VDC, Rs=534mΩ, Ls=471µH (auto-detected)
 * NOT the short DMB0224C10002 (Rs=2.54Ω, Ls=2.3mH) */
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
#define OC_CLPCI_ENABLE            0       /* Disabled for FOC: SVPWM incompatible with CLPCI chopping */

#elif MOTOR_PROFILE == 1
/* === A2212 1400KV (drone motor) ===
 * 14 poles, 12V, 0.065 ohm, ~30 uH
 * 1400 KV => ~16800 RPM @ 12V, 117600 eRPM */
#define MOTOR_POLE_PAIRS             7
#define DEADTIME_NS                300     /* Sweet spot (2026-04-21). 500 → 300 ns cut
                                            * Ibus commutation kickback ~55% at top throttle
                                            * (12 A pk → 5 A pk on A2212/12V bare), +4% eRPM
                                            * (103k → 108k). 200 ns tested and marginal:
                                            * Ibus pk flat, top eRPM +1% only, but ALIGN
                                            * current rose 20% (early shoot-through signature).
                                            * 300 ns is the bottom of useful deadtime range
                                            * on this PWM + FET combo. */
#define ALIGN_DUTY_PERCENT          12     /* Was 8 (bare-motor). Prop load needs more torque
                                            * to hold alignment — 8% let the rotor walk under
                                            * cogging+prop inertia. 12V*12%/0.065=22A stall peak
                                            * (supply CC limits to 10A). */
#define RAMP_DUTY_PERCENT           22     /* Was 15 (bare-motor). Prop stall torque on 8x4.5
                                            * needs ~20-25% duty to break away and accelerate
                                            * through OL ramp. Bare motor will draw more current
                                            * but the 10A supply CC still protects. */
#define INITIAL_ERPM               100     /* Very slow start: 100ms per step — prop can follow */
#define RAMP_TARGET_ERPM          2000     /* Lowered from 3000 for prop start. Prop mass
                                            * + inertia cannot accelerate to 3000 eRPM in the
                                            * OL window. 2000 eRPM is still above the 1500
                                            * HWZC crossover (need sufficient BEMF margin) and
                                            * comfortably above the SW ZC floor. */
#define MAX_CLOSED_LOOP_ERPM    120000     /* 1400KV * 12V * 7pp */
#define RAMP_ACCEL_ERPM_PER_S      400     /* 4.75 s OL ramp — prop needs long dwell.
                                            * Bare-motor was 3000 (1 s ramp); dropped to 1000
                                            * didn't help because prop inertia at 8x4.5 is
                                            * ~6× higher than bare rotor. 400 eRPM/s gives
                                            * the rotor enough time per step to physically
                                            * accelerate the prop and produce real BEMF by
                                            * the time CL entry happens. */
#define SINE_ALIGN_MODULATION_PCT   10     /* Was 4 (bare-motor). Prop requires stronger
                                            * align current to actually hold the rotor in
                                            * position against static friction. */
#define SINE_RAMP_MODULATION_PCT    25     /* Was 12. Prop load + 0.065Ω A2212 + 12V: at
                                            * 12% modulation the peak phase-phase voltage is
                                            * ~2.9 V, current clamped at 10 A supply CC, but
                                            * the 10 A is short transient peaks — average
                                            * torque is not enough to accelerate the prop
                                            * through the sine ramp. 25% gives ~6 V peak
                                            * phase-phase — enough average torque even with
                                            * supply clamping. */
#define ZC_DEMAG_DUTY_THRESH        40     /* Low-L = more demag */
#define ZC_DEMAG_BLANK_EXTRA_PERCENT 18    /* Aggressive demag blanking */
#define HWZC_CROSSOVER_ERPM       1500     /* Reverted — 868b2ff milestone value.
                                            * Raising to 3000 moved HWZC activation to end
                                            * of ramp but SW ZC hadn't established a clean
                                            * lock yet, seed was wrong. Stick with 1500. */
#define CL_IDLE_DUTY_PERCENT        18     /* Was 12 (bare-motor). Prop load requires higher
                                            * idle duty to maintain rotation after morph exit —
                                            * 12% was only breaking the motor into CL then the
                                            * prop inertia stalled it, causing HWZC to lock onto
                                            * PWM noise at floor. 18% keeps the motor spinning
                                            * with an 8x4.5 prop at no-load on bench. */
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
#define OC_CLPCI_ENABLE            0       /* CLPCI disabled for A2212: OA3 ringing (25x gain)
                                            * causes 54-80% false trip rate, LEB can't fix.
                                            * Protection via software ADC + board FPCI instead. */

#elif MOTOR_PROFILE == 2
/* === 2810 1350KV (7-8" FPV/cine drone motor) ===
 * 12N14P, 14 poles (7PP), 5-6S LiPo (18.5-25.2V), Rs~50mΩ, Ls~25µH.
 * At 24V: no-load max eRPM = 1350 × 24 × 7 = 226,800 eRPM.
 * Bench target: 200k eRPM no-prop.
 *
 * Motor data ported from PATA6847/CK board (garuda_6step_ck.X MOTOR_PROFILE=2).
 * GEPRC EM2810 / T-Motor F100 / BrotherHobby Avenger 2810 range.
 *
 * At 24V supply, peak commutation currents can exceed 30A. Board shunt
 * saturates at ~22A. HW CMP3 is the primary protection. Expect occasional
 * BOARD_PCI at extreme duty — tune down if too aggressive. */
#define MOTOR_POLE_PAIRS             7
#define DEADTIME_NS                300     /* 300 ns (2026-04-21) — match A2212 sweet
                                            * spot. Cuts commutation kickback ~55% on
                                            * A2212 bench. At 24V/2810 the commutation
                                            * energy ½LI² is even higher per event →
                                            * kickback reduction potentially decisive for
                                            * the 22A BOARD_PCI trip threshold that held
                                            * 2810 top speed at 78k eRPM on 500 ns. */
#define ALIGN_DUTY_PERCENT           3     /* 24V * 3% / 0.050Ω = 14.4A stall.
                                            * Half of A2212 (8% at 12V) for same current */
#define RAMP_DUTY_PERCENT            8     /* 24V * 8% / 0.050Ω = 38A stall (briefly).
                                            * Motor spins up quickly so stall current is
                                            * momentary; bench CC limit protects */
#define INITIAL_ERPM               150     /* Slow first step — 2810 low-L picks up fast */
#define RAMP_TARGET_ERPM          3000     /* Same as A2212 — sine startup works well here */
#define MAX_CLOSED_LOOP_ERPM     70000     /* Lowered 220k → 70k (2026-04-20).
                                            * Theoretical no-load is 226.8k but the MCLV
                                            * board U25B trips at ~22 A di/dt, limiting
                                            * bench no-load to ~55 k. This value is also
                                            * the upper anchor of the timing-advance
                                            * interpolation — with 220k, advance at 55 k
                                            * was only 5° (vs the 22° MAX_DEG target) and
                                            * the SW-comparator ADC sampler added another
                                            * ~13° of detection latency, for a net of
                                            * ~-8° of effective advance. 70k anchor gives
                                            * ~17° advance at 55 k, offsetting the SW
                                            * latency. If the motor ever exceeds 70 k the
                                            * advance clamps at 22° (fine). */
#define RAMP_ACCEL_ERPM_PER_S     3000     /* 1 s OL ramp. Reverted 2s → 1s (2026-04-20):
                                            * see A2212 profile comment for the HWZC-IIR-
                                            * collapse bug that 2s exposed. */
#define SINE_ALIGN_MODULATION_PCT    3     /* Conservative — low Rs → current rises fast */
#define SINE_RAMP_MODULATION_PCT     8     /* Low because 2810 at 24V has strong BEMF */
#define ZC_DEMAG_DUTY_THRESH        40     /* Same as A2212 */
#define ZC_DEMAG_BLANK_EXTRA_PERCENT 20    /* More aggressive than A2212 (18).
                                            * Low L = longer demag tail under switching */
#define HWZC_CROSSOVER_ERPM       1500     /* Enable HWZC at morph handoff (same as A2212) */
#define CL_IDLE_DUTY_PERCENT         8     /* Match RAMP_DUTY_PERCENT so pot-at-zero CL idle
                                            * is the same as the OL→CL handoff duty. Lower
                                            * idle (6%) made the motor cross sectors slowly
                                            * enough at low pot that detection became
                                            * marginal. Sticking to startup duty gives the
                                            * motor consistent torque budget across the
                                            * full throttle range. */
#define SINE_PHASE_OFFSET_DEG       60     /* Same as other profiles */
#define OC_LIMIT_MA              20000     /* CMP3 chop. Board shunt saturates ~22A */
#define OC_STARTUP_MA            22000     /* Near sensor saturation — relaxed during startup */
#define OC_FAULT_MA              21000     /* SW hard fault just below saturation */
#define OC_SW_LIMIT_MA           18000     /* SW soft limit (duty reduction starts) */
#define RAMP_CURRENT_GATE_MA     10000     /* Hold ramp accel if ibus >10A during OL */
#define FEATURE_PRESYNC_RAMP       0       /* Standard forced OL_RAMP */
#define OC_CLPCI_ENABLE            0       /* CLPCI disabled: same OA3 ringing issue as A2212 */

#elif MOTOR_PROFILE == 3
/* === 5055 ~580KV (colleague's motor — ADJUST KV AS NEEDED) ===
 * Rs = 0.1 ohm pp (0.05 L-N), Ls = 35 uH pp (17.5 L-N)
 * Assumed: 14 poles (7PP), 4S/14.8V
 * 580 KV => ~8584 RPM @ 14.8V = ~60088 eRPM
 *
 * KEY: Very low Rs + heavy rotor = MUST ramp slowly. */
#define MOTOR_POLE_PAIRS             7
#define DEADTIME_NS                500     /* Low-L motor */
#define ALIGN_DUTY_PERCENT           4     /* 14.8V*4%/0.05=11.8A stall — low duty critical! */
#define RAMP_DUTY_PERCENT            8     /* Conservative: 14.8V*8%/0.05=23.7A stall.
                                            * Supply CC will limit in practice. */
#define INITIAL_ERPM               100     /* Very slow: heavy rotor needs time per step. */
#define RAMP_TARGET_ERPM          2000     /* Good BEMF at this speed. */
#define MAX_CLOSED_LOOP_ERPM     65000     /* 580KV * 14.8V * 7pp ≈ 60088, rounded up */
#define RAMP_ACCEL_ERPM_PER_S      150     /* Slow: 14.8s to reach ramp target. */
#define SINE_ALIGN_MODULATION_PCT    3     /* Low: 0.05 ohm means high current per % duty */
#define SINE_RAMP_MODULATION_PCT     8     /* Conservative modulation */
#define ZC_DEMAG_DUTY_THRESH        45     /* Low-L = more demag */
#define ZC_DEMAG_BLANK_EXTRA_PERCENT 16    /* Moderate */
#define HWZC_CROSSOVER_ERPM       1500     /* HWZC immediately after morph */
#define CL_IDLE_DUTY_PERCENT        10     /* Maintain ZC at idle */
#define SINE_PHASE_OFFSET_DEG       60     /* Tune if needed */
#define OC_LIMIT_MA              15000     /* CMP3 CLPCI chopping (15A) */
#define OC_STARTUP_MA            22000     /* Let supply CC limit during startup */
#define OC_FAULT_MA              20000     /* Software hard fault (20A) */
#define OC_SW_LIMIT_MA           10000     /* Soft limit (10A) */
#define RAMP_CURRENT_GATE_MA      6000     /* Hold ramp if bus current > 6A.
                                            * Critical for 0.05 ohm: prevents
                                            * runaway current during forced comm. */
#define FEATURE_PRESYNC_RAMP       0
#define OC_CLPCI_ENABLE            0       /* CLPCI disabled: OA3 ringing issue */

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
#define VBUS_OVERVOLTAGE_ADC       3600        /* ~67V at ratio 23.0 (tunable) */
#define VBUS_UNDERVOLTAGE_ADC      500         /* ~9.3V at ratio 23.0 (tunable) */
#define VBUS_FAULT_FILTER          3           /* Consecutive ADC samples to confirm fault (3 = ~125us) */
#define VBUS_UV_STARTUP_ADC        400         /* ~7.4V: relaxed UV during pre-sync startup.
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
#define POST_SYNC_SETTLE_MS           1000  /* Longer settle window after ZC sync (ms).
                                             * Raised from 500 to smooth morph→CL jerk.
                                             * During this window, duty ramps at 1/DIVISOR of
                                             * normal rate to prevent over-acceleration. */
#define POST_SYNC_SLEW_DIVISOR           4  /* Slew-up rate divisor during settle (4 = 0.5%/ms).
                                             * Raised from 2 for gentler CL entry ramp. */
#endif

/* Desync Recovery (Phase B2) */
#if FEATURE_DESYNC_RECOVERY
#define DESYNC_COAST_MS             200     /* Coast-down time before restart attempt */
#define DESYNC_MAX_RESTARTS         3       /* Max restart attempts before permanent fault */
#endif

/* Timing Advance (Phase B3) */
#if FEATURE_TIMING_ADVANCE
#define TIMING_ADVANCE_MIN_DEG      0       /* Degrees advance at low speed */
#define TIMING_ADVANCE_MAX_DEG      25      /* Degrees advance at TIMING_ADVANCE_MAX_ERPM
                                             * (interpolated linearly from MIN_DEG at
                                             * RAMP_TARGET_ERPM). Bumped 22 → 25 (post-203k,
                                             * 2026-05-26) to push past the advance ceiling
                                             * that capped peak speed. 25° is the static
                                             * assert hard limit (any higher risks desync).
                                             * NOTE: with FEATURE_GSP=1 the EEPROM value
                                             * (gspParams.timingAdvMaxDeg) overrides this
                                             * default. After flashing, run:
                                             *   tools/step6_reset_profile.py --profile 2
                                             * to push the profile-2 (2810) default of 25° into
                                             * EEPROM if it isn't already there. */
#endif

/* Dynamic Blanking (Phase C1) — motor-specific params in motor profile above */

/* Bus Voltage Sag Limiting (Phase C2) */
#if FEATURE_VBUS_SAG_LIMIT
#define VBUS_SAG_THRESHOLD_ADC   900      /* Vbus below this → reduce duty (~13V with typical divider) */
#define VBUS_SAG_RECOVERY_ADC    1000     /* Vbus above this → release limit (hysteresis band = 100) */
#define VBUS_SAG_GAIN            8        /* Duty reduction proportional to sag depth: (depth * gain) >> 4 */
#endif

/* Regen Brake (Phase C3, 2026-05-26): freeze duty-down slewing when Vbus is
 * elevated due to motor regenerating into the supply. Documented bench failure
 * mode: rapid throttle drop from 240k eRPM caused Vbus to climb 25V→32V before
 * triggering a (mislabeled) UV fault. Holding duty constant when Vbus > ON
 * threshold lets the motor coast down naturally instead of dumping regen
 * energy into the bus capacitor.
 *
 * Initial 2V hysteresis (28V/26V) was too narrow — brake chattered on/off as
 * motor briefly decelerated and Vbus dipped under 26V between regen pulses.
 * Widened to 4V hysteresis + minimum engage duration (~10ms = 240 ticks at
 * 24kHz ADC ISR) to make the brake "sticky" — once engaged, stays engaged
 * for at least N ticks regardless of Vbus dips, then releases only if Vbus
 * has fallen all the way to OFF threshold.
 *
 * ADC values at ratio 23.0:
 *   1500 ADC ≈ 28V (engage)
 *   1290 ADC ≈ 24V (release — 4V hysteresis) */
#define FEATURE_VBUS_REGEN_BRAKE 1
#if FEATURE_VBUS_REGEN_BRAKE
#define VBUS_REGEN_BRAKE_ON_ADC    1500    /* ~28V — engage brake */
#define VBUS_REGEN_BRAKE_OFF_ADC   1290    /* ~24V — release brake (4V hysteresis) */
#define VBUS_REGEN_BRAKE_MIN_TICKS 240     /* Minimum engaged duration ~10ms at 24kHz */
#define VBUS_REGEN_BRAKE_SLEW_DIVISOR 16   /* When brake engaged, slew-down rate
                                            * is reduced by this divisor (was a
                                            * full freeze, which made deceleration
                                            * feel clumsy — user couldn't slow
                                            * motor down while Vbus held above
                                            * brake-off threshold).
                                            * Default 5%/ms ÷ 16 = 0.31%/ms during
                                            * brake = ~320ms full-scale shutdown.
                                            * Bus cap absorbs regen at this rate. */
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
#define SINE_TRAP_DUTY_NUM           6  /* Sine->trap duty scale factor numerator. */
#define SINE_TRAP_DUTY_DEN           5  /* Sine->trap duty scale factor denominator.
                                         * 6/5 = 1.2x compensates for the sine-to-trap L-L
                                         * voltage conversion at RAMP_TARGET_ERPM.
                                         * Tune: increase NUM if motor brakes at transition,
                                         *        decrease NUM if motor surges forward. */

/* Waveform Morph: sine-to-trap transition (replaces coast gap) */
#define MORPH_CONVERGE_SECTORS    6   /* Sectors for duty convergence (1 e-cycle).
                                       * At 2000 eRPM: 6 sectors × 5ms = 30ms. */
#define MORPH_HIZ_MAX_SECTORS    36   /* Max sectors in Hi-Z before fault (6 e-cycles). */
#define MORPH_TIMEOUT_MS       2000   /* Absolute morph timeout (ms). */
#define MORPH_ZC_THRESHOLD        4   /* goodZcCount to exit morph → CL. Lowered from 6
                                       * for easier SW ZC lock during morph. */

/* Windowed Hi-Z: progressive float-phase Hi-Z acquisition.
 * Each entry = Hi-Z window as % of step period for that sector.
 * Window centered at 50% of step (≈30° electrical, expected ZC point).
 * Start at 10% (not 6%): 4-tick overhead (settle+init+open-skip)
 * leaves only 3 effective ticks at 6% — too thin for weak phases.
 * Individual macros so _Static_assert can verify the final entry. */
#define MORPH_WINDOW_PCT_0        10
#define MORPH_WINDOW_PCT_1        20
#define MORPH_WINDOW_PCT_2        35
#define MORPH_WINDOW_PCT_3        60
#define MORPH_WINDOW_PCT_4       100
#define MORPH_WINDOW_SCHEDULE     { MORPH_WINDOW_PCT_0, MORPH_WINDOW_PCT_1, \
                                    MORPH_WINDOW_PCT_2, MORPH_WINDOW_PCT_3, \
                                    MORPH_WINDOW_PCT_4 }
#define MORPH_WINDOW_SECTORS      5
#define MORPH_WINDOW_MIN_TICKS    8   /* Absolute minimum Hi-Z window width.
                                       * At 4-tick overhead, guarantees ≥4
                                       * effective sensing ticks per window. */
#endif

/* ADC Comparator ZC (Phase F) — crossover eRPM in motor profile above */
#if FEATURE_ADC_CMP_ZC
#define HWZC_USE_SW_COMPARE      0   /* 0 = ADC digital comparator @ 1 MHz (HW path).
                                      * 1 = software compare on mid-ON ADC sample (SW path).
                                      *
                                      * SW mode: the 24 kHz ADC ISR reads the floating phase
                                      * at PG1TRIGA (mid-ON valley, no PWM ripple visible
                                      * through the board's 5.5 kHz RC filter), compares
                                      * against zcThreshold in C, and schedules commutation
                                      * through the same BLANKING/WATCHING/COMM_PENDING
                                      * state machine. Eliminates the 40–60k/s phantom-ZC
                                      * rate seen on the HW path at 24 V (ripple crosses
                                      * the threshold twice per PWM cycle). Trade-off: ZC
                                      * detection resolution is 42 µs (one PWM period) vs
                                      * 1 µs on the HW path. At 55 k eRPM that's ~13° of
                                      * electrical angle, absorbed by TIMING_ADVANCE. */
/* AM32/CK-style signal-coherence check inside HWZC_OnZcDetected.
 * After comparator IRQ fires, re-read AD1CH5DATA/AD2CH1DATA N times with
 * small NOP delay between reads (~1µs each = spans one ADC sample at the
 * 1 MHz SCCP3 trigger rate). All N reads must show the BEMF still on the
 * expected side of the threshold. Single-sample noise (PWM ripple cross,
 * body-diode recovery, cross-coupling transient) fails this check and is
 * rejected before the plausibility gate sees it.
 *
 * Rationale: AKESC's stock filter is single-shot + interval gate only — at
 * high RPM the watching window is short and false comparator hits that
 * happen to land at >70% of stepPeriodHR get accepted as real ZCs, causing
 * mistimed commutation and high bench current. CK board uses the same
 * consecutive-read idea via filterCount; AM32 calls it `filter_level`.
 *
 * N=3 is conservative (~3µs ISR overhead). N=5 is more robust. */
#define FEATURE_HWZC_PWM_GATE      1   /* Reject ZC captures during PWM OFF time.
                                        * At BEMF crossover (Vapp ≈ Vbemf at ~79%
                                        * duty / 188k eRPM on 2810@24V), the
                                        * comparator can fire in BOTH PWM ON and
                                        * OFF windows at different rotor angles —
                                        * during ON, signal swings around motor
                                        * neutral (duty×Vbus/2); during OFF, all
                                        * driven phases are at GND so floating-
                                        * phase reads pure BEMF (zero-centered).
                                        * Both regimes trigger CMPLO crossings
                                        * but at different rotor positions. PI
                                        * ingesting mixed captures → occasional
                                        * 60° rotor slip → 22A phase spike → UV
                                        * cascade. Gate accepts only ON-time
                                        * captures. At 45 kHz × 79% duty, worst-
                                        * case latency = 4.6 µs OFF time = ~3°
                                        * at 200k eRPM, fits inside 25° advance
                                        * budget. Reads HS GPIO of currently
                                        * PWMing phase (PWM1H=RD2 / PWM2H=RD0 /
                                        * PWM3H=RC3) and rejects if LOW. */
#define FEATURE_HWZC_VERIFY_READS  1
#define HWZC_VERIFY_READS          1   /* 3 → 1 (2026-05-26 post-203k): the verify
                                        * loop now restructured to wait-then-read
                                        * so a single re-read after ~1 µs catches
                                        * the phantom-vs-real distinction. Cost is
                                        * one ADC conversion period regardless of
                                        * N, so N=1 captures the same signal info
                                        * as N=3 with 1/3 the latency. */
/* HWZC_VERIFY_SKIP_ERPM: skip verify reads above this eRPM.
 * The 1 µs verify wait costs ~7° at 200k and on this PCB the 5.5 kHz BEMF
 * filter only attenuates 45 kHz PWM ripple by ~8× — ripple at the ADC pin
 * is still ±125 counts during the verify wait, which can falsely reject real
 * BEMF crossings. Keep verify enabled at low RPM (where PWM ripple is more
 * harmful relative to small BEMF amplitude AND the latency cost is tiny) and
 * skip above 80k where speed/torque demand has no patience for added latency.
 *
 * (Restored 999999 → 80000 after the post-203k experiment showed peak motor
 * speed dropped from 203k → 152k when verify reads were active at high RPM.
 * The interval gate at 85% does the phantom-rejection job at high RPM for free.) */
#define HWZC_VERIFY_SKIP_ERPM      80000

#define HWZC_BLANKING_PERCENT   14   /* Blanking as % of step period (after commutation).
                                      * Bumped 8 → 14 (2026-05-26) to diagnose 50% miss rate
                                      * above ~130k eRPM. Hypothesis: AD2 sectors (Phase A or
                                      * C, PINSEL switch needed) get a first ADC conversion
                                      * with not-yet-settled S&H, causing the comparator to
                                      * either latch on garbage or miss the real ZC. With 8%
                                      * at 130k = 6 µs blanking, the settle window after
                                      * PINSEL was only ~4 µs. 14% at 130k = 10.5 µs gives
                                      * ~8 µs settle window with detection window still 42 µs.
                                      * ZC midpoint at ~26 µs — plenty of margin. */
#define HWZC_HYSTERESIS_ERPM   500   /* Hysteresis band for crossover (prevents oscillation) */
#define HWZC_SYNC_THRESHOLD      6   /* Consecutive HW ZCs to declare sync */
#define HWZC_MISS_LIMIT          3   /* Missed HW ZCs before fallback to software ZC (low for debug) */
#define HWZC_CMP_DEADBAND        4   /* ADC counts deadband for comparator sanity check */
#define HWZC_IIR_FREEZE_ZC_COUNT  3  /* Freeze stepPeriodHR IIR until goodZcCount reaches this.
                                      * Protects against phantom-driven IIR collapse during the
                                      * fragile first ~1ms after morph→CL handoff, where a single
                                      * mistimed ZC used to pull stepPeriodHR to floor and stall
                                      * the motor. Seeded stepPeriodHR drives commDelay until the
                                      * motor locks in and the adaptive IIR takes over. */
#define HWZC_MIN_INTERVAL_PCT   50   /* Reject ZC if interval < this % of stepPeriodHR.
                                      * Loosened 75 → 50 (2026-05-26) to address the 50%
                                      * miss rate at >135k eRPM. Mechanism: at high RPM
                                      * (sector ~ 3 PWM cycles), PWM ripple through the
                                      * 5.5kHz BEMF filter can shift the comparator firing
                                      * time by up to one PWM period (= ~30% of T at 135k).
                                      * The previous 75% gate then rejected these REAL but
                                      * PWM-shifted captures, causing sectors to miss
                                      * entirely. Plausibility gate (capValue > T) and PI
                                      * delta clamp absorb the residual noise. */
#define HWZC_SAMC               3    /* Sample time for high-speed channels (~205ns conversion) */
#define HWZC_ADC_SAMPLE_HZ   1000000  /* High-speed ADC trigger rate (SCCP3). Max ~4.9 MHz. */
#define HWZC_STALL_DUTY_PCT     30   /* % of MAX_DUTY below which floor-speed is implausible.
                                      * If stepPeriodHR is at floor (motor apparently at max
                                      * eRPM) but duty is below this, HWZC is tracking PWM
                                      * noise from a stalled motor. */
#define HWZC_STALL_DEBOUNCE_MS  100  /* ms of continuous implausible state before stall disable */

/* Filter phase-lag compensation (Path 1: CMPLO pre-distortion).
 *
 * The PCB BEMF RC filter (R=3kΩ × C=10nF on MCLV-48V-300W) has τ=30µs and a
 * 5.5kHz cutoff. The filtered signal at the ADC pin lags the true BEMF by
 * φ = arctan(2π·f_elec·τ). At 150k eRPM that's 24° — exceeds the 22°
 * TIMING_ADVANCE_MAX_DEG ceiling, leaving ~2° residual lag that drives
 * high current at the top of the speed range.
 *
 * This feature shifts CMPLO away from neutral by the amount the FILTERED
 * signal is offset at the moment of TRUE zero-crossing, so the HW comparator
 * fires at the pre-filter ZC instant rather than the post-filter one.
 *
 * Math (sinusoidal BEMF approximation near ZC):
 *   stepPeriodHR is in SCCP2 ticks (100 MHz / 10 ns each).
 *   ω·τ        = (π · τ_ns / 30) / stepPeriodHR
 *   ω·τ_Q15    = HWZC_FILTER_K_Q15 / stepPeriodHR
 *   offset_cnt = (zcThreshold × ω·τ_Q15 / 32768) × AMP_PCT / 100
 *   Rising sector  → CMPLO = zcThreshold − offset
 *   Falling sector → CMPLO = zcThreshold + offset
 *
 * K_Q15 derivation: K = 32768 · π · τ_ns / 30. For τ=30000 ns: K = 102_943_706.
 * If τ changes (different filter caps), recompute K manually.
 *
 * AMP_PCT scales the assumed BEMF amplitude (theory says A ≈ zcThreshold for
 * a 6-step motor at no-load, but linear approximation overshoots actual
 * sin(φ) by ~10% at high RPM and load reduces amplitude). Start at 75%, tune
 * by reading dbgFilterOffset on the bench and watching commutation phase.
 *
 * Disabled by default. Enable with FEATURE_HWZC_FILTER_COMP=1 to test. */
#define FEATURE_HWZC_FILTER_COMP    1   /* Restored after diagnostic. Comp off vs on
                                         * showed IDENTICAL ~135k miss-onset threshold
                                         * — so the misses aren't comp-caused. But
                                         * comp DOES help current/efficiency by giving
                                         * effective advance to compensate the 24° RC
                                         * filter lag at high RPM. Keep enabled. */
#define HWZC_FILTER_K_Q15           102943706UL  /* For τ=30µs; recompute if filter changes */
/* Sector PI synchronizer (Phase A — break the 204k reactive ceiling).
 *
 * Architecture inspired by AK ATA6847L's sector_pi.c (the 225k milestone).
 * Decouples DETECTION from COMMUTATION timing:
 *   - HWZC IRQ no longer schedules commutation; it just timestamps the ZC
 *     (lastCaptureHR) and sets captureValid.
 *   - An autonomous SCCP1 timer fires at lastCommHR + timerPeriod and
 *     advances the commutation step.
 *   - On each commutation, PI math adjusts timerPeriod from the phase
 *     error: delta = capValue − setValue, where setValue is the expected
 *     ZC position within the sector (driven by torque advance schedule).
 *
 * Why this lifts the ceiling: with the reactive HWZC path, a single
 * mis-timed ZC propagates directly into a mis-timed commutation, which
 * the IIR period then chases — a self-reinforcing wobble that walls at
 * ~204k. With sector PI, a noisy capValue is clamped at ±T/4 and barely
 * moves timerPeriod, so commutation stays smooth and the motor keeps
 * accepting more torque. Couples cleanly with FEATURE_HWZC_FILTER_COMP
 * (filter comp already aligns capValue with the true rotor ZC).
 *
 * PI math (HR ticks, signed):
 *   setValue    = (advancePlus30Fp8 × timerPeriod) >> 8
 *   delta       = capValue − setValue
 *   delta       = clamp(delta, ±timerPeriod >> CLAMP_SHIFT)
 *   integrator += delta >> KI_SHIFT     (slow drift)
 *   timerPeriod = integrator + (delta >> KP_SHIFT)   (fast dynamics)
 *
 * Disabled by default. Flip to 1 for the breakthrough run. */
#define FEATURE_HWZC_SECTOR_PI         1
#define HWZC_PI_KP_SHIFT               2   /* Kp = 1/4  — proportional gain  */
#define HWZC_PI_KI_SHIFT               4   /* Ki = 1/16 — integral gain      */
#define HWZC_PI_DELTA_CLAMP_SHIFT      3   /* ±T/8 per-sample clamp (default).
                                            * At low/mid RPM this gives responsive
                                            * accel without runaway. Tightens to
                                            * HWZC_PI_DELTA_CLAMP_SHIFT_HIGH at
                                            * very high RPM where the motor can't
                                            * physically accelerate per-event. */
#define HWZC_PI_DELTA_CLAMP_SHIFT_HIGH 4   /* +T/16 POSITIVE delta clamp at high RPM.
                                            * Positive delta = capture late = rotor
                                            * BEHIND expected → grow period → slow
                                            * commutation. Allowing this freely keeps
                                            * PI responsive when motor genuinely
                                            * slows (load increase, throttle drop). */
#define HWZC_PI_NEG_DELTA_CLAMP_SHIFT_HIGH 5  /* -T/32 NEGATIVE delta clamp at high RPM.
                                            * Negative delta = capture early = rotor
                                            * AHEAD expected → shrink period → faster
                                            * commutation. At BEMF ceiling the motor
                                            * CANNOT physically go faster, so any
                                            * shrinkage commutates ahead of rotor →
                                            * wrong-angle commutation → 22A current
                                            * spike → Vbus sag → UV.
                                            *
                                            * Bench data 2026-05-26: AMP_PCT=50 and
                                            * AMP_PCT=35 BOTH desync at 79-80% duty /
                                            * 190k eRPM. PI period at fault was 5217-
                                            * 5266 HR ticks, vs 5330+ in the one
                                            * successful trace. Symmetric clamp let
                                            * PI drift ~2% short over the dwell. Half
                                            * the clamp for negative deltas blocks
                                            * the drift while preserving full
                                            * responsiveness to real slowdowns. */
#define HWZC_PI_CLAMP_HIGH_ERPM      150000 /* Switch to tight clamp above this eRPM */
#define HWZC_PI_CLAMP_HIGH_TICKS     (1000000000UL / HWZC_PI_CLAMP_HIGH_ERPM)
/* Plausibility gate on capValue. Reject any capture more than this fraction
 * BELOW setValue — at lock, real capValue ≈ setValue (within a few %); a
 * capture <50% of setValue is almost certainly a phantom or accel transient.
 * Discarded captures don't update the PI (timerPeriod stays). */
#define HWZC_PI_CAP_MIN_NUM            1   /* lower bound = setValue × NUM/DEN */
#define HWZC_PI_CAP_MIN_DEN            2   /* default: 1/2 of setValue */
#define HWZC_PI_CAP_MAX_NUM            3   /* upper bound = setValue × NUM/DEN */
#define HWZC_PI_CAP_MAX_DEN            2   /* default: 3/2 of setValue */
#define HWZC_FILTER_AMP_PCT         50    /* BEMF amplitude as % of zcThreshold.
                                           * Tested 50 vs 35: AMP=35 helped at 74-78% duty
                                           * (ibusPkNeg -0.61A vs -0.88A) but made 78-82% WORSE
                                           * (-4.87A vs -3.53A) and PI period SHORTER (5217 vs
                                           * 5266 HR ticks), still desyncing. This proved
                                           * filter comp was NOT the dominant driver of the
                                           * 79% wall. The real cause is symmetric PI delta
                                           * clamp letting integrator drift toward shorter
                                           * period at BEMF ceiling. Restored to 50 since
                                           * larger drift correction is handled by the new
                                           * asymmetric clamp (see HWZC_PI_NEG_DELTA_CLAMP_*). */
#define HWZC_FILTER_MAX_OMEGA_Q15   24000 /* Cap ω·τ at ~0.73 rad (≈42° equiv).
                                           * Bumped 20000→24000 after 203k run where cap was
                                           * engaged at 200k+ (ω·τ_Q15=20588 → clamped). Lets
                                           * comp track the full 200-220k band. Caller still
                                           * limits absolute offset via HWZC_FILTER_MAX_OFFSET. */
#define HWZC_FILTER_MAX_OFFSET      600   /* Cap CMPLO shift in ADC counts (12-bit ADC, ~15% FS) */
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
#define OC_LEB_BLANKING_NS     1000   /* Leading-edge blanking (ns). Applied to BOTH the
                                       * board U25B FPCI input (RPn) AND our CMP3 CLPCI.
                                       * Blanks the switching transient (FET rise + ringing
                                       * + deadtime body-diode transient). 2810 @ 24V has
                                       * ~1µs ringing tail. 1000ns = 400 clocks @ 400MHz. */

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

/* P0: ZC_DUTY_THRESHOLD_SHIFT replaced by exact ZC_DUTY_DIVISOR in
 * garuda_calc_params.h. Old >>18 had +1.7% bias (2^18=262144 vs
 * 2*LOOPTIME_TCY=266636). See adaptive ZC threshold plan. */
/* #define ZC_DUTY_THRESHOLD_SHIFT 18 */
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

/* ── Phase H: RX Input Configuration ──────────────────────────────── */

/* Static guard: at least one throttle source must be enabled */
#if !FEATURE_ADC_POT && !FEATURE_GSP && !FEATURE_RX_PWM \
    && !FEATURE_RX_DSHOT && !FEATURE_RX_AUTO
#error "No throttle source enabled — enable at least one of FEATURE_ADC_POT, FEATURE_GSP, FEATURE_RX_PWM, FEATURE_RX_DSHOT, or FEATURE_RX_AUTO"
#endif

/* AUTO requires at least one RX protocol to detect */
#if FEATURE_RX_AUTO && !FEATURE_RX_PWM && !FEATURE_RX_DSHOT
#error "FEATURE_RX_AUTO requires FEATURE_RX_PWM or FEATURE_RX_DSHOT"
#endif

#if (FEATURE_RX_PWM || FEATURE_RX_DSHOT || FEATURE_RX_AUTO)
#define RX_TIMER_HZ             SCCP_CLOCK_HZ
#define RX_COUNTS_PER_US        (RX_TIMER_HZ / 1000000UL)
#define RX_PWM_MIN_US           950
#define RX_PWM_MAX_US           2050
#define RX_PWM_DEADBAND_US      25
#define RX_PWM_PERIOD_MIN_US    2000
#define RX_PWM_PERIOD_MAX_US    25000
#define RX_LOCK_COUNT           10
#define RX_TIMEOUT_MS           200
#define RX_DSHOT_CMD_MAX        47
#define RX_DSHOT_EDGES          64   /* wire truth: 16 bits x 2 edges */
#define RX_DSHOT_DMA_COUNT      RX_DSHOT_EDGES  /* DMA register load;
    change to (RX_DSHOT_EDGES - 1) if count-1 semantics confirmed in M0 */
#define RX_ALIGN_MAX_SHIFTS_PER_CALL  4
#endif

/* C.0 DMA gate test configuration */
#if C0_DMA_TEST
#define C0_STARTUP_TIMEOUT_MS   2000
#define C0_STARTUP_TIMEOUT_COUNTS \
    ((uint32_t)((uint64_t)C0_STARTUP_TIMEOUT_MS * SCCP_CLOCK_HZ / 1000ULL))
#define C0_TARGET_FRAMES        370000  /* ~10s at DShot600 37kHz */
#define C0_MEAS_TIMEOUT_MS      15000   /* 10s + 5s margin */
#define C0_MEAS_TIMEOUT_COUNTS \
    ((uint32_t)((uint64_t)C0_MEAS_TIMEOUT_MS * SCCP_CLOCK_HZ / 1000ULL))

/* IFS clear method — MUST be set after Milestone 0 probe */
/* #define IFS_IS_W1C  1 */  /* W1C: write 1 clears flag */
/* #define IFS_IS_W1C  0 */  /* Direct: write 0 clears flag */
#ifndef IFS_IS_W1C
#error "IFS_IS_W1C not defined — run Milestone 0 probe first"
#endif

/* C.0 error codes */
#define C0_ERR_NONE         0
#define C0_ERR_IC_DRAIN     1  /* ICBNE stuck */
#define C0_ERR_NO_SIGNAL    2  /* no DMA-TC within startup timeout */
#define C0_ERR_MEAS_STALL   3  /* c0Done not set within meas timeout */
#endif /* C0_DMA_TEST */

#ifdef __cplusplus
}
#endif

#endif /* GARUDA_CONFIG_H */
