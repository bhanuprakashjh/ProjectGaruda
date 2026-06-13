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

/* ── Build target (auto-detected from the compiler's device macro) ──────
 * GARUDA_TARGET_AK512: dsPIC33AK512MC510 MC DIM (EV67N21A) port — see
 * docs/migration_dsPIC33AK512MC510.md. Everything above hal/ is shared;
 * only HAL pin/channel tables may branch on this. */
#if defined(__dsPIC33AK512MC510__)
  #define GARUDA_TARGET_AK512   1
  #define AK512_BRINGUP_DIAG        /* TEMP: spi_* telemetry carries raw ADC diagnostics */
#else
  #define GARUDA_TARGET_AK512   0   /* dsPIC33AK128MC106 (original target) */
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Feature Flags (0=disabled, 1=enabled) */
#define FEATURE_BEMF_CLOSED_LOOP 1  /* Phase 2: BEMF ZC detection — ENABLED for 6-step (2026-05-25) */
#define FEATURE_VBUS_FAULT       1  /* Phase A4: Bus voltage OV/UV fault enforcement */
#define FEATURE_DESYNC_RECOVERY  1  /* Phase B2: Controlled restart-on-desync (ESC_RECOVERY) */
#define FEATURE_DUTY_SLEW        1  /* Phase B1: Asymmetric duty slew rate limiter */
#define FEATURE_CL_SOFT_ENTRY    0  /* Smooth OL->CL hand-off: gentle duty ramp at CL entry.
                                     * Default OFF (232k path untouched). When 1, for the
                                     * first CL_SOFT_ENTRY_MS after hand-off the duty slews up
                                     * far more gently so the motor accelerates from the
                                     * low-BEMF hand-off speed to the idle equilibrium with
                                     * bounded current, instead of the structural ~22A slam.
                                     * Requires FEATURE_DUTY_SLEW=1. */
#define CL_SOFT_ENTRY_MS       500  /* Soft-entry window after CL entry (ms). Must exceed the
                                     * time the motor needs to accelerate hand-off->idle. */
#define CL_SOFT_ENTRY_DIVISOR   64  /* Entry up-rate = DUTY_SLEW_UP_RATE / this. Bigger =
                                     * gentler (lower current, slower spin-up). Bench-tune. */
#define FEATURE_IF_BRIDGE        0  /* Option D: I-f current-limited OL->CL hand-off bridge.
                                     * MOTOR-AGNOSTIC smoothing. At CL entry, ramp duty up
                                     * from MIN_DUTY but BACK OFF whenever bus current
                                     * exceeds IF_BRIDGE_LIMIT_MA — so the motor accelerates
                                     * from the (low-BEMF) hand-off speed to the idle
                                     * equilibrium at a BOUNDED current instead of the
                                     * structural ~22A slam. One knob (the current cap)
                                     * scales across motors — no per-motor speed/duty tune.
                                     * Final cap (after OC limiter); only lowers duty, so
                                     * regen/OV/OC protections still win. Default OFF. */
#define IF_BRIDGE_LIMIT_MA   10000  /* Bridge bus-current cap. Keep < active ocSwLimitMa
                                     * (18A on profile 2). Lower = gentler/slower spin-up. */
#define IF_BRIDGE_MS           800  /* Safety: max bridge duration after CL entry (ms).
                                     * Must exceed the hand-off->idle spin-up time. */
#define IF_BRIDGE_UP_PCT_PER_MS   2 /* Duty ramp-up rate while UNDER the current cap. */
#define IF_BRIDGE_DOWN_PCT_PER_MS 8 /* Duty back-off rate when OVER the cap (faster). */
#define IF_BRIDGE_PEAK_DECAY_SHIFT 6 /* Bridge current sense = PEAK-HOLD of |ibusRaw-bias|,
                                     * not the instantaneous sample. ibusRaw is sampled at
                                     * the PWM valley (~0 there), so the real hand-off
                                     * current shows up only as a RECURRING spike (the −22A
                                     * freewheel of the ~22A phase current) that a single
                                     * per-tick read mostly misses. Peak-hold catches it;
                                     * it decays by >>SHIFT/tick (6 ≈ 1.5%/tick, holds the
                                     * peak ~2-3ms across the inter-commutation gap so the
                                     * back-off doesn't chatter). Smaller = holds longer. */
#define FEATURE_HANDOFF_CHOP     0  /* Sub-MIN_DUTY OL->CL current bound via the CMP3 HARDWARE
                                     * cycle-by-cycle chop (CLPCI), not duty. The startup CMP3
                                     * threshold is set HIGH (OC_CMP3_STARTUP_DAC ~22A) to not
                                     * chop startup torque — which is exactly why the hand-off
                                     * pulse reaches ~22A. This holds CMP3 at a LOW chop level
                                     * (OC_CMP3_HANDOFF_MA) for a window at CL entry, so the
                                     * hardware truncates each PWM pulse at that current —
                                     * effective duty goes BELOW MIN_DUTY naturally (no deadtime
                                     * issue) and the phase current (hence the −freewheel pulse)
                                     * is bounded, WITHOUT the regen-oscillation the duty-clamp
                                     * IF_BRIDGE caused. CMP3 is analog/continuous so it sees the
                                     * true ON-time motoring peak the valley-sampled ADC misses.
                                     * Armed only at CL entry (align/OL/morph keep STARTUP_DAC). */
#define OC_CMP3_HANDOFF_MA   12000  /* Hand-off chop current cap (mA). Below STARTUP(~22A) and
                                     * operational(~20A); above idle draw. Lower = gentler
                                     * spin-up / smaller pulse; too low may not clear the gap. */
#define HANDOFF_CHOP_MS       1000  /* Window after CL entry to hold the low chop (ms). Must
                                     * exceed the hand-off->past-the-gap accel time. Idle draw
                                     * is < cap so holding it there is inert. */
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
#define FEATURE_SINE_STARTUP     1  /* RESTORED 2026-06-10: back to the proven sine startup (I-f parked).
                                     * With 0, STARTUP_Init does NOT call SineInit → no bridge
                                     * drive at ARMED→IF_RAMP → I-f owns a clean bring-up.
                                     * Original: Re-enabled (2026-05-28) — try sine startup with
                                     * current HWZC architecture. Profile sets
                                     * HWZC_CROSSOVER_ERPM=1500 → HWZC arms during morph.
                                     * Code: motor/startup.c (SineInit/SineAlign/SineRamp,
                                     * MorphInit/CheckSectorBoundary/ComputeDuties).
                                     * Sequence: ALIGN (sine hold @ 90°) → OL_RAMP (V/f
                                     * ramp 150→3000 eRPM) → MORPH (sine→trap blend, then
                                     * windowed Hi-Z ZC search) → CL (HWZC + sector PI). */
#define FEATURE_ADC_CMP_ZC       1  /* Phase F: ADC comparator-based high-speed ZC — required for 6-step (2026-05-25). */
#define FEATURE_HW_OVERCURRENT  1  /* Phase G: Hardware overcurrent protection via CMP3+OA3 */
#define FEATURE_OC_AUTOZERO     1  /* 2026-06-10: the OC math assumes a 1.65V/2048-count bus-ADC
                                    * bias, but the chain MEASURABLY rests at ~78 counts — so every
                                    * OC trip (SW + CMP3 DAC) fired ~22A above the configured mA
                                    * (full-code read + bench). Fix: measure the true rest during
                                    * ARMED (bridge off), re-center ibusRaw into the 2048 frame
                                    * (thresholds, window tracking AND host telemetry become true —
                                    * the phantom −21A idle reading disappears), and shift the CMP3
                                    * DAC threshold by the same delta inside HAL_CMP3_SetThreshold.
                                    * Until the first arm completes calibration, bias stays 2048 =
                                    * exactly the legacy behavior (graceful fallback). */
#define OC_AUTOZERO_SAMPLES    64  /* ARMED rest samples (~1.4ms @45kHz) */
#define OC_AUTOZERO_MAX_SPREAD 16  /* max (max−min) ADC counts in the cal
                                    * window (~0.17A). A rotor still spinning
                                    * down puts regen ripple on the bus that
                                    * fails this → retry until quiescent. */
#define FEATURE_PLL_STARTUP     0  /* 2026-06-11 twin design study (task #10): after ALIGN,
                                    * enter CL directly — the sector-PI/SCCP1 machinery runs a
                                    * BLIND accelerating commutation schedule from
                                    * PLL_START_ERPM0, comparator armed the whole way; captures
                                    * are consumed-and-discarded below PLL_START_CAPTURE_FLOOR
                                    * (phantom-proof), counted above it; PLL_START_SYNC_CAPS
                                    * consecutive plausible captures => declared synced and the
                                    * NORMAL sector PI takes over seamlessly (no morph, no OL
                                    * ramp, no lock gate, no hand-off event). AM32-style
                                    * "closed loop with training wheels". Prototype in
                                    * garuda_sil before any bench flash. */
#define PLL_START_ERPM0              300   /* first commanded speed after align */
#define PLL_START_ACCEL_ERPM_PER_S 32000   /* blind schedule acceleration */
#define PLL_START_CAPTURE_FLOOR_ERPM 2500  /* ignore captures below (BEMF noise floor) */
#define PLL_START_SYNC_CAPS            6   /* consecutive plausible captures = synced */

#define FEATURE_AM32_STARTUP    1  /* 2026-06-12 bench experiment: AM32-style "kick + listen".
                                    * NO align, NO ramp, NO blind schedule: on arm-complete,
                                    * one blind commutation at MIN_DUTY from the unknown rotor
                                    * angle, HWZC armed immediately with the period seeded at
                                    * AM32_START_SEED_ERPM, zcSynced trusted from event 1 --
                                    * the normal sector PI + defensive machinery do EVERYTHING
                                    * (AM32 main.c:977 startMotor() semantics; their polling/
                                    * voting low-speed mode maps onto our defensive PI).
                                    * Known risk: phantom captures below the BEMF floor can
                                    * fiction-lock (the VEX failure mode) -- that is what the
                                    * experiment measures. Mutually exclusive w/ PLL_STARTUP. */
#define AM32_START_SEED_ERPM       0   /* initial period guess. 0 = derive from the
                                            * active profile: (2/3)*rampTargetErpm —
                                            * gives the bench-proven 2000 on profile 2
                                            * (rampTarget 3000) and scales for high-BEMF-
                                            * floor motors (VEX: 8000). Nonzero = use as-is. */

#if FEATURE_AM32_STARTUP && FEATURE_PLL_STARTUP
#error "FEATURE_AM32_STARTUP and FEATURE_PLL_STARTUP both own CL entry - pick one"
#endif

#define FEATURE_ARM_BEEP        1  /* 2026-06-12: arm melody through the motor windings.
                                    * Sequence: button -> quiet 500ms arm (OC auto-zero
                                    * calibrates undisturbed) -> ARM_BEEP_MS of music (three
                                    * sequential pitches, 45kHz PWM burst-gated at each note's
                                    * rate, align sector, ARM_BEEP_DUTY_PCT drive; rotor stays
                                    * parked) -> startup. Startup is therefore delayed by
                                    * ARM_BEEP_MS after the normal arm window. */
#define ARM_BEEP_MS          2000  /* melody length (delays startup by this much) */
#define ARM_BEEP_TICKS       (uint32_t)(ARM_BEEP_MS * 10u)
#define ARM_BEEP_FREQ1_HZ     800  /* note 1 (first ~ARM_BEEP_MS/3) */
#define ARM_BEEP_FREQ2_HZ    1200  /* note 2 */
#define ARM_BEEP_FREQ3_HZ    1600  /* note 3 — rising chirp */
#define ARM_BEEP_DUTY_PCT       3  /* drive strength — louder; = profile align duty class. 4+ NOT
                                    * recommended at 24V: burst peaks approach the 18A SW OC. */
#define PLL_START_TARGET_ERPM      10000   /* blind schedule ceiling (hold if unsynced) */
#define FEATURE_SKIP_MORPH      0  /* PARKED 2026-06-10 (bench-proven 9/9 but engage is
                                    * effectively blind at the 3k entry: the post-sine coast
                                    * listen reads a uniform 2× crossing artifact — likely
                                    * 3rd-harmonic/neutral wobble dominating the weak 3k
                                    * fundamental — "5696 eRPM" every run; starts succeed via
                                    * HWZC self-capture, not verified engage. User chose to
                                    * return to the proven morph baseline. Revive for feel:
                                    * single-jerk startup, one climb instead of kick+climb. */
                                   /* ORIGINAL NOTE: skip ESC_MORPH entirely (CW only; needs
                                    * FEATURE_CL_COAST_VERIFY). The morph's job — establish ZC
                                    * lock before CL — is done better by the coast-listen
                                    * measurement at CL entry. New flow: sine OL ramp to 3k →
                                    * bridge cut → coast-listen → engage CL at the MEASURED
                                    * sector/period → single climb to the MIN_DUTY equilibrium.
                                    * Deletes startup jerk #1 (the trap-converge/sub-B kick:
                                    * conventional MIN_DUTY engaging at 3k = ~13A + commutation
                                    * peaks for ~100ms) — one climb instead of kick-pause-climb.
                                    * STARTUP_MorphInit still runs for its rampStepPeriod sync
                                    * (CL entry init + coast timeout fallback depend on it).
                                    * Morph stays compiled as the CCW / flag-off fallback. */
#define FEATURE_CL_COAST_VERIFY 0  /* OFF 2026-06-10 (user call: back to the a491c30 hot
                                    * hand-off; the coast made startup feel 2-stage). The
                                    * machinery is PROVEN (1a: 6/6 measured engages post-morph,
                                    * commit d5d784b) and stays compiled-out for 1b fast
                                    * re-sync / 1c windmill catch, where BEMF is strong and
                                    * the listen is clean. */
                                   /* ORIGINAL NOTE (step 1a of the baseline plan): at morph→CL
                                    * hand-off, coast the bridge 1-2 e-cycles and listen to
                                    * phase B's clean BEMF (no PWM = perfect signal), then enter
                                    * CL with the MEASURED sector + period instead of trusting
                                    * the morph's lock. Coast-listen bench-proven (it exposed
                                    * the morph dragging a slipping rotor at ~900 eRPM while
                                    * claiming 3k lock — those fiction hand-offs are the worst
                                    * slams / failed starts). Engage always uses the measured
                                    * truth (rotor slow → correct-angle drive at its real speed
                                    * beats a wrong-angle slam); listen timeout (~100ms) falls
                                    * back to today's hot hand-off. Conventional waveform only —
                                    * none of the CL_DIFF_IDLE machinery activates. */
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
#define FEATURE_PARAMS_FORCE_DEFAULTS 0 /* Bring-up: 1 = skip the EEPROM overlay so
                                         * compiled profileDefaults[]/.h edits ALWAYS
                                         * win on reflash (NVM save still builds). Set
                                         * back to 0 for normal persistence. See also
                                         * the automatic defaults-signature guard in
                                         * gsp_params.c, which invalidates stale EEPROM
                                         * whenever profileDefaults[MOTOR_PROFILE] changes. */
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
 * 4 = Cobra CM-2814/36 470KV (12N14P 7PP, 4-6S, 0.188Ω pp, 117g — high-R/heavy)
 * 5 = Hobbywing XRotor 3110 1150KV (12N14P 7PP, 4-6S, 0.045Ω pp, 88g — ~2810)
 *
 * All motor-dependent parameters are grouped here for easy swapping.
 * Board-specific and feature-tuning parameters are below.
 *──────────────────────────────────────────────────────────────────────────*/
#define MOTOR_PROFILE  2   /* 2 = 2810 (bench baseline). 4 = Cobra, 5 = XRotor, 6 = VEX 4000KV micro. */

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
#define SINE_RAMP_MODULATION_PCT     5     /* Lowered 8→5 (2026-05-28): bench showed
                                            * 12.1A end-of-ramp + 21.88A MORPH peak
                                            * (shunt saturated). 24V/5%/0.05Ω = 24A
                                            * stall worst-case, much safer for the
                                            * sine→trap transition on this low-Rs motor */
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

#elif MOTOR_PROFILE == 4
/* === Cobra CM-2814/36 470KV (12N14P, 7PP, 4-6S, 117g, 36T delta) ===
 * Rs(pp)=0.188Ω, KV=470, max cont 17A, 24V/6S. Opposite regime to the 2810:
 * ~4× R + heavy rotor + low KV → needs much higher sine amplitude + slow ramp.
 * Low KV = strong BEMF = easy ZC. PHYSICS-BASED STARTING values; iterate from
 * the GSP fault code. NOTE: with FEATURE_GSP=1 these are overridden by
 * profileDefaults[GSP_PROFILE_COBRA] in gsp_params.c — keep the two in sync. */
#define MOTOR_POLE_PAIRS             7
#define DEADTIME_NS                400
#define ALIGN_DUTY_PERCENT           8
#define RAMP_DUTY_PERCENT           12
#define INITIAL_ERPM               100
#define RAMP_TARGET_ERPM          3000
#define MAX_CLOSED_LOOP_ERPM     83000     /* 470 * 24V * 7pp ≈ 79k */
#define RAMP_ACCEL_ERPM_PER_S     1000     /* slow — heavy rotor needs dwell */
#define SINE_ALIGN_MODULATION_PCT   15
#define SINE_RAMP_MODULATION_PCT    30     /* high field: 4× R + heavy + low KV */
#define ZC_DEMAG_DUTY_THRESH        45
#define ZC_DEMAG_BLANK_EXTRA_PERCENT 18
#define HWZC_CROSSOVER_ERPM       1500
#define CL_IDLE_DUTY_PERCENT         8
#define SINE_PHASE_OFFSET_DEG       60
#define OC_LIMIT_MA              20000
#define OC_STARTUP_MA            22000
#define OC_FAULT_MA              21000
#define OC_SW_LIMIT_MA           16000     /* ≈ rated 17A continuous */
#define RAMP_CURRENT_GATE_MA     12000
#define FEATURE_PRESYNC_RAMP       0
#define OC_CLPCI_ENABLE            0

#elif MOTOR_PROFILE == 5
/* === Hobbywing XRotor 3110 1150KV (12N14P, 7PP, 4-6S, 88g) ===
 * Rs(pp)=0.045Ω, KV=1150. Same regime as the 2810 (profile 2): low R, high KV,
 * light rotor — brings up almost identically. Only KV-driven fields differ.
 * NOTE: with FEATURE_GSP=1 these are overridden by
 * profileDefaults[GSP_PROFILE_XROTOR] in gsp_params.c — keep the two in sync. */
#define MOTOR_POLE_PAIRS             7
#define DEADTIME_NS                300
#define ALIGN_DUTY_PERCENT           3
#define RAMP_DUTY_PERCENT            8
#define INITIAL_ERPM               150
#define RAMP_TARGET_ERPM          3000
#define MAX_CLOSED_LOOP_ERPM    210000     /* 1150 * 24V * 7pp ≈ 193k */
#define RAMP_ACCEL_ERPM_PER_S     3000
#define SINE_ALIGN_MODULATION_PCT    3
#define SINE_RAMP_MODULATION_PCT     5
#define ZC_DEMAG_DUTY_THRESH        40
#define ZC_DEMAG_BLANK_EXTRA_PERCENT 20
#define HWZC_CROSSOVER_ERPM       1500
#define CL_IDLE_DUTY_PERCENT         6
#define SINE_PHASE_OFFSET_DEG       60
#define OC_LIMIT_MA              20000
#define OC_STARTUP_MA            22000
#define OC_FAULT_MA              21000
#define OC_SW_LIMIT_MA           18000
#define RAMP_CURRENT_GATE_MA     10000
#define FEATURE_PRESYNC_RAMP       0
#define OC_CLPCI_ENABLE            0

#elif MOTOR_PROFILE == 6
/* === VEX 14mm micro 4000KV (6PP, 7.4V rated / 10V max, ~20g) ===
 * Rs(pp)=0.44Ω, Ld/Lq≈18.4µH(pp), no-load 0.65A, max torque 7.25A, stall 14A.
 * Run at 10V on the MCLV-48V-300W. TWO things make this motor different:
 * (1) 4000KV through 48V-scaled dividers → BEMF at the stock 3k hand-off is
 *     ~6 ADC counts = below the detection floor. Bench-proven 2026-06-11 by
 *     starving the 2810 to the same counts (hand-off 1200): 2/3 starts
 *     fiction-locked then OC'd — the exact reported VEX failure. Hence
 *     RAMP_TARGET 12k / CROSSOVER 6k (only ~2k mech RPM for this KV).
 * (2) stall current is 14A — the 24V-class OC chain (18/20/21A) sits ABOVE
 *     stall and protects nothing; chain scaled to 7.25/9/9.5/10A.
 * NOTE: with FEATURE_GSP=1 these are overridden by
 * profileDefaults[GSP_PROFILE_VEX] in gsp_params.c — keep the two in sync. */
#define MOTOR_POLE_PAIRS             6
#define DEADTIME_NS                300
#define ALIGN_DUTY_PERCENT          25
#define RAMP_DUTY_PERCENT           25
#define INITIAL_ERPM               300
#define RAMP_TARGET_ERPM         12000
#define MAX_CLOSED_LOOP_ERPM    252000     /* 4000 * 10V * 6pp ≈ 240k */
#define RAMP_ACCEL_ERPM_PER_S     8000
#define SINE_ALIGN_MODULATION_PCT   50     /* ~6A against 0.44Ω at 10V */
#define SINE_RAMP_MODULATION_PCT    50
#define ZC_DEMAG_DUTY_THRESH        45
#define ZC_DEMAG_BLANK_EXTRA_PERCENT 18
#define HWZC_CROSSOVER_ERPM       6000
#define CL_IDLE_DUTY_PERCENT        14
#define SINE_PHASE_OFFSET_DEG       60
#define OC_LIMIT_MA               9000
#define OC_STARTUP_MA            10000
#define OC_FAULT_MA               9500
#define OC_SW_LIMIT_MA            7250
#define RAMP_CURRENT_GATE_MA      7000
#define FEATURE_PRESYNC_RAMP       0
#define OC_CLPCI_ENABLE            0

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

/* Proactive high-RPM slew-down limit (2026-05-28).
 *
 * At high eRPM, the standard 5%/ms slew-down rate dumps regen current
 * into the bus fast enough to spike Vbus above the bridge's safe
 * operating range. Bench-observed: rapid 4095→mid-throttle drops at
 * 230k+ eRPM produce Vbus spikes to 29-33V, tripping OC_SW or
 * BOARD_PCI within one telemetry sample.
 *
 * This feature gates the duty-down rate by current eRPM. Above
 * HIGH_RPM_SLEW_THRESHOLD_HR (= ~150k eRPM in HR ticks), the
 * effective rate is divided by HIGH_RPM_SLEW_DIVISOR. Below threshold,
 * normal slew rate applies — no penalty for low-RPM operation.
 *
 * Distinct from FEATURE_VBUS_REGEN_BRAKE: that one is REACTIVE
 * (engages after Vbus already spiked). This one is PROACTIVE
 * (prevents the spike in the first place). Both can be on
 * simultaneously; the more restrictive rate wins. */
#define FEATURE_HIGH_RPM_SLEW_DOWN      1   /* Default ON — bench fix for OC_SW/BOARD_PCI */
#define HIGH_RPM_SLEW_THRESHOLD_HR   6666   /* HR ticks (= 1e9/eRPM_threshold).
                                             * 6666 ≈ 150k eRPM threshold. */
#define HIGH_RPM_SLEW_DIVISOR           8   /* Divide RT_DUTY_SLEW_DOWN_RATE by this
                                             * above the threshold. 5%/ms / 8 = 0.625%/ms,
                                             * giving ~160 ms full-scale slew at top RPM. */
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

/* Emergency Vbus hard-hold tier (2026-05-29).
 *
 * Above the regen brake (which slows the slew-down /16), this tier
 * FREEZES the slew-down entirely. It engages at 30V — 2V above the
 * regen brake's 28V threshold, so the brake gets a chance to do its
 * job first. If Vbus keeps climbing past 30V, the emergency tier
 * kicks in and the duty literally cannot decrease.
 *
 * Purpose: break the regen positive-feedback loop that caused the
 * 47.8V PSU OV trip on bench (session 2026-05-28). The regen brake's
 * /16 slowdown still allowed the bus to climb because deceleration
 * continued — just slowly. Emergency hold prevents any further
 * deceleration, so the rotor stops dumping regen energy into the bus.
 *
 * Slew-UP is NOT frozen — if the user accidentally raises throttle
 * during emergency hold, the rising duty consumes regen energy (good
 * for the bus). Only the regen direction (slew-down) is frozen.
 *
 * Wide hysteresis (30V ON, 27V OFF) prevents chatter.
 * Minimum 20ms hold ensures the regen pulse fully dissipates before
 * the user regains full control. */
#define FEATURE_VBUS_EMERGENCY_HOLD  1
#if FEATURE_VBUS_EMERGENCY_HOLD
#define VBUS_EMERGENCY_HOLD_ON_ADC  1620   /* ~30V — engage hard freeze */
#define VBUS_EMERGENCY_HOLD_OFF_ADC 1450   /* ~27V — release (3V hysteresis) */
#define VBUS_EMERGENCY_HOLD_MIN_TICKS 480  /* Minimum engaged duration ~20ms at 24kHz */
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
#define SINE_TRAP_DUTY_NUM          16  /* Sine->trap duty scale factor numerator.
                                         * Math: sine.amplitude = LOOPTIME × MOD_PCT / 200
                                         * (peak-to-center, not peak-to-peak — easy to miss).
                                         * So with sineRampModPct=5, amplitude = 2.5% LOOPTIME.
                                         * To make trap_duty = 8% LOOPTIME (= clIdleDutyPct),
                                         * need ratio = 8/2.5 = 3.2 = 16/5. This gives MORPH
                                         * displayed duty matching CL idle so the visible step
                                         * 2%→6% goes away. RAMP_DUTY_CAP=8% clamps the result
                                         * — anything higher gets pegged to cap. */
#define SINE_TRAP_DUTY_DEN           5  /* Sine->trap duty scale factor denominator. */

/* Waveform Morph: sine-to-trap transition (replaces coast gap) */
#define MORPH_CONVERGE_SECTORS   12   /* Sectors for duty convergence (was 6 = 1 e-cycle,
                                       * raised 2026-05-28 to 12 = 2 e-cycles for smoother
                                       * sine→trap blend. At 3000 eRPM: 12 sectors × 3.3ms
                                       * = ~40ms blend, halves dV/dt on the float phase). */
#define MORPH_HIZ_MAX_SECTORS    36   /* Max sectors in Hi-Z before fault (6 e-cycles). */
#define MORPH_TIMEOUT_MS       2000   /* Absolute morph timeout (ms). */
#define MORPH_ZC_THRESHOLD        4   /* goodZcCount to exit morph → CL. Lowered from 6
                                       * for easier SW ZC lock during morph. */
#define FEATURE_MORPH_LOCK_GATE   1   /* Stricter morph→CL handoff: in addition to the
                                       * goodZcCount gate, require RT_MORPH_LOCK_ZC_COUNT
                                       * consecutive Hi-Z ZC intervals that are STABLE
                                       * (within RT_MORPH_LOCK_TOL_PCT of the smoothed
                                       * period) and NOT a half-period harmonic, so CL
                                       * engages on a trustworthy angle instead of a noisy
                                       * /phantom lock (the wrong-angle regen at hand-off).
                                       * The MORPH_HIZ_MAX_SECTORS partial-lock fallback
                                       * still applies, so a hard-to-lock motor degrades to
                                       * the old behaviour rather than failing to start. */
#define MORPH_LOCK_ZC_COUNT       4   /* fallback (non-GSP) for RT_MORPH_LOCK_ZC_COUNT */
#define MORPH_LOCK_TOL_PCT       25   /* fallback (non-GSP) for RT_MORPH_LOCK_TOL_PCT */
#endif /* close FEATURE_SINE_STARTUP early — the I-f config below must be top-level
        * (it was trapped in this block, so FEATURE_IF_STARTUP never compiled). */

/* ── I-f current-controlled spin-up (hybrid: I-f start → 6-step run) ──────
 * MILESTONE 1 (this build): replace the voltage-mode sine OL ramp with a
 * CURRENT-controlled SVPWM spin-up so Ibus is bounded (no 22A slam) and the
 * effective voltage can go BELOW MIN_DUTY (SVPWM differential), dissolving the
 * OL→CL speed gap. No 6-step handoff yet — ALIGN → IF_RAMP holds at handoff
 * speed so we can bench-verify the current cap. Current loop reuses the
 * profile's tuned FOC gains (focKpDqMilli / focKiDq). Default OFF. */
#define FEATURE_IF_STARTUP        0   /* PARKED 2026-06-10: op-amp current-sense wall (see memory). Code stays, compiled out. */
#define IF_START_ERPM           800   /* (reserved) initial open-loop eRPM */
#define IF_HANDOFF_ERPM       11000   /* speed I-f ramps to and holds (M1) — just above
                                       * the ~10k MIN_DUTY 6-step idle so it's continuous */
#define IF_ALIGN_MS             150   /* hold forced angle (rotor settles) before ramping */

#if FEATURE_SINE_STARTUP   /* reopen: the Windowed Hi-Z config below is sine-specific */

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
#define HWZC_USE_SW_COMPARE      0   /* Reverted to known-good HW path after the 2026-06-06
                                      * valley-sample PROOF: SW path gave 0% reject (vs 98%) but
                                      * desyncs the 2810 on acceleration (Hurst-tuned). Concept
                                      * validated; needs tuning or speed-adaptive crossover.
                                      * 0 = ADC digital comparator @ 1 MHz (HW path).
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
#define HWZC_NO_CAPTURE_MS      150  /* ms with zero new hwzc.totalZcCount increments while
                                      * zcSynced is true. Catches the failure mode where the
                                      * autonomous SCCP1 timer keeps commutating on a stale
                                      * period but no real BEMF edges arrive (motor physically
                                      * stalled). At 14k eRPM idle, a sector is ~12 ms so this
                                      * tolerates ~12 missed sectors before forcing recovery. */

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
#if GARUDA_TARGET_AK512
/* AK512 (2026-06-13): OFF. This comp gives speed-growing effective advance to
 * cancel the BEMF-divider RC lag — necessary on the AK128 where the threshold
 * was the duty-MODEL neutral (zero lag, so the float's RC lag was uncompensated).
 * But the AK512 runs FEATURE_VIRTUAL_NEUTRAL: the measured (VA+VB+VC)/3 goes
 * through the SAME RC dividers as the floating phase, so the comparison already
 * self-cancels most of that lag. Running both DOUBLE-advances — ω·τ grows with
 * speed → ZC pinned at 67% of sector (scope, vs ~58% expected) → reactive
 * current that climbs with eRPM and exceeds the AK128 at every matched speed
 * (bench PSU, user-confirmed). With VN the comp is redundant; let the
 * timing-advance SCHEDULE be the only intentional advance.
 *
 * UPDATE 2026-06-13b: comp ON over-advanced -> REGEN (Ibus -17A) above ~120k,
 * 213k/UV. ROOT CAUSE: the offset magnitude uses zcAmpForFilterComp, which is a
 * slow-IIR of zcThreshold = the duty-MODEL neutral (vbusRaw*duty/2) -> it scales
 * with DUTY, not with the BEMF swing. At high speed duty is high (76%) so the
 * offset balloons and the detection-advance blows past the RC lag. WORSE, this
 * threshold-advance DOUBLE-COUNTS with the FEATURE_TIMING_ADVANCE schedule
 * (commDelay = stepPeriod*(30-advDeg)/60) — both are speed-proportional advances.
 * Two advances, one of them duty-corrupted, = the regen.
 * FIX: keep the ONE clean mechanism (the timing-advance schedule, a direct
 * angle/time advance, duty-independent) and TUNE it. timingAdvMaxDeg is a live
 * GSP param -> sweep the high-speed advance at the bench with no rebuild.
 *
 * UPDATE 2026-06-13c: comp RE-ENABLED with the fix the note above asked for —
 * it now uses a TRUE MEASURED BEMF amplitude (bemf.bemfDevPeak: peak
 * |float-neutral| per sector, garuda_service.c) instead of zcThreshold. That
 * kills the duty positive-feedback that caused the regen, so the comp is the
 * pure RC-lag (omega*tau) correction at all speeds — the 106's intent, made
 * scale/duty-robust. Keep timingAdvMaxDeg modest (~25) so the comp carries the
 * lag, not a double-counted schedule. */
#define FEATURE_HWZC_FILTER_COMP    1
#else
#define FEATURE_HWZC_FILTER_COMP    1   /* Restored after diagnostic. Comp off vs on
                                         * showed IDENTICAL ~135k miss-onset threshold
                                         * — so the misses aren't comp-caused. But
                                         * comp DOES help current/efficiency by giving
                                         * effective advance to compensate the 24° RC
                                         * filter lag at high RPM. Keep enabled. */
#endif
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

/* Hybrid per-polarity ZC detection (2026-06-07). Bench-proven root cause: the
 * ON-time HW comparator detects RISING sectors (even 0/2/4) perfectly at all
 * speeds but is SILENT on FALLING sectors (odd 1/3/5) above ~20k eRPM — the
 * falling crossing isn't in the ON-time window. Per-sector OFF-center telemetry
 * confirmed the falling ZC IS visible at the OFF-center sample (bemfRaw brackets
 * zcThreshold, swing GROWS with speed 118→1247 counts).
 *
 * When 1: RISING sectors keep the HW comparator (untouched, 232k-proven);
 * FALLING sectors are detected by a software compare of the OFF-center sample
 * (bemfRaw vs zcThreshold) in the 24 kHz ADC ISR, recording the capture into the
 * same sector-PI path (lastCaptureHR/captureValid). This restores real feedback
 * on the falling half (today coasted) — the low-speed-transient robustness win.
 * STATIC per-polarity routing by commutationTable.zcPolarity — NOT a dynamic
 * speed/duty handoff (see akesc_dual_window_zc_deadend). Requires SECTOR_PI.
 *
 * CAVEAT: OFF-center resolution is ~1 PWM period. Fine at low/mid speed (the
 * target regime); at the very top a sector is ~1-2 PWM periods so falling
 * timestamps are coarse — if that jitters the PI vs the precise rising captures,
 * gate falling-SW to engage only below HWZC_FALLING_SW_MAX_ERPM (0 = no gate).
 * Set 0 to revert to rising-only/coast (the proven 232k baseline). */
#define FEATURE_HWZC_FALLING_SW        1   /* falling ZC via SW OFF-center sample */
#if GARUDA_TARGET_AK512
/* AK512 bench 2026-06-12 (polarity-split cap diag, stepped GSP ramp): rising
 * comparator captures sit ~500 permille of T at every speed 30k-92k; falling
 * SW captures walk 547 -> 733 -> 901 permille (max 955) as the sector shrinks
 * toward the 45 kHz sample floor + RC lag. Above ~70k the late falling
 * captures destabilize the PI during accel transients (Ia spikes to the 22 A
 * region -> PSU sag -> UV at ~100k). Cap falling-SW at 70k; rising-only
 * carries the top end (the AK128 ran 234k rising-only above its ceiling). */
#define HWZC_FALLING_SW_MAX_ERPM       70000
#else
#define HWZC_FALLING_SW_MAX_ERPM       0   /* 0 = falling-SW at all speeds; else cap */
#endif

/* ── Virtual neutral (AK512 only) ──────────────────────────────────────────
 * The MC510 port samples ALL THREE phase voltages every PWM cycle from the
 * same PG1TRIGA instant (VA=AD1CH4, VB=AD1CH3, VC=AD2CH3). zcThreshold then
 * becomes the MEASURED neutral (VA+VB+VC)/3 instead of the duty*Vbus/2 model.
 * Math: at the sample instant driven phases read ~Vbus and ~0, so measured
 * neutral = Vbus/2 + e_float/3 — the float BEMF leak scales the detection
 * signal by 2/3 but the crossing fires EXACTLY at e_float = 0. Every model
 * error (deadtime, Vbus ripple, divider tolerance, duty model) is measured
 * in, not estimated. Bench motivation 2026-06-12: scope showed the model
 * threshold puts rising crossings at 78% of sector vs falling 67% (ideal
 * ~65% incl. RC lag) -> asymmetric over-advance -> 22 A at speed. Triplen
 * (3x f) neutral ripple is expected and smoothed by the zcThreshold IIR. */
/* A/B TEST 2026-06-13: VN OFF. The (VA+VB+VC)/3 neutral above ASSUMED 3
 * simultaneous independent ADC samples, but VA=AD1CH4 and VB=AD1CH3 BOTH live
 * on AD1 -> they convert SEQUENTIALLY (~0.3-1us skew), so the neutral is not a
 * simultaneous snapshot; the skew->threshold-error grows at high duty (sample
 * near the fast switching edges) = the same high-speed regime that fails.
 * Reverting to the duty*Vbus/2 model (the clean, noise-free reference the 106
 * used to reach 232k) to isolate whether VN is contributing to the high-duty
 * current climb. If high-speed improves -> VN's skewed neutral was a real term. */
#define FEATURE_VIRTUAL_NEUTRAL        0
#if 0  /* prior AK512 default (measured neutral) — re-enable to compare */
#define FEATURE_VIRTUAL_NEUTRAL        1
#endif
/* Falling-polarity ZC fix (2026-06-07, investigation closed). Root cause
 * (measured): the ON-time HW comparator detects RISING perfectly but is silent
 * on FALLING above ~20k — during PWM-ON the driven phase couples into the
 * floating phase and biases it up, assisting the rising crossing but swamping
 * the falling one. Clean falling BEMF only exists in the PWM-OFF/freewheel
 * window, captured by the RC-filtered OFF-center sample (bemfRaw). FEATURE_
 * HWZC_FALLING_SW detects falling there in the ADC ISR, into the same sector-PI
 * path, restoring falling feedback idle→~90k (the low-speed-transient win); it
 * coasts above ~90k where the 24kHz sample rate runs out (rising untouched →
 * 232k unaffected). Set 0 to revert to rising-only/coast (baseline).
 * Dead-ends tried & dropped: (1) symmetric filter-comp offset — refuted (falling
 * is silent, not mis-thresholded); (2) per-polarity comparator OFF-window gate —
 * underperformed (~50k: the 1 MHz comparator sees the UNFILTERED freewheel/edge
 * ring; the RC-filtered period-boundary sample is cleaner despite 24kHz). */

/* --- Phase 1 of the OL->CL smooth-handoff plan (2026-06-07) ---------------
 * Low-speed floating-phase ZC for BOTH polarities. The committed FALLING_SW
 * already detects falling on the RC-filtered OFF-center (PWM-OFF) sample; this
 * extends the SAME mechanism to RISING below a crossover eRPM, so that at low
 * speed neither polarity needs the PWM-ON comparator window. That removes the
 * reason for the ~6% duty floor (PWM-ON ZC detectability) and is the enabler
 * for a low-duty, current-bounded hand-off (Phase 2). Above the crossover,
 * rising keeps using the HW comparator unchanged (232k path untouched).
 * Reference basis: AM32/ESCape32 both detect ZC on the floating phase during
 * PWM-OFF (see akesc_ol_cl_reference_study). Default OFF — additive (runs
 * alongside the rising comparator; first capture per sector wins via
 * captureValid), so enabling it cannot remove existing detection, only add a
 * PWM-OFF rising path. Gate: bench at idle, confirm lock holds + rising
 * OFF-center captures appear, before Phase 2 lowers the duty. */
#define FEATURE_HWZC_LOWSPD_OFFCTR     1   /* rising ZC via SW OFF-center at low speed */
#define HWZC_LOWSPD_OFFCTR_MAX_ERPM 40000  /* rising OFF-center engages below this eRPM */

/* Hand-off period-collapse damp (OL->CL smooth plan, 2026-06-07). For the first
 * HWZC_HANDOFF_DAMP_EVENTS commutations after CL entry, tightly clamp how fast the
 * PI may SHRINK the period (negative delta), so a single too-early first capture
 * can't collapse timerPeriod to the half-period phantom (observed 2/3 of starts at
 * a 5k hand-off: first CL frame read 2x eRPM, then ran away to ~150k). Legit
 * acceleration is gradual and survives the damped rate; the phantom (sudden 2x in
 * a few events) is blocked. Releases after the window. Purely protective — only
 * limits period DECREASE during the entry window. Default ON. */
#define FEATURE_HWZC_HANDOFF_DAMP       1   /* clamp period-shrink rate at CL entry */
#define HWZC_HANDOFF_DAMP_EVENTS       24   /* # commutations the damp stays active */
#define HWZC_HANDOFF_NEG_SHIFT          6   /* period may shrink <= T>>this per event in window */

/* Lower the CL idle duty floor below MIN_DUTY (=2xdeadtime) to drop the idle
 * equilibrium and shrink the OL->CL gap/pulse (2026-06-07). SAFE: the PWM
 * peripheral still inserts the full deadtime (shoot-through guard unchanged) —
 * only the requested H-pulse gets shorter. Floor = deadtime x CL_LOW_IDLE_DT_PCT
 * /100. Lower = slower idle + smaller pulse, until the H-pulse is too short to
 * drive the FET cleanly and the motor won't hold (graceful: just back off). Only
 * touches the CL idle floor; commutation/morph/OC still use MIN_DUTY. Set
 * clIdleDutyPct at/below this so the floor governs. Default OFF.
 * BENCH RESULT 2026-06-07 (DEAD END): with the sag-block re-floor fixed to
 * respect this, idle duty did drop 5%->4% — but idle eRPM stayed ~10.5k and the
 * hand-off pulse stayed ~19A (idle Ia ripple slightly worse). Root: MIN_DUTY is
 * baked into the trap commutation waveform (low phase + virtual neutral), so the
 * effective differential drive barely changes. The deadtime floor is structural
 * in the waveform, not just this clamp. Left OFF (=0): no benefit. */
#define FEATURE_CL_LOW_IDLE             0   /* allow CL idle below MIN_DUTY (dead end, see above) */
#define CL_LOW_IDLE_DT_PCT            150   /* idle floor as % of one deadtime (150=1.5xDT) */

/* ── Differential-low CL idle (2026-06-10) ────────────────────────────────
 * Fixes what CL_LOW_IDLE couldn't: the MIN_DUTY voltage floor is baked into
 * the trap WAVEFORM (low phase = override-LOW → min line-line volts =
 * MIN_DUTY×Vbus ≈ 1.3V → idle equilibrium ~10k → the 3k→10k hand-off slam).
 * In CL below MIN_DUTY effective duty, drive the LOW phase COMPLEMENTARY at
 * MIN_DUTY (a PDC write, overrides released) — the same differential drive
 * MORPH_CONVERGE uses (bench: 0.78A coast). Line-line volts become
 * (duty − base) — controllable from ~0V, so CL can idle at ~3-4k and MEET the
 * sine/morph hand-off speed. garudaData.duty keeps meaning EFFECTIVE duty
 * everywhere; at any duty ≥ MIN_DUTY both waveforms produce identical volts,
 * so the swap to the proven conventional drive is seamless (hysteresis below).
 * Prop-safe: continuous drive every cycle, full torque authority (this is
 * NOT pulse skipping). Float phase / ZC handling unchanged; the duty-
 * proportional zcThreshold is floored at its proven MIN_DUTY level. */
#define FEATURE_CL_DIFF_IDLE            0   /* PARKED 2026-06-10 (steady-state diff idle: BEMF-quality
                                             * floor ~8-9k makes it marginal vs the 10.3k baseline;
                                             * see memory). Briefly revived as machinery for
                                             * FEATURE_CL_ENTRY_GLIDE — refuted same day, see below. */
#define CL_DIFF_IDLE_PCT_X10           34   /* idle floor: EFFECTIVE duty, 0.1% units (34 = 3.4%
                                             * ≈ 0.82V → idle ~8-9k on the 2810: the bench-measured
                                             * smooth zone. BENCH 2026-06-10 ladder: 1.0% → ~2.5k,
                                             * BEMF below threshold → desync. 2.2% → ~6.7k LOCKED but
                                             * ROUGH (eRPM sd≈1400: BEMF slope too shallow → ZC
                                             * jitter; user-confirmed — tiny pot lift = instantly
                                             * smooth). 7-8% duty (16-18k): sd≈400-700 = smooth.
                                             * Floor sits at the BEMF-quality limit, still below
                                             * the ~10-11k MIN_DUTY wall. */
#define CL_DIFF_EXIT_HYST_DIV           4   /* swap to conventional drive at duty ≥ MIN_DUTY +
                                             * MIN_DUTY/DIV; re-enter diff below MIN_DUTY */

#define FEATURE_CL_ENTRY_GLIDE          0   /* REFUTED ON BENCH 2026-06-10 — keep 0. Idea: coast
                                             * engage starts the DIFF waveform at speed-matched
                                             * volts, ramps to MIN_DUTY over ~300ms → linear climb.
                                             * Reality: the glide starts at ~5.5k, BELOW the diff
                                             * BEMF-quality floor (~8-9k, bench ladder) → SW ZC
                                             * never locked (rej=0%, deadline-forced eRPM crawl)
                                             * → open-loop drag at Ia 22A SATURATED for ~450ms
                                             * (more charge than the 250ms ballistic it replaced),
                                             * exits randomly: OC_SW fault, or phantom 18.7k lock
                                             * at 20A. Matched volts didn't stay matched: the
                                             * +2×MIN_DUTY deadtime comp is calibrated near zero
                                             * current; at real current the DT error flips →
                                             * overdrive → positive feedback to the OC limiter.
                                             * Also shifted idle 10.4k→12.6k (pinned-floor mapping
                                             * adds ~1% duty). Don't retry without a ZC source
                                             * that works below 8k in diff mode. */
#define CL_GLIDE_EQ_ERPM            10400u  /* bench-measured MIN_DUTY equilibrium speed (2810@24V).
                                             * Matched engage duty = MIN_DUTY × erpm_meas / this. */
#define CL_GLIDE_DIV                    8   /* glide ramp rate: +1 duty tick per N ADC ISR ticks.
                                             * 8 → ~0.47×MIN_DUTY span in ~320ms @45kHz. */

/* Float-port of the sector PI (Phase 1 of pi_controller_research.md).
 * When 1, HWZC_OnPiPeriodExpired runs the same algorithm in float instead
 * of integer bit-shifts. Allows non-power-of-2 gains. With HWZC_PI_FF
 * also enabled, adds physics feedforward (Phase 2) — open-loop period
 * estimate from KV / Vbus / duty so the integrator only carries the
 * residual error. Sim shows ~5× tighter steady-state and ~2× faster lock.
 *
 * Both paths still maintain the integer integrator/timerPeriod fields
 * so existing telemetry and the reactive fallback continue to work. */
#define FEATURE_HWZC_PI_FLOAT          1   /* 1 = float PI (bench-validated 2026-05-28) */
#define HWZC_PI_FF_ENABLE              0   /* Disabled. Per-iteration FF stalled the
                                            * motor at CL entry (predicted no-load
                                            * period while rotor was still at 6k eRPM
                                            * from SW ramp). Needs to be re-engineered
                                            * as an in-CL-only adjustment, not a
                                            * handoff seed. See bench session
                                            * 2026-05-28 in chat history. */
/* PI gain specification — PLL form (2026-05-29).
 *
 * Express gains in terms of natural frequency ω_n and damping ratio ζ
 * of the equivalent 2nd-order tracking loop. Easier to retune per motor:
 *
 *   - Raise ω_n → faster tracking, more sensitivity to capture jitter
 *   - Raise ζ   → more damping (less overshoot, slower settle)
 *
 * Standard relationships for the discrete tracking-PI we use:
 *
 *   Kp = 2ζω_n           ← proportional (transient response)
 *   Ki = ω_n²            ← integral     (steady-state tracking)
 *
 * ω_n is expressed in rad per sample-period (one PI sector event), so
 * the absolute Hz bandwidth scales with rotor speed:
 *   f_n(Hz) = ω_n × sector_rate / (2π)
 *   At 14k eRPM (sector rate 1400Hz): f_n =  ~56 Hz
 *   At 230k eRPM (sector rate 23kHz): f_n = ~915 Hz
 * This is a desirable property for sensorless motor tracking — the loop
 * naturally widens its bandwidth when the rotor is fast and tightens it
 * when slow, matching the rotor's dynamic timescale.
 *
 * Bench-validated combination (= original Kp=0.25, Ki=0.0625):
 *   ω_n = 0.25  (slow tracking)
 *   ζ   = 0.5   (slightly underdamped — small overshoot, fast settle)
 *
 * Reverse-derive from any old Kp/Ki pair:
 *   ω_n = sqrt(Ki)
 *   ζ   = Kp / (2 × sqrt(Ki))
 */
#define HWZC_PI_OMEGA_N                0.25f
#define HWZC_PI_DAMPING_RATIO          0.5f

/* Derived gains — compile-time constant, no runtime cost. */
#define HWZC_PI_KP_FLOAT \
    (2.0f * HWZC_PI_DAMPING_RATIO * HWZC_PI_OMEGA_N)
#define HWZC_PI_KI_FLOAT \
    (HWZC_PI_OMEGA_N * HWZC_PI_OMEGA_N)

#define HWZC_PI_FF_RESIDUAL_FRAC       0.20f  /* Integrator anti-windup band:
                                                 * residual clamped to ±20% of P_ff */

/* Defensive PI (Phase 2 candidate, 2026-05-28). When captures go silent
 * (true silence — no capture event at all, not just rejected capValue),
 * relax the controller's commutation rate instead of freezing at the
 * stale period. Walks integratorF larger by 1% per event when defensive.
 * Exits when 2 consecutive good captures resume.
 *
 * Designed to replace the HWZC_NO_CAPTURE_TICKS watchdog's hard
 * ESC_RECOVERY behavior with a soft re-sync. Requires
 * FEATURE_HWZC_PI_FLOAT=1 (uses integratorF).
 *
 * Sim results (commit f7e81f4): byte-identical to baseline on normal
 * scenarios (0 events triggered), 5-12× lower RMS error in stress
 * scenarios where baseline crashes. NOT YET BENCH-TESTED. */
#define FEATURE_HWZC_PI_DEFENSIVE      1    /* 0 = disabled (default) */
#define HWZC_PI_DEFENSIVE_TRIGGER      6    /* miss streak (sectors of true silence) to enter */
#define HWZC_PI_DEFENSIVE_EXIT         2    /* good streak (consecutive captures) to exit */
#define HWZC_PI_DEFENSIVE_GROW_PCT     1    /* walk T by this % per event when defensive */

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

/* ── Absolute, operating-point-aware period floor (PLL anti-harmonic-lock) ──
 * The relative gates above (70% interval, ±T/N delta clamp) bound the per-STEP
 * change but NOT cumulative drift, so a run of slightly-short captures can
 * ratchet the float PLL onto a noise harmonic — the pot-zero "phantom": the
 * controller spins the commutation fast (e.g. 150k+) while the rotor has
 * coasted to idle. Literature calls this PLL false/harmonic lock; the standard
 * cure is an ABSOLUTE plausibility bound in ADDITION to the relative gates.
 *
 * This clamps the commanded period so it can't imply a speed faster than the
 * no-load physics allows for the present duty/Vbus (× a margin). Motor-agnostic:
 * reuses the SAME no-load-period formula already used for the FF seed,
 *   P_ff = 181.380 * lambda / (Vbus * dutyFrac)   [HR ticks],
 * lambda = gspParams.focKeUvSRad. Floor = P_ff * 100/OVERSPEED_PCT (shorter
 * period = faster, so dividing by the margin sets the speed ceiling). Only
 * bites when the PLL commands > OVERSPEED_PCT% of no-load — i.e. the phantom;
 * legitimate operation (≈ no-load at full duty) sits above the floor untouched.
 * Validated offline in tools/garuda_debug/garuda_gui/pisim.py. Needs valid lambda. */
#define FEATURE_HWZC_ABS_FLOOR          1
#define HWZC_ABS_FLOOR_OVERSPEED_PCT  130   /* allow eRPM up to this % of no-load */
#define HWZC_ABS_FLOOR_MIN_DUTYFRAC  0.03f  /* skip below this duty (P_ff invalid) */
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

/* Per-polarity filter-comp scale for the FALLING-ZC branch (odd sectors 1/3/5).
 *
 * 2026-06-06 root cause: per-sector miss telemetry proved the rising-ZC sectors
 * (even 0/2/4) NEVER miss at any speed, while the falling-ZC sectors (odd 1/3/5)
 * carry 100% of the guesses, speed-progressively (clean at idle → fully masked
 * ≥90k eRPM, a perfect 50% measured). The two polarities sit under opposite
 * freewheel/demag conditions, so the SYMMETRIC offset (rising thresh−offset,
 * falling thresh+offset) over-shifts falling: as ω grows the falling CMPLO is
 * pushed toward 4095 until the downward crossing can't register.
 *
 * This scales ONLY the falling-branch offset (rising still uses
 * HWZC_FILTER_AMP_PCT). 0 = falling filter-comp OFF (first bench point — confirm
 * via tools/analyze_sectors.py that odd-sector misses collapse). If falling
 * recovers but tops out below rising, sweep this UP (e.g. 15, 25) toward the
 * sweet spot. If misses DON'T collapse at 0, the masking is physical (ON-time
 * window can't see the falling crossing at speed) → per-polarity SAMPLE POINT,
 * not a threshold tweak. Rising at AMP_PCT=50 is bench-proven to 232k — leave it.
 *
 * 2026-06-06 BENCH RESULT: set to 0 (falling comp OFF) — per-sector telemetry
 * showed ZERO change (odd sectors still 100% of guesses, still 50% measured
 * ≥90k). This REFUTED the offset hypothesis. Reject count (~650) ≪ falling
 * misses (10,223) at ≥90k proved the falling sectors are SILENT (comparator
 * never crosses), not firing-and-rejected → pure visibility problem. Restored
 * to 50 (symmetric, identical to the proven 232k build). The knob stays for the
 * eventual per-polarity SAMPLE-POINT fix; filter-comp is NOT that fix. */
#define HWZC_FILTER_AMP_PCT_FALLING 50
#endif

/* Speed PI — per-ZC interval-based speed PID (Phase 2 of CLAUDE.md roadmap).
 *
 * v2 architecture (2026-05-29, after bench data analysis):
 *
 *   throttle → target_eRPM (linear)
 *        │
 *        ├─→ duty_ff = K_ff × target_eRPM + offset  (covers ~95% of duty)
 *        │
 *        └─→ error_eRPM = target − measured  (in eRPM units)
 *                │
 *                ├─→ deadband (fractional, kills HWZC capture noise)
 *                │
 *                └─→ PI (gains operate on RESIDUAL eRPM error)
 *                          │
 *                          ▼
 *           output_duty = duty_ff + PI_correction  (clamped CL_IDLE..MAX)
 *
 * Why eRPM, not period?
 *   - Plant gain in (eRPM, duty_ticks) is constant: 3.2 eRPM/tick across the
 *     14k–230k eRPM range. Constant-gain plant + constant-gain PI = uniform
 *     loop dynamics at every operating point.
 *   - Plant gain in (period, duty) varies as 1/ω² — 16× difference between
 *     idle and top. PI tuned for one extreme blows up at the other.
 *   - Costs one FPU division per PI event to compute measured_eRPM from
 *     stepPeriodHR. Negligible on dsPIC33AK FPU.
 *
 * Why feedforward?
 *   - Without it, integrator alone has to build up to operating duty (6-89%).
 *     At idle's slow event rate, this takes seconds.
 *   - Linear duty(eRPM) fit from bench data covers 95%+ of the steady-state
 *     duty. PI only has to trim the residual ±1-2%. Much faster convergence.
 *
 * Why deadband?
 *   - HWZC capture noise is bimodal: 3-4% in the mid range, 10% at extremes.
 *   - Per-event Kp × noise causes the PI to chase its own noise → limit cycle.
 *   - 3% deadband on the eRPM error suppresses noise hunting while leaving
 *     real load disturbances (>3% deviation) free to drive the loop.
 *
 * Calibration (re-measure for a different motor or supply voltage):
 *   - SPEED_PI_FF_SLOPE/OFFSET: fit duty(eRPM) from a bench sweep
 *   - SPEED_PI_TARGET_ERPM_IDLE/MAX: choose throttle endpoints
 *   - Gains tuned for plant K=3.2 eRPM/tick (which holds across speeds)
 *
 * Default ON (FEATURE_SPEED_PI=1).
 */
#define FEATURE_SPEED_PI                  0     /* DISABLED 2026-05-29 — v2 architecture
                                                   * (eRPM control + FF + deadband + tight
                                                   * integrator clamp) bench-validated at
                                                   * IDLE/MID but triggers PSU OV cascade
                                                   * at high RPM (>80k). Per-event duty
                                                   * wobble + regen sensitivity = fault.
                                                   *
                                                   * Code retained for future iteration:
                                                   *   - Add output slew limiter (max ±200
                                                   *     PWM ticks/event regardless of PI)
                                                   *   - Freeze integral above HIGH_RPM
                                                   *   - Vbus-dip detection to fold back PI
                                                   * Multi-session integration with the
                                                   * existing Vbus protection stack. */

#if FEATURE_SPEED_PI

/* Throttle → target eRPM mapping. Linear from idle to max.
 *
 * MAX lowered from 230k to 200k (2026-05-29) — the motor's actual
 * BEMF ceiling at 24V is ~200k. Commanding 230k creates persistent
 * positive error at top throttle → integrator winds up → catastrophic
 * transient on throttle changes. 200k is reachable, leaves margin. */
#define SPEED_PI_TARGET_ERPM_IDLE         14000u   /* at throttle = 0 */
#define SPEED_PI_TARGET_ERPM_MAX          200000u  /* at throttle = 4095 */

/* eRPM ↔ stepPeriodHR conversion. HR clock = 100 MHz (10 ns/tick). */
#define SPEED_PI_ERPM_FROM_TICKS          1000000000UL

/* Linear feedforward: duty_ticks ≈ FF_SLOPE × eRPM + FF_OFFSET
 *
 * Bench-fit values for 2810 1350KV at 24V, MIN_DUTY=3840 (Phase E,
 * 2026-05-29 session step6_20260529_140654). At eRPM=14k → 4970 ticks;
 * at 200k → 61700 ticks. Within ±2% of bench across the full sweep.
 *
 * To recalibrate on a different motor:
 *   1. Run a step6_session.py with FEATURE_SPEED_PI=0 (direct duty)
 *   2. Sweep throttle slowly across the full range
 *   3. Filter samples to steady-state (e.g., |spi_error| < 1000)
 *   4. Linear regression on (eRPM, output_duty)
 *   5. Plug in slope and offset here, rebuild
 */
#define SPEED_PI_FF_SLOPE                 0.310f  /* +1.6% — bench v2 run showed
                                                   * corr drifting to +600 at top.
                                                   * Slightly steeper slope absorbs it. */
#define SPEED_PI_FF_OFFSET                400.0f  /* Lowered from 700 (bench v2 fit). */

/* Throttle deadband: ADC counts below this map to target=IDLE eRPM.
 * Above this, linear interpolation up to MAX. Matches the arming
 * logic's ARM_THROTTLE_ZERO_ADC threshold (typically 200). */
#define SPEED_PI_THROTTLE_DEADBAND        200u

/* Deadband: ignore eRPM errors smaller than this fraction of target.
 * Raised from 3% to 5% (2026-05-29) — bench v2 showed motor genuinely
 * oscillates ±4-5% at mid-range as PI chases capture noise that partially
 * exceeds 3%. 5% silences the PI when motor is "close enough" to target. */
#define SPEED_PI_DEADBAND_PCT             5u

/* PI gains operating on RESIDUAL eRPM error (= remaining after FF).
 *
 * Plant gain K_plant = 3.2 eRPM/tick is constant across speeds (bench
 * data 2026-05-29). For a unity-loop-gain proportional response:
 *   Kp_max = 1/K_plant = 0.31 PWM tick per eRPM
 *
 * Lowered Kp 0.1 → 0.03 (2026-05-29) — bench v2 showed per-event Kp×err
 * was driving visible duty oscillation at mid-range. Smaller Kp means
 * each capture contributes less to output change → smoother feel.
 * Trade-off: slightly slower transient response (~300ms vs ~100ms).
 *
 * Ki kept at 0.001 — integrator does the heavy lifting now. */
#define SPEED_PI_KP_FLOAT                 0.03f
#define SPEED_PI_KI_FLOAT                 0.001f

/* Integrator clamp — TIGHTER than the v1 ±max_integ.
 * Bench v2 ran integrator to −12700 ticks (−18% LOOPTIME) over 50s at
 * high RPM because FF overshoots actual duty there. When throttle then
 * stepped, the recovery to 0 caused a +18% duty jump → OC_SW fault.
 *
 * ±8000 ticks (= ±11% LOOPTIME) allows compensating for normal FF
 * inaccuracy without enabling catastrophic transients. */
#define SPEED_PI_INTEGRATOR_CLAMP         8000.0f

/* Integral disable window: ZCs after CL entry during which I-term is
 * frozen (only FF + P-term active). Lets the system settle without
 * integral windup during the CL-entry transient (rotor accelerating
 * from sine ramp endpoint to operating point). */
#define SPEED_PI_INTEGRAL_DISABLE_ZCS     100

/* Back-calculation anti-windup gain. When PI output saturates, pull
 * integrator back by K_aw × (clamped − unclamped) so the next tick
 * doesn't have a runaway I-term to dig out of. 1/Kp is the standard. */
#define SPEED_PI_AW_KBC                   (1.0f / SPEED_PI_KP_FLOAT)

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
