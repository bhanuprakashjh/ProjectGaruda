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

/* ════════════════════════════════════════════════════════════════════
 * ORIENTATION (2026-07-04 config-clarity pass)
 *   WHICH MOTOR?          -> garuda_motors.h  (MOTOR_PROFILE + all motor data)
 *   6-STEP ENGINE         -> this file: feature flags below, then HWZC/ZC
 *                            detection, protection, startup(sine|classic),
 *                            duty pipeline, GSP/telemetry sections.
 *   FOC / learn / x2cscope-> legacy, all flags 0, scheduled for extraction
 *                            (kept only because hwzc.c/commutation.c still
 *                            include two of their headers - see audit doc).
 *   Startup types are exactly TWO: FEATURE_SINE_STARTUP 1=sine/morph,
 *   0=classic align->ramp->direct CL (profile override in garuda_motors.h
 *   region of this file, line ~360).
 * ════════════════════════════════════════════════════════════════════ */

/* Feature Flags (0=disabled, 1=enabled) */
#define FEATURE_BEMF_CLOSED_LOOP 1  /* Phase 2: BEMF ZC detection — ENABLED for 6-step (2026-05-25) */
#define FEATURE_VBUS_FAULT       1  /* Phase A4: Bus voltage OV/UV fault enforcement */
#define FEATURE_DESYNC_RECOVERY  1  /* Phase B2: Controlled restart-on-desync (ESC_RECOVERY) */
#define FEATURE_DUTY_SLEW        1  /* Phase B1: Asymmetric duty slew rate limiter */
#define FEATURE_CL_SOFT_ENTRY    1  /* ENABLED 2026-07-03 (U3 startup smoothing): gentle duty ramp at CL entry.
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
#define FEATURE_SPINDOWN_FLOOR (MOTOR_PROFILE == 9)
                                    /* Spin-down BEMF duty floor (2026-07-03, 20V pot-0 desync).
                                     * A pot-0 from cruise drops duty full-scale in ~4ms
                                     * (DUTY_SLEW_DOWN 5%/ms; the HIGH_RPM penalty only bites
                                     * >150k eRPM) while the rotor needs ~1s. With duty far
                                     * below BEMF the freewheel diodes clamp the floating
                                     * phase: NO valid ZC exists, rej=100%, the tracker coasts,
                                     * the rotor decelerates through the coasted rate and the
                                     * drive phantom-locks (bench 20V: eRPM frozen 8.7k, -6.8A
                                     * bus / 11A phase sustained; hides below stall 14A and
                                     * the 40% stall duty gate). Fix: never command duty more
                                     * than a margin below the BEMF-equivalent duty of the
                                     * MEASURED eRPM. The floor descends exactly as fast as
                                     * the rotor actually slows, so captures stay alive all
                                     * the way down (mirrors the manual slow-pot-down that
                                     * always worked). Protections (sag, SW OC limiter) run
                                     * AFTER this clamp and can still cut below it. */
#define SPINDOWN_FLOOR_KV_NUM  107  /* dutyPct*100 = KV_NUM * eRPM / vbusRaw. Bench-fit on U3
                                     * KV700/7pp, 5 steady points 4.7k..82k eRPM, 16-20V:
                                     * duty%*Vbus/eRPM = 0.0202..0.0210 (=1/Kv_e with the
                                     * 23.2 Vbus divider; theory 100/(700*7)/0.018694V-per-
                                     * count = 109). Per-motor constant. */
#define SPINDOWN_FLOOR_MARGIN_PCT_X10 30
                                    /* Brake margin below BEMF-equivalent duty, %*10 (30 =
                                     * 3.0%). Bigger = faster spin-down but more circulating
                                     * regen current and a longer diode-clamp fraction of the
                                     * PWM cycle; smaller = gentler. 3% of 20V ≈ the ~1-2A
                                     * brake the proven manual pot-downs showed. Bench-tune. */
#define SPINDOWN_FLOOR_KV_DROOP_ERPM 14000
                                    /* LATCH FIX (bench 2026-07-03): at 81k/98% the flat
                                     * KV_NUM=107 model read equiv ~102% — above the real
                                     * sustaining duty (98%) — so floor = equiv-3% >= 98%
                                     * HELD full duty at pot 0 indefinitely (self-sustaining:
                                     * the floor maintains the speed that justifies the
                                     * floor; only STOP recovered). Cause: timing advance
                                     * (0->25 deg with speed) field-weakens — more eRPM per
                                     * volt — so the low-speed-fitted constant overestimates
                                     * at the top by ~5%. Model the droop linearly:
                                     * kvNum = KV_NUM - erpm/THIS (81k -> 107-5.8 = 101,
                                     * matching the measured top-end c). */
#define SPINDOWN_FLOOR_KV_MIN    90 /* droop clamp (sanity for erpm beyond the fit range) */
#define SPINDOWN_FLOOR_MAX_PCT_X100 9400
                                    /* Hard cap, %*100: the floor may NEVER command more
                                     * than 94% duty regardless of model output. Guarantees
                                     * the pot can always pull duty below the sustaining
                                     * point at the top end even if the model drifts (worst
                                     * case: pot-0 from full speed starts with a ~4% brake
                                     * gap until eRPM falls under the cap region, then the
                                     * normal 3% glide takes over). Belt-and-braces vs the
                                     * droop term above — either alone breaks the latch. */

/* ── CL-ENTRY SOFT-START ──────────────────────────────────────────────────
 * At CL entry the duty normally steps straight to the idle floor (e.g. 8%)
 * while BEMF is ~0, so the phase current slams to its inrush peak. This ramps
 * the duty up from CL_ENTRY_START_PCT to the idle floor over CL_ENTRY_RAMP_MS,
 * so the current builds GRADUALLY as the rotor speeds up and BEMF rises ->
 * smaller inrush PEAK + longer/gentler ramp, with the SAME final idle speed
 * (unlike lowering CL idle duty, which also lowers idle RPM). Unlike the CMP3
 * chop, this limits the actual applied voltage so it cuts the inductive current
 * the chop can't. Tune: START_PCT lower = smaller peak, but must stay above the
 * entry-speed equilibrium (~2k handoff) or the rotor coasts/desyncs -> raise it;
 * RAMP_MS longer = gentler. START_PCT must be >= the deadtime MIN_DUTY (~3%). */

/* ── Pot start/stop (2026-06-18) ──────────────────────────────────────────
 * FEATURE_POT_START_STOP=1: zero pot = motor STOPPED (bridge off, coasts to
 * standstill); raise the pot past THROTTLE_START_ADC = motor starts. RC-ESC
 * model with an arm-at-zero safety: the motor only launches after the pot has
 * been seen at/near zero (ARMED "ready"), so a pot left up at power-on won't
 * spin until it's cycled to zero first. Pulls in the existing throttle-zero
 * auto-disarm (the stop half) + adds start-on-pot-raise + auto-re-arm.
 * NOTE ("as is"): the start still runs the align/OL/morph sequence, so the
 * motor JUMPS to the ~10.5k CL idle when the pot crosses the threshold, then
 * tracks the pot up — not proportional from 0. Smooth-from-standstill is the
 * follow-on soft-start current-control phase. Set 0 to revert to idle-floor. */
#define FEATURE_POT_START_STOP   0   /* DISABLED 2026-06-24: pot does not start/stop the motor
                                      * (and FEATURE_THROTTLE_ZERO_AUTO_DISARM follows it → off).
                                      * Arm/start via GSP/switch; pot is throttle only. */
#define THROTTLE_START_ADC      400   /* armed motor launches when pot ADC rises above this (hysteresis vs ARM_THROTTLE_ZERO_ADC=200) */
#define FEATURE_TIMING_ADVANCE   1  /* Phase B3: Linear timing advance by RPM — RE-ENABLED 2026-05-26 to compensate detection-chain latency at high RPM. Original baseline schedule: 0° below 3k eRPM, linear ramp to 22° at MAX_CLOSED_LOOP_ERPM (70k for 2810), clamped 22° above. */
#define FEATURE_DYNAMIC_BLANKING 1  /* Phase C1: Speed+duty-aware blanking (extra blank at high duty/demag) */
#define FEATURE_ZC_CURRENT_BLANK 1  /* WS1: LOAD-adaptive demag blanking on the HW-ZC path. Adds extra
                                     * blanking proportional to bus current above a deadband — the
                                     * freewheel demag (diode-conduction) interval grows with load
                                     * current and masks BEMF at sector start → ZC window collapses →
                                     * desync (the U3 prop-load failure). The existing
                                     * zcDemagBlankExtraPct only touches the SW/integration paths;
                                     * U3 runs the HW comparator path (hwzc.c), so the current term is
                                     * applied there. Expressed in RAW ADC counts → gain-independent
                                     * (works regardless of the 24.95-vs-8.3 op-amp gain question).
                                     * Default 0 = baseline HWZC_BLANKING_PERCENT only. Tunables:
                                     * zcDemagBlankPerA, zcDemagBlankIbusDb (live, profile 9). */
/* Compile-time fallbacks for the WS1 current-blank tunables (used only when
 * FEATURE_GSP=0; with GSP on, the live gspParams fields win). */
#ifndef ZC_DEMAG_BLANK_PER_A
#define ZC_DEMAG_BLANK_PER_A     0   /* % of sector added per 256 ADC counts of ibus over deadband */
#endif
#ifndef ZC_DEMAG_BLANK_IBUS_DB
#define ZC_DEMAG_BLANK_IBUS_DB   30  /* ibus deadband (raw counts) before the current term engages */
#endif
#ifndef ZC_DEMAG_BLANK_MAX_PCT
#define ZC_DEMAG_BLANK_MAX_PCT   25  /* total HW-ZC blank cap, % of period (25 = legacy period/4) */
#endif
#define FEATURE_PHASE_STALL_FAULT (MOTOR_PROFILE == 9) /* WS2: ENABLED for U3 2026-07-03:
                                     * bench-captured phantom lock circulated 21A for 0.8s
                                     * (bus OC blind to it) until the PSU collapsed into UV.
                                     * stallIphaseAdc=360 (~12A) + 50ms debounce -> FAULT_STALL.
                                     * Original note: */ /* phase-current stall/desync backstop. The bus-current OC is
                                     * blind to a freewheeling phantom lock (current circulates in the
                                     * phase, bus reads ~0). This watches PHASE current (iaRaw/ibRaw,
                                     * already sampled every ISR): if |ia|/|ib| stays above a high
                                     * threshold for stallDebounceMs while in closed loop → latch
                                     * FAULT_STALL and kill the bridge. Catches the silent phantom that
                                     * NEITHER firmware detects. Threshold in RAW counts (gain-
                                     * independent). Default 0. Tunables: stallIphaseAdc, stallDebounceMs. */
#ifndef STALL_IPHASE_ADC
#define STALL_IPHASE_ADC       1600  /* |phase current| magnitude (raw counts over 2048 bias) to call stall */
#endif
#ifndef STALL_DEBOUNCE_MS
#define STALL_DEBOUNCE_MS        50  /* sustained-ms above threshold before latching FAULT_STALL */
#endif
#ifndef STALL_ARM_ERPM
#define STALL_ARM_ERPM         5000  /* WS2 arm gate: eRPM the rotor must reach once before stall can fire */
#endif
/* WS4 redesign safety constants (2026-06-26): the three fixes that stop the
 * cascade railing the board (prior bench: Ia pinned 22A, Vbus 16→8V for 8s).
 *   (1) ERPM_FLOOR  — outer target never commands below the CL sync floor, so
 *       a low/zero throttle can't order a decel that drops the loaded rotor
 *       out of sync.
 *   (2) IFILT_ALPHA — IIR on the inner-loop phase current (raw per-ISR sample
 *       is commutation-dominated/saturating; the PI was chasing garbage and
 *       bouncing duty MIN↔MAX). ~1/16 → ~0.36ms TC at 45kHz, faster than the
 *       per-ZC outer loop, slow enough to smooth commutation ripple.
 *   (3) IHARD_MARGIN + FOLDBACK — a hard duty foldback INDEPENDENT of the PI:
 *       if filtered phase current exceeds (ceiling + margin) the duty is
 *       collapsed and the integrator pulled with it, regardless of the loop.
 *       This is the clamp the reference-only ceiling lacked. UV stays the
 *       last-resort hardware backstop below all of this. */
#ifndef CASCADE_ERPM_FLOOR
#define CASCADE_ERPM_FLOOR     3000  /* outer target eRPM hard floor — BELOW natural CL idle (~3500)
                                      * so at zero throttle the loop doesn't fight to accelerate the
                                      * motor (4500 was ABOVE idle → forced current ramp at idle → rail) */
#endif
#ifndef CASCADE_IFILT_ALPHA
#define CASCADE_IFILT_ALPHA  0.25f   /* inner current IIR coeff (1/4) — light. A heavy filter (1/16)
                                      * lagged so far it hid a current rail from the PI and foldback */
#endif
#ifndef CASCADE_IHARD_MARGIN_ADC
#define CASCADE_IHARD_MARGIN_ADC 150 /* hard-foldback trips at ceiling + this (raw counts ≈5A) */
#endif
#ifndef CASCADE_FOLDBACK_NUM
#define CASCADE_FOLDBACK_NUM      9  /* over-limit duty ×(NUM/DEN) per ISR → fast collapse */
#endif
#ifndef CASCADE_FOLDBACK_DEN
#define CASCADE_FOLDBACK_DEN     10
#endif
/* ── Engage gate + slew limits (the gate+seed+slew redesign) ────────────
 * The loop ENGAGES (takes over duty) only once measured eRPM climbs past
 * CASCADE_ENGAGE_ERPM — well clear of the 2.5–4k sync-fragile zone where
 * forcing current desyncs the motor. Below CASCADE_DISENGAGE_ERPM it hands
 * back to the normal map (hysteresis). The proven startup/accel is never
 * touched; the loop only HOLDS/trims speed once the motor is established. */
#ifndef CASCADE_ENGAGE_ERPM
#define CASCADE_ENGAGE_ERPM    8000   /* take over above this (stable) speed */
#endif
#ifndef CASCADE_DISENGAGE_ERPM
#define CASCADE_DISENGAGE_ERPM 6000   /* fall back to normal map below this  */
#endif
#ifndef CASCADE_IREF_SLEW_PER_ZC
#define CASCADE_IREF_SLEW_PER_ZC  8   /* max current-ref change per ZC (counts) */
#endif
#ifndef CASCADE_DUTY_SLEW_PER_ISR
#define CASCADE_DUTY_SLEW_PER_ISR 1   /* max duty change per ISR (ticks) — the
                                       * hard guarantee against a one-ISR slam */
#endif
#define FEATURE_SPEED_CASCADE    0  /* OFF for the load-desync campaign (2026-07-03):
                                     * the conservative WS4 gains made the pot need a crawl
                                     * (fast ramps let go), limit-cycled mid-speed, and gave
                                     * near-freewheel ramp-down. Direct duty (this=0, SPEED_PI=0)
                                     * = pot->duty, detection-under-load observable without loop
                                     * dynamics. Re-enable + bench-tune WS4 AFTER detection
                                     * survives load - it is the hold-speed-under-load feature.
                                     * WS4: nested speed→current→duty cascade (the "hold speed under load
                                     * by pushing current to a ceiling" feature). FRESH implementation —
                                     * NOT motor/speed_pi.c (which outputs duty directly and had high-RPM
                                     * OV issues). Outer speed PI (per-ZC) outputs a CURRENT reference
                                     * clamped to cascadeIrefCeilingAdc; inner current PI (per ADC ISR)
                                     * regulates ibus to that ref by setting duty, replacing the direct
                                     * throttle→duty map in the CL state. The ceiling is THE safety —
                                     * keep it BELOW the WS1 demag desync current and the WS2 stall
                                     * threshold. Current quantities are RAW counts (gain-independent).
                                     * Mutually exclusive with FEATURE_SPEED_PI. Default 0 = baseline
                                     * throttle→duty. Gains MUST be bench-tuned from the conservative
                                     * defaults below. Caveat: with this on, FEATURE_DUTY_SLEW may fight
                                     * the inner PI — disable it if the loop feels sluggish/hunts. */
#if FEATURE_SPEED_CASCADE && FEATURE_SPEED_PI
#error "FEATURE_SPEED_CASCADE and FEATURE_SPEED_PI are mutually exclusive — enable only one"
#endif
/* Compile-time fallbacks for the WS4 cascade tunables (FEATURE_GSP=0 only). */
#ifndef CASCADE_SPEED_KP_MILLI
#define CASCADE_SPEED_KP_MILLI    20  /* outer Kp ×1000  (eRPM err → current-ref counts) */
#endif
#ifndef CASCADE_SPEED_KI_MICRO
#define CASCADE_SPEED_KI_MICRO  2000  /* outer Ki ×1e6   (per-ZC integral) */
#endif
#ifndef CASCADE_CURR_KP_MILLI
#define CASCADE_CURR_KP_MILLI   1000  /* inner Kp ×1000  (current-err counts → duty ticks) */
#endif
#ifndef CASCADE_CURR_KI_MICRO
#define CASCADE_CURR_KI_MICRO  20000  /* inner Ki ×1e6   (per-ADC-ISR integral) */
#endif
#ifndef CASCADE_IREF_CEILING_ADC
#define CASCADE_IREF_CEILING_ADC 750  /* current ceiling (raw counts over bias) — the safety clamp.
                                       * ~24A at the corrected 8.3 gain (~30.9 cts/A). */
#endif
#ifndef CASCADE_TGT_ERPM_IDLE
#define CASCADE_TGT_ERPM_IDLE   2000  /* throttle=0 target eRPM */
#endif
#ifndef CASCADE_TGT_ERPM_MAX
#define CASCADE_TGT_ERPM_MAX   90000  /* throttle=full target eRPM */
#endif
#define FEATURE_VARIABLE_BEMF_TRIGGER 1 /* WS3: duty-adaptive BEMF ADC sample point. Today the floating-
                                     * phase sample is FIXED at LOOPTIME_TCY/2 (freewheel-center), which
                                     * degrades above ~50% duty as the ON pulse swallows that instant.
                                     * This recomputes PG1TRIGA each ISR from duty: triga = basePct% of
                                     * period, plus a duty-proportional shift above a threshold. NOTE our
                                     * sampling is freewheel-center (not the reference's in-ON-pulse
                                     * 0.6/0.8/0.9×duty), so the schedule must be BENCH-MAPPED (map max
                                     * clean-lock eRPM vs fire-time). Default 0; even on, the defaults
                                     * (basePct=50, shiftQ=0) reproduce today's fixed point exactly. */
#ifndef BEMF_TRIG_BASE_PCT
#define BEMF_TRIG_BASE_PCT       50  /* base sample point as % of LOOPTIME_TCY (50 = today's MPER/2) */
#endif
#ifndef BEMF_TRIG_DUTY_THRESH_PCT
#define BEMF_TRIG_DUTY_THRESH_PCT 50 /* duty% above which the duty-proportional shift engages */
#endif
#ifndef BEMF_TRIG_SHIFT_Q
#define BEMF_TRIG_SHIFT_Q         0  /* shift gain: triga += (LOOPTIME_TCY/256)*shiftQ*(dutyPct-thresh)/100. 0 = no shift */
#endif
#define FEATURE_VBUS_SAG_LIMIT   0  /* OFF (2026-07-03 U3 campaign): CAUGHT IN THE ACT strangling
                                     * the loaded U3 at a 13.6V bench. Threshold 900 (~13V) with
                                     * recovery 1000 (~14.4V) means a 13.6V bench sits permanently
                                     * inside the engage band and can never release. Under ~2-3A
                                     * load the PSU dips 13.3-13.5V -> limiter cuts duty -> friction
                                     * load doesn't shrink -> speed falls, current stays -> more sag
                                     * -> POSITIVE FEEDBACK duty spiral 62%%->5%% -> near-stall -> UV
                                     * FAULT (flt=2). Telemetry: thr pinned ~2062 while duty walked
                                     * down alone; sync=1 throughout - detection never blinked. This
                                     * is very likely the original '2A load desync'. Re-enable only
                                     * with thresholds derived from the ACTUAL bus voltage (they are
                                     * hardcoded ADC counts below, not profile params). Phase C2:
                                     * Bus voltage sag power limiting (reduce duty on Vbus dip) */
#define FEATURE_BEMF_INTEGRATION 0  /* Phase E shadow estimator OFF (2026-07-04 audit): write-only
                                     * diagnostics at 45kHz, nothing reads them. Re-enable only when
                                     * actively A/B-ing the integration observer. */
#define FEATURE_SINE_STARTUP     1  /* RESTORED 2026-06-10: back to the proven sine startup (I-f parked).
                                     * NOTE: profile 9 (U3) overrides this to 0 via #undef AFTER
                                     * MOTOR_PROFILE is #defined (the override can't live here — it would
                                     * read MOTOR_PROFILE as 0, the AM32 used-before-define trap).
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

/* ── Soft-start via CMP3 chop (2026-06-18) ────────────────────────────────
 * FEATURE_SOFTSTART_CHOP=1: hold the CMP3 cycle-by-cycle chop LOW during the
 * open-loop spin-up (ALIGN + OL_RAMP), so the hardware current limit caps the
 * startup inrush (~14-22A today) at OC_CMP3_SOFTSTART_MA. The motor then
 * accelerates at a bounded current = a smooth soft-start, instead of slamming
 * full startup torque. Morph onward restores the operational chop. This just
 * re-derives the startup chop DAC; it does NOT touch ocStartupMa (so the
 * ocStartupMa>=ocLimitMa validation is unaffected) or any state transition.
 * TUNE: lower = gentler but may not break away from standstill; if the motor
 * won't spin or slips during OL, raise this and/or lower rampAccelErpmPerS.
 * Bench-proven window on the 2810 was ~4-6A; 7A leaves break-away margin. */
#define FEATURE_SOFTSTART_CHOP   0   /* DISABLED 2026-06-24: on the live AM32 path there is no
                                      * ALIGN/OL spin-up phase, so this chop never applied (see
                                      * startup review). Re-enable only with a sine/regular startup. */
#define OC_CMP3_SOFTSTART_MA  7000   /* CMP3 chop current during ALIGN/OL spin-up */
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

/* Motor selection + per-profile motor data live in garuda_motors.h. */
#include "garuda_motors.h"

/* 2026-06-17 PER-PROFILE: high-KV micro motors (VEX prof 6, 1407 prof 7/8 @10V)
 * can't use the AM32 kick — BEMF is below the detection floor at the kick instant
 * so it phantom-locks instantly. They force the SINE OL ramp (drags the rotor to
 * rampTargetErpm where BEMF is real before any ZC). Everything else (2810 etc.)
 * keeps the AM32 kick+listen default. 2026-06-25: U3 (profile 9) joins the sine
 * carve-out — a heavy 97 g PROPPED rotor must NOT take the AM32 blind kick (it
 * phantom-locks / slams). It uses the classic 6-step align->ramp->morph->CL. */
#if MOTOR_PROFILE == 6 || MOTOR_PROFILE == 7 || MOTOR_PROFILE == 8 || MOTOR_PROFILE == 9
#else
#endif

/* U3 (profile 9, 2026-06-25): REGULAR trapezoidal 6-step startup, NOT sine. Done as
 * an #undef HERE (not at the FEATURE_SINE_STARTUP definition ~line 150) because
 * MOTOR_PROFILE is only #defined just above — referencing it earlier reads 0 (the
 * AM32 used-before-define trap). Path with sine OFF: STARTUP_Align (trap) ->
 * ESC_OL_RAMP (STARTUP_OpenLoopRamp, forced trap commutation) -> ESC_CLOSED_LOOP
 * DIRECTLY, no MORPH. The sine MORPH seeded the CL period + trusted sync at handoff,
 * letting CL run PHANTOM (good_zc=0, even sectors 0/2/4 masked 100%, Ia ~14A). Regular
 * startup enters CL from the forced ramp and must detect REAL floating-phase ZC. */
#if MOTOR_PROFILE == 1 || MOTOR_PROFILE == 8 || MOTOR_PROFILE == 9
/* Profile 1 A2212 (2026-07-04): joined the classic carve-out - sine's morph
 * handoff parked it in a stable MISTIMED lock at idle (10.3k @ 10.5A for 5s,
 * snapped to 1.2A same speed on a throttle poke; reproduced twice). Classic
 * enters CL from the forced ramp and must detect real ZC - A/B for that bug.
 * Profile 9 (2026-07-04): classic 6-step startup selected on the bench -
 * align -> forced OL ramp -> direct CL handoff, no sine/morph. Flip back
 * to sine by removing profile 9 from this override. */
#undef  FEATURE_SINE_STARTUP
#define FEATURE_SINE_STARTUP     0
#endif


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
#define FEATURE_EEPROM_V2        1  /* NVM persistent storage DRIVER for GSP params */

/* ── EEPROM parameter persistence — THE development/production switch ──────
 * 0 = DEVELOPMENT: gsp_params.c profileDefaults[] are the ONLY source of truth.
 *     The EEPROM overlay is never read at boot and GSP "save" never writes NVM,
 *     so code edits ALWAYS win and no stale EEPROM can shadow them (the trap that
 *     cost flash cycles tuning the A2212). Live GSP tuning still works but is
 *     RAM-only and reverts to the compiled values on the next power cycle.
 * 1 = PRODUCTION: persist tuned params to EEPROM and overlay them at boot
 *     (signature-gated). Requires FEATURE_EEPROM_V2=1. Flip to 1 for release.
 * Gates: gsp_params.c LoadFromConfig/SaveToConfig (no-op when 0), the main.c boot
 * overlay, and the GSP SAVE_CONFIG / LOAD_PROFILE EEPROM writes. */
#define FEATURE_GSP_EEPROM       0

/* Legacy name, now DERIVED from the switch above (was a separate bring-up flag).
 * Forces compiled defaults at boot whenever EEPROM persistence is disabled. */
#define FEATURE_PARAMS_FORCE_DEFAULTS  (!FEATURE_GSP_EEPROM)

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
/* ── Bus-current ON-center / OA3-ring probe (dev-only, defined early so the
 * per-profile OC_CLPCI_ENABLE gate below can see it) ───────────────────────
 * Step toward bus-current current-limited startup (CMP3 hardware cycle-chop).
 * =1 turns ARM into a static sector-0 hold: pot sweeps the LEB blanking window
 * (PG1LEB) while CMP3 cycle-chop is enabled, so the chop trip-rate (surfaced in
 * the eRPM telemetry column) reveals how long the OA3 turn-on ring lasts — sized
 * blind, no scope. Default 0. MUST be 0 before merge. */
#define FEATURE_IBUS_PROBE  0

/* 2026-06-14: sample BUS current (AD3CH1/OA3) at the ACTIVE-VECTOR PULSE CENTER
 * (PG1TRIGB=0) instead of sharing the BEMF freewheel-center trigger (PG1TRIGA=
 * MPER/2). The freewheel-center sample reads link current ≈0/negative (idle 0.2A
 * vs PSU 0.6A; negative & 2-level-aliased at top speed). Pulse-center samples
 * where motoring current actually flows through the DC-link shunt → true Ibus.
 * No added ADC load (same one conversion/period, own AD3 core; BEMF untouched).
 * Set 0 to revert to the shared freewheel-center trigger. */
/* ISOLATION TEST 2026-06-14: generate the ADTR2 trigger (PG1TRIGB) but leave ALL
 * ADC channels on ADTR1 (nothing consumes ADTR2). If the motor still UVs at high
 * speed -> the ADTR2 trigger GENERATION itself disrupts (PWM/ADC arbiter). If it
 * reaches 260k clean -> the disruption is AD3CH1 CONVERTING on ADTR2 (extra
 * conversion load). Set 0 after the test. Independent of FEATURE_IBUS_ONCENTER. */
#define AK512_ADTR2_ISOLATION_TEST  0   /* RESULT: ADTR2 generation alone = clean 260k. So the
                                          * high-speed break is AD3CH1 CONVERTING on ADTR2 (splitting
                                          * the two AD3 conversions across two trigger instants), NOT
                                          * the trigger broadcast. Second-trigger ibus path is dead. */

#define FEATURE_IBUS_ONCENTER  0   /* 2026-06-14: KEEP OFF. Root cause (by elimination) is the
                                    * SECOND PWM-ADC trigger (ADTR2) itself, NOT the sample instant:
                                    * count-0, MPER/8, Vbus@MPER/2 vs count-0 ALL fail; only the
                                    * ADTR1-triggered (freewheel) config reaches 260k. Using ADTR2
                                    * for AD3CH1 breaks high-speed commutation -> desync -> UV. */

/* EXPERIMENT 1a (2026-06-15): every prior FEATURE_IBUS_ONCENTER attempt kept
 * VBUS on ADTR1 while IBUS moved to ADTR2 — so AD3 converted at TWO instants
 * (a SPLIT). This puts BOTH AD3 conversions (IBUS + VBUS) on the SINGLE ADTR2
 * (mid-ON) trigger: AD3 fires at exactly one instant, never split. VBUS at
 * mid-ON is harmless (bus voltage is slow). Hypothesis: the split — not the
 * mid-ON instant — was what broke high-speed commutation. WIN = stays clean to
 * ~260k AND Ibus reads real (idle ~0.6A, rises with load). Set 0 to revert. */
#define FEATURE_BUS_BOTH_ONCENTER  0   /* PARKED 2026-06-16 (proven: clean 260k + real bus current).
                                          * Set 1 to resume the mid-ON dual-AD3 bus-current measurement. */

/* BRINGUP DIAG (2026-06-13): force the CMP3 trip threshold below the OA3 rest
 * level so the comparator is permanently tripped. If the CMP3->CLPCI chop chain
 * is actually wired, PWM is chopped to ~0 and the motor CANNOT spin up. This is
 * a decisive go/no-go: motor refuses to spin => chop works; motor idles normally
 * => chain is dead (INPSEL/CLPCI wrong). Set to 0 (or remove) before merge. */
#define AK512_CHOP_ALWAYS_TEST  0   /* STEP-1 PASSED 2026-06-16: with ALWAYS_TEST=1 the motor made ZERO
                                      * current (Ia 0.5, Ibus 0) and could not spin -> CMP3->CLPCI chop
                                      * chain CONFIRMED LIVE. Now 0 = chop at the real DAC threshold. */

/* (per-profile motor data table moved to garuda_motors.h) */

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
#define VBUS_FILT_SHIFT            4           /* IIR shift for the OV/UV bus filter (~16 samples / ~355us).
                                                * vbusRaw is a single unfiltered per-ISR sample; at high duty
                                                * it shows brief switching-coupled dips that false-trip UV even
                                                * though the DC bus is steady. Filtering rejects those; a real
                                                * brownout / regen rise (ms-scale) passes through. 0 disables. */
#define VBUS_UV_STARTUP_ADC        200         /* ~7.4V: relaxed UV during pre-sync startup.
                                                * Below ~4.5V, bootstrap caps can't charge and
                                                * gate drive fails. 400 ADC (~5.6V) gives margin
                                                * above brownout while staying well below the
                                                * CC-sag floor (~636 ADC = 8.9V at 10A).
                                                * Normal UV threshold resumes after zcSynced. */

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

/* 2026-06-17 RETRACTED: HWZC_ADV_FULL_ERPM=25000 was added on the wrong theory
 * that the VEX's half-speed/high-current came from advance STARVATION. The real
 * cause was the ABS_FLOOR(λ) clamp using the 2810's flux (583 vs 230) — fixed by
 * focKeUvSRad=230 in the VEX profile. With λ correct, this override made things
 * WORSE: compressing the whole 0->20° ramp into 3k-25k put ~18° advance at ~22k,
 * over-advancing -> desync/phantom above 20k (snaps to the 32k ABS_FLOOR ceiling).
 * Removed so advance ramps gradually to maxClosedLoopErpm like every other motor
 * (~1.6° at 22k). If a motor genuinely needs more mid-band advance, raise
 * timingAdvMaxDeg (param 0x22), don't steepen the ramp endpoint. */
/* (no HWZC_ADV_FULL_ERPM define — RT_TIMING_ADV_FULL_ERPM falls back to maxCL) */

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
#define MORPH_CONVERGE_SECTORS   24   /* Sectors for duty convergence. 6 -> 12 (2026-05-28)
                                       * -> 24 (2026-07-03, U3 startup smoothing): 4 e-cycles,
                                       * ~96ms blend at 2500 eRPM. Halves float-phase dV/dt
                                       * again; still far inside MORPH_TIMEOUT_MS 2000. */
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
/* FEATURE_IBUS_PROBE defined near MOTOR_PROFILE (above). */
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
#if MOTOR_PROFILE == 9
#define FEATURE_HWZC_PWM_GATE      1   /* RE-ENABLED 2026-07-03 (load-desync campaign A/B): the clean
                                        * load run (sag limiter off, direct duty) reproduced the 2810's
                                        * MODE-A signature verbatim - healthy detection to the last
                                        * sample (sync=1, zcTh steady, smooth droop 41k->36k at 5A),
                                        * then a ONE-SAMPLE current-rail slam (+21.4A) -> UV. Mode A =
                                        * single 60deg slip from one bad capture; on the 2810 the PWM
                                        * gate was THE fix. The June refutation below was measured
                                        * against a regressed ~33k-ceiling baseline that no longer
                                        * exists (today: 83k no-load clean). A/B: if no-load regresses
                                        * or the slam persists, flip back to 0.
                                        * --- prior U3 (2026-06-26) rationale (kept): back to continuous
                                        * comparator. Re-enabling the
                                        * gate did NOT help (same ~33k ceiling, same OC->UV) and made the
                                        * rej% telemetry go UP, not down — because the gate's own PWM-OFF
                                        * rejection path increments noiseRejectCount (hwzc.c:487), so every
                                        * PWM-OFF comparator fire is counted as a "reject". The metric was
                                        * conflated, not the detection improved. Continuous = better no-load
                                        * baseline (35k). See the measured-neutral notes above.
                                        * --- prior RE-ENABLE rationale (kept for history) ---
                                        * The continuous-comparator test
                                        * (=0) reached ~35k no-load but the PROPPED ceiling is set by
                                        * BEMF-vs-PWM-ripple SNR, not demag: bench proof — dropping Vbus
                                        * 24->16V (ripple ∝ Vbus, BEMF=Ke·ω fixed) moved the desync 32k->34k
                                        * with rej pinned 70-78% (= the continuous comparator eating ripple
                                        * at every PWM phase). This is the ORIGINAL board (RC τ=30µs,
                                        * K_Q15=102943706, ~8× ripple atten — NOT the 200pF board), so the
                                        * hardware ripple can't be improved here; the gate is the only ripple
                                        * lever. Phase-consistent sampling (accept only PWM-ON) now sits on
                                        * top of the measured-neutral threshold it never had at the old 31k
                                        * point. Revert to 0 for the continuous-comparator behaviour. */
#else
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
#endif
/* 2026-06-17 PER-PROFILE: high-KV micros (6/7/8) run the coherence check OFF —
 * it rejects marginal-but-real crossings on their weak 10V BEMF. 2810 etc. keep it ON. */
#if MOTOR_PROFILE == 6 || MOTOR_PROFILE == 7 || MOTOR_PROFILE == 8 || MOTOR_PROFILE == 2
/* 2026-06-18 TEST: profile 2 added — the coherence re-read corrupts ZC timing at
 * high speed (~175k = ~2.5 ADC samples/step) -> circulating 22A. Trusting the
 * comparator edge (OFF) is what runs profile 6 clean. Revert to the #else if this
 * doesn't clear the 2810 top-end. */
#define FEATURE_HWZC_VERIFY_READS  0
#else
#define FEATURE_HWZC_VERIFY_READS  1   /* default ON */
#endif
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
/* 2026-06-17 PER-PROFILE: high-KV micros (6/7/8) use a smaller deadband so their
 * ~6-count 10V BEMF can cross; 2810 etc. keep the default 4. */
#if MOTOR_PROFILE == 6 || MOTOR_PROFILE == 7 || MOTOR_PROFILE == 8 || MOTOR_PROFILE == 2
#define HWZC_CMP_DEADBAND        2   /* 2026-06-18 TEST: profile 2 on the relaxed deadband */
#else
#define HWZC_CMP_DEADBAND        4   /* ADC counts deadband for comparator sanity check (default) */
#endif
#define HWZC_THRESH_BIAS_DOWN    0   /* 2026-06-16 back to 0: using the proportional ZC_DUTY_DIVISOR
                                      * knob instead (fixed bias couldn't scale across duty). Was: ADC counts subtracted from
                                      * zcThreshold to pull the ZC detection DOWN toward the true
                                      * neutral. VN parks the threshold ABOVE where this motor's BEMF
                                      * crosses, and the filter-comp can't subtract on falling sectors.
                                      * Tune: ZC still high -> raise; overshot below -> lower. 0 = off
                                      * (REVERT to 0 for the 2810). */
#define HWZC_IIR_FREEZE_ZC_COUNT  3  /* Freeze stepPeriodHR IIR until goodZcCount reaches this.
                                      * Protects against phantom-driven IIR collapse during the
                                      * fragile first ~1ms after morph→CL handoff, where a single
                                      * mistimed ZC used to pull stepPeriodHR to floor and stall
                                      * the motor. Seeded stepPeriodHR drives commDelay until the
                                      * motor locks in and the adaptive IIR takes over. */
#define HWZC_MIN_INTERVAL_PCT   50   /* 2026-06-17 (VEX) 30->50: restored. The 30 was only needed
                                      * while the ABS_FLOOR(λ=563) clamp pinned the motor at half
                                      * speed (weak BEMF). With λ=230 the motor reaches true speed,
                                      * BEMF is healthy, and 30 was letting an early false ZC collapse
                                      * the period into a too-fast phantom (~32k @ 10% duty desync).
                                      * 50 rejects sub-half-interval crossings. (orig 75->50.) */
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

/* Capture-RATE watchdog (2026-07-03, U3 pot-0 phantom). A phantom lock is
 * SELF-DEFENDING: every few hundred ms one noise fire lands in an ON window
 * with a plausible interval and gets ACCEPTED, resetting the zero-capture
 * watchdog above forever while being far too sparse to re-lock the PI
 * (bench 16V: 1.7s+ at eRPM frozen 6.9k, -5.5A bus, 9.6A phase, rej=100%,
 * below every fault threshold). A healthy sector-PI lock accepts ~1 capture
 * per commutation; the towel-drag worst case (60% single-polarity misses)
 * still gave ~40%. So compare RATES: accepted captures vs commutations over
 * a window; sustained ratio below the floor = the PLL is blind → RECOVERY. */
#define FEATURE_HWZC_CAPRATE_WATCHDOG  1
#define HWZC_CAPRATE_WINDOW_MS   50  /* rate-evaluation window */
#define HWZC_CAPRATE_MIN_PCT     25  /* accepts must be >= this % of comms
                                      * (healthy ~100%, towel-drag ~40%,
                                      * phantom ~1%) */
#define HWZC_CAPRATE_MIN_COMMS   20  /* min commutations per window to judge
                                      * (below this the motor is too slow for
                                      * a meaningful ratio — stay silent) */
#define HWZC_CAPRATE_STRIKES      3  /* consecutive bad windows before firing
                                      * (150 ms total — rides through chop
                                      * storms and accel transients) */

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

/* ZC threshold source — measured electrical neutral vs duty model.
 * Profile 9 (U3 KV700, 2026-06-25): the duty model (vbusRaw*duty/divisor)
 * rides increasingly ABOVE the BEMF center as duty climbs (measured on the
 * "zc plotn1" capture), and that per-polarity bias drives the 32k↔34k
 * alternation → ~35k desync. This replaces it (in CL only) with the MEASURED
 * neutral = (phaseBHigh + phaseBLow)/2 — the real star-point voltage in the
 * SAME bemfRaw ADC scale, duty-independent. The FILTER_COMP RC-lag term stays
 * on top ("vdc/2 + filter corrections"). Computation already runs at
 * garuda_service.c ~1021/1033; this flag makes it the threshold SOURCE.
 * NOTE the historical disable reason (it read ~30% lower → noise-floor
 * rejects on the OLD HW-comparator ON-time path); current regime is PWM-gate
 * OFF + OFF-center SW detect, so re-evaluating on the bench. MOTOR_PROFILE is
 * #defined at line ~216, so this direct compare is safe. */
#if MOTOR_PROFILE == 9
#define FEATURE_HWZC_MEASURED_NEUTRAL  1
#else
#define FEATURE_HWZC_MEASURED_NEUTRAL  0
#endif
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
/* FEATURE_HWZC_FALLING_HW — detect FALLING ZC with the hardware digital
 * comparator (same as rising), GATED to the PWM-OFF (freewheel) window so the
 * PWM-ON driven-phase coupling — which biases the floating phase UP and buries
 * the downward crossing — cannot false-trigger. This removes the falling
 * MPER/2-sample 50%-duty wall: the SW OFF-center sample is taken once per
 * period at a FIXED point (MPER/2) and is swallowed by the ON pulse above 50%
 * duty (U3 prop-desync, 2026-06-26 bench); the freewheel-gated comparator
 * watches continuously and is duty-independent. Mutually exclusive with
 * FALLING_SW. ConfigComparator already sets CMPMOD for falling each
 * commutation; this only arms the IE for falling + adds the freewheel gate. */
#if MOTOR_PROFILE == 9
#define FEATURE_HWZC_FALLING_HW        1   /* U3: falling ZC via HW comparator, freewheel-gated */
#else
#define FEATURE_HWZC_FALLING_HW        0
#endif


/* FEATURE_HWZC_FALLING_OFFWIN — detection-window schedule step 1 (2026-07-03).
 * Below the engage duty, FALLING sectors detect their ZC in the PWM-OFF
 * (freewheel) window against a small near-ground threshold instead of hunting
 * the buried ON-window crossing. Physics: during low-side freewheel both
 * driven terminals are at ground, so the floating terminal IS the raw BEMF —
 * a falling crossing is a clean descent through ~0. The OFF window is wide
 * exactly where the ON window is a keyhole (6% duty = ~1.3us ON at 45kHz =
 * ~1 ADC sample), which is where falling sectors starve today (odd-sector
 * miss asymmetry, idle rej ~100%, spin-down phantom). The epsilon threshold
 * self-gates: during ON the floating terminal sits at neutral+BEMF, volts
 * above epsilon, so the comparator is physically silent there. Above the
 * release duty the OFF window itself becomes the keyhole — fall back to the
 * ON-window neutral path (bench-proven to 98% duty / 96k eRPM). RISING
 * sectors are untouched everywhere (driven-phase coupling assists them in
 * the ON window at any duty). Set 0 to restore the ON-only baseline. */
#define FEATURE_HWZC_FALLING_OFFWIN  0
                                    /* BENCH-FALSIFIED 2026-07-03 (prop-less U3, 16V): DIAG
                                     * showed falling sectors 100% MISSING below 75% duty and
                                     * green above (= exactly where the release band returns
                                     * them to the ON path). Root cause: the epsilon-vs-ground
                                     * reference assumes CONTINUOUS freewheel conduction; at
                                     * ~1A prop-less phase current the freewheel decays early
                                     * in each OFF window (DCM), all three terminals float,
                                     * and the floating phase reads BEMF + undefined common
                                     * mode - it never descends through epsilon. The correct
                                     * window-independent, DCM-proof falling detector is the
                                     * Atomberg-style SOFTWARE NEUTRAL (same-instant 3-phase
                                     * sampling, compare floating phase to (va+vb+vc)/3 -
                                     * common-mode rejecting in ON, OFF, and DCM alike). Keep
                                     * this code for a future CCM/under-load A/B only. */
#define HWZC_OFFWIN_EPSILON_ADC     10  /* OFF-window falling threshold, ADC counts above
                                         * ground (~0.19V at the phase with the 23.2 divider).
                                         * Must clear the noise floor but stay small vs BEMF
                                         * amplitude (idle ~1V = ~53 counts): the capture fires
                                         * when BEMF descends to epsilon, i.e. slightly EARLY
                                         * by epsilon/slope — keep small. Bench-tune. */
#define HWZC_OFFWIN_ENGAGE_DUTY_PCT  70 /* falling->OFF window when duty below this */
#define HWZC_OFFWIN_RELEASE_DUTY_PCT 75 /* falling->ON window (legacy path) above this;
                                         * 70-75 band holds previous state (hysteresis) */

#if FEATURE_HWZC_FALLING_HW
#else
#endif
#if GARUDA_TARGET_AK512
/* AK512 bench 2026-06-12 (polarity-split cap diag, stepped GSP ramp): rising
 * comparator captures sit ~500 permille of T at every speed 30k-92k; falling
 * SW captures walk 547 -> 733 -> 901 permille (max 955) as the sector shrinks
 * toward the 45 kHz sample floor + RC lag. Above ~70k the late falling
 * captures destabilize the PI during accel transients (Ia spikes to the 22 A
 * region -> PSU sag -> UV at ~100k). Cap falling-SW at 70k; rising-only
 * carries the top end (the AK128 ran 234k rising-only above its ceiling).
 *
 * UPDATE 2026-06-13: TESTED cap 70k->150k (idea: the now-active filter-comp
 * compensates the falling RC lag, so falling sectors could stay active higher).
 * RESULT: REVERTED — it reintroduced the ~100k accel-transient UV exactly as
 * warned above (every run died 95-107k with Ibus -22A). The comp fixes the
 * STEADY-STATE falling timing but not the ACCEL-TRANSIENT instability (the
 * late falling capture still feeds the PI a bad period during fast ramps).
 * Conclusion: 70k cap stays. ALSO proves the 120-140k rough band is NOT the
 * falling-coast (extending falling-SW made it worse, not better) -> it's the
 * comp-amp saturation transition (~125k = float starts railing). */
#if MOTOR_PROFILE == 1
#elif MOTOR_PROFILE == 6
#elif MOTOR_PROFILE == 7 || MOTOR_PROFILE == 8
#else
#endif
#else
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
#if FEATURE_HWZC_FALLING_HW
#else
#endif

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
#define HWZC_ABS_FLOOR_OVERSPEED_PCT  130   /* HIGH-duty ceiling: advance carries the rotor ~125% past no-load at the top */
#define HWZC_ABS_FLOOR_MIN_DUTYFRAC  0.03f  /* skip below this duty (P_ff invalid) */
/* 2026-06-18 decel-phantom fix: the 130% ceiling is only needed at HIGH duty,
 * where timing advance legitimately carries the rotor past no-load. At IDLE/low
 * duty there is NO advance overspeed (the motor sits at ~90% of no-load), so the
 * 130% slack lets a fast decel-to-zero chop lock a phantom at ~140% of no-load
 * (~15.9k @5% duty drawing ~13A) instead of coasting to true idle (~10.5k/1.1A).
 * Below LOW_DUTYFRAC use a TIGHT ceiling so that phantom is clamped down to ~no-
 * load. 100% = the physical idle ceiling; drop to ~95/92 if it still settles a
 * touch high (true idle ≈ 92% of the formula no-load on this 2810). */
#define HWZC_ABS_FLOOR_OVERSPEED_PCT_LOW  130  /* ALL profiles (2026-07-04): was 100 for non-U3.
                                                * The tight 100% ceiling is itself an attractor:
                                                * IR-drop-skewed early ZCs pin the PLL at exactly
                                                * the floor (U3: 6.7k/12A half-blind; A2212: 10.3k
                                                * /9A at 8% duty — bench-proven via focKeUvSRad A/B:
                                                * floor moved out of reach -> mistimed transients
                                                * self-correct in <200ms). 130% keeps the phantom
                                                * ceiling while sitting above the self-correcting
                                                * transient band (~110-122% of no-load). */
#define HWZC_ABS_FLOOR_LOW_DUTYFRAC      0.12f /* below this duty, use the LOW ceiling */

/* ── Mistimed-lock watchdog (A2212 bench 2026-07-04) ─────────────────────
 * A STABLE early-crossing lock at idle: rotor sustained ~110% of no-load
 * speed at 8% duty with ~10x idle phase current. Captures flow (caprate
 * watchdog blind) and the ABS_FLOOR shrink-only clamp never engages (the
 * state is entered by genuine tracked acceleration and held at steady
 * state, not by the PI driving the period down). Judge the OPERATING
 * POINT instead: eRPM above the lambda-derived no-load ceiling at low
 * duty, sustained across windows and NOT falling (falling = legitimate
 * pot-down coast), is provably mistimed -> coast + resync (bench-proven:
 * every gentle re-entry lands in the clean 1A idle). Uses per-profile
 * gspParams.focKeUvSRad - correct for all seeded motors. */
#define FEATURE_HWZC_MISTIME_WATCHDOG    1
#define HWZC_MISTIME_MARGIN_PCT         15   /* trip above (100 + this)% of formula no-load.
                                              * Sits between true idle (~92-97% of formula) and
                                              * the 130% floor; self-correcting transients peak
                                              * ~110-122% but for <200ms — the 8x50ms persistence
                                              * filter passes them, a pinned lock does not. */
#define HWZC_MISTIME_STRIKES             8   /* consecutive 50ms windows (~400ms) */
#define HWZC_MISTIME_DECEL_EXCL_PCT      5   /* window-over-window fall > this % = coasting */

/* ── Phase-current clip trip (desync distress witness) ────────────────────
 * Bench 2026-07-04 (2810 @24V, twice): a mistimed/desynced lock at 37-46%
 * duty circulates 21+ A phase-to-phase (Ia pinned at ADC full scale) while
 * BUS current stays moderate — so bus-side OC never trips and the caprate
 * watchdog sees a dense-but-poisoned capture stream. Both episodes braked
 * for 2-3 s until the Vbus sag hit the UV latch. Hypothesis-independent
 * witness: healthy operation never holds phase current near ADC clip for
 * consecutive windows (CL-entry pulse peaks ~17.5 A for <0.5 s). Window
 * MAX of |Ia-2048| >= trip level for STRIKES consecutive 50 ms windows at
 * dutyFrac > MIN_DUTYFRAC -> same coast-resync mirror as the watchdogs.
 * (Low-duty mistimed locks are the mistime watchdog's job.) */
#define FEATURE_IPHASE_CLIP_TRIP         1
#define IPHASE_CLIP_TRIP_MA          20000   /* window-max threshold (ADC clips ~22 A) */
#define IPHASE_CLIP_TRIP_STRIKES         3   /* x 50 ms windows = 150 ms sustained */
#define IPHASE_CLIP_TRIP_MIN_DUTYFRAC 0.15f  /* below this the CL-entry pulse lives; excluded */

/* ── Anti cap-slam: hold duty at the maxClosedLoopErpm clamp ──────────────
 * In direct-duty mode the throttle->duty map raises duty toward 100% regardless
 * of speed. When eRPM is pinned at the maxClosedLoopErpm clamp (commutation
 * period at its floor RT_HWZC_MIN_STEP_TICKS), the rotor can't commutate any
 * faster, so the surplus duty over-drives the held speed: the rotor OUTRUNS the
 * clamped commutation, the ZCs arrive early and get rejected (rej->0), the angle
 * goes wrong, and Ia slams to ~22A -> BOARD_PCI. Fix: while eRPM is within a
 * small margin of the clamp, don't let duty exceed its current value -> the
 * motor settles at the cap at the duty that reached it, cleanly. This is the
 * HWZC-period twin of the existing SW-period guard (which is blind once HWZC
 * owns the period at speed). Only ever HOLDS/LOWERS duty -> one-directional safe.
 * Calibration-free (no λ); works at whatever maxClosedLoopErpm is set. Default OFF. */
#define FEATURE_HWZC_CAP_DUTY_HOLD       1
#define CAP_DUTY_HOLD_MARGIN_SHIFT       3   /* engage when within period-floor/2^N (~12%) of cap */
#define CAP_DUTY_HOLD_MAX_PCT            86  /* hard duty ceiling in the cap band (BEMF-match @24V; tune) */
/* Plausibility gate on capValue. Reject any capture more than this fraction
 * BELOW setValue — at lock, real capValue ≈ setValue (within a few %); a
 * capture <50% of setValue is almost certainly a phantom or accel transient.
 * Discarded captures don't update the PI (timerPeriod stays). */
#define HWZC_PI_CAP_MIN_NUM            1   /* lower bound = setValue × NUM/DEN */
#define HWZC_PI_CAP_MIN_DEN            2   /* default: 1/2 of setValue */
#define HWZC_PI_CAP_MAX_NUM            3   /* upper bound = setValue × NUM/DEN */
#define HWZC_PI_CAP_MAX_DEN            2   /* default: 3/2 of setValue */
#define HWZC_FILTER_AMP_PCT         50    /* reverted to 2810 value 2026-06-16 (VEX ZC-comp
                                           * experiments removed). BEMF amplitude as % of
                                           * zcThreshold for the RC filter-lag compensation. */
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
#define HWZC_FILTER_AMP_PCT_FALLING 50   /* reverted to 2810 value 2026-06-16 */
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
#define OC_GAIN_X100          2495       /* 24.95 x 100. REVERTED from 830 (2026-07-03, bench-proven):
                                          * the June-26 "8.3 correction" (from the Microchip reference
                                          * project's datasheet value) does NOT match this board. Proof:
                                          * (1) a PSU CC-limited at 10A ran for 30+s a load the 8.3
                                          * scale read as 10.5A bus - impossible; at 24.95 (93 cts/A)
                                          * the same raw reads 3.5A = PSU actual = GSP telemetry.
                                          * (2) phantom-lock rail latches at 21.4A on the 93 scale =
                                          * ADC clip, physically consistent (66A FS would mean dead
                                          * FETs). CONSEQUENCE OF THE 830 ERA: every OC mA label meant
                                          * ~1/3 of face value - OC_SW_LIMIT_MA 18000 chopped duty at
                                          * ~6A REAL, which the burst scope caught mid-slam corrupting
                                          * BEMF/ZC -> 60-degree slip -> 21A phantom = THE U3 "load
                                          * desync". With 2495 the limiter engages at its labeled 18A. */
#define OC_VREF_MV            1650       /* 1.65V bias in millivolts */
#define OC_VADC_MV            3300       /* 3.3V ADC reference in millivolts */

/* CMP3 settings */
#define OC_CMP_HYSTERESIS       0b11  /* 45mV hysteresis (max, for noise immunity) */
#define OC_CMP_FILTER_EN        1     /* 1=enable CMP3 digital filter, 0=disable */
#define OC_LEB_BLANKING_NS      1000  /* Leading-edge blanking (ns). 2026-06-18 REVERTED 200->1000
                                       * to the 260k-clean (8565783) value. The 200ns experiment
                                       * (2026-06-13, to let the low-duty CL-idle chop engage) left the
                                       * OC comparator only 200ns of blanking -> at high speed/high duty
                                       * it can false-trip on the FET switching edge, chopping the cycle
                                       * and pinning Ia at the ~22A chop ceiling (the 2810 top-end 22A
                                       * regression; profile 6 stayed at 4A, below the chop, so never
                                       * exposed it). 1000ns fully clears the edge. TRADEOFF: the
                                       * low-duty idle chop won't engage (the 5% ON-pulse is ~1.1us, ~90%
                                       * blanked) — that's the known-good 260k behavior. Re-add a
                                       * duty-scaled LEB if the idle chop is needed later. */

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
