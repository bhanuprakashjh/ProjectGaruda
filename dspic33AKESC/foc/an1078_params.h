/**
 * @file  an1078_params.h
 * @brief AN1078 float port — motor + control constants.
 *
 * One-to-one float translation of Microchip AN1078 `userparms.h`,
 * tuned for PRODRONE 2810 1350KV @ 24V on the AK board.
 *
 * Naming convention:
 *   AN_*  = direct port of an AN1078 constant (same role, float units)
 *
 * Units throughout: SI (volts, amps, seconds, radians, rad/s electrical).
 * No Q15 normalization.
 */
#ifndef AN1078_PARAMS_H
#define AN1078_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ── Hardware references (mirror of garuda_foc_params.h, kept local
 *    to AN1078 module so it can be tuned independently) ─────────── */

/** Phase-current sensing scale (A per ADC count, signed about offset).
 *  MCLV-48V-300W: I_peak = 22.04 A → scale = 22.04 / 2048 = 0.01077 A/count.
 *  Polarity inverted at the op-amp (amplifier flip). */
#define AN_CURRENT_A_PER_COUNT      0.01076636f
#define AN_CURRENT_INVERT           1                /* op-amp inverts → negate */
#define AN_ADC_MIDPOINT             2048

/** Bus voltage scale (V per ADC count).  V_per_count = Vref·divider/full_scale
 *  = 3.3 × 23.2 / 4095 ≈ 0.01870 V/count. */
#define AN_VBUS_V_PER_COUNT         0.01870189f

/* ── Loop timing ────────────────────────────────────────────────── */

/** ADC ISR runs at PWM rate.  PWMFREQUENCY_HZ in garuda_config.h is
 *  60 kHz (bumped 48→60 on 2026-04-25 chasing 196k eRPM benchmark).
 *  AN_FS_HZ MUST match — F_PLANT/G_PLANT/KSLF_SCALE all derive from
 *  AN_TS, and a mismatch causes the SMC current model to be wrong
 *  by FS_real/FS_assumed.  Previous 24000 was stale → G_PLANT 67%
 *  over-aggressive → observer destabilized at ~85k eRPM. */
#define AN_FS_HZ                    60000.0f
#define AN_TS                       (1.0f / AN_FS_HZ)

/** Speed loop / observer averaging period in fast-loop ticks.
 *  AN1078: SPEEDLOOPFREQ = 1000 Hz.  At 60 kHz: 60 ticks → 1 kHz. */
#define AN_IRP_PERCALC              60

/** Theta-error transition pacing: AN1078 uses TRANSITION_STEPS = IRP_PERCALC/4. */
#define AN_TRANSITION_STEPS         (AN_IRP_PERCALC / 4)

/* ── Motor (PRODRONE 2810 @ 24V) ───────────────────────────────── */

#define AN_NOPOLESPAIRS             7                /* 7 PP (14 magnets) */

/** Phase-to-neutral resistance (Ω).  Measured ~22 mΩ. */
#define AN_MOTOR_RS                 0.022f

/** Phase-to-neutral inductance (H).  Measured ~10 µH. */
#define AN_MOTOR_LS                 10e-6f

/** Per-phase peak flux linkage λ (V·s/rad_electrical).
 *  Computed from KV: λ = 60 / (√3 · 2π · KV · PP) ≈ 0.000583 for 1350 KV. */
#define AN_MOTOR_LAMBDA             0.000583f

/** Discrete plant pole F = 1 - Rs·Ts/Ls.
 *  2810: 1 - 0.022 × 41.67e-6 / 10e-6 = 0.908.  Stable (must be 0..1). */
#define AN_F_PLANT                  (1.0f - AN_MOTOR_RS * AN_TS / AN_MOTOR_LS)

/** Discrete plant gain G = Ts/Ls. */
#define AN_G_PLANT                  (AN_TS / AN_MOTOR_LS)

/* ── Operating speed envelope ──────────────────────────────────── */

/** Open-loop ramp-up end value, mechanical RPM.
 *  Handoff is disabled (see an1078_motor.c) — motor stays in OL at
 *  this speed indefinitely.  Pick a safe bench speed: 500 RPM mech
 *  matches AN1078 reference and gives 0.21 V BEMF (visible on scope). */
#define AN_END_SPEED_RPM_MECH       500.0f

/** End-speed in electrical rad/s.  Used as min closed-loop speed
 *  floor and as the LPF Kslf clamp floor. */
#define AN_END_SPEED_ELEC_RS        \
    (AN_END_SPEED_RPM_MECH * (float)AN_NOPOLESPAIRS * 6.28318530718f / 60.0f)

/** Nominal motor speed (mech RPM) — full-throttle target speed.
 *  CL throttle range maps 0→full to AN_END_SPEED → AN_NOMINAL_SPEED.
 *
 *  Theoretical no-load max at 24V: Vbus×0.95/√3/λ ≈ 22500 rad/s elec
 *  (~215k eRPM, 30700 RPM mech).  PLL-upgraded observer (2026-04-25)
 *  tracks cleanly to at least 176k eRPM — the throttle map ceiling is
 *  the actual limit now, not the observer.  Push to 30000 RPM mech
 *  (215k eRPM) to let throttle command up to motor's electrical max. */
#define AN_NOMINAL_SPEED_RPM_MECH   30000.0f     /* = 215k eRPM */

/** Maximum mechanical RPM. */
#define AN_MAX_SPEED_RPM_MECH       3500.0f

/** Maximum electrical rad/s — protection clamp on speed reference. */
#define AN_MAX_SPEED_ELEC_RS        \
    (AN_MAX_SPEED_RPM_MECH * (float)AN_NOPOLESPAIRS * 6.28318530718f / 60.0f)

#define AN_NOMINAL_SPEED_ELEC_RS    \
    (AN_NOMINAL_SPEED_RPM_MECH * (float)AN_NOPOLESPAIRS * 6.28318530718f / 60.0f)

/* ── Open-loop startup ────────────────────────────────────────── */

/** Lock time (ticks): total duration before OL ramp begins.  Includes:
 *    - Warmup (50 ms, gate drivers settle, no PI activity)
 *    - Iq soft-ramp (200 ms, 0 → ramp_iq linearly through PI)
 *    - Steady alignment (150 ms, full Iq holds rotor at θ=0)
 *  Total: 400 ms = 9600 ticks at 24 kHz. */
#define AN_LOCK_TIME                9600

/** Open-loop ramp acceleration in electrical rad/s².  Used ONLY for
 *  the OL startup ramp (0→AN_END_SPEED).  Slower is safer — gives
 *  cogging detents time to clear, lets observer settle before handoff.
 *  2000 rad/s² → OL ramp completes in ~180 ms.  With prop: increase
 *  AN_Q_CURRENT_REF_OPENLOOP and keep this slow. */
#define AN_OL_RAMP_RATE_RPS2        2000.0f

/** Closed-loop velRef slew rate (rad/s²).  Used to slew the speed
 *  setpoint toward throttle target — sets throttle response feel.
 *  Independent of OL ramp so high-throttle response is snappy without
 *  destabilizing OL→CL handoff.  12000 rad/s² → full sweep
 *  (366→22000 rad/s) in ~1.8 s. */
#define AN_CL_VELREF_SLEW_RPS2      12000.0f

/** Open-loop q-current reference (A peak).
 *  Need to overpower 2810's cogging detents (~15-20 mN·m peak).
 *  4 A × Kt(0.0061) = 24 mN·m — comfortably above cogging.
 *  Voltage: 4 A × 22 mΩ = 88 mV (4% of Vbus).  Very safe. */
#define AN_Q_CURRENT_REF_OPENLOOP   4.0f

/** PWM warmup phase (ticks).  When motor first starts, hold Vd=Vq=0
 *  (50% duty everywhere = zero net motor voltage) for this many ticks
 *  before engaging PI control.  Allows gate drivers to settle and
 *  bootstrap caps to top off after override-release transient.
 *  50 ms = 1200 ticks at 24 kHz. */
#define AN_WARMUP_TICKS             1200U

/** Handoff dwell — BEMF must hold above gate threshold for this many
 *  consecutive fast-loop ticks before OL→CL transitions.  100 ms is
 *  enough to filter noise; observer is already locked at this point so
 *  short dwell is fine.  100 ms = 2400 ticks at 24 kHz. */
#define AN_HANDOFF_DWELL_TICKS      2400U

/** Iq soft-start ramp duration (ticks), AFTER warmup.  Linearly ramps
 *  iq_ref from 0 to AN_Q_CURRENT_REF_OPENLOOP over this many ticks.
 *  200 ms = 4800 ticks at 24 kHz. */
#define AN_IQ_SOFT_START_TICKS      4800U

/** Speed PI Iq-output saturation (A peak).  Iq commanded by speed PI
 *  is clamped to ±this.  Bumped 9→12 to give the speed loop headroom
 *  alongside aggressive field weakening at high speed (Id can reach
 *  −12A; total |I|=√(Id²+Iq²) up to ~17A which is within MCLV-48V-300W
 *  inverter rating). */
#define AN_OVER_CURRENT_LIMIT       12.0f

/** Speed reference ramp limit: AN1078 = Q15(0.00003) per IRP_PERCALC tick.
 *  In their RPM-Q15 form that's tiny.  We treat as rad/s electrical per tick. */
#define AN_SPEEDREF_RAMP_RAD_S      1.0f

/** Number of IRP_PERCALC ticks between speed-PI updates. */
#define AN_SPEEDREFRAMP_COUNT       3

/* ── PI gains (continuous form) ───────────────────────────────────
 *
 * AN1078 uses Q15 PI:
 *   D_CURRCNTR_PTERM = 0.02 (Q15)   — proportional gain
 *   D_CURRCNTR_ITERM = 0.001 (Q15)  — applied once per tick
 *   D_CURRCNTR_CTERM = 0.999 (Q15)  — anti-windup back-calc
 *
 * Q15 PI dimensional analysis:
 *   In Q15, voltage and current are both [-1..1) of full-scale.
 *   For our motor: V_FS = Vbus = 24 V, I_FS = 22 A_peak.
 *   So Q15-Kp = 0.02 means: 0.02 × (V_FS / I_FS) = 0.02 × 24/22 = 0.022 V/A
 *   And Q15-Ki = 0.001 per tick at 20 kHz = 20 V/A·s continuous.
 *
 * For 2810 (Rs=22 mΩ, Ls=10 µH), pole-cancellation gives:
 *   Kp = bw × Ls,  Ki = bw × Rs
 * For bw = 2π·1000 = 6283 rad/s:  Kp = 0.063 V/A,  Ki = 138 V/A·s.
 *
 * Use those values — they came from v2 detection on this same motor. */

#define AN_KP_DQ                    0.063f
#define AN_KI_DQ                    138.0f

/* Speed PI:
 *   AN1078: SPEEDCNTR_PTERM = 0.5 Q15, SPEEDCNTR_ITERM = 0.005 Q15.
 *   Kp dim: A_FS/ω_FS = 22 A / (3500 RPM × 5 PP × 2π/60) = 22/1833 = 0.012 A·s/rad
 *   Q15 Kp = 0.5 → 0.5 × 0.012 = 0.006 A·s/rad (units: amps per rad/s).
 *
 * For 2810 use modest values; we'll tune from bench. */

/* Speed PI: original conservative values.  Tracking speed comes from
 * AN_CL_VELREF_SLEW_RPS2 slewing velRef in fast — PI doesn't need
 * high gain to feel snappy.  High PI gain causes overshoot at
 * OL→CL handoff: motor 5k→70k in 40 ms with no-load, which would
 * stall under prop load. */
#define AN_KP_SPD                   0.006f
#define AN_KI_SPD                   0.10f

/* Anti-windup back-calculation gain (1.0 = no anti-windup, 0 = full).
 * AN1078 uses 0.999 — close to disabled. */
#define AN_PI_KC                    0.999f

/* ── Voltage clamps ───────────────────────────────────────────── */

/** Maximum voltage vector magnitude as fraction of Vbus.  AN1078: 0.98. */
#define AN_MAX_VOLTAGE_VECTOR_FRAC  0.95f

/* ── SMC observer ─────────────────────────────────────────────── */

/** Slide-mode controller gain (V).  Z signal feeds the current-model
 *  alongside V and E.  AN1078's 0.85·Vbus = 20.4V is way too aggressive
 *  for low-impedance motors like 2810 (Rs=22mΩ).  Z saturates on any
 *  small model error → LPF averages near zero → BEMF buried.
 *
 *  Empirical: keep Z in linear range relative to actual V swings.
 *  Steady-state V is ~0.5V — set Kslide ~5x bigger to allow fast
 *  correction without saturating.  Bumping above this breaks
 *  low-speed handoff (Z dominates over weak BEMF, observer can't lock). */
#define AN_SMC_KSLIDE               2.5f

/** Maximum linear-region current error (A).  Below this, Z = K·err/MaxErr;
 *  above, Z saturates at ±K.  Bigger boundary keeps Z in linear range
 *  for 4A-class operating currents on low-impedance motors. */
#define AN_SMC_MAX_LINEAR_ERR       1.0f

/** Phase shift applied to atan2 output (radians).
 *
 * theta_offset = AN_SMC_THETA_OFFSET_BASE + AN_SMC_THETA_OFFSET_K × ω
 *
 * BASE: calibrated at 366 rad/s → 20° (0.349 rad).
 * K:    speed coefficient.  At higher speeds the SMC's LPF phase
 *       relationship to fundamental shifts; observer angle drifts.
 *       Tune K so Id stays near zero across the operating range.
 *
 * Tuning: run motor at low and high speed, read Id at each:
 *   - Low speed (366 rad/s):  set BASE so Id ≈ 0 there
 *   - Medium speed (~5000 rad/s): observe Id; if positive, K negative
 *     (subtract more from offset at higher ω); if negative, K positive
 *   - K_initial = 0 (constant offset).  Step in -1e-5 increments.
 *
 * AN1078 reference uses BASE=π/2, K=0.  Our 2810 needed BASE=20°.
 * K=0 worked clean up to ~80k eRPM, then observer started losing it. */
#define AN_SMC_THETA_OFFSET_BASE    0.349f         /* 20° at zero speed */
#define AN_SMC_THETA_OFFSET_K       1.0e-4f        /* rad per (rad/s elec) — post FS_HZ fix: observer leads 29° at 115k with K=1.5e-4 → trim */

/* Backwards-compat alias for code that hardcodes a single constant. */
#define AN_SMC_THETA_OFFSET         AN_SMC_THETA_OFFSET_BASE

/** BEMF LPF coefficient scale factor (per |ω| in rad/s electrical).
 *
 * AN1078 default: AN_TS (1·Ts).  Earlier 3·Ts gave better low-speed
 * bootstrap on 2810 but Kslf saturated at high speed (Kslf=1 means
 * LPF passes Z directly, observer breaks down past ~8000 rad/s elec).
 *
 * Use 1·Ts (AN1078 default) — Kslf saturates only above 24000 rad/s
 * elec (~230k eRPM at 7PP).  Low-speed bootstrap relies on
 * AN_SMC_KSLF_MIN floor instead. */
#define AN_SMC_KSLF_SCALE           AN_TS

/** Minimum Kslf — bootstrap floor at low speeds.  0.05 gives cutoff
 *  191 Hz, enough headroom for our 366 rad/s end-speed (58 Hz fund). */
#define AN_SMC_KSLF_MIN             0.05f

/** Maximum Kslf — cap below 1.0 so LPF retains filtering action even
 *  at very high speeds.  Each tick: y += Kslf·(x - y).  Kslf=0.5 →
 *  saturation at ω = 0.5 / Ts·SCALE = 12000 rad/s (114k eRPM at 7PP).
 *  Above that, the LPF cutoff stops scaling with speed and observer
 *  phase response degrades.
 *
 *  Bumped to 0.85 → saturation at ~204k eRPM, gives headroom for
 *  full-speed runs.  Above 0.85 LPF approaches passthrough; observer
 *  noise rejection collapses, so don't push higher. */
#define AN_SMC_KSLF_MAX             0.85f

/* ── Theta_error bleed (CL transition) ───────────────────────────
 *
 * AN1078 pmsm.c bleeds 0.05° each tick subject to (trans_counter==0)
 * which is true 1 of every TRANSITION_STEPS ticks.
 * Effective rate = 0.05° / (TRANSITION_STEPS × Ts) ≈ 200°/s.
 *
 * For 2810 (low Ke), observer angle has small drift — slow bleed gives
 * observer time to settle and avoids accumulated angle error in CL.
 * Smaller bleed step = slower migration from OL angle to observer angle. */
#define AN_THETA_ERROR_BLEED_RAD    (0.005f * 3.14159265f / 180.0f) /* 10x slower */

/* ── Throttle deadband (ADC counts) ───────────────────────────── */
#define AN_THROTTLE_DEADBAND        50

#ifdef __cplusplus
}
#endif

#endif /* AN1078_PARAMS_H */
