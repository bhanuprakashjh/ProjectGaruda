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
#include "../garuda_config.h"   /* PWMFREQUENCY_HZ (mode-aware) */

/* ── Hardware references (mirror of garuda_foc_params.h, kept local
 *    to AN1078 module so it can be tuned independently) ─────────── */

/** Phase-current sensing scale (A per ADC count, signed about offset).
 *  AK board (EV43F54A + ATA6847L DIM): 3mΩ shunt × OA G=24.95× → 10.7654
 *  mA/count (ADC_MA_PER_COUNT_Q8 = 2756 in garuda_config.h).  Matches the
 *  MCLV source scaling within 0.1%; reuse as-is.  Stock board ±22A range
 *  (saturates under prop — see garuda_config.h for the shunt-mod recipe). */
#define AN_CURRENT_A_PER_COUNT      0.01076636f
#define AN_CURRENT_INVERT           1                /* op-amp inverts → negate */
#define AN_ADC_MIDPOINT             2048

/** Bus voltage scale (V per ADC count).  AK board: 16:1 divider (R47 36kΩ
 *  / R49 2.4kΩ) → 12.8906 mV/count (ADC_VBUS_MV_PER_COUNT_Q8 = 3300).
 *  Different from MCLV's 23.2:1 divider — recalibrated for AK hardware. */
#define AN_VBUS_V_PER_COUNT         0.0128906f

/* ── Loop timing ────────────────────────────────────────────────── */

/** ADC ISR runs at PWM rate.  Derive both AN_FS_HZ and AN_IRP_PERCALC
 *  from the active PWMFREQUENCY_HZ (which is PWMFREQUENCY_HZ_FOC when
 *  FEATURE_FOC_AN1078=1).  Single source of truth — bumping the FOC PWM
 *  rate in garuda_config.h propagates here automatically so the SMC
 *  plant model can never drift from the actual switching rate. */
#define AN_FS_HZ                    ((float)PWMFREQUENCY_HZ)
#define AN_TS                       (1.0f / AN_FS_HZ)

/** Speed loop / observer averaging period in fast-loop ticks.
 *  AN1078: SPEEDLOOPFREQ = 1000 Hz.  N ticks = PWMFREQUENCY_HZ/1000.  */
#define AN_IRP_PERCALC              ((uint16_t)(PWMFREQUENCY_HZ / 1000U))

/** Theta-error transition pacing: AN1078 uses TRANSITION_STEPS = IRP_PERCALC/4. */
#define AN_TRANSITION_STEPS         (AN_IRP_PERCALC / 4)

/* ── Motor (PRODRONE 2810 1350KV @ 24V) ─────────────────────────
 *
 * Switched back from A2212@12V on 2026-04-27.
 *
 * 2810 specs:
 *   KV = 1350 RPM/V, 7 PP, Rs ≈ 22 mΩ, Ls ≈ 10 µH
 *   λ = 60 / (√3·2π·1350·7) = 0.000583 V·s/rad
 *   No-load max @ 24V = 24 / 0.000583 = 41200 rad/s elec ≈ 393k eRPM
 *   Practical max with FW + observer headroom: ~210k eRPM (validated
 *   2026-04-25, see an1078_200k_milestone memory note). */

#define AN_NOPOLESPAIRS             7                /* 7 PP (14 magnets) */

/** Phase-to-neutral resistance (Ω).  Measured ~22 mΩ. */
#define AN_MOTOR_RS                 0.022f

/** Phase-to-neutral inductance (H).  Measured ~10 µH. */
#define AN_MOTOR_LS                 10e-6f

/** Per-phase peak flux linkage λ (V·s/rad_electrical).  λ = 60 / (√3·2π·KV·PP)
 *  for 1350 KV @ 7PP gives 0.000583. */
#define AN_MOTOR_LAMBDA             0.000583f

/** Discrete plant pole F = 1 - Rs·Ts/Ls.
 *  2810: 1 - 0.022 × 41.67e-6 / 10e-6 = 0.908.  Stable (must be 0..1). */
#define AN_F_PLANT                  (1.0f - AN_MOTOR_RS * AN_TS / AN_MOTOR_LS)

/** Discrete plant gain G = Ts/Ls. */
#define AN_G_PLANT                  (AN_TS / AN_MOTOR_LS)

/** LPF time constant on id_meas/iq_meas used by FF cross-coupling.
 *  Filters ADC noise + PWM ripple before currents feed back through
 *  the FF terms (which multiply by ω·L → noise gets amplified at high
 *  speed).  VESC uses τ ≈ 10 ms for the same purpose.  2 ms here is a
 *  middle ground: noise suppression up to ~80 Hz cutoff, but still
 *  tracks the slow operating-point drift well enough.
 *
 *  α = AN_TS / τ.  At 30 kHz PWM, AN_TS = 33.3 µs → α ≈ 0.0167. */
#define AN_FF_I_LPF_TAU_S           0.002f
#define AN_FF_I_LPF_ALPHA           (AN_TS / AN_FF_I_LPF_TAU_S)

/* ── Operating speed envelope ──────────────────────────────────── */

/** Open-loop ramp-up end value, mechanical RPM.  Two roles:
 *  (1) OL→CL handoff speed: motor must reach this before observer
 *      gating can transition to closed loop.
 *  (2) Idle CL speed when throttle is below deadband.
 *
 *  500 RPM mech (= 3500 eRPM, BEMF 0.21V) was a bench-friendly low
 *  value but too slow for prop operation: prop drag at 500 RPM is
 *  significant and BEMF SNR is marginal.
 *
 *  1500 RPM mech (= 10500 eRPM, BEMF 0.64V) is prop-friendly:
 *    - Prop spins at a clean idle, ready to ramp on throttle
 *    - BEMF triple — observer locks more reliably
 *    - OL ramp at 1000 rad/s² takes 1.1s to reach this from rest
 *  Bumped 500→1500 on 2026-04-26 for prop testing. */
#define AN_END_SPEED_RPM_MECH       4000.0f   /* AK port first-light:
 * doubled from 2000 → 4000 so SMO sees ~2.5× more BEMF (~1.7V) at
 * handoff. With AK at 30 kHz the observer LPFs more aggressively →
 * needs bigger signal to lock cleanly. Revert/tune down after CL works. */

/** End-speed in electrical rad/s.  Used as min closed-loop speed
 *  floor and as the LPF Kslf clamp floor. */
#define AN_END_SPEED_ELEC_RS        \
    (AN_END_SPEED_RPM_MECH * (float)AN_NOPOLESPAIRS * 6.28318530718f / 60.0f)

/** Nominal motor speed (mech RPM) — full-throttle target speed.
 *  CL throttle range maps 0→full to AN_END_SPEED → AN_NOMINAL_SPEED.
 *
 *  2810 @ 24V: theoretical no-load = 24 × 1350 = 32400 RPM mech
 *  (= 227k eRPM at 7PP).  6-step on same motor/Vbus reaches 225k.
 *
 *  Was 30000 (= 210k eRPM) → motor reached this and stopped under FOC
 *  because throttle topped out there.  Bumped to 45000 (= 315k eRPM,
 *  way above motor's physical ceiling) so full throttle exposes the
 *  REAL bottleneck (voltage saturation → FWC engages hard).  Motor
 *  will cap at whatever it can physically do; this define no longer
 *  artificially clips. */
#define AN_NOMINAL_SPEED_RPM_MECH   45000.0f     /* = 315k eRPM target (motor caps lower) */

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
 *  the OL startup ramp (0→AN_END_SPEED = 366 rad/s).  Slower is safer
 *  with load — prop inertia needs time to come up to speed.  At
 *  1000 rad/s² OL ramp completes in ~366 ms — gentle but BEMF-gate
 *  still fires before any handoff timeout. */
#define AN_OL_RAMP_RATE_RPS2        1000.0f

/** Closed-loop velRef slew rate (rad/s²).  Used to slew the speed
 *  setpoint toward throttle target — sets throttle response feel.
 *  Independent of OL ramp so high-throttle response is snappy without
 *  destabilizing OL→CL handoff.  12000 rad/s² → full sweep
 *  (366→22000 rad/s) in ~1.8 s. */
#define AN_CL_VELREF_SLEW_RPS2      12000.0f

/** Open-loop q-current reference (A peak).
 *  Must overcome cogging + prop inertia + steady prop drag at OL end speed.
 *  Sizing: prop drag at 1500 RPM mech ≈ 30-40 mN·m on small motors → need
 *  > 50 mN·m of motor torque margin to follow synth angle.
 *
 *  Per motor:
 *    A2212 (Kt=0.0059 N·m/A): 12A → 71 mN·m  ← good for prop tests
 *    2810  (Kt=0.0061 N·m/A): 12A → 73 mN·m  ← also fine, was 8A before
 *
 *  Voltage drop @ 12A:
 *    A2212 (Rs=65mΩ): 0.78V (~6% of 12V)
 *    2810  (Rs=22mΩ): 0.26V
 *  Both well under bus voltage. */
#define AN_Q_CURRENT_REF_OPENLOOP   12.0f

/** PWM warmup phase (ticks).  When motor first starts, hold Vd=Vq=0
 *  (50% duty everywhere = zero net motor voltage) for this many ticks
 *  before engaging PI control.  Allows gate drivers to settle and
 *  bootstrap caps to top off after override-release transient.
 *  50 ms = 1200 ticks at 24 kHz. */
#define AN_WARMUP_TICKS             1200U

/** Handoff dwell — BEMF must hold above gate threshold for this many
 *  consecutive fast-loop ticks before OL→CL transitions.
 *  AK port: 50 ms = 1500 ticks at 30 kHz. We already confirmed handoff
 *  can fire (LED went fast-blink). Tightening so it only fires when
 *  observer has actually settled, not just briefly above threshold. */
#define AN_HANDOFF_DWELL_TICKS      1500U

/** Iq soft-start ramp duration (ticks), AFTER warmup.  Linearly ramps
 *  iq_ref from 0 to AN_Q_CURRENT_REF_OPENLOOP over this many ticks.
 *  200 ms = 4800 ticks at 24 kHz. */
#define AN_IQ_SOFT_START_TICKS      4800U

/** Speed PI Iq-output saturation (A peak).  Iq commanded by speed PI
 *  is clamped to ±this.  AK bench (2026-05-20): bumped 12→18 for prop
 *  testing — at 12A speed PI saturated at ~64k eRPM (Iq held at clamp
 *  while throttle climbed 50→80% with no further accel).  18A keeps
 *  headroom under the ±22A shunt saturation ceiling on the stock AK
 *  board.  Field-weak Id can reach −12A; total |I|=√(Id²+Iq²) up to
 *  ~22A peak — at that point watch the motor windings on the bench. */
#define AN_OVER_CURRENT_LIMIT       18.0f

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

/* Speed PI: bumped 2026-04-26 after OL→CL integrator-reset fix made it
 * safe to run aggressive gains without handoff overshoot.  Original
 * 0.006/0.10 was too sluggish (4 seconds for +15k eRPM under throttle).
 * Tracking speed comes from BOTH AN_CL_VELREF_SLEW_RPS2 (slewing the
 * setpoint) AND the speed PI (closing the loop on it).  Need decent
 * KP for the latter to actually track. */
#define AN_KP_SPD                   0.015f
#define AN_KI_SPD                   0.30f

/* Anti-windup back-calculation gain (1.0 = no anti-windup, 0 = full).
 * AN1078 uses 0.999 — close to disabled. */
#define AN_PI_KC                    0.999f

/* ── Voltage clamps ───────────────────────────────────────────── */

/** Maximum voltage vector magnitude as fraction of Vbus.  AN1078: 0.98.
 *  Bumped 0.95→0.99 on 2026-05-22 to push base speed higher without FW
 *  and reduce voltage saturation (which was causing PI windup + Id swings).
 *  ω_base = (Vbus/√3·0.99) / λ ≈ 23.6k rad/s ≈ 225 k eRPM with 24 V Vbus
 *  on the 2810 (vs ~216k @ 0.95). */
#define AN_MAX_VOLTAGE_VECTOR_FRAC  0.99f

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
/* Bench history 2026-05-23:
 *   2.5 → 5.0: |E|² rose 7 → 30 V² (closer to physics ~46), confirming
 *              observer was previously running at ~1/4 signal strength.
 *              BUT Id span 17 → 45 A, Iq span 12 → 48 A, vbus dipped to
 *              22V on transients (foc_run_132249).  Rest of loop was
 *              implicitly tuned around the *attenuated* signal; restoring
 *              it exposes downstream gain mismatch.  Reverted to 2.5.
 * To use Kslide=5 (or higher), the PLL and current PI gains need scaling
 * down to match.  Not done.  Quiet, weak-signal observer is the
 * working point until a coordinated tuning pass is undertaken. */

/** Maximum linear-region current error (A).  Below this, Z = K·err/MaxErr;
 *  above, Z saturates at ±K.  Bigger boundary keeps Z in linear range
 *  for 4A-class operating currents on low-impedance motors. */
#define AN_SMC_MAX_LINEAR_ERR       1.0f

/* ── Super-twisting algorithm (STA) observer ───────────────────
 *
 * Higher-order sliding-mode observer.  Replaces the chattering Z signal
 * + LPF chain with two cascaded continuous integrators that reconstruct
 * BEMF in finite time without lag and without the 1/2 scaling factor.
 *
 * Plant:        L·di/dt   = V − R·i − e
 * Observer:     L·di_hat/dt = V − R·i_hat − e_hat − L·z1
 *               de_hat/dt   = k2 · sign(i_err)            ← second integrator
 *               z1          = k1 · √|i_err| · sign(i_err) ← first-order term
 *
 * i_err = i_meas − i_hat (standard convention; opposite sign from our
 * existing IalphaError struct field which still holds EstI − I).
 *
 * At sliding: i_err → 0 in finite time, e_hat → e exactly.  No LPF, no
 * phase lag, no scaling factor.
 *
 * Gains (conservative starting point — bench-tune from here):
 *   ESMO_STA_K1 — proportional-like gain, units V·s^(−1/2)·A^(−1/2).
 *     Levant: k1 > √(2·k2·(1+p)/(1−p)) for some p in (0,1).  Pick
 *     k1 ≈ 1500 — covers reasonable disturbance bound.
 *   ESMO_STA_K2 — integrator gain, units V/s.  Must exceed max |de/dt|
 *     which is ω²·λ ≈ 23000²·0.000583 ≈ 308 kV/s at top.  Pick k2 ≈
 *     500 kV/s = 5e5.
 *
 * If motor doesn't lock at startup, lower k1 (overshooting).
 * If observer lags at top, raise k2 (can't track BEMF rate).
 * If observer chatters in steady state, raise k1 (insufficient damping). */
#ifndef FEATURE_ESMO_STA
#define FEATURE_ESMO_STA            0   /* 1 = use STA, 0 = use classic SMC.
                                         *
                                         * 2026-05-23 first bench (foc_run_133838):
                                         * STA diverged catastrophically — z2
                                         * integrators ran to 5×10⁶ V because
                                         * sign(i_err) had sustained bias during
                                         * OL ramp (observer-model angle vs real
                                         * rotor angle mismatch).  At CL handoff
                                         * z2 was garbage, PLL locked wrong, motor
                                         * stalled at CL with Iq saturated at 18A.
                                         *
                                         * To make STA work properly we need:
                                         *  (a) anti-windup on z2 (clamp ±30V)
                                         *  (b) suppress integration during OL
                                         *      (use classic SMC until CL)
                                         *  (c) initial-value priming at OL→CL
                                         *  (d) co-tuned k1, k2 via Lyapunov
                                         * Code preserved under flag for when
                                         * that engineering investment is made. */
#endif
#define ESMO_STA_K1                 1500.0f
#define ESMO_STA_K2                 500000.0f

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
