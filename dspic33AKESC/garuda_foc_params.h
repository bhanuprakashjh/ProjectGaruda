/**
 * @file garuda_foc_params.h
 *
 * @brief Motor-specific and FOC algorithm parameters for Project Garuda.
 *
 * Motor profiles selected by MOTOR_PROFILE in garuda_config.h:
 *   0 = Hurst DMB0224C10002  (5PP, 24V, 2.54 ohm, dev bench)
 *   1 = A2212 1400KV          (7PP, 12V, 0.065 ohm, drone)
 *   2 = Flycat 5010-750KV     (7PP, 14.8V, 0.08 ohm, heavy-lift)
 *
 * Switching frequency: 24 kHz (Ts = 41.67 us)
 *
 * Component: FOC PARAMETERS
 */

#ifndef GARUDA_FOC_PARAMS_H
#define GARUDA_FOC_PARAMS_H

#include "garuda_config.h"   /* MOTOR_PROFILE, USE_INTERNAL_OPAMP */

/* ================================================================
 *          MOTOR-SPECIFIC PARAMETERS  (per-profile)
 * ================================================================ */

#if MOTOR_PROFILE == 0
/* ----------------------------------------------------------------
 * Hurst DMB2424B10002 (Long Hurst / Hurst300 / AC300022)
 * Also: Hurst DMA0204024B101
 * 10 poles (5PP), 24VDC, 3.4A RMS rated, nominal 2500 RPM
 *
 * Reference: Microchip hurst300.h (AN1292 dsPIC33AK FOC-PLL)
 *   Rs = 0.3715 Ω, Ls = 359 µH, Ke = 6.7316 Vpk_LL/kRPM
 *
 * Auto-detect measured Rs=534mΩ, Ls=471µH — higher than Microchip
 * reference, possibly due to temperature or motor variant. Using
 * Microchip values as baseline; tune from GUI if needed.
 *
 * NOT the short DMB0224C10002 (Rs=2.54Ω, Ls=2.21mH — 7x different!)
 * ---------------------------------------------------------------- */

/** Phase resistance (ohm). Auto-detect measured 0.534Ω.
 *  Microchip reference: 0.3715Ω (cold).  Using measured value —
 *  observer convergence depends critically on accurate Rs. */
#define MOTOR_RS_OHM            0.534f
/** Phase inductance (H). Microchip reference: 359µH.
 *  Auto-detect measured 471µH — use measured value for SMO accuracy.
 *  SMO coefficients F,G depend directly on Ls (31% error → divergence). */
#define MOTOR_LS_H              0.000471f
/** Per-phase flux linkage λ_pm (V·s/rad, electrical).
 *  Ke_LL = 6.7316 Vpk/kRPM (Microchip hurst300.h).
 *  KV = 1000/6.7316 = 148.6 RPM/V.
 *  λ_pm = 60/(√3 × 2π × KV × PP) = 60/(1.732×6.2832×148.6×5) = 0.00742 */
#define MOTOR_KE_VPEAK          0.00742f
#define MOTOR_POLE_PAIRS_FOC    5
#define MOTOR_VBUS_NOM_V        24.0f
/** Peak phase current limit (A). Rated 3.4A RMS = 4.8A peak.
 *  Allow 2x for startup transients. */
#define MOTOR_MAX_CURRENT_A     10.0f
/** Max electrical speed (rad/s).
 *  3500 RPM max × 5PP × 2π/60 = 1833. Round to 2000. */
#define MOTOR_MAX_ELEC_RAD_S    2000.0f
/** Flux linkage = λ_pm (per-phase). */
#define MOTOR_FLUX_LINKAGE      0.00742f

/* D/Q Current Loop PI (BW ~ 530 Hz, parallel form, Tustin integration)
 *   Kp = ωbw × Ls = 2π×530 × 0.000471 = 1.568 ≈ 1.57
 *   Ki = ωbw × Rs = 2π×530 × 0.534 = 1778
 *   PI zero = Ki/Kp = 1778/1.57 = 1132 ≈ Rs/Ls = 0.534/0.000471 = 1134
 *   → pole cancellation with measured Rs and Ls. */
#define KP_DQ                   1.57f
#define KI_DQ                   1778.0f

/* Speed Loop PI (outer loop — active after CL handoff)
 *   Kt_FOC = (3/2)×5×0.00742 = 0.0557 N·m/A.
 *   Previous Kp=0.010, Ki=1.0 was too aggressive: at 100 rad/s error,
 *   Ki integrates 0.1A/ms → hits 10A clamp in 100ms → OC at high speed.
 *   Reduced: at 100 rad/s error, Kp gives 0.3A, Ki integrates 0.01A/ms
 *   → 1A in 100ms. Hurst no-load needs ~0.3A; under load, ~2A. */
#define KP_SPD                  0.003f
#define KI_SPD                  0.1f

/* I/f (Current-Forced) Startup
 * Microchip reference: OPEN_LOOP_CURRENT=1.0A, LOCK_CURRENT=1.0A,
 * handoff at 500 RPM, ramp rate 2000 RPM/s. */
/** Alignment Id (A) — 1.0A matches Microchip reference LOCK_CURRENT. */
#define STARTUP_ALIGN_IQ_A      1.0f
/** OL running Iq (A). Microchip reference: 1.0A.
 *  Using 1.5A for margin during CL entry — SMO residual angle error
 *  reduces effective torque. 1.5A×Kt=84mN·m, friction≈50mN·m. */
#define STARTUP_RAMP_IQ_A       1.5f
/** Iq ramp ticks (24kHz).  4800 = 200ms — smooth d→q rotation. */
#define STARTUP_IQ_RAMP_TICKS   4800U
/** Alignment dwell (ticks).  12000 = 500ms (matches Microchip LOCK_TIME). */
#define STARTUP_ALIGN_TICKS     12000U
/** Ramp rate (rad/s² electrical).
 *  Microchip: 2000 RPM/s mech = 2000×5×2π/60 = 1047 rad/s² elec.
 *  Using 500 for margin — still reaches handoff in <0.5s. */
#define STARTUP_RAMP_RATE_RPS2  500.0f
/** CL handoff speed (rad/s elec.).
 *  Microchip: 500 RPM mech = 500×5×2π/60 = 262 rad/s elec.
 *  BEMF = 0.00742×262 = 1.94V (8.1% of 24V). Good SNR. */
#define STARTUP_HANDOFF_RAD_S   262.0f
/** Min OL speed. FOC v2 uses dynamic calculation. */
#define STARTUP_MIN_OL_RAD_S    500.0f
/** Max OL speed. Vbus/Ke = 24/0.00742 = 3234, cap at 85% = 2749. */
#define STARTUP_MAX_OL_RAD_S    2749.0f

/* Fault Thresholds */
#define FAULT_OC_A              10.0f
#define FAULT_STALL_RAD_S       10.0f

/** Startup mode: 0=I/f (PI current control), 1=V/f (direct voltage).
 *  Hurst Rs=0.534Ω: I/f produces noisy PI voltages during OL ramp
 *  that confuse the SMO (PLL tracks backwards). V/f gives clean,
 *  predictable voltages. V/f curve: VF_BOOST + Ke×ω = 0.5+1.94 = 2.44V
 *  at handoff. */
#define STARTUP_USE_VF          1

/* Observer LPF -- long Hurst (moderate-speed motor)
 * Max elec freq = 2000/(2pi) = 318 Hz.
 * Microchip BEMF filter cutoff: 250 Hz → alpha ≈ 0.065.
 * Using 0.08: slightly faster than short Hurst (0.06). */
#define OBS_LPF_ALPHA           0.08f
#define SMO_LPF_ALPHA           0.08f

#elif MOTOR_PROFILE == 1
/* ----------------------------------------------------------------
 * A2212 1400KV (drone motor)
 * 12N14P, 7 pole pairs, 12V, Rs ~ 0.065 ohm, Ls ~ 30 uH
 * 1400 KV -> no-load 16800 RPM @ 12V
 *
 * FOC challenges:
 *   - Very low Ke: BEMF only 0.975V at 1000 rad/s (8% of Vbus)
 *   - Very low Rs: 0.065 ohm -> any small Vd causes large Id
 *   - High max elec freq: 1910 Hz -> observer LPF must be fast
 *   - L/R time constant: 30e-6/0.065 = 0.46 ms (11 ISR ticks)
 * ---------------------------------------------------------------- */

#define MOTOR_RS_OHM            0.065f
#define MOTOR_LS_H              30e-6f
/** λ_pm = 60/(sqrt3 * 2pi * KV * pp) = 60/(1.732 * 2pi * 1400 * 7) = 0.000563
 *  Per-phase flux linkage for FOC d-q frame.  Previous value (0.000975) was
 *  line-to-line Ke — missing √3 caused BEMF feedforward to consume 73% more
 *  voltage budget, limiting speed to ~8300 RPM instead of ~14300 RPM. */
#define MOTOR_KE_VPEAK          0.000563f
#define MOTOR_POLE_PAIRS_FOC    7
#define MOTOR_VBUS_NOM_V        12.0f
/** Peak Iq clamp — limited by MCLV-48V-300W board U25B (no LEB).
 *  Previous prop test: 6.9A Iq OK, 12A trips U25B from switching transients.
 *  15A — A2212 handles 15-20A burst in RC use.  U25B disabled for FOC.
 *  Production board (no U25B): raise to 20A+. */
#define MOTOR_MAX_CURRENT_A     15.0f
/** A2212 max electrical speed.
 *  6000 rad/s = 8163 RPM mech.  Speed PI + adaptive omega filter
 *  handle the full range.  MCLV board U25B may trip at very high
 *  duty due to no LEB — this is hardware, not a control limit. */
#define MOTOR_MAX_ELEC_RAD_S    6000.0f
/** Flux linkage = λ_pm (per-phase). */
#define MOTOR_FLUX_LINKAGE      0.000563f

/* D/Q Current Loop PI (BW ~ 1 kHz, parallel form)
 *   Kp = ωbw * Ls = 2pi*1000 * 30e-6 = 0.1885 ≈ 0.19
 *   Ki = ωbw * Rs = 2pi*1000 * 0.065 = 408  (parallel form)
 *   NOTE: Rs/Ls = 2167 is series-form Ki — NOT correct for our Tustin PI
 *   which uses: output = Kp*e + integral, integral += Ki*dt*0.5*(e+e_prev)
 *   First-tick: Kp*1A = 0.19V → 0.19/0.065 = 2.9A (safe, rated 20A) */
#define KP_DQ                   0.19f
#define KI_DQ                   408.0f

/* Speed Loop PI (outer loop — active after CL handoff)
 *   Very low gains needed: A2212 SMO has ±50 rad/s PLL jitter at
 *   500 rad/s.  Higher gains create growing speed oscillation.
 *   KI=0.02 proven in 57.5s CL run (500→9000 rad/s).
 *   KI=0.01 caused desync at 3500 — too slow to track pot sweep. */
#define KP_SPD                  0.0005f
#define KI_SPD                  0.02f

/* I/f (Current-Forced) Startup
 * PI controllers active from tick 0 — eliminates V/f current waste.
 * On low-Rs motors, V/f wastes Id ∝ sin(θ_err)/Rs → 5A+ at 0.065Ω.
 * I/f commands Id=0 and PI instantly cancels angle-error current.
 *
 * Alignment: hold θ=0, ramp Iq from 0 → ALIGN_IQ over ALIGN_TICKS.
 * OL ramp: advance θ at pot-controlled rate, PI maintains Id=0 + Iq=RAMP_IQ.
 * CL handoff: PLL tracks OL within tolerance → switch to PLL angle + speed PI. */
/** Alignment Id (A) — locks rotor at θ=0.
 *  A2212 Kt_FOC = 0.00591 N·m/A.  2.5A = 14.8 mN·m (overcomes cogging). */
#define STARTUP_ALIGN_IQ_A      2.5f
/** OL running Iq (A) — torque during forced-angle ramp.
 *  4A × Kt = 23.6 mN·m — enough for 8x4.5 prop.
 *  5A trips U25B OC on MCLV board (no LEB on comparator). */
#define STARTUP_RAMP_IQ_A       4.0f
/** Iq ramp ticks (24kHz).  2400 = 100ms — fast d→q rotation. */
#define STARTUP_IQ_RAMP_TICKS   2400U
/** Alignment dwell (ticks).  7200 = 300ms — allow prop inertia to settle. */
#define STARTUP_ALIGN_TICKS     7200U
/** 500 rad/s^2 — reaches handoff (1000) in 2s.
 *  Slow enough for 8x4.5 prop inertia. Adaptive ramp
 *  will further slow (or back off) if motor can't keep up. */
#define STARTUP_RAMP_RATE_RPS2  500.0f
/** V/f handoff at 500 rad/s — proven reliable on A2212.
 *  1000 rad/s caused rotor sync loss (Id=7-8A angle error).
 *  BEMF at 500 = 0.000563×500 = 0.28V (2.3% of Vbus) — marginal
 *  but sufficient with SMO adaptive LPF at this speed. */
#define STARTUP_HANDOFF_RAD_S   500.0f
/** Min OL speed at pot=0: 2000 rad/s ~ 2730 RPM mech. */
#define STARTUP_MIN_OL_RAD_S    2000.0f
/** At 12V: Vbus/Ke = 12/0.000975 = 12308, cap at 85% → 10000. */
#define STARTUP_MAX_OL_RAD_S    10000.0f

/** Startup mode: V/f required. Rs=65mΩ → PI at 4A gives only 0.26V.
 *  V/f at 1.0V provides 15A starting torque. */
#define STARTUP_USE_VF          1

/* Fault Thresholds */
#define FAULT_OC_A              25.0f
#define FAULT_STALL_RAD_S       50.0f

/* Observer LPF -- A2212 (high-speed motor: light filtering)
 * Max elec freq = 12000/(2pi) = 1910 Hz.
 * alpha=0.06 (Hurst default) gives 229 Hz cutoff -- way too low!
 * 0.35 -> cutoff ~ 1340 Hz: passes fundamental up to ~8000 rad/s
 * with < 30 deg phase lag. PLL compensates residual lag. */
#define OBS_LPF_ALPHA           0.35f

/* SMO tuning -- A2212 (low-Ls motor: heavy LPF needed)
 * K=Vbus=12V → switching noise ±12V. Need 2-stage LPF to attenuate
 * to below BEMF (0.28V at 500 rad/s handoff).
 * alpha=0.10 @ handoff → adaptive gives 0.05 at 500 rad/s.
 * 2-stage residual: 12V × 0.026² = 0.008V (vs 0.28V signal).
 * Signal loss: 14% amplitude, 2° phase lag — acceptable. */
#define SMO_LPF_ALPHA           0.10f

#elif MOTOR_PROFILE == 2
/* ----------------------------------------------------------------
 * Flycat i-Motor 5010-750KV (heavy-lift drone motor)
 * 12N14P, 7 pole pairs, 4S/14.8V nominal
 * Rs ~ 80 mohm, Ls ~ 30 uH (ESTIMATED -- measure on bench!)
 * 750 KV -> no-load 11100 RPM @ 14.8V
 * Typical prop: 14x4.7 to 16x5.5, frame: 450-850mm multirotor
 * ---------------------------------------------------------------- */

/** Phase resistance (ohm) -- ESTIMATED, needs bench measurement.
 *  RCTimer 5010 series: 60-120 mohm typical. Using 80 mohm as starting point. */
#define MOTOR_RS_OHM            0.080f
/** Phase inductance (H) -- ESTIMATED, needs LCR measurement.
 *  Similar 12N14P outrunners measure 20-50 uH. Using 30 uH. */
#define MOTOR_LS_H              30e-6f
/** λ_pm = 60/(sqrt3 * 2pi * 750 * 7) = 0.001050 V*s/rad_elec (per-phase) */
#define MOTOR_KE_VPEAK          0.001050f
#define MOTOR_POLE_PAIRS_FOC    7
#define MOTOR_VBUS_NOM_V        14.8f       /* 4S LiPo nominal */
#define MOTOR_MAX_CURRENT_A     30.0f
/** 11100 RPM * 7PP * 2pi/60 = 8134, round up for headroom. */
#define MOTOR_MAX_ELEC_RAD_S    8500.0f
/** Flux linkage = λ_pm (per-phase). */
#define MOTOR_FLUX_LINKAGE      0.001050f

/* D/Q Current Loop PI (BW ~ 1 kHz, parallel form)
 *   Kp = ωbw * Ls = 2pi*1000 * 30e-6 = 0.1885 ≈ 0.19
 *   Ki = ωbw * Rs = 2pi*1000 * 0.080 = 503  (parallel form) */
#define KP_DQ                   0.19f
#define KI_DQ                   503.0f

/* Speed Loop PI (outer loop — active after CL handoff) */
#define KP_SPD                  0.005f
#define KI_SPD                  0.5f

/* I/f (Current-Forced) Startup — heavier rotor + big prop
 * Kt_FOC = (3/2)*7*0.001050 = 0.01103 N·m/A (with √3-corrected λ_pm).
 * NOTE: With corrected Kt, consider bumping RAMP_IQ to 5-6A for heavy
 * 14"+ props — 3A gives only 33 mN·m which may be marginal under load. */
/** Alignment Id (A) — 3A × 0.01103 = 33 mN·m, locks 5010 rotor. */
#define STARTUP_ALIGN_IQ_A      3.0f
/** OL running Iq (A) — 3A for 14"+ prop inertia during acceleration.
 *  Consider 5-6A if prop stalls during I/f ramp (Kt*5 = 55 mN·m). */
#define STARTUP_RAMP_IQ_A       3.0f
/** Iq ramp ticks (24kHz).  12000 = 500ms — smooth transition. */
#define STARTUP_IQ_RAMP_TICKS   12000U
/** Alignment dwell (ticks).  18000 = 750ms — heavier rotor settling. */
#define STARTUP_ALIGN_TICKS     18000U
/** 100 rad/s^2 — slow ramp for 14" prop inertia.
 *  ~10 seconds to reach handoff at 1000 rad/s with full pot. */
#define STARTUP_RAMP_RATE_RPS2  100.0f
/** BEMF at 1000 = 0.001050*1000 = 1.05V — 7.1% of 14.8V Vbus. */
#define STARTUP_HANDOFF_RAD_S   1000.0f
#define STARTUP_MIN_OL_RAD_S    1500.0f     /* ~2045 RPM mech */
/** At 14.8V: Vbus/Ke = 14.8/0.001050 = 14095, cap at 85% → 12000. */
#define STARTUP_MAX_OL_RAD_S    12000.0f

/** Startup: V/f needed (Rs=80mΩ, low like A2212). */
#define STARTUP_USE_VF          1

/* Fault Thresholds */
#define FAULT_OC_A              35.0f
#define FAULT_STALL_RAD_S       50.0f

/* Observer LPF -- 5010 (moderate-speed motor)
 * Max elec freq = 8500/(2pi) = 1353 Hz.
 * alpha=0.20 -> cutoff ~ 765 Hz: adequate for speeds up to ~5000 rad/s.
 * At max speed, ~30 deg phase lag -- PLL compensates. */
#define OBS_LPF_ALPHA           0.20f

/* SMO tuning -- 5010 (moderately high-speed motor)
 * LPF cutoff ~ 2x max elec freq = 2 * 8500/(2pi) = 2706 Hz
 * alpha = 2pi * 2706 * Ts = 0.71, round to 0.70 */
#define SMO_LPF_ALPHA           0.70f

#elif MOTOR_PROFILE == 3
/* ----------------------------------------------------------------
 * Generic 5055 ~580KV (colleague's motor — ADJUST KV AS NEEDED)
 * Rs = 0.1 ohm phase-to-phase -> 0.05 ohm L-N
 * Ls = 35 uH phase-to-phase -> 17.5 uH L-N
 * Assumed: 14P (7 pole pairs), 4S-6S (tuned for 4S = 14.8V)
 * 580 KV -> no-load ~8584 RPM @ 14.8V
 *
 * KEY TUNING NOTES:
 *   - Very low Rs (0.05 ohm): small voltage = huge current
 *   - Heavy rotor + prop: MUST use slow ramp rate (50-100 rad/s^2)
 *   - Update KV/Ke/flux if your motor is a different KV rating
 * ---------------------------------------------------------------- */

/** Phase resistance, L-N (ohm). Rpp=0.1 -> Rs=0.05. */
#define MOTOR_RS_OHM            0.050f
/** Phase inductance, L-N (H). Lpp=35uH -> Ls=17.5uH. */
#define MOTOR_LS_H              17.5e-6f
/** λ_pm = 60/(sqrt3 * 2pi * KV * pp) = 60/(1.732 * 2pi * 580 * 7)
 *  = 0.001355 V*s/rad_elec (per-phase flux linkage).
 *  ADJUST THIS IF YOUR KV IS DIFFERENT. */
#define MOTOR_KE_VPEAK          0.001355f
#define MOTOR_POLE_PAIRS_FOC    7
#define MOTOR_VBUS_NOM_V        14.8f       /* 4S LiPo nominal */
/** Peak phase current limit — 5055 can handle 30A+ but start conservative. */
#define MOTOR_MAX_CURRENT_A     25.0f
/** 8584 RPM * 7PP * 2pi/60 = 6290, round up. */
#define MOTOR_MAX_ELEC_RAD_S    7000.0f
#define MOTOR_FLUX_LINKAGE      0.001355f

/* D/Q Current Loop PI (BW ~ 800 Hz — slightly detuned for low-Rs stability)
 *   Kp = ωbw * Ls = 2pi*800 * 17.5e-6 = 0.088
 *   Ki = ωbw * Rs = 2pi*800 * 0.050 = 251
 *   Lower BW than A2212 to prevent current ringing on very-low-Rs motor. */
#define KP_DQ                   0.088f
#define KI_DQ                   251.0f

/* Speed Loop PI */
#define KP_SPD                  0.003f
#define KI_SPD                  0.3f

/* I/f (Current-Forced) Startup — SLOW for heavy 5055 rotor
 *
 * CRITICAL: Ramp rate must be low enough that the rotor can physically
 * follow the forced angle. A 5055 rotor + 10"+ prop has 5-10x the
 * inertia of an A2212 + 8x4.5. Too fast = vibration + overcurrent.
 *
 * Kt_FOC = (3/2)*7*0.001355 = 0.01423 N-m/A */
/** Alignment Iq (A) — 4A * 0.01423 = 57 mN-m, locks heavy rotor. */
#define STARTUP_ALIGN_IQ_A      4.0f
/** OL running Iq (A) — conservative: let rotor track without fighting.
 *  3A gives 43 mN-m — enough for 10-12" prop without excess current. */
#define STARTUP_RAMP_IQ_A       3.0f
/** Iq ramp: 12000 ticks = 500ms — very gradual alignment. */
#define STARTUP_IQ_RAMP_TICKS   12000U
/** Alignment dwell: 24000 ticks = 1000ms — heavy rotor needs time. */
#define STARTUP_ALIGN_TICKS     24000U
/** 80 rad/s^2 — SLOW ramp for heavy rotor. A2212 uses 500!
 *  At 80, takes ~10s to reach handoff. Increase to 150 if reliable. */
#define STARTUP_RAMP_RATE_RPS2  80.0f
/** BEMF at 800 = 0.001355*800 = 1.08V — 7.3% of 14.8V Vbus.
 *  Adequate for PLL. Lower than A2212 because better Ke/Vbus ratio. */
#define STARTUP_HANDOFF_RAD_S   800.0f
/** Min OL speed at pot=0: 1200 rad/s ~ 1636 RPM mech. */
#define STARTUP_MIN_OL_RAD_S    1200.0f
/** At 14.8V: Vbus/Ke = 14.8/0.001355 = 10923, cap at 80%. */
#define STARTUP_MAX_OL_RAD_S    8700.0f

/** Startup: V/f needed (Rs=50mΩ, very low). */
#define STARTUP_USE_VF          1

/* Fault Thresholds */
#define FAULT_OC_A              30.0f
#define FAULT_STALL_RAD_S       30.0f

/* Observer LPF -- 5055 (moderate-speed motor)
 * Max elec freq = 7000/(2pi) = 1114 Hz.
 * alpha=0.20 -> cutoff ~ 765 Hz: adequate. */
#define OBS_LPF_ALPHA           0.20f

/* SMO tuning -- 5055
 * LPF cutoff ~ 2x max elec freq = 2228 Hz -> alpha = 0.58 */
#define SMO_LPF_ALPHA           0.58f

#else
#error "Unknown MOTOR_PROFILE for FOC params -- see garuda_config.h"
#endif

/* ================================================================
 *          SHARED PARAMETERS  (all motor profiles)
 * ================================================================ */

/* -- Dead-time compensation for observer -------------------------
 *   Dead time = DEADTIME_NS (500 ns for A2212/5010/5055, 750 ns for Hurst).
 *   dt_comp_frac = 2 × Td / Tpwm (fractional voltage error per Vbus).
 *   Observer: v_eff = v_cmd - dt_comp_v × sign(I), where
 *   dt_comp_v = Vbus × dt_comp_frac.
 *
 *   A2212 at 12V: 12 × 2×500ns/41.67µs = 0.288V.
 *   Without this, observer sees 0.288V systematic voltage error,
 *   causing ~7° angle drift per elec cycle at 4000 rad/s.
 *
 *   VESC always applies dead-time comp in observer. */
#define FOC_DT_COMP_FRAC  (2.0f * DEADTIME_NS * 1e-9f * PWMFREQUENCY_HZ)

/* -- Phase lag compensation for observer angle -------------------
 *   Digital control has 0.5-sample transport delay between ADC
 *   sample and PWM update. At high speed, this systematic lag
 *   causes commutation angle error:
 *     lag_rad = ω × dt × 0.5  (4.8° at 4000 rad/s)
 *
 *   VESC: phase += pll_speed × dt × (0.5 + offset)
 *   We add 0.5 × dt advance to observer angle for commutation. */
#define FOC_PHASE_LAG_COMP  0.5f

/* -- Timing (24 kHz switching frequency) ------------------------- */

/** Fast control loop sample period (s) -- 1/24000 = 41.67 us */
#define FOC_TS_FAST_S           (1.0f / 24000.0f)
/** Slow loop divisor: 24 ticks -> 1 kHz slow loop */
#define FOC_SLOW_DIV            24
/** Slow loop sample period (s) */
#define FOC_TS_SLOW_S           (FOC_TS_FAST_S * (float)FOC_SLOW_DIV)

/* -- D/Q axis voltage clamp -------------------------------------- */

#define CLAMP_VDQ               (MOTOR_VBUS_NOM_V * 0.95f * 0.577350269f)
/** Speed PI output clamp: peak Iq reference (A) */
#define CLAMP_IQ_REF_A          MOTOR_MAX_CURRENT_A

/* -- PLL Estimator (target BW ~ 50 Hz) --------------------------- */
/*   Reduced from 200 Hz -- previous Kp=2513 too aggressive for
 *   low-BEMF conditions (any small angle error slams omega to clamp).
 *   Kp = 2pi * 50 * 2 = 628
 *   Ki = Kp^2 / 4 = 98600  (critically damped)
 *   Microchip reference uses ~55 Hz velocity filter (similar BW). */
#define PLL_KP                  628.0f
#define PLL_KI                  98600.0f
/** PLL output speed clamp (rad/s elec.) */
#define PLL_SPEED_CLAMP         MOTOR_MAX_ELEC_RAD_S
/** PLL angle offset (rad): BEMF leads rotor by pi/2.
 *  Subtract pi/2 to get rotor angle for Park/InvPark. */
#define PLL_ANGLE_OFFSET        1.5707963f   /* pi/2 = 90 deg */
/** PLL correction BW (Hz) -- gradual OL->PLL tracking.
 *  50 Hz → 0.0131 rad/tick → 3ms settling. First-order loop: stable,
 *  no overshoot. At 200 rad/s^2 accel, steady-state error = 0.12°.
 *  5 Hz was too slow (10.8° error at 3883 rad/s → 5A waste).
 *  Direct PLL assignment stalled motor (800 BEMF too weak).
 *  50 Hz is smooth + fast — best of both. */
#define PLL_CORRECTION_BW_HZ   50.0f

/* -- Back-EMF Observer (Voltage Model) --------------------------- */

/** Observer design pole (~10x current-loop BW) -- documentation only. */
#define OBS_LAMBDA              62832.0f
/* OBS_LPF_ALPHA: per-profile (above) -- EMA coefficient for observer
 * output filtering. Higher for high-speed motors to avoid phase lag.
 *   Hurst (Profile 0):  0.06  (229 Hz cutoff -- high-Ke, low-speed)
 *   A2212 (Profile 1):  0.35  (1340 Hz cutoff -- low-Ke, high-speed)
 *   5010  (Profile 2):  0.20  (765 Hz cutoff -- moderate speed)
 *
 * Observer EMA phase lag = (1-alpha)/alpha * omega_e * Ts.
 * At 2537 rad/s with alpha=0.35: 11.2 deg. This lag biases the PLL
 * angle behind the true rotor. OBS_LAG_COEFF used for compensation. */
#define OBS_LAG_COEFF           ((1.0f - OBS_LPF_ALPHA) / OBS_LPF_ALPHA)
#define OBS_LAG_COMP_MAX_RAD    0.52f   /* Clamp at 30 deg for safety */

/* -- Sliding Mode Observer (SMO) — V4 correct classical SMO ------- */
/*   SMO_K_BASE: minimum sliding gain (V). Must maintain sliding
 *     condition: G×K > max current error. Cap at 25% Vbus.
 *   SMO_K_BEMF_SCALE: K_bemf = scale × λ × |ω| ensures convergence
 *     at high speed.
 *   SMO_K_ADAPT_ALPHA: LP filter rate for adaptive K tracking.
 *   SMO_SIGMOID_PHI: boundary layer width for sigmoid switching.
 *   SMO_LPF_ALPHA: per-profile (above) — base alpha at omega_ref.
 *   SMO_ALPHA_MIN/MAX: adaptive LPF clamp range.
 *   SMO_CONF_MIN: health threshold for observable flag.
 *   SMO_RESIDUAL_MAX: health threshold for observable flag.
 *   SMO_*_HANDOFF: handoff-specific thresholds. */

/** Legacy SMO gain for V1 smo_observer.c */
#define SMO_GAIN                MOTOR_VBUS_NOM_V

/** Minimum sliding gain (V).
 *  K_base = 1.5·Ls/dt, capped at 25% Vbus.
 *  A2212: 1.5×30e-6/41.67e-6 = 1.08V (< 3V cap).
 *  Hurst: 1.5×471e-6/41.67e-6 = 16.9V → capped to 6.0V. */
/* Computed at init time in SMO_Config_t — these are defaults.
 * 1.5·Ls/dt capped at 25% Vbus. */
#define SMO_K_BASE_RAW      (1.5f * MOTOR_LS_H / FOC_TS_FAST_S)
#define SMO_K_BASE_CAP      (MOTOR_VBUS_NOM_V * 0.25f)
#define SMO_K_BASE          ((SMO_K_BASE_RAW > SMO_K_BASE_CAP) ? \
                             SMO_K_BASE_CAP : SMO_K_BASE_RAW)

/** BEMF-proportional floor: K >= scale × λ × |ω| */
#define SMO_K_BEMF_SCALE    1.5f

/** LP filter rate for adaptive K (0.01 = ~4ms settling) */
#define SMO_K_ADAPT_ALPHA   0.01f

/** Dead-time compensation mode for observer voltage reconstruction.
 *  0 = none (raw duty), 1 = per-phase ABC, 2 = αβ approximation.
 *  A2212 dt_comp=0.29V ≈ BEMF at handoff — DT comp error can
 *  dominate the signal. Test 0 vs 1 vs 2 to find best residual. */
#define SMO_DT_COMP_MODE   1

/** Sigmoid boundary layer thickness (A).
 *  0.1A: sharp tracking, smooth enough for ADC noise floor. */
#define SMO_SIGMOID_PHI     0.1f

/** Adaptive LPF clamp range.
 *  alpha_max=0.30 keeps 2-stage LPF active at high speed.
 *  At 4200 rad/s: -10 dB 3rd harmonic rejection, 0.9 dB fundamental loss.
 *  Phase delay = ~1.1 rad (64°) — compensated by arctan model. */
#define SMO_ALPHA_MIN       0.02f
#define SMO_ALPHA_MAX       0.30f

/** Observer health thresholds (for observable flag) */
#define SMO_CONF_MIN        0.10f     /* min confidence for observable=true */
#define SMO_RESIDUAL_MAX    5.0f      /* max residual for observable=true (A) */

/** Handoff-specific thresholds (can be tighter than runtime) */
#define SMO_CONF_MIN_HANDOFF     0.15f
#define SMO_RESIDUAL_MAX_HANDOFF 3.0f
#define SMO_BEMF_MIN_HANDOFF     0.01f  /* min BEMF mag for handoff (V) */

/** PLL speed-adaptive bandwidth tunables.
 *  BW scales linearly from bw_min at ω=0 to bw_max at ω=omega_bw_ref.
 *  Rule of thumb: PLL BW should be ~10-25% of electrical frequency.
 *  20 Hz min → 60 Hz max covers handoff (262-500 rad/s) to full speed. */
#define SMO_PLL_BW_MIN_HZ   20.0f    /* Min BW at low speed (Hz) */
#define SMO_PLL_BW_MAX_HZ   60.0f    /* Max BW at high speed (Hz) */
#define SMO_PLL_BW_REF_FRAC 0.5f     /* BW=max at this fraction of omega_max */

/* -- Startup Mode Flags ------------------------------------------ */

/** Enable closed-loop FOC handoff from open-loop I/f.
 *  1: PLL correction at STARTUP_HANDOFF_RAD_S, then CL transition.
 *  0: stay in I/f open-loop forever. */
#define FEATURE_CLOSED_LOOP     1

/** I/f → closed-loop transition.
 *  During I/f: PI current control (Id=0, Iq=RAMP_IQ), forced angle.
 *  After PLL angle tracks OL within CL_ANGLE_TOL for CL_LOCK_COUNT
 *  consecutive fast-loop ticks: switch to PLL angle + speed PI.
 *  Requires FEATURE_CLOSED_LOOP=1. */

/** PLL-to-OL angle error tolerance for CL handoff (rad).
 *  15 degrees = 0.262 rad — conservative; allows for observer noise. */
#define CL_ANGLE_TOL_RAD       0.262f

/** Consecutive ticks PLL must track within tolerance before CL handoff.
 *  480 ticks at 24kHz = 20ms — enough to confirm stable lock. */
#define CL_LOCK_COUNT           480U

/** Max Iq reference in closed-loop speed-PI mode (A). */
#define CL_IQ_MAX_A            MOTOR_MAX_CURRENT_A

/* -- CL ↔ V/f Assist hysteresis --------------------------------- */
/** Enter CL when omega > CL_ENTER_RAD_S with good SMO health.
 *  Equal to handoff speed — proven reliable for initial CL entry. */
#define CL_ENTER_RAD_S         STARTUP_HANDOFF_RAD_S

/** Exit CL to V/f assist when omega < CL_EXIT_RAD_S.
 *  Must be < CL_ENTER to provide hysteresis band.
 *  300 rad/s = 60% of handoff — innovation ~0.6 rad at this speed,
 *  still acceptable for maintaining loose SMO tracking. */
#define CL_EXIT_RAD_S          300.0f

/** V/f assist speed floor — minimum speed to hold in V/f assist.
 *  Below this, throttle=0 causes full stop (re-arm). */
#define VF_ASSIST_MIN_RAD_S    100.0f

/* -- Throttle (POT ADC on RA11 / AD1CH1) ------------------------- */

/** ADC counts below this = zero throttle dead-band. */
#define THROTTLE_DEADBAND       300U
/** Scale ADC (0-4095) -> electrical speed reference (rad/s). */
#define THROTTLE_SPEED_SCALE    (MOTOR_MAX_ELEC_RAD_S / 4095.0f)

/* -- Fault Thresholds (shared) ----------------------------------- */

/** Stall timeout in slow-loop ticks (1 kHz -> 1 tick per ms). */
#define FAULT_STALL_TIMEOUT_MS  500U

/* -- Arming ------------------------------------------------------ */

/** Throttle-at-zero duration before ARMED -> STARTUP (ms). */
#define FOC_ARM_TIME_MS         500U

/* -- Current Sensing Hardware (MCLV-48V-300W + DIM EV68M17A) ------
 *   Shunt: 3 mohm low-side (on MCLV-48V-300W board, NOT on DIM)
 *   Op-amp: DIM differential amp (U1A/U1B), gain = RF/RIN = 4990/200 = 24.95
 *     I_peak = Vref_half/(Gain*Shunt) = 1.65/(24.95*0.003) = 22.04 A
 *     Matches AN1292 MC1_PEAK_CURRENT = 22.0 A
 *   Polarity: inverting topology — raw_to_amps() negates.
 *   ADC: 3.3V reference, 12-bit (0-4095), mid-point ~2048
 * ---------------------------------------------------------------- */
#define SHUNT_RESISTANCE_OHM    0.003f
#define OPAMP_GAIN              24.95f
#define ADC_VREF_V              3.3f
#define ADC_FULL_SCALE_F        4095.0f
#define ADC_MIDPOINT            2048

/** Amps per ADC count (signed, centred at zero-current offset).
 *  I_peak = Vref_half/(Gain*Shunt) = 1.65/(24.95*0.003) = 22.04 A
 *  Scale = Vref/(FS*Gain*Shunt) = 3.3/(4095*24.95*0.003) = 0.01077 A/count */
#define CURRENT_SCALE_A_PER_COUNT \
    (ADC_VREF_V / (ADC_FULL_SCALE_F * OPAMP_GAIN * SHUNT_RESISTANCE_OHM))

/* -- Bus Voltage Sensing (MCLV-48V-300W board) --------------------
 *   Calibrated: 12.1V actual / 8.07V displayed = correction 1.50
 *   x original 15.47 = 23.2
 * ---------------------------------------------------------------- */
#define VBUS_DIVIDER_RATIO      23.2f
#define VBUS_SCALE_V_PER_COUNT  (ADC_VREF_V * VBUS_DIVIDER_RATIO / ADC_FULL_SCALE_F)

#endif /* GARUDA_FOC_PARAMS_H */
