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
 *  Auto-detect measured 471µH (31% higher). */
#define MOTOR_LS_H              0.000359f
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
 *   Kp = ωbw × Ls = 2π×530 × 0.000359 = 1.195 ≈ 1.20
 *   Ki = ωbw × Rs = 2π×530 × 0.534 = 1778
 *   PI zero = Ki/Kp = 1778/1.20 = 1482 ≈ Rs/Ls = 0.534/0.000359 = 1488
 *   → pole cancellation with measured Rs. */
#define KP_DQ                   1.20f
#define KI_DQ                   1778.0f

/* Speed Loop PI (outer loop — active after CL handoff)
 *   Kt_FOC = (3/2)×5×0.00742 = 0.0557 N·m/A.
 *   Microchip reference: Kp=0.00573, Ki=0.0000129.
 *   Using slightly higher for better pot response. */
#define KP_SPD                  0.010f
#define KI_SPD                  1.0f

/* I/f (Current-Forced) Startup
 * Microchip reference: OPEN_LOOP_CURRENT=1.0A, LOCK_CURRENT=1.0A,
 * handoff at 500 RPM, ramp rate 2000 RPM/s. */
/** Alignment Id (A) — 1.0A matches Microchip reference LOCK_CURRENT. */
#define STARTUP_ALIGN_IQ_A      1.0f
/** OL running Iq (A) — 1.0A matches Microchip OPEN_LOOP_CURRENT. */
#define STARTUP_RAMP_IQ_A       1.0f
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

/* Observer LPF -- long Hurst (moderate-speed motor)
 * Max elec freq = 2000/(2pi) = 318 Hz.
 * Microchip BEMF filter cutoff: 250 Hz → alpha ≈ 0.065.
 * Using 0.08: slightly faster than short Hurst (0.06). */
#define OBS_LPF_ALPHA           0.08f
#define SMO_LPF_ALPHA           0.20f

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
#define MOTOR_MAX_CURRENT_A     20.0f
/** 16800 RPM * 7PP * 2pi/60 = 12315, round to 12000 */
#define MOTOR_MAX_ELEC_RAD_S    12000.0f
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

/* Speed Loop PI (outer loop — active after CL handoff) */
#define KP_SPD                  0.005f
#define KI_SPD                  0.5f

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
/** BEMF at 1000 = 0.000563*1000 = 0.56V (4.7% of Vbus).
 *  A2212 has excellent Ls/λ ratio → observer works at low BEMF. */
#define STARTUP_HANDOFF_RAD_S   1000.0f
/** Min OL speed at pot=0: 2000 rad/s ~ 2730 RPM mech. */
#define STARTUP_MIN_OL_RAD_S    2000.0f
/** At 12V: Vbus/Ke = 12/0.000975 = 12308, cap at 85% → 10000. */
#define STARTUP_MAX_OL_RAD_S    10000.0f

/* Fault Thresholds */
#define FAULT_OC_A              25.0f
#define FAULT_STALL_RAD_S       50.0f

/* Observer LPF -- A2212 (high-speed motor: light filtering)
 * Max elec freq = 12000/(2pi) = 1910 Hz.
 * alpha=0.06 (Hurst default) gives 229 Hz cutoff -- way too low!
 * 0.35 -> cutoff ~ 1340 Hz: passes fundamental up to ~8000 rad/s
 * with < 30 deg phase lag. PLL compensates residual lag. */
#define OBS_LPF_ALPHA           0.35f

/* SMO tuning -- A2212 (high-speed motor: fast LPF)
 * LPF cutoff ~ 2x max elec freq = 2 * 12000/(2pi) = 3820 Hz
 * alpha = 2pi * 3820 * Ts = 1.0, cap at 0.80 (PLL handles rest) */
#define SMO_LPF_ALPHA           0.80f

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

/* -- Sliding Mode Observer (SMO) --------------------------------- */
/*   Parallel observer with inherent robustness to Rs/Ls errors.
 *   Runs alongside the voltage-model observer for comparison.
 *   Outputs feed a separate PLL instance for angle/speed estimation.
 *
 *   SMO_GAIN: must exceed max BEMF. Using Vbus_nom (BEMF < Vbus
 *     at any operating point under normal conditions).
 *   SMO_SIGMOID_PHI: boundary layer width for sigmoid switching
 *     function. Controls chattering vs tracking tradeoff (amps).
 *   SMO_LPF_ALPHA: per-profile (above) -- filters switching signal
 *     to extract back-EMF. Higher for high-speed motors. */

/** Sliding gain (V) -- auto-derived from motor profile Vbus. */
#define SMO_GAIN                MOTOR_VBUS_NOM_V

/** Sigmoid boundary layer thickness (A).
 *  Transition region width for F(x) = x/(|x|+phi).
 *  0.1A: sharp enough for good tracking, smooth enough to suppress
 *  chattering at the ADC noise floor (~4 LSB * 0.01 A/count = 0.04A). */
#define SMO_SIGMOID_PHI         0.1f

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
