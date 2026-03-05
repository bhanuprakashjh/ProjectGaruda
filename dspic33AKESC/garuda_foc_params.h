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
 * Hurst DMB0224C10002 (dev bench motor)
 * 10 poles, 24VDC, 1.0A rated, L-L 4.03 ohm / 4.60 mH
 * No-load 3125 RPM @ 24V, Ke 7.24 Vpk/KRPM
 * ---------------------------------------------------------------- */

/** Phase resistance, line-to-neutral (ohm).
 *  Microchip reference (hurst075.h) measures 2.54 ohm for this motor.
 *  Our earlier value (L-L/2 = 2.015) under-compensated Rs in the observer. */
#define MOTOR_RS_OHM            2.54f
/** Phase inductance, line-to-neutral (H).  L-L = 4.60 mH -> Ls = 2.30 mH */
#define MOTOR_LS_H              2.30e-3f
/** Back-EMF constant (V*s/rad, electrical).
 *  Ke_mech = 7.24e-3 Vpk/RPM * 60/(2pi) = 0.06914 V*s/rad_mech
 *  Ke_elec = 0.06914 / 5 = 0.01383 */
#define MOTOR_KE_VPEAK          0.01383f
#define MOTOR_POLE_PAIRS_FOC    5
#define MOTOR_VBUS_NOM_V        24.0f
/** Peak phase current limit (A) -- 5x rated for startup transient margin. */
#define MOTOR_MAX_CURRENT_A     5.0f
/** Max electrical speed (rad/s).  3125 RPM * 5PP * 2pi/60 = 1636, round up. */
#define MOTOR_MAX_ELEC_RAD_S    2000.0f
/** Flux linkage (V-s/rad_elec) = Ke_elec.
 *  Used by MXLEMMING observer for flux magnitude clamping. */
#define MOTOR_FLUX_LINKAGE      0.01383f

/* D/Q Current Loop PI (BW ~ 1 kHz, parallel form)
 *   Kp = ωbw * Ls = 2pi*1000 * 2.30e-3 = 14.45
 *   Ki = ωbw * Rs = 2pi*1000 * 2.54 = 15959 (parallel form)
 *   Using 876 from original Microchip tuning — conservative, proven stable. */
#define KP_DQ                   14.45f
#define KI_DQ                   876.0f

/* Speed Loop PI (outer loop — active after CL handoff) */
#define KP_SPD                  0.005f
#define KI_SPD                  0.5f

/* I/f (Current-Forced) Startup
 * Alignment: hold θ=0, ramp Iq from 0 → ALIGN_IQ over ALIGN_TICKS.
 * OL ramp: advance θ at pot-controlled rate, PI maintains Id=0 + Iq=RAMP_IQ.
 * CL handoff: when PLL tracks OL within tolerance, switch to PLL angle + speed PI. */
/** Alignment Iq (A) — 0.2A × 2.54Ω = 0.51V, enough to lock Hurst rotor. */
#define STARTUP_ALIGN_IQ_A      0.20f
/** OL running Iq (A) — torque current during forced-angle ramp.
 *  Hurst: low friction, 0.5A is ample (rated 1.0A). */
#define STARTUP_RAMP_IQ_A       0.50f
/** Iq ramp ticks (24kHz).  12000 = 500ms — gradual Iq transition. */
#define STARTUP_IQ_RAMP_TICKS   12000U
/** Alignment dwell (ticks).  24000 = 1000ms — lets rotor fully settle. */
#define STARTUP_ALIGN_TICKS     24000U
/** Ramp rate (rad/s^2 electrical).  50 → ~5s to handoff with full pot. */
#define STARTUP_RAMP_RATE_RPS2  50.0f
/** PLL correction begins above this speed (rad/s elec.).
 *  BEMF = 0.01383*260 = 3.6V — good PLL SNR. */
#define STARTUP_HANDOFF_RAD_S   260.0f
/** Min OL speed (pot=0): 1000 RPM * 5PP * 2pi/60 = 523.6 */
#define STARTUP_MIN_OL_RAD_S    523.6f
/** Max OL speed (pot=max): Vbus/Ke = 24/0.01383 = 1736, cap at 85%. */
#define STARTUP_MAX_OL_RAD_S    735.0f

/* Fault Thresholds */
#define FAULT_OC_A              10.0f
#define FAULT_STALL_RAD_S       10.0f

/* Observer LPF -- Hurst (low-speed motor: heavy filtering OK)
 * Max elec freq = 2000/(2pi) = 318 Hz
 * OBS cutoff = 0.06 -> 229 Hz: fine, BEMF signal is strong (high Ke).
 * SMO cutoff ~ 2x max = 637 Hz -> alpha = 0.17, round to 0.15 */
#define OBS_LPF_ALPHA           0.06f
#define SMO_LPF_ALPHA           0.15f

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
/** Ke_elec = 60/(2pi * 1400 * 7) = 0.000975 V*s/rad_elec */
#define MOTOR_KE_VPEAK          0.000975f
#define MOTOR_POLE_PAIRS_FOC    7
#define MOTOR_VBUS_NOM_V        12.0f
#define MOTOR_MAX_CURRENT_A     20.0f
/** 16800 RPM * 7PP * 2pi/60 = 12315, round to 12000 */
#define MOTOR_MAX_ELEC_RAD_S    12000.0f
/** Flux linkage (V-s/rad_elec) = Ke_elec. */
#define MOTOR_FLUX_LINKAGE      0.000975f

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
/** Alignment Iq (A) — must overcome cogging torque + prop friction.
 *  A2212 Kt_FOC = (3/2)*7*0.000975 = 0.01024 N·m/A.
 *  Cogging ~10-20 mN·m → need ≥2A. 3A gives 30.7 mN·m (3× margin).
 *  Power: 3²×0.065 = 0.585W/phase — negligible for 500ms alignment. */
#define STARTUP_ALIGN_IQ_A      3.0f
/** OL running Iq (A) — torque current during forced-angle ramp.
 *  Must overcome prop drag (A2212 + 8x4.5) during acceleration.
 *  At 4A: 41 mN·m torque — strong enough for 8" prop inertia. */
#define STARTUP_RAMP_IQ_A       4.0f
/** Iq ramp ticks (24kHz).  6000 = 250ms — smooth alignment→running. */
#define STARTUP_IQ_RAMP_TICKS   6000U
/** Alignment dwell (ticks).  12000 = 500ms — locks rotor position. */
#define STARTUP_ALIGN_TICKS     12000U
/** 500 rad/s^2 — reaches handoff (1500) in 3s with full pot. */
#define STARTUP_RAMP_RATE_RPS2  500.0f
/** BEMF at 1500 = 0.000975*1500 = 1.46V (12.2% of Vbus).
 *  PLL reliable at this level. */
#define STARTUP_HANDOFF_RAD_S   1500.0f
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
/** Ke_elec = 60/(2pi * 750 * 7) = 0.001819 V*s/rad_elec */
#define MOTOR_KE_VPEAK          0.001819f
#define MOTOR_POLE_PAIRS_FOC    7
#define MOTOR_VBUS_NOM_V        14.8f       /* 4S LiPo nominal */
#define MOTOR_MAX_CURRENT_A     30.0f
/** 11100 RPM * 7PP * 2pi/60 = 8134, round up for headroom. */
#define MOTOR_MAX_ELEC_RAD_S    8500.0f
/** Flux linkage (V-s/rad_elec) = Ke_elec. */
#define MOTOR_FLUX_LINKAGE      0.001819f

/* D/Q Current Loop PI (BW ~ 1 kHz, parallel form)
 *   Kp = ωbw * Ls = 2pi*1000 * 30e-6 = 0.1885 ≈ 0.19
 *   Ki = ωbw * Rs = 2pi*1000 * 0.080 = 503  (parallel form) */
#define KP_DQ                   0.19f
#define KI_DQ                   503.0f

/* Speed Loop PI (outer loop — active after CL handoff) */
#define KP_SPD                  0.005f
#define KI_SPD                  0.5f

/* I/f (Current-Forced) Startup — heavier rotor + big prop */
/** Alignment Iq (A) — 2A locks heavy 5010 rotor against prop drag. */
#define STARTUP_ALIGN_IQ_A      2.0f
/** OL running Iq (A) — 3A for 14"+ prop inertia during acceleration. */
#define STARTUP_RAMP_IQ_A       3.0f
/** Iq ramp ticks (24kHz).  12000 = 500ms — smooth transition. */
#define STARTUP_IQ_RAMP_TICKS   12000U
/** Alignment dwell (ticks).  18000 = 750ms — heavier rotor settling. */
#define STARTUP_ALIGN_TICKS     18000U
/** 100 rad/s^2 — slow ramp for 14" prop inertia.
 *  ~10 seconds to reach handoff at 1000 rad/s with full pot. */
#define STARTUP_RAMP_RATE_RPS2  100.0f
/** BEMF at 1000 = 0.001819*1000 = 1.82V — 12.3% of 14.8V Vbus. */
#define STARTUP_HANDOFF_RAD_S   1000.0f
#define STARTUP_MIN_OL_RAD_S    1500.0f     /* ~2045 RPM mech */
/** At 14.8V: Vbus/Ke = 14.8/0.001819 = 8136, cap at 85% → 7000. */
#define STARTUP_MAX_OL_RAD_S    7000.0f

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

/* -- Current Sensing Hardware (MCLV-48V-300W DIM EV68M17A) --------
 *   Shunt: 10 mohm low-side (DIM shunts)
 *   Op-amp gain: x8 (OA1/OA2 external resistors on DIM for phase current)
 *     Confirmed by: garuda_calc_params.h comment, RK1 reference (OPAMP_GAIN=8.0),
 *     AN1292 MC1_PEAK_CURRENT=22A matches 1.65/(8*0.010)=20.6A.
 *     NOTE: OA3 (bus current) uses different resistors → gain=24.95.
 *   Polarity: OA1/OA2 inverting topology — counts_to_amps_cal() negates.
 *     AN1292 uses (HALF_ADC_COUNT - ADC_DATA) for same reason.
 *   ADC: 3.3V reference, 12-bit (0-4095), mid-point ~2048
 * ---------------------------------------------------------------- */
#define SHUNT_RESISTANCE_OHM    0.010f
#define OPAMP_GAIN              8.0f
#define ADC_VREF_V              3.3f
#define ADC_FULL_SCALE_F        4095.0f
#define ADC_MIDPOINT            2048

/** Amps per ADC count (signed, centred at zero-current offset).
 *  I_peak = Vref_half/(Gain*Shunt) = 1.65/(8*0.010) = 20.6 A
 *  Scale = Vref/(FS*Gain*Shunt) = 3.3/(4095*8*0.010) = 0.01007 A/count */
#define CURRENT_SCALE_A_PER_COUNT \
    (ADC_VREF_V / (ADC_FULL_SCALE_F * OPAMP_GAIN * SHUNT_RESISTANCE_OHM))

/* -- Bus Voltage Sensing (MCLV-48V-300W board) --------------------
 *   Calibrated: 12.1V actual / 8.07V displayed = correction 1.50
 *   x original 15.47 = 23.2
 * ---------------------------------------------------------------- */
#define VBUS_DIVIDER_RATIO      23.2f
#define VBUS_SCALE_V_PER_COUNT  (ADC_VREF_V * VBUS_DIVIDER_RATIO / ADC_FULL_SCALE_F)

#endif /* GARUDA_FOC_PARAMS_H */
