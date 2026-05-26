# FOC V3 Implementation — SMO Sensorless Closed-Loop

**Date:** 2026-03-11 (updated with review corrections)
**Status:** CL achieved, observer path not yet fully correct — see §9
**Motor:** A2212 1400KV (7pp) + 8×4.5 prop on MCLV-48V-300W at 12V
**Best result:** 57.5s CL, pot-tracked 500→9000 rad/s (CSV: `logs/smo_sweep_20260311_163921.csv`)
**Caveat:** Headline result obtained with known inconsistencies in observer path (see §9.5). Results are engineering-journal grade, not conclusive.

---

## Table of Contents
1. [Architecture Overview](#1-architecture-overview)
2. [SMO Observer](#2-smo-observer)
3. [PLL Speed Estimation](#3-pll-speed-estimation)
4. [Startup Sequence](#4-startup-sequence)
5. [Closed-Loop Commutation](#5-closed-loop-commutation)
6. [Speed Control Loop](#6-speed-control-loop)
7. [Protection & Fault Detection](#7-protection--fault-detection)
8. [Bugs & Lessons Learned](#8-bugs--lessons-learned)
9. [Open Issues & Questions](#9-open-issues--questions)
   - 9.5 [Known Observer Path Inconsistencies](#95-known-observer-path-inconsistencies-fixed--remaining)
   - 9.6 [Recommended Implementation Path](#96-recommended-implementation-path)
10. [A2212 vs Hurst Comparison](#10-a2212-vs-hurst-comparison)

---

## 1. Architecture Overview

### Files
| File | Purpose |
|------|---------|
| `foc/foc_v3_control.c` | State machine, fast loop (24kHz), slow loop (1kHz) |
| `foc/foc_v3_smo.c` | SMO observer + PLL implementation |
| `foc/foc_v3_smo.h` | SMO/PLL public API |
| `foc/foc_v3_types.h` | Data structures (SMO_Observer_t, SMO_PLL_t, V3_State_t) |
| `garuda_foc_params.h` | Per-motor-profile parameters (4 profiles) |

### Control Pipeline (per fast tick, 24 kHz)
```
ADC → Current (Ia,Ib) → Clarke (Iα,Iβ)
                                  ↓
Prev duties → Voltage recon (Vα,Vβ) → SMO observer → PLL
                                                        ↓
                                          Phase comp → θ_drive
                                                        ↓
(Iα,Iβ) → Park(θ_drive) → (Id,Iq) → PI_d, PI_q → (Vd,Vq)
                                                        ↓
                              Circular clamp → InvPark → SVPWM → PWM duties
```

### Slow Loop (1 kHz)
```
Throttle → Speed target → Speed PI → Iq_ref
                              ↑
                    st->omega (LP-filtered PLL)
```

### State Machine
```
IDLE → ARMED → ALIGN → OL_RAMP → CLOSED_LOOP
                                       ↓ (phantom/low-speed/throttle=0)
                                     ARMED (clean re-arm, auto-restart)
                                       ↓ (desync/OC/board fault)
                                     FAULT (latching)
```

---

## 2. SMO Observer

**Reference:** Microchip AN1078 (adapted from Q15 to float for dsPIC33AK FPU)

### Algorithm
```
1. Current estimation error:     err = Î - I_measured
2. Switching function:           Z = K · sigmoid(err, φ)
3. Current model (forward Euler): Î[k+1] = F·Î[k] + G·(V - E - Z)
4. BEMF extraction (2-stage LPF): E₁ = LPF(Z),  E₂ = LPF(E₁)
5. Angle:                        θ = atan2(-Eα_filt, Eβ_filt)
```

### Key Parameters (A2212)
| Parameter | Value | Notes |
|-----------|-------|-------|
| F (plant pole) | 0.910 | 1 - Rs·dt/Ls = 1 - 0.065×41.67µs/30µH |
| G (plant gain) | 1.389 | dt/Ls = 41.67µs/30µH |
| K_base | 1.08V | 1.5·Ls/dt, capped at 25% Vbus |
| K_max | 12V | = Vbus |
| φ (sigmoid) | 0.1A | Boundary layer width |
| LPF α (base) | 0.10 | Speed-adaptive: α × |ω|/ω_ref |

### Sigmoid vs Signum
```
sigmoid(x, φ) = x / (|x| + φ)
```
Smooth approximation of sign(x). |F(x)| < 1 always. Reduces chattering while maintaining sliding mode convergence. φ=0.1A chosen as balance between tracking sharpness and ADC noise floor (~0.04A).

### Adaptive K (Sliding Gain)
```
K_target = K_base×0.5 + K_base × err_mag_filt
K_adapt  += 0.01 × (K_target - K_adapt)       // LP filter, τ≈4ms
K_bemf   = 1.5 × λ_pm × |ω|                   // BEMF-proportional floor
K_now    = max(K_bemf, K_adapt), clamped to K_max
```
- Large error (startup/transient): K rises → maintain sliding condition
- Small error (converged): K falls → less chattering → cleaner BEMF
- K_bemf ensures convergence at high speed even if K_adapt lags

### 2-Stage Cascaded LPF
**Why 2 stages?** K=Vbus=12V creates ±12V switching noise. BEMF at handoff (500 rad/s) is only 0.28V. Single-stage LPF at α=0.10 leaves 1.2V residual (4× signal). Two stages: residual = 12V × (α_eff)² ≈ 0.008V.

```
Stage 1: E₁ += α × (Z - E₁)    // fed back to current model
Stage 2: E₂ += α × (E₁ - E₂)   // used for angle extraction
```

Speed-adaptive alpha: `α = base_α × |ω|/ω_ref`, clamped [0.02, 0.95].
At 500 rad/s (handoff): α = 0.10 × 500/500 = 0.10 → effective per-stage α = 0.05.
At 5000 rad/s: α = 0.10 × 5000/500 = 1.0 → clamped to 0.95.

### Confidence Metric
```
confidence = LP(|E_filt|) / (λ_pm × |ω|)
```
- Converged: confidence ≈ 0.5–1.0 (A2212 peaks at ~0.21 due to 2-stage LPF attenuation)
- Non-converged: confidence < 0.1
- Used for handoff gating (threshold 0.15)

### Rs Online Adaptation (DISABLED)
Adaptation law: `dRs = gain × (err·i_meas) / |I|²`
- **Disabled** because at borderline speed (~500 rad/s), intermittent adaptation modifies SMO model → angle drift → desync
- Will re-enable once CL is stable at higher speeds

---

## 3. PLL Speed Estimation

### Architecture
Second-order PI-type PLL tracking the SMO angle:
```
phase_err = θ_smo - θ_pll          (wrapped to ±π)
ω_pll    += Ki × phase_err × dt    (integrator → speed)
θ_pll    += (ω_pll + Kp × phase_err) × dt
```

### Speed-Adaptive Bandwidth
```
BW = bw_min + frac × (bw_max - bw_min)
frac = |ω_pll| / ω_bw_ref, clamped [0, 1]
```
| Speed | BW | Purpose |
|-------|----|---------|
| 0 (startup) | 20 Hz | Heavy noise rejection |
| ω_bw_ref/2 | 40 Hz | Moderate tracking |
| ω_bw_ref | 60 Hz | Fast tracking at speed |

Damping ratio ζ = 1.2 (slightly overdamped for smoother transients).

### Unidirectional Clamp
`ω_pll` clamped to [0, ω_max]. Allowing negative omega causes PLL to lock onto wrong direction at low speed → positive feedback desync.

### Why PLL Angle for Commutation (Not Raw SMO atan2)
**The drift problem:** With raw SMO atan2, the motor drifted from 500→660+ rad/s at constant throttle over 6–13s. The working hypothesis was Rs thermal drift causing angle bias, but this was never isolated.

**What changed when switching to PLL angle (all at once):**
1. PLL angle instead of raw atan2 (smoother, fewer ±180° jumps)
2. Phase compensation for SMO LPF delay
3. Rate limiter on commutation angle
4. LP-filtered omega feedback (breaking noise amplification loop)
5. Duty-based voltage reconstruction

**Result:** 44.8–57.5s CL runs (vs 12s with raw SMO angle).

**Caution on attribution:** The PLL is a PI tracking loop with integral action — it WILL follow any persistent phase drift in theta_meas until phase error returns to zero. It does NOT inherently reject DC angle drift. The improvement most likely came from the entire bundle of changes (especially items 2–5), not from PLL drift rejection specifically. This has not been isolated experimentally.

---

## 4. Startup Sequence

### V/f Mode (used for A2212 — low Rs)

**Why V/f instead of I/f for A2212:**
Rs = 65mΩ. PI at 4A produces Vq = Rs × Iq = 0.26V — insufficient for rotor lock-in. V/f at 1.0V provides ~15A starting torque (1.0V / 65mΩ). The current is uncontrolled but the alignment duration (300ms) is brief.

#### Phase 1: ALIGN (300ms)
- Hold θ = 0, ramp Vq from 0 → VF_BOOST (0.5V)
- Voltage ramp prevents inrush: 1.0V/65mΩ = 15.4A step → ramp over 300ms
- SMO reset at ALIGN end (clean seed for OL_RAMP)

#### Phase 2: OL_RAMP (~1s)
- Advance forced angle at accelerating rate: `ω_ol += 200 rad/s² × dt`
- V/f voltage: `Vq = VF_BOOST + Ke × ω_ol`
- Speed cap at STARTUP_HANDOFF_RAD_S (500 rad/s)
- SMO runs free — receives real I and V but its angle is NOT used for commutation
- PLL tracks SMO independently

#### Phase 3: Handoff Dwell (125ms at handoff speed)
Four simultaneous checks:
1. **PLL tracking:** `ω_pll` within ±50% of `ω_ol`
2. **Confidence:** SMO confidence > 0.15
3. All sustained for 3000 fast-loop ticks (125ms)

(Angle coherence check was removed — on low-Ls motors like A2212, SMO BEMF is ~0.07V at handoff causing atan2 wraps that reset the dwell counter.)

#### Phase 4: CL Entry
State seeding at handoff:
- PLL seeded with `ω_ol` (prevents speed PI mismatch)
- `st->theta` = current SMO angle (rate limiter tracks from valid point)
- `st->omega` = `ω_ol` (LP-filtered speed starts correct)
- Vq PI preloaded with `VF_BOOST + Ke × ω_ol + Rs × 0.3`
- Speed PI preloaded with 0.3A Iq

---

## 5. Closed-Loop Commutation

### Phase Compensation
The 2-stage LPF introduces group delay:
```
τ_stage = (1 - α) / (α × fs)
τ_total = 2 × τ_stage + dt_fast
phase_comp = ω_filt × τ_total
```
At 500 rad/s: α ≈ 0.05, τ_stage = 0.95/(0.05×24000) = 0.79ms, τ_total = 1.62ms, comp = 0.81 rad (46°).

### Angle Rate Limiter
```
dtheta = θ_target - θ_current    (wrapped to ±π)
dtheta = clamp(dtheta, -0.5, +0.5)    // ~28.6° max per tick
θ_current += dtheta
```
Prevents ±180° jumps when PLL has transients. Rarely activates during normal operation with PLL angle (much smoother than raw SMO).

### Omega LP Filter
```
α = 0.010 + spd_frac × 0.006    (spd_frac = ω_filt/1000, clamped [0,1])
st->omega += α × (ω_pll_raw - st->omega)
```
- **Input:** raw PLL omega (ω_pll_raw)
- **Output:** st->omega (used by speed PI, phase comp, SMO hint)
- Breaks noise feedback loop: PLL error → wrong LPF tuning → more PLL error

### Voltage Reconstruction
Applied voltage for SMO is reconstructed from *actual PWM duties* (post-SVPWM), not commanded Vd/Vq:
```
Vα = (2·da - db - dc) / 3 × Vbus
Vβ = (db - dc) / √3 × Vbus
```
This accounts for SVM zero-sequence injection and duty clipping.

**Dead-time compensation** (added 2026-03-11): During dead-time, the output voltage is determined by current direction, not the gate signal. The error is compensated in αβ frame:
```
Vα -= dt_comp × soft_sign(Iα, 2.0)
Vβ -= dt_comp × soft_sign(Iβ, 2.0)
```
Where `dt_comp = Vbus × 2×Td/Tpwm`. On A2212 at 12V: dt_comp ≈ 0.43V — this is **larger than the BEMF signal at handoff (0.28V)**. Without this correction, the SMO sees a ~0.4V systematic voltage error that corrupts the current model and BEMF extraction.

*Note: dt_comp was computed every tick but was NOT applied to the voltage reconstruction until this fix. The V2 MXLEMMING observer already had this correction (foc_v2_observer.c:61).*

---

## 6. Speed Control Loop

### Architecture (1 kHz slow loop)
```
Throttle → speed_target = cl_idle + pot_frac × (ω_max_v - cl_idle)
speed_ref = speed_target    (direct — no ramp)
speed_err = speed_ref - st->omega
Iq_ref = PI(speed_err)
```

### Speed PI Parameters (A2212)
| Parameter | Value | Notes |
|-----------|-------|-------|
| KP_SPD | 0.0005 | At 500 rad/s error: Kp → 0.25A |
| KI_SPD | 0.02 | Integrates 10 A/s at 500 error |
| Iq clamp | [-1.0, +2.0] A | -1A brake, +2A max drive |
| Anti-windup | 0.97 decay | When err<0 and integral>0, ~23ms half-life |

### Voltage-Limited Speed Cap
```
ω_max_v = 0.70 × Vbus / (√3 × Ke)
```
At 12V: 0.70 × 12 / (1.732 × 0.000563) = 8613 rad/s ≈ 11,700 RPM mech.
Previous cap of 0.80 allowed 9000 rad/s → modIndex hit 0.95 → voltage saturation → desync.

### Why No Speed Ramp
Speed ramp (200 rad/s²) was removed because it ran ahead of the motor's actual acceleration capability during pot increases → persistent positive speed_err → spdI wound up to 5.0 → overshoot → desync. The PI's own dynamics (KP=0.0005, KI=0.02) naturally limit acceleration rate.

### Why No Gain Scheduling
Attempted gain scheduling (0.2×–1.0× proportional to speed) to reduce low-speed oscillation:
- Made medium-speed worse (4.5% stdev vs 1.3% without scheduling)
- Didn't help low-speed oscillation (root cause is PLL noise, not PI gains)
- At 0.5× gains: PI couldn't hold idle → motor decelerated and exited CL in 0.7s
- **Conclusion:** fixed gains are the best compromise

---

## 7. Protection & Fault Detection

### Software Overcurrent (fast loop, 24kHz)
```
|I|² = Id² + Iq² > fault_oc_a² for 48 ticks (2ms) → FAULT
```

### Board Fault Pin (fast loop)
RB11 = U25A(OV) + U25B(OC) active-low. 48-tick debounce → FAULT (BOARD_PCI). Latching.

### Desync Detection (slow loop)
```
|Id| > 2.0A for 200ms → FAULT (DESYNC)
```
Only active after CL holdoff and above 1.5× handoff speed.

### Phantom Commutation Detection (slow loop)
```
mod = |V| / (Vbus/√3)
mod_expected = Ke × ω / (Vbus/√3)
if (mod < 0.30 × mod_expected) for 50ms → clean stop (ARMED, auto-restart)
```
Catches: PLL locked onto switching noise with motor stalled. Symptom: PLL reports high speed but modIndex is only 5–12% of expected (BEMF collapsed).

### Low-Speed CL Exit (slow loop)
```
if ω < handoff_speed × 0.5 → clean stop (ARMED)
```
SMO has poor SNR below handoff speed; exit before desync.

---

## 8. Bugs & Lessons Learned

### Critical Bugs Found and Fixed

#### 8.1 Self-Referencing LP Filter (Session 2, Test 1)
**Symptom:** Motor entered CL but "phantom commutated" at 0 RPM.
**Root cause:** Changed omega feedback for phase comp to use `st->omega` (LP-filtered output), but the LP filter's INPUT was also `st->omega`:
```c
st->omega += alpha * (st->omega - st->omega);  // = no update!
```
**Fix:** Separated into `omega_pll_raw` (raw PLL, LP filter input) and `omega_filt` (LP-filtered output for phase comp/SMO hint).

#### 8.2 SMO Chicken-and-Egg Problem
**Symptom:** SMO converges poorly during startup.
**Root cause:** PLL omega was fed to SMO for adaptive LPF, but PLL is garbage because SMO hasn't converged yet.
**Fix:** During OL modes (ALIGN, OL_RAMP), pass forced `omega_ol` to SMO. During CL, use LP-filtered `st->omega`.

#### 8.3 Raw SMO Angle Drift → Runaway (6–13s)
**Symptom:** Speed drifts from 500 → 660+ rad/s at constant throttle over 6–13s.
**Root cause:** Raw SMO atan2 has slow DC drift from Rs thermal change: `e_bias = −ΔRs × I × cos(θ)`. This shifts the angle ahead → pulls motor faster → positive feedback.
**Fix:** Use PLL angle for commutation. PLL tracks frequency (AC) and filters DC drift. Extended CL from 12s to 57.5s.

#### 8.4 Speed PI Integrator Windup → Desync
**Symptom:** Motor accelerates uncontrollably after pot increase, desyncs.
**Root causes (multiple):
- Speed ramp ran ahead of actual acceleration → persistent positive error → spdI hit 5.0
- iq_clamp was 5.0A (50% of motor max) → way too high for actual need (~0.3–1.5A)
- Anti-windup decay (0.995) was too slow to unwind
**Fixes:**
- Removed speed ramp (direct speed_ref = speed_target)
- Reduced iq_clamp from 5.0 to 2.0A
- Increased anti-windup decay from 0.995 to 0.97

#### 8.5 Phantom Commutation Not Detected
**Symptom:** After desync, PLL locked onto SMO switching noise at 3648–5384 rad/s. Motor stalled but telemetry showed high speed. Confidence=1.0 (misleading).
**Root cause:** Fixed modIndex threshold (0.02) too low — phantom modIndex was 0.05–0.12.
**Fix:** Ratio-based check: `mod < 30% of expected (Ke×ω/v_max)`. At 5384 phantom: expected=0.43, actual=0.08 → 18% → caught.

#### 8.6 V/f Alignment Inrush
**Symptom:** Large current spike at ALIGN start.
**Root cause:** 1.0V / 65mΩ = 15.4A instantaneous at θ=0.
**Fix:** Voltage ramp 0 → VF_BOOST over alignment period.

#### 8.7 SMO LPF Alpha Way Too High (Initial Tuning)
**Symptom:** SMO angle output was pure noise, handoff failed repeatedly.
**Root cause:** K=Vbus=12V creates ±12V switching noise. With α=0.40, single-stage residual = 12V × 0.40 = 4.8V, drowning 0.28V BEMF.
**Fix:** α=0.10 base with 2-stage cascaded LPF. Residual = 12V × (0.05)² = 0.03V.

### Approaches That Failed

| Approach | Why It Failed |
|----------|--------------|
| **Speed error deadband (2–5%)** | Low-speed oscillation amplitude (±130 rad/s = ±26%) far exceeds any reasonable deadband |
| **Gain scheduling (0.2–0.7×)** | Made medium-speed worse; at low gains, PI couldn't hold idle |
| **Slower omega LP filter (α=0.005)** | Combined with reduced PI gains, motor couldn't maintain idle speed |
| **KI_SPD = 0.01** | Too slow to track pot sweeps → desync at 3500 rad/s |
| **Adaptive K based on BEMF** | Reducing K to 2×BEMF broke convergence: G×K=0.78A << 15A real current |
| **Rs online adaptation** | At borderline speed, intermittent adaptation → model drift → angle error |
| **CL idle at 800 rad/s** | Speed PI couldn't handle 500→800 step change at CL entry |

---

## 9. Open Issues & Questions

### 9.1 Low-Speed Oscillation (±100 rad/s at 500 rad/s idle)
**What:** ±100–130 rad/s oscillation at ~0.86 Hz period when motor is at idle speed (500 rad/s). Clearly audible as a pulsating/wobbling sound.

**Data (steady-state at 500 rad/s):**
```
time     omega   Iq      spdI
1.93    494     0.31    0.300
2.22    351     0.31    0.247
2.51    311     0.08    0.099
2.80    501    -0.01    0.041
3.09    510     0.36    0.307
3.38    365     0.02    0.057
3.67    309     0.17    0.119
3.96    472     0.02    0.043
```

**Analysis:** This is a limit cycle, not random noise. The speed PI responds to PLL noise, applies corrective Iq, motor accelerates past target, PI brakes, motor decelerates past target. Period ≈ 1.2s matches the mechanical time constant of A2212 + 8×4.5 prop at low speed.

**Why it's hard to fix:**
- Reducing PI gains (KP, KI) → CL can't hold idle or can't track pot changes
- Deadband → oscillation amplitude exceeds any usable deadband
- Heavier LP filter → PI response too slow → motor decelerates below low-speed exit threshold
- Gain scheduling → worse medium-speed performance

**Possible root causes to investigate:**
1. **PLL noise at low speed:** BEMF at 500 rad/s = 0.28V (2.3% of Vbus). SMO switching noise = ±12V. Even with 2-stage LPF, the SNR is marginal. PLL locks onto noise sidebands.
2. **Phase compensation error:** At low speed, the adaptive LPF phase compensation may be inaccurate — if the compensated angle leads/lags by even 10°, torque efficiency drops and the PI has to work harder.
3. **Current PI interaction:** The current PI bandwidth (~1kHz) is much faster than the speed loop (~1Hz at these gains). But if the current PI has any steady-state error at low speed (e.g., from dead-time effects or low voltage resolution), it could feed into the speed oscillation.
4. **ADC noise at low duty:** At 500 rad/s, duty cycles are near 50% (very low voltage). Current measurement SNR may be poor at these duty cycles.

**What other ESCs do:** Most sensorless FOC ESCs (VESC, AM32) don't operate at 500 rad/s on a 1400KV motor — they use higher handoff speeds (1000+ rad/s) where BEMF gives better SNR. At low speed, they either use I/f (forced current, no speed feedback) or HFI (high-frequency injection for position estimation independent of BEMF).

### 9.2 Mid-Speed Desync (~3500 rad/s)
Motor occasionally desyncs at ~3500 rad/s during pot sweep (26–37s CL runs). modIndex collapses from 0.30 → 0.05 suddenly. Not voltage saturation (mod was only 0.30).

**Possible causes:**
- Rapid speed reference change during pot sweep → transient angle error
- Cross-coupling effects at medium speed (ωLsIq becomes significant)
- PLL bandwidth transition (adaptive BW changes rapidly in 1000–3000 range)

### 9.3 High-Speed Desync (>8000 rad/s)
Voltage saturation: modIndex approaches 0.95. Inverter can't produce commanded voltage → current PI loses control → rotor slips.

**Mitigation:** Reduced cap from 0.80 → 0.70. Proper fix: field weakening (inject negative Id to reduce flux → extend speed range). Not implemented.

### 9.4 Sound Quality vs Hurst Motor
The A2212 sounds "rough" compared to the Hurst motor even when running stable. Possible reasons:

| Factor | Hurst (DMB2424B10002) | A2212 1400KV |
|--------|----------------------|--------------|
| Rs | 534 mΩ | 65 mΩ |
| Ls | 471 µH | 30 µH |
| Ke (λ_pm) | 0.00742 V·s/rad | 0.000563 V·s/rad |
| BEMF at 500 rad/s | 3.71V (31% Vbus) | 0.28V (2.3% Vbus) |
| L/R time constant | 0.88 ms (21 ISR ticks) | 0.46 ms (11 ISR ticks) |
| Pole pairs | 5 | 7 |
| Handoff speed | 262 rad/s | 500 rad/s |
| SMO F coefficient | 0.953 | 0.910 |

**Why Hurst sounds better:**
1. **13× more BEMF at handoff.** SMO angle estimate is vastly more accurate → cleaner commutation → less torque ripple.
2. **Higher Ls.** Acts as natural current filter. On A2212, current responds almost instantly to voltage errors (0.46ms vs 0.88ms time constant), amplifying noise.
3. **Higher Rs.** Small voltage errors produce smaller current errors (V/R). On A2212, 0.1V error → 1.5A current error.
4. **Slower electrical frequency.** 5pp vs 7pp means lower electrical frequency at the same mechanical speed → PLL has an easier job.

**In short:** The A2212 is a challenging motor for sensorless SMO FOC — low Rs, low Ls, low Ke, high KV. Every source of noise (ADC, switching, observer) is amplified more than on the Hurst. However, the current observer path still has known inconsistencies (see §9.5) that may account for some of the performance gap before attributing it purely to SNR limits.

### 9.5 Known Observer Path Inconsistencies (Fixed / Remaining)

These issues mean the current implementation is **not yet a fully correct classical SMO**. Performance conclusions should be treated as preliminary until these are resolved.

#### Fixed (2026-03-11, post-review)

1. **Compile-time omega_ref in SMO and phase comp** — Both the SMO adaptive LPF (`foc_v3_smo.c`) and the CL phase compensation path (`foc_v3_control.c`) used the compile-time constant `STARTUP_HANDOFF_RAD_S` instead of the runtime `st->handoff_rad_s`. If the runtime value differed (e.g., via GSP parameter update), the observer LPF scaling and the commutation phase compensation would be wrong.
   - **Fix:** Added `omega_ref` field to `SMO_Observer_t`, initialized from `params->handoff_rad_s` at `v3_smo_init()`. Both SMO and phase comp now use `smo->omega_ref`.

2. **Missing dead-time compensation in voltage reconstruction** — `dt_comp` was computed every tick but never applied to the voltage fed to the SMO. On A2212: dt_comp ≈ 0.43V, which is **larger than the BEMF at handoff (0.28V)**. The SMO's current model saw ~0.4V systematic voltage error.
   - **Fix:** Added `v_alpha -= dt_comp × soft_sign(Iα, 2.0)` (and β axis) to voltage reconstruction, matching the V2 observer implementation.

#### Remaining (Not Yet Fixed)

3. **Causal attribution of PLL angle improvement is unproven** — The 12s→57.5s improvement is attributed to "PLL rejects DC drift" in code comments, but the PLL is a PI tracking loop with integral action — it WILL follow any persistent phase drift. The improvement most likely came from the bundle of changes (phase comp, rate limiter, LP-filtered omega, duty recon), not PLL drift rejection per se. Needs isolation testing.

4. **SMO LPF alpha and phi are still compile-time** — `SMO_LPF_ALPHA` and `SMO_SIGMOID_PHI` come from `garuda_foc_params.h`. These should be in the runtime motor params struct for GSP-updatable profiles.

5. **Phase compensation assumes steady-state LPF behavior** — The `τ_stage = (1-α)/(α×fs)` formula is the steady-state group delay of a first-order IIR. During transients (speed changes, startup), the actual delay differs. No transient compensation is applied.

6. **Rate limiter is part of the commutation path** — The ±0.5 rad/tick clamp acts as a normal part of commutation (dead-reckoning when PLL angle jumps), not just a protection clamp. A fully correct observer path should not need dead-reckoning.

7. **No observer current residual metric for handoff/fault** — Handoff uses PLL speed coherence and BEMF confidence but does not check `|Î - I|` (the current estimation error). A well-converged observer should have small residual. This would be a more direct measure of observer health than BEMF magnitude ratio.

### 9.6 Recommended Implementation Path

Based on review, the recommended order to reach a correct classical SMO:

1. **✅ Duty-based voltage reconstruction with dead-time correction** (done)
2. **✅ Runtime-consistent SMO parameters** (done — omega_ref)
3. **Dynamic LPF phase compensation** — account for transient behavior, not just steady-state τ
4. **Observer-angle commutation** — theta_drive from θ_pll + phase_comp as primary path; rate limiter only as protection, not normal mechanism
5. **Confidence-based handoff with current residual** — add |Î - I| to handoff gate
6. **Rs adaptation** — re-enable once observer path is fully correct
7. **Only then:** consider super-twisting SMO or more advanced observers

---

## 10. A2212 vs Hurst Comparison

### Why Hurst "Just Works"
The Hurst DMB2424B10002 is Microchip's reference motor, designed for their FOC dev boards. Its parameters are almost ideally suited for sensorless FOC:

- **BEMF at handoff (262 rad/s) = 1.94V** — 16% of 12V Vbus. Clean SMO signal.
- **Rs = 534 mΩ** — current errors from voltage noise: 0.1V / 0.534 = 0.19A (vs 1.5A on A2212)
- **Ls = 471 µH** — K_base = 1.5 × 471e-6 / 41.67e-6 = 16.9V → capped to 6V (25% Vbus). This means K is only 50% of Vbus, so switching noise is ±6V instead of ±12V.
- **PLL at handoff sees 1.94V BEMF** vs 6V switching noise → SNR = 0.32. On A2212: 0.28V BEMF vs 12V noise → SNR = 0.023. **14× worse SNR.**

### What Would Make A2212 Sound Clean?

1. **Higher handoff speed (1000+ rad/s):** BEMF = 0.56V, doubles SNR. But V/f to 1000 proved unreliable — rotor loses sync at high forced speed.

2. **High-Frequency Injection (HFI):** Inject ~1kHz signal on d-axis, measure response. Position comes from saliency, not BEMF. Works at zero speed. Requires motor with sufficient saliency (Ld ≠ Lq). A2212 as SPM may have low saliency.

3. **Extended Kalman Filter (EKF) observer:** Better noise rejection than SMO because it uses a probabilistic model. More computationally expensive. VESC uses an EKF variant.

4. **Flux observer (MXLEMMING-style):** Integrates voltage for flux, corrects with current. Less sensitive to switching noise than SMO. We had this in V2 but it diverged at high speed due to integration drift. Could revisit with better drift correction (e.g., hybrid flux/SMO).

5. **Accept the noise and mask it:** Use higher handoff speed with I/f bridge. Run I/f up to 1000–1500 rad/s (where SMO has good SNR), then switch to CL. The motor sounds clean in I/f (fixed current, no speed feedback). Only the handoff transient needs to be smooth.

---

## Appendix: Current Tuning State (2026-03-11)

```c
// garuda_foc_params.h — A2212 profile
KP_DQ  = 0.19      KI_DQ  = 408.0     // Current PI
KP_SPD = 0.0005    KI_SPD = 0.02      // Speed PI
Speed PI clamp: [-1.0, +2.0] A
Anti-windup decay: 0.97

// foc_v3_control.c
Omega LP filter: α = 0.010 + spd_frac × 0.006
Voltage cap: 0.70 × Vbus / (√3 × Ke)
Phase comp: ω_filt × (2 × τ_stage + dt), using smo.omega_ref (runtime)
Dead-time comp: v_αβ -= dt_comp × soft_sign(I_αβ, 2.0)
Rate limiter: ±0.5 rad/tick
CL holdoff: 20ms
Phantom detection: mod < 30% expected, 50ms debounce
Desync detection: |Id| > 2.0A, 200ms debounce
Low-speed exit: ω < handoff/2
```

## Appendix: Test Artifacts

| Run | CSV | Duration | Max Speed | Exit | Notes |
|-----|-----|----------|-----------|------|-------|
| Best CL | `logs/smo_sweep_20260311_163921.csv` | 57.5s | 9000 rad/s | BOARD_PCI (voltage sat) | Pre-dt-comp, pre-omega_ref fix |
| Speed osc analysis | `logs/smo_sweep_20260311_164403.csv` | 40.5s | 3660 rad/s | CL→ARMED (phantom) | Deadband experiment |
| Gain schedule fail | `logs/smo_sweep_20260311_165045.csv` | 7.3s | — | CL→ALIGN (repeated) | PI too weak |
| Last pre-review | `logs/smo_sweep_20260311_165754.csv` | 26.4s | 3428 rad/s | CL→ARMED (phantom) | Restored KI=0.02 |
