# ProjectGaruda FOC Architecture & Roadmap

## Current Architecture (2026-03-08)

### Observer: MXLEMMING Flux Integration
Sensorless position estimation via back-EMF flux integration in the stationary α-β frame:

```
flux_α += (Vα - Rs·Iα)·dt - Ls·(Iα - Iα_prev)
flux_β += (Vβ - Rs·Iβ)·dt - Ls·(Iβ - Iβ_prev)
θ_est = atan2(flux_β, flux_α) + π
```

- **L·dI cancellation**: Uses `L*(I_now - I_last)` — no differentiation, no derivative noise
- **Drift prevention**: Circular normalization clamps flux magnitude to λ_pm (not per-axis)
- **Adaptive lambda**: `lambda_est` tracked in [0.3×, 2.5×] nominal range
- **Gain scheduling**: Observer gain scaled by duty cycle (low duty = low SNR = lower gain)
- **PLL**: 100 Hz BW for speed estimation only — observer atan2 drives commutation (zero lag)

### Startup: I/f (Current-Forced) + Time-Based Handoff
1. **Calibration** (256 samples): ADC offset measurement for Ia/Ib
2. **Alignment** (300ms): Id current at θ=0, locks rotor
3. **I/f Ramp**: Force angle at ω_if, ramp speed at 500 rad/s², Iq=4A
4. **Handoff** (500ms dwell): After reaching handoff speed, wait 500ms, check PLL sees forward rotation, then switch to closed-loop
5. **Closed-Loop**: Observer atan2 drives commutation, PI controllers for Id/Iq

### Control Loop (24 kHz)
```
ADC ISR → Read Ia/Ib → Clarke → Observer → Park → PI_d/PI_q → Inv Park → SVM → PWM
                                    ↓
                                   PLL (speed estimation)
```

- **SVM**: Space Vector Modulation with duty clamp [0.04, 0.96]
- **PI controllers**: Tustin-discretized with anti-windup clamp
- **Voltage limit**: `0.80 × Vbus / (√3 × Ke)` prevents saturation
- **Board fault**: RB11 pin polling (active-low, 2ms debounce) detects U25B OC

### Telemetry
- **GSP Snapshot** (114 bytes, 50 Hz): Id, Iq, θ, ω, Vbus, Ia, Ib, Vd, Vq, θ_obs, subState
- **foc_logger.py**: CSV capture with 60+ columns for AI-assisted debugging
- **GUI Scope**: 31 channels with presets (FOC Debug, Currents, Angles, Power)

---

## Observer Research Summary

### Why MXLEMMING Is The Right Choice

| Observer | Compute | Low Speed | High Speed | Param Robustness | Drone Fit |
|---|---|---|---|---|---|
| **MXLEMMING flux** | ~20 ops | Poor (I/f) | Excellent | Rs, Ls sensitive | **Best** |
| Ortega nonlinear | ~30 ops | Poor | Excellent | + gamma tuning | Equivalent, more knobs |
| SMO (sliding mode) | ~30 + LPF | Poor + lag | Good | Most robust | LPF lag at high ω_e |
| EKF (Kalman) | ~150 ops | Best theory | OK | Q/R tuning complex | Too expensive for 24kHz |
| HFI (injection) | ~40 @ 2×PWM | **0 RPM** | N/A | Needs saliency | **Won't work** (SPM) |
| MRAS | ~30 ops | Poor | Good | Online Rs | Useful as add-on |

**Key findings:**
- **HFI is impossible** for SPM outrunner drone motors — Ld/Lq ≈ 1.0, no saliency signal
- **EKF consumes 15-30µs** of 41.6µs budget — too tight with FOC + SVM + ADC
- **SMO's low-pass filter** introduces phase lag at high electrical frequencies (7PP × 4000RPM = 2930Hz)
- **MXLEMMING is the industry consensus** — VESC, MESC, SimpleFOC all use it as default
- **Improvement path**: adaptive lambda (done), Rs temperature compensation (planned), gain scheduling (done)

### What Each Project Uses
- **VESC**: 7 MXLEMMING/Ortega variants, adaptive lambda (default), HFI for IPM only
- **MESC**: MXLEMMING (original author David Molony), HFI option
- **ODrive**: Ortega + PLL, sensorless is "experimental"
- **AM32/BLHeli**: 6-step with HW comparator ZC, no FOC
- **SimpleFOC**: MXLEMMING ported from MESC/VESC

---

## Instrumentation Roadmap

### Tier 1: Expanded Snapshot (+36 bytes)
New fields to add for observer and controller visibility:

| Field | Purpose |
|---|---|
| `obs.x1`, `obs.x2` | Flux α/β — Lissajous XY plot = instant observer health |
| `obs.lambda_est` | Adaptive flux estimate — drift = wrong λ_pm |
| `obs.gain` | Gain scheduling state |
| `pid_d.integral`, `pid_q.integral`, `pid_spd.integral` | Windup detection |
| `modulation_index` | Voltage utilization — >0.95 = can't go faster |
| `obs_confidence` | Observer tracking quality metric |

### Tier 2: Triggered High-Rate Scope
ISR-rate (24kHz) ring buffer with triggered capture — like a digital oscilloscope:
- 128 samples × 32 bytes = 4.1KB RAM
- Trigger on: fault, threshold crossing, state transition, manual
- Pre/post trigger history — see exactly what happened before a fault
- Bulk transfer via GSP after trigger

### Tier 3: Automated Test Sequences
Programmatic stimuli via GSP commands:
1. **Current step response** → measure PI bandwidth
2. **Speed step response** → measure speed loop performance
3. **Startup reliability** → N-attempt statistics
4. **Frequency chirp** → Bode plot (system identification)

### Tier 4: AI-Assisted Auto-Tuning
Python Bayesian optimization script that:
1. Sets PI gains via GSP
2. Runs automated test sequence
3. Measures performance metrics from scope data
4. Selects next candidate using Gaussian process model
5. Converges in ~15-30 experiments

---

## Feature Roadmap

### Tier 1 — Flight Reliability
1. **Startup retry with backoff** — 3 attempts with increasing align time
2. **Circular current limiting** — `√(Id² + Iq²) ≤ Imax`, priority to Iq
3. **Stall detection + auto-restart** — observer confidence < 50% → re-sync
4. **Desync recovery** — fall back to I/f instead of permanent FAULT
5. **Active braking** — negative Iq for controlled deceleration

### Tier 2 — Performance
6. **Field weakening** — negative Id extends speed 30-50% beyond BEMF limit
7. **Rs temperature compensation** — NTC or online MRAS estimation
8. **DShot integration** — standard FC protocol (Phase H code exists)
9. **Bidirectional rotation** — turtle mode, 3D flight
10. **Throttle linearization** — stick → thrust (not RPM)

### Tier 3 — Efficiency
11. **Dead-time compensation** (revisited) — per-phase, current-threshold gated
12. **Motor auto-detect v2** — spin-up for dynamic Ke + inertia estimation
13. **Thermal derating** — progressive current reduction with temperature

---

## Development Method

The key innovation is **instrumented iterative development**:

```
1. INSTRUMENT — add telemetry fields for the specific behavior
2. LOG BASELINE — capture CSV/scope of current behavior
3. IMPLEMENT — code feature with tunable GSP params
4. LOG RESULT — capture under identical conditions
5. COMPARE — AI analysis of before/after data
6. TUNE — adjust params, repeat 3-5
7. COMMIT — lock in verified improvement
```

This closes the loop between code changes and real motor behavior, making each iteration data-driven rather than guesswork-driven. The `foc_logger.py` tool captures structured data that AI tools can directly analyze, identify issues, and suggest specific parameter changes.

---

## Hardware Platform
- **MCU**: dsPIC33AK128MC106 (200MHz, 32-bit, hardware FPU, 16KB RAM)
- **Board**: MCLV-48V-300W (dev), custom ESC (production)
- **PWM**: 24kHz, 3-phase complementary with hardware dead-time
- **Current sensing**: Op-amp OA1/OA2 → ADC differential
- **Known HW limitation**: U25B overcurrent comparator has no LEB — trips on SVM switching transients at ~5500 rad/s. Detected by RB11 pin polling.

## Motor Support
| Motor | KV | PP | Rs | Ls | λ_pm | Status |
|---|---|---|---|---|---|---|
| A2212 1400KV | 1400 | 7 | 65mΩ | 30µH | 0.000563 | **Active, prop-tested** |
| Hurst DMB2424B | 149 | 5 | 534mΩ | 471µH | 0.00742 | Tested (bench) |
| 5010 360KV | 360 | 7 | 88mΩ | 68µH | 0.00218 | Profile only |
| 5055 1400KV | 1400 | 7 | 38mΩ | 13µH | 0.000563 | Profile only |
