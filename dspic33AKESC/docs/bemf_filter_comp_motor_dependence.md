# How Motor Inductance (Ls) and Resistance (Rs) Affect Filter Compensation

**Date**: 2026-05-28
**Status**: Research note — companion to `bemf_filter_compensation.md`.
**Scope**: Analytical and quantitative. No code changes.

This doc answers the question: when we change the motor (different KV,
different Rs, different Ls), what about the filter-comp math actually
depends on the motor — and what doesn't?

The short answer: **the filter time constant τ doesn't depend on the
motor at any meaningful level. The BEMF amplitude at the ADC pin
does, and that's what `HWZC_FILTER_AMP_PCT` is empirically chasing.**

The long answer is the rest of this doc.

---

## 1. The Signal Chain (Reminder)

```
            ┌──────────────────────────┐         ┌─────────────┐
  rotor →  │   stator phase coil      │  V_term │ PCB filter  │  V_ADC
  flux     │   Rs (resistance)        │───────►─│  R=3 kΩ     │────►──►  comparator
           │   Ls (inductance)        │         │  C=10 nF    │
           │   open-circuit BEMF: E   │         │  to AGND    │
           └──────────────────────────┘         └─────────────┘
              │                                                │
              │ ◄── motor side ──►   ◄── PCB side ──►          │
              └──────────────── total signal path ─────────────┘
```

There are **two** sources of phase shift / amplitude attenuation
between the rotor's true BEMF and the comparator input:

1. **Motor side**: BEMF appears across the coil, internal voltage
   divider between (E source — Rs — Ls) and the rest of the circuit.
2. **PCB side**: external R+C low-pass filter.

The filter compensation in `motor/hwzc.h` corrects for **(2)** only.
This doc unpacks **(1)** to clarify what it does and doesn't do.

---

## 2. The Filter τ Is Set Almost Entirely by the PCB

### 2.1 General form

The transfer function from motor terminal voltage to ADC pin is a
classic RC low-pass with a finite source impedance:

```
H(jω) = 1 / (1 + jω·(Z_source + R_filter)·C_filter)
```

The motor's contribution to `Z_source` is `Rs + jωLs` (assuming the
floating phase sees the motor coil as the source impedance, which is
approximately true once the freewheel transient settles).

The **effective time constant** is:

```
τ_eff = (Rs + R_motor_inductive_part + R_filter) · C_filter

where R_motor_inductive_part is the magnitude |jωLs| = ωLs
```

### 2.2 Numerical comparison

For our system: `R_filter = 3 kΩ`, `C_filter = 10 nF`. The motor's
contribution at the relevant operating frequencies:

**Motor parameters from `gsp_params.c`**:

| Motor | Rs | Ls |
|---|---|---|
| Hurst (large) | 534 mΩ | 471 µH |
| A2212 (medium) | 65 mΩ | 30 µH |
| 2810 (small, high-KV) | 22 mΩ | 25 µH |

**Reactance |ωLs| at the electrical fundamental** (the frequency the
filter compensation cares about):

| eRPM | f_elec | A2212 \|ωLs\| | 2810 \|ωLs\| | Hurst \|ωLs\| |
|---|---|---|---|---|
| 14 k | 233 Hz | 44 mΩ | 37 mΩ | 690 mΩ |
| 60 k | 1000 Hz | 188 mΩ | 157 mΩ | 2.96 Ω |
| 120 k | 2000 Hz | 377 mΩ | 314 mΩ | 5.92 Ω |
| 200 k | 3333 Hz | 628 mΩ | 524 mΩ | 9.86 Ω |
| 240 k | 4000 Hz | 754 mΩ | 628 mΩ | 11.8 Ω |

Then `Z_source = Rs + |ωLs|`:

| Motor | Z_source at 200k eRPM | Z_source / R_filter |
|---|---|---|
| A2212 | 65 mΩ + 628 mΩ = 0.69 Ω | **0.023 %** |
| 2810 | 22 mΩ + 524 mΩ = 0.55 Ω | **0.018 %** |
| Hurst | 534 mΩ + 9.86 Ω = 10.4 Ω | **0.35 %** |

The motor's source impedance is **three to four orders of magnitude
smaller** than the filter's 3 kΩ. Even for Hurst — the highest-Rs/Ls
motor we run — the contribution to `τ_eff` is below 0.5 % at the top
of the speed range.

**Conclusion**: `τ_eff ≈ R_filter · C_filter` for any motor we run.
The `K_Q15` constant doesn't need motor-specific tuning.

### 2.3 When motor impedance WOULD start to matter

The motor's contribution to `τ_eff` becomes 10 % of the filter's at:

```
(Rs + ωLs) = 0.1 × R_filter = 300 Ω
```

For `Rs = 50 mΩ` and `Ls = 30 µH`, that requires `ωLs = 300 Ω` →
`f = 1.59 MHz`. That corresponds to 95 million eRPM — orders of
magnitude beyond any physical motor.

If a future board used a much smaller filter resistor (say
`R_filter = 100 Ω` instead of 3 kΩ), the threshold drops to 10 Ω →
`f ≈ 53 kHz` ≈ 3.2 M eRPM. Still beyond realistic operation, but
the safety margin starts to shrink. So:

- **3 kΩ filter (current)**: motor impedance is completely
  negligible across all motors at all speeds we care about
- **300 Ω filter (hypothetical)**: still completely negligible
- **100 Ω filter (hypothetical low-impedance bench)**: motor starts
  to matter above 1 M eRPM — still well outside our envelope

This is why the filter R is intentionally chosen large in this design.

---

## 3. The Motor DOES Set the BEMF Amplitude at the Filter Input

### 3.1 What `amp` in the filter-comp formula really is

In `HWZC_ApplyFilterComp()`:

```c
offset = (amp × ω·τ_Q15) >> 15
```

`amp` is the BEMF amplitude **as seen by the comparator** — i.e.,
the peak excursion of the BEMF signal at the ADC pin, in 12-bit ADC
counts. The code reads it from `bemf.zcAmpForFilterComp`, a slow IIR
of measured peak amplitudes.

This is **not the open-circuit BEMF**. It's the loaded amplitude
after going through:

- Motor terminal effects (Rs voltage drop under current load)
- The voltage divider implicit in the star-connected motor
- PCB filter attenuation at the electrical fundamental
- Any common-mode rejection in the differential / single-ended
  comparator front-end

The slow IIR (`zcAmpForFilterComp`) is the right abstraction because
it tracks whatever is actually happening at the ADC pin without
needing the firmware to model all four effects analytically.

### 3.2 But motor Rs and Ls predict the AMPLITUDE DERATE

For a given operating condition, the open-circuit BEMF and the
terminal-visible BEMF differ. Two effects:

#### 3.2.1 Resistive derate

When current flows in the driven phases, there's `I·Rs` drop in each
energized winding. The floating phase's terminal voltage references
the virtual neutral, which is shifted by these drops.

For a sinusoidal current `I(t) = I_peak · sin(ω·t − π/6)` (the BLDC
current waveform leads the BEMF zero-crossing by 30° for ideal
6-step), the derate near ZC is approximately:

```
ΔV_resistive ≈ I_peak · Rs · sin(30°) = I_peak · Rs / 2
```

For our 2810 at moderate load (5 A peak): `ΔV ≈ 5 · 0.022 / 2 = 55 mV`.
On a ~12 V terminal BEMF, that's a 0.5 % derate. Negligible.

For Hurst at moderate load (3 A peak): `ΔV ≈ 3 · 0.534 / 2 = 800 mV`.
On a ~6 V terminal BEMF, that's a **13 % derate**. Significant.

→ **High-Rs motors have noticeable amplitude derate under load.**

#### 3.2.2 Inductive (di/dt) reactance

Phase current can't change instantaneously through Ls. During the
freewheel transient after commutation, the phase voltage spikes to
the rail clamp. The body diode dumps inductive energy to the bus.
This appears as a brief spike on the floating phase terminal, which
the filter smooths.

The time constant of this freewheel decay (current through body
diode at clamped voltage):

```
τ_freewheel ≈ Ls / (Rs + R_diode)  ≈ Ls / Rs   (Rs typically dominates)
```

For our motors:

| Motor | Ls / Rs | Sector time @ 100 k eRPM | Sector time @ 200 k eRPM |
|---|---|---|---|
| Hurst | 471 / 0.534 = 882 µs | 100 µs | 50 µs |
| A2212 | 30 / 0.065 = 462 µs | 100 µs | 50 µs |
| 2810 | 25 / 0.022 = 1140 µs (or 25/0.043=580 µs for phase-phase) | 100 µs | 50 µs |

**The motor's electrical time constant `Ls/Rs` is 5–20× the sector
time at high RPM.** That means the freewheel transient has NOT fully
decayed before the BEMF detection window opens. The HW blanking and
the PWM-state gate handle the worst of this, but residual transient
contributes to the signal at the ADC pin.

Effect on filter comp: this transient adds a **slope** to the BEMF
signal during the early part of each sector. The filter passes most
of that slope through. The ZC point shifts slightly depending on
where in the freewheel decay the detection window opens. The slow
IIR `zcAmpForFilterComp` averages over many sectors so the comp
sees a stable amplitude estimate.

### 3.3 PWM ripple amplitude

This is the biggest motor-dependent effect on the BEMF signal seen
at the ADC pin.

Phase current ripple from PWM (per
`pwm_frequency_effects.md`, Section 2):

```
Δi_pp ≈ V_bus · D · (1−D) · T_PWM / Ls
```

For 45 kHz PWM at 50 % duty, 24 V bus:

| Motor | Ls | Δi_pp |
|---|---|---|
| Hurst | 471 µH | 0.28 A |
| A2212 | 30 µH | 4.44 A |
| 2810 | 25 µH | 5.33 A |
| Hypothetical small motor | 10 µH | 13.3 A |

This ripple is the **switching noise that has to be attenuated by the
PCB filter**. Smaller Ls → larger ripple → more noise → comparator
sees more switching artifacts.

The filter does its job (−18 dB at 45 kHz vs the 5.5 kHz cutoff), but
the residual passing through scales with the source amplitude. **High
Ls motors are easier for the filter; low Ls motors are harder.**

This is also why the PCB filter R/C is chosen relatively
conservatively — has to handle the worst-case (small Ls) motor
without letting too much PWM noise leak through.

---

## 4. Motor Dependence Summary

| Quantity | Depends on motor? | Why |
|---|---|---|
| Filter time constant `τ` (in K_Q15) | **No** | PCB R·C dominates; motor impedance < 0.5% of R_filter |
| Linear approximation accuracy | **No** | Pure math; depends only on ω·τ |
| Amplitude `amp` fed to formula | **Yes** | Slow IIR tracks real terminal BEMF |
| `AMP_PCT` empirical knob | **Yes** | Calibrates the formula against load-dependent derate |
| Safety caps (MAX_OMEGA, MAX_OFFSET) | **No** | Numerical guards; envelope-independent |

### 4.1 Predicting AMP_PCT for a new motor

`AMP_PCT` compensates for the linear-approximation overshoot and the
load-dependent amplitude derate. Per Section 3.2.1, the derate scales
roughly with `I_load · Rs`. As a rough first-cut prediction:

```
AMP_PCT_predicted ≈ 100 % − (linear_overshoot_at_top_speed)
                              − (typical_load_derate_at_running_speed)

linear_overshoot ≈ 10–15 % at ω·τ around 0.5
load_derate     ≈ I_peak · Rs / V_bemf_peak × 100 %
```

For 2810 at typical bench load:
- Linear overshoot: ~12 %
- Load derate: ~5 A · 0.022 Ω / 12 V · 100 % ≈ 0.9 %
- Predicted: 100 − 12 − 1 = **~87 %**

But we landed at **50 %** empirically. Why the discrepancy?

The linear approximation isn't the only source of overshoot. The
flat-trapezoidal BEMF (vs the sinusoidal model used in the math)
has its own shape mismatch. The PCB filter isn't strictly
first-order (stray capacitance, MCU input impedance). And the
phase-phase voltage measured at the ADC is geometrically related to
the per-phase BEMF in a way that introduces a √3 / 2 type factor
that's not in the simple model.

The honest summary: **AMP_PCT is an integrated fudge factor that
absorbs everything the analytical model can't predict precisely.**
Per-motor predictions get you in the right ballpark; bench tuning
finds the exact value.

### 4.2 Quick first-guesses for different motors

| Motor type | Rs | Ls | Suggested AMP_PCT start | Reasoning |
|---|---|---|---|---|
| Small high-KV (2810-class) | < 50 mΩ | < 30 µH | 50–60 % | Low derate, big PWM ripple, large empirical fudge |
| Medium (A2212-class) | 50–100 mΩ | 30–60 µH | 60–75 % | Moderate everything |
| Larger (Hurst-class) | > 200 mΩ | > 200 µH | 70–85 % | Less PWM ripple, more derate, formula closer to truth |

These are starting points. Bench-tune for minimum `Ibus` from there.

---

## 5. Operating-Regime Effects

### 5.1 Light vs heavy load

Heavy load means more phase current → more I·Rs derate → smaller
amplitude. The slow IIR on `zcAmpForFilterComp` tracks this
automatically, so the `offset` magnitude in the formula adjusts down
with load. Good.

What it doesn't track: the **phase shift** introduced by motor Ls at
high current. When `di/dt` is large, the back-EMF "effective" angle
shifts. This is normally compensated by `TIMING_ADVANCE_MAX_DEG`,
not by filter comp. So:

- **Filter comp** = correct for the PCB filter's phase lag
- **Timing advance** = correct for everything else (motor Ls
  reactance, torque-angle optimization, switching dead-time)

Both stack in `garuda_service.c`. Keeping them conceptually
separate matters for debugging.

### 5.2 Saturation

At very heavy load, motor flux saturates → effective Ls drops →
PWM ripple gets bigger → noise floor at the comparator increases.
Filter comp formula is unaffected (it doesn't use Ls anywhere). But
the practical SNR degrades.

This is mostly a "don't run at the absolute current limit if you
care about smooth high-speed control" trade-off.

### 5.3 Cold vs hot motor

Rs has a positive temperature coefficient (copper: +0.39 %/°C).
A 50 °C rise increases Rs by ~20 %, which:
- Increases load derate proportionally (Section 3.2.1)
- Slightly increases freewheel decay time constant
- Doesn't change filter comp formula

The slow IIR on amplitude tracks this. If the motor heat-soaks
during a long bench run, the comp adapts within ~50 ms of the
amplitude change.

---

## 6. What Would Need to Change For a Different Motor

To answer the user's underlying question: **almost nothing changes in
the filter-comp code for a different motor.** The PCB-filter τ stays
the same. The formula stays the same. The Q15 constants stay the
same.

Two things to revisit:

### 6.1 `HWZC_FILTER_AMP_PCT` — bench-tune per motor

Per Section 4.2, the starting AMP_PCT for a typical small-to-medium
sensorless BLDC motor is in the 50–80 % range. The exact value is
load-dependent and motor-specific.

**Procedure**:
1. Pick starting value from Section 4.2 table
2. Build, flash, run motor to ~70 % of top speed at steady throttle
3. Note steady-state `Ibus` from telemetry
4. Bump AMP_PCT by 5 %, re-test
5. Repeat until `Ibus` stops dropping → previous value is optimum

### 6.2 Validate `dbgFilterOffset` is in sensible range

If a motor has very different `zcAmpForFilterComp` (large or small),
`dbgFilterOffset` should reach reasonable values (10–400 ADC counts
in typical operation; 600 only at the cap, briefly).

If `dbgFilterOffset` sits at the cap for sustained intervals,
something is off (probably `AMP_PCT` is too high OR the slow IIR
hasn't settled).

If `dbgFilterOffset` is ~0 even at high RPM, the IIR amplitude is
too low (probably the BEMF signal at the ADC is being clipped by
common-mode rejection or some other front-end issue — board problem,
not comp problem).

---

## 7. What Does NOT Change Across Motors

| Constant | Why it stays |
|---|---|
| `HWZC_FILTER_K_Q15` | Filter τ depends on PCB R/C, not motor |
| `HWZC_FILTER_MAX_OMEGA_Q15` | Linear-approx breakdown is at the math, not the motor |
| `HWZC_FILTER_MAX_OFFSET` | Absolute clamp, physics-based |
| `HWZC_ApplyFilterComp()` function | Pure math, motor-agnostic |
| Two call sites (per-commutation + ADC-ISR live) | Architectural, not motor-related |

The slow-IIR fields `stepPeriodForFilterComp` and `zcAmpForFilterComp`
adapt automatically to the new motor. No code changes.

---

## 8. Cross-references

- `dspic33AKESC/docs/bemf_filter_compensation.md` — full reference
  for the formula and Q15 derivation
- `dspic33AKESC/docs/pwm_frequency_effects.md` — phase current ripple
  vs Ls (Section 2)
- `dspic33AKESC/gsp/gsp_params.c` — motor profile Rs/Ls values
- `dspic33AKESC/docs/akesc_session_2026_05_26_191k_to_243k.md` — bench
  data showing AMP_PCT=50 % was the practical sweet spot for 2810
- `memory/akesc_filter_comp_path1_203k_2026_05_26.md` — original
  bench validation; landed at AMP_PCT=75 %, later refined to 50 %
  with no significant performance change at top eRPM, slight
  improvement in mid-range current

---

## 9. Open Questions / Future Work

1. **Does AMP_PCT need to be eRPM-dependent?** Current code uses one
   value across the whole range. Section 3.2.1 suggests the load
   derate varies with current (which varies with eRPM). A lookup
   table indexed by eRPM might shave another few percent of current
   off the top end. Bench data doesn't strongly suggest this matters,
   but worth measuring.

2. **Can we model the load derate analytically and pre-set AMP_PCT?**
   With known Rs, the derate is `I·Rs / V_bemf`. If we estimate
   I from `Vbus·duty/Rs_total` and V_bemf from the slow IIR, we
   could compute a load-aware AMP_PCT in firmware. Probably not
   worth the complexity — the IIR amplitude already captures the
   net effect.

3. **Switch to the exact `sin(arctan(ω·τ))` form** (deferred to FPU
   work — see `bemf_filter_compensation.md` Section 9.1). This
   eliminates the 12 % linear-approx overshoot, which would shift
   the empirical AMP_PCT closer to its theoretical value of 88–95 %.
   Cleaner mental model.

4. **Auto-tune AMP_PCT at boot** with a no-load sweep. Removes the
   "bench-tune per motor" step. Cost: one extra second of startup
   noise. Probably worth it eventually.

---

## 10. TL;DR

- **Motor Rs and Ls do NOT change the filter time constant τ.** The
  PCB filter dominates by 3+ orders of magnitude at all real motor
  parameters and operating speeds.
- **Motor Rs and Ls DO affect the BEMF signal amplitude** at the ADC
  pin, but this is tracked automatically by the slow-IIR
  `zcAmpForFilterComp` field. The formula reads that IIR, not the
  motor parameters directly.
- **The empirical knob `AMP_PCT` absorbs**: linear-approximation
  overshoot, load-dependent amplitude derate, real-world filter
  non-ideality, and BEMF-waveform shape mismatch. Bench-tuned per
  motor in 5 % steps.
- **Porting to a new motor**: tune `AMP_PCT`. That's it. No formula
  changes, no constants to recompute, no special handling per motor
  in the firmware path.
- **Porting to a new PCB** (different R/C): recompute `K_Q15`. Per
  the companion doc.
- **High-Rs / high-Ls motors** (Hurst-class): formula matches theory
  more closely, AMP_PCT closer to 80 %.
- **Low-Rs / low-Ls motors** (2810-class): formula needs more empirical
  scaling, AMP_PCT closer to 50 %.
