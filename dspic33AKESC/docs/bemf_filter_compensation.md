# BEMF Filter Phase-Lag Compensation — Full Reference

**Date**: 2026-05-27
**Status**: Implemented and enabled by default
(`FEATURE_HWZC_FILTER_COMP=1`).
**Where**: `motor/hwzc.h:48-99` (formula), `garuda_config.h:605-640`
(constants).

This doc is the complete reference for **why** the firmware shifts
the zero-crossing detection threshold, **how** the math is derived,
**which knobs** are theoretical vs empirical, and **what to change**
when porting to a different PCB.

---

## 1. Problem Statement

### 1.1 What we want

In sensorless 6-step BLDC drive, commutation timing comes from
detecting when the floating phase's back-EMF crosses the neutral
(virtual midpoint) voltage. The HW digital comparator on this MCU
fires when the BEMF signal crosses a programmable threshold (`CMPLO`).

We want the comparator to fire at the **physical moment** the rotor's
true (pre-filter) BEMF crosses zero. That's the canonical commutation
phase reference.

### 1.2 What actually happens

The BEMF signal does NOT go directly from motor terminal to ADC pin.
It passes through a passive RC low-pass filter on the PCB:

```
  motor phase   R = 3 kΩ              ADC pin
  ────────────[\\\\\\]────────┬───────────►  to comparator
                              │
                          C = 10 nF
                              │
                             GND
```

This filter does two useful things:
- Attenuates PWM switching noise (5.5 kHz cutoff vs 24–60 kHz PWM)
- Smooths comparator behavior so it doesn't fire on every ripple peak

And one **unwanted** thing:
- Introduces a **frequency-dependent time delay** on the signal

That delay shifts the comparator's firing point in the wrong direction:
**the comparator sees zero AFTER the rotor has already crossed it**.

### 1.3 How much delay

For a first-order RC filter, the **phase lag** of a sinusoidal input
at angular frequency ω is:

```
φ(ω) = arctan(ω · τ)
```

where `τ = R · C` is the filter time constant.

For our board: `τ = 3000 × 10e-9 = 30 µs`.

The rotor's electrical frequency at eRPM `N`:

```
f_elec = N / 60     (Hz)
ω      = 2π · f_elec = 2π · N / 60     (rad/s)
```

Phase lag in electrical degrees at various speeds:

| eRPM | f_elec | ω | ω·τ | arctan(ω·τ) |
|---|---|---|---|---|
| 14 k | 233 Hz | 1465 rad/s | 0.044 | **2.5°** |
| 60 k | 1000 Hz | 6283 rad/s | 0.188 | **10.7°** |
| 120 k | 2000 Hz | 12566 rad/s | 0.377 | **20.6°** |
| 180 k | 3000 Hz | 18850 rad/s | 0.565 | **29.5°** |
| 240 k | 4000 Hz | 25133 rad/s | 0.754 | **37.0°** |

At 240 k eRPM the **filter alone** is eating 37° of phase. The
controller's `TIMING_ADVANCE_MAX_DEG` cap is 25° — even maxing it out
can't keep up. The motor commutates late, current rises, efficiency
drops, and eventually the bridge can't deliver enough phase voltage at
the right time → desync.

### 1.4 Why this isn't fixable in hardware

You can't:
- **Lower R or C** without losing PWM-noise rejection. The 5.5 kHz
  cutoff is already at the edge — pushing it higher lets switching
  noise through and the comparator becomes useless.
- **Use an active filter** with phase compensation. Adds op-amp,
  cost, board area, noise.
- **Sample faster and lead-compensate digitally on raw samples.** This
  works in principle (e.g. FOC observers do it) but a 6-step
  controller needs a clean comparator edge — software lead-comp on
  comparator output is too coarse.

So we have to live with the filter and **shift the comparator
threshold** so it fires at the right moment despite the lag.

---

## 2. Solution: Pre-Distort the Threshold

### 2.1 The idea

The filter delays the signal by `τ·arctan(ω·τ)/ω` seconds. In that
window, a sinusoidal BEMF moves by some amount `Δv` from the true
zero-crossing. If we **lower the comparator threshold by Δv** for a
rising ZC, the comparator fires earlier — by exactly the filter's
delay amount — landing at the true ZC instant.

```
  True BEMF      ──┐                ▲ comparator threshold (default = neutral)
                   │                │
                   │\               │
                   │ \              │
                   │  \             │ ← shift this DOWN for rising ZC
                   │   \            │
                   │    \────── new threshold (neutral − offset)
                   │     \          
                   │      \         
  ────────────────────────╳───────────── true ZC instant (BEMF = neutral)
                   │       \        
                   │        \       
  ──Filtered BEMF─┐│         \      
                  ││          \     
                  │└───────────╳─── filtered signal hits neutral here
                  │             \   (after the rotor already crossed)
```

The HW comparator fires when **filtered** BEMF crosses the (shifted)
threshold. We pick the shift to make that moment coincide with the
**true** BEMF crossing zero.

### 2.2 How much to shift

Assume the BEMF near the zero-crossing is approximately a sinusoid:

```
V_bemf(t) = A · sin(ω·t)
```

near the zero crossing at `t = 0`. The filtered signal lags:

```
V_filt(t) = A · sin(ω·t − φ)     where φ = arctan(ω·τ)
```

At `t = 0` (true ZC instant):

```
V_filt(0) = A · sin(−φ) = −A · sin(φ)
```

So at the moment the rotor's true BEMF crosses zero, the filtered
signal is sitting at `−A·sin(φ)` (for a rising ZC where ω·t goes
from negative to positive). We want the comparator to fire here, so
the threshold must be:

```
threshold_rising = neutral − A·sin(φ)
threshold_falling = neutral + A·sin(φ)
```

The shift magnitude is:

```
offset = A · sin(arctan(ω·τ))
```

Using the identity `sin(arctan(x)) = x / √(1 + x²)`:

```
offset = A · ω·τ / √(1 + (ω·τ)²)
```

### 2.3 The linear approximation we actually use

For `ω·τ < 0.5` (eRPM < 160 k on our board), the difference between
`sin(arctan(ω·τ))` and just `ω·τ` is less than 12 %:

| ω·τ | sin(arctan(ω·τ)) | Linear ω·τ | Error |
|---|---|---|---|
| 0.1 | 0.0995 | 0.1 | +0.5 % |
| 0.2 | 0.1961 | 0.2 | +2.0 % |
| 0.3 | 0.2873 | 0.3 | +4.4 % |
| 0.4 | 0.3714 | 0.4 | +7.7 % |
| 0.5 | 0.4472 | 0.5 | +11.8 % |
| 0.6 | 0.5145 | 0.6 | +16.6 % |
| 0.7 | 0.5735 | 0.7 | +22.1 % |
| 1.0 | 0.7071 | 1.0 | +41.4 % |

The firmware uses the **linear approximation**:

```
offset ≈ A · ω·τ
```

This over-estimates the true offset at high speed (linear is bigger
than the sine). That over-estimation is **partially compensated** by
the empirical `AMP_PCT` knob (Section 5) which scales the offset down.

Why use the linear approximation:
- One division (ω·τ = K/stepPeriod), no sqrt
- Fits cleanly in Q15 fixed-point
- The fast ISR can't afford a sqrt or trig call

> **Note**: The dsPIC33AK128MC106 does have a single-precision FPU.
> The current Q15 implementation predates the firmware fully
> exercising the FPU on this MCU; revisiting to use float (or the
> exact `x/√(1+x²)` form) is a separate piece of work for later.
> See Section 9.

---

## 3. Code Walk

### 3.1 The inline function

`motor/hwzc.h:48-99` — `HWZC_ApplyFilterComp()`. Called every time
CMPLO is computed.

```c
static inline uint16_t HWZC_ApplyFilterComp(volatile GARUDA_DATA_T *pData,
                                            uint16_t thresh,
                                            bool risingZc)
{
#if FEATURE_HWZC_FILTER_COMP

    // 1. Pick step period source (slow IIR when PI mode is active)
    uint32_t stepP = pData->hwzc.stepPeriodForFilterComp;
    if (stepP == 0) stepP = pData->hwzc.stepPeriodHR;
    if (stepP == 0) { pData->hwzc.dbgFilterOffset = 0; return thresh; }

    // 2. Compute ω·τ in Q15
    uint32_t omegaTauQ15 = HWZC_FILTER_K_Q15 / stepP;
    if (omegaTauQ15 > HWZC_FILTER_MAX_OMEGA_Q15)
        omegaTauQ15 = HWZC_FILTER_MAX_OMEGA_Q15;

    // 3. Pick amplitude (slow IIR, not instantaneous thresh)
    uint16_t amp = pData->bemf.zcAmpForFilterComp;
    if (amp == 0) amp = thresh;   // fallback before IIR seeds

    // 4. offset = amp · ω·τ · AMP_PCT/100   (Q15 stage, then scale)
    uint32_t offset = ((uint32_t)amp * omegaTauQ15) >> 15;
    offset = (offset * HWZC_FILTER_AMP_PCT) / 100UL;
    if (offset > HWZC_FILTER_MAX_OFFSET) offset = HWZC_FILTER_MAX_OFFSET;
    pData->hwzc.dbgFilterOffset = (uint16_t)offset;

    // 5. Apply with sign
    if (risingZc)
        return (thresh > offset) ? (uint16_t)(thresh - offset) : 0;
    else
        return (thresh + offset < 4095u) ? (uint16_t)(thresh + offset) : 4095u;

#else
    return thresh;   // feature off → pass-through
#endif
}
```

Five steps in code, each lines up with a math step above.

### 3.2 The two call sites

1. **`HWZC_OnCommutation()`** (`motor/hwzc.c:211`) — runs once per
   sector boundary. Re-computes CMPLO with the current `stepPeriodHR`
   and the current `thresh` from `Vbus · duty / 2`.

2. **ADC ISR live CMPLO refresh** — runs every PWM cycle inside
   `_AD1CH0Interrupt`. Same helper, same arguments. Keeps the offset
   tracking Vbus changes mid-sector (Vbus moves with bench supply
   sag, regen pulses, etc.).

Having both call sites use the **same inline helper** is intentional:
ensures consistent behavior. A single source of truth.

### 3.3 The `stepPeriodForFilterComp` field

`motor/hwzc.c:750-764` updates a slow IIR of `stepPeriodHR`
(~46 ms time constant). This — not the instantaneous `stepPeriodHR`
— is used as ω's source when PI mode is active.

Why: PI mode shrinks `stepPeriodHR` event-by-event. If filter comp
used the instantaneous value, every PI tick would shrink ω·τ, which
would shrink offset, which would shift CMPLO closer to neutral, which
would make the comparator fire even earlier — **positive feedback**.

The slow IIR breaks that loop. Filter comp sees a smoothed period,
so offset stays stable across PI activity.

### 3.4 The `zcAmpForFilterComp` field

Same idea on the amplitude side. `bemf.zcAmpForFilterComp` is a slow
IIR of the BEMF amplitude estimate (`bemf_zc.c`). Decoupled from
instantaneous `thresh` so that **duty steps** don't whip the offset
around.

`thresh = Vbus · duty / 2` jumps immediately when duty changes. The
real BEMF amplitude lags by rotor inertia (~tens of ms). Using the
slow IIR for `amp` keeps offset magnitude matched to the **actual**
BEMF, not the duty command.

---

## 4. The K_Q15 Constant — Step by Step

This is the only place the filter time constant `τ` enters the math.
Everything else is parametric in `stepPeriodHR`.

### 4.1 Derivation

We want `ω·τ` in Q15 fixed-point (1.0 = 32768) as a function of
`stepPeriodHR` (the period of one electrical sector in SCCP2 ticks,
10 ns each).

`stepPeriodHR` is in **sector** units, not full electrical period:

```
sector_seconds = stepPeriodHR · 10e-9
electrical_period_seconds = 6 · sector_seconds = 6 · stepPeriodHR · 10e-9
```

(6 sectors per electrical revolution in 6-step.)

Electrical angular frequency:

```
ω = 2π / electrical_period_seconds
  = 2π / (6 · stepPeriodHR · 10e-9)
  = π / (3 · stepPeriodHR · 10e-9)
  = π · 1e9 / (3 · stepPeriodHR · 10)
  = π · 1e9 / (30 · stepPeriodHR)        (in rad/s)
```

Now multiply by τ (in seconds):

```
ω·τ = (π · 1e9 / (30 · stepPeriodHR)) · τ
    = π · τ · 1e9 / (30 · stepPeriodHR)
```

If we express τ in **nanoseconds** (τ_ns = τ · 1e9):

```
ω·τ = π · τ_ns / (30 · stepPeriodHR)
```

For Q15 fixed-point (multiply by 32768):

```
ω·τ_Q15 = 32768 · π · τ_ns / (30 · stepPeriodHR)
        = K_Q15 / stepPeriodHR

where K_Q15 = 32768 · π · τ_ns / 30
            ≈ 3431.9 · τ_ns
```

### 4.2 Plug in our τ

For `τ = 30 µs = 30000 ns`:

```
K_Q15 = 32768 · π · 30000 / 30
      = 32768 · π · 1000
      ≈ 102,943,706
```

Which matches `garuda_config.h:640`:

```c
#define HWZC_FILTER_K_Q15  102943706UL  /* For τ=30µs; recompute if filter changes */
```

### 4.3 Sanity check at runtime

At 200 k eRPM:

```
stepPeriodHR = 1e9 / 200000 / 6 sectors  ... wait
```

Actually, sector period at N eRPM:

```
sector_time = 60 / (N · 6)   seconds per sector
            = 10 / N          seconds per sector
```

At 200 k eRPM: sector_time = 50 µs = 50000 ns = 5000 SCCP2 ticks.
So `stepPeriodHR ≈ 5000`.

`ω·τ_Q15 = 102943706 / 5000 = 20588` (in Q15, = 0.628)
Linear `ω·τ` = 0.628 → 36° equivalent (close to our table value of
37° at 240k — proportional).

---

## 5. Empirical Knobs (`AMP_PCT`)

```c
#define HWZC_FILTER_AMP_PCT  50
```

This scales the computed offset:

```
offset_final = offset_theoretical · AMP_PCT / 100
```

Why it's NOT 100 %:

### 5.1 The linear approximation overshoots

From the table in Section 2.3, the linear `ω·τ` overshoots the true
`sin(arctan(ω·τ))` by 7 % at ω·τ=0.4, 12 % at ω·τ=0.5, 22 % at 0.7.
Averaged across the speed range we typically run (50 k to 200 k eRPM),
the overshoot is roughly 5–15 %.

### 5.2 BEMF amplitude under load < open-circuit

The theoretical formula assumes `A` = open-circuit BEMF amplitude.
But under load, the motor's `Rs·I` voltage drop subtracts from BEMF
at the terminals:

```
V_terminal = A_oc − I · Rs
```

For our 2810 with Rs ≈ 50 mΩ and phase current up to 10 A, that's a
~0.5 V drop on a ~12–20 V BEMF — 3–10 % reduction.

### 5.3 Filter isn't quite first-order

Real PCBs have stray capacitance, layout inductance, source impedance
from the motor's own L. The effective filter is slightly higher order
than RC, with a slightly different phase response. The first-order
model is **close enough** for compensation but isn't exact.

### 5.4 The combined empirical fudge factor

All three effects above push the true offset **below** what the
linear theory predicts. Net: theory says ~100 %, empirical sweet
spot is 50–80 %.

### 5.5 How to tune

1. Pick a starting value: 75 % for a typical 30 µs filter
2. Run the motor up to ~150 k eRPM at a steady load
3. Read `Ibus` (bench current)
4. Increase `AMP_PCT` by 5 %; rebuild, flash, re-run
5. Note the new `Ibus`
6. Repeat. The optimum is the **minimum** of the Ibus curve.

Past the optimum, current rises again because the comp is
over-correcting — comparator fires too early → commutation leads the
rotor → motoring becomes inefficient.

### 5.6 Why not auto-tune

Could be done with an online minimum search (perturb-and-observe).
Stability concerns:
- The current minimum is shallow (~3–5 % current change over the
  full AMP_PCT sweep)
- Bench noise (supply ripple, load variation) is comparable
- Online perturbation injects torque ripple

Left as static tuning for now. A future enhancement could
auto-tune at boot from a brief no-load sweep.

---

## 6. Safety Clamps

```c
#define HWZC_FILTER_MAX_OMEGA_Q15   24000   /* ~0.73 rad ≈ 42° equiv */
#define HWZC_FILTER_MAX_OFFSET      600     /* ~15% of 12-bit ADC FS */
```

### 6.1 `MAX_OMEGA_Q15`

Caps the linear `ω·τ` from runaway. Two reasons:

1. **Linear approximation diverges from sine past 0.7** (22 % error
   at 0.7, 41 % at 1.0). Beyond `ω·τ ≈ 0.7` you should stop using the
   linear form anyway — switch to sine-form (Section 9).
2. **Overflow protection**: `offset = amp · ω·τ_Q15 >> 15`. With
   `amp` up to 4095 (12-bit ADC) and `ω·τ_Q15` up to 32768, the
   pre-shift product is up to 1.34e8, fits comfortably in uint32_t.
   But pushed higher there's tail risk on the `× AMP_PCT` stage.

Cap of 24000 (~0.73 rad ≈ 42°) is conservative.

### 6.2 `MAX_OFFSET`

Final clamp on the absolute CMPLO shift. 600 ADC counts ≈ 0.48 V
out of 3.3 V FS (15 %). Why:

- A CMPLO shift larger than ~15 % of FS is past what makes physical
  sense for normal operation. If the math wants more than that, it's
  reading a stale `stepPeriodHR` or an outlier amplitude
- Hard-clamping at 600 prevents a single bad iteration from putting
  CMPLO outside the BEMF signal range entirely (which would silence
  the comparator for that sector)

### 6.3 When the clamps fire

`dbgFilterOffset` lets you read the **actual applied** offset. If it
sits at exactly 600 for any sustained time, the clamp is firing →
either the speed went past the design envelope OR `AMP_PCT` is too
high.

---

## 7. Where the Numbers Live

| File | Constant / field | Purpose |
|---|---|---|
| `garuda_config.h:634` | `FEATURE_HWZC_FILTER_COMP` | Master enable |
| `garuda_config.h:640` | `HWZC_FILTER_K_Q15` | Encodes τ (= 102_943_706 for 30 µs) |
| `garuda_config.h:712` | `HWZC_FILTER_AMP_PCT` | Empirical scale, default 50 % |
| `garuda_config.h:723` | `HWZC_FILTER_MAX_OMEGA_Q15` | Cap on ω·τ, default 24000 |
| `garuda_config.h:728` | `HWZC_FILTER_MAX_OFFSET` | Cap on absolute offset, default 600 |
| `motor/hwzc.h:48` | `HWZC_ApplyFilterComp()` | The formula |
| `motor/hwzc.c:211` | call site #1 | Per-commutation update |
| `garuda_service.c` (ADC ISR) | call site #2 | Per-PWM-cycle live refresh |
| `garuda_types.h` | `hwzc.dbgFilterOffset` | Last applied offset (telemetry) |
| `garuda_types.h:298` | `hwzc.stepPeriodForFilterComp` | Slow IIR of period |
| `garuda_types.h:65` | `bemf.zcAmpForFilterComp` | Slow IIR of amplitude |

---

## 8. Porting to a Different PCB

### 8.1 The mandatory change (1 number)

Recompute `K_Q15` for the new filter's `τ`:

```
K_Q15 = 3431.9 · τ_ns
```

Quick reference:

| Filter R | C | τ | K_Q15 |
|---|---|---|---|
| 3 kΩ | 10 nF | 30 µs | **102,943,706** (current) |
| 1 kΩ | 10 nF | 10 µs | 34,314,569 |
| 3 kΩ | 3.3 nF | 9.9 µs | 33,971,423 |
| 1.5 kΩ | 22 nF | 33 µs | 113,238,074 |
| 2.2 kΩ | 22 nF | 48.4 µs | 166,103,994 |
| 4.7 kΩ | 22 nF | 103.4 µs | 354,872,773 |
| 10 kΩ | 10 nF | 100 µs | 343,145,687 |

For arbitrary R and C: `K = 3431.9 × R_kΩ × C_nF` (within rounding).

### 8.2 The probably-needs-retuning knob (1 number)

`HWZC_FILTER_AMP_PCT` is empirical. Per Section 5, start at 50–75 %
and tune for minimum bench Ibus. Different motor / filter / supply
combinations will land at different optima.

### 8.3 Maybe re-evaluate if τ is much different

For τ between 10 µs and 100 µs, the default caps (24000 and 600)
work fine.

For τ > 100 µs you're in territory where:
- Linear approximation has > 20 % error at moderate eRPM
- Phase lag exceeds 30° at modest speeds
- Consider switching to **sine-form** math (Section 9)

For τ < 10 µs (very high cutoff PCB filter, lots of switching noise
gets through):
- Filter comp barely matters — phase lag stays < 5° even at
  240 k eRPM
- Either leave it on (no harm) or set `FEATURE_HWZC_FILTER_COMP=0`

### 8.4 Validation procedure

After changing `K_Q15`:

1. Build, flash
2. Run motor at known speed (say 100 k eRPM)
3. Read `dbgFilterOffset` from snapshot
4. Compute expected:
   ```
   stepPeriodHR ≈ 10000 / N   (where N is eRPM in k, gives µs)
                = 100         (for 100k eRPM in SCCP2 ticks, ~10000)
   ```
   Actually `stepPeriodHR` in SCCP2 ticks at 100 k eRPM:
   ```
   sector_µs = 10 / N_thousand_eRPM × 1000 / 6 = (10000 / 100) / 6 = 16.67 µs
   stepPeriodHR_ticks = 16.67e-6 / 10e-9 = 1667
   ```
   ω·τ_Q15 = K_Q15 / stepPeriodHR = K_new / 1667
   offset = thresh × (ω·τ_Q15 / 32768) × AMP_PCT/100

5. If `dbgFilterOffset` matches within 5 %, the K_Q15 is right

### 8.5 Porting checklist

- [ ] Measure R and C on new PCB (or read schematic)
- [ ] Compute τ_ns = R_Ω × C_F × 1e9
- [ ] Compute K_Q15 = 3432 × τ_ns
- [ ] Update `HWZC_FILTER_K_Q15` in `garuda_config.h`
- [ ] Build, flash
- [ ] Validate dbgFilterOffset at known eRPM (Section 8.4)
- [ ] Tune `HWZC_FILTER_AMP_PCT` for minimum bench Ibus
- [ ] Only if τ > 100 µs: lower `MAX_OMEGA_Q15`, consider sine-form

---

## 9. Future Work

### 9.1 Switch to FPU + exact formula

The MCU has a single-precision FPU. The current Q15 implementation
was carried over from a Q15-only era of the firmware. Re-implementing
as float is straightforward:

```c
float omega_tau = K_FLOAT / stepP;
float offset_f = amp * omega_tau / sqrtf(1.0f + omega_tau * omega_tau);
offset_f *= AMP_PCT * 0.01f;
```

Benefits:
- Exact `sin(arctan(ω·τ))` — no more linear approximation error
- `AMP_PCT` becomes a pure "BEMF amplitude derate" knob with clear
  physical meaning (~ 90 % for our motor under load)
- Cleaner code, easier to reason about

Cost:
- Compare ISR timing on the new path — `sqrtf` is single-cycle on
  this FPU but the FPU register save/restore on ISR entry adds a
  few cycles
- One float multiply per sector, one per PWM cycle — needs to fit
  the ISR budget

This goes on the cleanup list. Not blocking; current Q15 path is
fine for the operating envelope we're in.

### 9.2 Auto-tune `AMP_PCT`

Per Section 5.6: brief no-load sweep at boot, find the AMP_PCT that
minimizes current. Couple iterations of perturb-and-observe at
moderate eRPM, lock in the value. Avoids the manual tune step.

### 9.3 Per-eRPM amplitude profile

Current code uses one `AMP_PCT` value for the whole speed range.
Reality: amplitude derate from load varies with eRPM (because torque
varies with eRPM at a given duty). A small lookup table indexed by
eRPM range could shave off the residual error.

Probably not worth it — bench data suggests the residual after a
good `AMP_PCT` tune is < 3 % current variation across the range.

---

## 10. Cross-References

- `docs/akesc_session_2026_05_26_191k_to_243k.md` Appendix A — the
  linear-vs-sine choice with bench data showing it works in practice
- `docs/pwm_frequency_effects.md` — why the filter cutoff is where
  it is and why we can't move it
- `docs/control_loop_pwm_coupling.md` — the broader architecture
  context (this comp is firmly on the PWM-independent "fast path")
- `memory/akesc_filter_comp_path1_203k_2026_05_26.md` — the original
  bench milestone that validated this approach (203 k eRPM
  breakthrough)

---

## 11. TL;DR

- **Problem**: PCB BEMF filter introduces phase lag → comparator
  fires late → commutation wrong → high current at high RPM.
- **Solution**: Shift comparator threshold so it fires earlier by
  exactly the filter delay. Math is `offset = A · sin(arctan(ω·τ))`,
  approximated linearly as `offset = A · ω·τ` for speed.
- **Implementation**: Single inline function `HWZC_ApplyFilterComp()`
  in `motor/hwzc.h`. Called from two sites (per-commutation +
  per-PWM-cycle live refresh).
- **Theoretical knob**: `K_Q15` encodes τ = 30 µs. Recompute if the
  PCB filter changes.
- **Empirical knob**: `AMP_PCT` = 50 %. Bench-tune for minimum
  current. Compensates for linear-approx error, load amplitude
  derate, real-world filter deviation from ideal RC.
- **Safety caps**: `MAX_OMEGA_Q15` and `MAX_OFFSET` prevent the
  formula from going wild at extreme speeds or numerical edge cases.
- **Future**: switch to FPU + exact `sin(arctan)`, possibly auto-tune
  `AMP_PCT`.
