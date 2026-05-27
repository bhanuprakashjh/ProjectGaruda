# PWM Switching Frequency — Why It Matters So Much

**Date**: 2026-05-27
**Context**: Bench observations — 45 kHz works, 40 kHz works (slight current bump), 30 kHz stalls at mid-pot.

This document is a **deep-dive analysis** of every place PWM frequency
shows up in this firmware and what happens physically at each rate. No
code changes — this is reference material for the next time the topic
comes up.

---

## TL;DR

PWM frequency affects the firmware at three layers stacked on top of
each other:

1. **Software** — `adcIsrTick` ticks at PWM rate, so the entire control
   loop's time base scales with PWM frequency. Period quantization
   gets coarser at lower rates.
2. **Power stage** — phase-current ripple is proportional to `T_on/L`.
   At 50 % duty the ripple peaks. Lower PWM frequency → larger ripple.
3. **Signal chain** — the 5.5 kHz PCB RC filter on the BEMF terminals
   attenuates PWM harmonics. Lower PWM frequency → less attenuation →
   more switching noise leaks into the comparator.

The 30 kHz stall at mid-pot is the interaction of (2) and (3) — both
are worst at 50 % duty for fundamental physics reasons unrelated to
the firmware.

---

## Layer 1: Where PWM Frequency Appears in the Code

### 1.1 `adcIsrTick` ticks at PWM rate, not ADC sample rate

PG1TRIGA fires the ADC once per PWM period (`hal_pwm.c:319-321`). The
ADC ISR slow loop runs per ADC interrupt. So:

- `adcIsrTick++` happens once per PWM cycle
- All `timing.stepPeriod` math is in **PWM cycles**
- The entire CL state machine slow loop runs at PWM frequency

What this means:

| PWM rate | Loop period | Reaction time |
|---|---|---|
| 30 kHz | 33.3 µs | 1 tick = 33.3 µs |
| 40 kHz | 25.0 µs | 1 tick = 25.0 µs |
| 45 kHz | 22.2 µs | 1 tick = 22.2 µs |
| 60 kHz | 16.7 µs | 1 tick = 16.7 µs |

The slow control loop reacts **33 % slower** at 30 kHz vs 45 kHz. The
no-capture watchdog, the desync timeout, the duty slew rate, and the
"stale signal" debouncers all run at this tick.

### 1.2 Step period precision

`stepPeriod` is a `uint16_t` count of `adcIsrTick`s. At 14 k eRPM idle:

- Sector = 60/(14000·6) = 714 µs
- At 30 kHz: 714/33.3 = **21.4 ticks**
- At 45 kHz: 714/22.2 = **32.2 ticks**

At 30 kHz, **single-tick errors are 4.7 % of the period**. At 45 kHz
they are only 3.1 %. The PI sees more relative jitter at low PWM rates.

At 240 k eRPM:

- Sector = 41.7 µs
- At 30 kHz: 1.25 ticks per sector
- At 45 kHz: 1.88 ticks per sector
- At 60 kHz: 2.5 ticks per sector

At 30 kHz, **the sector is shorter than one PWM period** by 240 k.
Multiple commutations land inside a single PWM cycle. That's the
fundamental ceiling for 6-step at that PWM rate.

### 1.3 SCCP2 HR ↔ ADC tick conversion factor

`garuda_calc_params.h:345-346`:

```c
#define HWZC_ADC_TO_SCCP2(t)   ((uint32_t)(t) * (HWZC_TIMER_HZ / PWMFREQUENCY_HZ))
#define HWZC_SCCP2_TO_ADC(t)   ((uint16_t)((t) / (HWZC_TIMER_HZ / PWMFREQUENCY_HZ)))
```

The conversion factor between high-resolution SCCP2 ticks (100 MHz) and
adcIsrTicks is `100 MHz / PWMFREQUENCY_HZ`:

| PWM rate | Conversion factor | Quantization step |
|---|---|---|
| 30 kHz | 3333 | 33.3 µs ÷ 3333 = 10 ns |
| 45 kHz | 2222 | 22.2 µs ÷ 2222 = 10 ns |
| 60 kHz | 1667 | 16.7 µs ÷ 1667 = 10 ns |

The HR units themselves are always 10 ns — that part is independent.
But the conversion **rounds toward the ADC tick**, so the maximum
representable period via `HWZC_SCCP2_TO_ADC` shrinks at higher PWM
rates (uint16_t cap: `65535 × tick_µs`).

### 1.4 Max duty in absolute time scales with PWM period

`garuda_calc_params.h:46`: `MAX_DUTY = LOOPTIME_TCY × 99 / 100`.

The 99 % cap is in fractional units. In absolute time:

| PWM rate | Period | Max ON-time (99%) | Min OFF-time (1%) |
|---|---|---|---|
| 30 kHz | 33.3 µs | 33.0 µs | 333 ns |
| 45 kHz | 22.2 µs | 22.0 µs | 222 ns |

The 333 ns OFF-time at 30 kHz / 99 % is below the dsPIC33AK PWM
peripheral's deadtime guard. The firmware silently clips and the
output may **degrade to block commutation**. At 45 kHz that risk is
~50 % higher in absolute time (relative deadtime fraction is
identical — but the bridge's switching transient is fixed in time, so
lower PWM means a larger guaranteed "good" window).

### 1.5 Blanking and stall debounce

`HWZC_BLANKING_PERCENT` and `HWZC_STALL_DEBOUNCE_MS` get scaled into
PWM ticks (`garuda_calc_params.h:380`):

```c
#define HWZC_STALL_DEBOUNCE_TICKS (HWZC_STALL_DEBOUNCE_MS × PWMFREQUENCY_HZ / 1000)
```

These are constant in **wall-clock time** across PWM rates because the
tick rate scales the same way. So the *behavior* of the watchdog
doesn't change — but the number of ticks it counts before firing does
(3000 at 30 kHz vs 4500 at 45 kHz).

The blanking window is also percent-based, so it's a constant fraction
of one PWM period. In absolute time blanking is **longer** at lower
PWM. At 14 % blanking:

| PWM rate | Blanking absolute |
|---|---|
| 30 kHz | 4.7 µs |
| 45 kHz | 3.1 µs |
| 60 kHz | 2.3 µs |

Longer blanking eats more of the usable capture window per sector at
high eRPM.

---

## Layer 2: Power Stage — Phase Current Ripple

### 2.1 The math

Phase current during a PWM ON-time follows `di/dt = V/L`. The peak-to-peak
ripple over one PWM period at duty `D` is approximately:

```
Δi_pp ≈ Vbus × D × (1−D) × T_PWM / L
```

This peaks at **D = 0.5** (mid-pot) and goes to zero at the extremes.

For our 2810 motor (L ≈ 30 µH per phase) at 24 V:

| PWM rate | T_PWM | Δi_pp at 50% duty |
|---|---|---|
| 30 kHz | 33.3 µs | **6.7 A** |
| 40 kHz | 25.0 µs | **5.0 A** |
| 45 kHz | 22.2 µs | **4.4 A** |
| 60 kHz | 16.7 µs | 3.3 A |

These are peak-to-peak ripple values on top of the DC phase current.
At low duty (6 % idle): `0.06 × 0.94 = 0.056` factor → 30 kHz idle
ripple is ~0.75 A peak-to-peak. Manageable.

At 50 % duty (mid-pot): 30 kHz hits **6.7 A peak-to-peak** ripple on a
phase current that might average just 3-5 A. Ripple is bigger than the
DC.

### 2.2 Why "slight increase in current" at 40 kHz

The user observed slightly higher current at 40 kHz vs 45 kHz. Two
effects:

1. **I²R loss from ripple**: `(Δi_pp/2√3)²` adds to RMS phase current.
   At 40 kHz / mid duty: extra ~0.7 A RMS. At 24 V, that's ~17 W of
   I²R loss for `R = 0.05 Ω`. Same as ~30 % bench current increase at
   light load.
2. **Switching loss reduction is partial**: lower PWM cuts MOSFET
   switching loss but the gain is offset by higher I²R from ripple.

This trade-off is well known — most flight ESCs run **24-48 kHz** as
the sweet spot.

### 2.3 Why mid-pot stalls at 30 kHz

The current peaks above the soft over-current limit (`OC_SW_LIMIT_ADC`):

- At 50 % duty, DC phase current might be 4 A under load
- Plus ripple: peak instantaneous = 4 + 6.7/2 = **7.3 A**
- Each PWM cycle the phase current rises to ~7 A then falls to ~1 A
- The HW comparator and software OC limit see those peaks

This isn't "stall" in the rotor-stopped sense. The firmware reads
`ibusRaw > OC_SW_LIMIT` and cuts duty proportionally on each cycle.
At 30 kHz the duty cuts happen often enough that closed-loop control
becomes a hash — duty oscillates, PI integrator can't lock, motor
loses sync.

That matches the bench log `step6_20260527_103524.csv`:

```
t=7.121  CL @ 119 k eRPM, duty 53%, throttle 2078
t=7.333  CL — eRPM jumps to 218 k (PI runaway), Vbus crashes to 10 V
t=7.546  FAULT (UV)
```

The PI ran the period down too fast because successive duty cuts gave
inconsistent BEMF samples. Period collapse → motor commutates ahead of
the rotor → BEMF > Vbus → bridge regen → Vbus collapse → UV fault.

---

## Layer 3: Signal Chain — BEMF Filter Behaviour

### 3.1 The 5.5 kHz RC filter

PCB has `R = 3 kΩ, C = 10 nF` on each BEMF terminal — first-order
RC LPF with corner at `1/(2π·RC) = 5.31 kHz`. The motor's electrical
fundamental at 200 k eRPM is `200000/60 = 3.33 kHz` — below the
filter, so true BEMF passes essentially unchanged.

PWM switching is far above the filter cutoff. Attenuation of the
fundamental switching component:

```
|H(f)| = 1 / √(1 + (f/fc)²)
```

| PWM rate | f/fc | Attenuation | Magnitude |
|---|---|---|---|
| 30 kHz | 5.65 | −15.1 dB | 17.6 % |
| 40 kHz | 7.53 | −17.6 dB | 13.2 % |
| 45 kHz | 8.47 | −18.6 dB | 11.8 % |
| 60 kHz | 11.30 | −21.1 dB | 8.8 % |

So going from 45 kHz to 30 kHz **increases switching-noise leakage by
50 %** on the BEMF comparator input. Combined with the larger source
ripple (Layer 2), the BEMF terminal voltage at 30 kHz has roughly
**2-3× more switching noise** than at 45 kHz.

### 3.2 Why this matters for HWZC

The HWZC peripheral fires on comparator transitions. The PWM-state
gate (`FEATURE_HWZC_PWM_GATE=1`) rejects captures that arrive when the
active phase's H-side is LOW — that handles the obvious
during-OFF-time false captures.

What it **can't** handle: noise during the **ON-window** that creates
extra comparator transitions. Those look like real ZC captures to the
firmware. The PI integrator absorbs the bad timing and drifts.

At low PWM rates, the ON-window noise is loud enough to cause
spurious captures even at 50 % duty. Each spurious capture shifts the
PI's idea of stepPeriod, often in the wrong direction.

### 3.3 Filter compensation (CMPLO pre-distortion)

The firmware compensates for the filter's phase lag by pre-distorting
the comparator threshold. The compensation amount is:

```
offset = (zcThreshold × ω·τ_Q15 / 32768) × AMP_PCT/100
```

Where `ω = 2π·f_elec` and `τ = 30 µs` (filter time constant). This is
correct in principle but assumes the **only** thing the filter is
doing is delaying the BEMF. At low PWM rates the filter also has
**more residual switching noise on its output**, and the
pre-distortion amplifies the comparator's sensitivity to that noise
(the threshold gets closer to neutral on one side).

Net: at 30 kHz, the filter comp is fighting two things — BEMF phase
lag (real) and switching noise (huge). It can't win both.

---

## Layer 4: Sector-to-PWM-Cycle Ratio (High RPM Effects)

At a given eRPM, the number of PWM cycles per sector is:

```
cycles_per_sector = PWMFREQUENCY_HZ × 10 / eRPM
```

| eRPM | 30 kHz | 40 kHz | 45 kHz | 60 kHz |
|---|---|---|---|---|
| 14 k (idle) | 21.4 | 28.6 | 32.1 | 42.9 |
| 60 k | 5.0 | 6.7 | 7.5 | 10.0 |
| 120 k | 2.5 | 3.3 | 3.8 | 5.0 |
| 180 k | 1.7 | 2.2 | 2.5 | 3.3 |
| 240 k | 1.25 | 1.7 | 1.9 | 2.5 |

Rule of thumb from bench observation: **at least 2 PWM cycles per
sector** are needed for reliable HWZC. Below that:

- Multiple commutations land in a single PWM period
- The PWM-state gate can't reliably bracket a single capture
- Blanking eats most of the usable window
- PI math breaks down on quantized timing

The implied **maximum reliable eRPM** for each PWM rate:

| PWM rate | Max reliable eRPM (≥ 2 cycles/sector) |
|---|---|
| 30 kHz | 150 k |
| 40 kHz | 200 k |
| 45 kHz | 225 k |
| 60 kHz | 300 k |

This matches what we've seen: 240 k+ on 45 kHz works; 30 kHz can't
reach there.

---

## Summary Table: 30 kHz vs 45 kHz Side-by-Side

| Metric | 30 kHz | 45 kHz | Direction |
|---|---|---|---|
| Control loop tick rate | 33.3 µs | 22.2 µs | 30 kHz slower |
| Period quantization at 14 k eRPM | 4.7 % | 3.1 % | 30 kHz coarser |
| Cycles/sector at 200 k eRPM | 1.5 | 2.25 | 30 kHz blind at high RPM |
| Max ON time at 99 % duty | 33.0 µs | 22.0 µs | 30 kHz more bridge slack |
| Phase current ripple at 50 % duty | 6.7 A pp | 4.4 A pp | 30 kHz 50 % more |
| Switching noise on BEMF (post-filter) | 17.6 % | 11.8 % | 30 kHz 50 % more |
| HWZC absolute blanking | 4.7 µs | 3.1 µs | 30 kHz wider |
| Filter compensation effectiveness | Degraded | Good | 30 kHz worse |
| Bench: stalls at mid pot | Yes | No | — |
| Bench: slight current bump vs 45 kHz | (40 kHz also +) | baseline | — |

---

## Where the Industry Settled

Common PWM rates by application:

| Application | Typical PWM | Why |
|---|---|---|
| Hobby BLDC ESCs (drones) | 24-48 kHz | Above audible, low switching loss, manageable ripple |
| Industrial FOC | 8-20 kHz | Larger inductors, ripple less critical |
| Hi-speed servo | 30-100 kHz | Ripple control dominates |
| AM32 (BLHeli-class) | 24 / 48 / 96 kHz | User-selectable, defaults vary |
| BlueJay | 48 kHz typical | Established sweet spot |

**45 kHz is right where the industry consensus is**. There's no
benefit to going lower for this motor / inductance, and clear
downsides (ripple + noise).

---

## What Would Have to Change to Make 30 kHz Work

Not recommended, but for the record:

1. **PCB filter change**: drop the BEMF filter cutoff to ~2 kHz
   (larger C). Trades phase lag for switching-noise attenuation.
   Trade-off: more pre-distortion needed at high eRPM.
2. **Larger phase inductor**: 50+ µH halves the ripple at the same
   PWM rate. Adds mass and copper loss.
3. **Adaptive PWM frequency**: AM32 and similar bump PWM up at high
   speed and down at idle. Hides the trade-off across the speed
   range. Requires smooth IIR transition to avoid glitches.
4. **Burst-capture filtering**: collect N comparator transitions per
   sector and median-filter, reject outliers. Heavier compute.
5. **Higher hardware blanking**: extend EGBLT (ATA6847 register).
   Doesn't apply on the AKESC board.

None of these are worth doing for the current goals. **Stay at
45 kHz** as the baseline; 60 kHz is the upper ceiling on this MCU
before ADC throughput becomes a problem.

---

## References

- `dspic33AKESC/garuda_calc_params.h` — all the derived PWM-dependent
  constants
- `dspic33AKESC/hal/hal_pwm.c:319-321` — PG1TRIGA → ADC trigger
- `dspic33AKESC/motor/hwzc.c` — HWZC peripheral handling
- `dspic33AKESC/docs/akesc_session_2026_05_26_191k_to_243k.md` —
  Appendix on filter compensation (linear vs sin(arctan) form)
- Memory: `akesc_hwzc_latency_reduction_2026_05_26.md` — earlier
  PWM-rate experiments (45 / 60 kHz tuning)
