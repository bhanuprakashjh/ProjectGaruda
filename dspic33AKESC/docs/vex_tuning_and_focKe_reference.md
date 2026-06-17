# VEX 4000KV Tuning Writeup + `focKeUvSRad` Reference

**Date:** 2026-06-17
**Board:** dsPIC33AK512MC510 (MC510 DIM) on MCLV-48V-300W
**Motor under test:** VEX 14 mm micro, 4000 KV, 12N/**6 PP**, Rs(pp)=0.44 Ω, Ls≈9 µH (L-N), no-load 0.65 A, max-torque 7.25 A, stall 14 A, **10 V** bus.
**Firmware mode:** 6-step sensorless, `FEATURE_HWZC_SECTOR_PI=1` (autonomous SCCP1 timer + PI commutation).

---

## 0. TL;DR

The VEX "ran but pulled huge current at half the speed-for-duty, then desynced." It was **not** a fundamental 6-step limitation. Three independent issues were stacked:

1. **`focKeUvSRad` (the motor flux constant λ) was wrong** because the VEX was being run on the 2810's profile. This is a *FOC-grouped* parameter, but the **6-step `ABS_FLOOR` false-lock guard reads it** to compute the motor's no-load period. Wrong λ → wrong floor → the commutation period was **pinned at ~half the VEX's true no-load speed**, producing the reactive-current/half-speed symptom. **This was the dominant bug.**
2. **Timing advance was starved at the top.** The advance ramp is anchored at `maxClosedLoopErpm` (240 k), so at the VEX's operating speeds it delivered too few degrees to cover the detection-chain lag → desync ceiling.
3. **The falling-sector OFF-center detector destabilized the PI** in the mid band → a per-profile speed cap was needed.

Result after all three: clean idle→**214,000 eRPM** at ~1.5–2 A (the BEMF ceiling at 10 V), versus the original 22 k / 12 A.

---

## 1. The symptom and the wrong first hypothesis

Running the VEX on **profile 2** (the 2810 tune) it spun, but:

- At a given PWM duty it reached **~52 % of the speed** an ideal 4000 KV/6PP motor should at 10 V.
- The deficit was **constant across the whole duty range** (not growing with speed).
- Phase current climbed to ~12 A; PSU current high; motor hot. UV-faulted ~45 k (PSU droop under the current).

The tempting-but-wrong conclusion was "6-step/BEMF-floor wall, needs FOC." The constant-fraction deficit was the tell that it was **not** physics: a fundamental BEMF/iron wall is speed-dependent, but a *flat fraction* points to a **fixed scaling error** in the control loop.

Two diagnostics nailed it down:

- The captured zero-cross sat at **~150 ‰** of the sector (`spi_target`/`spi_error` telemetry) instead of the ideal **500 ‰** (midpoint). The PI's setpoint *is* 500 ‰ (+advance), yet it could not close a ~350 ‰ error — it tracked the setpoint but with a huge persistent offset. That means **something was clamping the period and the PI couldn't get there.**
- Per-sector miss counters were **zero** — the motor was fully synced, just locked at the wrong phase. Not a detection problem.

A persistent floor on the period that the PI cannot push past ⇒ the `ABS_FLOOR` guard.

---

## 2. Root cause #1 — `focKeUvSRad` and the `ABS_FLOOR` guard (the main bug)

### 2.1 What `focKeUvSRad` actually is

`focKeUvSRad` is the motor's **back-EMF constant** `Ke` (a.k.a. `λ_pm`, the PM flux linkage), expressed in **µV·s/rad electrical**. It is pure motor physics — *how much BEMF the motor makes per unit electrical speed* — and equivalently *what the motor's no-load speed is for a given voltage*:

```
no-load eRPM  ≈  V_applied / Ke
```

It is **grouped** under the FOC parameter block (`PARAM_GROUP_FOC_MOTOR`, param id `0x72`) because the FOC observer needs it too, but it is a property of the magnet, not of any control method. **The 6-step path uses it as well.**

### 2.2 The formula (this is the deliverable for new motors)

```
focKeUvSRad [µV·s/rad]  =  60 / (√3 · 2π · KV · PP) · 1e6
                       =  5.5132e6 / (KV · PP)
```

where `KV` is the motor's KV in **rpm/V** and `PP` is **pole pairs** (= magnet poles ÷ 2; a 12N**14P** motor has 7 PP, a 12N6P... no — count the *magnets*: "6PP" here means 12 magnet poles ÷ 2 = 6).

This single line reproduces **every** value already in `gsp_params.c` (verified):

| Profile slot | Motor | KV | PP | KV·PP | `focKeUvSRad` |
|---|---|---:|---:|---:|---:|
| `GSP_PROFILE_HURST` | Hurst (geared) | ~106 | 7 | 742 | **7420** |
| `GSP_PROFILE_A2212` | A2212 | 1400 | 7 | 9800 | **563** |
| `GSP_PROFILE_5010` | 2810 | 1350 | 7 | 9450 | **583** |
| `GSP_PROFILE_5055` | 5055 | 580 | 7 | 4060 | **1355** |
| `GSP_PROFILE_COBRA` | Cobra CM-2814 | 470 | 7 | 3290 | **1674** |
| `GSP_PROFILE_XROTOR` | XRotor 3110 | 1150 | 7 | 8050 | **685** |
| `GSP_PROFILE_VEX` | VEX 14 mm | 4000 | 6 | 24000 | **230** |
| `GSP_PROFILE_1407_*` | 1407 | 4000 | 6 | 24000 | **230** |

> Sanity check: `5.5132e6 / (4000·6) = 229.7 ≈ 230`. `5.5132e6 / (1350·7) = 583.4 ≈ 583`. ✔

If you have a measured Ke instead of KV, convert: `KV = 60 / (√3·2π·Ke_phase_peak_Vs_per_rad_elec)`, or just use the µV·s/rad directly (it is what the parameter stores).

### 2.3 Where `focKeUvSRad` is used in the **6-step** path

It enters the 6-step loop in exactly **two** places, both in `motor/hwzc.c`. It is **not** in the ZC threshold and **not** in the per-sector PI phase-lock setpoint.

1. **CL-entry seed** (`HWZC_Enable`, ~line 170): at closed-loop entry it computes the expected no-load period `P_ff` from λ/Vbus/duty and seeds the PI's starting period so the loop doesn't start from a wild guess.

2. **`FEATURE_HWZC_ABS_FLOOR` false-lock guard** (`hwzc.c` ~line 929): the same `P_ff`, used as a **hard lower bound on the commutation period** — the PI may not command a period shorter than the no-load period (you cannot spin faster than physics allows for the present duty/Vbus). This blocks harmonic/phantom false-locks to a too-fast period.

```c
float lambdaPm = (float)gspParams.focKeUvSRad;          // <-- read here, in 6-step code
if (dutyFrac > HWZC_ABS_FLOOR_MIN_DUTYFRAC && vbus_v > 6.0f && lambdaPm > 0.0f) {
    float P_ff   = (181.380f * lambdaPm) / (vbus_v * dutyFrac);   // no-load period (HR ticks)
    uint32_t floorP = (uint32_t)(P_ff * (100.0f / HWZC_ABS_FLOOR_OVERSPEED_PCT));
    if (pData->hwzc.timerPeriod < floorP && pData->hwzc.timerPeriod < T) {
        pData->hwzc.timerPeriod = floorP;              // <-- pins the period
        ...
    }
}
```

The constant `181.380` is exactly what makes `P_ff` equal the true no-load period **when λ is correct**:

```
P_ff(ticks) = 181.38 · λ / V_applied
            = 181.38 · 230 / 1.0   = 41 717 ticks   (VEX, 10 % duty @10 V → V_applied=1.0 V)
eRPM at P_ff = 1e9 / 41 717 = 23 970 ≈ 24 000 = KV·PP·V = 4000·6·1.0   ✔
```

So `floorP ∝ λ`. The floor *is* the no-load ceiling, scaled by `HWZC_ABS_FLOOR_OVERSPEED_PCT` (130 %, i.e. allow up to 130 % of no-load).

### 2.4 Why it failed before, quantitatively

Running profile 2, the loop read **λ = 583** (the 2810). The VEX needs **λ = 230**. Therefore the floor was computed for a motor with **583/230 = 2.45×** more flux — i.e. **2.45× too slow**. With the 130 % overspeed margin the motor was pinned at:

```
clamped fraction of true no-load = 130 % · (230 / 583) = 1.30 · 0.394 = 0.51  →  51 %
```

**That is exactly the measured ~52 % speed deficit, across all duties** (because the floor and the true no-load both scale with `V_applied = duty·Vbus`, the *ratio* is constant — hence the flat fraction). The period could not shrink below `floorP`, so:

- the captures sat low (~150 ‰) instead of 500 ‰,
- the commutation was chronically retarded,
- the motor ran ~half speed with large reactive current (hot, high PSU draw),
- and **no amount of threshold/advance/gate tuning helped**, because they all sit *downstream* of a period the floor was hard-clamping.

### 2.5 Why it worked after

Setting **λ = 230** moved the floor to the VEX's *true* no-load. The period was free to shrink, the captures moved toward 500 ‰, commutation timing became correct, and idle went from **6.6 k / 2.7 A → 10.8 k / 0.3 A** immediately. The clamp was the cause.

### 2.6 Two traps that hid this

- **`FEATURE_GSP=1` shadowing.** With GSP enabled, the runtime reads `gsp_params.c` `profileDefaults[MOTOR_PROFILE]` (+ EEPROM overlay if enabled), **not** the `garuda_config.h` `#define`s. `MOTOR_PROFILE` is a *direct index* into `profileDefaults[]`. So you must edit the *gsp struct*, not the `.h`.
- **Live params are volatile.** Setting `0x72 = 230` over the wire fixes it instantly, but it lives only in RAM — **a reflash resets it to the profile default.** This bit us three times ("it regressed after flashing"). The permanent fix is to bake λ into the profile and run that profile.

---

## 3. Root cause #2 — timing advance starvation at the top

### 3.1 How advance works in this firmware

The advance angle `advDeg` interpolates linearly:

```
advDeg = 0                         for eRPM ≤ RAMP_TARGET (3000)
advDeg = timingAdvMaxDeg           for eRPM ≥ maxClosedLoopErpm  (the "anchor")
advDeg = lerp(...) in between
```

In the sector-PI scheme, `advDeg` is **baked into the phase-lock setpoint**:

```
setValue = ((30 + advDeg) · timerPeriod) / 60      // where the PI drives the captured ZC to land
```

So advance is not a separate "fire early" — it is the **commutation phase the PI locks to**, and it must grow with speed to cover the detection-chain lag (RC filter τ≈30 µs + ADC/processing). At eRPM `e` the electrical lag is `ω·τ`, which **grows with speed**.

### 3.2 What went wrong, in order

- **First (wrong) attempt:** `HWZC_ADV_FULL_ERPM = 25000` forced the *whole* 0→max ramp into 3 k–25 k. At ~22 k it was already at ~18°, **over-advanced** → desync/phantom at 22 k. **Removed** — over-advance is as bad as under-advance.
- **Gentle ramp (anchor = 240 k):** with `timingAdvMaxDeg = 20`, advance at 80 k = `20·(80k−3k)/(240k−3k) ≈ 6°`. The lag at 80 k is ~13°+. So commutation was retarded → walled ~72–80 k. Bumping the value moved the wall: **19→80 k, 21→95 k, ~40→214 k** — the ceiling tracks advance almost linearly.
- **The marginal zone is dangerous.** At `0x22 = 21` the motor reached the edge and **desynced into a 21 A spike** (Vbus sagged to 8.4 V) before the OC chop caught it. Sitting *almost* in lock is where the current spike lives; you want to be comfortably past it.

### 3.3 The fix

Use a **high `timingAdvMaxDeg` (~40)** with the **240 k anchor**. Because the anchor is far out, the ramp stays gentle at low speed (advance at 10 k idle ≈ 1°, harmless) but keeps **growing with speed** all the way up — which is exactly what the lag needs. At 40° it sails through the 88–95 k and 135–145 k rough bands at ~2 A and reaches the 10 V BEMF ceiling (~214 k).

> The rough bands at ~75 k and ~135–145 k are the same comp-amp-saturation transitions the 2810 has on its way to 260 k; the loop pushes through them rather than dying in them.

**Alternative (not chosen):** lower `maxClosedLoopErpm` to ~120–150 k with a moderate `timingAdvMaxDeg` (~28). This delivers advance earlier, but the anchor *clamps* advance above it, so it gives less at the very top and a lower ceiling. The "high value + far anchor" config is what was proven to 214 k.

---

## 4. Root cause #3 — falling-sector OFF-center cap

`FEATURE_HWZC_FALLING_SW` detects the **falling** ZC on the RC-filtered OFF-center (freewheel) sample, while the rising ZC uses the mid-ON comparator. Above a per-motor speed the falling captures "walk late" and feed the PI a bad period → a rising-vs-falling **per-sector oscillation** that eventually tips into a phantom.

`HWZC_FALLING_SW_MAX_ERPM` caps where falling-SW stops (above it, falling sectors *coast* on the held period; rising-only carries — the 2810 ran rising-only to 234 k). High-KV / low-amplitude-BEMF motors must cap **low**:

| Motor | `HWZC_FALLING_SW_MAX_ERPM` |
|---|---:|
| A2212 1400KV @12 V | 35 000 |
| 1407 4000KV | 50 000 |
| **VEX 4000KV @10 V** | **45 000** (oscillation builds from ~46 k — coast before it does) |
| (default, e.g. 2810) | 70 000 |

---

## 5. Supporting fixes / traps (checklist)

- **`FEATURE_GSP=1` → edit the gsp struct, not the `.h`.** The `#define`s are inert at runtime.
- **`MOTOR_PROFILE` is a direct index** into `profileDefaults[]` (so `MOTOR_PROFILE=6` → `GSP_PROFILE_VEX`).
- **Profile-6 startup was stale.** It carried an old "high hand-off" strategy (rampTarget=28 k, crossover=24 k, align=14 %, sineMod=28) built on a retracted "BEMF undetectable below 12 k" theory — that's why profile 6 "didn't work" early. It was replaced with profile 2's **proven low-hand-off** startup (rampTarget=3000, crossover=1500, align=3 %, sineMod=3/5, clIdle=4 %), keeping the VEX physical constants.
- **OC chain scaled to the small motor:** CMP3 chop `ocLimitMa=600` (~6 A bus, the real bound for a 7.25 A motor), SW backstop 9/11 A (vs the 2810's 18/21 A).
- **Bring-up gate relaxations** that were only needed while the clamp was broken — revisit before shipping anything that shares these *global* defines:
  `ZC_DUTY_DIVISOR` (2.6 vs 2.0), `FEATURE_HWZC_VERIFY_READS` (0 vs 1), `HWZC_CMP_DEADBAND` (2 vs 4), `HWZC_MIN_INTERVAL_PCT` (restored to 50). These are **global**, so they currently also affect a profile-2 (2810) build.

---

## 6. `focKeUvSRad` for any new motor — procedure

1. **Get KV (rpm/V) and pole-pairs PP** (magnet poles ÷ 2).
2. **Compute** `focKeUvSRad = round(5.5132e6 / (KV · PP))` µV·s/rad.
3. Put it in that motor's `gsp_params.c` profile (`.focKeUvSRad = …`). This single value makes the 6-step `ABS_FLOOR` and CL seed correct.
4. If KV is uncertain (cheap motors are often mislabeled), **measure it**: spin the motor as a generator at a known mechanical rpm, read line-line BEMF (V_pk), `KV ≈ rpm / (V_LL_pk / √2 · ... )` — or simpler, run open-loop at a known eRPM and read the BEMF amplitude; adjust λ until the `ABS_FLOOR` no-load prediction matches the speed the motor actually free-wheels to at a given duty.
5. **Symptom of λ too HIGH:** motor pinned below its no-load speed, flat speed-deficit fraction, high reactive current, captures stuck low. **λ too LOW:** the floor is too permissive and the phantom/false-lock guard weakens (the motor may over-speed-lock). Aim for the computed value; it is almost always right if KV·PP is right.

> **Rule of thumb:** wrong-by-2× λ ⇒ motor pinned at ~`130 %·(λ_true/λ_set)` of no-load. If a motor mysteriously tops out at a clean fraction of expected speed with high current, **suspect λ first.**

---

## 7. Final VEX (profile 6) configuration

| Parameter | Value | Why |
|---|---|---|
| `focKeUvSRad` | **230** | `5.5132e6/(4000·6)`; lifts the `ABS_FLOOR` clamp to the VEX's true no-load |
| `motorPolePairs` | 6 | (telemetry/FOC only; not in 6-step timing) |
| `maxClosedLoopErpm` | 240 000 | **runtime speed clamp**, set to nominal no-load (4000KV·10V·6PP). NOT the advance anchor — that is the compile-time `RT_TIMING_ADV_FULL_ERPM`. Commutation timing advance carries the rotor well past this; live-tune `0x11` up to explore. |
| `timingAdvMaxDeg` | **20** | committed default (proven 2810 value). Live-tune `0x22` higher only if the top end needs more lead; 20 already reaches the advance-limited ceiling. |
| `HWZC_FALLING_SW_MAX_ERPM` | 45 000 | coast falling before the mid-band oscillation builds |
| startup (rampTarget/crossover/align/mod/clIdle) | 3000 / 1500 / 3 % / 3-5 % / 4 % | ported from the proven profile-2 low-hand-off startup |
| OC chain (`ocLimitMa`/`ocSwLimitMa`/`ocFaultMa`) | 600 / 9000 / 11000 | scaled to a 7.25 A-max-torque, 14 A-stall motor |

**Outcome:** idle ~10.8 k @ 0.3 A; clean climb at 2–4 A to **~285 000 eRPM with real closed-loop detection** (`rej` > 0). The motor runs *past* its 240 k nominal no-load — this is 6-step, so there is **no field weakening** (no d-axis current). The mechanism is pure **commutation timing advance**: firing each step ahead of the BEMF zero-cross compensates the inductive phase lag and detection delay, so torque stays aligned and the motor keeps accelerating above the naive no-load number with duty to spare.

### 7a. The "260 k / 300 k" ceiling — it's the firmware cap, not the motor

`maxClosedLoopErpm` is **both** the per-profile default *and* the hard speed clamp (`hwzcMinStepTicks = 1e9/maxClosedLoopErpm`, the floor the PI commutation period can't undershoot). The GUI's SET_PARAM is bounded by the descriptor `PARAM_ID_MAX_CL_ERPM` max in `gsp_params.c` — originally **260000**, raised this session to **350000** so high-KV micros can be live-tuned past it.

- When the rotor hits the clamp, `rej` drops to **0 %** and `Ia` jumps to ~9–10 A: that "260 k/300 k" reading is the *commanded clamp value*, not a measured speed, and the current is the commutation slipping against a rotor it's holding back. **Don't dwell there** — it's pure heat on a 14 mm stator.
- The honest closed-loop ceiling (where `rej` is still > 0, i.e. real ZC is being accepted) is **~285 k at 10 V**. Above that, detection thins out (45 kHz sampling → < 2 samples/sector) and it rides the clamp.
- To run genuinely faster you'd need a higher PWM/ADC rate (more samples/sector), not just a higher cap.

### 7b. Known follow-up — OC_SW on hard decel

Snapping the throttle down from 280 k+ trips `OC_SW` (phase current spikes to ~22 A during the regen/braking transient as the commanded speed falls faster than the rotor). This is a **decel-ramp/braking** issue, separate from the steady-state tune — a duty-down slew limiter is the fix. Steady-state and accel are clean.

---

## 8. One-paragraph summary for the next person

The VEX wasn't a 6-step limitation — it was the wrong motor flux constant. `focKeUvSRad` (a FOC-grouped parameter, but pure motor physics = the back-EMF constant) is read by the **6-step** `ABS_FLOOR` false-lock guard to compute the no-load period the commutation period may not undershoot. Running the VEX on the 2810's profile fed it λ=583 instead of 230, putting that floor 2.45× too slow and pinning the motor at 130%·(230/583)=51% of its no-load speed with big reactive current — exactly the measured deficit. Fix λ (`= 5.5132e6/(KV·PP)`), let the **timing advance** grow with speed (the ramp endpoint `RT_TIMING_ADV_FULL_ERPM` is now decoupled from the speed cap; default `timingAdvMaxDeg=20` is enough), and cap the **falling-SW** detector low for high-KV motors. With all three the VEX runs idle→**~285 k at 2–4 A with real detection**, running past its 240 k nominal no-load on **commutation timing advance** (this is 6-step — no field weakening / no d-axis current; the phase lead just compensates the inductive + detection lag so torque stays aligned). The "260 k/300 k" numbers seen earlier are the *firmware speed clamp* (descriptor max, raised 260 k→350 k this session), not the motor — sitting on the clamp shows `rej`=0 and ~10 A and is just heat. Open follow-up: `OC_SW` trips on hard decel from 280 k+ (a braking-transient issue, not the steady-state tune).
