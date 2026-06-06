# Bringing Up a New Motor — Field Guide

> For anyone flashing this firmware onto a bench they can watch but didn't tune.
> Read this top to bottom **once** before your first start. It will save you an hour.

This firmware was developed and proven on a **2810 1350KV** motor at 24V. Two more
profiles are included as **physics-based starting points** that have NOT been run on
real hardware yet: **Cobra CM-2814/36 470KV** (profile 4) and **Hobbywing XRotor 3110
1150KV** (profile 5). Expect the XRotor to "just work" (it's electrically a twin of the
2810). Expect the **Cobra to need a few iterations** — it's a different regime (4× the
winding resistance, heavier rotor, low KV).

---

## Prerequisites — install these first (not in the repo, and can't be)

The repo has everything *project-specific* (all source, the `dspic33AKESC.X` MPLAB
project, Makefiles). But three Microchip toolchain pieces are machine installs — a clone
won't build without them:

1. **MPLAB X IDE** v6.30 or newer — https://www.microchip.com/mplab/mplab-x-ide
2. **XC-DSC compiler** v3.30 (the dsPIC33A toolchain — **NOT XC16**). MPLAB prompts to
   download it on first build if missing. Target device: **dsPIC33AK128MC106**.
3. **dsPIC33AK-MC Device Family Pack (DFP)** — install via MPLAB *Tools → Packs* (it also
   auto-prompts when you open the project). Without the DFP the device is "unknown" and
   nothing compiles.

**To build:** open `dspic33AKESC/dspic33AKESC.X` in MPLAB X → accept any prompt to fetch
a missing compiler/DFP → configuration `default` → Production build (hammer icon). Or from
a terminal with the toolchain on `PATH`:
```bash
cd dspic33AKESC && make -C dspic33AKESC.X CONF=default
```
First build on a fresh machine fetches the DFP + compiler once; after that it's a normal
build. The `build/` and `dist/` folders are gitignored (regenerated) — expected, not
missing files.

---

## 0. The three golden rules (do these first)

1. **POWER-CYCLE the board before every fresh test session.** Pull the power, wait 2s,
   re-apply. A *reflash does NOT reset the rotor*. After a fault the rotor parks at a
   random angle, and a bad park makes the *next* start fault too — a self-perpetuating
   loop that looks exactly like "the firmware is broken" but isn't. A clean power-up
   from rest breaks it. **If starts suddenly go flaky, power-cycle before changing
   anything.** (We burned a whole session learning this.)

2. **Code values win — what's in the file is what runs.** This build ships with
   `FEATURE_PARAMS_FORCE_DEFAULTS = 1` (`garuda_config.h`), so the firmware *ignores
   saved EEPROM* and uses the compiled profile. Edit `gsp/gsp_params.c` → build → flash →
   that's live. No EEPROM surprises. (Set it back to `0` only when you want to persist
   live-tuned values across power cycles.)

3. **One change at a time, and write down what you changed.** Startup is a coupled
   system. Change one parameter, reflash, observe, repeat. Shotgun-tuning five values
   at once tells you nothing.

---

## 1. Select your motor and flash

In `garuda_config.h`:

```c
#define MOTOR_PROFILE 2   // 2810 1350KV   (the proven reference)
//#define MOTOR_PROFILE 4 // Cobra CM-2814/36 470KV
//#define MOTOR_PROFILE 5 // XRotor 3110 1150KV
```

Build (Linux, XC-DSC / dsPIC33A toolchain):

```bash
cd dspic33AKESC
export PATH="/opt/microchip/xc16/v2.10/bin:$PATH"
make -C dspic33AKESC.X CONF=default
# hex: dspic33AKESC.X/dist/default/production/dspic33AKESC.X.production.hex
```

Flash with MPLAB X / IPE, or your usual programmer. Then **power-cycle** (rule 1).

---

## 2. Watch the motor start

```bash
python3 tools/step6_session.py     # live telemetry @ 50 Hz, logs CSV
```

You'll see the state machine walk:

```
IDLE → ARMED → ALIGN → OL_RAMP → MORPH → CL
```

| State | What's happening | What "good" looks like |
|-------|------------------|------------------------|
| **ALIGN** | DC sine vector holds the rotor at a known angle | rotor visibly locks; `Ia_pk` a few A, no fault |
| **OL_RAMP** | Open-loop sine V/f ramp, 150→3000 eRPM | smooth spin-up, `Ia_pk` rising but under OC |
| **MORPH** | sine→trap blend, hunts for the first real zero-cross | brief (~50 ms), then jumps to CL |
| **CL** | Closed loop — HWZC + sector PI runs the show | tracks throttle; `|Ibus|` *drops* as it speeds up |

A healthy start spends <1.5 s getting to CL. The `◀` markers in the log flag each state
change so you can see exactly where it died.

**The 22 A pulse at the MORPH→CL instant is NORMAL** on low-resistance motors — it's the
bridge energizing against the rotor for a few microseconds. It's bounded by hardware OC
and is brief. Only worry if it *latches* an OC_SW fault (see below).

---

## 3. Fault → cause → fix (the decision tree)

Find the state where it died (the row before `FAULT`) and the `fault` column.

### Died in ALIGN or OL_RAMP with `OC_SW`, `eRPM≈0`
**Too much startup current** — the sine amplitude is driving more current than the OC
soft-limit allows. This is the #1 first-flash failure on a high-resistance motor.
- **Lower `sineRampModPct`** (OL_RAMP) or **`sineAlignModPct`** (ALIGN) by ~25%.
- Sanity check the current: `peak A ≈ (modPct / 200) × Vbus / R_phase_to_phase`.
  Keep that under `ocSwLimitMa`.

### Stalls / desyncs in OL_RAMP or MORPH, no fault or `STALL` — rotor stops or stutters
**Too little startup torque, or ramping too fast for the inertia.**
- **Raise `sineRampModPct`** in steps of ~4 (more field → more torque). Watch `Ia_pk`
  stays under `ocSwLimitMa`.
- If it spins but then loses sync, **lower `rampAccelErpmPerS`** (slower ramp; heavy
  rotors need more dwell per step). Halve it and retry.
- Make sure the rotor actually *locked* in ALIGN first. If ALIGN looked weak, raise
  `sineAlignModPct`.

### Reaches MORPH but never enters CL (bounces MORPH→FAULT or times out)
**Zero-cross detection isn't locking.** Either the ZC is being blanked too long, or the
handoff is too strict.
- **Lower `zcDemagBlankExtraPct`** (less blanking → ZC seen earlier). Try -4.
- The morph exit needs `MORPH_ZC_THRESHOLD` (=4) good zero-crosses. Low-KV motors have
  *strong* BEMF so this is usually easy — if it's failing here, suspect blanking or a
  bad rotor park (power-cycle and retry first!).

### Enters CL then immediately faults `OC_SW` on throttle-up
**Commutation timing off, or accelerating into the current limit.**
- **Lower `timingAdvMaxDeg`** a few degrees (wrong advance = wrong-angle current spike).
- Raise the throttle *slowly* the first time — a fast slew can trip OC even when steady
  operation is fine.

### `OC_SW` only on hard throttle-DOWN, Vbus jumps above ~28V
**Regenerative braking** dumping energy back into the supply (bench PSU can't sink it).
This is benign on a battery; on a bench PSU just chop throttle more gently. Not a tuning
bug.

### Faults repeatedly no matter what you change
**STOP changing parameters.** Power-cycle the board (golden rule 1). A bad rotor park
mimics every symptom above. Confirm a clean start from rest *first*, then tune.

---

## 4. The startup parameters that matter (in priority order)

All in `gsp/gsp_params.c`, in your motor's `[GSP_PROFILE_*]` block:

| Param | What it does | Raise if… | Lower if… |
|-------|--------------|-----------|-----------|
| `sineRampModPct` | **Main startup torque knob** (OL_RAMP field strength) | stalls in ramp | OC_SW in ramp |
| `sineAlignModPct` | Align field strength (rotor lock-in) | weak/no lock | OC_SW in align |
| `rampAccelErpmPerS` | How fast the open-loop ramp accelerates | — | desyncs mid-ramp |
| `rampTargetErpm` | Open-loop→CL handoff speed | ZC won't lock low | overshoots |
| `timingAdvMaxDeg` | Commutation advance at speed | sluggish top end | OC/desync at speed |
| `zcDemagBlankExtraPct` | Post-commutation ZC blanking | false early ZC | ZC seen too late |
| `ocSwLimitMa` | Software OC trip | nuisance trips (cautiously!) | — |

> **The real knob is `sineRampModPct` / `sineAlignModPct`, NOT the trap duty %**
> (`rampDutyPct` / `alignDutyPct`). On the sine startup path those only cap the
> MORPH/CL duty — the sine *amplitude* is what makes startup torque.

### Quick current math (do this before raising any Mod%)
```
startup peak current ≈ (ModPct / 200) × Vbus / R_phase_to_phase
```
Keep it under `ocSwLimitMa`. Example, Cobra at 24V, R=0.188Ω:
`sineRampModPct=20 → 10% × 24 / 0.188 ≈ 12.8A` ✓ (under 16A limit).

---

## 5. Plan B: if the sine startup just won't cooperate

The firmware has a **second, simpler startup path** — classic forced trap commutation,
like AM32/BLHeli. Enable it in `garuda_config.h`:

```c
#define FEATURE_SINE_STARTUP 0
```

This switches ALIGN→`STARTUP_Align` and OL_RAMP→`STARTUP_OpenLoopRamp`→CL (no sine, no
MORPH — its absence is correct, not a fault). Fewer knobs (`alignDutyPct`, `rampDutyPct`,
`rampAccelErpmPerS`, `rampTargetErpm`).

**Bench-validated 2026-06-06** on the 2810: trap startup reaches CL cleanly, no faults, no
tuning — it just brute-forces the rotor up to handoff speed. So it's a *tested* fallback,
not a gamble. Two things to know:
- **It's rougher than sine** — forced commutation steps are audible and it pulls higher
  ramp current (the 2810 hit ~22 A in OL_RAMP vs sine's gentler ramp). Expect a harsher
  spin-up sound.
- **Roughness scales with 1/resistance.** The 2810 is low-R (0.05 Ω) so the ramp current
  is high — worst case. On the **higher-resistance Cobra (0.188 Ω) the same duty makes
  ~4× less current**, so trap is *gentler* there. **For the Cobra, trap is worth trying
  first if sine won't lock** — its brute-force nature suits an uncharacterized motor and
  the high R keeps the current civilized.

Tune `alignDutyPct` (rotor lock) and `rampDutyPct` (ramp torque) with the same current
math (`duty% × Vbus / R`). If OL_RAMP trips `OC_SW`, lower `rampDutyPct`; if it stalls,
raise it or slow `rampAccelErpmPerS`.

---

## 6. The two new profiles — starting values & what to expect

### XRotor 3110 1150KV (profile 5) — *should mostly just work*
Electrically a twin of the 2810 (R≈0.045Ω, 7 pole-pairs, light rotor). Profile copied
from the proven 2810, with `maxClosedLoopErpm=210000` (1150KV × 24V × 7pp ≈ 193k).
**Likely behavior:** starts clean. The only risk is bumping the BEMF/speed ceiling near
full throttle — a top-end issue, not a start issue. If it faults near max throttle,
that's the ceiling, not your startup.

### Cobra CM-2814/36 470KV (profile 4) — *expect to iterate*
Opposite regime: R=0.188Ω (4× the 2810), 117g rotor, low KV. Starting values:
- `sineAlignModPct=15`, `sineRampModPct=20` — **current-matched to the 2810's proven
  ~12A startup** (NOT arbitrary; see the current math). Safely under the 16A OC limit.
- `rampAccelErpmPerS=1000` — slow ramp (~3s) for the heavy rotor.
- `maxClosedLoopErpm=83000` (470KV × 24V × 7pp ≈ 79k).
- `ocSwLimitMa=16000` ≈ rated 17A continuous.

**Most likely first-flash outcome:** it either starts (great) or stalls in OL_RAMP
(raise `sineRampModPct` toward 24-26) or OC_SW trips in OL_RAMP (lower toward 16). One
of those three, and the telemetry tells you which. Iterate `sineRampModPct` first — it's
the dominant knob.

**Unknown we couldn't get from the datasheet:** the Cobra's *inductance* (Ls). It only
affects the FOC observer (unused in this 6-step build) and the ZC blanking. If you see
ZC-lock trouble in MORPH, nudge `zcDemagBlankExtraPct`.

---

## 7. When you get a clean run — capture it

`step6_session.py` writes a CSV per session under `sessions/`. When a motor starts
reliably and reaches a good speed, **save that CSV and note the parameter values** — that
pair (telemetry + the profile that produced it) is exactly what's needed to lock the
profile in, and to compare against the next motor.

---

*Questions this guide can't answer usually come down to: did you power-cycle? is the
current under the OC limit? are you changing one thing at a time? Start there.*
