# Startup Methods — Selection & Tuning Guide

Updated 2026-06-12. All methods are selectable by flags in `garuda_config.h`
and are **independent of each other** (no cross-dependencies; mutually
exclusive pairs are enforced by `#error` guards in `garuda_calc_params.h`).

## Quick selection table

| Method | Flag(s) | State trace | Time to idle (2810@24V) | Bench status |
|---|---|---|---|---|
| **AM32 kick+listen** (DEFAULT) | `FEATURE_AM32_STARTUP=1` | ARMED→CL | **~0.75s** | 6/6 (2026-06-12) |
| PLL-from-align | `FEATURE_PLL_STARTUP=1` | ARMED→ALIGN→CL | ~1.65s | 5/5 + a32000 2/2 (2026-06-12) |
| Sine + morph (classic) | `FEATURE_SINE_STARTUP=1`, both above =0 | ARMED→ALIGN→OL_RAMP→MORPH→CL | ~2.1s | long-proven baseline |
| Classic trapezoid OL | all three =0 | ARMED→ALIGN→OL_RAMP→CL | ~2s | original baseline |
| Skip-morph | `FEATURE_SKIP_MORPH=1` (+`CL_COAST_VERIFY`) | ALIGN→OL→coast→CL | ~1.95s | PARKED (9/9 but 2-stage feel) |
| I-f current-controlled | `FEATURE_IF_STARTUP=1` | ARMED→IF_RAMP→CL | n/a | PARKED (op-amp sense wall) |

All listener-based methods (AM32, PLL) require `FEATURE_ADC_CMP_ZC=1` +
`FEATURE_HWZC_SECTOR_PI=1` (the HWZC capture + sector-PI/SCCP1 machinery) —
those are core to 6-step operation, not startup choices.

Per-motor parameters (`rampTargetErpm`, `hwzcCrossoverErpm`, align duty…)
come from the **active GSP profile** (`gsp/gsp_params.c`), so one firmware
image adapts per motor. The startup *method* is compile-time.

---

## 1. AM32 kick+listen (`FEATURE_AM32_STARTUP`) — default

**How it works** (modeled on AM32 `main.c:977 startMotor()`): on
arm-complete there is *no align and no ramp*. One blind commutation at
`MIN_DUTY` from the unknown rotor angle, HWZC armed immediately with the
period seeded near `(2/3)·rampTargetErpm`, and `zcSynced` trusted from
event 1. The normal sector PI consumes captures (including early phantoms);
the rotor pulls into the rotating field organically and the PI follows it
up to the duty-floor equilibrium.

**Tuning knobs**

| Knob | Default | Effect |
|---|---|---|
| `AM32_START_SEED_ERPM` | `0` = derive `(2/3)·rampTargetErpm` from profile | Initial listener period guess. Lower = easier rotor catch, more time in phantom territory. Derived default reproduces the bench-proven 2000 on profile 2. |
| Profile `rampTargetErpm` | per-profile | Drives the derived seed. |
| Defensive PI (`FEATURE_HWZC_PI_DEFENSIVE`) | on | Handles capture silence (period relaxes ~1%/event until captures resume) — this is the analog of AM32's polling/voting mode. Leave on. |

**Failure signatures & fixes**
- *Fiction lock*: telemetry shows plausible eRPM but the motor doesn't spin
  (phantom captures captured the PI). Likeliest on motors whose BEMF floor
  is above the seed. Fix: raise `AM32_START_SEED_ERPM` toward the profile's
  `hwzcCrossoverErpm`, or use PLL-from-align (its consistency gate rejects
  phantoms).
- *No catch* (motor twitches, never follows): seed too high for the rotor
  inertia to catch from rest. Lower the seed. Twin data (2810): catch fails
  for initial field ≥ ~2000 eRPM jump from rest *with* schedule; pure
  kick+listen catches because the PI slows to the rotor.
- *Desync recovery cycles* (IDLE→CL loops): the 150 ms no-capture watchdog
  fired. Usually a wiring/sense problem, not a tuning one.

**When to prefer**: bench/fan/prop motors with a low BEMF floor
(crossover ≤ ~2k) where startup feel and time matter. Not recommended for
VEX-class (floor 6k) without raising the seed — and even then, PLL is safer.

## 2. PLL-from-align (`FEATURE_PLL_STARTUP`)

**How it works**: ALIGN parks the rotor at a known angle (sine align if
`FEATURE_SINE_STARTUP=1`, classic step-0 align otherwise — both supported).
Then CL directly: the sector-PI/SCCP1 machinery runs a **blind accelerating
schedule** from `PLL_START_ERPM0` at `PLL_START_ACCEL_ERPM_PER_S`, engaging
one step ahead of the park angle (60° lead). Captures are consumed but
**discarded below the capture floor** (phantom-proof) and gated by a
**consistency check** above it: `PLL_START_SYNC_CAPS` consecutive captures
landing in the same sector fraction (|Δ|<T/4, plausibility T/8..7T/8) ⇒
declared synced, the normal PI takes over seamlessly. No morph, no hand-off
event.

**Tuning knobs**

| Knob | Default | Effect |
|---|---|---|
| `PLL_START_ERPM0` | 300 | First commanded speed after align. Keep low — the catch is the fragile part (twin: ≥2000 from rest fails on the 2810). |
| `PLL_START_ACCEL_ERPM_PER_S` | 32000 | Blind schedule accel. Bench-proven 4000/16000/32000 on the 2810; from e0=300 the twin clears 32000 with margin. Higher = shorter high-slip-current window. If the rotor buzzes and never syncs, halve it. |
| `PLL_START_CAPTURE_FLOOR_ERPM` | 2500, **auto-raised to the profile's `hwzcCrossoverErpm` if higher** | Below this, captures are discarded as phantoms. |
| `PLL_START_SYNC_CAPS` | 6 | Consecutive consistent captures to declare sync. Raise if false syncs appear; lower for faster handover. |
| `PLL_START_TARGET_ERPM` | 10000 | Blind schedule ceiling if sync never fires (then it holds — drop throttle to stop). |

**Failure signatures & fixes**
- *Ramps but never syncs, motor spinning*: capture position inconsistent —
  check `rej%` (sense noise) or widen `SYNC_CAPS` window understanding; on a
  new motor verify the floor is above its real BEMF noise region.
- *Motor doesn't follow the ramp*: accel too high for rotor inertia at
  MIN_DUTY torque — lower `ACCEL`; or raise align duty (profile
  `alignDutyPct`) so the park is solid.
- *Wrong-angle lock at low speed*: floor too low for this motor — raise it
  (or rely on the auto-raise from `hwzcCrossoverErpm`).

**When to prefer**: anything where robustness across motors matters more
than 0.9s of align time; high-BEMF-floor motors (VEX-class); first bring-up
of an unknown motor (the gate gives clean diagnostics: `pllStartGood`,
capture fraction).

## 3. Sine + morph (classic, `FEATURE_SINE_STARTUP`)

Sine align → open-loop sine ramp to `rampTargetErpm` → MORPH (progressive
sine→trapezoid blend with Hi-Z windows for BEMF) → CL. The long-proven
baseline; smoothest audible start; slowest; the morph hand-off is the
historically fragile part (see `akesc_*` session docs).

Key knobs: profile `sineRampModPct` (sine drive strength), `rampTargetErpm`
(hand-off speed — must be where BEMF is real for this motor: see the ×12/5
conversion-bug saga, commit 9767bb1), `MORPH_CONVERGE` sector count.

## 4. Classic trapezoid OL ramp (all startup flags 0)

Step-0 align → forced 6-step commutation accelerating at profile
`rampAccelErpmPerS` to `rampTargetErpm` → CL hand-off. Simplest; the
hand-off slam is bounded but real ((V_duty−BEMF)/R at the floor). Knobs are
all per-profile: `alignTimeMs/alignDutyPct`, `rampAccelErpmPerS`,
`rampTargetErpm`, `olRampDutyPct`.

## 5/6. Parked methods

- **Skip-morph** (`FEATURE_SKIP_MORPH` + `FEATURE_CL_COAST_VERIFY`): OL ramp
  → bridge-off coast-listen (measures TRUE rotor period/sector from clean
  BEMF) → synced engage. Bench 9/9 single-jerk but two-stage feel; parked.
- **I-f** (`FEATURE_IF_STARTUP`): current-controlled SVPWM ramp. Blocked by
  the phase op-amp sense wall (OA1/OA2 unresponsive in 6-step builds);
  compiled out until the analog chain is fixed.

---

## Per-profile startup audit (2026-06-12)

Derived values with defaults: AM32 seed = (2/3)·rampTarget; PLL floor =
max(2500, crossover).

| # | Profile | rampTarget | crossover | AM32 seed | PLL floor | AM32 verdict | PLL verdict |
|---|---|---|---|---|---|---|---|
| 0 | HURST 5PP | 2000 | 5000 | 1333 | 5000 | ⚠ seed ≪ crossover: HWZC captures unreliable below 5k for this motor — fiction risk. Prefer PLL or classic. | ✓ floor auto-raises to 5000 |
| 1 | A2212 | 3000 | 1500 | 2000 | 2500 | ✓ same regime as bench motor | ✓ |
| 2 | 5010/2810 (bench) | 3000 | 1500 | 2000 | 2500 | ✓ **bench-proven 6/6** | ✓ **bench-proven** |
| 3 | 5055 | 2000 | 1500 | 1333 | 2500 | ✓ (heavy rotor: if no catch, raise seed) | ✓ (accel 32000 may be high for the inertia — try 8000) |
| 4 | COBRA 470KV | 3000 | 1500 | 2000 | 2500 | ✓ expected OK (high-R limits slip current naturally) | ✓ |
| 5 | XROTOR 1150KV | 3000 | 1500 | 2000 | 2500 | ✓ ~2810 regime | ✓ |
| 6 | VEX 4000KV | 12000 | 6000 | 8000 | 6000 | ⚠ untested: seed 8000 is detectable territory but the **catch from rest at an 8k field is unproven** — micro rotor may catch (light), may not. Test with PLL first. | ✓ floor auto-raises to 6000; set `PLL_START_TARGET_ERPM ≥ 15000` for this motor |

Notes:
- The deeper portability issue for VEX-class is not the startup method but
  the BEMF counts at low speed (see the VEX analysis: ~6 ADC counts at 3k).
  Both new methods respect that through the profile-derived floors.
- All verdicts above except profile 2 are **twin/derivation, not bench** —
  treat ⚠ rows as "needs a bench session with that motor".

## Verified flag combinations (2026-06-12)

| Combo | Compiles | Twin functional | BENCH (2810 @24V) |
|---|---|---|---|
| AM32=1, SINE=1 (default) | ✓ | 12/12 angles | ✓ 3/3 (derived seed reads 2010) |
| AM32=1, SINE=0 | ✓ | 4/4 angles | — |
| PLL=1, SINE=1 | ✓ | 12/12 angles | ✓ 5/5 + a32000 2/2 (06-11/12) |
| PLL=1, SINE=0 (classic align) | ✓ | 4/4 angles | ✓ 3/3 + floor auto-raise verified live (`set_param hwzcCrossoverErpm 4000` → sync plateau 3.2k→4.4k) |
| SINE=1 only (sine+morph baseline) | ✓ | (long-proven) | ✓ 2/2 regression — full 5-state trace, true ~3k CL entries, unchanged |
| all 0 (classic trapezoid) | ✓ | (baseline path) | — (least-used; untested post-decoupling) |
| AM32=1 && PLL=1 | `#error` (by design) | — | — |
