# T-Motor U3 KV700 — Profile Port from 2810 (profile 2)

**Target motor:** T-Motor U-Power **U3 KV700**, 12N14P, 4 mm shaft
**Source profile:** `MOTOR_PROFILE == 2` → runtime slot `GSP_PROFILE_5010` (the 2810 1350KV @24V)
**Mode:** 6-step (FOC params noted separately, change only if running FOC)

> With `FEATURE_GSP=1` the `garuda_config.h` `#define`s are **inert** — the runtime
> reads `gsp/gsp_params.c` `profileDefaults[GSP_PROFILE_5010]` (+ EEPROM overlay).
> Edit the values in `gsp_params.c`, or tune live via GSP params.

---

## Headline

The U3 KV700 and the 2810 share the two hardest-to-change things — **7 pole pairs**
and **~50 mΩ Rs** — so this is mostly a **KV + voltage rescale**, not a ground-up profile.

## Motor comparison

| | 2810 (profile 2) | U3 KV700 |
|---|---|---|
| Config | 12N14P | 12N14P |
| **Pole pairs** | 7 | **7 — SAME** |
| **KV** | 1350 | **700 (~½)** |
| **Rs** | ~50 mΩ pp | **~50 mΩ pp — SAME** |
| Ls | ~25 µH | **unknown — bigger stator, likely higher** |
| Voltage | 24 V (6S) | **rated 3–4S (11.1–16.8 V)** |
| Rotor mass | light | **97 g — heavier** |
| Idle current | — | 0.5 A @10V |
| Max continuous | — | 25 A / 500 W |

---

## CRITICAL change

**`focKeUvSRad` (λ): 583 → 1125**

This is the exact parameter behind the VEX half-speed bug — it feeds the 6-step
`ABS_FLOOR` BEMF clamp. Using the profile's own formula:

```
λ = 60 / (√3 · 2π · KV · PP)
U3: 60 / (1.732 · 6.283 · 700 · 7) = 0.001125  →  focKeUvSRad = 1125
```

Lower KV → ~2× the λ. Get this wrong and the U3 pins at part-speed / high current
exactly like the VEX did.

---

## Deciding input: VOLTAGE

The U3 KV700 is rated **3–4S**. Almost every duty/threshold below scales with voltage.
Numbers here assume **4S (16.8 V)**. Because Rs is the same as the 2810, holding the
same current at the lower voltage needs **1.43× the duty** (`24 / 16.8 = 1.43`).

- **Running at 24 V (6S, over-spec — ~117k eRPM no-load, risky):** keep the 2810 duty values.
- **Running at 4S (16.8 V):** use the scaled values below.

---

## Parameter deltas (assuming 4S / 16.8 V)

### Must change
| Param | 2810 | U3 KV700 | Why |
|---|---|---|---|
| `focKeUvSRad` | 583 | **1125** | λ for 6-step ABS_FLOOR (critical) |
| `motorPolePairs` | 7 | 7 | no change |

### Voltage-dependent (×1.43 for 4S; keep 2810 values if 24 V)
| Param | 2810 | U3 KV700 | Why |
|---|---|---|---|
| `alignDutyPct` | 3 | **4–5** | + heavier rotor lock |
| `rampDutyPct` | 8 | **11** | hold ramp current |
| `clIdleDutyPct` | 4 | **6** | lower KV gives more low-speed BEMF — don't overshoot |
| `sineAlignModPct` | 3 | **4** | |
| `sineRampModPct` | 5 | **7** | |

### Speed / advance (KV-driven)
| Param | 2810 | U3 KV700 | Why |
|---|---|---|---|
| `maxClosedLoopErpm` (advance **anchor**, not a limiter) | 260000 | **~85000** | U3 no-load ceiling @4S = 700·16.8·7 ≈ 82k. Leaving 260k makes advance ~5° at real speed (the old 2810 220k bug). |
| `timingAdvMaxDeg` | 20 | **15 (start)** | lower speed + stronger BEMF need less advance; tune up if rough |

### Startup / inertia (heavier 97 g rotor)
| Param | 2810 | U3 KV700 | Why |
|---|---|---|---|
| `rampAccelErpmPerS` | 3000 | **1800** | heavy rotor can't follow fast accel |
| `initialErpm` | 150 | **120** | gentler first step |
| `rampTargetErpm` | 3000 | **2500** | lower KV → ZC appears earlier |

### Demag (inductance)
| Param | 2810 | U3 KV700 | Why |
|---|---|---|---|
| `zcDemagBlankExtraPct` | 20 | **24** *(if Ls higher)* | bigger stator → longer demag tail. Directly relevant to load-desync. Confirm with Ls. |

### Leave as-is (board-limited, not motor-limited)
- `ocSwLimitMa` 18000 / `ocFaultMa` 21000 / `ocLimitMa` 20000 / `ocStartupMa` 22000
  — shunt saturates ~22 A regardless; U3's 25 A continuous still lands here.
- `rampCurrentGateMa` 10000 — keep.

### Voltage protection (set for your supply — in TUNING_DEFAULTS / global, not this block)
- `vbusOvAdc` / `vbusUvAdc`: currently scaled for 24 V (OV ≈ 28.8 V, UV ≈ 4 V).
  For a **4S LiPo** reset OV (~17.5–18 V) and UV (~12–13 V = 3.0 V/cell) or there is
  no pack protection. On a bench PSU, at least set OV sanely.

### FOC-only — change ONLY if running FOC mode (inert in 6-step)
- `focRsMilliOhm` ≈ 25
- `focLsMicroH` — needs measurement
- `focMaxElecRadS` → ~17000 (4S)
- `focKpDqMilli` / `focKiDq` — recompute from Ls
- `an1078ThetaBaseDegX10` / `an1078ThetaKE7` / `an1078KslideMv` / `an1078IdFwMaxDecia` — re-tune

---

## Open items to finalize the numbers
1. **Bench voltage / cell count** — 4S (assumed here) or over-volting to 24 V? Sets all duty rows.
2. **Ls of the U3** — measured or datasheet? Sets demag blanking and (FOC) PI gains.
   If unknown, start `zcDemagBlankExtraPct = 24` and watch for early ZC misses.
3. **Prop or no-prop on the bench?** Sets how conservative the startup/accel rows should be.

## Recommended approach
Clone `GSP_PROFILE_5010` into a new dedicated `GSP_PROFILE_U3` slot (with a new
`MOTOR_PROFILE` mapping) rather than overwriting the dialed-in 2810 — keeps the 2810
tuning intact and gives the U3 its own home.

---

## Sources
- T-Motor U3 KV700 — official store: https://store.tmotor.com/goods-317-U3+KV700.html
- GetFPV U3 700KV specs: https://www.getfpv.com/tiger-motor-u3-700kv-u-power-professional-motor.html

*Specs used: 12N14P (7 PP), KV 700, Rs ~50 mΩ, 4 mm shaft, 97 g, idle 0.5 A @10V,
max continuous 25 A / 500 W, rated 3–4S.*
