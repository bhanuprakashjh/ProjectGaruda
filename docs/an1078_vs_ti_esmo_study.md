# TI C2000 ESMO vs Our AN1078+PLL — Detailed Study

After seeing the actual TI source (`ti-c2000-motor-control-sdk/libraries/observers/esmo/`),
**the conclusions from my earlier verbal comparisons need correcting**.
This document is the read-the-source-carefully version.

## Side-by-side comparison

### Current model (identical)

Both observers use the same Luenberger-style current model:

```c
EstIalpha = F · EstIalpha + G · (Valpha - Ealpha - Zalpha)
EstIbeta  = F · EstIbeta  + G · (Vbeta  - Ebeta  - Zbeta)
```

Where `F = 1 - Rs·Ts/Ls` and `G = Ts/Ls`. **TI uses separate `Fdsmopos`,
`Fqsmopos`, `Gdsmopos`, `Gqsmopos` to allow different d/q axis params for
salient motors** (we use one pair since 2810 and A2212 are non-salient).

### Sliding-mode switching (essentially identical)

```c
// TI:
Zalpha = __fsat(IalphaError, +E0, -E0) * Kslide;

// AN1078:
if (|IalphaError| < MaxSMCError) Zalpha = Kslide × IalphaError / MaxSMCError;
else                              Zalpha = ±Kslide;
```

I previously claimed TI used a continuous sigmoid (tanh). **It does not.**
Both saturate. The only differences:

| | TI ESMO | AN1078 |
|---|---|---|
| Saturation symbol | `__fsat(err, ±E0)` | branch on `|err| < MaxSMCError` |
| Linear-region slope | `Kslide / 1.0` (after fsat) | `Kslide / MaxSMCError` |
| Saturation magnitude | `Kslide × E0` | `Kslide` |
| Numerical structure | identical | identical |

So the **chattering reduction story I told earlier was wrong** — TI doesn't
have it either. Both still produce switching at the boundary.

### LPF chain (genuine difference: 1 stage vs 2)

```c
// TI ESMO — single first-order LPF:
Ealpha = Ealpha + Kslf · (Zalpha - Ealpha);
Ebeta  = Ebeta  + Kslf · (Zbeta  - Ebeta);

// AN1078 — two cascaded:
Ealpha      = Ealpha      + Kslf      · (Zalpha - Ealpha);
EalphaFinal = EalphaFinal + KslfFinal · (Ealpha - EalphaFinal);
```

**This is the real architectural difference.** Two LPF stages give 90° of
phase lag at fundamental motor frequency; one stage gives only 45°.
AN1078 needs more chatter suppression but pays double phase lag.

### PLL (the big difference: dq-frame vs αβ-cross-product)

#### Our PLL (cross-product on αβ)
```c
phase_err = e_beta·cos(θ_pll) - e_alpha·sin(θ_pll);
ω_pll    += K_i · phase_err · Ts;
θ_pll    += (ω_pll + K_p · phase_err) · Ts;
```

The discriminator amplitude scales with `|E|`, so loop gain varies with
speed. We compensated by accepting variable bandwidth.

#### TI ESMO PLL (dq-frame)
```c
// Project Eα/Eβ into the rotor's dq frame using current θ estimate
Ed = Ealpha · cos(θ) + Ebeta · sin(θ);
Eq = Ebeta  · cos(θ) - Ealpha · sin(θ);
Eq_mag = sqrt(Eα² + Eβ²);          // total BEMF magnitude

// Sign-corrected theta error, NORMALIZED by BEMF magnitude
thetaErr = Ed · sign(Eq) / Eq_mag;

// PI on thetaErr → speed
pll_ui   += pll_Ki · thetaErr;
pll_Out   = clamp(pll_Kp · thetaErr + pll_ui, ±Umax);

speedFlt  = lpf(pll_Out);
θ        += speedFlt · Ts;          // integrate speed → angle
```

**Three meaningful improvements** here:

1. **`thetaErr` is normalized** (`Ed / Eq_mag`) — small angle error gives
   nearly the same value regardless of speed. Loop gain is constant.
   No bandwidth-vs-speed compromise.

2. **Sign-corrected** — when motor reverses (Eq < 0), the sign flip is
   tracked correctly. Our cross-product PLL has trouble with reversal.

3. **Speed-driven, angle-integrated** — PLL outputs speed, integrate to
   angle. Smoother angle than direct angle output.

### Theta offset compensation (similar, with TI's auto-derive)

#### Our approach (`an1078_smc.c`)
```c
dyn_offset = AN_SMC_THETA_OFFSET_BASE + AN_SMC_THETA_OFFSET_K × |ω|;
Theta = pll.theta_est - π/2 + dyn_offset;
```

`BASE` and `K` are tuned empirically per motor.

#### TI ESMO approach
```c
// Auto-derived from Kslf and operating speed
thetaOffset = atan2(speedRef × offsetSF, Kslf);
thetaPll    = θ - thetaOffset;
```

`offsetSF` and `Kslf` are physical (sample period, LPF coefficient). The
`atan2` directly computes the LPF's phase lag at the current speed,
**no empirical tuning of K needed**. This is `arctan(ωτ)` which is
exactly the lag of a single-pole filter at frequency ω.

This is **much cleaner** — our `BASE + K·ω` is a linear approximation
to `atan(ω·τ)` that requires manual tuning. TI does it from physics.

### What TI doesn't have that we do
- **GUI live-tuning of 4 SMO params** (BASE, K, Kslide, FW max) — nice
  to have but not algorithmic
- **Per-profile motor model from EEPROM** — same
- **Field weakening with hysteresis** — TI ESMO doesn't ship FW; you
  add it externally. Our integrator+hysteresis FW works fine.
- **OL→CL handoff with thetaError bleed** — TI MotorWare has its own
  startup state machine, but it's separate from ESMO. Neither is
  inherently better; the bleed is well-suited to AN1078.

## Summary: real vs imagined differences

| Claim from my earlier verbal compare | Reality |
|--------------------------------------|---------|
| "TI uses continuous sigmoid" | ❌ False. TI uses hard saturation, just like AN1078. |
| "TI auto-tunes Kslide" | ❌ False. Kslide is a `setKslideParams` config, set once. |
| "TI eliminates chattering" | ❌ False. Same chattering, both rely on LPF. |
| "TI has dq-frame PLL" | ✅ True. This is the big real win. |
| "TI has single LPF (less lag)" | ✅ True. Half the phase lag. |
| "TI has speed-adaptive theta offset from physics" | ✅ True. `atan2(ω·SF, Kslf)`, no tuning. |

So my earlier comparison oversold TI. The **real** advantages are:
1. dq-frame PLL with normalized error → constant loop gain, robust at low speed
2. Single-stage LPF → less phase lag → smaller offset compensation needed
3. Auto-derived theta offset → one less tuning knob

## Adaptation paths

### Option A — Minimal port (drop second LPF stage)

**~30 lines of code change** in `an1078_smc.c`:
- Remove `EalphaFinal`/`EbetaFinal` second LPF
- Use `Ealpha`/`Ebeta` directly for PLL input
- Re-tune `THETA_OFFSET_K` (will be ~half of current)
- Possibly increase `KSLF_MAX` since one LPF needs less attenuation

**Risk**: low. The cascaded LPF was for chatter rejection. With the PLL
we already added, chatter is filtered at the PLL level too. Dropping
one stage may improve high-speed phase margin without hurting noise.

**Gain**: less phase lag → smaller `THETA_OFFSET_K` needed → cleaner
operation across speeds. Possibly reduces the Vd error we tuned out
empirically.

**Effort**: 1 hour including bench validation.

### Option B — Replace PLL with dq-frame normalized PLL

**~80 lines** in `pll_estimator.c` (new variant) or directly in
`an1078_smc.c`:
- Project `Ealpha`/`Ebeta` into rotor dq using current `θ_est`
- Compute `thetaErr = Ed · sign(Eq) / Eq_mag`
- Run PI on this normalized error
- Integrate speed → angle (instead of direct angle integration)

**Risk**: medium. Different PLL dynamics, will need fresh tuning of
`PLL_Kp`, `PLL_Ki`. Speed reading also changes character (filtered, vs
our raw `omega_est`).

**Gain**:
- Constant PLL bandwidth across speed range (no more hunting at low BEMF)
- Robust to motor reversal (sign-corrected error)
- Cleaner low-speed CL (the zero-throttle desync we hit)

**Effort**: 4-6 hours including bench validation across speed range.

### Option C — Full TI ESMO port (replace `an1078_smc.c` entirely)

**~500 lines** new `ti_esmo.c` + `ti_esmo.h`. Port the TI MotorControl
SDK's `esmo.c` body to fit our `AN_SMC_T` interface:

- Adapt to our types/units (no `_iq`, no `MATH_vec3`, no DC bus scaling)
- Replace `__fsat`, `__sinpuf32`, `__atan2puf32` with `fsat`, `sinf`, `atan2f`
- Drop `voltage_sf`, `current_sf`, `thetaDelta` scaling (we work in SI units)
- Map ESMO config fields to gspParams where appropriate
- Keep our motor.c, gsp_params, GUI, FW logic untouched

**Risk**: medium-high. Lots of fiddly param mapping. TI's tuning recipes
in their app notes won't translate 1:1 (different units, different
ranges). Need to derive new tuning workflow.

**Gain**: All three improvements above (single LPF + dq PLL + auto offset)
in one shot. Plus TI has tested it across many motor sizes.

**Loss**: Loses our existing per-tick `THETA_OFFSET` live-tuning slider —
the auto-derived offset doesn't have a knob. Could re-add as a "trim"
on top, but adds complexity. Also loses the `id_ref_fw` integration
since TI's ESMO doesn't include FW (have to keep that in motor.c).

**Effort**: 1-2 days including bench validation, doc updates, GUI tweaks.

### Option D — Stay with current AN1078+PLL, do nothing

**Effort: zero.** Current state already hits 213k eRPM peak on 2810,
clean prop tests on A2212, no-prop reliable, profile-driven motors,
GUI live tuning. The remaining limitation (zero-throttle desync at
low end speed) is mitigated by raising idle to 2000 RPM mech.

## Trade-off table

| Aspect | A: Drop 2nd LPF | B: dq PLL | C: Full ESMO | D: Stay |
|---|---|---|---|---|
| Code change | ~30 lines | ~80 lines | ~500 lines | 0 |
| Bench time | 1 h | 4-6 h | 1-2 days | 0 |
| Risk | Low | Medium | Medium-High | None |
| Phase lag improvement | -45° at fundamental | small | -45° + adaptive | none |
| Low-speed PLL stability | unchanged | better | better | current state |
| Theta offset auto-derive | no | optional | yes | no |
| FW integration | keep ours | keep ours | needs re-wiring | keep ours |
| Live tuning of THETA_K | still useful | maybe redundant | replaced by auto | active |
| Loss of working code | none | small | larger | none |
| Maintainability | minor refactor | new file | new file | unchanged |

## Recommendation

For your use case (drone ESC, two motors validated, working state in
production), **the right path is A → maybe B, not C**.

**Concretely:**

1. **First — Option A (drop 2nd LPF stage)**. Low-risk, low-effort,
   addresses the LPF-lag root cause that drove all the `THETA_K`
   tuning. Validate. If it works clean across both motors, that's
   probably enough.

2. **If A still leaves zero-throttle desync** — do **Option B (dq PLL)**.
   The normalized-error PLL is a real-world improvement for low-speed
   CL stability. We'd see the benefit specifically at idle.

3. **Skip Option C unless** you're starting fresh on a new board family
   or motor class where AN1078 fundamentally doesn't fit. The risk vs
   reward isn't there for incremental improvement.

4. **Option D is also valid** if "good enough" is good enough. You have
   213k eRPM, profile switching, prop validation, GUI tuning. Lots of
   ESCs ship with worse.

## What about copying TI's licensing?

TI's source is **BSD 3-clause** (per the header). Compatible with our
project's license. No legal blocker for porting (Option C).

## Concrete TODO if you choose A

1. Edit `an1078_smc.c` `CalcBEMF`:
   ```c
   // Remove these two lines:
   s->EalphaFinal += s->KslfFinal * (s->Ealpha - s->EalphaFinal);
   s->EbetaFinal  += s->KslfFinal * (s->Ebeta  - s->EbetaFinal);

   // Use Ealpha/Ebeta directly in pll_update and atan2.
   ```
2. Update `AN_SMC_T` to drop `EalphaFinal`/`EbetaFinal` (or keep for
   telemetry, just don't use them in the control path).
3. Bench test: motor should start cleaner because total phase lag is
   halved. `THETA_OFFSET_K` should reduce from 1e-4 toward 5e-5.
4. If observer becomes noisy at high speed (chatter visible in Vd/Vq),
   bump `KSLF_MAX` from 0.85 → 0.7 to keep the single LPF reasonably
   suppressive.

## Concrete TODO if you choose B

1. New file `foc/dq_pll.c` based on TI's PLL block (lines 264-291 of
   `esmo.c`). Adapt to our `pll_estimator.h` interface.
2. In `an1078_smc.c`, swap `pll_update()` for `dq_pll_update()`.
3. Re-tune `PLL_KP`, `PLL_KI` for the new error normalization (TI uses
   `Kp ≈ 100`, `Ki ≈ 1` in their reference for typical motors;
   different scaling than our 628 / 98600).
4. Verify on bench across speed range, especially zero-throttle idle.

## References

- TI source: `ti-c2000-motor-control-sdk/libraries/observers/esmo/`
- TI app note SPRABZ0: "Sensorless Field-Oriented Control of 3-Phase PMSM"
- AN1078 reference: `lvmc-dspic33ck256mp508-an1078/`
- Our implementation: `dspic33AKESC/foc/an1078_smc.{c,h}`
- Our PLL: `dspic33AKESC/foc/pll_estimator.{c,h}`
