# AN1078 Sensorless FOC — 80k → 206k eRPM Optimization

**Motor**: PRODRONE 2810, 1350 KV, 7 pole pairs, Rs=22 mΩ, Ls=10 µH, λ=0.000583 V·s/rad
**Hardware**: dsPIC33AK128MC106 on MCLV-48V-300W board, 24V supply
**Date**: 2026-04-25
**Result**: **206,637 eRPM peak measured**, 200k+ sustainable, 0 faults across 70+ s runs.
Beats 196k 6-step trapezoidal benchmark on the same motor.

This document is a postmortem of the optimization journey — what was wrong, what
fixed it, and why each fix mattered.

## TL;DR — five stacked fixes

| # | Fix | Ceiling Δ | Mechanism |
|---|-----|-----------|-----------|
| 1 | Dynamic θ-offset (BASE + K·ω) | 80k → 90k | speed-adaptive LPF lag compensation |
| 2 | `AN_FS_HZ` matched to real PWM rate | 90k → 115k | discretization was wrong by 67% |
| 3 | PWM bump to 48 kHz (then 60 kHz) | 115k → 171k | per-tick angle resolution |
| 4 | Angle PLL on (Eα,Eβ) | 171k → 176k | kills per-tick atan2 wobble |
| 5 | Field weakening (negative Id) | 176k → **206k** | breaks 24V voltage ceiling |

**The PLL (#4) is the most generalizable** — any AN1078-derived sensorless FOC
system pushing high speeds will hit the same per-tick atan2 wall. The PLL is
the standard fix used by AN1292, TI C2000 ESMO, and MXLEMMING.

## The bottlenecks, in order

### Bottleneck 1: Single-point θ-offset calibration

AN1078 inserts a constant `THETA_OFFSET` after `atan2(BEMF)` to compensate
the cascaded LPF's group delay between BEMF and rotor angle. The LPF's group
delay grows with motor frequency, so a single empirical offset is correct at
exactly one speed and wrong everywhere else.

**Symptom**: Vd grows linearly with motor speed despite the d-PI fighting it.
At 80k eRPM, observer V vector is ~80° off the q-axis.

**Fix**: speed-dependent offset.
```c
#define AN_SMC_THETA_OFFSET_BASE  0.349f   /* 20° at zero speed */
#define AN_SMC_THETA_OFFSET_K     1.0e-4f  /* rad per (rad/s elec) */

dyn_offset = BASE + K × |OmegaFltred|;
Theta = atan2(...) + dyn_offset;
```

The slope `K = 1e-4 rad/(rad/s)` was extracted from `(Vd, Vq)` measurements
at multiple speeds. Final tune: `Vd` stays within ±1.5V from 5k to 160k eRPM.

### Bottleneck 2: Discretization mismatch (silent 67 % error)

`AN_FS_HZ = 24000 Hz` in `an1078_params.h` was stale — the actual hardware
PWM frequency had been bumped to 40 kHz two weeks earlier in unrelated work.
Every constant derived from `AN_TS = 1/AN_FS_HZ` was wrong:

| Constant | Used in code | Correct (40 kHz) | Error |
|----------|-------------|-----------------|-------|
| `AN_TS` | 41.67 µs | 25 µs | 67% high |
| `G_PLANT = Ts/Ls` | 4.17 | 2.5 | 67% high |
| `F_PLANT = 1 - Rs·Ts/Ls` | 0.908 | 0.945 | over-damped |
| `KSLF_SCALE = Ts` | 41.67 µs | 25 µs | 67% high |

The SMC's current model was over-driven: every tick advanced `EstIalpha` by
67% more than physics dictated. This caused observer instability at 85-90k.

**Lesson**: any time PWM frequency moves, audit ALL constants derived from
`Ts`. The build hash now folds `AN_FS_HZ` so changes are detectable host-side.

**Fix**: `AN_FS_HZ = 60000.0f` matched to `PWMFREQUENCY_HZ`. They MUST move
together. Single change pushed ceiling from 90k to 115k eRPM — biggest single
win in the optimization.

### Bottleneck 3: Per-tick angle resolution (PWM rate)

At 40 kHz PWM and 115k eRPM (1917 Hz electrical), one PWM tick = 17° of
electrical rotation. The SMC's atan2 output reflects the LPF state at the
current tick — but with 17° of motor rotation between samples, even small
LPF residual ripple translates to angle hops bigger than the d-PI can track.

**Symptom**: stable Vd/Vq but Iq/Id swing chaotically (±15A) — the Park
transform aliases consistent physical currents into chaotic dq frame because
the angle bouncing around.

**Fix**: bump PWM rate. Linear scaling — at 48 kHz, the same 17°/tick limit
moves to 137k eRPM; at 60 kHz, 171k.

**This was a hardware band-aid**, not a real algorithmic fix. The proper
solution is the PLL (next bottleneck). 60 kHz pushes FET switching losses
50% above the 40 kHz baseline this hardware was designed for. Production
should fall back to 48 kHz once the PLL is in place.

### Bottleneck 4: AN1078's atan2 has no smoothing

This is the **fundamental architectural limit** of AN1078 (2008). Newer
Microchip references (AN1292), TI C2000 ESMO, NXP RTCESL, and MXLEMMING
flux observers all wrap a PLL around the BEMF estimates instead of feeding
atan2 directly into Park.

**The PLL** does cross-product phase discrimination on (Eα, Eβ):
```
phase_err = e_β·cos(θ_pll) - e_α·sin(θ_pll)
ω_pll    += K_i·Ts·phase_err
θ_pll    += (ω_pll + K_p·phase_err)·Ts
```

PI loop filter bandwidth ~ 50 Hz × |E|/(typical_E). Self-adapts: at low
BEMF (low speed), PLL is slow and noise-rejecting; at high BEMF, fast and
tracking.

**Why it eliminates the wobble**: PLL output is smooth even when input
(atan2) is noisy. The PI loop's bandwidth (~21 Hz at 0.2V BEMF, ~1 kHz at
10V BEMF) is well below the per-tick noise frequency, so wobble is rejected.

**Implementation reused** existing `foc/pll_estimator.{c,h}` from v2/v3 —
no new code, just integrated into AN1078's observer step.

```c
/* In AN_SMC_Position_Estimation() — replaces the atan2 line */
pll_update(&s->pll, s->EalphaFinal, s->EbetaFinal, AN_TS);
s->Theta = s->pll.theta_est - PLL_ANGLE_OFFSET + dyn_offset;
s->OmegaFltred = s->pll.omega_est;
```

`PLL_ANGLE_OFFSET = π/2` because BEMF leads rotor by 90° in PMSM convention.

After PLL: motor sustained 176k eRPM cleanly for 139 seconds, no faults.
Vd held within ±1.5V across the entire 5k → 160k range. Iq stayed under
1.5A (gloriously efficient at no-load).

### Bottleneck 5: Voltage ceiling at 24V

At 180k eRPM, `BEMF = ω·λ ≈ 11 V`. With `Vmax = 0.95·Vbus/√3 ≈ 13.16 V`,
the speed PI starts saturating: it demands more Iq, voltage limiter clamps
Vq, motor speed plateaus. Hard wall.

**Fix**: field weakening — drive Id negative to partially cancel rotor flux.
In rotor frame:
```
Vq = R·Iq + ω·L·Id + ω·λ
```
With `Id < 0`, `Vq` required is reduced, freeing voltage headroom for
higher speed. The trade-off: reactive current (no torque-producing benefit
at no-load), heating.

**Implementation**: integrator-based, not direct.
```c
const float FW_TRIGGER     = 0.91f;   /* mod threshold */
const float FW_KP_INT      = 400.0f;  /* A/s per (mod-thresh) unit */
const float FW_DECAY       = 0.9995f; /* per tick → ~50ms recovery */
const float ID_FW_MAX_NEG  = -12.0f;  /* limit FW current */

mod_now = sqrt(vd² + vq²) / vmax;
if (mod_now > FW_TRIGGER) {
    id_ref_fw -= FW_KP_INT × (mod_now - FW_TRIGGER) × dt;
    clamp(id_ref_fw, ID_FW_MAX_NEG, 0);
} else {
    id_ref_fw *= FW_DECAY;
}
id_ref = id_ref_fw;  // fed into d-PI
```

Plus: `AN_OVER_CURRENT_LIMIT` bumped 9 → 12 A so speed PI's Iq output
isn't capped below FW current draw. Total |I| can hit ~17A peaks
during throttle transients — within MCLV-48V-300W's rating but on the edge.

After FW: 200k+ eRPM clean. Throttle pegged at 4095, motor at its actual
electrical max for 24V supply. Going higher would need either field
weakening with deeper |Id| (current-limited by hardware) or higher Vbus.

## Architecture diagram (post-optimization)

```
   ADC IB,IA      ─►   Clarke   ─► (I_α, I_β)
                                       │
                                       ├──► Park ─► (I_d_meas, I_q_meas)
                                       │      ▲
   PWM V_α,V_β ◄── Inv Park ◄─ DQ ◄────┤      │ uses θ_drive
   commands              ▲             │      │
                         │             │      │
                  d-PI:  V_d ◄── id_ref_fw ◄──┼─ FW integrator ◄ |V|/Vmax
                  q-PI:  V_q ◄── speed PI ◄───┘   ▲
                                                  │
                                                 velRef ◄── throttle
                                                  │
                                                ω_pll
                                                  ▲
   ──────────────── SMC Observer ──────────────────┤
   I_α,I_β ─► CalcEstI ─► Z ─► CalcBEMF ─► (Eα,Eβ)F ─► PLL ──► θ_pll
                                                          ┐    │
                                                       offset  ▼
                                                          └►  θ_drive
```

## Tunable summary (working values)

```c
/* Loop timing */
#define AN_FS_HZ                    60000.0f   /* MUST = PWMFREQUENCY_HZ */
#define AN_IRP_PERCALC              60         /* 1 kHz speed loop */

/* Theta offset (dynamic) */
#define AN_SMC_THETA_OFFSET_BASE    0.349f     /* 20° at zero */
#define AN_SMC_THETA_OFFSET_K       1.0e-4f    /* rad per (rad/s elec) */

/* SMC observer */
#define AN_SMC_KSLIDE               2.5f       /* tuned for 2810 weak BEMF */
#define AN_SMC_MAX_LINEAR_ERR       1.0f
#define AN_SMC_KSLF_SCALE           AN_TS
#define AN_SMC_KSLF_MIN             0.05f
#define AN_SMC_KSLF_MAX             0.85f

/* PI controllers */
#define AN_KP_DQ                    0.063f     /* current loop */
#define AN_KI_DQ                    138.0f
#define AN_KP_SPD                   0.006f     /* speed loop, conservative */
#define AN_KI_SPD                   0.10f

/* Speed envelope */
#define AN_END_SPEED_RPM_MECH       500.0f    /* OL→CL handoff @ 366 rad/s */
#define AN_NOMINAL_SPEED_RPM_MECH   30000.0f  /* throttle ceiling = 215k eRPM */
#define AN_OL_RAMP_RATE_RPS2        1000.0f   /* slow OL ramp — prop friendly */
#define AN_CL_VELREF_SLEW_RPS2      12000.0f  /* fast CL throttle response */
#define AN_Q_CURRENT_REF_OPENLOOP   8.0f      /* OL torque — overcomes prop inertia */
#define AN_OVER_CURRENT_LIMIT       12.0f     /* matches FW |I| range */

/* Field weakening (in motor.c, not params.h) */
const float FW_TRIGGER     = 0.91f;
const float FW_KP_INT      = 400.0f;
const float FW_DECAY       = 0.995f;          /* fast decay — avoid stuck FW */
const float ID_FW_MAX_NEG  = -12.0f;
const float FW_IQ_GATE     = 1.0f;            /* only engage when Iq_ref > this */

/* Hardware */
#define PWMFREQUENCY_HZ   60000   /* PUSHING IT — see comment in garuda_config.h */
```

## Cautionary notes

### 60 kHz PWM is on the edge of this hardware

This is documented in `garuda_config.h:PWMFREQUENCY_HZ` but worth restating:

- **Switching losses ~50 % above the 40 kHz baseline**. The MCLV-48V-300W
  board's FET selection assumed 24-40 kHz operation. At 60 kHz with full
  throttle, expect inverter heating you wouldn't see at 40 kHz.
- **300W continuous board rating** includes some margin, but 60 kHz erodes it.
- **For production: fall back to 48 kHz**. PLL + FW still reach ~195k eRPM at
  48 kHz — past the 196k 6-step benchmark target, with comfortable thermal
  margin.

### Startup with prop load — tuning + a critical bug found

After the 200k no-load milestone, prop-load startup uncovered a **PI integrator
windup bug** that wasn't visible in no-load runs.

#### Tuning changes for prop startup

Two parameters needed for prop-load startup (light-load tested 2026-04-25):

| Param | No-load | **Prop** | Reason |
|-------|---------|----------|--------|
| `AN_Q_CURRENT_REF_OPENLOOP` | 4 A (24 mNm) | **8 A (49 mNm)** | overcome prop inertia during OL ramp |
| `AN_OL_RAMP_RATE_RPS2` | 2000 rad/s² (180 ms) | **1000 rad/s² (366 ms)** | slower ramp lets prop accelerate without slipping the synth angle |

The CL slew rate (`AN_CL_VELREF_SLEW_RPS2 = 12000 rad/s²`) and PI gains stay
the same — separating OL ramp from CL slew (split made earlier in the session)
means we can have a slow OL and snappy CL independently.

#### The PI windup bug (subtle but lethal)

**Symptom**: BOARD_PCI fault about 3 seconds into CL operation. Trace shows:
- ALIGN phase: `Vq` railed at +13.16 V (max)
- OL ramp: `Vq` still railed
- CL handoff: motor at 10k eRPM (overshot the 3500 eRPM target)
- 3.30 s onward: `Vd = -13.27 V` steady, `Vq = 0` forced, motor coasting at 36k
- 5.74 s: BOARD_PCI fires after 2+ seconds of railed voltage

**Root cause chain**:
1. ALIGN phase commands `Iq_ref = 8 A` to lock rotor at θ=0. Rotor under prop load
   doesn't snap to θ=0 instantly — current doesn't flow as quickly as PI demands.
2. q-PI integrator winds up rapidly toward +13 V (its `outMax` clamp = `vmax`).
3. OL ramp begins. Rotor starts rotating, BEMF appears, but the q-PI integrator
   doesn't unwind because the inner-loop current error doesn't drive it down
   fast enough.
4. CL handoff inherits the wound-up integrator. Vq is still ~13 V.
5. With Vq railed, the speed PI sees measured speed >> velRef (because high Vq
   accelerated the rotor during OL). Speed PI demands negative Iq.
6. Field weakening sees mod=0.95 and starts pumping `id_ref_fw` negative.
7. d-PI tries to drive Id to FW setpoint, rails Vd at `-vmax` = -13.27 V.
8. Critical: `vq_lim² = vmax² - vd² < 0` → **`vq_lim = 0`** → q-PI output
   forced to zero. No torque current can flow. Motor stuck coasting.
9. After 2+ seconds of zero-current operation with railed voltages, eventually
   a transient triggers BOARD_PCI.

**The fix** (in `an1078_motor.c`, at OL→CL transition):

```c
if (m->changeMode) {
    m->changeMode = false;
    an_pi_preload(&m->pi_spd, AN_Q_CURRENT_REF_OPENLOOP);  /* keep bumpless */
    m->pi_d.integrator = 0.0f;   /* clear stale OL/ALIGN windup */
    m->pi_q.integrator = 0.0f;
    m->id_ref_fw       = 0.0f;
}
```

Plus **gate field weakening on `iq_ref > 1 A`** so it never engages during
deceleration, coasting, or stale-integrator startup:

```c
if (mod_now > FW_TRIGGER && iq_ref > FW_IQ_GATE) {
    /* accumulate negative id_ref_fw */
} else {
    m->id_ref_fw *= FW_DECAY;  /* decay 10× faster: 0.995 not 0.9995 */
}
```

**Lesson**: outer-loop bumpless transfer (`speed PI preload`) is well known.
Inner-loop integrator reset at the same handoff point is just as important
when the outer-loop produces large transient errors that can saturate an
inherited integrator.

#### Verified after fix (2026-04-25, build 0x66834FD9)

- 44 s run, **0 faults**, motor reached 206k eRPM peak
- ALIGN: `Vq = 0.86 V` healthy (not railed)
- OL ramp: `Vq = 0.86 V` stable, `Iq = 7-8 A` flows cleanly
- OL→CL handoff: bumpless, motor at 3700 eRPM with no overshoot
- CL settling at 3500 eRPM (low throttle): perfect, Iq drops to ~0.5 A

#### Other prop-load gotchas worth knowing

- **Bench supply current limit (UNDERVOLT)**: the 10 A bench supply collapses
  (Vbus 24 V → 9 V in microseconds) when the motor pulls a large inrush
  during throttle bumps. Not a firmware bug — the supply is current-limiting.
  Mitigations: slow `AN_CL_VELREF_SLEW_RPS2` for prop testing (e.g. 4000
  instead of 12000), or use a small LiPo as supply.
- **Regen pump-up during decel**: dropping throttle at high speed regens
  energy into Vbus, which rose to 38 V in one trace. No OV clamp on this
  hardware, so subsequent acceleration after regen pulled supply down to
  UNDERVOLT range. Add overvoltage clamp (chop into shunt or back off
  throttle PI) for production.
- Watch for OL→CL handoff stalls — the BEMF gate (`HANDOFF_DWELL_TICKS = 2400`,
  i.e. 40 ms at 60 kHz) may need extension if observer is slow to lock under
  heavier prop load.

### FW currents flow even at no-load

At 200k eRPM with `Id_FW = -8 A` and `Iq = 2 A`:
```
|I| = √(I_d² + I_q²) ≈ 8.2 A peak in steady state
```
During throttle transients, peaks of 12-15 A. Don't run sustained at top
speed — copper losses heat the motor.

### Build hash bug fix

`__DATE__/__TIME__` only update when `gsp_commands.c` itself is preprocessed.
Incremental Make can leave it stale when only `an1078_params.h` changes.
**The fix**: `gsp_commands.c` now `#includes` the AN1078 headers (so Make
rebuilds it on those header changes), AND folds key tunable values into the
hash via XOR. Hash now changes reliably on parameter edits — but `.c`-only
edits to `an1078_motor.c` still won't bump the hash without a clean rebuild.

## Files touched

- `dspic33AKESC/foc/an1078_smc.{c,h}` — PLL field added, atan2 replaced with PLL
- `dspic33AKESC/foc/an1078_motor.{c,h}` — `id_ref_fw` field, FW integrator, separate CL slew rate
- `dspic33AKESC/foc/an1078_params.h` — many tunings
- `dspic33AKESC/garuda_config.h` — `PWMFREQUENCY_HZ` 40→60 (cautionary comment)
- `dspic33AKESC/garuda_foc_params.h` — `MOTOR_MAX_ELEC_RAD_S` 15700→25000 (PLL clamp)
- `dspic33AKESC/gsp/gsp_commands.c` — build hash fix

## Bench data references

CSV logs in `logs/` directory:
- `foc_20260425_205355.csv` — K=1.5e-4 first push to 115k
- `foc_20260425_211153.csv` — 60 kHz pre-PLL, hit 171k chaos wall
- `foc_20260425_212006.csv` — **PLL milestone: 176k clean, 139s, 0 faults**
- `foc_20260425_213237.csv` — **FW milestone: 200,109 eRPM peak**
- `foc_20260425_214204.csv` — final tune: 206,637 eRPM peak with snappy throttle
