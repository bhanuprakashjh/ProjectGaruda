# AN1078 Sensorless FOC — Motor Tuning Guide

Practical recipe for getting AN1078 closed-loop FOC working on a new
PMSM/BLDC motor with the dsPIC33AK + MCLV-48V-300W board.

This guide is written from the bring-up of PRODRONE 2810 (1350 KV, 7
PP, 22 mΩ Rs, 10 µH Ls).  The reference is the Microchip AN1078
"Sensorless FOC of PMSM" application note, ported to float in
`dspic33AKESC/foc/an1078_*`.

---

## Prerequisites

1. **Board working in OL** — confirm hardware basics (PWM gates,
   current sensing polarity, ADC sampling) using a 6-step build first,
   or by manual voltage injection.
2. **Motor parameters measured**:
   - `Rs` (phase-to-neutral resistance, mΩ) — multimeter phase-to-phase
     ÷ 2.
   - `Ls` (phase-to-neutral inductance, µH) — LCR phase-to-phase ÷ 2.
   - `Ke` or `λ_pm` — from KV: `λ = 60 / (√3 · 2π · KV · PP)`
     in V·s/rad electrical.
   - `PP` (pole pairs) — count magnets ÷ 2.
3. **Bench setup**:
   - Bus supply current-limited to ~3 A (initial), increase to 10 A
     after first successful run.
   - Throttle pot wired to ADC0.
   - SW1 wired to start/stop (active-low button).
   - `tools/foc_logger.py` ready to capture telemetry.
4. **Logger build-hash** — every recompile produces a different
   `Build hash: 0x........` on logger connect.  If you see the same
   hash twice, the chip wasn't reprogrammed.

---

## Step 0 — Configure motor parameters

Edit `dspic33AKESC/foc/an1078_params.h`:

```c
#define AN_NOPOLESPAIRS         7         /* count magnets / 2 */
#define AN_MOTOR_RS             0.022f    /* Ω, phase-to-neutral */
#define AN_MOTOR_LS             10e-6f    /* H, phase-to-neutral */
#define AN_MOTOR_LAMBDA         0.000583f /* V·s/rad_electrical */
```

Set:
```c
#define FEATURE_FOC_AN1078      1   /* in garuda_config.h */
#define FEATURE_FOC_V2          0
#define FEATURE_FOC_V3          0
```

Build and verify with logger:
```
GSP proto v3  FW v0.2.0
Build hash: 0x........  ← changes every build
Mode: FOC
```

---

## Step 1 — Verify open-loop operation

Goal: motor must spin reliably in OL before attempting closed loop.

Defaults to start with:
```c
#define AN_END_SPEED_RPM_MECH       500.0f
#define AN_OL_RAMP_RATE_RPS2        500.0f
#define AN_Q_CURRENT_REF_OPENLOOP   4.0f   /* tune to 2810's 12 mN·m holding torque */
#define AN_LOCK_TIME                9600   /* 400 ms total: warmup + soft-start + steady */
#define AN_WARMUP_TICKS             1200   /* 50 ms gates settle, no PI activity */
#define AN_IQ_SOFT_START_TICKS      4800   /* 200 ms 0→Iq linear ramp */
```

Test sequence:
1. Power up board, throttle pot at zero
2. Run logger — should show `state=IDLE`
3. Press SW1 — state goes ARMED → ALIGN
4. After ~400 ms, state goes OL_RAMP and motor begins spinning
5. Motor settles at `AN_END_SPEED_RPM_MECH × electrical/mechanical` RPM
6. **Stays in OL_RAMP indefinitely** (CL handoff disabled until step 3)

If OL fails — symptoms and fixes:

| Symptom | Likely cause | Fix |
|---|---|---|
| PCI fault at SW1 press | Inrush from PWM enable | Lower `AN_Q_CURRENT_REF_OPENLOOP` to 1A, longer warmup |
| Motor doesn't follow OL angle, slips | Cogging > applied torque | Raise `AN_Q_CURRENT_REF_OPENLOOP` (24 mN·m at Kt × Iq must beat cogging) |
| Motor spins backwards | Phase wiring | Swap any two phase leads at motor connector |
| Vbus dips during ramp | Supply current limit | Raise supply, or reduce `AN_OL_RAMP_RATE_RPS2` |

Once OL is stable for 30+ seconds with no faults, **proceed**.

---

## Step 2 — Bootstrap the SMC observer

The SMC observer can lock onto BEMF only when the LPF cutoff is wide
enough to pass the fundamental.  With low-Ke motors, the BEMF signal
is weak, so the LPF can't bootstrap from zero.

Two changes to `an1078_params.h`:

```c
#define AN_SMC_KSLF_SCALE   (3.0f * AN_TS)   /* LPF cutoff above fundamental */
#define AN_SMC_KSLF_MIN     0.05f            /* floor for bootstrap, ~191 Hz cutoff */
```

In `an1078_motor.c`, the SMC's `OmegaFltred` is overridden with the
known OL speed (`startupRamp`) so Kslf computes correctly.  This
already works in the shipped code.

Test: in OL operation, observe these telemetry fields:
```
focFluxAlpha (smc.E2_alpha) ── should oscillate ±λ·ω peak
focFluxBeta  (smc.E2_beta)  ── 90° phase from E2_alpha
focObsConfidence            ── should reach 1.0
```

For 2810 at 366 rad/s elec: `|BEMF| = 0.000583 × 366 = 0.213 V`.
Expect `bemf_a/bemf_b` peaks of 0.2–0.25 V.  Confidence = 1.0.

If observer doesn't lock — **the algorithm itself doesn't suit your
motor**.  Skip to Step 5 (replace observer).

---

## Step 3 — Calibrate `AN_SMC_THETA_OFFSET`

The SMC LPF introduces phase lag at the operating frequency.  AN1078's
default `π/2` (90°) compensates for the BEMF↔rotor 90° geometric
relationship only.  You need to add the LPF group delay too.

**Method**: enable CL handoff with current-PI-only mode (no speed PI).
In `an1078_motor.c`, the CL branch is currently fixed at `iq_ref =
AN_Q_CURRENT_REF_OPENLOOP`.  This stays disabled until observer is
trusted.

Press SW1, motor should enter CL after OL (state → CL).  Read
`focIdMeas` and `focIqMeas`:
```
arctan(Id_meas / Iq_meas) ≈ angle bias between observer and rotor
```

If `Id_meas ≈ 0`, observer is aligned — done.

If `Id_meas > 0`, observer LEADS rotor — **subtract** from offset.
If `Id_meas < 0`, observer LAGS rotor — **add** to offset.

Iterate.  On 2810:

| Offset (rad) | Offset (°) | Id   | Iq   | Bias |
|--------------|-----------|------|------|------|
| 1.57 (π/2)   | 90        | +3.8 | +3.9 | 44°  |
| 0.80         | 46        | +1.4 | +4.1 | 19°  |
| 0.524        | 30        | +0.7 | +4.4 | 9°   |
| **0.349**    | **20**    | **-0.07** | **+4.08** | **0°** ✓ |

Slope is roughly 0.6° bias reduction per 1° offset reduction.

**Important caveat**: this offset is correct **only at the speed you
calibrated at**.  LPF phase lag varies with operating frequency.  At
higher speeds the offset will need adjustment.

---

## Step 4 — One-shot tuning recipe for new motors

The values below are starting points.  Don't run yet.

```c
/* Motor-specific (measured) */
#define AN_NOPOLESPAIRS         <PP>
#define AN_MOTOR_RS             <Rs in Ω>
#define AN_MOTOR_LS             <Ls in H>
#define AN_MOTOR_LAMBDA         <λ_pm>
#define AN_END_SPEED_RPM_MECH   500.0f         /* low-speed bench; raise later */

/* Open-loop drive — torque must exceed cogging */
#define AN_Q_CURRENT_REF_OPENLOOP   <choose to give 2-3× max cogging>
                                                /* For 2810: 4A → 24 mN·m */

#define AN_OL_RAMP_RATE_RPS2        500.0f     /* universal default */
#define AN_LOCK_TIME                9600       /* 400 ms, enough for any motor */
#define AN_WARMUP_TICKS             1200       /* 50 ms */
#define AN_IQ_SOFT_START_TICKS      4800       /* 200 ms */

/* Current PI — pole-cancellation tuning */
/* Choose closed-loop bandwidth ω_bw (rad/s).  1 kHz = 6283 is reasonable. */
#define AN_KP_DQ            (omega_bw_rad_s × Ls)        /* V/A */
#define AN_KI_DQ            (omega_bw_rad_s × Rs)        /* V/A·s */
/* For 2810 at 1 kHz BW: Kp = 6283 × 10e-6 = 0.063, Ki = 6283 × 0.022 = 138 */

/* SMC observer — empirical defaults that worked on low-Ke motor */
#define AN_SMC_KSLIDE               2.5f           /* small enough to stay in linear range */
#define AN_SMC_MAX_LINEAR_ERR       1.0f           /* match operating Iq peak */
#define AN_SMC_KSLF_SCALE           (3.0f * AN_TS) /* LPF above fundamental */
#define AN_SMC_KSLF_MIN             0.05f          /* bootstrap floor */
#define AN_SMC_THETA_OFFSET         1.5708f        /* π/2 — START HERE, calibrate per Step 3 */

/* Handoff & bleed */
#define AN_HANDOFF_DWELL_TICKS      2400           /* 100 ms BEMF stable before CL */
#define AN_THETA_ERROR_BLEED_RAD    (0.005f × π/180)  /* very slow CL angle migration */
```

Procedure:
1. Plug in measured motor params.
2. Build + program. Verify build hash.
3. Run with **CL handoff disabled** (existing code stays in OL).  Confirm
   30+ s of OL operation, no faults.
4. Confirm observer locks: `focObsConfidence → 1.0`, `focFluxAlpha/Beta`
   oscillating at expected magnitude.
5. Re-enable CL handoff.  Observe `focIdMeas`.
6. Iterate `AN_SMC_THETA_OFFSET` until `Id_meas ≈ 0` (slope ~0.6°
   bias-reduction per 1° offset change, see Step 3 table).
7. Once Id ≈ 0 and CL is stable for several minutes, motor is tuned at
   that operating point.

---

## Step 5 — Known limitations of this AN1078 port

What works:
- Stable OL ramp and CL handoff for low-Ke motors after empirical tuning
- Reliable startup with 50 ms warmup + 200 ms soft-start
- Build-hash verification end-to-end (host ↔ chip)

What doesn't work yet:
- **Speed control via throttle** is disabled in CL.  Speed PI feedback
  uses the SMC's `OmegaFltred`, which has 5-10% noise on low-Ke motors;
  combined with observer angle bias, the closed-loop speed regulator
  drove the system unstable in early bring-up.  Re-enable only after
  observer angle is robust at all expected speeds.
- **Theta offset is speed-specific**.  Optimal offset depends on LPF
  phase lag at fundamental frequency.  A motor running at 1000 rad/s
  needs a different offset than 366 rad/s.  Real fix is dynamic phase
  comp (offset = base + k·ω).
- **Observer angle bias varies with current**.  At higher Iq, dt_comp
  errors grow → angle drift.  Acceptable at no-load bench; will need
  re-tuning under load.

When the SMC limits become a problem, replace the observer with:
- TI ESMO-style PLL on dq-frame BEMF (cleaner angle, no per-speed
  tuning) — see `memory/vendor_smo_implementations.md`
- Or NXP-style Luenberger observer (proprietary, no source available).

---

## Reference: AN1078 dispatch architecture

`dspic33AKESC/foc/an1078_*` is a faithful float port of Microchip's
AN1078 SMC + state machine, organised as:

```
an1078_params.h   — all tunables
an1078_smc.h/c    — sliding-mode observer (port of smcpos.c)
an1078_motor.h/c  — state machine + ISR pipeline (port of pmsm.c)
```

Wired into `garuda_service.c` ADC ISR via `FEATURE_FOC_AN1078=1`.
Telemetry shares fields with V3 path.

State machine: `STOPPED → LOCK → OPEN_LOOP → CLOSED_LOOP → STOPPED`.
Triggered by `garudaData.state == ESC_ARMED` (set by main-loop SW1
handler or GSP run command).
