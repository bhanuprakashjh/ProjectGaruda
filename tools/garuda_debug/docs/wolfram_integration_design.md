# Wolfram Integration + ML/ZC-Lab/Diagnostics Roadmap (2026-07-04)

Grounded in a code-level audit of the three GUI subsystems (facts + line refs in the
audit transcript). The unifying idea: **stop hand-tuning model constants — fit them
from bench captures, and make every simulator read the fitted set.**

## What the audit established

| Subsystem | State | Core gap |
|---|---|---|
| ML collect (collect_ml, zcml) | Collects 128-sample BEMF windows to JSONL; trains a tiny MLP (18→24→1) predicting ZC position | Model never persisted, never deployed, never validated on-board |
| zcsim (ZC Lab) | Lumped BEMF+RC+spike/ring model, matches bench falling-capture envelope | Constants hardcoded to 2810; parasitics (spike_amp=30, ring_amp=8) hand-tuned |
| spice_engine | Device-level ngspice sandbox | Explicitly "does NOT yet quantitatively reproduce bench" |
| pisim | Exact port of the firmware sector-PI | Constants copy-pasted, drift risk; no plant coupling |
| wizard | Datasheet→profile heuristics calibrated to 2810 | Equilibrium-only; FOC block computed but unused |
| Diagnose tab | 2-tier engine + 4 signal checks, campaign taxonomy encoded | Cannot see: per-sector history, burst captures, param-change log, zcDesyncCount, transients between 10Hz frames |

## Phase W1 — Data bridge (IMPLEMENTED with this doc)

1. **Wolfram run bundle**: GUI console command `wl` (and after any run) exports
   `sessions/run_<ts>.wl.json`: `{info, params(raw+physical), telemetry[...],
   scope[...]}` — telemetry and the last burst capture in ONE aligned file.
2. **`tools/wolfram/garuda.wl`**: `GarudaImport[path]` → Association of Datasets;
   `GarudaKeFit[run]` (duty↔eRPM linear fit → per-motor KSPEED/Ke, replacing the
   hardcoded 235k in analyze.py); `GarudaScopePlot[run]` (BEMF vs threshold vs
   sector overlay); `GarudaRCFit[run]` skeleton (exponential settling → filter τ).

## Phase W2 — Parameter identification (the "calibrated digital twin")

Fit, per motor, from existing capture types (no firmware change needed):
- **Ke / KSPEED**: steady CL points (duty, eRPM, Vbus) — linear fit. Consumers:
  analyze.py phantom floor, wizard no-load model, spindown-floor KV_NUM check.
- **RC filter τ + spike/ring**: burst captures around PWM edges; `NonlinearModelFit`
  of zcsim's parasitic model against real v_sense. Consumers: zcsim defaults become
  *fitted*, per-motor.
- **Demag time vs current**: per-sector miss/reject rate vs Ia from ML JSONL sweeps
  — logistic fit of capture-kill vs current. Consumers: zcDemagBlankPerA seeding.
- **R (and roughly L)**: align-phase Ibus vs sine amplitude (R); duty-step burst
  captures for di/dt (L). Consumers: wizard inputs verified vs measured.
Output: `sessions/motor_cal_<name>.json` — a fitted-constant set that zcsim, pisim,
wizard, and analyze load in preference to their hardcoded values.

## Phase W3 — Model-in-the-loop

- **pisim constants auto-sync**: parse firmware header (or GSP GET_INFO extension)
  instead of copy-paste; CI-check drift.
- **Replay**: drive the approved Wolfram/Modelica plant (M1-M3) with measured duty
  traces from run bundles; compare simulated vs measured eRPM/cap% — the back-EMF
  observer A/B from the PLL campaign runs against *replayed bench data*.
- **ML repurpose**: with physics fits in hand, the MLP's honest role is residual
  anomaly detection (predicted-vs-actual ZC position drift), not primary detection.
  Prereq either way: persist the model (npz) + overlay predictions on Burst Scope.

## Diagnostics tab roadmap (from the gap list)

1. **Attach last burst capture to diagnosis** — auto-run half_period_check + the
   new fits' sanity checks on the most recent capture (data already in memory).
2. **Per-sector counters over time** — record miss_by_sector[6] deltas per snapshot
   into the session; plot rising/falling capture-rate vs time; feeds polarity fit.
3. **Param-change journal** — log every SET_PARAM (t, name, old, new) into the
   session so runs are comparable ("what changed since the good run?").
4. **Decode zcDesyncCount** (+ watchdog fire counters) — needed to distinguish one
   long phantom from twenty transient recoveries. Firmware snapshot may need a field.
5. **Baseline fingerprints** — store a healthy run's cap%-vs-duty curve per motor;
   diagnosis compares against baseline instead of absolute thresholds.

## Suggested order
W1 (done) → diagnostics #1+#3 (cheap, GUI-only) → W2 Ke+RC fits (first real Wolfram
value) → diagnostics #2+#4 (needs firmware touch) → W2 demag/R/L → W3.
