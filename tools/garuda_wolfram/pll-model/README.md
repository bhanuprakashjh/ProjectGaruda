# PLL phase-budget & observer model (Wolfram Layer W)

Analytic layer of the sensing→commutation model. See the spec:
`dspic33AKESC-Simplified/docs/superpowers/specs/2026-07-01-pll-sensing-chain-wolfram-model-design.md`.

## Run
- Activate once: `wolframscript -activate`
- All tests: `./run_tests.sh`
- The analysis + verdict: `wolframscript -file driver.wl run`

## Files
- `motor_params.wl` — per-motor constants (source of truth; emits Modelica record)
- `phase_budget.wl` — arctan(ωτ) lag, comp forms, advance ramp, wall solver
- `calibrate.wl`   — fit τ/margin to bench walls; lever ranking
- `observer_design.wl` — back-EMF observer pole placement
- `driver.wl`/`.nb` — the answer, plots, Manipulate

The decisive output is `oneTauExplains`: if one τ fits the bench walls, the wall is
front-end-latency-limited (answer = lower τ / virtual neutral / more voltage), not
PLL-specific.

**Caveat:** with the current anchors ({150k, 214k}), `oneTauExplains` is
degenerate-by-construction — both anchors sit above the advance-saturation knee
with zero comp gain, so the fit equation has exactly one arctan root and a
single τ always "explains" both points. The current `False` result is a
geometric artifact of this anchor set, not a physical rejection of the
single-tau hypothesis. A non-degenerate test requires including the 80k
pre-comp anchor (planned follow-up).
