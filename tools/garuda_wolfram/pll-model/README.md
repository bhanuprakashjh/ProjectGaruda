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
