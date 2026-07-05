# Softneutral A/B bench card (spec §8)

Build A (baseline): `GARUDA_ZC_SOFTNEUTRAL_SEL` commented out (default) — current
comparator engine, byte-identical to the proven baseline.
Build B: uncomment `#define GARUDA_ZC_SOFTNEUTRAL_SEL 1` at the TOP of
`garuda_config.h` (line ~46, above the feature flags — it must precede the
falling-SW block). One line; revert = re-comment.

1. **Card 1 — lane rate check (optional, zero risk):** on the Build A config set
   `FEATURE_ZC_FE_SAMPLER 1` (in the ZC front-end block), flash, connect, motor
   ARMED at pot 0 or hand-spin: `get dbgFeSamples` > 0 proves the 400 kHz lane.
   Do NOT run closed-loop sweeps on this combined build (the legacy comparator
   lane loses its channels to the sampler). Revert the define after.
2. **Build B, flash. Starts:** 10× from pot min. Expect: ≥ Build A (clean).
3. **Slow sweep** 5→30%→5 (the standard protocol). Auto-CSV. Judge vs the 07-05
   baseline sessions (`gui_auto_20260705_142351` / `_144058`):
   - `spi_target` / `spi_error`: BOTH polarities mid-sector (no 150‰ wall, no
     250‰ floor-hugging)
   - form% at 20–30% duty: ≥ baseline + 10 points (baseline 40–50%)
   - `ibus_win_A`: no negative-dominant medians at ≤30% duty
   - `get dbgFeSamples` at ~25k: ≥ 8 (expect ~40); note `get dbgFeVoteResets`
   - CPU load within +20‰ of Build A idle
   - zero DESYNC/faults across 3 sweeps
4. **Ceiling probe:** pot to max, no prop. Record top eRPM + failure mode
   (governed vs desync) vs the ~120k wall.
5. Any regression: re-comment the define, reflash Build A. Report the auto-saved
   session timestamps.

Notes:
- First boot of any new build: `params=factory` (the param-table schema grew with
  the two new debug params — saved user params intentionally invalidate).
- `dbgFeSamples` = fast-lane samples in the last sector (latched each
  commutation); `dbgFeVoteResets` = cumulative wrong-sample vote resets since
  HWZC enable (glitch exposure metric; large values with clean running are
  fine — it is the DETECTOR's noise diet, not an error count).
