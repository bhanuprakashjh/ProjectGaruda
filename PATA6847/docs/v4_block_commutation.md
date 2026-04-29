# V4 Block Commutation — 238k eRPM Milestone

**Date:** 2026-04-29
**Result:** A2212 1400KV @ 25V Vbus, peak **238k eRPM** (vs 228k prior PWM-only ceiling, +4.4%)
**Status:** Stable across many full-pot ramp/release cycles. Block-comm engages automatically at duty saturation, exits cleanly on pot release without thrashing.

## Concept

Above ~95% commanded duty, PWM chopping at 60 kHz wastes switching headroom we can't use — duty is already clipped near the period cap. **Block commutation** drives the active-phase H-side gate solid-ON for the whole sector via OVRENH/OVRENL/OVRDAT overrides, bypassing the PG[123]DC compare entirely. Equivalent to infinite switching frequency. The L-side stays solid OFF.

ATA6847 uses an internal charge pump (datasheet: "100% PWM Duty Cycle Control"), so 100% high-side drive is sustained indefinitely with no bootstrap-refresh requirement.

## Why This Works for V4

V4 captures rising-edge BEMF only — the falling-sector path is gated to `MIDPOINT_ZC=0`. At 100% drive, there's no PWM switching noise on the floating phase, so the existing ADC midpoint sampling reads cleaner BEMF than it does at 96% PWM. Same capture path, less noise → better lock at high speed.

## Engagement State Machine

State machine lives in `motor/sector_pi.c` `SectorPI_TimeTick()` (1 kHz tick, evaluated inside the speed-window block at 50 Hz).

**Entry conditions** (all must hold):
- `actualAmplitude ≥ 31130` (95% Q15)
- `erpm ≥ 150000`
- `blockCommStableTicks ≥ 10` — eRPM held above 150k for ≥ 200 ms
- `commandEnabled && !v4_spActive`
- `blockExitCooldown == 0`

**Exit conditions** (any one):
- `actualAmplitude < 29490` (90% Q15) — pot released
- `erpm < 130000` — motor unloaded/decel
- `corridorMissStreak > 5` — true lock-loss (5+ consecutive ZC sentinels)

On exit, `blockExitCooldown` is set to 25 (25 × 20 ms = **500 ms re-entry lockout**) to prevent BLK↔PWM thrashing when the pot is jittered near the entry threshold.

## File Map

| Change | File:Line | Description |
|---|---|---|
| Override application | `hal/hal_pwm.c:52-65` | `ApplyPhaseState()` checks `g_blockCommActive` before PWM logic. OVRENH=1, OVRENL=1, OVRDAT=10 → H pin LOW (POL inverts → HS FET ON), L pin LOW (LS FET OFF). |
| Global flag (decl) | `hal/hal_pwm.h:30` | `extern volatile bool g_blockCommActive;` |
| Global flag (def) | `hal/hal_pwm.c:31` | `volatile bool g_blockCommActive = false;` |
| Stability counter | `motor/sector_pi.c:166` | `static uint16_t blockCommStableTicks` — counts speed-windows above 150k eRPM. |
| Miss-streak counter | `motor/sector_pi.c:165` | `static uint8_t corridorMissStreak` — increments on `CAP_SENTINEL`, resets on hit. Used for exit lock-loss check. |
| Cooldown counter | `motor/sector_pi.c:170` | `static uint16_t blockExitCooldown` — 500 ms re-entry lockout after exit. |
| State machine | `motor/sector_pi.c:991-1030` | Entry/exit logic with hysteresis. |
| Telemetry | `gsp/gsp_commands.c:807` | `d[37]` bit 2 reflects `g_blockCommActive`. Bit 0 = SP active, bit 1 = SP requested, bit 2 = block. |
| Honest duty% | `gsp/gsp_commands.c:757-762` | `dutyPct = 100` when `g_blockCommActive` (override path bypasses PG[123]DC, so the compare register doesn't reflect the gate state). |

## Tuning Knobs

All thresholds are at `motor/sector_pi.c:1003-1029`. To disable block-comm entirely without rebuilding the override path, set the entry eRPM threshold above any reachable speed:

```c
// In sector_pi.c TimeTick state machine
if (actualAmplitude >= 31130U
    && erpm >= 99999999UL  // ← unreachable
    ...
```

**To engage at lower speed** (smaller motors, lower Vbus): drop the entry eRPM threshold from 150k. Rule of thumb — the threshold should be above the speed where the motor naturally saturates into the duty clamp; below that, PWM gives the same torque as block at less noise.

**To engage at higher amplitude** (less aggressive): raise the entry Q15 threshold from 31130 (95%) toward 32400 (~99%). Wider hysteresis with the 90% exit reduces flapping.

## Build / Flash

CLI:
```bash
cd PATA6847/garuda_6step_ck.X
make -f Makefile-cli.mk MOTOR_PROFILE=1  # A2212 tunables, tested driving the 2810 bench motor
```

MPLAB X: open `garuda_6step_ck.X`, set `MOTOR_PROFILE` default at `garuda_config.h:17`, Build & Program.

## Test Result Summary (2026-04-29)

`MOTOR_PROFILE=1` (A2212 startup tunables) driving the 2810 1350KV bench motor at 25 V bench supply, 89 s pot-capture run. The startup ramp is sized for A2212 but the 2810 spins up cleanly on it; CL behaviour is governed by the V4 sector-PI which doesn't depend on the profile's startup tunables.

- **14+ BLK engage/disengage cycles** observed across multiple full-pot pushes
- All transitions clean — no thrashing on pot jitter near threshold
- **Peak eRPM: 238,000** (sustained for 5+ seconds)
- Telemetry visibility: `pot_capture.py` shows `BLK` in the SP column, `Duty=100%`
- Vbus regen spikes (24.7V → 25.7-26V) observed on some BLK exits at full speed; recovered via natural decel through PWM

## Known Limitations

1. **Regen spike on exit** — when BLK exits at 235k eRPM, the H gate transitions from solid ON to PWM at ~88-89% effective duty. The 11% off-time engages complementary L-FET (active braking), which dumps stored kinetic energy back into Vbus. Spike is typically 1-1.5V. Within ATA6847 OV margin (28V threshold for production, 40V firmware OV), but precipitates rare desyncs at the highest speeds.
2. **No graceful exit ramp** — exit is binary: BLK gate state to PWM at slewed-down amplitude in a single ISR period. A future improvement would ramp duty from 100% → target over ~50 ms instead of stepping.
3. **Profile-independent thresholds** — entry/exit thresholds are in absolute Q15/eRPM, not motor-profile-tuned. Works because the duty saturation phenomenon is universal, but per-profile tuning would optimize entry timing.

## History

- **228k eRPM (prior ceiling)** at 96.9% effective duty — limited by per-100 MIN_OFF clamp and switching losses near saturation.
- **First implementation (incorrect)**: gated entry on `corridorGoodStreak > 50`. Unreachable because V4 captures rising-edge sectors only — falling sectors return `CAP_SENTINEL` by design → consecutive-hit streak is structurally bounded near 1. Motor held at 225k at 96% duty without ever engaging.
- **Fix 1 (entry reachable)**: replaced streak gate with `blockCommStableTicks ≥ 10` (200 ms above 150k). Block engaged on first run → 237k peak.
- **Bug observed**: thrashing on pot jitter at entry threshold, plus rare desync on exit at full speed (Vbus regen).
- **Fix 2 (cooldown)**: added `blockExitCooldown` (500 ms re-entry lockout) and `corridorMissStreak` for true lock-loss exit. Eliminated thrashing. Peak 238k.

## Related Documents

- `v4_228k_milestone_session.md` — pre-block-comm 228k baseline, all PWM-only optimizations.
- `v4_architecture.md` — V4 sector-PI architecture overview.
