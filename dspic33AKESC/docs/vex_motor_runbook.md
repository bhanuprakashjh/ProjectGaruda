# VEX 4000KV Micro Motor — Bring-up Runbook

For the VEX 14mm motor (4000KV, 6 pole pairs, 0.44 Ω line-line, ~9.2 µH/ph,
7.4 V rated / 10 V max, ~14 A stall). Written 2026-06-12 against main
`593bf1a`. Companion doc: `startup_methods.md` (method selection + tuning).

## Why this motor is hard (read this first)

BEMF at the phase divider is ~**3 ADC counts at 3,000 eRPM** — below the
comparator deadband (4-8 counts). Any startup that hands off or listens
below ~6k eRPM is reading noise ("fiction lock": telemetry shows a healthy
eRPM, motor isn't spinning). Everything in this runbook exists to keep the
firmware deaf until the rotor is fast enough to be audible:

| Profile 6 value | Meaning |
|---|---|
| `rampTargetErpm = 12000` | BEMF ≈ 12 counts — first truly reliable speed |
| `hwzcCrossoverErpm = 6000` | HWZC capture floor; **PLL startup auto-raises its capture floor to this** |
| `alignDutyPct = 25` | stiff align — the micro rotor needs the hold |
| UV/OC chain | scaled for 7.4-10 V / micro-motor currents |

## Build (one-time)

1. Pull main (needs ≥ `593bf1a`: conversion fix + decoupled startups +
   profile-derived floors).
2. `garuda_config.h`:
   ```c
   #define MOTOR_PROFILE        6     // line ~294 — VEX profile
   #define FEATURE_PLL_STARTUP  1     // line ~117 — RECOMMENDED for this motor
   #define FEATURE_AM32_STARTUP 0     // line ~133 — mutually exclusive with PLL
   ```
   Leave everything else at defaults. (`FEATURE_SINE_STARTUP` may stay 1 —
   PLL uses the sine align if present, classic align if not; both verified.)
3. **Clean build** — delete `build/` and `dist/` (MPLAB: Clean and Build).
   Stale `.o` files have burned us repeatedly.
4. Why PLL and not the AM32 default: twin study (`tools/garuda_sil/sil/
   experiment_vex.py`, results below) — AM32's derived seed (8k) kicks
   harder than a loaded micro rotor can follow → OC; PLL starts the
   schedule at 300 eRPM and discards all captures below 6k, which is
   exactly the phantom-rejection this motor needs.

## Bench procedure

1. **Bus at 10 V max** (7.4 V nominal is fine). Current-limited supply
   strongly recommended for the first runs (≈5 A limit).
2. Connect the GUI (`tools/garuda_debug`) — it auto-records every run.
3. Arm with **throttle at zero** (arming requires it), then start.
4. Expected good run, PLL startup:
   ```
   ARMED (~0.5s) → ALIGN (~0.65s, stiff 25% hold, motor clicks)
   → CL at eRPM=300 → fast blind ramp → sync ABOVE 6,000
   → physical swing-up → idle (twin predicts roughly 25-30k eRPM at the
     duty floor on 10 V — this motor idles FAST; that is normal physics:
     4000KV × low duty floor)
   ```
5. Run it **several times** (5+). Single-run success on a micro motor
   means little.

## Outcome decision table (match your telemetry to a row)

| You see | Diagnosis | Action |
|---|---|---|
| Trace as above, idle ~25-30k, fw/rotor consistent | WORKING | enjoy; sweep throttle gently |
| eRPM climbs to 10,000 and **holds forever**, motor stationary/buzzing | Blind-hold: rotor never caught the schedule (load inertia too high for MIN_DUTY torque) | drop throttle (benign, no fault). Reduce load; or raise `PLL_START_ACCEL_ERPM_PER_S` DOWN is *not* the fix — the fix is engage-duty (ask for the boost knob / raise `alignDutyPct` further) |
| Plausible eRPM on screen, motor **not spinning**, no hold at exactly 10k | Fiction lock — captures below the real BEMF floor got through | verify `hwzcCrossoverErpm` actually reads 6000 (`get_param`); raise it live: `set_param hwzcCrossoverErpm 8000` and retry |
| OC fault during spin-up | Pull-in current exceeded the (deliberately tight) micro OC chain | confirm rotor is free; if loaded, same as blind-hold row; as a *test only*, OC thresholds can be raised via `set_param` |
| IDLE→CL restart loops every ~2s | no-capture watchdog cycling — usually wiring/sense, not tuning | check phase order + BEMF divider connections |

## Live tuning (no reflash — GSP params via GUI/MCP)

These take effect on the *next start*; `save_config` persists to EEPROM.

| Param | Default | When to touch |
|---|---|---|
| `hwzcCrossoverErpm` | 6000 | raise to 7-8k if fiction-y behavior persists (also raises the PLL floor — verified live on the bench 2026-06-12) |
| `rampTargetErpm` | 12000 | only relevant to sine/classic startups and the AM32 seed |
| OC thresholds | profile | only with a current-limited supply, only diagnostically |

Compile-time knobs (reflash): `PLL_START_ACCEL_ERPM_PER_S` (32000 default —
fine for the bare rotor; halve if the ramp audibly outruns the motor),
`PLL_START_TARGET_ERPM` (10000; raising to ~15000 gives the gate more
above-floor time on this motor), `PLL_START_SYNC_CAPS` (6).

## What the twin predicts (calibration caveats below)

From `experiment_vex.py` (4 rotor angles × idle/mid throttle each):

| Load | PLL | AM32 | Note |
|---|---|---|---|
| Bare rotor (J≈2e-7) | **4/4** | 4/4 | idle ~29k, throttle ~38k, fw/rotor 1.00 |
| ~2.5× inertia | **4/4** | 0/4 (OC) | this is why PLL is the recommendation |
| ~5× (geared) | 0/4 benign hold | 0/4 OC | needs the engage-duty boost (future knob) — report if you hit this |

Caveats: the VEX plant is **uncalibrated** (inertia/drag estimated; sense
chain modeled as the MCLV board). Treat the table as a failure-mode map,
not a guarantee. Your first sessions are the calibration data — the GUI
auto-records; send the `sessions/` folder back with notes.

## Windows serial-port troubleshooting (GUI can't find/open the board)

Windows COM ports are **exclusive** — only one program can hold one. The GUI's
connect error lists every port it saw; a port tagged `[BUSY]` is held by
another program. In order of likelihood:

1. **MPLAB X / Data Visualizer is still attached** to the PKoB COM after
   flashing. Close the Data Visualizer tab (or MPLAB entirely), click Rescan.
2. **A terminal app** (PuTTY/TeraTerm/Arduino monitor) is connected. Close it.
3. **A stale broker** from a previous GUI session that was killed via Task
   Manager. Check Task Manager for a leftover `python` running
   `garuda_gsp.broker` and end it (or just reboot).
4. **No ports listed at all**:
   - using the PKoB USB? Win10+ has the CDC driver inbox — try another cable
     (charge-only cables are real) and another USB port.
   - using an external CH340/CP210x UART dongle? Install its driver.
   - running our tools under **WSL**? WSL2 cannot see COM ports — run the GUI
     with native Windows Python, or attach the device with `usbipd-win`.
5. **Two COM ports, picks the wrong one**: normal — the PKoB debug COM and the
   data UART look identical. 'Auto-detect' probes each with GET_INFO and picks
   whichever answers; if you select manually, the board is the one that
   connects.

## If it still won't start

Capture one failing run in the GUI and report: the session folder, which
decision-table row it matched, bus voltage, and whether the motor was
loaded. The startup telemetry (state trace + eRPM + rej%) localizes the
failure to align/catch/floor/sync in one read.
