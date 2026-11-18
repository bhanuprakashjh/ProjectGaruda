# CLAUDE.md — garuda-debug (standalone Python GUI + GSP library)

Orientation for AI assistants and new developers working in
`tools/garuda_debug/`. Read it before changing code. The user-facing
description is in [USER_MANUAL.md](USER_MANUAL.md); the library, broker, MCP
server and analyzers are described in [README.md](README.md).

## What this is

The standalone desktop GUI for the Garuda sensorless ESC firmware (dsPIC33AK),
plus the shared serial-protocol library everything else is built on.

- `garuda_gsp/` — the GSP (Garuda Serial Protocol) library. Pure Python,
  depends only on `pyserial`. **This is the reference implementation of the
  protocol on the PC side.**
- `garuda_gui/` — the GUI (PySide6 + pyqtgraph + numpy).
- The web GUI, Garuda Studio, is a TypeScript port of this code and lives in a
  separate repo (`garuda-studio`). Changes here usually need mirroring there.

## Which branch

**`ak512-port` is the current line** for this tool. `main` (the GitHub default)
carries an older copy. Work on `ak512-port` unless told otherwise, and do not
assume a file on `main` is up to date.

The repository is a firmware monorepo with several worktrees and a lot of
untracked reference material at the top level. **Stage files by explicit path**
(`git add tools/garuda_debug/...`), never `git add -A` or `git add .`.

## Commands

Run from the repo root, with the repo's virtual environment active (`venv/` at
the repo root on the bench machine):

```bash
pip install -e "tools/garuda_debug[gui]"     # editable install
garuda-gui                                   # GUI; auto-starts the broker
garuda-gui --port /dev/ttyACM0
garuda-gui --sim [motor]                     # against the software motor twin
garuda-gsp-info                              # identity + link sanity check
garuda-gsp-record --name <run>               # record a session bundle
garuda-broker                                # own the port, share it over TCP
garuda-mcp                                   # MCP server (read-only by default)
```

Entry points are declared in `pyproject.toml` under `[project.scripts]`.

## Verifying a change

There is **no test suite**. Do at least this before committing:

```bash
cd tools/garuda_debug
python -m py_compile garuda_gsp/*.py garuda_gui/*.py

# Headless GUI smoke test: builds the whole window with no board attached.
QT_QPA_PLATFORM=offscreen python - <<'EOF'
import sys
from garuda_gui import app as A
qa = A.QtWidgets.QApplication(sys.argv)
w = A.MainWindow(None)            # None = no port; nothing is opened
print([w.tabs.tabText(i) for i in range(w.tabs.count())])
EOF
```

`MainWindow(None)` constructs every tab without touching a serial port, so it
catches import errors, missing attributes and broken layout code. From there
you can call a handler directly (for example `w._lab_falling_sweep()`) to
exercise a code path. Anything that changes decoding or motor behaviour still
needs a real board or `--sim`; say so plainly if you could not run it.

## Layout

```
garuda_gsp/
  protocol.py     command ids, state/fault/profile/param name tables, feature
                  bits, ADC scaling, board selection (set_board), NTC maths
  framing.py      packet build/parse + CRC16
  decode.py       GET_INFO and GET_SNAPSHOT decoders — by payload length
  client.py       GspClient: direct serial client, port auto-detect
  broker.py       single-owner serial broker + BrokerClient (TCP 127.0.0.1:47800)
  session.py      session bundle writer/loader (sessions/<name>/…)
  params_meta.py  units, descriptions, raw→physical conversion per parameter
  analyze.py      rule-based analyzers over a recorded run (no dependencies)
  autotune.py     guarded single-parameter sweep with a heartbeat dead-man
  garuda_mcp.py   MCP server exposing the board and analyzers as tools
garuda_gui/
  app.py          the whole GUI: SerialWorker thread, widgets, MainWindow,
                  the five tab builders, console commands, main()
  diagnose.py     rule engine + optional Claude deepening
  wizard.py       motor-profile wizard physics + C code generation
  zcsim.py        zero-cross detection model (lumped/analytic)
  zcsim_falling.py  falling-sector analytic study (falling_walk)
  zcml.py         small numpy MLP for zero-cross position estimation
  pisim.py, spice_engine.py   PI-loop model; experimental SPICE engine
gui_launcher.py   PyInstaller entry point (GUI, or broker with --broker)
garuda-gui.spec   PyInstaller spec
scripts/          gsp_info, gsp_record, Linux desktop-entry installer
```

`app.py` is one large file (~2,600 lines) by design so far. Find your way with
the method names: `_build_monitor_tab`, `_build_tune_tab`, `_build_wizard_tab`,
`_build_scope_tab`, `_build_sim_tab` (ZC Lab), `on_snapshot` (every telemetry
sample lands here), `_on_command` (console commands). To add a console command:
add a branch in `_on_command` and add the word to the `help` string.

## How the link works

- `SerialWorker` (a `QThread`) owns the connection. The GUI thread never touches
  the port; it calls `worker.submit(action, **kw)` and receives Qt signals
  (`snapshot`, `param_written`, `scope_ready`, `failed`, `stopped`).
- `connect_auto()` returns a `BrokerClient` if a broker is running, otherwise a
  direct `GspClient`. The GUI starts a broker itself (`ensure_broker`). The
  broker polls telemetry at 30 Hz, caches the latest snapshot and serialises
  commands from all clients, which is how the GUI, the MCP server and analyzers
  share one board.
- The broker speaks line-delimited JSON on `127.0.0.1:47800`. It is local-only
  and unauthenticated — a bench tool. Do not bind it to another interface.
- Frame format: `[0x02][LEN = 1 + payload][CMD][payload…][CRC_hi][CRC_lo]`,
  CRC16-CCITT (poly 0x1021, init 0xFFFF) over LEN, CMD and payload.
- Port auto-detect probes each port with `GET_INFO` and takes the one that
  answers, which is how the debugger's own COM port gets skipped.

## Rules that matter

1. **`protocol.py` mirrors the firmware headers** (`gsp/gsp_commands.h`,
   `gsp/gsp_params.h`). When the firmware adds a command or parameter, update
   `protocol.py`; do not scatter ids elsewhere.
2. **Decode by payload length, never by assumption.** INFO has been 20, 24 and
   26 bytes; the snapshot ranges from 68 bytes (the Simplified build, which
   ships real units) up to 254. Every new field is guarded by a length check and
   defaults when absent, so old and new firmware both decode without crashing.
3. **Snapshot fields are sometimes reused — check the firmware before trusting a
   name.** In the 6-step build, `gsp_snapshot.c` reuses the three speed-PI slots
   for hardware zero-cross diagnostics, exposed as `cap_rise_pm`,
   `cap_fall_pm` and `pi_nocap`. They are valid **only** when `spi_en == 0xDD`
   (the bring-up marker). `app.py` gates the display on that marker; keep the
   gate, or stale bytes get plotted as measurements.
4. **Name tables depend on firmware version.** Motor profiles were renumbered
   from zero in firmware 1.x (`garuda-ak512-6step`): `decode_info` picks
   `PROFILE_NAMES_V1` when the firmware major version is ≥ 1 and
   `PROFILE_NAMES` otherwise. Follow that pattern for any future renumbering.
5. **Scaling is per board.** `protocol.set_board()` switches bus-voltage and
   current scaling between the MCLV-48V-300W board (divider 23.2, 93 counts/A)
   and GarudaESE (divider 13.0, 90.3 counts/A), chosen on connect from the
   INFO board id. A new board means a new branch there, not a new constant in
   the GUI. `params_meta.py` has its own copy of the MCLV constants; keep the
   two in step.
6. **Bus current has several views.** The instantaneous valley sample is
   unreliable (phantom negative current at idle). Prefer the windowed or
   firmware-averaged value for display and for any safety logic; auto-tune uses
   bus current, not phase current, for its limits.
7. **This drives a real motor.**
   - `Esc` must always send STOP (application-wide shortcut in `_build`).
   - On-screen start/stop/throttle is shown only for the simulator. Do not add
     motor-start controls for a real board without the owner asking.
   - MCP write tools (`set_param`, `save_config`, `motor`, `autotune`) refuse
     unless `GARUDA_MCP_ALLOW_WRITE=1`. Never weaken that gate, and never set
     it yourself to "just test something".
   - Auto-tune only touches `SAFE_PARAMS`, never over-current, voltage or
     speed limits. Keep it that way.
8. **Never invent numbers.** Constants here are bench measurements with dates in
   the comments. If you do not have a measured value, leave a clearly marked
   placeholder and say so; do not estimate. `NTC_BETA` is an example of a value
   already flagged as unconfirmed.
9. **`save` is flash-wearing and state-dependent.** It only works with the
   motor stopped, and on `FORCE_DEFAULTS=1` firmware parameter edits are
   RAM-only. Do not automate repeated saves.

## Packaging constraints

- The frozen `.exe` plays two roles: `garuda-gui.exe` is the GUI and
  `garuda-gui.exe --broker` is the broker (there is no `python -m` inside a
  frozen bundle). `gui_launcher.py` handles this. **Do not rename the exe** and
  do not change how the GUI spawns the broker without updating the launcher.
- A new module imported dynamically may need adding to `garuda-gui.spec`.
- `pyproject.toml` lists the packages explicitly (`garuda_gsp`, `scripts`,
  `garuda_gui`). A new package directory must be added there or it will be
  missing from installs.
- `requires-python` is `>=3.8`. Avoid syntax newer than that in `garuda_gsp/`,
  which is meant to be importable everywhere.

## Do not commit

`sessions/`, virtual environments, `__pycache__`, `*.egg-info` and
`*.gspbundle` are ignored by `tools/garuda_debug/.gitignore`. Bench captures,
plots and scratch scripts at the repo top level are not part of this tool.

Note the repo-root `.gitignore` ignores `*_GUIDE.md`, which is why the manual
is named `USER_MANUAL.md`.

## Keeping the web GUI in step

Garuda Studio ports these files; when you change one, the counterpart probably
needs the same change:

| Here | Garuda Studio |
|---|---|
| `garuda_gsp/protocol.py` | `src/protocol/gsp.ts`, `src/protocol/types.ts` |
| `garuda_gsp/decode.py` | `src/protocol/decode.ts` |
| `garuda_gui/zcsim.py`, `zcsim_falling.py` | `src/sim/zcsim.ts`, `src/sim/fallingZc.ts` |
| `garuda_gui/zcml.py` | `src/sim/zcml.ts` |
| `garuda_gui/wizard.py` | `src/sim/wizard.ts` |
| `garuda_gui/diagnose.py` (local rules only) | `src/sim/diagnose.ts` |
| console commands in `app.py` | `src/components/ConsolePanel.tsx` |

Not yet mirrored in Studio as of 2026-11-18: the `cap_rise_pm` / `cap_fall_pm`
/ `pi_nocap` readout, `PROFILE_NAMES_V1`, and per-board bus-voltage scaling
(Studio hard-codes a 19.8 divider for non-Simplified firmware).

## Loose ends elsewhere in the repo

- Local branch `gsp-hwzc-diag-0x18` has an unmerged client for a
  `GET_HWZC_DIAG` command (0x18) and a per-sector bias probe script. It is not
  in `ak512-port`; check whether the firmware you are targeting implements that
  command before porting it.
- The `pll-wolfram-model` worktree held an older copy of this tool. Its
  falling-ZC sweep was ported here on 2026-11-18; its bus-current-average
  changes were not, because `ak512-port` already decodes that field its own
  way.
