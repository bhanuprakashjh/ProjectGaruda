# Garuda GUI (standalone) — User Manual

This is the **standalone desktop GUI** for the Garuda ESC: a Python application
(PySide6 + pyqtgraph) that runs on Linux, Windows and macOS and talks to the ESC
over USB serial. Its window title is "Garuda Studio — v2"; in this manual it is
called the desktop GUI, to tell it apart from the web GUI.

- Source: this folder (`tools/garuda_debug/`) of the `ProjectGaruda` repo. It is
  developed on branch **`ak512-port`**; `main` carries a synced copy (last
  synced 2026-11-18). If the two differ, `ak512-port` is newer.
- Sister tool: the web GUI, Garuda Studio
  (<https://bhanuprakashjh.github.io/garuda-studio/>), which needs no install
  but has fewer features. See [section 10](#10-desktop-gui-or-web-gui).

Other documents in this folder:
[README.md](README.md) (library, broker, MCP server and analyzers),
[INSTALL_WINDOWS.md](INSTALL_WINDOWS.md),
[WINDOWS_BUILD_EXE.md](WINDOWS_BUILD_EXE.md) (single-file `.exe`),
[CLAUDE.md](CLAUDE.md) (for developers).

---

## 1. Safety first

This tool monitors and re-tunes a **real motor**.

- Bench only. Secure the motor and keep clear of the rotor.
- **Esc stops the motor** from anywhere in the application. Learn it before
  you spin anything.
- On a real board the run is normally started with the board's physical button
  and potentiometer; the GUI watches and tunes. On-screen Start, Stop and
  Throttle controls appear only when connected to the simulator.
- Parameter edits are pushed to the board as soon as you press Enter. A bad
  startup value can trip over-current. The Tune tab colours a cell red when the
  value is out of range or is estimated to exceed the over-current soft limit.
- The MCP server and auto-tune tools described in the README can also spin the
  motor. They are write-protected unless you start them with
  `GARUDA_MCP_ALLOW_WRITE=1`. Supervise the bench whenever that is set.

---

## 2. Install

### Linux / macOS

From the repo root, inside a virtual environment:

```bash
python3 -m venv venv && source venv/bin/activate
pip install -e "tools/garuda_debug[gui]"
```

`-e` keeps the install editable, so a `git pull` updates the tool with no
reinstall. On Linux your user needs permission to open the serial device
(commonly membership of the `dialout` group).

For a clickable desktop launcher on Linux, run
`tools/garuda_debug/scripts/install_desktop_entry.sh`. It expects the virtual
environment at `<repo root>/venv`.

### Windows

Use native Windows Python or `uv` — **not WSL**, which cannot see COM ports.
[INSTALL_WINDOWS.md](INSTALL_WINDOWS.md) has the two-command `uv` install.

To install the development version straight from GitHub, name the branch in
the URL. Without `@ak512-port`, the installer fetches `main`, which holds the
last synced copy:

```powershell
uv tool install --python 3.11 "garuda-debug[gui] @ git+https://github.com/bhanuprakashjh/ProjectGaruda@ak512-port#subdirectory=tools/garuda_debug"
```

For a machine with no Python at all, build the single-file `garuda-gui.exe`
once by following [WINDOWS_BUILD_EXE.md](WINDOWS_BUILD_EXE.md) and copy it over.

### Optional extras

| Extra | Adds |
|---|---|
| `[gui]` | The desktop GUI (PySide6, pyqtgraph, numpy). |
| `[ai]` | "Diagnose" deepening with Claude. Also needs `ANTHROPIC_API_KEY` set. |
| `[mcp]` | The `garuda-mcp` server, so an MCP client can drive the board. |

---

## 3. Start it

```bash
garuda-gui                       # auto-starts the broker, auto-detects the board
garuda-gui --port /dev/ttyACM0   # or COM5 on Windows: force a port
garuda-gui --sim                 # no board: run against the simulated motor
garuda-gui --sim vex             # simulated motor, a different motor model
```

Then:

1. Pick the port in the **Port** box (the ⟳ button rescans) and press
   **Connect**. Ports held by another program are tagged `[BUSY]`.
2. The label on the right shows the board identity: firmware version, board,
   motor profile, pole pairs.
3. Telemetry starts flowing into the Monitor tab and the console.
4. Start the motor with the board's button and potentiometer (or the green
   simulator bar when using `--sim`).

Only one program can hold a serial port. Close MPLAB Data Visualizer and any
terminal first. The GUI talks to the board through a small background
**broker** so that other Garuda tools can share the same board at the same
time; the GUI starts it for you.

`--sim` needs the simulator built once (`bash tools/garuda_sil/build.sh`).

---

## 4. Screen layout

- **Top bar** — Port, rescan, **Connect**, **● Record**, **🔍 Diagnose**,
  **🤖 Copy report**, and the board identity.
- **Simulator bar** (green, only with `--sim`) — Start motor, Stop, Throttle.
- **Tabs** — Monitor, Tune, Wizard, Burst Scope, ZC Lab.
- **Console** (bottom dock) — a telemetry mirror, a command line and a Clear
  button. See [section 6](#6-console-commands).

---

## 5. The tabs

### 📈 Monitor

- **State bar** — the ESC state machine (IDLE → ARMED → ALIGN → OL_RAMP →
  MORPH → CL), with the current state highlighted and faults shown in red.
- **Readouts** — eRPM, duty, bus voltage, bus current, throttle, peak phase
  current, hardware zero-cross rejection rate, CPU load.
- **Scope** — a normalised multi-channel plot. Tick the channels you want; each
  can be autoscaled, and there is a time-window selector, **Freeze** and
  autoscale-all. 6-step channels include eRPM, phase and bus current, rejection
  rate, bus voltage, BEMF, zero-cross threshold, duty, throttle and CPU load.
  On an AN1078 FOC build the FOC channels (Iq, Id, modulation index, observer
  confidence, third-phase current, real bus current, temperature) switch on
  automatically and the 6-step-only ones switch off.
- **Sector bars** — per commutation sector, how many commutations came from a
  real zero-cross and how many were guessed by the timer. A sector that is
  mostly guessed is where detection is failing.
- **Rotor view and operating map** — the capture pattern around one electrical
  revolution, and a scatter of peak phase current against eRPM.

Every run is **recorded automatically** to `sessions/` in the folder you
launched from; you do not need to press Record. See [section 7](#7-files-it-writes).

### 🎛 Tune

The full parameter table read from the board: name, raw value, the same value
in physical units, min, max and group. A filter box narrows the list.

- Click a **Value** cell, type, press Enter. The value is written to the board
  immediately.
- **Red** means out of range, or estimated to exceed the over-current soft
  limit.
- Edits live in RAM. Type `save` in the console to write them to flash (motor
  stopped). On firmware built with `FORCE_DEFAULTS=1`, edits never persist; use
  **⤓ Export as C profile** and paste the block into the firmware.
- **↻ Reload from board** re-reads everything.

### 🧙 Wizard

Enter a motor's datasheet numbers — name, KV, phase-to-phase resistance, pole
count, weight, maximum current, bus voltage, optionally inductance, and the
profile slot — and press **⚙ Generate profile**. The wizard produces a
complete, current-safe parameter profile as C code for the firmware files, with
startup values chosen so they will not trip over-current. **Save .c** writes it
to a file. Nothing is written to the board.

### 🔬 Burst Scope

Telemetry arrives at tens of hertz and aliases the real waveform. The burst
scope records 128 samples at 24 kHz (about 5.3 ms) inside the MCU and reads
them back, so you see the true BEMF against the zero-cross threshold, plus
phase current and sector.

- **Trigger**: Manual, On fault (freezes the buffer at a desync), On state
  change (catches the MORPH → CL handoff), or Threshold (pick the channel,
  edge and level).
- **% pre** sets how much of the window is before the trigger.
- **Save CSV** stores the capture.

### 🧪 ZC Lab

A model of sensorless zero-cross detection, usable with no board.

- Pick a motor and move the speed slider to see how much of each sector's
  zero-cross is captured as speed rises.
- **⚡ Deep-dive** runs the switching-level waveform. The SPICE engine option is
  experimental: it runs, but does not yet reproduce the masking effect.
- **📐 Falling-ZC sweep** opens the analytic study of the falling sector: where
  the zero-cross truly is against where the sampler measures it, across eRPM,
  with a written verdict underneath.
- **⏺ Collect ML dataset** and **🧠 Train / Eval** run the zero-cross position
  estimator experiment on burst-scope captures.

---

## 6. Console commands

Type in the box at the bottom and press Enter.

| Command | What it does |
|---|---|
| `help` or `?` | List the commands. |
| `clear` | Clear the console (same as the Clear button). |
| `pause` / `resume` | Stop or restart the telemetry mirror. |
| `mark <text>` | Print a marker line. |
| `params` | Print every parameter; out-of-range values are flagged. |
| `get <param>` | Print one parameter. |
| `set <param> <int>` | Write one parameter (raw integer). |
| `reload` | Re-read the parameters from the board. |
| `save` | Write the parameter table to flash. Motor must be stopped. |
| `defaults` | Restore factory defaults into RAM (use `save` to keep them). |
| `export` | Export the live parameters as a C profile block. |
| `record` | Toggle manual recording. |
| `diagnose` | Run the diagnosis (same as the 🔍 button). |
| `report` | Build the debug report and copy it to the clipboard (same as 🤖). |
| `wl` | Export the run as a bundle for the Wolfram analysis tools. |
| `sweep [start stop step dwell]` | 6-step only: step the BEMF sample point through a range, dwell at each, and report which phases cross the threshold. The motor must stay in closed loop. `sweep stop` aborts. |
| `raw` | Dump the raw current-sense chain values. |
| `ata` | GarudaESE only: read the ATA6847 gate-driver status and fault registers. |

### Reading the live line

While the motor runs, the console prints one line per sample with state,
throttle, duty, eRPM, bus voltage, bus and phase current, rejection rate and
fault. On the 6-step bring-up firmware it also shows the zero-cross capture
split:

- `cR` — where in the sector the **rising** zero-cross landed, in per-mille of
  the sector period (hardware comparator).
- `cF` — the same for the **falling** zero-cross (software sample, one PWM
  period of resolution).
- `d` — `cR − cF`. A large or growing gap means the two polarities are giving
  the loop inconsistent timing: the detector is the limit.
- `nc` — control-loop events that fired with no capture at all.

These show `--` when the firmware is not sending them.

---

## 7. Files it writes

Everything goes under `sessions/` in the folder the GUI was started from.

```
sessions/<name>/
  meta.json        firmware, build hash, board, profile, pole pairs, features, host
  params.json      every parameter (value, min, max) at capture time
  telemetry.csv    the time series
  faults.json      fault transitions
```

A session folder is self-describing: zip it and send it to someone else for
offline diagnosis. Debug reports, scope captures and exported profiles are
saved alongside.

---

## 8. Diagnose and the debug report

- **🔍 Diagnose** runs the built-in rule engine over the recent run and lists
  findings: phantom speed readings, half-period lock, current-sense offset,
  missed zero-crosses clustered on one polarity, lock stability, circulating
  current. With the `[ai]` extra and `ANTHROPIC_API_KEY` set, it also asks
  Claude about cases the rules do not cover.
- **🤖 Copy report** builds a compact markdown report — firmware, parameters in
  real units, run statistics, faults and the console tail — copies it to the
  clipboard and saves it in `sessions/`. Paste it into a chat or a ticket.

---

## 9. Troubleshooting

| Symptom | What to check |
|---|---|
| No ports listed | Another USB cable or port; a USB-serial driver if the board uses a CH340/CP210x adapter. On Windows, never run under WSL. |
| Port tagged `[BUSY]` | Another program holds it: MPLAB X / Data Visualizer, a terminal, or a leftover `garuda-broker` / `python` process. |
| Connects, no telemetry | The board may expose two serial ports; pick the other. `garuda-gsp-info` prints the board identity and tells you whether the link itself works. |
| Linux: permission denied on the port | Add your user to the `dialout` group and log in again. |
| Bus voltage reads low or high | The scaling is chosen from the board the firmware reports (MCLV-48V-300W or GarudaESE). An unknown board falls back to MCLV scaling. |
| Temperature looks wrong (GarudaESE) | The thermistor constant is not yet bench-confirmed; treat the reading as approximate. |
| `save` fails or reports busy | The motor must be stopped. Retry once it is idle. |
| `sweep` refuses to run | It is 6-step only and needs firmware that exposes the BEMF sample-point parameter. |
| Simulator will not start | Build it first (`bash tools/garuda_sil/build.sh`), and make sure a real broker is not already running — they use the same local port. |

---

## 10. Desktop GUI or web GUI?

| | Desktop GUI (this) | Web GUI (Garuda Studio) |
|---|---|---|
| Install | Python or the `.exe` | None — open a link |
| Browser | Not needed | Chrome or Edge only |
| Auto-recorded sessions | Yes | No (CSV on demand) |
| Share the board with other tools | Yes (broker) | No |
| Simulator | Yes | No |
| SPICE deep-dive, real-capture ML | Yes | No |
| Claude-assisted diagnosis, debug report | Yes | No |
| Rising/falling capture readout, firmware 1.x profile names | Yes | Not yet |

Use the desktop GUI for bench debugging. Use the web GUI to hand someone a link.
