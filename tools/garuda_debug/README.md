# garuda-debug

Cross-platform debug tooling for the Garuda ESC, built on one shared serial-protocol
library so every tool (and the upcoming GUI) speaks GSP identically — no more
"works on my machine" or a tool breaking when the firmware wire format changes.

Contents: the GSP protocol library, the **live debug GUI** (PySide6 +
pyqtgraph: telemetry, scope, ZC lab, params, auto-recorded sessions), the
serial **broker** (GUI + MCP + analyzers share one board), the **MCP server**,
and rule-based analyzers.

## Install (Linux / Windows / macOS)

```bash
# from the repo root; -e keeps it editable while we iterate
pip install -e "tools/garuda_debug[gui]"
#   protocol library only (no GUI):  pip install -e tools/garuda_debug
```

### Windows notes

- Use **native Windows Python** (3.9+ from python.org) — NOT WSL: WSL2 cannot
  see COM ports.
- In PowerShell: `py -m venv venv ; .\venv\Scripts\activate ;
  pip install -e "tools\garuda_debug[gui]"`.
- COM ports are exclusive on Windows: close MPLAB's Data Visualizer / any
  terminal before connecting. The GUI tags held ports as `[BUSY]` in the
  connect error and the port list.

## Run the GUI

```bash
garuda-gui          # auto-starts the broker, auto-detects the board port
```

Every run is auto-recorded to `sessions/` as a shareable bundle.

## Use

```bash
garuda-gsp-info                 # identity + sanity check (auto-detects the port)
garuda-gsp-info --list-ports
garuda-gsp-record --name cobra_try1     # record a shareable bundle, Ctrl+C to stop
```

As a library:

```python
from garuda_gsp import GspClient, Session
with GspClient() as c:            # auto-detect port, version-negotiated
    print(c.get_info())           # handles INFO 20B/24B
    print(c.get_snapshot())       # handles snapshot 68..228B
    params = c.dump_params()      # full GET_PARAM dump with min/max
```

## Session bundle

A self-describing run anyone can send for offline diagnosis:

```
sessions/<name>/
  meta.json        fw / buildHash / board / profile / pole-pairs / features / host
  params.json      full param dump (value + min/max) at capture time
  telemetry.csv    the timeseries
  faults.json      fault transitions
```

Load it back with `Session.load(path)`. The diagnostic engine (v2) will read a
bundle and append `diagnosis.json` (ranked root causes + the exact param fixes).

## AI-assisted debugging — analyzers + MCP server

`garuda_gsp.analyze` encodes the recurring bench signatures as pure functions
(no deps), so they're flagged instantly with zero AI tokens:

- **phantom** — commanded eRPM above what the duty can physically produce (PLL
  harmonic false-lock / pot-zero phantom)
- **half_period** — scope sector cadence vs reported eRPM ≈ 2× (the benign,
  stable form of the same mechanism)
- **current_offset** — Ia ≠ 0 at standstill (phase-A sense DC offset)
- **polarity_miss** — odd(falling)/even(rising) ZC-miss clustering

```python
from garuda_gsp import analyze as A
A.analyze("sessions/<run>/telemetry.csv")     # telemetry findings
A.analyze_scope("sessions/scope_*.csv")       # scope findings (half-period etc.)
```

The **MCP server** exposes the live board + these analyzers as tools, so an MCP
client (Claude Code/Desktop) can drive the loop directly — capture → flag →
suggest/apply a param → re-capture — with no copy-paste:

```bash
pip install -e ".[mcp]"
# tools: connect, status, snapshot, stream, scope, params, get_param,
#        set_param, save_config, list_sessions, analyze_session
```

**Sharing one board between the GUI and the MCP/analyzers** — a USB-CDC port is
single-reader, so start the **broker** first and everything connects through it
concurrently (`connect_auto()` uses the broker if it's up, else opens the port
directly):

```bash
garuda-broker        # owns the port, caches telemetry, serializes commands
garuda-gui           # connects via the broker  ┐ both live on the same board
garuda-mcp           # connects via the broker  ┘ at the same time
```

**Autonomous auto-tune** (`autotune.py`, MCP tool `autotune`): drives the motor
over GSP (throttle→GSP, start, **heartbeat dead-man**), sweeps ONE whitelisted
param over a range at a fixed throttle, measures a cost (`miss_rate` /
`erpm_stability` / `top_speed`), restores the original, and returns the cost
curve + best value. Hard rails: aborts + stops on any fault, `Ibus` over limit,
or eRPM over cap; only `SAFE_PARAMS` (never OC/UV/OV/maxRPM); `Ia` not used for
safety (the offset issue) — `Ibus` is. The `motor` tool gives direct
start/stop/throttle/source control. **Supervise it — it spins a real motor**,
and apply the result yourself (`set_param` + `save_config`); the sweep only
measures, it doesn't keep the change.

Read-only by default; `set_param`/`save_config`/`motor`/`autotune` refuse unless
the server is started with `GARUDA_MCP_ALLOW_WRITE=1` (they change motor
behaviour). Register
it in the project `.mcp.json` (stdio), pointing `command` at this venv's python,
`cwd` at this folder. The "Diagnose with Claude" button (`garuda_gui/diagnose.py`,
needs `.[ai]` + `ANTHROPIC_API_KEY`) calls the same analyzers plus `claude-opus-4-8`
for the novel cases.

## Why a shared library

- **One protocol.** `garuda_gsp.protocol` is the single source of truth for
  commands, params, state/fault names — mirrors the firmware headers.
- **Version-negotiated.** Decoders branch on payload length, so an older or newer
  board both decode without crashing.
- **Reusable.** The existing `tools/*.py` scripts can drop their copy-pasted
  framing/decoders and `from garuda_gsp import ...` instead.
