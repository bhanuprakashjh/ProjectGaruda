# garuda-debug

Cross-platform debug tooling for the Garuda ESC, built on one shared serial-protocol
library so every tool (and the upcoming GUI) speaks GSP identically — no more
"works on my machine" or a tool breaking when the firmware wire format changes.

This is **v0**: the protocol library + the shareable session-bundle format.
Live debug GUI (PySide6 + pyqtgraph) and the diagnostic engine build on top.

## Install

```bash
# from this directory; -e keeps it editable while we iterate
pip install -e tools/garuda_debug
#   or, isolated:
pipx install ./tools/garuda_debug
```

Only runtime dep is `pyserial`. The GUI extras (`pip install -e ".[gui]"`) come in v1.

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

## Why a shared library

- **One protocol.** `garuda_gsp.protocol` is the single source of truth for
  commands, params, state/fault names — mirrors the firmware headers.
- **Version-negotiated.** Decoders branch on payload length, so an older or newer
  board both decode without crashing.
- **Reusable.** The existing `tools/*.py` scripts can drop their copy-pasted
  framing/decoders and `from garuda_gsp import ...` instead.
