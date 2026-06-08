"""garuda_mcp.py — MCP server exposing the Garuda ESC + analyzers as tools.

Lets an MCP client (Claude Code / Desktop) drive the live board and read
structured findings directly — no copy-paste. The loop becomes: request a
capture -> analyzers flag the known signatures -> suggest/apply a param ->
re-capture, all in one turn.

Read-mostly by default. Mutating tools (set_param, save_config) refuse unless
GARUDA_MCP_ALLOW_WRITE=1 — they change motor behavior.

Run (stdio):  python -m garuda_gsp.garuda_mcp     (see .mcp.json)
Install:      pip install -e ".[mcp]"
Env:          GARUDA_MCP_PORT (force a port), GARUDA_MCP_ALLOW_WRITE=1 (allow writes)
"""
import glob
import os
import time

from mcp.server.fastmcp import FastMCP

from . import protocol as P
from . import analyze as A
from .client import GspClient, candidate_ports, _score
from .broker import connect_auto, broker_running
from serial.tools import list_ports as _list_ports
from .session import Session

mcp = FastMCP("garuda")

_client = None
_port = os.environ.get("GARUDA_MCP_PORT") or None
WRITE_OK = os.environ.get("GARUDA_MCP_ALLOW_WRITE") == "1"


def _c() -> GspClient:
    """Lazily connect (probes for the board if no port forced)."""
    global _client
    if _client is None:
        _client = connect_auto(_port)        # via broker if running, else direct
        _client.connect()
    return _client


def _resolve(name):
    if name in P.PARAM_IDS:
        return P.PARAM_IDS[name]
    try:
        return int(str(name), 0)
    except ValueError:
        raise ValueError(f"unknown param '{name}'")


def _summary(samples):
    if not samples:
        return {}
    def pk(k):
        return round(max((s.get(k, 0) or 0) for s in samples))
    states = {}
    for s in samples:
        st = s.get("state_name", "?")
        states[st] = states.get(st, 0) + 1
    return {"peak_eRPM": pk("eRPM"), "peak_ia_A": pk("ia_pk_mag"),
            "states": states}


# ── connection / live ──────────────────────────────────────────────────────
@mcp.tool()
def connect(port: str = "") -> dict:
    """Connect to the board (probes for it if `port` is empty). Returns GET_INFO
    (fw version, build hash, motor profile, pole pairs, PWM, eRPM cap)."""
    global _client, _port
    if _client is not None:
        try:
            _client.close()
        except Exception:
            pass
        _client = None
    _port = port or None
    info = _c().connect()
    info["_port"] = _client.port
    return info


@mcp.tool()
def status() -> dict:
    """Connection state and, if disconnected, the serial ports seen."""
    if _client is None:
        usb = [p.device for p in _list_ports.comports() if _score(p) < 6]
        return {"connected": False, "broker": broker_running(),
                "ports": usb or [d for d, _ in candidate_ports()]}
    return {"connected": True, "port": _client.port,
            "via_broker": str(_client.port).startswith("broker://"),
            "info": _client.info}


@mcp.tool()
def snapshot() -> dict:
    """One decoded live telemetry snapshot."""
    return _c().get_snapshot()


@mcp.tool()
def stream(seconds: float = 3.0, hz: float = 20.0) -> dict:
    """Stream telemetry for up to `seconds`, then run the analyzers. Returns the
    findings (phantom / current-offset / polarity-lock) plus a compact summary —
    not the raw samples (keeps it token-cheap)."""
    c = _c()
    samples = []
    period = 1.0 / max(hz, 1.0)
    t_end = time.monotonic() + min(max(seconds, 0.2), 30.0)
    while time.monotonic() < t_end:
        try:
            samples.append(c.get_snapshot())
        except Exception:
            pass
        time.sleep(period)
    return {"n": len(samples), "summary": _summary(samples),
            "findings": A.analyze(samples)}


@mcp.tool()
def scope(trigger: str = "Manual", pre_pct: int = 25) -> dict:
    """Arm the 24 kHz burst scope, wait for the trigger, read the ring, and run
    the scope analyzers (half-period 2×, current offset). trigger ∈
    {Manual, On fault, On state change, Threshold}."""
    c = _c()
    mode = P.SCOPE_TRIG.get(trigger, 0)
    c.scope_arm(trig_mode=mode, pre_pct=pre_pct)
    rows = []
    for _ in range(150):                      # ~3 s for the trigger
        if c.scope_status().get("state") == 3:    # READY
            rows = c.scope_read_all()
            break
        time.sleep(0.02)
    return {"n": len(rows), "findings": A.analyze_scope(rows)}


# ── parameters ──────────────────────────────────────────────────────────────
@mcp.tool()
def params() -> dict:
    """Full readable parameter state: {name: {id, value, min, max, group}}."""
    return _c().dump_params()


@mcp.tool()
def get_param(name: str) -> dict:
    """Read one parameter by name (or numeric id)."""
    pid = _resolve(name)
    return {"name": name, "id": pid, "value": _c().get_param(pid)}


@mcp.tool()
def set_param(name: str, value: int) -> dict:
    """Write a parameter (name or id) and read it back. CHANGES MOTOR BEHAVIOR —
    refused unless GARUDA_MCP_ALLOW_WRITE=1. Does not persist (call save_config
    to write EEPROM)."""
    if not WRITE_OK:
        return {"ok": False,
                "error": "writes disabled — start the server with "
                         "GARUDA_MCP_ALLOW_WRITE=1 to allow set_param"}
    pid = _resolve(name)
    c = _c()
    c.set_param(pid, int(value))
    return {"ok": True, "name": name, "id": pid, "value": c.get_param(pid)}


@mcp.tool()
def save_config() -> dict:
    """Persist the current parameters to EEPROM (requires GARUDA_MCP_ALLOW_WRITE=1)."""
    if not WRITE_OK:
        return {"ok": False, "error": "writes disabled (GARUDA_MCP_ALLOW_WRITE=1)"}
    _c().save_config()
    return {"ok": True}


# ── offline analysis ────────────────────────────────────────────────────────
# sessions/ lives next to this package (tools/garuda_debug/sessions), resolved
# absolutely so it works no matter what cwd the MCP runner launches us in.
_SESSIONS = os.environ.get("GARUDA_SESSIONS_DIR") or os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "sessions")


@mcp.tool()
def list_sessions() -> list:
    """Saved telemetry/scope CSVs under the sessions/ folder (absolute path)."""
    return sorted(glob.glob(os.path.join(_SESSIONS, "**", "telemetry.csv"),
                            recursive=True)
                  + glob.glob(os.path.join(_SESSIONS, "*.csv")))


@mcp.tool()
def analyze_session(path: str) -> dict:
    """Run the analyzers on a saved telemetry/scope CSV or a session-bundle dir.
    Auto-detects scope captures (have sector + t_us) vs telemetry."""
    if path.endswith(".csv"):
        rows = A.load_csv(path)
        if rows and "sector" in rows[0] and "t_us" in rows[0]:
            return {"kind": "scope", "n": len(rows),
                    "findings": A.analyze_scope(rows)}
        return {"kind": "telemetry", "n": len(rows),
                "findings": A.analyze(rows)}
    s = Session.load(path)
    return {"kind": "bundle", "n": len(s.samples), "info": s.info,
            "findings": A.analyze(s.samples)}


def main():
    mcp.run()


if __name__ == "__main__":
    main()
