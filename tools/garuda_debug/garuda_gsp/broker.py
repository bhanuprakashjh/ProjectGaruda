"""broker.py — single-owner serial broker so the GUI, the MCP server, and
analyzers can all share ONE live board.

A USB-CDC port (/dev/ttyACM0, COMx) is single-reader, so only one process can
talk to the board. The broker opens the port once, polls telemetry, caches the
latest snapshot, and serializes GSP commands from many clients over a localhost
TCP socket (line-delimited JSON). Clients use `BrokerClient`, which mirrors the
`GspClient` interface by RPC. `connect_auto()` returns a BrokerClient if a broker
is up, else a direct GspClient — so the GUI worker and the MCP server work
unchanged; start the broker first and they share the board.

    Terminal 1:  garuda-broker            # owns the port
    Terminal 2:  garuda-gui               # connects via the broker
    (MCP):       garuda-mcp               # also via the broker — concurrently

localhost only. Not authenticated — bench tool.
"""
import json
import socket
import subprocess
import sys
import threading
import time

from .client import GspClient

HOST = "127.0.0.1"
PORT = 47800                              # localhost only
POLL_HZ = 30.0


# ── daemon ──────────────────────────────────────────────────────────────────
class _Conn:
    """One client connection with its own send lock (the poller broadcast and
    the command reply both write to this socket)."""
    def __init__(self, sock):
        self.sock = sock
        self._slock = threading.Lock()

    def send(self, msg):
        line = (json.dumps(msg) + "\n").encode()
        with self._slock:
            self.sock.sendall(line)


class Broker:
    def __init__(self, gsp):
        self.gsp = gsp                   # the single GspClient that owns the port
        self.lock = threading.Lock()     # serialize all UART transactions
        self.conns = []
        self.last_snap = {}
        self.running = True

    def _poll(self):
        period = 1.0 / POLL_HZ
        while self.running:
            t = time.monotonic()
            try:
                with self.lock:
                    snap = self.gsp.get_snapshot()
                self.last_snap = snap
                self._broadcast({"telem": snap})
            except Exception:
                pass
            dt = period - (time.monotonic() - t)
            if dt > 0:
                time.sleep(dt)

    def _broadcast(self, msg):
        for c in list(self.conns):
            try:
                c.send(msg)
            except Exception:
                self._drop(c)

    def _drop(self, c):
        if c in self.conns:
            self.conns.remove(c)
        try:
            c.sock.close()
        except Exception:
            pass

    def _handle(self, conn):
        c = _Conn(conn)
        self.conns.append(c)
        f = conn.makefile("rb")
        try:
            for raw in f:
                try:
                    req = json.loads(raw)
                except Exception:
                    continue
                rid = req.get("id")
                method = req.get("method")
                params = req.get("params", []) or []
                resp = {"id": rid}
                try:
                    if method == "get_snapshot":
                        resp["result"] = self.last_snap          # cached, no UART
                    elif method == "info":
                        resp["result"] = self.gsp.info
                    else:
                        with self.lock:                          # serialized UART
                            resp["result"] = getattr(self.gsp, method)(*params)
                except Exception as e:                           # noqa
                    resp["error"] = f"{type(e).__name__}: {e}"
                c.send(resp)
        except Exception:
            pass
        finally:
            self._drop(c)

    def serve(self, host=HOST, port=PORT):
        threading.Thread(target=self._poll, daemon=True).start()
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((host, port))
        srv.listen(8)
        print(f"garuda-broker: serving {self.gsp.port} on {host}:{port}", flush=True)
        try:
            while self.running:
                conn, _ = srv.accept()
                threading.Thread(target=self._handle, args=(conn,),
                                 daemon=True).start()
        finally:
            self.running = False
            srv.close()


def serve(port=None, baud=115200, host=HOST, tcp_port=PORT, gsp=None):
    """Open the board (probes if `port` None) and serve the broker. `gsp` may be
    injected (tests)."""
    if gsp is None:
        gsp = GspClient(port, baud=baud)
        gsp.connect()
    Broker(gsp).serve(host, tcp_port)


# ── client shim (GspClient-compatible) ─────────────────────────────────────
class BrokerClient:
    """Talks to the broker over TCP; forwards GspClient method calls as RPC and
    receives the pushed telemetry stream. Drop-in for GspClient."""
    def __init__(self, host=HOST, port=PORT, timeout=2.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.settimeout(None)
        self._wf = self.sock.makefile("wb")
        self._id = 0
        self._pending = {}
        self._cv = threading.Condition()
        self._telem_cb = None
        self.last_telem = {}
        self.info = {}
        self.port = f"broker://{host}:{port}"
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        rf = self.sock.makefile("rb")
        for raw in rf:
            try:
                msg = json.loads(raw)
            except Exception:
                continue
            if "telem" in msg:
                self.last_telem = msg["telem"]
                cb = self._telem_cb
                if cb:
                    try:
                        cb(msg["telem"])
                    except Exception:
                        pass
            elif "id" in msg:
                with self._cv:
                    self._pending[msg["id"]] = msg
                    self._cv.notify_all()

    def _rpc(self, method, *params, timeout=4.0):
        with self._cv:
            self._id += 1
            rid = self._id
        self._wf.write((json.dumps({"id": rid, "method": method,
                                    "params": list(params)}) + "\n").encode())
        self._wf.flush()
        end = time.time() + timeout
        with self._cv:
            while rid not in self._pending:
                if not self._cv.wait(timeout=max(0.0, end - time.time())):
                    raise TimeoutError(f"broker RPC timeout: {method}")
            r = self._pending.pop(rid)
        if r.get("error"):
            raise RuntimeError(r["error"])
        return r.get("result")

    # explicit GspClient surface ------------------------------------------------
    def get_info(self):
        self.info = self._rpc("get_info")
        return self.info

    def connect(self, **kw):
        return self.get_info()

    def ping(self):
        try:
            return bool(self._rpc("ping"))
        except Exception:
            return False

    def get_snapshot(self):
        return self._rpc("get_snapshot")

    def set_telem_callback(self, cb):
        """Receive each broadcast snapshot (push) instead of polling."""
        self._telem_cb = cb

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass

    # everything else (get_param/set_param/dump_params/scope_*/save_config/...)
    # forwards to the broker as RPC
    def __getattr__(self, name):
        if name.startswith("_"):
            raise AttributeError(name)
        def _f(*params):
            return self._rpc(name, *params)
        return _f


def broker_running(host=HOST, port=PORT) -> bool:
    try:
        with socket.create_connection((host, port), timeout=0.3):
            return True
    except OSError:
        return False


def ensure_broker(port=None, wait=5.0):
    """If no broker is running, spawn one (so the port owner is the broker, not
    us) and wait until it's ready. Returns the Popen if WE started it (caller
    should terminate it on exit), or None if one was already running / it
    couldn't come up (caller then falls back to a direct port via connect_auto).
    The board's port must be free — close any direct connection first."""
    if broker_running():
        return None
    args = [sys.executable, "-m", "garuda_gsp.broker"]
    if port:
        args += ["--port", str(port)]
    try:
        proc = subprocess.Popen(args, stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)
    except Exception:
        return None
    end = time.time() + wait
    while time.time() < end:
        if broker_running():
            return proc
        if proc.poll() is not None:          # broker exited (port busy / no board)
            return None
        time.sleep(0.1)
    return proc if broker_running() else None


def connect_auto(port=None, baud=115200, timeout=1.0, probe=True):
    """Share the live board if a broker is running, else open the port directly.
    Returns a BrokerClient or a GspClient — both expose the same interface."""
    if broker_running():
        try:
            return BrokerClient()
        except OSError:
            pass
    return GspClient(port, baud=baud, timeout=timeout, probe=probe)


def main():
    import argparse
    ap = argparse.ArgumentParser(description="Garuda serial broker (shares one board)")
    ap.add_argument("--port", default=None, help="serial port (default: probe)")
    ap.add_argument("--tcp-port", type=int, default=PORT)
    args = ap.parse_args()
    serve(port=args.port, tcp_port=args.tcp_port)


if __name__ == "__main__":
    main()
