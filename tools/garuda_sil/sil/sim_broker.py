"""sim_broker.py — serve the garuda_sil twin over the GUI's broker protocol.

The debug GUI / MCP / analyzers never talk to a serial port directly — they
talk to the broker on 127.0.0.1:47800 (line-JSON RPC + pushed telemetry).
This process speaks that exact protocol but is backed by the SIL twin, so
`garuda-gui` connects to the SIMULATOR with zero GUI changes: live plots,
state trace, session auto-record, MCP — all of it, no board required.

Usage (from tools/garuda_sil):
  ../garuda_debug/.venv/bin/python sil/sim_broker.py                # 2810 @24.3V
  ... sil/sim_broker.py --so libgaruda_sil_vex_pll.so --motor vex --vbus 10
  ... sil/sim_broker.py --pot 40 --speed 1.0 --tcp 47800

Then start the GUI normally (garuda-gui). Do NOT run the real broker at the
same time (same TCP port). Throttle: the GUI's pot is the simulated pot —
change it live via MCP/script RPC:  set_throttle(<adc 0..4095>).
Extra sim-only RPCs: sim_set_vbus(volts), sim_state() (plant truth).
"""
from __future__ import annotations
import argparse
import json
import socket
import threading
import time
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from garuda_sil import Sim, Plant2810, STATES, PWM_HZ, _adc  # noqa: E402

HOST, TCP_PORT = "127.0.0.1", 47800
POLL_HZ = 30.0

# sil STATES index == GSP state id (verified against protocol.STATE_NAMES)
FAULT_IDS = {"NONE": 0, "OV": 1, "UV": 2, "OC": 3, "BOARD_PCI": 4,
             "STALL": 5, "DESYNC": 6}


def make_plant(args):
    if args.motor == "vex":
        return Plant2810(vbus=args.vbus, kv=4000.0, pp=6, r_ll=0.44,
                         j=2e-7, b_visc=2e-6, v_drop_a=0.08, v_drop_c=0.0)
    return Plant2810(vbus=args.vbus)            # calibrated 2810 defaults


class SimBroker:
    def __init__(self, sim: Sim, speed=1.0):
        self.sim = sim
        self.speed = speed
        self.lock = threading.Lock()            # sim state guard
        self.conns = []
        self.running = True
        self.t0 = time.time()
        self._ia_pk = 0.0
        self._ibus_pk = 0.0
        self.last_snap = {}

    # ── sim loop: paced to wall clock ───────────────────────────────────
    def _sim_loop(self):
        batch = max(1, int(PWM_HZ * self.speed / POLL_HZ))
        period = 1.0 / POLL_HZ
        while self.running:
            t = time.monotonic()
            with self.lock:
                for _ in range(batch):
                    self.sim.step_tick()
                    ia = abs(self.sim.plant.i_ph[0])
                    if ia > self._ia_pk:
                        self._ia_pk = ia
                    ib = abs(self.sim.plant.ibus)
                    if ib > self._ibus_pk:
                        self._ibus_pk = ib
                self.last_snap = self._snapshot()
            self._broadcast({"telem": self.last_snap})
            dt = period - (time.monotonic() - t)
            if dt > 0:
                time.sleep(dt)

    # ── GSP-shaped views ────────────────────────────────────────────────
    def _info(self):
        s = self.sim
        return {
            "protocolVersion": 3, "fwVersion": "SIL",
            "boardId": 0, "motorProfileId": -1,
            "motorProfile": f"SIM {type(s.plant).__name__} "
                            f"kv={s.plant.pp and round(60/(2*3.14159*s.plant.ke_ll))}",
            "motorPolePairs": s.plant.pp, "featureFlags": 0, "isFoc": False,
            "pwmFrequency": PWM_HZ, "maxErpm": 260000,
            "buildHash": 0, "infoBytes": 0, "sim": True,
        }

    def _snapshot(self):
        s, lib, p = self.sim, self.sim.lib, self.sim.plant
        state = lib.sil_state()
        fault = lib.sil_fault()
        ia_pk, self._ia_pk = self._ia_pk, 0.0
        ibus_pk, self._ibus_pk = self._ibus_pk, 0.0
        T = lib.sil_hwzc_period()
        return {
            "t": time.time() - self.t0,
            "state": state,
            "state_name": ("CL" if STATES[state] == "CLOSED_LOOP"
                           else STATES[state]),
            "fault": fault,
            "fault_name": s.fault_name(),
            "throttle": s.pot,
            "duty": lib.sil_duty(),
            "duty_pct": 100.0 * lib.sil_duty() / max(1, s.looptime),
            "vbus_V": p.vbus,
            "ibus_A": p.ibus,
            "ibus_win_A": p.ibus,
            "eRPM": lib.sil_erpm(),
            "bemf_raw": lib.sil_bemf_raw(),
            "zc_thresh": lib.sil_zc_threshold(),
            "step_period": lib.sil_step_period(),
            "good_zc": lib.sil_good_zc(),
            "synced": lib.sil_zc_synced(),
            "zc_confirmed": 0, "zc_timeout": 0,
            "hwzc_en": lib.sil_hwzc_enabled(),
            "hwzc_hr": T,
            "hwzc_zc": lib.sil_hwzc_total_zc(),
            "hwzc_miss": 0, "hwzc_reject": 0,
            "ia_pk_mag": int(ia_pk * 1000),
            "ib_pk_mag": int(ia_pk * 1000),
            "ibus_pk_mag": int(ibus_pk * 1000),
            "uptime": int(time.time() - self.t0),
            "spi_en": 0, "spi_zcs": 0, "spi_target": 0, "spi_error": 0,
            "spi_output": 0, "spi_integ": 0,
            "cpu_load_pct": 0.0, "miss_by_sector": [0] * 6,
            "fall_off_min": 0, "fall_off_max": 0, "snapBytes": 0,
            # ── sim-only truth channels (GUI ignores; analyzers love) ──
            "sim": True,
            "plant_erpm": p.erpm(),
            "plant_theta_deg": (p.theta_e * 57.29578) % 360.0,
            "pll_active": lib.sil_pll_active(),
            "cap_frac_pm": lib.sil_hwzc_cap_frac_pm(),
        }

    # ── RPC surface (GspClient method names) ────────────────────────────
    def _dispatch(self, method, params):
        s = self.sim
        with self.lock:
            if method in ("get_info", "info", "connect"):
                return self._info()
            if method == "ping":
                return True
            if method == "get_snapshot":
                return self.last_snap or self._snapshot()
            if method == "start_motor":
                s.cmd_start(); return True
            if method == "stop_motor":
                s.cmd_stop(); return True
            if method == "clear_fault":
                s.cmd_stop(); return True
            if method == "set_throttle":
                s.pot = max(0, min(4095, int(params[0]))); return True
            if method == "set_throttle_src":
                return True
            if method in ("heartbeat", "telem_start", "telem_stop",
                          "save_config"):
                return True
            if method == "get_param_list":
                return []
            if method == "dump_params":
                return {}
            if method == "scope_status":
                return {"state": 0, "samples": 0}
            # sim-only controls
            if method == "sim_set_vbus":
                s.plant.vbus = float(params[0])
                s.vbus_counts = _adc(s.plant.vbus)
                return True
            if method == "sim_set_pot":
                s.pot = int(params[0]); return True
            if method == "sim_state":
                return {"plant_erpm": s.plant.erpm(),
                        "state": s.state_name(), "fault": s.fault_name()}
        raise RuntimeError(f"sim broker: unsupported method {method}")

    # ── TCP plumbing (same wire format as garuda_gsp.broker) ───────────
    def _broadcast(self, msg):
        line = (json.dumps(msg) + "\n").encode()
        for c in list(self.conns):
            try:
                c[1].acquire()
                try:
                    c[0].sendall(line)
                finally:
                    c[1].release()
            except Exception:
                self._drop(c)

    def _drop(self, c):
        if c in self.conns:
            self.conns.remove(c)
        try:
            c[0].close()
        except Exception:
            pass

    def _handle(self, conn):
        c = (conn, threading.Lock())
        self.conns.append(c)
        f = conn.makefile("rb")
        try:
            for raw in f:
                try:
                    req = json.loads(raw)
                except Exception:
                    continue
                resp = {"id": req.get("id")}
                try:
                    resp["result"] = self._dispatch(req.get("method"),
                                                    req.get("params") or [])
                except Exception as e:
                    resp["error"] = f"{type(e).__name__}: {e}"
                try:
                    c[1].acquire()
                    try:
                        conn.sendall((json.dumps(resp) + "\n").encode())
                    finally:
                        c[1].release()
                except Exception:
                    break
        finally:
            self._drop(c)

    def serve(self, host=HOST, port=TCP_PORT):
        threading.Thread(target=self._sim_loop, daemon=True).start()
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((host, port))
        srv.listen(8)
        print(f"garuda-sim-broker: twin on {host}:{port} "
              f"(start garuda-gui now; Ctrl+C stops)", flush=True)
        try:
            while True:
                conn, _ = srv.accept()
                threading.Thread(target=self._handle, args=(conn,),
                                 daemon=True).start()
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            srv.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--so", default=None, help="variant .so (default baseline)")
    ap.add_argument("--motor", choices=["2810", "vex"], default="2810")
    ap.add_argument("--vbus", type=float, default=24.3)
    ap.add_argument("--pot", type=int, default=40)
    ap.add_argument("--speed", type=float, default=1.0,
                    help="sim speed vs real time (1.0 = real time)")
    ap.add_argument("--tcp", type=int, default=TCP_PORT)
    args = ap.parse_args()

    kw = {}
    if args.so:
        kw["so_path"] = os.path.abspath(args.so)
    sim = Sim(plant=make_plant(args), pot=args.pot, **kw)
    sim.run(1000)                               # boot to IDLE
    SimBroker(sim, speed=args.speed).serve(port=args.tcp)


if __name__ == "__main__":
    main()
