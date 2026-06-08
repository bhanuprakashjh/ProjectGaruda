#!/usr/bin/env python3
"""
Garuda Studio GUI (v2) — monitor + live tuning workbench.

Tabbed shell (pillars land as tabs):
  • Monitor  — state machine, live readouts, normalized scope, operating map.
  • Tune     — read AND edit every parameter; edits push to the board live via
               SET_PARAM (RAM-only under FORCE_DEFAULTS=1), with an over-current
               safety check, and export the live set as a C profile block.
Console dock (shared) mirrors telemetry and takes commands.

    garuda-gui            # auto-detect port
    garuda-gui --port /dev/ttyACM0
"""
import argparse
import os
import platform
import sys
import time
from collections import deque
from queue import Queue, Empty

from PySide6 import QtCore, QtGui, QtWidgets
import numpy as np
import pyqtgraph as pg

from garuda_gsp import GspClient, GspError, Session, __version__ as GSP_VER
from garuda_gsp import protocol as P
from . import diagnose as DIAG
from . import wizard as WIZ
from . import zcsim as ZCSIM

STATES = ["IDLE", "ARMED", "ALIGN", "OL_RAMP", "MORPH", "CL"]
WINDOW_S = 20.0          # default rolling plot window
MAX_WINDOW_S = 60.0      # largest selectable time base (sizes the ring buffers)
SCOPE_DIVS = 4           # vertical half-span: Y axis runs -SCOPE_DIVS..+SCOPE_DIVS
POLL_HZ = 30

# Param group code -> human label (matches gsp_params.h PARAM_GROUP_*)
GROUP_NAMES = {
    0: "Startup", 1: "Closed-loop", 2: "Over-current", 3: "ZC detect",
    4: "Duty slew", 5: "Voltage", 6: "Recovery", 7: "Motor HW",
    8: "FOC motor", 9: "FOC tuning", 10: "FOC startup", 11: "AN1078",
}

# Selectable scope channels: key, label, color, full-scale (for normalization),
# bipolar?, live-value formatter, default-on?
SIGNALS = [
    ("eRPM",     "eRPM",     "#42a5f5", 260000, False, lambda v: f"{v:,.0f}",  True),
    ("ia",       "Ia_pk",    "#ffa726", 30,     False, lambda v: f"{v:.1f}A",  True),
    ("ibus",     "Ibus",     "#ef5350", 30,     True,  lambda v: f"{v:+.1f}A", True),
    ("rejrate",  "HWZC rej", "#ab47bc", 100,    False, lambda v: f"{v:.0f}%",  True),
    ("vbus",     "Vbus",     "#66bb6a", 40,     False, lambda v: f"{v:.1f}V",  False),
    ("bemf",     "BEMF",     "#90caf9", 4095,   False, lambda v: f"{v:.0f}",   False),
    ("zcthr",    "ZC thresh","#bdbdbd", 4095,   False, lambda v: f"{v:.0f}",   False),
    ("goodzc",   "good_zc",  "#80cbc4", 8,      False, lambda v: f"{v:.0f}",   False),
    ("duty",     "Duty",     "#26c6da", 100,    False, lambda v: f"{v:.0f}%",  False),
    ("throttle", "Throttle", "#ffee58", 4096,   False, lambda v: f"{v:.0f}",   False),
    ("cpu",      "CPU load", "#f06292", 100,    False, lambda v: f"{v:.0f}%",  False),
]

# Params whose value implies a startup/standing current = (modPct or duty%) of Vbus
# over phase-to-phase resistance. Used for the OC-safety estimate in the tuner.
AMPLITUDE_PARAMS = {"sineAlignModPct", "sineRampModPct"}     # peak = pct/200
DUTY_PARAMS = {"rampDutyPct", "alignDutyPct", "clIdleDutyPct"}  # = pct/100


# ─────────────────────────────────────────────────────────────────────────
def _port_hint() -> str:
    """Human hint listing the serial ports we can see — shown on connect failure
    so a Windows user knows which COM to pick (and to avoid the debug COM)."""
    try:
        from garuda_gsp.client import candidate_ports
        cs = candidate_ports()
        if not cs:
            return "No serial ports found — check the USB cable and the UART driver."
        return ("Ports seen: " + ", ".join(d for d, _ in cs) +
                ".  Try 'Auto-detect', or pick each port until one connects "
                "(the board is whichever answers GET_INFO).")
    except Exception:
        return ""


class SerialWorker(QtCore.QThread):
    """Owns the GspClient on a background thread so the UI never blocks.
    UI-thread code must NEVER touch the port directly — it enqueues actions via
    submit(); the worker drains them between snapshot polls (one owner, no race)."""
    connected = QtCore.Signal(dict)
    params_ready = QtCore.Signal(dict)
    snapshot = QtCore.Signal(dict)
    param_written = QtCore.Signal(dict)   # {ok, pid, name, value} | {ok:False, ..., error}
    scope_ready = QtCore.Signal(dict)     # {ok, samples, status}
    failed = QtCore.Signal(str)
    stopped = QtCore.Signal()

    def __init__(self, port):
        super().__init__()
        self.port = port
        self._run = True
        self._q = Queue()

    def submit(self, action, **kw):
        """Thread-safe: enqueue an action for the worker to run on the port."""
        self._q.put((action, kw))

    def _drain(self, c):
        while True:
            try:
                action, kw = self._q.get_nowait()
            except Empty:
                break
            try:
                if action == "set_param":
                    c.set_param(kw["pid"], kw["value"])
                    rb = c.get_param(kw["pid"])           # read back what stuck
                    self.param_written.emit({"ok": True, "pid": kw["pid"],
                                             "name": kw.get("name", ""), "value": rb})
                elif action == "refresh_params":
                    self.params_ready.emit(c.dump_params())
                elif action == "scope_capture":
                    c.scope_arm(trig_mode=kw.get("mode", 0), pre_pct=kw.get("pre", 25))
                    st, ready = {"state": 0}, False
                    for _ in range(150):          # ~3s waiting for the trigger
                        st = c.scope_status()
                        if st.get("state") == 3:  # READY
                            ready = True
                            break
                        self.msleep(20)
                    samples = c.scope_read_all() if ready else []
                    self.scope_ready.emit({"ok": ready, "samples": samples, "status": st})
            except Exception as e:  # noqa
                self.param_written.emit({"ok": False, "pid": kw.get("pid"),
                                         "name": kw.get("name", ""), "error": str(e)})

    def run(self):
        try:
            c = GspClient(self.port)        # port=None -> probes for the board
            try:
                info = c.connect()          # GET_INFO with retry (Windows-safe)
            except GspError:
                self.failed.emit(f"No response on {c.port} (baud/power/wrong port?).  "
                                 f"{_port_hint()}")
                return
            info["_port"] = c.port          # the port actually connected
            self.connected.emit(info)
            try:
                self.params_ready.emit(c.dump_params())
            except GspError:
                pass
            period = 1.0 / POLL_HZ
            while self._run:
                t = time.monotonic()
                self._drain(c)                            # writes first, then read
                try:
                    snap = c.get_snapshot()
                    self.snapshot.emit(snap)
                except GspError:
                    pass
                dt = period - (time.monotonic() - t)
                if dt > 0:
                    self.msleep(int(dt * 1000))
            c.close()
        except Exception as e:  # noqa
            self.failed.emit(f"{e}.  {_port_hint()}")
        finally:
            self.stopped.emit()

    def stop(self):
        self._run = False
        self.wait(1500)


# ─────────────────────────────────────────────────────────────────────────
class StateBar(QtWidgets.QWidget):
    """Horizontal chips for the startup state machine; active one lit."""
    def __init__(self):
        super().__init__()
        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.chips = {}
        for i, s in enumerate(STATES):
            lbl = QtWidgets.QLabel(s)
            lbl.setAlignment(QtCore.Qt.AlignCenter)
            lbl.setStyleSheet(self._css(False))
            lay.addWidget(lbl)
            if i < len(STATES) - 1:
                arr = QtWidgets.QLabel("→")
                arr.setStyleSheet("color:#888;")
                lay.addWidget(arr)
            self.chips[s] = lbl

    @staticmethod
    def _css(active, fault=False):
        if fault:
            return ("background:#b71c1c;color:#fff;border-radius:6px;"
                    "padding:6px 12px;font-weight:bold;")
        if active:
            return ("background:#1b5e20;color:#fff;border-radius:6px;"
                    "padding:6px 12px;font-weight:bold;")
        return ("background:#2a2a2a;color:#bbb;border-radius:6px;padding:6px 12px;")

    def set_state(self, name, fault):
        for s, lbl in self.chips.items():
            lbl.setStyleSheet(self._css(s == name, fault and s == name))


class Readout(QtWidgets.QFrame):
    """Big labelled value tile."""
    def __init__(self, title, unit=""):
        super().__init__()
        self.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.unit = unit
        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(8, 4, 8, 4)
        self.t = QtWidgets.QLabel(title)
        self.t.setStyleSheet("color:#9e9e9e;font-size:11px;")
        self.val = QtWidgets.QLabel("—")
        self.val.setStyleSheet("font-size:22px;font-weight:bold;")
        v.addWidget(self.t)
        v.addWidget(self.val)

    def set(self, value, color=None):
        self.val.setText(f"{value}{self.unit}")
        self.val.setStyleSheet(f"font-size:22px;font-weight:bold;"
                               f"{'color:'+color+';' if color else ''}")


class SectorMissBar(QtWidgets.QFrame):
    """Six bars showing per-sector 'guess' rate (PI period with no captured ZC).

    Fed cumulative hwzcMissBySector[6] each frame; tracks the delta-per-second
    so the bars read 'guesses/s' in each commutation sector. An EVEN row means
    misses are spread (state-driven / healthy); a CLUSTERED row (one or two
    tall bars) means a sector/polarity the comparator structurally can't see —
    the signature that windowed-comparator sampling would fix. Also prints the
    aggregate measured:guess ratio."""
    def __init__(self):
        super().__init__()
        self.setFrameShape(QtWidgets.QFrame.StyledPanel)
        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(8, 4, 8, 4); v.setSpacing(2)
        self.hdr = QtWidgets.QLabel("Per-sector guesses/s  (measured-vs-guess)")
        self.hdr.setStyleSheet("color:#9e9e9e;font-size:11px;")
        v.addWidget(self.hdr)
        row = QtWidgets.QHBoxLayout(); row.setSpacing(6)
        self.bars, self.labs = [], []
        for i in range(6):
            col = QtWidgets.QVBoxLayout(); col.setSpacing(1)
            bar = QtWidgets.QProgressBar()
            bar.setOrientation(QtCore.Qt.Vertical)
            bar.setRange(0, 100); bar.setValue(0); bar.setTextVisible(False)
            bar.setFixedSize(26, 56)
            lab = QtWidgets.QLabel(f"S{i}")
            lab.setAlignment(QtCore.Qt.AlignCenter)
            lab.setStyleSheet("color:#9e9e9e;font-size:10px;")
            col.addWidget(bar, 0, QtCore.Qt.AlignHCenter); col.addWidget(lab)
            row.addLayout(col)
            self.bars.append(bar); self.labs.append(lab)
        v.addLayout(row)
        self._prev = None          # (miss_by_sector tuple, t)
        self._prev_acc = None      # (hwzc_zc, hwzc_miss)

    def update_from(self, s: dict):
        miss = s.get("miss_by_sector") or [0]*6
        t = s.get("t", 0.0)
        # aggregate measured:guess (per-sector misses sum could wrap u16; use
        # the full u32 totals from hwzc_zc/hwzc_miss for the headline ratio).
        acc, mss = s.get("hwzc_zc", 0), s.get("hwzc_miss", 0)
        if self._prev_acc is not None:
            d_acc = max(0, acc - self._prev_acc[0])
            d_mss = max(0, mss - self._prev_acc[1])
            tot = d_acc + d_mss
            if tot > 0:
                pct = 100.0 * d_acc / tot
                col = "#66bb6a" if pct >= 90 else ("#ffa726" if pct >= 70 else "#ef5350")
                self.hdr.setText(
                    f"Per-sector guesses/s — measured {pct:.0f}% / guess {100-pct:.0f}%")
                self.hdr.setStyleSheet(f"color:{col};font-size:11px;font-weight:bold;")
        self._prev_acc = (acc, mss)

        if self._prev is not None:
            dt = max(1e-3, t - self._prev[1])
            rates = [max(0, (miss[i] - self._prev[0][i]) & 0xFFFF) / dt for i in range(6)]
            peak = max(rates) if max(rates) > 0 else 1.0
            for i in range(6):
                pctbar = int(100 * rates[i] / peak)
                self.bars[i].setValue(pctbar)
                hot = rates[i] > 0.5 * peak and peak > 1.0
                self.bars[i].setStyleSheet(
                    "QProgressBar::chunk{background:%s;}" %
                    ("#ef5350" if hot else "#42a5f5"))
                self.labs[i].setText(f"S{i}\n{rates[i]:.0f}")
        self._prev = (list(miss), t)


# ─────────────────────────────────────────────────────────────────────────
class SectorAcceptGuessBar(QtWidgets.QFrame):
    """Per-sector ACCEPTED (green) vs GUESSED (blue) commutations, plus a
    COMPOSITE stacked bar of the aggregate accepted/guessed split.

    The board only sends per-sector GUESS counters (hwzcMissBySector[6]) and
    the aggregate accept/guess totals (hwzc_zc / hwzc_miss).  The 6 commutation
    sectors are visited uniformly each electrical revolution, so per-sector
    accepted is recovered as:
        accept[i] ≈ (Δacc + Δmiss)/6 − guess[i]
    Bars read events/second (delta over dt).  The composite is the aggregate
    accepted% stacked over guessed% (sums to 100), same headline as the run."""
    GREEN = (102, 187, 106)
    BLUE = (66, 165, 245)

    def __init__(self):
        super().__init__()
        self.setFrameShape(QtWidgets.QFrame.StyledPanel)
        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(8, 4, 8, 4); v.setSpacing(2)
        self.hdr = QtWidgets.QLabel(
            "Per-sector accepted vs guessed  —  🟩 accepted   🟦 guessed")
        self.hdr.setStyleSheet("color:#9e9e9e;font-size:11px;")
        v.addWidget(self.hdr)

        row = QtWidgets.QHBoxLayout(); row.setSpacing(6)

        # ── per-sector grouped bars (green accepted | blue guessed) ──
        self.p = pg.PlotWidget()
        self.p.setMouseEnabled(x=False, y=False)
        self.p.setMenuEnabled(False); self.p.hideButtons()
        self.p.showGrid(x=False, y=True, alpha=0.2)
        self.p.setLabel("left", "events/s")
        self.p.getAxis("bottom").setTicks([[(i, f"S{i}") for i in range(6)]])
        self.p.setXRange(-0.6, 5.6, padding=0)
        self.bar_acc = pg.BarGraphItem(
            x=[i - 0.2 for i in range(6)], height=[0] * 6, width=0.36,
            brush=pg.mkBrush(*self.GREEN), pen=None)
        self.bar_gss = pg.BarGraphItem(
            x=[i + 0.2 for i in range(6)], height=[0] * 6, width=0.36,
            brush=pg.mkBrush(*self.BLUE), pen=None)
        self.p.addItem(self.bar_acc); self.p.addItem(self.bar_gss)
        row.addWidget(self.p, 1)

        # ── composite: accepted% stacked over guessed% ──
        self.pc = pg.PlotWidget()
        self.pc.setMouseEnabled(x=False, y=False)
        self.pc.setMenuEnabled(False); self.pc.hideButtons()
        self.pc.showGrid(x=False, y=True, alpha=0.2)
        self.pc.setLabel("left", "%")
        self.pc.setYRange(0, 100, padding=0)
        self.pc.setXRange(-0.7, 0.7, padding=0)
        self.pc.getAxis("bottom").setTicks([[(0, "ALL")]])
        self.pc.setFixedWidth(96)
        self.c_gss = pg.BarGraphItem(x=[0], y0=[0], height=[0], width=0.8,
                                     brush=pg.mkBrush(*self.BLUE), pen=None)
        self.c_acc = pg.BarGraphItem(x=[0], y0=[0], height=[0], width=0.8,
                                     brush=pg.mkBrush(*self.GREEN), pen=None)
        self.pc.addItem(self.c_gss); self.pc.addItem(self.c_acc)
        row.addWidget(self.pc)

        v.addLayout(row)
        self.setMaximumHeight(180)
        self._prev = None          # (acc, miss, [sector misses], t)

    def update_from(self, s: dict):
        acc = int(s.get("hwzc_zc", 0)); mss = int(s.get("hwzc_miss", 0))
        sect = s.get("miss_by_sector") or [0] * 6
        t = float(s.get("t", 0.0))
        if self._prev is not None:
            d_acc = max(0, acc - self._prev[0])
            d_mss = max(0, mss - self._prev[1])
            dt = max(1e-3, t - self._prev[3])
            guess = [max(0, (sect[i] - self._prev[2][i]) & 0xFFFF) for i in range(6)]
            total = d_acc + d_mss
            per_sector = total / 6.0
            accept = [max(0.0, per_sector - guess[i]) for i in range(6)]
            self.bar_acc.setOpts(height=[a / dt for a in accept])
            self.bar_gss.setOpts(height=[g / dt for g in guess])
            if total > 0:
                acc_pct = 100.0 * d_acc / total
                gss_pct = 100.0 - acc_pct
                self.c_gss.setOpts(y0=[0], height=[gss_pct])
                self.c_acc.setOpts(y0=[gss_pct], height=[acc_pct])
                col = ("#66bb6a" if acc_pct >= 90 else
                       "#ffa726" if acc_pct >= 70 else "#ef5350")
                self.hdr.setText(
                    f"Per-sector accepted vs guessed  —  "
                    f"🟩 accepted {acc_pct:.0f}%   🟦 guessed {gss_pct:.0f}%")
                self.hdr.setStyleSheet(
                    f"color:{col};font-size:11px;font-weight:bold;")
        self._prev = (acc, mss, list(sect), t)


# ─────────────────────────────────────────────────────────────────────────
class RotorWidget(QtWidgets.QWidget):
    """Animated rotor + 6-sector commutation wheel. Sectors colored by per-sector
    ZC capture probability (green=detected, red=guessed); the rotor bar spins at a
    (scaled) rate and a pointer marks the live electrical angle / floating phase."""
    PHASE_NAME = ["A", "B", "C"]
    # firmware commutationTable: (floating phase, zc polarity) per 60deg sector
    SEC = [(2, +1), (0, -1), (1, +1), (2, -1), (0, +1), (1, -1)]

    def __init__(self):
        super().__init__()
        self.setMinimumSize(280, 280)
        self.angle = 0.0                 # electrical deg 0..360
        self.per_sector = [0.0] * 6
        self.erpm = 0
        self.cap_n = 0
        self.guess_n = 0

    def set_capture(self, per_sector, erpm):
        self.per_sector = list(per_sector)
        self.erpm = erpm
        self.update()

    def reset_tally(self):
        self.cap_n = self.guess_n = 0

    def paintEvent(self, _ev):
        import math
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w / 2.0, h / 2.0
        R = min(w, h) * 0.44
        rect = QtCore.QRectF(cx - R, cy - R, 2 * R, 2 * R)
        # sector wedges (Qt: angle in 1/16 deg, 0=3 o'clock, CCW)
        for i in range(6):
            pr = self.per_sector[i]
            col = QtGui.QColor(int(235 * (1 - pr)) + 20, int(190 * pr) + 25, 55)
            p.setBrush(QtGui.QBrush(col))
            p.setPen(QtGui.QPen(QtGui.QColor("#202020"), 1))
            p.drawPie(rect, int(i * 60 * 16), int(60 * 16))
            # sector label (R/F)
            mid = math.radians(i * 60 + 30)
            lx, ly = cx + math.cos(mid) * R * 0.78, cy - math.sin(mid) * R * 0.78
            p.setPen(QtGui.QPen(QtGui.QColor("#111")))
            p.drawText(QtCore.QRectF(lx - 12, ly - 8, 24, 16),
                       QtCore.Qt.AlignCenter, "R" if self.SEC[i][1] > 0 else "F")
        # rotor hub
        rr = R * 0.5
        p.setBrush(QtGui.QBrush(QtGui.QColor("#161616")))
        p.setPen(QtGui.QPen(QtGui.QColor("#333"), 1))
        p.drawEllipse(QtCore.QRectF(cx - rr, cy - rr, 2 * rr, 2 * rr))
        # rotor bar N(red)/S(blue) at electrical angle
        ang = math.radians(self.angle)
        dx, dy = math.cos(ang), -math.sin(ang)
        lw = max(5, int(R * 0.10))
        penN = QtGui.QPen(QtGui.QColor("#ef5350"), lw); penN.setCapStyle(QtCore.Qt.RoundCap)
        p.setPen(penN)
        p.drawLine(QtCore.QPointF(cx, cy), QtCore.QPointF(cx + dx * rr * 0.92, cy + dy * rr * 0.92))
        penS = QtGui.QPen(QtGui.QColor("#42a5f5"), lw); penS.setCapStyle(QtCore.Qt.RoundCap)
        p.setPen(penS)
        p.drawLine(QtCore.QPointF(cx, cy), QtCore.QPointF(cx - dx * rr * 0.92, cy - dy * rr * 0.92))
        # center text
        sec = int(self.angle // 60) % 6
        fphase = self.PHASE_NAME[self.SEC[sec][0]]
        p.setPen(QtGui.QPen(QtGui.QColor("#e0e0e0")))
        f = p.font(); f.setPointSize(10); f.setBold(True); p.setFont(f)
        p.drawText(QtCore.QRectF(cx - rr, cy - 22, 2 * rr, 16),
                   QtCore.Qt.AlignCenter, "%s sector" % ("rising" if self.SEC[sec][1] > 0 else "falling"))
        f.setPointSize(8); f.setBold(False); p.setFont(f)
        p.drawText(QtCore.QRectF(cx - rr, cy - 2, 2 * rr, 14),
                   QtCore.Qt.AlignCenter, "float %s | %d eRPM" % (fphase, self.erpm))
        p.end()


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, port):
        super().__init__()
        self.setWindowTitle("Garuda Studio — v2")
        self.resize(1180, 760)
        self.port = port
        self.worker = None
        self.info = {}
        self.params = {}
        self.recording = False
        self.session = None
        self._auto_session = None   # auto-records every run -> CSV, no button
        self._ml_file = None        # ML dataset collection (ZC Lab tab)
        self.t0 = None
        self._last_vbus = None
        self._populating = False
        self._row_name = {}
        # one buffer per selectable channel (+ time)
        self.buf = {k: deque(maxlen=int(MAX_WINDOW_S * POLL_HZ)) for k in
                    (["t"] + [s[0] for s in SIGNALS])}
        self.scope_window_s = WINDOW_S
        self.scope_frozen = False
        self.map_pts = deque(maxlen=2000)        # (eRPM, Ia_pk) operating-point scatter
        self.live = deque(maxlen=4000)           # rolling samples for on-demand diagnosis
        self._prev_rej = None                    # (zc, reject) for windowed reject-rate
        self._console_paused = False
        self._was_running = False                 # gate console mirror to run-only

        pg.setConfigOptions(antialias=True, background="#101418", foreground="#ccc")
        self._build()

    # ── UI ──────────────────────────────────────────────────────────────
    def _build(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)

        # top bar (global, above tabs)
        top = QtWidgets.QHBoxLayout()
        self.portbox = QtWidgets.QComboBox()
        self.portbox.setEditable(True)
        self._refresh_ports()
        self.btn_rescan = QtWidgets.QPushButton("⟳")
        self.btn_rescan.setToolTip("Rescan serial ports")
        self.btn_rescan.setFixedWidth(32)
        self.btn_rescan.clicked.connect(self._refresh_ports)
        self.btn_connect = QtWidgets.QPushButton("Connect")
        self.btn_connect.clicked.connect(self.toggle_connect)
        self.btn_record = QtWidgets.QPushButton("● Record")
        self.btn_record.setEnabled(False)
        self.btn_record.clicked.connect(self.toggle_record)
        self.btn_diag = QtWidgets.QPushButton("🔍 Diagnose")
        self.btn_diag.setEnabled(False)
        self.btn_diag.clicked.connect(self.run_diagnosis)
        self.lbl_id = QtWidgets.QLabel("not connected")
        self.lbl_id.setStyleSheet("color:#9e9e9e;")
        top.addWidget(QtWidgets.QLabel("Port:"))
        top.addWidget(self.portbox, 2)
        top.addWidget(self.btn_rescan)
        top.addWidget(self.btn_connect)
        top.addWidget(self.btn_record)
        top.addWidget(self.btn_diag)
        top.addStretch(1)
        top.addWidget(self.lbl_id, 3)
        root.addLayout(top)

        # tabbed pillars
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.addTab(self._build_monitor_tab(), "📈 Monitor")
        self.tabs.addTab(self._build_tune_tab(), "🎛 Tune")
        self.tabs.addTab(self._build_wizard_tab(), "🧙 Wizard")
        self.tabs.addTab(self._build_scope_tab(), "🔬 ZC Explainer")
        self.tabs.addTab(self._build_sim_tab(), "🧪 ZC Lab")
        root.addWidget(self.tabs, 1)

        # console dock (shared across tabs)
        dock = QtWidgets.QDockWidget("Console", self)
        dock.setAllowedAreas(QtCore.Qt.BottomDockWidgetArea | QtCore.Qt.TopDockWidgetArea)
        cw = QtWidgets.QWidget()
        cv = QtWidgets.QVBoxLayout(cw)
        cv.setContentsMargins(4, 2, 4, 4)
        self.console = QtWidgets.QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(2000)
        self.console.setStyleSheet("font-family: monospace; font-size: 11px;"
                                   "background:#0c0f12; color:#cdd6cf;")
        self.cmd = QtWidgets.QLineEdit()
        self.cmd.setPlaceholderText("command…  (try: help)")
        self.cmd.returnPressed.connect(self._on_command)
        self.cmd.setStyleSheet("font-family: monospace;")
        cv.addWidget(self.console, 1)
        cv.addWidget(self.cmd)
        dock.setWidget(cw)
        self.addDockWidget(QtCore.Qt.BottomDockWidgetArea, dock)
        self._log("Garuda console. 'help' for commands. Telemetry streams here once connected.")

        self.status = self.statusBar()
        self.status.showMessage("Select a port and Connect.")

    def _build_sim_tab(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        hint = QtWidgets.QLabel(
            "ZC Lab — live model of sensorless zero-cross detection. Pick a motor, drag the "
            "speed slider, and watch which commutation sectors get a real measured timestamp "
            "(green) vs coast on a guess (red). The capture model encodes the validated "
            "mechanism (rising always seen in PWM-ON; falling via the RC-filtered OFF-center "
            "sample, limited by samples-per-sector + RC lag) calibrated to the bench. "
            "'Deep-dive' runs the switching-level waveform; the SPICE engine is EXPERIMENTAL "
            "(runs, but does not yet reproduce the masking from first principles).")
        hint.setWordWrap(True); hint.setStyleSheet("color:#9e9e9e;font-size:11px;")
        v.addWidget(hint)

        c = QtWidgets.QHBoxLayout()
        c.addWidget(QtWidgets.QLabel("Motor:"))
        self.lab_motor = QtWidgets.QComboBox()
        self.lab_motor.addItems(list(ZCSIM.MOTORS.keys()))
        self.lab_motor.currentIndexChanged.connect(self._lab_set_motor)
        c.addWidget(self.lab_motor)
        self.lab_mlabel = QtWidgets.QLabel("")
        self.lab_mlabel.setStyleSheet("color:#9e9e9e;")
        c.addWidget(self.lab_mlabel)
        c.addStretch(1)
        c.addWidget(QtWidgets.QLabel("waveform engine:"))
        self.sim_engine = QtWidgets.QComboBox()
        self.sim_engine.addItems(["lumped (numpy)", "SPICE (experimental)"])
        self.sim_engine.setToolTip("SPICE is an experimental switching-physics sandbox — "
                                   "it runs but does NOT yet quantitatively reproduce the "
                                   "bench masking. The live capture model (and the bench data) "
                                   "are the trustworthy parts.")
        c.addWidget(self.sim_engine)
        b_spice = QtWidgets.QPushButton("⚡ Deep-dive")
        b_spice.clicked.connect(self._lab_waveform)
        c.addWidget(b_spice)
        v.addLayout(c)

        s = QtWidgets.QHBoxLayout()
        s.addWidget(QtWidgets.QLabel("Speed"))
        self.lab_speed = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.lab_speed.setRange(0, 235000)
        self.lab_speed.valueChanged.connect(self._lab_update)
        s.addWidget(self.lab_speed, 1)
        self.lab_speed_lbl = QtWidgets.QLabel("0 eRPM")
        self.lab_speed_lbl.setMinimumWidth(180)
        self.lab_speed_lbl.setStyleSheet("font-weight:bold;")
        s.addWidget(self.lab_speed_lbl)
        v.addLayout(s)

        # --- ML data collection (uses the live burst-scope, NOT the slider model) ---
        ml = QtWidgets.QHBoxLayout()
        self.ml_btn = QtWidgets.QPushButton("⏺ Collect ML dataset")
        self.ml_btn.setCheckable(True)
        self.ml_btn.setToolTip(
            "While the motor runs, repeatedly grab the 24 kHz burst scope and log each "
            "128-sample window (bemf/zcthr/sector + eRPM/duty) to sessions/ml_*.jsonl. "
            "The offline trainer extracts the TRUE ZC per sector as the label.")
        self.ml_btn.toggled.connect(self._ml_toggle_collect)
        ml.addWidget(self.ml_btn)
        self.ml_lbl = QtWidgets.QLabel("dataset: idle")
        self.ml_lbl.setStyleSheet("color:#9e9e9e;font-family:monospace;")
        ml.addWidget(self.ml_lbl)
        ml.addStretch(1)
        self.ml_train_btn = QtWidgets.QPushButton("🧠 Train / Eval")
        self.ml_train_btn.setToolTip(
            "Train the pure-numpy MLP on all sessions/ml_*.jsonl and print the "
            "ZC-prediction MAE (elec deg) vs eRPM, rising vs falling, to the log.")
        self.ml_train_btn.clicked.connect(self._ml_train)
        ml.addWidget(self.ml_train_btn)
        v.addLayout(ml)
        self._ml_file = None
        self._ml_count = 0

        split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        left = QtWidgets.QWidget(); lv = QtWidgets.QVBoxLayout(left)
        self.lab_rotor = RotorWidget()
        lv.addWidget(self.lab_rotor, 1)
        self.lab_cap = QtWidgets.QLabel("measured —")
        self.lab_cap.setAlignment(QtCore.Qt.AlignCenter)
        self.lab_cap.setStyleSheet("font-size:20px;font-weight:bold;")
        lv.addWidget(self.lab_cap)
        self.lab_split = QtWidgets.QLabel("rising — / falling —")
        self.lab_split.setAlignment(QtCore.Qt.AlignCenter)
        lv.addWidget(self.lab_split)
        self.lab_tally = QtWidgets.QLabel("captured 0 / guessed 0")
        self.lab_tally.setAlignment(QtCore.Qt.AlignCenter)
        self.lab_tally.setStyleSheet("color:#9e9e9e;font-family:monospace;")
        lv.addWidget(self.lab_tally)
        split.addWidget(left)

        right = QtWidgets.QWidget(); rv = QtWidgets.QVBoxLayout(right)
        glw1 = pg.GraphicsLayoutWidget()
        self.lab_curve = glw1.addPlot(title="ZC capture vs eRPM (marker = slider)")
        self.lab_curve.showGrid(x=True, y=True, alpha=0.2); self.lab_curve.addLegend(offset=(10, 10))
        self.lab_curve.setLabel('left', 'capture %'); self.lab_curve.setLabel('bottom', 'eRPM')
        self.lab_curve.setYRange(0, 105)
        self.lab_c_meas = self.lab_curve.plot(pen=pg.mkPen("#42a5f5", width=2), name="measured %")
        self.lab_c_fall = self.lab_curve.plot(pen=pg.mkPen("#ffa726", width=2), name="falling %")
        self.lab_marker = pg.InfiniteLine(angle=90, pen=pg.mkPen("#e0e0e0", style=QtCore.Qt.DashLine))
        self.lab_curve.addItem(self.lab_marker)
        rv.addWidget(glw1, 1)

        glw2 = pg.GraphicsLayoutWidget()
        self.sim_p = glw2.addPlot(title="Floating-phase waveform through the ZC (ADC) — Deep-dive")
        self.sim_p.showGrid(x=True, y=True, alpha=0.2); self.sim_p.addLegend(offset=(10, 10))
        self.sim_raw = self.sim_p.plot(pen=pg.mkPen("#90caf9", width=1), name="V_node (ON spikes)")
        self.sim_filt = self.sim_p.plot(pen=pg.mkPen("#66bb6a", width=2), name="V_sense (RC-filtered)")
        self.sim_thr = self.sim_p.plot(pen=pg.mkPen("#bdbdbd", width=1, style=QtCore.Qt.DashLine), name="threshold")
        self.sim_p.setLabel('bottom', 'time', 's', siPrefix=True)
        rv.addWidget(glw2, 1)
        split.addWidget(right); split.setSizes([360, 660])
        v.addWidget(split, 1)

        self._lab_last_sector = -1
        self._lab_timer = QtCore.QTimer(self)
        self._lab_timer.timeout.connect(self._lab_tick)
        self._lab_timer.start(33)
        QtCore.QTimer.singleShot(0, self._lab_set_motor)
        return w

    def _run_sim_spice(self, erpm, duty, sector):
        """Call the isolated SPICE venv as a subprocess; return the result dict."""
        import subprocess, json as _json
        here = os.path.dirname(os.path.abspath(__file__))     # garuda_gui/
        base = os.path.dirname(here)                           # garuda_debug/
        venv_py = os.path.join(base, ".venv-spice", "bin", "python")
        nglib = os.path.join(base, ".venv-spice", "nglib")
        script = os.path.join(here, "spice_engine.py")
        if not os.path.exists(venv_py):
            return {"ok": False, "error": "SPICE venv not found (.venv-spice). "
                    "See spice_engine.py header for setup."}
        env = dict(os.environ)
        env["LD_LIBRARY_PATH"] = nglib + os.pathsep + env.get("LD_LIBRARY_PATH", "")
        params = _json.dumps({"erpm": erpm, "duty": duty, "sector": sector})
        try:
            p = subprocess.run([venv_py, script, params], capture_output=True,
                               text=True, env=env, timeout=90)
        except Exception as e:
            return {"ok": False, "error": "subprocess: %s" % e}
        out = (p.stdout or "").strip().splitlines()
        if not out:
            return {"ok": False, "error": "no output\n" + (p.stderr or "")[-400:]}
        try:
            return _json.loads(out[-1])
        except Exception as e:
            return {"ok": False, "error": "parse: %s\n%s" % (e, (p.stderr or "")[-400:])}

    def _lab_set_motor(self):
        m = ZCSIM.MOTORS[self.lab_motor.currentText()]
        self.lab_mlabel.setText("KV %d | %d pole-pairs | %.0f V | max %d eRPM"
                                % (m["kv"], m["poles_pp"], m["vbus"], m["erpm_max"]))
        self.lab_speed.blockSignals(True)
        self.lab_speed.setRange(0, m["erpm_max"])
        if self.lab_speed.value() == 0:
            self.lab_speed.setValue(int(m["erpm_max"] * 0.3))
        self.lab_speed.blockSignals(False)
        erpms = np.linspace(2000, m["erpm_max"], 120)
        meas, fall = [], []
        for e in erpms:
            cm = ZCSIM.capture_model(int(e), m)
            meas.append(cm["measured"] * 100); fall.append(cm["falling"] * 100)
        self.lab_c_meas.setData(erpms, meas)
        self.lab_c_fall.setData(erpms, fall)
        self.lab_curve.setXRange(0, m["erpm_max"])
        self._lab_update()

    def _lab_update(self):
        m = ZCSIM.MOTORS[self.lab_motor.currentText()]
        erpm = self.lab_speed.value()
        cm = ZCSIM.capture_model(erpm, m)
        self._lab_cm = cm
        self.lab_speed_lbl.setText("%d eRPM  (duty %.0f%%)" % (erpm, cm["duty"] * 100))
        self.lab_marker.setValue(erpm)
        self.lab_rotor.set_capture(cm["per_sector"], erpm)
        self.lab_rotor.reset_tally(); self._lab_last_sector = -1
        col = "#66bb6a" if cm["measured"] >= 0.9 else ("#ffa726" if cm["measured"] >= 0.7 else "#ef5350")
        self.lab_cap.setText("measured %.0f%%" % (cm["measured"] * 100))
        self.lab_cap.setStyleSheet("font-size:20px;font-weight:bold;color:%s;" % col)
        self.lab_split.setText("rising %.0f%%  /  falling %.0f%%   (%.1f samples/sector)"
                               % (cm["rising"] * 100, cm["falling"] * 100, cm["spp"]))
        if self.sim_engine.currentIndex() == 0:
            self._lab_waveform_lumped(erpm, cm["duty"])

    def _lab_tick(self):
        if not hasattr(self, "_lab_cm"):
            return
        import random
        m = ZCSIM.MOTORS[self.lab_motor.currentText()]
        erpm = self.lab_speed.value()
        if erpm < 1:
            return
        rev_s = 0.2 + 2.3 * min(1.0, erpm / max(1, m["erpm_max"]))   # watchable spin
        self.lab_rotor.angle = (self.lab_rotor.angle + 360.0 * rev_s * 0.033) % 360.0
        sec = int(self.lab_rotor.angle // 60) % 6
        if sec != self._lab_last_sector and self._lab_last_sector >= 0:
            prob = self._lab_cm["per_sector"][self._lab_last_sector]
            if random.random() < prob:
                self.lab_rotor.cap_n += 1
            else:
                self.lab_rotor.guess_n += 1
            self.lab_tally.setText("captured %d / guessed %d"
                                   % (self.lab_rotor.cap_n, self.lab_rotor.guess_n))
        self._lab_last_sector = sec
        self.lab_rotor.update()

    def _lab_waveform_lumped(self, erpm, duty):
        if erpm < 1500:
            return
        r = ZCSIM.simulate(int(erpm), float(duty), sector_index=1)   # falling sector
        t = r["t_us"] * 1e-6
        self.sim_raw.setData(t, r["v_node"]); self.sim_filt.setData(t, r["v_sense"])
        self.sim_thr.setData([t[0], t[-1]], [r["thr_adc"], r["thr_adc"]])

    def _lab_waveform(self):
        erpm = self.lab_speed.value()
        duty = getattr(self, "_lab_cm", {}).get("duty", 0.4)
        if self.sim_engine.currentIndex() == 1:
            self.lab_tally.setText("running SPICE …")
            QtWidgets.QApplication.processEvents()
            d = self._run_sim_spice(erpm, duty, "falling")
            if not d.get("ok"):
                self.lab_tally.setText("SPICE error: " + str(d.get("error"))[:90])
                return
            t = np.array(d["t_us"]) * 1e-6
            self.sim_raw.setData(t, np.array(d["v_node_adc"]))
            self.sim_filt.setData(t, np.array(d["v_sense_adc"]))
            self.sim_thr.setData([t[0], t[-1]], [d["thr_adc"], d["thr_adc"]])
            ce, oe = d["comp_on_env"], d["offc_env"]
            self.lab_tally.setText(
                "SPICE falling: comparator %s (%d crossings) | OFF-center %s | "
                "ON-env %.0f..%.0f RC %.0f..%.0f (bench 12..2559 / 54..1282)"
                % ("DETECT" if d.get("comp_detect") else "MISS", d.get("comp_ncross", 0),
                   "DETECT" if d.get("offc_detect") else "MISS",
                   ce[0], ce[1], oe[0], oe[1]))
        else:
            self._lab_waveform_lumped(erpm, duty)

    def _build_monitor_tab(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)

        self.statebar = StateBar()
        v.addWidget(self.statebar)
        self.fault = QtWidgets.QLabel("")
        self.fault.setAlignment(QtCore.Qt.AlignCenter)
        self.fault.setStyleSheet("font-weight:bold;")
        v.addWidget(self.fault)

        rr = QtWidgets.QHBoxLayout()
        self.ro = {
            "eRPM": Readout("eRPM"), "duty": Readout("Duty", "%"),
            "vbus": Readout("Vbus", " V"), "ibus": Readout("Ibus", " A"),
            "thr": Readout("Throttle"), "iapk": Readout("Ia peak", " A"),
            "hwzc": Readout("HWZC rej", "%"), "cpu": Readout("CPU", "%"),
        }
        for x in self.ro.values():
            rr.addWidget(x)
        v.addLayout(rr)

        # ── Per-sector ZC: where measured timestamps vs guesses fall ──
        # The reject% above is comparator noise-fires filtered; THIS shows
        # which of the 6 commutation sectors are guessing (PI period expired
        # with no captured ZC). Bars = per-second guess count per sector;
        # an even row = healthy/state-driven, a clustered row = structural
        # (a polarity/phase the comparator can't see) → the case the windowed
        # comparator would actually fix.
        self.sector_box = SectorAcceptGuessBar()
        v.addWidget(self.sector_box)

        # ── oscilloscope: per-channel vertical controls (gain/position) +
        #    time base + freeze, like a bench scope. Y axis is in DIVISIONS;
        #    each trace = value / (units-per-div) + position. ──
        scope_box = QtWidgets.QHBoxLayout()

        rack = QtWidgets.QWidget()
        grid = QtWidgets.QGridLayout(rack)
        grid.setContentsMargins(2, 2, 2, 2)
        grid.setHorizontalSpacing(4); grid.setVerticalSpacing(2)
        for c, h in enumerate(("Ch", "units/div", "pos", "")):
            lab = QtWidgets.QLabel(h)
            lab.setStyleSheet("color:#9e9e9e;font-size:10px;")
            grid.addWidget(lab, 0, c)
        self.checks, self.ch_scale, self.ch_pos = {}, {}, {}
        self.sigcfg = {s[0]: s for s in SIGNALS}
        for i, (key, label, color, scale, bip, fmt, on) in enumerate(SIGNALS, start=1):
            cb = QtWidgets.QCheckBox(label)
            cb.setChecked(on)
            cb.setStyleSheet(f"color:{color}; font-weight:bold;")
            cb.stateChanged.connect(self._refresh_curve_visibility)
            sc = QtWidgets.QDoubleSpinBox()
            sc.setDecimals(3); sc.setRange(0.001, 1e7)
            sc.setValue(self._nice_scale(scale / 3.0))
            sc.setMaximumWidth(92)
            ps = QtWidgets.QDoubleSpinBox()
            ps.setDecimals(1); ps.setRange(-SCOPE_DIVS, SCOPE_DIVS); ps.setSingleStep(0.5)
            ps.setValue(0.0 if bip else -float(SCOPE_DIVS - 1))
            ps.setMaximumWidth(52)
            ab = QtWidgets.QToolButton(); ab.setText("A")
            ab.setToolTip("autoscale this channel to fit")
            ab.clicked.connect(lambda _=False, k=key: self._autoscale_channel(k))
            grid.addWidget(cb, i, 0); grid.addWidget(sc, i, 1)
            grid.addWidget(ps, i, 2); grid.addWidget(ab, i, 3)
            self.checks[key] = cb; self.ch_scale[key] = sc; self.ch_pos[key] = ps
        grid.setRowStretch(len(SIGNALS) + 1, 1)
        scr = QtWidgets.QScrollArea()
        scr.setWidget(rack); scr.setWidgetResizable(True)
        scr.setFixedWidth(252)
        scr.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        scope_box.addWidget(scr)

        pcol = QtWidgets.QVBoxLayout()
        ctl = QtWidgets.QHBoxLayout()
        ctl.addWidget(QtWidgets.QLabel("Time window:"))
        self.tb = QtWidgets.QComboBox()
        for s in ("2 s", "5 s", "10 s", "20 s", "60 s"):
            self.tb.addItem(s)
        self.tb.setCurrentText("20 s")
        self.tb.currentTextChanged.connect(self._set_window)
        ctl.addWidget(self.tb)
        self.btn_freeze = QtWidgets.QPushButton("⏸ Freeze")
        self.btn_freeze.setCheckable(True)
        self.btn_freeze.toggled.connect(self._toggle_freeze)
        ctl.addWidget(self.btn_freeze)
        self.btn_autoall = QtWidgets.QPushButton("Autoscale all")
        self.btn_autoall.clicked.connect(self._autoscale_all)
        ctl.addWidget(self.btn_autoall)
        self.cb_zclink = QtWidgets.QCheckBox("🔗 BEMF↔ZC")
        self.cb_zclink.setChecked(True)
        self.cb_zclink.setToolTip("Gang the ZC-thresh trace to BEMF's gain+position so the "
                                  "threshold sits on the BEMF where the zero-cross happens "
                                  "(they're the same physical signal).")
        ctl.addWidget(self.cb_zclink)
        ctl.addStretch(1)
        pcol.addLayout(ctl)

        self.p_main = pg.PlotWidget()
        self.p_main.showGrid(x=True, y=True, alpha=0.25)
        self.p_main.setYRange(-SCOPE_DIVS, SCOPE_DIVS, padding=0)
        self.p_main.setMouseEnabled(x=False, y=False)
        self.p_main.getAxis("left").setTicks(
            [[(d, "") for d in range(-SCOPE_DIVS, SCOPE_DIVS + 1)]])
        self.p_main.setLabel("bottom", "time (s)")
        self.curves = {}
        for key, label, color, _s, _b, _f, _on in SIGNALS:
            self.curves[key] = self.p_main.plot(pen=pg.mkPen(color, width=2))
        pcol.addWidget(self.p_main, 1)
        scope_box.addLayout(pcol, 1)

        self.p_map = pg.PlotWidget(title="Ia_pk (A) vs eRPM")
        self.p_map.showGrid(x=True, y=True, alpha=0.2)
        self.p_map.enableAutoRange(enable=True)
        self.scatter_map = pg.ScatterPlotItem(size=4, pen=None,
                                              brush=pg.mkBrush(66, 165, 245, 90))
        self.p_map.addItem(self.scatter_map)
        mapw = QtWidgets.QWidget(); mapl = QtWidgets.QVBoxLayout(mapw)
        mapl.setContentsMargins(0, 0, 0, 0); mapl.addWidget(self.p_map)
        mapw.setFixedWidth(260)
        scope_box.addWidget(mapw)

        v.addLayout(scope_box, 1)
        self._refresh_curve_visibility()
        return w

    # ── oscilloscope controls ───────────────────────────────────────────
    @staticmethod
    def _nice_scale(x):
        import math
        if x <= 0:
            return 1.0
        e = math.floor(math.log10(x))
        f = x / (10 ** e)
        nf = 1 if f < 1.5 else 2 if f < 3.5 else 5 if f < 7.5 else 10
        return nf * (10 ** e)

    def _set_window(self, txt):
        try:
            self.scope_window_s = float(txt.split()[0])
        except ValueError:
            pass

    def _toggle_freeze(self, on):
        self.scope_frozen = on
        self.btn_freeze.setText("▶ Run" if on else "⏸ Freeze")

    def _autoscale_channel(self, key):
        vals = list(self.buf[key])
        if not vals:
            return
        lo, hi = min(vals), max(vals)
        span = (hi - lo) or abs(hi) or 1.0
        scale = self._nice_scale(span / (2.0 * SCOPE_DIVS - 1)) or 1.0
        self.ch_scale[key].setValue(scale)
        mid = (hi + lo) / 2.0
        self.ch_pos[key].setValue(max(-SCOPE_DIVS, min(SCOPE_DIVS, -mid / scale)))

    def _autoscale_all(self):
        enabled = [k for k in self.checks if self.checks[k].isChecked()]
        n = len(enabled) or 1
        top = SCOPE_DIVS - 1
        for i, key in enumerate(enabled):
            vals = list(self.buf[key])
            if vals:
                lo, hi = min(vals), max(vals)
                span = (hi - lo) or abs(hi) or 1.0
                self.ch_scale[key].setValue(self._nice_scale(span / 1.5) or 1.0)
            pos = top - (2.0 * top * i / (n - 1)) if n > 1 else 0.0
            self.ch_pos[key].setValue(round(pos * 2) / 2.0)

    def _build_tune_tab(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        hint = QtWidgets.QLabel(
            "Edit a Value cell and press Enter — it pushes to the board live (SET_PARAM). "
            "Red = outside [min,max] or estimated to exceed the OC soft limit. Under "
            "FORCE_DEFAULTS=1 edits are RAM-only (lost on power-cycle) — use Export to bake "
            "the tuned set into a C profile block.")
        hint.setWordWrap(True)
        hint.setStyleSheet("color:#9e9e9e;font-size:11px;")
        v.addWidget(hint)

        bar = QtWidgets.QHBoxLayout()
        self.btn_reload = QtWidgets.QPushButton("↻ Reload from board")
        self.btn_reload.clicked.connect(self._reload_params)
        self.btn_export = QtWidgets.QPushButton("⤓ Export as C profile")
        self.btn_export.clicked.connect(self._export_profile_c)
        self.tune_filter = QtWidgets.QLineEdit()
        self.tune_filter.setPlaceholderText("filter parameters…")
        self.tune_filter.textChanged.connect(self._apply_tune_filter)
        bar.addWidget(self.btn_reload)
        bar.addWidget(self.btn_export)
        bar.addSpacing(12)
        bar.addWidget(self.tune_filter, 1)
        v.addLayout(bar)

        self.tbl = QtWidgets.QTableWidget(0, 5)
        self.tbl.setHorizontalHeaderLabels(
            ["Parameter", "Value", "Min", "Max", "Group · est."])
        self.tbl.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        self.tbl.verticalHeader().setVisible(False)
        self.tbl.itemChanged.connect(self._on_tune_edit)
        v.addWidget(self.tbl, 1)
        return w

    def _build_wizard_tab(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        hint = QtWidgets.QLabel(
            "Enter the motor's datasheet numbers → generate a complete, current-safe "
            "profile for all four firmware files. The heuristics are calibrated to the "
            "proven 2810 profile, so the output sits in the same trusted regime, scaled "
            "by this motor's physics. It will not emit an OC-tripping startup value.")
        hint.setWordWrap(True)
        hint.setStyleSheet("color:#9e9e9e;font-size:11px;")
        v.addWidget(hint)

        form = QtWidgets.QFormLayout()
        self.wz = {}
        self.wz["name"] = QtWidgets.QLineEdit("MyMotor")
        self.wz["enum"] = QtWidgets.QLineEdit("CUSTOM")
        self.wz["kv"] = QtWidgets.QSpinBox(); self.wz["kv"].setRange(1, 5000); self.wz["kv"].setValue(1000)
        self.wz["r"] = QtWidgets.QDoubleSpinBox(); self.wz["r"].setRange(0.001, 2.0)
        self.wz["r"].setDecimals(3); self.wz["r"].setSingleStep(0.005); self.wz["r"].setValue(0.050)
        self.wz["poles"] = QtWidgets.QSpinBox(); self.wz["poles"].setRange(2, 28); self.wz["poles"].setValue(14)
        self.wz["weight"] = QtWidgets.QDoubleSpinBox(); self.wz["weight"].setRange(1, 2000); self.wz["weight"].setValue(100)
        self.wz["imax"] = QtWidgets.QDoubleSpinBox(); self.wz["imax"].setRange(1, 200); self.wz["imax"].setValue(20)
        self.wz["vbus"] = QtWidgets.QDoubleSpinBox(); self.wz["vbus"].setRange(5, 60); self.wz["vbus"].setValue(24)
        self.wz["ls"] = QtWidgets.QDoubleSpinBox(); self.wz["ls"].setRange(0, 2000); self.wz["ls"].setValue(0)
        self.wz["ls"].setSuffix(" µH (0 = unknown)")
        self.wz["profile"] = QtWidgets.QSpinBox(); self.wz["profile"].setRange(0, 15); self.wz["profile"].setValue(6)
        form.addRow("Name", self.wz["name"])
        form.addRow("KV (RPM/V)", self.wz["kv"])
        form.addRow("Resistance ph-ph (Ω)", self.wz["r"])
        form.addRow("Magnet poles", self.wz["poles"])
        form.addRow("Weight (g)", self.wz["weight"])
        form.addRow("Max continuous (A)", self.wz["imax"])
        form.addRow("Bus voltage (V)", self.wz["vbus"])
        form.addRow("Inductance", self.wz["ls"])
        form.addRow("Profile id", self.wz["profile"])
        form.addRow("Enum suffix (GSP_PROFILE_…)", self.wz["enum"])
        fw = QtWidgets.QWidget(); fw.setLayout(form); fw.setMaximumWidth(360)

        right = QtWidgets.QVBoxLayout()
        bar = QtWidgets.QHBoxLayout()
        btn_gen = QtWidgets.QPushButton("⚙ Generate profile")
        btn_gen.clicked.connect(self._run_wizard)
        self.btn_wz_copy = QtWidgets.QPushButton("Copy"); self.btn_wz_copy.clicked.connect(
            lambda: QtWidgets.QApplication.clipboard().setText(self.wz_out.toPlainText()))
        self.btn_wz_save = QtWidgets.QPushButton("Save .c"); self.btn_wz_save.clicked.connect(self._save_wizard)
        bar.addWidget(btn_gen); bar.addStretch(1); bar.addWidget(self.btn_wz_copy); bar.addWidget(self.btn_wz_save)
        right.addLayout(bar)
        self.wz_out = QtWidgets.QPlainTextEdit()
        self.wz_out.setReadOnly(True)
        self.wz_out.setStyleSheet("font-family:monospace;font-size:11px;")
        self.wz_out.setPlaceholderText("Generated C blocks for all 4 files appear here…")
        right.addWidget(self.wz_out, 1)
        rw = QtWidgets.QWidget(); rw.setLayout(right)

        split = QtWidgets.QHBoxLayout()
        split.addWidget(fw); split.addWidget(rw, 1)
        v.addLayout(split, 1)
        return w

    def _build_scope_tab(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        hint = QtWidgets.QLabel(
            "24 kHz triggered capture — 128 samples (~5.3 ms) of the REAL waveform, "
            "un-aliased. See BEMF vs the ZC threshold (where they cross IS the zero-cross), "
            "plus phase current and sector. 'On fault' freezes the buffer at a desync; "
            "'On state change' catches the MORPH→CL handoff.")
        hint.setWordWrap(True)
        hint.setStyleSheet("color:#9e9e9e;font-size:11px;")
        v.addWidget(hint)

        bar = QtWidgets.QHBoxLayout()
        bar.addWidget(QtWidgets.QLabel("Trigger:"))
        self.scope_mode = QtWidgets.QComboBox()
        for k in P.SCOPE_TRIG:
            self.scope_mode.addItem(k)
        bar.addWidget(self.scope_mode)
        self.scope_pre = QtWidgets.QSpinBox()
        self.scope_pre.setRange(0, 90); self.scope_pre.setValue(25)
        self.scope_pre.setSuffix(" % pre")
        bar.addWidget(self.scope_pre)
        self.btn_scope = QtWidgets.QPushButton("◉ Arm & Capture")
        self.btn_scope.clicked.connect(self._capture_scope)
        bar.addWidget(self.btn_scope)
        self.btn_scope_save = QtWidgets.QPushButton("Save CSV")
        self.btn_scope_save.clicked.connect(self._save_scope)
        bar.addWidget(self.btn_scope_save)
        self.scope_status_lbl = QtWidgets.QLabel("idle")
        self.scope_status_lbl.setStyleSheet("color:#9e9e9e;")
        bar.addWidget(self.scope_status_lbl)
        bar.addStretch(1)
        v.addLayout(bar)

        glw = pg.GraphicsLayoutWidget()
        self.sc_p1 = glw.addPlot(row=0, col=0, title="BEMF vs ZC threshold (raw ADC) — crossings = zero-cross")
        self.sc_p1.showGrid(x=True, y=True, alpha=0.2)
        self.sc_p1.addLegend(offset=(10, 10))
        self.sc_bemf = self.sc_p1.plot(pen=pg.mkPen("#90caf9", width=2), name="BEMF")
        self.sc_zc = self.sc_p1.plot(pen=pg.mkPen("#ffb74d", width=2,
                                     style=QtCore.Qt.DashLine), name="ZC thresh")
        self.sc_zcross = pg.ScatterPlotItem(size=10, brush=pg.mkBrush("#ef5350"), pen=None)
        self.sc_p1.addItem(self.sc_zcross)
        self.sc_p2 = glw.addPlot(row=1, col=0, title="Phase current Ia (A)")
        self.sc_p2.showGrid(x=True, y=True, alpha=0.2)
        self.sc_ia = self.sc_p2.plot(pen=pg.mkPen("#ffa726", width=2))
        self.sc_p3 = glw.addPlot(row=2, col=0, title="Sector (0-5)")
        self.sc_p3.showGrid(x=True, y=True, alpha=0.2)
        self.sc_sec = self.sc_p3.plot(pen=pg.mkPen("#80cbc4", width=2))
        self.sc_p2.setXLink(self.sc_p1); self.sc_p3.setXLink(self.sc_p1)
        self.sc_p3.setLabel("bottom", "time (µs)")
        self.sc_trig_lines = []
        for pl in (self.sc_p1, self.sc_p2, self.sc_p3):
            ln = pg.InfiniteLine(angle=90, movable=False,
                                 pen=pg.mkPen("#66bb6a", style=QtCore.Qt.DotLine))
            ln.setVisible(False); pl.addItem(ln); self.sc_trig_lines.append(ln)
        v.addWidget(glw, 1)
        self._scope_samples = []
        return w

    def _capture_scope(self):
        if not self.worker:
            self._log("scope: not connected"); return
        mode = P.SCOPE_TRIG[self.scope_mode.currentText()]
        self.scope_status_lbl.setText("arming / waiting for trigger…")
        self.btn_scope.setEnabled(False)
        self._log(f"scope: arm ({self.scope_mode.currentText()}, {self.scope_pre.value()}% pre)")
        self.worker.submit("scope_capture", mode=mode, pre=self.scope_pre.value())

    def on_scope_ready(self, data):
        self.btn_scope.setEnabled(True)
        if not data.get("ok"):
            self.scope_status_lbl.setText("no trigger (timed out)")
            self._log("scope: timed out waiting for trigger"); return
        samples = data.get("samples", [])
        self._scope_samples = samples
        if not samples:
            self.scope_status_lbl.setText("empty buffer"); return
        n = len(samples)
        dt_us = 1e6 / 24000.0
        xs = [i * dt_us for i in range(n)]
        bemf = [s["bemf_raw"] for s in samples]
        zc = [s["zc_thresh"] for s in samples]
        self.sc_bemf.setData(xs, bemf)
        self.sc_zc.setData(xs, zc)
        self.sc_ia.setData(xs, [s["ia_A"] for s in samples])
        self.sc_sec.setData(xs, [s["sector"] for s in samples])
        # zero-cross = where (BEMF - threshold) changes sign
        zx, zy = [], []
        for i in range(1, n):
            d0, d1 = bemf[i - 1] - zc[i - 1], bemf[i] - zc[i]
            if d0 == 0 or (d0 < 0) != (d1 < 0):
                zx.append(xs[i]); zy.append(zc[i])
        self.sc_zcross.setData(zx, zy)
        st = data.get("status", {})
        trig = st.get("trig_idx", 0)
        for ln in self.sc_trig_lines:
            ln.setPos(trig * dt_us); ln.setVisible(True)
        self.scope_status_lbl.setText(
            f"{n} samples · {len(zx)} zero-crossings · trig idx {trig}")
        self._log(f"scope: {n} samples, {len(zx)} ZC, "
                  f"state={P.SCOPE_STATE_NAMES.get(st.get('state'), '?')}")
        # ML collect: append this window + operating point, then grab the next one
        if self._ml_file is not None:
            self._ml_record(samples)

    def _save_scope(self):
        if not self._scope_samples:
            self._log("scope: capture first"); return
        os.makedirs("sessions", exist_ok=True)
        path = os.path.join("sessions", f"scope_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        cols = ["ia_A", "ib_A", "ibus_A", "bemf_raw", "zc_thresh", "sector",
                "eRPM", "duty_pct", "state_name", "fault", "tick"]
        with open(path, "w") as f:
            f.write("idx,t_us," + ",".join(cols) + "\n")
            for i, s in enumerate(self._scope_samples):
                f.write(f"{i},{i*1e6/24000.0:.2f}," +
                        ",".join(str(s.get(c, "")) for c in cols) + "\n")
        self._log(f"scope: wrote {path}")
        self.scope_status_lbl.setText(f"saved {path}")

    # ---- ML dataset collection / training ---------------------------------
    def _ml_toggle_collect(self, on):
        if on:
            if not self.worker:
                self._log("ml: not connected"); self.ml_btn.setChecked(False); return
            os.makedirs("sessions", exist_ok=True)
            path = os.path.join("sessions", f"ml_{time.strftime('%Y%m%d_%H%M%S')}.jsonl")
            self._ml_file = open(path, "w")
            self._ml_path = path
            self._ml_count = 0
            self.ml_btn.setText("⏹ Stop collecting")
            self.ml_lbl.setText(f"recording → {os.path.basename(path)}  (0)")
            self._log(f"ml: collecting to {path} (run the motor across a speed sweep)")
            # kick the capture loop (Manual = immediate trigger, free-running)
            self.worker.submit("scope_capture", mode=P.SCOPE_TRIG["Manual"], pre=25)
        else:
            if self._ml_file is not None:
                self._ml_file.close()
                self._log(f"ml: saved {self._ml_count} windows → {self._ml_path}")
                self.ml_lbl.setText(f"saved {self._ml_count} windows: "
                                    f"{os.path.basename(self._ml_path)}")
                self._ml_file = None
            self.ml_btn.setText("⏺ Collect ML dataset")

    def _ml_record(self, samples):
        """Write one capture window as a JSONL line, then request the next."""
        import json as _json
        try:
            erpm = float(np.median([s.get("eRPM", 0) for s in samples]))
            duty = float(np.median([s.get("duty_pct", 0) for s in samples]))
            rec = {
                "erpm": erpm, "duty": duty,
                "motor": self.lab_motor.currentText(),
                "bemf": [s.get("bemf_raw", 0) for s in samples],
                "zcthr": [s.get("zc_thresh", 0) for s in samples],
                "sector": [s.get("sector", 0) for s in samples],
            }
            self._ml_file.write(_json.dumps(rec) + "\n")
            self._ml_file.flush()
            self._ml_count += 1
            self.ml_lbl.setText(f"recording → {os.path.basename(self._ml_path)}  "
                                f"({self._ml_count})  last {erpm:.0f} eRPM")
        except Exception as e:  # noqa
            self._log(f"ml: record error {e}")
        # request the next window (keeps the loop running while collecting)
        if self._ml_file is not None and self.worker:
            self.worker.submit("scope_capture", mode=P.SCOPE_TRIG["Manual"], pre=25)

    def _ml_train(self):
        """Train the numpy MLP on all sessions/ml_*.jsonl; report MAE to the log."""
        import glob as _glob
        from . import zcml
        paths = sorted(_glob.glob(os.path.join("sessions", "ml_*.jsonl")))
        if not paths:
            self._log("ml: no sessions/ml_*.jsonl yet — collect a run first"); return
        caps = zcml.load_jsonl(paths)
        X, y, meta = zcml.extract_examples(caps)
        self._log(f"ml: {len(caps)} windows from {len(paths)} files → "
                  f"{len(X)} sector examples")
        if len(X) < 30:
            self._log("ml: too few examples (need varied speeds; >90k eRPM is too "
                      "coarse for the 24 kHz scope to resolve a sector)"); return
        rng = np.random.default_rng(0)
        idx = rng.permutation(len(X))
        ntr = int(0.8 * len(X))
        tr, te = idx[:ntr], idx[ntr:]
        model = zcml.MLP(X.shape[1]).fit(X[tr], y[tr])
        import io, contextlib
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            zcml.evaluate(model, X[te], y[te], [meta[i] for i in te])
        for line in buf.getvalue().rstrip().splitlines():
            self._log("ml: " + line)

    def _wizard_spec(self):
        ls = self.wz["ls"].value()
        return WIZ.MotorSpec(
            name=self.wz["name"].text().strip() or "MyMotor",
            kv=self.wz["kv"].value(), r_pp_ohm=self.wz["r"].value(),
            poles=self.wz["poles"].value(), weight_g=self.wz["weight"].value(),
            max_current_a=self.wz["imax"].value(), vbus_nom=self.wz["vbus"].value(),
            inductance_uh=(ls if ls > 0 else None),
            profile_id=self.wz["profile"].value(),
            enum_name=self.wz["enum"].text().strip() or "CUSTOM")

    def _run_wizard(self):
        try:
            p = WIZ.compute_profile(self._wizard_spec())
        except Exception as e:  # noqa
            self.wz_out.setPlainText(f"error: {e}"); return
        self.wz_out.setPlainText(WIZ.render_all(p))
        self._log("wizard: " + WIZ.report(p).replace("\n", " | "))

    def _save_wizard(self):
        text = self.wz_out.toPlainText()
        if not text.strip():
            self._log("wizard: generate first"); return
        os.makedirs("sessions", exist_ok=True)
        name = self.wz["name"].text().strip() or "motor"
        path = os.path.join("sessions",
                            f"wizard_{name}_{time.strftime('%Y%m%d_%H%M%S')}.c")
        with open(path, "w") as f:
            f.write(text + "\n")
        self._log(f"wizard: wrote {path}")
        self.status.showMessage(f"Saved {path}")

    def _refresh_ports(self):
        from garuda_gsp.client import candidate_ports, _score
        from serial.tools import list_ports
        self.portbox.clear()
        # "Auto-detect" probes every port for the one that answers GET_INFO —
        # the reliable default on Windows (data=None signals probe).
        self.portbox.addItem("Auto-detect (probe)", None)
        # Only list real USB devices; hide the dozens of phantom legacy ttyS*
        # ports. (Box is editable, so a custom port can still be typed.)
        usb = [(p.device, p.description or p.device)
               for p in sorted(list_ports.comports(), key=_score) if _score(p) < 6]
        for dev, desc in (usb or candidate_ports()):
            self.portbox.addItem(f"{dev}  ({desc})", dev)
        if self.port:                       # explicit --port from the CLI wins
            self.portbox.insertItem(1, f"{self.port}  (cmdline)", self.port)
            self.portbox.setCurrentIndex(1)
        else:
            self.portbox.setCurrentIndex(0)

    def _selected_port(self):
        txt = self.portbox.currentText().strip()
        if txt.lower().startswith("auto"):
            return None                     # -> GspClient probes
        data = self.portbox.currentData()
        if data:
            return data
        return txt.split()[0] if txt else None

    # ── connection ──────────────────────────────────────────────────────
    def toggle_connect(self):
        if self.worker:
            self.worker.stop()
            self.worker = None
            self.btn_connect.setText("Connect")
            self.btn_record.setEnabled(False)
            self.status.showMessage("Disconnected.")
            return
        port = self._selected_port()
        self.worker = SerialWorker(port)
        self.worker.connected.connect(self.on_connected)
        self._pending_port = port
        self.worker.params_ready.connect(self.on_params)
        self.worker.snapshot.connect(self.on_snapshot)
        self.worker.param_written.connect(self.on_param_written)
        self.worker.scope_ready.connect(self.on_scope_ready)
        self.worker.failed.connect(self.on_failed)
        self.worker.start()
        self.btn_connect.setText("Disconnect")
        self.status.showMessage("Auto-detecting board…" if port is None
                                else f"Connecting to {port}…")

    def on_connected(self, info):
        self.info = info
        self.port = info.get("_port", self.port)   # the port the probe landed on
        bh = ("0x%08X" % info["buildHash"]) if info.get("buildHash") else "n/a"
        self.lbl_id.setText(
            f"fw v{info['fwVersion']}  build={bh}  "
            f"profile={info['motorProfile']}  PP={info['motorPolePairs']}  "
            f"{'FOC' if info.get('isFoc') else '6-step'}  "
            f"PWM={info['pwmFrequency']:,}Hz  cap={info['maxErpm']:,}")
        self.btn_record.setEnabled(True)
        self.btn_diag.setEnabled(True)
        self._log(f"connected: fw v{info['fwVersion']} build={bh} "
                  f"profile={info['motorProfile']} {'FOC' if info.get('isFoc') else '6-step'}")
        self.status.showMessage(f"Connected ({self.port}).")

    def on_failed(self, msg):
        self.status.showMessage(f"Error: {msg}")
        QtWidgets.QMessageBox.warning(self, "Connection", msg)
        self.worker = None
        self.btn_connect.setText("Connect")
        self.btn_record.setEnabled(False)

    # ── parameters / live tuning ────────────────────────────────────────
    def on_params(self, params):
        self.params = params
        self._populating = True
        rows = sorted(params.items(), key=lambda kv: (kv[1].get("group", 99), kv[0]))
        self.tbl.setRowCount(len(rows))
        self._row_name = {}
        for r, (name, p) in enumerate(rows):
            self._row_name[r] = name
            self._fill_tune_row(r, name, p)
        self._populating = False
        self._apply_tune_filter(self.tune_filter.text() if hasattr(self, "tune_filter") else "")

    def _fill_tune_row(self, r, name, p):
        """Populate one tuner row. Caller must set self._populating around bulk fills."""
        val, mn, mx = p.get("value"), p.get("min"), p.get("max")
        grp = GROUP_NAMES.get(p.get("group"), "")
        est = self._est_current(name, val) if val is not None else None
        note = grp if est is None else f"{grp} · ~{est:.0f}A"

        name_it = QtWidgets.QTableWidgetItem(name)
        name_it.setFlags(name_it.flags() & ~QtCore.Qt.ItemIsEditable)
        val_it = QtWidgets.QTableWidgetItem("" if val is None else str(val))
        val_it.setTextAlignment(QtCore.Qt.AlignCenter)
        mn_it = QtWidgets.QTableWidgetItem("" if mn is None else str(mn))
        mn_it.setFlags(mn_it.flags() & ~QtCore.Qt.ItemIsEditable)
        mn_it.setForeground(QtGui.QColor("#777"))
        mx_it = QtWidgets.QTableWidgetItem("" if mx is None else str(mx))
        mx_it.setFlags(mx_it.flags() & ~QtCore.Qt.ItemIsEditable)
        mx_it.setForeground(QtGui.QColor("#777"))
        note_it = QtWidgets.QTableWidgetItem(note)
        note_it.setFlags(note_it.flags() & ~QtCore.Qt.ItemIsEditable)
        note_it.setForeground(QtGui.QColor("#9e9e9e"))

        warn = self._param_warn(name, val)
        if warn:
            val_it.setForeground(QtGui.QColor("#ef5350"))
            val_it.setToolTip(warn)
        for col, it in enumerate((name_it, val_it, mn_it, mx_it, note_it)):
            self.tbl.setItem(r, col, it)

    def _est_current(self, name, val):
        """Estimated peak current (A) a startup amplitude/duty value implies, else None.
        peak ≈ frac × Vbus / R_phase-to-phase ; R_pp ≈ 2 × focRsMilliOhm/1000."""
        if val is None:
            return None
        rs = self.params.get("focRsMilliOhm", {}).get("value")
        if not rs:
            return None
        r_pp = 2.0 * rs / 1000.0
        if r_pp <= 0:
            return None
        vbus = self._last_vbus or 24.0
        if name in AMPLITUDE_PARAMS:
            frac = val / 200.0
        elif name in DUTY_PARAMS:
            frac = val / 100.0
        else:
            return None
        return frac * vbus / r_pp

    def _param_warn(self, name, val):
        if val is None:
            return None
        p = self.params.get(name, {})
        mn, mx = p.get("min"), p.get("max")
        if mn is not None and mx is not None and not (mn <= val <= mx):
            return (f"outside [{mn},{mx}] — profileDefaults bypasses SET_PARAM limits, "
                    "so the board may have shipped a value the SET_PARAM path will reject")
        est = self._est_current(name, val)
        if est is not None:
            ocsw = self.params.get("ocSwLimitMa", {}).get("value")
            if ocsw and est > ocsw / 1000.0:
                return (f"~{est:.0f}A estimated peak > OC soft limit {ocsw/1000.0:.0f}A "
                        "— likely to trip OC_SW")
        return None

    def _on_tune_edit(self, item):
        if self._populating or item.column() != 1:
            return
        r = item.row()
        name = self._row_name.get(r)
        if not name:
            return
        p = self.params.get(name, {})
        txt = item.text().strip()
        try:
            value = int(txt)
        except ValueError:
            self._log(f"tune: '{txt}' is not an integer")
            self._restore_cell(r, name)
            return
        if not self.worker:
            self._log("tune: not connected")
            self._restore_cell(r, name)
            return
        # OC-safety pre-check — warn, allow override
        est = self._est_current(name, value)
        ocsw = self.params.get("ocSwLimitMa", {}).get("value")
        if est is not None and ocsw and est > ocsw / 1000.0:
            ret = QtWidgets.QMessageBox.warning(
                self, "Over-current risk",
                f"{name} = {value} ≈ {est:.0f} A peak, above the OC soft limit "
                f"{ocsw/1000.0:.0f} A.\nThis is likely to trip OC_SW. Push it anyway?",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No)
            if ret != QtWidgets.QMessageBox.Yes:
                self._restore_cell(r, name)
                return
        self._log(f"tune → {name} = {value} (pushing…)")
        self.worker.submit("set_param", pid=p.get("id"), name=name, value=value)

    def on_param_written(self, res):
        if not res.get("ok"):
            self._log(f"tune ✗ {res.get('name')}: {res.get('error')}")
            if self.worker:
                self.worker.submit("refresh_params")     # restore truth
            return
        name, val = res.get("name"), res.get("value")
        self._log(f"tune ✓ {name} = {val}")
        if name in self.params:
            self.params[name]["value"] = val
        # re-fill just that row (value text + warning colour + est.)
        for r, n in self._row_name.items():
            if n == name:
                self._populating = True
                self._fill_tune_row(r, name, self.params[name])
                self._populating = False
                break

    def _restore_cell(self, r, name):
        self._populating = True
        v = self.params.get(name, {}).get("value")
        it = self.tbl.item(r, 1)
        if it is not None:
            it.setText("" if v is None else str(v))
        self._populating = False

    def _reload_params(self):
        if self.worker:
            self._log("reloading params from board…")
            self.worker.submit("refresh_params")
        else:
            self._log("reload: not connected")

    def _apply_tune_filter(self, text):
        text = (text or "").lower()
        for r, name in self._row_name.items():
            grp = GROUP_NAMES.get(self.params.get(name, {}).get("group"), "").lower()
            self.tbl.setRowHidden(r, bool(text) and text not in name.lower() and text not in grp)

    def _export_profile_c(self):
        if not self.params:
            self._log("export: no params loaded"); return
        prof = self.info.get("motorProfile", "?")
        ts = time.strftime("%Y%m%d_%H%M%S")
        lines = [f"/* Garuda Studio live-param export — {time.strftime('%Y-%m-%d %H:%M:%S')}",
                 f" * fw v{self.info.get('fwVersion','?')} profile {prof} "
                 f"build {('0x%08X' % self.info['buildHash']) if self.info.get('buildHash') else 'n/a'}",
                 " * Field names follow GSP param names = GSP_PARAMS_T members. Reconcile",
                 " * with the [GSP_PROFILE_*] block in gsp/gsp_params.c. */",
                 "{"]
        for name in sorted(self.params):
            p = self.params[name]
            v = p.get("value")
            if v is None:
                continue
            warn = self._param_warn(name, v)
            tag = f"   // ⚠ {warn}" if warn else ""
            lines.append(f"    .{name:<22} = {v},{tag}")
        lines.append("},")
        text = "\n".join(lines)

        os.makedirs("sessions", exist_ok=True)
        path = os.path.join("sessions", f"profile_export_p{prof}_{ts}.c")
        with open(path, "w") as f:
            f.write(text + "\n")
        self._log(f"exported {path}")
        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle(f"C profile export → {path}")
        dlg.resize(620, 520)
        dv = QtWidgets.QVBoxLayout(dlg)
        te = QtWidgets.QPlainTextEdit(text)
        te.setReadOnly(True)
        te.setStyleSheet("font-family:monospace;font-size:11px;")
        dv.addWidget(te)
        bb = QtWidgets.QHBoxLayout()
        btn_copy = QtWidgets.QPushButton("Copy")
        btn_copy.clicked.connect(lambda: QtWidgets.QApplication.clipboard().setText(text))
        btn_close = QtWidgets.QPushButton("Close")
        btn_close.clicked.connect(dlg.accept)
        bb.addStretch(1); bb.addWidget(btn_copy); bb.addWidget(btn_close)
        dv.addLayout(bb)
        dlg.exec()

    # ── live data ───────────────────────────────────────────────────────
    def on_snapshot(self, s):
        if self.t0 is None:
            self.t0 = time.monotonic()
        t = time.monotonic() - self.t0
        self._last_vbus = s.get("vbus_V")
        self.statebar.set_state(s["state_name"], s["fault"] != 0)
        if s["fault"]:
            self.fault.setText(f"⚠  FAULT: {s['fault_name']}")
            self.fault.setStyleSheet("color:#fff;background:#b71c1c;"
                                     "font-weight:bold;padding:4px;")
        else:
            self.fault.setText("")
            self.fault.setStyleSheet("")

        rej_tot = s["hwzc_zc"] + s["hwzc_reject"]
        if self._prev_rej is not None:
            d_acc = max(0, s["hwzc_zc"] - self._prev_rej[0])
            d_rej = max(0, s["hwzc_reject"] - self._prev_rej[1])
            denom = d_acc + d_rej
            rejrate = (100.0 * d_rej / denom) if denom > 0 else 0.0
        else:
            rejrate = (100.0 * s["hwzc_reject"] / rej_tot) if rej_tot else 0.0
        rejrate = max(0.0, min(100.0, rejrate))   # can't exceed 100%
        self._prev_rej = (s["hwzc_zc"], s["hwzc_reject"])

        ia = s.get("ia_pk_mag", 0.0)
        ibus = s.get("ibus_win_A", s["ibus_A"])   # windowed = trustworthy (not valley artifact)
        self.ro["eRPM"].set(f"{s['eRPM']:,}")
        self.ro["duty"].set(s["duty"])
        self.ro["vbus"].set(f"{s['vbus_V']:.1f}", "#ef5350" if s["vbus_V"] > 30 else None)
        self.ro["ibus"].set(f"{ibus:.1f}")
        self.ro["thr"].set(s["throttle"])
        self.ro["iapk"].set(f"{ia:.1f}", "#ef5350" if ia > 18 else None)
        self.ro["hwzc"].set(f"{rejrate:.0f}",
                            "#ef5350" if rejrate > 80 else ("#ffa726" if rejrate > 50 else None))
        cpu = s.get("cpu_load_pct", 0.0)
        self.ro["cpu"].set(f"{cpu:.0f}",
                           "#ef5350" if cpu > 85 else ("#ffa726" if cpu > 60 else "#66bb6a"))
        self.sector_box.update_from(s)

        raw = {"eRPM": s["eRPM"], "ia": ia, "ibus": ibus, "cpu": cpu,
               "rejrate": rejrate, "vbus": s["vbus_V"],
               "bemf": s.get("bemf_raw", 0), "zcthr": s.get("zc_thresh", 0),
               "goodzc": s.get("good_zc", 0),
               "duty": s["duty"], "throttle": s["throttle"]}
        self.buf["t"].append(t)
        for k, vv in raw.items():
            self.buf[k].append(vv)
        ts = list(self.buf["t"])
        for key, label, color, scale, bip, fmt, _on in SIGNALS:
            cb = self.checks[key]
            cb.setText(f"{label}  {fmt(raw[key])}")     # selector doubles as live readout
            if cb.isChecked() and not self.scope_frozen:
                if key == "zcthr" and self.cb_zclink.isChecked():
                    # gang to BEMF so the threshold sits on the BEMF (same signal)
                    sdiv = self.ch_scale["bemf"].value() or 1.0
                    pos = self.ch_pos["bemf"].value()
                else:
                    sdiv = self.ch_scale[key].value() or 1.0
                    pos = self.ch_pos[key].value()
                self.curves[key].setData(ts, [vv / sdiv + pos for vv in self.buf[key]])
        if ts and not self.scope_frozen:
            self.p_main.setXRange(max(0.0, ts[-1] - self.scope_window_s), ts[-1], padding=0)
        if s["eRPM"] > 100:
            self.map_pts.append((s["eRPM"], ia))
            self.scatter_map.setData([p[0] for p in self.map_pts],
                                     [p[1] for p in self.map_pts])

        s2 = dict(s); s2["t"] = t
        self.live.append(s2)
        if self.recording and self.session is not None:
            self.session.add(s2)

        # Console mirror is RUN-ONLY: idle is silent (the continuous idle stream is
        # a waste). Stream from start (leaving IDLE) to stop (back to IDLE), with
        # banners. FAULT still streams so a desync is visible.
        running = s["state_name"] != "IDLE"
        # Auto-record every run to a CSV bundle (with per-sector miss + CPU) so
        # each run is analyzable without manually toggling Record. Starts on
        # leaving IDLE, saves on return to IDLE. Skipped while manually
        # recording (that path saves its own bundle).
        if (running and not self._was_running
                and self._auto_session is None and not self.recording):
            self._auto_session = Session(
                info=self.info, params=self.params,
                host={"os": platform.system()}, tool_version=GSP_VER,
                started_at=time.strftime("%Y-%m-%dT%H:%M:%S"))
        if running and self._auto_session is not None:
            self._auto_session.add(s2)
        if (not running and self._was_running
                and self._auto_session is not None):
            name = f"sessions/gui_auto_{time.strftime('%Y%m%d_%H%M%S')}"
            try:
                self._auto_session.save(name)
                self._log(f"💾 auto-saved {name}/telemetry.csv "
                          f"({len(self._auto_session.samples)} samples)")
            except Exception as e:
                self._log(f"auto-save failed: {e}")
            self._auto_session = None

        if not self._console_paused:
            if running and not self._was_running:
                self._log(f"▶──────── RUN START @ {t:6.2f}s ────────")
            if running:
                mark = " ◀" if (self.live and len(self.live) >= 2 and
                                self.live[-2]["state_name"] != s["state_name"]) else ""
                self._log(f"{t:6.2f} {s['state_name']:<8} thr={s['throttle']:>4} "
                          f"duty={s['duty']:>3}% eRPM={s['eRPM']:>7,} Vbus={s['vbus_V']:4.1f} "
                          f"Ibus={ibus:+5.1f} Ia={ia:4.1f} rej={rejrate:3.0f}% "
                          f"{s['fault_name']}{mark}")
            elif self._was_running:
                self._log(f"■──────── STOPPED @ {t:6.2f}s ────────")
        self._was_running = running

    # ── recording ───────────────────────────────────────────────────────
    def toggle_record(self):
        if not self.recording:
            self.session = Session(
                info=self.info, params=self.params,
                host={"os": platform.system()}, tool_version=GSP_VER,
                started_at=time.strftime("%Y-%m-%dT%H:%M:%S"))
            self.recording = True
            self.btn_record.setText("■ Stop")
            self.status.showMessage("Recording…")
        else:
            self.recording = False
            self.btn_record.setText("● Record")
            name = f"sessions/gui_{time.strftime('%Y%m%d_%H%M%S')}"
            self.session.save(name)
            self.status.showMessage(
                f"Saved bundle: {name}  ({len(self.session.samples)} samples)")
            QtWidgets.QMessageBox.information(self, "Recorded", f"Saved {name}")

    def _refresh_curve_visibility(self):
        for key, cb in self.checks.items():
            self.curves[key].setVisible(cb.isChecked())

    # ── console + diagnosis ─────────────────────────────────────────────
    def _log(self, line):
        self.console.appendPlainText(line)

    def _on_command(self):
        cmd = self.cmd.text().strip()
        self.cmd.clear()
        if not cmd:
            return
        self._log(f"> {cmd}")
        parts = cmd.split()
        op = parts[0].lower()
        if op in ("help", "?"):
            self._log("commands: help · clear · pause · resume · mark <text> · "
                      "diagnose · params · get <param> · set <param> <value> · "
                      "export · reload · save")
        elif op == "clear":
            self.console.clear()
        elif op == "pause":
            self._console_paused = True; self._log("(telemetry mirror paused)")
        elif op == "resume":
            self._console_paused = False; self._log("(telemetry mirror resumed)")
        elif op == "mark":
            self._log("── MARK: " + cmd[4:].strip() + " ──")
        elif op == "params":
            for name in sorted(self.params):
                p = self.params[name]
                flag = "" if p.get("min") is None or p["min"] <= (p.get("value") or 0) <= p["max"] else "  ⚠ out-of-range"
                self._log(f"  {name:24} = {p.get('value')}{flag}")
        elif op == "get":
            if len(parts) < 2 or parts[1] not in self.params:
                self._log("usage: get <paramName>"); return
            self._log(f"  {parts[1]} = {self.params[parts[1]].get('value')}")
        elif op == "set":
            if len(parts) < 3:
                self._log("usage: set <paramName> <value>"); return
            name = parts[1]
            if name not in self.params:
                self._log(f"unknown param: {name}"); return
            try:
                value = int(parts[2])
            except ValueError:
                self._log(f"'{parts[2]}' is not an integer"); return
            if not self.worker:
                self._log("set: not connected"); return
            self._log(f"tune → {name} = {value} (pushing…)")
            self.worker.submit("set_param", pid=self.params[name].get("id"),
                               name=name, value=value)
        elif op == "export":
            self._export_profile_c()
        elif op == "reload":
            self._reload_params()
        elif op == "diagnose":
            self.run_diagnosis()
        elif op == "save":
            self.toggle_record()
        else:
            self._log(f"unknown command: {op}  (try 'help')")

    def run_diagnosis(self):
        if not self.live:
            self._log("diagnose: no telemetry yet"); return
        engine = "claude-opus-4-8" if DIAG.claude_available() else "local rules"
        self._log(f"── running diagnosis ({engine}) …")
        QtWidgets.QApplication.processEvents()
        sess = Session(info=self.info, params=self.params)
        for x in self.live:
            sess.add(x)
        try:
            r = DIAG.diagnose(sess)
        except Exception as e:  # noqa
            self._log(f"diagnosis error: {e}"); return
        self._render_diag(r)

    def _render_diag(self, r):
        sev = r.get("severity", "?").upper()
        self._log(f"╔═ DIAGNOSIS [{sev}]  via {r.get('engine','?')}")
        self._log(f"║ {r.get('summary','')}")
        for f in r.get("findings", []):
            self._log("║")
            self._log(f"║ • [{f.get('confidence','?')}] {f.get('cause','')}")
            self._log(f"║   evidence: {f.get('evidence','')}")
            for fx in f.get("fixes", []):
                self._log(f"║   → {fx.get('param','')}: {fx.get('current','?')} "
                          f"⇒ {fx.get('suggested','')}   ({fx.get('why','')})")
        if not r.get("findings"):
            self._log("║ no issues found")
        self._log("╚" + "═" * 40)

    def closeEvent(self, e):
        if self.worker:
            self.worker.stop()
        e.accept()


def main():
    ap = argparse.ArgumentParser(description="Garuda Studio GUI")
    ap.add_argument("--port", default=None)
    args = ap.parse_args()
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    pal = QtGui.QPalette()
    pal.setColor(QtGui.QPalette.Window, QtGui.QColor("#181c20"))
    pal.setColor(QtGui.QPalette.Base, QtGui.QColor("#101418"))
    pal.setColor(QtGui.QPalette.Text, QtGui.QColor("#dddddd"))
    pal.setColor(QtGui.QPalette.WindowText, QtGui.QColor("#dddddd"))
    pal.setColor(QtGui.QPalette.Button, QtGui.QColor("#2a2f35"))
    pal.setColor(QtGui.QPalette.ButtonText, QtGui.QColor("#ffffff"))
    app.setPalette(pal)
    w = MainWindow(args.port)
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
