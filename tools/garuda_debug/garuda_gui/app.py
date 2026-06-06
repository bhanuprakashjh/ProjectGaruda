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
import pyqtgraph as pg

from garuda_gsp import GspClient, GspError, Session, __version__ as GSP_VER
from garuda_gsp import protocol as P
from . import diagnose as DIAG
from . import wizard as WIZ

STATES = ["IDLE", "ARMED", "ALIGN", "OL_RAMP", "MORPH", "CL"]
WINDOW_S = 20.0          # rolling plot window
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
]

# Params whose value implies a startup/standing current = (modPct or duty%) of Vbus
# over phase-to-phase resistance. Used for the OC-safety estimate in the tuner.
AMPLITUDE_PARAMS = {"sineAlignModPct", "sineRampModPct"}     # peak = pct/200
DUTY_PARAMS = {"rampDutyPct", "alignDutyPct", "clIdleDutyPct"}  # = pct/100


# ─────────────────────────────────────────────────────────────────────────
class SerialWorker(QtCore.QThread):
    """Owns the GspClient on a background thread so the UI never blocks.
    UI-thread code must NEVER touch the port directly — it enqueues actions via
    submit(); the worker drains them between snapshot polls (one owner, no race)."""
    connected = QtCore.Signal(dict)
    params_ready = QtCore.Signal(dict)
    snapshot = QtCore.Signal(dict)
    param_written = QtCore.Signal(dict)   # {ok, pid, name, value} | {ok:False, ..., error}
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
            except Exception as e:  # noqa
                self.param_written.emit({"ok": False, "pid": kw.get("pid"),
                                         "name": kw.get("name", ""), "error": str(e)})

    def run(self):
        try:
            c = GspClient(self.port)
            if not c.ping():
                self.failed.emit("no PING response (port / baud / power?)")
                return
            info = c.get_info()
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
            self.failed.emit(str(e))
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


# ─────────────────────────────────────────────────────────────────────────
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
        self.t0 = None
        self._last_vbus = None
        self._populating = False
        self._row_name = {}
        # one buffer per selectable channel (+ time)
        self.buf = {k: deque(maxlen=int(WINDOW_S * POLL_HZ)) for k in
                    (["t"] + [s[0] for s in SIGNALS])}
        self.map_pts = deque(maxlen=2000)        # (eRPM, Ia_pk) operating-point scatter
        self.live = deque(maxlen=4000)           # rolling samples for on-demand diagnosis
        self._prev_rej = None                    # (zc, reject) for windowed reject-rate
        self._console_paused = False

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
            "hwzc": Readout("HWZC rej", "%"),
        }
        for x in self.ro.values():
            rr.addWidget(x)
        v.addLayout(rr)

        # one scope: channel-selector column + single normalized plot + X-Y map
        prow = QtWidgets.QHBoxLayout()
        sel = QtWidgets.QWidget()
        selv = QtWidgets.QVBoxLayout(sel)
        selv.setContentsMargins(2, 2, 2, 2)
        selv.addWidget(QtWidgets.QLabel("Channels"))
        sel.setFixedWidth(150)
        self.checks = {}
        self.sigcfg = {s[0]: s for s in SIGNALS}
        for key, label, color, _scale, _bip, _fmt, on in SIGNALS:
            cb = QtWidgets.QCheckBox(label)
            cb.setChecked(on)
            cb.setStyleSheet(f"color:{color}; font-weight:bold;")
            cb.stateChanged.connect(self._refresh_curve_visibility)
            selv.addWidget(cb)
            self.checks[key] = cb
        selv.addStretch(1)
        prow.addWidget(sel)

        self.p_main = pg.PlotWidget(title="Signals (normalized — live values in the selectors)")
        self.p_main.showGrid(x=True, y=True, alpha=0.2)
        self.p_main.enableAutoRange(axis="y", enable=True)
        self.p_main.getAxis("left").setStyle(showValues=False)
        self.curves = {}
        for key, label, color, _s, _b, _f, _on in SIGNALS:
            self.curves[key] = self.p_main.plot(pen=pg.mkPen(color, width=2))
        prow.addWidget(self.p_main, 4)

        self.p_map = pg.PlotWidget(title="Operating map: Ia_pk (A) vs eRPM")
        self.p_map.showGrid(x=True, y=True, alpha=0.2)
        self.p_map.enableAutoRange(enable=True)
        self.scatter_map = pg.ScatterPlotItem(size=4, pen=None,
                                              brush=pg.mkBrush(66, 165, 245, 90))
        self.p_map.addItem(self.scatter_map)
        prow.addWidget(self.p_map, 1)

        pw2 = QtWidgets.QWidget(); pw2.setLayout(prow)
        v.addWidget(pw2, 1)
        self._refresh_curve_visibility()
        return w

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
        from garuda_gsp import list_ports_human
        self.portbox.clear()
        for dev, desc in list_ports_human():
            self.portbox.addItem(f"{dev}  ({desc})", dev)
        if self.port:
            self.portbox.insertItem(0, self.port, self.port)
            self.portbox.setCurrentIndex(0)

    def _selected_port(self):
        data = self.portbox.currentData()
        if data:
            return data
        return self.portbox.currentText().split()[0]

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
        self.worker.params_ready.connect(self.on_params)
        self.worker.snapshot.connect(self.on_snapshot)
        self.worker.param_written.connect(self.on_param_written)
        self.worker.failed.connect(self.on_failed)
        self.worker.start()
        self.btn_connect.setText("Disconnect")
        self.status.showMessage(f"Connecting to {port}…")

    def on_connected(self, info):
        self.info = info
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
            d_acc = s["hwzc_zc"] - self._prev_rej[0]
            d_rej = s["hwzc_reject"] - self._prev_rej[1]
            denom = d_acc + d_rej
            rejrate = (100.0 * d_rej / denom) if denom > 0 else 0.0
        else:
            rejrate = (100.0 * s["hwzc_reject"] / rej_tot) if rej_tot else 0.0
        self._prev_rej = (s["hwzc_zc"], s["hwzc_reject"])

        ia = s.get("ia_pk_mag", 0.0)
        self.ro["eRPM"].set(f"{s['eRPM']:,}")
        self.ro["duty"].set(s["duty"])
        self.ro["vbus"].set(f"{s['vbus_V']:.1f}", "#ef5350" if s["vbus_V"] > 30 else None)
        self.ro["ibus"].set(f"{s['ibus_A']:.1f}")
        self.ro["thr"].set(s["throttle"])
        self.ro["iapk"].set(f"{ia:.1f}", "#ef5350" if ia > 18 else None)
        self.ro["hwzc"].set(f"{rejrate:.0f}",
                            "#ef5350" if rejrate > 80 else ("#ffa726" if rejrate > 50 else None))

        raw = {"eRPM": s["eRPM"], "ia": ia, "ibus": s["ibus_A"],
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
            cb.setText(f"{label}  {fmt(raw[key])}")
            if cb.isChecked():
                vals = self.buf[key]
                if bip:
                    norm = [0.5 + (x / (2.0 * scale)) for x in vals]
                else:
                    norm = [x / scale for x in vals]
                self.curves[key].setData(ts, norm)
        if s["eRPM"] > 100:
            self.map_pts.append((s["eRPM"], ia))
            self.scatter_map.setData([p[0] for p in self.map_pts],
                                     [p[1] for p in self.map_pts])

        s2 = dict(s); s2["t"] = t
        self.live.append(s2)
        if self.recording and self.session is not None:
            self.session.add(s2)

        if not self._console_paused:
            mark = " ◀" if (self.live and len(self.live) >= 2 and
                            self.live[-2]["state_name"] != s["state_name"]) else ""
            self._log(f"{t:6.2f} {s['state_name']:<8} thr={s['throttle']:>4} "
                      f"duty={s['duty']:>3}% eRPM={s['eRPM']:>7,} Vbus={s['vbus_V']:4.1f} "
                      f"Ibus={s['ibus_A']:+5.1f} Ia={ia:4.1f} rej={rejrate:3.0f}% "
                      f"{s['fault_name']}{mark}")

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
