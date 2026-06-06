#!/usr/bin/env python3
"""
Garuda Debug GUI (v1) — live, monitor-focused.

Connects over GSP, shows the state machine, live readouts, real-time plots and the
parameter table. Read-only/safe: NO motor-start or param-write in v1 (those land
later, behind guards). Record button saves a shareable session bundle.

    garuda-gui            # auto-detect port
    garuda-gui --port /dev/ttyACM0
"""
import argparse
import platform
import sys
import time
from collections import deque

from PySide6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

from garuda_gsp import GspClient, GspError, Session, __version__ as GSP_VER
from garuda_gsp import protocol as P
from . import diagnose as DIAG

STATES = ["IDLE", "ARMED", "ALIGN", "OL_RAMP", "MORPH", "CL"]
WINDOW_S = 20.0          # rolling plot window
POLL_HZ = 30

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


# ─────────────────────────────────────────────────────────────────────────
class SerialWorker(QtCore.QThread):
    """Owns the GspClient on a background thread so the UI never blocks."""
    connected = QtCore.Signal(dict)
    params_ready = QtCore.Signal(dict)
    snapshot = QtCore.Signal(dict)
    failed = QtCore.Signal(str)
    stopped = QtCore.Signal()

    def __init__(self, port):
        super().__init__()
        self.port = port
        self._run = True

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
        self.setWindowTitle("Garuda Debug — v1")
        self.resize(1180, 720)
        self.port = port
        self.worker = None
        self.info = {}
        self.params = {}
        self.recording = False
        self.session = None
        self.t0 = None
        # one buffer per selectable channel (+ time)
        self.buf = {k: deque(maxlen=int(WINDOW_S * POLL_HZ)) for k in
                    (["t"] + [s[0] for s in SIGNALS])}
        self.map_pts = deque(maxlen=2000)        # (eRPM, Ibus) operating-point scatter
        self.live = deque(maxlen=4000)           # rolling samples for on-demand diagnosis
        self._prev_rej = None                    # (t, hwzc_reject) for reject-rate
        self._console_paused = False

        pg.setConfigOptions(antialias=True, background="#101418", foreground="#ccc")
        self._build()

    # ── UI ──────────────────────────────────────────────────────────────
    def _build(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)

        # top bar
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

        # state bar + fault banner
        self.statebar = StateBar()
        root.addWidget(self.statebar)
        self.fault = QtWidgets.QLabel("")
        self.fault.setAlignment(QtCore.Qt.AlignCenter)
        self.fault.setStyleSheet("font-weight:bold;")
        root.addWidget(self.fault)

        # readouts
        rr = QtWidgets.QHBoxLayout()
        self.ro = {
            "eRPM": Readout("eRPM"), "duty": Readout("Duty", "%"),
            "vbus": Readout("Vbus", " V"), "ibus": Readout("Ibus", " A"),
            "thr": Readout("Throttle"), "iapk": Readout("Ia peak", " A"),
            "hwzc": Readout("HWZC rej", "%"),
        }
        for w in self.ro.values():
            rr.addWidget(w)
        root.addLayout(rr)

        # plots + param table
        split = QtWidgets.QSplitter()
        plots = QtWidgets.QWidget()
        pv = QtWidgets.QVBoxLayout(plots)
        pv.setContentsMargins(0, 0, 0, 0)
        # One scope: a channel-selector column + a single normalized plot + the
        # X-Y operating map (kept separate — it doesn't share the time axis).
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
        self.p_main.enableAutoRange(axis="y", enable=True)   # auto-scale Y to active band
        self.p_main.getAxis("left").setStyle(showValues=False)
        self.curves = {}
        for key, label, color, _s, _b, _f, _on in SIGNALS:
            self.curves[key] = self.p_main.plot(pen=pg.mkPen(color, width=2))
        prow.addWidget(self.p_main, 4)

        self.p_map = pg.PlotWidget(title="Operating map: Ia_pk (A) vs eRPM")
        self.p_map.showGrid(x=True, y=True, alpha=0.2)
        self.p_map.enableAutoRange(enable=True)   # auto-fit the cloud
        self.scatter_map = pg.ScatterPlotItem(size=4, pen=None,
                                              brush=pg.mkBrush(66, 165, 245, 90))
        self.p_map.addItem(self.scatter_map)
        prow.addWidget(self.p_map, 1)

        pw2 = QtWidgets.QWidget(); pw2.setLayout(prow)
        pv.addWidget(pw2)
        self._refresh_curve_visibility()
        split.addWidget(plots)

        self.tbl = QtWidgets.QTableWidget(0, 4)
        self.tbl.setHorizontalHeaderLabels(["Parameter", "Value", "Min", "Max"])
        self.tbl.horizontalHeader().setSectionResizeMode(
            0, QtWidgets.QHeaderView.Stretch)
        self.tbl.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        split.addWidget(self.tbl)
        split.setSizes([760, 420])
        root.addWidget(split, 1)

        # ── embedded terminal/console (telemetry stream + command line) ──
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

    def _plot(self, parent_layout, title, color):
        pw = pg.PlotWidget(title=title)
        pw.showGrid(x=True, y=True, alpha=0.2)
        pw.setMouseEnabled(x=False, y=True)
        parent_layout.addWidget(pw)
        return pw

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

    def on_params(self, params):
        self.params = params
        self.tbl.setRowCount(len(params))
        for r, (name, p) in enumerate(sorted(params.items())):
            val, mn, mx = p.get("value"), p.get("min"), p.get("max")
            out_of_range = (val is not None and mn is not None and mx is not None
                            and not (mn <= val <= mx))
            cells = [name, str(val), str(mn), str(mx)]
            for col, text in enumerate(cells):
                it = QtWidgets.QTableWidgetItem(text)
                if out_of_range and col in (1,):
                    it.setForeground(QtGui.QColor("#ef5350"))
                    it.setToolTip("value is outside descriptor [min,max] "
                                  "(profileDefaults bypasses SET_PARAM limits)")
                self.tbl.setItem(r, col, it)

    # ── live data ───────────────────────────────────────────────────────
    def on_snapshot(self, s):
        if self.t0 is None:
            self.t0 = time.monotonic()
        t = time.monotonic() - self.t0
        self.statebar.set_state(s["state_name"], s["fault"] != 0)
        if s["fault"]:
            self.fault.setText(f"⚠  FAULT: {s['fault_name']}")
            self.fault.setStyleSheet("color:#fff;background:#b71c1c;"
                                     "font-weight:bold;padding:4px;")
        else:
            self.fault.setText("")
            self.fault.setStyleSheet("")

        # windowed HWZC reject rate (leading desync indicator), from deltas
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
        for k, v in raw.items():
            self.buf[k].append(v)
        ts = list(self.buf["t"])
        for key, label, color, scale, bip, fmt, _on in SIGNALS:
            cb = self.checks[key]
            cb.setText(f"{label}  {fmt(raw[key])}")     # selector doubles as live legend
            if cb.isChecked():
                vals = self.buf[key]
                if bip:
                    norm = [0.5 + (x / (2.0 * scale)) for x in vals]
                else:
                    norm = [x / scale for x in vals]
                self.curves[key].setData(ts, norm)
        if s["eRPM"] > 100:                       # operating-point map (skip idle clutter)
            self.map_pts.append((s["eRPM"], ia))  # Ia_pk varies (Ibus is valley-sampled ≈0)
            self.scatter_map.setData([p[0] for p in self.map_pts],
                                     [p[1] for p in self.map_pts])

        s2 = dict(s); s2["t"] = t
        self.live.append(s2)
        if self.recording and self.session is not None:
            self.session.add(s2)

        # mirror to console (step6-style line), unless paused
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
            QtWidgets.QMessageBox.information(self, "Recorded",
                                              f"Saved {name}")

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
                      "diagnose · params · save")
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
            self._log(f"║")
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
    ap = argparse.ArgumentParser(description="Garuda Debug GUI")
    ap.add_argument("--port", default=None)
    args = ap.parse_args()
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    # dark palette
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
