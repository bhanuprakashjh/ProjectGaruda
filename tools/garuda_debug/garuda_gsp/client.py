"""
GspClient — one connection object every tool/GUI uses. Handles port
auto-detection (Linux/Mac/Windows), version negotiation, and all commands.
"""
import struct
import time

import serial
from serial.tools import list_ports

from . import protocol as P
from .framing import build_packet, read_packet, send_cmd
from .decode import decode_info, decode_snapshot


# Known USB-UART bridge vendor IDs (the board's data UART).
#   FTDI 0x0403 · Silicon Labs CP210x 0x10C4 · WCH CH340 0x1A86/0x4348 ·
#   Prolific 0x067B · mbed/CDC 0x0D28
# Microchip (0x04D8) is deliberately NOT auto-trusted: the PKOB on-board
# debugger enumerates a Microchip virtual COM that does NOT speak GSP, so we
# disambiguate by PROBING (sending GET_INFO) rather than trusting the VID.
# USB-UART bridge + on-board-debugger VIDs that commonly carry the GSP UART.
#   FTDI 0x0403 · SiLabs CP210x 0x10C4 · WCH CH340 0x1A86/0x4348 · Prolific
#   0x067B · mbed/CDC 0x0D28 · Microchip 0x04D8 (the PKoB virtual COM — on this
#   board the dsPIC UART is bridged THROUGH the PKoB, so it's a valid GSP port).
_UART_VIDS = {0x0403, 0x10C4, 0x1A86, 0x4348, 0x067B, 0x0D28, 0x04D8}
_BRIDGE_HINTS = ("ftdi", "cp210", "ch340", "ch910", "wch", "silicon labs",
                 "prolific", "usb serial", "usb-serial", "uart", "pkob",
                 "mplab", "curiosity")
# Ports that are essentially never the board — try them last (or skip).
_AVOID_HINTS = ("bluetooth", "standard serial over")


def _score(p):
    dev = (p.device or "").lower()
    desc = " ".join(filter(None, (p.description, p.manufacturer,
                                  getattr(p, "product", None)))).lower()
    vid = getattr(p, "vid", None)
    if any(h in desc for h in _AVOID_HINTS):
        return 8
    if "acm" in dev or "usbmodem" in dev:
        return 0
    if vid in _UART_VIDS:
        return 1
    if vid is not None:                     # any real USB device (probe decides)
        return 2
    if any(h in desc for h in _BRIDGE_HINTS):
        return 3
    if "usb" in dev or "ttyusb" in dev:
        return 4
    return 6                                # legacy ttyS / no VID — last


def candidate_ports():
    """Ranked [(device, description)] most-likely-first. Real USB devices
    (which carry a VID) sort ahead of legacy motherboard ttyS* ports. Works on
    Windows (COMx), Linux (ttyUSB/ACM), macOS."""
    scored = sorted(list_ports.comports(), key=_score)
    return [(p.device or "", p.description or (p.device or "")) for p in scored]


def _default_port() -> str:
    import sys
    if sys.platform.startswith("win"):
        return "COM1"
    if sys.platform == "darwin":
        return "/dev/tty.usbmodem"
    return "/dev/ttyACM0"


def find_port(preferred: str = None) -> str:
    """Best-guess port (no I/O). Use probe_ports() for a verified connection."""
    if preferred:
        return preferred
    cands = candidate_ports()
    return cands[0][0] if cands else _default_port()


def probe_ports(preferred: str = None, baud: int = 115200,
                timeout: float = 0.5, on_try=None) -> str:
    """Open each candidate port and send GET_INFO; return the device that
    actually answers (the real board), or None if none do. This is what makes
    auto-connect reliable on Windows, where COM names carry no hint and the
    PKOB debug COM looks just like the data UART. on_try(dev, desc, ok) is an
    optional progress callback."""
    if preferred:
        cands = [(preferred, preferred)]
    else:
        # Only probe real USB devices (score < 6); skip the dozens of phantom
        # legacy ttyS* ports so we don't stall for ~timeout × 32.
        usb = [p for p in list_ports.comports() if _score(p) < 6]
        cands = [(p.device, p.description or p.device)
                 for p in sorted(usb, key=_score)]
        if not cands:                       # nothing USB — fall back to all
            cands = candidate_ports()
    for dev, desc in cands:
        ok = False
        try:
            # exclusive=True so a port already held by another app errors out
            # immediately instead of blocking the probe.
            with serial.Serial(dev, baud, timeout=timeout, exclusive=True) as ser:
                # leave DTR/RTS at default (asserted) — the PKoB bridge needs it
                time.sleep(0.15)            # Windows settle after open
                ser.reset_input_buffer()
                for _ in range(2):          # first cmd is often dropped on Windows
                    resp = send_cmd(ser, P.CMD_GET_INFO, b"", timeout=timeout)
                    if resp is not None and resp[0] == P.CMD_GET_INFO:
                        ok = True
                        break
                    ser.reset_input_buffer()
        except Exception:
            ok = False
        if on_try:
            on_try(dev, desc, ok)
        if ok:
            return dev
    return None


def list_ports_human():
    return [(p.device, p.description) for p in list_ports.comports()]


class GspError(Exception):
    pass


class GspClient:
    def __init__(self, port: str = None, baud: int = 115200, timeout: float = 1.0,
                 probe: bool = True):
        # If no port was given, PROBE for the one that actually answers GET_INFO
        # (reliable on Windows); fall back to the best guess if none reply.
        if port:
            self.port = port
        elif probe:
            self.port = probe_ports(baud=baud) or find_port(None)
        else:
            self.port = find_port(None)
        self.baud = baud
        # Open like the known-good path: DO NOT touch DTR/RTS — the PKoB/USB-UART
        # bridge needs the default (asserted) line state to pass data; forcing
        # them low kills the link. write_timeout so a hung/unplugged board raises
        # instead of blocking the worker thread forever (the QThread warning).
        self.ser = serial.Serial(self.port, baud, timeout=timeout,
                                 write_timeout=max(timeout, 1.0))
        # Windows USB-serial needs a moment after open before the first byte is
        # reliable, and the first TX/RX often gets dropped — settle and flush so
        # the handshake doesn't fail on the first command (Linux cdc_acm is fine
        # without this, but it's harmless there).
        time.sleep(0.15)
        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except Exception:
            pass
        self.info = None            # cached GET_INFO
        self.protocol_version = None

    # ── lifecycle ───────────────────────────────────────────────────────
    def close(self):
        try:
            self.telem_stop()
        except Exception:
            pass
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()

    # ── core ────────────────────────────────────────────────────────────
    def _cmd(self, cmd_id, payload=b"", timeout=1.0, expect=None):
        resp = send_cmd(self.ser, cmd_id, payload, timeout=timeout)
        if resp is None:
            raise GspError(f"no response to cmd 0x{cmd_id:02X}")
        rcmd, rpl = resp
        if rcmd == P.CMD_ERROR and cmd_id != P.CMD_ERROR:
            code = rpl[0] if rpl else 0
            raise GspError(f"firmware error {P.ERR_NAMES.get(code, code)} "
                           f"for cmd 0x{cmd_id:02X}")
        if expect is not None and rcmd != expect:
            raise GspError(f"expected 0x{expect:02X}, got 0x{rcmd:02X}")
        return rpl

    def ping(self) -> bool:
        try:
            self._cmd(P.CMD_PING, expect=P.CMD_PING)
            return True
        except GspError:
            return False

    def get_info(self) -> dict:
        rpl = self._cmd(P.CMD_GET_INFO, expect=P.CMD_GET_INFO)
        self.info = decode_info(rpl)
        self.protocol_version = self.info.get("protocolVersion")
        return self.info

    def connect(self, retries: int = 5, delay: float = 0.15) -> dict:
        """Handshake with retry. The first GET_INFO after open is frequently
        dropped on Windows USB-serial, so we flush and retry a few times before
        giving up. Returns the info dict or raises the last GspError."""
        last = None
        for _ in range(max(1, retries)):
            try:
                return self.get_info()
            except GspError as e:
                last = e
                try:
                    self.ser.reset_input_buffer()
                except Exception:
                    pass
                time.sleep(delay)
        raise last or GspError("no response")

    def get_snapshot(self) -> dict:
        rpl = self._cmd(P.CMD_GET_SNAPSHOT, expect=P.CMD_GET_SNAPSHOT)
        return decode_snapshot(rpl, t=time.monotonic())

    # ── parameters ──────────────────────────────────────────────────────
    def get_param(self, param) -> int:
        pid = P.PARAM_IDS[param] if isinstance(param, str) else param
        rpl = self._cmd(P.CMD_GET_PARAM, struct.pack("<H", pid), expect=P.CMD_GET_PARAM)
        _id, val = struct.unpack_from("<HI", rpl, 0)
        return val

    def set_param(self, param, value: int):
        pid = P.PARAM_IDS[param] if isinstance(param, str) else param
        self._cmd(P.CMD_SET_PARAM, struct.pack("<HI", pid, value), expect=P.CMD_SET_PARAM)

    def get_param_list(self) -> list:
        """Page through GET_PARAM_LIST. Returns [{id,name,type,group,min,max}]."""
        out, start = [], 0
        while True:
            rpl = self._cmd(P.CMD_GET_PARAM_LIST, bytes([start]),
                            expect=P.CMD_GET_PARAM_LIST)
            if len(rpl) < 3:
                break
            # header: [totalCount][startIndex(echo)][entryCount]
            total, _start_echo, count = rpl[0], rpl[1], rpl[2]
            off = 3
            for _ in range(count):
                pid, ptype, grp = struct.unpack_from("<HBB", rpl, off)
                mn, mx = struct.unpack_from("<II", rpl, off + 4)
                off += 12
                out.append({"id": pid, "name": P.PARAM_NAMES.get(pid, f"0x{pid:02X}"),
                            "type": ptype, "group": grp, "min": mn, "max": mx})
            start += count
            if count == 0 or start >= total:
                break
        return out

    def dump_params(self) -> dict:
        """{name: {id, value, min, max}} — full readable param state."""
        descs = self.get_param_list()
        result = {}
        for d in descs:
            try:
                v = self.get_param(d["id"])
            except GspError:
                v = None
            result[d["name"]] = {"id": d["id"], "value": v,
                                 "min": d["min"], "max": d["max"],
                                 "type": d["type"], "group": d["group"]}
        return result

    def save_config(self):
        self._cmd(P.CMD_SAVE_CONFIG)

    def load_profile(self, profile_id: int):
        self._cmd(P.CMD_LOAD_PROFILE, bytes([profile_id]))

    # ── motor control (remote drive — used by the auto-tuner) ────────────
    def start_motor(self):
        self._cmd(P.CMD_START_MOTOR)

    def stop_motor(self):
        self._cmd(P.CMD_STOP_MOTOR)

    def clear_fault(self):
        self._cmd(P.CMD_CLEAR_FAULT)

    def set_throttle_src(self, src: int):
        """0=ADC/pot, 1=GSP/remote, 2=PWM, 3=DShot, 4=auto. IDLE-only."""
        self._cmd(P.CMD_SET_THROTTLE_SRC, bytes([int(src) & 0xFF]))

    def set_throttle(self, val: int):
        """0..2000, GSP source only. Clamped."""
        v = max(0, min(2000, int(val)))
        self._cmd(P.CMD_SET_THROTTLE, struct.pack("<H", v))

    def heartbeat(self):
        """Dead-man's-switch — must be sent within GSP_HEARTBEAT_TIMEOUT_MS while
        running on GSP throttle, or the firmware safe-stops the motor."""
        self._cmd(P.CMD_HEARTBEAT)

    # ── burst scope (24 kHz triggered ring) ─────────────────────────────
    def scope_arm(self, trig_mode=0, pre_pct=25, trig_ch=0, trig_edge=0, threshold=0):
        """Arm the scope. Payload: [trigMode,prePct,trigCh,trigEdge,thr(2),rsv(2)]."""
        payload = struct.pack("<BBBBhH", trig_mode, pre_pct, trig_ch, trig_edge,
                              int(threshold), 0)
        self._cmd(P.CMD_SCOPE_ARM, payload, expect=P.CMD_SCOPE_ARM)

    def scope_status(self) -> dict:
        rpl = self._cmd(P.CMD_SCOPE_STATUS, b"", expect=P.CMD_SCOPE_STATUS)
        state, trig_mode, pre_pct, trig_idx, count, size = struct.unpack_from("<BBBBBB", rpl, 0)
        return {"state": state, "trig_mode": trig_mode, "pre_pct": pre_pct,
                "trig_idx": trig_idx, "sample_count": count, "sample_size": size}

    def scope_read_all(self) -> list:
        """Page out the whole frozen buffer; returns decoded samples in order."""
        st = self.scope_status()
        n = st.get("sample_count") or P.SCOPE_BUF_SIZE
        from .decode import decode_scope_sample
        out, off = [], 0
        while off < n:
            rpl = self._cmd(P.CMD_SCOPE_READ, struct.pack("<BB", off, P.SCOPE_MAX_CHUNK),
                            expect=P.CMD_SCOPE_READ)
            _r_off, actual = rpl[0], rpl[1]
            if actual == 0:
                break
            for i in range(actual):
                base = 2 + i * P.SCOPE_SAMPLE_SIZE
                out.append(decode_scope_sample(rpl[base:base + P.SCOPE_SAMPLE_SIZE]))
            off += actual
        return out

    # ── telemetry / motor (guarded) ─────────────────────────────────────
    def telem_start(self):
        self.ser.write(build_packet(P.CMD_TELEM_START))

    def telem_stop(self):
        self.ser.write(build_packet(P.CMD_TELEM_STOP))
        time.sleep(0.05)
        self.ser.reset_input_buffer()

    def read_telem_frame(self, timeout=0.5):
        """Read one unsolicited TELEM_FRAME (after telem_start). Returns a
        decoded snapshot dict or None."""
        resp = read_packet(self.ser, timeout=timeout)
        if resp and resp[0] == P.CMD_TELEM_FRAME:
            return decode_snapshot(resp[1], t=time.monotonic())
        return None
