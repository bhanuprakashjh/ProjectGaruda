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


def find_port(preferred: str = None) -> str:
    """Auto-detect the board's serial port across OSes. Prefers CDC-ACM /
    usbmodem devices; falls back to the first available port."""
    if preferred:
        return preferred
    ports = list(list_ports.comports())
    # Rank: explicit ACM/usbmodem first, then USB serial, then anything.
    def score(p):
        d = (p.device or "").lower()
        desc = ((p.description or "") + (p.manufacturer or "")).lower()
        if "acm" in d or "usbmodem" in d:
            return 0
        if "microchip" in desc or "pkob" in desc or "curiosity" in desc:
            return 0
        if "usb" in d or "ttyusb" in d or "wch" in desc or "ftdi" in desc:
            return 1
        return 2
    ports.sort(key=score)
    if ports:
        return ports[0].device
    # last-resort OS defaults
    return "/dev/ttyACM0"


def list_ports_human():
    return [(p.device, p.description) for p in list_ports.comports()]


class GspError(Exception):
    pass


class GspClient:
    def __init__(self, port: str = None, baud: int = 115200, timeout: float = 1.0):
        self.port = find_port(port)
        self.baud = baud
        self.ser = serial.Serial(self.port, baud, timeout=timeout)
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
