"""
GSP packet framing + CRC. Lifted verbatim (behaviour-identical) from the proven
tools/gsp_test.py and tools/step6_session.py so every tool frames identically.

Frame: [0x02][LEN=1+payload][CMD][PAYLOAD...][CRC_H][CRC_L]
CRC16-CCITT (poly 0x1021, init 0xFFFF) over [LEN][CMD][PAYLOAD].
"""
import struct
import time

from .protocol import GSP_START_BYTE


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def build_packet(cmd_id: int, payload: bytes = b"") -> bytes:
    pkt_len = 1 + len(payload)
    body = bytes([pkt_len, cmd_id]) + payload
    return bytes([GSP_START_BYTE]) + body + struct.pack(">H", crc16(body))


def read_packet(ser, timeout: float = 1.0):
    """Read one framed response. Returns (cmd_id, payload) or None on
    timeout / CRC failure. Tolerant of unsolicited telemetry interleaving."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b[0] == GSP_START_BYTE:
            break
    else:
        return None

    ser.timeout = max(deadline - time.monotonic(), 0.01)
    b = ser.read(1)
    if not b:
        return None
    pkt_len = b[0]

    ser.timeout = max(deadline - time.monotonic(), 0.01)
    data = ser.read(pkt_len + 2)
    if len(data) < pkt_len + 2:
        return None

    cmd_id = data[0]
    payload = data[1:pkt_len]
    rx_crc = struct.unpack(">H", data[pkt_len:pkt_len + 2])[0]
    if crc16(bytes([pkt_len]) + data[:pkt_len]) != rx_crc:
        return None
    return (cmd_id, payload)


def send_cmd(ser, cmd_id: int, payload: bytes = b"", timeout: float = 1.0):
    """Write a command and read one response packet."""
    ser.write(build_packet(cmd_id, payload))
    return read_packet(ser, timeout=timeout)
