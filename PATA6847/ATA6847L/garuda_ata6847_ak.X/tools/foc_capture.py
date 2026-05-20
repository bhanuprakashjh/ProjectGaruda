#!/usr/bin/env python3
"""
FOC GSP capture — streams the AN1078 observer snapshot over GSP binary
protocol. Mirror of pot_capture.py but parses the 80-byte FOC frame.

The host distinguishes FOC vs 6-step builds by polling GSP_CMD_GET_INFO
and checking GSP_FEATURE_FOC_AN1078 (bit 23) in featureFlags. If unset
this script bails and tells you to use pot_capture.py.

Usage:
  python3 tools/foc_capture.py /dev/ttyACM1
  python3 tools/foc_capture.py /dev/ttyACM1 --duration 30 --csv foc.csv

Press Ctrl-C to stop. Motor start/stop via board SW1/SW2 (or send the
START/STOP GSP commands manually).
"""

import serial
import struct
import time
import sys
import argparse
import csv
from datetime import datetime

GSP_START = 0x02
CMD_PING            = 0x00
CMD_GET_INFO        = 0x01
CMD_GET_SNAPSHOT    = 0x02
CMD_START_MOTOR     = 0x03
CMD_STOP_MOTOR      = 0x04
CMD_TELEM_START     = 0x14
CMD_TELEM_STOP      = 0x15
CMD_TELEM_FRAME     = 0x80

GSP_FEATURE_FOC_AN1078 = (1 << 23)

# AN1078 mode codes (an1078_motor.h)
AN_MODE_NAMES = {
    0: "STOPPED", 1: "LOCK", 2: "OPEN_LOOP",
    3: "CLOSED_LOOP", 4: "FAULT",
}

ESC_STATE_NAMES = ['IDLE', 'ARMED', 'ALIGN', 'OL_RAMP', 'CL', 'RECOVERY', 'FAULT']


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc


def build_packet(cmd_id, payload=b''):
    pkt_len = 1 + len(payload)
    body = bytes([pkt_len, cmd_id]) + payload
    crc = crc16(body)
    return bytes([GSP_START]) + body + bytes([crc >> 8, crc & 0xFF])


def parse_packet(buf):
    while len(buf) >= 5:
        idx = buf.find(bytes([GSP_START]))
        if idx < 0:
            return None, b''
        if idx > 0:
            buf = buf[idx:]
        if len(buf) < 2:
            return None, buf
        pkt_len = buf[1]
        if pkt_len < 1 or pkt_len > 249:
            buf = buf[1:]
            continue
        total = 2 + pkt_len + 2
        if len(buf) < total:
            return None, buf
        body = buf[1:2 + pkt_len]
        crc_recv = (buf[2 + pkt_len] << 8) | buf[2 + pkt_len + 1]
        crc_calc = crc16(body)
        if crc_recv != crc_calc:
            print(f"  [CRC FAIL] len={pkt_len} cmd=0x{buf[2]:02x} "
                  f"recv=0x{crc_recv:04x} calc=0x{crc_calc:04x}", file=sys.stderr)
            buf = buf[1:]
            continue
        cmd = buf[2]
        payload = buf[3:2 + pkt_len]
        return (cmd, payload), buf[total:]
    return None, buf


def decode_foc_snapshot(data):
    """Parse the 114-byte FOC frame emitted by GSP_TelemTick in FEATURE_FOC_AN1078=1.

    Layout matches dspic33AKESC's GSP_SNAPSHOT_T so the GUI's decode.ts
    `decodeSnapshot()` consumes the same bytes — see also gsp_commands.c:
      [0]      state (gEscState)
      [1]      faultCode (AN1078 faultCode)
      [2]      currentStep slot — repurposed: AN1078 mode (0..4)
      [3]      direction (unused, zero)
      [4..5]   throttle u16
      [6]      dutyPct u8
      [7]      pad
      [8..9]   vbusRaw u16
      [10..11] ibusRaw (signed mA)
      [12..13] ibusMax
      [14..67] 6-step diag fields (zero in FOC)
      [60..63] systemTick u32  ← inside the zeros above
      [64..67] uptimeSec u32
      [68..71] focIdMeas f32 (A)
      [72..75] focIqMeas f32 (A)
      [76..79] focTheta f32  (commutation angle, rad)
      [80..83] focOmega f32  (rad/s elec)
      [84..87] focVbus f32   (V)
      [88..91] focIa f32     (A)
      [92..95] focIb f32     (A)
      [96..99] focThetaObs f32 (raw observer angle, rad)
      [100..103] focVd f32   (V)
      [104..107] focVq f32   (V)
      [108]    focSubState (mode mirror)
      [109]    calDone flag
      [110..111] focOffsetIa (mA)
      [112..113] focOffsetIb (mA)
    """
    if len(data) < 108:
        return None
    s = {}
    s['state']    = data[0]
    s['focFault'] = data[1]
    s['mode']     = data[2]
    s['throttle'] = struct.unpack_from('<H', data, 4)[0]
    s['dutyPct']  = data[6]
    s['vbusRaw']  = struct.unpack_from('<H', data, 8)[0]
    s['ibus_mA']  = struct.unpack_from('<h', data, 10)[0]
    s['systemTick'] = struct.unpack_from('<I', data, 60)[0] if len(data) >= 64 else 0
    s['idMeas']   = struct.unpack_from('<f', data, 68)[0]
    s['iqMeas']   = struct.unpack_from('<f', data, 72)[0]
    s['theta']    = struct.unpack_from('<f', data, 76)[0]
    s['omega']    = struct.unpack_from('<f', data, 80)[0]
    s['vbus']     = struct.unpack_from('<f', data, 84)[0]
    s['ia']       = struct.unpack_from('<f', data, 88)[0]
    s['ib']       = struct.unpack_from('<f', data, 92)[0]
    s['thetaObs'] = struct.unpack_from('<f', data, 96)[0]
    if len(data) >= 108:
        s['vd']    = struct.unpack_from('<f', data, 100)[0]
        s['vq']    = struct.unpack_from('<f', data, 104)[0]
    else:
        s['vd'] = s['vq'] = 0.0
    if len(data) >= 114:
        s['calDone'] = data[109]
    else:
        s['calDone'] = 0
    # Fields removed from this layout (now zeroed in firmware): bemfSq, dwell.
    s['bemfSq'] = 0.0
    s['dwell']  = 0
    return s


def send_and_recv(ser, cmd, payload=b'', timeout=0.5):
    """Send a GSP command; wait until either a matching response arrives
    or timeout. Returns (cmd, payload) tuple or None."""
    ser.write(build_packet(cmd, payload))
    deadline = time.time() + timeout
    buf = b''
    while time.time() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
        while True:
            result, buf = parse_packet(buf)
            if result is None:
                break
            rcmd, rpayload = result
            if rcmd == cmd:
                return rcmd, rpayload
    return None


def get_info(ser):
    """Returns the GSP_CMD_GET_INFO payload as a dict, or None."""
    r = send_and_recv(ser, CMD_GET_INFO)
    if r is None or len(r[1]) < 20:
        return None
    payload = r[1]
    info = {
        'protocolVersion': payload[0],
        'fwMajor':         payload[1],
        'fwMinor':         payload[2],
        'fwPatch':         payload[3],
        'boardId':         struct.unpack_from('<H', payload, 4)[0],
        'motorProfile':    payload[6],
        'polePairs':       payload[7],
        'featureFlags':    struct.unpack_from('<I', payload, 8)[0],
        'pwmFrequency':    struct.unpack_from('<I', payload, 12)[0],
        'maxErpm':         struct.unpack_from('<I', payload, 16)[0],
    }
    return info


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('port', nargs='?', default='/dev/ttyACM1')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--duration', type=float, default=None,
                    help='Stop after N seconds (default: until Ctrl-C)')
    ap.add_argument('--csv', default=None,
                    help='Log to CSV file (default: foc_run_HHMMSS.csv)')
    args = ap.parse_args()

    csv_path = args.csv or f"foc_run_{datetime.now():%H%M%S}.csv"

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    print(f"opened {args.port} @ {args.baud}", file=sys.stderr)

    # Confirm this is a FOC build by querying GET_INFO.
    info = get_info(ser)
    if info is None:
        print("ERROR: no response to GET_INFO — is the board running and is "
              "this the GSP UART port?", file=sys.stderr)
        sys.exit(2)
    fw = f"{info['fwMajor']}.{info['fwMinor']}.{info['fwPatch']}"
    print(f"board fw={fw} proto={info['protocolVersion']} "
          f"profile={info['motorProfile']} pwm={info['pwmFrequency']} "
          f"flags=0x{info['featureFlags']:08x}", file=sys.stderr)
    if not (info['featureFlags'] & GSP_FEATURE_FOC_AN1078):
        print(f"ERROR: GSP_FEATURE_FOC_AN1078 (bit 23) not set — this is a "
              f"6-step build. Use pot_capture.py instead.", file=sys.stderr)
        sys.exit(2)

    # Start telemetry stream
    ser.write(build_packet(CMD_TELEM_START))
    print(f"telem started — logging to {csv_path}", file=sys.stderr)
    print("─" * 60, file=sys.stderr)
    print("Press the board's SW1 to start motor, SW2 to stop. "
          "Ctrl-C to quit.", file=sys.stderr)
    print("─" * 60, file=sys.stderr)

    t0 = time.time()
    buf = b''
    csvf = open(csv_path, 'w', newline='')
    csv_writer = None

    try:
        last_print = 0.0
        while True:
            if args.duration is not None and (time.time() - t0) > args.duration:
                break
            chunk = ser.read(256)
            if chunk:
                buf += chunk
            while True:
                result, buf = parse_packet(buf)
                if result is None:
                    break
                cmd, payload = result
                if cmd != CMD_TELEM_FRAME:
                    continue
                # Strip 2-byte seq header (matches GUI's payload.slice(2))
                s = decode_foc_snapshot(payload[2:]) if len(payload) >= 2 else None
                if s is None:
                    continue
                ts = time.time() - t0
                row = {'t': f"{ts:.3f}", **s}
                if csv_writer is None:
                    csv_writer = csv.DictWriter(csvf, fieldnames=list(row.keys()))
                    csv_writer.writeheader()
                csv_writer.writerow(row)

                # Console output ~5 Hz
                if ts - last_print >= 0.2:
                    last_print = ts
                    mode = AN_MODE_NAMES.get(s['mode'], '?')
                    eRpm = s['omega'] * 60.0 / (2 * 3.14159265)  # rad/s elec → eRPM
                    print(f"[{ts:6.2f}] m={mode:<11} "
                          f"w={s['omega']:7.1f} eRPM={eRpm:7.0f} "
                          f"Id={s['idMeas']:+5.2f} Iq={s['iqMeas']:+5.2f} "
                          f"|E|²={s['bemfSq']:6.3f} dw={s['dwell']:4d} "
                          f"th={s['throttle']:4d} cal={s['calDone']} "
                          f"vbus={s['vbus']:5.2f}V", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            ser.write(build_packet(CMD_TELEM_STOP))
        except Exception:
            pass
        ser.close()
        csvf.close()
        print(f"\nlog saved to {csv_path}", file=sys.stderr)


if __name__ == "__main__":
    main()
