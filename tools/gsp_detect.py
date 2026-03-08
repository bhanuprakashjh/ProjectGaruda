#!/usr/bin/env python3
"""Send GSP auto-detect command and monitor results.

Usage: python3 gsp_detect.py /dev/ttyACM0
"""

import sys, serial, struct, time

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
BAUD = 115200

# GSP V2 framing
STX = 0x02
CMD_AUTO_DETECT = 0x20
CMD_GET_SNAPSHOT = 0x02
CMD_PING = 0x00
CMD_GET_PARAM = 0x10
CMD_SAVE_CONFIG = 0x12

# FOC param IDs
PARAM_FOC_RS_MOHM = 0x70
PARAM_FOC_LS_UH   = 0x71
PARAM_FOC_KE_UV   = 0x72

def crc16_ccitt(data):
    """CRC-CCITT (poly 0x1021, init 0xFFFF, MSB-first) — matches GUI gsp.ts."""
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def build_packet(cmd, payload=b''):
    length = 1 + len(payload)  # cmd + payload
    crc_data = bytes([length, cmd]) + payload
    crc = crc16_ccitt(crc_data)
    return bytes([STX, length, cmd]) + payload + struct.pack('>H', crc)  # Big-endian CRC

def send_cmd(ser, cmd, payload=b''):
    pkt = build_packet(cmd, payload)
    ser.write(pkt)
    ser.flush()

def read_response(ser, timeout=2.0):
    ser.timeout = timeout
    # Look for STX
    while True:
        b = ser.read(1)
        if not b:
            return None, None
        if b[0] == STX:
            break
    # Read length
    lb = ser.read(1)
    if not lb:
        return None, None
    length = lb[0]
    # Read cmd + payload + CRC (2 bytes big-endian)
    rest = ser.read(length + 2)
    if len(rest) < length + 2:
        return None, None
    cmd = rest[0]
    payload = rest[1:length]
    # Verify CRC
    rx_crc = (rest[length] << 8) | rest[length + 1]
    calc_crc = crc16_ccitt(bytes([length]) + rest[:length])
    if rx_crc != calc_crc:
        return None, None  # CRC mismatch
    return cmd, payload

def get_snapshot(ser):
    """Get snapshot and extract FOC fields.

    GSP_SNAPSHOT_T is 150 bytes packed (v3):
      offset 0:  state (u8)
      offset 1:  faultCode (u8)
      offset 4:  throttle (u16)
      offset 6:  dutyPct (u8)
      offset 8:  vbusRaw (u16)
      offset 68: focIdMeas (float)
      offset 72: focIqMeas (float)
      offset 76: focTheta (float)
      offset 80: focOmega (float)
      offset 84: focVbus (float)
      offset 88: focIa/Ib (float)
      offset 96: focThetaObs (float)
      offset 100-140: Vd/Vq/obs/PI/diag
      offset 144: focSubState (u8)
    """
    send_cmd(ser, CMD_GET_SNAPSHOT)
    cmd, payload = read_response(ser)
    if cmd != CMD_GET_SNAPSHOT or payload is None:
        return None
    if len(payload) < 10:
        return {'state': payload[0], 'len': len(payload)}

    state     = payload[0]
    faultCode = payload[1]
    throttle  = struct.unpack_from('<H', payload, 4)[0]
    dutyPct   = payload[6]
    vbusRaw   = struct.unpack_from('<H', payload, 8)[0]

    if len(payload) < 100:
        return {'state': state, 'faultCode': faultCode, 'throttle': throttle,
                'dutyPct': dutyPct, 'vbusRaw': vbusRaw, 'len': len(payload)}

    # FOC fields (offsets from GSP_SNAPSHOT_T packed struct)
    id_meas   = struct.unpack_from('<f', payload, 68)[0]
    iq_meas   = struct.unpack_from('<f', payload, 72)[0]
    theta     = struct.unpack_from('<f', payload, 76)[0]
    omega     = struct.unpack_from('<f', payload, 80)[0]
    vbus      = struct.unpack_from('<f', payload, 84)[0]
    ia        = struct.unpack_from('<f', payload, 88)[0]
    ib        = struct.unpack_from('<f', payload, 92)[0]
    theta_obs = struct.unpack_from('<f', payload, 96)[0]
    # focSubState offset depends on snapshot version
    if len(payload) >= 150:
        foc_sub = payload[144]  # v3
    elif len(payload) >= 114:
        foc_sub = payload[108]  # v2
    else:
        foc_sub = payload[100]  # v1

    return {
        'state': state,
        'faultCode': faultCode,
        'foc_sub': foc_sub,
        'Id': id_meas,
        'Iq': iq_meas,
        'theta': theta,
        'omega': omega,
        'vbus': vbus,
        'Ia': ia,
        'Ib': ib,
        'theta_obs': theta_obs,
        'throttle': throttle,
        'dutyPct': dutyPct,
    }

def get_param(ser, param_id):
    """Read a single GSP parameter. Request: uint16 param_id. Response: [id(2)+value(4)]."""
    send_cmd(ser, CMD_GET_PARAM, struct.pack('<H', param_id))
    cmd, payload = read_response(ser, timeout=1.0)
    if cmd == CMD_GET_PARAM and payload and len(payload) >= 6:
        return struct.unpack_from('<I', payload, 2)[0]
    return None

def main():
    print(f"Connecting to {PORT} @ {BAUD}...")
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(0.5)
    ser.reset_input_buffer()

    # Ping
    send_cmd(ser, CMD_PING)
    cmd, _ = read_response(ser)
    if cmd == CMD_PING:
        print("PING OK")
    else:
        print("PING failed - check connection")
        return

    # Check state
    snap = get_snapshot(ser)
    if snap is None:
        print("Failed to get snapshot")
        return
    state_names = {0:'IDLE', 1:'ARMED', 2:'DETECT', 3:'ALIGN', 4:'OL_RAMP',
                   5:'MORPH', 6:'CLOSED_LOOP', 7:'BRAKING', 8:'RECOVERY', 9:'FAULT'}
    sn = state_names.get(snap['state'], f"?({snap['state']})")
    print(f"State: {sn} ({snap['state']}), Vbus: {snap.get('vbus', snap.get('vbusRaw','?'))}V")

    if snap['state'] != 0:  # ESC_IDLE = 0
        print(f"ERROR: Motor must be IDLE (state=0), currently state={snap['state']}")
        return

    # Send auto-detect
    print("\n=== STARTING AUTO-DETECT ===")
    print("Phase 1: Rs measurement (~0.8s) - rotor locks at theta=0")
    print("Phase 2: Ls measurement (~3s) - voltage pulses")
    print("Phase 3: Lambda measurement (~4.5s) - motor spins!")
    print("Phase 4: Auto-tune (instant)")
    print()

    send_cmd(ser, CMD_AUTO_DETECT)
    cmd, payload = read_response(ser)
    if cmd == CMD_AUTO_DETECT:
        print("Detect command accepted!")
    elif cmd == 0xFF:
        print(f"ERROR: Command rejected (error code: {payload[0] if payload else '?'})")
        return
    else:
        print(f"Unexpected response: cmd=0x{cmd:02X}")
        return

    # Monitor progress
    print("\nMonitoring... (Ctrl+C to abort)")
    t0 = time.time()
    last_sub = -1

    phase_names = {
        0: "Idle",
        1: "Detecting Rs",
        2: "Detecting Ls",
        3: "Detecting Lambda (motor spinning!)",
        4: "Auto-tuning",
        5: "DONE",
        6: "FAIL",
    }

    while True:
        time.sleep(0.2)
        snap = get_snapshot(ser)
        if snap is None:
            continue

        elapsed = time.time() - t0
        sub = snap.get('foc_sub', 0)
        state = snap['state']

        if sub != last_sub:
            phase = phase_names.get(sub, f"Unknown({sub})")
            sname = state_names.get(state, f"?({state})")
            print(f"  [{elapsed:5.1f}s] Phase: {phase}  (state={sname}, sub={sub})")
            last_sub = sub

        # Print current readings
        if state == 2:  # ESC_DETECT
            print(f"          Id={snap.get('Id', 0):+6.3f}A  "
                  f"Iq={snap.get('Iq', 0):+6.3f}A  "
                  f"ω={snap.get('omega', 0):+7.1f} rad/s  "
                  f"Vbus={snap.get('vbus', 0):.1f}V  "
                  f"θ={snap.get('theta', 0):+5.2f}")

        # Detect complete: state goes back to ARMED (1) or FAULT (9)
        # ESC states: IDLE=0, ARMED=1, DETECT=2, ALIGN=3, ...FAULT=9
        if state == 2:  # ESC_DETECT — still running
            pass
        elif state == 1:  # ESC_ARMED — detect completed successfully
            print(f"\n=== DETECT COMPLETE ({elapsed:.1f}s) ===")
            # Read back detected params
            time.sleep(0.1)
            rs = get_param(ser, PARAM_FOC_RS_MOHM)
            ls = get_param(ser, PARAM_FOC_LS_UH)
            ke = get_param(ser, PARAM_FOC_KE_UV)
            print("Detected motor parameters:")
            if rs is not None:
                print(f"  Rs = {rs} mΩ  ({rs/1000:.4f} Ω)")
            if ls is not None:
                print(f"  Ls = {ls} µH  ({ls/1e6*1000:.3f} mH)")
            if ke is not None:
                print(f"  Ke = {ke} µV·s/rad  ({ke/1e6:.6f} V·s/rad)")
                if ke > 0:
                    # KV = 60 / (sqrt(3) * 2π * Ke * pp)
                    # Assume pp from last snapshot or default
                    print(f"       λ_pm = {ke/1e6:.6f} V·s/rad (per-phase)")
            # Save detected params to EEPROM so they persist across reset
            print("\nSaving to EEPROM...")
            send_cmd(ser, CMD_SAVE_CONFIG)
            cmd_s, payload_s = read_response(ser, timeout=2.0)
            if cmd_s == CMD_SAVE_CONFIG:
                print("Saved! Params will persist across reset.")
            else:
                print("WARNING: Save failed — params are RAM-only until saved.")
            print("Motor is now ARMED - will auto-start when pot is at zero.")
            break
        elif state == 9:  # ESC_FAULT
            print(f"\n=== DETECT FAILED ({elapsed:.1f}s) ===")
            print(f"Fault code: {snap.get('faultCode', '?')}")
            print("Check: motor connected? free to spin? Vbus OK?")
            break
        elif state == 0:  # ESC_IDLE — detect was interrupted or never started
            if elapsed > 2.0:
                print(f"\n=== DETECT STOPPED ({elapsed:.1f}s) ===")
                print("Motor returned to IDLE unexpectedly")
                break

        if elapsed > 15:
            print("\nTIMEOUT - detect took too long")
            break

    ser.close()

if __name__ == '__main__':
    main()
