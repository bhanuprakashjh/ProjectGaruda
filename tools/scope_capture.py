#!/usr/bin/env python3
"""Burst scope capture tool — arms trigger, polls status, reads & plots data.

Usage:
  python3 scope_capture.py /dev/ttyACM0 [--mode fault|manual|state|threshold]
                                        [--pre 50] [--channel iq] [--threshold 1000]
                                        [--edge rising|falling] [--csv out.csv]
"""

import sys, serial, struct, time, argparse

# GSP V2 framing
STX = 0x02
CMD_PING         = 0x00
CMD_SCOPE_ARM    = 0x30
CMD_SCOPE_STATUS = 0x31
CMD_SCOPE_READ   = 0x32

SCOPE_SAMPLE_SIZE = 26
SCOPE_MAX_CHUNK   = 9

# Trigger modes
TRIG_MODES = {'manual': 0, 'fault': 1, 'state': 2, 'threshold': 3}

# Channel map
CHANNELS = {
    'ia': 0, 'ib': 1, 'id': 2, 'iq': 3, 'vd': 4, 'vq': 5,
    'theta': 6, 'omega': 9, 'mod': 10
}

# Channel scaling (divisor to get real units)
SCALE = {
    'ia': 1000.0, 'ib': 1000.0, 'id': 1000.0, 'iq': 1000.0,
    'vd': 100.0, 'vq': 100.0, 'theta': 10000.0,
    'obs_x1': 100000.0, 'obs_x2': 100000.0,  # Observer flux (V·s)
    'omega': 1.0, 'mod_index': 10000.0
}

# Scope states
STATE_NAMES = {0: 'IDLE', 1: 'ARMED', 2: 'FILLING', 3: 'READY'}

def crc16_ccitt(data):
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
    length = 1 + len(payload)
    crc_data = bytes([length, cmd]) + payload
    crc = crc16_ccitt(crc_data)
    return bytes([STX, length, cmd]) + payload + struct.pack('>H', crc)

def send_cmd(ser, cmd, payload=b''):
    ser.write(build_packet(cmd, payload))
    ser.flush()

def read_response(ser, timeout=2.0):
    ser.timeout = timeout
    while True:
        b = ser.read(1)
        if not b:
            return None, None
        if b[0] == STX:
            break
    lb = ser.read(1)
    if not lb:
        return None, None
    length = lb[0]
    rest = ser.read(length + 2)
    if len(rest) < length + 2:
        return None, None
    cmd = rest[0]
    payload = rest[1:length]
    rx_crc = (rest[length] << 8) | rest[length + 1]
    calc_crc = crc16_ccitt(bytes([length]) + rest[:length])
    if rx_crc != calc_crc:
        return None, None
    return cmd, payload

def decode_sample(data, offset=0):
    """Decode a 26-byte SCOPE_SAMPLE_T from bytes."""
    s = struct.unpack_from('<11h 2B H', data, offset)
    return {
        'ia':    s[0] / 1000.0,
        'ib':    s[1] / 1000.0,
        'id':    s[2] / 1000.0,
        'iq':    s[3] / 1000.0,
        'vd':    s[4] / 100.0,
        'vq':    s[5] / 100.0,
        'theta': s[6] / 10000.0,
        'obs_x1': s[7] / 100000.0,  # Observer flux alpha (V·s)
        'obs_x2': s[8] / 100000.0,  # Observer flux beta (V·s)
        'omega': s[9],  # ×1 scaling (rad/s, integer resolution)
        'mod_index': s[10] / 10000.0,
        'flags': s[11],
        'state': s[12],
        'tick_lsb': s[13],
        'cl':    bool(s[11] & 0x01),
        'fault': bool(s[11] & 0x02),
        'mode':  (s[11] >> 2) & 0x07,
    }

def main():
    parser = argparse.ArgumentParser(description='Burst scope capture')
    parser.add_argument('port', nargs='?', default='/dev/ttyACM0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--mode', choices=TRIG_MODES.keys(), default='fault')
    parser.add_argument('--pre', type=int, default=50, help='Pre-trigger %% (0-100)')
    parser.add_argument('--channel', choices=CHANNELS.keys(), default='iq')
    parser.add_argument('--threshold', type=int, default=1000, help='Scaled threshold')
    parser.add_argument('--edge', choices=['rising', 'falling'], default='rising')
    parser.add_argument('--csv', type=str, default=None, help='Save to CSV file')
    parser.add_argument('--timeout', type=float, default=30.0, help='Wait timeout (s)')
    args = parser.parse_args()

    print(f"Connecting to {args.port} @ {args.baud}...")
    ser = serial.Serial(args.port, args.baud, timeout=1)
    time.sleep(0.5)
    ser.reset_input_buffer()

    # Ping
    send_cmd(ser, CMD_PING)
    cmd, _ = read_response(ser)
    if cmd != CMD_PING:
        print("PING failed")
        return
    print("PING OK")

    # Arm scope
    trig_mode = TRIG_MODES[args.mode]
    trig_ch   = CHANNELS.get(args.channel, 3)
    trig_edge = 0 if args.edge == 'rising' else 1
    threshold = args.threshold

    payload = struct.pack('<BBBBhBB',
                          trig_mode, args.pre, trig_ch, trig_edge,
                          threshold, 0, 0)
    print(f"\nArming scope: mode={args.mode}, pre={args.pre}%, "
          f"ch={args.channel}, edge={args.edge}, thresh={threshold}")
    send_cmd(ser, CMD_SCOPE_ARM, payload)
    cmd, _ = read_response(ser)
    if cmd != CMD_SCOPE_ARM:
        print(f"ARM failed: cmd=0x{cmd:02X}" if cmd else "ARM: no response")
        return
    print("Scope ARMED")

    # Poll status until READY
    print(f"\nWaiting for trigger (timeout {args.timeout}s)...")
    t0 = time.time()
    while True:
        if time.time() - t0 > args.timeout:
            print("TIMEOUT — no trigger")
            ser.close()
            return

        time.sleep(0.1)
        send_cmd(ser, CMD_SCOPE_STATUS)
        cmd, payload = read_response(ser, timeout=1.0)
        if cmd != CMD_SCOPE_STATUS or not payload or len(payload) < 6:
            continue

        scope_state = payload[0]
        sample_count = payload[4]
        sample_size  = payload[5]

        sname = STATE_NAMES.get(scope_state, f"?({scope_state})")
        elapsed = time.time() - t0
        print(f"\r  [{elapsed:5.1f}s] State: {sname}, samples: {sample_count}, "
              f"size: {sample_size}B", end='', flush=True)

        if scope_state == 3:  # READY
            print(f"\n\nTriggered! {sample_count} samples captured.")
            break

    # Read all samples in chunks
    samples = []
    offset = 0
    while offset < sample_count:
        count = min(SCOPE_MAX_CHUNK, sample_count - offset)
        send_cmd(ser, CMD_SCOPE_READ, struct.pack('<BB', offset, count))
        cmd, payload = read_response(ser, timeout=2.0)
        if cmd != CMD_SCOPE_READ or not payload or len(payload) < 2:
            print(f"Read failed at offset {offset}")
            break

        rx_offset = payload[0]
        rx_count  = payload[1]
        if rx_count == 0:
            break

        for i in range(rx_count):
            s = decode_sample(payload, 2 + i * SCOPE_SAMPLE_SIZE)
            samples.append(s)

        offset += rx_count
        print(f"  Read {offset}/{sample_count} samples", end='\r')

    print(f"\n\nRead {len(samples)} samples total.")

    # Time axis: each sample is 1/24000 = 41.67µs
    dt_us = 1e6 / 24000.0

    # Find trigger point from status
    send_cmd(ser, CMD_SCOPE_STATUS)
    _, st_payload = read_response(ser)
    trig_idx = st_payload[3] if st_payload and len(st_payload) >= 4 else 0
    pre_pct = st_payload[2] if st_payload and len(st_payload) >= 3 else args.pre

    # Print table
    print(f"\n{'#':>4} {'t(µs)':>8} {'Ia':>7} {'Ib':>7} {'Id':>7} {'Iq':>7} "
          f"{'Vd':>7} {'Vq':>7} {'θ':>7} {'ω':>8} {'mod':>6} {'Φα':>8} {'Φβ':>8} {'st':>3} {'fl':>2}")
    print('-' * 115)
    for i, s in enumerate(samples):
        t_us = (i - int(len(samples) * pre_pct / 100)) * dt_us
        marker = ' *' if i == int(len(samples) * pre_pct / 100) else '  '
        print(f"{i:4d} {t_us:8.1f} {s['ia']:7.3f} {s['ib']:7.3f} "
              f"{s['id']:7.3f} {s['iq']:7.3f} {s['vd']:7.2f} {s['vq']:7.2f} "
              f"{s['theta']:7.4f} {s['omega']:8.1f} {s['mod_index']:6.4f} "
              f"{s['obs_x1']:8.5f} {s['obs_x2']:8.5f} "
              f"{s['state']:3d} {s['flags']:2d}{marker}")

    # Save CSV
    if args.csv:
        import csv
        with open(args.csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['sample', 't_us', 'ia', 'ib', 'id', 'iq', 'vd', 'vq',
                         'theta', 'obs_x1', 'obs_x2', 'omega', 'mod_index',
                         'flags', 'state', 'tick_lsb', 'cl', 'fault', 'mode'])
            for i, s in enumerate(samples):
                t_us = (i - int(len(samples) * pre_pct / 100)) * dt_us
                w.writerow([i, f'{t_us:.1f}', f'{s["ia"]:.4f}', f'{s["ib"]:.4f}',
                            f'{s["id"]:.4f}', f'{s["iq"]:.4f}',
                            f'{s["vd"]:.3f}', f'{s["vq"]:.3f}',
                            f'{s["theta"]:.5f}', f'{s["obs_x1"]:.5f}',
                            f'{s["obs_x2"]:.5f}', f'{s["omega"]:.1f}',
                            f'{s["mod_index"]:.5f}', s['flags'], s['state'],
                            s['tick_lsb'], s['cl'], s['fault'], s['mode']])
        print(f"\nSaved to {args.csv}")

    # Try matplotlib plot
    try:
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)

        t_arr = [(i - int(len(samples) * pre_pct / 100)) * dt_us
                 for i in range(len(samples))]

        axes[0].plot(t_arr, [s['ia'] for s in samples], label='Ia')
        axes[0].plot(t_arr, [s['ib'] for s in samples], label='Ib')
        axes[0].set_ylabel('Phase Current (A)')
        axes[0].legend()
        axes[0].grid(True)
        axes[0].axvline(0, color='r', linestyle='--', alpha=0.5, label='trigger')

        axes[1].plot(t_arr, [s['id'] for s in samples], label='Id')
        axes[1].plot(t_arr, [s['iq'] for s in samples], label='Iq')
        axes[1].set_ylabel('DQ Current (A)')
        axes[1].legend()
        axes[1].grid(True)
        axes[1].axvline(0, color='r', linestyle='--', alpha=0.5)

        axes[2].plot(t_arr, [s['vd'] for s in samples], label='Vd')
        axes[2].plot(t_arr, [s['vq'] for s in samples], label='Vq')
        axes[2].set_ylabel('Voltage (V)')
        axes[2].legend()
        axes[2].grid(True)
        axes[2].axvline(0, color='r', linestyle='--', alpha=0.5)

        axes[3].plot(t_arr, [s['mod_index'] for s in samples], label='Mod Index')
        axes[3].plot(t_arr, [s['omega'] for s in samples], label='ω (rad/s)',
                     alpha=0.7)
        axes[3].set_ylabel('Mod / Speed')
        axes[3].set_xlabel('Time (µs)')
        axes[3].legend()
        axes[3].grid(True)
        axes[3].axvline(0, color='r', linestyle='--', alpha=0.5)

        fig.suptitle(f'Burst Scope — {args.mode} trigger, {len(samples)} samples')
        plt.tight_layout()
        plt.show()
    except ImportError:
        print("\nInstall matplotlib for plots: pip install matplotlib")

    ser.close()

if __name__ == '__main__':
    main()
