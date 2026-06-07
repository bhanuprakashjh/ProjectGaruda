"""
Version-negotiated decoders for GET_INFO and GET_SNAPSHOT.

The whole point of this module: the wire formats GROW over firmware versions
(INFO went 20B→24B, snapshot is 68..228B). We decode by *payload length*, never
assume, and fill missing fields with defaults — so an older or newer board both
decode without the tool crashing (the bug we hit twice by hand today).
"""
import struct

from . import protocol as P


def decode_info(payload: bytes) -> dict:
    """GSP_INFO_T. V2 = 20B; V3 = 24B (appends buildHash). maxErpm @ offset 16."""
    n = len(payload)
    if n < 20:
        return {"error": f"INFO too short ({n}B)"}
    f = struct.unpack_from("<BBBBHBBIII", payload, 0)
    info = {
        "protocolVersion": f[0],
        "fwVersion": f"{f[1]}.{f[2]}.{f[3]}",
        "boardId": f[4],
        "motorProfileId": f[5],
        "motorProfile": P.PROFILE_NAMES.get(f[5], f"unknown({f[5]})"),
        "motorPolePairs": f[6],
        "featureFlags": f[7],
        "isFoc": bool(f[7] & P.FEATURE_FOC_AN1078),
        "pwmFrequency": f[8],
        "maxErpm": f[9],
        "buildHash": None,
        "infoBytes": n,
    }
    if n >= 24:
        info["buildHash"] = struct.unpack_from("<I", payload, 20)[0]
    return info


def _adc_to_amp(raw):
    if raw in (0, 0xFFFF):
        return 0.0
    return (raw - P.IADC_BIAS) / P.IADC_COUNTS_PER_AMP


def decode_scope_sample(b: bytes) -> dict:
    """One 26-byte SCOPE_SAMPLE_T, decoded with the 6-step field repurposing
    (garuda_service.c streams these into the FOC-shaped struct):
      ia/ib = phase A/B current ×1000 (mA) ; id = bus current ×1000 (mA)
      vd = Vbus raw ; vq = zcThreshold raw ; theta = sector(0-5)
      obs_x1 = bemf raw ; omega = eRPM/10 ; mod_index = duty% ×100
      flags bit0=HWZC en, bit1=fault ; state = ESC state ; tick_lsb"""
    (ia, ib, idc, iq, vd, vq, theta, ox1, ox2,
     omega, mod, flags, state, tick) = struct.unpack("<hhhhhhhhhhhBBH", b[:26])
    return {
        "ia_A": ia / 1000.0, "ib_A": ib / 1000.0, "ibus_A": idc / 1000.0,
        "vbus_raw": vd, "zc_thresh": vq, "sector": theta, "bemf_raw": ox1,
        "eRPM": omega * 10, "duty_pct": mod / 100.0,
        "hwzc_en": bool(flags & 0x01), "fault": bool(flags & 0x02),
        "state": state, "state_name": P.STATE_NAMES.get(state, f"?{state}"),
        "tick": tick,
    }


def decode_snapshot(p: bytes, t: float = 0.0) -> dict:
    """GSP_SNAPSHOT_T, length-tolerant. Base 68B; optional extensions decoded
    only when present: hwzc_reject@170, phase peaks@174, ibus window@198,
    speed-PI@208. Mirrors tools/step6_session.py (proven on the bench)."""
    n = len(p)
    if n < 68:
        return {"error": f"snapshot too short ({n}B)"}

    state, fault, _step, _dir, throttle, duty = struct.unpack_from("<BBBBHBx", p, 0)
    vbus_raw, ibus_raw, _ibus_max = struct.unpack_from("<HHH", p, 8)
    bemf_raw, zc_thresh, step_period, good_zc = struct.unpack_from("<HHHH", p, 14)
    _rising, _falling, synced = struct.unpack_from("<BBB", p, 22)
    zc_confirmed, zc_timeout = struct.unpack_from("<HH", p, 26)
    hwzc_en, _hwzc_phase = struct.unpack_from("<BB", p, 30)
    hwzc_zc, hwzc_miss, hwzc_hr = struct.unpack_from("<III", p, 32)
    _sys_tick, uptime = struct.unpack_from("<II", p, 60)

    hwzc_reject = struct.unpack_from("<I", p, 170)[0] if n >= 174 else 0

    ia_pos = ia_neg = ib_pos = ib_neg = 0.0
    if n >= 198:
        _ia, _ib, ia_max, ia_min, ib_max, ib_min = struct.unpack_from("<HHHHHH", p, 174)
        ia_pos, ia_neg = _adc_to_amp(ia_max), _adc_to_amp(ia_min)
        ib_pos, ib_neg = _adc_to_amp(ib_max), _adc_to_amp(ib_min)

    ibus_pk_pos = ibus_pk_neg = 0.0
    if n >= 208:
        ibus_win_max, ibus_win_min = struct.unpack_from("<HH", p, 198)
        ibus_pk_pos, ibus_pk_neg = _adc_to_amp(ibus_win_max), _adc_to_amp(ibus_win_min)

    sp_en = sp_zcs = sp_target = sp_error = sp_output = 0
    sp_integ = 0.0
    if n >= 228:
        (sp_en, _pad, sp_zcs, sp_target, sp_error,
         sp_output, sp_integ) = struct.unpack_from("<BBHIiIf", p, 208)

    # Diagnostics added 2026-06-06: CPU load (‰) + per-sector miss tally.
    cpu_load_pct = 0.0
    miss_by_sector = [0, 0, 0, 0, 0, 0]
    if n >= 242:
        cpu_load_permille = struct.unpack_from("<H", p, 228)[0]
        cpu_load_pct = cpu_load_permille / 10.0
        miss_by_sector = list(struct.unpack_from("<6H", p, 230))

    # Falling-sector OFF-center BEMF envelope (242B+). min==0xFFFF → no samples.
    fall_off_min = fall_off_max = None
    if n >= 246:
        _fmin, _fmax = struct.unpack_from("<HH", p, 242)
        if _fmin != 0xFFFF:
            fall_off_min, fall_off_max = _fmin, _fmax

    vbus_v = vbus_raw * P.VBUS_SCALE_V
    # Instantaneous bus current: valley-sampled, so it lands wherever the bus
    # happens to be during freewheel — UNRELIABLE (gives a phantom ~-20A at idle
    # when ibus_raw drifts off the 2048 bias). Kept for compatibility.
    ibus_a = (ibus_raw - P.IBUS_BIAS) * P.IBUS_SCALE_A
    # Trustworthy bus current: the firmware's windowed min/max captured over the
    # PWM cycle. Signed by the dominant excursion (motoring +, regen -). At idle
    # the window sits at bias → ~0, no phantom. Falls back to instantaneous when
    # the window field isn't in this snapshot length.
    if n >= 208:
        ibus_win = (-abs(ibus_pk_neg) if abs(ibus_pk_neg) > abs(ibus_pk_pos)
                    else abs(ibus_pk_pos))
    else:
        ibus_win = ibus_a
    if hwzc_en and hwzc_hr > 0:
        erpm = P.HWZC_ERPM_FROM_TICKS // hwzc_hr
    elif step_period > 0:
        erpm = 450_000 // step_period
    else:
        erpm = 0

    return {
        "t": t,
        "state": state, "state_name": P.STATE_NAMES.get(state, f"?{state}"),
        "fault": fault, "fault_name": P.FAULT_NAMES.get(fault, f"?{fault}"),
        "throttle": throttle, "duty": duty,
        "vbus_V": vbus_v, "ibus_A": ibus_a, "ibus_win_A": ibus_win, "eRPM": erpm,
        "bemf_raw": bemf_raw, "zc_thresh": zc_thresh, "step_period": step_period,
        "good_zc": good_zc, "synced": synced,
        "zc_confirmed": zc_confirmed, "zc_timeout": zc_timeout,
        "hwzc_en": hwzc_en, "hwzc_hr": hwzc_hr,
        "hwzc_zc": hwzc_zc, "hwzc_miss": hwzc_miss, "hwzc_reject": hwzc_reject,
        "ia_pk_mag": max(abs(ia_pos), abs(ia_neg)),
        "ib_pk_mag": max(abs(ib_pos), abs(ib_neg)),
        "ibus_pk_mag": max(abs(ibus_pk_pos), abs(ibus_pk_neg)),
        "uptime": uptime,
        "spi_en": sp_en, "spi_zcs": sp_zcs, "spi_target": sp_target,
        "spi_error": sp_error, "spi_output": sp_output, "spi_integ": sp_integ,
        "cpu_load_pct": cpu_load_pct,
        "miss_by_sector": miss_by_sector,
        "fall_off_min": fall_off_min, "fall_off_max": fall_off_max,
        "snapBytes": n,
    }
