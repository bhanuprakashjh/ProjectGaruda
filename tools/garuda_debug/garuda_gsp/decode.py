"""
Version-negotiated decoders for GET_INFO and GET_SNAPSHOT.

The whole point of this module: the wire formats GROW over firmware versions
(INFO went 20B->24B->26B, snapshot is 68..228B). We decode by *payload length*,
never assume, and fill missing fields with defaults - so an older or newer
board both decode without the tool crashing (the bug we hit twice by hand
today).
"""
import struct

from . import protocol as P


def decode_info(payload: bytes) -> dict:
    """GSP_INFO_T. V2 = 20B; V3 = 24B (appends buildHash);
    V4 = 26B (appends paramSource + nvmSelfTest, 1B each). maxErpm @ offset 16."""
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
        # V4 fields — None on older/shorter payloads (pre-Task-5 firmware).
        "paramSource": None,
        "paramSourceName": None,
        "nvmSelfTest": None,
    }
    if n >= 24:
        info["buildHash"] = struct.unpack_from("<I", payload, 20)[0]
    if n >= 26:
        param_source, nvm_selftest = struct.unpack_from("<BB", payload, 24)
        info["paramSource"] = param_source
        info["paramSourceName"] = "user" if param_source == 1 else "factory"
        info["nvmSelfTest"] = nvm_selftest
    info["connectLine"] = _build_connect_line(info)
    return info


def _build_connect_line(info: dict) -> str:
    """Short human-readable suffix for identity/connect banners.

    params=<source> always shown (defaults to 'factory?' when the firmware
    is too old to report a source). flashtest=FAIL:<n> only appears when
    the self-test ran and reported a nonzero, non-"not run" step code —
    silent otherwise so a healthy or absent self-test doesn't clutter the
    line."""
    if info.get("paramSourceName"):
        parts = [f"params={info['paramSourceName']}"]
    else:
        parts = ["params=factory?"]

    selftest = info.get("nvmSelfTest")
    if selftest is not None and selftest != 0 and selftest != 0xFF:
        parts.append(f"flashtest=FAIL:{selftest}")

    return " ".join(parts)


def _adc_to_amp(raw):
    if raw in (0, 0xFFFF):
        return 0.0
    return (raw - P.IADC_BIAS) / P.IADC_COUNTS_PER_AMP


def decode_scope_sample(b: bytes, foc: bool = False) -> dict:
    """One 26-byte SCOPE_SAMPLE_T.

    foc=False — 6-step field repurposing (garuda_service.c streams these
    into the FOC-shaped struct):
      ia/ib = phase A/B current ×1000 (mA) ; id = bus current ×1000 (mA)
      vd = Vbus raw ; vq = zcThreshold raw ; theta = sector(0-5)
      obs_x1 = bemf raw ; omega = eRPM/10 ; mod_index = duty% ×100
      flags bit0=HWZC en, bit1=fault ; state = ESC state ; tick_lsb

    foc=True — AN1078 field map (garuda_service.c AN1078 scope block,
    2026-07-09; NOTE two scalings differ from the V3 block on purpose):
      ia/ib/id/iq ×1000 (mA) ; vd/vq ×100 (cV) ; theta ×10000 (rad, ±π)
      obs_x1/x2 = observer BEMF Eα/Eβ ×1000 (mV — NOT V3's ×1e5 flux)
      omega = elec rad/s (×1) ; mod_index ×10000 (0..1)
      flags bit0=CL, bit1=fault, bits2-4 = AN mode
    Legacy keys (bemf_raw/zc_thresh/ibus_A/sector/duty_pct) are aliased to
    the closest FOC signal so the 6-step-shaped scope plot stays useful:
      BEMF curve→Eα [V], ZC-thresh curve→Eβ [V], Ibus curve→iq [A],
      sector curve→theta [rad], duty→mod [%]."""
    (ia, ib, idc, iq, vd, vq, theta, ox1, ox2,
     omega, mod, flags, state, tick) = struct.unpack("<hhhhhhhhhhhBBH", b[:26])
    if foc:
        ea_v, eb_v = ox1 / 1000.0, ox2 / 1000.0
        theta_rad = theta / 10000.0
        return {
            "ia_A": ia / 1000.0, "ib_A": ib / 1000.0,
            "id_A": idc / 1000.0, "iq_A": iq / 1000.0,
            "vd_V": vd / 100.0, "vq_V": vq / 100.0,
            "theta_rad": theta_rad,
            "bemf_a_V": ea_v, "bemf_b_V": eb_v,
            "omega_rads": float(omega),
            "eRPM": int(omega * 9.5493),      # elec rad/s → elec RPM
            "mod_pct": mod / 100.0,
            "cl": bool(flags & 0x01), "fault": bool(flags & 0x02),
            "an_mode": (flags >> 2) & 0x07,
            "state": state, "state_name": P.STATE_NAMES.get(state, f"?{state}"),
            "tick": tick,
            # legacy aliases → existing plot curves stay meaningful
            "ibus_A": iq / 1000.0, "bemf_raw": ea_v, "zc_thresh": eb_v,
            "sector": theta_rad, "duty_pct": mod / 100.0,
            "vbus_raw": vd,
        }
    return {
        "ia_A": ia / 1000.0, "ib_A": ib / 1000.0, "ibus_A": idc / 1000.0,
        "vbus_raw": vd, "zc_thresh": vq, "sector": theta, "bemf_raw": ox1,
        "eRPM": omega * 10, "duty_pct": mod / 100.0,
        "hwzc_en": bool(flags & 0x01), "fault": bool(flags & 0x02),
        "state": state, "state_name": P.STATE_NAMES.get(state, f"?{state}"),
        "tick": tick,
    }


def decode_snapshot_simplified(p: bytes, t: float = 0.0) -> dict:
    """dspic33AKESC-Simplified 68-byte snapshot. UNLIKE the production layout, this
    firmware computes the REAL units on-chip and ships them in the tail (eRPM u32
    @50, Vbus mV @54, currents centi-amps signed @56..65) - the host just displays.
    Tail offsets overlap production fields (sys_tick/uptime @60), so this layout is
    ONLY valid when the snapshot is exactly 68B (production sends 242B+). See gsp.c
    sendSnapshot() for the byte map of record."""
    state, fault, _step, _dir, throttle, duty = struct.unpack_from("<BBBBHBx", p, 0)
    vbus_raw, ibus_raw, ibus_avg_raw = struct.unpack_from("<HHH", p, 8)
    zc_thresh, step_period, good_zc = struct.unpack_from("<HHH", p, 16)
    synced = p[24]
    zc_phase_pct = struct.unpack_from("<H", p, 28)[0]
    rising_zc, falling_zc = struct.unpack_from("<HH", p, 32)
    ia_raw, ib_raw, ibusi_raw, ibusa_raw = struct.unpack_from("<HHHH", p, 36)
    bias_a, bias_b, bias_bus = struct.unpack_from("<HHH", p, 44)
    # Firmware-scaled real units (the point of this format).
    erpm = struct.unpack_from("<I", p, 50)[0]
    vbus_mv = struct.unpack_from("<H", p, 54)[0]
    ia_ca, ib_ca, ic_ca, ibus_ca, ibus_avg_ca = struct.unpack_from("<hhhhh", p, 56)

    return {
        "t": t,
        "state": state, "state_name": P.STATE_NAMES.get(state, f"?{state}"),
        "fault": fault, "fault_name": P.FAULT_NAMES.get(fault, f"?{fault}"),
        "throttle": throttle, "duty": duty,
        "vbus_V": vbus_mv / 1000.0,
        # Real, firmware-scaled bus current (signed; computed off the measured bias
        # so there is no idle phantom). ibus_win_A = the smoothed EMA, which the gauge
        # treats as the trustworthy reading.
        "ibus_A": ibus_ca / 100.0,
        "ibus_win_A": ibus_avg_ca / 100.0,
        "eRPM": erpm,
        "zc_thresh": zc_thresh, "step_period": step_period,
        "good_zc": good_zc, "synced": synced,
        "zc_phase_pct": zc_phase_pct,
        "rising_zc": rising_zc, "falling_zc": falling_zc,
        "ia_A": ia_ca / 100.0, "ib_A": ib_ca / 100.0, "ic_A": ic_ca / 100.0,
        "ia_pk_mag": abs(ia_ca) / 100.0, "ib_pk_mag": abs(ib_ca) / 100.0,
        "ibus_pk_mag": abs(ibus_ca) / 100.0,
        "ia_raw": ia_raw, "ib_raw": ib_raw,
        "ibus_raw": ibusi_raw, "ibus_avg_raw": ibusa_raw,
        "bias_a": bias_a, "bias_b": bias_b, "bias_bus": bias_bus,
        "hwzc_en": 1, "simplified": True,
        # Superset of the production keys (filled with this firmware's analogues or
        # 0/None) so every decode_snapshot() consumer works unchanged. hwzc_hr is the
        # exact HR step period (firmware HR_ERPM_CONST = 1e9), hwzc_zc = accepted ZCs.
        "bemf_raw": 0,
        "zc_confirmed": good_zc, "zc_timeout": 0,
        "hwzc_hr": (1_000_000_000 // erpm) if erpm > 0 else 0,
        "hwzc_zc": good_zc, "hwzc_miss": 0, "hwzc_reject": 0,
        "uptime": 0,
        "spi_en": 0, "spi_zcs": 0, "spi_target": 0,
        "spi_error": 0, "spi_output": 0, "spi_integ": 0.0,
        "cpu_load_pct": 0.0, "miss_by_sector": [0, 0, 0, 0, 0, 0],
        "fall_off_min": None, "fall_off_max": None,
        "snapBytes": len(p),
    }


def decode_snapshot(p: bytes, t: float = 0.0) -> dict:
    """GSP_SNAPSHOT_T, length-tolerant. Base 68B; optional extensions decoded
    only when present: hwzc_reject@170, phase peaks@174, ibus window@198,
    speed-PI@208. Mirrors tools/step6_session.py (proven on the bench).

    A snapshot of exactly 68B is the dspic33AKESC-Simplified firmware (it ships
    real units in the tail, a different layout); route it to its own decoder."""
    n = len(p)
    if n < 68:
        return {"error": f"snapshot too short ({n}B)"}
    if n == 68:
        return decode_snapshot_simplified(p, t)

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
    _ia = _ib = 0
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

    # 2026-07-14: real 3rd-phase current (Iw, ATA CSA, best-2-of-3), real DC-bus
    # current (Ibus, ATA CSA), and NTC temperature. Compact int16 centi-amps +
    # raw NTC counts appended at offset 248 (snapshot grew 248→254B). Only
    # populated in the AN1078 FOC build; older/other builds stop before 254.
    foc_iw_A = foc_ibus_A = temp_c = None
    temp_raw = 0
    if n >= 254:
        iw_ca, ibus_ca, temp_raw = struct.unpack_from("<hhH", p, 248)
        foc_iw_A = iw_ca / 100.0
        foc_ibus_A = ibus_ca / 100.0
        temp_c = P.ntc_counts_to_c(temp_raw)

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

    # FOC telemetry floats (production snapshot; offsets mirror the React decoder
    # gui/src/protocol/decode.ts). The AN1078 firmware ships these in every frame
    # (gsp_snapshot.c:154-176) but this Qt decoder never surfaced them — so the
    # console "Ia/Ibus" columns read the dead 6-step phase-window (compiled OUT in
    # FOC, garuda_service.c:1382) while the REAL FOC currents sat unread. 2026-07-09.
    #   idMeas  — d-axis current; id_ref=0 so ~0 when angle is right, swings when the
    #             observer angle slips under load (live angle-error proxy → H3).
    #   iqMeas  — measured torque current (what the current loop actually delivers).
    #   spdInt  — speed-PI integrator ≈ iq demand (iq_ref proxy; H2 vs H4).
    #   modIdx  — modulation |v|/vmax (voltage saturation → H1).
    #   obsConf — observer BEMF confidence bemf_meas/(λ·ω) (lock quality → H3).
    def _focf(off):
        return struct.unpack_from("<f", p, off)[0] if n >= off + 4 else 0.0
    foc_id_meas   = _focf(68)
    foc_iq_meas   = _focf(72)
    foc_omega     = _focf(80)

    # High-speed eRPM resolution fix (2026-07-14). In FOC the firmware synthesizes
    # step_period = (PWMFREQ*10)/erpm as a uint16 (gsp_snapshot.c), so the host's
    # erpm = 450000/step_period quantizes hard at speed: step_period 8→7→6 gives
    # ONLY 56250 / 64286 / 75000 — a motor accelerating 56k→75k reads a frozen
    # "64285". focOmega (offset 80) is the observer's continuous elec rad/s at full
    # float resolution, so derive eRPM straight from it when FOC is live. 9.5493 =
    # 60/(2π): elec rad/s → electrical RPM. Bypasses the integer round-trip entirely.
    if abs(foc_omega) > 1.0:
        erpm = int(abs(foc_omega) * 9.54929659)
    foc_vd        = _focf(100)
    foc_vq        = _focf(104)
    foc_lambda    = _focf(116)   # AN1078: thetaError-at-handoff (bleeds → 0 in CL)
    foc_spd_integ = _focf(132)
    foc_mod_index = _focf(136)
    foc_obs_conf  = _focf(140)
    foc_ia        = _focf(88)    # phase A current, A (offset-subtracted)
    foc_ib        = _focf(92)
    # ADC offsets measured by the AN1078 boot calibration — RAW counts.
    # Healthy OA+UREF chain sits at ~2048 (1.65 V bias); ~0 or ~4095 means
    # the op-amp/UREF path is dead and every "current" downstream is fiction.
    foc_off_ia = foc_off_ib = 0
    foc_sub = 0
    if n >= 150:
        foc_sub = p[144]
        foc_off_ia, foc_off_ib = struct.unpack_from("<HH", p, 146)

    return {
        "t": t,
        "state": state, "state_name": P.STATE_NAMES.get(state, f"?{state}"),
        "fault": fault, "fault_name": P.FAULT_NAMES.get(fault, f"?{fault}"),
        "focIdMeas": foc_id_meas, "focIqMeas": foc_iq_meas, "focOmega": foc_omega,
        "focVd": foc_vd, "focVq": foc_vq, "focLambdaEst": foc_lambda,
        "focPidSpdInteg": foc_spd_integ, "focModIndex": foc_mod_index,
        "focObsConfidence": foc_obs_conf,
        "focIa": foc_ia, "focIb": foc_ib,
        # 2026-07-14 real ATA-CSA measurements + NTC temp (AN1078 build, ≥254B):
        "focIw_A": foc_iw_A, "focIbus_A": foc_ibus_A,
        "tempC": temp_c, "tempRaw": temp_raw,
        # AN1078 bring-up probes (2026-07-10 firmware): focSubState carries
        # mode|runMotor<<6|cal_done<<7; focObsGain slot = startupLock;
        # focLambdaEst slot = AN_MotorStart call count.
        "anMode": foc_sub & 0x0F, "anRun": (foc_sub >> 6) & 1,
        "anCal": (foc_sub >> 7) & 1,
        "anLock": _focf(120), "anStarts": foc_lambda,
        # ISR/trigger liveness probes (2026-07-10 fw, main-loop-read = live):
        "isrCount": _focf(150), "isrFlags": int(_focf(154)),
        "pgAlive": _focf(158), "bootRcon": int(_focf(162)),
        # PWM ground truth (2026-07-10 fw): PG1DC>>4 and override nibble
        # (OVRENH<<3|OVRENL<<2|OVRDAT; 0x0 = overrides released, full PWM)
        "pg1dc16": struct.unpack_from("<H", p, 166)[0] if n >= 170 else 0,
        "ovState": p[168] if n >= 170 else 0,
        "focOffsetIa": foc_off_ia, "focOffsetIb": foc_off_ib,
        "ia_raw": _ia, "ib_raw": _ib,
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
