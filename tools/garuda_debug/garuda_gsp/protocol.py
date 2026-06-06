"""
GSP protocol constants — single source of truth for the Garuda Serial Protocol.

Mirrors the firmware (gsp/gsp_commands.h, gsp/gsp_params.h). When the firmware
bumps GSP_PROTOCOL_VERSION or adds a param, update this file and nothing else.
"""

GSP_START_BYTE = 0x02

# ── Commands ────────────────────────────────────────────────────────────
CMD_PING            = 0x00
CMD_GET_INFO        = 0x01
CMD_GET_SNAPSHOT    = 0x02
CMD_START_MOTOR     = 0x03
CMD_STOP_MOTOR      = 0x04
CMD_CLEAR_FAULT     = 0x05
CMD_SET_THROTTLE    = 0x06
CMD_SET_THROTTLE_SRC = 0x07
CMD_HEARTBEAT       = 0x08
CMD_GET_PARAM       = 0x10
CMD_SET_PARAM       = 0x11
CMD_SAVE_CONFIG     = 0x12
CMD_LOAD_DEFAULTS   = 0x13
CMD_TELEM_START     = 0x14
CMD_TELEM_STOP      = 0x15
CMD_GET_PARAM_LIST  = 0x16
CMD_LOAD_PROFILE    = 0x17
CMD_TELEM_FRAME     = 0x80   # unsolicited
CMD_ERROR           = 0xFF

# ── Error codes ─────────────────────────────────────────────────────────
ERR_NAMES = {
    0x01: "UNKNOWN_CMD", 0x02: "BAD_LENGTH", 0x03: "BUSY",
    0x04: "WRONG_STATE", 0x05: "OUT_OF_RANGE", 0x06: "UNKNOWN_PARAM",
    0x07: "CROSS_VALIDATION", 0x08: "EEPROM_THROTTLED",
}

# ── ESC state + fault names (gsp/gsp_commands or garuda_types) ───────────
STATE_NAMES = {
    0: "IDLE", 1: "ARMED", 2: "DETECT", 3: "ALIGN", 4: "OL_RAMP",
    5: "MORPH", 6: "CL", 7: "BRAKING", 8: "RECOVERY", 9: "FAULT",
}
FAULT_NAMES = {
    0: "NONE", 1: "OV", 2: "UV", 3: "OC_SW", 4: "BOARD_PCI",
    5: "STALL", 6: "DESYNC", 7: "START_TO", 8: "MORPH_TO",
    9: "RX_LOSS", 10: "FOC_INT", 11: "FOC_BUS",
}

# ── Motor profiles (gsp/gsp_params.h) ───────────────────────────────────
PROFILE_NAMES = {
    0: "Hurst", 1: "A2212", 2: "2810(5010)", 3: "5055",
    4: "Cobra-2814", 5: "XRotor-3110", 6: "Custom",
}

# ── Parameter IDs → names (gsp/gsp_params.h, descriptor table, 31 params) ─
PARAM_NAMES = {
    0x15: "rampTargetErpm", 0x16: "rampAccelErpmPerS", 0x17: "rampDutyPct",
    0x20: "clIdleDutyPct", 0x22: "timingAdvMaxDeg", 0x30: "hwzcCrossoverErpm",
    0x41: "ocFaultMa", 0x42: "ocSwLimitMa",
    0x50: "motorPolePairs", 0x51: "alignDutyPct", 0x52: "initialErpm",
    0x53: "maxClosedLoopErpm", 0x54: "sineAlignModPct", 0x55: "sineRampModPct",
    0x56: "zcDemagDutyThresh", 0x57: "zcDemagBlankExtraPct",
    0x58: "ocLimitMa", 0x59: "ocStartupMa", 0x5A: "rampCurrentGateMa",
    0x60: "dutySlewUpPctPerMs", 0x61: "dutySlewDownPctPerMs",
    0x62: "postSyncSettleMs", 0x63: "postSyncSlewDivisor",
    0x64: "zcBlankingPercent", 0x65: "zcAdcDeadband",
    0x66: "zcSyncThreshold", 0x67: "zcFilterThreshold",
    0x68: "vbusOvAdc", 0x69: "vbusUvAdc",
    0x6A: "desyncCoastMs", 0x6B: "desyncMaxRestarts",
}
PARAM_IDS = {v: k for k, v in PARAM_NAMES.items()}

# ── Feature-flag bits (gsp BuildFeatureFlags) — extend as needed ─────────
# Known: bit 23 = FOC_AN1078 (FOC build vs 6-step). Others board/build specific.
FEATURE_FOC_AN1078 = 1 << 23

# ── ADC scaling — BOARD-SPECIFIC (MCLV-48V-300W). Key off boardId later. ─
# These match tools/step6_session.py and are correct for the AKESC bench board.
VBUS_SCALE_V        = 3.3 * 23.2 / 4095.0
IBUS_SCALE_A        = 3.3 / (4095.0 * 24.95 * 0.003)
IBUS_BIAS           = 2048
IADC_BIAS           = 2048
IADC_COUNTS_PER_AMP = 93.0
HWZC_ERPM_FROM_TICKS = 1_000_000_000
