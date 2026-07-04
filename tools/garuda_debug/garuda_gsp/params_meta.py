"""params_meta.py — human metadata for firmware parameters.

The firmware serves terse names + raw integers; this maps each to a unit,
a one-line description, and a raw→physical conversion so the Tune tab (and
the Claude debug report) can show real quantities next to the raw value.

Board constants (bench-proven 2026-07-03, U3 campaign):
  current: 93 ADC counts/A (OC gain 24.95, 22A full scale)
  vbus:    3.3V * 23.2 divider / 4096 = 18.694 mV/count
"""

COUNTS_PER_AMP = 93.0
VBUS_V_PER_COUNT = 3.3 * 23.2 / 4096.0


def _amps_from_adc(v):   return f"{v / COUNTS_PER_AMP:.1f} A"
def _amps_from_ma(v):    return f"{v / 1000.0:.1f} A"
def _amps_from_ca(v):    return f"{v / 100.0:.1f} A"
def _volts_from_adc(v):  return f"{v * VBUS_V_PER_COUNT:.1f} V"
def _pct(v):             return f"{v} %"
def _pct_x10(v):         return f"{v / 10.0:.1f} %"
def _ms(v):              return f"{v} ms"
def _erpm(v):            return f"{v:,} eRPM"
def _raw(v):             return ""


# name -> (converter, description). Names match protocol.PARAM_NAMES values.
META = {
    # ── speed / ramp ──
    "rampTargetErpm":    (_erpm, "Open-loop ramp end speed — CL handoff happens here"),
    "rampAccelErpmPerS": (lambda v: f"{v:,} eRPM/s", "Open-loop ramp acceleration"),
    "maxClosedLoopErpm": (_erpm, "CL overspeed clamp (governor holds duty at this speed)"),
    # ── zero-cross detection ──
    "zcBlankingPercent": (_pct, "Post-commutation blanking, % of sector (demag mask)"),
    "zcAdcDeadband":     (lambda v: f"{v} cnt", "Comparator deadband around threshold, ADC counts"),
    "zcSyncThreshold":   (lambda v: f"{v} ZCs", "Consecutive good ZCs to declare sync"),
    "zcFilterThreshold": (_raw, "SW ZC glitch filter (legacy path)"),
    "zcDemagDutyThresh": (_pct, "Duty floor for the load-adaptive demag blank (WS1)"),
    "zcDemagBlankPerA":  (lambda v: f"{v} %/256cnt", "Extra blank per phase-current excess (WS1)"),
    "zcDemagBlankIbusDb":(lambda v: f"{v} cnt ({v/COUNTS_PER_AMP:.1f} A)", "Current deadband before WS1 blank engages"),
    "zcDemagBlankExtraPct": (_pct, "Fixed extra blank percentage"),
    "zcDemagBlankMaxPct":(_pct, "Total blank cap, % of sector"),
    # ── protection ──
    "ocLimitMa":         (_amps_from_ma, "SW overcurrent soft-limiter engage (proportional duty cut)"),
    "ocFaultMa":         (_amps_from_ma, "SW overcurrent hard fault"),
    "ocStartupMa":       (_amps_from_ma, "OC threshold during startup (CMP3 DAC)"),
    "rampCurrentGateMa": (_amps_from_ma, "Ramp current gate"),
    "stallIphaseAdc":    (_amps_from_adc, "WS2 phase-stall fault threshold (raw ADC counts)"),
    "stallDebounceMs":   (_ms, "WS2 stall debounce (accumulates only at duty ≥40%)"),
    "stallArmErpm":      (_erpm, "WS2 stall check armed above this speed"),
    "vbusOvAdc":         (_volts_from_adc, "Overvoltage fault"),
    "vbusUvAdc":         (_volts_from_adc, "Undervoltage fault"),
    # ── duty pipeline ──
    "dutySlewUpPctPerMs":   (lambda v: f"{v} %/ms", "Max duty increase rate"),
    "dutySlewDownPctPerMs": (lambda v: f"{v} %/ms", "Max duty decrease rate (spindown floor also applies)"),
    "postSyncSettleMs":  (_ms, "Reduced slew-up window after ZC lock"),
    "postSyncSlewDivisor": (lambda v: f"÷{v}", "Slew-up divisor during settle"),
    # ── startup / morph ──
    "sineAlignModPct":   (_pct, "Sine align modulation depth"),
    "sineRampModPct":    (_pct, "Sine ramp modulation depth"),
    "morphLockZcCount":  (lambda v: f"{v} ZCs", "Morph exit: consecutive in-tolerance ZCs required"),
    "morphLockTolPct":   (_pct, "Morph exit: capture-vs-expected tolerance"),
    "desyncCoastMs":     (_ms, "Recovery coast before restart attempt"),
    "desyncMaxRestarts": (lambda v: f"{v}×", "Restart attempts before latched fault"),
    # ── I-f / FOC (legacy builds only) ──
    "ifCurrentCa":       (_amps_from_ca, "I-f startup current"),
    "ifRampErpmPerS":    (lambda v: f"{v:,} eRPM/s", "I-f ramp rate"),
}


def phys(name, value):
    """Physical rendering of a raw value ('' if unknown/no conversion)."""
    if value is None:
        return ""
    m = META.get(name)
    if m:
        try:
            return m[0](value)
        except Exception:
            return ""
    # suffix heuristics for params without explicit metadata
    if name.endswith("Ma"):      return _amps_from_ma(value)
    if name.endswith("Ca"):      return _amps_from_ca(value)
    if name.endswith("Erpm"):    return _erpm(value)
    if name.endswith("Ms"):      return _ms(value)
    if name.endswith("Pct"):     return _pct(value)
    if name.endswith("PctX10"):  return _pct_x10(value)
    return ""


def describe(name):
    m = META.get(name)
    return m[1] if m else ""
