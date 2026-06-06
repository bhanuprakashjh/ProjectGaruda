#!/usr/bin/env python3
"""
Garuda Studio — Pillar 2: Motor Profile Wizard (physics engine + C code-gen).

Turn a handful of datasheet numbers into a complete, current-safe motor profile
for all four firmware files — the thing we otherwise do by hand (and get wrong,
e.g. an OC-tripping sine amplitude). Pure functions, no GUI, so it's testable and
usable from a CLI:

    python -m garuda_gui.wizard --name Cobra2814 --kv 470 --r 0.188 \
        --poles 14 --weight 117 --imax 17 --vbus 24 --profile 4 --enum COBRA

The heuristics are calibrated to reproduce the proven 2810 profile (sine ramp
current ~12A → sineRampModPct=5, rampAccel=3000, timingAdv≈25) so the numbers it
emits sit in the same trusted regime, scaled by each motor's physics.
"""
from __future__ import annotations
import argparse
import math
from dataclasses import dataclass, field

# Board limit: the AKESC shunt saturates ~22A. OC values are clamped under it.
BOARD_OC_MAX_MA = 22000
# Reference startup current band (A). Calibrated so the 2810 lands on its proven
# sineRampModPct=5. Heavy/high-R motors stay current-matched, never OC-tripping.
RAMP_I_FRAC_OF_OC = 0.70          # ramp current ≈ 70% of OC soft limit …
RAMP_I_MIN, RAMP_I_MAX = 8.0, 15.0  # … clamped to a sane band
ALIGN_I_FRAC_OF_RAMP = 0.75       # align needs a bit less than ramp


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


@dataclass
class MotorSpec:
    name: str
    kv: float                 # RPM per volt
    r_pp_ohm: float           # phase-to-phase resistance (Ω)
    poles: int                # magnet poles (e.g. 14)
    weight_g: float           # rotor/motor mass (g) — inertia proxy
    max_current_a: float      # rated continuous current (A)
    vbus_nom: float = 24.0    # nominal bus voltage (V)
    inductance_uh: float | None = None   # phase inductance (µH), optional
    profile_id: int = 6
    enum_name: str = "CUSTOM"  # GSP_PROFILE_<enum_name>


@dataclass
class Profile:
    spec: MotorSpec
    pole_pairs: int
    no_load_erpm: int
    max_cl_erpm: int
    # OC chain (mA)
    oc_sw_ma: int
    oc_limit_ma: int
    oc_fault_ma: int
    oc_startup_ma: int
    ramp_current_gate_ma: int
    # startup
    sine_align_mod_pct: int
    sine_ramp_mod_pct: int
    align_duty_pct: int
    ramp_duty_pct: int
    cl_idle_duty_pct: int
    ramp_accel: int
    ramp_target_erpm: int
    initial_erpm: int
    timing_adv_deg: int
    zc_demag_duty_thresh: int
    zc_demag_blank_extra_pct: int
    # derived currents (for the report/comments)
    ramp_current_a: float
    align_current_a: float
    # FOC (unused at runtime in 6-step, but must exist so back_emf_obs.c compiles)
    foc_rs_mohm: int
    foc_ls_uh: int
    foc_ke_uv_s_rad: int
    ls_estimated: bool
    warnings: list = field(default_factory=list)


def compute_profile(spec: MotorSpec) -> Profile:
    pp = spec.poles // 2
    no_load = int(spec.kv * spec.vbus_nom * pp)
    max_cl = int(round(no_load * 1.05 / 1000.0) * 1000)   # ~5% over no-load, to 1k

    # ── OC chain, board-clamped, ordering enforced ──
    oc_sw = min(int(spec.max_current_a * 1000), BOARD_OC_MAX_MA - 4000)
    oc_limit = min(int(oc_sw * 1.25), BOARD_OC_MAX_MA - 2000)
    oc_fault = min(int(oc_limit * 1.05), BOARD_OC_MAX_MA - 1000)
    oc_startup = min(int(oc_fault * 1.05), BOARD_OC_MAX_MA)
    # guarantee sw < limit <= fault <= startup
    oc_limit = max(oc_limit, oc_sw + 500)
    oc_fault = max(oc_fault, oc_limit)
    oc_startup = max(oc_startup, oc_fault)

    oc_sw_a = oc_sw / 1000.0

    # ── startup currents, current-matched & OC-safe ──
    ramp_i = _clamp(RAMP_I_FRAC_OF_OC * oc_sw_a, RAMP_I_MIN, RAMP_I_MAX)
    align_i = ALIGN_I_FRAC_OF_RAMP * ramp_i

    def pct_amp(i):    # sine amplitude % (peak = pct/200 of Vbus)
        return _clamp(round(i * spec.r_pp_ohm / spec.vbus_nom * 200), 2, 50)

    def pct_duty(i, scale=1.0):   # trap duty % (= pct/100 of Vbus)
        return _clamp(round(i * spec.r_pp_ohm / spec.vbus_nom * 100 * scale), 2, 25)

    sine_ramp = pct_amp(ramp_i)
    sine_align = pct_amp(align_i)
    align_duty = pct_duty(align_i)
    ramp_duty = pct_duty(ramp_i, scale=1.5)          # trap path runs a bit hotter
    cl_idle_duty = _clamp(round(6.0 * spec.r_pp_ohm / spec.vbus_nom * 100), 5, 14)

    ramp_accel = int(_clamp(round(3000 * (spec.kv / 1350.0) * (90.0 / spec.weight_g)),
                            500, 5000))
    timing_adv = int(_clamp(round(10 + 15 * (max_cl / 260000.0)), 8, 28))
    initial_erpm = 150 if spec.weight_g < 100 else 100
    ramp_gate = min(oc_limit - 2000, int(ramp_i * 1000) + 2000)

    # ── FOC block (compile-only in 6-step) ──
    foc_rs = int(round(spec.r_pp_ohm / 2 * 1000))    # per-phase mΩ
    foc_ke = int(round(60.0 / (math.sqrt(3) * 2 * math.pi * spec.kv * pp) * 1e6))
    ls_est = spec.inductance_uh is None
    foc_ls = int(spec.inductance_uh) if spec.inductance_uh else 30

    prof = Profile(
        spec=spec, pole_pairs=pp, no_load_erpm=no_load, max_cl_erpm=max_cl,
        oc_sw_ma=oc_sw, oc_limit_ma=oc_limit, oc_fault_ma=oc_fault,
        oc_startup_ma=oc_startup, ramp_current_gate_ma=ramp_gate,
        sine_align_mod_pct=sine_align, sine_ramp_mod_pct=sine_ramp,
        align_duty_pct=align_duty, ramp_duty_pct=ramp_duty,
        cl_idle_duty_pct=cl_idle_duty, ramp_accel=ramp_accel,
        ramp_target_erpm=3000, initial_erpm=initial_erpm, timing_adv_deg=timing_adv,
        zc_demag_duty_thresh=45, zc_demag_blank_extra_pct=18,
        ramp_current_a=ramp_i, align_current_a=align_i,
        foc_rs_mohm=foc_rs, foc_ls_uh=foc_ls, foc_ke_uv_s_rad=foc_ke,
        ls_estimated=ls_est,
    )
    prof.warnings = validate(prof)
    return prof


def validate(p: Profile) -> list:
    w = []
    # OC-safety: recompute the ACTUAL current the emitted sine % implies
    vb, r = p.spec.vbus_nom, p.spec.r_pp_ohm
    ramp_a = p.sine_ramp_mod_pct / 200.0 * vb / r
    if ramp_a > p.oc_sw_ma / 1000.0:
        w.append(f"sineRampModPct={p.sine_ramp_mod_pct} → ~{ramp_a:.0f}A exceeds OC soft "
                 f"limit {p.oc_sw_ma/1000:.0f}A — lower it")
    if not (p.oc_sw_ma < p.oc_limit_ma <= p.oc_fault_ma <= p.oc_startup_ma):
        w.append("OC chain ordering violated (sw < limit ≤ fault ≤ startup)")
    if p.oc_startup_ma > BOARD_OC_MAX_MA:
        w.append(f"ocStartupMa {p.oc_startup_ma} > board shunt max {BOARD_OC_MAX_MA}")
    if p.ls_estimated:
        w.append("inductance was ESTIMATED (30µH) — affects only FOC + ZC blanking; "
                 "if ZC won't lock in MORPH, nudge zcDemagBlankExtraPct")
    if p.spec.kv * p.spec.vbus_nom * p.pole_pairs > 250000:
        w.append("no-load eRPM is very high — may approach the BEMF/speed ceiling near "
                 "full throttle (a top-end limit, not a start problem)")
    return w


# ── C code emitters ──────────────────────────────────────────────────────
def emit_gsp_params_c(p: Profile) -> str:
    s = p.spec
    return f"""    [GSP_PROFILE_{s.enum_name}] = {{
        /* === {s.name} ({s.poles}poles/{p.pole_pairs}PP, {s.vbus_nom:.0f}V, {s.weight_g:.0f}g) ===
         * Generated by Garuda Studio Wizard. Rs(ph-ph)={s.r_pp_ohm}Ω, KV={s.kv:.0f},
         * max cont {s.max_current_a:.0f}A. Startup current-matched: ramp≈{p.ramp_current_a:.1f}A,
         * align≈{p.align_current_a:.1f}A (both < OC soft {p.oc_sw_ma/1000:.0f}A). Iterate from
         * the GSP fault code; raise sineRampModPct if it stalls in OL_RAMP, lower if OC_SW. */
        .rampTargetErpm     = {p.ramp_target_erpm},
        .rampAccelErpmPerS  = {p.ramp_accel},
        .rampDutyPct        = {p.ramp_duty_pct},      /* trap-path cap (sine uses sineRampModPct) */
        .clIdleDutyPct      = {p.cl_idle_duty_pct},
        .timingAdvMaxDeg    = {p.timing_adv_deg},
        .hwzcCrossoverErpm  = 1500,
        .ocSwLimitMa        = {p.oc_sw_ma},
        .ocFaultMa          = {p.oc_fault_ma},
        .motorPolePairs     = {p.pole_pairs},
        .alignDutyPct       = {p.align_duty_pct},
        .initialErpm        = {p.initial_erpm},
        .maxClosedLoopErpm  = {p.max_cl_erpm},   /* {s.kv:.0f} * {s.vbus_nom:.0f}V * {p.pole_pairs}pp ≈ {p.no_load_erpm:,} */
        .sineAlignModPct    = {p.sine_align_mod_pct},
        .sineRampModPct     = {p.sine_ramp_mod_pct},
        .zcDemagDutyThresh  = {p.zc_demag_duty_thresh},
        .zcDemagBlankExtraPct = {p.zc_demag_blank_extra_pct},
        .ocLimitMa          = {p.oc_limit_ma},   /* CMP3 chop */
        .ocStartupMa        = {p.oc_startup_ma},
        .rampCurrentGateMa  = {p.ramp_current_gate_ma},
        /* --- FOC (unused at runtime in 6-step; present so back_emf_obs.c compiles) --- */
        .focRsMilliOhm       = {p.foc_rs_mohm},
        .focLsMicroH         = {p.foc_ls_uh},{'   /* ESTIMATE — measure */' if p.ls_estimated else ''}
        .focKeUvSRad         = {p.foc_ke_uv_s_rad},
        .focVbusNomCentiV    = {int(s.vbus_nom*100)},
        .focMaxCurrentCentiA = {int(s.max_current_a*100)},
        .focMaxElecRadS      = 9000,
        .focKpDqMilli        = 188,
        .focKiDq             = 590,
        .focObsLpfAlphaMilli = 200,
        .focAlignIqCentiA    = 400,
        .focRampIqCentiA     = 500,
        .focAlignTimeMs      = 800,
        .focIqRampTimeMs     = 300,
        .focRampRateRps2     = 200,
        .focHandoffRadS      = 800,
        .focFaultOcCentiA    = {int(s.max_current_a*100*1.2)},
        .focFaultStallDeciRadS = 50,
    }},"""


def emit_foc_params_h(p: Profile) -> str:
    s = p.spec
    return f"""#elif MOTOR_PROFILE=={s.profile_id}
/* {s.name} — FOC macros (compile-only in 6-step build) */
#define MOTOR_RS_OHM        {s.r_pp_ohm/2:.4f}f
#define MOTOR_LS_H          {p.foc_ls_uh/1e6:.7f}f{'   /* ESTIMATE */' if p.ls_estimated else ''}
#define MOTOR_KE_VPEAK      {p.foc_ke_uv_s_rad/1e6:.6f}f
#define MOTOR_POLE_PAIRS_FOC {p.pole_pairs}
#define MOTOR_VBUS_NOM      {s.vbus_nom:.1f}f
#define MOTOR_I_MAX         {s.max_current_a:.1f}f
#define OBS_LPF_ALPHA       0.2f"""


def emit_enum_and_config(p: Profile) -> str:
    s = p.spec
    return f"""/* ─ gsp/gsp_params.h : add to the GSP_PROFILE_* enum (then bump COUNT) ─ */
    GSP_PROFILE_{s.enum_name} = {s.profile_id},

/* ─ garuda_config.h : selector + a #elif block, and resize profileDefaults[] ─ */
//#define MOTOR_PROFILE {s.profile_id}   // {s.name}
#elif MOTOR_PROFILE=={s.profile_id}
  /* {s.name}: {s.kv:.0f}KV, {s.r_pp_ohm}Ω ph-ph, {s.poles} poles, {s.weight_g:.0f}g, {s.max_current_a:.0f}A */
"""


def render_all(p: Profile) -> str:
    bar = "/* " + "═" * 72 + " */\n"
    out = [bar,
           f"/* Garuda Studio — generated profile for {p.spec.name}  "
           f"(profile {p.spec.profile_id}, GSP_PROFILE_{p.spec.enum_name}) */",
           bar.rstrip()]
    if p.warnings:
        out.append("/* ⚠ WARNINGS:")
        for w in p.warnings:
            out.append(f" *   - {w}")
        out.append(" */")
    out.append("\n/* ── 1/4  gsp/gsp_params.c  (runtime source) ── */")
    out.append(emit_gsp_params_c(p))
    out.append("\n/* ── 2/4  garuda_foc_params.h ── */")
    out.append(emit_foc_params_h(p))
    out.append("\n/* ── 3/4 + 4/4  gsp_params.h enum + garuda_config.h ── */")
    out.append(emit_enum_and_config(p))
    out.append("\n/* Build-test: set MOTOR_PROFILE=%d, `make clean && make`. */"
               % p.spec.profile_id)
    return "\n".join(out)


def report(p: Profile) -> str:
    s = p.spec
    return (f"{s.name}: {p.pole_pairs}PP  no-load≈{p.no_load_erpm:,} eRPM  cap={p.max_cl_erpm:,}\n"
            f"  startup: sineAlign={p.sine_align_mod_pct}(~{p.align_current_a:.1f}A)  "
            f"sineRamp={p.sine_ramp_mod_pct}(~{p.ramp_current_a:.1f}A)  "
            f"accel={p.ramp_accel}  timingAdv={p.timing_adv_deg}\n"
            f"  OC chain (mA): sw={p.oc_sw_ma} < limit={p.oc_limit_ma} "
            f"≤ fault={p.oc_fault_ma} ≤ startup={p.oc_startup_ma}\n"
            + ("  ⚠ " + " | ".join(p.warnings) if p.warnings else "  ✓ no warnings"))


def main(argv=None):
    ap = argparse.ArgumentParser(description="Garuda motor profile wizard")
    ap.add_argument("--name", required=True)
    ap.add_argument("--kv", type=float, required=True)
    ap.add_argument("--r", type=float, required=True, help="phase-to-phase resistance (Ω)")
    ap.add_argument("--poles", type=int, required=True)
    ap.add_argument("--weight", type=float, required=True, help="grams")
    ap.add_argument("--imax", type=float, required=True, help="rated continuous A")
    ap.add_argument("--vbus", type=float, default=24.0)
    ap.add_argument("--ls", type=float, default=None, help="phase inductance (µH), optional")
    ap.add_argument("--profile", type=int, default=6)
    ap.add_argument("--enum", default="CUSTOM")
    ap.add_argument("--report-only", action="store_true")
    a = ap.parse_args(argv)
    spec = MotorSpec(name=a.name, kv=a.kv, r_pp_ohm=a.r, poles=a.poles,
                     weight_g=a.weight, max_current_a=a.imax, vbus_nom=a.vbus,
                     inductance_uh=a.ls, profile_id=a.profile, enum_name=a.enum)
    p = compute_profile(spec)
    print(report(p) if a.report_only else render_all(p))


if __name__ == "__main__":
    main()
