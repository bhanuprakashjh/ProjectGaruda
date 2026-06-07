"""garuda SPICE engine — switching-level 6-step floating-phase ZC model. EXPERIMENTAL.

STATUS (2026-06-07): runs (ngspice via PySpice) and shows the switching-level
floating-phase waveform, but does NOT yet quantitatively reproduce the bench
falling-masking from first principles, even with schematic-grounded parameters
(MCLV-48V-300W DS50003297: SUM60020E FETs, 10nF+2.9k RC, BAS40-04 clamp). The
gap is model fidelity (reduced single-sector netlist + ideal switches + a fragile
detection metric), not component values. A faithful proof needs a PLECS/Simulink-
Simscape electromechanical co-sim with Vishay's SUM60020E device model. The
TRUSTWORTHY parts are the bench data, the committed fix, and the lumped/mechanism
model in zcsim.py (which IS bench-faithful). Treat this as a physics sandbox.


Runs in the .venv-spice venv (PySpice -> libngspice.so.0). The GUI shells out to
this script; it prints JSON. Unlike the lumped numpy model, the floating-phase
ON-window corruption EMERGES from device physics (FET Coss + body diodes + motor
L + the floating-node stray C resonance the patent describes), not injected.

For one operating point (eRPM, duty, falling/rising sector) it simulates one
sector: phase A = high-side PWM, B = low-side on, C = floating. C's BEMF sweeps
through zero (the ZC). We record the floating node V(c) and the RC-filtered sense
node, split by PWM ON/OFF window, and report envelopes + whether a comparator
(ON-gated) or OFF-center sample would cleanly detect the crossing.

Usage:  python spice_engine.py '{"erpm":90000,"duty":0.43,"sector":"falling"}'
"""
from __future__ import annotations
import json
import sys
import numpy as np

KV = 1350.0
POLES_PP = 7
KE = 60.0 / (2 * np.pi * KV)        # V*s/rad mech
ADC_PER_V = 625.0 / 12.0
PWM_HZ = 45000.0


def _gate_pwl(window, duty, *, deadtime=0.2e-6, invert=False):
    """Center-aligned PWM gate PWL points over `window` (s). High-side: ON in the
    central `duty` fraction (so the period boundary = OFF-center). invert -> the
    complementary low-side gate (with deadtime on both edges)."""
    Tp = 1.0 / PWM_HZ
    pts = [(0.0, 0.0)]
    n = int(np.ceil(window / Tp)) + 1
    tr = 10e-9
    for i in range(n):
        t0 = i * Tp
        on_s = t0 + (1.0 - duty) / 2.0 * Tp
        on_e = t0 + (1.0 + duty) / 2.0 * Tp
        if not invert:                       # high-side: 0 -> 1 during [on_s,on_e]
            for t, v in [(on_s - tr, 0), (on_s, 1), (on_e, 1), (on_e + tr, 0)]:
                if 0 <= t <= window:
                    pts.append((t, v))
        else:                                # low-side: 1 except during ON (+deadtime)
            for t, v in [(on_s - deadtime - tr, 1), (on_s - deadtime, 0),
                         (on_e + deadtime, 0), (on_e + deadtime + tr, 1)]:
                if 0 <= t <= window:
                    pts.append((t, v))
    pts.append((window, pts[-1][1]))
    # dedupe/sort by time
    pts = sorted({round(t, 12): v for t, v in pts}.items())
    return " ".join(f"{t:.10g} {v:g}" for t, v in pts)


def build_netlist(erpm, duty, sector, *, R=0.05, L=30e-6, coss=1.2e-9,
                  cstray=3e-9, rf=2.9e3, cf=10e-9, kc=0.6, cpar=120e-12):
    # Grounded in MCLV-48V-300W schematic (DS50003297, sheet 2): MOSFET SUM60020E
    # (Coss ~1.2nF), BEMF sense = divider + 10nF + 2.9k (->5.5kHz) + BAS40-04 clamp
    # to AVcc(3.3V). Divider ratio ~1/24 folded into ADC_PER_V.
    w_e = erpm / 60.0 * 2 * np.pi
    w_m = w_e / POLES_PP
    E = KE * w_m
    T_e = 1.0 / (erpm / 60.0)
    window = T_e / 6.0                        # one sector
    rcfc = 1.0 / (2 * np.pi * rf * cf)
    # falling: floating C ramps +E -> -E. rising: -E -> +E. driven A=+E, B=-E.
    ec0, ec1 = (E, -E) if sector == "falling" else (-E, E)
    gah = _gate_pwl(window, duty, invert=False)
    gal = _gate_pwl(window, duty, invert=True)
    nl = f"""* 6-step floating-phase ZC (sector={sector}, erpm={erpm}, duty={duty})
.model SWM SW(ron=12m roff=1meg vt=0.5 vh=0.05)
.model DBD D(is=2e-12 rs=8m n=1.6 cjo=300p)
Vbus dcp 0 DC 24
Cbus dcp 0 100u
* --- half bridge A : high-side PWM, low-side complementary ---
SAH dcp na gah 0 SWM
DAH na dcp DBD
CossAH dcp na {coss}
SAL na 0 gal 0 SWM
DAL 0 na DBD
CossAL na 0 {coss}
* --- half bridge B : low-side held ON ---
SBL nb 0 vone 0 SWM
DBL 0 nb DBD
CossBL nb 0 {coss}
DBH nb dcp DBD
CossBH dcp nb {coss}
* --- half bridge C : floating (both off), body diodes + Coss present ---
DCH nc dcp DBD
CossCH dcp nc {coss}
DCL 0 nc DBD
CossCL nc 0 {coss}
* --- motor: phase R+L + BEMF source to neutral nn ---
La na ma {L}
Ra ma a2 {R}
VeA a2 nn PWL(0 {E:.6g} {window:.10g} {E:.6g})
Lb nb mb {L}
Rb mb b2 {R}
VeB b2 nn PWL(0 {-E:.6g} {window:.10g} {-E:.6g})
Lc nc mc {L}
Rc mc c2 {R}
VeC c2 nn PWL(0 {ec0:.6g} {window:.10g} {ec1:.6g})
Rnn nn 0 1meg
* mutual inductance between phases (3-phase machine coupling) -> the di/dt of the
* PWM-driven phase induces voltage in the FLOATING phase asymmetrically per sector.
* This is the source of the rising/falling ON-bias asymmetry.
K12 La Lb {kc}
K23 Lb Lc {kc}
K13 La Lc {kc}
* floating-node stray C (resonates with Lc -> the patent's ringing)
Cstray nc 0 {cstray}
* sense network: RC low-pass (fc~{rcfc:.0f}Hz) + a parasitic feedthrough cap that
* passes the fast switching edges the RC can't kill -> the comparator (1MHz) sees
* them as ON-window spikes while the OFF-center boundary sample stays clean.
Rf nc sns {rf}
Cf sns 0 {cf}
Cpar nc sns {cpar}
* gates
VgAH gah 0 PWL({gah})
VgAL gal 0 PWL({gal})
Vone vone 0 DC 1
.options reltol=2e-3 abstol=1e-7 vntol=1e-4 method=gear
.tran 20n {window:.10g} uic
.end
"""
    return nl, dict(E=E, window=window, rcfc=rcfc)


def run(params):
    erpm = int(params["erpm"]); duty = float(params["duty"])
    sector = params.get("sector", "falling")
    nl, meta = build_netlist(erpm, duty, sector)
    from PySpice.Spice.NgSpice.Shared import NgSpiceShared
    ng = NgSpiceShared.new_instance()
    ng.load_circuit(nl)
    try:
        ng.run()              # PySpice raises a false failure on ngspice v43; data is fine
    except Exception:
        pass
    plot = ng.plot(None, ng.last_plot)
    def vec(name):
        return np.asarray(plot[name]._data).real
    t = vec("time")
    vc = vec("nc")           # floating-phase node (volts)
    vs = vec("sns")          # RC-filtered sense (volts)
    # PWM ON/OFF mask reconstructed analytically (center-aligned, matches gates)
    Tp = 1.0 / PWM_HZ
    on = np.abs((t % Tp) / Tp - 0.5) < (duty / 2.0)
    # ADC scale + threshold (firmware: duty*Vbus/2)
    adc = lambda v: np.clip(v * ADC_PER_V, 0, 4095)
    thr = duty * 24.0 / 2.0 * ADC_PER_V
    vc_adc, vs_adc = adc(vc), adc(vs)
    pol = -1 if sector == "falling" else +1
    window = meta["window"]
    floor = 0.25 * window                 # firmware plausibility floor (1/4 sector)
    true_zc = 0.5 * window                 # ideal ZC at mid-sector
    db = 4.0                               # HWZC_CMP_DEADBAND (ADC counts)

    def edge_detect(times, sig):
        """First per-polarity threshold crossing past the floor + total crossing
        count (noise). 'clean' = first crossing lands near the true ZC (i.e. it
        wasn't a switching-kick false-fire earlier in the sector)."""
        n, first = 0, None
        for i in range(1, times.size):
            if times[i] < floor:
                continue
            a, b = sig[i - 1] - thr, sig[i] - thr
            hit = (a <= db and b > db) if pol > 0 else (a >= -db and b < -db)
            if hit:
                n += 1
                if first is None:
                    first = times[i]
        clean = first is not None and abs(first - true_zc) < 0.20 * window
        return (float(first * 1e6) if first is not None else None, n, bool(clean))

    # comparator: the 1 MHz HW comparator reads the sense line, GATED to PWM-ON.
    # detect only within contiguous ON runs (no edges across OFF gaps).
    on_t = t.copy(); on_v = vs_adc.copy()
    keep = on & np.roll(on, 1)            # both this and prev sample in ON
    keep[0] = False
    comp_first, comp_n, comp_clean = edge_detect(on_t[keep], on_v[keep]) \
        if keep.any() else (None, 0, False)
    comp_on = vs_adc[on]
    comp_env = (float(comp_on.min()), float(comp_on.max())) if comp_on.size else (0., 0.)

    # OFF-center: one sample per PWM period at the boundary (mid-OFF, clean).
    ph = (t % Tp)
    bidx = np.where(np.diff(ph) < 0)[0] + 1
    bt, bv = t[bidx], vs_adc[bidx]
    offc_first, offc_n, offc_clean = edge_detect(bt, bv) if bt.size > 1 else (None, 0, False)
    off_env = (float(bv.min()), float(bv.max())) if bv.size else (0., 0.)

    if t.size > 1500:
        idx = np.linspace(0, t.size - 1, 1500).astype(int)
    else:
        idx = np.arange(t.size)
    return {
        "ok": True, "sector": sector, "erpm": erpm, "duty": duty,
        "E_volts": meta["E"], "thr_adc": thr, "rcfc": meta["rcfc"],
        "t_us": (t[idx] * 1e6).tolist(),
        "v_node_adc": vc_adc[idx].tolist(),
        "v_sense_adc": vs_adc[idx].tolist(),
        "on_mask": on[idx].astype(int).tolist(),
        "comp_on_env": comp_env, "offc_env": off_env,
        # faithful detection: clean = caught the real ZC; n_cross = noise crossings
        "comp_detect": comp_clean, "comp_ncross": comp_n, "comp_first_us": comp_first,
        "offc_detect": offc_clean, "offc_ncross": offc_n, "offc_first_us": offc_first,
    }


if __name__ == "__main__":
    p = json.loads(sys.argv[1]) if len(sys.argv) > 1 else {"erpm": 90000, "duty": 0.43, "sector": "falling"}
    try:
        print(json.dumps(run(p)))
    except Exception as e:
        import traceback
        print(json.dumps({"ok": False, "error": str(e),
                          "trace": traceback.format_exc()[-1500:]}))
