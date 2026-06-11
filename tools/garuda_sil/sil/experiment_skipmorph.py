"""experiment_skipmorph.py — twin design study #1:
baseline (morph) vs FEATURE_SKIP_MORPH+CL_COAST_VERIFY startup,
swept over 12 initial rotor angles (deterministic twin = controlled study).

Metrics per start: success (reach CL and hold 0.5 s), time to CL, CL-entry
fw-vs-plant eRPM ratio (engage truthfulness), peak phase current to CL+0.5 s.
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from garuda_sil import Sim, Plant2810, STATES, PWM_HZ

HERE = os.path.dirname(os.path.abspath(__file__))
VARIANTS = {
    "baseline":  os.path.join(HERE, "..", "libgaruda_sil.so"),
    "skipmorph": os.path.join(HERE, "..", "libgaruda_sil_skipmorph.so"),
}
CL, FAULT = STATES.index("CLOSED_LOOP"), STATES.index("FAULT")


def one_start(so, theta0):
    sim = Sim(plant=Plant2810(vbus=24.3, theta0_deg=theta0), pot=40, so_path=so)
    sim.run(1000)
    sim.cmd_start()
    ipk, t_cl = 0.0, None
    deadline = int(5.0 * PWM_HZ)
    k = 0
    while k < deadline:
        sim.step_tick(); k += 1
        if k % 45 == 0:                       # ~1 kHz current sampling
            ipk = max(ipk, max(abs(x) for x in sim.plant.i_ph))
        st = sim.lib.sil_state()
        if st == CL and t_cl is None:
            t_cl = k / PWM_HZ
            entry_ratio = sim.lib.sil_erpm() / max(sim.plant.erpm(), 1.0)
            hold_end = k + int(0.5 * PWM_HZ)
        if st == FAULT:
            return dict(ok=False, why=sim.fault_name(), t_cl=t_cl, ipk=ipk)
        if t_cl is not None and k >= hold_end:
            return dict(ok=True, t_cl=t_cl, entry=entry_ratio, ipk=ipk,
                        idle=sim.plant.erpm())
    return dict(ok=False, why="TIMEOUT(state=%s)" % sim.state_name(),
                t_cl=t_cl, ipk=ipk)


def main():
    angles = list(range(0, 360, 30))
    for name, so in VARIANTS.items():
        print(f"\n=== {name} ===")
        oks, tcls, entries, ipks = 0, [], [], []
        for a in angles:
            r = one_start(so, a)
            if r["ok"]:
                oks += 1; tcls.append(r["t_cl"]); entries.append(r["entry"])
                ipks.append(r["ipk"])
                print(f"  θ0={a:3d}°  OK   t_CL={r['t_cl']:.2f}s  "
                      f"entry fw/plant={r['entry']:.2f}  ipk={r['ipk']:.1f}A  "
                      f"idle={r['idle']:.0f}")
            else:
                ipks.append(r["ipk"])
                print(f"  θ0={a:3d}°  FAIL {r['why']}  t_CL={r['t_cl']}  "
                      f"ipk={r['ipk']:.1f}A")
        n = len(angles)
        print(f"  -- success {oks}/{n}", end="")
        if tcls:
            print(f"  t_CL {min(tcls):.2f}-{max(tcls):.2f}s"
                  f"  entry-ratio {min(entries):.2f}-{max(entries):.2f}"
                  f"  ipk_max {max(ipks):.1f}A")
        else:
            print()


if __name__ == "__main__":
    main()
