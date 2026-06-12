"""experiment_am32.py — twin study #4: AM32-style "kick + listen" startup
(FEATURE_AM32_STARTUP: no align, no ramp, no blind schedule — one blind
commutation at MIN_DUTY, HWZC + sector PI trusted from event 1).

Success = rotor PHYSICALLY at a sane idle AND fw tracking it (ratio ~1).
Failure modes of interest:
  - fiction lock: fw eRPM sane, plant eRPM ~0 (phantom-capture lock)
  - grind: rotor oscillates, never builds speed
  - fault (OC/desync recovery cycle)
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from garuda_sil import Sim, Plant2810, STATES, PWM_HZ

HERE = os.path.dirname(os.path.abspath(__file__))
SO = os.path.join(HERE, "..", "libgaruda_sil_am32.so")
CL, FAULT = STATES.index("CLOSED_LOOP"), STATES.index("FAULT")


def one_start(theta0, trace=False, deadline_s=6.0):
    sim = Sim(plant=Plant2810(vbus=24.3, theta0_deg=theta0), pot=40, so_path=SO)
    sim.run(1000)
    sim.cmd_start()
    ipk, t_cl = 0.0, None
    k, deadline = 0, int(deadline_s * PWM_HZ)
    while k < deadline:
        sim.step_tick(); k += 1
        t = k / PWM_HZ
        if k % 45 == 0:
            ipk = max(ipk, max(abs(x) for x in sim.plant.i_ph))
        st = sim.lib.sil_state()
        if st == CL and t_cl is None:
            t_cl = t
        if trace and t_cl is not None and k % (45 * 100) == 0:   # 10 Hz
            T = sim.lib.sil_hwzc_period()
            fw = (1_000_000_000 // T) if T else 0
            print(f"    t={t:5.2f}s fw={fw:6d} rotor={sim.plant.erpm():6.0f} "
                  f"cap={sim.lib.sil_hwzc_cap_frac_pm()/1000:.2f}T "
                  f"ipk={ipk:5.1f}A")
        if st == FAULT:
            return dict(ok=False, why=f"FAULT:{sim.fault_name()}",
                        t_cl=t_cl, ipk=ipk, rotor=sim.plant.erpm())
    # verdict at deadline
    T = sim.lib.sil_hwzc_period()
    fw = (1_000_000_000 // T) if T else 0
    rotor = sim.plant.erpm()
    ratio = fw / max(rotor, 1.0)
    ok = rotor > 8000 and 0.85 < ratio < 1.15
    why = ("OK" if ok else
           "FICTION-LOCK" if rotor < 1000 and fw > 2000 else
           f"PARTIAL(rotor={rotor:.0f}, fw={fw})")
    return dict(ok=ok, why=why, t_cl=t_cl, ipk=ipk, rotor=rotor,
                fw=fw, ratio=ratio)


def main():
    print("=== detailed trace, theta0=0 ===")
    r = one_start(0, trace=True, deadline_s=4.0)
    print("   ", {k: (round(v, 2) if isinstance(v, float) else v)
                  for k, v in r.items()})

    print("\n=== 12-angle sweep ===")
    oks = 0
    for a in range(0, 360, 30):
        r = one_start(a)
        if r["ok"]:
            oks += 1
        print(f"  θ0={a:3d}°  {r['why']:14s} rotor={r['rotor']:6.0f} "
              f"fw={r.get('fw', 0):6d} ipk={r['ipk']:.1f}A")
    print(f"  -- success {oks}/12")


if __name__ == "__main__":
    main()
