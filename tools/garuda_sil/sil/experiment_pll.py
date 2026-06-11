"""experiment_pll.py — twin design study #2 (task #10):
FEATURE_PLL_STARTUP — ALIGN -> CL direct, blind accelerating schedule,
gated capture handover. Iteration #4: the 10x accel unit fix (T/1e9 -> T/1e8).

Per start: detailed schedule-vs-rotor trace + sync handover time, then a
12-angle sweep with the same success criterion as experiment_skipmorph.
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from garuda_sil import Sim, Plant2810, STATES, PWM_HZ

HERE = os.path.dirname(os.path.abspath(__file__))
SO = os.path.join(HERE, "..", "libgaruda_sil_pll.so")
CL, FAULT = STATES.index("CLOSED_LOOP"), STATES.index("FAULT")


def one_start(theta0, trace=False, deadline_s=6.0, hold_s=1.0):
    sim = Sim(plant=Plant2810(vbus=24.3, theta0_deg=theta0), pot=40, so_path=SO)
    sim.run(1000)
    sim.cmd_start()
    ipk, t_cl, t_sync, hold_end = 0.0, None, None, None
    k, deadline = 0, int(deadline_s * PWM_HZ)
    while k < deadline:
        sim.step_tick(); k += 1
        t = k / PWM_HZ
        if k % 45 == 0:
            ipk = max(ipk, max(abs(x) for x in sim.plant.i_ph))
        st = sim.lib.sil_state()
        if st == CL and t_cl is None:
            t_cl = t
        if t_cl is not None and t_sync is None and sim.lib.sil_zc_synced():
            t_sync = t
            hold_end = k + int(hold_s * PWM_HZ)
        if trace and t_cl is not None and k % (45 * 100) == 0:   # 10 Hz
            T = sim.lib.sil_hwzc_period()
            cmd = (1_000_000_000 // T) if T else 0
            print(f"    t={t:5.2f}s cmd={cmd:6d} rotor={sim.plant.erpm():6.0f} "
                  f"sync={sim.lib.sil_zc_synced()} good_zc={sim.lib.sil_good_zc()} "
                  f"cap={sim.lib.sil_hwzc_cap_frac_pm()/1000:.2f}T "
                  f"pllgood={sim.lib.sil_pll_good()} ipk={ipk:5.1f}A")
        if st == FAULT:
            return dict(ok=False, why=sim.fault_name(), t_cl=t_cl,
                        t_sync=t_sync, ipk=ipk)
        if hold_end is not None and k >= hold_end:
            T = sim.lib.sil_hwzc_period()
            cmd = (1_000_000_000 // T) if T else 0
            ratio = cmd / max(sim.plant.erpm(), 1.0)
            return dict(ok=True, t_cl=t_cl, t_sync=t_sync, ipk=ipk,
                        idle=sim.plant.erpm(), ratio=ratio)
    return dict(ok=False, why=f"TIMEOUT(sync={bool(sim.lib.sil_zc_synced())}, "
                              f"state={sim.state_name()}, "
                              f"rotor={sim.plant.erpm():.0f})",
                t_cl=t_cl, t_sync=t_sync, ipk=ipk)


def main():
    print("=== detailed trace, theta0=0 ===")
    r = one_start(0, trace=True)
    print("   ", r)

    print("\n=== 12-angle sweep ===")
    oks, syncs, ipks = 0, [], []
    for a in range(0, 360, 30):
        r = one_start(a)
        ipks.append(r["ipk"])
        if r["ok"]:
            oks += 1; syncs.append(r["t_sync"])
            print(f"  θ0={a:3d}°  OK   t_CL={r['t_cl']:.2f}s  "
                  f"t_sync={r['t_sync']:.2f}s  fw/rotor={r['ratio']:.2f}  "
                  f"ipk={r['ipk']:.1f}A  idle={r['idle']:.0f}")
        else:
            print(f"  θ0={a:3d}°  FAIL {r['why']}  t_sync={r['t_sync']}  "
                  f"ipk={r['ipk']:.1f}A")
    print(f"  -- success {oks}/12", end="")
    if syncs:
        print(f"  t_sync {min(syncs):.2f}-{max(syncs):.2f}s  "
              f"ipk_max {max(ipks):.1f}A")
    else:
        print()


if __name__ == "__main__":
    main()
