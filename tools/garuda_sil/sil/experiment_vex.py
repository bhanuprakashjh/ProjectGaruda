"""experiment_vex.py — twin study #5: does the VEX 4000KV micro motor start
with the new startups (profile 6 firmware: MOTOR_PROFILE=6)?

VEX 14mm: 4000KV, 6pp, 0.44Ω line-line, 9.2µH/ph, 10V max, stall 14A.
Profile 6: rampTarget 12000 (AM32 derived seed = 8000), crossover 6000
(PLL floor auto-raises to 6000).

KEY PHYSICS this models (int-quantized ADC):
  BEMF line-neutral ≈ 0.021 mV/eRPM → ~3 ADC counts @3k, ~6 @6k, ~12 @12k
  (vs the 4-8 count comparator deadband — the colleague's wall).
  MIN_DUTY equilibrium ≈ only a few k eRPM at 10V → idle pot may sit BELOW
  the capture floor; higher pot gives the listener real BEMF to work with.

Unknowns (swept): rotor inertia J, viscous drag b. Plant is UNCALIBRATED
for this motor — results are qualitative go/no-go + failure-mode map.
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from garuda_sil import Sim, Plant2810, STATES, PWM_HZ

HERE = os.path.dirname(os.path.abspath(__file__))
SO = {
    "AM32":     os.path.join(HERE, "..", "libgaruda_sil_vex_am32.so"),
    "PLL":      os.path.join(HERE, "..", "libgaruda_sil_vex_pll.so"),
    "PLL_a8k":  os.path.join(HERE, "..", "libgaruda_sil_vex_pll_a8k.so"),
}
CL, FAULT = STATES.index("CLOSED_LOOP"), STATES.index("FAULT")


def vex_plant(theta0, j, b):
    return Plant2810(vbus=10.0, kv=4000.0, pp=6, r_ll=0.44,
                     j=j, b_visc=b, tau_c=0.0,
                     v_drop_a=0.08, v_drop_c=0.0,
                     theta0_deg=theta0)


def one_start(so, theta0, j, b, pot, deadline_s=6.0, trace=False):
    # ARM at idle pot (arming requires ~zero throttle), raise to the target
    # pot only after CL entry — the real operator procedure.
    sim = Sim(plant=vex_plant(theta0, j, b), pot=40, so_path=so)
    sim.run(1000)
    sim.cmd_start()
    ipk, t_cl = 0.0, None
    k, deadline = 0, int(deadline_s * PWM_HZ)
    while k < deadline:
        sim.step_tick(); k += 1
        if t_cl is not None and sim.pot != pot and k > (t_cl * PWM_HZ) + 9000:
            sim.pot = pot                     # +0.2s after CL entry
        if k % 45 == 0:
            ipk = max(ipk, max(abs(x) for x in sim.plant.i_ph))
        st = sim.lib.sil_state()
        if st == CL and t_cl is None:
            t_cl = k / PWM_HZ
        if trace and t_cl is not None and k % (45 * 100) == 0:
            T = sim.lib.sil_hwzc_period()
            fw = (1_000_000_000 // T) if T else 0
            print(f"    t={k/PWM_HZ:5.2f}s fw={fw:6d} rotor={sim.plant.erpm():6.0f} "
                  f"sync={sim.lib.sil_zc_synced()} pll={sim.lib.sil_pll_active()} "
                  f"cap={sim.lib.sil_hwzc_cap_frac_pm()/1000:.2f}T ipk={ipk:4.1f}A")
        if st == FAULT:
            return dict(ok=False, why=f"FAULT:{sim.fault_name()}",
                        rotor=sim.plant.erpm(), fw=0, ipk=ipk)
    T = sim.lib.sil_hwzc_period()
    fw = (1_000_000_000 // T) if T else 0
    rotor = sim.plant.erpm()
    ratio = fw / max(rotor, 1.0)
    spinning = rotor > 2000
    tracking = 0.8 < ratio < 1.25
    ok = spinning and tracking and sim.state_name() == "CLOSED_LOOP"
    why = ("OK" if ok else
           "FICTION" if fw > 3000 and rotor < 1000 else
           "STALL" if rotor < 1000 else
           f"PARTIAL(state={sim.state_name()})")
    return dict(ok=ok, why=why, rotor=rotor, fw=fw, ipk=ipk)


def main():
    js = [2e-7, 5e-7, 1e-6]
    b = 2e-6
    pots = [40, 250]
    angles = [0, 90, 180, 270]

    for name, so in SO.items():
        print(f"\n===== {name} startup, VEX profile =====")
        # one detailed trace first
        print(f"  trace: J=2e-7 pot=250")
        r = one_start(so, 0, 2e-7, b, 250, trace=True)
        print("   ", {k: (round(v, 2) if isinstance(v, float) else v)
                      for k, v in r.items()})
        for j in js:
            for pot in pots:
                oks, whys = 0, []
                for a in angles:
                    r = one_start(so, a, j, b, pot)
                    oks += r["ok"]
                    whys.append(f"{r['why']}(r={r['rotor']:.0f},fw={r['fw']})")
                print(f"  J={j:.0e} pot={pot:3d}: {oks}/4   " + "  ".join(whys))


if __name__ == "__main__":
    main()
