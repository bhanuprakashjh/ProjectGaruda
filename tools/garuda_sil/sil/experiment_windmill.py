"""Twin design study #2: WINDMILL starts — rotor already spinning at start
command. baseline(morph) vs skipmorph(+coast-verify), initial eRPM sweep.
Entry ratio measured 100 ms after CL entry (post-engage)."""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from garuda_sil import Sim, Plant2810, STATES, PWM_HZ
HERE = os.path.dirname(os.path.abspath(__file__))
VARIANTS = {
    "baseline":  os.path.join(HERE, "..", "libgaruda_sil.so"),
    "skipmorph": os.path.join(HERE, "..", "libgaruda_sil_skipmorph.so"),
}
CL, FAULT = STATES.index("CLOSED_LOOP"), STATES.index("FAULT")
import math
def one(so, erpm0):
    # COAST-FAITHFUL plant config (drag model v2): physical J, light viscous,
    # Vbus-scaled volt-drop. Coast TC ~1.5 s like the bench, so windmill spin
    # survives the 0.5 s ARM window. Equilibrium runs ~50% high in this config
    # (twin angle-bias, see NOTES.md) — fine for a comparative catch study.
    plant = Plant2810(vbus=24.3, j=5.0e-6, b_visc=3.3e-6,
                      v_drop_a=0.0822, v_drop_c=0.0053)
    plant.omega_m = erpm0 / 7.0 / 60.0 * 2 * math.pi   # spin it up first
    sim = Sim(plant=plant, pot=40, so_path=so)
    sim.run(200)
    sim.cmd_start()
    ipk, t_cl, entry, hold_end = 0.0, None, None, None
    for k in range(int(6.0 * PWM_HZ)):
        sim.step_tick()
        if k % 45 == 0:
            ipk = max(ipk, max(abs(x) for x in sim.plant.i_ph))
        st = sim.lib.sil_state()
        if st == CL and t_cl is None:
            t_cl = k / PWM_HZ; hold_end = k + int(0.5 * PWM_HZ)
            entry_at = k + int(0.1 * PWM_HZ)
        if t_cl is not None and entry is None and k >= entry_at:
            entry = sim.lib.sil_erpm() / max(sim.plant.erpm(), 1.0)
        if st == FAULT:
            return f"FAIL {sim.fault_name():14s} ipk={ipk:5.1f}A t_CL={t_cl}"
        if t_cl is not None and k >= hold_end:
            return (f"OK  t_CL={t_cl:4.2f}s entry={entry:4.2f} "
                    f"ipk={ipk:5.1f}A idle={sim.plant.erpm():6.0f}")
    return f"FAIL TIMEOUT({sim.state_name()}) ipk={ipk:5.1f}A"

for name, so in VARIANTS.items():
    print(f"=== {name} ===")
    for erpm0 in (0, 2000, 5000, 10000, 15000):
        print(f"  spin0={erpm0:6d}  {one(so, erpm0)}")
