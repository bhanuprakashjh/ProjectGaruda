"""M1 acceptance test (DESIGN.md) — run with the garuda_debug venv python:
  .../tools/garuda_debug/.venv/bin/python sil/test_m1.py
Plain script (asserts); pytest-compatible.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from garuda_sil import Sim, Plant2810, STATES

CL = STATES.index("CLOSED_LOOP")
FAULT = STATES.index("FAULT")


def startup_run(vbus=24.3, pot=40):
    sim = Sim(plant=Plant2810(vbus=vbus), pot=pot)
    sim.run(2000)
    assert sim.state_name() == "IDLE", f"boot state {sim.state_name()}"
    sim.cmd_start()
    ok = sim.run(45000 * 5, until_state=CL, fail_state=FAULT)
    assert ok, f"never reached CL: {sim.state_name()} fault={sim.fault_name()}"
    return sim


def test_state_walk_and_hold():
    sim = startup_run()
    names = [s for _, s in sim.trace_named()]
    assert names == ["IDLE", "ARMED", "ALIGN", "OL_RAMP", "MORPH",
                     "CLOSED_LOOP"], names
    # CL entry seed must be the TRUE rotor period (the ×12/5 fix, in sim):
    s = sim.snapshot()
    ratio = s["fw_erpm"] / max(s["plant_erpm"], 1.0)
    assert 0.7 < ratio < 1.4, f"CL entry seed vs rotor: {ratio:.2f}"
    # hold CL ≥1 simulated second without faulting
    for _ in range(10):
        sim.run(4500)
        assert sim.state_name() == "CLOSED_LOOP", (
            f"left CL: {sim.state_name()} fault={sim.fault_name()}")
    print("PASS state_walk_and_hold")


def test_deterministic():
    t1 = startup_run().trace
    t2 = startup_run().trace
    assert t1 == t2, "two identical runs diverged"
    print("PASS deterministic")


def test_equilibrium_ladder():
    """Bench truth 2026-06-11: 24.3V→10.4k, 14.2V→5.75k, 13.1V→5.3k.
    Calibrated 2026-06-11: v_drop/b_visc from the 3-point equilibrium fit,
    J=2e-5 (passes morph; coast-down realism tracked in NOTES.md).
    Twin matches the ladder within ~1.5%."""
    import statistics
    for vbus, target in [(24.3, 10400), (14.2, 5750), (13.1, 5300)]:
        sim = startup_run(vbus=vbus)
        sim.run(45000)
        vals = []
        for _ in range(10):
            sim.run(450)
            vals.append(sim.snapshot()["plant_erpm"])
        mean = statistics.mean(vals)
        err = mean / target - 1
        print(f"  V={vbus}: twin {mean:.0f} bench {target} ({err:+.1%})")
        assert abs(err) < 0.05, f"equilibrium off >5% at {vbus}V"
    print("PASS equilibrium_ladder (loose tolerance)")


if __name__ == "__main__":
    test_state_walk_and_hold()
    test_deterministic()
    test_equilibrium_ladder()
    print("M1 ACCEPTANCE: PASS")
