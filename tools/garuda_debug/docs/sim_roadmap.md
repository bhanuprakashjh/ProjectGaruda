# Garuda Digital Twin — state-of-the-art simulation roadmap

*Research synthesis 2026-06-11 (3 parallel deep-research passes: local inventory,
SOTA simulation methods, digital-twin/SIL practice). Full citations inline.*

## Verdict

No off-the-shelf tool — open or commercial — simulates the sense chain of a
6-step sensorless ESC (floating-phase BEMF + comparator + RC filter + blanking
+ valley-sampled shunt). motulator/gym-electric-motor stop at FOC fidelity;
SPICE is ~1000× too slow closed-loop; HIL rigs are 5-figure and still don't
model the firmware's detection logic. AM32/Bluejay have no simulators; VESC's
in-firmware virtual motor (plant injected at the ADC seam) is the lone
precedent and the right seam. **A bench-calibrated 6-step sense-chain twin is
unoccupied territory.** Garuda's unique assets: hardware-abstracted C firmware
(the HAL seam) + hundreds of auto-recorded session bundles as ground truth +
per-sector event telemetry (hwzcMissBySector) that exceeds what published work
uses for validation.

## Current state (inventory, 2026-06-11)

- GUI (app.py, PySide6): broker-shared live telemetry @30 Hz, bench-scope plot,
  24 kHz burst scope, param tuner w/ OC safety check, wizard, 2-tier diagnosis,
  auto-recorded session bundles. No session replay into GUI. No sim endpoints
  on broker/MCP.
- Sims are four islands: zcsim (capture statistics bench-faithful; zero
  electrical dynamics — R/L unused), pisim (hand-ported PI loop copy, CLI-only,
  drifts from C silently), spice_engine (honest: does not reproduce bench),
  zcml (estimator, not physics). Four disjoint motor-parameter representations.
  All calibration constants hand-fit literals; no fitting infrastructure.
- Deep flaw: firmware math reimplemented in Python 3×, never reused. The ×12/5
  conversion bug was invisible to pisim because it copied intent, not code.

## Architecture: three pillars

### P1 — garuda_sil: the real firmware C compiled for x86 (days; highest leverage)
- gcc-compile portable modules (garuda_service state machines, bemf_zc.c,
  hwzc.c, startup.c, commutation.c, OC logic, gsp_snapshot encoder) into a .so
  behind a mocked HAL: one C struct of virtual registers (PDC/IOCON-equivalents,
  ADC channel values, comparator state + DAC, SCCP timers, tick counters).
- Drive from Python (cffi) at logical 45 kHz ticks — deterministic logical
  time, not wall clock (Betaflight SITL / Zephyr native_sim pattern:
  betaflight.com/docs/development/SITL, docs.zephyrproject.org native_sim).
- Emit real GSP snapshot bytes → existing broker/GUI/MCP attach to the SIL
  binary unchanged (TCP socket instead of serial). Same tooling, two backends.
- Watch: int sizes (prefer -m32 or explicit stdint), XC intrinsics behind
  port.h, ISR-priority semantics emulated by call order (prio7 SCCP/comparator
  events before the 45 kHz tick body, Timer1 every 4.5 ticks... = 100 µs).

### P2 — plant: ideal-switch abc model + sense chain as first-class (1-2 kLOC)
- Electrical: per-phase tri-state conduction (high / low / floating with
  body-diode clamps to the rails ±Vf), explicit deadtime intervals, coupled
  3×3 L matrix (mutual coupling = the PWM-ON floating-phase bias we measured),
  Rs, tabulated e(θ) BEMF waveshape (FEMM/pyleecan for shape — ~15% absolute
  accuracy is fine, amplitude/Ke fit from bench; femm.info/wiki/rotormotion).
- Mechanical: J, viscous + Coulomb friction (fit from coast-down CSVs).
- Sense chain (the part no tool has): BEMF divider + 5.5 kHz RC per phase
  (first-order ODE), ADC sampled at firmware-commanded instants (ON-center /
  OFF-center / valley), comparator with hysteresis + DAC threshold + latency
  as passive events, bus shunt sampled at PWM valley (reproduces ibus
  blindness), deadtime voltage distortion emerges from conduction logic.
- Numerics: motulator's CarrierComparison idea — compute PWM switching
  instants ANALYTICALLY per period; piecewise-linear ODE between events
  (matrix-exponential or RK4 steps); bisect only on diode-current /
  comparator-voltage sign changes (event-driven hybrid sim ≈10× faster than
  fixed-step; refs in research notes). numpy first (~0.01-0.1× real time =
  fine), Numba/JAX port later for vmap'd 1000-run sweeps + differentiability.
- Acceptance tests = this month's bench phenomena, each measured & root-caused:
  falling-ZC masking curve vs eRPM, OL→CL 22 A slam profile, diode-ring coast
  artifact (the 2× crossing), deadtime volt loss (~0.65 V diff / ~0.32 V conv),
  valley-blind Ibus, RC filter phase lag at 200k.

### P3 — replay-and-score + calibration + CI (turns the corpus into a net)
- Replay recorded throttle/Vbus traces through SIL+plant; score continuous
  RMSE (eRPM, Ibus_win, duty) AND event statistics: per-sector miss rates,
  reject %, desync occurrences/timing, handoff peak current, startup success.
  Event-statistics validation exceeds published practice (which stops at
  waveform THD) — our telemetry was built for this.
- Calibration: seed Rs/Ls from foc_v2_detect bench values (Ls=14.4 µH 2810);
  scipy.optimize.least_squares on spin-up/coast-down transients for Ke/J/B;
  CMA-ES (pycma) over non-smooth sense-chain params (RC τ, hysteresis,
  deadtime volts, ADC bias) minimizing replay error — the standard sim-to-real
  recipe. Cross-check vs Microchip motorBench self-commissioning.
- CI: pytest suite — each test = recorded golden session (or scripted
  scenario) → SIL run → assertions on event metrics. Every firmware commit
  regression-checked BEFORE flashing (digital-twin-prototype CI:
  arxiv.org/abs/2401.07985; ISO 26262 back-to-back MIL/SIL idea).
- Fault injection at the mock HAL (OPAL-RT's standard scenarios): railed ADC,
  offset drift (slow ramp + step + frozen), Vbus sag, missed ticks, open
  phase. The 2026-06-11 OA3 rail incident and the 22A-late OC bias bug are
  exactly these scenarios — both become permanent twin test cases; future
  protection logic gets designed in the twin first.

## Unification chores (cheap, do alongside P1)
- One MotorModel parameter object shared by zcsim/pisim/wizard/plant (replaces
  4 disjoint representations); profiles loadable from session params.json.
- Session replay into the GUI plots (bench-scope from CSV) and broker sim
  endpoints so the MCP agent can drive the twin like the board.
- Retire pisim's hand-ported PI once SIL runs the real hwzc.c; keep zcsim's
  capture_model as a fast surrogate; keep spice_engine as a sandbox for
  characterizing the sense divider ringing only (fit its result INTO P2's
  parasitic params).

## Skip list (researched, rejected)
QEMU/Renode/MPLAB-SIM (no usable dsPIC33AK emulation exists), whole-loop
ngspice (sub-model characterization only), pure PINNs (residual learning on
twin error later, maybe), OpenModelica, commercial HIL rigs, full saturation
flux maps (surface-PM low-saliency — constant fitted L sufficient for ZC
fidelity).

## Milestones
M1 (week 1-2): garuda_sil .so + minimal plant (R/L/Ke trap BEMF, ideal
  switches, RC+ADC sense) → twin boots, arms, runs the full startup state
  machine, GUI attaches → demo: replay a recorded startup, reproduce the
  OL→CL current spike within ~20%.
M2: calibration pipeline + 5 golden-session pytest CI; falling-masking and
  valley-blind acceptance tests pass.
M3: fault-injection layer + protection test suite; deadtime/diode fidelity
  (reproduce the diff-mode 0.65 V and coast diode-ring).
M4: Numba/JAX core for sweeps; startup-trajectory optimization in the twin
  (the OL→CL problem, finally with unlimited flash-free iterations); residual
  learner on twin-vs-bench error for >130k regime.
