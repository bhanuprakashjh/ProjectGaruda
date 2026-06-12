# garuda_sil — implementation notes / status

## M1 STATUS (2026-06-11): ACCEPTANCE PASSING
- The REAL firmware (garuda_service.c + motor/* + gsp_params.c, byte-identical
  sources, ZERO firmware edits) compiles to libgaruda_sil.so and walks
  IDLE→ARMED→ALIGN→OL_RAMP→MORPH→CLOSED_LOOP against the simulated 2810.
- Deterministic: identical runs produce identical state traces.
- Equilibrium ladder calibrated to the 2026-06-11 bench measurements
  (24.3V→10.4k, 14.2V→5.75k, 13.1V→5.3k): twin within +0.5/+1.5/+0.6%.
- Run: `tools/garuda_debug/.venv/bin/python sil/test_m1.py`
- Build: `bash build.sh` (gcc -m64, -Wno-attributes; the interrupt/no_auto_psv
  attributes warn and are ignored on x86).

## Plant calibration state
- v_drop=0.0847 V, b_visc=1.1573e-4 N·m·s/rad: least-squares fit of the
  steady-state equilibrium model to the 3-point bench ladder (±0.5% analytic).
- J=2.0e-5 kg·m²: NOT yet bench-fit. Constraints discovered: J=2.5e-6 (toy)
  → rotor bleeds 3k→1.8k during the morph coast (entry-seed ratio 1.72);
  J=8e-5 → morph lock-gate times out (window/phase-lag interaction worth
  studying — it may mirror a real heavy-rotor failure mode!). J=2e-5 passes
  everything. PROPER FIT: extract a pot-zero coast-down transient from a
  recorded session CSV and fit J/b to it (P3 of the roadmap).
- Known open item: during the un-settled transient the fw_erpm getter and
  plant_erpm were seen ~1.22 apart at one sample point; at settled lock they
  agree. Verify the sil_erpm() getter unit vs hwzc stepPeriodHR at some point.

## Architecture (agent-built, supervised)
- mock/xc.h: fake SFRs (bitfield structs exactly as referenced); -I mock
  shadows <xc.h> for the real hal/*.h headers.
- sil/virtual_hw.{h,c}: SilHw register file — the firmware↔plant bridge.
- sil/hal_sil.c: HAL function surface reimplemented against SilHw.
- sil/stubs.c: EEPROM_*/GSP_/Scope_/RX_ no-ops (logged here as the stub list).
- sil/sil_api.{h,c}: flat cffi API — ISR entry points (sil_adc_isr,
  sil_timer1_isr, sil_cct1_isr, sil_cmp_isr), plant I/O setters, garudaData
  getters, command intents (start/stop/fault-clear replicate main.c).
- sil/garuda_sil.py: cffi harness; logical time (tick = 1/45kHz; Timer1 from
  its own 100µs accumulator; prio-7 events fired inside sub-steps before the
  ADC ISR — ISR priority by call order). Plant2810: quasi-static R-only
  electrical + trapezoid BEMF + RC-filtered terminal synthesis per zcsim.py
  conventions (PWM-ON Vn+1.5e, OFF freewheel diode-clamped, ~52 counts/V).

## Replay-and-score (sil/replay.py) — first results, golden session
gui_auto_20260611_090540 (truth-entry startup + 232k sweep, 31 s) replayed
at ~1.5x real time. MEASURED FIDELITY ENVELOPE:
- state edges ALIGN/OL_RAMP/MORPH/CL: within ±40 ms of bench over 31 s ✓
- idle equilibria: ±1.5% (see ladder) ✓
- CL-entry eRPM: twin +71% (4820 vs 2810) — plant accelerates too easily
  through OL/morph (J underfit at 2e-5; torque model has no L, no load
  angle softness). Coast-down fit will pin J.
- high-speed: twin desyncs ~61k eRPM (bench 234k) then OC — the M1
  quasi-static plant + linear b_visc (fit at ~10k) does not extrapolate;
  sense-chain synthesis fidelity at sub-30 µs sectors untested (n_sub=4
  → 5.5 µs comparator granularity). P2 work, expected.
USABLE DESIGN-STUDY ENVELOPE TODAY: 0–~15k eRPM — exactly the startup
regime, which is where every open design question lives (PLL-from-align,
hand-off studies, VEX-class profiles).

## Experiment session #1 (2026-06-11 evening) — skip-morph & windmill
Variant builds: build_variant.sh (temp flag edit + git restore; SIL_OUT names
the .so). libgaruda_sil_skipmorph.so = FEATURE_SKIP_MORPH+CL_COAST_VERIFY.
- Rest starts (12 rotor angles): both variants 12/12. skipmorph −140 ms
  time-to-CL (1.95 vs 2.09 s), ipk 33.4 vs 35.1 A. Angle sweep is a null
  experiment — ALIGN erases θ0 perfectly in this plant (no cogging/striction).
- WINDMILL starts (0–15 k initial spin, coast-faithful plant): NEITHER
  variant catches. Spin survives the ARM coast (15k→10.7k, TC 1.5 s ✓) and
  then ALIGN BRAKES IT TO ZERO in <0.5 s (~44 A peaks, baseline). Both then
  start from rest identically. ⇒ the 1c windmill-catch feature (pre-align
  coast-listen → engage CL directly when spinning) is the missing piece;
  the twin now quantifies the cost of not having it.
- Drag model v2 discovered a twin ANGLE BIAS: with light drag, equilibrium
  runs ~50% high ⇒ effective commutation ~25° early (BEMF factor ~0.66) in
  the twin's sense-chain synthesis (bench equilibrium implies aligned
  commutation). P2 investigation item — was masked by heavy drag in v1.
- Plant drag configs: v1 (defaults) equilibrium-faithful, coast wrong
  (TC 0.17 s); v2 (j=5e-6, b=3.3e-6, v_drop=0.0822+0.0053·Vbus)
  coast-faithful (TC 1.5 s) + ladder-exact analytically, but exposes the
  angle bias. Unify after the angle bias is fixed.

## PLL-from-align prototype (task #10) — 3 twin iterations, IN PROGRESS
FEATURE_PLL_STARTUP=0 (twin-only WIP). State trace achieved:
IDLE→ARMED→ALIGN→CLOSED_LOOP — no OL ramp, no morph. The twin caught
three real design bugs in three ~60s iterations (would have been three
bench days):
1. PHANTOM SYNC LEAK: garuda_service.c:~3552 promotes zcSynced from raw
   hwzc.goodZcCount — low-speed phantom captures synced it at t=1.151s
   every time, bypassing the PllStartTick gates. FIXED (gated on
   !pllStartActive).
2. ZERO-TORQUE ENGAGE: first commutation at the align sector = field ON
   the rotor → no pull. FIXED (engage at s0+1, 60° lead) — rotor now
   tracks the blind schedule within ~20%.
3. RESOLVED (iteration #4): blind schedule 10× slow was a UNIT BUG, not
   timing ownership — accel term divided sector time by 1e9 but HR ticks
   are 10 ns (SCCP_CLOCK_HZ=1e8). Twin 673 eRPM at t=2s; formula gives 672.
4. RESOLVED (iteration #5): rotor then tracked within 1% but never synced —
   the absolute window (0.30T..0.75T) is wrong-headed: the locked rotor's
   ZC sat pinned at 0.14T (twin ~25° angle bias + blanking clip; on real HW
   load angle/advance/RC lag shift it too). Replaced with a CONSISTENCY
   gate: plausible (T/8..7T/8) AND |cap−prevCap| < T/4 for 6 consecutive
   sectors. Lock is consistent, phantoms are random.
RESULT (commit 6e46121): 12/12 rotor angles. Sync t=1.59s (~2.8k eRPM,
just above the 2.5k floor), PI pulls cap 0.25T→0.49T post-handover, idle
10456 at fw/rotor=1.00. experiment_pll.py is the study harness; sil_api
gained cap-position/pll debug getters.
Gates: floor 2500, caps 6, consistency T/4, accel 4000, target 10000.
BENCH HEX BUILT 2026-06-11 23:41 (FEATURE_PLL_STARTUP=1, flag restored to
0 in repo): /tmp/garuda_pll_startup_234209.hex — awaiting hardware trial.

## Next (roadmap P2/P3)
1. Replay-and-score: feed a recorded session's throttle/Vbus through the twin,
   score state durations, CL-entry seed, idle eRPM, event stats.
2. Coast-down J/b fit from session CSVs.
3. DONE 2026-06-12: sil/sim_broker.py serves the twin over the broker's
   TCP protocol (port 47800) — garuda-gui/MCP connect UNCHANGED and see
   the simulator as a board (live plots, sessions, start/stop, throttle
   via set_throttle RPC; sim-only truth channels plant_erpm/theta in
   every snapshot). --motor vex|2810, --so <variant>, --vbus, --speed.
4. First design study: PLL-from-align startup (no-morph, capture-weighted
   transition) — prototype in twin before any bench time.
