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

## Next (roadmap P2/P3)
1. Replay-and-score: feed a recorded session's throttle/Vbus through the twin,
   score state durations, CL-entry seed, idle eRPM, event stats.
2. Coast-down J/b fit from session CSVs.
3. GSP snapshot bytes out + broker attach (GUI shows the twin like a board).
4. First design study: PLL-from-align startup (no-morph, capture-weighted
   transition) — prototype in twin before any bench time.
