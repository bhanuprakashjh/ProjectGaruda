# garuda_sil — the REAL firmware compiled for x86 behind a mock HAL

M1 of the digital-twin roadmap (`tools/garuda_debug/docs/sim_roadmap.md`).
Pattern: Betaflight-SITL / Zephyr-native_sim — logical time, deterministic,
the actual C control code as a host shared library driven from Python.

## What gets compiled (firmware sources, UNMODIFIED, from dspic33AKESC/)
- garuda_service.c        — both state machines + ISR bodies (the core)
- motor/bemf_zc.c, motor/commutation.c, motor/startup.c, motor/hwzc.c
- gsp/gsp_params.c        — gspParams/profiles/derived (runtime params)
- (later) gsp/gsp_snapshot.c for real snapshot bytes

NOT compiled: hal/*.c (reimplemented), main.c, gsp/gsp.c+commands (UART),
foc/* (flags off), learn/* (flags off), x2cscope.

## Mock layers (all under tools/garuda_sil/, firmware tree untouched)
1. `mock/xc.h` — fake SFRs as plain globals. Only what the compiled set
   references: AD1CH5CONbits/AD2CH1CONbits (TRG1SRC etc. fields used),
   AD1CMPSTATbits/AD2CMPSTATbits, _CCT1IE/_CCT1IF, _T1IE/_T1IF, IPC bits,
   LED1/LED2 targets, PGxSTAT/PGxIOCON words if referenced directly, etc.
   Grow it from compile errors. `-I mock` precedes the HAL dirs so real
   hal/*.h that `#include <xc.h>` resolve here.
2. `sil/hal_sil.c` — implements the HAL FUNCTION surface (whatever the
   link demands: HAL_PWM_SetDutyCycle, HAL_PWM_SetCommutationStep,
   HAL_MC1PWMDisableOutputs, HAL_ADC_SelectBEMFChannel,
   HAL_ADC_UpdateComparatorThreshold, HAL_CMP3_SetThreshold,
   HAL_SCCP1_StartOneShot/Stop, HAL_SCCP2_ReadTimestamp, board LEDs, …)
   against `sil/virtual_hw.h`'s `g_silHw` register file.
3. `sil/virtual_hw.h/.c` — the bridge struct Python reads/writes:
   - OUT (firmware→plant): pdc[3], phase mode per leg
     (PWM/OVR_LOW/HIZ), outputs_enabled, cmp thresholds+enables per core,
     cmp3 dac, sccp1 deadline+armed, bemf mux channel selection.
   - IN (plant→firmware): adc values (phaseB, phaseAC, ibus, vbus, pot),
     comparator trip events, sccp2 timestamp (logical 100 MHz clock).
4. `sil/sil_api.c` — exported flat API for cffi:
   `sil_init()`, `sil_reset()`, `sil_adc_isr()` (calls the firmware ADC ISR
   exactly once), `sil_timer1_isr()`, `sil_cct1_isr()`, `sil_cmp_isr(core)`,
   plus `SilHw* sil_hw(void)` and getters for garudaData fields of interest
   (state, faultCode, stepPeriod, currentStep, duty, zcSynced, eRPM calc).
   Also `sil_cmd_*` poking garudaData (runCommandActive, throttle source).

## Time model (logical, deterministic)
- Master tick = one PWM period = 1/45000 s. Python loop per tick:
  1. plant advances electrical+mechanical state across the tick
     (sub-stepped), computes ADC samples at the trigger point and
     comparator crossings within the tick
  2. fire prio-7 events first (comparator ISR if armed+crossed, CCT1 if
     sccp1 deadline passed) — matching ISR priorities by call order
  3. call sil_adc_isr()
  4. every 4.5 ticks call sil_timer1_isr() (Timer1 = 100 µs; alternate
     4/5 ticks to average 4.5 — or run timer1 from its own accumulator)
- sccp2 timestamp = tick_count * (100e6/45000) + intra-tick offset.

## Build (build.sh)
gcc -m32 if available else -m64 (document which), -fPIC -shared,
-DSIL_BUILD, -Wno-attributes (interrupt/no_auto_psv attrs), includes:
`-I mock -I <fw root>`. The firmware's int sizes: dsPIC33AK is 32-bit;
int=32 on both → x86-64 acceptable for M1 (watch long: fw long=32,
x86-64 long=64 — firmware uses stdint types mostly; flag any bare-long
arithmetic found).

## Python (sil/garuda_sil.py)
cffi ABI-mode loading the .so + a Plant2810 class (R/L/Ke trapezoid,
J/friction from bench: Ke from KV1350/7pp, Rs 0.05Ω pp, Ls 14.4 µH,
equilibrium line bench-fit: eRPM = 8.43k/V × (0.054×Vbus − 0.078)).
Plant v1 can be quasi-static (torque ∝ net volts at commutation angle)
— fidelity grows later; M1 acceptance is the FIRMWARE behaving.

## M1 acceptance test (sil/test_m1.py)
1. boot → IDLE; arm (runCommandActive=1, throttle src pot, pot≈40)
2. state walks IDLE→ARMED→ALIGN→OL_RAMP→MORPH→CL on the simulated 2810
3. CL entry stepPeriod ≈ true rotor period (the conversion fix visible)
4. reaches a stable CL idle ~10.4k eRPM at 24 V without faulting
5. determinism: two runs bit-identical state traces.

## Rules
- NEVER modify files under dspic33AKESC/ to make the sil build work
  (that's the whole point — same bytes the bench runs). If genuinely
  impossible, record the blocker in BLOCKERS.md instead.
- Every mock register the firmware writes must be visible in SilHw —
  no silently swallowed writes for things the plant needs.
- Keep a NOTES.md of every assumption made.
