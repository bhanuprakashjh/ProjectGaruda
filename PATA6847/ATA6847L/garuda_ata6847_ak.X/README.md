# Garuda ATA6847L 6-step ESC firmware (dsPIC33AK128MC106)

**Target board**: ATA6847L QFN48 BLDC Motor Driver Evaluation Board (EV92R69A) + dsPIC33AK128MC106 Motor Control DIM (EV68M17A)
**Source-of-truth firmware**: `../../garuda_6step_ck.X/` @ commit `24703f6` (228k eRPM milestone, frozen)
**Plan**: `../../docs/garuda_6step_ak_port_plan.md` (v6)

## Status

**Phase 0 scaffold landed** (2026-05-11) — ~15,000 lines of code in place,
project opens in MPLAB X.  Will not build clean without the bench-time HAL
rework called out below.  See plan §8 for the milestone list.

### What's in
- **MPLAB X project**: `nbproject/configurations.xml` + `project.xml` (target
  dsPIC33AK128MC106, XC-DSC v3.30, DFP v1.4.172, PKoB4 Hybrid).
- **Algorithm-side** (verbatim CK copy, no changes needed):
  `garuda_types.h`, `garuda_service.{c,h}`, `main.c`, `motor/*`, `gsp/*`,
  `tools/pot_capture.py`, `tools/gsp_ck_test.py`.
- **Config + bring-up HAL** (AK port complete, ready for bench):
  `garuda_config.h`, `hal/clock.{c,h}`, `hal/hal_timer1.{c,h}`,
  `hal/hal_trap.c`, `hal/hal_uart.{c,h}`, `hal/port_config.{c,h}`,
  `hal/hal_spi.{c,h}`, `hal/hal_ata6847.{c,h}`, `hal/delay.h`.
- **HAL — copied verbatim, AK rework flagged**:
  `hal/hal_pwm.{c,h}`, `hal/hal_adc.{c,h}`, `hal/hal_capture.{c,h}`,
  `hal/hal_diag.{c,h}`, `hal/board_service.{c,h}`. These have a [AK PORT]
  banner naming the items that need silicon validation/rewrite.

### Open items before first flash
1. **Hardware probe (10 min)**: confirm `nCS` is on DIM 6 vs DIM 8 →
   patch `port_config.h` and `port_config.c`.
2. **`hal_adc.c` rewrite** — dsPIC33AK ADC uses ADCCON1L/H + per-core
   power sequence; CK's `ADCON5L.C0PWR` SFR doesn't exist on AK.
   Reference: `../../dspic33AKESC/hal/hal_adc.c`.
3. **`hal_capture.c` re-map** — CK's `_CCP5Interrupt` doesn't exist
   (no SCCP5 on AK).  Re-target to SCCP3 once needed.  Currently
   gated off via `FEATURE_V4_CCP_DIAG=0` so the build links.
4. **`garuda_service.c` ISR vector audit** — CK uses `_CCP2Interrupt`,
   AK uses `_SCCP2Interrupt`. `_CCP5Interrupt` references must be
   removed or re-mapped.
5. **SPI1 / UART1 PPS function codes** in `port_config.c` are
   placeholder values from the AK FOC sibling — verify against
   DS70005527 PPS table before bench.
6. **PG1CONH master/slave bits** in `hal_pwm.c` need a scope check
   on bring-up — bit pattern may differ from CK.

## Directory layout

```
garuda_ata6847_ak.X/
├── nbproject/        MPLAB X project files (target dsPIC33AK128MC106, XC-DSC v3.30)
├── hal/              HAL — rewritten for AK silicon (see §3 of plan)
├── motor/            COPY from CK + ~120-line sector_pi.c cleanup per plan §2
├── gsp/              COPY from CK + extend (EEPROM persistence, ATA register R/W cmds)
├── scope/            COPY from CK
├── input/            COPY from CK (pot now; DShot later)
├── tools/            COPY pot_capture.py from CK
├── garuda_config.h   COPY from CK + retune clock constants
├── garuda_service.c  COPY + adapt — ISR vector names, _CCP2/5 ISRs gated behind FEATURE_V4_CCP_DIAG
├── garuda_types.h    COPY from CK
└── main.c            COPY + retarget HAL init order
```

## Scope (from plan v6 §1)

**In scope**: ADC-ISR ZC detection path, block-commutation, V4 sector PI, milestone parity with CK 228k eRPM on 2810 motor.

**Out of scope for milestone**:
- SP (single-pulse) mode — capture-owned, disabled via `SP_ENTER_ERPM = 999999` in `hal_pwm.h`
- DShot RX — Phase 7 follow-on
- CLC fallback — Phase 3.5 contingent if raw-BEMF noise problems emerge

## SPI pin map (resolved from EV92R69A user guide schematic A.3)

| Signal | Carrier DIM pin | AK pin | AK function |
|---|---|---|---|
| **nCS** (= GATE_DVR_ENABLE) | **6 or 8** (probe to confirm) | 41 or 42 | RC2/RP35 or RC5/RP38 — GPIO output |
| **nIRQ** | **40** | 32 | RB11 / RP28 — GPIO input + IOC; **do NOT enable SPI2 SDI** |
| **MOSI** | **42** | 35 | RC6 / RP39 — PPS-route to SPIx SDO |
| **MISO** | **44** | 36 | RC7 / RP40 — PPS-route to SPIx SDI |
| **SCK** | **46** | 33 | RC8 / RP41 — PPS-route to SPIx SCKO |

Carrier resistor config: populate **R20, R70, R94, R96** (DIM-side); ensure R69, R97, R98, R72 are depopulated (MIKRO/Xplained mirrors) if shipped populated.

SPI peripheral: use **SPI1 or SPI3** (avoid SPI2 — its native SDI is on the same pin as our nIRQ input).

## Open gates

1. **Phase 0 trap-handler + PMD-clear smoke test** — math trap must fire on injected division-by-zero
2. **Phase 0 multimeter probe** — confirm `GATE_DVR_ENABLE` is on DIM pin 6 vs 8 (10-minute task on bare carrier)

## Build

Pending MPLAB X project creation. CLI build via `Makefile-cli.mk` once scaffolded.
