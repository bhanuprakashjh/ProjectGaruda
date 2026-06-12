# Migration Plan: dsPIC33AK128MC106 → dsPIC33AK512MC510 MC DIM (EV67N21A)

2026-06-12. Target: run the full Garuda 6-step firmware (AM32/PLL startups,
HWZC sector PI, GSP, protections, arm melody) on the **dsPIC33AK512MC510
Motor Control DIM (EV67N21A)** in the same MCLV-48V-300W board.

## Reference documents (fetched)

| Doc | Where |
|---|---|
| DIM info sheet **DS70005553** (pin map, op-amp jumpers) | `docs/datasheets/dsPIC33AK512MC510-MC-DIM-InfoSheet-DS70005553.pdf` |
| Family datasheet **DS70005591** (dsPIC33AK512MPS512/MC510, ~2100 pp) | `docs/datasheets/` (local only, 47 MB — not committed; [download](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU16/ProductDocuments/DataSheets/dsPIC33AK512MPS512-Family-Data-Sheet-DS70005591.pdf)) |
| Microchip AN957 trapezoidal example **for this exact DIM+MCLV** | cloned at `reference-an957-ak512mc510/` — its `project/hal/` is the authoritative pin/peripheral oracle |

## Why this is a tractable port

1. **Same core, same toolchain**: dsPIC33A ISA on both; XC-DSC compiles both
   (new DFP + device selection only).
2. **Same DIM standard**: EV67N21A is drop-in for the MCLV socket — the
   *board-side* signals (shunts, BEMF dividers, 1.65 V VREF, pot, Vbus
   divider) arrive at the **same DIM pin positions** as today. OA1OUT is
   still DIM:017, OA2OUT DIM:025, OA3OUT DIM:033, VREF DIM:037.
3. **Same ADC architecture style**: `ADxCHyCON1bits.PINSEL` channels, same
   register idiom — a *remap*, not a rewrite. MC510 has **more ADC modules
   (AD1/AD2/AD3...)**, which lets every signal own a dedicated channel.
4. **The Garuda HAL is the isolation boundary** — and the SIL twin proves
   it: the entire control core compiles and runs against `mock/xc.h` with
   zero source edits. Everything above `hal/` is untouched by this port.

## Confirmed signal map (from the AN957 example — adopt as-is)

| Signal | MC510 channel | Device pin | Today on AK128MC106 |
|---|---|---|---|
| IA (OA1OUT) | AD1CH0, PINSEL 0 (AD1AN0) | RA2 | OA1 ch |
| IB (OA2OUT) | AD2CH0, PINSEL 0 (AD2AN0) | RB0 | OA2 ch |
| IC (board amp) | AD3CH0, PINSEL 3 (AD3AN3) | RB13 | n/a |
| IBUS (OA3OUT) | **AD3CH1, PINSEL 0 (AD3AN0)** | RA5 | OA3 → bus OC chain |
| VBUS | AD3CH2, PINSEL 4 (AD3AN4) | RF0 | vbus ch |
| POT | AD2CH1, PINSEL 5 (AD2AN5) | RB15 | pot ch |
| **VA (BEMF)** | AD1CH1, PINSEL 3 (AD1AN3) | RA9 | AD2CH1 PINSEL 10 |
| **VB (BEMF)** | AD1CH2, PINSEL 4 (AD1AN4) | RA10 | AD1CH5 (fixed) |
| **VC (BEMF)** | AD2CH2, PINSEL 4 (AD2AN4) | RB8 | AD2CH1 PINSEL 7 |
| PWM 3 phases | PG1/PG2/PG3 (PWM1..3 H/L on RD2/3, RD0/1, RC3/4) | — | PG1..3 |

**Design upgrade unlocked**: today the 106 shares AD2CH1 between phases A/C
via runtime PINSEL swaps (a whole class of HWZC complexity + the "runtime
PINSEL never took effect" bug family). On the MC510, **each BEMF phase has
its own channel** → `HWZC_OnCommutation` selects a channel/comparator
instead of rewriting PINSEL. Simpler and faster in the ISR.

## The one dangerous delta: clocks

AN957 runs the MC510 at **FCY = 200 MHz** (8 MHz DIM MEMS osc → PLL 800 →
200). Our tree assumes **100 MHz** (`SCCP_CLOCK_HZ=100000000`,
`LOOPTIME_TCY`, `HWZC_ERPM_TO_TICKS = 1e9/x`, blanking/advance math, the
PLL-startup accel term). *This is exactly the class of bug as the ×12/5
conversion saga* — a stale clock constant silently scales every period.

Mitigation (Binding Rule discipline):
- All clock-derived constants live in `garuda_calc_params.h` behind
  `SCCP_CLOCK_HZ` / `PWMFREQUENCY_HZ` / `LOOPTIME_TCY` — re-derive **once,
  there**, and grep for any literal `100000000`/`1000000000UL` outside it.
- Decision to make at P1: run MC510 at 200 MHz (full perf; constants
  re-derived; HWZC tick = 5 ns → better resolution) **or** clock it at
  100 MHz initially (constants byte-identical; lowest-risk first spin).
  Recommendation: **bring up at 100 MHz, switch to 200 MHz as its own
  one-variable step** after the startup suite passes.

## Work plan

### P0 — project scaffolding (½ day)
- Install `dsPIC33AK-MC_DFP` version covering AK512MC510; XC-DSC current.
- New MPLAB configuration (or sibling project) with device
  dsPIC33AK512MC510; keep the 106 config buildable (`GARUDA_TARGET` define
  selects HAL pin tables; everything above HAL has no target ifdefs).
- Port config bits/fuses from `reference-an957-ak512mc510/project/hal/
  device_config.c` (note: 512K device has dual-partition options — keep
  single partition).

### P1 — HAL port, file by file (1-2 days, example as oracle)
| Our file | Change |
|---|---|
| `hal/clock` (in board_service) | adopt AN957 `clock.c` (decide 100 vs 200 MHz per above) |
| `hal/port_config` | pin tables from AN957 `port_config.c` (PPS numbers all differ) |
| `hal/hal_adc.c` | channel map per table above; triggers; **ADC digital comparators**: HWZC uses ADxCMPy — re-instance (VB comparator was AD1 ch5-based; now VA/VB/VC are AD1CH1/AD1CH2/AD2CH2 → comparator per module; verify each ADx has ≥1 digital comparator in DS70005591 ADC chapter) |
| `hal/hal_pwm.c` | PG1-3 mostly identical registers; PPS/IO remap; deadtime re-check at new PWM clock |
| `hal/hal_comparator.c` (CMP3-DAC OC) | OA3OUT = CMP3A input on MC510 (same concept); DAC register names per datasheet |
| SCCP1/2/3 (timers) | same peripheral; re-derive tick constants if 200 MHz |
| UART1 (GSP) | same; new PPS codes; baud divisor per FCY |
| op-amps OA1-3 | same 3-op-amp internal config; **keep OMONEN=1×3 rule until proven otherwise on this chip**; DIM jumpers R16/R6, R35/R26, R13/R8 select internal amps (default = internal, matches us) |
| ISR vector names | `__AD1CH5Interrupt`-class names change with channels — sweep all `__*Interrupt` against the new device header |
| EEPROM emulation | flash row/page geometry differs on 512K device — check `eeprom/` driver constants + the defaults-signature offset |

### P2 — twin & build verification (½ day, no hardware)
- The SIL twin compiles the control core against mocks → prove the core is
  untouched: twin M1 acceptance + startup experiments must pass unchanged.
- Add a CI-style check: build both MPLAB configs (106 + 510) clean.

### P3 — bench bring-up (1-2 days, incremental, GUI at every step)
1. LED heartbeat + UART GSP `get_info` (new buildHash, profile reports)
2. ADC sanity at rest: pot, Vbus, **rest bias ~2048 on IA/IB/IBUS** (we
   know exactly what broken looks like after the DIM-connector saga)
3. Arm melody (audible = PWM + override path works without spinning)
4. ALIGN hold (2810 motor, 14 V, current-limited supply)
5. AM32 startup at 14 V → 24 V; then PLL variant
6. Protections: OC autozero latch, UV/OV thresholds vs new ADC scaling
7. Full sweep to ~233k; compare session CSVs against the AK128 golden
   sessions with `replay.py` — the twin's replay scoring works as a
   cross-device regression harness
8. If 100 MHz bring-up chosen: switch to 200 MHz as a single change and
   re-run 4-7.

### P4 — follow-ons enabled by this device
- Dedicated BEMF channel per phase → remove the PINSEL-swap machinery from
  HWZC (simpler ISR, possibly better >90k falling-ZC ceiling)
- 512 KB flash → room for the FOC stack + 6-step in one image
- 200 MHz → 2× HWZC timestamp resolution (5 ns ticks)

## Risk register

| Risk | Mitigation |
|---|---|
| Clock-constant drift (the ×12/5 class) | single derivation point + literal-grep audit; 100 MHz first |
| ADC digital-comparator instance differences (HWZC core) | verify in DS70005591 ADC chapter before P1; fallback = SW-compare mode (`HWZC_USE_SW_COMPARE` already exists) |
| PPS code differences | all PPS via AN957 example values |
| OA behavior differences (OMONEN erratum-class quirks) | keep OMONEN=1 rule; rest-bias check is step P3.2 |
| EEPROM flash geometry | verify NVM row/page sizes; defaults-signature guards stale layouts |
| Dual-partition config surprises | explicitly program single-partition |

## Effort estimate

~3-5 working days to "AM32 startup runs on MC510 at 24 V", of which ~2 are
bench. The twin + the AN957 oracle + the clean HAL are what keep it short.
