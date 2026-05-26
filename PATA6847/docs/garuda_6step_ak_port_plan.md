# Garuda 6-step — Port `garuda_6step_ck.X` to dsPIC33AK128MC106 + ATA6847L

**Status:** plan v6 (post fourth Codex review — closes the SP-mode capture dependency, splits the CK A/B gate into two stages, sweeps remaining stale "verbatim" statements), 2026-05-11. See §11 for full revisions log.

**Goal:** re-host the working CK 6-step firmware (`garuda_6step_ck.X/` @ commit 24703f6, 228k eRPM milestone) on the **dsPIC33AK128MC106 DIM** plugged into the **ATA6847L QFN48 carrier**. The actual milestone code does ZC detection inside the ADC ISR via GPIO-read of the BEMF comparator pins + an HR free-running timer for timestamping — **not** via SCCP input-capture. The port carries the algorithm with a bounded ~250-line cleanup in `motor/sector_pi.c` to remove hardware-capture SFR references (which won't link on AK because SCCP5 doesn't exist), and re-implements the HAL.

**Scope boundary for the milestone**: AK port targets the **228k eRPM ADC-ISR ZC path only**. Block-commutation is in scope (engages cleanly under ADC-ZC). **Single-pulse (SP) mode is out of scope** — SP path is owned by CCP IC capture (see §1) and is never triggered in the validated milestone build anyway (SP_ENTER_ERPM = 250000, above any tested motor). For the AK milestone we disable SP entry by setting `SP_ENTER_ERPM = 999999` in `hal_pwm.h`. SP mode is deferred to a follow-on task if peak eRPM ever needs to exceed 250k.

Every tuning knob currently in `garuda_config.h` gets promoted to a runtime GSP parameter so the GUI alone can configure the ESC.

---

## 1. The architecture being ported (verified by code, not comments)

In the milestone build (`FEATURE_V4_MIDPOINT_ZC = 1`, `v4_spActive = false`), ZC detection happens entirely in **`_ADCInterrupt`**, gated by `SectorPI_IsRunning()` and `SectorPI_GetPhase() == 3` **and `!v4_spActive`** (line 395):

```c
/* garuda_service.c:404–481 */
uint16_t nowHR = CCP4TMRL;                          /* HR timer read */
if ((int16_t)(nowHR - v4_blankingEndHR) < 0) {
    v4_adcBlankReject++;                            /* still inside blanking */
} else {
    uint8_t r1 = ReadBEMFComp();                    /* GPIO read */
    Nop(); Nop(); Nop(); Nop();
    uint8_t r2 = ReadBEMFComp();
    Nop(); Nop(); Nop(); Nop();
    uint8_t r3 = ReadBEMFComp();
    uint8_t comp = ((uint8_t)(r1 + r2 + r3) >= 2u) ? 1u : 0u;
    /* compare against expected post-ZC state ... */
    if (comp == expectedPost) {
        v4_lastCaptureHR = nowHR;
        v4_captureValid  = true;
    }
}
```

The motor algorithm (`motor/sector_pi.c`) reads `v4_captureValid` and `v4_lastCaptureHR` as plain volatile variables. Both are written by this ADC ISR path.

**SCCP2 and SCCP5 input capture exist in code but are inert during the milestone path** (i.e. with `v4_spActive=false`). All three branches of `_CCP2Interrupt` are guarded by `FEATURE_V5_SCHEDULER`, `v4_spActive`, or `FEATURE_V4_MIDPOINT_ZC == 0`. With the AK milestone configured as in the §1 scope boundary (V5 off, SP disabled via `SP_ENTER_ERPM=999999`, midpoint ZC on), none of those branches activate. The ISRs drain FIFO and clear flags only.

**Important caveat: SP mode is capture-owned.** When `v4_spActive=true` (CK enables this above SP_ENTER_ERPM = 250k eRPM):
- ADC ISR's ZC detection is **bypassed** — see `garuda_service.c:395` (`if (!v4_spActive && ...)`).
- `_CCP2Interrupt` line 648 owns SP-mode ZC via the hardware capture buffer.
- Sector PI documents the source switch at `sector_pi.c:708` and uses `actualStepPeriodHR` (forced-comm period) instead of `timerPeriod` in scheduling at `:938, 968`.

So **CCP capture is load-bearing in SP mode**, which is why the AK milestone explicitly disables SP (§1 scope). For the AK port:
- AK milestone: motor never crosses 250k eRPM in normal operation. Set `SP_ENTER_ERPM = 999999` in AK `hal_pwm.h` to prevent any SP entry path.
- Future SP port (deferred): re-implement SP-mode ZC. The simplest path is to keep `FEATURE_V4_CCP_DIAG=1` for SP builds and have SP mode read the diagnostic capture path; this adds ~50 lines of SP-specific glue but doesn't change the motor-spin architecture.

With SP disabled and V5 off, everything Codex worried about in v3 — CCPON-while-changing-MOD, IC IRQ vs ICDIS, PPS-swap hazard windows, cross-domain timestamp offsets, DMA FIFO setup, single-IC mode-flip feasibility — is **moot for the AK milestone path**. The port doesn't need any of it for the 228k goal.

We keep the CCP IC infrastructure in the AK codebase as **diagnostic/debug-only** (build-flag gated, off by default) so the engineer on a bench can enable hardware ZC capture for measurement without re-wiring the firmware. None of it is on the hot path.

---

## 2. What the port has to do, four layers

| Layer | Files | Action |
|---|---|---|
| **Algorithm — clean** (no silicon dependency) | `motor/bemf_zc.c`, `motor/commutation.c`, `motor/startup.c`, `motor/v4_params.{c,h}`, `garuda_types.h` | **Copy verbatim.** Only `LOOPTIME_TCY` / HR-tick constants recompute against AK clock. |
| **Algorithm — needs cleanup** | `motor/sector_pi.c` | **Copy + adapt (~100–150 lines).** Sector PI directly references `CCP5STATL`, `CCP5BUFL`, `_CCP5IF`, `_CCP5IE` at 11 lines (audited: `:280, 282, 542, 561, 563, 588, 616, 715, 717, 748, 750, 834`) plus 6 `HAL_Capture_*` calls. AK has no SCCP5 — build won't link. Cleanup approach: hoist every SFR access into a new `hal_zc_state` API; sector_pi keeps its logic flow but calls HAL functions for what it currently does inline. CCP IRQ enable/disable lines become no-ops on AK motor builds, real ops on diag builds. (See §3 file split.) |
| **ISR shells + state machine** | `garuda_service.c`, `main.c`, `garuda_config.h` | **Copy and adapt (~50 lines).** Keep `_T1Interrupt`, `_ADCInterrupt` (the load-bearing one), `_CCT3Interrupt` (commutation scheduler). `_CCP2Interrupt` / `_CCP5Interrupt` get build-flag-gated as diagnostic ISRs (`FEATURE_V4_CCP_DIAG`, default 0). The ADC ISR still calls `HAL_Capture_IsRisingZc()` at line 424 — that function moves out of `hal_capture.c` into `hal_zc_state.c` so it's always available regardless of the diag flag. |
| **GSP + scope + EEPROM** | `gsp/*`, `scope/*`, EEPROM | **Copy + extend** with EEPROM persistence (CK is RAM-only; AK gains it), runtime promotion of `#define`s (§7), and ATA register R/W commands. Wire format unchanged. |
| **HAL** | `hal/*` | **Rewrite for AK** module by module — same public API where it exists, new files where the AK silicon differs from CK. |

### Net algorithm-side delta estimate (revised post Codex)

| Item | Lines |
|---|---|
| `motor/sector_pi.c` — hoist 11 CCP5 SFR refs + 6 HAL_Capture refs into `hal_zc_state` calls or `#if FEATURE_V4_CCP_DIAG` guards | ~120 |
| `garuda_service.c` — split `HAL_Capture_IsRisingZc()` location, guard `_CCP2Interrupt`/`_CCP5Interrupt` bodies | ~50 |
| New `hal_zc_state.{c,h}` (floating phase / expected polarity API extracted from `hal_capture.c`) | ~80 |
| **Total** | **~250 lines** |

This is small relative to the 91 KB `garuda_service.c` + 51 KB `sector_pi.c`, but it is not zero. The v4 claim of "~0 lines algorithm-side changes" was wrong; v5 budgets for it.

### Note on FPU (AK has 64-bit FPU; CK does not)

dsPIC33AK128MC106 has a hardware double-precision FPU; dsPIC33CK does not. **For the 6-step port we keep CK's Q15 / integer arithmetic verbatim** — sector_pi.c's hot path is overwhelmingly integer math on `uint16_t` HR timestamps and an `int32_t` PI integrator, with Q15 used only for the throttle amplitude (`actualAmplitude`). Converting to float would be ~200 more lines of refactor for marginal benefit in this code path. The FPU is left idle in the 6-step build; that's fine. A future float refactor can take advantage of it but is out of scope for this port. (The FOC path in `dspic33AKESC/foc/an1078_*` is float-native and uses the FPU.)

---

## 3. New project layout

The new project lives under `PATA6847/ATA6847L/` alongside the Microchip vendor reference and the carrier datasheets, so all new-board materials are grouped:

```
PATA6847/
├── garuda_6step_ck.X/              (FROZEN at 24703f6 — reference only, never touched again)
├── ATA6847L/                       (new-board materials directory)
│   ├── AN6285-...pdf, EV94U98A...pdf, dsPIC33AK128MC106...pdf  (carrier + DIM datasheets)
│   ├── EV94U98A_CK_x05.X/          (Microchip vendor reference firmware — ATA6847L register layout reference only)
│   ├── EV53A92A+Design+Files/      (carrier schematic — needed to close §9 SPI-pin gap)
│   └── garuda_ata6847_ak.X/        (NEW — this port lives HERE)
│   ├── nbproject/                  (MPLAB X, target dsPIC33AK128MC106, XC-DSC v3.30)
│   ├── garuda_config.h             (COPY from CK, retune clock constants for AK)
│   ├── garuda_service.c            (COPY + adapt — ISR vector names, drop SCCP IC ISRs to diag)
│   ├── garuda_types.h              (COPY from CK)
│   ├── main.c                      (COPY + retarget HAL init order)
│   ├── Makefile-cli.mk             (CLI build, MOTOR_PROFILE=N)
│   ├── motor/                      (COPY from CK + ~120-line sector_pi.c cleanup per §2)
│   ├── gsp/                        (COPY from CK + extend; see §7)
│   ├── scope/                      (COPY from CK)
│   ├── input/                      (COPY from CK if present; DShot later)
│   ├── hal/                        (REWRITTEN for AK)
│   │   ├── port_config.{c,h}       (PPS + GPIO map for AK pin numbering)
│   │   ├── clock.c                 (200 MHz Fosc / 100 MHz Fp / 400 MHz PWM clock, with PMD clears)
│   │   ├── hal_trap.c              (copied from CK)
│   │   ├── hal_pwm.{c,h}           (PG1/2/3 60 kHz complementary, PG1TRIGA for ADC)
│   │   ├── hal_adc.{c,h}           (AD1/AD2 + OA1/2/3, PG1TRIGA-triggered, ISR is the ZC detector)
│   │   ├── hal_zcd.{c,h}           (GPIO reads for BEMF1/2/3 + ANSEL=0/TRIS=1 init)
│   │   ├── hal_zc_state.{c,h}      (NEW — always-present API for floating-phase + expected-polarity state,
│   │   │                            extracted from CK's hal_capture.c. Used by ADC ISR + sector_pi.c.)
│   │   ├── hal_com_timer.{c,h}     (SCCP4 HR free-running 640 ns/tick + SCCP3 one-shot scheduler)
│   │   ├── hal_capture.{c,h}       (OPTIONAL diagnostic SCCP2 IC — FEATURE_V4_CCP_DIAG=0 by default.
│   │   │                            ONLY hardware-capture code lives here, no state tracking.)
│   │   ├── hal_spi.{c,h}           (SPI to ATA6847L; pins per §4 gap)
│   │   ├── hal_ata6847.{c,h}       (port CK driver + ATA6847L register deltas)
│   │   ├── hal_comparator.{c,h}    (CMP3 HW overcurrent on OA3OUT)
│   │   ├── hal_opa.{c,h}           (OA1/2/3 enable)
│   │   ├── hal_timer1.{c,h}        (50 µs system tick — Timer1)
│   │   ├── hal_uart.{c,h}          (UART1 @ 115200 for GSP)
│   │   ├── hal_eeprom.{c,h}        (flash emulation, V3 schema)
│   │   │   └── hal_clc.{c,h}       (FALLBACK only — see §2 ladder. Not built by default.)
│   │   └── tools/
│   │       └── pot_capture.py      (COPY from CK)
└── docs/
    └── garuda_6step_ak_port_plan.md  (this file)
```

**After scaffolding, all subsequent firmware work happens in `PATA6847/ATA6847L/garuda_ata6847_ak.X/`.** No further commits to `garuda_6step_ck.X/`.

---

## 4. Pin map (DIM standard, AK128MC106 ↔ ATA6847L carrier)

Derived from AN6285 (MP205 → ATA6847L) cross-referenced with the dsPIC33AK128MC106 DIM info sheet DS70005527:

| Function | DIM pin | ATA6847L | AK pin (TQFP) | AK function |
|---|---|---|---|---|
| **PWM1H** (phase A high → IH1) | 1 | 36 | 53 | RD2 / **PWM1H** / RP51 |
| **PWM1L** | 3 | 35 | 54 | RD3 / **PWM1L** / RP52 / TDI |
| **PWM2H** | 5 | 38 | 51 | RD0 / **PWM2H** / RP49 |
| **PWM2L** | 7 | 37 | 52 | RD1 / **PWM2L** / RP50 / TCK |
| **PWM3H** | 2 | 40 | 43 | RC3 / **PWM3H** / RP36 |
| **PWM3L** | 4 | 39 | 44 | RC4 / **PWM3L** / RP37 |
| **BEMF1** (phase A ZCD digital) | 9 | 31 | 24 | **RB9** / RP26 / AD2AN10 |
| **BEMF2** (phase B ZCD digital) | 11 | 32 | 23 | **RB8** / RP25 / AD1AN11 |
| **BEMF3** (phase C ZCD digital) | 22 | 33 | 9 | **RA10** / RP11 / AD2AN7 |
| Shunt1 IA+ / – | 13 / 15 | (alt OPP1/OPN1) | 14 / 13 | RA4 OA1IN+ / RA3 OA1IN- |
| Shunt1 OA1OUT (ADC) | 17 | — | 12 | RA2 / OA1OUT / AD1AN0 |
| Shunt2 IB+ / – | 21 / 23 | 29 / 28 | 22 / 21 | RB2 OA2IN+ / RB1 OA2IN- |
| Shunt2 OA2OUT | 25 | — | 20 | RB0 / OA2OUT / AD2AN1 |
| ShuntBUS IBUS+ / – | 29 / 31 | 26 / 25 | 17 / 16 | RB5 OA3IN+ / RA6 OA3IN- |
| ShuntBUS OA3OUT | 33 | — | 15 | RA5 / OA3OUT / AD1AN3 / CMP3A |
| Pot RV1 | 28 | — | 6 | RA11 / AD1AN10 |
| VBUS (÷16) | 39 | — | 2 | RA7 / AD1AN6 |
| TEMP NTC | 26 | — | 7 | RA8 / AD2AN9 |
| VREF 1.65 V | 37 | — | (DIM bias) | OA1/2/3+ via R12/R18/R6 |
| **nCS** (= `GATE_DVR_ENABLE`) | **6 or 8** | nCS direct | pin 41 (DIM 6: RC2/RP35) or pin 42 (DIM 8: RC5/RP38) | GPIO output — confirm DIM 6 vs 8 by probe at Phase 0 |
| **nIRQ** | **40** | nIRQ via R20 0R | pin 32 | RB11 / RP28 — GPIO input + IOC (do NOT enable SPI2's SDI routing) |
| **MOSI** | **42** | via R94 0R | pin 35 | RC6 / RP39 — PPS-route to SPIx SDO output |
| **MISO** | **44** | via R70 0R | pin 36 | RC7 / RP40 — PPS-route to SPIx SDI input |
| **SCK** | **46** | via R96 0R | pin 33 | RC8 / RP41 — PPS-route to SPIx SCKO output |

**Jumper config (EV94U98A user guide Table 2-2):**
- `J5 = BEMF3`, `J7 = BEMF2`, `J9 = BEMF1` → route ATA's BEMF comparator outputs to DIM 9 / 11 / 22
- `J11 = 3V3` (AK is 3.3 V)
- `J2 = VDD2` (carrier supplies MCU)
- ATA6847L firmware: `GDUCR1.BEMFEN = 1`, `CCSCR.CSA1EN = 0` (CSA1 unused — MCU's OA1 amplifies shunt1 differential)

**Shunt1 path clarification**: J5/J7/J9 in BEMF position does not disconnect Shunt1's differential nets. M1_SHUNT_IA_P (DIM 13) and M1_SHUNT_IA_N (DIM 15) still carry the shunt-amp differential into the MCU's internal OA1. What jumpering disables is the ATA6847L's internal CSA1 (= `M1_IA_EXT` path on DIM 19). **MCU OA1 path remains; ATA CSA1 path is unavailable.**

---

## 5. Peripheral substitution map (CK → AK), simplified to the load-bearing path

| Function | CK peripheral | AK peripheral | Notes |
|---|---|---|---|
| 60 kHz center-aligned PWM | PG1/2/3 | PG1/2/3 | AK already configured in `dspic33AKESC/hal/hal_pwm.c`; bump from 45 kHz to 60 kHz |
| ADC (currents, Vbus, Pot, Temp, **+ ZC detection in ISR**) | ADC w/ PG1TRIGA | AD1/AD2 + Shared w/ PG1TRIGA | AK already wired in `dspic33AKESC/hal/hal_adc.c`; AD1CH3=IA, AD2CH2=IB, AD1CH5=Pot, AD2CH1=Vbus |
| HR free-running timer (640 ns/tick) | SCCP4 mode 0000, prescale 1:64, never toggled | SCCP4 same config | Read via `CCP4TMRL` inside the ADC ISR for ZC timestamping |
| One-shot commutation scheduler | SCCP3 | SCCP3 | `_CCP3Interrupt` calls `SectorPI_Commutate()` — same vector |
| System tick (50 µs) | Timer1 | Timer1 | AK already has this in `timer1.c` (but needs PR1/TCKPS actually configured — currently a skeleton) |
| BEMF comparator inputs | RC6 / RC7 / RD10 (GPIO read in ADC ISR via `ReadBEMFComp`) | **RB9 / RB8 / RA10** (GPIO read in ADC ISR via same pattern) | DIGITAL pins, not analog. Set ANSEL=0, TRIS=1 in port_config. |
| HW overcurrent | CMP3 + DAC | CMP3 + DAC | AK already in `hal_comparator.c` |
| SPI to ATA6847L | SPI1 | SPI1 or SPI2 | Pins per §4 gap |
| UART for GSP | UART1 @ 115200 | UART1 | AK already in `uart1.c` |
| EEPROM (CK has none) | — | Flash emulation V3 schema | AK has this in `eeprom.c` — finally a working SAVE_CONFIG |
| Optional diagnostic IC capture | SCCP2 + SCCP5 | SCCP2 only (no SCCP5 on AK) — diagnostic flag-gated | `FEATURE_V4_CCP_DIAG = 0` by default; build-gated path for bench debugging |

### Hot-path ISR map

| Vector | Source | Rate | Role |
|---|---|---|---|
| `_T1Interrupt` | Timer1 match | 20 kHz (50 µs) | State machine tick, OL ramp, 1 ms sub-tick |
| `_ADCInterrupt` (or AK's named ADC vector) | ADC done, triggered by PG1TRIGA | 20 kHz (PWM-mid) | Currents/Vbus/Pot reads **+ BEMF ZC detection + `CCP4TMRL` timestamp** |
| `_CCT3Interrupt` | SCCP3 period match | per commutation | Sector PI: read `v4_captureValid` / `v4_lastCaptureHR`, run PI, schedule next |

That's it for the hot path. No CCP2/CCP5 ISRs in the motor-spin build.

### Diagnostic ISRs (optional, off by default)

| Vector | Build flag | Purpose |
|---|---|---|
| `_CCP2Interrupt` | `FEATURE_V4_CCP_DIAG = 1` | Bench debugging: capture BEMF edges with hardware timestamping, log to scope buffer. Not consumed by motor logic. |

When `FEATURE_V4_CCP_DIAG = 0`, the ISR body is `#if 0`-ed out and the SCCP2 module is left disabled. No PPS routing, no MOD configuration, no FIFO concerns.

### PWM clock

- `CLK5 = 400 MHz` from PLL VCO-div path (already done in `dspic33AKESC/hal/clock.c:166`).
- `PCLKCONbits.MCLKSEL = 1` (use CLK5 as PWM master clock).
- `PG{1,2,3}CONbits.CLKSEL = 0b01` (master clock direct, no divide).
- At 60 kHz center-aligned: `MPER ≈ 400e6 / (2 × 60e3) - 1 = 3332`. Deadtime granularity 2.5 ns/count.

### PWM deadtime

CK uses small MCU deadtime (~50 ns / 20 counts at 400 MHz). ATA6847L's `GDUCR1.CCEN = 1` does cross-conduction protection internally (`CCPT` configurable 0–6300 ns). Start with 20 counts on both PG{1,2,3}DTH and DTL; tune up only on scope evidence of shoot-through.

---

## 6. New code: just `hal_ata6847.c` and `hal_zcd.c`

Algorithm-side: ~250 lines per §2 layer table breakdown (sector_pi.c cleanup of CCP5 SFR refs, garuda_service.c ISR guards, new hal_zc_state state API). Hot-path HAL changes follow:

### 6a. `hal_ata6847.c` — SPI driver
Port CK's working driver (`garuda_6step_ck.X/hal/hal_ata6847.c`) and apply the ATA6847L register deltas from the vendor reference at `PATA6847/ATA6847L/EV94U98A_CK_x05.X/ATA6847_drv/ATA6847.{c,h}`:
- `GDUCR4` adds bit 6 `VDHOVSD` (bus OV shutdown)
- `DOPMCR` adds bit 6 `VdIOOVSD`
- `WDCR1` adds `WDSLP` bit
- `MLDRR` widens DIAGn to 2 bits each

Public API matches CK exactly. Every register field is exposed as a GSP parameter (§7).

### 6b. `hal_zcd.c` — BEMF comparator reads

```c
/* GPIO read called from ADC ISR via ReadBEMFComp(). Pins MUST be DIGITAL.
 * dspic33AKESC/hal/port_config.c:101 currently sets RB9/RB8/RA10 ANALOG
 * for legacy AK ADC phase-voltage sensing — override to DIGITAL here. */
static inline uint8_t HAL_ZCD_ReadPhaseA(void) { return _RB9; }
static inline uint8_t HAL_ZCD_ReadPhaseB(void) { return _RB8; }
static inline uint8_t HAL_ZCD_ReadPhaseC(void) { return _RA10; }

/* Phase-selector: motor/bemf_zc.c knows which phase is floating per step;
 * ReadBEMFComp() dispatches to the correct GPIO read. */
uint8_t HAL_ZCD_ReadComp(uint8_t floatingPhaseIdx);

/* ANSEL=0, TRIS=1 for RB9/RB8/RA10. Called once from port_config. */
void HAL_ZCD_PinInit(void);
```

That's the whole shim. No PPS routing, no MOD flipping, no atomic re-arm sequence, no FIFO drain. The ADC ISR just calls `HAL_ZCD_ReadComp()` three times (deglitch), takes a majority vote, and stamps `CCP4TMRL`.

### 6c. `hal_zc_state.{c,h}` — always-present state API (extracted from CK's hal_capture.c)

Per Codex review item 3: CK's `hal_capture.c` does **two things** today — it tracks the *floating phase and expected polarity for the upcoming sector*, **and** it owns the hardware IC capture peripherals. Only the second part is diagnostic-optional. The first part is consumed by `_ADCInterrupt` (line 424 calls `HAL_Capture_IsRisingZc()`) and by `sector_pi.c` (calls `HAL_Capture_Configure()` to declare the next sector's floating phase + expected polarity). Splitting the two:

```c
/* hal_zc_state.h — always present, no hardware capture dependency */

void HAL_ZcState_SetSector(uint8_t floatingPhaseIdx, bool risingZcExpected);
uint8_t HAL_ZcState_GetFloatingPhase(void);          /* used by ADC ISR ReadBEMFComp() */
bool    HAL_ZcState_IsRisingZcExpected(void);        /* used by ADC ISR for the comp-match check */
void    HAL_ZcState_SetBlankingEnd(uint16_t blankingEndHR);
uint16_t HAL_ZcState_GetBlankingEnd(void);
```

Sector PI calls `HAL_ZcState_SetSector()` once per commutation. ADC ISR calls `HAL_ZcState_GetFloatingPhase()` and `HAL_ZcState_IsRisingZcExpected()` every PWM mid-trigger. **Both work regardless of `FEATURE_V4_CCP_DIAG`.**

### 6d. `hal_capture.c` — hardware IC, diagnostic only

```c
#if FEATURE_V4_CCP_DIAG
/* OPTIONAL diagnostic: SCCP2 IC capture for bench debugging only.
 * NOT used by the motor-spin path (ADC ISR is authoritative for ZC).
 * Outputs are logged to scope buffer; no motor-control consumption.
 * No PPS swap, no MOD flipping — fixed-polarity, single-channel only. */
void HAL_Capture_Init(void);
uint16_t HAL_Capture_GetLastTimestamp(void);
#endif
```

When `FEATURE_V4_CCP_DIAG = 0`, this whole file is `#if`-ed out. SCCP2 module stays disabled. PPS unused. Zero impact on motor.

When `FEATURE_V4_CCP_DIAG = 1`, the developer gets a bench-only hardware capture path for A/B comparison against ADC-ISR ZC. Motor still runs from ADC-ISR ZC; hardware capture is read-only telemetry.

### 6d. `hal_com_timer.c` — SCCP4 HR + SCCP3 scheduler

```c
/* SCCP4: HR free-running timer, mode 0000, prescale 1:64, never toggled. */
void HAL_ComTimer_Init(void);
static inline uint16_t HAL_ComTimer_ReadTimer(void) { return CCP4TMRL; }

/* SCCP3: one-shot commutation scheduler. */
void HAL_ComTimer_ScheduleAbsolute(uint16_t targetHR);
```

Same public API as CK. Once initialized, `CCP4TMRL` is the canonical HR timestamp for every consumer (ADC ISR, SCCP3 ISR, commutation telemetry).

---

## 7. Every tuning knob becomes a GSP param

CK GSP today: **38 params, 8 groups, 3 profiles**. After the port: **~60 params, 12 groups, 4 profiles** (add HiZ1460). Every compile-time `#define` in `garuda_config.h` that affects runtime behavior is promoted.

| Group | New runtime params (was compile-time) |
|---|---|
| Motor identity | (existing) POLE_PAIRS, KV, RS, LS |
| **PWM** | **PWM_FREQUENCY_HZ**, **PWM_DRIVE_MODE** (complementary/unipolar), **DEADTIME_NS** |
| Startup | (existing) ALIGN_TIME_MS, ALIGN_DUTY_PCT, INITIAL_STEP_PERIOD, MIN_STEP_PERIOD, RAMP_ACCEL, RAMP_DUTY_PCT, RAMP_TARGET_ERPM, INIT_ERPM, SINE_ALIGN_MOD_PCT, SINE_RAMP_MOD_PCT |
| ZC Detection | (existing) ZC_DEGLITCH_MIN/MAX, ZC_TIMEOUT_MULT, ZC_DESYNC_THRESH, ZC_MISS_LIMIT, **+ new:** ZC_BLANK_PCT_SLOW/FAST, ZC_DEMAG_DUTY_THRESH, ZC_DEMAG_BLANK_EXTRA, ZC_BLANKING_PCT |
| **V4 Sector PI** | **V4_KP_SHIFT, V4_KI_SHIFT, V4_PHASE_ADVANCE_DEG_X10, V4_RC_DELAY_HR, V4_PLAUSIBILITY_LOW_HR, V4_PLAUSIBILITY_HIGH_HR, V4_HANDOFF_ERPM, V4_BLOCK_COMMUTATION_EN, V4_BLOCK_ENTER_ERPM, V4_BLOCK_EXIT_ERPM** |
| Timing Advance | (existing) TIM_ADV_MIN_DEG, TIM_ADV_MAX_DEG, TIM_ADV_START_ERPM, **+ new:** TIMING_ADVANCE_LEVEL (0–7 AM32-style band override) |
| CL | (existing) CL_IDLE_DUTY_PCT, MAX_CL_ERPM, DUTY_SLEW_UP, DUTY_SLEW_DOWN |
| **ATA Protect** | (existing) ILIM_ENABLE, ILIM_SHUTDOWN, ILIM_DAC, ILIM_FILTER, SC_ENABLE, SC_THRESHOLD, SC_FILTER |
| **ATA GDU** | (existing) BEMF_ENABLE, CROSS_COND_TIME, EDGE_BLANKING, CSA_GAIN, **+ new:** HS_SLEW_RATE, LS_SLEW_RATE, ADDT_HS, ADDT_LS, VGS_UV_FILTER, VDH_OV_SHUTDOWN |
| Voltage | (existing) VBUS_OV, VBUS_UV |
| Recovery | (existing) DESYNC_RESTART_MAX, RECOVERY_TIME_MS |
| **Input** | **INPUT_SOURCE** (pot/PWM/DShot), DSHOT_RATE, PWM_CAL_MIN, PWM_CAL_MAX, ARM_THROTTLE_PCT |
| **Diagnostic** | **FEATURE_V4_CCP_DIAG** (toggleable at boot — gates the SCCP2 IC diagnostic ISR) |

**Hot-writable (no motor stop)**: timing advance, blanking %, ATA EGBLT/SCTHSEL/slew (writes through SPI between commutations), V4 PI gains, duty slew rates, block-comm enter/exit thresholds.

**Stop-required**: PWM frequency, deadtime, motor pole pairs, profile select, drive mode, diagnostic flag.

Two new GSP commands for live ATA register tinkering:
- `0x40 GET_ATA_REG` — payload `u8 addr` → response `u8 value`
- `0x41 SET_ATA_REG` — payload `u8 addr + u8 value` → echo

EEPROM persistence: lift `dspic33AKESC/hal/eeprom.c`, bump to V3 schema (~3 KB block: 4 profiles × ~60 params × ~3 bytes + ATA register cache + active profile + CRC). `SAVE_CONFIG` writes and verifies; `LOAD_DEFAULTS` resets active profile to ROM defaults.

Snapshot stays CK-compatible (368 bytes) as the contract. Append AK-specific fields at the tail: `ata6847_dsr1`, `ata6847_sir1_latched`, `pwm_freq_active`, `eeprom_dirty`, `input_source_active`.

---

## 8. Phased milestones

### CK A/B gate — DROPPED

v5 carried a two-stage CK A/B gate to empirically prove CCP capture is inert. **Dropped in v6 after author confirmation**: the firmware author has already verified on the CK bench that the ADC ISR is the authoritative ZC path. The `sector_pi.c:825` "load-bearing" comment is a stale artifact from a 2026-04-21 debug session that was never cleaned up after the actual mechanism was identified. No new information would come from running the gate.

**Safety valve preserved**: `FEATURE_V4_CCP_DIAG` remains in the AK codebase as a build flag (default 0). If — hypothetically — AK Phase 4 surfaces an unexpected CCP dependency, flipping the flag to 1 wires up the diagnostic SCCP2 capture path in hours, not weeks. The architectural risk has a fast escape hatch.

**No further CK firmware work is in scope.** CK tree (`garuda_6step_ck.X/`) is frozen at commit 24703f6 as reference; all new work happens in `PATA6847/ATA6847L/garuda_ata6847_ak.X/`.

### Phase 0 — Project scaffold + blink (1–2 days)
- Create `garuda_ata6847_ak.X/` MPLAB X project, target `dsPIC33AK128MC106`, DFP v1.4.172
- Reference and adapt from `dspic33AKESC/hal/`:
  - `clock.c` — **add PMD register clears** (missing on AK; CK does this at `garuda_6step_ck.X/hal/clock.c:105`)
  - `timer1.c` — **actually configure** PR1, TCKPS, ON for a real 50 µs tick (current `dspic33AKESC/hal/timer1.c:96` is a skeleton)
  - `port_config.c` — **override** RB9/RB8/RA10 to DIGITAL (`dspic33AKESC` sets them analog for legacy ADC phase-voltage sensing); add SPI PPS once §4 gap closes
  - `eeprom.c` — enable real flash + validate write path (`:68` defaults real flash off and flags PAC adaptation concern)
  - `hal_comparator.c` — `#if 0` the CMP1/CMP2 BEMF-comparator block at `:61`; keep only CMP3 for HW overcurrent
- Bring over **trap handlers** from `garuda_6step_ck.X/hal/hal_trap.c`
- Audit IVT vector names — AK ADC vector differs from CK's `_ADCInterrupt`. Confirm before copying ISR signatures.
- ADC calibration per AK datasheet (PWREN, WARMTIME settle, optional self-cal trigger)
- `garuda_config.h`: `FEATURE_FOC_*=0`, `FEATURE_GSP=1`, `FEATURE_V4_SECTOR_PI=1` and `FEATURE_V4_MIDPOINT_ZC=1` (the milestone path — V3 with SCCP1 FastPoll is **not** what we're porting; see Phase 3 note below), `FEATURE_V4_CCP_DIAG=0`, board ID = AK_ATA6847L
- **Gate:** LD blinks at 1 Hz; GSP PING responds at 115200 baud; injected division-by-zero hits the math trap.

### Phase 1 — ATA6847L SPI bring-up (2–3 days)
- SPI DIM-pin mapping resolved (post v6 schematic extraction): nIRQ=DIM 40 (RB11), MOSI=DIM 42 (RC6/RP39), MISO=DIM 44 (RC7/RP40), SCK=DIM 46 (RC8/RP41). nCS = GATE_DVR_ENABLE on either DIM 6 (RC2) or DIM 8 (RC5) — confirm with multimeter probe at Phase 0. Populate carrier 0R resistors R20, R70, R94, R96 (DIM-side); depopulate R69, R97, R98, R72 (MIKRO/Xplained mirrors) if shipped populated.
- Implement `hal_spi.c` + `hal_ata6847.c` (port from CK + apply L-variant register deltas from §6a)
- GSP commands 0x40/0x41 (ATA register R/W) so GUI drives chip manually
- **Gate:** read DSR1 = 0b00100100 (GDUS=1 after standby→normal); round-trip read/write loop test on RWPCR

### Phase 2 — PWM + override + scope-verified timing (3–4 days, was 2–3) — *Codex v4 review added scope gate*
- Port `hal_pwm.c` for AK: PG1/2/3 at 60 kHz center-aligned complementary, master/slave, deadtime, override-controlled 6-step
- Port `motor/commutation.c` + bootstrap charge sequence
- GSP `START_MOTOR` drives manual step rotation (no BEMF yet)
- **Scope gate (new):** before motor testing, **first capture the CK reference**, then verify AK matches. Per Codex v5 review item 5, the "PG1TRIGA fires at PWM mid-ON valley" expectation depends on POLH polarity and complementary-mode timing that's easier to measure than reason about.

  **Step 2A — capture CK reference (~30 min on CK board, do this once):**
  - On the running CK board at full pot (60 kHz milestone build), instrument the CK ADC ISR with a GPIO toggle on the spare debug pin.
  - Probe PWM1H (RB10) and the GPIO toggle on a 2-channel scope.
  - Record: PWM frequency, the ADC-ISR phase relative to PWM1H rising/falling edge, deadtime measured directly.
  - Save the screenshot. This is the **AK reference target**.

  **Step 2B — verify AK matches:**
  - **PWM1H pin (AK: RD2) at ~96% duty**: AK PWM frequency must match CK measured value within ±1 kHz. If 45 kHz or 90 kHz, AK PWM clock config is wrong — fix before moving on.
  - **PG1TRIGA-driven ADC ISR timing**: AK ADC-ISR GPIO toggle must fire at the **same phase relative to PWM1H** as CK does, within ±1 µs. If the phase differs, fix `PG1EVTL` ADTRG configuration on AK before Phase 3. (The numeric "8.3 µs after falling edge" target from v5 was a calculated approximation; the actual reference is whatever CK measures, since CK's empirically-validated configuration is the contract.)
  - **Deadtime**: AK PWM1H/PWM1L deadtime must match CK measured value within ±20 ns. If CK measures e.g. 50 ns, AK should too. (CK's `750 ns` macro from v2 of the plan was an error — measure CK's actual deadtime and match it.)
- **Motor gate:** Hurst spins at 1–2k eRPM on bench with manual step rate; current peaks visible in IA/IB/IBUS ADC; ATA6847L stays fault-free

### Phase 3 — Low-speed V4 ADC-ZC bring-up (3–5 days) — *renamed per Codex v4 review*

Codex v4 review item 4 was correct: the v4 plan said "Phase 3 — V3 reactive ZC" but the ADC-ISR ZC code being ported is under `FEATURE_V4_MIDPOINT_ZC` (V4-specific), not the V3 SCCP1 FastPoll path. We're porting the V4 milestone path, not V3. Phase 3 is renamed to reflect this.

- Implement `hal_zcd.c` (GPIO reads on RB9/RB8/RA10 + `HAL_ZCD_ReadComp(floatingPhaseIdx)`)
- Implement `hal_zc_state.c` (always-present API extracted from CK `hal_capture.c` — see §6c)
- Implement `hal_com_timer.c` (SCCP4 HR free-running + SCCP3 one-shot scheduler)
- Port `motor/sector_pi.c` with the ~120-line cleanup per §2 layer table: hoist CCP5 SFR refs into `hal_zc_state` calls or guard behind `FEATURE_V4_CCP_DIAG=0`
- Wire ADC ISR to do BEMF 3-read deglitch + state check + `CCP4TMRL` timestamp + `v4_lastCaptureHR`/`v4_captureValid` set (matches `garuda_service.c:404–481` exactly)
- Wire `_CCT3Interrupt` to `SectorPI_Commutate()`
- **Build with `FEATURE_V4_SECTOR_PI=1`, `FEATURE_V4_MIDPOINT_ZC=1`, `FEATURE_V4_CCP_DIAG=0` from the start.**
- **Low-speed gate:** Hurst (low KV) reaches steady CL at 10–30k eRPM no-load; sector PI tracking visible in telemetry; ZC accept rate >95%; no `actualForcedComm` increments for 60 s sustained.

### Phase 4 — Milestone parity (4–6 days)
- Tune blanking, advance, V4 PI gains per `docs/v4_motor_tuning_guide.md` and `docs/v4_high_speed_ceiling_debug.md`
- Run the full bench protocol from `CLAUDE.md` Testing Checklist
- **Milestone gate:** A2212 ≥150k eRPM at 12V no-load, **2810 ≥228k eRPM at 25V no-load with block-commutation engaged** (matching CK's commit 24703f6 milestone, not a softer 200k target). Sync mode SHADOW→OWNED transition observed. Block-commutation engages cleanly.

### Phase 5 — Runtime configurability rollout (3–4 days)
- Promote every listed `#define` to GSP param (§7)
- Per-profile defaults in ROM (Hurst/A2212/2810/HiZ1460)
- EEPROM V3 schema; SAVE_CONFIG writes and verifies; LOAD_DEFAULTS resets
- Hot-writable param metadata flag enforced
- **Gate:** with GUI alone, change timing advance, EGBLT, SCTHSEL, V4 PI gains live on a spinning motor; settings persist across power cycle.

### Phase 6 — Prop validation + protection sweep (3–5 days)
- A2212 + 8x4.5 prop at 12 V: idle stability, full throttle, pot release transients
- 2810 + prop at 18 V
- HiZ1460 profile validation
- Block-commutation exit hardening per `docs/v4_block_comm_exit_tuning.md`
- Vbus OV/UV fault injection, ATA6847L `SCSDEN` co-operation with MCU `CMP3`
- **Gate:** all 4 profiles pass the CK board's `Testing Checklist` (CLAUDE.md). New baseline merged to `main`. CK firmware tree (`garuda_6step_ck.X/`) frozen.

### Phase 7 — Diagnostic CCP capture + DShot (3–5 days, optional)
- Build `hal_capture.c` behind `FEATURE_V4_CCP_DIAG`. SCCP2 IC with fixed-polarity per phase (no MOD flipping). Outputs logged to scope buffer, not consumed by motor.
- A/B compare ADC-ISR ZC vs hardware-IC ZC at high speed; document jitter and miss-rate envelopes.
- DShot RX: bring over `dspic33AKESC/hal/hal_input_capture.c` (SCCP4 currently configured for DShot — but SCCP4 is now the HR timer for the motor). DShot peripheral allocation TBD; candidate paths: (a) replace V3 fast-poll with PWM-trigger-driven sampling in ADC ISR, free SCCP1 for DShot; (b) drop V3 entirely after V4 milestone, use SCCP1 for DShot.
- **Gate:** Betaflight rig drives DShot300; ESC streams eRPM/temp/voltage telemetry back; CCP_DIAG capture confirms or refutes a hardware-IC speed advantage.

---

## 9. Open questions / risks

### Still open
1. **SPI DIM-pin assignments — RESOLVED.** Extracted from EV92R69A user guide page 27 (schematic A.3) via pdftotext: **nIRQ=DIM 40, MOSI=DIM 42, MISO=DIM 44, SCK=DIM 46**. Cross-referenced AK DIM info sheet → AK pin assignments: **RB11/RP28 (nIRQ), RC6/RP39 (MOSI), RC7/RP40 (MISO), RC8/RP41 (SCK)**. **nCS = GATE_DVR_ENABLE** (per SAM-C21 firmware which uses PB10 for both); located on DIM 6 (RC2/RP35) or DIM 8 (RC5/RP38) — confirm with probe at Phase 0. SPI peripheral choice (SPI1 / SPI2 / SPI3) is free via PPS — but **avoid SPI2 SDI routing** because DIM 40 (RB11) is the SDI2 native pin and we use it as a GPIO input for nIRQ. Recommend SPI1 or SPI3 for the ATA6847L SPI peripheral. Phase 1 unblocked.
2. **EEPROM PAC adaptation**. `dspic33AKESC/hal/eeprom.c:68` defaults real flash off, comments say PAC (Peripheral Access Control) may need adapting. Validate in Phase 0.
3. **GUI board profile**. GUI must learn `boardId = GSP_BOARD_AK_ATA6847L` (suggest `0x0003`). Small change in `gui/src/serial/protocol.ts`.
4. **AK ADC calibration procedure** differs from CK. Follow AK datasheet (PWREN, WARMTIME, optional self-cal). Existing `dspic33AKESC/hal/hal_adc.c` claims to do this; validate when bringing up ADC in Phase 2.
5. **`__builtin_disi` unsupported on XC-DSC** (per CLAUDE.md). Grep `motor/*` for any use; replace with double-read / seqlock pattern. Quick audit before Phase 4 build.
6. **CLC fallback compile-level proof**. If raw-BEMF noise turns out problematic, `hal_clc.c` is the recovery option (§2 ladder). Phase 0/1 should include a `HAL_CLC_DryRun()` to validate AK CLC input MUX feasibility before any Phase 3 reliance.

### Resolved by code verification (no longer relevant)
- ~~SCCP5 doesn't exist on AK~~ → **not load-bearing in the hot path**, but the absence is still a **compile blocker** until all SCCP5-named SFR references in `motor/sector_pi.c` (lines 280, 282, 542, 561, 563, 588, 616, 715, 717, 748, 750, 834) are removed or guarded behind `FEATURE_V4_CCP_DIAG`. See §2 layer table for the ~120-line cleanup budget.
- ~~Timer2/Timer3 chain for HR timer~~ → moot; SCCP4 HR timer matches CK directly.
- ~~Single-IC mode-flip equivalence to dual-IC~~ → moot; motor doesn't use IC capture.
- ~~CCPON-while-changing-MOD~~ → moot; no MOD changes.
- ~~PPS-swap hazard during commutation~~ → moot; no PPS swap.
- ~~Cross-domain timestamp conversion~~ → moot; only one domain (CCP4TMRL).
- ~~DMA / FIFO setup for capture~~ → moot; no FIFO consumption.
- ~~CK CLC PWM-windowed sampling fallback~~ → still relevant but lower priority since the ADC ISR's 3-read deglitch + blanking gate already gives PWM-aligned noise rejection.

---

## 10. Success criteria

When this plan completes:

1. **Same algorithm, same milestones** — `garuda_ata6847_ak.X/` matches or exceeds the CK 228k eRPM milestone on the 2810 motor.
2. **GUI alone configures everything** — no recompile to switch profiles, change PWM frequency, tune ATA registers, tune V4 PI gains, change advance level / blanking / slew rate.
3. **EEPROM persistence works** — SAVE_CONFIG actually saves; settings survive power cycle.
4. **CCP IC capture remains as an optional diagnostic** — engineer on bench can enable `FEATURE_V4_CCP_DIAG=1` to compare against ADC-ISR ZC without disturbing the motor path.
5. **CK firmware frozen** — `garuda_6step_ck.X/` stays at commit `24703f6` as the reference. All future motor work happens on the AK tree.
6. **Single source of truth** — one project, one branch, one GUI board profile.

---

## 11. Revisions log

### v1 → v2 (post first Codex review)
Eight findings, six adopted. Notably: SCCP5 doesn't exist on AK (so dual-IC won't port directly), CLC deletion not justified (3-layer fallback ladder kept), "lift verbatim" from `dspic33AKESC` wrong for 5 files, PPS-swap hazard, missing PMD clears / traps / `__builtin_disi` audit.

### v2 → v3 (post second Codex review)
Three P0s + four P1s. Replaced Timer2/3 HR-timer hallucination with SCCP4-as-HR (matches CK). Added explicit timestamp-domain conversion (ccp2HrOffset). Reframed Phase 4 as a feasibility gate. Fixed PPS-swap sequence ordering. Corrected PWM deadtime to ~50 ns. Dropped DShot reservation. Added CLC compile-level proof requirement. Swept stale v1 statements.

### v3 → v4 (post code verification — this revision)

**The headline:** the v3 plan was over-engineered around CCP input-capture mechanisms that the milestone V4 build does not actually use for ZC detection.

Code verification at `garuda_service.c:360–481` (the `_ADCInterrupt` body) confirms:
- ZC detection happens in the ADC ISR, gated by `FEATURE_V4_MIDPOINT_ZC=1` and active sector phase
- BEMF comparator state is read via three GPIO polls with majority vote
- Timestamp comes from `CCP4TMRL` read inline in the ISR
- `v4_lastCaptureHR` and `v4_captureValid` are written here — the same variables `motor/sector_pi.c` reads

Code verification at `garuda_service.c:611–722` (`_CCP2Interrupt`/`_CCP5Interrupt`) confirms:
- All paths gated by `FEATURE_V5_SCHEDULER`, `v4_spActive`, or `FEATURE_V4_MIDPOINT_ZC == 0`
- None active in the 228k milestone build
- ISRs drain FIFO and clear flag but produce no consumed output

**Consequences for the AK port:**

| v3 plan element | v4 status |
|---|---|
| SCCP2 single-IC with MOD flip per phase | **Dropped from motor path.** |
| Atomic `HAL_ZCD_ArmCapture()` PPS-swap sequence | **Dropped from motor path.** |
| CCPON-while-changing-mode debate | **Moot.** |
| IC-IRQ-disable vs ICDIS distinction | **Moot.** |
| DMA/FIFO setup for IC FIFO access | **Moot.** |
| `ccp2HrOffset` cross-domain timestamp conversion | **Moot — single HR domain (CCP4TMRL).** |
| Phase 4 feasibility gate around single-IC | **Replaced.** Phase 4 gate is now "V4 milestone parity on AK with ADC-ISR ZC." No architectural feasibility risk to gate. |
| Single-FIFO refactor in `motor/sector_pi.c` | **Dropped.** sector_pi reads `v4_captureValid`/`v4_lastCaptureHR` as plain volatile vars — no FIFO interaction. |
| `garuda_service.c` "copy verbatim" | **Copy + adapt.** Drop `_CCP2Interrupt`/`_CCP5Interrupt` bodies (gate behind `FEATURE_V4_CCP_DIAG` for optional diagnostic use). Adapt ADC ISR vector name. |
| Algorithm-side changes | **~0 lines.** No motor-code refactor needed for the AK port. |

CCP IC capture infrastructure is **retained as a diagnostic-only path** behind `FEATURE_V4_CCP_DIAG` (default 0). A developer on a bench can enable it to A/B compare hardware IC capture against ADC-ISR ZC, but motor behavior is unaffected by either the flag's state or any captured data.

**Net plan impact**:
- Phase 4 budget drops from 7–10 days (v3 with feasibility gate) to 4–6 days (v4 with milestone parity check).
- HAL files reduced: `hal_capture.c` becomes optional/diagnostic. `hal_zcd.c` simplifies to GPIO reads with no PPS or atomic re-arm.
- Algorithm-side changes drop to ~0 lines.
- Critical pre-implementation gates reduce from 3 (SPI pins, CLC mux proof, Phase 0 trap test) to 2 (SPI pins + Phase 0 trap test). CLC dry-run becomes a Phase 3.5 contingent activity, not a pre-Phase-1 blocker.

### Codex findings preserved across revisions
- SPI DIM-pin gap (still requires schematic / probe)
- EEPROM PAC adaptation validation
- PMD clears, traps, `__builtin_disi` audit, ADC calibration in Phase 0
- "Lift verbatim" claims for `dspic33AKESC/hal/` files are wrong — replaced with named file:line concerns
- PWM deadtime ~50 ns matching CK practice, not 750 ns
- CLC kept as §2 fallback ladder (not deleted, not relied on)

### v4 → v5 (post third Codex review — this revision)

Codex re-reviewed v4. Confirmed the headline simplification (ADC-ISR is the actual ZC path) is correct, but flagged six concrete over-corrections. All six addressed in v5:

| # | Codex v4 finding | Severity | Action in v5 |
|---|---|---|---|
| 1 | **"Algorithm-side changes ~0 lines" is wrong.** `motor/sector_pi.c` has 11 direct CCP5 SFR references (verified: lines 280, 282, 542, 561, 563, 588, 616, 715, 717, 748, 750, 834) plus 6 `HAL_Capture_*` calls. AK has no SCCP5 — won't link. | P0 | §2 layer table revised: split into "algorithm — clean" (verbatim) and "algorithm — needs cleanup" (~120 lines in sector_pi.c). Total algorithm-side delta budgeted at **~250 lines**, not zero. |
| 2 | **"CCP ISRs are inert, drop is safe" is not proven.** CK comments at `sector_pi.c:825` and `garuda_service.c:382` say preemption / side effects were load-bearing. Hardware-level subtleties (FIFO drain jitter, ISR priority chain) may stabilize CK in ways v4 doesn't account for. | P0 | **New Pre-Phase-0 CK A/B gate.** Before any AK code is written: disable `_CCP2IE`/`_CCP5IE` and FIFO drains on the CK board, prove 228k milestone still reached. If yes → v4 assumption confirmed. If no → investigate the specific load-bearing side effect first. |
| 3 | **`hal_capture.c` is not fully optional.** `HAL_Capture_IsRisingZc()` is called from `garuda_service.c:424` (the ADC ISR), and `HAL_Capture_Configure()` from sector PI. These aren't diagnostic — they're state-tracking for the always-on ZC pipeline. | P0 | Split `hal_capture.c` into two files: **`hal_zc_state.{c,h}`** (always present — floating phase + expected polarity + blanking-end state, ~80 new lines) and **`hal_capture.{c,h}`** (diagnostic-only hardware IC behind `FEATURE_V4_CCP_DIAG`). §3 layout updated. §6c added. |
| 4 | **Phase 3 internally inconsistent.** Phase 0 was setting `FEATURE_V4_SECTOR_PI=0` to "do V3 first," but the ADC-ISR ZC code being ported is V4-only (`FEATURE_V4_MIDPOINT_ZC=1`). V3's SCCP1 FastPoll path is a different architecture. | P1 | Phase 3 renamed to "Low-speed V4 ADC-ZC bring-up." Phase 0 now sets `FEATURE_V4_SECTOR_PI=1` from the start. The V3 SCCP1 path is dropped from the port scope entirely. |
| 5 | **ADC trigger timing is load-bearing.** CK's `PG1EVTL = 0x118` and specific PG1 trigger postscale configure the exact moment the ADC ISR fires within the PWM cycle (mid-ON valley). "ADC at 20 kHz" is not enough — it has to fire at the right PWM phase. | P1 | **Phase 2 scope gate added.** Before motor testing in Phase 2, scope-verify: (a) PWM frequency 60.0±1 kHz on PWM1H, (b) ADC ISR GPIO toggle fires at PWM mid-ON valley (8.3 µs after PWM1H falling edge), (c) deadtime is ~50 ns. If any of these are wrong, Phase 3 will produce broken BEMF samples and ZC acceptance will collapse. |
| 6 | **PWM period math needs scope-backed verification.** v5 says `MPER≈3332` at 60 kHz on 400 MHz clock; CK's macro path involves integer truncation that may produce slightly different actual frequency. | P1 | Folded into the Phase 2 scope gate above. The scope decides, not the math. |

**Net v5 impact:**
- Algorithm-side delta estimate corrected: **~250 lines** (was "~0" in v4).
- New Pre-Phase-0 CK A/B gate added — 1 day of bench work on CK board, must close before AK port begins.
- Phase 2 scope gate added — explicit timing verification before any closed-loop testing.
- `hal_capture.c` split into two files: always-on state API + diagnostic hardware path.
- Phase 3 renamed and rescoped to "Low-speed V4 ADC-ZC bring-up" — V3 SCCP1 path dropped.
- v4 budget grew by ~1 day for the A/B gate, ~1 day for the scope gate, ~1 day for the larger sector_pi.c cleanup. Total: ~3 days added.

**Note on AK FPU**: v5 §2 added a paragraph clarifying that dsPIC33AK128MC106 has a hardware double-precision FPU, while CK does not. For the 6-step port, **Q15 / integer arithmetic stays verbatim** — sector_pi.c hot path is mostly integer math on `uint16_t` HR timestamps. A future float refactor can take advantage of the FPU but is out of scope. (FOC code in `dspic33AKESC/foc/an1078_*` is float-native and already uses the FPU.)

### v5 → v6 (post fourth Codex review — this revision)

Codex re-reviewed v5. Confirmed v5 is "close" but flagged six remaining items, all addressed in v6:

| # | Codex v5 finding | Severity | Action in v6 |
|---|---|---|---|
| 1 | **SP mode is still a capture-owned path.** `garuda_service.c:395` disables ADC-ZC when `v4_spActive=true`; `_CCP2Interrupt:648, 757` owns SP-mode ZC. If SP triggers on AK with CCP disabled → no ZC. | P0 | Added explicit **§1 scope boundary**: SP mode disabled in AK milestone via `SP_ENTER_ERPM = 999999`. Added SP-mode caveat to §1 architecture description. Future SP port deferred (with a noted path: re-use diagnostic `FEATURE_V4_CCP_DIAG=1` build for SP). |
| 2 | Block-comm and SP mode need clearer separation. | P1 | §1 scope boundary now explicitly states: **block-comm is in scope and works under ADC-ZC; SP mode is out of scope and disabled via SP_ENTER_ERPM=999999**. |
| 3 | **CK A/B gate doesn't fully match AK config.** v5 gate disabled IRQs but left hardware capture init intact. AK config will disable SCCP2 entirely. | P0 | A/B gate split into **two stages**. Stage 1 disables IRQs only (v5 original). Stage 2 also stubs `HAL_Capture_Init/Start/Configure` and prevents CCPON=1, mirroring AK's `FEATURE_V4_CCP_DIAG=0` build. Both stages must reach 228k for the gate to pass. |
| 4 | **Stale "verbatim" / "zero changes" wording remains** at §1 goal, §3 layout (`motor/` row), §6 intro. | P2 | All three swept. §1 now says "carries the algorithm with a bounded ~250-line cleanup." §3 layout `motor/` row now says "COPY from CK + ~120-line sector_pi.c cleanup per §2." §6 intro corrected to "~250 lines per §2." |
| 5 | **Phase 2 ADC timing reference may be fragile** — "8.3 µs after PWM1H falling edge" depends on POLH polarity, complementary timing details. Better to measure CK first and match. | P1 | Phase 2 scope gate rewritten as a **two-step** procedure: **Step 2A** captures CK reference (PWM frequency, ADC-ISR phase relative to PWM1H, deadtime — measured on running CK board). **Step 2B** verifies AK matches CK measured values within tolerance. The numeric target moves from "calculated 8.3 µs" to "CK measures X, AK must match X ± 1 µs." |
| 6 | **"SCCP5 moot" wording too broad** — it's not load-bearing for accepted ZC, but it's still a compile blocker. | P2 | §9 resolved-risks line for SCCP5 expanded: "not load-bearing in hot path, but compile blocker until all SCCP5 SFR references in sector_pi.c are removed/guarded — see §2 layer table for the ~120-line cleanup budget." |

**Net v6 impact:**
- SP mode explicitly out of scope. AK milestone takes a one-line config change (`SP_ENTER_ERPM = 999999`) to disable. Future SP port path noted but not committed.
- CK A/B gate grows from ~1 day to ~2 days (two stages instead of one). Now actually tests what AK will do, not a halfway state.
- Phase 2 timing gate becomes empirical (match CK measurement) instead of theoretical (match computed value). More robust against subtle PWM-config differences.
- Stale "verbatim" wording in §1/§3/§6 swept; all paths now point to the v5 §2 layer table for the canonical ~250-line cleanup budget.
- Document is now internally consistent across all sections — no remaining "zero changes" or "verbatim" claims that contradict §2.

### Pre-implementation gates (final for v6, SPI gap closed)

**One gate** remains:

1. **Phase 0 trap-handler + PMD-clear bring-up passes the injected-fault smoke test** (math trap fires on division by zero)

SPI DIM-pin gap **closed** by pdftotext extraction of EV92R69A user guide page 27 schematic — see §9 item 1. One small Phase-0 task remains: probe DIM 6 vs DIM 8 with multimeter to confirm which carries `GATE_DVR_ENABLE` (= nCS). 10-minute task on the bare carrier.

Phase 2 scope gate (CK reference capture + AK match) replaces the v5 theoretical timing target; that's a Phase-2 gate, not pre-implementation.

CLC dry-run remains a Phase 3.5 contingent activity (only if raw-BEMF noise problems emerge).

CK A/B gate **dropped** after author confirmation that ADC-ISR is the verified ZC path. `FEATURE_V4_CCP_DIAG` is the architectural safety valve if anything unexpected surfaces during AK Phase 4.
