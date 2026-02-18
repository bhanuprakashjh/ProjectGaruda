# dsPIC33AK128MC106 ADC + Comparator Notes (for Project Garuda)

## Scope
This note summarizes practical, implementation-relevant details of the dsPIC33AK128MC106 ADC and comparator blocks, plus board mapping constraints for:
- Motor Control DIM: `EV68M17A`
- Inverter board: MCLV-48V-300W (`DS50003297`)
- Device datasheet: `DS70005539`

Use this as a design reference for Phase 2 closed-loop sensorless commutation.

## Summary: Two Types of Comparators on dsPIC33AK128MC106

This chip has **two completely different comparator subsystems** that are often
confused. Understanding the distinction is critical for ZC detection design.

### Analog Comparator (CMP1/CMP2/CMP3) — Section 16

**What it is**: A true analog comparator that continuously monitors an analog
input against a DAC-generated threshold. Operates in the analog domain — no ADC
conversion needed.

**How it works**: The analog voltage on the comparator input pin is continuously
compared against the DAC output. When the input crosses the threshold, the
comparator output changes state. This is **instantaneous** (sub-nanosecond
propagation delay, limited only by comparator bandwidth).

**Built-in filtering**: YES — `FLTREN` enables a digital filter on the comparator
output. Requires 3 consecutive filter clock edges with the same output state before
the event propagates. Also has `HYSSEL` for selectable hysteresis (15/30/45 mV) and
an internal pulse stretcher. These features prevent noise-induced false triggers.

**Why we can't use it**: The BEMF sensing pins (RB8, RB9, RA10) are NOT routable to
CMP module inputs on the MCLV-48V-300W DIM. CMP1 input domain includes RA4, CMP2
includes RB2, CMP3 includes RB5 — all wired to op-amps for current sensing, not
phase voltage dividers. Physically impossible on this dev board. A custom production
board could route BEMF dividers to CMP-capable pins.

### ADC Digital Comparator (per-channel) — Section 15.4.7

**What it is**: A digital comparison circuit that checks the **converted ADC result**
(`ADnCHxDATA`) against threshold registers (`ADnCHxCMPLO`, `ADnCHxCMPHI`) after
each ADC conversion completes.

**How it works**: This is NOT an analog comparator. The sequence is:
1. ADC trigger fires (SCCP3 at 1 MHz in our case)
2. ADC samples the analog input (S&H circuit, ~65ns at SAMC=3)
3. ADC converts the sample to a 12-bit digital value (~140ns)
4. The digital comparator checks the conversion result against the threshold
5. If the condition (`CMPMOD`) is met, `ADnCMPSTAT.CHxCMP` flag is set
6. If the interrupt is enabled (`ADnCMPxIE`), `ADnCMPxIF` fires

**Built-in filtering**: NO. There is no consecutive-match count, no digital filter,
no hysteresis. A **single conversion** that meets the threshold condition immediately
fires the event. The comparator modes are:
- `CMPMOD=011`: Greater than `CMPLO` (for rising ZC)
- `CMPMOD=100`: Less or equal to `CMPLO` (for falling ZC)
- Other modes: between/outside window using both CMPLO and CMPHI

**Key implication**: The comparator only "checks" once per ADC conversion. It does
NOT continuously monitor the analog signal between conversions. The effective
detection rate equals the ADC conversion rate — in our case, 1 MHz via SCCP3
peripheral trigger.

### How We Achieve 1 MHz Detection Rate

The 24kHz PWM-triggered ADC channels (CH0, CH1, CH4, AD2CH0) handle pot, vbus,
threshold computation, and software ZC. These run at the PWM frequency because
duty can only change once per PWM cycle.

The comparator channels (AD1CH5, AD2CH1) are **independent** — they have their own:
- **Trigger source**: `TRG1SRC=14` (SCCP3 timer output, configured at 1 MHz)
- **Sample-and-hold circuit**: Each channel has its own S&H, so they sample
  independently of the 24kHz channels
- **Data register**: `AD1CH5DATA` and `AD2CH1DATA` update at 1 MHz, containing
  fresh conversion results from new samples triggered by SCCP3
- **Comparator circuit**: Checks each new conversion result against the threshold

Each SCCP3 tick (every 1us) triggers a fresh ADC conversion on the comparator
channels. The analog input is re-sampled, converted to digital, and the digital
comparator checks the result — all in hardware, with zero CPU involvement. Only
when the threshold condition is met does the CPU get interrupted.

**Verified**: 41,714 hardware ZC detections with zero misses on Hurst motor
across full speed range.

### Comparison Table

| Feature | Analog CMP (Section 16) | ADC Digital Comparator (Section 15.4.7) |
|---------|------------------------|----------------------------------------|
| Domain | Analog (continuous) | Digital (per-conversion) |
| Detection latency | Sub-nanosecond | ~205ns (conversion time) |
| Effective rate | Continuous | = ADC trigger rate (1 MHz) |
| Built-in filter | YES (FLTREN, 3-edge) | NO |
| Hysteresis | YES (HYSSEL, 15/30/45mV) | NO (software deadband only) |
| Threshold source | Internal DAC | Register (CMPLO/CMPHI) |
| Input pins | CMP-specific domain | Any ADC-capable pin |
| Used for ZC? | NO (can't route BEMF pins) | YES (Phase F) |

### Noise Immunity Without Built-In Filter

Since the ADC digital comparator has no built-in filter, noise immunity is
achieved through a combination of:
1. **Blanking period** (5% of step period) — masks demagnetization ringing
2. **Deadband** (4 ADC counts) — prevents triggering on noise near threshold
3. **IIR-smoothed threshold** — prevents rapid threshold changes
4. **ADC hardware oversampling** (MODE=11, ACCNUM=00, 4 samples) — +6 dB noise
   reduction, comparator fires on averaged result. Ready to enable for production.
5. **Integration observer validation** (planned) — confirms comparator ZC with
   accumulated BEMF integral before committing to commutation

---

## 1) Comparator block (CMP1/2/3): details
The device includes high-speed analog comparator instances with integrated DAC threshold generation.

### 1.1 Input/threshold model
- Comparator positive input source is selected by `DACxCON.INSEL[2:0]`.
- Comparator negative input is internally tied to the DAC output.
- Output polarity is controlled by `DACxCON.CMPPOL`.

### 1.2 Event path and timing behavior
- Comparator events can feed PWM fault/current-limit logic and interrupts.
- `DACxCON.FLTREN` enables digital filtering of comparator output.
- There is also an internal pulse stretcher path before/with filter handling.
- Enabling filtering improves noise immunity but adds delay (important for ZC phase timing).

### 1.3 Hysteresis
- `DACxCON.HYSSEL[1:0]` selects hysteresis level.
- Typical selectable hysteresis levels are 15/30/45 mV.
- Larger hysteresis reduces chatter but shifts apparent crossing time.

### 1.4 Key electrical constraints (datasheet table)
- Input offset is non-zero (characterized; around ±35 mV class depending package).
- Common-mode input range is AVSS..AVDD.
- Comparator can be very fast in core response, but configured output path/filter dominates effective event timing in firmware.

### 1.5 DAC range caveat
- Comparator DAC operating code range is constrained (effective output range approx 5%..95% of AVDD).
- Ensure chosen threshold + expected BEMF swing stay away from rails.

## 2) ADC block: important advanced capabilities
The ADC subsystem is more flexible than typical MCU ADC blocks and can be used beyond simple polling.

### 2.1 Flexible channel mapping
- Any ADC channel can map analog positive/negative inputs via `ADnCHxCON.PINSEL/NINSEL`.
- This is crucial for board-level routing mismatches (you can remap in firmware without PCB changes if the signal reaches some ADC-capable pin).

### 2.2 Trigger system (`TRG1SRC` / `TRG2SRC`)
- Supports software, back-to-back, repeat timer, and peripheral triggers.
- Allows deterministic PWM-aligned sampling windows.
- Supports gated/windowed conversions in hardware modes.

### 2.3 Conversion modes per channel
- Single conversion (`MODE=00`) — one sample per TRG1SRC trigger. TRG2SRC is IGNORED.
- Window mode (`MODE=01`) — not relevant for ZC detection.
- Integration (`MODE=10`) — accumulates raw sum of CNT samples. First conversion by TRG1SRC,
  all repeats by TRG2SRC. Comparator fires on final accumulated sum (not per-sample).
  Threshold must be scaled by CNT.
- Oversampling (`MODE=11`) — averages N samples (4/16/64/256 via ACCNUM). First conversion
  by TRG1SRC, repeats by TRG2SRC. Result is right-shifted (implicit average). Comparator
  fires on final averaged result. Threshold works directly (same scale as single sample).

**Critical discovery**: TRG2SRC=2 (immediate re-trigger) is IGNORED in MODE=00 but WORKS
in MODE=10/11 for repeat samples within the accumulation window.

Practical guidance:
- Use single conversion (`MODE=00`) for minimum-latency ZC edge timing at 1 MHz+.
- Use oversampling (`MODE=11`, `ACCNUM=00`, 4 samples) for +6 dB noise reduction while
  keeping 1 MHz result rate. Recommended for production environments with switching noise.
- Use oversampling with higher ACCNUM for cleaner VBUS reference (not timing-critical).
- Integration mode requires threshold scaling — oversampling is preferred for ZC.

See `ADC_HIGH_SPEED_TRIGGER_INVESTIGATION.md` for complete filter mode analysis.

### 2.4 Non-interruptible burst behavior
- `ACCBRST` can prevent higher-priority conversion interruption during oversampling bursts.
- Useful when deterministic multi-sample accumulation is needed.
- Recommended for BEMF oversampling to prevent 24kHz PWM-triggered channels from
  splitting the accumulation mid-burst.

### 2.5 Per-channel digital comparator
- Each ADC channel has a **digital** comparator (`ADnCHxCMPLO/CMPHI`, `CMPMOD`).
- Compares the **converted digital result** (`ADnCHxDATA`) — NOT the analog input.
- Fires only when a conversion completes and the result meets the `CMPMOD` condition.
- NO built-in filter, NO hysteresis, NO consecutive-match count (unlike analog CMP).
- Can generate per-channel comparator flags/interrupts (`ADnCMPSTAT`, `ADnCMPxIF`).
- See "Summary" section above for full comparison with analog comparators.

### 2.6 Early interrupt (`EIEN`)
- Channel interrupt can occur before final result availability to cut latency.
- Only valid for single conversion mode.
- Must be used carefully; ISR must not read result too early.

### 2.7 Calibration support
- ADC supports startup/periodic calibration controls (`CALREQ`, `ACALEN`, `CALRATE`, `CALRDY`).
- Recommended to verify calibration-ready status during initialization.

### 2.8 Sampling-time design rule (critical)
ADC acquisition must satisfy source impedance and hold capacitor charging:
- For 12-bit, practical equation from datasheet: `Tsample >= 9 * Rtotal * Chold`
- If divider/network impedance is high, increase sampling time (`SAMC`) or lower source impedance.

## 3) Board mapping reality for this project

### 3.1 MCLV phase-voltage feedback nets (board guide)
From MCLV-48V-300W mapping:
- `M1_VA -> DIM:009`
- `M1_VB -> DIM:011`
- `M1_VC -> DIM:022`

### 3.2 EV68M17A DIM mapping (DIM info sheet)
From EV68M17A pin mapping:
- `DIM:009 -> RB9 / AD2AN10` (direct)
- `DIM:011 -> RB8 / AD1AN11` (direct)
- `DIM:022 -> RA10 / AD2AN7` (direct)

Therefore, phase voltages on this stack are available on:
- VA: `RB9 (AD2AN10)`
- VB: `RB8 (AD1AN11)`
- VC: `RA10 (AD2AN7)`

### 3.3 Why original comparator-ZC failed
Original code used:
- `RA4 (CMP1B/AD1AN1)`
- `RB2 (CMP2B/AD2AN4)`
- `RB5 (CMP3B/AD2AN2)`

On EV68M17A + MCLV these correspond to op-amp/shunt-related nets (e.g. DIM:013, DIM:021, DIM:029 paths), not phase-voltage feedback (`M1_VA/VB/VC`).
Result: comparator never saw phase-voltage crossing behavior.

## 4) Current architecture (as of 2026-02-17)

### 4.1 Dual-mode ZC detection
BEMF sensing on:
- `RB9/AD2AN10` (VA)
- `RB8/AD1AN11` (VB)
- `RA10/AD2AN7` (VC)
- plus `VBUS` for threshold computation.

**Low speed (< 5000 eRPM)**: Software ZC in the 24kHz ADC ISR (PWM-triggered,
TRG1SRC=4). Threshold comparison, deadband, filter count, polarity check.
ADC ISR owns commutation timing via deadline/timeout framework.

**High speed (> 5000 eRPM)**: ADC digital comparator ZC via Phase F
(`FEATURE_ADC_CMP_ZC`). Dedicated channels AD1CH5 and AD2CH1 sample at 1 MHz
(SCCP3-triggered, TRG1SRC=14). Hardware comparator fires interrupt on threshold
crossing. SCCP1 timer handles blanking, commutation scheduling, and timeout.
ADC ISR continues running for threshold updates, pot, and vbus — but does NOT
drive commutation.

Crossover between modes is automatic with hysteresis. See Phase F plan for details.

### 4.2 Control architecture
- 24kHz ADC ISR: threshold computation, duty control, pot/vbus reading (always active).
- Software ZC path: commutation via deadline/timeout (active below crossover).
- Hardware ZC path: SCCP1 state machine + comparator ISRs (active above crossover).
- Analog comparator (CMP1/2/3): NOT used for ZC — BEMF pins cannot route to CMP
  inputs on the MCLV-48V-300W DIM. Used only for fault protection (CMP3 → PCI).

### 4.3 Staged enhancements (status as of 2026-02-17)
1. ~~Add ADC deadband around `VbusHalf` for noise immunity.~~ **DONE** — `HWZC_CMP_DEADBAND` (4 counts).
2. ~~Add ADC channel digital comparator for threshold events.~~ **DONE** — Phase F implemented.
   AD1CH5 (Phase B) and AD2CH1 (Phase A/C) with SCCP3 trigger at 1 MHz. Zero misses
   across 40K+ detections on Hurst motor.
3. ~~Add oversampling (`MODE=11`, `ACCNUM=00`, 4 samples) for BEMF channels.~~ **DONE** —
   Enabled on AD1CH5 and AD2CH1. TRG2SRC=2 (immediate re-trigger), ACCBRST=1.
   38,522 HW ZC detections with zero misses (x14 series). +6 dB noise immunity.
4. Evaluate `EIEN` only if ISR latency budget becomes tight.

## 5) Implementation cautions
- Confirm `ANSEL`/`TRIS` states for all remapped analog pins.
- Verify ADC trigger alignment point relative to PWM switching edges.
- Avoid sampling too close to switching transitions unless intentionally filtered.
- Validate with scope at TP15/TP16/TP17 and compare with ADC telemetry.
- Keep feature-flag rollback to forced/open-loop for safe bring-up.

## 6) Bring-up checklist (completed)
- [x] Confirm EV68M17A physically installed.
- [x] Confirm MCLV test points TP16/TP17/TP15 show phase-voltage feedback waveforms.
- [x] Update HAL ADC mapping to RB9/RB8/RA10 paths.
- [x] Software ZC via ADC threshold comparison (Phase 2, 24kHz ISR).
- [x] ADC digital comparator ZC at 1 MHz via SCCP3 (Phase F). Zero misses, 40K+ detections.
- [x] Analog comparator (CMP3) used for fault/protection only — not ZC.
- [x] Build with zero warnings and validate with diagnostic counters.
