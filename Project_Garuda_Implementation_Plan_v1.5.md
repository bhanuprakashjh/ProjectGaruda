# Project Garuda — Claude Code Implementation Plan v1.5
## dsPIC33AK Drone ESC Firmware Build Strategy

**Reference Template:** `mclv48v300w-33ak128mc106-pmsm-an1292-foc-pll` (branch 2.0.0)
**Target Device:** dsPIC33AK128MC106 (200 MHz, HW FPU, 40 MSPS ADC, HRPWM)
**Specification:** Project Garuda v1.3 — Clean-Room ESC Firmware

### Revision History
| Version | Changes |
|---|---|
| v1.0–1.3 | Initial plan, phase structure, directory layout |
| v1.4 | Addresses 10 review findings: Phase 0 HW bring-up, safety-first, bootloader geometry, BEMF windows, CLC spike, DShot non-interference, etc. |
| **v1.5** | **Addresses 7 remaining findings:** (1) Address conversion contract for byte/phantom/instruction, (2) ZC capture moved to hardware timer input — ISR relaxed, (3) RC filter purpose corrected + second stage added, (4) ADC-majority → comparator-edge auto-switch at high eRPM, (5) Zone B Vf compensation made adaptive, (6) Bootloader write-protection enforcement + power-fail model, (7) Bidir DShot PPS remap latency proof-point added |

---

## 1. Architecture Overview & Reference Repo Mapping

The AN1292 FOC-PLL reference project (`pmsm.X`) provides a complete MPLAB X project for the **same dsPIC33AK128MC106** device on the MCLV-48V-300W board. Its peripheral initialization and MCC-generated code serve as our template for all hardware abstraction. Project Garuda repurposes the same peripheral set but replaces the FOC control algorithm with 6-step trapezoidal/BEMF sensorless commutation optimized for drone ESCs.

### What We Reuse from the Reference Repo

| Reference Component | Garuda Usage | Notes |
|---|---|---|
| `mcc_generated_files/` — PWM (PG1, PG2, PG3) | Reuse PWM init, adapt for 6-step override control | Same 3 half-bridge PWM generators; change from center-aligned SVM to 6-step override mode |
| `mcc_generated_files/` — ADC (multi-core) | Reuse ADC init for BEMF sampling, Vbus, temperature | Reference uses ADC for current sensing; we add BEMF floating-phase sampling |
| `mcc_generated_files/` — UART | Reuse for GSP protocol & X2C-Scope debug | Same UART1 peripheral config |
| `mcc_generated_files/` — OP-AMP, CMP, DAC | Reuse comparator for hardware ZC detection | Reference uses op-amp for current amplification; we repurpose for BEMF conditioning |
| `mc1_user_params.h` | Replace with ESC-specific parameters | Motor poles, timing advance, DShot config instead of FOC PI gains |
| Clock configuration (200 MHz) | Direct reuse | Same oscillator + PLL config |
| GPIO / Port mapping | Adapt for ESC board pinout | Same device, different board layout |

### What We Build New (Clean-Room)

| New Module | Specification Section | Priority |
|---|---|---|
| 6-Step Commutation Engine | §3.1, §3.2 | Phase 1 |
| **Minimum Safety (OC, UV, signal loss)** | §6.1, Appendix B.3 | **Phase 0–1** |
| BEMF Zero-Crossing Detection | §3.2.3 | Phase 2 |
| DShot Protocol Decoder | §2.1 | Phase 3 |
| Bidirectional DShot + GCR Telemetry | §2.2 | Phase 3 |
| EDT Telemetry Scheduler | §2.3 | Phase 3 |
| State Machine (8-state) | §3.1.1-3.1.2 | Phase 1-2 |
| EEPROM/DEE Configuration System | §4.1 | Phase 4 |
| Garuda Serial Protocol (GSP) | §7.2 | Phase 4 |
| Bootloader (single-image + UART recovery) | §5 | Phase 4 |
| CLC-Accelerated Commutation | §3.3.6 | Phase 2 (feasibility spike in Phase 0) |
| Advanced Algorithms (Flux, Ternary, etc.) | §3.3 | Phase 5 |

---

## 2. Addressing Conventions & Flash Geometry

> **[FIX #2 from review]** — All addresses in this document and in bootloader protocol are expressed in **byte addresses** unless explicitly marked otherwise. The linker script, bootloader write commands, and PC tools all use byte addressing for consistency.

### dsPIC33AK128MC106 Flash Geometry

| Property | Value | Notes |
|---|---|---|
| Total Program Flash | 128 KB (131,072 bytes) | Byte-addressable via TBLRD/TBLWT |
| Erase granularity | **Page = 4096 bytes** (4 KB) | Minimum erase unit |
| Write granularity | **Row = 512 bytes** | Minimum write unit (latched row) |
| Instruction word | 24 bits packed in 32-bit phantom byte space | 3 instruction bytes + 1 phantom per word |
| TBLPAG/TBLRD | Byte-level access to program memory | Always use byte offsets in protocol |

### Program Memory Map (Byte Addresses)

```
0x00_0000 - 0x00_0FFF : Bootloader          (4 KB = 1 page, write-protected)
0x00_1000 - 0x01_DFFF : Application          (116 KB = 29 pages)
0x01_E000 - 0x01_EFFF : DEE / EEPROM area    (4 KB = 1 page, wear-leveled)
0x01_F000 - 0x01_FFFF : Reserved / metadata  (4 KB = 1 page)
```

> **[FIX #1]** Dual-image (A/B) is **removed** from the default plan. With 128 KB total and 8 KB reserved for boot + EEPROM + metadata, there isn't enough space for two full application images. The bootloader uses **single-image with robust CRC-32 validation + UART recovery** as the anti-brick strategy. If a future device provides ≥256 KB, A/B can be reconsidered.

### Bootloader Write Protocol — Aligned to Flash Geometry

| Command | Payload | Alignment |
|---|---|---|
| BL_ERASE | Start page address (4KB-aligned) | Erases one 4KB page at a time |
| BL_WRITE | 512-byte row: ADDR(4) + DATA(512) + CRC(2) | **Row-aligned (512B)**, not the previous 256B |
| BL_READ | 512-byte row read for verification | Row-aligned |
| BL_VERIFY | CRC-32 over entire application region | Byte-scanned |

### Address Conversion Contract [v1.5 FIX #1]

> dsPIC33AK program memory uses 24-bit instruction words packed into a 32-bit "phantom byte" address space (3 real bytes + 1 phantom byte per instruction word). Every layer of the system must agree on how addresses are expressed, or the bootloader will write correct bytes to wrong physical locations.

**The rule:** All addresses in this project — linker script, bootloader protocol, PC updater tool, and EEPROM metadata — use **byte addresses in phantom-byte space** (i.e., the addresses TBLRD/TBLWT use natively).

| Context | Address Unit | Example | Conversion |
|---|---|---|---|
| **Linker script** | Byte (phantom space) | `.text 0x001000` | Native — no conversion |
| **Bootloader BL_WRITE ADDR field** | Byte (phantom space) | `ADDR = 0x001000` for first app row | Native — no conversion |
| **PC updater tool → ESC** | Byte (phantom space) | Tool sends byte offsets | Native — no conversion |
| **Instruction word address** (PC register) | Word address = byte_addr / 2 | `PC = 0x000800` for byte 0x001000 | `word_addr = byte_addr >> 1` |
| **TBLPAG:TBLOFF** | TBLPAG = byte_addr[23:16], TBLOFF = byte_addr[15:0] | `TBLPAG=0x00, TBLOFF=0x1000` | Split byte address into page + offset |
| **Hex file (.hex) records** | Byte (phantom space) | Standard MPLAB output | Native — no conversion |

**Phantom byte rule:** Every 4th byte (offset 0x3, 0x7, 0xB, ...) in program memory is a phantom byte that reads as 0x00 and is not physically present. The bootloader **must include phantom bytes in row data** (512 bytes = 128 instruction words × 4 bytes each). The PC updater pads phantom positions with 0x00 when building row payloads from .hex files.

**Validation test (Phase 4):** After BL_WRITE, perform BL_READ of the same row and compare. Any mismatch indicates an addressing conversion error — halt and report.

---

## 3. Project Directory Structure for Claude Code

```
project-garuda/
├── CLAUDE.md                          # Claude Code project instructions
├── reference/                         # [FIX #10] AN1292 FOC-PLL repo clone
│   └── project/pmsm.X/               # Full reference project (read-only)
│       ├── mcc_generated_files/       # Peripheral init templates
│       └── ...
├── garuda.X/                          # MPLAB X project
│   ├── mcc_generated_files/           # Copied & adapted from reference
│   │   ├── system/                    # Clock, oscillator, pin config
│   │   ├── pwm/                       # PG1, PG2, PG3 init
│   │   ├── adc/                       # ADC core init (BEMF + Vbus + Temp)
│   │   ├── uart/                      # UART1 for GSP/debug
│   │   ├── cmp/                       # Comparator for ZC detection
│   │   ├── dac/                       # DAC for ZC threshold
│   │   ├── clc/                       # CLC for HW commutation (Phase 2)
│   │   ├── tmr/                       # Timers: commutation, system tick
│   │   └── dee/                       # Data EEPROM Emulation
│   ├── garuda/                        # Application firmware (clean-room)
│   │   ├── garuda_main.c/.h          # Main entry, state machine loop
│   │   ├── garuda_config.h           # Build-time configuration macros
│   │   ├── motor/                     # Motor control core
│   │   │   ├── commutation.c/.h      # 6-step commutation table & execution
│   │   │   ├── bemf.c/.h             # BEMF ZC detection (ADC + comparator)
│   │   │   ├── startup.c/.h          # ALIGN → OPEN_LOOP_RAMP sequence
│   │   │   ├── timing.c/.h           # Commutation timing, advance angle
│   │   │   ├── desync.c/.h           # Desync detection & recovery
│   │   │   └── pi_control.c/.h       # Throttle-to-duty PI controller
│   │   ├── protocol/                  # Communication protocols
│   │   │   ├── dshot.c/.h            # DShot150/300/600/1200 decoder
│   │   │   ├── dshot_bidir.c/.h      # Bidirectional DShot + GCR encoder
│   │   │   ├── edt.c/.h             # Extended DShot Telemetry scheduler
│   │   │   ├── dshot_commands.c/.h   # DShot command handler (§2.1.3)
│   │   │   └── gsp.c/.h             # Garuda Serial Protocol (§7.2)
│   │   ├── config/                    # Configuration management
│   │   │   ├── eeprom.c/.h          # EEPROM layout (§4.1) + DEE interface
│   │   │   ├── defaults.h           # Factory default values
│   │   │   └── calibration.c/.h     # Dead-time calibration (factory mode)
│   │   ├── protection/               # Safety & protection
│   │   │   ├── overcurrent.c/.h     # HW fault path + ISR logging
│   │   │   ├── thermal.c/.h         # Temperature monitoring & derating
│   │   │   ├── voltage.c/.h         # Under/over voltage detection
│   │   │   └── signal_loss.c/.h     # Input signal timeout failsafe
│   │   ├── telemetry/                # Telemetry system
│   │   │   ├── telemetry.c/.h       # Centralized telemetry data store
│   │   │   └── beep.c/.h            # Motor-as-speaker beep generation
│   │   └── hal/                       # Hardware Abstraction Layer
│   │       ├── hal_pwm.c/.h         # PWM override control for 6-step
│   │       ├── hal_adc.c/.h         # ADC channel routing for BEMF
│   │       ├── hal_clc.c/.h         # CLC truth table for commutation
│   │       ├── hal_timer.c/.h       # Commutation timer, system tick
│   │       ├── hal_comparator.c/.h  # ZC comparator control
│   │       └── hal_gpio.c/.h        # LED, button, misc I/O
│   ├── bootloader/                    # Separate bootloader project
│   │   ├── bl_main.c
│   │   ├── bl_protocol.c/.h         # BL commands (§7.4)
│   │   └── bl_flash.c/.h            # Flash erase/write (row-aligned)
│   └── tests/                         # Unit test stubs (PC-hosted)
│       ├── test_dshot_crc.c          # CRC test vectors from §2.2.3
│       ├── test_edt_framing.c
│       └── test_state_machine.c
├── docs/
│   └── Project_Garuda_Spec_v1.3.docx
└── tools/
    └── garuda_configurator/           # PC tool (Phase 5)
```

---

## 4. ISR Architecture — Hardware vs Firmware Response

> **[FIX #3]** — ISR timing budgets are split into **hardware response** (autonomous, no CPU involvement) and **firmware response** (ISR latency + processing).

### Hardware Autonomous Responses (No ISR Required)

| Event | Hardware Path | Response Time | Mechanism |
|---|---|---|---|
| **Overcurrent fault** | CMP output → PWM Fault input → all outputs disabled | **<100 ns** | PWM FLTDAT override, CPU not involved |
| **Commutation switching** | CLC sector register → PWM override | **<10 ns** | Combinatorial logic, no clock |

These are **not ISR budgets** — they happen regardless of CPU state. The PWM module's hardware fault path (FLTIENx + FLTDATx) autonomously tristates all outputs when the comparator trips. The ISR only logs the event and transitions the state machine.

### Firmware ISR Budgets

| Priority | Source | ISR Budget | ISR Purpose |
|---|---|---|---|
| 7 (highest) | PWM Fault (post-HW-trip) | <1 µs | Log fault, set FAULT state flag, **outputs already disabled by HW** |
| 6 | Comparator ZC (timer-captured) | **<2 µs (relaxed)** | Read hardware-captured timestamp from SCCPx timer latch, set ZC flag. **The ZC instant is captured in hardware — ISR latency does not affect timing accuracy** |
| 5 | Commutation Timer | <1 µs | Write 3-bit sector to CLC (or update PWM overrides in SW mode) |
| 4 | ADC Complete | <2 µs | Read BEMF sample, update majority filter, check Vbus/temp |
| 3 | DShot Input Capture | DMA-handled | DMA captures edge timestamps to buffer, **no ISR per bit** |
| 2 | DShot Frame Complete | <5 µs | Decode frame from DMA buffer, update throttle, queue telemetry |
| 1 | 1ms System Tick | <10 µs | State machine transitions, timeout checks, EDT scheduling |
| 0 | Background | N/A | EEPROM writes, diagnostics, idle loop |

### ZC Timestamp Architecture [v1.5 FIX #2]

> **The comparator ZC edge is captured by hardware, not by ISR entry timing.** The comparator output is routed to an SCCP timer capture input. When the ZC edge fires, the timer capture register latches the current timer value with **zero software jitter** — exactly like DShot edge capture. The ISR (Priority 6) then reads this pre-captured value at its leisure. Even if the ISR is delayed by 5 µs due to a Priority 7 handler or flash wait state, the timestamp is still cycle-accurate.
>
> **Implementation:** `CMP1_OUT → PPS → SCCPx ICx input → capture register` (same pattern as DShot input capture, using a different SCCP instance).
>
> This means the Priority 6 ISR budget is **relaxed to <2 µs** — it only reads a register and sets a flag. Commutation timing accuracy depends on the hardware capture path, not ISR latency.

### Non-Interference Constraint [FIX #9]

> **RULE: No commutation-critical ISR (Priority 5–7) may be blocked or delayed by DShot processing.** All DShot RX is DMA-assisted (Priority 3 captures edges without CPU). DShot TX (GCR telemetry response) uses DMA output compare — the 30 µs response window is handled entirely by DMA + timer, not by spinning in an ISR. Frame decoding (Priority 2) is deferred work that runs only when no higher-priority interrupt is pending.

---

## 5. BEMF Sampling Strategy — Concrete Specification

> **[FIX #4 + #5]** — Replaces the vague "sample at PWM center" with a concrete sampling window definition and analog front-end specification.

### 5.1 Analog Front-End Specification

| Parameter | Specification | Notes |
|---|---|---|
| **BEMF voltage divider** | 10:1 (e.g., 100kΩ / 10kΩ + 10kΩ) | Scales 0–30V motor phase to 0–3V ADC range |
| **Stage 1: Anti-ringing RC** | R₁ = 1kΩ, C₁ = 100pF → **fc₁ ≈ 1.6 MHz** | Snubs high-frequency ringing from FET switching edges (10–50 MHz). Does NOT significantly attenuate 24 kHz PWM — that is handled by blanking + digital filtering |
| **Stage 2: Noise-rejection RC** | R₂ = 4.7kΩ, C₂ = 1nF → **fc₂ ≈ 34 kHz** | Attenuates PWM switching harmonics (24 kHz × N). Passes BEMF content up to ~30 kHz (sufficient for 100k eRPM / 14-pole → ~14 kHz electrical) |
| **Combined group delay** | ~5 µs at BEMF frequencies (dominated by Stage 2) | Accounted for in commutation timing formula — see §5.4 |
| **ADC source impedance** | R₁ + R₂ = 5.7 kΩ total | Must be < SAR acquisition impedance limit. dsPIC33AK TAD spec: verify Rsource < 5kΩ recommendation. If marginal, reduce R₂ to 3.3kΩ (fc₂ → 48 kHz, delay → ~3.3 µs) |
| **Total divider output Z** | ~15 kΩ (divider ∥ filter) | Verify against ADC Rsource max in FRM |
| **Virtual neutral reference** | **NOT simply Vbus/2** — see calibration | DAC output, calibrated per operating point |

> **[v1.5 FIX #3]** — The v1.4 claim that a 1.6 MHz RC "attenuates 24 kHz PWM" was physically incorrect — a 1.6 MHz cutoff is >60× above the PWM fundamental and provides <0.01 dB attenuation there. The Stage 1 RC is now correctly described as an **anti-ringing snubber** for high-frequency FET switching transients. A **Stage 2 RC at ~34 kHz** is added to actually attenuate PWM harmonics before the ADC. The tradeoff is ~5 µs group delay, which is explicitly accounted for in the timing formula.

### 5.2 Virtual Neutral Calibration

> **[FIX #5]** — Vbus/2 is only a first approximation. The actual motor neutral point depends on duty cycle, diode conduction state, and motor inductance.

**Startup calibration (in IDLE state, motor not spinning):**
1. Apply 0% duty — all low-side switches ON — measure phase voltages → establish ground offset
2. Apply brief test pulses, measure all 3 phases simultaneously → derive neutral offset vs duty
3. Store DAC calibration table: `neutral_dac[duty_bucket]` indexed by duty cycle range

**Runtime correction:**
- During closed-loop operation, the non-floating phases' ADC readings provide a real-time neutral estimate: `V_neutral = (V_active_high + V_active_low) / 2`
- DAC is updated every commutation cycle from this estimate
- Comparator hysteresis: **±50 mV** (programmable via DAC step size) to prevent chatter

### 5.3 Sampling Window per PWM Cycle

Within each PWM cycle, there are multiple distinct windows. The BEMF sample must occur in the **correct** window.

```
One PWM cycle at 24 kHz (41.67 µs period, center-aligned):
 ___________                          ___________
|           |    PWM OFF (freewheel) |           |
|  PWM ON   |________________________|  PWM ON   |
|___________|                        |___________|
     ^                ^                    ^
     |                |                    |
  [Zone A]         [Zone B]            [Zone A]
  "ON window"     "OFF window"        "ON window"
```

**Zone A — PWM ON window (driven phase HIGH, low-side complementary):**
- Floating phase voltage = BEMF + diode drop artifacts
- **Usable for BEMF sampling IF blanking has elapsed**
- ADC trigger: **PGxTRIGA set to 75% of ON-time** (past initial switching transient)
- This is where the traditional "center sample" approach works

**Zone B — PWM OFF window (freewheeling through body diodes):**
- Floating phase sees motor BEMF minus diode drops
- **More noise-immune** but voltage is offset by diode Vf (~0.7V)
- Alternative sampling point for low-speed/high-noise conditions

### 5.4 Per-Step Sampling Configuration

| Commutation Step | Driven Phase | Floating Phase | BEMF ADC Channel | ZC Direction | Blanking (% of step) | Sample Zone |
|---|---|---|---|---|---|---|
| 0 | A=HIGH, B=LOW | C | AN2 | Rising | 15% | A |
| 1 | A=HIGH, C=LOW | B | AN1 | Falling | 15% | A |
| 2 | B=HIGH, C=LOW | A | AN0 | Rising | 15% | A |
| 3 | B=HIGH, A=LOW | C | AN2 | Falling | 15% | A |
| 4 | C=HIGH, A=LOW | B | AN1 | Rising | 15% | A |
| 5 | C=HIGH, B=LOW | A | AN0 | Falling | 15% | A |

**Blanking window:** After each commutation event, ignore comparator output and ADC samples for 15% of the expected commutation period. This masks switching noise and demagnetization transients.

**Default strategy:** Sample in Zone A at 75% of ON-time.
**Low-speed fallback (duty < 15%):** Switch to Zone B sampling with adaptive Vf compensation (see §5.6).

### 5.5 ZC Detection Mode Switch — ADC-Majority vs Comparator-Edge [v1.5 FIX #4]

> At high eRPM, the number of PWM cycles available within a 60° electrical sector shrinks. The ADC majority filter (3 consecutive same-polarity samples) requires at minimum 3 PWM cycles of usable samples after blanking. When this margin disappears, the system must automatically switch to comparator-edge timing as the primary ZC source.

**Mode selection rule (evaluated every commutation):**

```
pwm_cycles_per_sector = commutation_period / pwm_period
usable_cycles = pwm_cycles_per_sector × (1.0 - blanking_fraction)  // 85% usable

IF usable_cycles >= 5:
    mode = ADC_MAJORITY_PRIMARY    // ADC majority filter is primary, comparator validates
    // Robust: ≥5 samples to work with, majority of 3 is reliable
    
ELIF usable_cycles >= 2:
    mode = COMPARATOR_PRIMARY_ADC_VALIDATE  // Comparator edge is primary, ADC validates
    // Fewer samples: trust hardware comparator timing, use ADC to reject false edges
    
ELSE:  // usable_cycles < 2 (eRPM > ~80k at 24kHz PWM)
    mode = COMPARATOR_ONLY         // Pure hardware comparator, no ADC validation
    // At extreme speed, comparator capture is the only viable path
```

**Threshold examples at 24 kHz PWM (41.67 µs period):**

| eRPM (14-pole) | Commutation Period | PWM Cycles/Sector | Usable (85%) | Mode |
|---|---|---|---|---|
| 10,000 | 857 µs | 20.6 | 17.5 | ADC_MAJORITY |
| 30,000 | 286 µs | 6.9 | 5.8 | ADC_MAJORITY |
| 50,000 | 171 µs | 4.1 | 3.5 | COMPARATOR_PRIMARY |
| 80,000 | 107 µs | 2.6 | 2.2 | COMPARATOR_PRIMARY |
| 120,000 | 71 µs | 1.7 | 1.4 | COMPARATOR_ONLY |

**Implementation:** `bemf.c` evaluates this at each commutation and stores the current mode. The ZC ISR (Priority 6) and ADC ISR (Priority 4) both check this mode flag to decide their behavior.

### 5.6 Zone B Vf Compensation — Adaptive Estimation [v1.5 FIX #5]

> Zone B (PWM-OFF freewheel window) gives cleaner BEMF readings but is offset by body diode forward voltage drop (Vf). A fixed Vf compensation introduces systematic commutation phase error because Vf varies with current (30–50 mV/A) and temperature (~-2 mV/°C).

**Adaptive Vf estimation (avoids fixed compensation):**

```c
// Vf is estimated from operating conditions, not a fixed constant
// Method: measure the BEMF step discontinuity at each commutation edge

// At each commutation, the newly-floating phase transitions from 
// driven (Zone A voltage known) to freewheeling (Zone B voltage = BEMF - Vf).
// The difference between the last Zone A sample and first Zone B sample
// of the SAME phase includes Vf:
//   Vf_estimate = V_zone_a_last - V_zone_b_first - V_bemf_expected
// V_bemf_expected is estimated from speed (known from commutation period).

// Exponential moving average: Vf_filtered = 0.9 * Vf_filtered + 0.1 * Vf_estimate
// Updated every commutation cycle. Typical range: 0.3V–0.8V.
```

**Fallback if Vf estimation is unreliable (startup / very low speed):**
- Use lookup table: `Vf_lut[current_bucket][temp_bucket]` with 4×4 entries
- Populated from Phase 0 bench characterization with known motor + load
- Accuracy: ±100 mV (sufficient for low-speed fallback where timing is relaxed)

### 5.7 Commutation Timing Formula

**Timing advance accounting for filter delay:**
```
T_commutation = T_zc_captured + T_30deg - T_advance - T_filter_delay
Where:
  T_zc_captured = hardware timer capture value (from SCCP latch, zero jitter)
  T_30deg = commutation_period / 2  (half of 60° step)
  T_advance = configurable 0-30° (default 15°)
  T_filter_delay = Stage2_group_delay + ADC_conversion_time ≈ 5.5 µs
                   (5 µs RC Stage 2 + 0.5 µs ADC conversion)
```
> **Note:** The ~5.5 µs filter delay is constant and deterministic. At minimum commutation period (10 µs @ 166k eRPM), this represents ~55% of a step — which is significant. At that speed, the system has already switched to comparator-edge primary mode (see §5.5), where the Stage 2 RC delay applies to the comparator input, not to ADC sampling. The comparator has a different (faster) input path if needed: Stage 1 only (1.6 MHz cutoff, <1 µs delay) can be routed to comparator while Stage 1+2 feeds ADC.

---

## 6. CLC Feasibility Spike [FIX #6]

> The claim "CLC outputs drive PWM override directly with <10 ns latency" depends on whether the dsPIC33AK128MC106 can route CLC outputs into the PWM generator override control register bits, not just to GPIO pins. This must be verified before building the architecture around it.

### Phase 0 Feasibility Task: CLC → PWM Override Path

**Objective:** Prove or disprove that CLC can directly control PWM output states without CPU intervention per commutation event.

**Test procedure:**
1. Configure one CLC cell with a simple truth table (e.g., 2 inputs → 1 output)
2. Route CLC output to PG1 override control (if possible via PPS or direct peripheral interconnect)
3. Toggle CLC input, measure PWM output response on oscilloscope
4. If latency <100 ns → CLC architecture confirmed
5. If CLC cannot route to PWM override → **fallback to software commutation ISR** (Priority 5, <1 µs)

**Possible routing paths to investigate:**
- CLC output → PPS → PWM fault/override input
- CLC output → PWM PCI (Pulse Capture Input) → SWAP override
- CLC output → direct peripheral interconnect (device-specific)

**Decision gate:** If CLC→PWM is not feasible, remove §3.3.6 from the architecture. Software commutation at Priority 5 with <1 µs ISR is perfectly adequate for drone ESC eRPM ranges (typically <100k eRPM → commutation period >10 µs → ISR overhead <10%).

---

## 7. Dead-Time Auto-Calibration — Safety-Constrained [FIX #8]

> The original plan proposed ramping duty while watching for current spikes. This is **dangerous** on a drone ESC — shoot-through during calibration can destroy FETs in microseconds.

### Revised Calibration Approach

**Mode:** Factory/bench-only calibration. **NOT run automatically at first power-up in the field.**

**Entry:** Explicit GSP command `CALIBRATE_DEADTIME` (0x0A) with safety interlock:
- Motor must be disconnected (no load) OR on test fixture with current-limited supply
- Throttle must be 0 (ARMED state, not running)
- User must confirm via second command within 2 seconds

**Procedure (with hard safety clamps):**
1. Set duty cycle hard limit: **MAX 3%** during entire calibration
2. Set DC bus current trip threshold: **500 mA** (well below FET destruction threshold)
3. Start with dead time = 500 ns (conservative)
4. Apply test pattern: alternate commutation steps at fixed 1 kHz rate
5. Gradually **decrease** dead time by 25 ns steps (safer direction: start safe, find minimum)
6. At each step, sample bus current over 100 PWM cycles
7. If current_peak > 300 mA at 3% duty → shoot-through starting → **stop immediately**
8. Optimal dead time = last safe value + 100 ns margin
9. Store to EEPROM, mark as calibrated

**Hard safety clamps (non-bypassable):**
- Hardware overcurrent comparator remains active during calibration
- Calibration aborts on ANY fault
- Maximum calibration duration: 5 seconds, then auto-abort
- Duty cycle register is hardware-limited (PGxDC clamp) during calibration mode

**Pre-calibration defaults (used when uncalibrated):**
- Dead time: **300 ns** (safe for most MOSFETs)
- Max duty limit: **90%**
- EDT status flag: UNCALIBRATED warning

---

## 8. CLAUDE.md — Updated for Claude Code

```markdown
# Project Garuda - Claude Code Instructions

## Project Overview
Drone ESC firmware for dsPIC33AK128MC106 (200MHz, HW FPU, 40MSPS ADC).
6-step trapezoidal BLDC control with DShot protocol, bidirectional telemetry, EDT.

## Reference Code
The `reference/` directory contains the Microchip AN1292 FOC-PLL project
(branch 2.0.0) for the SAME dsPIC33AK128MC106 device. Use its MCC-generated
peripheral init code as the canonical template for:
- PWM generator setup (PG1/PG2/PG3) — registers, clock dividers, dead time
- ADC configuration — core assignment, trigger sources, channel mapping
- UART1 — baud rate, pin mapping
- Comparator (CMP) and DAC — for BEMF zero-crossing detection
- CLC — configurable logic cell setup
- System clock — 200MHz PLL configuration
- Pin Manager — I/O port assignments

## CRITICAL: Clean-Room Rules
- NEVER reference, copy, or derive from BLHeli, AM32, Bluejay, or KISS source
- ALL motor control algorithms must be original, based ONLY on:
  - Microchip datasheets (dsPIC33AK FRM) and app notes (AN1160, AN1292)
  - Public protocol specs (DShot from Betaflight docs, EDT from public spec)
  - Motor control textbook theory (6-step, BEMF ZC detection)
  - The Garuda specification document (docs/Project_Garuda_Spec_v1.3.docx)

## Addressing Convention
ALL memory addresses in this project are BYTE ADDRESSES.
- Linker script: byte addresses
- Bootloader protocol: byte addresses for page/row operations
- Flash geometry: Page = 4096 bytes (erase), Row = 512 bytes (write)
- NEVER mix instruction-word addresses and byte addresses

## Build System
- MPLAB X IDE v6.25 with XC-DSC compiler v3.20+
- Device: dsPIC33AK128MC106
- DFP: dsPIC33AK-MC_DFP v1.1.109
- Project file: garuda.X/

## Code Style
- C11, no dynamic allocation (bare-metal real-time)
- All ISR handlers must meet budgets (see plan §4)
- Use stdint.h types (uint8_t, uint16_t, etc.)
- Prefix all globals with module name: dshot_frame, bemf_zc_count, etc.
- Hardware registers via MCC macros where possible
- Comments cite spec section: /* §3.2.1 Step 1: A=HIGH, B=LOW, C=Float */

## Safety-First Design Rules
1. Hardware overcurrent fault path MUST be configured before ANY PWM output
2. Undervoltage cutoff MUST be active before motor can spin
3. Signal loss timeout (100ms) MUST be configured before ARMED state
4. Open-loop ramp MUST have duty clamp (configurable, default 50%)
5. Dead-time default is 300ns until explicit calibration

## ISR Architecture (Hardware vs Firmware)
- Overcurrent: HARDWARE path (CMP → PWM fault → outputs disabled, <100ns, no CPU)
- Overcurrent ISR (Priority 7): logging/state only, outputs ALREADY disabled
- ZC detection: HARDWARE capture (CMP out → SCCP timer capture latch, zero jitter)
- ZC ISR (Priority 6): reads captured timestamp register, budget <2µs (relaxed)
- Commutation: CLC hardware path if feasible, else software ISR at Priority 5
- DShot RX: DMA captures edges, NO per-bit ISR
- DShot TX (GCR): DMA + timer output compare, NO CPU spinning

## ZC Detection Modes
- Below ~30k eRPM: ADC majority filter primary, comparator validates
- 30k-80k eRPM: Comparator primary, ADC validates
- Above ~80k eRPM: Comparator only (insufficient PWM cycles for ADC majority)
- Mode switch is automatic based on PWM-cycles-per-sector

## Analog Front-End
- Stage 1 RC (1kΩ/100pF, 1.6MHz): anti-ringing snubber for FET edges
- Stage 2 RC (4.7kΩ/1nF, 34kHz): PWM harmonic attenuation for ADC
- Comparator can be fed from Stage 1 only (fast path, <1µs delay)
- ADC uses Stage 1+2 (clean signal, ~5µs delay, accounted in timing formula)

## Non-Interference Rule
Commutation ISRs (Priority 5-7) must NEVER be blocked by DShot processing.
All DShot decode/encode is DMA + deferred work at Priority 2 or lower.

## BEMF Sampling
- Sample floating phase at 75% of PWM ON-time (Zone A)
- Blanking: 15% of commutation period after each step change
- Virtual neutral: calibrated DAC, NOT simply Vbus/2
- Majority filter: 3 consecutive same-polarity samples = valid ZC
- Auto-switch to comparator-primary when PWM cycles/sector < 5
- Low-speed fallback: Zone B (PWM OFF window) with adaptive Vf estimation
- Analog: Stage 1 (snubber) + Stage 2 (LPF at 34kHz) — ~5µs total delay
```

---

## 9. Phase-by-Phase Implementation Plan

### Phase 0: Hardware Bring-Up & Safety Foundation (Week 0-1) [FIX #7, NEW]

**Goal:** Board validated, protection active, CLC path proven or ruled out.

#### Task 0.1: Board-Level Validation
```
Claude Code prompt:
"Using the reference project's system init as template, bring up the
dsPIC33AK128MC106:
- 200MHz clock configuration (copy from reference)
- GPIO: toggle an LED to prove boot
- UART: printf 'Garuda alive' at 115200 baud
- ADC: read DC bus voltage, print to UART
- Verify gate driver polarity (active high/low) with scope
- Verify PWM dead-time polarity matches gate driver requirements"
```

**Scope measurements required:**
- Gate driver HIGH/LOW output vs PWM pin state (confirm polarity)
- Dead-time between high-side OFF and low-side ON (confirm no shoot-through at 300ns)
- DC bus voltage ADC reading vs multimeter (confirm divider ratio)
- Comparator fault pin behavior (confirm active level)

#### Task 0.2: Minimum Safety — Hardware Overcurrent Path
```
Claude Code prompt:
"Configure the hardware overcurrent protection path BEFORE any PWM outputs:
1. Set up DAC to output overcurrent threshold voltage
2. Configure comparator: phase current sense → CMP input, DAC → CMP reference
3. Route CMP output directly to PWM Fault input (FLTIENx)
4. Configure PWM FLTDAT registers: all outputs → low-impedance safe state
5. Test: inject a voltage on CMP input above threshold, verify PWM outputs
   are disabled within <100ns on oscilloscope
This is HARDWARE autonomous — no ISR needed for protection.
Add ISR at Priority 7 for fault logging only."
```

#### Task 0.3: Minimum Safety — Voltage & Signal Loss
```
Claude Code prompt:
"Implement minimum voltage protection and signal loss:
- voltage.c: Read Vbus via ADC in 1ms tick. If < VBUS_MIN (default 10V)
  for 10 consecutive readings → disable all PWM, enter FAULT
- signal_loss.c: Timer-based watchdog. Reset on valid input signal.
  If 100ms elapses without valid input → transition to BRAKING/IDLE
- Duty clamp: hal_pwm.c enforces MAX_DUTY (default 50% during Phase 0-1)
  via register clamp, not just software check"
```

#### Task 0.4: CLC Feasibility Spike [FIX #6]
```
Claude Code prompt:
"Prove or disprove CLC → PWM override routing on dsPIC33AK128MC106:
1. Configure one CLC cell with 2-input AND gate truth table
2. Attempt to route CLC output to PG1 override control via PPS or
   direct peripheral interconnect
3. Toggle CLC input (use GPIO or another timer as stimulus)
4. Measure PG1 output on oscilloscope
5. Report: can CLC control PWM override? What is measured latency?
If NOT feasible: document findings, we use software ISR at Priority 5."
```

**Phase 0 Deliverable:**
- Board boots, UART prints, ADC reads correctly
- Hardware overcurrent trips verified on scope (<100ns)
- Undervoltage and signal-loss protection functional
- CLC→PWM feasibility: confirmed or ruled out with evidence

---

### Phase 1: Foundation (Weeks 2-5)
**Goal:** Motor spins open-loop with all safety active

> **[FIX #7]** — All Phase 1 motor spinning happens with Phase 0 safety already active: hardware OC trip, undervoltage cutoff, signal loss timeout, and duty clamps.

#### Task 1.1: Project Scaffolding
```
Claude Code prompt:
"Create the complete project directory structure per the plan.
Copy MCC peripheral init from reference/project/pmsm.X/mcc_generated_files/
into garuda.X/mcc_generated_files/. Adapt for ESC board pinout.
Initialize all .c/.h files with proper headers, guards, and spec citations.
Ensure Phase 0 safety code is integrated into the init sequence."
```

#### Task 1.2: PWM Setup for 6-Step Control
```
Claude Code prompt:
"Adapt reference PWM init for 6-step trapezoidal control:
- PWM OVERRIDE mode (PGxIOCONH.OVRENx) for direct gate control
- Each step: one phase = PWM modulated, one = always LOW, one = floating
- Dead time: 300ns default (from Phase 0 validation)
- PWM frequency: 24 kHz default
- ADC trigger at 75% of ON-time (PGxTRIGA) for BEMF sampling
- DUTY CLAMP: PGxDC cannot exceed MAX_DUTY register value
Build commutation table per §3.2.1.
hal_pwm_set_step(uint8_t step, uint16_t duty) with safety checks."
```

#### Task 1.3: Open-Loop Motor Spinning
```
Claude Code prompt:
"Implement ALIGN and OPEN_LOOP_RAMP states:
- ALIGN: fixed commutation state, configurable 50-500ms
- OPEN_LOOP_RAMP: step through commutation at increasing frequency
  Starting frequency: ~500 eRPM, ramp rate: configurable
- Duty clamp active: max 50% during ramp (prevent overcurrent on stall)
- Timer ISR (Priority 5) drives commutation stepping
- If hardware overcurrent trips during ramp → FAULT state
- If no back-EMF detected after ramp timeout → FAULT state
Motor spins with throttle from potentiometer (ADC)."
```

#### Task 1.4: State Machine (Partial)
```
Claude Code prompt:
"Implement states: POWER_ON_RESET, IDLE, ARMED, ALIGN, OPEN_LOOP_RAMP, FAULT.
Transition table per §3.1.2 (subset for Phase 1).
Run in 1ms system tick. Protection checks in every state:
- OC: hardware + ISR logging
- UV: ADC check every tick
- Signal loss: timer watchdog
- Duty clamp: enforced in hal_pwm_set_step()
CLOSED_LOOP and BRAKING are stubs for now."
```

**Phase 1 Deliverable:** Motor spins open-loop, all safety protection active and verified.

---

### Phase 2: Sensorless Closed-Loop (Weeks 6-9)
**Goal:** Reliable BEMF-based commutation on 3+ motor types

#### Task 2.1: ADC BEMF Sampling (Per Sampling Spec §5)
```
Claude Code prompt:
"Configure ADC for BEMF sampling per the plan's BEMF specification:
- 3x BEMF channels (AN0-AN2), only floating phase sampled per step
- ADC trigger from PWM at 75% of ON-time (Zone A sampling)
- Also sample Vbus (AN3) and Temperature (AN5) in ADC scan
- hal_adc.c routes to correct channel based on commutation step
- Implement blanking: ignore samples for first 15% of commutation period
Reference the per-step sampling table (Step 0: floating=C/AN2, etc.)"
```

#### Task 2.2: Zero-Crossing Detection (With Calibrated Neutral)
```
Claude Code prompt:
"Implement bemf.c with calibrated virtual neutral per plan §5.2:
- At startup (IDLE state): run neutral calibration sequence
- During operation: DAC tracks (V_active_high + V_active_low) / 2
- Majority filter: 3 consecutive same-polarity samples = valid ZC
- Hardware comparator as primary ZC detector (fast path)
- ADC integration as validation (reject noise-triggered ZC events)
- Track ZC timestamp via timer capture for timing engine
- Comparator hysteresis: ±50mV to prevent chatter"
```

#### Task 2.3: Commutation Timing with Filter Delay Compensation
```
Claude Code prompt:
"Implement timing.c per plan §5.4 timing formula:
T_commutation = T_zc + T_30deg - T_advance - T_filter_delay
Where T_filter_delay ≈ 1.5µs (RC + ADC conversion)
- Track commutation period over 6-step cycle for speed estimate
- Blanking window: 15% of period after each commutation
- Timing advance: 0-30° configurable (default 15°)
- Min commutation period: 10µs (hardware limit, ~166k eRPM)
- Max commutation period: 65ms (below → transition to stop)"
```

#### Task 2.4: Desync Detection & Recovery
```
Claude Code prompt:
"Implement desync.c per §3.2.4:
- Period variance > 25% over 6 steps → warning
- No ZC within 150% expected time → force commutation
- 3+ consecutive forced commutations → desync confirmed
- Recovery: reduce duty 50%, attempt resync, 3 failures → FAULT
- Stress metric: forced_commutations / total ratio for EDT"
```

#### Task 2.5: CLC Commutation (If Feasible) or Software Fallback
```
Claude Code prompt:
"Based on Phase 0 CLC feasibility results:
IF CLC→PWM proven: implement hal_clc.c with §3.3.6 truth table.
  CPU writes 3-bit sector register, CLC handles all 6 gate outputs.
IF CLC NOT feasible: implement software commutation in ISR Priority 5.
  commutation_execute_step() writes PWM override registers directly.
  Budget: <1µs per commutation event. Document decision in CLAUDE.md."
```

#### Task 2.6: Complete State Machine
```
"Add CLOSED_LOOP and BRAKING states. Full 8-state machine with all
transitions per §3.1.2. BRAKING: active regenerative braking or
coast-down based on config flag."
```

**Phase 2 Deliverable:** Closed-loop sensorless on 3+ motor types with protection active.

---

### Phase 3: DShot Protocol (Weeks 10-13)
**Goal:** Full DShot600 + EDT compatibility with Betaflight

> **[FIX #9 enforced]** — All DShot implementation uses DMA for edge capture and GCR transmission. No DShot processing may delay commutation ISRs.

#### Task 3.1: DShot Receiver (DMA-Assisted)
```
Claude Code prompt:
"Implement dshot.c for DShot150/300/600/1200:
- Input Capture peripheral captures rising+falling edges
- DMA transfers edge timestamps to circular buffer (NO per-bit ISR)
- On frame-complete (DMA transfer count = 16 bits): trigger Priority 2 ISR
- ISR decodes pulse widths from timestamp differences
- CRC: (value ^ (value >> 4) ^ (value >> 8)) & 0x0F
- Auto-detect rate from first frame's bit period
CRITICAL: DMA handles all bit-level capture. CPU only sees complete frames."
```

#### Task 3.2: DShot Commands
```
"Implement dshot_commands.c per §2.1.3. Commands 0-14 with repeat
requirements. Throttle 48-2047. Repeat counter with 100ms timeout."
```

#### Task 3.3: Bidirectional DShot + GCR Telemetry
```
Claude Code prompt:
"Implement dshot_bidir.c per §2.2:
- Detect bidir mode by idle HIGH polarity
- Invert received bits before CRC validation (§2.2.2)
- After frame RX: switch pin to output via PPS remapping
- Encode eRPM as GCR 5-bit encoding
- DMA + Output Compare transmits GCR bitstream (NO CPU bit-banging)
- Response starts 30µs ±5µs after FC frame end (use timer)
CRITICAL: GCR transmission is DMA-driven, not CPU-driven.
CRITICAL: PPS remap + GPIO direction change must be measured — see proof-point below."
```

**Bidir DShot PPS Remap Proof-Point [v1.5 FIX #7]:**

> The pin direction change (input → output) and PPS remap from Input Capture to Output Compare is a hidden timing risk. If this takes too long or introduces jitter, it can violate the 30 µs ±5 µs response window or — worse — block a commutation ISR.

**Measurement required (Phase 3 gate):**
1. Set a GPIO test pin HIGH at the start of the DShot frame-complete ISR (Priority 2)
2. Inside that ISR, perform the PPS remap + pin direction change
3. Set the test pin LOW after remap is complete
4. Measure the pulse width on oscilloscope → this is the remap latency
5. Simultaneously, verify no commutation ISR (Priority 5) stacking delay during remap

**Acceptance criteria:**
- PPS remap latency: **< 2 µs** (leaves >28 µs for timer setup + DMA start before 30 µs deadline)
- Commutation ISR jitter with bidir DShot active: **< 500 ns** increase vs DShot-disabled baseline
- Both measured on scope, captured as Phase 3 evidence

**If remap latency exceeds 5 µs:** Consider alternative — dedicate separate pins for DShot RX and TX (many ESC boards have this option), eliminating the PPS remap entirely.

#### Task 3.4: EDT Telemetry
```
"Implement edt.c per §2.3. Frame types, scheduling at 1000ms base
interval with staggered offsets, rate cap, immediate status on error."
```

**Phase 3 Deliverable:** DShot600 + bidir + EDT working with Betaflight FC.

---

### Phase 4: Production Features (Weeks 14-17)
**Goal:** Production-ready firmware

#### Task 4.1: EEPROM Configuration
```
"Implement eeprom.c per §4.1 using DEE library from MCC.
64-byte config with CRC-16-CCITT. Row-aligned flash writes.
Wear leveling in 4KB reserved area."
```

#### Task 4.2: Garuda Serial Protocol
```
"Implement gsp.c per §7.2. Packet structure, all commands.
Half-duplex UART for 1-wire mode."
```

#### Task 4.3: Full Protection Suite
```
"Complete overcurrent.c (logging, configurable threshold),
thermal.c (NTC curve, derating, shutdown), voltage.c (over+under).
All passing Appendix B.3 test criteria."
```

#### Task 4.4: Dead-Time Calibration (Factory Mode) [FIX #8]
```
"Implement calibration.c per plan §7 (safety-constrained):
- Factory/bench-only mode, explicit GSP command with interlock
- Decrease dead-time from 500ns in 25ns steps (safe direction)
- Hard clamps: 3% max duty, 500mA current trip, 5s max duration
- Hardware OC comparator remains active throughout
- Store optimal + 100ns margin to EEPROM"
```

#### Task 4.5: Bootloader (Single-Image + UART Recovery)
```
Claude Code prompt:
"Implement bootloader per plan §2 addressing conventions:
- 4KB at 0x000000-0x000FFF (1 flash page, write-protected)
- Commands: BL_PING, BL_GET_INFO, BL_ERASE (page-aligned 4KB),
  BL_WRITE (row-aligned 512B), BL_READ, BL_VERIFY, BL_BOOT
- Single-image architecture (NO A/B partitioning)
- CRC-32 validation before boot
- Anti-brick: UART always accessible, bootloader cannot be overwritten
- Entry: DShot CMD 255 x10, UART LOW at power-up, or GSP command
- On boot failure: 500ms watchdog → return to bootloader
CRITICAL: All addresses are BYTE addresses. Write unit = 512B row."
```

### Bootloader Protection & Power-Fail Model [v1.5 FIX #6]

**Bootloader page write-protection enforcement:**

| Mechanism | Implementation | Notes |
|---|---|---|
| **Code Guard / Boot Segment** | dsPIC33AK `FSEC` config register: `BSS = HIGH` (Boot Segment protected) | Hardware-enforced — application code physically cannot erase/write pages 0x000000–0x000FFF |
| **Linker script enforcement** | Bootloader linked to `0x000000–0x000FFF`; app linked to `0x001000+` with `ASSERT` that no app symbol falls in boot region | Build-time verification |
| **Runtime guard in bl_flash.c** | `if (addr < APP_START_ADDR) return BL_ERR_PROTECTED;` | Defense-in-depth even if config bits are wrong |

**Power-fail model during firmware update:**

The critical failure window is during BL_ERASE and BL_WRITE. If power is lost:

| Failure Point | State After Power Restore | Recovery |
|---|---|---|
| During BL_ERASE (page being erased) | Page contents undefined (partially erased) | CRC-32 check fails at boot → stays in bootloader → user re-flashes |
| During BL_WRITE (row being written) | Row contents undefined; rest of image intact or erased | Same: CRC-32 fails → bootloader |
| After BL_WRITE, before BL_VERIFY | Image written but unverified | CRC-32 check at boot — if passes, boots normally; if fails, stays in bootloader |
| After BL_VERIFY, before BL_BOOT | Image verified, not yet booted | Bootloader re-verifies CRC-32 at next boot — boots normally |

**Boot sequence (executed by bootloader every power-on):**
1. Check if UART entry condition (pin LOW at startup) → if yes, stay in bootloader
2. Validate application CRC-32: read entire app region, compute CRC, compare to stored CRC in metadata page
3. If CRC valid → jump to application
4. If CRC invalid → stay in bootloader, blink LED error pattern, wait for UART connection
5. Watchdog timeout: if application jumps to invalid code and hangs, 500ms WDT resets to bootloader

**Key invariant:** The bootloader itself is never modified during firmware update. It resides in a hardware-protected page. The UART entry path is always available regardless of application state. This makes the device **unbrickable** short of hardware damage.

**Phase 4 Deliverable:** Production-ready, all Appendix B tests pass.

---

### Phase 5: Advanced Features (Weeks 18-21)
**Goal:** Performance optimization

#### Tasks (unchanged from v1.3):
- **5.1:** Flux Linkage Integration (§3.3.1) — sensorless to 2% throttle
- **5.2:** Ternary Hysteresis Control (§3.3.3) — 40-60% torque ripple reduction
- **5.3:** Adaptive Lead Angle (§3.3.4) — dynamic timing advance
- **5.4:** Mixed Decay Mode (§3.3.5) — acoustic optimization
- **5.5:** MSP Passthrough (§7.3) — Betaflight FC passthrough compatibility

---

## 10. Key Peripheral Register Patterns from Reference

*(Unchanged from v1.3 — PWM override, ADC trigger, comparator patterns)*

### PWM Override for 6-Step (adapted from AN1292 reference)
```c
// PGxIOCONH — OUTPUT OVERRIDE CONTROL (critical for 6-step)
//   OVRENH = 1: Override high-side output
//   OVRENL = 1: Override low-side output
//   OVRDAT<1:0>: Override values (HIGH/LOW/Float)
//
// SAFETY: PGxFLTDAT configured in Phase 0 for hardware fault path
//   FLTIENx = 1: Fault input enabled from comparator
//   FLTDAT = safe state (all outputs low/tristate)
//
// DUTY CLAMP: PGxDC limited by software before write
//   if (duty > max_duty) duty = max_duty;  // enforced in HAL
```

---

## 11. DShot CRC Test Vectors — Gate Check

Before any DShot work ships, verify against §2.2.3:
```
Throttle=0,    Telem=0 → CRC=0x0, Frame=0x0000
Throttle=48,   Telem=0 → CRC=0x3, Frame=0x0603
Throttle=1047, Telem=0 → CRC=0x7, Frame=0x82F7
Throttle=2047, Telem=0 → CRC=0xF, Frame=0xFFEF
Throttle=1047, Telem=1 → CRC=0x6, Frame=0x82E6
```

---

## 12. Verification Checklist per Phase

### Phase 0 ✓ Criteria
- [ ] dsPIC33AK boots at 200MHz, LED blinks, UART prints
- [ ] ADC reads Vbus within 2% of multimeter reading
- [ ] Gate driver polarity confirmed on scope
- [ ] **Hardware overcurrent trip verified <100ns on scope**
- [ ] Undervoltage cutoff functional (Vbus < threshold → outputs disabled)
- [ ] Signal loss timer functional (simulated disconnect → disarm)
- [ ] CLC→PWM feasibility: confirmed with latency measurement OR ruled out with rationale

### Phase 1 ✓ Criteria
- [ ] PWM outputs toggling at 24kHz on all 3 phases
- [ ] 6-step commutation sequence verified on oscilloscope
- [ ] Dead time = 300ns verified on scope (no shoot-through)
- [ ] Motor spins open-loop with duty clamp active
- [ ] **All Phase 0 safety protections remain active during motor operation**

### Phase 2 ✓ Criteria
- [ ] ADC sampling BEMF in Zone A at 75% ON-time (verified on scope)
- [ ] Virtual neutral calibration runs at startup
- [ ] **ZC timestamp captured by hardware timer (SCCP latch), not ISR entry** — verified by comparing capture value stability with/without ISR load
- [ ] Zero-crossing detection working (LED toggle on ZC for debug)
- [ ] Closed-loop commutation stable at multiple speeds
- [ ] **ZC mode-switch verified:** ADC_MAJORITY below 30k eRPM, COMPARATOR_PRIMARY above 50k eRPM (measured on scope with speed sweep)
- [ ] Desync detection and recovery functional
- [ ] 3+ different motors tested successfully
- [ ] CLC commutation active (if feasible) or software fallback documented

### Phase 3 ✓ Criteria
- [ ] DShot CRC test vectors pass (all 5 from §2.2.3)
- [ ] DShot600 decode from Betaflight FC — verified DMA-only bit capture
- [ ] Bidirectional DShot eRPM telemetry verified on FC
- [ ] **Bidir PPS remap latency measured < 2 µs** (scope measurement)
- [ ] EDT frames decoded correctly on FC side
- [ ] All DShot commands functional
- [ ] **Commutation ISR jitter < 500 ns increase with DShot active** (scope measurement, compared to DShot-disabled baseline)

### Phase 4 ✓ Criteria
- [ ] EEPROM read/write with CRC validation, row-aligned writes
- [ ] GSP protocol: all commands functional from PC tool
- [ ] Overcurrent fault trips in <100ns (hardware) + ISR logs within <1µs
- [ ] Signal loss failsafe stops motor in <500ms
- [ ] **Bootloader page write-protected:** application erase of page 0 fails gracefully
- [ ] **Bootloader BL_WRITE → BL_READ round-trip matches** (address conversion validated)
- [ ] **Power-fail test:** interrupt power during BL_WRITE, verify bootloader recovers on next boot
- [ ] Bootloader: firmware update via UART (512B row writes)
- [ ] Dead time calibration runs safely with all clamps active

### Phase 5 ✓ Criteria
- [ ] Sensorless operation down to 2% throttle (flux linkage)
- [ ] Measurable torque ripple reduction (ternary hysteresis)
- [ ] Adaptive lead angle improves efficiency across speed range
- [ ] Acoustic noise reduced at low speeds (mixed decay)

---

## 13. Risk Mitigations & Contingencies (Updated)

| Risk | Mitigation | Contingency |
|---|---|---|
| CLC cannot route to PWM override | **Phase 0 feasibility spike proves/disproves** | Software commutation ISR (<1µs) is fully adequate |
| Shoot-through during dead-time calibration | **Factory-only mode with hard clamps (3% duty, 500mA trip)** | Use conservative 300ns default, skip auto-calibration |
| BEMF noise at low speed / wrong neutral | **Calibrated DAC + majority filter + adaptive Vf + Zone B fallback** | Higher minimum throttle; add flux integration in Phase 5 |
| Bootloader bricking device | **Single-image + HW write-protect + UART always accessible + CRC + power-fail model** | ICSP/PICkit recovery as last resort |
| DShot timing interferes with commutation | **DMA-only capture, non-interference rule, PPS remap latency measured** | Start with DShot300; use separate RX/TX pins if remap too slow |
| Flash write geometry mismatch | **Row-aligned 512B writes, page-aligned 4KB erases, phantom-byte contract** | BL_WRITE→BL_READ round-trip validated in Phase 4 |
| ADC majority filter fails at high eRPM | **Auto-switch to COMPARATOR_PRIMARY when PWM cycles/sector < 5** | Comparator-only mode at extreme speed |
| ZC ISR jitter from interrupt stacking | **ZC captured by hardware timer latch, ISR reads captured value** | ISR latency does not affect timing accuracy |
| Bidir DShot PPS remap blocks commutation | **Measured proof-point: remap < 2 µs, jitter < 500 ns** | Separate RX/TX pins eliminate remap entirely |
| RC filter delay too large at high eRPM | **Comparator fed from Stage 1 only (1.6 MHz, <1 µs delay)** | ADC uses Stage 1+2 for noise immunity; comparator path is faster |
| XC-DSC compiler unavailable in Claude Code | Write portable C, validate registers from datasheet | Test compilation on developer workstation |

---

## Appendix A: Changes from v1.3 → v1.4 Summary

| # | Review Finding | Fix Applied |
|---|---|---|
| 1 | Bootloader A/B doesn't fit in 128KB | Removed A/B, single-image + UART recovery |
| 2 | Byte vs instruction-word addressing confusion | Added §2 Addressing Conventions, all byte addresses |
| 3 | ISR budgets not physically enforceable | Split into Hardware Response (<100ns) vs Firmware ISR budget |
| 4 | BEMF "sample at center" too vague | Added §5 with Zone A/B, blanking, per-step table |
| 5 | Vbus/2 virtual neutral is approximate | Added calibration procedure, runtime correction, hysteresis spec |
| 6 | CLC→PWM routing unverified | Added Phase 0 feasibility spike with explicit decision gate |
| 7 | Protection comes too late | Added Phase 0 with HW overcurrent + UV + signal loss before any motor spinning |
| 8 | Dead-time calibration dangerous | Factory-only mode with hard clamps (3% duty, 500mA trip, 5s timeout) |
| 9 | DShot may interfere with commutation | Added Non-Interference Rule, DMA-only design, verified in Phase 3 checklist |
| 10 | `reference/` directory missing from tree | Added to directory structure |

## Appendix B: Changes from v1.4 → v1.5 Summary

| # | Review Finding | Fix Applied |
|---|---|---|
| 1 | "Byte addressing everywhere" needs conversion contract | Added Address Conversion Contract: phantom byte rule, TBLPAG/TBLOFF mapping, hex file convention, BL_WRITE→BL_READ validation test |
| 2 | Comparator ZC ISR <500ns is fragile | ZC edge now captured by hardware SCCP timer latch. ISR reads captured register — budget relaxed to <2µs. Timing accuracy depends on hardware, not ISR latency |
| 3 | RC filter fc=1.6MHz doesn't attenuate 24kHz PWM | Renamed Stage 1 as anti-ringing snubber. Added Stage 2 (R=4.7kΩ, C=1nF, fc≈34kHz) for PWM harmonic attenuation. Group delay updated to ~5µs and reflected in timing formula. Comparator gets Stage 1 only (fast path) |
| 4 | "3 consecutive samples" majority filter fails at high eRPM | Added §5.5 ZC Detection Mode Switch: auto-selects ADC_MAJORITY / COMPARATOR_PRIMARY / COMPARATOR_ONLY based on PWM-cycles-per-sector. Thresholds tabulated for 14-pole motor |
| 5 | Zone B Vf compensation with fixed value causes phase error | Vf now estimated adaptively from Zone A→B step discontinuity with EMA filter. Fallback: 4×4 current×temperature lookup table from bench characterization |
| 6 | Bootloader "write-protected" needs enforcement details | Added: FSEC config register BSS=HIGH for hardware protection, linker ASSERT for build-time guard, runtime address check in bl_flash.c. Added power-fail model with recovery analysis for every failure point. Boot sequence defined. |
| 7 | Bidir DShot PPS remap timing is hidden risk | Added PPS remap latency proof-point: measured with scope in Phase 3, acceptance <2µs remap + <500ns jitter increase. Contingency: separate RX/TX pins |
