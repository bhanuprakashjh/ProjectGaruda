# Reference ESC Firmware Comparison — Lessons for Garuda

## 1. BEMF Zero-Crossing Detection Approaches

### BLHeli_S (Comparator-Based)
- Hardware comparator reads floating phase voltage directly
- Single-bit output: above/below threshold
- Fast (sub-microsecond) but no amplitude information
- Demag detection: if comparator output doesn't match expected polarity
  after commutation, demag is still occurring → extend blanking
- `Demag_Detected_Metric` — sliding average of consecutive demags
- If metric exceeds `Demag_Pwr_Off_Thresh`, power is cut to prevent sync loss

### VESC BLDC (Flux Integration)
- Integrates BEMF voltage over time: `cycle_integrator = Σ(BEMF × dt)`
- Zero-crossing when integral crosses threshold (`cycle_int_limit`)
- Coupling coefficient: `bemf = sl_bemf_coupling_k / max(RPM, sl_min_erpm)`
- Works well even during rapid speed changes (physical basis: magnetic flux)
- More robust than instantaneous voltage comparison

### ASAC-ESC (ADC Threshold)
- ADC samples floating phase, 5-tap moving average filter
- Zero-cross point = `Vmax / 2` (tracks peak voltage dynamically)
- Rising/falling polarity tracked per commutation step
- Simple and educational, but noisy at low speeds
- Filter introduces ~33µs phase delay (compensated in timing)

### Sapog (Least-Squares Regression)
- Collects multiple ADC samples during each step's BEMF window
- Runs least-squares linear fit: `y = slope*x + intercept`
- Solves for exact timestamp where `y = 0` (the zero-crossing)
- Most precise method — sub-sample timing resolution
- Validates solution against timing constraints before using
- Gracefully handles noise (regression averages it out)

### **Garuda Decision**
**Phase 1-2:** Start with ASAC-style ADC threshold + majority filter (simple,
proven, matches our current plan). Add BLHeli-style demag compensation.

**Phase 2 Enhancement:** Add hardware comparator as secondary ZC detector
(hybrid approach already in plan — confirmed as good strategy).

**Phase 5:** Upgrade to Sapog-style least-squares regression for precision
at extreme low and high speeds.

---

## 2. Startup Sequences

### BLHeli_S
1. Forced timed commutations (no BEMF) for ~24 steps
2. Power starts low (`Startup_Pwr`), ramps gradually
3. Transitions to `INITIAL_RUN_PHASE` — BEMF enabled but with low RPM protection
4. After stable BEMF detection → `MOTOR_STARTED` (full control)
5. No explicit "alignment" phase — first commutation is the alignment

### VESC BLDC
1. Open-loop forced commutation at `foc_openloop_rpm` (1500 ERPM)
2. Linear voltage ramp to reach target RPM
3. Observer runs in background tracking motor position
4. Transition when observer output matches forced commutation angle
5. Seamless handoff with angle/voltage offset compensation

### ASAC-ESC
1. Open-loop at 4ms (250 Hz) commutation period, fixed throttle (500/2048)
2. Period decreases by 100µs per step (accelerating)
3. Monitors ZC in background during open-loop
4. Switches to closed-loop when ZC detected within ±100µs of expected time
5. Requires minimum 30 open-loop commutations before attempting transition
6. Simple and clean — good reference implementation

### Sapog
1. Fixed commutation timing initially (`mot_spup_st_cp` = 100ms)
2. Voltage ramp from `mot_v_spinup` (0.5V) over `mot_spup_vramp_t` (3.0s)
3. Monitors BEMF integral to detect synchronization
4. 3x averaging to handle phase asymmetry
5. 5-second timeout protection (configurable)
6. If motor is already spinning (external rotation), detects direction first

### **Garuda Decision**
Adopt ASAC's approach as baseline with enhancements from Sapog:
1. **Alignment phase:** 200ms at first commutation step (like current plan)
2. **Open-loop ramp:** Decreasing period from 4ms, minimum 30 commutations
3. **Transition criterion:** ZC within ±100µs of expected time (from ASAC)
4. **Enhancement:** Track BEMF integral during open-loop (from Sapog)
   to handle cases where ZC timing alone isn't reliable
5. **External rotation detection:** Sense BEMF before starting to determine
   if motor is already spinning (from Sapog) — critical for crash recovery
6. **Timeout:** 5 seconds max for startup, then FAULT (from Sapog)

---

## 3. Demag Compensation (NEW — not in original plan)

**Learned from BLHeli:** After commutation, the previously-active phase
undergoes demagnetization. During demag, BEMF readings are invalid (phase
voltage is clamped by body diode). Sensing too early gives false ZC.

### BLHeli's Approach
- After commutation, check if comparator shows expected polarity
- If NOT expected → demag still occurring, extend blanking
- Track `Demag_Detected_Metric` (sliding average of demag events)
- If metric exceeds threshold → cut power momentarily to prevent desync
- Three levels: Disabled / Low / High compensation

### **Garuda Implementation**
Add `demag.c` module:
```
1. After each commutation, wait BLANKING_TIME
2. Then sample BEMF — if polarity doesn't match expected:
   - demag_count++
   - Extend blanking by 50% of step period
   - Re-check comparator
3. Track demag_metric (exponential moving average of demag events)
4. If demag_metric > DEMAG_POWER_THRESHOLD:
   - Reduce duty by 25% for 6 commutations
   - If still excessive → FAULT
5. Report demag stress via EDT telemetry
```

---

## 4. Timing Advance

### BLHeli_S
- Fixed timing: Low(7.5°), MedLow(11.25°), Medium(15°), MedHigh(18.75°), High(22.5°)
- Selected at configuration time, not adaptive during operation
- Commutation sequence: ZC → wait 15° → commutate → wait advance → scan

### VESC
- Phase advance mapped linearly between 0 ERPM and `BR ERPM`
- Speed-dependent: increases with RPM to compensate for inductance delay

### Sapog
- Adaptive: linearly interpolated between `mot_tim_adv_min` (5°) and `mot_tim_adv_max` (15°)
- Based on commutation period: shorter period (higher speed) → more advance
- Reduced to minimum during sync recovery
- Fixed at minimum during spinup

### **Garuda Decision**
Phase 2: Fixed 15° advance (like BLHeli Medium — good default for drone motors)
Phase 5: Adaptive advance like Sapog — interpolate between min/max based on eRPM

---

## 5. Communication Protocol & GUI Configurator

### VESC Tool Protocol (Gold Standard)
```
Packet: [LEN_TYPE][LENGTH][PAYLOAD][CRC16_HI][CRC16_LO][0x03]
- LEN_TYPE: 2=8-bit len, 3=16-bit len, 4=24-bit len
- CRC16: CCITT over payload
- ~100 command codes for everything
- USB CDC (virtual COM port)
- Same protocol over UART, CAN, BLE
```

Key commands:
- `COMM_GET_VALUES` (4) — bulk telemetry read
- `COMM_SET_MCCONF` (13) / `COMM_GET_MCCONF` (14) — full config read/write
- `COMM_DETECT_MOTOR_PARAM` (24) — auto-detection
- `COMM_FORWARD_CAN` (34) — CAN bridge

GUI is Qt-based desktop app. Features:
- Real-time telemetry plots
- Motor parameter auto-detection wizard
- Configuration read/write/backup as XML
- Firmware update
- Multi-motor management via CAN

### BLHeliSuite
- Serial passthrough via flight controller (Betaflight/iNav)
- 4-way interface protocol for ESC communication
- Reads/writes configuration blocks
- Firmware flashing
- Multi-ESC management (all 4 ESCs at once)
- Desktop application (Windows, .NET-based)

### Sapog
- UAVCAN parameter server — no custom protocol needed
- DroneCAN GUI Tool reads/writes parameters by name
- Firmware update via UAVCAN file transfer service
- No separate configurator app — uses standard DroneCAN tools

### **Garuda GSP Protocol (Updated)**

Based on VESC's proven pattern, simplified:

```
Packet: [0x24][LENGTH][CMD][PAYLOAD...][CRC16_LO][CRC16_HI]
- Start byte: 0x24 ('$')
- LENGTH: payload + cmd (1 byte, max 255)
- CMD: command code (1 byte)
- CRC16: CCITT over [CMD][PAYLOAD]
```

Command codes:
```
0x01  GET_INFO       → [magic:'GRDA'][fw_ver][hw_ver][capabilities]
0x02  GET_CONFIG     → [full 64-byte config block]
0x03  SET_CONFIG     → [full 64-byte config block]
0x04  SAVE_CONFIG    → (commit to EEPROM)
0x05  RESET_DEFAULT  → (factory reset)
0x06  ENTER_BOOTLOADER
0x07  GET_TELEMETRY  → [eRPM:4][V:2][I:2][T:1][duty:1][state:1][faults:1]
0x08  MOTOR_TEST     → [step:1][duty:2][duration:2]
0x09  BEEP           → [tone:1][duration:2]
0x0A  GET_DEBUG      → [bemf_raw:2][zc_count:4][demag_metric:1][timing_advance:1]
0x0B  MOTOR_DETECT   → (auto-detect pole count, direction, timing)
```

### **Garuda Configurator Tool**

**Technology choice:** Web-based using WebSerial API
- Runs in Chrome/Edge browser — no install needed
- WebSerial connects directly to USB/UART
- Framework: Svelte or vanilla JS + Chart.js for real-time plots
- Alternative: Electron app if WebSerial insufficient

**Features (prioritized):**
1. **P0:** Read/write configuration parameters
2. **P0:** Real-time telemetry display (eRPM, voltage, current, temperature)
3. **P0:** Motor direction and timing configuration
4. **P1:** Real-time BEMF waveform plot (debug mode)
5. **P1:** Firmware update via bootloader
6. **P1:** Configuration backup/restore (JSON export)
7. **P2:** Motor auto-detection wizard
8. **P2:** Multi-ESC management (via FC passthrough)
9. **P2:** Startup tune editor

---

## 6. Safety & Protection (Comprehensive)

### From VESC — Soft Thermal Derating
```
if temp > start_temp:
    factor = (end_temp - temp) / (end_temp - start_temp)
    current_limit *= factor
if temp > end_temp:
    shutdown()
```
Separate start/end for FET and motor temperatures.
**Adopt for Garuda:** Linear derating from (tempLimit - 20°C) to tempLimit.

### From BLHeli — Demag Power Cutoff
Track `Demag_Detected_Metric`. If excessive, cut power briefly.
**Adopt for Garuda:** Already described in section 3 above.

### From Sapog — Consecutive Failure Lockout
Count consecutive failed starts. After 7 failures, lock motor.
Requires explicit reset (power cycle or command).
**Adopt for Garuda:** 5 consecutive startup failures → FAULT with lockout.
Clear via DShot CMD 0 (disarm) held for 2 seconds, or power cycle.

### From Sapog — Command TTL (Time-To-Live)
Commands expire if not refreshed within 200ms.
Motor stops gracefully if command stream stops.
**Adopt for Garuda:** Already have signal_loss.c (500ms timeout).
Tighten to 250ms for DShot, keep 500ms for GSP/UART.

### From VESC — Proportional Current Limiting
Instead of hard cutoff, reduce duty proportionally to overcurrent:
```
if current > soft_limit:
    duty_reduction = (current - soft_limit) * ki_current
    duty -= duty_reduction
if current > hard_limit:
    FAULT (immediate shutdown)
```
**Adopt for Garuda:** Two-tier current protection:
- Soft: proportional duty reduction (Sapog-style P-controller)
- Hard: hardware comparator → PWM PCI → instant shutdown

### From Sapog — External Rotation Detection
Before starting, sense BEMF to check if motor is already spinning.
If spinning, determine direction and speed before engaging.
**Adopt for Garuda:** Add to ESC_IDLE state — periodically sample BEMF.
If significant voltage detected, determine direction via BEMF sequence.
On next start command, use detected direction/speed as initial conditions.

### From VESC — Flash CRC Integrity Monitoring
Continuous CRC checking of firmware flash during runtime.
Detects bit-flip corruption, triggers safe reset.
**Adopt for Garuda Phase 4:** CRC32 of application area, checked once at
boot and periodically (every 60 seconds) in background.

---

## 7. Updated Feature Priority Matrix

### Must-Have (Phase 1-4)
- [x] 6-step commutation with override control
- [x] ADC-based BEMF ZC detection with majority filter
- [x] Comparator-assisted ZC (hybrid)
- [x] Open-loop startup → closed-loop transition (±100µs criterion)
- [x] **Demag compensation** (NEW from BLHeli study)
- [x] **External rotation detection** (NEW from Sapog study)
- [x] DShot150/300/600/1200 + bidirectional + EDT
- [x] **Proportional current limiting** (NEW from VESC/Sapog study)
- [x] Soft thermal derating (linear curve)
- [x] Consecutive failure lockout (5 attempts)
- [x] EEPROM config with CRC-16
- [x] GSP protocol (VESC-inspired packet format)
- [x] Bootloader with dual-image

### Should-Have (Phase 5)
- [ ] Least-squares ZC regression (from Sapog)
- [ ] Adaptive timing advance (from Sapog)
- [ ] Flux linkage integration for low-speed (from VESC)
- [ ] Motor auto-detection (pole count, direction, timing)
- [ ] **Web-based configurator tool** (NEW)
- [ ] Flash CRC integrity monitoring (from VESC)
- [ ] Startup voltage ramp (from Sapog)

### Nice-to-Have (Future)
- [ ] CAN bus / DroneCAN support
- [ ] MSP passthrough for Betaflight
- [ ] Ternary hysteresis control
- [ ] Mixed decay mode
- [ ] Multi-ESC synchronization

---

## 8. Code Style Reference

### Best: ASAC-ESC
Cleanest, most readable code. Simple state machine, clear variable names,
inline comments explaining physics. Good template for our style.

### Most Complete: VESC
Production-grade but complex. Good reference for safety features and
configuration management. Too complex to use as style template.

### Most Relevant Algorithm: Sapog
Closest to what we're building (sensorless 6-step for drones).
Least-squares BEMF is the algorithm we should aspire to.
ChibiOS adds complexity but the motor control core is clean C.

### DShot Reference: BLHeli_S + Bluejay
BLHeli_S has the actual DShot bit-timing (2T=0, 3T=1).
Bluejay has bidirectional DShot + EDT implementation.
Both are assembly — need to translate concepts, not code.
