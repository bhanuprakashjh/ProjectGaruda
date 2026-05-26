# VESC (Vedder Electronic Speed Controller) — Complete Feature Research

**Date**: 2026-03-18
**Source**: VESC firmware v7.00 (vedderb/bldc on GitHub, 3500+ commits, GPLv3)
**Purpose**: Reference for Garuda ESC feature planning

---

## 1. Motor Control Modes

### 1.1 Motor Types
- **BLDC** (trapezoidal 6-step commutation)
- **FOC** (sinusoidal field-oriented control — the primary mode)
- **DC** (brushed DC motor)

### 1.2 FOC Sensor Modes
| Mode | Description |
|------|-------------|
| `SENSORLESS` | Observer-only, no position sensor needed |
| `ENCODER` | Absolute/incremental encoder feedback |
| `HALL` | Hall effect sensors (6-state) |
| `HFI` | High-Frequency Injection for standstill/low-speed position |
| `HFI_START` | HFI at startup only, transition to observer |
| `HFI_V2` through `HFI_V5` | Iterative HFI algorithm improvements |

### 1.3 BLDC Sensor Modes
- `SENSORLESS` — BEMF zero-crossing integration
- `SENSORED` — Hall sensors
- `HYBRID` — Hall startup, sensorless at speed

### 1.4 Control Modes (runtime selectable)
| Mode | Description |
|------|-------------|
| `DUTY` | Direct duty cycle command |
| `SPEED` | Closed-loop RPM control via speed PID |
| `CURRENT` | Torque (Iq) control |
| `CURRENT_BRAKE` | Regenerative braking with current setpoint |
| `POS` | Position control (servo mode) |
| `HANDBRAKE` | Active short-circuit braking (hold position) |
| `OPENLOOP` | Open-loop current drive at commanded frequency |
| `OPENLOOP_PHASE` | Open-loop at commanded electrical angle |
| `OPENLOOP_DUTY` | Open-loop duty at commanded frequency |
| `OPENLOOP_DUTY_PHASE` | Open-loop duty at commanded angle |

### 1.5 FOC Observer Types
| Observer | Description |
|----------|-------------|
| `ORTEGA_ORIGINAL` | Ortega flux observer (original) |
| `MXLEMMING` | MXLemming's improved flux observer (best for SPM) |
| `ORTEGA_LAMBDA_COMP` | Ortega with lambda (flux linkage) compensation |
| `MXLEMMING_LAMBDA_COMP` | MXLemming with lambda compensation |
| `MXV` | MXV observer variant |
| `MXV_LAMBDA_COMP` | MXV with lambda compensation |
| `MXV_LAMBDA_COMP_LIN` | MXV with linearized lambda compensation |

### 1.6 FOC Advanced Features
- **MTPA** (Maximum Torque Per Ampere): modes OFF, IQ_TARGET, IQ_MEASURED — for IPM motors with reluctance torque
- **Field Weakening**: extends speed range above base speed by injecting negative Id
- **Current Controller Decoupling**: DISABLED, CROSS, BEMF, CROSS_BEMF — feed-forward decoupling of d/q axes
- **Saturation Compensation**: DISABLED, FACTOR, LAMBDA, LAMBDA_AND_FACTOR — compensates iron saturation effects on inductance
- **Dead-time Compensation**: software compensation for inverter dead-time distortion
- **Current Sampling Modes**: LONGEST_ZERO, ALL_SENSORS, HIGH_CURRENT, BEST_SENSOR
- **Control Sampling Modes**: V0 only, V0+V7, V0+V7 interpolated
- **Speed PID Sources**: PLL, FAST, FASTER — different speed estimation filters
- **FOC Speed Source**: CORRECTED or raw OBSERVER

### 1.7 HFI (High Frequency Injection)
- Enables sensorless position estimation at zero/low speed where BEMF is insufficient
- Amplitude modes: SIX_VECTOR, D_SINGLE_PULSE, D_DOUBLE_PULSE
- Configurable samples: 8, 16, or 32
- Multiple algorithm versions (V1-V5) for different motor saliency characteristics

### 1.8 PWM Modes
- `NONSYNCHRONOUS_HISW` — non-synchronous, high-side switching
- `SYNCHRONOUS` — complementary synchronous switching
- `BIPOLAR` — bipolar PWM

---

## 2. Protection Features

### 2.1 Fault Codes (29 distinct faults)
| Fault | Description |
|-------|-------------|
| `OVER_VOLTAGE` | Bus voltage exceeds limit |
| `UNDER_VOLTAGE` | Bus voltage below minimum |
| `DRV` | Gate driver (DRV8301/8302/8323) fault |
| `ABS_OVER_CURRENT` | Hardware comparator overcurrent trip |
| `OVER_TEMP_FET` | MOSFET temperature exceeds limit |
| `OVER_TEMP_MOTOR` | Motor winding temperature exceeds limit |
| `GATE_DRIVER_OVER_VOLTAGE` | Gate driver supply overvoltage |
| `GATE_DRIVER_UNDER_VOLTAGE` | Gate driver supply undervoltage |
| `MCU_UNDER_VOLTAGE` | MCU supply brownout |
| `BOOTING_FROM_WATCHDOG_RESET` | Watchdog triggered reboot |
| `ENCODER_SPI` | SPI encoder communication failure |
| `ENCODER_SINCOS_BELOW_MIN_AMPLITUDE` | Sin/cos encoder signal too weak |
| `ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE` | Sin/cos encoder signal too strong |
| `FLASH_CORRUPTION` | Flash memory CRC mismatch |
| `HIGH_OFFSET_CURRENT_SENSOR_1/2/3` | Current sensor offset drift (per phase) |
| `UNBALANCED_CURRENTS` | Phase current imbalance detected |
| `BRK` | Brake fault |
| `RESOLVER_LOT/DOS/LOS` | Resolver loss of tracking/DOS/signal |
| `FLASH_CORRUPTION_APP_CFG` | App config flash corrupt |
| `FLASH_CORRUPTION_MC_CFG` | Motor config flash corrupt |
| `ENCODER_NO_MAGNET` | Magnetic encoder — no magnet detected |
| `ENCODER_MAGNET_TOO_STRONG` | Magnetic encoder — magnet too close |
| `PHASE_FILTER` | Phase voltage filter fault |
| `ENCODER_FAULT` | Generic encoder fault |
| `LV_OUTPUT_FAULT` | Low-voltage output fault |

### 2.2 Fault Data Capture
On every fault, VESC captures a complete snapshot:
- Motor number, fault code
- Instantaneous and filtered current
- Bus voltage, gate driver voltage
- Duty cycle, RPM, tachometer count
- Cycles running, timer sample values
- Commutation step, temperature
- DRV8301 fault register
- Info string + 2 numeric arguments

### 2.3 Current Limiting
- **Absolute motor current limit** (positive and negative): l_current_max, l_current_min
- **Absolute input (battery) current limit**: l_in_current_max, l_in_current_min (regen)
- **Hardware overcurrent**: DRV8301 comparator with configurable modes:
  - `OC_LIMIT` — limit current
  - `OC_LATCH_SHUTDOWN` — latch off on trip
  - `OC_REPORT_ONLY` — report but don't trip
  - `OC_DISABLED` — ignore
- **Software current limits applied continuously** in the control loop (not just fault threshold)
- Input current limiting uses P=V*I conservation to back-calculate battery current from motor current

### 2.4 Temperature Monitoring
- **MOSFET temperature**: 3 individual FET temp sensors (temp_mos_1/2/3) + aggregate
- **Motor temperature**: external NTC/PTC on motor windings
- **Supported sensor types**: NTC 10K, PTC 1K, KTY83-122, NTC 100K, KTY84-130, custom NTC/PTC curves, PT1000, or disabled
- **Derating**: soft current reduction between start/end temperature thresholds (not hard cutoff)
- **BMS temperature integration**: cell temp limits from CAN-connected BMS

### 2.5 Battery Protection
- **Battery cutoff start/end voltages**: soft ramp-down between start and end (not hard cutoff)
- **Predefined battery types**: Li-ion (3.0-4.2V/cell), LiFePO4 (2.6-3.6V/cell), Lead-acid
- **BMS integration**: SOC-based limiting, voltage-based limiting, configurable via CAN

### 2.6 RPM Limiting
- Maximum ERPM limit (both directions)
- Minimum ERPM for direction change (prevents oscillation)
- Configurable PID rate for speed control: 25Hz to 10000Hz

### 2.7 Watt Limiting
- Maximum watt limit (motor output power)
- Maximum input watt limit (battery power draw)
- Limits applied as current reduction proportional to power excess

### 2.8 Duty Cycle Limiting
- Maximum duty cycle limit
- Minimum duty cycle for current control stability

### 2.9 Safe Start
- `DISABLED` — motor can spin immediately on power-up
- `REGULAR` — requires throttle at zero before enabling
- `NO_FAULT` — only blocks start if there's an active fault

### 2.10 Kill Switch
- PPM-based: trigger on low or high PPM signal
- ADC-based: trigger on low or high ADC2 voltage

### 2.11 Watchdog
- Hardware watchdog on STM32 — auto-reboot on firmware hang
- Post-reboot: `FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET` reported

---

## 3. Communication Protocols

### 3.1 VESC UART/USB Protocol
**Packet format**:
- Start byte: `0x02` (payload <=256 bytes) or `0x03` (payload <=65536 bytes)
- Length: 1 or 2 bytes depending on start byte
- Payload: command ID (1 byte) + command data
- CRC16: 2 bytes over payload
- End byte: `0x03`

**~160 command IDs** covering:
- **Firmware**: version query, bootloader jump, firmware erase/write (with LZO compression)
- **Motor control**: set duty/current/current_brake/RPM/position/handbrake (absolute and relative)
- **Configuration**: get/set motor config, get/set app config, defaults, temporary config
- **Telemetry**: GET_VALUES (returns mc_values struct), GET_VALUES_SELECTIVE (bitmask), GET_VALUES_SETUP (multi-VESC aggregate)
- **Detection**: detect R/L, detect flux linkage, detect encoder offset, detect hall sensors, auto-apply FOC
- **Sampling**: scope/debug sample capture with trigger modes (manual, fault-triggered)
- **Terminal**: send text commands, receive printf output
- **Plotting**: init plot, add graph, set graph, send data points (real-time graphing in VESC Tool)
- **IMU**: get quaternions, accelerometer, gyroscope data
- **BMS**: cell voltages, temperatures, SOC, charge control
- **IO Board**: ADC read, digital I/O, PWM output (CAN-connected expansion)
- **GNSS**: GPS lat/lon/alt/speed/time
- **File system**: list/read/write/mkdir/remove files on flash
- **Logging**: start/stop/configure data logging
- **Lisp**: read/write/erase/stream Lisp code, REPL, stats
- **Stats**: odometer, runtime, energy counters

### 3.2 CAN Bus Protocol
**68+ CAN packet types** with extended ID encoding:
- **Control commands**: duty, current, current_brake, RPM, position (all relative variants too)
- **Status broadcasts** (6 status messages at configurable rates):
  - Status 1: RPM, current, duty
  - Status 2: Ah consumed, Ah charged
  - Status 3: Wh consumed, Wh charged
  - Status 4: FET temp, motor temp, input current, PID position
  - Status 5: Input voltage, tachometer
  - Status 6: ADC1, ADC2, ADC3, PPM
- **Configuration**: current limits, FOC ERPM, battery cut — store or temporary
- **Firmware update**: erase/write/bootloader over CAN (all nodes or specific)
- **BMS**: 20+ BMS CAN packets (voltages, currents, temps, SOC, balancing)
- **Power switch**: PSW status, on/off control (anti-spark)
- **GNSS**: GPS data relay over CAN
- **IO Board**: ADC 1-12, digital I/O, PWM outputs
- **Baud rates**: 10K, 20K, 50K, 75K, 100K, 125K, 250K, 500K, 1M
- **Modes**: VESC native, UAVCAN, comm bridge

### 3.3 PPM Input
- Configurable pulse range: start, end, center (for center-return sticks)
- Median filter option
- Hysteresis for noise rejection
- Throttle expo: exponential, natural, polynomial curves
- Ramp time: separate positive/negative rates
- 12 control types: current, current no-reverse, duty, speed PID, position (180/360 deg), smart reverse, etc.
- Multi-ESC mode (for CAN-linked ESCs)
- Traction control: limit current difference between ESCs

### 3.4 ADC Input
- Dual ADC channels (throttle + brake or dual throttle)
- Configurable voltage range: start, end, min, max, center
- Polarity inversion
- Filter option
- 15 control types: current, duty, PID speed — with reverse center, reverse button, brake on ADC2
- Configurable update rate

### 3.5 Nunchuk/NRF Remote
- Wii Nunchuk compatible joystick (analog stick + accelerometer + 2 buttons)
- NRF24L01+ wireless link: 250K/1M/2M speed, configurable power (-18 to 0 dBm)
- Configurable channel, address (3-5 bytes), CRC (disabled/1B/2B), retry delay/count
- Pairing protocol with handshake
- Control types: current, current no-reverse, current bidirectional
- Smart reverse with configurable max duty and ramp time

### 3.6 PAS (Pedal Assist System)
- For e-bike applications
- Sensor types: quadrature
- Control types: cadence-based, torque-based, torque with cadence timeout
- Configurable pedal RPM range, current scaling, magnet count

### 3.7 Application Combinations
- `APP_NONE`, `APP_PPM`, `APP_ADC`, `APP_UART`, `APP_PPM_UART`, `APP_ADC_UART`
- `APP_NUNCHUK`, `APP_NRF`, `APP_CUSTOM`, `APP_PAS`, `APP_ADC_PAS`

---

## 4. VESC Tool (Configurator GUI)

### 4.1 Motor Configuration Parameters (~140+ parameters)
**Current limits**:
- Motor current max/min (positive/negative)
- Battery current max/min
- Absolute current max
- Slow ABS current limit toggle

**Voltage limits**:
- Battery cutoff start/end
- Max input voltage

**RPM limits**:
- Max ERPM (forward/reverse)
- Min ERPM for direction change

**Power limits**:
- Max motor watt
- Max input watt

**Temperature limits**:
- FET temp start/end (derating curve)
- Motor temp start/end
- Sensor type selection (NTC/PTC/KTY/PT1000)

**FOC parameters**:
- Current controller gains (Kp, Ki)
- Observer gain
- Observer offset
- Flux linkage
- Motor resistance (R), inductance (Ld, Lq)
- Motor poles
- Duty cycle ramp rate
- Current filter coefficient
- Decoupling mode
- HFI voltage, start voltage, observation mode
- Saturation compensation mode/factor
- Speed PID: Kp, Ki, Kd, filter, min/max ERPM, rate
- Position PID: Kp, Ki, Kd, angle division

**BLDC parameters**:
- Timing advance
- Commutation mode (integrate/delay)
- Hall sensor table (6 entries)
- PWM mode

**Encoder parameters**:
- Encoder type (16 types supported)
- Encoder counts (PPR)
- Encoder offset
- Encoder ratio
- SPI speed, filter

### 4.2 Real-Time Data Display
The `mc_values` struct provides live telemetry:
- Input voltage (Vbus)
- MOSFET temperatures (3 individual + aggregate)
- Motor temperature
- Motor current, input current
- Id, Iq (FOC d-q axis currents)
- Vd, Vq (FOC d-q axis voltages)
- RPM (electrical)
- Duty cycle
- Amp-hours consumed/charged
- Watt-hours consumed/charged
- Tachometer (absolute and signed)
- Rotor position
- Fault code
- VESC ID

**Setup values** (multi-VESC aggregate):
- Total Ah, Wh consumed/charged
- Total current (motor + input)
- Number of connected VESCs

**Statistics**:
- Runtime, speed average/max, power average/max
- Temperature averages/maxes (motor + FET)
- Current average/max

### 4.3 Motor Detection Wizard
Automated detection commands:
1. **Detect R, L**: measures motor resistance and inductance (Ld, Lq separately for IPM motors)
2. **Detect flux linkage (open-loop)**: spins motor open-loop to measure BEMF and compute lambda
3. **Detect encoder**: finds encoder offset and direction
4. **Detect hall sensors (FOC)**: maps hall states to electrical angles
5. **Detect and apply all FOC**: one-button auto-setup — runs R/L, flux, hall, encoder detection and writes all parameters

### 4.4 Scope / Debug Sampling
- **Trigger modes**: OFF, NOW, START, TRIGGER_START, TRIGGER_FAULT
- Captures time-series of internal variables at ISR rate
- Can send immediately or store for later retrieval
- Single-sample mode for slow polling
- VESC Tool displays waveforms graphically

### 4.5 Real-Time Plotting
- Firmware can emit plot data via `COMM_PLOT_*` commands
- Multiple named graphs per plot
- Used by LispBM scripts and motor detection routines
- VESC Tool renders as live strip charts

### 4.6 Terminal / Console
- Text command interface via UART/USB
- `COMM_TERMINAL_CMD` sends commands, `COMM_PRINT` receives output
- Synchronous command mode available
- Used for diagnostics, debug, custom commands

### 4.7 Firmware Update
- Over USB: erase + write with LZO compression support
- Over CAN: update all connected VESCs from one USB connection
- Hardware-specific firmware variants
- Bootloader management (erase, program via SWD or BlackMagic probe)

### 4.8 Multi-VESC (CAN)
- Each VESC has a unique controller_id (CAN address)
- Status messages broadcast at configurable rates (2 rate groups)
- Aggregated telemetry (total current, power, Ah across all nodes)
- Traction control: max current difference between CAN-linked ESCs
- Coordinated firmware update over CAN
- Forward-any-command to specific CAN node

### 4.9 Configuration Persistence
- Motor config (mc_configuration) and app config (app_configuration) stored in flash
- CRC-based signature validation (MCCONF_SIGNATURE, APPCONF_SIGNATURE)
- Separate corruption detection for motor and app configs
- Default restore on corruption
- Backup data structure preserves: odometer, runtime, encoder correction table, CAN settings
- Temporary config mode (applies without flash write — for experimentation)

---

## 5. Encoder Support (16 types)

| Encoder | Interface | Notes |
|---------|-----------|-------|
| ABI (quadrature) | Digital | Standard incremental encoder |
| AS5047/AS5048 SPI | SPI HW | 14-bit magnetic |
| AS5x47U SPI | SPI | Updated AS504x |
| AD2S1205 | Resolver-to-digital | Resolver interface |
| Sin/Cos | Analog | 1Vpp analog encoder |
| TS5700N8501 | Serial | Tamagawa encoder (single + multiturn) |
| MT6816 SPI | SPI HW | MagnTek magnetic |
| BiSS-C | Serial | Open protocol absolute encoder |
| TLE5012 SSC | SSC SW/HW | Infineon magnetic |
| MA782 | SPI | MPS magnetic |
| PWM | Digital | PWM-output encoders |
| PWM + ABI | Digital | Combined PWM absolute + ABI incremental |
| Custom | Callback | User-defined via LispBM or custom app |

Features: error rate tracking, fault checking, multiturn support, index detection, configurable offset.

---

## 6. LispBM Scripting Engine

- **Full Lisp interpreter** running on the ESC in a sandboxed environment
- Scripts stored in flash, auto-start on boot
- Access to motor control API (current, duty, RPM, position commands)
- Access to all sensor data (ADC, encoder, IMU, temperature, voltage, current)
- CAN bus communication for multi-device coordination
- GPIO, PWM output, I2C, UART, input capture
- Real-time variable monitoring in VESC Tool
- CPU and memory usage tracking
- REPL (interactive console)
- Used for: custom throttle curves, balance vehicle control, robotics, automation

---

## 7. Additional Features

### 7.1 IMU Integration
- Internal IMU or external (MPU9x50, ICM20948, BMI160, LSM6DS3)
- AHRS algorithms: Madgwick, Mahony, Madgwick Fusion
- Configurable filter levels (low/medium/high)
- Quaternion output, accelerometer, gyroscope
- Used for balance vehicles, attitude-aware throttle

### 7.2 BMS Integration
- CAN-connected VESC BMS
- Cell voltage monitoring (up to 50 cells)
- Balancing control (per-cell override, self-test)
- Temperature monitoring (50 ADC channels + IC temp + humidity)
- SOC/SOH estimation
- Charge allowed/disallowed control
- Counters: Ah and Wh (charge + discharge totals)

### 7.3 GNSS
- GPS data: lat, lon, height, speed, HDOP, time/date
- Relayed over CAN

### 7.4 IO Board
- CAN-connected expansion board
- 12 ADC channels, digital inputs, PWM outputs, digital outputs

### 7.5 Power Switch (Anti-spark/Precharge)
- `PSW_GET_STATUS`: read Vin, Vout, temp, output/precharge/discharge state
- `PSW_SWITCH`: on/off control
- Integrated in some VESC hardware variants

### 7.6 Servo Output
- Standard RC servo PWM output (1000-2000us at 50Hz)

### 7.7 Shutdown Modes
- Always off, always on, toggle button
- Auto-off after: 10s, 1m, 5m, 10m, 30m, 1h, 5h

### 7.8 Auxiliary Output
- Configurable based on: time delay (2/5/10s), running state, temperature thresholds (motor/MOSFET 50/70 degC)
- Used for cooling fans

### 7.9 Odometer & Statistics
- Persistent odometer (survives reboot)
- Persistent runtime counter
- Session statistics: speed, power, temperature, current (sum, max, sample count)

### 7.10 File System
- Flash-based file operations: list, read, write, mkdir, remove
- Used by LispBM for data storage

### 7.11 QML UI
- Custom QML (Qt Quick) UI definitions stored on the VESC
- Hardware-specific and app-specific UI pages
- Rendered in VESC Tool and mobile apps

---

## 8. Production Features

### 8.1 Bootloader
- STM32 bootloader with SWD programming
- BlackMagic Debug probe support (connect, erase, write, reboot, pin mapping)
- USB DFU bootloader jump
- CAN bootloader (update over CAN bus)

### 8.2 Firmware Update
- USB: erase + write with LZO compression (faster updates)
- CAN: broadcast update to all nodes or targeted by hardware type
- Hardware-specific firmware selection
- Bootloader can be erased/reprogrammed remotely

### 8.3 Configuration Backup/Restore
- Binary serialization with CRC signature
- VESC Tool can save/load config files
- Default config restore on corruption
- Temporary config (no flash write) for testing
- Separate motor config and app config

### 8.4 Motor Parameter Persistence
- All motor parameters stored in flash with signature validation
- Survives power cycles
- Configurable defaults per hardware variant
- Backup structure preserves calibration data across firmware updates

### 8.5 Anti-Spark / Precharge
- Power switch module with precharge circuit
- CAN-controlled on/off
- Status monitoring (Vin, Vout, temperature, state)

### 8.6 Hardware Abstraction
- Each hardware variant has its own `hw_*.h` defining:
  - ADC channel mapping, current sense resistor values, amplifier gains
  - Voltage divider ratios
  - DRV8301/8302/8323 configuration
  - Temperature sensor types and parameters
  - Phase filter presence
  - CAN transceiver enable pins
  - Anti-spark/precharge pins
  - Servo output pins
  - LED pins
- 100+ hardware variants supported ("All of them!")

---

## 9. VESC vs BLHeli / AM32 / ESCape32

### 9.1 What Makes VESC Superior

| Feature | VESC | BLHeli_32 | AM32 | ESCape32 |
|---------|------|-----------|------|----------|
| **Motor control** | FOC + BLDC + DC | BLDC only | BLDC only | BLDC + limited FOC |
| **FOC quality** | 7 observer types, HFI, MTPA, field weakening, saturation comp | N/A | N/A | Basic |
| **Current control** | True d-q axis PI with decoupling | Duty-cycle only | Duty-cycle only | Basic |
| **Position sensors** | 16 encoder types + hall + sensorless | Sensorless only | Sensorless only | Sensorless + hall |
| **HFI** | Yes (5 versions) | No | No | No |
| **Motor detection** | Full auto (R, L, flux, encoder, hall) | Manual timing only | None | Basic R/L |
| **Current sensing** | 3-shunt + offset calibration | Usually 0 or 1 shunt | 0 shunt (virtual) | 1 shunt |
| **Protection depth** | 29 fault types, per-sensor offset check, flash CRC | OC, OT basic | OC basic | OC, OT |
| **Temperature** | 3 FET + motor NTC/PTC with soft derating | FET only, hard cutoff | None | FET only |
| **Battery management** | Soft cutoff ramp + full BMS integration | Hard cutoff | Hard cutoff | Basic |
| **Communication** | UART + USB + CAN + NRF + PPM + ADC + PAS | PWM/DShot only | PWM/DShot only | PWM/DShot + serial |
| **CAN bus** | Full VESC + UAVCAN with 68+ message types | No | No | No |
| **Multi-ESC** | CAN-linked with traction control | No | No | No |
| **Scripting** | Full Lisp interpreter | No | No | No |
| **IMU** | 5 IMU types, AHRS fusion | No | No | No |
| **GPS** | Full GNSS integration | No | No | No |
| **BMS** | Deep integration (50 cells, balancing, SOC) | No | No | No |
| **GUI** | VESC Tool (desktop + mobile) with RT plotting | BLHeli Suite | No dedicated tool | Basic serial |
| **Scope** | ISR-rate debug sampling with triggers | No | No | No |
| **Firmware update** | USB + CAN + LZO compression | Bootloader | Bootloader | Bootloader |
| **File system** | Flash-based file operations | No | No | No |
| **Custom UI** | QML pages on device | No | No | No |
| **Parameter count** | 140+ motor + 50+ app | ~20 | ~15 | ~30 |
| **Supported HW** | 100+ boards | Specific ESC silicon | Specific ESC silicon | Specific ESC silicon |
| **Typical MCU** | STM32F4/F7/H7 (168-480 MHz, 256K+ RAM) | EFM8 or STM32F0 | AT32/GD32 | STM32F051 |

### 9.2 Key Architectural Advantages
1. **True FOC** with sine-wave commutation produces smoother torque, less noise, higher efficiency, and better low-speed performance than trapezoidal BLDC
2. **Multiple observer algorithms** allow matching the best observer to the motor type (SPM vs IPM)
3. **HFI** enables zero-speed position estimation — critical for position control and heavy-load startup
4. **Comprehensive fault system** catches issues before they cause damage (29 fault types vs 2-3 in drone ESCs)
5. **Soft current/temperature derating** rather than hard cutoffs prevents sudden loss of control
6. **CAN bus** enables coordinated multi-motor systems with a single configuration point
7. **LispBM** enables custom control algorithms without firmware recompilation
8. **Configuration depth** (140+ parameters) allows tuning for any motor/application combination

---

## 10. What Features Are Overkill for a Drone ESC

### 10.1 Unnecessary for Drone Application
| Feature | Why Overkill |
|---------|-------------|
| **Position control mode** | Drones use throttle (current/duty), not servo positioning |
| **PAS (Pedal Assist)** | E-bike specific |
| **Nunchuk input** | Skateboard/vehicle specific |
| **Balance vehicle support** | Not applicable |
| **BLDC trapezoidal mode** | FOC is strictly better for drones if MCU supports it; if MCU can't do FOC, you wouldn't run VESC anyway |
| **DC motor mode** | Drones use BLDC/PMSM |
| **16 encoder types** | Drone motors are sensorless; at most hall sensors for startup |
| **HFI** | Drone props have minimal load at standstill; not needed |
| **MTPA** | Drone motors are SPM (no reluctance torque) |
| **Field weakening** | Drones don't need to exceed base speed; prop load curve naturally limits |
| **Saturation compensation** | Drone motors are airgap-limited, minimal saturation |
| **Full BMS integration** | Flight controller handles battery monitoring |
| **GNSS** | Flight controller has GPS |
| **IMU** | Flight controller has IMU |
| **IO Board expansion** | Not needed |
| **File system** | No storage needs |
| **QML custom UI** | Overkill for drone ESC configuration |
| **LispBM scripting** | Flight controller handles logic |
| **Servo output** | Not applicable |
| **Nunchuk/NRF remote** | Not applicable |
| **Traction control** | Flight controller manages motor mixing |
| **Odometer** | Not relevant |
| **Shutdown modes** | ESC should be always-on when powered |

### 10.2 Essential Features a Drone ESC SHOULD Take from VESC
| Feature | Why Essential |
|---------|-------------|
| **FOC with flux observer** | Efficiency, smooth torque, quiet operation |
| **Auto-detect R, L, flux** | No manual motor parameter entry |
| **Current control (Iq)** | Flight controller sends torque commands |
| **Soft current limiting** | Prevents abrupt thrust loss |
| **Temperature derating** | Prevents thermal damage in enclosed frames |
| **DShot/PWM input** | Standard drone protocol |
| **Fault capture with context** | Post-crash diagnostics |
| **Battery voltage soft cutoff** | Graceful low-battery behavior |
| **Watchdog** | Auto-recovery from firmware hang |
| **Configuration persistence** | Flash storage of motor parameters |
| **Bootloader + CAN update** | Field firmware updates |
| **Observer health monitoring** | Desync detection and recovery |
| **Debug sampling** | Development and tuning |
| **UART telemetry** | ESC telemetry to flight controller (voltage, current, temp, RPM) |

### 10.3 Nice-to-Have Features Worth Considering
| Feature | Rationale |
|---------|-----------|
| **CAN bus (with UAVCAN/DroneCAN)** | Industry standard for modern drone ESC-FC communication |
| **Speed PID mode** | Governor mode for fixed-wing propellers |
| **Hall sensor startup** | For high-KV motors with weak BEMF at low speed |
| **Multi-shunt current sense** | Better current measurement accuracy |
| **Real-time plotting protocol** | Valuable during development |
| **Temporary config mode** | Test parameters without committing to flash |
| **Current sensor offset calibration** | Startup self-calibration |
| **Unbalanced current detection** | Catches wiring/motor faults |

---

## 11. VESC Packet Protocol Detail

### 11.1 Framing
```
Short packet (payload <= 256 bytes):
  [0x02] [len:1] [payload:N] [crc16:2] [0x03]

Long packet (payload <= 65536 bytes):
  [0x03] [len:2] [payload:N] [crc16:2] [0x03]
```
- CRC16 computed over payload only
- Payload[0] = command ID (COMM_xxx enum)
- Payload[1..] = command-specific data

### 11.2 Key Command Flows

**Get real-time values**:
```
TX: [0x02][0x01][COMM_GET_VALUES][crc][0x03]
RX: [0x02][len][COMM_GET_VALUES][temp_mos:2][temp_motor:2][current_motor:4]
    [current_in:4][id:4][iq:4][rpm:4][duty:2][ah:4][ah_chg:4][wh:4]
    [wh_chg:4][tacho:4][tacho_abs:4][fault:1][pid_pos:4][vesc_id:1]
    [vd:4][vq:4][crc][0x03]
```

**Set current (torque command)**:
```
TX: [0x02][0x05][COMM_SET_CURRENT][current_mA:4][crc][0x03]
```

**Motor detection (FOC auto-setup)**:
```
TX: COMM_DETECT_MOTOR_R_L → returns R, L, Ld-Lq
TX: COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP → returns lambda
TX: COMM_DETECT_HALL_FOC → returns hall table
TX: COMM_DETECT_APPLY_ALL_FOC → runs all above and applies
```

### 11.3 CAN Protocol
- Extended CAN ID: upper bits = packet type, lower 8 bits = VESC ID
- 8-byte CAN frames, multi-frame buffering for large packets
- Status messages broadcast periodically (configurable rate per group)
- Any UART command can be forwarded over CAN via `COMM_FORWARD_CAN`

---

## 12. Summary Statistics

| Metric | Count |
|--------|-------|
| COMM_ command IDs | ~160 |
| CAN packet types | 68+ |
| Fault codes | 29 |
| Motor config parameters | 140+ |
| App config parameters | 50+ |
| FOC observer types | 7 |
| FOC sensor modes | 9 |
| Encoder types | 16 |
| Control modes | 11 |
| PPM control types | 12 |
| ADC control types | 15 |
| Temperature sensor types | 9 |
| CAN baud rates | 10 |
| Supported HW variants | 100+ |
| Firmware commits | 3500+ |
