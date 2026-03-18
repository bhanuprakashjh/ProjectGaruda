# Garuda CK — Production ESC Plan

**Target**: Production-grade drone ESC on dsPIC33CK + ATA6847
**Date**: 2026-03-18
**Current State**: Sensorless 6-step BLDC with GSP protocol, current sensing, HW ILIM

---

## 1. Industry Feature Matrix

### Legend
- **G** = Garuda CK (current)
- **P** = Planned for Garuda
- AM32 / BLH_S / BLH_32 / ESC32 / VESC = competitor status

| Feature | G | P | AM32 | BLH_S | BLH_32 | ESC32 | VESC |
|---------|---|---|------|-------|--------|-------|------|
| **Motor Control** | | | | | | | |
| 6-step sensorless BLDC | Y | | Y | Y | Y | Y | Y |
| FOC sensorless | - | Ph4 | - | - | - | - | Y |
| Sinusoidal startup | - | Ph3 | Y | - | - | Y | Y |
| Variable PWM frequency | - | Ph3 | Y | - | Y | Y | Y |
| Complementary PWM | Y | | Y | Y | Y | Y | Y |
| Timing advance (linear) | Y | | Y | Y | Y | Y | Y |
| Timing advance (auto) | - | Ph2 | - | Y | Y | - | Y |
| Motor auto-detect (R/L/Ke) | - | Ph2 | - | - | - | - | Y |
| **Speed Control** | | | | | | | |
| Pot/ADC throttle | Y | | Y | - | - | Y | Y |
| DShot input | - | Ph1 | Y | Y | Y | Y | - |
| PWM RC input | - | Ph1 | Y | Y | Y | Y | Y |
| Speed PI controller | - | Ph2 | - | - | - | - | Y |
| RPM-dependent duty ramp | - | Ph1 | - | Y | Y | Y | - |
| Idle speed hold | Y | | Y | - | Y | - | Y |
| **Current Control** | | | | | | | |
| Phase current sensing (2-shunt) | Y | | - | - | - | - | Y |
| Bus current (reconstructed) | Y | | Y | - | Y | Y | Y |
| HW cycle-by-cycle ILIM | Y | | - | - | - | - | - |
| SW current limit (PD loop) | - | Ph1 | Y | - | Y | Y | Y |
| Overcurrent hard fault | Y | | - | - | - | - | Y |
| Current derating (soft limit) | - | Ph2 | - | - | - | - | Y |
| DC average current tracking | - | Ph1 | Y | - | Y | Y | Y |
| **Protection** | | | | | | | |
| Overvoltage | Y | | - | - | - | - | Y |
| Undervoltage | Y | | Y | - | Y | Y | Y |
| VDS short circuit (HW) | Y | | - | - | - | - | - |
| Over-temperature (gate driver) | - | Ph1 | - | - | - | - | Y |
| Over-temperature (MOSFET) | - | Ph2 | - | - | Y | Y | Y |
| Stall/desync detection | Y | | Y | Y | Y | Y | Y |
| Desync auto-recovery | Y | | Y | - | - | - | Y |
| Motor line diagnostics | - | Ph2 | - | - | - | - | - |
| Locked rotor protection | - | Ph1 | Y | - | - | - | Y |
| Demag compensation | - | Ph2 | - | Y | Y | - | - |
| Fault context capture | - | Ph2 | - | - | - | - | Y |
| **Communication** | | | | | | | |
| GSP binary protocol | Y | | - | - | - | - | - |
| Debug UART console | Y | | - | - | - | Y | Y |
| DShot telemetry (bidir) | - | Ph2 | Y | - | Y | Y | - |
| KISS telemetry | - | Ph3 | Y | - | Y | - | - |
| CAN bus / DroneCAN | - | Ph4 | - | - | - | - | Y |
| 4-way FC passthrough | - | Ph3 | Y | Y | Y | - | - |
| **Configuration** | | | | | | | |
| Runtime parameter tuning | - | Ph1 | Y | - | Y | Y | Y |
| Motor profiles (named) | Y | | - | - | - | - | Y |
| EEPROM/flash persistence | - | Ph1 | Y | Y | Y | Y | Y |
| Config backup/restore | - | Ph2 | Y | Y | Y | - | Y |
| **GUI / Configurator** | | | | | | | |
| Web-based (WebSerial) | - | Ph1 | Y | - | - | - | - |
| Real-time telemetry gauges | - | Ph1 | - | - | - | - | Y |
| Real-time scope/charts | - | Ph2 | - | - | - | - | Y |
| Motor setup wizard | - | Ph2 | - | - | - | - | Y |
| Parameter editor | - | Ph1 | Y | Y | Y | Y | Y |
| Motor spin test | - | Ph1 | Y | - | - | - | Y |
| Firmware update | - | Ph4 | Y | Y | Y | - | Y |
| **Production** | | | | | | | |
| Bootloader | - | Ph3 | Y | Y | Y | - | Y |
| Startup beep/melody | - | Ph2 | Y | Y | Y | Y | Y |
| LED status indication | Y | | Y | - | Y | Y | Y |
| HW watchdog (ATA6847) | - | Ph1 | - | - | - | - | - |
| Register write protection | - | Ph1 | - | - | - | - | - |
| Trap handlers | Y | | - | - | - | - | Y |

### Unique Garuda CK Advantages (vs all competitors)
1. **ATA6847 HW ILIM** — cycle-by-cycle current chopping in gate driver, faster than any software
2. **ATA6847 VDS short circuit** — hardware desaturation detection, sub-µs response
3. **640ns HR commutation timer** — 78x better timing resolution than Timer1
4. **Hybrid ZC (100kHz poll + HR scheduling)** — best of both worlds
5. **GSP protocol with board detection** — single GUI for multiple board variants
6. **Motor line diagnostics** — pre-flight winding check (unique to ATA6847)
7. **2-shunt phase current + reconstruction** — full current visibility without 3 shunts

---

## 2. Production ESC Feature Roadmap

### Phase 1: Flight-Ready Core (4-6 weeks)
*Goal: ESC can fly a drone safely with DShot input*

| # | Feature | Effort | Priority | Depends On |
|---|---------|--------|----------|------------|
| 1.1 | **Software PD current limit** | 3 days | Critical | Current sensing ✓ |
| | AM32-style PD controller at 1kHz. Caps duty setpoint. | | | |
| | Configurable limit per profile (e.g., 20A for A2212). | | | |
| | HW ILIM is safety net, SW limit is operating range. | | | |
| 1.2 | **DC average current tracking** | 1 day | High | IBus ✓ |
| | IIR filter: `ibus_dc = (ibus_dc * 15 + ibus_peak * duty/100) / 16` | | | |
| | Add to snapshot. Matches bench supply reading. | | | |
| 1.3 | **DShot input** (300/600) | 5 days | Critical | — |
| | SCCP4 input capture on RD8 (RP57). Decode DShot frame. | | | |
| | Map 48-2047 → throttle. Handle DShot commands (beep, dir, etc.) | | | |
| | Feature flag: FEATURE_RX_DSHOT. | | | |
| 1.4 | **RPM-dependent duty ramp** | 2 days | High | — |
| | Max duty = duty_spup + (RPM / duty_ramp_erpm) * (100 - duty_spup). | | | |
| | Prevents locked-rotor overcurrent at startup. | | | |
| | Configurable: duty_spup (5-25%), duty_ramp_erpm (5k-30k). | | | |
| 1.5 | **ATA6847 thermal monitoring** | 1 day | High | nIRQ ✓ |
| | Poll DSR1 bits 3-6 every 200ms. Reduce duty on pre-warning. | | | |
| 1.6 | **ATA6847 hardware watchdog** | 1 day | High | — |
| | WDCR1 timeout mode, 100ms period. Kick in main loop. | | | |
| | MCU hang → ATA6847 resets MCU → boots to safe IDLE. | | | |
| 1.7 | **Register write protection** | 0.5 day | Medium | — |
| | RWPCR=1 after init. Unlock only for runtime changes. | | | |
| 1.8 | **Locked rotor protection** | 1 day | High | IBus ✓ |
| | If IBus > threshold AND eRPM < stall_threshold for > 500ms → fault. | | | |
| 1.9 | **GSP parameter system** (basic) | 3 days | High | GSP ✓ |
| | GET_PARAM, SET_PARAM for ~15 CK-specific params. | | | |
| | ILIM DAC, deglitch FL, timing advance, slew rates, Vbus thresholds. | | | |
| 1.10 | **GUI CK Dashboard** | 3 days | High | GSP ✓ |
| | Board detection → CK UI. Gauges: eRPM, Ia/Ib/IBus, Vbus, duty, ILIM. | | | |
| | Motor start/stop buttons. Throttle slider. | | | |
| 1.11 | **GUI CK Parameter Panel** | 2 days | High | 1.9 |
| | Grouped parameter editor. Save/load from firmware. | | | |
| 1.12 | **Motor spin test** (from GUI) | 0.5 day | Medium | 1.10 |
| | Low-duty spin for direction verification. | | | |

**Phase 1 Total: ~23 days**

---

### Phase 2: Robustness & Tuning (4-6 weeks)
*Goal: Reliable in all conditions, auto-tuning, telemetry to FC*

| # | Feature | Effort | Priority |
|---|---------|--------|----------|
| 2.1 | **Motor auto-detect** (Rs/Ls/Ke) | 5 days | High |
| | Inject test current → measure Rs. AC injection → measure Ls. | | |
| | Spin at low duty → measure Ke from BEMF. Auto-set profile. | | |
| 2.2 | **DShot bidirectional telemetry** | 3 days | High |
| | eRPM on EDT frames. 10-byte KISS frame on telemetry wire. | | |
| | Voltage, current, temperature, eRPM, CRC. | | |
| 2.3 | **EEPROM/flash persistence** | 3 days | High |
| | NVM flash driver for dsPIC33CK. Page erase/write. | | |
| | V1 schema: motor profile + all runtime params. CRC protection. | | |
| 2.4 | **Soft current derating** | 2 days | High |
| | VESC-style: reduce max current linearly from Imax at Tstart | | |
| | to 0 at Tmax. Smoother than hard cutoff. | | |
| 2.5 | **Auto timing advance** | 2 days | Medium |
| | Monitor demag events. Increase advance when demag detected, | | |
| | decrease when clean. BLHeli_32 approach. | | |
| 2.6 | **Demag compensation** | 2 days | Medium |
| | Detect demag during ZC sensing. Extend blanking or cut power | | |
| | before next commutation. BLHeli_S approach. | | |
| 2.7 | **Motor line diagnostics** | 2 days | Medium |
| | ATA6847 MLDCR: pre-flight winding check. Open/short detect. | | |
| | Run before ARMED. Fault if abnormal. | | |
| 2.8 | **Fault context capture** | 2 days | Medium |
| | On fault: capture snapshot (currents, voltage, speed, duty, step, | | |
| | ATA6847 SIR1-5, timestamp). Store in flash. Read via GSP. | | |
| 2.9 | **Startup beep/melody** | 1 day | Low |
| | PWM tone generation. Configurable startup melody. | | |
| | Power-on beep, arming beep, error tones. | | |
| 2.10 | **Config backup/restore** | 1 day | Medium |
| | GSP command: dump all params as binary blob. Restore from blob. | | |
| | GUI: save/load config file. | | |
| 2.11 | **GUI scope/charts** | 3 days | Medium |
| | Time-series plots: currents, speed, duty, ZC timing. | | |
| | Multiple channels, configurable timebase. | | |
| 2.12 | **GUI motor setup wizard** | 2 days | Medium |
| | Enter KV/PP → auto-calc timing advance, current limits, startup. | | |
| | Or run auto-detect and populate from measurements. | | |
| 2.13 | **Speed PI controller** | 2 days | Medium |
| | Closed-loop speed control (RPM target from DShot/GSP). | | |
| | Governor mode for constant-RPM applications. | | |

**Phase 2 Total: ~30 days**

---

### Phase 3: Production Hardening (4-6 weeks)
*Goal: Ready for volume production, FC integration, bootloader*

| # | Feature | Effort | Priority |
|---|---------|--------|----------|
| 3.1 | **Bootloader** | 5 days | Critical |
| | Self-programming via UART (GSP protocol extension). | | |
| | No debugger needed for firmware updates. | | |
| | Checksum verification, rollback on failure. | | |
| 3.2 | **4-way FC passthrough** | 3 days | High |
| | FC sends MSP_PASSTHROUGH → bit-bangs motor signal wire. | | |
| | ESC bootloader protocol compatibility (AM32 BLB). | | |
| | Allows configuration via Betaflight/ArduPilot without USB. | | |
| 3.3 | **KISS telemetry** | 2 days | High |
| | 10-byte frame: temp, voltage, current, consumption, eRPM, CRC. | | |
| | 115200 baud on telemetry wire. FC reads for OSD/logging. | | |
| 3.4 | **Variable PWM frequency** | 2 days | Medium |
| | Adjust PWM freq with RPM: lower at idle (less switching loss), | | |
| | higher at speed (better current control). AM32 approach. | | |
| 3.5 | **Sinusoidal startup** | 3 days | Medium |
| | V/f sine drive for smooth low-speed startup. | | |
| | Better for heavy props and high-inertia loads. | | |
| 3.6 | **Adaptive dead time** | 1 day | Low |
| | ATA6847 GDUCR3: ADDTHS/ADDTLS = 01 (50-150ns). | | |
| | Requires oscilloscope validation. | | |
| 3.7 | **Production test mode** | 2 days | High |
| | Automated board test via GSP: ADC cal, current cal, motor spin, | | |
| | ATA6847 register verify, EEPROM write/read, fault inject. | | |
| 3.8 | **EMI optimization** | 2 days | Medium |
| | Slew rate tuning (GDUCR3). Spread-spectrum PWM (if dsPIC33CK | | |
| | supports it). PCB layout review for production board. | | |
| 3.9 | **Multi-motor GUI** | 2 days | Medium |
| | Configure 4 ESCs from one session. Individual or bulk settings. | | |
| 3.10 | **GUI firmware update** | 2 days | High |
| | WebSerial firmware flash via bootloader. Progress bar. | | |
| | Select hex file, verify, flash, reboot. | | |

**Phase 3 Total: ~24 days**

---

### Phase 4: Advanced Features (ongoing)
*Goal: Feature parity with VESC for advanced use cases*

| # | Feature | Effort | Notes |
|---|---------|--------|-------|
| 4.1 | FOC sensorless (SMO/PLL) | 4 weeks | Port from AK board (already working) |
| 4.2 | CAN bus / DroneCAN | 2 weeks | Multi-ESC, FC telemetry, param sync |
| 4.3 | Hall sensor support | 1 week | For sensored startup on large motors |
| 4.4 | Active braking | 3 days | Regenerative braking to battery |
| 4.5 | Power limiting (watts) | 2 days | Cap power = V × I instead of just I |
| 4.6 | Battery SOC estimation | 1 week | Cell count, voltage curve, consumption |
| 4.7 | Custom PCB design | 4 weeks | Production board, BOM optimization |

---

## 3. Architecture Principles

### From VESC (adopt)
1. **Soft derating, not hard cutoffs** — temperature and current limits should ramp down gradually, not trip instantly. Prevents mid-flight crashes.
2. **Fault context capture** — on every fault, snapshot ALL state (currents, voltage, duty, speed, temperature, fault registers). Critical for post-mortem debugging.
3. **Configuration persistence with CRC** — flash-stored config with schema versioning and checksum. Detect corruption on boot.
4. **Auto-detect everything** — motor R/L/Ke, battery cell count, current sensor offset. Zero manual config for basic operation.

### From AM32 (adopt)
5. **PD current limiter (Ki=0)** — proportional + derivative, no integral. Avoids windup during commutation transients. Simple and effective.
6. **50-sample SMA for current** — heavy filtering prevents noise trips. 50ms lag is acceptable for thermal protection.
7. **User-configurable PID gains** — expose P/I/D via GUI for advanced users.
8. **ESC configurator compatibility** — support 4-way interface for FC passthrough.

### From BLHeli (adopt)
9. **Demag compensation** — detect winding demagnetization during ZC sensing. Essential for high-current operation with props.
10. **Low RPM power protect** — RPM-dependent duty ramp prevents overcurrent during stall/startup.

### From ESCape32 (adopt)
11. **duty_spup / duty_ramp** — configurable startup power limit + RPM ramp. Simple and effective.
12. **Text CLI for debug** — keep our debug UART console alongside GSP. Useful for field debugging.

### Garuda-Specific (keep)
13. **ATA6847 hardware protection** — leverage every feature the gate driver offers. No other ESC has cycle-by-cycle HW ILIM.
14. **Hybrid ZC architecture** — 100kHz fast poll + 640ns HR scheduling. Best timing resolution in the industry.
15. **Dual-board GUI** — single React app supports AK (FOC) and CK (6-step) boards via board detection.
16. **Step-aware current reconstruction** — per-commutation-step IBus from 2 shunts. No Phase C shunt needed.

---

## 4. Critical Path

```
Phase 1 (flight-ready):
  Week 1-2: SW current limit (1.1) + DC avg tracking (1.2) + DShot (1.3)
  Week 3:   RPM duty ramp (1.4) + thermal (1.5) + watchdog (1.6)
  Week 4:   GSP params (1.9) + GUI dashboard (1.10)
  Week 5:   GUI params (1.11) + locked rotor (1.8) + testing
  Week 6:   Integration testing, prop testing, first flight attempt

Phase 2 (robustness):
  Week 7-8:  Auto-detect (2.1) + EEPROM (2.3) + DShot bidir (2.2)
  Week 9-10: Soft derating (2.4) + demag (2.6) + fault capture (2.8)
  Week 11-12: GUI scope (2.11) + wizard (2.12) + motor line diag (2.7)

Phase 3 (production):
  Week 13-14: Bootloader (3.1) + 4-way passthrough (3.2)
  Week 15-16: KISS telemetry (3.3) + production test (3.7)
  Week 17-18: Variable PWM (3.4) + sinusoidal startup (3.5)
```

---

## 5. Testing Strategy

### Unit Tests (Python, per feature)
- GSP protocol: gsp_ck_test.py (existing, extend per feature)
- Current sensing: verify scaling at known loads
- ILIM: verify trip point at multiple DAC values
- DShot: inject frames, verify decode
- Parameter system: set/get/save/load round-trip

### Integration Tests (motor running)
- Startup → CL → full speed → decel → stop (no desync)
- ILIM activation during high throttle (motor keeps running)
- Fault injection: unplug motor wire → detect stall → recover
- Thermal: run at max continuous → verify derating kicks in
- Power cycle: verify EEPROM params persist across reset

### Flight Tests
- Static thrust: verify current matches expected for prop/motor combo
- Hover: 4 ESCs on frame, manual throttle, verify stable hover
- Aggressive maneuver: pitch/roll snap, verify no ILIM false trips
- Failsafe: kill throttle → motor stops, resume → motor restarts

### Compliance Tests (production)
- EMI/EMC: conducted/radiated emissions measurement
- Temperature: ambient range -10°C to +50°C
- Vibration: motor vibration on PCB (check solder joints)
- Voltage range: 3S-4S LiPo (9.6V-16.8V) full operating range

---

## 6. Hardware Roadmap

### Current: EV43F54A Evaluation Board
- dsPIC33CK64MP205 + ATA6847 QFN40
- 3 × 3mΩ shunt resistors (RS1/RS2/RS3)
- Hurst or A2212 motor
- PKoB4 USB-UART for programming + GSP

### Next: Custom ESC PCB v1
- Same dsPIC33CK + ATA6847 (proven combination)
- 4-layer PCB, 30mm × 30mm target
- Single-connector motor + signal cable
- DShot signal input + telemetry output
- CAN bus pads (for v2)
- Temperature sensor (NTC on MOSFET)
- Optimized for A2212-class motors (12V, 20A burst)

### Future: High-Power ESC PCB v2
- dsPIC33CK256MP506 (more flash for FOC)
- ATA6847 + higher-current MOSFETs
- CAN bus connector
- 6S LiPo support (up to 25.2V)
- 40A+ continuous

---

## 7. Risk Assessment

| Risk | Impact | Mitigation |
|------|--------|------------|
| ATA6847 ILIM noise trips during flight | Motor stutters, altitude loss | Tune ILIMFLT, test with multiple props. SW limit as primary, HW as safety net. |
| DShot timing conflict with SCCP4 | ZC detection disrupted | Use separate SCCP for DShot IC. dsPIC33CK has SCCP1-4. |
| EEPROM/flash write corruption | Config lost on power cycle | CRC on stored config. Dual-bank with fallback. |
| Bootloader bricking | ESC unrecoverable without debugger | Keep SWD header on PCB. Bootloader in protected flash region. |
| EMI from PWM switching | Fails compliance testing | Spread-spectrum PWM, slew rate tuning, proper PCB layout. |
| Thermal shutdown during flight | ESC cuts out mid-air | Soft derating (Phase 2.4) — reduce power before shutdown. |
| ZC loss at high current (demag) | Desync → crash | Demag compensation (Phase 2.6). Tested with max-thrust prop. |

---

## 8. Metrics for Production Readiness

| Metric | Target | Current |
|--------|--------|---------|
| Desync events per 1000 flights | 0 | 0 (30s bench runs) |
| False ZC rate | < 0.1% | 0.02% |
| Startup success rate | > 99.5% | ~100% (bench) |
| Current limit accuracy | ±10% | ~15% (DAC empirical) |
| Telemetry dropout rate | < 0.1% | 0% (20Hz/10Hz) |
| Boot-to-ready time | < 500ms | ~200ms |
| Flash write endurance | > 10k cycles | TBD (not implemented) |
| Temperature derating onset | +10°C from max | TBD (not implemented) |
| DShot latency | < 50µs | TBD (not implemented) |
| CAN bus throughput | > 1000 msg/s | TBD (not implemented) |
