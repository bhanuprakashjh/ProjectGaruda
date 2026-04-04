# Testing Guide — Garuda CK Board ESC

## For Team Members: How to Test, Modify, and Debug

---

## 1. Hardware Setup

### Board: EV43F54A
- **MCU**: dsPIC33CK64MP205 (U2A, TQFP-48)
- **Gate Driver**: ATA6847 QFN40 (U1)
- **Power**: 5-32V on VBat terminal (J1)
- **Programming**: PKoB4 on-board debugger (USB micro-B, J5)
- **UART**: USB-UART adapter on J3 (RX=RC8, TX=RC9), **NOT** the PKoB4 USB

### Connections
```
Power Supply (+) ──→ VBat+ (J1 pin 1)
Power Supply (-) ──→ VBat- (J1 pin 2)
Motor Phase A ─────→ PhA (J2 pin 1)
Motor Phase B ─────→ PhB (J2 pin 2)
Motor Phase C ─────→ PhC (J2 pin 3)
USB-UART TX ───────→ RC8 (J3 RX)
USB-UART RX ───────→ RC9 (J3 TX)
USB-UART GND ──────→ GND (J3)
```

### Important Quirks
1. **Two USB ports**: PKoB4 (programming) and USB-UART (telemetry). They're different ports!
2. **After programming**: the USB-UART port may need unplug/replug to reconnect
3. **Serial port names**: typically `/dev/ttyACM0` = PKoB4, `/dev/ttyACM1` = UART. Check with `ls /dev/ttyACM*`
4. **pot_capture.py and GUI can't share the serial port** — close one before using the other

---

## 2. Selecting a Motor Profile

### In garuda_config.h line 17:
```c
#define MOTOR_PROFILE   1   /* 0=Hurst, 1=A2212, 2=2810 */
```

Change the number, rebuild, and flash. **All universal parameters (CLC, IC, advance, poll frequency) apply automatically.**

### Motor-specific parameters that matter:
| Parameter | Hurst (24V) | A2212 (12V) | 2810 (18-24V) |
|-----------|------------|-------------|---------------|
| MOTOR_POLE_PAIRS | 5 | 7 | 7 |
| MOTOR_KV | 149 | 1400 | 1350 |
| ALIGN_DUTY | 5% | 2.5% | 2.5% |
| RAMP_ACCEL_ERPM_S | 1500 | 1500 | 500 |
| VBUS_OV_THRESHOLD | ~28V | ~18V | ~40V |
| CL_IDLE_DUTY | 10% | 10% | 10% |

### Adding a new motor:
1. Copy an existing profile section in `garuda_config.h`
2. Change KV, pole pairs, startup parameters
3. Start conservative: slow ramp (500 eRPM/s), low align duty (2%)
4. Test with no load first, then add prop
5. If motor doesn't start: increase `ALIGN_DUTY` and `ALIGN_TIME_MS`
6. If motor desyncs at high speed: check Vbus OV threshold, try lower max pot

---

## 3. Running a Bench Test

### Basic pot test:
1. Flash firmware with correct motor profile
2. Connect power supply, motor, USB-UART
3. Start `pot_capture.py`:
   ```bash
   python3 tools/pot_capture.py --port /dev/ttyACM1 --duration 120
   ```
4. Press SW1 to start motor
5. Slowly increase pot — listen for smooth operation
6. Watch terminal output for eRPM, duty, timeouts
7. Press Ctrl+C to stop capture — CSV saved automatically

### What to look for:
- **Zero timeouts** at steady speed = good
- **Timeout bursts during pot changes** = normal (estimator lag)
- **Continuous timeouts at a speed** = problem (comparator noise at that speed)
- **eRPM jumping to 100k+ at low duty** = phantom commutation (bad)
- **Motor stopping and not restarting** = recovery failure

### GUI test:
1. Start GUI: `cd gui && npm run dev`
2. Open `http://localhost:5173/ProjectGaruda/` in Chrome
3. Click Connect → select UART port
4. Dashboard shows: eRPM, duty, current, Vbus, ZC diagnostics
5. **eRPM shows 0 at idle** — this is correct, updates when motor runs
6. Motor Test tab: automated throttle sweeps with CSV export

---

## 4. Interpreting Test Data

### Per-step ZC summary (printed at end of pot_capture):
```
Step | Phase | Polarity | Accepted | Timeouts
  0  |   C   | Rising   |    20574 |       0    ← GOOD: 0 timeouts
  5  |   B   | Falling  |    20572 |     537    ← BAD: step 5 has issues
```
- **Equal accepted counts** across steps = balanced detection
- **One step with many timeouts** = drive-vector-specific noise
- **Rising >> Falling timeouts** = polarity-dependent issue

### IC capture ratio:
- `ic_accepted` = poll path ZC acceptances
- `ic_false` = ZCs rejected by RecordZcTiming (should be ~0 in steady state)
- High ic_false at CL entry (first 100ms) is normal

### Current spikes:
- `ibus_mA > 20000` = potential desync, check if Vbus also drops
- Vbus sag + current spike = bench supply limitation, use LiPo

---

## 5. Troubleshooting

### Motor doesn't start
- Check Vbus on GUI dashboard (should show supply voltage)
- Check `ataStatus` in ZC diagnostics (0x00 = OK)
- If ATA6847 fault: power cycle the board, check for short circuits
- Try increasing `ALIGN_TIME_MS` (200→400) and `ALIGN_DUTY`

### Motor starts but desyncs immediately
- Check motor wiring (phase order A/B/C)
- Try swapping any two motor wires (reverses direction)
- Reduce `RAMP_ACCEL_ERPM_S` (slower ramp)
- Check if pot is at zero when starting (should be)

### Motor runs but rough above certain speed
- Check the pot_capture CSV for timeout patterns
- If timeouts at specific duty% (50%, 80%): CLC clock position issue (known)
- If timeouts with Vbus swings: power supply issue (use LiPo for 24V)
- If step-specific timeouts: drive-vector noise (motor/voltage dependent)

### Board hangs after fault
- Press SW1 to clear fault and restart
- If SW1 doesn't work: power cycle
- Check fault code in GUI: NONE/OV/UV/STALL/DESYNC/STARTUP_TO/ATA6847

### GUI doesn't show eRPM
- Make sure you're on **localhost** not GitHub Pages (GitHub Pages version may be stale)
- Run `cd gui && npm run dev` and open `http://localhost:5173/ProjectGaruda/`
- Check browser console (F12) for connection errors

---

## 6. Modifying for Different Motors

### Low KV motors (< 500 KV, e.g., Hurst 149KV)
- Higher `ALIGN_DUTY` (5-10%) — high Ls needs more current for alignment
- Slower ramp (`RAMP_ACCEL_ERPM_S = 1000-1500`)
- Higher `ZC_BLANK_FLOOR_US` (30-40µs) — more blanking for high-Ls ringing
- Lower max eRPM (15-30k)

### High KV motors (> 1500 KV)
- Lower `ALIGN_DUTY` (1-2%) — low Rs means high current at low duty
- Faster ramp possible (`RAMP_ACCEL_ERPM_S = 2000-3000`)
- Lower `ZC_BLANK_FLOOR_US` (15-20µs) — low Ls, fast BEMF
- Watch for VDS faults at 24V (increase `SCTHSEL` in hal_ata6847.c)

### Different pole counts
- Change `MOTOR_POLE_PAIRS` — this affects eRPM calculation and mech RPM display
- Advance scaling is automatic (fraction of interval, not degrees vs eRPM)

### Different voltages
- Adjust `VBUS_OV_THRESHOLD` and `VBUS_UV_THRESHOLD`
- At 24V+: increase `SCTHSEL` to 7 (2000mV) and `EGBLT` to 15 (3.75µs)
- At 6-12V: default settings work fine

---

## 7. PWM Mode Selection

### Complementary (default, PWM_DRIVE_UNIPOLAR=0)
- Proper motor braking — pot directly controls speed
- Needs CLC for clean ZC — 2 switching edges per cycle
- Best for prop testing (linear throttle response)

### Unipolar (PWM_DRIVE_UNIPOLAR=1)
- 34x fewer ZC timeouts — only 1 switching edge
- No braking — motor free-wheels, duty controls power not speed
- Needs speed PID for production use (not yet implemented)
- Lower `CL_IDLE_DUTY` (6%) and `MAX_DUTY` (50%) for no-load

To switch: change `PWM_DRIVE_UNIPOLAR` in `garuda_config.h` and rebuild.

---

## 8. Key Configuration Reference

All in `garuda_config.h`:

```c
// Motor selection
MOTOR_PROFILE           0/1/2       // Hurst/A2212/2810

// Features (leave at defaults unless debugging)
FEATURE_IC_ZC           1           // SCCP1 fast poll
FEATURE_IC_ZC_CAPTURE   1           // SCCP2 IC timestamps
FEATURE_CLC_BLANKING    1           // CLC D-FF noise filter
PWM_DRIVE_UNIPOLAR      0           // 0=complementary, 1=unipolar

// Tuning
TIMING_ADVANCE_LEVEL    2           // 0-3 (0°/7.5°/15°/22.5°)
PWMFREQUENCY_HZ         24000      // PWM switching frequency
ZC_POLL_FREQ_HZ         210526     // Non-integer poll (anti-aliasing)
DUTY_RAMP_ERPM          60000      // Duty governor threshold
CL_IDLE_DUTY_PERCENT    10         // Minimum duty in CL (complementary)

// ATA6847 (in hal_ata6847.c)
EGBLT                   12-15      // Edge blanking (250ns units)
SCTHSEL                 5-7        // VDS threshold (for voltage)
```

---

## 9. Firmware Update Procedure

1. Pull latest from GitHub: `git pull origin main`
2. Set `MOTOR_PROFILE` in `garuda_config.h`
3. Build from MPLAB X or CLI
4. Flash via MPLAB X (Build & Program) or MDB CLI
5. **After programming**: unplug/replug USB-UART if telemetry doesn't connect
6. Test with pot_capture.py to verify

---

## 10. Safety Notes

- **Never run motor without prop guard** during bench testing
- **Start with low voltage** (12V) before testing at 24V
- **Keep pot at zero** when pressing SW1 to start
- **Watch Vbus on GUI** — if it drops below 50% of supply, bench supply is overloaded
- **Current spikes > 30A** can damage the board — use LiPo for high-current testing
- **ATA6847 fault** (LED_FAULT on) — power cycle before retrying
