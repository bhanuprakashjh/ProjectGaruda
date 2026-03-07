# ProjectGaruda Tools

Python scripts for testing, debugging, and tuning the dsPIC33AK ESC firmware
via the GSP (Garuda Serial Protocol) interface.

All scripts require `pyserial`: `pip install pyserial`

## Scripts

### foc_logger.py — FOC Telemetry Logger

Captures all FOC snapshot fields to timestamped CSV files for closed-loop
AI-assisted debugging. Supports both streaming (50 Hz telemetry frames)
and polling (GET_SNAPSHOT) modes.

```
python3 tools/foc_logger.py [--port /dev/ttyACM0] [--rate 50] [--duration 30]
```

**Output:** `logs/foc_YYYYMMDD_HHMMSS.csv` with 60+ columns including:
- ESC state, fault code, throttle, duty, Vbus
- FOC fields: Id, Iq, theta, omega, Vd, Vq, observer angle, sub-state
- Derived: RPM, power (Vq*Iq + Vd*Id)

**Live display** at 5 Hz showing key FOC variables. First snapshot is
hex-dumped for format debugging.

### gsp_test.py — GSP Protocol Test Suite

Comprehensive test suite for the GSP protocol. Tests all 20 commands
including ping, snapshot, params, profiles, EEPROM save/load, telemetry
streaming, and FOC-specific commands.

```
python3 tools/gsp_test.py [--port /dev/ttyACM0]
```

### gsp_detect.py — Motor Auto-Detect Tool

Sends the GSP auto-detect command (0x20) and monitors the detect phases
(Rs, Ls, lambda measurement). Reads back the measured motor parameters
after detection completes.

```
python3 tools/gsp_detect.py [/dev/ttyACM0]
```

### gsp_rx_test.py — RX Input Testing

Tests the RX (receiver) input subsystem — PWM and DShot protocol
decoding, link state monitoring, and GSP RX status queries.

```
python3 tools/gsp_rx_test.py [--port /dev/ttyACM0]
```

## CSV Log Analysis

The CSV files from `foc_logger.py` are designed for analysis with AI tools.
Example Python analysis:

```python
import csv
with open('logs/foc_20260307_233509.csv') as f:
    r = list(csv.DictReader(f))
    for row in r:
        if row['focSubName'] == 'closed_loop':
            print(f"t={row['time_s']}  Iq={row['focIqMeas']}  RPM={row['focRPM']}")
```

Key columns for FOC debugging:
- `focSubName`: idle / armed / align / if_ramp / closed_loop
- `fault_name`: NONE / BOARD_PCI / OVERCURRENT / STALL / etc.
- `focIqMeas`, `focIdMeas`: measured d/q currents (A)
- `focVq`, `focVd`: voltage commands (V)
- `focOmega`: PLL speed estimate (electrical rad/s)
- `focTheta`, `focThetaObs`: forced angle vs observer angle (rad)
