# Reproducing the Commutation Kickback Experiments

This guide walks through flashing the instrumented firmware on each
board and collecting the phase-current peak data that produced
`commutation_kickback_findings.md`.

## What you need

### Hardware
- **AKESC experiment** (A2212 kickback case):
  - MCLV-48V-300W evaluation board (Microchip)
  - dsPIC33AK128MC106 (fitted on MCLV)
  - A2212 1400KV motor, 7PP, 65 mΩ/30 µH
  - 12 V bench supply (≥ 10 A CC limit — protects during startup faults)
  - USB cable to the MCLV's onboard ICSP (PKoB)

- **CK experiment** (2810 drive-current case):
  - EV43F54A evaluation board (Microchip)
  - dsPIC33CK256MP503 (fitted on EV43F54A)
  - ATA6847 integrated gate driver (on-board)
  - 2810 1350KV motor, 7PP, 50 mΩ/25 µH
  - 24 V bench supply (≥ 15 A CC limit)
  - USB cable to EV43F54A's onboard ICSP (PKoB)

### Software
- MPLAB X IDE v6.30+
- XC-DSC compiler v3.30+ (dsPIC33AK path) **and/or** XC16 v2.10+
  (dsPIC33CK path)
- Python 3.10+ with `pyserial` for the logger tools
- GNU `make`

### Motor safety
- Secure the motor to a bench fixture. Even at idle duty the bare
  motor will spin up to 10-20 k RPM mechanical. With a prop, treat
  the entire test rig as a live weapon.
- Bench supplies with current-limiting clamps are strongly preferred
  — the tests intentionally walk into the limit.

## AKESC (A2212/12V) — the kickback reduction test

### Build

```bash
cd dspic33AKESC/dspic33AKESC.X
make -f nbproject/Makefile-default.mk CONF=default build
```

Hex lands at `dist/default/production/dspic33AKESC.X.production.hex`.
The project currently defaults to `MOTOR_PROFILE=1` (A2212).
`DEADTIME_NS=300` per commit `17bf682` — this is the post-
experiment sweet spot. To reproduce the before/after comparison,
edit `dspic33AKESC/garuda_config.h` line ~149 and flip it between
500 and 300 ns.

### Flash

Open the project in MPLAB X and program with the onboard PKoB.
Or via MDB CLI:
```bash
/opt/microchip/mplabx/v6.30/bin/mdb.sh
> device dsPIC33AK128MC106
> hwtool PKOB4
> program dspic33AKESC/dspic33AKESC.X/dist/default/production/dspic33AKESC.X.production.elf
> run
> quit
```

### Run

```bash
python3 tools/step6_logger.py --port /dev/ttyACM0
```

The logger prints state + phase currents at 5 Hz and writes
`logs/step6_YYYYMMDD_HHMMSS.csv`. Fields of interest:
- `dutyPct`: applied PWM duty %
- `eRPM`: electrical RPM
- `Ia_now(pk)`, `Ib_now(pk)`: instantaneous + rolling-max peak
- `Ibus(pk)`: bus current rolling-max peak
- `AT-FAULT`: frozen snapshot from the moment of BOARD_PCI trip

### Expected behaviour

Sweep the pot from 0 to max slowly (10 seconds). You should see
the A2212 accelerate through OL_RAMP → CL and plateau near 118 k
eRPM. At the plateau read `Ibus(pk)`:
- With `DEADTIME_NS=500`: ~10-12 A peak
- With `DEADTIME_NS=300`: ~5-6 A peak

That is the 55 % kickback reduction documented in the findings
paper.

## CK (2810/24V) — the physics validation test

### Build

```bash
cd PATA6847/garuda_6step_ck.X
make -f Makefile-cli.mk MOTOR_PROFILE=2
```

(`MOTOR_PROFILE=2` is the 2810 profile; `=1` is A2212; `=0` is Hurst.)
Hex lands at `dist/default/production/garuda_6step_ck.X.production.hex`.

The project defaults to `FEATURE_V4_SECTOR_PI=1`. Do not disable
this — V4 is what got the motor to 196 k eRPM in the first place
and is the only path that has the phase-current peak instrumentation
wired on this board.

### Flash

Via MPLAB X (recommended — the MCC plugin can miss on CLI):
1. Open `PATA6847/garuda_6step_ck.X` in MPLAB X
2. Set configuration to `default`
3. Set `Active Project` and `Make and Program`

### Run

```bash
python3 tools/ck_logger.py --port /dev/ttyACM0
```

Fields rendered:
- `eRPM`: electrical RPM (eRpm from PI filter)
- `dutyPct`: applied PWM duty %
- `Ia +nnn/+max/-min|pk|`: instantaneous Ia / rolling max / rolling min
  / absolute-peak magnitude
- `Ib +nnn/...`: same for Phase B
- `caps/pi`: hardware ZC captures accepted / PI iterations

### Expected behaviour

Sweep pot 0 → max over 20-30 seconds. Motor should ramp through
OL → CL and climb to 196 k eRPM at 99 % duty. At each eRPM
plateau capture Ia/Ib peaks:

| eRPM    | Duty  | Ia pk (counts) | Ia pk (≈ A) |
|---------|-------|-----------------|-------------|
| 80 k    | 36 %  | ~23,000         | ~23 A       |
| 100 k   | 46 %  | ~26,000         | ~26 A       |
| 150 k   | 67 %  | ~24,000         | ~24 A       |
| 196 k   | 99 %  | ~22,000         | ~22 A       |

Brief ADC saturation (±32,752 counts ≈ 34 A) is expected during
heavy acceleration transients. ATA6847 protection does not fire.
Compare these numbers to AKESC/MCLV running the same 2810 motor
on 24 V, which trips at ~78 k eRPM on the MCLV board's U25B
comparator (see `Ibus` pk ≈ 22 A in the AT-FAULT snapshot of
`logs/step6_20260420_191012.csv`).

## Data you should collect to validate / extend the work

- **AKESC deadtime sweep**: 500 → 300 → 200 ns at A2212/12V top
  throttle. Capture `logs/step6_*.csv` for each. Plot
  `Ibus(pk)` vs duty cycle.
- **AKESC 2810/24V**: document where BOARD_PCI fires (eRPM + duty).
- **CK 2810/24V**: full throttle sweep. Confirm motor reaches
  ~196 k at 99 % duty and capture Ia/Ib peaks at each eRPM.
- **CK 2810/24V with prop**: predicted to clamp equilibrium below
  80 k eRPM at ~10-15 A steady. Not yet measured.
- **CK A2212/12V**: matching profile to the AKESC A2212 test.
  Confirm whether the 300 ns deadtime reduction also shows on CK,
  or whether the MCLV shunt/comparator path was the differentiator.
  Not yet measured.

## Coordinate system & gotchas

- **ADC scaling**: both boards use signed 12-bit fractional ADC.
  AKESC has OA1 at 25× on 3 mΩ → ~75 counts/A. CK has OA2/OA3 at
  16× on 3 mΩ → ~1000 counts/A (different ADC format). If the
  logger output looks wildly out of scale, check the scale constant
  at the top of the Python tool.
- **Ibus sampling path differs across boards**:
  - AKESC samples OA1 (bus shunt) directly via AD1CH3 at
    PG1TRIGA (mid-ON, 24 kHz).
  - CK reconstructs Ibus in software from Ia/Ib and the active PWM
    phase per sector (the V4 logger currently emits 0 for the ibus
    peak field; host can compute from Ia/Ib extrema).
- **MPLAB X project files**: both projects commit `nbproject/` and
  `dspic33AKESC.X/`/`garuda_6step_ck.X/` with their Makefile-default
  plus configurations.xml. Cloning either repo + opening the .X in
  MPLAB should just build.
- **First-time ATA6847 register config on CK**: the ATA6847 needs
  a short SPI configuration sequence at boot. The CK firmware does
  this in `HAL_ATA6847_Init`. If you get persistent FAULT_ATA6847
  at boot, check the SPI wiring between the dsPIC33CK and the
  ATA6847 — the EV43F54A schematic is in
  `PATA6847/EV43F54A_SCH.PDF`.

## Where the data lives

Raw logs from the sessions that produced the findings paper:

- `logs/step6_20260420_191012.csv` — AKESC 2810/24V bare,
  BOARD_PCI at 78 k eRPM, 21.9 A Ibus pk
- `logs/step6_20260420_195948.csv` — AKESC A2212/12V bare,
  DEADTIME_NS=300, 108 k eRPM at 95 % duty with ~5 A Ibus pk
- `logs/ck_20260421_103443.csv` — CK 2810/24V bare, V4,
  196 k eRPM at 99 % duty with ~22 A Ia pk

CSVs are committed at the session's commit; future experimenters
can plot them directly without re-running the hardware tests.

## Contributing

If you reproduce this on a different board or motor combination,
append a row to the table in `commutation_kickback_findings.md`
with your data and open a PR. Particularly interesting:
- 5010/12V bare (would fill in the BEMF/Vbus = 0.5 middle regime)
- Any motor with significantly different L/R time constant
- Boards with adjustable U25B-equivalent comparator threshold
  (lets us sweep the regime boundary directly)
