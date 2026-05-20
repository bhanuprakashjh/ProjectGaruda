# AN1078 SMO FOC Port — AK ATA6847L Board

**Status**: Working. Peak observed: **211k eRPM** (22,100 rad/s elec) on
PRODRONE 2810 1350KV @ 24V, sensorless SMO, GUI-driven start/stop.

The PATA6847 AK board (dsPIC33AK128MC106 + EV43F54A + ATA6847L gate
driver) can now run either:

- The legacy **6-step trapezoidal** path (sector_pi + BEMF ZC), or
- The new **AN1078 sensorless SMO FOC** path (sliding-mode observer
  + d/q current PIs + space-vector PWM).

Mode selection is a single compile-time flag in `garuda_config.h`:

```c
#define FEATURE_FOC_AN1078  0   // 0 = 6-step, 1 = FOC
```

Both modes share the same firmware tree but produce mutually-exclusive
binaries — see [Hard isolation](#hard-isolation).

---

## Quick start

### Build & flash

**FOC mode:**
1. Edit `garuda_config.h` line 20: `#define FEATURE_FOC_AN1078  1`
2. MPLAB X → Build & Program.
3. Optional CLI: `make -f nbproject/Makefile-default.mk MP_EXTRA_CC_PRE="-DFEATURE_FOC_AN1078=1"`

**6-step mode:**
1. Same file, set the flag to `0` (default).
2. Build & Program.

### Connect

| Tool | Works in 6-step | Works in FOC | Notes |
|---|---|---|---|
| GUI (`gui/`, Vite+React) | ✓ | ✓ | Auto-detects mode via `featureFlags` bit 23 |
| `tools/pot_capture.py` | ✓ | × | 6-step snapshot parser only |
| `tools/foc_capture.py` | × | ✓ | Refuses to run if `featureFlags` bit 23 unset |
| `tools/foc_debug_watch.py` | only with `FEATURE_GSP=0` | only with `FEATURE_GSP=0` | Text UART debug |

The GUI is the primary tool. Both Python scripts mirror its protocol for
CSV-logging during bench tests.

---

## Architecture

### Source files

```
foc/
├── an1078_motor.{c,h}     AN1078 state machine (LOCK / OL_RAMP / CL / FAULT)
├── an1078_smc.{c,h}       Sliding-mode observer (current + BEMF model)
├── an1078_params.h        Tunables (Rs, Ls, λ, Ki/Kp, handoff thresholds)
├── pll_estimator.{c,h}    Angle PLL (cross-product discriminator + PI)
├── foc_runtime.{c,h}      Boot init, SW1/SW2 entry, ADC ISR hook
├── foc_shim_gsp.h         Stub `gspParams` until GUI param wiring lands
├── foc_types.h            ThreePhase/AlphaBeta/DQ structs
```

All `.c` files are wrapped in `#if FEATURE_FOC_AN1078 ... #endif` at the
file level, so the FOC=0 build emits zero FOC bytes (verified via
`xc-dsc-nm`).

### Boot flow (FOC mode)

```
main()
  ├── CLOCK_Initialize / SetupGPIOPorts / HAL_UART_Init
  ├── HAL_SPI_Init / HAL_ATA6847_Init        (shared with 6-step)
  ├── HAL_OPA_Init / HAL_ADC_Init / HAL_PWM_Init
  ├── HAL_Timer1_Init / GarudaService_Init   (sector_pi stays at running=false)
  ├── FOC_Init                               (AN_MotorInit)
  └── main loop
        ├── BoardService (SW1/SW2 debounce)
        ├── on SW1 press → FOC_StartMotor
        │     ├── ATA6847 EnterGduNormal
        │     ├── PWM enable + override-assert (FETs OFF for cal)
        │     ├── HAL_ADC_InterruptEnable
        │     └── gEscState = ESC_ARMED
        ├── on SW2 press → FOC_StopMotor
        └── ~10 Hz: FOC_DebugPrint / FOC_BlinkTick
```

### ADC ISR (30 kHz, PWM-triggered)

```c
_AD1CH4Interrupt:
    raw_ia/ib/ibus/vbus/pot      ← latch ADC buffers
    gIa_mA / gIb_mA / gIbus_mA  ← convert to mA (legacy 6-step telemetry)
#if FEATURE_FOC_AN1078
    FOC_AdcIsrTick(raw_ia_u, raw_ib_u, raw_vbus, raw_pot)
      ├── arm AN_MotorStart when (state==ARMED && mode==STOPPED && cal_done)
      ├── AN_MotorFastTick(...)
      ├── on first LOCK tick → HAL_PWM_ReleaseAllOverrides
      ├── HAL_PWM_SetDutyFloat3Phase(da, db, dc)
      └── mode mirror → gEscState
#endif
```

### Hard isolation

`FEATURE_FOC_AN1078` is the master switch. All 6-step source files are
also wrapped at the file level (`motor/sector_pi.c`, `motor/commutation.c`,
`hal/hal_com_timer.c`, `hal/hal_ptg.c`) so the FOC build doesn't link
them. Verified with `xc-dsc-nm` symbol audit:

| Build | 6-step symbols | FOC symbols |
|---|---|---|
| FOC=0 | 43 | **0** |
| FOC=1 | **0** | 26 |

Saves ~8 KB of flash and removes the runtime "is dormant code firing?"
question.

---

## Tunable parameters (`foc/an1078_params.h`)

| Param | AK Port Value | Source-Board Value | Rationale |
|---|---|---|---|
| `AN_FS_HZ` | `(float)PWMFREQUENCY_HZ` | hardcoded 45000 | Auto-derives from PWMFREQUENCY_HZ_FOC |
| `AN_VBUS_V_PER_COUNT` | 0.0128906 | 0.01870189 | AK board's 16:1 divider (vs MCLV 23.2:1) |
| `AN_CURRENT_A_PER_COUNT` | 0.01076636 | 0.01076636 | matches MCLV within 0.1% |
| `AN_END_SPEED_RPM_MECH` | 4000 | 2000 | AK at 30 kHz needs ~2× more BEMF for observer to lock cleanly |
| `AN_HANDOFF_DWELL_TICKS` | 1500 | 2400 | 50 ms at 30 kHz; was 100 ms at 24 kHz |
| `AN_OVER_CURRENT_LIMIT` | 18.0 A | 12.0 A | Prop testing — at 12 A speed PI saturated at ~64k eRPM |
| `AN_SMC_KSLIDE` | 2.5 V | 2.5 V | Source-board tuning (works on this motor) |
| `AN_SMC_THETA_OFFSET_BASE` | 0.349 rad (20°) | 0.349 rad | Same |
| `AN_OL_RAMP_RATE_RPS2` | 1000 | 1000 | Same |

### PWM frequency (separate for the two modes)

In `garuda_config.h`:

```c
#define PWMFREQUENCY_HZ_6STEP   60000U   // 60 kHz (proven at 225k eRPM)
#define PWMFREQUENCY_HZ_FOC     30000U   // 30 kHz (FOC first-light)
```

`PWMFREQUENCY_HZ` is `#if`-selected from these. `AN_FS_HZ`, `AN_IRP_PERCALC`,
`LOOPTIME_TCY`, `MIN/MAX_DUTY` all derive — bump one number, everything
follows.

To test FOC at 60 kHz for cleaner high-speed CL:

```bash
make MP_EXTRA_CC_PRE="-DPWMFREQUENCY_HZ_FOC=60000 -DFEATURE_FOC_AN1078=1"
```

(Re-tune handoff dwell/gate after the change.)

---

## GSP protocol unification

Single firmware codebase, two distinct GUI presentations:

| Mode | `boardId` | `featureFlags` bit 23 | GUI uses | Snapshot |
|---|---|---|---|---|
| 6-step | `0x0002` (CK) | clear | `decodeCkSnapshot` (48-byte) | 186-byte legacy frame |
| FOC | `0x0001` (MCLV/AK) | **set** | `decodeSnapshot` (FOC-aware) | 114-byte source-layout |

Both modes use the same `GSP_CMD_GET_SNAPSHOT` / `TELEM_FRAME` commands.
The handler dispatches based on `FEATURE_FOC_AN1078`:

- **FOC `TELEM_FRAME`**: `[u16 seq LE][114 B snapshot]`. The GUI strips the
  seq via `payload.slice(2)` (matches source-board format). Snapshot
  field offsets match source's `GSP_SNAPSHOT_T` exactly:
  `focIdMeas@68, focIqMeas@72, focTheta@76, focOmega@80, focVbus@84,
  focIa@88, focIb@92, focThetaObs@96, focVd@100, focVq@104`.
- **FOC `GET_SNAPSHOT`**: raw 114-byte snapshot (no seq).
- **6-step paths**: unchanged.

`BuildFocSnapshot()` in `gsp_commands.c` is shared between both response
paths.

`GSP_CMD_START_MOTOR` / `STOP_MOTOR` route to `FOC_StartMotor` /
`FOC_StopMotor` in FOC mode, `GarudaService_StartMotor` /
`GarudaService_StopMotor` in 6-step mode. Same wire-level command.

---

## Bench results (24V / 2810 1350KV / no-load)

```
[16:14:02.893] *F m=CLOSED_LOOP w=2822.8 eRPM=26956 Id=+0.45 Iq=+12.43 cal=1 vbus=24.72V
...           (throttle ramped up)
[16:14:11.965]  F m=CLOSED_LOOP w=21900 eRPM≈210000 Id=-2 Iq=+5 cal=1 vbus=24.5V
[16:14:18.460]  F m=CLOSED_LOOP w=6880    eRPM=65656  Id=-0.4 Iq=+1.5 vbus=24.7V
[16:14:26.09 ] *F m=STOPPED  (SW2 pressed)
```

Captured live via `tools/foc_capture.py /dev/ttyACM1` → `foc_run_HHMMSS.csv`.

### Bench results with prop (24V, AN_OVER_CURRENT_LIMIT=18A)

At 18 A `AN_OVER_CURRENT_LIMIT`, prop equilibrium settles at **~80k eRPM**
under load (45% throttle), Iq peaks 18-22 A. Speed PI hits the clamp;
higher speed needs less prop drag or further (risky) current bump.

---

## Common gotchas (caught during bring-up)

1. **`FEATURE_GSP` and UART debug are mutually exclusive on UART1.**
   When `FEATURE_GSP=1`, `HAL_UART_WriteString` becomes a no-op stub
   (binary protocol takes UART1). For text-mode FOC debug, set
   `FEATURE_GSP=0` temporarily.
2. **`FEATURE_FOC_AN1078` must come before the PWM block in `garuda_config.h`**
   because `PWMFREQUENCY_HZ` is mode-dependent.
3. **`GarudaService_Tasks()` had a 6-step stall detector** that
   mis-fired at OL→CL handoff because it observed `gEscState==CLOSED_LOOP`
   without checking `SectorPI_IsRunning()`. Now gated `#if !FEATURE_FOC_AN1078`.
4. **GUI's `decodeSnapshot()` throws on empty payload** (no try-catch).
   The unhandled exception killed the read loop and silently disconnected
   the WebSerial port. Fix: `GET_SNAPSHOT` returns real data in FOC mode.
5. **`TELEM_FRAME` payload starts with 2-byte `seq` header** —
   GUI does `payload.slice(2)` before decoding. Easy to forget when
   building the snapshot.

---

## Files added

```
foc/an1078_motor.c                   AN1078 state machine
foc/an1078_motor.h
foc/an1078_smc.c                     Sliding-mode observer
foc/an1078_smc.h
foc/an1078_params.h                  Tunable constants
foc/pll_estimator.c                  Angle PLL (cross-product discriminator)
foc/pll_estimator.h
foc/foc_runtime.c                    Boot + ADC ISR hook + SW1/SW2 routing
foc/foc_runtime.h
foc/foc_shim_gsp.h                   Stub gspParams (until GUI param wiring)
foc/foc_types.h                      ThreePhase/AlphaBeta/DQ structs
tools/foc_capture.py                 Python GSP client (FOC mode)
tools/foc_debug_watch.py             UART text watcher (when GSP=0)
docs/ak_an1078_foc_port.md           this file
```

## Files modified

```
garuda_config.h                      FEATURE_FOC_AN1078 default + mode-aware PWM
garuda_service.c                     ADC ISR hook + 6-step stall gate
gsp/gsp_commands.{c,h}               FOC snapshot, mode-dependent boardId
hal/hal_pwm.{c,h}                    SVPWM helpers (Release/SetDutyFloat3Phase)
hal/hal_com_timer.c                  file-level !FOC gate
hal/hal_ptg.c                        file-level !FOC gate
motor/sector_pi.c                    file-level !FOC gate
motor/commutation.c                  file-level !FOC gate
main.c                               FOC init + SW1/SW2 routing
nbproject/Makefile-default.mk        foc/*.c added to build
```

## Future work

1. **FOC at 60 kHz** — cleaner SMO, smoother CL above 100k eRPM.
2. **Angle PLL** on top of `atan2(BEMF)` — for sub-tick angle resolution
   at high speed.
3. **Software current limit** — AN1078 has none; rely on the `AN_OVER_CURRENT_LIMIT`
   clamp + the 22 A shunt saturation ceiling.
4. **GUI param wiring** — currently `gspParams` is stubbed to zero so
   SMC reads compile-time defaults. Wire to GSP `SET_PARAM` for live tuning.
5. **Field-weakening tuning** — Id swings ±5–7 A at high speed today;
   smoother with PLL angle.
