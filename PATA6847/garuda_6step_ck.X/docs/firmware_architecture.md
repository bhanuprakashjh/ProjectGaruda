# Garuda 6-Step CK Firmware Architecture

**Target**: dsPIC33CK64MP205 + ATA6847 gate driver (EV43F54A board)
**Motor**: A2212 1400KV (7PP, 12V) / Hurst DMB2424B10002 (5PP, 24V)
**Commutation**: Sensorless 6-step trapezoidal BLDC
**Compiler**: XC16 v2.10

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Hardware Architecture](#2-hardware-architecture)
3. [Software Architecture](#3-software-architecture)
4. [State Machine](#4-state-machine)
5. [Zero-Crossing Detection Algorithm](#5-zero-crossing-detection-algorithm)
6. [Commutation Scheduling](#6-commutation-scheduling)
7. [ISR Architecture](#7-isr-architecture)
8. [Key Data Structures](#8-key-data-structures)
9. [API Reference](#9-api-reference)
10. [Configuration Reference](#10-configuration-reference)
11. [Timing Analysis](#11-timing-analysis)
12. [Verified Performance](#12-verified-performance)

---

## 1. System Overview

```
                    +-----------+
     12-14.4V ----->| ATA6847   |----> 3-phase inverter ----> BLDC motor
                    | Gate Drv  |<---- BEMF comparators (RC6/RC7/RD10)
                    +-----+-----+
                          |  SPI (16-bit)
                    +-----+-----+
                    | dsPIC33CK |
                    | 64MP205   |
                    |           |
                    | PWM1/2/3  |----> ATA6847 INH/INL (6 outputs)
                    | SCCP1     |----> 100kHz ZC fast poll timer
                    | SCCP4     |----> 640ns HR commutation timer
                    | Timer1    |----> 50us state machine tick
                    | ADC       |----> Phase currents, pot, Vbus
                    | UART1     |----> Debug console (115200)
                    +-----+-----+
                          |
                     Pot (speed ref)
```

The firmware implements sensorless 6-step trapezoidal commutation using the
ATA6847's built-in BEMF comparators. Zero-crossing detection uses a hybrid
approach: a 100kHz fast poll timer (SCCP1) for primary detection with a
640ns-resolution hardware commutation timer (SCCP4) for precise scheduling.

---

## 2. Hardware Architecture

### 2.1 Pin Mapping

| Function       | Pin   | Peripheral       | Notes                          |
|----------------|-------|------------------|--------------------------------|
| PWM1H (Ph A)   | RB10  | PWM Generator 1  | Master, center-aligned         |
| PWM1L (Ph A)   | RB11  | PWM Generator 1  | Complementary, dead-time       |
| PWM2H (Ph B)   | RB12  | PWM Generator 2  | Slave of PG1                   |
| PWM2L (Ph B)   | RB13  | PWM Generator 2  |                                |
| PWM3H (Ph C)   | RB14  | PWM Generator 3  | Slave of PG1                   |
| PWM3L (Ph C)   | RB15  | PWM Generator 3  |                                |
| BEMF_A         | RC6   | GPIO input        | ATA6847 comparator output      |
| BEMF_B         | RC7   | GPIO input        |                                |
| BEMF_C         | RD10  | GPIO input        |                                |
| SPI_SDI        | RC4   | SPI1              | ATA6847 data in                |
| SPI_SCK        | RC5   | SPI1              | ATA6847 clock                  |
| SPI_SDO        | RC10  | SPI1              | ATA6847 data out               |
| SPI_nCS        | RC11  | GPIO output       | ATA6847 chip select            |
| UART_RX        | RC8   | UART1             | Debug console                  |
| UART_TX        | RC9   | UART1             | Debug console                  |
| nIRQ           | RD1   | INT1              | ATA6847 fault interrupt        |
| BTN1 (Start)   | RC0   | INT3              | User button                    |
| BTN2 (Stop)    | RD13  | INT2              | User button                    |
| LED_FAULT      | RC3   | GPIO output       |                                |
| LED_RUN        | RC13  | GPIO output       |                                |
| Pot (ADC)      | AN6   | ADC shared        | Speed reference                |
| Vbus (ADC)     | AN9   | ADC shared        | Bus voltage monitor            |

### 2.2 ATA6847 Gate Driver

The ATA6847 provides:
- **3-phase half-bridge drivers** with integrated bootstrap
- **BEMF comparators** with digital output (vs internal virtual neutral)
- **Overcurrent protection** (hardware, no LEB)
- **SPI control** (16-bit: 7-bit addr + R/W + 8-bit data)

Initialization sequence:
1. Disable watchdog
2. Clear power-up faults
3. Configure GDU registers (GDUCR1-4), enable BEMF comparators (BEMFEN=1)
4. Enter Normal mode via GOPMCR, poll DSR1.GDUS for ready

### 2.3 Timer Architecture

```
Timer1 (20kHz / 50us)        SCCP1 (100kHz / 10us)       SCCP4 (1.5625MHz / 640ns)
+---------------------+      +---------------------+      +---------------------+
| State machine tick   |      | ZC fast poll         |      | Free-running HR      |
| Forced step timeout  |      | Adaptive deglitch    |      | Output compare →     |
| System tick (1ms)    |      | BLANKING→ARMED→DONE  |      |   commutation ISR    |
| Priority: 4          |      | Priority: 5          |      | Priority: 6          |
+---------------------+      +---------------------+      +---------------------+
        |                              |                            |
        v                              v                            v
  Timer1 ISR                    SCCP1 ISR                    SCCP4 ISR
  (garuda_service.c)            (garuda_service.c)           (garuda_service.c)
  - ESC state machine           - BEMF_ZC_FastPoll()         - COMMUTATION_Advance
  - ZC timeout check            - Schedule comm on ZC        - BEMF_ZC_OnComm
  - Forced commutation
```

**ADC ISR (20kHz, Priority 3)**: Triggered by PWM, reads all channels, backup
ZC poll, commutation deadline check, Vbus fault monitoring.

---

## 3. Software Architecture

### 3.1 File Organization

```
garuda_6step_ck.X/
+-- main.c                  Init sequence, main loop, debug output
+-- garuda_config.h         All #defines, motor profiles, timing constants
+-- garuda_types.h          Structs: GARUDA_DATA_T, TIMING_STATE_T, etc.
+-- garuda_service.h/c      State machine, ISRs (Timer1, ADC, SCCP1, SCCP4)
+-- motor/
|   +-- commutation.h/c     6-step table, AdvanceStep(), ApplyStep()
|   +-- bemf_zc.h/c         ZC detection, scheduling, timeout, fast poll
|   +-- startup.c           Alignment and open-loop ramp
+-- hal/
|   +-- clock.h             PLL configuration (200MHz)
|   +-- port_config.c       GPIO, PPS, analog config
|   +-- hal_uart.h/c        Debug UART (115200 baud)
|   +-- hal_spi.h/c         SPI for ATA6847
|   +-- hal_ata6847.h/c     Gate driver init, read/write, mode control
|   +-- hal_adc.h/c         ADC init (currents, pot, Vbus)
|   +-- hal_opa.h/c         Internal op-amps for current sensing
|   +-- hal_pwm.h/c         PWM init, duty, commutation step application
|   +-- hal_timer1.h/c      Timer1 init (50us tick)
|   +-- hal_ic.h/c          SCCP1 fast poll timer (100kHz)
|   +-- hal_com_timer.h/c   SCCP4 commutation timer (640ns)
|   +-- hal_clc.h/c         CLC D-FF BEMF blanking (optional)
|   +-- hal_trap.c          Trap handlers (address error, stack, math, etc.)
|   +-- hal_diag.h/c        UART diagnostic commands
|   +-- board_service.h/c   LED/button handling
+-- tools/
|   +-- zc_capture.py       Telemetry capture to CSV
+-- docs/
    +-- firmware_architecture.md  (this document)
```

### 3.2 Call Graph

```
main()
  +-- CLOCK_Initialize()
  +-- SetupGPIOPorts()
  +-- HAL_UART_Init()
  +-- HAL_SPI_Init()
  +-- HAL_ATA6847_Init()
  +-- HAL_OPA_Init()
  +-- HAL_ADC_Init()
  +-- HAL_PWM_Init()
  +-- HAL_Timer1_Init()
  +-- HAL_ZcTimer_Init()          [FEATURE_IC_ZC]
  +-- HAL_ComTimer_Init()         [FEATURE_IC_ZC]
  +-- GarudaService_Init()
  +-- while(1):
      +-- BoardService()
      +-- HAL_ADC_PollPotVbus()   [IDLE only]
      +-- DIAG_ProcessCommand()
      +-- GarudaService_MainLoop()

_T1Interrupt() [Timer1, 20kHz]
  +-- ESC_ARMED:   countdown → STARTUP_Init() → ESC_ALIGN
  +-- ESC_ALIGN:   STARTUP_Align() → ESC_OL_RAMP
  +-- ESC_OL_RAMP: STARTUP_OpenLoopRamp()
  |                  +-- COMMUTATION_AdvanceStep()
  |                  +-- BEMF_ZC_OnCommutation()
  |                On rampDone+zcSynced → ESC_CLOSED_LOOP
  +-- ESC_CL:     MapThrottleToDuty(), duty slew
  |                  +-- BEMF_ZC_CheckTimeout()
  |                     On FORCE_STEP → COMMUTATION_AdvanceStep()
  |                     On DESYNC → EnterRecovery() or EnterFault()

_ADCInterrupt() [ADC, 20kHz, PWM-triggered]
  +-- Read all ADC buffers (pot, Vbus, currents)
  +-- BEMF_ZC_Poll()              [OL_RAMP or CL backup]
  +-- BEMF_ZC_ScheduleCommutation()  [on ZC detected]
  +-- Commutation deadline check   [if Timer1 tick >= deadline]
  |     +-- COMMUTATION_AdvanceStep()
  |     +-- BEMF_ZC_OnCommutation()
  +-- Vbus fault checking (OV/UV with debounce)

_CCT1Interrupt() [SCCP1, 100kHz, FEATURE_IC_ZC]
  +-- BEMF_ZC_FastPoll()
      +-- BLANKING: check HR timestamp → ARMED
      +-- ARMED: read comparator, deglitch filter
      +-- On confirmed: RecordZcTiming()
      +-- BEMF_ZC_ScheduleCommutation()

_CCP4Interrupt() [SCCP4, one-shot, FEATURE_IC_ZC]
  +-- COMMUTATION_AdvanceStep()
  +-- BEMF_ZC_OnCommutation()
```

---

## 4. State Machine

```
                                BTN1
                                 |
                                 v
+------+  StartMotor  +-------+  200ms   +-------+  align done  +---------+
| IDLE |------------->| ARMED |--------->| ALIGN |------------->| OL_RAMP |
+------+              +-------+          +-------+              +---------+
   ^                                                                |
   |                                                    zcSynced && |
   |                                                    rampDone    |
   |                                                                v
   |   StopMotor    +---------+  desync   +---------+        +-----------+
   +<---------------| RECOVERY|<----------| FAULT   |<-------| CLOSED    |
   |                +---------+           +---------+        | LOOP      |
   |                     |                     ^             +-----------+
   |    auto-restart     |                     |                  |
   +<--------------------+                     +------------------+
        (max 3 retries)                       12 consecutive misses
```

### State Descriptions

| State | Duration | What Happens |
|-------|----------|--------------|
| **IDLE** | Until BTN1 | Motor off, pot/Vbus polled in main loop |
| **ARMED** | 200ms | GDU power-up, bootstrap charge, ADC/timer enable |
| **ALIGN** | 150-200ms | Hold step 0, ramp duty to ALIGN_DUTY (locks rotor) |
| **OL_RAMP** | ~1s | Forced commutation, accelerate at 1500 eRPM/s |
| **CLOSED_LOOP** | Indefinite | ZC-based commutation, pot throttle control |
| **RECOVERY** | 200ms | All outputs disabled, auto-restart if retries remain |
| **FAULT** | Until reset | Latching fault, PWM disabled |

### OL_RAMP to CLOSED_LOOP Handoff

Two conditions must be met simultaneously:
1. `rampDone`: step period has reached `MIN_STEP_PERIOD` (ramp target speed)
2. `zcSynced`: 6 consecutive valid ZC detections (`ZC_SYNC_THRESHOLD`)

At handoff:
- Seed HR step period from Timer1: `stepPeriodHR = stepPeriod * 625/8`
- Start SCCP1 fast poll timer (`HAL_ZcTimer_Start()`)
- Set up first fast poll blanking (`BEMF_ZC_OnCommutation()`)

---

## 5. Zero-Crossing Detection Algorithm

### 5.1 Overview

The system uses a **hybrid detection** approach:

```
                   +------------------+
                   |  ATA6847 BEMF    |
                   |  Comparators     |
                   |  (RC6/RC7/RD10)  |
                   +--------+---------+
                            |
              +-------------+-------------+
              |                           |
     +--------v--------+        +--------v--------+
     | SCCP1 Fast Poll  |        | ADC ISR Backup  |
     | 100kHz (10us)    |        | 20kHz (50us)    |
     | Primary in CL    |        | Primary in OL   |
     | Adaptive deglitch|        | Fixed filter=3  |
     | FL: 3-8          |        |                 |
     +---------+--------+        +--------+--------+
               |                          |
               +----------+  +-----------+
                           |  |
                    +------v--v------+
                    | RecordZcTiming |
                    | IIR step period|
                    | HR timestamps  |
                    +-------+--------+
                            |
                    +-------v--------+
                    | Schedule Comm  |
                    | Timing advance |
                    | SCCP4 absolute |
                    | compare        |
                    +----------------+
```

### 5.2 SCCP1 Fast Poll State Machine (100kHz)

Called from `_CCT1Interrupt()` every 10us during closed-loop:

```
+----------+    HR timestamp    +-------+   filter >= FL   +------+
| BLANKING |--- >= blankEnd --->| ARMED |--- confirmed --->| DONE |
+----------+                    +-------+                  +------+
                                   |  ^
                                   |  | mismatch: pollFilter--
                                   |  | (bounce-tolerant)
                                   +--+
                                match: pollFilter++
                                first match: record zcCandidateHR
```

**BLANKING phase**:
- After each commutation, blank for `ZC_BLANKING_PERCENT` of step period
- Uses SCCP4 HR timestamps (640ns resolution) for precise blanking
- Minimum blanking: 25us (39 HR ticks)

**ARMED phase**:
- Read comparator for the floating phase
- Compare against expected post-ZC state (rising ZC → expect 1, falling → expect 0)
- **Deglitch filter**: require `filterLevel` consecutive matching reads
  - First match records `zcCandidateHR` (eliminates filter delay from timing)
  - Mismatch: decrement filter (bounce-tolerant, costs 2 polls instead of full reset)
- On confirmation: record ZC timing, schedule commutation

**Adaptive filter level** (scales with step period):
```
FL = 3  when Tp <= 5   (high speed, 30us latency = 4.3deg at 100k eRPM)
FL = 8  when Tp >= 50  (low speed, 80us latency = 2.9deg at 4k eRPM)
Linear interpolation between thresholds
```

On deceleration, FL uses `max(IIR stepPeriod, latest zcInterval)` to
re-increase promptly instead of staying stuck at minimum.

### 5.3 Step Period IIR Filter

Both Timer1 and HR step periods are tracked with an IIR filter:

```
stepPeriod = (stepPeriod * 3 + newInterval) / 4    (75% old, 25% new)
```

With 2-step averaging when both current and previous intervals are valid:
```
avgInterval = (zcInterval + prevZcInterval) / 2
stepPeriod = (stepPeriod * 3 + avgInterval) / 4
```

**Rejection gates**:
- Intervals > 1.5x stepPeriod rejected (multi-step intervals from missed ZCs)
- Intervals < stepPeriod/2 rejected (PWM noise, not real BEMF)
- Floor: `stepPeriod >= MIN_CL_STEP_PERIOD` (2 for A2212, prevents runaway)

### 5.4 Forced Step Timeout

When no ZC is detected within `ZC_TIMEOUT_MULT * stepPeriod` (2x):
1. Increment `consecutiveMissedSteps`
2. Halve `goodZcCount` (soft penalty)
3. If >= 3 consecutive misses: clear `zcSynced` → duty drops to idle floor
4. If >= 12 consecutive misses: desync fault → recovery
5. When desynced: increase stepPeriod by 12.5% per forced step (prevents
   runaway high-speed false ZCs by slowing forced commutation to match
   decelerating motor)

---

## 6. Commutation Scheduling

### 6.1 Timing Advance

Linear interpolation from low to high speed:

```
A2212:  0deg at 5k eRPM  -->  20deg at 100k eRPM
Hurst:  0deg at 6k eRPM  -->  8deg at 15k eRPM
```

Delay from ZC to commutation:
```
delay = stepPeriod * (30 - advanceDeg) / 60
```

At 30deg advance = 0 delay (commutate at ZC). Standard 6-step uses
30deg delay (commutate at midpoint). Typical advance of 15-20deg at
high speed compensates for switching delay and inductance.

### 6.2 HR Scheduling Path

```
ZC detected (SCCP1 fast poll)
  |
  v
Compute advanceDeg from eRPM lookup
  |
  v
delayHR = stepPeriodHR * (30 - advDeg) / 60
  |
  v
targetHR = lastZcTickHR + delayHR
  |
  v
Is targetHR in the future?
  |           |
  YES         NO (latency exceeded delay)
  |           |
  v           v
Schedule    targetHR = now + 2  (fire ASAP, ~1.3us)
SCCP4       Schedule SCCP4
  |           |
  v           v
SCCP4 ISR fires at targetHR --> COMMUTATION_AdvanceStep()
```

The "fire ASAP when target is past" fix prevents falling through to the
Timer1 path which has 50us jitter (±50% at Tp:2). This was the root cause
of high-speed desync on A2212.

### 6.3 Commutation Table

Standard 6-step pattern (CW rotation):

```
Step | Phase A | Phase B | Phase C | Float | ZC Polarity
-----|---------|---------|---------|-------|------------
  0  |  PWM    |  LOW    |  FLOAT  |   C   |  Rising
  1  |  FLOAT  |  LOW    |  PWM    |   A   |  Falling
  2  |  LOW    |  FLOAT  |  PWM    |   B   |  Rising
  3  |  LOW    |  PWM    |  FLOAT  |   C   |  Falling
  4  |  FLOAT  |  PWM    |  LOW    |   A   |  Rising
  5  |  PWM    |  FLOAT  |  LOW    |   B   |  Falling
```

Phase states via PWM override registers:
- **PWM_ACTIVE**: Override OFF → normal PWM switching
- **LOW**: Override ON, OVRDAT=01 → high-side OFF (via POLH inversion), low-side ON
- **FLOAT**: Override ON, OVRDAT=00 → both FETs OFF

---

## 7. ISR Architecture

### 7.1 Priority Map

| Priority | ISR | Rate | Purpose |
|----------|-----|------|---------|
| 6 | SCCP4 Compare | On-demand | Commutation execution (640ns precision) |
| 5 | SCCP1 Period | 100kHz | ZC fast poll + deglitch filter |
| 4 | Timer1 | 20kHz | State machine, forced step timeout |
| 3 | ADC Complete | 20kHz | Read sensors, backup ZC, Vbus faults |

Higher priority ISRs can preempt lower ones. SCCP4 (commutation) is highest
to minimize jitter. SCCP1 (ZC poll) is next to maintain consistent 10us
sampling. Timer1 and ADC handle state machine logic and sensing.

### 7.2 ISR Timing Budget (at 100MHz Fp)

| ISR | Typical | Max | Overhead at rate |
|-----|---------|-----|-----------------|
| SCCP1 (100kHz) | 150ns | 2us (ZC+schedule) | 1.5% idle, 20% on ZC |
| Timer1 (20kHz) | 500ns | 3us (CL+forced) | 1.0% idle, 6% worst |
| ADC (20kHz) | 1us | 4us (ZC+comm) | 2.0% idle, 8% worst |
| SCCP4 (on-demand) | 800ns | 800ns | <0.1% |
| **Total worst case** | | | **~15% CPU** |

### 7.3 Critical Timing Constraints

1. **SCCP1 ISR must complete within 10us** (next poll at 100kHz). Typical: 150ns.
2. **SCCP4 ISR must not be blocked** by lower-priority ISRs. SCCP1 can preempt
   Timer1/ADC but not SCCP4.
3. **ADC ISR must read all buffers** to clear data-ready flags. Missing a read
   causes ADC to stop generating interrupts.
4. **Clear ISR flags at END of handler** (after all work), not at beginning.

---

## 8. Key Data Structures

### 8.1 GARUDA_DATA_T (Main ESC State)

```c
typedef struct {
    ESC_STATE_T state;          // Current state machine state
    uint8_t currentStep;        // 0-5 commutation step
    uint32_t duty;              // PWM duty cycle (0 to LOOPTIME_TCY)
    uint16_t potRaw;            // ADC pot reading (speed reference)
    uint16_t vbusRaw;           // ADC bus voltage reading
    FAULT_CODE_T faultCode;     // Active fault code
    uint16_t timer1Tick;        // 50us counter (wraps at 65535)
    uint32_t systemTick;        // 1ms counter (divides Timer1 by 20)

    BEMF_STATE_T bemf;          // Comparator state & deglitch filter
    TIMING_STATE_T timing;      // Step period, ZC timing, deadlines
    IC_ZC_STATE_T icZc;         // Fast poll state machine [FEATURE_IC_ZC]

    // Startup
    uint32_t alignCounter;      // Alignment countdown
    uint32_t rampStepPeriod;    // OL ramp step period
    uint32_t rampCounter;       // OL ramp step countdown

    // Recovery
    uint8_t desyncRestartAttempts;  // Auto-restart counter (max 3)
    uint32_t recoveryCounter;       // Recovery delay countdown
} GARUDA_DATA_T;
```

### 8.2 TIMING_STATE_T

```c
typedef struct {
    uint16_t stepPeriod;        // IIR-filtered step period (Timer1 ticks)
    uint16_t lastCommTick;      // Timer1 tick at last commutation
    uint16_t lastZcTick;        // Timer1 tick at last ZC detection
    uint16_t zcInterval;        // Ticks between last two ZCs
    uint16_t prevZcInterval;    // Previous zcInterval (for 2-step avg)
    uint16_t commDeadline;      // Timer1 tick for next commutation
    uint16_t forcedCountdown;   // Ticks until forced step
    uint16_t goodZcCount;       // Consecutive good ZC detections
    uint8_t consecutiveMissedSteps;
    uint8_t stepsSinceLastZc;
    bool zcSynced;              // TRUE when goodZcCount >= threshold
    bool deadlineActive;        // TRUE when comm is scheduled
    bool hasPrevZc;

    // High-resolution (FEATURE_IC_ZC, SCCP4 640ns ticks)
    uint16_t stepPeriodHR;      // IIR-filtered HR step period
    uint16_t lastZcTickHR;      // SCCP4 tick at last ZC
    uint16_t zcIntervalHR;      // HR ticks between ZCs
    uint16_t prevZcIntervalHR;
    bool hasPrevZcHR;
} TIMING_STATE_T;
```

### 8.3 IC_ZC_STATE_T (Fast Poll, FEATURE_IC_ZC)

```c
typedef struct {
    IC_ZC_PHASE_T phase;        // BLANKING -> ARMED -> DONE
    uint8_t activeChannel;      // Floating phase to poll (0=A, 1=B, 2=C)
    uint8_t pollFilter;         // Consecutive matching reads
    uint8_t filterLevel;        // Current deglitch threshold (3-8)
    uint16_t blankingEndHR;     // SCCP4 tick when blanking expires
    uint16_t lastCommHR;        // SCCP4 tick at last commutation
    uint16_t zcCandidateHR;     // SCCP4 tick at first filter match
    uint16_t zcCandidateT1;     // Timer1 tick at first filter match

    // Diagnostics
    uint16_t diagAccepted;      // Total accepted ZCs (fast poll)
    uint16_t diagLcoutAccepted; // Total accepted ZCs (ADC backup)
    uint16_t diagFalseZc;       // Rejected by interval check
    uint16_t diagPollCycles;    // Total poll ISR invocations
} IC_ZC_STATE_T;
```

---

## 9. API Reference

### 9.1 Motor Control (garuda_service.h)

| Function | Context | Description |
|----------|---------|-------------|
| `GarudaService_Init()` | Main | Initialize ESC data, start Timer1 |
| `GarudaService_StartMotor()` | Main | Enter ARMED state, begin startup sequence |
| `GarudaService_StopMotor()` | Main | Kill PWM, enter IDLE |
| `GarudaService_MainLoop()` | Main | Handle deferred restart/fault cleanup |

### 9.2 ZC Detection (motor/bemf_zc.h)

| Function | Context | Description |
|----------|---------|-------------|
| `BEMF_ZC_Init(pData)` | Any | Reset all BEMF and timing state |
| `BEMF_ZC_OnCommutation(pData)` | ISR | Reset filter, set blanking, configure timeout |
| `BEMF_ZC_Poll(pData)` | ADC ISR | 20kHz comparator poll, returns true on ZC |
| `BEMF_ZC_FastPoll(pData)` | SCCP1 ISR | 100kHz fast poll with adaptive deglitch |
| `BEMF_ZC_ScheduleCommutation(pData)` | ISR | Compute advance, schedule SCCP4 compare |
| `BEMF_ZC_CheckTimeout(pData)` | Timer1 ISR | Check forced step timeout, returns status |

### 9.3 Commutation (motor/commutation.h)

| Function | Context | Description |
|----------|---------|-------------|
| `COMMUTATION_AdvanceStep(pData)` | ISR | Increment step (0-5), apply phase states |
| `COMMUTATION_ApplyStep(pData, step)` | ISR | Apply specific step to PWM outputs |

### 9.4 Startup (motor/startup.c)

| Function | Context | Description |
|----------|---------|-------------|
| `STARTUP_Init(pData)` | ISR | Initialize startup counters and step |
| `STARTUP_Align(pData)` | Timer1 ISR | Hold step 0, ramp duty. Returns true when done |
| `STARTUP_OpenLoopRamp(pData)` | Timer1 ISR | Forced commutation ramp. Returns true when done |

### 9.5 HAL — Timers (FEATURE_IC_ZC)

| Function | Context | Description |
|----------|---------|-------------|
| `HAL_ZcTimer_Init()` | Main | Configure SCCP1 as 100kHz periodic timer |
| `HAL_ZcTimer_Start()` | ISR | Enable SCCP1 period interrupt |
| `HAL_ZcTimer_Stop()` | ISR | Disable SCCP1 period interrupt |
| `HAL_ComTimer_Init()` | Main | Configure SCCP4 as free-running HR timer |
| `HAL_ComTimer_ReadTimer()` | ISR | Read SCCP4 counter (640ns/tick) |
| `HAL_ComTimer_ScheduleAbsolute(tick)` | ISR | Set SCCP4 compare, enable one-shot ISR |
| `HAL_ComTimer_Cancel()` | ISR | Disable SCCP4 compare ISR |

### 9.6 HAL — PWM (hal/hal_pwm.h)

| Function | Context | Description |
|----------|---------|-------------|
| `HAL_PWM_Init()` | Main | Configure 20kHz center-aligned complementary PWM |
| `HAL_PWM_SetDutyCycle(duty)` | ISR | Set duty on all generators (slaves first) |
| `HAL_PWM_SetCommutationStep(step)` | ISR | Apply 6-step phase states via overrides |
| `HAL_PWM_DisableOutputs()` | Any | Force all outputs low (fault-safe) |
| `HAL_PWM_ChargeBootstrap()` | Main | Force low-sides on for bootstrap charge |

---

## 10. Configuration Reference

### 10.1 Motor Profiles

| Parameter | Hurst (Profile 0) | A2212 (Profile 1) |
|-----------|-------------------|-------------------|
| Pole pairs | 5 | 7 |
| KV (RPM/V) | 149 | 1400 |
| Rs (mOhm) | 534 | 65 |
| Ls (uH) | 471 | 30 |
| Align time (ms) | 200 | 150 |
| Align duty (%) | 5 | 2.5 |
| Init step period | 1000 (50ms) | 800 (40ms) |
| Min step period | 66 (3.3ms) | 50 (2.5ms) |
| Min CL step period | 66 | 2 (100us) |
| Ramp accel (eRPM/s) | 1500 | 1500 |
| Ramp duty cap (%) | 17 | 17 |
| Advance range | 0-8deg | 0-20deg |
| Advance eRPM range | 6k-15k | 5k-100k |
| Max CL eRPM | 15,000 | 100,000 |
| CL idle duty (%) | 8 | 10 |
| Blanking (% step) | 20 | 20 |

### 10.2 ZC Detection Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `ZC_SYNC_THRESHOLD` | 6 | Good ZCs to declare synced |
| `ZC_TIMEOUT_MULT` | 2 | Forced step at 2x step period |
| `ZC_DESYNC_THRESH` | 3 | Misses to clear zcSynced |
| `ZC_MISS_LIMIT` | 12 | Misses to fault |
| `ZC_FILTER_THRESHOLD` | 3 | ADC poll deglitch (20kHz path) |
| `ZC_DEGLITCH_MIN` | 3 | Fast poll min filter (100kHz) |
| `ZC_DEGLITCH_MAX` | 8 | Fast poll max filter |
| `ZC_DEGLITCH_FAST_TP` | 5 | Tp threshold for min filter |
| `ZC_DEGLITCH_SLOW_TP` | 50 | Tp threshold for max filter |
| `ZC_BLANKING_PERCENT` | 20 | % of step period blanked |
| `ZC_POLL_FREQ_HZ` | 100,000 | SCCP1 poll rate |

### 10.3 Feature Flags

| Flag | Default | Description |
|------|---------|-------------|
| `FEATURE_IC_ZC` | 1 | Enable SCCP1 fast poll + SCCP4 HR timing |
| `MOTOR_PROFILE` | 1 | 0=Hurst, 1=A2212 |

---

## 11. Timing Analysis

### 11.1 ZC Detection Latency

At 100kHz poll rate (10us period):

| Filter Level | Latency | Elec. Degrees at 100k eRPM |
|-------------|---------|---------------------------|
| FL=3 | 30us | 4.3deg |
| FL=4 | 40us | 5.8deg |
| FL=5 | 50us | 7.2deg |
| FL=8 | 80us | 11.5deg |

The first matching read records the ZC candidate timestamp, so the scheduling
delay is only from the ZC event to the first match (~10us worst case), not
the full filter latency.

### 11.2 Commutation Scheduling Latency

From ZC detection to commutation:

```
ZC event → ~10us → first match (zcCandidateHR recorded)
         → 20-70us → filter confirmed
         → ~1us → RecordZcTiming + ScheduleCommutation
         → SCCP4 absolute compare scheduled
         → delayHR ticks → SCCP4 ISR fires
         → ~800ns → step applied to PWM
```

Total jitter: dominated by the 640ns HR timer resolution. At Tp:2 (100us
step), measured TpHR jitter is 3 ticks = 1.9us (2.0%).

### 11.3 Speed Range and Resolution

| Metric | Timer1 (50us) | SCCP4 HR (640ns) |
|--------|---------------|-------------------|
| Min measurable Tp | 2 ticks (100us) | ~150 ticks (96us) |
| Max eRPM | 100,000 | ~103,000 |
| Resolution at 100k eRPM | 50% (1 tick = ±50k eRPM) | 0.65% (1 tick = ±670 eRPM) |
| Resolution at 10k eRPM | 5% | 0.065% |

The HR timer provides **78x better resolution** than Timer1. It is used for
all commutation scheduling when `stepPeriod < HR_MAX_STEP_PERIOD` (800).

---

## 12. Verified Performance

### 12.1 Test Configuration

- **Motor**: A2212 1400KV, 7 pole pairs, no load
- **Voltage**: 14.4V bench supply
- **Board**: EV43F54A (dsPIC33CK64MP205 + ATA6847)
- **Capture**: 30 seconds via `zc_capture.py`, 100ms telemetry interval

### 12.2 Results

| Metric | Value |
|--------|-------|
| Speed range | 10,272 - 102,124 eRPM (1,467 - 14,589 mech RPM) |
| Duty range | 999 - 9,799 (10% - 98%) |
| Max missed ZCs | 0 |
| Desync events | 0 |
| IC false ZCs | 3 (all at CL entry, 0.02% rate) |
| TpHR jitter at max speed | 2.0% (153-156 HR ticks) |
| ZC interval alternation | 0.6 mean diff (perfectly balanced) |
| Vbus sag | 1.2% (14.28V → 14.10V) |
| Effective advance angle | ~3 electrical degrees |
| Pot-to-duty linearity | R^2 = 0.999989 |
| eRPM/duty ratio | 10.3-10.8 (linear V/Hz) |
| Commutation quality score | 9.0 / 10 |

### 12.3 Key Bugs Fixed

1. **High-speed desync** (commit `309fb8e`): HR commutation target was in the
   past at Tp:2 due to filter+compute latency. Fell through to Timer1 path
   with 50us jitter. Fix: schedule ASAP instead of falling through.

2. **False IC rate 50%** (commit `84ca70e`): FL:2 deglitch at 100k eRPM was
   too thin — only 2 matching reads (20us) to confirm ZC. Raised to FL:3
   (30us), added hysteretic FL on decel. False rate dropped from 52,753 to 3.

3. **DefaultInterrupt trap name** (commit `309fb8e`): dsPIC33CK uses single
   underscore `_DefaultInterrupt`, not double `__DefaultInterrupt`.

---

## Appendix A: Conversion Constants

```
Timer1 tick to SCCP4 HR ticks:  HR = T1 * 625 / 8   (78.125x)
SCCP4 HR tick to microseconds:  us = HR * 0.64
Timer1 tick to microseconds:    us = T1 * 50
eRPM from Timer1:               eRPM = 200000 / Tp
eRPM from SCCP4 HR:             eRPM = 15625000 / TpHR
Mechanical RPM:                 mechRPM = eRPM / MOTOR_POLE_PAIRS
```

## Appendix B: Diagnostic Commands

Connect to debug UART at 115200 baud. Type single characters:

| Cmd | Description |
|-----|-------------|
| `h` | Help — list all commands |
| `v` | Toggle verbose mode (100ms telemetry) |
| `s` | Full ESC state dump |
| `a` | ATA6847 register dump |
| `b` | BEMF comparator pin states |
| `d` | ADC readings (pot, Vbus in volts) |
| `p` | PWM register dump |
| `x` | IC ZC statistics (FEATURE_IC_ZC) |
| `f` | Read and clear ATA6847 faults |
| `1` | Manual step advance |
| `0` | Force all phases float |

## Appendix C: Telemetry Format

Verbose mode outputs at 100ms intervals:

```
S:CL P:64944 V:14272 D:9799 St:0 Tp:2 ZC:15907 SYN Miss:0 B:101 DC1:9799
IO:3000/3400/0000 FP:15551/BK:2/3 Cyc:41308 TpHR:153 ZcI:2 ZcIHR:141
PZcI:2 Ph:0 FL:3 Frc:1 eRPM:102124 D%:979
```

| Field | Description |
|-------|-------------|
| S: | ESC state (IDLE/ARMED/ALIGN/OL_RAMP/CL/RECOV/FAULT) |
| P: | Pot ADC raw (0-65535) |
| V: | Vbus ADC raw (~1211 per volt) |
| D: | Duty (0 to LOOPTIME_TCY) |
| St: | Current commutation step (0-5) |
| Tp: | Step period (Timer1 ticks, 50us each) |
| ZC: | Good ZC count (cumulative) |
| SYN/--- | ZC synced flag |
| Miss: | Consecutive missed steps |
| B:xyz | BEMF comparator states (A/B/C, 0=low 1=high) |
| DC1: | PG1DC register readback |
| IO: | PG1/2/3 IOCONL register readback (hex) |
| FP: | IC accepted count (fast poll) |
| BK: | IC backup accepted / false ZC count |
| Cyc: | Poll cycle counter (wraps at 65535) |
| TpHR: | HR step period (SCCP4 ticks, 640ns each) |
| ZcI: | ZC interval (Timer1 ticks) |
| ZcIHR: | ZC interval (SCCP4 HR ticks) |
| PZcI: | Previous ZC interval (Timer1 ticks) |
| Ph: | Fast poll phase (0=BLANKING, 1=ARMED, 2=DONE) |
| FL: | Current deglitch filter level (3-8) |
| Frc: | Steps since last ZC |
| eRPM: | Electrical RPM |
| D%: | Duty as percent x10 (e.g. 979 = 97.9%) |
