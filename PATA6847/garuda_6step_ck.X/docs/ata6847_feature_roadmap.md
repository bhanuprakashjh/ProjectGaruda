# ATA6847 Feature Enablement Roadmap

**Board**: EV43F54A (dsPIC33CK64MP205 + ATA6847 QFN40)
**Status**: Evaluation firmware — most ATA6847 safety features are disabled
**Goal**: Production-grade drone ESC with full hardware protection

---

## Current State vs Target

| Feature | Reference Code | Our Code Now | Target | Priority |
|---------|---------------|-------------|--------|----------|
| BEMF comparators | Disabled | **Enabled** | Enabled | Done |
| Cross-conduction protection | Enabled (700ns) | **Enabled** | Enabled | Done |
| VDS short-circuit shutdown | Conflicting | Enabled (500mV) | Enabled | Done |
| Edge blanking | 1250ns | **1250ns** | 1250ns | Done |
| VGS under-voltage detect | Enabled | **Enabled** | Enabled | Done |
| CSA gain/offset | Gain=16, VRef/2 | **Gain=16, VRef/2** | Keep | Done |
| **HW current limit (ILIM)** | **Disabled** | **Disabled** | **Chopping mode** | **P1** |
| **nIRQ fault monitoring** | **Not in ISR** | **Not used** | **Poll in Timer1** | **P2** |
| **Pre-flight motor diag** | **Disabled** | **Disabled** | **Before ARMED** | **P3** |
| **Thermal pre-warning** | **Not runtime** | **Not used** | **Poll in main loop** | **P4** |
| **Hardware watchdog** | **OFF** | **OFF** | **Timeout mode** | **P5** |
| **Register write protect** | **Not used** | **Not used** | **After init** | **P6** |
| **Charge pump monitoring** | **Not runtime** | **Not used** | **Poll with thermal** | **P7** |
| **Adaptive dead time** | **Disabled** | **Disabled** | **Test and enable** | **P8** |
| **LIN standby** | Normal/Standby | Normal | **Standby** | **P9** |
| **SIR3 fault decode** | Read but not decoded | Read but not decoded | **Decode + log** | **P10** |

---

## P1: Hardware Current Limit — Cycle-by-Cycle Chopping

### Registers

**ILIMCR (0x09)**:
```
Bit 7: ILIMEN    = 1  (ENABLE — currently 0)
Bit 6-3: ILIMFLT = 4  (1000ns filter — already set)
Bit 2: ILIMSDEN  = 0  (CHOPPING mode, NOT shutdown)
                       Currently 1 (shutdown) — MUST change for drone use
Bits 1-0: reserved
```
**Change**: `(1 << 7) | (4 << 3) | (0 << 2)` = 0xA0 (was 0x24)

**ILIMTH (0x0A)**:
```
Bits 6-0: DAC = 7-bit threshold (0-127)
Formula: VILIM_TH = 3.3V / 128 × DAC
Trip current: I = (VILIM_TH - 1.65V) / (Gain × Rshunt)
            = (VILIM_TH - 1.65) / (16 × 0.003)
            = (VILIM_TH - 1.65) / 0.048
```

**DAC lookup table** (Gain=16, Rs=3mΩ, Offset=VRef/2=1.65V):

| DAC | VILIM_TH (V) | Trip Current (A) | Use Case |
|-----|-------------|------------------|----------|
| 64 | 1.650 | 0.0 | — |
| 70 | 1.804 | 3.2 | — |
| 73 | 1.882 | 4.8 | Hurst conservative |
| 80 | 2.063 | 8.6 | Hurst max |
| 85 | 2.191 | 11.3 | A2212 conservative |
| 90 | 2.320 | 14.0 | A2212 normal |
| 95 | 2.449 | 16.6 | A2212 with headroom |
| 100 | 2.578 | 19.3 | A2212 burst |
| 110 | 2.836 | 24.7 | High-current motor |
| 120 | 3.094 | 30.1 | Maximum practical |
| 127 | 3.274 | 33.8 | ADC rail (no protection) |

**Per-motor-profile configuration**:
- Hurst (Profile 0): DAC=80 → 8.6A trip (2.5× rated 3.4A)
- A2212 (Profile 1): DAC=95 → 16.6A trip (allows 9A peak with 80% headroom)

### How Chopping Mode Works

```
PWM cycle starts
  → High-side FET turns ON
  → Current ramps through motor winding
  → ATA6847 CSA monitors shunt voltage continuously
  → If V_shunt × Gain + Offset > VILIM_TH for > ILIMFLT:
      → ATA6847 turns OFF the active high-side gate
      → Current freewheels through body diode / low-side
      → SIR1.ILIM bit is SET (can be polled or generates nIRQ)
  → Next PWM cycle: gate turns ON normally
  → If still overcurrent: chops again
  → Motor keeps running at clamped current
```

No firmware intervention needed — the ATA6847 handles it autonomously.
The firmware only needs to:
1. Configure ILIMCR and ILIMTH at init
2. Optionally monitor SIR1.ILIM to know chopping is occurring
3. Optionally reduce duty in software when ILIM is active (AM32 approach)

### Implementation Steps

1. Add `ILIM_DAC_HURST` and `ILIM_DAC_A2212` defines to `garuda_config.h`
2. Change ILIMCR write in `hal_ata6847.c`: ILIMEN=1, ILIMSDEN=0
3. Set ILIMTH per motor profile
4. Add SIR1.ILIM bit check to nIRQ/fault monitoring (P2)
5. Add telemetry flag when ILIM is chopping
6. Test with bench current limit at known values
7. Verify motor keeps spinning during chop events

### Test Plan

1. Set DAC=73 (4.83A trip) for easy bench testing
2. Run motor, slowly increase pot until 4.83A peak is reached
3. Verify: motor keeps running (chopping, not shutdown)
4. Check SIR1.ILIM flag is set
5. Verify: reducing pot → ILIM stops chopping → normal operation resumes
6. Increase DAC to per-profile value, repeat at higher current
7. Test locked-rotor: hold motor shaft, apply throttle, verify ILIM chops without FET damage

### Risks

- **Nuisance trips during commutation switching transients**: ILIMFLT=1000ns should filter these. If not, increase to 1250-1500ns.
- **CSA offset drift with temperature**: VRef/2 offset tracks with VRef, so drift is proportional. Should be stable.
- **Interaction with BEMF sensing**: ILIM chops the gate, which interrupts BEMF. After blanking, BEMF should be clean. Monitor for false ZC during chop events.

---

## P2: nIRQ Fault Monitoring

### Hardware

```
ATA6847 nIRQ (pin 35) → RD1 → PPS INT1R = 0x0041 (already routed)
Active LOW: nIRQ=0 when any enabled fault occurs
```

### Implementation

**Option A — Poll in Timer1 ISR (simplest)**:
```c
// In Timer1 ISR, every tick (50µs):
if (!nIRQ_GetValue())  // nIRQ asserted (active low)
{
    gData.ata6847FaultPending = true;
}
```

Then in main loop:
```c
if (gData.ata6847FaultPending)
{
    uint8_t sir1 = HAL_ATA6847_ReadReg(ATA_SIR1);
    // Decode and log fault
    // Take appropriate action (stop motor, reduce duty, etc.)
    HAL_ATA6847_ClearFaults();
    gData.ata6847FaultPending = false;
}
```

**Option B — INT1 interrupt (faster, more complex)**:
Enable INT1 falling-edge interrupt, set flag in ISR, process in main loop.

**Recommendation**: Start with Option A (poll). 50µs latency is fine — the ATA6847
has already taken protective action (gate chopping or shutdown) before we even read it.

### Fault Decode Table (SIR1)

| Bit | Flag | Meaning | Action |
|-----|------|---------|--------|
| 0 | VGSUV | Gate not fully enhanced | Stop motor, log |
| 1 | VDSSC | VDS short circuit | Motor already stopped by HW, log which FET (SIR3) |
| 2 | OVTF | Over-temperature failure | Motor stopped by HW, cool down |
| 3 | LDOF | LDO regulator failure | Stop motor, check SIR2 for details |
| 4 | ILIM | Current limit tripped | Log, optionally reduce duty |
| 5 | SYS | System event | Check SIR5 (clock fail, SPI error) |
| 6 | WAKE | Wake-up event | Ignore during operation |
| 7 | VSUPF | Supply failure (VDH OV or VS UV) | Stop motor |

### Response Matrix

| Fault | Severity | Response |
|-------|----------|----------|
| ILIM (bit 4) | Info | Log + optionally reduce duty. Motor keeps running. |
| VDSSC (bit 1) | Critical | Motor already dead. Log SIR3 (which FET). Enter FAULT. |
| OVTF (bit 2) | Critical | Motor stopped by HW. Enter FAULT. Wait for cool-down. |
| VGSUV (bit 0) | Serious | Gate drive failure. Stop motor. Check charge pump (DSR2). |
| VSUPF (bit 7) | Critical | Power supply issue. Stop motor. |
| SYS (bit 5) | Varies | Read SIR5. SPI error=retry, clock fail=fatal. |

---

## P3: Pre-Flight Motor Line Diagnostics

### Concept

Before ARMED phase, inject small test currents through each motor winding
using ATA6847's internal current sources. Check for:
- Open circuit (disconnected motor wire)
- Short to ground
- Phase-to-phase short
- Abnormal impedance

### Registers

**MLDCR (0x0E)**: Motor Line Diagnosis Control
```
Bit 0: MLDEN    — Enable motor line diagnosis
Bit 1: SINK1    — Sink current source, phase 1
Bit 2: SOURCE1  — Source current source, phase 1
Bit 3: SINK2    — Phase 2 sink
Bit 4: SOURCE2  — Phase 2 source
Bit 5: SINK3    — Phase 3 sink
Bit 6: SOURCE3  — Phase 3 source
```

**MLDRR (0x12)**: Motor Line Diagnosis Results
- Per-phase diagnostic result (open/short/OK)

### Sequence

1. GDU must be in Standby (not Normal) for motor line diagnosis
2. Enable MLDEN + SOURCE1 → check MLDRR for phase 1
3. Enable SINK1 → check MLDRR
4. Repeat for phases 2 and 3
5. Disable MLDEN
6. If all OK → proceed to GDU Normal
7. If any fault → enter FAULT with diagnostic code

### Implementation Location

Insert between ARMED and ALIGN states, or within ARMED countdown:
```
IDLE → ARMED → [motor line diag] → ALIGN → OL_RAMP → CL
```

---

## P4: Runtime Thermal Monitoring

### Registers

**DSR1 (0x10)** bits 3-6: Over-temperature PRE-warnings
```
Bit 3: LOTPWS    — LIN over-temp pre-warning
Bit 4: GDUOTPWS  — GDU over-temp pre-warning
Bit 5: VDD1OTPWS — VDD1 LDO over-temp pre-warning
Bit 6: VDD2OTPWS — VDD2 over-temp pre-warning
```

These fire at a LOWER temperature than the hard shutdown threshold.
Gives firmware time to reduce load before the ATA6847 kills everything.

### Implementation

Poll DSR1 every 200ms from main loop:
```c
uint8_t dsr1 = HAL_ATA6847_ReadReg(ATA_DSR1);
if (dsr1 & 0x78)  // Any over-temp pre-warning
{
    // Reduce max duty by 50%, set thermal derating flag
    // Log which subsystem is hot
}
```

---

## P5: Hardware Watchdog

### Registers

**WDCR1 (0x21)**:
```
Bits 7-5: WDC = 010 (Timeout mode)
Bit 1: WDLW = 0 (short window)
Bits 4-3: WDPRE = 01 (prescaler)
```

**WDCR2 (0x22)**:
```
Bits 7-4: WWDP = period (e.g., 100ms)
Bits 3-0: WRPL = reset pulse length
```

**WDTRIG (0x20)**: Write alternating 0x55/0xAA to kick

### Implementation

1. Configure WDCR1/WDCR2 for ~100ms timeout
2. Add `HAL_ATA6847_KickWatchdog()` — writes 0x55 or 0xAA (alternating)
3. Call from main loop every iteration
4. If main loop hangs → ATA6847 resets MCU after 100ms
5. MCU boots into IDLE, GDU off, motor safe

### Caution

Must ensure the main loop can always kick within the timeout period.
Heavy UART output or SPI operations could delay the main loop.
Set timeout conservatively (100ms, not 10ms).

---

## P6: Register Write Protection

### Implementation

After all init is complete (end of `HAL_ATA6847_Init()`):
```c
HAL_ATA6847_WriteReg(ATA_RWPCR, 0x01);  // Lock registers
```

To modify registers at runtime:
```c
HAL_ATA6847_WriteReg(ATA_RWPCR, 0x00);  // Unlock
// ... modify registers ...
HAL_ATA6847_WriteReg(ATA_RWPCR, 0x01);  // Re-lock
```

---

## P7: Charge Pump Monitoring

### Register

**DSR2 (0x11)**:
```
Bit 0: VCPUVS — Charge pump under-voltage
Bit 1: VGUVS  — Gate voltage under-voltage
```

### Implementation

Add to P4 thermal polling (same 200ms main-loop poll):
```c
uint8_t dsr2 = HAL_ATA6847_ReadReg(ATA_DSR2);
if (dsr2 & 0x03)  // Charge pump or gate UV
{
    // High-side FETs not fully enhanced → stop motor
    EnterFault(FAULT_ATA6847);
}
```

---

## P8: Adaptive Dead Time

### Register

**GDUCR3 (0x07)** bits 7-4:
```
Bits 7-6: ADDTHS = 01 (50-150ns high-side adaptive DT)
Bits 5-4: ADDTLS = 01 (50-150ns low-side adaptive DT)
```

### Benefit

Reduces effective dead time from fixed 700ns to 50-150ns when the ATA6847
detects the MOSFET has actually turned off (monitors VDS). This improves:
- Efficiency (less body diode conduction)
- BEMF sensing quality (less voltage clamping during dead time)

### Risk

Needs validation with oscilloscope on actual switching waveforms.
If VDS detection is noisy, could cause shoot-through.
Start with minimum range (01) and verify.

---

## P9: LIN Standby

### Change

```c
HAL_ATA6847_WriteReg(ATA_LOPMCR, 0x01);  // Standby (was 0x02 = Normal)
```

Saves ~5-10mA. No functional impact (we don't use LIN).

---

## P10: SIR3 Fault Decode

### Register

**SIR3 (0x15)**:
```
Bit 0: SCLS1 — Low-side switch 1 (Phase A) short circuit
Bit 1: SCLS2 — Low-side switch 2 (Phase B) short circuit
Bit 2: SCLS3 — Low-side switch 3 (Phase C) short circuit
Bit 3: SCHS1 — High-side switch 1 (Phase A) short circuit
Bit 4: SCHS2 — High-side switch 2 (Phase B) short circuit
Bit 5: SCHS3 — High-side switch 3 (Phase C) short circuit
Bit 6: VCPUV — Charge pump under-voltage
Bit 7: VGUV  — Gate voltage under-voltage
```

### Implementation

When SIR1.VDSSC is set, read SIR3 before clearing:
```c
uint8_t sir3 = HAL_ATA6847_ReadReg(ATA_SIR3);
HAL_UART_WriteString("SC:");
if (sir3 & 0x01) HAL_UART_WriteString("LS_A ");
if (sir3 & 0x02) HAL_UART_WriteString("LS_B ");
if (sir3 & 0x04) HAL_UART_WriteString("LS_C ");
if (sir3 & 0x08) HAL_UART_WriteString("HS_A ");
if (sir3 & 0x10) HAL_UART_WriteString("HS_B ");
if (sir3 & 0x20) HAL_UART_WriteString("HS_C ");
```

---

## Implementation Order

```
Phase 1 (Current Session):
  P1: ILIM chopping mode — 2 register changes + per-profile DAC
  P2: nIRQ polling — poll in Timer1, decode in main loop
  Test with prop, verify chopping works

Phase 2 (Next Session):
  P3: Motor line diagnostics — pre-flight check
  P4: Thermal monitoring — main loop polling
  P7: Charge pump monitoring — add to P4 polling
  P10: SIR3 decode — enhance fault logging

Phase 3 (Production Hardening):
  P5: Hardware watchdog — needs careful timeout tuning
  P6: Register write protection — trivial but needs unlock for runtime changes
  P9: LIN standby — trivial power saving
  P8: Adaptive dead time — needs oscilloscope validation
```
