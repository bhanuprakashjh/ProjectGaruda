# dsPIC33AK128MC106 ADC High-Speed Trigger Investigation

## Objective

Achieve continuous high-speed (~1-5 Msps) single-sample conversions on ADC channels
AD1CH5 and AD2CH1, with the digital comparator checking each individual result,
for BEMF zero-crossing detection at >120K eRPM.

The ADC hardware is capable of **4.9 Msps per channel** (100 MHz TAD clock), but we
can only trigger conversions at **24 kHz** (PWM-synced). We're using 0.0005% of the
ADC's bandwidth.

## Hardware

- **MCU**: dsPIC33AK128MC106 (32-bit DSC, dual 12-bit ADC cores AD1 and AD2)
- **Board**: MCLV-48V-300W development board with DIM
- **ADC clock**: 100 MHz (CLK6, PLL1 FOUT 200 MHz / 2) — confirmed in `hal/clock.c`
- **ADC pins**: RB8 (AD1AN11, Phase B), RB9 (AD2AN10, Phase A), RA10 (AD2AN7, Phase C)
- **Datasheet**: DS70005539C

## ADC Timing at 100 MHz TAD Clock

- 1 TAD = 10ns
- At SAMC=3: sampling = 6.5 TADs (65ns), conversion = 14 TADs (140ns)
- **Total = 20.5 TADs = 205ns per conversion = ~4.9 Msps per channel**
- Headline "40 Msps" is across both cores with multiple channels in parallel

## What Works (baseline)

Both ADC cores are initialized and enabled (`AD1CONbits.ON = 1`, wait for `ADRDY`).
**NOTE: `AD1CONbits.MODE` is NOT explicitly set** (see Investigation Point #4 below).

These channels work perfectly with `TRG1SRC = 4` (PWM1 ADC Trigger 1, 24kHz):

```c
// Existing 24kHz channels — all work
AD1CH0: PINSEL=11, TRG1SRC=4  // Phase B (RB8) — ISR source
AD2CH0: PINSEL=10, TRG1SRC=4  // Phase A (RB9), muxed with Phase C (PINSEL=7)
AD1CH1: PINSEL=10, TRG1SRC=4  // Potentiometer (RA11)
AD1CH4: PINSEL=6,  TRG1SRC=4  // Vbus (RA7)

// High-speed comparator channels — ALSO work with TRG1SRC=4
AD1CH5: PINSEL=11, TRG1SRC=4  // Phase B (same pin as CH0)
AD2CH1: PINSEL=10, TRG1SRC=4  // Phase A/C (same pin as AD2CH0, muxed)
```

With TRG1SRC=4, the digital comparator on AD1CH5 and AD2CH1 correctly detects BEMF
zero crossings and fires interrupts. Confirmed with tens of thousands of successful
detections and zero misses in hardware testing.

## Channel Configuration (identical across all attempts except TRG1SRC/TRG2SRC)

```c
AD1CH5CONbits.PINSEL = 11;   // RB8 = AD1AN11
AD1CH5CONbits.SAMC   = 3;    // 6.5 TADs sample time
AD1CH5CONbits.LEFT   = 0;    // Right-aligned
AD1CH5CONbits.DIFF   = 0;    // Single-ended
AD1CH5CONbits.CMPMOD = 0;    // Comparator off at init (enabled later per step)
// MODE bits (31:30) = 00 (single sample, default)
```

## Init Sequence

1. `InitializeADCs()` — configures CH0/CH1/CH4 with TRG1SRC=4, turns on AD1 and AD2,
   waits for ADRDY. Does NOT set `AD1CONbits.MODE` or `AD2CONbits.MODE`.
2. `HAL_ADC_InitHighSpeedBEMF()` — configures CH5 and AD2CH1 (ADC cores already ON)
3. PWM is started later by `HAL_PWM_Enable()`

## Failed Attempts

### Attempt 1: Software Trigger + Immediate Re-Trigger (TRG2SRC=2)

```c
AD1CH5CONbits.TRG1SRC = 1;   // Software trigger
AD1CH5CONbits.TRG2SRC = 2;   // Immediate re-trigger
// Same for AD2CH1
// Then call: AD1SWTRGbits.CH5TRG = 1; AD2SWTRGbits.CH1TRG = 1;
```

**Result**: AD1CH5DATA = 0x0000, AD2CH1DATA = 0x0000. Comparator never fires.

**Analysis**: Datasheet register description for MODE[1:0] (ADnCHxCON bits 31:30):
- MODE=00 (single): "Single sample initiated by TRG1SRC trigger" — no mention of TRG2SRC
- MODE=10 (integration): "First conversion by TRG1SRC, all others by TRG2SRC"
- MODE=11 (oversampling): Same as integration

**Conclusion**: TRG2SRC is IGNORED in MODE=00. Only MODE=10/11 use TRG2SRC.
But even TRG1SRC=1 (SW trigger) produced zero — the SW trigger itself didn't work.

### Attempt 2: RPTCNT Repeat Timer (TRG1SRC=3)

```c
AD1CONbits.RPTCNT = 0;       // 1 ADC clock between triggers (fastest)
AD2CONbits.RPTCNT = 0;
AD1CH5CONbits.TRG1SRC = 3;   // RPTCNT repeat timer
AD2CH1CONbits.TRG1SRC = 3;
```

**Result**: AD1CH5DATA = 0x0000, AD2CH1DATA = 0x0000. Comparator never fires.

**Hardware diagnostic** (from x12mid.csv watch capture):
```
hwzc.totalZcCount     = 0x00000000  // Zero ZC detections
hwzc.totalMissCount   = 0x00000003  // Hit miss limit (3)
hwzc.dbgLatchDisable  = 0x01        // Latched disabled
hwzc.dbgTimeoutAdcVal = 0x0000      // ADC reading ZERO at timeout
hwzc.dbgTimeoutThresh = 0x00C0      // Threshold was set correctly (192)
hwzc.dbgTimeoutCmpmod = 0x03        // Comparator configured (rising, greater-than)
hwzc.dbgTimeoutCmpstat= 0x00        // Comparator never triggered
hwzc.dbgTimeoutCore   = 0x02        // Was on AD2CH1
```

Motor fell back to software ZC gracefully (15,482 SW ZC confirmations, zero desyncs).

---

## SOLUTION: SCCP3 Peripheral Trigger (TRG1SRC=14)

### Root Cause of Failed Attempts

Two issues combined to prevent non-PWM triggers from working:

1. **TRG2SRC=2 (immediate re-trigger) is ignored in MODE=00**. Only MODE=10
   (integration) and MODE=11 (oversampling) use TRG2SRC for repeat samples
   within an accumulation window.

2. **Channel configuration after ADC ON is unpredictable for non-PWM triggers**.
   The datasheet warns that configuring channel registers while ADON=1 has
   undefined behavior for certain trigger sources. PWM triggers (TRG1SRC=4)
   happen to tolerate post-ON configuration, but SW triggers (TRG1SRC=1) and
   RPTCNT (TRG1SRC=3) do not.

### The Fix: Two Changes

1. **Move CH5/CH1 configuration BEFORE ADC ON** in `InitializeADCs()`:
   ```c
   // Configure high-speed channels BEFORE AD1CONbits.ON = 1
   #if FEATURE_ADC_CMP_ZC
       AD1CH5CONbits.PINSEL = 11;      // Phase B (RB8)
       AD1CH5CONbits.SAMC = HWZC_SAMC;
       AD1CH5CONbits.LEFT = 0;
       AD1CH5CONbits.DIFF = 0;
       AD1CH5CONbits.TRG1SRC = 14;     // SCCP3 Trigger out
       AD1CH5CONbits.TRG2SRC = 0;      // No secondary trigger
       AD1CH5CONbits.CMPMOD = 0;       // Comparator off initially
       AD1CH5CMPLO = 0;
       AD1CH5CMPHI = 0;

       AD2CH1CONbits.PINSEL = 10;      // Phase A (RB9), muxed with Phase C (7)
       AD2CH1CONbits.SAMC = HWZC_SAMC;
       AD2CH1CONbits.LEFT = 0;
       AD2CH1CONbits.DIFF = 0;
       AD2CH1CONbits.TRG1SRC = 14;     // SCCP3 Trigger out
       AD2CH1CONbits.TRG2SRC = 0;
       AD2CH1CONbits.CMPMOD = 0;
       AD2CH1CMPLO = 0;
       AD2CH1CMPHI = 0;
   #endif

   AD1CONbits.ON = 1;   // NOW turn on ADC
   while (!AD1CONbits.ADRDY);
   ```

2. **Use SCCP3 peripheral timer as trigger source** (TRG1SRC=14):
   ```c
   // SCCP3 configured as periodic timer at 1 MHz
   void HAL_SCCP3_InitPeriodic(uint32_t periodTicks)
   {
       CCP3CON1 = 0;
       CCP3CON1bits.T32 = 1;        // 32-bit mode
       CCP3CON1bits.MOD = 0b0000;   // Timer mode
       CCP3CON1bits.TMRPS = 0b00;   // Prescaler 1:1
       CCP3CON1bits.CLKSEL = 0b000; // FCY clock (100 MHz)
       CCP3PR = periodTicks;         // 100 ticks = 1 MHz
       CCP3TMR = 0;
       CCP3CON1bits.ON = 1;
   }
   ```

### Why SCCP3 Works

Datasheet Table 15-3 lists trigger sources:
```
01100 (12) = SCCP1 Trigger out
01101 (13) = SCCP2 Trigger out
01110 (14) = SCCP3 Trigger out  ← Our choice
01111 (15) = SCCP4 Trigger out
00100 (4)  = PWM1 ADC Trigger 1
```

PWM trigger (TRG1SRC=4) and SCCP trigger (TRG1SRC=14) are both **peripheral
triggers** — they use the same hardware path inside the ADC. Internal triggers
(TRG1SRC=1 software, TRG1SRC=3 RPTCNT) appear to use a different mechanism
that requires channels to be configured before ADC ON.

Datasheet Example 15-10 (line 131091) confirms `TRG1SRC = 12` (SCCP1 Trigger
out) as a valid ADC trigger source.

### Hardware Verification Results

Tested on Hurst DMB0224C10002 motor on MCLV-48V-300W board, 2026-02-17.
SCCP3 configured at 1 MHz (period = 100 FCY ticks).

#### x13min.csv — Below Crossover (~4,138 eRPM)
```
hwzc.enabled     = false        // Below 5000 eRPM crossover
hwzc.totalZcCount  = 0          // HW ZC never activated
hwzc.totalMissCount = 0
zcDiag.zcConfirmedCount = 0x022D (557)  // Software ZC running fine
zcDiag.zcDesyncCount    = 0
```
Software ZC handles low speed perfectly. HW ZC stays dormant.

#### x13mid.csv — Mid Speed (~8,300 eRPM)
```
hwzc.enabled        = true
hwzc.totalZcCount   = 0x00002842 (10,306)    // 10,306 HW ZC detections
hwzc.totalMissCount = 0x00000000             // ZERO misses
hwzc.totalCommCount = 0x00002842 (10,306)    // All comms from HW ZC
hwzc.missCount      = 0                      // Current streak: 0 misses
hwzc.stepPeriodHR   = 0x0001D6C4 (120,516 SCCP2 ticks = 1.2ms)
hwzc.dbgAdvanceDeg  = 5                      // 5° timing advance
```

#### x13midplus.csv — Mid-High Speed (~8,700 eRPM)
```
hwzc.totalZcCount   = 0x0000232B (9,003)     // 9,003 HW ZC detections
hwzc.totalMissCount = 0x00000000             // ZERO misses
hwzc.stepPeriodHR   = 0x0001BF4E (114,510 ticks = 1.15ms)
hwzc.dbgAdvanceDeg  = 5                      // 5° timing advance
```

#### x13max.csv — Full Speed (~16,460 eRPM)
```
hwzc.totalZcCount   = 0x00005785 (22,405)    // 22,405 HW ZC detections
hwzc.totalMissCount = 0x00000000             // ZERO misses
hwzc.totalCommCount = 0x00005785 (22,405)    // All comms from HW ZC
hwzc.stepPeriodHR   = 0x0000ED51 (60,753 ticks = 607us)
hwzc.dbgAdvanceDeg  = 12                     // 12° timing advance (auto-scaled)
```

#### Summary: SCCP3 Trigger at 1 MHz

| Speed | eRPM | HW ZC Count | Misses | Miss Rate | Timing Advance |
|-------|------|-------------|--------|-----------|----------------|
| Below crossover | ~4,138 | 0 (SW ZC) | N/A | N/A | 2° |
| Mid | ~8,300 | 10,306 | 0 | 0.000% | 5° |
| Mid-High | ~8,700 | 9,003 | 0 | 0.000% | 5° |
| Max (Hurst) | ~16,460 | 22,405 | 0 | 0.000% | 12° |

**Zero misses across 41,714 total HW ZC detections.** Perfect detection.

#### x14 Series — 4x Oversampling Enabled (MODE=11, ACCNUM=00)

After enabling 4-sample hardware oversampling on the comparator channels,
re-tested across the speed range. Each SCCP3 trigger at 1 MHz now produces
a burst of 4 conversions (820ns), averaged via right-shift by 2 bits.
Comparator fires on the averaged result. +6 dB noise immunity.

**x14mid.csv — Mid Speed (~10,000 eRPM)**
```
hwzc.enabled        = true
hwzc.totalZcCount   = 0x00004575 (17,781)    // 17,781 HW ZC detections
hwzc.totalMissCount = 0x00000000             // ZERO misses
hwzc.totalCommCount = 0x00004575 (17,781)    // All comms from HW ZC
hwzc.missCount      = 0                      // Current streak: 0 misses
hwzc.stepPeriodHR   = 0x00018426 (99,366 SCCP2 ticks = 994us)
hwzc.dbgAdvanceDeg  = 6                      // 6° timing advance
```

**x14high.csv — Full Speed (~18,650 eRPM)**
```
hwzc.enabled        = true
hwzc.totalZcCount   = 0x00005105 (20,741)    // 20,741 HW ZC detections
hwzc.totalMissCount = 0x00000000             // ZERO misses
hwzc.totalCommCount = 0x00005105 (20,741)    // All comms from HW ZC
hwzc.missCount      = 0                      // Current streak: 0 misses
hwzc.stepPeriodHR   = 0x0000D146 (53,574 SCCP2 ticks = 536us)
hwzc.dbgAdvanceDeg  = 13                     // 13° timing advance
potRaw              = 0x0FFF                  // Full throttle
```

| Test | Mode | eRPM | HW ZC Count | Misses | Timing Advance |
|------|------|------|-------------|--------|----------------|
| x13 series | Single sample (MODE=00) | 4K-16K | 41,714 | 0 | 2-12° |
| x14mid | 4x oversample (MODE=11) | ~10,000 | 17,781 | 0 | 6° |
| x14high | 4x oversample (MODE=11) | ~18,650 | 20,741 | 0 | 13° |

**Zero misses across 80,236 total HW ZC detections (single-sample + oversampled).**
4x oversampling introduces zero regression while adding +6 dB noise immunity.

### Theoretical eRPM Capability

At 1 MHz trigger rate with 4x oversampling (1 averaged result per trigger):

| eRPM | Step Period | Samples/Step | Sufficient? |
|------|------------|--------------|-------------|
| 20,000 | 500us | 500 | Excellent |
| 60,000 | 167us | 167 | Excellent |
| 120,000 | 83us | 83 | Good |
| 300,000 | 33us | 33 | Good |
| 500,000 | 20us | 20 | Adequate |

The previous PWM-synced limit of ~120K eRPM (2 samples/step at 24kHz) is now
removed. At 1 MHz, we have 33+ samples per step even at 300K eRPM — sufficient
for all FPV drone motors (14-pole at 43K mechanical RPM = 301K eRPM).

---

## Investigation Points — Resolved

### 1. Missing `ADnCONbits.MODE = 2` ("RUN mode")

**Status**: Dead end. The `ADnCON.OMODE[1:0]` field (bits 25:24) is **read-only
status** — it reflects the current operating state of the ADC core, not a writable
control. The XC-DSC header maps `AD1CONbits.MODE` to this field. Datasheet code
examples that write `AD1CONbits.MODE = 2` may be errata or refer to a different
silicon revision. On dsPIC33AK128MC106, the ADC enters run mode automatically
when `ON=1` and `ADRDY` asserts. Our code correctly omits this write. Both PWM
and SCCP3 triggers work without it. No action needed.

### 2. Channel Configuration After ADC ON

**Status**: CONFIRMED as critical issue for non-PWM triggers. Moving CH5/CH1
configuration before `AD1CONbits.ON = 1` was one of the two required fixes.
Post-ON configuration works for PWM triggers but NOT for SCCP triggers.

### 3. RPTCNT Timer Enable

**Status**: Not usable on this silicon. TRG1SRC=3 (RPTCNT) produced ADC=0x0000
on both AD1CH5 and AD2CH1 regardless of init ordering. The device-specific
trigger table treats this value as reserved/non-functional on dsPIC33AK128MC106.
Use peripheral triggers (TRG1SRC=4/12-15) instead.

### 4. Errata for Higher-Numbered Channels

**Status**: Not the issue. CH5 and AD2CH1 work perfectly with SCCP3 triggers.
The problem was init ordering and trigger source type, not channel number.

### 5. Integration Mode + Comparator Interaction

**Status**: Researched — see Digital Filter Modes section below.

---

## Digital Filter Modes (dsPIC33AK ADC)

The ADC supports three hardware filtering modes via `ADnCHxCON.MODE[1:0]`.
These are relevant for noise immunity in production drone environments where
switching noise, demagnetization ringing, and Vbus ripple can cause false
zero-crossing detections.

### Overview

| MODE | Name | Description | TRG2SRC Used? | Comparator Fires On |
|------|------|-------------|---------------|---------------------|
| 00 | Single | One sample per trigger | No | Each sample |
| 01 | Window | Not relevant for ZC | No | N/A |
| 10 | Integration | Sum of N samples | Yes (repeat trigger) | Final accumulated sum |
| 11 | Oversampling | Average of N samples | Yes (repeat trigger) | Final shifted result |

**Key finding**: In MODE=10 and MODE=11, `TRG2SRC=2` (immediate re-trigger)
IS used — it triggers the repeat conversions within the accumulation window.
This is the same TRG2SRC=2 that is IGNORED in MODE=00. The behavior difference
is documented in the datasheet register descriptions:
- MODE=00: "Single sample initiated by TRG1SRC trigger" (TRG2SRC ignored)
- MODE=10/11: "First conversion by TRG1SRC, all others by TRG2SRC"

### Mode 11: Oversampling (Recommended for Production)

**Configuration**: `ADnCHxCON.MODE = 0b11`, `ACCNUM[1:0]` selects sample count.

| ACCNUM | Samples | Result Bits | Right Shift | Effective Resolution | SNR Gain |
|--------|---------|-------------|-------------|---------------------|----------|
| 00 | 4 | 13-bit | 1 bit | 12.5-bit | +6 dB (2x) |
| 01 | 16 | 14-bit | 2 bits | 13-bit | +12 dB (4x) |
| 10 | 64 | 15-bit | 3 bits | 13.5-bit | +18 dB (8x) |
| 11 | 256 | 16-bit | 4 bits | 14-bit | +24 dB (16x) |

**How it works**:
1. TRG1SRC fires → first sample acquired
2. TRG2SRC fires → repeat samples (N-1 more)
3. All N samples are accumulated internally
4. Result is right-shifted by log2(√N) bits (implicit averaging)
5. **Comparator fires on the final shifted result** — not on each individual sample
6. `ADnCHxDATA` contains the averaged result

**Critical: TRG2SRC behavior in oversampling mode**:
When `TRG2SRC=2` (immediate re-trigger), each completed conversion immediately
starts the next one within the accumulation window. This creates a fast burst
of N conversions per TRG1SRC trigger:
- TRG1SRC fires → sample 1 (205ns) → TRG2SRC → sample 2 (205ns) → ... → sample N
- Total burst time = N × 205ns (at SAMC=3)
- After burst completes, channel waits for next TRG1SRC

**`ACCBRST` bit**: When set, the oversampling burst is non-interruptible —
higher-priority channels cannot preempt the accumulation. Recommended for
BEMF channels to prevent split-accumulation artifacts from 24kHz PWM-triggered
channels interrupting mid-burst.

**Effective sample rates with SCCP3 at 1 MHz**:

| ACCNUM | Samples | Burst Time | Effective Rate | 300K eRPM Samples/Step |
|--------|---------|------------|----------------|----------------------|
| 00 (4) | 4 | 820ns | ~1 MHz (1 result/trigger) | 33 filtered results |
| 01 (16) | 16 | 3.28us | ~305 kHz | 10 filtered results |
| 10 (64) | 64 | 13.1us | ~76 kHz | 2.5 results |
| 11 (256) | 256 | 52.5us | ~19 kHz | 0.6 results |

**Recommendation**: `ACCNUM=00` (4 samples). Provides +6 dB noise reduction
while maintaining the full 1 MHz result rate. The 820ns burst fits comfortably
within the 1us SCCP3 trigger period (100 ticks at 100 MHz). Higher ACCNUM
values reduce the effective comparator check rate too much for high-eRPM motors.

**Burst vs trigger period constraint**: The accumulation burst must complete
BEFORE the next SCCP3 trigger fires. At 1 MHz:
- SCCP3 period = 1us = 1000ns
- 4-sample burst = 820ns → fits (180ns margin)
- 16-sample burst = 3280ns → does NOT fit at 1 MHz (need SCCP3 ≤ 305 kHz)

If using ACCNUM=01 (16 samples), either:
- Lower SCCP3 frequency to 250 kHz (400 ticks period), or
- Use `ACCBRST=1` and accept that some triggers will be ignored during burst

**Configuration example** (4-sample oversampling at 1 MHz):
```c
// In InitializeADCs(), before ADC ON:
AD1CH5CONbits.MODE = 0b11;       // Oversampling mode
AD1CH5CONbits.ACCNUM = 0b00;     // 4 samples
AD1CH5CONbits.ACCBRST = 1;       // Non-interruptible burst
AD1CH5CONbits.TRG1SRC = 14;      // SCCP3 (1 MHz trigger)
AD1CH5CONbits.TRG2SRC = 2;       // Immediate re-trigger (for repeat samples)
// Same for AD2CH1
```

### Mode 10: Integration (Alternative — Raw Sum)

**Configuration**: `ADnCHxCON.MODE = 0b10`, `ADnCHxCON.CNT[15:0]` sets sample count.

- Accumulates raw sum of 1 to 65,535 samples (no right-shift)
- Result in `ADnCHxDATA` is the full sum
- **Comparator fires on the accumulated sum**, not individual samples
- Threshold (`CMPLO/CMPHI`) must be scaled by the sample count:
  `CMPLO_integration = CMPLO_single × CNT`
- Overflow risk: 12-bit samples × CNT can exceed 32-bit result register
  - Max safe CNT without overflow: 2^32 / 4095 = 1,048,831 samples
  - In practice, CNT=4-256 is reasonable for ZC detection

**When to use**: Integration mode is more flexible than oversampling (arbitrary
sample count 1-65535 vs fixed 4/16/64/256), but requires manual threshold
scaling and does not provide the implicit averaging that makes oversampling
results directly comparable to single-sample thresholds.

**Not recommended for ZC detection**: The threshold scaling requirement adds
complexity. Oversampling (MODE=11) is cleaner — the shifted result has the
same scale as a single-sample result, so `zcThreshold` from the 24kHz ISR
can be used directly in `CMPLO`.

### CIC Filter (Second-Order, Channels 17-19 Only)

The dsPIC33AK ADC includes a second-order CIC (Cascaded Integrator-Comb) filter
using a secondary accumulator (`ADnCHxACC`) with `ACCRO=1` (accumulator
read-only, controlled by hardware).

**Availability**: Only on ADC channels 17, 18, and 19, which have the secondary
accumulator hardware. Our BEMF channels (AD1CH5, AD2CH1) do NOT have this
hardware. **CIC filtering is not usable for BEMF ZC detection on our channels.**

**How it works** (for reference):
1. Channel configured in integration mode (MODE=10) with CNT=M
2. Secondary accumulator performs CIC decimation
3. Output is filtered with frequency response H(z) = (1-z^-M)^2 / (1-z^-1)^2
4. Provides better noise rejection than simple oversampling at the cost of
   group delay and limited channel availability

**Datasheet Example 15-10**: Uses SCCP1 trigger (TRG1SRC=12) with CIC on CH17.
This is the example that confirmed peripheral triggers work with the ADC.

---

## Noise Concerns for Production Drone Applications

### Sources of Noise

In a real drone ESC environment, several noise sources can cause false ZC:

1. **Switching noise**: PWM transitions couple into BEMF sense lines via parasitic
   capacitance and inductance. Amplitude: 100-500mV spikes, duration: 50-200ns.

2. **Demagnetization ringing**: After commutation, the floating phase shows a
   demagnetization spike as current in the motor inductance decays through the
   body diode. Can exceed Vbus momentarily. Duration: 1-10us depending on L/R.

3. **Vbus ripple/sag**: Under load, bus voltage droops during high-current PWM-ON
   periods and recovers during PWM-OFF. Causes threshold to oscillate if derived
   from Vbus. Amplitude: 0.5-3V on a 24V bus.

4. **PWM coupling to floating phase**: Even the undriven phase picks up capacitive
   coupling from the driven phases' PWM switching. Creates small-amplitude noise
   aligned with the PWM frequency.

5. **Ground bounce**: High di/dt in power stage creates transient ground offsets
   between power ground and analog ground. Can shift ADC readings by 1-10 LSB.

### Current Mitigations (Already Implemented)

1. **Blanking period**: After each commutation, the comparator is disabled for
   `HWZC_BLANKING_PERCENT` (5%) of the step period. This masks demagnetization
   artifacts and initial ringing.

2. **Deadband**: `HWZC_CMP_DEADBAND` (4 ADC counts) is added/subtracted from
   the threshold to prevent triggering on noise near the crossing point.

3. **IIR-smoothed threshold**: The zcThreshold is computed with asymmetric IIR
   smoothing (rise τ=0.33ms, fall τ=10.7ms) in the 24kHz ADC ISR. This prevents
   rapid threshold changes from causing false crossings.

### Additional Mitigations for Production

1. **ADC Hardware Oversampling (MODE=11, ACCNUM=00)**:
   4-sample averaging provides +6 dB noise reduction. Single switching spikes
   that affect 1 of 4 samples are attenuated to 25% amplitude in the averaged
   result. Configuration requires only 3 register changes per channel. The
   comparator fires on the averaged result, so existing thresholds work unchanged.

2. **Analog RC Filter on BEMF Sense Lines**:
   A simple RC low-pass on the phase voltage divider output (e.g., R=1kΩ, C=100pF
   → fc=1.6MHz) attenuates switching spikes while preserving the BEMF waveform
   (fundamental frequency at 300K eRPM = 5kHz, well below the filter cutoff).
   This is a board-level change for the custom production board.

3. **Post-Fire Validation (Software)**:
   After the comparator fires, read `ADnCHxDATA` from the 24kHz ISR on the next
   tick and verify the crossing is real (sample is on the correct side of threshold).
   This catches false triggers from transient noise that reverted before the next
   24kHz sample. Adds 1 ADC tick (~42us at 24kHz) latency — negligible at high eRPM.

4. **Dynamic Threshold Refresh**:
   Currently, `CMPLO` is loaded once per commutation from `zcThreshold`. In a noisy
   environment, threshold may drift during the step. Refreshing `CMPLO` from the
   24kHz ISR (which continuously updates `zcThreshold`) keeps the comparator tracking
   the real midpoint. Must disable comparator IE during the update to prevent
   false triggers from partially-written threshold.

5. **Wider Deadband at High Duty**:
   At high duty cycle, BEMF amplitude is large and noise is relatively small.
   At low duty cycle, BEMF amplitude shrinks and noise becomes proportionally
   larger. Scaling `HWZC_CMP_DEADBAND` proportionally to duty (e.g.,
   `deadband = 4 + duty/8192`) improves immunity at low-speed operation.

### Recommended Implementation Priority

1. **Phase F (current)**: Single-sample comparator (MODE=00) at 1 MHz via SCCP3.
   Working perfectly on Hurst motor (zero misses across 40K+ detections).

2. **Phase F+**: Enable 4-sample oversampling (MODE=11, ACCNUM=00). Software-only
   change — modify 3 register values per channel in `InitializeADCs()`. No board
   modification. Test on Hurst motor, then on FPV motor.

3. **Production board**: Add RC filter on BEMF sense lines. Route BEMF pins to
   CMP inputs for analog comparator option as fallback.

---

## Hardware Sample Rate Limits by Mode

| Mode | Samples per Result | Time per Result (SAMC=3) | Max Results/sec | Notes |
|------|-------------------|--------------------------|-----------------|-------|
| Single (00) | 1 | 205ns | 4.9 MHz | Comparator per sample |
| Oversample 4 (11, ACCNUM=00) | 4 | 820ns | 1.2 MHz | +6 dB, fits 1 MHz SCCP3 |
| Oversample 16 (11, ACCNUM=01) | 16 | 3.28us | 305 kHz | +12 dB, need slower SCCP3 |
| Oversample 64 (11, ACCNUM=10) | 64 | 13.1us | 76 kHz | +18 dB, probably too slow |
| Oversample 256 (11, ACCNUM=11) | 256 | 52.5us | 19 kHz | +24 dB, too slow for HW ZC |
| Integration (10, CNT=4) | 4 | 820ns | 1.2 MHz | Raw sum, threshold × CNT |

**1 MHz is our configuration choice, not a hardware limit.** The ADC can run faster
(up to 4.9 MHz in single-sample mode) but 1 MHz provides excellent coverage even
at 300K+ eRPM while leaving headroom for oversampling.

SCCP3 frequency can be adjusted via `HWZC_ADC_SAMPLE_HZ` in `garuda_config.h`.
The derived constant `HWZC_SCCP3_PERIOD` is computed automatically:
```c
#define HWZC_ADC_SAMPLE_HZ    1000000   // 1 MHz (configurable)
#define HWZC_SCCP3_PERIOD     (HWZC_TIMER_HZ / HWZC_ADC_SAMPLE_HZ)  // = 100 ticks
```

Static assert ensures minimum conversion time is met:
```c
_Static_assert(HWZC_SCCP3_PERIOD >= 21,
               "SCCP3 period too short (ADC needs ~205ns = 21 ticks at 100MHz)");
```

---

## Reference: How Other ESCs Handle This

| ESC | MCU | ZC Method | Max eRPM |
|-----|-----|-----------|----------|
| AM32 | STM32 | Hardware analog comparators + EXTI interrupt | 50K+ |
| Bluejay | SiLabs EFM8 | Hardware analog comparators, polled in scan window | 50K+ |
| BLHeli_S | SiLabs EFM8 | Hardware analog comparators | 50K+ |
| ASAC-ESC | STM32C011 | ADC polling in main loop | ~3K |
| Sapog | STM32F105 | ADC multi-sample + least-squares line fit | High |
| VESC | STM32F4 | DMA-driven ADC + PLL/BEMF observer (FOC) | 100K+ |
| **Garuda** | **dsPIC33AK** | **ADC digital comparator + SCCP3 1MHz trigger** | **300K+** |

**Key insight**: No production ESC solves "fast ADC sampling" — they either use
hardware analog comparators (AM32/Bluejay/BLHeli) which operate continuously in
the analog domain, or mathematical observers (VESC) that don't need to catch the
exact crossing instant. Our ADC digital comparator approach achieves comparator-like
event-driven behavior using the digital domain — the hardware comparator examines
each ADC result at 1 MHz without any software intervention, and fires an interrupt
only when the threshold is crossed. This gives us analog-comparator-like performance
using purely digital hardware.

On the MCLV-48V-300W dev board, CMP-based ZC is physically impossible (BEMF pins
can't route to CMP inputs). On a custom board, we'd route BEMF dividers to CMP
inputs like AM32 does, but the ADC comparator approach means we don't have to.

## Relevant Datasheet Sections

- Section 15.4.7: Digital Comparator
- Section 15.5.4: Integration of Multiple Samples (Example 15-6, line 130803)
- Section 15.5.5: Oversampling (Example 15-7, line 130851)
- Section 15.5.6: Channel Comparator (Example 15-8, line 130899)
- Section 15.5.7: Multiple Channels Scan (Example 15-9, line 130962)
- Section 15.5.8: Second-Order CIC Filter (Example 15-10, line 131075)
- Table 15-3: TRG1SRC Trigger Source Selection Bits (line 120795)
- Table 15-4: TRG2SRC Trigger Source Selection Bits (line 120867)
- Register ADnCHxCON: MODE[1:0] at bits 31:30 (line 126803)
- Register ADnCHxCON: ACCNUM[1:0] at bits 29:28 (line 126803)
- Register ADnCHxCON: ACCBRST at bit 27 (line 126803)
- Register ADnCON: RPTCNT[5:0] at bits 23:18 (line 125633)
- ADC Clock: CLK6 = PLL1 FOUT 200MHz / 2 = 100 MHz (clock.c line 179)
