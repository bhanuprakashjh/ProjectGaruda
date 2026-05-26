# Implementation Prompt: PWM/DShot RX Input for PATA6847 CK Board

## Objective

Add PWM and DShot speed control input to the PATA6847 ESC project (dsPIC33CK64MP205 + ATA6847 gate driver) on the EV43F54A evaluation board. Port the proven implementation from dspic33AKESC (dsPIC33AK128MC106) with CK-specific register adaptations.

---

## Target Platform

| Parameter | Value |
|-----------|-------|
| MCU | dsPIC33CK64MP205 — **16-bit** (NOT 32-bit like dsPIC33AK) |
| Package | 48-pin TQFP (Table 6 in DS70005349) |
| Fosc | 200 MHz, **Fcy = 100 MHz** |
| SCCP Clock | 100 MHz (Fcy) |
| Gate Driver | ATA6847 via SPI1 |
| Board | EV43F54A (Microchip ATA6847 QFN40 Evaluation Board) |
| Compiler | XC-DSC v3.30 (XC16-compatible) |
| Project Path | `PATA6847/garuda_ata6847_ck.X/` |
| Datasheet | `PATA6847/dsPIC33CK256MP508-Family-Data-Sheet-DS70005349.pdf` |
| Schematic | `PATA6847/EV43F54A_SCH.PDF` (11 pages) |
| Ref firmware | `PATA6847/EV43F54A_SMO_Lib.X/` (Microchip original) |

---

## Reference Implementation (Working, Hardware-Verified)

```
/media/bhanu1234/Development/ProjectGaruda/dspic33AKESC/
  hal/hal_input_capture.c    — SCCP4 IC + DMA ISR + DshotDecodeFrame (gap-based)
  hal/hal_input_capture.h    — HAL API
  input/rx_decode.c          — protocol detection, lock FSM, throttle conversion
  input/rx_decode.h          — mailbox types, cached throttle externs
```

Diff report: `docs/rx_autodetect_v3_diff_report.md`
Colleague's DShot guide: `/home/bhanu1234/Downloads/DSHOT_DMA_IMPLEMENTATION (1).md`

---

## RX Input Pin Selection — RESOLVED

### Chosen Pin: **RD8 (pin 30) = RP72**

From 48-pin TQFP Table 6 (DS70005349 page 9):
```
Pin 30: RP72/SDO2/PCI19/RD8
```

**Why RD8/RP72:**
- RP-capable → can be mapped to SCCP4 IC input via PPS
- Currently unused — SDO2 (SPI2) is not used in this project
- Pin 30 on the 48-pin TQFP — accessible on EV43F54A board
- Digital-only (ANSELD = 0x0000, so RD8 is already digital)
- 5V tolerant (shaded pin in datasheet) — safe for 3.3V DShot signals

### PPS Mapping (from Table 8-5, DS70005349 page 136)

```c
/* SCCP Capture 4 input → RPINR6, bits ICM4R[7:0] */
/* RP72 = RD8 (Table 8-4: RPINRx value 72 = Port Pin RD8) */
RPINR6bits.ICM4R = 72;   /* Map RD8/RP72 → ICM4 (SCCP4 capture input) */
```

### Pin Configuration

```c
/* In SetupGPIOPorts() / MapGPIOHWFunction(): */
TRISDbits.TRISD8 = 1;    /* RD8 = input */
/* ANSELD bit 8 already 0 (ANSELD = 0x0000) — digital */
```

### Pins NOT to use:
- RC9/RP57 — already U1TX (GSP UART)
- RC8/RP56 — already U1RX (GSP UART)
- RC4/RC5/RC10/RC11 — SPI1 (ATA6847)
- RD1 — nIRQ (ATA6847 fault)
- RC6/RC7/RD10 — BEMF status pins
- RB10-RB15 — PWM outputs
- RD13/RC0 — buttons

---

## dsPIC33CK DMA Register Map (CRITICAL — Different from AK!)

The dsPIC33CK DMA is fundamentally different from dsPIC33AK. All register names and bit layouts differ.

### Source: DS70005349 pages 211-220, Registers 10-1 through 10-3

### Global DMA Enable

| | dsPIC33AK | dsPIC33CK |
|---|-----------|-----------|
| Module enable | `DMACONbits.ON = 1` | **`DMACON = 0x8000`** (bit 15 = DMAEN) |
| Address limits | `DMALOW` / `DMAHIGH` | **`DMAL`** / **`DMAH`** |

```c
/* CK: Enable DMA module */
DMAConbits.DMAEN = 1;     /* bit 15 of DMACON (Register 10-1) */
DMAL = 0x0000;             /* Low address limit (Register 10-1 note) */
DMAH = 0x4FFF;             /* High address limit — covers all 8K SRAM */
```

### Channel Control Register: DMACHn (Register 10-2)

**Single 16-bit register** with all channel config bits:

| Bit | Name | AK Equivalent | CK Value |
|-----|------|---------------|----------|
| 15-13 | — | — | Unimplemented |
| 12 | Reserved | — | Write 0 |
| 11 | — | — | Unimplemented |
| 10 | NULLW | — | 0 (no null write) |
| 9 | RELOAD | `RELOADD`/`RELOADC` | **1 = auto-reload SRC/DST/CNT** |
| 8 | CHREQ | — | 0 (hardware trigger) |
| 7-6 | SAMODE[1:0] | `DMA0CHbits.SAMODE` | 00 = source fixed |
| 5-4 | DAMODE[1:0] | `DMA0CHbits.DAMODE` | 01 = dest incremented |
| 3-2 | TRMODE[1:0] | `DMA0CHbits.TRMODE` | 00 = One-Shot |
| 1 | SIZE | `DMA0CHbits.SIZE` | **0 = word (16-bit)** or **1 = byte** |
| 0 | CHEN | `DMA0CHbits.CHEN` | 1 = enable |

**CRITICAL DIFFERENCE: No 32-bit transfer mode!** The CK DMA is 16-bit only (`SIZE`: 0=word, 1=byte). To capture 32-bit SCCP timestamps, you must either:
- Option A: Use two 16-bit DMA transfers per edge (CCP4BUFL + CCP4BUFH) — complex
- Option B: Read CCP4BUF in the ISR directly (no DMA for capture) — simpler for PWM
- Option C: Use RELOAD mode and capture words, then assemble 32-bit values in ISR

**Recommendation**: For the initial implementation, start with **ISR-based capture** (no DMA) for both PWM and DShot. The CK has `__builtin_disi()` for atomic reads of 32-bit capture values. DMA can be added later if ISR overhead becomes an issue.

### Channel Interrupt Register: DMAINTn (Register 10-3)

| Bit | Name | AK Equivalent | Purpose |
|-----|------|---------------|---------|
| 15 | DBUFWF | — | Buffer write flag |
| 14-8 | CHSEL[6:0] | `DMA0SELbits.CHSEL` | **DMA trigger source** |
| 7 | HIGHIF | — | Address high limit flag |
| 6 | LOWIF | — | Address low limit flag |
| 5 | DONEIF | `DMA0CHbits.DONEEN` (sort of) | **Done flag — read/clear** |
| 4 | HALFIF | — | Half-transfer flag |
| 3 | OVRUNIF | — | Overrun flag |
| 2-1 | — | — | Unimplemented |
| 0 | HALFEN | — | Enable half-transfer interrupt |

**CRITICAL: No `DONEEN` on CK!** The CK DMA always generates an interrupt on completion when `_DMAn_IE` is set. The `DONEIF` flag in `DMAINTn` is a status flag, not a gate. This means the AK's `DONEEN` bug does not apply — but you still need `_DMA0IE = 1` and the correct IEC/IFS register.

### DMA Trigger Source for SCCP4 (Table 10-1, page 219)

```
CHSEL = 0x15 (hex) = SCCP4 Interrupt
```

**This is different from AK** which used `0x1B`.

### Complete CK DMA Channel 0 Config for DShot

```c
/* WARNING: This is for DShot with 16-bit DMA (word mode).
   Each SCCP4 capture triggers one 16-bit word transfer.
   You get CCP4BUFL only (low 16 bits of 32-bit timestamp).
   For full 32-bit timestamps, use ISR-based capture instead. */

DMAConbits.DMAEN = 1;           /* Enable DMA module */
DMAL = 0x0000;
DMAH = 0x4FFF;                  /* Cover all SRAM */

DMACH0 = 0x0000;                /* Clear all bits, disable channel */
DMACH0bits.SAMODE = 0b00;       /* Source fixed (CCP4BUFL) */
DMACH0bits.DAMODE = 0b01;       /* Dest incremented */
DMACH0bits.TRMODE = 0b00;       /* One-Shot */
DMACH0bits.SIZE   = 0;          /* Word (16-bit) */
DMACH0bits.RELOAD = 1;          /* Auto-reload SRC/DST/CNT on re-enable */

DMASRC0 = (uint16_t)&CCP4BUFL; /* Source: SCCP4 capture buffer low word */
DMADST0 = (uint16_t)&dmaEdgeBuf[0][0];
DMACNT0 = 64;                   /* 64 word transfers (N, not N-1 on CK) */

DMAINT0bits.CHSEL = 0x15;       /* SCCP4 trigger */
DMAINT0bits.HALFEN = 0;         /* No half-transfer interrupt */

_DMA0IP = 1;                    /* Low priority */
_DMA0IF = 0;
_DMA0IE = 1;

DMACH0bits.CHEN = 1;            /* Enable channel */
```

---

## SCCP4 Input Capture on dsPIC33CK

### Register Names (from DS30003035 — MCCP/SCCP Family Reference Manual)

The CK SCCP register names use `L`/`H` suffix convention:

| Function | dsPIC33AK | dsPIC33CK |
|----------|-----------|-----------|
| Config word 1 | `CCP4CON1` | `CCP4CON1L` / `CCP4CON1H` |
| Config word 2 | `CCP4CON2` | `CCP4CON2L` / `CCP4CON2H` |
| Status | `CCP4STAT` | `CCP4STATL` |
| Capture buffer | `CCP4BUF` (32-bit) | `CCP4BUFL` (low 16) + `CCP4BUFH` (high 16) |
| Timer | `CCP4TMR` (32-bit) | `CCP4TMRL` + `CCP4TMRH` |
| ISR vector | `_CCP4Interrupt` | `_CCT4Interrupt` (timer) / `_CCP4Interrupt` (IC) |
| ISR flag | `_CCP4IF` | `_CCT4IF` (timer) / `_CCP4IF` (IC) |

### SCCP4 IC Configuration

```c
void HAL_IC4_Init(void)
{
    /* PPS: map RP72 (RD8) to ICM4 input */
    __builtin_write_RPCON(0x0000);  /* unlock PPS */
    RPINR6bits.ICM4R = 72;          /* RP72 = RD8 → ICM4 */
    __builtin_write_RPCON(0x0800);  /* lock PPS */

    /* Configure SCCP4 for Input Capture:
     * T32=1: 32-bit timer (CCP4TMRL:CCP4TMRH)
     * MOD=0b0001: capture every edge (both rise and fall)
     * CLKSEL=0b000: Fcy = 100 MHz */
    CCP4CON1L = 0;
    CCP4CON1Lbits.CCSEL = 1;       /* Input capture mode */
    CCP4CON1Lbits.T32   = 1;       /* 32-bit timer */
    CCP4CON1Lbits.MOD   = 0b0001;  /* Capture every edge */
    CCP4CON1Lbits.CLKSEL = 0b000;  /* Fcy clock = 100 MHz */
    CCP4CON1Lbits.TMRPS  = 0b00;   /* 1:1 prescaler */

    CCP4CON1H = 0;                  /* No sync, no output */
    CCP4CON2L = 0;
    CCP4CON2H = 0;
    CCP4CON3H = 0;

    CCP4STATL = 0;                  /* Clear status (including ICOV) */

    /* ISR priority 4 */
    _CCP4IP = 4;

    /* Clear state */
    icMode = IC_MODE_DETECT;
}

void HAL_IC4_Enable(void)
{
    _CCP4IF = 0;
    _CCP4IE = 1;
    CCP4CON1Lbits.ON = 1;
}
```

### Reading 32-bit Capture Value Atomically

On dsPIC33CK, the 32-bit capture is split across `CCP4BUFL` and `CCP4BUFH`. The SCCP hardware latches both when the low word is read, so:

```c
/* In ISR — read low FIRST, then high (hardware latches on low read) */
uint16_t lo = CCP4BUFL;    /* This latches BUFH */
uint16_t hi = CCP4BUFH;    /* Read latched high word */
uint32_t capture = ((uint32_t)hi << 16) | lo;
```

Alternatively, if `T32=1` and the compiler supports it:
```c
uint32_t capture = CCP4BUF;  /* May work as 32-bit read on some CK devices */
```
Verify in the device header `p33CK64MP205.h` whether `CCP4BUF` is defined as a 32-bit SFR.

---

## Recommended Implementation Strategy

### Phase 1: ISR-Based Capture (No DMA)

Given the CK's 16-bit DMA limitation, start with pure ISR-based capture for both PWM and DShot. This avoids the 32-bit DMA complexity entirely.

```
RD8/RP72 → SCCP4 IC (both edges, 32-bit @ 100MHz)
  → _CCP4Interrupt reads CCP4BUFL+BUFH as uint32_t
  → IC_MODE_DETECT: buffer 16 edges for protocol detection
  → IC_MODE_PWM: rise/fall pairing → pulse width → mailbox
  → IC_MODE_DSHOT: buffer 64 edges in ISR → decode in main loop
```

**For DShot at Fcy=100MHz:**
- DShot600 bit period = 167 counts = 1.67µs
- At 2 edges/bit × 16 bits = 32 ISR entries per frame
- ISR execution: ~20 instructions × 10ns = 200ns per entry
- Total ISR time per frame: 32 × 200ns = 6.4µs
- DShot600 frame time: ~27µs → ISR load = 24% — **acceptable**

**For DShot1200**: ISR load doubles to ~48%. If this is too high, add DMA later (Phase 2) using 16-bit word captures with post-assembly of 32-bit values.

### Phase 2 (Optional): DMA-Assisted DShot

If ISR overhead is too high, use DMA in word mode to capture `CCP4BUFL` values only:
- 16-bit timestamps wrap every 655µs at 100MHz — sufficient for intra-frame timing
- Inter-frame gap detection uses delta (wrapping subtraction handles overflow)
- DshotDecodeFrame works with `uint16_t` edges instead of `uint32_t`
- This eliminates ISR entirely for the capture phase — only DMA TC ISR runs

---

## Integration Points (Already Scaffolded)

The CK project already has all the integration wiring. Verify these work when you enable the feature flags:

### main.c (lines 57-60, 237-291)
```c
#if (FEATURE_RX_PWM || FEATURE_RX_DSHOT || FEATURE_RX_AUTO)
#include "input/rx_decode.h"
#include "hal/hal_input_capture.h"
#endif
// ... RX_Init(), RX_Service(), auto-arm logic all present
```

### garuda_service.c (lines 505-515)
```c
case THROTTLE_SRC_PWM:
case THROTTLE_SRC_DSHOT:
case THROTTLE_SRC_AUTO:
    garudaData.throttle = rxCachedLocked ? rxCachedThrottleAdc : 0;
```

### garuda_config.h
```c
#define FEATURE_RX_PWM    0  /* ← Change to 1 */
#define FEATURE_RX_DSHOT  0  /* ← Change to 1 */
#define FEATURE_RX_AUTO   0  /* ← Change to 1 */
#define RX_DSHOT_EDGES    32 /* ← Change to 64 */
```

### garuda_types.h
Add `RX_PROTO_DSHOT1200` to enum (port from AK).

---

## Files to Create

| File | Source | Adaptation Needed |
|------|--------|-------------------|
| `hal/hal_input_capture.c` | Port from AK | **Heavy** — all register names differ |
| `hal/hal_input_capture.h` | Port from AK | Light — API same, signature update |
| `input/rx_decode.c` | Port from AK | **None** — pure software logic |
| `input/rx_decode.h` | Port from AK | **None** — pure types/externs |

## Files to Modify

| File | Change |
|------|--------|
| `hal/port_config.c` | Add `TRISDbits.TRISD8 = 1` + PPS mapping `RPINR6bits.ICM4R = 72` |
| `garuda_config.h` | Enable RX features, set `RX_DSHOT_EDGES=64` |
| `garuda_types.h` | Add `RX_PROTO_DSHOT1200` |
| `gsp/gsp_commands.c` | Verify DMA register names in RX_INJECT handler |
| `nbproject/configurations.xml` | Add new .c files |
| `nbproject/Makefile-default.mk` | Add new .c files |

---

## CK-Specific Register Cheat Sheet

```c
/* ═══ PPS ═══ */
__builtin_write_RPCON(0x0000);      /* unlock */
RPINR6bits.ICM4R = 72;              /* RD8/RP72 → SCCP4 IC input */
__builtin_write_RPCON(0x0800);      /* lock */

/* ═══ SCCP4 Input Capture ═══ */
CCP4CON1Lbits.CCSEL  = 1;          /* IC mode */
CCP4CON1Lbits.T32    = 1;          /* 32-bit timer */
CCP4CON1Lbits.MOD    = 0b0001;     /* Both edges */
CCP4CON1Lbits.CLKSEL = 0b000;      /* Fcy = 100 MHz */
CCP4CON1Lbits.ON     = 1;          /* Enable */
CCP4STATL             = 0;         /* Clear ICOV + status */
_CCP4IP = 4;  _CCP4IF = 0;  _CCP4IE = 1;

/* Reading 32-bit capture (ISR): */
uint16_t lo = CCP4BUFL;            /* Latches BUFH */
uint16_t hi = CCP4BUFH;
uint32_t capture = ((uint32_t)hi << 16) | lo;

/* ═══ DMA (if used) ═══ */
DMAConbits.DMAEN = 1;              /* Global DMA enable */
DMAL = 0x0000;  DMAH = 0x4FFF;     /* Address limits */
DMACH0bits.CHEN   = 0;             /* Disable for config */
DMACH0bits.SAMODE = 0b00;          /* Src fixed */
DMACH0bits.DAMODE = 0b01;          /* Dst increment */
DMACH0bits.TRMODE = 0b00;          /* One-shot */
DMACH0bits.SIZE   = 0;             /* Word (16-bit) */
DMACH0bits.RELOAD = 1;             /* Auto-reload on re-enable */
DMASRC0 = (uint16_t)&CCP4BUFL;
DMADST0 = (uint16_t)&dmaEdgeBuf[0];
DMACNT0 = 64;
DMAINT0bits.CHSEL = 0x15;          /* SCCP4 trigger (Table 10-1) */
_DMA0IP = 1;  _DMA0IF = 0;  _DMA0IE = 1;
DMACH0bits.CHEN = 1;               /* Enable */

/* DMA ISR re-arm (one-shot mode): */
DMACH0bits.CHEN = 1;               /* Must re-enable after completion */
/* With RELOAD=1, SRC/DST/CNT auto-restore — no manual re-set needed */

/* ═══ Key Differences from AK ═══ */
/* AK: DMACONbits.ON        → CK: DMAConbits.DMAEN                    */
/* AK: DMA0CHbits.*          → CK: DMACH0bits.*  (single register)     */
/* AK: DMA0SELbits.CHSEL    → CK: DMAINT0bits.CHSEL                   */
/* AK: DMA0CHbits.DONEEN    → CK: NOT NEEDED (auto-fires on IE)       */
/* AK: DMALOW/DMAHIGH       → CK: DMAL/DMAH                          */
/* AK: DMA0SRC (32-bit addr)→ CK: DMASRC0 (16-bit addr)               */
/* AK: CHSEL=0x1B (SCCP4)   → CK: CHSEL=0x15 (SCCP4)                 */
/* AK: SIZE=0b10 (32-bit)   → CK: SIZE=0 (16-bit word) NO 32-BIT!    */
/* AK: CCP4BUF (32-bit)     → CK: CCP4BUFL + CCP4BUFH (read L first) */
/* AK: CCP4CON1              → CK: CCP4CON1L + CCP4CON1H              */
/* AK: MOD=0b0011 (every edge) → CK: MOD=0b0001 (every edge) VERIFY! */
```

---

## Bugs That Won't Apply on CK (but verify)

1. **`DONEEN` missing** — CK has no `DONEEN`. DMA interrupt fires automatically when `_DMA0IE=1` and transfer completes. This was the #1 AK bug.
2. **`DMA0SRC` reset** — CK with `RELOAD=1` auto-restores SRC/DST/CNT. No manual re-set needed in ISR.
3. **32-bit DMA** — CK only has 16-bit DMA. If using DMA, you get CCP4BUFL only. If using ISR capture, you read both L and H atomically.

## Bugs That WILL Apply on CK

1. **DMA module enable** (`DMAConbits.DMAEN = 1`) — still required
2. **DMAL/DMAH address limits** — still required (different register names)
3. **One-shot CHEN re-arm** — still required in DMA ISR
4. **Buffer index** — `completedBuf = dmaActiveBuf` (not `^1`)
5. **64-edge buffer** — still required for gap-based decode
6. **`volatile` on DMA buffers** — still required
7. **`_CCP4IF` clear early** — still required
8. **ICOV clear** — still required (via `CCP4STATL`)
9. **56.25% threshold** — unchanged (pure math)
10. **Detection thresholds** (50/100/200/500) — unchanged

---

## Testing

### Python Test Script

Port `tools/gsp_rx_test.py` → `tools/ck_rx_test.py`:
- Change serial port path for CK board
- Verify GSP command IDs match
- 12 test cases: PING, inject, PWM lock, DShot lock, timeout, auto-arm, sweep, re-detect

### Hardware Test Sequence

1. Build with `FEATURE_RX_PWM=1, DSHOT=0` first (PWM only, no DMA)
2. Test with RC receiver (1000-2000µs PWM)
3. Enable `FEATURE_RX_DSHOT=1`
4. Test with DShot150 source (widest timing margins)
5. Progress to DShot300, DShot600
6. Enable `FEATURE_RX_AUTO=1`, test auto-detect

### Build Configurations

```
1. RX=0                    (baseline, no RX code compiled)
2. RX_PWM=1, DSHOT=0       (PWM only, ISR capture)
3. RX_PWM=1, DSHOT=1, AUTO=1  (full support)
```

---

## Success Criteria

1. All 3 build configs compile with zero errors
2. All 12 injection tests pass (`ck_rx_test.py`)
3. PWM lock from RC receiver, throttle sweep
4. DShot150/300/600 lock from flight controller
5. Auto-detect classifies correctly on power-up
6. Signal loss → LOST → motor stop → re-detect on reconnect
7. Motor responds to throttle from DShot/PWM source
