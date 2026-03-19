# RX Auto-Detection V3 — Complete Diff Report

**Original**: `dspic33AKESC/` (main branch, injection-tested only)
**Fixed**: `esc_autodetection_v3/foc_v1/ProjectGaruda/dspic33AKESC/` (hardware-tested, working)
**Date**: 2026-03-19

---

## Scope of Changes

| File | Changed? | Impact |
|------|----------|--------|
| `hal/hal_input_capture.c` | **YES — major rewrite** | DMA init, ISR, decode algorithm |
| `hal/hal_input_capture.h` | NO (stale doc only) | Header signature not updated |
| `input/rx_decode.c` | **YES** | Detection thresholds, DShot1200, type change |
| `input/rx_decode.h` | **YES** | `rxCachedLocked` type uint8_t→bool |
| `garuda_config.h` | **YES** | `RX_DSHOT_EDGES` 32→64 |
| `garuda_types.h` | **YES** | `RX_PROTO_DSHOT1200` enum added |
| `garuda_service.c` | NO | ADC ISR throttle mux unchanged |
| `garuda_service.h` | NO | — |
| `main.c` | NO | Init sequence, service loop unchanged |
| `gsp/gsp_commands.c` | NO | RX_INJECT, GET_RX_STATUS unchanged |
| `hal/port_config.c` | NO | RD8 pin config unchanged |

**PWM capture logic is code-identical.** PWM issues on real hardware were caused by
upstream problems (DMA init, ISR flag clearing, detection) that corrupted the overall
RX state machine before PWM mode was ever reached or maintained.

---

## 1. DMA Controller Never Enabled

**File**: `hal/hal_input_capture.c` — `HAL_IC4_ConfigDmaDshot()`

**Original** (line 270):
```c
DMA0CHbits.CHEN = 0;           /* Disable channel during config */
DMA0CHbits.SIZE   = 0b10;     /* ... */
```

**Fixed** (line 283):
```c
DMACONbits.ON = 1;             /* ← ADDED: enable DMA module globally */
DMA0CHbits.CHEN = 0;
DMA0CHbits.SIZE = 0b10;
```

**Impact**: SHOWSTOPPER — Without `DMACONbits.ON = 1`, the DMA controller itself is
disabled at the module level. No channel can transfer any data. Zero DMA interrupts
fire, zero edge timestamps captured. DShot is completely dead.

**Why injection tests passed**: GSP `RX_INJECT` writes directly to the seqlock mailbox
via `gsp_commands.c`, bypassing DMA entirely.

---

## 2. DMA Done-Event Not Enabled

**File**: `hal/hal_input_capture.c` — `HAL_IC4_ConfigDmaDshot()`

**Original** (line 276-277):
```c
DMA0CHbits.TRMODE = 0b00;     /* One-shot */
DMA0CHbits.FLWCON = 0b00;     /* Read SRC, write DST */
```

**Fixed** (line 288-289):
```c
DMA0CHbits.TRMODE = 0b00;
DMA0CHbits.DONEEN = 1;        /* ← ADDED: enable done/transfer-complete event */
```

**Impact**: SHOWSTOPPER — `DONEEN` gates the DMA transfer-complete interrupt. Without
it, `_DMA0Interrupt` never fires even if `_DMA0IE = 1`. The DMA silently completes
transfers but never notifies the CPU.

**Debug breadcrumb**: The fixed code also has a commented-out alternative:
```c
// DMA0INTbits.DONEIE = 1;   // ? ADD THIS
```
This shows the developer tried both the channel-level (`DONEEN`) and interrupt-level
(`DONEIE`) enable bits while debugging which register actually gates the TC interrupt
on dsPIC33AK.

---

## 3. DMA Address Range Not Set

**File**: `hal/hal_input_capture.c` — `HAL_IC4_ConfigDmaDshot()`

**Original**: Not present.

**Fixed** (lines 292-293):
```c
DMALOW  = 0x0000UL;
DMAHIGH = 0x00FFFFFFUL;
```

**Impact**: HIGH — The dsPIC33AK DMA has address-range guard registers. If
`DMALOW`/`DMAHIGH` don't encompass the destination buffer's address, DMA writes are
silently blocked. POR defaults may be 0/0, which would block all writes.

---

## 4. DMA Not Re-Armed After Transfer

**File**: `hal/hal_input_capture.c` — `_DMA0Interrupt()`

**Original** (lines 419-421):
```c
dmaActiveBuf ^= 1;
DMA0DST = (uint32_t)&dmaEdgeBuf[dmaActiveBuf][0];
DMA0CNT = RX_DSHOT_DMA_COUNT;
/* (no CHEN re-enable) */
```

**Fixed** (lines 448-453):
```c
dmaActiveBuf ^= 1;
DMA0SRC = (uint32_t)&CCP4BUF;              /* ← ADDED: re-set source */
DMA0DST = (uint32_t)&dmaEdgeBuf[dmaActiveBuf][0];
DMA0CNT = RX_DSHOT_DMA_COUNT;
DMA0CHbits.CHEN = 1;                        /* ← ADDED: re-arm channel */
```

**Impact**: SHOWSTOPPER — In one-shot mode (`TRMODE=0b00`), the DMA channel auto-
disables after completing its transfer count. The original code reloads DST and CNT
but never re-enables `CHEN`, so only ONE DMA transfer ever occurs. After that, edge
timestamps accumulate in CCP4BUF but are never read.

The fixed version also re-sets `DMA0SRC` as a defensive measure — the datasheet is
ambiguous about whether SRC resets on channel disable.

**Debug breadcrumbs** (commented-out alternatives tried):
```c
//  DMA0CHbits.RELOADD=1;   /* auto-reload dest — didn't work or not available */
//  DMA0CHbits.RELOADC=1;   /* auto-reload count — same */
//  DMA0DST = (uint32_t)&U1TXB;  /* routed DMA to UART TX for debug output */
```

---

## 5. Wrong DMA Buffer Decoded

**File**: `hal/hal_input_capture.c` — `_DMA0Interrupt()`

**Original** (line 409):
```c
uint8_t completedBuf = dmaActiveBuf ^ 1;  /* just-completed buffer */
```

**Fixed** (line 439):
```c
uint8_t completedBuf = dmaActiveBuf;      /* this IS the completed one */
```

**Impact**: SHOWSTOPPER — When `_DMA0Interrupt` fires, DMA has just finished writing
to `dmaActiveBuf`. The original code XOR'd to get the "other" buffer, which was
either empty (first transfer) or contained the previous transfer's stale data.
Every decode attempt was reading the wrong buffer.

---

## 6. DShot Decode Algorithm — Complete Rewrite

**File**: `hal/hal_input_capture.c` — `DshotDecodeFrame()`

### Original: Offset-Based, Per-Bit Period (lines 120-170)

```c
bool DshotDecodeFrame(const volatile uint32_t *edges, uint8_t offset,
                       uint16_t *throttle, uint8_t *telemetry)
{
    uint16_t raw = 0;
    uint8_t baseIdx = offset * 2;

    for (uint8_t bit = 0; bit < 16; bit++)
    {
        uint8_t idx = (baseIdx + bit * 2) & 63;
        uint32_t rise = edges[idx];
        uint32_t fall = edges[(idx + 1) & 63];
        uint32_t highTime = fall - rise;

        uint32_t period = edges[(idx + 2) & 63] - rise;

        if (bit == 15) {  /* last bit: no next rise */
            raw <<= 1;
            if (period > 0 && highTime * 16 > period * 10)
                raw |= 1;
            continue;
        }
        if (period == 0) return false;

        raw <<= 1;
        if (highTime * 16 > period * 10)  /* 62.5% threshold */
            raw |= 1;
    }
    /* ... CRC check ... */
}
```

**Problems**:
- Assumes frame starts at `offset * 2` in the ring buffer — requires external
  alignment search to find the right offset
- Computes period per-bit from rise[N] to rise[N+2] — noisy, each bit gets a
  different threshold
- Last bit (bit 15) has no next rise available — special-cased with weaker check
- 62.5% threshold (`highTime * 16 > period * 10`) is not the DShot spec midpoint

### Fixed: Gap-Based Frame Finding, Single Threshold (lines 123-186)

```c
bool DshotDecodeFrame(const volatile uint32_t *edges, uint8_t edgeCount,
                      uint16_t *throttle, uint8_t *telemetry)
{
    /* Step 1: Find inter-frame gap (>20us = >2000 counts @ 100MHz) */
    uint8_t frameStart = 0;
    bool    gapFound   = false;

    for (uint8_t i = 0; i < edgeCount; i++)
    {
        uint8_t  fallIdx     = (i * 2 + 1) % edgeCount;
        uint8_t  nextRiseIdx = (fallIdx + 1) % edgeCount;
        uint32_t gap         = edges[nextRiseIdx] - edges[fallIdx];

        if (gap > 2000)  /* >20us inter-frame gap */
        {
            frameStart = nextRiseIdx;
            gapFound   = true;
            break;
        }
    }
    if (!gapFound) return false;

    /* Step 2: Compute bit period from first two rises — ONE threshold for all bits */
    uint32_t period = edges[(frameStart + 2) % edgeCount] - edges[frameStart];
    if (period == 0) return false;

    uint32_t threshold = (period * 9) / 16;  /* 56.25% — exact midpoint */

    /* Step 3: Decode all 16 bits with uniform threshold */
    uint16_t raw = 0;
    for (uint8_t bit = 0; bit < 16; bit++)
    {
        uint8_t riseIdx = (frameStart + bit * 2) % edgeCount;
        uint8_t fallIdx = (riseIdx + 1) % edgeCount;
        uint32_t highTime = edges[fallIdx] - edges[riseIdx];

        raw <<= 1;
        if (highTime >= threshold)
            raw |= 1;
    }
    /* ... CRC check ... */
}
```

**What changed**:

| Aspect | Original | Fixed |
|--------|----------|-------|
| Frame sync | External alignment offset (0-15) | Inter-frame gap detection (>20us) |
| 2nd parameter | `offset` (bit position) | `edgeCount` (buffer size) |
| Period calc | Per-bit (rise[N] to rise[N+2]) | Once from first two rises |
| Threshold | 62.5% per-bit (`h*16 > p*10`) | 56.25% uniform (`(p*9)/16`) |
| Bit 15 | Special-cased (no next rise) | Same as all other bits |
| Min edges needed | 32 (exactly 1 frame) | 64 (guarantees gap + full frame) |
| Noise resilience | Each bit has different threshold | Uniform — one noisy edge can't skew one bit |

**Why 56.25%**: DShot spec defines bit-0 as 37.5% duty and bit-1 as 75% duty. The
exact midpoint is (37.5 + 75) / 2 = 56.25%. The original's 62.5% was biased toward
detecting 1-bits, making 0-bits near the boundary more likely to be misread.

**Why gap-based**: Real DShot has a mandatory inter-frame gap (the "pause" between
frames). This is a reliable sync marker that works regardless of buffer alignment.
The original's offset-search approach required trying all 16 bit positions and
validating CRC at each — slow and fragile.

---

## 7. DMA Edge Count Doubled

**File**: `garuda_config.h` (line 495)

**Original**: `#define RX_DSHOT_EDGES 32`
**Fixed**: `#define RX_DSHOT_EDGES 64`

**Impact**: SHOWSTOPPER (for gap-based decode) — 32 edges = exactly one DShot frame
(16 bits x 2 edges). With perfect alignment you get the frame, but there's no room
for the inter-frame gap. With 64 edges you're guaranteed to capture at least one
complete frame PLUS the gap before/after it, so the gap-based decoder always works.

**Also affects**: `dmaEdgeBuf[2][64]` (hardcoded in fixed version with `volatile`
and `__attribute__((aligned(4)))`), and all `RX_DSHOT_DMA_COUNT` uses since it
equals `RX_DSHOT_EDGES`.

---

## 8. DMA Buffer Declaration Changes

**File**: `hal/hal_input_capture.c` (line 50/53)

**Original**:
```c
static uint32_t dmaEdgeBuf[2][RX_DSHOT_EDGES];
```

**Fixed**:
```c
static volatile uint32_t dmaEdgeBuf[2][64] __attribute__((aligned(4)));
```

**Three changes**:
1. **`volatile` added** — DMA writes asynchronously; without `volatile`, the compiler
   may optimize out reads of this buffer, serving stale cached values in the ISR.
2. **Hardcoded `64`** — Instead of using `RX_DSHOT_EDGES` macro (which is also 64).
   Likely done during debugging to eliminate any macro-expansion uncertainty.
3. **`__attribute__((aligned(4)))`** — Ensures 4-byte alignment for 32-bit DMA
   transfers. The dsPIC33AK DMA may fault or silently corrupt data on unaligned
   32-bit writes.

---

## 9. FLWCON Register Removed

**File**: `hal/hal_input_capture.c` — `HAL_IC4_ConfigDmaDshot()`

**Original** (line 277):
```c
DMA0CHbits.FLWCON = 0b00;     /* Read SRC, write DST */
```

**Fixed**: Removed entirely.

**Impact**: MEDIUM — `FLWCON` may not exist on this DMA variant or may have different
semantics. Writing to a non-existent bitfield is undefined. The fixed version relies
on POR default (which is 0b00 = read-source/write-dest anyway).

---

## 10. DMA ISR Priority Lowered

**File**: `hal/hal_input_capture.c` — `HAL_IC4_ConfigDmaDshot()`

**Original** (line 288): `_DMA0IP = 4;`
**Fixed** (line 306): `_DMA0IP = 1;`

**Impact**: MEDIUM — Priority 4 matched the IC4 ISR priority. At priority 1, the DMA
ISR runs at the lowest urgency, below IC4 (4), ADC (6), and commutation (7). This
prevents DMA processing from delaying time-critical motor control. The DMA TC ISR
just copies buffers and attempts decode — not time-critical.

---

## 11. IC Overflow Clear on Init

**File**: `hal/hal_input_capture.c` — `HAL_IC4_Init()`

**Original** (lines 195-199): No overflow handling.

**Fixed** (line 216):
```c
CCP4STATbits.ICOV = 0;
```

**Impact**: MEDIUM — If IC4 overflowed during a previous run (e.g., watchdog reset
without power cycle), the ICOV flag stays latched. This can cause CCP4BUF to contain
stale data or prevent new captures from registering until cleared.

---

## 12. IC Overflow Clear in DMA ISR

**File**: `hal/hal_input_capture.c` — `_DMA0Interrupt()`

**Original**: No overflow handling.

**Fixed** (lines 433-436):
```c
if (CCP4STATbits.ICOV)
{
      CCP4STATbits.ICOV = 0;  // clear overflow
}
```

Also in `_CCP4Interrupt()` (commented out — tried then abandoned):
```c
//     if (CCP4STATbits.ICOV)
//     {
//          CCP4STATbits.ICOV = 0;  // clear overflow
//     }
```

**Impact**: MEDIUM — IC overflow means the FIFO was full when a new edge arrived; that
edge is lost. Without clearing ICOV, subsequent captures may be blocked or corrupted.
Clearing it in the DMA ISR (where data is consumed) is the correct location.

---

## 13. IC ISR Flag Clear Position

**File**: `hal/hal_input_capture.c` — `_CCP4Interrupt()`

**Original** (line 370): `_CCP4IF = 0;` at **end** of ISR.

**Fixed** (line 340): `_CCP4IF = 0;` **immediately after** reading `CCP4BUF`.

**Impact**: MEDIUM — On dsPIC33AK, the interrupt flag must be cleared before the ISR
returns. If a new edge arrives while the ISR is processing (common at DShot600 rates
where edges are 1.67us apart), clearing IF late means:
- Original: New edge sets IF, but ISR clears it on exit → edge lost silently
- Fixed: IF cleared early, new edge re-sets IF during processing → CPU re-enters ISR

This matters most for PWM detection mode where edges are slow but also for the DShot
detection phase before DMA takes over.

---

## 14. Protocol Detection — DShot1200 + Threshold Shift + UINT32_MAX Guard

**File**: `input/rx_decode.c` — `DetectProtocol()`

### 14a. UINT32_MAX Guard

**Original**: No guard after minDelta loop.

**Fixed** (lines 79-80):
```c
if (minDelta == UINT32_MAX)
    return RX_PROTO_NONE;  /* only gaps found, no valid edges */
```

**Impact**: MEDIUM — Without this, if all edge deltas are skipped (e.g., all gaps),
`minDelta` stays at `UINT32_MAX` and falls through to `> 50000` → falsely detected
as PWM. This could happen on a noisy floating input.

### 14b. Threshold Boundaries Shifted

| Rate | Original | Fixed |
|------|----------|-------|
| DSHOT1200 | — | `< 50` counts |
| DSHOT600 | `< 200` | `< 100` |
| DSHOT300 | `< 400` | `< 200` |
| DSHOT150 | `< 500` | `< 500` (unchanged) |

**Impact**: MEDIUM — The original thresholds were too wide. A DShot300 signal with
minDelta ~150 counts would be misclassified as DShot600, causing the decoder to
use wrong timing expectations.

### 14c. DShot1200 Support

**Fixed** adds:
```c
if (minDelta < 50)
    return RX_PROTO_DSHOT1200;
```

Requires `RX_PROTO_DSHOT1200` added to `garuda_types.h` enum (line 497).

### 14d. Commented-Out Gap Filter

**Fixed** has commented-out code (lines 74-75):
```c
//         if (delta > 10000)   /* skip inter-frame gaps >100us */
//          continue;
```

This was tried to prevent inter-frame gaps from being counted as valid edge deltas.
It was abandoned — the `UINT32_MAX` guard handles the edge case instead.

---

## 15. `rxCachedLocked` Type: `uint8_t` → `bool`

**Files**: `input/rx_decode.h` (line 38), `input/rx_decode.c` (line 43)

**Original**:
```c
extern volatile uint8_t  rxCachedLocked;       /* rx_decode.h */
volatile uint8_t  rxCachedLocked = 0;           /* rx_decode.c */
```

**Fixed**:
```c
extern volatile bool  rxCachedLocked;           /* rx_decode.h */
volatile bool  rxCachedLocked = 0;              /* rx_decode.c */
```

**Impact**: LOW — `bool` is 1 byte on XC-DSC, same as `uint8_t`. The ADC ISR reads
this as `rxCachedLocked ? rxCachedThrottleAdc : 0` which is identical for both types.
Semantic improvement only.

---

## 16. `RX_PROTO_DSHOT1200` Enum Added

**File**: `garuda_types.h` (lines 494-497)

**Original**:
```c
    RX_PROTO_DSHOT300,
    RX_PROTO_DSHOT600
} RX_PROTOCOL_T;
```

**Fixed**:
```c
    RX_PROTO_DSHOT300,
    RX_PROTO_DSHOT600,
    RX_PROTO_DSHOT1200
} RX_PROTOCOL_T;
```

---

## 17. DShot Rate Telemetry — DShot1200 Reporting

**File**: `input/rx_decode.c` — `RX_Service()` (lines 225-227)

**Original**: Only reports rates 6 (600), 3 (300), 1 (150).

**Fixed** adds:
```c
if (proto == RX_PROTO_DSHOT1200)
    garudaData.rxDshotRate = 12;  /* 1200 */
```

---

## 18. DshotDecodeFrame Call Sites — Parameter Change

The function signature changed from `offset` to `edgeCount`. Both call sites updated:

**DMA ISR** (`hal_input_capture.c`):
- Original: `DshotDecodeFrame(dshotRingBuf, dshotAlignOffset, ...)`
- Fixed: `DshotDecodeFrame(dshotRingBuf, RX_DSHOT_EDGES, ...)`

**Alignment search** (`rx_decode.c`):
- Original: `DshotDecodeFrame(dshotRingBuf, offset, ...)`
- Fixed: `DshotDecodeFrame(dshotRingBuf, RX_DSHOT_EDGES, ...)`

**Note**: The header file `hal_input_capture.h` still documents the old parameter name
(`uint8_t offset`), but this is a doc-only mismatch — C doesn't use parameter names
for linkage.

---

## 19. Debug Override in DetectProtocol

**File**: `input/rx_decode.c` (line 198)

**Fixed** has a commented-out debug line:
```c
               // RX_PROTOCOL_T proto = RX_PROTO_PWM;
```

Used during debugging to force PWM detection, bypassing auto-detect. Left as
breadcrumb.

---

## Unchanged Components (Verified Identical)

These were diffed line-by-line and confirmed unchanged:

- **PWM ISR capture logic** (`_CCP4Interrupt` IC_MODE_PWM case) — rise/fall edge
  pairing, pin state read via `PORTDbits.RD8`, pulse width calculation, validation
  against `RX_PWM_MIN_US`/`RX_PWM_MAX_US`, MailboxWrite — all identical.

- **PWM throttle conversion** (`PwmUsToThrottleAdc`) — deadband, mapping, scaling —
  identical.

- **DShot throttle conversion** (`DshotToThrottleAdc`) — command range, mapping,
  1999 divisor — identical.

- **Seqlock mailbox** (`MailboxWrite`, `RX_MAILBOX_T`) — structure, write pattern —
  identical.

- **Lock state machine** (`RX_Service` LOCKING/LOCKED/LOST cases) — frame counting,
  timeout, fault latch, re-detection — identical.

- **Throttle caching** (`rxCachedThrottleAdc`, write ordering for ADC ISR) — identical.

- **ADC ISR throttle mux** (`garuda_service.c` line 494) —
  `garudaData.throttle = rxCachedLocked ? rxCachedThrottleAdc : 0;` — identical.

- **Auto-arm logic** (`main.c`) — RX lock + zero throttle → ESC_ARMED — identical.

- **GSP RX_INJECT** (`gsp_commands.c`) — inject path, DMA disable, mailbox write —
  identical.

- **GSP GET_RX_STATUS** (`gsp_commands.c`) — status response format — identical.

- **Pin configuration** (`port_config.c`) — RD8 input setup — identical.

- **Config constants** (`garuda_config.h`) — All PWM timing, lock count, timeout,
  deadband, alignment shifts — identical (only `RX_DSHOT_EDGES` changed).

---

## Root Cause Analysis

### Why DShot Failed on Real Hardware

Five showstopper bugs in the DMA path prevented any real DShot frames from being
processed:

1. **DMA module off** (`DMACONbits.ON` never set) — no transfers occur
2. **TC interrupt gated** (`DONEEN` not set) — ISR never fires
3. **One-shot not re-armed** (`CHEN` not re-set) — only first transfer works
4. **Wrong buffer read** (`^1` inverts buffer index) — decodes empty/stale data
5. **32 edges insufficient** — no room for inter-frame gap, can't find frame boundary

All five were masked by injection testing because `GSP_CMD_RX_INJECT` writes directly
to the seqlock mailbox, completely bypassing DMA.

### Why PWM Failed on Real Hardware

The PWM capture ISR code itself is unchanged and correct. PWM failures were caused by:

1. **Detection phase corruption** — `_CCP4IF` cleared late (bug #13) could drop edges
   during protocol detection, causing DetectProtocol() to misclassify or fail
2. **False PWM detection** — `UINT32_MAX` guard missing (bug #14a) could cause noise
   on a floating input to be classified as PWM when no signal was present
3. **State machine stuck after DShot attempt** — If DShot was detected first (e.g.,
   noise), the DMA bugs would prevent any frames from being decoded, timeout would
   fire, state goes to LOST → re-detect → potentially misclassify again in a loop
4. **IC overflow not cleared** — Stale `ICOV` flag from DShot attempt could corrupt
   subsequent PWM captures after falling back to detection mode

### Why Auto-Detect Failed

Auto-detect depends on:
1. Clean edge capture during detection phase → broken by #13 (IF clear late)
2. Correct protocol classification → broken by #14a (UINT32_MAX), #14b (thresholds)
3. Successful operation in detected mode → DShot broken by #1-5, PWM broken by #3-4
4. Clean fallback on timeout → broken by #11 (ICOV not cleared on re-init)

---

## Summary Table

| # | Change | File | Severity | Category |
|---|--------|------|----------|----------|
| 1 | `DMACONbits.ON = 1` added | hal_input_capture.c | **SHOWSTOPPER** | DMA won't start |
| 2 | `DMA0CHbits.DONEEN = 1` added | hal_input_capture.c | **SHOWSTOPPER** | ISR never fires |
| 3 | `DMA0CHbits.CHEN = 1` re-arm added | hal_input_capture.c | **SHOWSTOPPER** | One transfer only |
| 4 | `completedBuf = dmaActiveBuf` (was `^1`) | hal_input_capture.c | **SHOWSTOPPER** | Wrong buffer decoded |
| 5 | `RX_DSHOT_EDGES` 32→64 | garuda_config.h | **SHOWSTOPPER** | No room for frame gap |
| 6 | Gap-based `DshotDecodeFrame` rewrite | hal_input_capture.c | **SHOWSTOPPER** | Frame sync impossible |
| 7 | `DMALOW`/`DMAHIGH` address range set | hal_input_capture.c | **HIGH** | DMA writes may be blocked |
| 8 | `volatile` + aligned on `dmaEdgeBuf` | hal_input_capture.c | **HIGH** | Compiler optimization |
| 9 | `DMA0SRC` re-set in DMA ISR | hal_input_capture.c | **HIGH** | Source addr may reset |
| 10 | `FLWCON` register removed | hal_input_capture.c | **MEDIUM** | May not exist on silicon |
| 11 | `_DMA0IP` 4→1 | hal_input_capture.c | **MEDIUM** | Priority inversion risk |
| 12 | `CCP4STATbits.ICOV = 0` in Init | hal_input_capture.c | **MEDIUM** | Stale overflow |
| 13 | `CCP4STATbits.ICOV` clear in DMA ISR | hal_input_capture.c | **MEDIUM** | Overflow data loss |
| 14 | `_CCP4IF = 0` moved to start of ISR | hal_input_capture.c | **MEDIUM** | Edge drop at high rate |
| 15 | `UINT32_MAX` guard in DetectProtocol | rx_decode.c | **MEDIUM** | False PWM on noise |
| 16 | Detection thresholds shifted for accuracy | rx_decode.c | **MEDIUM** | Wrong rate classification |
| 17 | `RX_PROTO_DSHOT1200` enum added | garuda_types.h | **LOW** | Feature addition |
| 18 | DShot1200 rate telemetry reporting | rx_decode.c | **LOW** | Telemetry completeness |
| 19 | `rxCachedLocked` uint8_t→bool | rx_decode.h/c | **LOW** | Semantic clarity |
| 20 | Debug breadcrumbs (4 commented lines) | hal_input_capture.c | **INFO** | Debugging artifacts |
| 21 | Debug override (force PWM) | rx_decode.c | **INFO** | Debugging artifact |

**Total: 6 showstoppers, 3 high, 6 medium, 3 low, 2 info = 20 differences**
