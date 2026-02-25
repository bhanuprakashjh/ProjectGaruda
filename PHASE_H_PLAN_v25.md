# Phase H: PWM Capture + DShot Input (v25)

## Context

The ESC currently supports two throttle sources: ADC pot and GSP
(serial). Real drone usage requires RC PWM input (1000-2000µs from
flight controller) and DShot digital protocol (150/300/600 from FC).
Both arrive on the same pin: RD8/RP57 (already mapped to ICM1 in
port_config.c:150, but IC1 is not initialized).

**Key design principles**:
- Same-pin dual-protocol strategy (auto-detect from edge timing)
- Decode outside ISR (ISR captures edges, main loop decodes)
- Lock/timeout FSM with fail-safe zero throttle
- AUTO priority: DShot > PWM; all RX loss = zero + fault latch

---

## Review Findings (cumulative, 89 items across 25 review rounds)

### v1→v2
1. FCY = 100 MHz (SCCP bus), not 200 MHz (CPU)
2. DMA initially deferred → now mandatory for DShot (#12)
3. Throttle domain 0-4095 — `* 4095 / 2000` conversion
4. Milestone 0 compile-probe
5. CLC kept as Phase 2 optional
6. Stale frame guard in ADC ISR mux

### v2→v3
7. `RX_TIMER_HZ` independent of HWZC feature guard
8. SCCP4 symbols: `CCP4CON1`/`_CCT4Interrupt` (match existing)
9. Snapshot stays 68B; new `GET_RX_STATUS` (0x26) instead
10. Frame FIFO replaces edge ring → mailbox (#18)
11. ADC ISR reads cached volatiles, no function calls

### v3→v4
12. DMA mandatory for DShot (1.2M ISR/sec infeasible)
13. Counted-edge framing for DShot (gap invalid for DShot150)
14. AUTO fallback → zero only, no pot (#20, #23)
15. Shared `SCCP_CLOCK_HZ` constant
16. Write ordering for cached volatiles

### v4→v5
17. PWM framing via rise/fall pairing (no gap detection)
18. DShot mailbox replaces FIFO (no overflow)
19. Feature flags default to 0

### v5→v6
20. AUTO loss = zero + FAULT_RX_LOSS latch, no pot fallback (#23)
21. DShot CPU budget: mandatory GPIO measurement in Milestone C
22. Mailbox starvation guard: `rxDroppedFrames` counter
23. Floating ADC pot on flight boards → no pot fallback ever

### v6→v7
24. **DShot DMA alignment + resync** — DMACNT=32 only works if DMA
    starts on a frame boundary. **Fix**: 3-phase approach:
    (a) **Detect**: ISR captures edges, identifies inter-frame gap
        (rising edge after >20µs silence = frame start).
    (b) **Arm**: on detected frame start, disable ISR, arm DMA with
        DMACNT=32. DMA captures exactly one frame.
    (c) **Resync**: DMA-TC ISR decodes + CRC check. If N consecutive
        CRC failures (e.g., 5), disable DMA, re-enable ISR, return
        to detect phase. This handles mid-stream starts and glitches.

25. **AUTO = DShot/PWM only** — AUTO means "auto-detect between
    DShot and PWM". ADC pot is not an RX protocol. Remove all
    "DShot > PWM > ADC" language. AUTO detects which signal is
    present on the RX pin. If neither → LOST → zero + fault.
    ADC pot source (`THROTTLE_SRC_ADC`) is a separate, explicit
    choice for bench use — never part of AUTO fallback chain.

26. **FAULT_RX_LOSS propagation** — New fault enum entry needed in
    3 locations:
    - `garuda_types.h:478`: add `FAULT_RX_LOSS` after `FAULT_MORPH_TIMEOUT` (index 9)
    - `gui/src/protocol/types.ts:64`: append `'RX_LOSS'` to `FAULT_CODES` array
    - `tools/gsp_test.py:279`: add `9: "RX_LOSS"` to `FAULT_NAMES` dict

27. **RX_Service() ownership in main.c** — `RX_Service()` is a
    main-loop poller (like GSP_Service), NOT an ISR or Timer1
    service routine. It belongs in `main.c:97` while(1) loop,
    not in garuda_service.c. Plan text corrected to always say
    "main.c while(1)" for RX_Service() calls.

### v7→v8
28. **DShot frame-start gap threshold must be rate-adaptive** — fixed
    20µs is wrong. DShot spec uses 21 bit-times of silence for frame
    boundary: DShot600=35µs, DShot300=70µs, DShot150=140µs. **Fix**:
    initial detection phase uses conservative threshold (>100µs rising
    edge = probable frame start). After rate detection, switch to
    `21 * bitPeriod` threshold for resync. This replaces the fixed
    ">20µs" criterion from finding #24.

29. **Ping-pong double-buffer DMA for DShot** — single-buffer re-arm
    has a decode+re-arm window where incoming edges are lost. At
    DShot600 37kHz (~27µs/frame), this drops frames. **Fix**: use two
    DMA buffers in ping-pong mode. DMA-TC ISR swaps active buffer and
    re-arms immediately, then decodes the completed buffer. Zero edge
    loss between consecutive frames.

30. **Throttle source switching needs explicit gating for RX sources**
    — current `HandleSetThrottleSrc()` (gsp_commands.c:174) only gates
    GSP→IDLE. **Fix**: add IDLE-only gating for PWM/DSHOT/AUTO sources,
    reject sources whose `FEATURE_RX_*` is compiled out (return
    `GSP_ERR_OUT_OF_RANGE`), and validate that the selected source
    matches an enabled feature flag.

31. **rxMailbox.seqNum: uint8_t → uint16_t** — at DShot600 37kHz,
    uint8_t wraps every ~6.9ms making drop accounting unreliable.
    uint16_t wraps every ~1.77s — sufficient for `rxDroppedFrames`
    gap detection. Also use uint16_t for `lastSeqNum` in RX_Service().

32. **FAULT_RX_LOSS: verify no hidden max-fault-index assumptions** —
    Confirmed: `faultCode` is uint8_t on wire (gsp_commands.h:93),
    GUI uses `FAULT_CODES[idx] ?? 'UNKNOWN'` (StatusPanel.tsx:19).
    No hardcoded max constant. Just keep enum/array/dict in sync at
    all 3 locations (garuda_types.h:467, types.ts:64, gsp_test.py:276).

### v8→v9
33. **Rolling CRC-based DShot frame alignment** — silence-based gap
    detection (even rate-adaptive 21*bitPeriod) is fragile at high FC
    update rates where inter-frame idle can be very short. **Fix**:
    primary alignment = rolling CRC. Capture 32 edges continuously,
    decode as a frame, check CRC. If pass → aligned. If fail → shift
    window by 2 edges (one bit) and retry. Gap detection becomes a
    *hint* to speed up initial lock, not a hard requirement. This
    replaces silence-only alignment from #24/#28.

34. **DMA ping-pong must be hardware-verified, not just symbol-probed**
    — Milestone 0 tests symbol existence only. "Zero edge loss" with
    ping-pong is a claim that must be proven on-target. **Fix**: add
    Milestone C.1 hardware gate test: continuous DShot600 stream,
    edge counter in ISR vs frame counter in DMA-TC, run for 10s,
    verify zero lost edges. If dsPIC33AK DMA doesn't support truly
    atomic ping-pong swap, fall back to **circular DMA** with
    half-transfer + transfer-complete interrupts (proven pattern).

35. **Mailbox needs proper seqlock** — write-ordered value/valid/seqNum
    can still tear if consumer reads across a producer update. **Fix**:
    producer writes `seqNum++` (odd = writing), writes payload, writes
    `seqNum++` (even = complete). Consumer reads seq1, copies payload,
    reads seq2; accepts only if `seq1 == seq2 && (seq1 & 1) == 0`.
    With uint16_t seqNum and dsPIC33AK atomic 16-bit reads, this is
    tear-free. Implemented in rx_decode.c (consumer) and
    hal_input_capture.c / DMA-TC ISR (producer).

36. **Full throttle source transition matrix** — current handler
    (gsp_commands.c:174) is ADC/GSP only. GUI mirrors this with
    IDLE-only gate on GSP button (ControlPanel.tsx:70), no gate on
    ADC (line 68). **Fix**: define complete matrix:
    ```
    FROM \ TO  | ADC    | GSP      | PWM/DSHOT/AUTO
    -----------|--------|----------|----------------
    Any state  | ALLOW  | IDLE-only| IDLE-only
    ```
    - TO ADC: always allowed (safe fallback, zeroes GSP/RX state)
    - TO GSP/PWM/DSHOT/AUTO: IDLE-only (WRONG_STATE if running)
    - TO RX source when FEATURE_RX_* off: OUT_OF_RANGE
    - GUI: disable RX source buttons when `!isIdle || !connected`,
      hide buttons for features not reported in GET_INFO feature flags

### v9→v10
37. **C.0 gate test: ISR edge counter impossible when _CCT4IE=0** —
    DShot DMA mode disables the IC ISR, so "edge counter in ISR vs
    DMA-TC frame counter" is self-contradictory. **Fix**: use
    logic analyzer or scope edge-count trigger for ground truth.
    Alternatively, run C.0 in ISR-only mode first (count raw edges
    for 10s), then switch to DMA mode (count frames for 10s),
    compare: `edges == frames * 32`. No ISR needed during DMA phase.

38. **Rolling CRC needs overlap buffer, not disjoint 32-edge chunks**
    — DMACNT=32 delivers disjoint windows. "Shift by 2 edges" requires
    data from the previous transfer boundary. **Fix**: maintain a
    64-edge ring buffer. DMA-TC copies its 32 new edges into the ring.
    Alignment search runs across the full 64-edge window (32 possible
    bit offsets × 32-edge decode). Once aligned, subsequent transfers
    decode at the known offset directly — no search needed unless
    CRC fails N times.

39. **TO ADC must be IDLE-only (consistent with no-pot rationale)** —
    v9 said "TO ADC always allowed", but on flight boards RA11 floats
    → random throttle. This contradicts the FAULT_RX_LOSS safety
    design. **Fix**: ALL source switches are IDLE-only, no exceptions.
    This matches the existing GSP→IDLE gate (gsp_commands.c:181).
    Updated matrix:
    ```
    FROM \ TO  | ADC      | GSP      | PWM/DSHOT/AUTO
    -----------|----------|----------|----------------
    Any state  | IDLE-only| IDLE-only| IDLE-only
    ```
    Compile-time: `FEATURE_ADC_POT=1` (default on MCLV-48V-300W),
    `=0` on flight boards. When off, TO ADC returns OUT_OF_RANGE.

40. **DMA-TC ISR must cap per-interrupt work** — worst case: CRC fail
    + 16 shift attempts in ISR = unbounded ISR time that can starve
    commutation. **Fix**: DMA-TC ISR does ONE decode attempt at the
    current offset. If CRC fails, increment `pendingShift` counter
    and return. `RX_Service()` in main.c:97 while(1) loop performs
    the shift search (non-ISR context, no timing constraint). ISR
    is O(1) — decode + CRC + mailbox write OR just set a flag.

41. **Detection lock must require protocol-specific validity** — timing
    thresholds alone can false-trigger on noisy edges. **Fix**:
    - DShot lock: N consecutive CRC-valid decoded frames (not just
      "edges look fast"). N = RX_LOCK_COUNT (10).
    - PWM lock: N consecutive pulses with valid width (950-2050µs)
      AND valid rise-to-rise period (2-25ms). N = RX_LOCK_COUNT (10).
    - This tightens the existing "Require N valid frames" language
      with explicit per-protocol validity criteria.

### v10→v11
42. **FEATURE_ADC_POT=0 leaves boot source dangling** —
    `GARUDA_ServiceInit()` (garuda_service.c:76) never sets
    `throttleSource`; struct zero-init → `THROTTLE_SRC_ADC` (enum 0,
    garuda_types.h:462). With `FEATURE_ADC_POT=0`, ESC boots into a
    disabled source. **Fix**: `GARUDA_ServiceInit()` must explicitly
    set `throttleSource` based on compile-time config:
    - `FEATURE_ADC_POT=1`: `THROTTLE_SRC_ADC` (current behavior)
    - `FEATURE_ADC_POT=0 && FEATURE_RX_AUTO=1`: `THROTTLE_SRC_AUTO`
    - `FEATURE_ADC_POT=0 && FEATURE_RX_PWM=1`: `THROTTLE_SRC_PWM`
    - `FEATURE_ADC_POT=0 && FEATURE_RX_DSHOT=1`: `THROTTLE_SRC_DSHOT`
    - `FEATURE_GSP=1` (no RX): `THROTTLE_SRC_GSP`
    Static assert if no source is enabled. Zero throttle cache in init.

43. **GSP heartbeat timeout force-switches to ADC while running** —
    `main.c:248` does `garudaData.throttleSource = THROTTLE_SRC_ADC`
    during motor run. This violates "all transitions IDLE-only" and
    is unsafe with `FEATURE_ADC_POT=0`. **Fix**: heartbeat timeout
    should trigger `gspStopIntent = true` (already set at line 250)
    WITHOUT changing throttleSource. The stop path already zeros
    throttle and shuts down motor. Remove the source switch at
    line 248 entirely. If `FEATURE_ADC_POT=0`, the source stays
    as-is (GSP) and the motor stops via zero throttle + stop intent.

44. **C.0 gate test: logic analyzer is mandatory, not secondary** —
    two separate 10s runs (ISR then DMA) can mask loss due to source
    jitter/drift. **Fix**: logic analyzer edge count is the primary
    acceptance criterion. The two-phase firmware counter comparison
    is a supplementary cross-check only.

45. **RX_Service() alignment search needs per-call iteration cap** —
    shift search across 32 offsets in a single main-loop call can
    starve GSP_Service() and BoardService() (main.c:97). **Fix**:
    cap at `RX_ALIGN_MAX_SHIFTS_PER_CALL` (e.g., 4). If not aligned
    after 4 shifts, return and resume next call. Total alignment
    time: at most 8 main-loop iterations to search all 32 offsets.

### v11→v12
46. **Feature-flag bit allocation frozen** — current bits 0-18 used
    (gsp_commands.c:46-64). New allocations:
    ```
    bit 19: FEATURE_ADC_POT
    bit 20: FEATURE_RX_PWM
    bit 21: FEATURE_RX_DSHOT
    bit 22: FEATURE_RX_AUTO
    ```
    Mirror in 3 locations:
    - `gsp_commands.c:BuildFeatureFlags()` — add 4 lines after bit 18
    - `gui/src/protocol/types.ts:168` (FEATURE_NAMES) — add 4 entries
    - `tools/gsp_test.py:260` (FEATURE_NAMES) — append 4 strings
    - `gui/src/components/HelpPanel.tsx:302` — bump `length: 19` → `23`
    GUI ControlPanel uses these bits to show/hide source buttons.

47. **Static source-config guard — exact condition** — in
    `garuda_config.h`, after all feature defines:
    ```c
    #if !FEATURE_ADC_POT && !FEATURE_GSP && !FEATURE_RX_PWM \
        && !FEATURE_RX_DSHOT && !FEATURE_RX_AUTO
    #error "No throttle source enabled — at least one of FEATURE_ADC_POT, FEATURE_GSP, FEATURE_RX_PWM, FEATURE_RX_DSHOT, or FEATURE_RX_AUTO must be 1"
    #endif
    ```
    Runtime: `GARUDA_ServiceInit()` (garuda_service.c:76) sets
    `throttleSource` via compile-time `#if` chain (same priority
    order: ADC_POT > RX_AUTO > RX_PWM > RX_DSHOT > GSP).

48. **C.0 acceptance criteria — numeric thresholds**:
    - **Test signal**: DShot600, fixed throttle 1024 (mid-range),
      continuous stream from sig gen or FC
    - **Duration**: 10 seconds minimum
    - **Primary**: logic analyzer edge count over test window
    - **Pass**: `LA_edges == firmware_dma_frames × 32`, zero mismatch
    - **Fail**: any nonzero mismatch → investigate, do NOT proceed
    - **Environment**: no motor running, no other DMA active, dedicated
      test to isolate DMA behavior

### v12→v13
49. **`throttleSource` trapped inside `#if FEATURE_GSP`** — in
    `garuda_types.h:547`, `throttleSource` lives inside the
    `#if FEATURE_GSP` block. RX-only builds (`FEATURE_GSP=0`,
    `FEATURE_RX_PWM=1`) cannot compile: `throttleSource` doesn't
    exist. **Fix**: move `throttleSource` out of the `#if FEATURE_GSP`
    block into the unconditional section of `GARUDA_DATA_T`. It is a
    throttle-mux field, not GSP-specific. The GSP-specific fields
    (`gspThrottle`, `lastGspPacketTick`, intent flags) stay guarded.
    Also move `THROTTLE_SOURCE_T` enum definition outside any guard
    (it already is outside — just the struct field is trapped).

50. **C.0 window sync needed** — `LA_edges == dma_frames × 32` is
    only strict-valid if measurement start/stop is aligned to frame
    boundaries. Without sync, partial frames at window edges cause
    false mismatches. **Fix**: add GPIO sync markers. Firmware
    toggles a spare GPIO (e.g., LED2) HIGH at DMA-TC frame count
    reset and LOW when count reaches target. LA triggers on these
    edges to define the exact measurement window. Compare LA edge
    count within that GPIO-bounded interval only.

51. **Offset search cardinality: 16, not 32** — DShot = 16 bits =
    32 edges (rise+fall per bit). Each bit-shift = 2 edges. So the
    search domain is 16 possible bit offsets, not 32. Previous plan
    said "32 possible bit offsets" — incorrect. **Fix**: replace all
    "32 offsets" with "16 bit offsets" throughout. This halves search
    time: `RX_ALIGN_MAX_SHIFTS_PER_CALL=4` covers all 16 offsets in
    at most 4 main-loop iterations (not 8).

52. **GUI: explicit GSP-only fallback state** — with
    `FEATURE_ADC_POT=0` and all `FEATURE_RX_*=0`, only GSP remains
    as a throttle source. If ControlPanel hides all feature-gated
    buttons, the user sees no selectable source — confusing. **Fix**:
    GSP button is ALWAYS shown when connected (GSP is the transport
    layer — if you're connected, GSP exists). Other source buttons
    are shown/hidden via GET_INFO feature bits. If only GSP is
    available, ControlPanel shows "GSP (only source)" with a tooltip
    explaining why. No functional change — just UI clarity.

### v13→v14
53. **ADC ISR throttle mux trapped inside `#if FEATURE_GSP`** — the
    throttle-source switch at `garuda_service.c:174-182` is wrapped
    by `#if FEATURE_GSP`. The `#else` path (line 181) hardcodes
    `throttle = potRaw`, so a `FEATURE_GSP=0 + FEATURE_RX_PWM=1`
    build never routes `rxCachedThrottleAdc` into the motor. **Fix**:
    make the throttle-source switch unconditional. Replace the
    `#if FEATURE_GSP` / `#else` block with a single `switch` on
    `throttleSource` that has per-source `#if` guards inside:
    See finding 56 for final safe switch (ADC guarded, default=0).

54. **GSP-only fields stay guarded; init consistency** — after moving
    `throttleSource` out of `#if FEATURE_GSP` (finding 49), the
    GSP-specific fields (`gspThrottle`, `lastGspPacketTick`, intent
    flags) remain inside the guard. `GARUDA_ServiceInit()` must
    explicitly init `throttleSource = THROTTLE_SRC_ADC` (or feature-
    selected default) unconditionally — NOT inside a `#if FEATURE_GSP`
    block. The GSP field inits (`gspThrottle = 0` etc.) stay guarded.
    This is already what finding 42 specifies; this finding makes the
    split explicit: unconditional `throttleSource` init BEFORE guarded
    GSP-field init.

55. **CI/test consistency check for feature bits 19-22** — finding 46
    lists 3 mirror locations. Add a simple test to `tools/gsp_test.py`
    that reads the firmware GET_INFO `featureFlags` and checks that
    every set bit has a matching name in the local `FEATURE_NAMES`
    list (i.e., list length ≥ highest set bit + 1). This catches
    drift where firmware adds a bit but test/GUI doesn't. For
    firmware↔GUI consistency, the deploy-gui CI build will fail if
    `types.ts` FEATURE_NAMES doesn't cover all bits (TypeScript
    strict indexing). Additionally, add a comment block in
    `gsp_commands.c:BuildFeatureFlags()` listing the canonical bit
    allocation table so firmware devs see it at the source of truth.

### v14→v15
56. **ADC ISR `default:` unsafe with FEATURE_ADC_POT=0** — the v14
    switch has `case THROTTLE_SRC_ADC: default: throttle = potRaw`.
    On flight boards (FEATURE_ADC_POT=0), RA11 floats → random
    throttle from a corrupted/invalid enum value. **Fix**: split
    `default:` from `THROTTLE_SRC_ADC`. ADC case is guarded:
    ```c
    #if FEATURE_ADC_POT
        case THROTTLE_SRC_ADC:
            garudaData.throttle = garudaData.potRaw;
            break;
    #endif
        default:
            garudaData.throttle = 0;
            break;
    ```
    `default: throttle = 0` is the only safe fallback — it covers
    corrupted enum values AND misconfigured builds. Zero throttle
    never causes harm; a floating ADC can.

57. **Feature-bit test needs explicit name verification** — "highest
    set bit < len(FEATURE_NAMES)" catches missing entries but not
    wrong/swapped names. **Fix**: in `gsp_test.py`, add an explicit
    assertion for each new bit:
    ```python
    assert FEATURE_NAMES[19] == "ADC_POT"
    assert FEATURE_NAMES[20] == "RX_PWM"
    assert FEATURE_NAMES[21] == "RX_DSHOT"
    assert FEATURE_NAMES[22] == "RX_AUTO"
    ```
    This catches both missing entries AND misaligned names. The
    highest-bit-vs-length check remains as a general guard.

58. **C.0 GPIO sync + counter reset must be atomic** — if DMA-TC
    frame counter reset and GPIO-HIGH happen in separate instructions,
    a DMA-TC between them causes off-by-one. **Fix**: in
    `hal_input_capture.c`, the C.0 test uses a critical section
    (disable DMA channel 0 interrupt) around both operations:
    ```c
    /* Start measurement window — atomic */
    _DMA0IE = 0;          /* block DMA0 TC ISR */
    dmaFrameCount = 0;    /* reset counter */
    LED2 = 1;             /* GPIO sync HIGH */
    _DMA0IE = 1;          /* re-enable */
    ```
    Same pattern for stop. Superseded by finding 59 which adds full
    DMA restart (CHEN=0/1) to the critical section.

### v15→v16
59. **C.0 must restart DMA transfer, not just software counter** —
    resetting `dmaFrameCount` without restarting the DMA block means
    the first "frame" in the window may be partial (DMA mid-transfer
    when GPIO goes HIGH). LA sees full 32 edges for that frame, but
    DMA-TC fires partway through → count is off by one. **Fix**: C.0
    start sequence must fully restart DMA:
    ```c
    _DMA0IE = 0;           /* block DMA-TC ISR */
    DMA0CHbits.CHEN = 0;   /* stop DMA channel */
    DMA0CNT = 32;          /* reload count */
    dmaFrameCount = 0;     /* reset software counter */
    LED2 = 1;              /* GPIO sync HIGH */
    DMA0CHbits.CHEN = 1;   /* restart DMA — first frame aligned */
    _DMA0IE = 1;           /* re-enable ISR */
    ```
    Stop sequence:
    ```c
    _DMA0IE = 0;
    finalCount = dmaFrameCount;
    LED2 = 0;              /* GPIO sync LOW */
    DMA0CHbits.CHEN = 0;   /* stop DMA */
    _DMA0IE = 1;
    ```
    Now `finalCount` frames correspond to exactly `finalCount × 32`
    edges within the GPIO window. No partial-frame mismatch.

60. **FEATURE_RX_AUTO requires at least one RX protocol** — with
    `FEATURE_RX_AUTO=1` but both `FEATURE_RX_PWM=0` and
    `FEATURE_RX_DSHOT=0`, auto-detection has nothing to detect.
    Boot-source init (finding 42) would select AUTO, which can never
    lock → permanent FAULT_RX_LOSS on every boot. **Fix**: add
    compile-time guard in `garuda_config.h` after the static
    source guard:
    ```c
    #if FEATURE_RX_AUTO && !FEATURE_RX_PWM && !FEATURE_RX_DSHOT
    #error "FEATURE_RX_AUTO requires FEATURE_RX_PWM or FEATURE_RX_DSHOT"
    #endif
    ```

61. **DMA interrupt symbols are per-channel** — dsPIC33AK has 6 DMA
    channels with per-channel IE/IF: `_DMA0IE`/`_DMA0IF` (in IEC2),
    NOT `_DMAIE`. Control: `DMA0CH`/`DMA0CHbits.CHEN`/`.DONEEN`.
    Status: `DMA0STATbits.DONE`/`.DBUFWF` (double-buffer write flag).
    Source/dest/count: `DMA0SRC`, `DMA0DST`, `DMA0CNT`. Trigger
    select: `DMA0SEL`. ISR vector: `_DMA0Interrupt`. **Fix**: replace
    all `_DMAIE` references in plan with `_DMA0IE` (assuming DMA
    channel 0). Compile probe (Milestone 0) confirms these symbols.

62. **Test case for ADC ISR safe default** — `default: throttle = 0`
    is now safety-critical behavior. **Fix**: add a test in
    `gsp_test.py` that sends `SET_THROTTLE_SRC` with an invalid
    source value (e.g., 0xFF) and verifies the firmware returns
    `GSP_ERR_OUT_OF_RANGE` (existing validation in
    `HandleSetThrottleSrc()`). The ISR default path itself isn't
    directly testable from GSP (can't corrupt the enum remotely),
    but the command validator prevents invalid sources from ever
    reaching the switch. Add a comment in garuda_service.c noting
    the `default: 0` rationale: "Safety fallback — zero throttle
    for corrupted enum or disabled source. Never reads floating ADC."

### v16→v17
63. **C.0 must clear stale DMA state before restart** — `CHEN=0` +
    `DMA0CNT=32` + `CHEN=1` is not enough. A stale `_DMA0IF`,
    `DMA0STATbits.DONE`, or buffer-select state (`DBUFWF`) from the
    previous transfer can cause the ISR to fire immediately or the
    first transfer to target the wrong ping-pong buffer. **Fix**:
    ```c
    _DMA0IE = 0;
    DMA0CHbits.CHEN = 0;
    _DMA0IF = 0;              /* clear stale interrupt flag */
    /* Clear status — use W1C until Milestone 0 confirms method */
    DMA0STATbits.DONE = 1;
    DMA0STATbits.HALF = 1;
    DMA0STATbits.OVERRUN = 1;
    DMA0CNT = RX_DSHOT_DMA_COUNT;
    dmaFrameCount = 0;
    LED2 = 1;
    DMA0CHbits.CHEN = 1;
    _DMA0IE = 1;
    ```

64. **DMA0CNT semantics: define `RX_DSHOT_DMA_COUNT` macro** —
    dsPIC33AK DMA transfers `DMA0CNT` items (count=N means N
    transfers, NOT N-1 like some DMA peripherals). Must be confirmed
    in Milestone 0 compile probe by observing actual transfer count.
    **Fix**: define macro in `garuda_config.h`:
    ```c
    /* DMA0CNT = number of DMA transfers per frame.
     * dsPIC33AK: count=N means N transfers. Verify in Milestone 0.
     * If device uses count-1 semantics, change to (RX_DSHOT_EDGES - 1). */
    #define RX_DSHOT_DMA_COUNT  RX_DSHOT_EDGES  /* 32 */
    ```
    All C.0 validation math uses this macro. If probe reveals
    count-1 semantics, only one define changes.

65. **C.0 sync GPIO: use dedicated test pin, not LED2** — LED2 is
    toggled by fault handlers (garuda_service.c) and potentially
    main-loop diagnostics. In C.0 test mode, LA markers would be
    polluted. **Fix**: use `DIAGNOSTIC_MANUAL_STEP` pattern — a
    `C0_DMA_TEST` compile flag in `garuda_config.h` (default 0).
    When enabled: (a) disables all other LED2 writes, (b) LED2
    becomes exclusive C.0 sync GPIO, (c) main loop runs C.0 test
    sequence instead of normal operation. When disabled: zero code
    impact. Alternatively, use a spare GPIO (e.g., test point on
    board) if available — resolve during Milestone 0 board survey.

### v17→v18
66. **Split wire-truth from DMA register load** — `RX_DSHOT_DMA_COUNT`
    conflates the physical edge count (always 32) with the DMA
    register value (32 or 31 depending on count-1 semantics). If
    DMA0CNT is count-1, the acceptance math `frames × DMA_COUNT`
    becomes `frames × 31` — wrong. **Fix**: two constants:
    ```c
    #define RX_DSHOT_EDGES      32  /* wire truth: 16 bits × 2 edges */
    #define RX_DSHOT_DMA_COUNT  RX_DSHOT_EDGES  /* DMA register load;
        change to (RX_DSHOT_EDGES - 1) if count-1 semantics */
    ```
    C.0 acceptance: `LA_edges == dmaFrames × RX_DSHOT_EDGES` (always
    wire truth). DMA register: `DMA0CNT = RX_DSHOT_DMA_COUNT`. These
    are intentionally separate — the validation formula never changes.

67. **C.0 start must flush CCP4 capture pipeline** — pending captures
    in `CCP4BUF` from before the window can be the first DMA transfer,
    skewing the edge count. **Fix**: after `CHEN=0` and before window
    start, flush IC state:
    ```c
    CCP4CON1bits.ON = 0;       /* stop IC */
    (void)CCP4BUF;             /* drain pending capture */
    _CCT4IF = 0;               /* clear IC interrupt flag */
    CCP4CON1bits.ON = 1;       /* restart IC — clean pipeline */
    ```
    Full C.0 start sequence (in order):
    1. `_DMA0IE = 0;`
    2. `DMA0CHbits.CHEN = 0;`
    3. Flush IC: `ON=0; drain CCP4BUF; _CCT4IF=0; ON=1;`
    4. Clear DMA: `_DMA0IF=0; DMA0STAT clear; DMA0CNT=RX_DSHOT_DMA_COUNT;`
    5. `dmaFrameCount = 0; GPIO = 1;`
    6. `DMA0CHbits.CHEN = 1; _DMA0IE = 1;`

68. **C.0 stop must be frame-aligned** — stopping at an arbitrary
    instruction boundary can include partial-frame edges in the LA
    window. The DMA-TC ISR fires at exact frame boundaries. **Fix**:
    DMA-TC ISR checks `dmaFrameCount >= C0_TARGET_FRAMES`. When
    reached, ISR itself latches `c0FinalCount = dmaFrameCount`,
    sets `GPIO = 0`, sets `DMA0CHbits.CHEN = 0`, sets `c0Done = 1`.
    Main loop polls `c0Done`. This guarantees the GPIO-LOW edge
    occurs exactly at a DMA transfer-complete boundary — no partial
    frames in the LA window. The ISR stop is O(1) (one compare +
    three writes) and only happens once.

69. **DMA0STAT write semantics: verify in Milestone 0** — many
    Microchip status registers use "write-1-to-clear" for flag bits
    (writing 0 has no effect). If DMA0STAT is W1C, `DMA0STAT = 0`
    does NOT clear flags — must write `DMA0STAT = 0x3F` (or clear
    each bit individually). **Fix**: Milestone 0 compile probe adds
    a DMA0STAT clear test: write 0, read back; if flags persist,
    use W1C pattern. Document result. Until confirmed, plan code
    uses explicit bit clears as the safe default:
    ```c
    DMA0STATbits.DONE = 1;     /* W1C: write 1 to clear */
    DMA0STATbits.HALF = 1;
    DMA0STATbits.OVERRUN = 1;
    ```

### v18→v19
70. **C.0 ISR stop block must latch `c0FinalCount` before GPIO-LOW**
    — v18 says ISR sets `GPIO=0; CHEN=0; c0Done=1` but main loop
    reads `c0FinalCount` that was never written. **Fix**: exact ISR
    stop sequence:
    ```c
    if (dmaFrameCount >= C0_TARGET_FRAMES) {
        c0FinalCount = dmaFrameCount;  /* latch FIRST */
        LED2 = 0;                       /* GPIO-LOW = LA window end */
        DMA0CHbits.CHEN = 0;
        _DMA0IE = 0;                   /* self-disable */
        c0Done = 1;
    }
    ```
    `c0FinalCount` is latched before `GPIO=0` so LA window end and
    frame count are exactly synchronized.

71. **IC flush must drain fully + clear overflow** — a single
    `(void)CCP4BUF` may not empty the FIFO if multiple captures
    are pending. **Fix**: drain in a loop + clear overflow:
    ```c
    CCP4CON1bits.ON = 0;
    while (CCP4STATbits.ICBNE) { (void)CCP4BUF; }  /* drain FIFO */
    CCP4STATbits.ICOV = 0;   /* clear overflow if W1C, else =1 */
    _CCT4IF = 0;
    CCP4CON1bits.ON = 1;
    ```
    `ICBNE` (IC Buffer Not Empty) is the standard Microchip SCCP
    status bit for pending captures. Verify symbol in Milestone 0.

72. **C.0 shared vars: volatile uint32_t** — DShot600 at ~37kHz for
    10s = ~370K frames. uint16_t overflows at 65535 (~1.8s). **Fix**:
    ```c
    static volatile uint32_t dmaFrameCount;   /* ISR writes, main reads */
    static volatile uint32_t c0FinalCount;    /* ISR writes, main reads */
    static volatile uint8_t  c0Done;          /* ISR writes, main polls */
    ```
    uint32_t handles 4B+ frames (~32 hours at DShot600). `volatile`
    required because ISR writes, main loop reads. `c0Done` is a flag
    so uint8_t suffices. Main loop ONLY reads `c0FinalCount` after
    `c0Done == 1` — the ISR-to-main handoff is the correctness
    mechanism, not atomicity assumptions.

### v19→v20
73. **C.0 ISR latency can skew GPIO-LOW vs DMA boundary** — GPIO=0
    fires when ISR executes, not at the hardware transfer-complete
    edge. If priority-7 ISRs (ADC, HWZC) delay DMA-TC by even a
    few µs, next-frame edges can enter the LA window before GPIO-LOW.
    **Fix**: in C.0 test mode, raise DMA0 ISR to priority 7 (highest)
    AND suppress competing ISRs. Since C.0 runs without motor (no
    commutation, no ADC control loop needed):
    ```c
    #if C0_DMA_TEST
    _DMA0IP = 7;   /* max priority for C.0 */
    _AD1CH0IE = 0; /* suppress ADC ISR */
    _CCT1IE = 0;   /* suppress HWZC timer ISR */
    #endif
    ```
    Document measured worst-case ISR latency (scope: DMA-TC trigger
    to GPIO toggle). At DShot600, inter-frame idle is ~5-21µs.
    If measured latency < idle time, strict `==` is safe. If not,
    add tolerance: `abs(LA_edges - frames × EDGES) <= 32` (one
    frame). This tolerance is only for the C.0 gate test, not for
    production DShot decode which is inherently frame-aligned.

74. **ICOV clear semantics: verify in Milestone 0** — plan assumes
    `CCP4STATbits.ICOV = 0` clears overflow, but it may be W1C
    (write-1-to-clear) like many Microchip status bits. **Fix**:
    add to Milestone 0 probe verification list alongside DMA0STAT:
    ```
    Probe item: CCP4STAT ICOV clear method
    Test: force overflow (rapid captures), try ICOV=0 then read back.
          If persists, try ICOV=1 (W1C). Document result.
    ```
    IC flush code uses conditional:
    ```c
    /* Use whichever method Milestone 0 confirms */
    CCP4STATbits.ICOV = 1;  /* W1C default until proven otherwise */
    ```

75. **Don't assume 32-bit atomicity for ISR-shared data** — even
    on a 32-bit MCU, compiler-generated load/store sequences can
    be non-atomic for 32-bit values. The `c0Done` latch protocol
    is the correctness mechanism: main loop reads `c0FinalCount`
    ONLY after `c0Done == 1`, and ISR writes `c0FinalCount` before
    `c0Done = 1`. This is a happens-before relationship enforced by
    `volatile` ordering. **Rule**: no main-loop reads of
    `dmaFrameCount` while ISR is active. If future code needs
    live frame count reads, use interrupt masking:
    ```c
    _DMA0IE = 0;
    uint32_t snap = dmaFrameCount;
    _DMA0IE = 1;
    ```

### v20→v21
76. **C.0 start not frame-aligned — GPIO=1 before CHEN=1** — the v20
    start sequence sets `GPIO=1` then `CHEN=1`. At DShot600 (~1.67µs
    per bit), edges arriving in the gap enter the LA window but not
    DMA frame count. Strict `==` fails even with perfect DMA. **Fix**:
    mirror the stop pattern — use DMA-TC boundary for start too:
    1. Arm DMA silently: flush IC, clear DMA state, `CHEN=1; _DMA0IE=1;`
       (GPIO still LOW — LA window not yet open)
    2. DMA-TC ISR: on first callback (`dmaFrameCount == 1`), set
       `GPIO = 1` and reset `dmaFrameCount = 0` (this frame is the
       alignment frame — not counted). All subsequent frames are
       within the GPIO window.
    3. Stop: unchanged (ISR latches count, GPIO=0 at boundary).
    Now both window edges are at DMA-TC boundaries. LA window
    contains exactly `c0FinalCount × RX_DSHOT_EDGES` edges.

77. **C.0 ISR suppression must be comprehensive** — v20 disables
    `_AD1CH0IE` and `_CCT1IE` only, but AD1CMP5, AD2CMP1, PWMFault,
    Timer1, and other sources can still preempt or delay DMA-TC.
    **Fix**: in C.0 test mode, globally suppress all non-DMA
    interrupts by saving and clearing IEC registers:
    ```c
    #if C0_DMA_TEST
    uint32_t saved_iec0 = IEC0; IEC0 = 0;
    uint32_t saved_iec1 = IEC1; IEC1 = 0;
    uint32_t saved_iec2 = IEC2; IEC2 = 0;
    uint32_t saved_iec3 = IEC3; IEC3 = 0;
    /* Re-enable only DMA0 */
    _DMA0IE = 1;
    _DMA0IP = 7;
    #endif
    ```
    Restore on C.0 completion. This is the only way to guarantee
    zero ISR contention. Safe because C.0 runs without motor.

78. **IC FIFO drain loop needs bounded escape** — if `ICBNE` behaves
    unexpectedly with `ON=0` (silicon errata, flag stuck), the
    `while(ICBNE)` loop hangs. SCCP IC FIFO depth is 4 on dsPIC33AK.
    **Fix**: bound the loop:
    ```c
    CCP4CON1bits.ON = 0;
    for (int i = 0; i < 8 && CCP4STATbits.ICBNE; i++) {
        (void)CCP4BUF;
    }
    if (CCP4STATbits.ICBNE) {
        /* FIFO drain failed — log error, abort C.0 */
        c0Error = 1;
        return;
    }
    ```
    Max 8 iterations (2× FIFO depth) is safe. Failure → abort C.0
    with diagnostic, do NOT proceed to DMA test.

### v21→v22
79. **C.0 can hang if no DShot signal present** — after CHEN=1 with
    all IEC cleared, if no edges arrive (cable disconnected, sig gen
    off), `c0Started` never sets, main loop polls forever. **Fix**:
    main-loop C.0 controller has a startup timeout:
    ```c
    uint32_t c0StartTick = CCP4TMR;  /* 100 MHz free-run counter */
    /* ... arm DMA ... */
    while (!c0Started && !c0Error) {
        if ((CCP4TMR - c0StartTick) >= C0_STARTUP_TIMEOUT_COUNTS) {
            c0Error = C0_ERR_NO_SIGNAL;
            c0Cleanup();
            break;
        }
    }
    ```
    `C0_STARTUP_TIMEOUT_COUNTS` derived from 2000 ms × 100 MHz
    (see finding 85). Same pattern for measurement phase with
    `C0_MEAS_TIMEOUT_COUNTS`. `CCP4TMR` (32-bit, 100 MHz) wraps
    every ~42.9s — both timeouts fit within a single wrap. All exit
    paths call `c0Cleanup()` (finding 86) for IEC/IFS/DMA0IP restore.

80. **IEC save/clear/restore needs full coverage + IF hygiene** —
    dsPIC33AK128MC106 has IEC0-IEC3 (4 banks). All must be saved,
    cleared, and restored. Before restore, clear all pending IFS
    flags to prevent an ISR burst from interrupts that fired while
    suppressed. **Superseded by finding 87**: all IFS cleanup now goes
    through `c0ClearIfs()` inside `c0Cleanup()` (findings 86, 88).
    The macro's implementation (W1C vs direct) is determined by
    Milestone 0 and codified as `IFS_IS_W1C` in `garuda_config.h`.
    Verify IFS write semantics in Milestone 0 alongside DMA0STAT.

81. **C.0 acceptance must be strict — no ±1 tolerance** — v20 allowed
    ±1 frame tolerance if ISR latency exceeded inter-frame idle. But
    a one-frame loss is exactly the failure mode C.0 is designed to
    catch. **Fix**: remove all tolerance language. C.0 acceptance is
    always `LA_edges == c0FinalCount × RX_DSHOT_EDGES`, strict `==`.
    Any mismatch = FAIL, investigate, do NOT proceed. Measured ISR
    latency is diagnostic evidence only (helps explain failures),
    never a pass criterion relaxation.

### v22→v23
82. **C.0 must save/restore `_DMA0IP`** — setting `_DMA0IP=7` in C.0
    without restoring means DMA0 permanently runs at max priority if
    C.0 build continues to normal operation. **Fix**: save original
    priority before C.0 and restore in cleanup:
    ```c
    uint8_t saved_dma0ip = _DMA0IP;
    _DMA0IP = 7;
    /* ... C.0 test ... */
    /* Cleanup (all paths: success, timeout, error) */
    _DMA0IP = saved_dma0ip;
    ```

83. **IEC/IFS save/restore must use exact saved values, not blanket
    writes** — `IECx = 0` is safe (clearing all enables), but
    `IFSx = 0xFFFFFFFF` can interact with reserved bits. **Fix**:
    for IEC save/clear/restore, the existing save+clear+restore
    pattern is correct. For IFS cleanup before restore, clear only
    the bits that were enabled (i.e., the saved IEC bits correspond
    to potentially-pending flags):
    ```c
    /* Clear only flags whose interrupts were enabled */
    IFS0 &= ~saved_iec0;
    IFS1 &= ~saved_iec1;
    IFS2 &= ~saved_iec2;
    IFS3 &= ~saved_iec3;
    /* Restore enables */
    IEC0 = saved_iec0; IEC1 = saved_iec1;
    IEC2 = saved_iec2; IEC3 = saved_iec3;
    ```
    This avoids touching reserved bits entirely. If IFS bits are W1C,
    the `&= ~` pattern writes 0 to the flag bits (no-op for W1C) —
    wrong. **Superseded by finding 87**: both strategies unified into
    `c0ClearIfs()` function (finding 88), selected by `IFS_IS_W1C` after M0.
    Document which pattern the device requires.

84. **C.0 timeout: distinct error codes for startup vs measurement**
    — v22 uses `c0Error=2` for startup timeout but doesn't define
    a separate code for measurement-phase timeout. **Fix**:
    ```c
    #define C0_ERR_NONE         0
    #define C0_ERR_IC_DRAIN     1  /* ICBNE stuck */
    #define C0_ERR_NO_SIGNAL    2  /* no DMA-TC within startup timeout */
    #define C0_ERR_MEAS_STALL   3  /* c0Done not set within meas timeout */
    ```
    Main-loop C.0 controller has two explicit timeout loops:
    1. Startup: `while (!c0Started && !c0Error)` with
       `C0_STARTUP_TIMEOUT_COUNTS` (finding 85)
       → `c0Error = C0_ERR_NO_SIGNAL`
    2. Measurement: `while (!c0Done && !c0Error)` with
       `C0_MEAS_TIMEOUT_COUNTS` (finding 85)
       → `c0Error = C0_ERR_MEAS_STALL`
    Both paths call `c0Cleanup()` (finding 86) for full restore.
    Error code reported via LED blink pattern or debug print.

### v23→v24
85. **C.0 timeout: CCP4TMR counts ≠ milliseconds** — v23 uses
    `CCP4TMR` (100 MHz free-run counter) as timeout source but
    compares raw delta to `C0_STARTUP_TIMEOUT_MS = 2000`. That
    gives ~20 µs, not 2 s. **Fix**: define count-based thresholds
    using 64-bit intermediate to avoid overflow:
    ```c
    /* In garuda_config.h */
    #define C0_STARTUP_TIMEOUT_MS   2000
    #define C0_STARTUP_TIMEOUT_COUNTS \
        ((uint32_t)((uint64_t)C0_STARTUP_TIMEOUT_MS * SCCP_CLOCK_HZ / 1000ULL))
    /* 2000 ms × 100 MHz / 1000 = 200,000,000 counts — fits uint32_t */

    /* Measurement timeout: derived from target frames */
    #define C0_TARGET_FRAMES        370000  /* ~10s at DShot600 37kHz */
    #define C0_MEAS_TIMEOUT_MS      15000   /* generous: 10s + 5s margin */
    #define C0_MEAS_TIMEOUT_COUNTS \
        ((uint32_t)((uint64_t)C0_MEAS_TIMEOUT_MS * SCCP_CLOCK_HZ / 1000ULL))
    /* 15000 ms × 100 MHz / 1000 = 1,500,000,000 — fits uint32_t (max ~42.9s) */
    ```
    Timeout loops use wrap-safe delta: `(CCP4TMR - c0StartTick)`.
    Since CCP4TMR is 32-bit @ 100 MHz, wraps every ~42.9s — both
    timeouts (2s, 15s) fit within a single wrap cycle. Code:
    ```c
    uint32_t c0StartTick = CCP4TMR;
    while (!c0Started && !c0Error) {
        if ((CCP4TMR - c0StartTick) >= C0_STARTUP_TIMEOUT_COUNTS) {
            c0Error = C0_ERR_NO_SIGNAL;
            break;
        }
    }
    ```
    Same pattern for measurement phase with `C0_MEAS_TIMEOUT_COUNTS`.
    Replace all `systemTickRaw` references with `CCP4TMR` and all
    `_MS` comparisons with `_COUNTS` comparisons. The `_MS` defines
    remain as human-readable source-of-truth; `_COUNTS` are derived.

86. **C.0 `_DMA0IP` restore missing from all exit paths** — finding
    82 defines save/restore but the consolidated Milestone C.0
    description (line 1260) only mentions restoring IEC0-IEC3. The
    actual exit paths (success in DMA-TC ISR, no-signal timeout,
    measurement timeout, IC drain error) must ALL restore `_DMA0IP`.
    **Fix**: define a `c0Cleanup()` helper that is called from every
    exit path:
    ```c
    static void c0Cleanup(void) {
        LED2 = 0;                    /* GPIO sync LOW */
        DMA0CHbits.CHEN = 0;         /* stop DMA */
        _DMA0IE = 0;                 /* disable DMA ISR */
        /* IFS hygiene — method determined by Milestone 0 (finding 88) */
        c0ClearIfs(saved_iec0, saved_iec1, saved_iec2, saved_iec3);
        /* Restore interrupt enables */
        IEC0 = saved_iec0; IEC1 = saved_iec1;
        IEC2 = saved_iec2; IEC3 = saved_iec3;
        /* Restore DMA priority */
        _DMA0IP = saved_dma0ip;
    }
    ```
    Called from: (a) main-loop success path after `c0Done==1`,
    (b) startup timeout, (c) measurement timeout, (d) IC drain
    error. Note: for success, DMA-TC ISR already sets `CHEN=0;
    _DMA0IE=0; GPIO=0;` — `c0Cleanup()` re-executes these (safe,
    idempotent) to keep a single cleanup path. `saved_iec0-3` and
    `saved_dma0ip` are file-scope statics, not locals (ISR needs
    them in the success path if we later refactor ISR-side cleanup).

87. **IFS cleanup: single strategy with M0-selected helper** —
    findings 80 and 83 give conflicting strategies (`=0xFFFFFFFF`
    W1C vs masked `&= ~saved_iec`). This will cause implementation
    drift. **Fix**: define a single `C0_CLEAR_IFS()` macro that
    Milestone 0 resolves to exactly one implementation:
    **Superseded by finding 88**: clear logic moved to `c0ClearIfs()`
    static inline function in `hal_input_capture.c` (not a macro in
    `garuda_config.h`). Takes masks as arguments — no coupling to
    file-scope locals. `garuda_config.h` only defines `IFS_IS_W1C`
    flag (finding 89: undefined until M0, `#error` if C0_DMA_TEST
    builds without it).
    All C.0 exit paths call `c0Cleanup()` which calls `c0ClearIfs()`
    — single point of truth, no conflicting inline code. Finding 80's `IFS0 = 0` / `= 0xFFFFFFFF`
    and finding 83's `IFS0 &= ~saved_iec0` / `IFS0 = saved_iec0` are
    both superseded by this macro — use the M0-determined variant only.

### v24→v25
88. **`C0_CLEAR_IFS()` macro coupled to file-scope locals** — v24
    puts `C0_CLEAR_IFS()` in `garuda_config.h` but it references
    `saved_iec0..3`, which are runtime symbols in
    `hal_input_capture.c`. If the macro is accidentally expanded
    elsewhere, it won't compile. **Fix**: move `C0_CLEAR_IFS()` out
    of `garuda_config.h` and into `hal_input_capture.c` as a
    `static inline` function that takes the masks as arguments:
    ```c
    /* In hal_input_capture.c (file-scope, C.0 test only) */
    #if C0_DMA_TEST
    static inline void c0ClearIfs(uint32_t m0, uint32_t m1,
                                   uint32_t m2, uint32_t m3) {
    #if IFS_IS_W1C
        IFS0 = m0; IFS1 = m1; IFS2 = m2; IFS3 = m3;
    #else
        IFS0 &= ~m0; IFS1 &= ~m1; IFS2 &= ~m2; IFS3 &= ~m3;
    #endif
    }
    #endif
    ```
    `c0Cleanup()` calls `c0ClearIfs(saved_iec0, saved_iec1,
    saved_iec2, saved_iec3)`. `garuda_config.h` retains only the
    `IFS_IS_W1C` flag (board-level config), not the clear logic.
    This keeps the config header pure (constants + feature flags)
    and the clear logic local to its only consumer.

89. **`IFS_IS_W1C` must not have a default value pre-M0** — v24
    hardcodes `#define IFS_IS_W1C 1` as a guess. If Milestone 0
    reveals direct-write semantics but the developer forgets to
    update, the wrong clear behavior is silently baked in. **Fix**:
    before Milestone 0, leave `IFS_IS_W1C` undefined. After M0,
    define it with the verified value. Guard with `#ifndef`:
    ```c
    /* In garuda_config.h — MUST be set after Milestone 0 probe */
    /* #define IFS_IS_W1C  1 */  /* Uncomment after M0 verification */
    #ifndef IFS_IS_W1C
    #error "IFS_IS_W1C not defined — run Milestone 0 probe first"
    #endif
    ```
    This forces a compile error if C.0 test code is built before
    the probe result is recorded. Normal builds (`C0_DMA_TEST=0`)
    never expand `c0ClearIfs()` so the `#error` is only triggered
    when C.0 is actually attempted — exactly the right gate.
    Wrap the guard inside `#if C0_DMA_TEST` so non-test builds
    are unaffected:
    ```c
    #if C0_DMA_TEST
    #ifndef IFS_IS_W1C
    #error "IFS_IS_W1C not defined — run Milestone 0 probe first"
    #endif
    #endif
    ```

---

## Hardware Resources

| Resource | Current Use | Plan |
|----------|-------------|------|
| RD8/RP57 | Mapped to ICM1, unused | RX input pin → ICM4R |
| SCCP4 | Available | IC4: edge capture, 32-bit timer |
| DMA ch | Available | **Required** for DShot (DMACNT=32) |
| CLC1 | Available | Phase 2: deglitch sync filter |
| SCCP1/2/3 | HWZC + ADC trigger | **Untouched** |

**SCCP4 IC mode**: `CCP4CON1bits.CCSEL=1, T32=1, MOD=0b0011`
= Every Rise/Fall, 32-bit capture.

**Timer clock**: `CLKSEL=0b000` = SCCP peripheral bus = **100 MHz**.
- 32-bit timer wraps every ~42.9 seconds
- 1000µs = 100,000 counts, 2000µs = 200,000 counts
- DShot600 bit period 1.667µs = ~167 counts

---

## Protocol Details

### RC PWM
- Pulse: 1000-2000µs high, period ~2.5-20ms (400-50 Hz)
- At 100 MHz: 1000µs = 100,000 counts, 2000µs = 200,000 counts
- **Framing**: rise/fall edge pairing in ISR (no gap detection)
  - Rising edge → save timestamp
  - Falling edge → width = fall - rise, validate 950-2050µs
  - Rise-to-rise delta validates period (reject <2ms or >25ms)
- Map: 1000µs → throttle 0, 2000µs → throttle 2000
- Deadband: ±25µs around 1000µs = zero throttle

### DShot (150/300/600)
- 16-bit frame: [11-bit throttle][1-bit telem][4-bit CRC]
- Bit-1: ~75% duty, Bit-0: ~37% duty
- DShot600: bit period 1.667µs = ~167 counts @ 100 MHz
- DShot300: bit period 3.333µs = ~333 counts
- DShot150: bit period 6.667µs = ~667 counts
- **Framing**: Ping-pong DMA DMACNT=32 (counted edges = one frame)
- CRC: `(val ^ (val >> 4) ^ (val >> 8)) & 0x0F`
- Throttle 0 = disarm, 1-47 = commands, 48-2047 = throttle
- Map: DShot 48 → throttle 0, DShot 2047 → throttle 2000

### Auto-Detection
Initial detection via ISR (before DMA configured):
- Compute minimum edge-to-edge delta from first captures
- DShot candidate: min delta < 500 counts (< 5µs at 100 MHz)
- PWM candidate: min delta > 50,000 counts (> 500µs)
- Once candidate identified: DShot → arm DMA; PWM → stay ISR
- **Lock requires protocol-specific validity** (not just timing):
  - DShot: N=10 consecutive CRC-valid decoded frames
  - PWM: N=10 consecutive valid width (950-2050µs) AND
    valid rise-to-rise period (2-25ms)

---

## Architecture

```
  RD8 (RP57) ── ICM4R PPS ── SCCP4 IC (every rise/fall, 32-bit)
                                    │
                    ┌───────────────┼───────────────┐
                    │               │               │
              [DETECTING]     [PWM LOCKED]    [DSHOT LOCKED]
              ISR per-edge    ISR per-edge    Ping-pong DMA
              (short burst)   (rise/fall      DMACNT=32 → 64-edge
                               pairing)       ring, rolling CRC
                    │               │          (gap = hint only)
                    │          seqlock write        │
                    │          mailbox         DMA-TC: O(1) decode
                    │               │          CRC ok → mailbox
                    │               │          CRC fail → flag
                    │               │               │
                    └───────────────┼───────────────┘
                                    │
                          RX_Service() [main.c while(1)]
                          ├── seqlock read mailbox (uint16_t seq)
                          ├── if pendingShift: alignment search
                          ├── starvation guard (gap > 1 = dropped)
                          ├── PWM: width → throttle 0-2000
                          ├── DShot: throttle already decoded
                          ├── lock state machine + timeout
                          └── write rxCachedThrottleAdc THEN rxCachedLocked
                                    │
                                    ▼
                         ADC ISR Throttle Mux (24 kHz)
                         read rxCachedLocked THEN rxCachedThrottleAdc
                         ├── ADC pot     → potRaw
                         ├── GSP serial  → gspThrottle * 4095 / 2000
                         ├── PWM/DSHOT   → locked ? cached : 0
                         └── AUTO        → locked ? cached : 0
                              (AUTO = DShot/PWM only, not ADC)
                         On RX loss: latch FAULT_RX_LOSS
```

### Mailbox Design (seqlock pattern)
```c
typedef struct {
    volatile uint16_t seqNum;   /* odd = writing, even = complete */
    uint16_t value;             /* pulse width (PWM) or throttle 0-2047 (DShot) */
    uint8_t  valid;             /* 1 = CRC ok / width in range */
    uint8_t  pad0;
} RX_MAILBOX_T;

volatile RX_MAILBOX_T rxMailbox;
```
- **Producer** (ISR/DMA-TC): `seqNum++` (now odd), write value+valid,
  `seqNum++` (now even). Two increments per update.
- **Consumer** (`RX_Service()`): read seq1, copy value+valid, read
  seq2. Accept only if `seq1 == seq2 && (seq1 & 1) == 0`.
  Gap = `(seq2 - lastSeq) / 2`; gap > 1 = dropped frames.
- **No overflow** — latest frame always available
- **Tear-free** — consumer always gets a consistent snapshot
- **Starvation visible** via `rxDroppedFrames` in GET_RX_STATUS

### ISR Priority

| Priority | ISRs | Purpose |
|----------|------|---------|
| 7 | AD1CMP5, AD2CMP1, _CCT1Interrupt, PWMFault | Commutation + HWZC |
| 6 | AD1CH0 | ADC control loop 24 kHz |
| 5 | Timer1 | Heartbeat, service tick |
| 4 | **_CCT4Interrupt** | PWM edge pair + detection phase |
| 4 | **DMA-TC ISR** | DShot: decode + mailbox |

---

## File Changes

### New Files (4 files)

**`dspic33AKESC/hal/hal_input_capture.h`**
```c
void HAL_IC4_Init(void);
void HAL_IC4_Enable(void);
void HAL_IC4_Disable(void);
void HAL_IC4_ConfigDmaDshot(void);
void HAL_IC4_DisableDma(void);
```

**`dspic33AKESC/hal/hal_input_capture.c`**
- PPS: `_ICM4R = 57` (moved from port_config.c)
- SCCP4 config (matches hal_timer.c:29-39):
  ```c
  CCP4CON1 = 0;
  CCP4CON1bits.T32 = 1;
  CCP4CON1bits.MOD = 0b0011;
  CCP4CON1bits.CLKSEL = 0b000;
  CCP4CON1bits.CCSEL = 1;
  ```
- **`_CCT4Interrupt`** (PWM + detection):
  - Read `CCP4BUF`, track polarity
  - Rising → save timestamp; Falling → width, validate, mailbox write
  - Detection phase: buffer edges for protocol classification
  - Clear `_CCT4IF` at end
- **DShot DMA with rolling CRC alignment + ping-pong**:
  - **Initial lock**: `_CCT4Interrupt` ISR captures edges for protocol
    detection. Gap (>100µs) = *hint* to seed DMA offset; not required.
  - **Arm**: on probable frame boundary, `_CCT4IE = 0`, arm ping-pong
    DMA: source=`&CCP4BUF`, dest=`dmaEdgeBuf[2][32]`, count=32.
    Two buffers alternate — DMA-TC swaps and re-arms immediately.
  - **64-edge overlap buffer**: DMA-TC copies 32 new edges into a
    64-edge ring. Alignment search runs across the full window
    (16 possible bit offsets — each bit = 2 edges). Once aligned, subsequent transfers
    decode at known offset — no search unless CRC fails.
  - **ISR work cap**: DMA-TC ISR does ONE decode at current offset.
    CRC pass → seqlock mailbox. CRC fail → set `pendingShift` flag,
    return. `RX_Service()` (main.c while(1)) performs shift search
    in non-ISR context. ISR is always O(1).
  - **Ping-pong verification**: Milestone C.0 hardware gate test
    required — if dsPIC33AK DMA doesn't support atomic ping-pong,
    fall back to circular DMA + half/full interrupts.

**`dspic33AKESC/input/rx_decode.h`**
```c
void RX_Init(void);
void RX_Service(void);

extern volatile uint16_t rxCachedThrottleAdc;  /* 0-4095 */
extern volatile uint8_t  rxCachedLocked;       /* 0 or 1 */
```

**`dspic33AKESC/input/rx_decode.c`**
- `RX_Service()`: read mailbox, seqNum gap → `rxDroppedFrames`
- PWM: width → throttle; DShot: already decoded
- Lock FSM: UNLOCKED → DETECTING → LOCKING → LOCKED → LOST
- Write ordering: `rxCachedThrottleAdc` first, `rxCachedLocked` second
- Timeout: `rxCachedLocked = 0` first, `rxCachedThrottleAdc = 0` second
- On timeout while motor running (state > ESC_IDLE): latch
  `FAULT_RX_LOSS`, transition to ESC_FAULT (requires CLEAR_FAULT)

### Modified Files (11 files)

**`garuda_calc_params.h`** — Shared clock
```c
/* Top level, no feature guard */
#define SCCP_CLOCK_HZ  100000000UL

/* Inside #if FEATURE_ADC_CMP_ZC */
#define HWZC_TIMER_HZ  SCCP_CLOCK_HZ
```

**`garuda_config.h`** — Features + config + static guard
```c
/* ADC pot: ON for MCLV-48V-300W bench, OFF for flight boards */
#define FEATURE_ADC_POT    1

/* RX features: default OFF */
#define FEATURE_RX_PWM     0
#define FEATURE_RX_DSHOT   0
#define FEATURE_RX_AUTO    0

/* Static guard: at least one throttle source must be enabled */
#if !FEATURE_ADC_POT && !FEATURE_GSP && !FEATURE_RX_PWM \
    && !FEATURE_RX_DSHOT && !FEATURE_RX_AUTO
#error "No throttle source enabled"
#endif

/* AUTO requires at least one RX protocol to detect */
#if FEATURE_RX_AUTO && !FEATURE_RX_PWM && !FEATURE_RX_DSHOT
#error "FEATURE_RX_AUTO requires FEATURE_RX_PWM or FEATURE_RX_DSHOT"
#endif

#define RX_TIMER_HZ          SCCP_CLOCK_HZ
#define RX_COUNTS_PER_US     (RX_TIMER_HZ / 1000000UL)
#define RX_PWM_MIN_US        950
#define RX_PWM_MAX_US        2050
#define RX_PWM_DEADBAND_US   25
#define RX_PWM_PERIOD_MIN_US 2000
#define RX_PWM_PERIOD_MAX_US 25000
#define RX_LOCK_COUNT        10
#define RX_TIMEOUT_MS        200
#define RX_DSHOT_CMD_MAX     47
#define RX_DSHOT_EDGES       32  /* wire truth: always 32 */
#define RX_DSHOT_DMA_COUNT   RX_DSHOT_EDGES  /* DMA register load;
    change to (RX_DSHOT_EDGES - 1) if count-1 semantics */
#define RX_ALIGN_MAX_SHIFTS_PER_CALL  4

/* C.0 DMA gate test timeouts (CCP4TMR counts, not milliseconds) */
#define C0_STARTUP_TIMEOUT_MS      2000
#define C0_STARTUP_TIMEOUT_COUNTS  \
    ((uint32_t)((uint64_t)C0_STARTUP_TIMEOUT_MS * SCCP_CLOCK_HZ / 1000ULL))
#define C0_TARGET_FRAMES           370000  /* ~10s at DShot600 37kHz */
#define C0_MEAS_TIMEOUT_MS         15000   /* 10s + 5s margin */
#define C0_MEAS_TIMEOUT_COUNTS     \
    ((uint32_t)((uint64_t)C0_MEAS_TIMEOUT_MS * SCCP_CLOCK_HZ / 1000ULL))

/* IFS clear method — MUST be set after Milestone 0 probe.
 * Uncomment the correct line after M0 verification: */
/* #define IFS_IS_W1C  1 */  /* W1C: write 1 clears flag */
/* #define IFS_IS_W1C  0 */  /* Direct: write 0 clears flag */
#if C0_DMA_TEST
#ifndef IFS_IS_W1C
#error "IFS_IS_W1C not defined — run Milestone 0 probe first"
#endif
#endif
/* NOTE: c0ClearIfs() logic lives in hal_input_capture.c (finding 88),
 * NOT here. This header only defines the flag. */

/* C.0 error codes */
#define C0_ERR_NONE        0
#define C0_ERR_IC_DRAIN    1  /* ICBNE stuck */
#define C0_ERR_NO_SIGNAL   2  /* no DMA-TC within startup timeout */
#define C0_ERR_MEAS_STALL  3  /* c0Done not set within meas timeout */
```

**`garuda_types.h`** — Enums + data fields + FAULT_RX_LOSS
```c
typedef enum {
    THROTTLE_SRC_ADC=0, THROTTLE_SRC_GSP=1,
    THROTTLE_SRC_PWM=2, THROTTLE_SRC_DSHOT=3, THROTTLE_SRC_AUTO=4
} THROTTLE_SOURCE_T;

typedef enum {
    RX_LINK_UNLOCKED=0, RX_LINK_DETECTING, RX_LINK_LOCKING,
    RX_LINK_LOCKED, RX_LINK_LOST
} RX_LINK_STATE_T;
```
FAULT_CODE_T: add `FAULT_RX_LOSS` after `FAULT_MORPH_TIMEOUT` (index 9).
GARUDA_DATA_T changes:
- **Move `throttleSource` out of `#if FEATURE_GSP`** into the
  unconditional section (after `potRaw`, before `#if FEATURE_GSP`).
  It is a throttle-mux field used by all source types, not GSP-specific.
  `gspThrottle`, `lastGspPacketTick`, intent flags stay inside guard.
- Add (unconditional): `rxLinkState`, `rxProtocol`, `rxCrcErrors`,
  `rxPulseUs`, `rxDshotRate`, `rxDroppedFrames`.

**`garuda_service.c`** — Init + throttle mux
- `GARUDA_ServiceInit()` (line 76): explicitly set `throttleSource`
  unconditionally (NOT inside `#if FEATURE_GSP`), based on
  `FEATURE_ADC_POT` / `FEATURE_RX_*` / `FEATURE_GSP` flags.
  GSP-field inits (`gspThrottle = 0` etc.) stay inside `#if FEATURE_GSP`.
- ADC ISR throttle mux (line 174): **replace `#if FEATURE_GSP` /
  `#else` block** with unconditional `switch(throttleSource)`:
```c
switch (garudaData.throttleSource) {
#if FEATURE_GSP
    case THROTTLE_SRC_GSP:
        garudaData.throttle = (uint16_t)((uint32_t)garudaData.gspThrottle * 4095 / 2000);
        break;
#endif
#if (FEATURE_RX_PWM || FEATURE_RX_DSHOT)
    case THROTTLE_SRC_PWM:
    case THROTTLE_SRC_DSHOT:
    case THROTTLE_SRC_AUTO:
        garudaData.throttle = rxCachedLocked ? rxCachedThrottleAdc : 0;
        break;
#endif
#if FEATURE_ADC_POT
    case THROTTLE_SRC_ADC:
        garudaData.throttle = garudaData.potRaw;
        break;
#endif
    default:
        garudaData.throttle = 0;  /* safe fallback — zero never harms */
        break;
}
```
- Fault handler: `HAL_IC4_Disable()` on ESC_FAULT entry.

**`hal/port_config.c`** — Delete `_ICM1R = 57` (line 150)

**`main.c`** — `RX_Init()` in init + `RX_Service()` in while(1) loop.
- GSP heartbeat timeout (line 248): REMOVE `throttleSource = ADC`
  switch. Keep `gspStopIntent = true` + `gspThrottle = 0` only.
  Motor stops via zero throttle, source stays unchanged.

**`gui/src/protocol/types.ts`** — FAULT_RX_LOSS propagation
```typescript
// Append 'RX_LOSS' to FAULT_CODES array (index 9)
```

**`tools/gsp_test.py`** — FAULT_RX_LOSS propagation
```python
# Add 9: "RX_LOSS" to FAULT_NAMES dict
```

**`gsp/gsp_commands.h`** — `GET_RX_STATUS` (0x26)
```c
typedef struct __attribute__((packed)) {
    uint8_t  linkState;
    uint8_t  protocol;
    uint8_t  dshotRate;
    uint8_t  pad0;
    uint16_t throttle;
    uint16_t pulseUs;
    uint16_t crcErrors;
    uint16_t droppedFrames;  /* starvation counter */
} GSP_RX_STATUS_T;

_Static_assert(sizeof(GSP_RX_STATUS_T) == 12, "RX status wire size");
```

**`gsp/gsp_commands.c`** — Handler + throttle src + feature flags
- `BuildFeatureFlags()` (line 43): add after bit 18:
  `bit 19: FEATURE_ADC_POT, bit 20: RX_PWM, bit 21: RX_DSHOT, bit 22: RX_AUTO`
- `HandleSetThrottleSrc()` (line 174): ALL transitions IDLE-only
  (WRONG_STATE if running). Feature-off = OUT_OF_RANGE
  (`FEATURE_ADC_POT=0` blocks ADC, `FEATURE_RX_*=0` blocks RX).
- GET_RX_STATUS (0x26) handler.

**`Makefile-default.mk`** + **`configurations.xml`** — New files

### GUI Changes (6 files)
- `types.ts` — `GspRxStatus` type + FEATURE_NAMES bits 19-22
- `decode.ts` — `decodeRxStatus()` for 0x26
- `gsp.ts` — Command + telem poll
- `ControlPanel.tsx` — Source selector (show/hide via feature bits);
  GSP always shown when connected; "(only source)" label when no
  other sources reported in GET_INFO feature flags
- `StatusPanel.tsx` — RX status + dropped frames
- `HelpPanel.tsx` — `length: 19` → `23` (line 302)

---

## Implementation Milestones

### Milestone 0: Compile Probe
```c
#include <xc.h>
void rx_probe(void) {
    volatile uint32_t a = CCP4CON1;
    CCP4CON1bits.T32 = 1;
    CCP4CON1bits.CCSEL = 1;
    CCP4CON1bits.MOD = 0b0011;
    CCP4CON1bits.CLKSEL = 0b000;
    volatile uint32_t b = CCP4BUF;
    CCP4CON1bits.ON = 1;
    _ICM4R = 57;
    _CCT4IF = 0;
    _CCT4IE = 1;
    volatile uint32_t c = DMA0CH;
    volatile uint32_t d = DMA0SRC;
    volatile uint32_t e = DMA0DST;
    volatile uint32_t f = DMA0CNT;
    volatile uint32_t g = DMA0SEL;
    volatile uint32_t h = DMA0STAT;
    _DMA0IF = 0;
    _DMA0IE = 1;
    volatile uint32_t i = CLC1CONL;
}
```
Build. Document symbols. Additionally verify:
- **DMA0CNT semantics**: arm DMA with CNT=2, count actual transfers (2 vs 3).
- **DMA0STAT clear method**: set DONE=1 flag, try `DMA0STAT=0` then
  read back. If DONE persists, use W1C (`DMA0STATbits.DONE=1`).
- **CCP4STAT ICBNE symbol**: confirm `CCP4STATbits.ICBNE` exists.
- **CCP4STAT ICOV clear**: force overflow, try `ICOV=0` read back.
  If persists, use W1C (`ICOV=1`). Document result.
- **IFS clear method**: set a known IF bit, try `IFSx=0` read back.
  If persists, use W1C (`IFSx=0xFFFFFFFF`). Document result.
  **Action**: uncomment the correct `#define IFS_IS_W1C` line (0 or 1)
  in `garuda_config.h`. Until set, `C0_DMA_TEST` builds will `#error`
  (finding 89). This selects `c0ClearIfs()` implementation (finding 88).
Delete probe after documenting results. **Gate for all work.**

### Milestone A: PWM Capture via ISR + Mailbox
1. `SCCP_CLOCK_HZ` in garuda_calc_params.h (top-level)
2. `HWZC_TIMER_HZ = SCCP_CLOCK_HZ` (inside guard, verify no regression)
3. Feature flags (default 0) + config in garuda_config.h
4. Types in garuda_types.h (move `throttleSource` out of `#if FEATURE_GSP`)
5. hal_input_capture: SCCP4 IC init, `_CCT4Interrupt` with rise/fall
   edge pairing + seqlock mailbox write
6. rx_decode: seqlock read + PWM decode + lock FSM + cached volatiles
   + starvation guard
7. PPS in HAL_IC4_Init(), remove from port_config.c
8. Init + RX_Service() in main.c
9. Build `FEATURE_RX_PWM=1`, sig gen 1500µs → throttle ~1000

### Milestone B: Throttle Mux + Fail-Safe
1. ADC ISR mux: **replace `#if FEATURE_GSP` / `#else` block** at
   garuda_service.c:174 with unconditional `switch(throttleSource)`.
   Per-source `#if` guards inside the switch. All RX sources → zero
   on unlock (no pot fallback).
2. RX loss → `FAULT_RX_LOSS` latch (requires CLEAR_FAULT to restart)
3. Timeout: LOST → zero cached values
4. Arming gate for RX sources
5. `GARUDA_ServiceInit()` (garuda_service.c:76): unconditional
   `throttleSource` init based on feature flags (NOT inside
   `#if FEATURE_GSP`). GSP-field inits stay guarded. Static assert
   if no source enabled. Zero throttle cache.
6. GSP heartbeat timeout (main.c:248): remove `throttleSource = ADC`
   switch. Keep `gspStopIntent + gspThrottle = 0` only.
7. `HandleSetThrottleSrc()` full transition matrix:
   - ALL transitions: IDLE-only (WRONG_STATE if running)
   - Feature-off: OUT_OF_RANGE (`FEATURE_ADC_POT=0` blocks ADC,
     `FEATURE_RX_*=0` blocks RX sources)
8. `FEATURE_ADC_POT` flag: default 1 (MCLV-48V-300W has pot),
   set 0 for flight boards (blocks ADC source selection)
9. GUI `ControlPanel.tsx`: ALL source buttons disabled when !isIdle,
   hide buttons for features not reported in GET_INFO. GSP button
   always shown when connected. "(only source)" label when no other
   sources reported in feature flags.
10. Build: PWM → motor spins; disconnect → motor stops within 200ms

### Milestone C: DShot via DMA + Mailbox
1. **C.0 Ping-pong DMA gate test**: `C0_DMA_TEST=1` compile flag;
   dedicated sync GPIO (LED2 exclusive, or spare test point).
   Shared vars: `volatile uint32_t dmaFrameCount, c0FinalCount;
   volatile uint8_t c0Done, c0Started, c0Error;` Main reads
   `c0FinalCount` only after `c0Done == 1` (happens-before).
   **ISR isolation**: save+clear IEC0-IEC3, save `_DMA0IP`, then
   re-enable only `_DMA0IE=1; _DMA0IP=7;`.
   **Cleanup**: single `c0Cleanup()` helper called from ALL exit
   paths (success, no-signal timeout, measurement timeout, IC drain
   error). Performs: GPIO=0, CHEN=0, `_DMA0IE=0`, `c0ClearIfs()`
   (M0-determined, finding 88), restore IEC0-IEC3, restore `_DMA0IP`.
   **Timeout**: main-loop polls use `CCP4TMR` (100MHz free-run,
   not suppressed Timer1) with `C0_STARTUP_TIMEOUT_COUNTS` (2s)
   and `C0_MEAS_TIMEOUT_COUNTS` (15s). Thresholds derived from
   `_MS × SCCP_CLOCK_HZ / 1000` with 64-bit intermediate (finding 85).
   No-signal or stuck → `c0Cleanup()` + set error code.
   **Start (frame-aligned)**: flush IC (bounded drain: max 8 iter,
   abort on stuck ICBNE); clear DMA state (W1C); arm DMA silently
   (`CHEN=1; _DMA0IE=1;` — GPIO still LOW). DMA-TC ISR: on first
   callback (`dmaFrameCount==1`), set `GPIO=1; dmaFrameCount=0;
   c0Started=1;` (alignment frame, not counted).
   **Stop (frame-aligned)**: DMA-TC ISR: `if (c0Started &&
   dmaFrameCount >= C0_TARGET_FRAMES) { c0FinalCount=dmaFrameCount;
   GPIO=0; CHEN=0; _DMA0IE=0; c0Done=1; }`. Main polls `c0Done`,
   then calls `c0Cleanup()` (idempotent re-exec of ISR's CHEN/IE/GPIO).
   **Both window edges are at DMA-TC boundaries.**
   **Acceptance**: `LA_edges == c0FinalCount × RX_DSHOT_EDGES`
   (strict ==, zero tolerance — any mismatch = FAIL). Measured ISR
   latency is diagnostic only, never relaxes acceptance. Milestone 0
   verifies DMA0CNT semantics, DMA0STAT/IFS clear method, ICBNE,
   ICOV. Supplementary: two-phase firmware counter. If ping-pong
   not atomic, fall back to circular DMA. **Gate for C.1+**
2. **C.1 DMA arm**: ping-pong DMA source=`&CCP4BUF`,
   dest=`dmaEdgeBuf[2][32]`, count=32. Gap hint seeds initial offset.
3. **C.2 Overlap buffer + rolling CRC alignment**: 64-edge ring buffer.
   DMA-TC copies 32 new edges into ring. ISR does ONE decode at current
   offset — CRC pass → seqlock mailbox. CRC fail → set `pendingShift`
   flag, return (O(1) ISR). `RX_Service()` tries up to
   `RX_ALIGN_MAX_SHIFTS_PER_CALL` (4) offsets per call; resumes next
   call if not aligned. Max 4 iterations to search all 16 bit offsets.
   All offsets exhausted → re-detect via ISR.
4. **C.3 Seqlock producer**: DMA-TC ISR writes seqNum++ (odd), payload,
   seqNum++ (even) for tear-free mailbox.
5. DShot rate auto-detect (from edge timing in detect phase)
6. **CPU measurement**: toggle GPIO in DMA-TC ISR, scope duty cycle.
   **Must be < 5%** before proceeding. Document measured value.
7. Build `FEATURE_RX_DSHOT=1`, DShot600 from FC → tracks

### Milestone D: Auto-Detection + Arbitration
1. Detection phase: ISR captures initial edges
2. `detectProtocol()` in RX_Service(): min-delta classification
3. DShot detected → arm DMA mode; PWM detected → stay ISR mode
4. AUTO = DShot/PWM only (ADC pot is not an RX protocol)
5. Signal loss → LOST → zero + FAULT_RX_LOSS → re-detect on CLEAR_FAULT
6. Build `FEATURE_RX_AUTO=1`

### Milestone E: CLC Deglitch (optional Phase 2)
1. CLC1 D-FF sync, spare RPn loopback
2. `FEATURE_RX_CLC` flag
3. Only if real edge noise causes failures

### Milestone F: GSP/GUI Integration + Testing
1. `GET_RX_STATUS` (0x26) with `droppedFrames` field
2. Throttle src + feature flags (canonical bit table comment in
   `gsp_commands.c:BuildFeatureFlags()`)
3. GUI: decode, selector, status
4. HW test: PWM sig gen, DShot FC
5. Regression: features at 0 = identical binary
6. Feature bit consistency test in `gsp_test.py`:
   - Highest set bit in GET_INFO featureFlags < `len(FEATURE_NAMES)`
   - Explicit name assertions: `FEATURE_NAMES[19]=="ADC_POT"`,
     `[20]=="RX_PWM"`, `[21]=="RX_DSHOT"`, `[22]=="RX_AUTO"`
7. Invalid source test in `gsp_test.py`: SET_THROTTLE_SRC with 0xFF
   → verify GSP_ERR_OUT_OF_RANGE response

---

## Safety Rules

1. **RX loss = zero throttle + FAULT_RX_LOSS** for all RX sources
   (PWM, DSHOT, AUTO). No pot fallback — flight boards have no pot,
   floating ADC pin (RA11) would produce random throttle values.
   Fault state requires explicit CLEAR_FAULT to restart.
2. **Seqlock mailbox**: tear-free reads via seq1/copy/seq2 pattern
3. **Arming gate**: low throttle window required
4. **Source switch matrix**: ALL transitions IDLE-only, feature-off=
   rejected (`FEATURE_ADC_POT=0` blocks ADC); GUI mirrors constraints
5. **DShot commands 1-47**: ignored
6. **Features default OFF**: zero regression
7. **Existing protections untouched**: OC, CLPCI, Vbus, desync
8. **CPU budget gate**: DShot ISR must measure < 5% before Milestone D

---

## Verification

- `FEATURE_RX_*=0` (default): identical binary, zero regression
- Compile probe: all symbols resolve
- `SCCP_CLOCK_HZ` refactor: HWZC unchanged
- PWM 50 Hz + 400 Hz: 1000/1500/2000µs → throttle 0/1000/2000 ±10
- PWM rise-to-rise: reject edges outside 2-25ms period
- PWM jitter: ±50µs → stable throttle (< ±20)
- PWM dropout: LOST within 200ms → throttle 0
- DShot600: 48-2047 → 0-2000, CRC pass > 99.9%
- DShot150: no false splits (DMA counted-edge)
- DShot CPU: measured < 5% via GPIO toggle
- DShot starvation: `rxDroppedFrames` counts correctly under load
- Auto-detect: PWM → ISR; DShot → DMA (AUTO = DShot/PWM only)
- DShot rolling CRC alignment: 64-edge ring, lock without requiring gap
- DShot resync: 5 CRC fails + all 16 bit offsets exhausted → re-detect
- DShot ping-pong DMA: C.0 LA primary (GPIO-bounded 10s DShot600)
- DShot ping-pong fallback: circular DMA + half/full if atomic swap fails
- DShot DMA-TC ISR: O(1) always — shift search deferred to RX_Service()
- RX_Service() alignment: max 4 shifts per call, no main-loop starvation
- Seqlock: mailbox read always consistent (seq1 == seq2 && even)
- Throttle src matrix: ALL transitions IDLE-only, feature-off=reject
- FEATURE_ADC_POT=0: ADC source blocked, boot source auto-selected
- FEATURE_ADC_POT=0: GARUDA_ServiceInit() sets safe default source
- GSP heartbeat timeout: no source switch, motor stops via zero throttle
- GUI mirrors transition matrix (ALL buttons disabled when !isIdle)
- Detection lock: DShot needs N CRC-valid frames, PWM needs N valid width+period
- seqNum uint16_t: no wrap-induced false drop counts under sustained DShot600
- RX loss (any source): zero throttle + FAULT_RX_LOSS latch
- FAULT_RX_LOSS: motor stops, requires CLEAR_FAULT to restart
- Motor running + RX: no jitter, no ISR starvation
- Write ordering: no transient stale-true windows
- GET_RX_STATUS: link, protocol, errors, droppedFrames
- GET_SNAPSHOT: unchanged 68B
- `throttleSource` compiles with FEATURE_GSP=0 + FEATURE_RX_PWM=1 (RX-only build)
- ADC ISR mux: unconditional switch, not wrapped by `#if FEATURE_GSP`
- RX-only build (GSP=0, RX_PWM=1): rxCachedThrottleAdc reaches motor
- GARUDA_ServiceInit(): `throttleSource` init is unconditional, GSP fields guarded
- C.0 GPIO sync: LA window bounded by firmware GPIO markers
- Offset search: 16 bit offsets (not 32), max 4 iterations
- GUI: GSP-only config shows "GSP (only source)" — no empty selector
- Feature bit test: highest set bit < len(FEATURE_NAMES) + explicit name assertions for 19-22
- ADC ISR default: always 0 (safe for corrupted enum + FEATURE_ADC_POT=0)
- SET_THROTTLE_SRC with invalid value (0xFF) → GSP_ERR_OUT_OF_RANGE
- C.0 start: frame-aligned (GPIO=1 from first DMA-TC ISR, not main loop)
- C.0 stop: frame-aligned (GPIO=0 from DMA-TC ISR at target count)
- C.0 strict == only — any mismatch = FAIL, no ±1 tolerance
- C.0 ISR isolation: IEC0-IEC3 saved/cleared, only _DMA0IE at priority 7
- C.0 IEC restore: `c0ClearIfs()` (M0-determined W1C/direct, in hal_input_capture.c), then restore IEC
- C.0 timeout: CCP4TMR count-based (`_COUNTS` derived from `_MS × SCCP_CLOCK_HZ / 1000`)
- C.0 no-signal abort: `c0Cleanup()` restores IEC/IFS/DMA0IP, c0Error=C0_ERR_NO_SIGNAL
- C.0 all exit paths call `c0Cleanup()` (success, no-signal, meas stall, IC drain)
- C.0 `_DMA0IP` saved/restored in every path (finding 86)
- C.0 IFS strategy: single `c0ClearIfs()` in hal_input_capture.c, no macro in config header
- C.0 `IFS_IS_W1C`: undefined pre-M0, `#error` if C0_DMA_TEST built without it (finding 89)
- C.0 IC drain: bounded loop (max 8 iter), abort on stuck ICBNE
- C.0 shared vars: volatile uint32_t; main reads c0FinalCount only after c0Done
- C.0 DMA symbols: `_DMA0IE`/`DMA0CH`/`DMA0CNT` (per-channel)
- C.0 Milestone 0: DMA0CNT semantics + DMA0STAT/IFS clear + ICBNE + ICOV
- C.0 sync GPIO: dedicated (C0_DMA_TEST flag disables other LED2 writes)
- FEATURE_RX_AUTO=1 + RX_PWM=0 + RX_DSHOT=0 → compile error
