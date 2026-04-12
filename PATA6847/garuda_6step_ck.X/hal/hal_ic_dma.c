/**
 * @file hal_ic_dma.c
 * @brief Dual-CCP + DMA shadow ring implementation.
 *
 * See hal_ic_dma.h for architecture.
 */

#include "hal_ic_dma.h"

#if FEATURE_IC_ZC && FEATURE_IC_DMA_SHADOW

#include <xc.h>
#include "hal_com_timer.h"   /* HAL_ComTimer_ReadTimer */

/* DMA trigger source IDs for this device (Table 10-1, datasheet page 219) */
#define DMA_CHSEL_SCCP2   0x0Cu   /* SCCP2 Interrupt */
#define DMA_CHSEL_SCCP5   0x16u   /* SCCP5 Interrupt */

/* ── Ring buffers in RAM (256 bytes each) ───────────────────────────── */
static volatile uint16_t dmaRingRising [DMA_ZC_RING_SIZE];  /* CCP2 falling edges */
static volatile uint16_t dmaRingFalling[DMA_ZC_RING_SIZE];  /* CCP5 rising edges */

/* CCP2/CCP5 → HR (CCP4) domain offsets, measured once at init.
 *   HR_tick = ccp_tick + offset
 * All values are 16-bit unsigned; arithmetic wraps naturally. */
static uint16_t ccp2_to_hr_offset;
static uint16_t ccp5_to_hr_offset;

/* Commutation markers — captured head positions at commutation time.
 * Used to bound ring scans to captures-since-this-commutation. */
static volatile uint8_t commHeadRising;   /* DMACNT0 value at commutation */
static volatile uint8_t commHeadFalling;  /* DMACNT1 value at commutation */

/* ──────────────────────────────────────────────────────────────────── */

static inline uint8_t dmaRingHeadRising(void)
{
    /* DMACNTn decrements from initial count toward zero, then wraps.
     * Effective write index = (DMA_ZC_RING_SIZE - DMACNTn) & MASK.
     * DMACNT0 starts at DMA_ZC_RING_SIZE and decrements on each transfer.
     * After wrap it goes back to DMA_ZC_RING_SIZE. So the "next write"
     * index is (DMA_ZC_RING_SIZE - DMACNT0) modulo DMA_ZC_RING_SIZE. */
    uint16_t cnt = DMACNT0;
    return (uint8_t)((DMA_ZC_RING_SIZE - cnt) & DMA_ZC_RING_MASK);
}

static inline uint8_t dmaRingHeadFalling(void)
{
    uint16_t cnt = DMACNT1;
    return (uint8_t)((DMA_ZC_RING_SIZE - cnt) & DMA_ZC_RING_MASK);
}

/* ──────────────────────────────────────────────────────────────────── */

void HAL_ZcDma_Init(void)
{
    /* ── Global DMA module enable (REQUIRED, datasheet §10.2 step 1) ─── */
    /* DMACONbits.DMAEN defaults to 0 at POR which disables ALL DMA
     * regardless of channel config. Must be set before any channel work. */
    DMACONbits.DMAEN  = 1;           /* DMA module ON */
    DMACONbits.PRSSEL = 0;           /* Fixed priority scheme */

    /* ── DMA address limits (REQUIRED, datasheet §10.2 step 2) ──────── */
    /* DMAL is the LOWEST address the DMA is allowed to read/write in
     * the range above 0x07FF (the base SFR space is always allowed).
     * DMAH is the HIGHEST allowed address.
     *
     * Setting DMAL=0x1000 would FORBID the range [0x800, 0x0FFF] which
     * is where the DMA-capable SFRs live including CCP2BUFL (@0x0994)
     * and CCP5BUFL (@0x0A00). That silently blocks every DMA read
     * from the capture buffers — we see this as zero captures in
     * the ring. Set DMAL low enough to include the whole DMA SFR
     * bus, and DMAH to the end of data RAM. */
    DMAL = 0x0800u;                  /* include entire DMA-capable SFR range */
    DMAH = 0x2FFFu;                  /* end of data RAM on 33CK64MP205 */

    /* ── Configure CCP2: free-running capture, falling edge (rising ZC) ── */
    CCP2CON1L = 0;
    CCP2CON1H = 0;
    CCP2CON2L = 0;
    CCP2CON2H = 0;

    CCP2CON1Lbits.CCSEL  = 1;       /* Input Capture mode */
    CCP2CON1Lbits.T32    = 0;       /* 16-bit */
    CCP2CON1Lbits.CLKSEL = 0b000;   /* Fp — same as CCP4 */
    CCP2CON1Lbits.TMRPS  = 0b11;    /* 1:64 prescaler → 640 ns/tick */
    CCP2CON1Lbits.MOD    = 0b0010;  /* Every falling edge → rising ZCs
                                     * (ATA6847 comparator is inverted) */
    CCP2PRL = 0xFFFFu;              /* Free-running, no period reset */

    /* CCP2 CPU interrupt: DISABLED. The DMA trigger on this family
     * fires on the peripheral IF signal regardless of IE — DMA
     * acknowledges the transfer by clearing IF itself (FRM
     * DS30009742C §4.3: "acknowledges the transfer by clearing the
     * peripheral's interrupt request"). Enabling IE adds a parasitic
     * CPU ISR dispatch on every comparator edge — at ~15 cycles per
     * dispatch × tens of thousands of edges per step that's enough
     * to break the motor even with a trivial stub ISR. */
    _CCP2IP = 1;
    _CCP2IF = 0;
    _CCP2IE = 0;                    /* DMA triggers on IF alone */
    _CCT2IP = 1;
    _CCT2IF = 0;
    _CCT2IE = 0;

    CCP2CON1Lbits.CCPON = 1;        /* Start — never toggled again */

    /* ── Configure CCP5: free-running capture, rising edge (falling ZC) ── */
    CCP5CON1L = 0;
    CCP5CON1H = 0;
    CCP5CON2L = 0;
    CCP5CON2H = 0;

    CCP5CON1Lbits.CCSEL  = 1;
    CCP5CON1Lbits.T32    = 0;
    CCP5CON1Lbits.CLKSEL = 0b000;
    CCP5CON1Lbits.TMRPS  = 0b11;
    CCP5CON1Lbits.MOD    = 0b0001;  /* Every rising edge → falling ZCs */
    CCP5PRL = 0xFFFFu;

    _CCP5IP = 1;
    _CCP5IF = 0;
    _CCP5IE = 0;                    /* DMA triggers on IF alone — no CPU dispatch */
    _CCT5IP = 1;
    _CCT5IF = 0;
    _CCT5IE = 0;

    CCP5CON1Lbits.CCPON = 1;

    /* ── Configure DMA0: CCP2BUFL → dmaRingRising[] ─────────────────── */
    /* DMACHn register bits:
     *   CHEN     bit 0   — 0 (enable after setup)
     *   SIZE     bit 1   — 0 = word (16-bit), 1 = byte
     *   TRMODE   bits 3:2 — 00 = single (one transfer per trigger)
     *   DAMODE   bits 5:4 — 01 = post-increment continuous (circular)
     *   SAMODE   bits 7:6 — 00 = fixed (source stays at CCPxBUFL)
     *   CHREQ    bit 8   — software trigger (not used)
     *   RELOAD   bit 9   — 1 = reload src/dst/cnt on match (auto-rearm)
     *   NULLW    bit 10  — 0
     *
     * DMAINTn.CHSEL[6:0] at bits 14:8 — trigger source selector
     */
    DMACH0 = 0;                                 /* clear */
    DMAINT0 = 0;
    DMASRC0 = (uint16_t)((unsigned int)&CCP2BUFL);    /* source: SCCP2 capture buffer */
    DMADST0 = (uint16_t)((unsigned int)&dmaRingRising[0]);
    DMACNT0 = DMA_ZC_RING_SIZE;                 /* transfer count before wrap */
    DMACH0bits.SIZE   = 0;                      /* 16-bit word */
    DMACH0bits.TRMODE = 0b01;                   /* REPEATED One-Shot:
                                                 * one transfer per trigger,
                                                 * DMACNT auto-reloads to
                                                 * DMA_ZC_RING_SIZE on rollover,
                                                 * channel stays enabled forever.
                                                 * NB: TRMODE=00 (One-Shot)
                                                 * would disable the channel
                                                 * after the first DMACNT=0 — */
    DMACH0bits.DAMODE = 0b01;                   /* dst post-increment, wraps
                                                 * via automatic DMADST reload
                                                 * in Repeated One-Shot mode */
    DMACH0bits.SAMODE = 0b00;                   /* src fixed */
    DMACH0bits.RELOAD = 1;                      /* CRITICAL: reload DMADST+DMACNT on
                                                 * rollover. Without this the
                                                 * Repeated-One-Shot mode only
                                                 * reloads DMACNT and leaves
                                                 * DMADST incrementing forever,
                                                 * which walks the DMA write
                                                 * pointer past the ring buffer
                                                 * into neighbouring .bss and
                                                 * clobbers GARUDA_DATA_T.
                                                 * With RELOAD=1 DMADST resets
                                                 * to the ring base on every
                                                 * wrap → true circular ring. */
    DMAINT0bits.CHSEL = DMA_CHSEL_SCCP2;        /* trigger on SCCP2 capture IF */
    DMACH0bits.CHEN   = 1;                      /* enable */

    /* ── Configure DMA1: CCP5BUFL → dmaRingFalling[] ────────────────── */
    DMACH1 = 0;
    DMAINT1 = 0;
    DMASRC1 = (uint16_t)((unsigned int)&CCP5BUFL);
    DMADST1 = (uint16_t)((unsigned int)&dmaRingFalling[0]);
    DMACNT1 = DMA_ZC_RING_SIZE;
    DMACH1bits.SIZE   = 0;
    DMACH1bits.TRMODE = 0b01;                   /* REPEATED One-Shot */
    DMACH1bits.DAMODE = 0b01;
    DMACH1bits.SAMODE = 0b00;
    DMACH1bits.RELOAD = 1;                      /* circular ring — see DMA0 */
    DMAINT1bits.CHSEL = DMA_CHSEL_SCCP5;
    DMACH1bits.CHEN   = 1;

    /* ── Establish CCP → HR domain offsets ──────────────────────────── */
    /* Both CCP2 and CCP5 are now running with the same prescaler as
     * CCP4 (Fp/64 = 640 ns/tick). Take a paired snapshot to capture
     * the fixed offset. The 16-bit wrap-around arithmetic works:
     *   HR_tick = (ccp_tick + offset) mod 2^16 */
    {
        /* Minimal latency paired reads — keep these adjacent. */
        uint16_t hr_before = HAL_ComTimer_ReadTimer();
        uint16_t c2        = CCP2TMRL;
        uint16_t c5        = CCP5TMRL;
        uint16_t hr_after  = HAL_ComTimer_ReadTimer();
        (void)hr_before;
        ccp2_to_hr_offset = (uint16_t)(hr_after - c2);
        ccp5_to_hr_offset = (uint16_t)(hr_after - c5);
    }

    /* Initial commutation markers at head 0 */
    commHeadRising  = 0;
    commHeadFalling = 0;
}

/* ──────────────────────────────────────────────────────────────────── */

void HAL_ZcDma_OnCommutation(uint16_t rpPin, bool risingZc)
{
    /* Route floating-phase RP pin to BOTH CCP2 and CCP5 input captures.
     * PPS input selectors are independent 8-bit fields in different
     * RPINRn registers; pointing both at the same RP is supported
     * (verified in datasheet, no exclusion rule). */
    __builtin_write_RPCON(0x0000);       /* unlock */
    RPINR4bits.ICM2R = (uint8_t)rpPin;   /* CCP2 IC input */
    RPINR7bits.ICM5R = (uint8_t)rpPin;   /* CCP5 IC input */
    __builtin_write_RPCON(0x0800);       /* lock */

    /* Capture current write heads — ring scan for this step will only
     * look at captures added since this commutation. */
    commHeadRising  = dmaRingHeadRising();
    commHeadFalling = dmaRingHeadFalling();

    (void)risingZc;  /* selected at probe time via the ring parameter */
}

/* ──────────────────────────────────────────────────────────────────── */

void HAL_ZcDma_Probe(uint16_t windowOpenHR,
                     uint16_t expectedHR,
                     uint16_t pollAcceptedHR,
                     bool     risingZc,
                     HAL_ZcDma_Result *result)
{
    volatile uint16_t *ring;
    uint16_t           offset;
    uint8_t            mark;
    uint8_t            head;

    if (risingZc)
    {
        ring   = dmaRingRising;
        offset = ccp2_to_hr_offset;
        mark   = commHeadRising;
        head   = dmaRingHeadRising();
    }
    else
    {
        ring   = dmaRingFalling;
        offset = ccp5_to_hr_offset;
        mark   = commHeadFalling;
        head   = dmaRingHeadFalling();
    }

    /* Count captures since commutation marker. With DAMODE=01 and
     * RELOAD=1, the ring wraps automatically; the "captures since mark"
     * is the forward distance from mark to head modulo ring size. */
    uint8_t captures = (uint8_t)((head - mark) & DMA_ZC_RING_MASK);

    result->edgeCount = captures;
    result->found = false;
    result->ringWrappedSinceMark = false;
    result->earliestHR = 0;
    result->closestHR  = 0;
    result->earliestVsPoll = 0;
    result->earliestVsExpected = 0;
    result->closestVsExpected  = 0;

    /* Ring near-full heuristic: if we're consuming more than 3/4 of the
     * ring in one step, treat as wrapped (we may have lost old entries). */
    if (captures > (DMA_ZC_RING_SIZE * 3u / 4u))
    {
        result->ringWrappedSinceMark = true;
        /* Still scan — we have SOME entries, just may have missed some. */
    }

    if (captures == 0)
    {
        return;  /* no captures this step */
    }

    /* Walk forward from mark, convert to HR domain, find:
     *   - earliest entry with hr >= windowOpenHR
     *   - entry closest to expectedHR (min |hr - expectedHR|) */
    bool     haveEarliest = false;
    uint16_t earliestHR   = 0;
    uint16_t closestHR    = 0;
    uint16_t bestAbsErr   = 0xFFFFu;

    uint8_t idx = mark;
    for (uint8_t i = 0; i < captures; i++)
    {
        uint16_t rawCap = ring[idx];
        uint16_t hrCap  = (uint16_t)(rawCap + offset);

        /* "hrCap >= windowOpenHR" in a wrap-safe sense: the distance
         * (hrCap - windowOpenHR) interpreted as int16_t should be >= 0
         * for captures in the current window. Captures from previous
         * steps would be far in the past (large negative int16). */
        int16_t sinceOpen = (int16_t)(hrCap - windowOpenHR);
        if (sinceOpen >= 0)
        {
            if (!haveEarliest)
            {
                earliestHR   = hrCap;
                haveEarliest = true;
            }

            int16_t err = (int16_t)(hrCap - expectedHR);
            uint16_t absErr = (err < 0) ? (uint16_t)(-err) : (uint16_t)err;
            if (absErr < bestAbsErr)
            {
                bestAbsErr = absErr;
                closestHR  = hrCap;
            }
        }

        idx = (uint8_t)((idx + 1u) & DMA_ZC_RING_MASK);
    }

    if (haveEarliest)
    {
        result->found = true;
        result->earliestHR         = earliestHR;
        result->closestHR          = closestHR;
        result->earliestVsPoll     = (int16_t)(earliestHR  - pollAcceptedHR);
        result->earliestVsExpected = (int16_t)(earliestHR  - expectedHR);
        result->closestVsExpected  = (int16_t)(closestHR   - expectedHR);
    }
}

/* ──────────────────────────────────────────────────────────────────── */

uint16_t HAL_ZcDma_RefineTimestamp(uint16_t  pollHR,
                                   uint16_t  maxLookbackHR,
                                   bool      risingZc,
                                   int16_t  *correctionOut)
{
    volatile uint16_t *ring;
    uint16_t           offset;
    uint8_t            mark;
    uint8_t            head;

    if (risingZc)
    {
        ring   = dmaRingRising;
        offset = ccp2_to_hr_offset;
        mark   = commHeadRising;
        head   = dmaRingHeadRising();
    }
    else
    {
        ring   = dmaRingFalling;
        offset = ccp5_to_hr_offset;
        mark   = commHeadFalling;
        head   = dmaRingHeadFalling();
    }

    uint8_t captures = (uint8_t)((head - mark) & DMA_ZC_RING_MASK);

    if (captures == 0)
    {
        if (correctionOut) *correctionOut = 0;
        return pollHR;
    }

    /* ── Last-cluster-before-poll selector ──────────────────────────
     *
     * Walk all captures since commutation in forward order, clustering
     * edges with ≤ DMA_CLUSTER_GAP_HR between them. Track the last
     * cluster with count ≥ 2 that ends strictly before pollHR.
     *
     * Return the cluster midpoint = (first + last) / 2, which offline
     * analysis showed has stdev 1.3–7 µs across 25–90k eRPM,
     * collapsing the rising/falling polarity split to < 3 µs.
     *
     * The maxLookbackHR bound still applies: if the winning cluster
     * midpoint is further than maxLookbackHR from pollHR, fall back
     * to pollHR unchanged. */

    #define DMA_CLUSTER_GAP_HR  10u   /* 6.4 µs — matches offline gap=6 µs */

    /* Cluster state — tracked as we walk forward. */
    uint16_t clStart = 0;     /* HR of first edge in current cluster */
    uint16_t clEnd   = 0;     /* HR of last edge so far in cluster */
    uint8_t  clCount = 0;     /* edges in current cluster */

    /* Best candidate: the last qualifying cluster before pollHR. */
    bool     haveCandidate = false;
    uint16_t candMidHR     = pollHR;

    uint8_t idx = mark;
    for (uint8_t i = 0; i < captures; i++)
    {
        uint16_t rawCap = ring[idx];
        uint16_t hrCap  = (uint16_t)(rawCap + offset);

        /* Is this edge before pollHR? (wrap-safe signed compare) */
        int16_t vsPoll = (int16_t)(hrCap - pollHR);
        if (vsPoll >= 0)
        {
            /* At or past poll — stop scanning.  First close any open
             * cluster that was still accumulating. */
            if (clCount >= 2)
            {
                uint16_t mid = (uint16_t)(((uint32_t)clStart + clEnd) / 2u);
                candMidHR     = mid;
                haveCandidate = true;
            }
            break;
        }

        /* Cluster logic: if gap from previous edge > threshold,
         * close the old cluster and open a new one. */
        if (clCount == 0)
        {
            /* First edge — start a new cluster. */
            clStart = hrCap;
            clEnd   = hrCap;
            clCount = 1;
        }
        else
        {
            uint16_t gap = (uint16_t)(hrCap - clEnd);
            if (gap <= DMA_CLUSTER_GAP_HR)
            {
                /* Same cluster — extend. */
                clEnd = hrCap;
                clCount++;
            }
            else
            {
                /* Gap exceeded — close old cluster, possibly save it. */
                if (clCount >= 2)
                {
                    uint16_t mid = (uint16_t)(((uint32_t)clStart + clEnd) / 2u);
                    candMidHR     = mid;
                    haveCandidate = true;
                }
                /* Open new cluster. */
                clStart = hrCap;
                clEnd   = hrCap;
                clCount = 1;
            }
        }

        idx = (uint8_t)((idx + 1u) & DMA_ZC_RING_MASK);
    }

    /* If we exhausted the ring without hitting pollHR, close any
     * trailing cluster. */
    if (clCount >= 2)
    {
        int16_t tailVsPoll = (int16_t)(clEnd - pollHR);
        if (tailVsPoll < 0)
        {
            uint16_t mid = (uint16_t)(((uint32_t)clStart + clEnd) / 2u);
            candMidHR     = mid;
            haveCandidate = true;
        }
    }

    if (!haveCandidate)
    {
        if (correctionOut) *correctionOut = 0;
        return pollHR;
    }

    /* Enforce the lookback bound. */
    int16_t correction = (int16_t)(candMidHR - pollHR);
    uint16_t absDist = (correction < 0) ? (uint16_t)(-correction)
                                        : (uint16_t)correction;
    if (absDist > maxLookbackHR)
    {
        if (correctionOut) *correctionOut = 0;
        return pollHR;
    }

    if (correctionOut)
        *correctionOut = correction;
    return candMidHR;

    #undef DMA_CLUSTER_GAP_HR
}

/* ──────────────────────────────────────────────────────────────────── */

uint8_t HAL_ZcDma_DumpSinceCommutation(bool      risingZc,
                                       uint16_t *outBuf,
                                       uint8_t   maxCount)
{
    volatile uint16_t *ring;
    uint16_t           offset;
    uint8_t            mark;
    uint8_t            head;

    if (risingZc)
    {
        ring   = dmaRingRising;
        offset = ccp2_to_hr_offset;
        mark   = commHeadRising;
        head   = dmaRingHeadRising();
    }
    else
    {
        ring   = dmaRingFalling;
        offset = ccp5_to_hr_offset;
        mark   = commHeadFalling;
        head   = dmaRingHeadFalling();
    }

    uint8_t captures = (uint8_t)((head - mark) & DMA_ZC_RING_MASK);
    if (captures > maxCount) captures = maxCount;
    if (captures == 0 || outBuf == 0) return 0;

    uint8_t idx = mark;
    for (uint8_t i = 0; i < captures; i++)
    {
        uint16_t rawCap = ring[idx];
        outBuf[i] = (uint16_t)(rawCap + offset);
        idx = (uint8_t)((idx + 1u) & DMA_ZC_RING_MASK);
    }
    return captures;
}

/* ──────────────────────────────────────────────────────────────────── */

bool HAL_ZcDma_DetectZc(uint16_t  windowOpenHR,
                        bool      risingZc,
                        uint16_t *zcTimestampOut)
{
    volatile uint16_t *ring;
    uint16_t           offset;
    uint8_t            mark;
    uint8_t            head;

    if (risingZc)
    {
        ring   = dmaRingRising;
        offset = ccp2_to_hr_offset;
        mark   = commHeadRising;
        head   = dmaRingHeadRising();
    }
    else
    {
        ring   = dmaRingFalling;
        offset = ccp5_to_hr_offset;
        mark   = commHeadFalling;
        head   = dmaRingHeadFalling();
    }

    uint8_t captures = (uint8_t)((head - mark) & DMA_ZC_RING_MASK);
    if (captures < 2) return false;

    /* Walk forward from commutation marker, cluster edges with
     * gap ≤ DMA_CLUSTER_GAP_HR. Track the LAST cluster with ≥2
     * edges that starts at or after windowOpenHR. */
    #define DMA_DETECT_GAP_HR  10u   /* ~6.4 µs */

    uint16_t clStart   = 0;
    uint16_t clEnd     = 0;
    uint8_t  clCount   = 0;
    bool     haveMatch = false;
    uint16_t matchMid  = 0;

    uint8_t idx = mark;
    for (uint8_t i = 0; i < captures; i++)
    {
        uint16_t rawCap = ring[idx];
        uint16_t hrCap  = (uint16_t)(rawCap + offset);

        if (clCount == 0)
        {
            clStart = hrCap;
            clEnd   = hrCap;
            clCount = 1;
        }
        else
        {
            uint16_t gap = (uint16_t)(hrCap - clEnd);
            if (gap <= DMA_DETECT_GAP_HR)
            {
                clEnd = hrCap;
                clCount++;
            }
            else
            {
                /* Close old cluster — check if qualifying */
                if (clCount >= 2)
                {
                    int16_t sinceOpen = (int16_t)(clStart - windowOpenHR);
                    if (sinceOpen >= 0)
                    {
                        matchMid  = (uint16_t)(((uint32_t)clStart + clEnd) / 2u);
                        haveMatch = true;
                    }
                }
                clStart = hrCap;
                clEnd   = hrCap;
                clCount = 1;
            }
        }

        idx = (uint8_t)((idx + 1u) & DMA_ZC_RING_MASK);
    }

    /* Check trailing cluster */
    if (clCount >= 2)
    {
        int16_t sinceOpen = (int16_t)(clStart - windowOpenHR);
        if (sinceOpen >= 0)
        {
            matchMid  = (uint16_t)(((uint32_t)clStart + clEnd) / 2u);
            haveMatch = true;
        }
    }

    if (haveMatch && zcTimestampOut)
        *zcTimestampOut = matchMid;

    return haveMatch;

    #undef DMA_DETECT_GAP_HR
}

/* ──────────────────────────────────────────────────────────────────── */
/* Stub ISRs for CCP2 and CCP5.
 *
 * The DMA trigger on dsPIC33CK fires on the *interrupt source* being
 * active, which requires the peripheral IE bit to be enabled. But
 * enabling IE also causes the CPU to dispatch to the ISR vector on
 * each trigger. If the vector is undefined, the CPU traps to
 * _DefaultInterrupt and the motor halts.
 *
 * Under FEATURE_IC_DMA_SHADOW, FEATURE_IC_ZC_CAPTURE is forced to 0
 * (see garuda_config.h), which compiles out the production
 * _CCP2Interrupt in garuda_service.c. We provide minimal stubs here
 * that just clear the flag and return. DMA has already drained the
 * FIFO via the auto-triggered transfer. These ISRs run at IPL 1
 * (very low priority) and do no work.
 */
void __attribute__((interrupt, no_auto_psv)) _CCP2Interrupt(void)
{
    _CCP2IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _CCP5Interrupt(void)
{
    _CCP5IF = 0;
}

#endif /* FEATURE_IC_ZC && FEATURE_IC_DMA_SHADOW */
