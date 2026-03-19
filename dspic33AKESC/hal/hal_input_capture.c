/**
 * @file hal_input_capture.c
 *
 * @brief SCCP4 Input Capture HAL + ISRs for RX input (PWM / DShot).
 *
 * RD8/RP57 -> ICM4R -> SCCP4 IC (every rise/fall, 32-bit @ 100 MHz).
 *
 * PWM mode:  _CCP4Interrupt captures rise/fall pairs -> seqlock mailbox.
 * DShot mode: DMA ping-pong captures 32 edges -> _DMA0Interrupt decodes.
 * Detection:  ISR captures edges, RX_Service() classifies protocol.
 *
 * Component: HAL
 */

#include "garuda_config.h"

#if (FEATURE_RX_PWM || FEATURE_RX_DSHOT || FEATURE_RX_AUTO)

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "hal_input_capture.h"
#include "garuda_calc_params.h"

/* ── Forward declarations ──────────────────────────────────────────── */
/* Mailbox is defined in rx_decode.c; ISR writes via extern */
#include "../input/rx_decode.h"

/* ── Seqlock mailbox (ISR -> RX_Service) ──────────────────────────── */

volatile RX_MAILBOX_T rxMailbox;

/* ── Detection-phase edge buffer ──────────────────────────────────── */

#define DETECT_EDGE_BUF_SIZE  16
static volatile uint32_t detectEdgeBuf[DETECT_EDGE_BUF_SIZE];
static volatile uint8_t  detectEdgeHead;
static volatile uint8_t  detectEdgeCount;

/* ── PWM capture state (ISR-only) ─────────────────────────────────── */

static uint32_t pwmRiseStamp;
static uint32_t pwmLastRiseStamp;
static bool     pwmGotRise;

/* ── DShot DMA state ──────────────────────────────────────────────── */

#if FEATURE_RX_DSHOT
/* Ping-pong DMA buffers (64 edges each) */
static volatile uint32_t dmaEdgeBuf[2][64] __attribute__((aligned(4)));
static uint8_t  dmaActiveBuf;  /* 0 or 1: which buffer DMA is writing */

/* 64-edge overlap ring for rolling CRC alignment */
volatile uint32_t dshotRingBuf[64];
volatile uint8_t  dshotRingHead;     /* next write position (0-63) */

/* Alignment state */
volatile uint8_t  dshotAlignOffset;  /* current bit offset (0-15) */
volatile bool     dshotAligned;      /* true = CRC-locked offset found */
volatile uint8_t  dshotPendingShift; /* flag: CRC failed, need shift search */
volatile uint8_t  dshotConsecCrcFail;/* consecutive CRC failures */

/* Frame count for C.0 test */
#if C0_DMA_TEST
static volatile uint32_t dmaFrameCount;
static volatile uint32_t c0FinalCount;
static volatile uint8_t  c0Done;
static volatile uint8_t  c0Started;
static volatile uint8_t  c0Error;

/* Saved state for C.0 ISR isolation */
static uint32_t saved_iec0, saved_iec1, saved_iec2, saved_iec3;
static uint8_t  saved_dma0ip;

/* IFS cleanup — method selected by IFS_IS_W1C (Milestone 0) */
static inline void c0ClearIfs(uint32_t m0, uint32_t m1,
                               uint32_t m2, uint32_t m3)
{
#if IFS_IS_W1C
    IFS0 = m0; IFS1 = m1; IFS2 = m2; IFS3 = m3;
#else
    IFS0 &= ~m0; IFS1 &= ~m1; IFS2 &= ~m2; IFS3 &= ~m3;
#endif
}

static void c0Cleanup(void)
{
    LED2 = 0;
    DMA0CHbits.CHEN = 0;
    _DMA0IE = 0;
    c0ClearIfs(saved_iec0, saved_iec1, saved_iec2, saved_iec3);
    IEC0 = saved_iec0; IEC1 = saved_iec1;
    IEC2 = saved_iec2; IEC3 = saved_iec3;
    _DMA0IP = saved_dma0ip;
}
#endif /* C0_DMA_TEST */
#endif /* FEATURE_RX_DSHOT */

/* ── RX mode tracking ─────────────────────────────────────────────── */

typedef enum {
    IC_MODE_DETECT = 0,  /* ISR captures edges for protocol detection */
    IC_MODE_PWM,         /* ISR does rise/fall pairing for PWM */
    IC_MODE_DSHOT_DMA    /* DMA handles captures for DShot */
} IC_MODE_T;

static IC_MODE_T icMode = IC_MODE_DETECT;

/* ── DShot decode helpers ─────────────────────────────────────────── */

#if FEATURE_RX_DSHOT
/**
 * @brief Decode a 16-bit DShot frame from edge timestamps.
 * @param edges     Array of edge timestamps (alternating rise/fall).
 * @param edgeCount Number of edges in the array.
 * @param[out] throttle Decoded throttle value (0-2047).
 * @param[out] telemetry Telemetry request bit.
 * @return true if CRC passes.
 */
bool DshotDecodeFrame(const volatile uint32_t *edges, uint8_t edgeCount,
                      uint16_t *throttle, uint8_t *telemetry)
{
    /* --- Find frame start by locating the inter-frame gap --- */
    uint8_t frameStart = 0;
    bool    gapFound   = false;

    for (uint8_t i = 0; i < edgeCount; i++)
    {
        uint8_t  fallIdx     = (i * 2 + 1) % edgeCount;
        uint8_t  nextRiseIdx = (fallIdx + 1) % edgeCount;
        uint32_t gap         = edges[nextRiseIdx] - edges[fallIdx];

        if (gap > 2000)   /* >20us at 100MHz = inter-frame gap */
        {
            frameStart = nextRiseIdx;
            gapFound   = true;
            break;
        }
    }

    if (!gapFound)
        return false;   /* no frame boundary found — buffer not ready */

    /* --- Detect bit period from first two rises, compute threshold --- */
    uint8_t  rise0Idx = frameStart;
    uint8_t  rise1Idx = (frameStart + 2) % edgeCount;  /* next rise = +2 edges */
    uint32_t period   = edges[rise1Idx] - edges[rise0Idx];

    if (period == 0)
        return false;

    /* Threshold = 56.25% of bit period (midpoint between bit=0 at 37.5% and bit=1 at 75%) */
    uint32_t threshold = (period * 9) / 16;

    /* --- Decode 16 bits --- */
    uint16_t raw = 0;

    for (uint8_t bit = 0; bit < 16; bit++)
    {
        uint8_t riseIdx = (frameStart + bit * 2) % edgeCount;
        uint8_t fallIdx = (riseIdx + 1)           % edgeCount;

        uint32_t rise     = edges[riseIdx];
        uint32_t fall     = edges[fallIdx];
        uint32_t highTime = fall - rise;

        raw <<= 1;
        if (highTime >= threshold)
            raw |= 1;
    }

    /* --- CRC check --- */
    uint16_t data12  = raw >> 4;
    uint8_t crc_calc = (uint8_t)((data12 ^ (data12 >> 4) ^ (data12 >> 8)) & 0x0F);
    uint8_t crc_recv = (uint8_t)(raw & 0x0F);

    if (crc_calc != crc_recv)
        return false;

    *throttle  = (raw >> 5) & 0x07FF;
    *telemetry = (raw >> 4) & 0x01;
    return true;
}
#endif /* FEATURE_RX_DSHOT */

/* ── Seqlock mailbox write (ISR context) ──────────────────────────── */

static void MailboxWrite(uint16_t value, uint8_t valid)
{
    rxMailbox.seqNum++;          /* odd = writing */
    rxMailbox.value = value;
    rxMailbox.valid = valid;
    rxMailbox.seqNum++;          /* even = complete */
}

/* ── IC4 Initialization ───────────────────────────────────────────── */

void HAL_IC4_Init(void)
{
    /* PPS: map RP57 (RD8) to IC4 input */
    _ICM4R = 57;

    /* Configure SCCP4 for Input Capture:
     * CCSEL=1: IC mode
     * T32=1: 32-bit timer
     * MOD=0b0011: capture every rise and fall edge
     * CLKSEL=0b000: SCCP peripheral bus (100 MHz) */
    CCP4CON1 = 0;
    CCP4CON1bits.CCSEL = 1;     /* Input capture mode */
    CCP4CON1bits.T32 = 1;       /* 32-bit timer */
    CCP4CON1bits.MOD = 0b0011;  /* Every edge (rise + fall) */
    CCP4CON1bits.CLKSEL = 0b000; /* SCCP bus clock = 100 MHz */
    CCP4STATbits.ICOV = 0;

    /* ISR priority 4 (below ADC prio 6, below commutation prio 7) */
    _CCP4IP = 4;

    /* Clear state */
    rxMailbox.seqNum = 0;
    rxMailbox.value = 0;
    rxMailbox.valid = 0;

    detectEdgeHead = 0;
    detectEdgeCount = 0;

    pwmGotRise = false;
    pwmRiseStamp = 0;
    pwmLastRiseStamp = 0;

    icMode = IC_MODE_DETECT;
}

void HAL_IC4_Enable(void)
{
    _CCP4IF = 0;
    _CCP4IE = 1;
    CCP4CON1bits.ON = 1;
}

void HAL_IC4_Disable(void)
{
    CCP4CON1bits.ON = 0;
    _CCP4IE = 0;
    _CCP4IF = 0;
}

/* ── Switch IC4 mode ──────────────────────────────────────────────── */

void HAL_IC4_SetModePwm(void)
{
    icMode = IC_MODE_PWM;
    pwmGotRise = false;
}

void HAL_IC4_SetModeDetect(void)
{
    icMode = IC_MODE_DETECT;
    detectEdgeHead = 0;
    detectEdgeCount = 0;
}

#if FEATURE_RX_DSHOT
void HAL_IC4_ConfigDmaDshot(void)
{
    /* Disable IC ISR — DMA takes over capture */
    _CCP4IE = 0;

    /* Initialize DMA state */
    dmaActiveBuf = 0;
    dshotRingHead = 0;
    dshotAlignOffset = 0;
    dshotAligned = false;
    dshotPendingShift = 0;
    dshotConsecCrcFail = 0;

/* Configure DMA channel 0:
     * Source: CCP4BUF (IC4 capture buffer)
     * Dest: dmaEdgeBuf[0] (first ping-pong buffer)
     * Count: RX_DSHOT_DMA_COUNT transfers per block */
    DMACONbits.ON = 1;
    DMA0CHbits.CHEN = 0;
    DMA0CHbits.SIZE = 0b10;
    DMA0CHbits.SAMODE = 0b00;    /* 32-bit transfers (CCP4BUF is 32-bit) */
    DMA0CHbits.DAMODE = 0b01;  /* Increment destination address */
    DMA0CHbits.TRMODE = 0b00;
    DMA0CHbits.DONEEN = 1;
    DMALOW= 0x0000UL;
    DMAHIGH= 0x00FFFFFFUL;
    DMA0SRC = (uint32_t)&CCP4BUF;  /* Source: IC4 capture register */
    DMA0DST = (uint32_t)&dmaEdgeBuf[0][0];  /* Dest: buffer A */
    DMA0CNT = RX_DSHOT_DMA_COUNT;  /* Transfers per block */

    /* DMA trigger: SCCP4 IC capture event
     * Table 13-2 (DS70005539): CCP4 IC/OC = CHSEL 0x1B */
    DMA0SELbits.CHSEL = 0x1B;

    /* Enable DMA transfer-complete interrupt */
    _DMA0IP = 1;    /* Low priority — below IC4/ADC/commutation */
    _DMA0IF = 0;
    _DMA0IE = 1;

    /* Enable DMA channel */
    DMA0CHbits.CHEN = 1;


    icMode = IC_MODE_DSHOT_DMA;
}

void HAL_IC4_DisableDma(void)
{
    DMA0CHbits.CHEN = 0;
    _DMA0IE = 0;
    _DMA0IF = 0;

    /* Re-enable IC4 ISR for detection */
    _CCP4IF = 0;
    _CCP4IE = 1;
    icMode = IC_MODE_DETECT;
}
#endif /* FEATURE_RX_DSHOT */

/* ── SCCP4 Input Capture ISR ──────────────────────────────────────── */

void __attribute__((__interrupt__, no_auto_psv)) _CCP4Interrupt(void)
{
    uint32_t capture = CCP4BUF;  /* MUST read to clear data-ready */

    _CCP4IF = 0;

    switch (icMode)
    {
        case IC_MODE_DETECT:
        {
            /* Buffer edges for protocol classification by RX_Service() */
            detectEdgeBuf[detectEdgeHead] = capture;
            detectEdgeHead = (detectEdgeHead + 1) & (DETECT_EDGE_BUF_SIZE - 1);
            if (detectEdgeCount < DETECT_EDGE_BUF_SIZE)
                detectEdgeCount++;
            break;
        }

        case IC_MODE_PWM:
        {
            /* Rise/fall edge pairing for PWM decode.
             * We use the pin state to determine polarity.
             * RD8 high = rising edge just happened. */
            bool pinHigh = PORTDbits.RD8;

            if (pinHigh)
            {
                /* Rising edge */
                pwmLastRiseStamp = pwmRiseStamp;
                pwmRiseStamp = capture;
                pwmGotRise = true;
            }
            else if (pwmGotRise)
            {
                /* Falling edge — compute pulse width */
                uint32_t width = capture - pwmRiseStamp;
                uint32_t widthUs = width / RX_COUNTS_PER_US;

                /* Validate pulse width */
                if (widthUs >= RX_PWM_MIN_US && widthUs <= RX_PWM_MAX_US)
                {
                    /* Valid PWM pulse — write to seqlock mailbox */
                    MailboxWrite((uint16_t)widthUs, 1);
                }
                else
                {
                    MailboxWrite((uint16_t)widthUs, 0);
                }
                pwmGotRise = false;
            }
            break;
        }

        case IC_MODE_DSHOT_DMA:
            /* Should not fire — ISR disabled in DMA mode.
             * If it does, just drain the buffer. */
            break;
    }

}

/* ── DMA Transfer-Complete ISR (DShot mode) ───────────────────────── */

#if FEATURE_RX_DSHOT
void __attribute__((__interrupt__, no_auto_psv)) _DMA0Interrupt(void)
{
#if C0_DMA_TEST
    /* C.0 gate test mode: count frames, frame-aligned GPIO markers */
    dmaFrameCount++;

    if (!c0Started && dmaFrameCount == 1)
    {
        /* First DMA-TC = alignment frame. Open GPIO window, reset count. */
        LED2 = 1;           /* GPIO sync HIGH — LA window start */
        dmaFrameCount = 0;  /* This frame is not counted */
        c0Started = 1;
    }
    else if (c0Started && dmaFrameCount >= C0_TARGET_FRAMES)
    {
        /* Target reached — close window at DMA-TC boundary */
        c0FinalCount = dmaFrameCount;  /* latch FIRST */
        LED2 = 0;                       /* GPIO sync LOW — LA window end */
        DMA0CHbits.CHEN = 0;
        _DMA0IE = 0;                   /* self-disable */
        c0Done = 1;
    }

    /* Swap ping-pong buffer for next transfer */
    dmaActiveBuf ^= 1;
    DMA0DST = (uint32_t)&dmaEdgeBuf[dmaActiveBuf][0];
    DMA0CNT = RX_DSHOT_DMA_COUNT;
    /* Re-arm (CHEN still enabled unless c0Done) */

#else
    if (CCP4STATbits.ICOV)
    {
          CCP4STATbits.ICOV = 0;  // clear overflow
    }

    /* Copy completed buffer into 64-edge ring */
    uint8_t completedBuf = dmaActiveBuf;  /* just-completed buffer */
    uint8_t head = dshotRingHead;
    for (uint8_t i = 0; i < RX_DSHOT_EDGES; i++)
    {
        dshotRingBuf[head] = dmaEdgeBuf[completedBuf][i];
        head = (head + 1) & 63;
    }
    dshotRingHead = head;

   dmaActiveBuf ^= 1;
    DMA0SRC = (uint32_t)&CCP4BUF;  /* Source: IC4 capture register */
    DMA0DST = (uint32_t)&dmaEdgeBuf[dmaActiveBuf][0];
    DMA0CNT = RX_DSHOT_DMA_COUNT;
    DMA0CHbits.CHEN = 1;

    /* O(1) decode attempt at current offset */
    if (dshotAligned)
    {
        uint16_t throttle;
        uint8_t telemetry;
        if (DshotDecodeFrame(dshotRingBuf, RX_DSHOT_EDGES,
                             &throttle, &telemetry))
        {
            dshotConsecCrcFail = 0;
            MailboxWrite(throttle, 1);
        }
        else
        {
            dshotConsecCrcFail++;
            if (dshotConsecCrcFail >= 5)
            {
                /* Lost alignment — flag for RX_Service() shift search */
                dshotAligned = false;
                dshotPendingShift = 1;
            }
            MailboxWrite(0, 0);
        }
    }
    else
    {
        /* Not aligned — flag for main-loop search */
        dshotPendingShift = 1;
        MailboxWrite(0, 0);
    }
#endif /* C0_DMA_TEST */

    _DMA0IF = 0;
}
#endif /* FEATURE_RX_DSHOT */

/* ── Detection edge buffer accessors (for RX_Service) ─────────────── */

uint8_t HAL_IC4_GetDetectEdgeCount(void)
{
    return detectEdgeCount;
}

void HAL_IC4_GetDetectEdges(uint32_t *buf, uint8_t *count)
{
    _CCP4IE = 0;  /* brief ISR disable for consistent copy */
    uint8_t n = detectEdgeCount;
    uint8_t start = (detectEdgeHead - n) & (DETECT_EDGE_BUF_SIZE - 1);
    for (uint8_t i = 0; i < n; i++)
    {
        buf[i] = detectEdgeBuf[(start + i) & (DETECT_EDGE_BUF_SIZE - 1)];
    }
    *count = n;
    detectEdgeCount = 0;
    detectEdgeHead = 0;
    _CCP4IE = 1;
}

#endif /* FEATURE_RX_PWM || FEATURE_RX_DSHOT || FEATURE_RX_AUTO */
