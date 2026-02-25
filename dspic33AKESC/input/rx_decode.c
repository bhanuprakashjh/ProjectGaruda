/**
 * @file rx_decode.c
 *
 * @brief RX input decode: mailbox consumer, PWM/DShot decode, lock FSM.
 *
 * RX_Service() runs in main.c while(1) — NOT an ISR.
 *
 * Write ordering for ADC ISR:
 *   RX_Service writes: rxCachedThrottleAdc FIRST, rxCachedLocked SECOND.
 *   ADC ISR reads:     rxCachedLocked FIRST, rxCachedThrottleAdc SECOND.
 *   This ensures ADC ISR never reads a stale-high throttle with locked=true.
 *
 * Component: INPUT
 */

#include "garuda_config.h"

#if (FEATURE_RX_PWM || FEATURE_RX_DSHOT || FEATURE_RX_AUTO)

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "rx_decode.h"
#include "garuda_types.h"
#include "garuda_service.h"
#include "garuda_calc_params.h"
#include "hal/hal_input_capture.h"

/* ── Extern: hal_input_capture.c DShot state ─────────────────────── */
#if FEATURE_RX_DSHOT
extern volatile uint32_t dshotRingBuf[64];
extern volatile uint8_t  dshotAlignOffset;
extern volatile bool     dshotAligned;
extern volatile uint8_t  dshotPendingShift;
extern volatile uint8_t  dshotConsecCrcFail;

static uint8_t dshotTotalShifts;  /* total shift attempts across calls */
#endif

/* ── Cached throttle (ADC ISR reads these) ────────────────────────── */

volatile uint16_t rxCachedThrottleAdc = 0;
volatile uint8_t  rxCachedLocked = 0;

/* ── Lock FSM state ───────────────────────────────────────────────── */

static uint16_t lastSeqNum;
static uint8_t  lockCount;       /* consecutive valid frames */
static uint32_t lastValidTick;   /* systemTick of last valid frame */

/* ── Detection phase ──────────────────────────────────────────────── */

/**
 * @brief Classify protocol from captured edges.
 * Examines min edge-to-edge delta:
 *   < 500 counts (< 5us @ 100 MHz) -> DShot candidate
 *   > 50000 counts (> 500us) -> PWM candidate
 */
static RX_PROTOCOL_T DetectProtocol(void)
{
    uint32_t edges[16];
    uint8_t count;
    HAL_IC4_GetDetectEdges(edges, &count);

    if (count < 4)
        return RX_PROTO_NONE;

    /* Find minimum edge-to-edge delta */
    uint32_t minDelta = UINT32_MAX;
    for (uint8_t i = 1; i < count; i++)
    {
        uint32_t delta = edges[i] - edges[i - 1];
        if (delta < minDelta)
            minDelta = delta;
    }

    /* DShot: fast edges, min delta < 5us */
    if (minDelta < 500)
    {
        /* Estimate rate from bit period */
        if (minDelta < 200)
            return RX_PROTO_DSHOT600;  /* ~167 counts */
        else if (minDelta < 400)
            return RX_PROTO_DSHOT300;  /* ~333 counts */
        else
            return RX_PROTO_DSHOT150;  /* ~667 counts */
    }

    /* PWM: slow edges, min delta > 500us */
    if (minDelta > 50000)
        return RX_PROTO_PWM;

    return RX_PROTO_NONE;
}

/* ── PWM throttle conversion ──────────────────────────────────────── */

static uint16_t PwmUsToThrottleAdc(uint16_t pulseUs)
{
    /* Deadband: 1000 +/- 25us = zero throttle */
    if (pulseUs <= (1000 + RX_PWM_DEADBAND_US))
        return 0;

    /* Map 1000-2000us -> 0-2000, then scale to 0-4095 */
    uint16_t throttle2000;
    if (pulseUs >= 2000)
        throttle2000 = 2000;
    else
        throttle2000 = pulseUs - 1000;

    /* Scale 0-2000 -> 0-4095 */
    return (uint16_t)((uint32_t)throttle2000 * 4095 / 2000);
}

/* ── DShot throttle conversion ────────────────────────────────────── */

static uint16_t DshotToThrottleAdc(uint16_t dshotVal)
{
    /* DShot 0 = disarm, 1-47 = commands (ignored), 48-2047 = throttle */
    if (dshotVal <= RX_DSHOT_CMD_MAX)
        return 0;

    /* Map 48-2047 -> 0-2000, then scale to 0-4095 */
    uint16_t throttle2000 = dshotVal - 48;
    if (throttle2000 > 2000)
        throttle2000 = 2000;

    return (uint16_t)((uint32_t)throttle2000 * 4095 / 2000);
}

/* ── RX_Init ──────────────────────────────────────────────────────── */

void RX_Init(void)
{
    rxCachedThrottleAdc = 0;
    rxCachedLocked = 0;

    lastSeqNum = 0;
    lockCount = 0;
    lastValidTick = 0;

    rxMailbox.seqNum = 0;
    rxMailbox.value = 0;
    rxMailbox.valid = 0;

    garudaData.rxLinkState = RX_LINK_UNLOCKED;
    garudaData.rxProtocol = RX_PROTO_NONE;
    garudaData.rxCrcErrors = 0;
    garudaData.rxPulseUs = 0;
    garudaData.rxDshotRate = 0;
    garudaData.rxDroppedFrames = 0;

    HAL_IC4_Init();
    HAL_IC4_Enable();
}

/* ── RX_Service (main loop) ───────────────────────────────────────── */

void RX_Service(void)
{
    /* Only process if we're using an RX source */
    THROTTLE_SOURCE_T src = garudaData.throttleSource;
    if (src != THROTTLE_SRC_PWM && src != THROTTLE_SRC_DSHOT
        && src != THROTTLE_SRC_AUTO)
        return;

    /* ── Link state machine ───────────────────────────────────────── */

    switch (garudaData.rxLinkState)
    {
        case RX_LINK_UNLOCKED:
        case RX_LINK_DETECTING:
        {
            garudaData.rxLinkState = RX_LINK_DETECTING;

            /* Try to detect protocol from captured edges */
            if (HAL_IC4_GetDetectEdgeCount() >= 4)
            {
                RX_PROTOCOL_T proto = DetectProtocol();

                if (proto != RX_PROTO_NONE)
                {
                    garudaData.rxProtocol = proto;

                    if (proto == RX_PROTO_PWM)
                    {
#if FEATURE_RX_PWM
                        HAL_IC4_SetModePwm();
                        garudaData.rxLinkState = RX_LINK_LOCKING;
                        lockCount = 0;
                        lastValidTick = garudaData.systemTick;
#endif
                    }
                    else
                    {
#if FEATURE_RX_DSHOT
                        /* DShot detected — arm DMA */
                        HAL_IC4_ConfigDmaDshot();
                        dshotTotalShifts = 0;
                        garudaData.rxLinkState = RX_LINK_LOCKING;
                        lockCount = 0;
                        lastValidTick = garudaData.systemTick;

                        /* Set rate for telemetry */
                        if (proto == RX_PROTO_DSHOT600)
                            garudaData.rxDshotRate = 6;  /* 600 */
                        else if (proto == RX_PROTO_DSHOT300)
                            garudaData.rxDshotRate = 3;  /* 300 */
                        else
                            garudaData.rxDshotRate = 1;  /* 150 */
#endif
                    }
                }
            }
            break;
        }

        case RX_LINK_LOCKING:
        case RX_LINK_LOCKED:
        {
            /* Read seqlock mailbox */
            uint16_t seq1, seq2;
            uint16_t value;
            uint8_t  valid;

            seq1 = rxMailbox.seqNum;
            value = rxMailbox.value;
            valid = rxMailbox.valid;
            seq2 = rxMailbox.seqNum;

            if (seq1 != seq2 || (seq1 & 1) != 0)
            {
                /* Torn read — skip this cycle */
                break;
            }

            /* Detect dropped frames */
            if (lastSeqNum != 0)
            {
                uint16_t gap = (seq2 - lastSeqNum) / 2;
                if (gap > 1)
                    garudaData.rxDroppedFrames += (gap - 1);
            }
            lastSeqNum = seq2;

            if (valid)
            {
                lastValidTick = garudaData.systemTick;

                if (garudaData.rxLinkState == RX_LINK_LOCKING)
                {
                    lockCount++;
                    if (lockCount >= RX_LOCK_COUNT)
                    {
                        garudaData.rxLinkState = RX_LINK_LOCKED;
                    }
                }

                /* Decode and cache throttle */
                uint16_t throttleAdc;
                if (garudaData.rxProtocol == RX_PROTO_PWM)
                {
                    garudaData.rxPulseUs = value;
                    throttleAdc = PwmUsToThrottleAdc(value);
                }
                else
                {
                    throttleAdc = DshotToThrottleAdc(value);
                }

                /* Write ordering: value FIRST, lock flag SECOND */
                rxCachedThrottleAdc = throttleAdc;
                if (garudaData.rxLinkState == RX_LINK_LOCKED)
                    rxCachedLocked = 1;
            }
            else
            {
                /* Invalid frame */
                if (garudaData.rxProtocol != RX_PROTO_PWM)
                    garudaData.rxCrcErrors++;

                if (garudaData.rxLinkState == RX_LINK_LOCKING)
                    lockCount = 0;  /* reset lock count on invalid */
            }

#if FEATURE_RX_DSHOT
            /* DShot alignment search (deferred from ISR) */
            if (dshotPendingShift && !dshotAligned)
            {
                uint8_t shifts = 0;
                uint8_t offset = dshotAlignOffset;

                while (shifts < RX_ALIGN_MAX_SHIFTS_PER_CALL)
                {
                    offset = (offset + 1) & 0x0F;  /* 0-15 */
                    shifts++;

                    /* Try decode at this offset */
                    uint16_t thr;
                    uint8_t telem;
                    if (DshotDecodeFrame(dshotRingBuf, offset,
                                         &thr, &telem))
                    {
                        /* Found valid alignment */
                        dshotAlignOffset = offset;
                        dshotAligned = true;
                        dshotConsecCrcFail = 0;
                        dshotPendingShift = 0;
                        dshotTotalShifts = 0;
                        break;
                    }
                }

                if (!dshotAligned)
                {
                    dshotAlignOffset = offset;
                    dshotTotalShifts += shifts;

                    if (dshotTotalShifts >= 16)
                    {
                        /* All 16 bit offsets exhausted — re-detect */
                        HAL_IC4_DisableDma();
                        garudaData.rxLinkState = RX_LINK_DETECTING;
                        garudaData.rxProtocol = RX_PROTO_NONE;
                        lockCount = 0;
                        rxCachedLocked = 0;
                        rxCachedThrottleAdc = 0;
                        dshotPendingShift = 0;
                        dshotTotalShifts = 0;
                    }
                }
            }
#endif

            /* Timeout check */
            {
                uint32_t elapsed = garudaData.systemTick - lastValidTick;
                if (elapsed > RX_TIMEOUT_MS)
                {
                    /* Signal lost */
                    garudaData.rxLinkState = RX_LINK_LOST;

                    /* Write ordering: lock=0 FIRST, throttle=0 SECOND */
                    rxCachedLocked = 0;
                    rxCachedThrottleAdc = 0;

                    lockCount = 0;

                    /* Latch FAULT_RX_LOSS if motor was running */
                    if (garudaData.state > ESC_IDLE
                        && garudaData.state < ESC_FAULT)
                    {
                        garudaData.faultCode = FAULT_RX_LOSS;
                        garudaData.state = ESC_FAULT;
                        garudaData.runCommandActive = false;
                    }
                }
            }
            break;
        }

        case RX_LINK_LOST:
        {
            /* Stay in LOST until fault is cleared, then re-detect */
            if (garudaData.state == ESC_IDLE
                && garudaData.faultCode == FAULT_NONE)
            {
                /* Re-enter detection */
#if FEATURE_RX_DSHOT
                if (garudaData.rxProtocol != RX_PROTO_PWM)
                    HAL_IC4_DisableDma();
#endif
                HAL_IC4_SetModeDetect();
                garudaData.rxLinkState = RX_LINK_DETECTING;
                garudaData.rxProtocol = RX_PROTO_NONE;
                lockCount = 0;
                lastValidTick = garudaData.systemTick;
            }
            break;
        }
    }
}

#endif /* FEATURE_RX_PWM || FEATURE_RX_DSHOT || FEATURE_RX_AUTO */
