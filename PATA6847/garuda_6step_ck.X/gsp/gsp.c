/**
 * @file gsp.c
 * @brief GSP v2 protocol engine for dsPIC33CK — ring buffers, parser, CRC.
 *
 * Adapted from dsPIC33AK GSP for dsPIC33CK64MP205 UART1 registers.
 * Packet format: [0x02] [LEN] [CMD_ID] [PAYLOAD...] [CRC16_H] [CRC16_L]
 * All RX/TX is polled from main loop (no ISR-driven UART).
 */

#include "../garuda_config.h"

#if FEATURE_GSP

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "gsp.h"
#include "gsp_commands.h"

/* ── Tuning constants ────────────────────────────────────────────── */

#define GSP_START_BYTE        0x02
#define GSP_RX_RING_SIZE      256
#define GSP_TX_RING_SIZE      256
#define GSP_RX_RING_MASK      (GSP_RX_RING_SIZE - 1)
#define GSP_TX_RING_MASK      (GSP_TX_RING_SIZE - 1)
#define GSP_MAX_PAYLOAD_LEN   249
#define GSP_PARSER_TIMEOUT_MS 100
#define GSP_MAX_CMDS_PER_SVC  1

/* ── Ring buffer types ───────────────────────────────────────────── */

typedef struct {
    uint8_t  buf[GSP_RX_RING_SIZE];
    uint16_t head;
    uint16_t tail;
} RING_T;

/* ── Parser state machine ────────────────────────────────────────── */

typedef enum {
    PARSE_WAIT_START,
    PARSE_GOT_START,
    PARSE_COLLECTING
} PARSE_STATE_T;

typedef struct {
    PARSE_STATE_T state;
    uint8_t  pktBuf[1 + GSP_MAX_PAYLOAD_LEN];
    uint8_t  pktLen;
    uint8_t  pktIdx;
    uint8_t  crcBuf[2];
    uint8_t  crcIdx;
    bool     collectingCrc;
    uint32_t lastRxTick;
} PARSER_T;

/* ── Module state ────────────────────────────────────────────────── */

static RING_T  rxRing;
static RING_T  txRing;
static PARSER_T parser;

/* Access systemTick via gData (declared in garuda_service.h) */
#include "../garuda_service.h"

/* ── CRC-16-CCITT ────────────────────────────────────────────────── */

static uint16_t CRC16_Update(uint16_t crc, uint8_t byte)
{
    crc ^= (uint16_t)byte << 8;
    for (uint8_t i = 0; i < 8; i++) {
        if (crc & 0x8000)
            crc = (crc << 1) ^ 0x1021;
        else
            crc <<= 1;
    }
    return crc;
}

/* ── Ring buffer helpers ─────────────────────────────────────────── */

static inline uint16_t RingCount(const RING_T *r, uint16_t mask)
{
    return (r->head - r->tail) & mask;
}

static inline void RingPut(RING_T *r, uint16_t mask, uint8_t b)
{
    r->buf[r->head] = b;
    r->head = (r->head + 1) & mask;
}

static inline uint8_t RingGet(RING_T *r, uint16_t mask)
{
    uint8_t b = r->buf[r->tail];
    r->tail = (r->tail + 1) & mask;
    return b;
}

/* ── dsPIC33CK UART1 register access ────────────────────────────── */

static inline bool UART1_RxReady(void)
{
    return !U1STAHbits.URXBE;  /* RX buffer not empty */
}

static inline uint8_t UART1_RxRead(void)
{
    return (uint8_t)U1RXREG;
}

static inline bool UART1_TxFull(void)
{
    return U1STAHbits.UTXBF != 0;
}

static inline void UART1_TxWrite(uint8_t b)
{
    U1TXREG = b;
}

static inline bool UART1_RxOverflow(void)
{
    return U1STAbits.OERR != 0;
}

static inline void UART1_RxOverflowClear(void)
{
    U1STAbits.OERR = 0;
}

/* ── RX pump: HW FIFO → software ring ───────────────────────────── */

static void PumpRx(void)
{
    /* On dsPIC33CK, OERR halts the receiver. Must clear OERR and drain
     * any stale bytes before normal operation resumes. Without the drain,
     * URXBE can stay deasserted (not empty) indefinitely → infinite loop. */
    if (UART1_RxOverflow()) {
        UART1_RxOverflowClear();
        /* Drain stale bytes (max 4 in HW FIFO) */
        for (uint8_t i = 0; i < 8 && UART1_RxReady(); i++)
            (void)UART1_RxRead();
        return;  /* Skip this cycle, start fresh next call */
    }

    /* Normal RX: drain HW FIFO into software ring, bounded to avoid
     * hanging if URXBE is stuck (defensive, shouldn't happen). */
    uint8_t maxBytes = 16;  /* HW FIFO is 4-8 deep, 16 is generous */
    while (UART1_RxReady() && maxBytes-- > 0) {
        uint8_t b = UART1_RxRead();
        if (RingCount(&rxRing, GSP_RX_RING_MASK) < GSP_RX_RING_MASK)
            RingPut(&rxRing, GSP_RX_RING_MASK, b);
    }
}

/* ── TX pump: software ring → HW TX register ─────────────────────── */

static void PumpTx(void)
{
    while (RingCount(&txRing, GSP_TX_RING_MASK) > 0 && !UART1_TxFull())
        UART1_TxWrite(RingGet(&txRing, GSP_TX_RING_MASK));
}

/* ── Parser ──────────────────────────────────────────────────────── */

static void ParserReset(void)
{
    parser.state = PARSE_WAIT_START;
    parser.pktIdx = 0;
    parser.pktLen = 0;
    parser.crcIdx = 0;
    parser.collectingCrc = false;
}

static void ParserProcess(void)
{
    uint8_t cmdsProcessed = 0;

    if (parser.state != PARSE_WAIT_START) {
        uint32_t elapsed = gData.systemTick - parser.lastRxTick;
        if (elapsed > GSP_PARSER_TIMEOUT_MS)
            ParserReset();
    }

    while (RingCount(&rxRing, GSP_RX_RING_MASK) > 0 &&
           cmdsProcessed < GSP_MAX_CMDS_PER_SVC) {
        uint8_t b = RingGet(&rxRing, GSP_RX_RING_MASK);
        parser.lastRxTick = gData.systemTick;

        switch (parser.state) {
        case PARSE_WAIT_START:
            if (b == GSP_START_BYTE)
                parser.state = PARSE_GOT_START;
            break;

        case PARSE_GOT_START:
            if (b >= 1 && b <= GSP_MAX_PAYLOAD_LEN) {
                parser.pktLen = b;
                parser.pktIdx = 0;
                parser.crcIdx = 0;
                parser.collectingCrc = false;
                parser.state = PARSE_COLLECTING;
            } else {
                ParserReset();
            }
            break;

        case PARSE_COLLECTING:
            if (!parser.collectingCrc) {
                parser.pktBuf[parser.pktIdx++] = b;
                if (parser.pktIdx >= parser.pktLen) {
                    parser.collectingCrc = true;
                    parser.crcIdx = 0;
                }
            } else {
                parser.crcBuf[parser.crcIdx++] = b;
                if (parser.crcIdx >= 2) {
                    uint16_t crc = CRC16_Update(0xFFFF, parser.pktLen);
                    for (uint8_t i = 0; i < parser.pktLen; i++)
                        crc = CRC16_Update(crc, parser.pktBuf[i]);

                    uint16_t rxCrc = ((uint16_t)parser.crcBuf[0] << 8)
                                   | parser.crcBuf[1];

                    if (crc == rxCrc) {
                        uint8_t cmdId = parser.pktBuf[0];
                        uint8_t payloadLen = parser.pktLen - 1;
                        const uint8_t *payload = (payloadLen > 0)
                            ? &parser.pktBuf[1] : 0;
                        GSP_DispatchCommand(cmdId, payload, payloadLen);
                        cmdsProcessed++;
                    }
                    ParserReset();
                }
            }
            break;
        }
    }
}

/* ── Public: Send response ───────────────────────────────────────── */

bool GSP_SendResponse(uint8_t cmdId, const uint8_t *payload, uint8_t len)
{
    if (len > GSP_MAX_PAYLOAD_LEN)
        return false;
    if (len > 0 && payload == 0)
        return false;

    uint16_t frameLen = 1 + 1 + 1 + len + 2;
    uint16_t txFree = GSP_TX_RING_MASK - RingCount(&txRing, GSP_TX_RING_MASK);
    if (txFree < frameLen)
        return false;

    uint8_t pktLen = 1 + len;

    uint16_t crc = CRC16_Update(0xFFFF, pktLen);
    crc = CRC16_Update(crc, cmdId);
    for (uint8_t i = 0; i < len; i++)
        crc = CRC16_Update(crc, payload[i]);

    RingPut(&txRing, GSP_TX_RING_MASK, GSP_START_BYTE);
    RingPut(&txRing, GSP_TX_RING_MASK, pktLen);
    RingPut(&txRing, GSP_TX_RING_MASK, cmdId);
    for (uint8_t i = 0; i < len; i++)
        RingPut(&txRing, GSP_TX_RING_MASK, payload[i]);
    RingPut(&txRing, GSP_TX_RING_MASK, (uint8_t)(crc >> 8));
    RingPut(&txRing, GSP_TX_RING_MASK, (uint8_t)(crc & 0xFF));

    return true;
}

/* ── Public: Init ────────────────────────────────────────────────── */

void GSP_Init(void)
{
    memset(&rxRing, 0, sizeof(rxRing));
    memset(&txRing, 0, sizeof(txRing));
    ParserReset();
    parser.lastRxTick = 0;

    /* UART1 is already initialized by HAL_UART_Init().
     * GSP takes over RX/TX — debug prints are disabled when FEATURE_GSP=1. */
}

/* ── Public: Service ─────────────────────────────────────────────── */

void GSP_Service(void)
{
    PumpRx();
    ParserProcess();
    GSP_TelemTick();
    PumpTx();
}

#endif /* FEATURE_GSP */
