/**
 * @file gsp.c
 *
 * @brief GSP v1 protocol engine — ring buffers, parser, CRC, UART init.
 *
 * Packet format: [0x02] [LEN] [CMD_ID] [PAYLOAD...] [CRC16_H] [CRC16_L]
 * - LEN = byte count of CMD_ID + PAYLOAD (1-253)
 * - CRC-16-CCITT (poly 0x1021, init 0xFFFF) over [LEN][CMD_ID][PAYLOAD]
 * - CRC-fail: silent drop (host retries on timeout)
 *
 * All RX/TX is polled from main loop (no ISR-driven UART).
 * At 115.2kbps, 1 byte every 86.8us; UART1 HW FIFO (8-deep) buffers
 * between main loop iterations.
 *
 * Component: GSP
 */

#include "garuda_config.h"

#if FEATURE_GSP

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "gsp.h"
#include "gsp_commands.h"
#include "garuda_service.h"
#include "hal/uart1.h"

/* ── Tuning constants ────────────────────────────────────────────────── */

#define GSP_START_BYTE        0x02
#define GSP_RX_RING_SIZE      256   /* Must be power of 2 */
#define GSP_TX_RING_SIZE      256   /* Must be power of 2 */
#define GSP_RX_RING_MASK      (GSP_RX_RING_SIZE - 1)
#define GSP_TX_RING_MASK      (GSP_TX_RING_SIZE - 1)
#define GSP_MAX_PAYLOAD_LEN   249   /* Max response payload that fits TX ring.
                                       * Frame overhead = 5 (START+LEN+CMD+CRC16).
                                       * TX ring usable = GSP_TX_RING_SIZE-1 = 255.
                                       * 255 - 5 - 1(CMD in LEN) = 249. */
#define GSP_PARSER_TIMEOUT_MS 100   /* Reset parser after 100ms idle */
#define GSP_MAX_CMDS_PER_SVC  1     /* Commands processed per GSP_Service() */
#define GSP_BAUDRATE_DIVIDER  54    /* 100M/(16*55) = 115.7kbps */

_Static_assert((GSP_RX_RING_SIZE & GSP_RX_RING_MASK) == 0,
               "GSP_RX_RING_SIZE must be power of 2");
_Static_assert((GSP_TX_RING_SIZE & GSP_TX_RING_MASK) == 0,
               "GSP_TX_RING_SIZE must be power of 2");
/* Max frame = START(1) + LEN(1) + CMD(1) + payload + CRC(2) = 5 + payload.
 * Must fit in TX ring usable capacity (size - 1). */
_Static_assert(GSP_MAX_PAYLOAD_LEN + 5 <= GSP_TX_RING_MASK,
               "Max GSP response frame exceeds TX ring capacity");

/* ── Ring buffer types ───────────────────────────────────────────────── */

typedef struct {
    uint8_t  buf[GSP_RX_RING_SIZE];
    uint16_t head;  /* Write index (producer) */
    uint16_t tail;  /* Read index (consumer) */
} RX_RING_T;

typedef struct {
    uint8_t  buf[GSP_TX_RING_SIZE];
    uint16_t head;
    uint16_t tail;
} TX_RING_T;

/* ── Parser state machine ────────────────────────────────────────────── */

typedef enum {
    PARSE_WAIT_START,
    PARSE_GOT_START,
    PARSE_COLLECTING
} PARSE_STATE_T;

typedef struct {
    PARSE_STATE_T state;
    uint8_t  pktBuf[1 + GSP_MAX_PAYLOAD_LEN]; /* [CMD_ID][PAYLOAD...] */
    uint8_t  pktLen;        /* Expected LEN value (CMD_ID + payload bytes) */
    uint8_t  pktIdx;        /* Bytes collected so far into pktBuf */
    uint8_t  crcBuf[2];     /* CRC bytes [H][L] */
    uint8_t  crcIdx;        /* 0 or 1 while collecting CRC */
    bool     collectingCrc; /* true when pktBuf is full, collecting CRC */
    uint32_t lastRxTick;    /* systemTick at last received byte */
} PARSER_T;

/* ── Module state ────────────────────────────────────────────────────── */

static RX_RING_T rxRing;
static TX_RING_T txRing;
static PARSER_T  parser;
static uint32_t  gspTxOverflowCount;

/* ── CRC-16-CCITT ────────────────────────────────────────────────────── */

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

/* ── Ring buffer helpers ─────────────────────────────────────────────── */

static inline uint16_t RxCount(void)
{
    return (rxRing.head - rxRing.tail) & GSP_RX_RING_MASK;
}

static inline bool RxFull(void)
{
    return RxCount() == GSP_RX_RING_MASK; /* One slot reserved */
}

static inline void RxPut(uint8_t b)
{
    rxRing.buf[rxRing.head] = b;
    rxRing.head = (rxRing.head + 1) & GSP_RX_RING_MASK;
}

static inline uint8_t RxGet(void)
{
    uint8_t b = rxRing.buf[rxRing.tail];
    rxRing.tail = (rxRing.tail + 1) & GSP_RX_RING_MASK;
    return b;
}

static inline uint16_t TxCount(void)
{
    return (txRing.head - txRing.tail) & GSP_TX_RING_MASK;
}

static inline uint16_t TxFree(void)
{
    return GSP_TX_RING_MASK - TxCount(); /* One slot reserved */
}

static inline void TxPut(uint8_t b)
{
    txRing.buf[txRing.head] = b;
    txRing.head = (txRing.head + 1) & GSP_TX_RING_MASK;
}

static inline uint8_t TxGet(void)
{
    uint8_t b = txRing.buf[txRing.tail];
    txRing.tail = (txRing.tail + 1) & GSP_TX_RING_MASK;
    return b;
}

/* ── RX pump: HW FIFO → software ring ───────────────────────────────── */

static void PumpRx(void)
{
    /* Check for HW receive overflow first (RUNOVF=0 halts receiver) */
    if (UART1_IsReceiveBufferOverFlowDetected()) {
        UART1_ReceiveBufferOverrunErrorFlagClear();
    }

    while (UART1_IsReceiveBufferDataReady()) {
        uint8_t b = (uint8_t)UART1_DataRead();
        if (!RxFull()) {
            RxPut(b);
        }
        /* else: drop byte — host retries on timeout */
    }
}

/* ── TX pump: software ring → HW TX register ─────────────────────────── */

static void PumpTx(void)
{
    while (TxCount() > 0 && !UART1_StatusBufferFullTransmitGet()) {
        UART1_DataWrite(TxGet());
    }
}

/* ── Parser ──────────────────────────────────────────────────────────── */

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
    uint8_t cmdsProcesed = 0;

    /* Timeout check: reset parser if idle too long */
    if (parser.state != PARSE_WAIT_START) {
        uint32_t elapsed = garudaData.systemTick - parser.lastRxTick;
        if (elapsed > GSP_PARSER_TIMEOUT_MS) {
            ParserReset();
        }
    }

    while (RxCount() > 0 && cmdsProcesed < GSP_MAX_CMDS_PER_SVC) {
        uint8_t b = RxGet();
        parser.lastRxTick = garudaData.systemTick;

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
                /* Invalid LEN — reset */
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
                    /* Full packet received — validate CRC */
                    /* CRC is over [LEN][CMD_ID][PAYLOAD] */
                    uint16_t crc = CRC16_Update(0xFFFF, parser.pktLen);
                    for (uint8_t i = 0; i < parser.pktLen; i++)
                        crc = CRC16_Update(crc, parser.pktBuf[i]);

                    uint16_t rxCrc = ((uint16_t)parser.crcBuf[0] << 8)
                                   | parser.crcBuf[1];

                    if (crc == rxCrc) {
                        /* Valid packet — update heartbeat timer */
                        garudaData.lastGspPacketTick = garudaData.systemTick;

                        /* Dispatch */
                        uint8_t cmdId = parser.pktBuf[0];
                        uint8_t payloadLen = parser.pktLen - 1;
                        const uint8_t *payload = (payloadLen > 0)
                            ? &parser.pktBuf[1] : NULL;
                        GSP_DispatchCommand(cmdId, payload, payloadLen);
                        cmdsProcesed++;
                    }
                    /* CRC fail: silent drop */

                    ParserReset();
                }
            }
            break;
        }
    }
}

/* ── Public: Send response ───────────────────────────────────────────── */

bool GSP_SendResponse(uint8_t cmdId, const uint8_t *payload, uint8_t len)
{
    /* Defensive: reject oversized or NULL-with-length */
    if (len > GSP_MAX_PAYLOAD_LEN)
        return false;
    if (len > 0 && payload == NULL)
        return false;

    /* Total frame: START(1) + LEN(1) + CMD(1) + payload(len) + CRC(2) */
    uint16_t frameLen = 1 + 1 + 1 + len + 2;
    if (TxFree() < frameLen) {
        gspTxOverflowCount++;
        return false;
    }

    uint8_t pktLen = 1 + len; /* CMD_ID + payload */

    /* Compute CRC over [LEN][CMD_ID][PAYLOAD] */
    uint16_t crc = CRC16_Update(0xFFFF, pktLen);
    crc = CRC16_Update(crc, cmdId);
    for (uint8_t i = 0; i < len; i++)
        crc = CRC16_Update(crc, payload[i]);

    /* Enqueue frame */
    TxPut(GSP_START_BYTE);
    TxPut(pktLen);
    TxPut(cmdId);
    for (uint8_t i = 0; i < len; i++)
        TxPut(payload[i]);
    TxPut((uint8_t)(crc >> 8));
    TxPut((uint8_t)(crc & 0xFF));

    return true;
}

/* ── Public: Init ────────────────────────────────────────────────────── */

void GSP_Init(void)
{
    /* Reset ring buffers */
    memset(&rxRing, 0, sizeof(rxRing));
    memset(&txRing, 0, sizeof(txRing));

    /* Reset parser */
    ParserReset();
    parser.lastRxTick = 0;

    /* Reset diagnostics */
    gspTxOverflowCount = 0;

    /* Initialize UART1 — same setup as DiagnosticsInit() */
    UART1_InterruptReceiveDisable();
    UART1_InterruptReceiveFlagClear();
    UART1_InterruptTransmitDisable();
    UART1_InterruptTransmitFlagClear();
    UART1_Initialize();
    UART1_BaudRateDividerSet(GSP_BAUDRATE_DIVIDER);
    UART1_SpeedModeStandard();
    UART1_ModuleEnable();
}

/* ── Public: Service ─────────────────────────────────────────────────── */

void GSP_Service(void)
{
    PumpRx();
    ParserProcess();
    GSP_TelemTick();
    PumpTx();
}

#endif /* FEATURE_GSP */
