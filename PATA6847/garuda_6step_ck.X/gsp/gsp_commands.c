/**
 * @file gsp_commands.c
 * @brief GSP v2 command handlers for CK board.
 *
 * Phase 0: PING, GET_INFO, GET_SNAPSHOT
 * Phase 1: Motor control (START/STOP/CLEAR_FAULT/HEARTBEAT)
 * Phase 1: Telemetry streaming (TELEM_START/STOP/FRAME)
 */

#include "../garuda_config.h"

#if FEATURE_GSP

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "gsp_commands.h"
#include "gsp.h"
#include "gsp_snapshot.h"
#include "../garuda_types.h"
#include "../garuda_service.h"

/* ── Telemetry streaming state ──────────────────────────────────── */

static bool     telemStreaming;
static uint16_t telemSeq;
static uint32_t telemIntervalMs;
static uint32_t lastTelemTick;

/* systemTick accessed via gData (from garuda_service.h, already included) */

/* ── Feature flags bitmask ──────────────────────────────────────── */

static uint32_t BuildFeatureFlags(void)
{
    uint32_t f = 0;
#if FEATURE_IC_ZC
    f |= GSP_FEATURE_IC_ZC;
#endif
    f |= GSP_FEATURE_VBUS_FAULT;
    f |= GSP_FEATURE_DESYNC_REC;
    f |= GSP_FEATURE_DUTY_SLEW;
    f |= GSP_FEATURE_TIM_ADVANCE;
    f |= GSP_FEATURE_GSP;
    f |= GSP_FEATURE_ATA6847;
    f |= GSP_FEATURE_ILIM_HW;
    f |= GSP_FEATURE_CURRENT_SNS;
    return f;
}

/* ── Error helper ────────────────────────────────────────────────── */

static void SendError(uint8_t errCode)
{
    GSP_SendResponse(GSP_CMD_ERROR, &errCode, 1);
}

/* ── Phase 0 handlers ────────────────────────────────────────────── */

static void HandlePing(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payload;
    (void)payloadLen;
    GSP_SendResponse(GSP_CMD_PING, NULL, 0);
}

static void HandleGetInfo(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payload;
    (void)payloadLen;

    GSP_INFO_T info;
    memset(&info, 0, sizeof(info));

    info.protocolVersion = GSP_PROTOCOL_VERSION;
    info.fwMajor         = GSP_FW_MAJOR;
    info.fwMinor         = GSP_FW_MINOR;
    info.fwPatch         = GSP_FW_PATCH;
    info.boardId         = GSP_BOARD_EV43F54A;
    info.motorProfile    = MOTOR_PROFILE;
    info.motorPolePairs  = MOTOR_POLE_PAIRS;
    info.featureFlags    = BuildFeatureFlags();
    info.pwmFrequency    = PWMFREQUENCY_HZ;
    info.maxErpm         = MAX_CLOSED_LOOP_ERPM;

    GSP_SendResponse(GSP_CMD_GET_INFO, (const uint8_t *)&info, sizeof(info));
}

static void HandleGetSnapshot(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payload;
    (void)payloadLen;

    static GSP_CK_SNAPSHOT_T snapshot;
    GSP_CaptureSnapshot(&snapshot);
    GSP_SendResponse(GSP_CMD_GET_SNAPSHOT, (const uint8_t *)&snapshot,
                     sizeof(snapshot));
}

/* ── Phase 1: Motor control ──────────────────────────────────────── */

static void HandleStartMotor(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payload;
    (void)payloadLen;

    if (gData.state != ESC_IDLE) {
        SendError(GSP_ERR_WRONG_STATE);
        return;
    }
    GarudaService_StartMotor();
    GSP_SendResponse(GSP_CMD_START_MOTOR, NULL, 0);
}

static void HandleStopMotor(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payload;
    (void)payloadLen;
    GarudaService_StopMotor();
    GSP_SendResponse(GSP_CMD_STOP_MOTOR, NULL, 0);
}

static void HandleClearFault(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payload;
    (void)payloadLen;

    if (gData.state != ESC_FAULT) {
        SendError(GSP_ERR_WRONG_STATE);
        return;
    }
    gData.state = ESC_IDLE;
    gData.faultCode = FAULT_NONE;
    GSP_SendResponse(GSP_CMD_CLEAR_FAULT, NULL, 0);
}

static void HandleHeartbeat(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payload;
    (void)payloadLen;
    GSP_SendResponse(GSP_CMD_HEARTBEAT, NULL, 0);
}

/* ── Phase 1: Telemetry streaming ────────────────────────────────── */

static void HandleTelemStart(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payloadLen;

    uint8_t rateHz = payload[0];
    if (rateHz < 10) rateHz = 10;
    if (rateHz > 100) rateHz = 100;

    telemIntervalMs = 1000 / rateHz;
    telemStreaming = true;
    lastTelemTick = gData.systemTick;
    telemSeq = 0;

    uint8_t actualRate = (uint8_t)(1000 / telemIntervalMs);
    GSP_SendResponse(GSP_CMD_TELEM_START, &actualRate, 1);
}

static void HandleTelemStop(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payload;
    (void)payloadLen;
    telemStreaming = false;
    GSP_SendResponse(GSP_CMD_TELEM_STOP, NULL, 0);
}

void GSP_TelemTick(void)
{
    if (!telemStreaming)
        return;

    uint32_t now = gData.systemTick;
    if ((now - lastTelemTick) < telemIntervalMs)
        return;

    lastTelemTick = now;
    telemSeq++;

    uint8_t buf[2 + sizeof(GSP_CK_SNAPSHOT_T)];
    buf[0] = (uint8_t)(telemSeq & 0xFF);
    buf[1] = (uint8_t)(telemSeq >> 8);
    GSP_CaptureSnapshot((GSP_CK_SNAPSHOT_T *)&buf[2]);
    GSP_SendResponse(GSP_CMD_TELEM_FRAME, buf, sizeof(buf));
}

/* ── Dispatch table ──────────────────────────────────────────────── */

typedef struct {
    uint8_t           cmdId;
    uint8_t           expectedPayloadLen;  /* 0xFF = any length */
    GSP_CMD_HANDLER_T handler;
} CMD_ENTRY_T;

static const CMD_ENTRY_T cmdTable[] = {
    /* Phase 0 */
    { GSP_CMD_PING,            0, HandlePing           },
    { GSP_CMD_GET_INFO,        0, HandleGetInfo        },
    { GSP_CMD_GET_SNAPSHOT,    0, HandleGetSnapshot    },
    /* Phase 1: motor control */
    { GSP_CMD_START_MOTOR,     0, HandleStartMotor     },
    { GSP_CMD_STOP_MOTOR,      0, HandleStopMotor      },
    { GSP_CMD_CLEAR_FAULT,     0, HandleClearFault     },
    { GSP_CMD_HEARTBEAT,       0, HandleHeartbeat      },
    /* Phase 1: telemetry */
    { GSP_CMD_TELEM_START,     1, HandleTelemStart     },
    { GSP_CMD_TELEM_STOP,      0, HandleTelemStop      },
};

#define CMD_TABLE_SIZE (sizeof(cmdTable) / sizeof(cmdTable[0]))

void GSP_DispatchCommand(uint8_t cmdId, const uint8_t *payload,
                         uint8_t payloadLen)
{
    for (uint8_t i = 0; i < CMD_TABLE_SIZE; i++) {
        if (cmdTable[i].cmdId == cmdId) {
            if (cmdTable[i].expectedPayloadLen != 0xFF &&
                payloadLen != cmdTable[i].expectedPayloadLen) {
                SendError(GSP_ERR_BAD_LENGTH);
                return;
            }
            cmdTable[i].handler(payload, payloadLen);
            return;
        }
    }
    SendError(GSP_ERR_UNKNOWN_CMD);
}

#endif /* FEATURE_GSP */
