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
#include "gsp_ck_params.h"
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

    /* PING acts as a reconnect signal — stop any running telemetry
     * and flush TX ring so the PING response gets through immediately.
     * This fixes the "need to replug USB" issue after disconnect. */
    telemStreaming = false;
    GSP_FlushTx();  /* Clear stale telemetry frames from TX ring */

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
    info.motorProfile    = ckParams.activeProfile;
    info.motorPolePairs  = ckParams.polePairs;
    info.featureFlags    = BuildFeatureFlags();
    info.pwmFrequency    = PWMFREQUENCY_HZ;
    info.maxErpm         = ckParams.maxClosedLoopErpm;

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

/**
 * SET_THROTTLE: 2-byte LE uint16 throttle value (0-65535, same scale as pot ADC).
 * 0 = idle (pot dead zone applies). 65535 = full throttle.
 * Activates GSP throttle override — pot is ignored while active.
 * Send value 0xFFFF with flag byte 0x00 to release back to pot.
 * Payload: [value_lo] [value_hi] or [value_lo] [value_hi] [flags]
 *   flags: 0x01 = activate override, 0x00 = release to pot
 */
static void HandleSetThrottle(const uint8_t *payload, uint8_t payloadLen)
{
    if (payloadLen < 2) {
        SendError(0x03);  /* Invalid payload */
        return;
    }

    uint16_t value = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);

    if (payloadLen >= 3 && payload[2] == 0x00) {
        /* Release: return to pot control */
        gData.gspThrottleActive = false;
        gData.gspThrottleValue = 0;
    } else {
        /* Activate GSP throttle override */
        gData.gspThrottleActive = true;
        gData.gspThrottleValue = value;
    }

    GSP_SendResponse(GSP_CMD_SET_THROTTLE, (const uint8_t *)&value, 2);
}

static void HandleClearFault(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payload;
    (void)payloadLen;

    if (gData.state != ESC_FAULT) {
        SendError(GSP_ERR_WRONG_STATE);
        return;
    }
    GarudaService_ClearFault();
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

/* ── Parameter handlers ──────────────────────────────────────────── */

static void HandleGetParam(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payloadLen;
    uint16_t paramId = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);

    bool ok;
    uint32_t val = CK_ParamGet(paramId, &ok);
    if (!ok) {
        SendError(GSP_ERR_UNKNOWN_PARAM);
        return;
    }

    uint8_t resp[6];
    resp[0] = (uint8_t)(paramId & 0xFF);
    resp[1] = (uint8_t)(paramId >> 8);
    resp[2] = (uint8_t)(val & 0xFF);
    resp[3] = (uint8_t)((val >> 8) & 0xFF);
    resp[4] = (uint8_t)((val >> 16) & 0xFF);
    resp[5] = (uint8_t)((val >> 24) & 0xFF);
    GSP_SendResponse(GSP_CMD_GET_PARAM, resp, 6);
}

static void HandleSetParam(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payloadLen;

    if (gData.state != ESC_IDLE) {
        SendError(GSP_ERR_WRONG_STATE);
        return;
    }

    uint16_t paramId = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
    uint32_t value = (uint32_t)payload[2] |
                     ((uint32_t)payload[3] << 8) |
                     ((uint32_t)payload[4] << 16) |
                     ((uint32_t)payload[5] << 24);

    if (!CK_ParamSet(paramId, value)) {
        SendError(GSP_ERR_OUT_OF_RANGE);
        return;
    }

    if (!CK_ParamsCrossValidate()) {
        /* Revert not implemented yet — just warn */
        SendError(GSP_ERR_CROSS_VALIDATION);
        return;
    }

    CK_RecomputeDerived();

    /* Echo back the set value */
    uint8_t resp[6];
    resp[0] = payload[0];
    resp[1] = payload[1];
    resp[2] = payload[2];
    resp[3] = payload[3];
    resp[4] = payload[4];
    resp[5] = payload[5];
    GSP_SendResponse(GSP_CMD_SET_PARAM, resp, 6);
}

static void HandleSaveConfig(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payload;
    (void)payloadLen;
    /* RAM-only for now — no NVM persistence */
    GSP_SendResponse(GSP_CMD_SAVE_CONFIG, NULL, 0);
}

static void HandleLoadDefaults(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payload;
    (void)payloadLen;

    if (gData.state != ESC_IDLE) {
        SendError(GSP_ERR_WRONG_STATE);
        return;
    }

    CK_ParamsLoadProfile(ckParams.activeProfile);
    GSP_SendResponse(GSP_CMD_LOAD_DEFAULTS, NULL, 0);
}

static void HandleGetParamList(const uint8_t *payload, uint8_t payloadLen)
{
    uint8_t startIndex = 0;
    if (payloadLen >= 1)
        startIndex = payload[0];

    uint8_t totalCount;
    const CK_PARAM_DESC_T *table = CK_GetDescriptorTable(&totalCount);

    if (startIndex >= totalCount) {
        SendError(GSP_ERR_OUT_OF_RANGE);
        return;
    }

    /* Max 20 entries per page (20 * 12 = 240 + 3 header = 243 < 249 max) */
    uint8_t remaining = totalCount - startIndex;
    uint8_t pageSize = (remaining > 20) ? 20 : remaining;

    uint8_t buf[3 + 20 * 12];  /* max page */
    buf[0] = totalCount;
    buf[1] = startIndex;
    buf[2] = pageSize;

    uint8_t i;
    for (i = 0; i < pageSize; i++) {
        const CK_PARAM_DESC_T *d = &table[startIndex + i];
        uint8_t off = 3 + i * 12;
        buf[off + 0] = (uint8_t)(d->id & 0xFF);
        buf[off + 1] = (uint8_t)(d->id >> 8);
        buf[off + 2] = d->type;
        buf[off + 3] = d->group;
        buf[off + 4] = (uint8_t)(d->min & 0xFF);
        buf[off + 5] = (uint8_t)((d->min >> 8) & 0xFF);
        buf[off + 6] = (uint8_t)((d->min >> 16) & 0xFF);
        buf[off + 7] = (uint8_t)((d->min >> 24) & 0xFF);
        buf[off + 8] = (uint8_t)(d->max & 0xFF);
        buf[off + 9] = (uint8_t)((d->max >> 8) & 0xFF);
        buf[off + 10] = (uint8_t)((d->max >> 16) & 0xFF);
        buf[off + 11] = (uint8_t)((d->max >> 24) & 0xFF);
    }

    GSP_SendResponse(GSP_CMD_GET_PARAM_LIST, buf, 3 + pageSize * 12);
}

static void HandleLoadProfile(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payloadLen;

    if (gData.state != ESC_IDLE) {
        SendError(GSP_ERR_WRONG_STATE);
        return;
    }

    uint8_t profileId = payload[0];
    if (profileId >= CK_PROFILE_COUNT) {
        SendError(GSP_ERR_OUT_OF_RANGE);
        return;
    }

    CK_ParamsLoadProfile(profileId);
    GSP_SendResponse(GSP_CMD_LOAD_PROFILE, &profileId, 1);
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
    { GSP_CMD_SET_THROTTLE,    0xFF, HandleSetThrottle  },
    { GSP_CMD_CLEAR_FAULT,     0, HandleClearFault     },
    { GSP_CMD_HEARTBEAT,       0, HandleHeartbeat      },
    /* Phase 1: telemetry */
    { GSP_CMD_TELEM_START,     1, HandleTelemStart     },
    { GSP_CMD_TELEM_STOP,      0, HandleTelemStop      },
    /* Phase 2: parameters */
    { GSP_CMD_GET_PARAM,       2, HandleGetParam       },
    { GSP_CMD_SET_PARAM,       6, HandleSetParam       },
    { GSP_CMD_SAVE_CONFIG,     0, HandleSaveConfig     },
    { GSP_CMD_LOAD_DEFAULTS,   0, HandleLoadDefaults   },
    { GSP_CMD_GET_PARAM_LIST,  0xFF, HandleGetParamList },
    { GSP_CMD_LOAD_PROFILE,    1, HandleLoadProfile    },
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
