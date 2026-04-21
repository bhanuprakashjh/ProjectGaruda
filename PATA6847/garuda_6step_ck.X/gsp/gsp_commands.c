/**
 * @file gsp_commands.c
 * @brief GSP v2 command handlers for CK board.
 *
 * Phase 0: PING, GET_INFO, GET_SNAPSHOT
 * Phase 1: Motor control (START/STOP/CLEAR_FAULT/HEARTBEAT)
 * Phase 1: Telemetry streaming (TELEM_START/STOP/FRAME)
 */

#include "../garuda_config.h"

#if FEATURE_GSP && !FEATURE_V4_SECTOR_PI

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

#if FEATURE_DMA_BURST_CAPTURE
#include "../hal/hal_dma_burst.h"

static void HandleBurstArm(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payload; (void)payloadLen;
    HAL_DmaBurst_Arm();
    uint8_t ack = 1;
    GSP_SendResponse(GSP_CMD_BURST_ARM, &ack, 1);
}

static void HandleBurstStatus(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payload; (void)payloadLen;
    uint8_t resp[2];
    resp[0] = HAL_DmaBurst_GetState();
    resp[1] = HAL_DmaBurst_GetStepCount();
    GSP_SendResponse(GSP_CMD_BURST_STATUS, resp, 2);
}

static void HandleBurstGetStep(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payloadLen;
    uint8_t idx = payload[0];
    const DMA_BURST_STEP_T *step = HAL_DmaBurst_GetStep(idx);
    if (step == 0) {
        SendError(GSP_ERR_OUT_OF_RANGE);
        return;
    }
    GSP_SendResponse(GSP_CMD_BURST_GET_STEP,
                     (const uint8_t *)step,
                     (uint8_t)sizeof(DMA_BURST_STEP_T));
}
#endif /* FEATURE_DMA_BURST_CAPTURE */

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
#if FEATURE_DMA_BURST_CAPTURE
    { GSP_CMD_BURST_ARM,       0, HandleBurstArm       },
    { GSP_CMD_BURST_STATUS,    0, HandleBurstStatus    },
    { GSP_CMD_BURST_GET_STEP,  1, HandleBurstGetStep   },
#endif
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

#endif /* FEATURE_GSP && !FEATURE_V4_SECTOR_PI */

/* V4: minimal GSP command handlers — PING, START, STOP, CLEAR_FAULT */
#if FEATURE_GSP && FEATURE_V4_SECTOR_PI
#include <stdint.h>
#include <string.h>
#include "gsp_commands.h"
#include "gsp.h"
#include "../garuda_service.h"
#include "../garuda_config.h"
#include "../motor/sector_pi.h"
#include "../motor/v4_params.h"

extern volatile ESC_STATE_T gV4State;
extern volatile uint32_t gV4SystemTick;
extern volatile uint16_t gV4PotRaw;
extern volatile uint16_t gV4VbusRaw;
extern volatile int16_t  gV4IaRaw;
extern volatile int16_t  gV4IbRaw;

static bool     v4_telemActive = false;
static uint16_t v4_telemSeq = 0;
static uint32_t v4_telemLastTick = 0;

void GSP_DispatchCommand(uint8_t cmdId, const uint8_t *payload, uint8_t payloadLen)
{
    (void)payload; (void)payloadLen;

    switch (cmdId)
    {
        case GSP_CMD_PING:
            GSP_FlushTx();
            GSP_SendResponse(GSP_CMD_PING, NULL, 0);
            break;

        case GSP_CMD_GET_INFO:
        {
            /* GUI's decodeInfo expects 20 bytes (matches GSP_INFO_T):
             *   [0]    protocolVersion (u8)
             *   [1..3] fwMajor, fwMinor, fwPatch (u8 each)
             *   [4..5] boardId (u16 LE) — 0x0002 for CK board
             *   [6]    motorProfile (u8)
             *   [7]    motorPolePairs (u8)
             *   [8..11]  featureFlags (u32 LE)
             *   [12..15] pwmFrequency (u32 LE)
             *   [16..19] maxErpm (u32 LE)
             * Sending fewer bytes makes the GUI's DataView throw, which
             * kills the read loop → disconnect → writes fail. */
            uint8_t info[20];
            memset(info, 0, sizeof(info));
            info[0] = GSP_PROTOCOL_VERSION;
            info[1] = 4;                    /* fwMajor — V4 */
            info[2] = 0;                    /* fwMinor */
            info[3] = 0;                    /* fwPatch */
            uint16_t boardId = GSP_BOARD_EV43F54A;
            memcpy(&info[4], &boardId, 2);
            info[6] = MOTOR_PROFILE;
            info[7] = 7;                    /* polePairs (placeholder) */
            uint32_t f = 0x80000000UL;      /* bit 31 = V4 marker */
            memcpy(&info[8],  &f, 4);
            uint32_t pwmHz = PWMFREQUENCY_HZ;
            memcpy(&info[12], &pwmHz, 4);
            uint32_t maxErpm = 200000UL;
            memcpy(&info[16], &maxErpm, 4);
            GSP_SendResponse(GSP_CMD_GET_INFO, info, 20);
            break;
        }

        case GSP_CMD_GET_SNAPSHOT:
        case GSP_CMD_TELEM_START:
        case GSP_CMD_TELEM_STOP:
            /* Handled below in TelemTick */
            if (cmdId == GSP_CMD_TELEM_START)
                v4_telemActive = true;
            else if (cmdId == GSP_CMD_TELEM_STOP)
                v4_telemActive = false;
            GSP_SendResponse(cmdId, NULL, 0);
            break;

        case GSP_CMD_START_MOTOR:
            if (gV4State == ESC_IDLE)
                GarudaService_StartMotor();
            GSP_SendResponse(GSP_CMD_START_MOTOR, NULL, 0);
            break;

        case GSP_CMD_STOP_MOTOR:
            GarudaService_StopMotor();
            GSP_SendResponse(GSP_CMD_STOP_MOTOR, NULL, 0);
            break;

        case GSP_CMD_CLEAR_FAULT:
            GarudaService_ClearFault();
            GSP_SendResponse(GSP_CMD_CLEAR_FAULT, NULL, 0);
            break;

        case GSP_CMD_HEARTBEAT:
            /* GUI sends HEARTBEAT every 200ms while telemetry is active
             * to keep the connection alive. V4 has no watchdog action —
             * just acknowledge so the GUI doesn't spam unknown-command
             * error toasts. */
            GSP_SendResponse(GSP_CMD_HEARTBEAT, NULL, 0);
            break;

        case GSP_CMD_SET_THROTTLE:
            /* V4 reads pot directly via ADC; ignore GUI throttle commands
             * silently rather than spam unknown-command errors. */
            GSP_SendResponse(GSP_CMD_SET_THROTTLE, NULL, 0);
            break;

        case GSP_CMD_GET_PARAM:
        {
            /* Payload: 2-byte param ID (LE).
             * Response: 6 bytes — id (u16) + value (u32). */
            if (payloadLen < 2) {
                uint8_t err = GSP_ERR_BAD_LENGTH;
                GSP_SendResponse(GSP_CMD_ERROR, &err, 1);
                break;
            }
            uint16_t id = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
            bool ok;
            uint32_t val = V4Params_Get(id, &ok);
            if (!ok) {
                uint8_t err = GSP_ERR_UNKNOWN_PARAM;
                GSP_SendResponse(GSP_CMD_ERROR, &err, 1);
                break;
            }
            uint8_t resp[6];
            resp[0] = (uint8_t)(id & 0xFF);
            resp[1] = (uint8_t)(id >> 8);
            resp[2] = (uint8_t)(val & 0xFF);
            resp[3] = (uint8_t)((val >> 8) & 0xFF);
            resp[4] = (uint8_t)((val >> 16) & 0xFF);
            resp[5] = (uint8_t)((val >> 24) & 0xFF);
            GSP_SendResponse(GSP_CMD_GET_PARAM, resp, 6);
            break;
        }

        case GSP_CMD_SET_PARAM:
        {
            /* Payload: 2-byte ID + 4-byte value (LE).
             * V4 HOT params are change-while-running — no IDLE check.
             * Response: echo the 6-byte payload on success. */
            if (payloadLen < 6) {
                uint8_t err = GSP_ERR_BAD_LENGTH;
                GSP_SendResponse(GSP_CMD_ERROR, &err, 1);
                break;
            }
            uint16_t id = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
            uint32_t val = (uint32_t)payload[2]
                         | ((uint32_t)payload[3] << 8)
                         | ((uint32_t)payload[4] << 16)
                         | ((uint32_t)payload[5] << 24);
            if (!V4Params_Set(id, val)) {
                uint8_t err = GSP_ERR_OUT_OF_RANGE;
                GSP_SendResponse(GSP_CMD_ERROR, &err, 1);
                break;
            }
            GSP_SendResponse(GSP_CMD_SET_PARAM, payload, 6);
            break;
        }

        case GSP_CMD_GET_PARAM_LIST:
        {
            /* Paginated descriptor table. Payload: 1 byte = startIndex.
             * Response: [totalCount, startIndex, pageSize, entries...]
             * Each entry is 12 bytes: id(u16), type(u8), group(u8), min(u32), max(u32).
             * Page sized to fit GSP max payload (249 - 3 header = 246, /12 = 20). */
            uint8_t startIndex = (payloadLen >= 1) ? payload[0] : 0;
            uint8_t totalCount;
            const V4_PARAM_DESC_T *table = V4Params_GetDescriptorTable(&totalCount);

            if (startIndex >= totalCount) {
                /* Tell GUI "no more pages" by returning empty page header
                 * rather than error — matches V3 convention from V3 code. */
                uint8_t hdr[3] = { totalCount, startIndex, 0 };
                GSP_SendResponse(GSP_CMD_GET_PARAM_LIST, hdr, 3);
                break;
            }
            uint8_t remaining = totalCount - startIndex;
            uint8_t pageSize = (remaining > 20) ? 20 : remaining;

            uint8_t buf[3 + 20 * 12];
            buf[0] = totalCount;
            buf[1] = startIndex;
            buf[2] = pageSize;
            uint8_t i;
            for (i = 0; i < pageSize; i++) {
                const V4_PARAM_DESC_T *d = &table[startIndex + i];
                uint8_t off = 3 + i * 12;
                buf[off + 0]  = (uint8_t)(d->id & 0xFF);
                buf[off + 1]  = (uint8_t)(d->id >> 8);
                buf[off + 2]  = d->type;
                buf[off + 3]  = d->group;
                buf[off + 4]  = (uint8_t)(d->min & 0xFF);
                buf[off + 5]  = (uint8_t)((d->min >> 8) & 0xFF);
                buf[off + 6]  = (uint8_t)((d->min >> 16) & 0xFF);
                buf[off + 7]  = (uint8_t)((d->min >> 24) & 0xFF);
                buf[off + 8]  = (uint8_t)(d->max & 0xFF);
                buf[off + 9]  = (uint8_t)((d->max >> 8) & 0xFF);
                buf[off + 10] = (uint8_t)((d->max >> 16) & 0xFF);
                buf[off + 11] = (uint8_t)((d->max >> 24) & 0xFF);
            }
            GSP_SendResponse(GSP_CMD_GET_PARAM_LIST, buf, 3 + pageSize * 12);
            break;
        }

        case GSP_CMD_SAVE_CONFIG:
            /* Phase B will wire EEPROM. For now ack so GUI doesn't error. */
            GSP_SendResponse(GSP_CMD_SAVE_CONFIG, NULL, 0);
            break;

        case GSP_CMD_LOAD_DEFAULTS:
            V4Params_InitDefaults();
            GSP_SendResponse(GSP_CMD_LOAD_DEFAULTS, NULL, 0);
            break;

        default:
        {
            uint8_t err = GSP_ERR_UNKNOWN_CMD;
            GSP_SendResponse(GSP_CMD_ERROR, &err, 1);
            break;
        }
    }
}

void GSP_TelemTick(void)
{
    if (!v4_telemActive) return;

    /* Rate limit: ~10 Hz (every 100ms) */
    uint32_t now = gV4SystemTick;
    if ((now - v4_telemLastTick) < 100) return;
    v4_telemLastTick = now;

    /* Build a V3-compatible 64-byte snapshot so pot_capture.py and the
     * GUI can decode it. Slots 0-47 carry V3-mapped fields; 48-63 are
     * V4 capture-rate diagnostic counters. */
    V4_TELEM_T t;
    SectorPI_TelemGet(&t);

    uint8_t snap[148]; /* 2-byte seq + 64-byte snapshot + 8-byte off-mid
                        * + 4-byte V5 PTG fire counter
                        * + 16-byte V5 PTG per-polarity counters
                        * + 16-byte V5.1 ADC post-ZC shadow counters
                        * + 2-byte V5.2 measurement-PI tracker (tMeasHR)
                        * + 4-byte sectorCount (full 32-bit, for rate diag)
                        * + 32-byte phase-current peak block (2026-04-21) */
    memset(snap, 0, sizeof(snap));

    /* Seq counter (2 bytes) */
    snap[0] = (uint8_t)(v4_telemSeq & 0xFF);
    snap[1] = (uint8_t)(v4_telemSeq >> 8);
    v4_telemSeq++;

    uint8_t *d = &snap[2];  /* snapshot starts at offset 2 */

    /* Core state (offset 0-7) */
    d[0] = (uint8_t)gV4State;               /* state */
    d[1] = 0;                                /* fault */
    d[2] = t.position;                       /* step */
    d[3] = 0;                                /* ataStatus */
    d[4] = (uint8_t)(gV4PotRaw & 0xFF);     /* potRaw L */
    d[5] = (uint8_t)(gV4PotRaw >> 8);       /* potRaw H */
    uint8_t dutyPct = 0;
    if (t.actualAmplitude > 0)
        dutyPct = (uint8_t)((uint32_t)t.actualAmplitude * 100 / 32768);
    d[6] = dutyPct;                          /* dutyPct */
    d[7] = t.commandEnabled ? 1 : 0;        /* zcSynced (repurpose as PI active) */

    /* Electrical (offset 8-17) */
    memcpy(&d[8],  &gV4VbusRaw, 2);         /* vbusRaw */
    memcpy(&d[10], &gV4IaRaw, 2);           /* iaRaw */
    memcpy(&d[12], &gV4IbRaw, 2);           /* ibRaw */
    /* Reconstruct IBus from the active PWM phase per commutation step
     * (mirrors V3 logic at garuda_service.c:1431). The PWM-active phase
     * carries the full DC bus current; phases A/B sensed directly,
     * phase C inferred from -(Ia+Ib).
     *   Steps 0,5: A=PWM → IBus = |Ia|
     *   Steps 3,4: B=PWM → IBus = |Ib|
     *   Steps 1,2: C=PWM → IBus = |-(Ia+Ib)|
     * Computed here (cold path) instead of in the ADC ISR to keep the
     * ISR fast and to avoid touching shared globals from two paths. */
    {
        int16_t ibus;
        switch (t.position) {
            case 0: case 5: ibus = gV4IaRaw; break;
            case 3: case 4: ibus = gV4IbRaw; break;
            default:        ibus = (int16_t)(-(gV4IaRaw + gV4IbRaw)); break;
        }
        if (ibus < 0) ibus = (int16_t)(-ibus);
        memcpy(&d[14], &ibus, 2);            /* ibusRaw (abs, like V3) */
    }
    memcpy(&d[16], &t.actualAmplitude, 2);   /* duty (raw) */

    /* Speed/Timing (offset 18-31) */
    memcpy(&d[18], &t.timerPeriod, 2);       /* stepPeriod → timerPeriod */
    memcpy(&d[20], &t.timerPeriod, 2);       /* stepPeriodHR → same */
    uint32_t eRpm = SectorPI_ErpmGet();
    memcpy(&d[22], &eRpm, 4);               /* eRpm */
    memcpy(&d[26], &t.sectorCount, 2);      /* goodZc → sectorCount low */
    /* zcInterval, prevZcInterval = 0 */

    /* ZC diagnostics (offset 28-39) — repurposed for V4 diag */
    memcpy(&d[28], &t.diagLastCapValue, 2);  /* zcInterval → lastCapValue */
    memcpy(&d[30], &t.diagDelta, 2);         /* prevZcInterval → PI delta (signed) */
    memcpy(&d[32], &t.diagCaptures, 2);      /* icAccepted → captures accepted */
    memcpy(&d[34], &t.diagPiRuns, 2);        /* icFalse → PI run count */
    d[36] = t.stallCounter;                  /* filterLevel → stallCounter */
    /* Pack both SP bits: bit0 = active, bit1 = request.
     * 0 = no request, not active
     * 2 = request latched but SP not yet applied (boundary lag / error)
     * 3 = SP active and requested
     * 1 = stale/impossible active state */
    d[37] = (t.spMode ? 1U : 0U) | (t.spRequest ? 2U : 0U);
    { uint16_t erpm16 = (t.erpmNow > 0xFFFF) ? 0xFFFF : (uint16_t)t.erpmNow;
      memcpy(&d[38], &erpm16, 2); }          /* erpmNow from timerPeriod */

    /* System (offset 40-47) */
    memcpy(&d[40], &gV4SystemTick, 4);       /* systemTick */
    uint32_t uptime = gV4SystemTick / 1000;
    memcpy(&d[44], &uptime, 4);              /* uptime */

    /* V4 capture-rate diagnostics (offset 48-63). Reuse the V3 snapshot
     * tail (zcLatencyPct etc.) which V4 doesn't populate.
     *   adcBlankReject (uint32) : ADC fired pre-blanking-end
     *   adcStateMismatch(uint32): past blanking, GPIO != expected post-ZC
     *   adcCaptureSet  (uint32) : total captures (all 6 sectors)
     *   adcSetRising   (uint32) : subset of adcCaptureSet on rising-ZC
     *                             sectors (0,2,4). Falling = total - rising.
     *                             Replaces adcAlreadySet, which was redundant
     *                             given Set tracks every success 1:1.
     * 32-bit is required: ADC fires at 40 kHz so uint16 wraps every ~1.6s.
     * commutateNoCapture is no longer shipped; host computes it as
     * (sectorCount - diagCaptures). */
    memcpy(&d[48], &t.adcBlankReject,   4);
    memcpy(&d[52], &t.adcStateMismatch, 4);
    memcpy(&d[56], &t.adcCaptureSet,    4);
    memcpy(&d[60], &t.adcSetRising,     4);
    memcpy(&d[64], &t.offMidCapture,    4);
    memcpy(&d[68], &t.offMidMismatch,   4);
    memcpy(&d[72], &t.ptgFires,         4);  /* V5.0 PTG ISR fire count */
    memcpy(&d[76], &t.ptgRisingAcc,     4);  /* V5.0 per-polarity shadow */
    memcpy(&d[80], &t.ptgRisingRej,     4);
    memcpy(&d[84], &t.ptgFallingAcc,    4);
    memcpy(&d[88], &t.ptgFallingRej,    4);

    /* V5.1 ADC post-ZC shadow counters. */
    memcpy(&d[92],  &t.postZcRisingAcc,  4);
    memcpy(&d[96],  &t.postZcRisingRej,  4);
    memcpy(&d[100], &t.postZcFallingAcc, 4);
    memcpy(&d[104], &t.postZcFallingRej, 4);

    /* V5.2 measurement-PI tracker. */
    memcpy(&d[108], &t.tMeasHR, 2);

    /* Full 32-bit sectorCount for host-side rate computation. */
    memcpy(&d[110], &t.sectorCount, 4);

    /* Phase-current peaks (2026-04-21) — validate kickback theory on CK.
     * Rolling window reset on snapshot read; at-fault fields preserved
     * across snapshots until motor restart (valid==1). */
    extern volatile int16_t gV4IaPkMax, gV4IaPkMin;
    extern volatile int16_t gV4IbPkMax, gV4IbPkMin;
    extern volatile int16_t gV4IaAtFaultMax, gV4IaAtFaultMin;
    extern volatile int16_t gV4IbAtFaultMax, gV4IbAtFaultMin;
    extern volatile int16_t gV4IaAtFaultInst, gV4IbAtFaultInst;
    extern volatile uint8_t gV4FaultSnapshotValid;
    extern volatile int16_t gV4IaRaw, gV4IbRaw;

    /* Reconstruct ibus peaks host-side from Ia/Ib extrema. We emit
     * zero for ibus fields; host can compute |ibus|pk ≈ max(|ia|,|ib|)
     * which is a lower bound (exact for steps 0,5,3,4; underestimate by
     * up to √3 for steps 1,2 where C is PWM). Good enough to compare
     * against AKESC datasets. */
    int16_t ibusZero = 0;

    memcpy(&d[114], &gV4IaPkMax,   2);
    memcpy(&d[116], &gV4IaPkMin,   2);
    memcpy(&d[118], &gV4IbPkMax,   2);
    memcpy(&d[120], &gV4IbPkMin,   2);
    memcpy(&d[122], &ibusZero,     2);
    memcpy(&d[124], &ibusZero,     2);
    memcpy(&d[126], &gV4IaAtFaultMax,   2);
    memcpy(&d[128], &gV4IaAtFaultMin,   2);
    memcpy(&d[130], &gV4IbAtFaultMax,   2);
    memcpy(&d[132], &gV4IbAtFaultMin,   2);
    memcpy(&d[134], &ibusZero,          2);
    memcpy(&d[136], &ibusZero,          2);
    memcpy(&d[138], &gV4IaAtFaultInst,   2);
    memcpy(&d[140], &gV4IbAtFaultInst,   2);
    memcpy(&d[142], &ibusZero,           2);
    d[144] = gV4FaultSnapshotValid;
    d[145] = 0;  /* pad */

    /* Reset rolling peaks for next 20 ms window. Seed with current
     * instantaneous sample so the window doesn't start at stale extrema. */
    gV4IaPkMax = gV4IaPkMin = gV4IaRaw;
    gV4IbPkMax = gV4IbPkMin = gV4IbRaw;

    GSP_SendResponse(GSP_CMD_TELEM_FRAME, snap, sizeof(snap));
}
#endif
