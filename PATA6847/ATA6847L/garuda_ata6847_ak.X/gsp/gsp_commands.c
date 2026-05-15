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
#include "../motor/v4_params.h"   /* v4Params for buildHash fold */

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

    /* Build hash: djb2 of __DATE__ " " __TIME__ folded with key V4
     * tunables.  Updates every recompile AND every time a tunable
     * changes — host can verify "is the firmware I expect actually
     * on the chip?" by comparing the printed hash. */
    {
        static const char buildStamp[] = __DATE__ " " __TIME__;
        uint32_t hash = 5381UL;
        for (const char *p = buildStamp; *p; p++) {
            hash = ((hash << 5) + hash) ^ (uint32_t)(uint8_t)*p;
        }
        /* Fold V4 tunables so any GUI param tweak shifts the hash too. */
        hash ^= (uint32_t)v4Params.phaseAdvanceDegX10;
        hash ^= ((uint32_t)v4Params.blankingPct) << 8;
        hash ^= ((uint32_t)v4Params.piKpShift)   << 16;
        hash ^= ((uint32_t)v4Params.piKiShift)   << 20;
        hash ^= ((uint32_t)v4Params.minPeriodHr) << 0;
        info.buildHash = hash;
    }

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
#include "../hal/hal_ata6847.h"
#include "../hal/hal_capture.h"
#include "../hal/port_config.h"

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

        case GSP_CMD_PI_LOG:
        {
            /* Diagnostic dump of first PI runs after CL entry.
             * Response: [count][30×8B entries] — total <=241 bytes. */
            uint8_t resp[1 + 30 * 8];
            uint8_t n = 0;
            SectorPI_GetCaptureLog(&resp[1], &n);
            resp[0] = n;
            GSP_SendResponse(GSP_CMD_PI_LOG, resp, (uint8_t)(1U + (uint16_t)n * 8U));
            break;
        }

        case GSP_CMD_BEMF_PROBE:
        {
            /* Real-time BEMF GPIO read + sector context. Safe in all states;
             * use to diagnose phase-wiring and comparator polarity by hand-
             * rotating the motor and watching the three BEMF lines toggle. */
            extern volatile uint8_t v4_floatingPhase;
            extern volatile uint8_t v5_ptgExpectedComp;
            uint8_t probe[7];
            probe[0] = BEMF_A_GetValue();
            probe[1] = BEMF_B_GetValue();
            probe[2] = BEMF_C_GetValue();
            probe[3] = v4_floatingPhase;
            probe[4] = v5_ptgExpectedComp;
            probe[5] = (uint8_t)SectorPI_GetPhase();
            probe[6] = HAL_Capture_IsRisingZc() ? 1u : 0u;
            GSP_SendResponse(GSP_CMD_BEMF_PROBE, probe, sizeof(probe));
            break;
        }

        case GSP_CMD_ATA_DIAG:
        {
            /* Bring-up only: 8 raw register bytes from the ATA6847L.
             * Order matches HAL_ATA6847_ReadDiag():
             *   [0] DSR1   (GDU status — GDUS bit 2 = ready)
             *   [1] DSR2
             *   [2] SIR1   (fault summary — latched)
             *   [3] SIR2
             *   [4] SIR3
             *   [5] SIR4
             *   [6] SIR5
             *   [7] GOPMCR (current operating mode)
             * Skip while motor is running (SPI shared with start-path). */
            extern volatile uint8_t  gAta_LastDsr1AtNormal;
            extern volatile uint16_t gAta_LastGduAttempts;
            extern volatile uint8_t  gAta_LastGduResult;

            uint8_t diag[12];
            if (gV4State != ESC_IDLE && gV4State != ESC_FAULT) {
                uint8_t err = GSP_ERR_WRONG_STATE;
                GSP_SendResponse(GSP_CMD_ERROR, &err, 1);
            } else {
                HAL_ATA6847_ReadDiag(diag);
                /* Append bring-up instrumentation: what EnterGduNormal saw */
                diag[8]  = gAta_LastDsr1AtNormal;
                diag[9]  = (uint8_t)(gAta_LastGduAttempts & 0xFF);
                diag[10] = (uint8_t)(gAta_LastGduAttempts >> 8);
                diag[11] = gAta_LastGduResult;
                GSP_SendResponse(GSP_CMD_ATA_DIAG, diag, sizeof(diag));
            }
            break;
        }

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

    uint8_t snap[250]; /* 2-byte seq + 64-byte snapshot + 8-byte off-mid
                        * + 4-byte V5 PTG fire counter
                        * + 16-byte V5 PTG per-polarity counters
                        * + 16-byte V5.1 ADC post-ZC shadow counters
                        * + 2-byte V5.2 measurement-PI tracker (tMeasHR)
                        * + 4-byte sectorCount (full 32-bit, for rate diag)
                        * + 32-byte phase-current peak block (2026-04-21)
                        * + 10-byte elapsed-snapshot probe (2026-05-14, d[146..155])
                        * + 16-byte capture-layer comp tally (2026-05-14, d[156..171])
                        * + 4-byte PTG postscale skip counter (2026-05-15, d[172..175])
                        * + 24-byte per-sector hit counters (2026-05-15, d[176..199])
                        * + 12-byte per-sector total fires (2026-05-15, d[200..211])
                        * + 36-byte per-sector × per-phase BEMF comp=1 tally (2026-05-15, d[212..247]) */
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
    /* Duty% from the value the hardware actually wrote, not the upstream
     * commanded amplitude. g_pwmActualDuty is the post-clamp PG[123]DC
     * value and g_pwmPer is its denominator (LOOPTIME_TCY in normal mode,
     * sector-period in SP). Was previously `actualAmplitude * 100 / 32768`
     * which always showed 99% even when the per-200 (now per-100) clamp
     * pinned the gates at ~94-97%. */
    extern volatile uint16_t g_pwmActualDuty;
    extern volatile uint16_t g_pwmPer;
    extern volatile bool     g_blockCommActive;
    uint8_t dutyPct = 0;
    if (g_blockCommActive)
        dutyPct = 100;                          /* override path → solid ON */
    else if (g_pwmPer > 0U && t.actualAmplitude > 0U)
        dutyPct = (uint8_t)((uint32_t)g_pwmActualDuty * 100UL / g_pwmPer);
    d[6] = dutyPct;                          /* dutyPct */
    d[7] = t.commandEnabled ? 1 : 0;        /* zcSynced (repurpose as PI active) */

    /* Electrical (offset 8-17) — calibrated physical units.
     *   d[8..9]   vbus_mV   uint16   (0 .. ~52 V)
     *   d[10..11] ia_mA     int16    signed phase A current
     *   d[12..13] ib_mA     int16    signed phase B current
     *   d[14..15] ibus_mA   int16    signed DC bus current (direct from OA3)
     * All conversions happen in the ADC ISR using the Q8 constants in
     * garuda_config.h.  The host only divides by 1000 to display volts/amps;
     * it doesn't know the shunt, gain, or divider values. */
    {
        extern volatile uint16_t gV4Vbus_mV;
        extern volatile int16_t  gV4Ia_mA, gV4Ib_mA, gV4Ibus_mA;
        memcpy(&d[8],  &gV4Vbus_mV, 2);
        memcpy(&d[10], &gV4Ia_mA,   2);
        memcpy(&d[12], &gV4Ib_mA,   2);
        memcpy(&d[14], &gV4Ibus_mA, 2);
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
    d[37] = (t.spMode ? 1U : 0U) | (t.spRequest ? 2U : 0U) | (g_blockCommActive ? 4U : 0U);
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

    /* 2026-05-14: slots repurposed from V5 post-ZC shadow counters to the
     * per-polarity PI-feed accounting from sector_pi.c. Host pR%/pF% now
     * reflects the actual PI-feed success rate (not ISR-level matches).
     *   d[92]  = diagPiFedRising  (rising sector accepted into PI)
     *   d[96]  = diagPiMissRising (rising sector dropped/no-capture)
     *   d[100] = diagPiFedFalling (falling sector accepted into PI)
     *   d[104] = diagPiMissFalling(falling sector dropped/no-capture)
     * Host computes pR = fed/(fed+miss) per polarity. */
    {
        extern volatile uint32_t diagPiFedRising, diagPiFedFalling;
        extern volatile uint32_t diagPiMissRising, diagPiMissFalling;
        memcpy(&d[92],  &diagPiFedRising,   4);
        memcpy(&d[96],  &diagPiMissRising,  4);
        memcpy(&d[100], &diagPiFedFalling,  4);
        memcpy(&d[104], &diagPiMissFalling, 4);
    }

    /* V5.2 measurement-PI tracker. */
    memcpy(&d[108], &t.tMeasHR, 2);

    /* Full 32-bit sectorCount for host-side rate computation. */
    memcpy(&d[110], &t.sectorCount, 4);

    /* Phase + bus current peaks. Rolling window reset on snapshot read;
     * at-fault fields preserved across snapshots until motor restart
     * (valid==1). ibus is now a direct OA3 shunt read (see hal_adc.c)
     * — previously zero with a host-side max(|Ia|,|Ib|) fallback. */
    extern volatile int16_t gV4IaPkMax, gV4IaPkMin;
    extern volatile int16_t gV4IbPkMax, gV4IbPkMin;
    extern volatile int16_t gV4IbusPkMax, gV4IbusPkMin;
    extern volatile int16_t gV4IaAtFaultMax, gV4IaAtFaultMin;
    extern volatile int16_t gV4IbAtFaultMax, gV4IbAtFaultMin;
    extern volatile int16_t gV4IbusAtFaultMax, gV4IbusAtFaultMin;
    extern volatile int16_t gV4IaAtFaultInst, gV4IbAtFaultInst, gV4IbusAtFaultInst;
    extern volatile uint8_t gV4FaultSnapshotValid;
    extern volatile int16_t gV4IaRaw, gV4IbRaw, gV4IbusRaw;

    memcpy(&d[114], &gV4IaPkMax,        2);
    memcpy(&d[116], &gV4IaPkMin,        2);
    memcpy(&d[118], &gV4IbPkMax,        2);
    memcpy(&d[120], &gV4IbPkMin,        2);
    memcpy(&d[122], &gV4IbusPkMax,      2);
    memcpy(&d[124], &gV4IbusPkMin,      2);
    memcpy(&d[126], &gV4IaAtFaultMax,   2);
    memcpy(&d[128], &gV4IaAtFaultMin,   2);
    memcpy(&d[130], &gV4IbAtFaultMax,   2);
    memcpy(&d[132], &gV4IbAtFaultMin,   2);
    memcpy(&d[134], &gV4IbusAtFaultMax, 2);
    memcpy(&d[136], &gV4IbusAtFaultMin, 2);
    memcpy(&d[138], &gV4IaAtFaultInst,   2);
    memcpy(&d[140], &gV4IbAtFaultInst,   2);
    memcpy(&d[142], &gV4IbusAtFaultInst, 2);
    d[144] = gV4FaultSnapshotValid;
    d[145] = 0;  /* pad */

    /* 2026-05-14 mechanism probe: elapsed-snapshot per polarity (10 bytes).
     * Read by pot_capture.py when len(data) >= 156 to gate parsing.
     *   d[146] = diagElapsedAcceptRising  (last accepted elapsed, rising)
     *   d[148] = diagElapsedAcceptFalling (last accepted elapsed, falling)
     *   d[150] = diagElapsedRejectRising  (last rejected elapsed, rising)
     *   d[152] = diagElapsedRejectFalling (last rejected elapsed, falling)
     *   d[154] = diagFilterHRLast         (filterHR at last decision) */
    {
        extern volatile uint16_t diagElapsedAcceptRising,  diagElapsedAcceptFalling;
        extern volatile uint16_t diagElapsedRejectRising,  diagElapsedRejectFalling;
        extern volatile uint16_t diagFilterHRLast;
        memcpy(&d[146], &diagElapsedAcceptRising,  2);
        memcpy(&d[148], &diagElapsedAcceptFalling, 2);
        memcpy(&d[150], &diagElapsedRejectRising,  2);
        memcpy(&d[152], &diagElapsedRejectFalling, 2);
        memcpy(&d[154], &diagFilterHRLast,         2);
    }

    /* 2026-05-14 capture-layer probe: comp value × sector polarity, past
     * blanking. Tells us if rising-sector captures fail because comp
     * never leaves the pre-ZC state (physics asymmetry) or because the
     * accept logic discards a valid post-ZC sample (software bug). */
    {
        extern volatile uint32_t v4_compRising_High,  v4_compRising_Low;
        extern volatile uint32_t v4_compFalling_High, v4_compFalling_Low;
        memcpy(&d[156], &v4_compRising_High,  4);
        memcpy(&d[160], &v4_compRising_Low,   4);
        memcpy(&d[164], &v4_compFalling_High, 4);
        memcpy(&d[168], &v4_compFalling_Low,  4);
    }

    /* 2026-05-15 PTG postscale experiment: count of fires that bypassed
     * V4_ProcessBemfSample(). v5_ptgFires is total fires; effective
     * BEMF-sample rate = (ptgFires - ptgSkipped) / window. */
    {
        extern volatile uint32_t v5_ptgSkipped;
        memcpy(&d[172], &v5_ptgSkipped, 4);
    }

    /* Per-sector hit counters (2026-05-15). 6 × uint32 at d[176..199].
     * Tells whether all 6 commutation positions actually fire. If
     * hits[0]/[2]/[4] stay at 0 while [1]/[3]/[5] climb, position is
     * incrementing by 2 somewhere and explains why cR=0 / pR=0%. */
    {
        extern volatile uint32_t v4_sectorHits[6];
        memcpy(&d[176], (const void *)v4_sectorHits, 24);
    }

    /* Multi-phase BEMF tally (2026-05-15).
     *   d[200..211]: v4_bemfTallyTotal[6]     (6 × uint16)
     *   d[212..247]: v4_bemfTally[6][3]       (6 × 3 × uint16)
     * Host computes ratio comp=1 / total per (sector, phase) to spot
     * phase-mapping bugs.  Ratio at 0% or 100% on the supposed floating
     * phase = bug; ratio in (10..90)% = actually floating.
     *
     * 2026-05-15 (b): reset these tallies after copying so each frame
     * is a fresh delta over the ~50 ms inter-snapshot window. uint16
     * wraps at 65 536 and at 60 kHz PWM we hit the wrap inside one
     * sector's post-blanking window — without the reset, ratios are
     * meaningless. With the reset, max samples per window are well
     * inside uint16 range. */
    {
        extern volatile uint16_t v4_bemfTally[6][3];
        extern volatile uint16_t v4_bemfTallyTotal[6];
        memcpy(&d[200], (const void *)v4_bemfTallyTotal, 12);
        memcpy(&d[212], (const void *)v4_bemfTally,      36);
        memset((void *)v4_bemfTallyTotal, 0, sizeof(v4_bemfTallyTotal));
        memset((void *)v4_bemfTally,      0, sizeof(v4_bemfTally));
    }

    /* Stale-floatingPhase diagnostic (2026-05-15b). uint16 at d[248..249].
     * Per-window count of PTG fires (past blanking) where v4_floatingPhase
     * disagreed with commutationTable[v4_currentSector].floatingPhase.
     * Resolves the contradiction between the multi-phase tally and the
     * deglitched comp (preR≈100%). Reset alongside the tally. */
    {
        extern volatile uint16_t v4_fpStaleCount;
        memcpy(&d[248], (const void *)&v4_fpStaleCount, 2);
        v4_fpStaleCount = 0;
    }

    /* Reset rolling peaks for next 20 ms window. Seed with current
     * instantaneous sample so the window doesn't start at stale extrema.
     * Peaks track milliamps (same scale as the live mA values). */
    {
        extern volatile int16_t gV4Ia_mA, gV4Ib_mA, gV4Ibus_mA;
        gV4IaPkMax   = gV4IaPkMin   = gV4Ia_mA;
        gV4IbPkMax   = gV4IbPkMin   = gV4Ib_mA;
        gV4IbusPkMax = gV4IbusPkMin = gV4Ibus_mA;
    }

    GSP_SendResponse(GSP_CMD_TELEM_FRAME, snap, sizeof(snap));
}
#endif
