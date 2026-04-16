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
            uint8_t info[16];
            memset(info, 0, sizeof(info));
            info[0] = 2;                    /* protocol version */
            info[1] = MOTOR_PROFILE;
            info[2] = 4;                    /* firmware version = V4 */
            info[3] = 0;
            /* feature flags: just mark V4 */
            uint32_t f = 0x80000000UL;      /* bit 31 = V4 */
            memcpy(&info[4], &f, 4);
            GSP_SendResponse(GSP_CMD_GET_INFO, info, 8);
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

    /* Build a V3-compatible 48-byte snapshot so pot_capture.py can decode it.
     * Fields we can populate are filled; rest is zeroed. */
    V4_TELEM_T t;
    SectorPI_TelemGet(&t);

    uint8_t snap[50];  /* 2-byte seq + 48-byte snapshot */
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
    /* ibusRaw = 0 */
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

    GSP_SendResponse(GSP_CMD_TELEM_FRAME, snap, sizeof(snap));
}
#endif
