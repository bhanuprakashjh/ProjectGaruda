/**
 * @file gsp_commands.c
 *
 * @brief GSP v2 command handlers, dispatch table, and telemetry streaming.
 *
 * Phase 0: PING, GET_INFO, GET_SNAPSHOT
 * Phase 1: Motor control (START/STOP/CLEAR_FAULT/SET_THROTTLE/SET_THROTTLE_SRC/HEARTBEAT)
 *          Parameter system (GET/SET_PARAM, SAVE_CONFIG, LOAD_DEFAULTS, GET_PARAM_LIST)
 *          Telemetry streaming (TELEM_START/STOP, TELEM_FRAME)
 * Phase 1.5: LOAD_PROFILE, paginated V2 GET_PARAM_LIST, u32 maxErpm
 *
 * Component: GSP
 */

#include "garuda_config.h"

#if FEATURE_GSP

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "gsp_commands.h"
#include "gsp.h"
#include "gsp_snapshot.h"
#include "gsp_params.h"
#include "garuda_calc_params.h"
#include "garuda_types.h"
#include "garuda_service.h"
#if FEATURE_EEPROM_V2
#include "hal/eeprom.h"
#endif

/* ── Telemetry streaming state ──────────────────────────────────────── */

static bool     telemStreaming;
static uint16_t telemSeq;
static uint32_t telemIntervalMs;
static uint32_t lastTelemTick;

/* ── Feature flags bitmask (same order as garuda_config.h) ───────────── */

static uint32_t BuildFeatureFlags(void)
{
    uint32_t f = 0;
    if (FEATURE_BEMF_CLOSED_LOOP) f |= (1UL << 0);
    if (FEATURE_VBUS_FAULT)       f |= (1UL << 1);
    if (FEATURE_DESYNC_RECOVERY)  f |= (1UL << 2);
    if (FEATURE_DUTY_SLEW)        f |= (1UL << 3);
    if (FEATURE_TIMING_ADVANCE)   f |= (1UL << 4);
    if (FEATURE_DYNAMIC_BLANKING) f |= (1UL << 5);
    if (FEATURE_VBUS_SAG_LIMIT)   f |= (1UL << 6);
    if (FEATURE_BEMF_INTEGRATION) f |= (1UL << 7);
    if (FEATURE_SINE_STARTUP)     f |= (1UL << 8);
    if (FEATURE_ADC_CMP_ZC)       f |= (1UL << 9);
    if (FEATURE_HW_OVERCURRENT)   f |= (1UL << 10);
    if (FEATURE_LEARN_MODULES)    f |= (1UL << 11);
    if (FEATURE_ADAPTATION)       f |= (1UL << 12);
    if (FEATURE_COMMISSION)       f |= (1UL << 13);
    if (FEATURE_EEPROM_V2)        f |= (1UL << 14);
    if (FEATURE_X2CSCOPE)         f |= (1UL << 15);
    if (FEATURE_GSP)              f |= (1UL << 16);
    if (OC_CLPCI_ENABLE)          f |= (1UL << 17);
    if (FEATURE_PRESYNC_RAMP)     f |= (1UL << 18);
    /* Phase H: RX input features (bits 19-22) */
    if (FEATURE_ADC_POT)         f |= (1UL << 19);
    if (FEATURE_RX_PWM)          f |= (1UL << 20);
    if (FEATURE_RX_DSHOT)        f |= (1UL << 21);
    if (FEATURE_RX_AUTO)         f |= (1UL << 22);
    return f;
}

/* ── Error helper ────────────────────────────────────────────────────── */

static void SendError(uint8_t errCode)
{
    GSP_SendResponse(GSP_CMD_ERROR, &errCode, 1);
}

/* ── Phase 0 handlers ────────────────────────────────────────────────── */

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
    info.boardId         = GSP_BOARD_MCLV48V300W;
    info.motorProfile    = GSP_ParamsGetActiveProfile();
    info.motorPolePairs  = gspParams.motorPolePairs;
    info.featureFlags    = BuildFeatureFlags();
    info.pwmFrequency    = PWMFREQUENCY_HZ;
    info.maxErpm         = gspParams.maxClosedLoopErpm;

    GSP_SendResponse(GSP_CMD_GET_INFO, (const uint8_t *)&info, sizeof(info));
}

static void HandleGetSnapshot(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payload;
    (void)payloadLen;

    static GSP_SNAPSHOT_T snapshot;
    GSP_CaptureSnapshot(&snapshot);
    GSP_SendResponse(GSP_CMD_GET_SNAPSHOT, (const uint8_t *)&snapshot,
                     sizeof(snapshot));
}

/* ── Phase 1: Motor control handlers ────────────────────────────────── */

static void HandleStartMotor(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payload;
    (void)payloadLen;

    if (garudaData.state != ESC_IDLE) {
        SendError(GSP_ERR_WRONG_STATE);
        return;
    }
    garudaData.gspStartIntent = true;
    GSP_SendResponse(GSP_CMD_START_MOTOR, NULL, 0);
}

static void HandleStopMotor(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payload;
    (void)payloadLen;

    garudaData.gspStopIntent = true;
    GSP_SendResponse(GSP_CMD_STOP_MOTOR, NULL, 0);
}

static void HandleClearFault(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payload;
    (void)payloadLen;

    if (garudaData.state != ESC_FAULT) {
        SendError(GSP_ERR_WRONG_STATE);
        return;
    }
    garudaData.gspFaultClearIntent = true;
    GSP_SendResponse(GSP_CMD_CLEAR_FAULT, NULL, 0);
}

static void HandleSetThrottle(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payloadLen;

    if (garudaData.throttleSource != THROTTLE_SRC_GSP) {
        SendError(GSP_ERR_WRONG_STATE);
        return;
    }

    uint16_t val;
    memcpy(&val, payload, 2);
    if (val > 2000) {
        SendError(GSP_ERR_OUT_OF_RANGE);
        return;
    }

    garudaData.gspThrottle = val;
    GSP_SendResponse(GSP_CMD_SET_THROTTLE, NULL, 0);
}

static void HandleSetThrottleSrc(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payloadLen;

    uint8_t src = payload[0];

    /* ALL transitions are IDLE-only (Finding 39) */
    if (garudaData.state != ESC_IDLE) {
        SendError(GSP_ERR_WRONG_STATE);
        return;
    }

    /* Validate source and check feature availability */
    switch ((THROTTLE_SOURCE_T)src) {
        case THROTTLE_SRC_ADC:
#if FEATURE_ADC_POT
            garudaData.throttleSource = THROTTLE_SRC_ADC;
            break;
#else
            SendError(GSP_ERR_OUT_OF_RANGE);
            return;
#endif

        case THROTTLE_SRC_GSP:
#if FEATURE_GSP
            garudaData.throttleSource = THROTTLE_SRC_GSP;
            garudaData.gspThrottle = 0;
            garudaData.lastGspPacketTick = garudaData.systemTick;
            break;
#else
            SendError(GSP_ERR_OUT_OF_RANGE);
            return;
#endif

        case THROTTLE_SRC_PWM:
#if FEATURE_RX_PWM
            garudaData.throttleSource = THROTTLE_SRC_PWM;
            break;
#else
            SendError(GSP_ERR_OUT_OF_RANGE);
            return;
#endif

        case THROTTLE_SRC_DSHOT:
#if FEATURE_RX_DSHOT
            garudaData.throttleSource = THROTTLE_SRC_DSHOT;
            break;
#else
            SendError(GSP_ERR_OUT_OF_RANGE);
            return;
#endif

        case THROTTLE_SRC_AUTO:
#if FEATURE_RX_AUTO
            garudaData.throttleSource = THROTTLE_SRC_AUTO;
            break;
#else
            SendError(GSP_ERR_OUT_OF_RANGE);
            return;
#endif

        default:
            SendError(GSP_ERR_OUT_OF_RANGE);
            return;
    }

    GSP_SendResponse(GSP_CMD_SET_THROTTLE_SRC, NULL, 0);
}

static void HandleHeartbeat(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payload;
    (void)payloadLen;
    GSP_SendResponse(GSP_CMD_HEARTBEAT, NULL, 0);
}

/* ── Phase 1: Parameter system handlers ─────────────────────────────── */

static void HandleGetParam(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payloadLen;

    uint16_t paramId;
    memcpy(&paramId, payload, 2);

    uint32_t value;
    if (!GSP_ParamGet(paramId, &value)) {
        SendError(GSP_ERR_UNKNOWN_PARAM);
        return;
    }

    uint8_t resp[6];
    memcpy(&resp[0], &paramId, 2);
    memcpy(&resp[2], &value, 4);
    GSP_SendResponse(GSP_CMD_GET_PARAM, resp, 6);
}

static void HandleSetParam(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payloadLen;

    if (garudaData.state != ESC_IDLE) {
        SendError(GSP_ERR_WRONG_STATE);
        return;
    }

    uint16_t paramId;
    uint32_t value;
    memcpy(&paramId, payload, 2);
    memcpy(&value, payload + 2, 4);

    PARAM_RESULT_T result = GSP_ParamSet(paramId, value);

    switch (result) {
    case PARAM_OK: {
        uint8_t resp[6];
        memcpy(&resp[0], &paramId, 2);
        memcpy(&resp[2], &value, 4);
        GSP_SendResponse(GSP_CMD_SET_PARAM, resp, 6);
        break;
    }
    case PARAM_ERR_UNKNOWN_ID:
        SendError(GSP_ERR_UNKNOWN_PARAM);
        break;
    case PARAM_ERR_OUT_OF_RANGE:
        SendError(GSP_ERR_OUT_OF_RANGE);
        break;
    case PARAM_ERR_CROSS_VALIDATION:
        SendError(GSP_ERR_CROSS_VALIDATION);
        break;
    }
}

static void HandleSaveConfig(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payload;
    (void)payloadLen;

#if FEATURE_EEPROM_V2
    uint32_t remaining = EEPROM_GetCooldownRemainingMs(garudaData.systemTick);
    if (remaining > 0) {
        uint8_t resp[2];
        resp[0] = GSP_ERR_EEPROM_THROTTLED;
        resp[1] = (uint8_t)((remaining + 999) / 1000);
        GSP_SendResponse(GSP_CMD_ERROR, resp, 2);
        return;
    }

    GARUDA_CONFIG_T cfg;
    EEPROM_LoadConfig(&cfg);
    GSP_ParamsSaveToConfig(&cfg);

    if (!EEPROM_SaveConfig(&cfg, garudaData.systemTick)) {
        SendError(GSP_ERR_BUSY);
        return;
    }

    GSP_SendResponse(GSP_CMD_SAVE_CONFIG, NULL, 0);
#else
    GSP_SendResponse(GSP_CMD_SAVE_CONFIG, NULL, 0);
#endif
}

static void HandleLoadDefaults(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payload;
    (void)payloadLen;

    if (garudaData.state != ESC_IDLE) {
        SendError(GSP_ERR_WRONG_STATE);
        return;
    }

    uint8_t profile = GSP_ParamsGetActiveProfile();

    if (profile < GSP_PROFILE_COUNT) {
        /* Built-in profile: reload from profile defaults */
        GSP_ParamsLoadProfile(profile);
    } else {
        /* Custom profile: reload from EEPROM V2 if available */
#if FEATURE_EEPROM_V2
        GARUDA_CONFIG_T cfg;
        EEPROM_LoadConfig(&cfg);
        /* Re-init defaults then overlay from EEPROM */
        GSP_ParamsInitDefaults();
        GSP_ParamsLoadFromConfig(&cfg);
        GSP_RecomputeDerived();
#else
        /* No EEPROM — can't restore custom profile */
        SendError(GSP_ERR_WRONG_STATE);
        return;
#endif
    }

    GSP_SendResponse(GSP_CMD_LOAD_DEFAULTS, NULL, 0);
}

/* ── Phase 1.5: V2 GET_PARAM_LIST (paginated, u32 min/max) ──────────── */

static void HandleGetParamList(const uint8_t *payload, uint8_t payloadLen)
{
    if (payloadLen > 1) {
        SendError(GSP_ERR_BAD_LENGTH);
        return;
    }

    uint8_t startIndex = (payloadLen == 1) ? payload[0] : 0;
    uint8_t totalCount = GSP_ParamGetCount();

    if (startIndex >= totalCount) {
        SendError(GSP_ERR_OUT_OF_RANGE);
        return;
    }

    /* 12 bytes/entry: id(u16) type(u8) group(u8) min(u32) max(u32)
     * 3-byte header + max 20 entries × 12 = 243 bytes (< 249 payload max) */
    uint8_t buf[3 + 20 * 12];
    uint8_t entryCount = 0;
    uint8_t idx = 3; /* skip header */

    for (uint8_t i = startIndex; i < totalCount && entryCount < 20; i++) {
        const PARAM_DESCRIPTOR_T *d = GSP_ParamGetDescriptor(i);
        if (!d) break;
        memcpy(&buf[idx], &d->id, 2);      idx += 2;
        buf[idx++] = d->type;
        buf[idx++] = d->group;
        memcpy(&buf[idx], &d->minVal, 4);  idx += 4;
        memcpy(&buf[idx], &d->maxVal, 4);  idx += 4;
        entryCount++;
    }

    buf[0] = totalCount;
    buf[1] = startIndex;
    buf[2] = entryCount;
    GSP_SendResponse(GSP_CMD_GET_PARAM_LIST, buf, idx);
}

/* ── Phase 1.5: LOAD_PROFILE ────────────────────────────────────────── */

static void HandleLoadProfile(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payloadLen;

    if (garudaData.state != ESC_IDLE) {
        SendError(GSP_ERR_WRONG_STATE);
        return;
    }

    uint8_t profileId = payload[0];

    if (!GSP_ParamsLoadProfile(profileId)) {
        SendError(GSP_ERR_OUT_OF_RANGE);
        return;
    }

    /* Respond with ACK + profile ID */
    uint8_t resp = profileId;
    GSP_SendResponse(GSP_CMD_LOAD_PROFILE, &resp, 1);
}

/* ── Phase H: RX status handler ──────────────────────────────────────── */

static void HandleGetRxStatus(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payload;
    (void)payloadLen;

    GSP_RX_STATUS_T status;
    memset(&status, 0, sizeof(status));

    status.linkState    = (uint8_t)garudaData.rxLinkState;
    status.protocol     = (uint8_t)garudaData.rxProtocol;
    status.dshotRate    = garudaData.rxDshotRate;
    status.throttle     = garudaData.throttle;
    status.pulseUs      = garudaData.rxPulseUs;
    status.crcErrors    = garudaData.rxCrcErrors;
    status.droppedFrames = garudaData.rxDroppedFrames;

    GSP_SendResponse(GSP_CMD_GET_RX_STATUS, (const uint8_t *)&status,
                     sizeof(status));
}

/* ── Phase 1: Telemetry streaming ────────────────────────────────────── */

static void HandleTelemStart(const uint8_t *payload, uint8_t payloadLen)
{
    (void)payloadLen;

    uint8_t rateHz = payload[0];
    if (rateHz < 10) rateHz = 10;
    if (rateHz > 100) rateHz = 100;

    telemIntervalMs = 1000 / rateHz;
    telemStreaming = true;
    lastTelemTick = garudaData.systemTick;
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

    uint32_t now = garudaData.systemTick;
    if ((now - lastTelemTick) < telemIntervalMs)
        return;

    lastTelemTick = now;
    telemSeq++;

    uint8_t buf[70];
    buf[0] = (uint8_t)(telemSeq & 0xFF);
    buf[1] = (uint8_t)(telemSeq >> 8);
    GSP_CaptureSnapshot((GSP_SNAPSHOT_T *)&buf[2]);
    GSP_SendResponse(GSP_CMD_TELEM_FRAME, buf, 70);
}

/* ── Dispatch table ──────────────────────────────────────────────────── */

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
    { GSP_CMD_SET_THROTTLE,    2, HandleSetThrottle    },
    { GSP_CMD_SET_THROTTLE_SRC,1, HandleSetThrottleSrc },
    { GSP_CMD_HEARTBEAT,       0, HandleHeartbeat      },
    /* Phase 1: params */
    { GSP_CMD_GET_PARAM,       2, HandleGetParam       },
    { GSP_CMD_SET_PARAM,       6, HandleSetParam       },
    { GSP_CMD_SAVE_CONFIG,     0, HandleSaveConfig     },
    { GSP_CMD_LOAD_DEFAULTS,   0, HandleLoadDefaults   },
    { GSP_CMD_TELEM_START,     1, HandleTelemStart     },
    { GSP_CMD_TELEM_STOP,      0, HandleTelemStop      },
    { GSP_CMD_GET_PARAM_LIST,  0xFF, HandleGetParamList },
    /* Phase 1.5: profiles */
    { GSP_CMD_LOAD_PROFILE,    1, HandleLoadProfile    },
    /* Phase H: RX status */
    { GSP_CMD_GET_RX_STATUS,   0, HandleGetRxStatus    },
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
