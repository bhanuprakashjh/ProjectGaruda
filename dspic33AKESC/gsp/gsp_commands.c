/**
 * @file gsp_commands.c
 *
 * @brief GSP v1 command handlers and dispatch table.
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
#include "garuda_calc_params.h"

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
    return f;
}

/* ── Handlers ────────────────────────────────────────────────────────── */

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
    info.motorProfile    = MOTOR_PROFILE;
    info.motorPolePairs  = MOTOR_POLE_PAIRS;
    info.featureFlags    = BuildFeatureFlags();
    info.pwmFrequency    = PWMFREQUENCY_HZ;
    info.maxErpm         = (MAX_CLOSED_LOOP_ERPM > 65535)
                           ? 65535 : (uint16_t)MAX_CLOSED_LOOP_ERPM;
    info.reserved        = 0;

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

static void SendError(uint8_t errCode)
{
    GSP_SendResponse(GSP_CMD_ERROR, &errCode, 1);
}

/* ── Dispatch ────────────────────────────────────────────────────────── */

typedef struct {
    uint8_t           cmdId;
    uint8_t           expectedPayloadLen;  /* 0xFF = any length */
    GSP_CMD_HANDLER_T handler;
} CMD_ENTRY_T;

static const CMD_ENTRY_T cmdTable[] = {
    { GSP_CMD_PING,         0, HandlePing        },
    { GSP_CMD_GET_INFO,     0, HandleGetInfo     },
    { GSP_CMD_GET_SNAPSHOT, 0, HandleGetSnapshot },
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
