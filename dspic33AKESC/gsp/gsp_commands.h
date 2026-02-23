/**
 * @file gsp_commands.h
 *
 * @brief GSP v1 command IDs, error codes, and wire-format structures.
 *
 * All structs are little-endian packed (dsPIC33AK native byte order).
 * Wire sizes are verified with _Static_assert.
 *
 * Component: GSP
 */

#ifndef GSP_COMMANDS_H
#define GSP_COMMANDS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Firmware version (no existing macros — define here for GSP_INFO_T) */
#define GSP_FW_MAJOR    0
#define GSP_FW_MINOR    1
#define GSP_FW_PATCH    0

/* GSP protocol version */
#define GSP_PROTOCOL_VERSION  1

/* Board IDs */
#define GSP_BOARD_MCLV48V300W  0x0001

/* Command IDs */
typedef enum {
    /* Phase 0: telemetry */
    GSP_CMD_PING            = 0x00,
    GSP_CMD_GET_INFO        = 0x01,
    GSP_CMD_GET_SNAPSHOT    = 0x02,
    /* Phase 1: motor control */
    GSP_CMD_START_MOTOR     = 0x03,
    GSP_CMD_STOP_MOTOR      = 0x04,
    GSP_CMD_CLEAR_FAULT     = 0x05,
    GSP_CMD_SET_THROTTLE    = 0x06,
    GSP_CMD_SET_THROTTLE_SRC= 0x07,
    GSP_CMD_HEARTBEAT       = 0x08,
    /* Phase 1: parameter system */
    GSP_CMD_GET_PARAM       = 0x10,
    GSP_CMD_SET_PARAM       = 0x11,
    GSP_CMD_SAVE_CONFIG     = 0x12,
    GSP_CMD_LOAD_DEFAULTS   = 0x13,
    GSP_CMD_TELEM_START     = 0x14,
    GSP_CMD_TELEM_STOP      = 0x15,
    GSP_CMD_GET_PARAM_LIST  = 0x16,
    /* Unsolicited stream frame */
    GSP_CMD_TELEM_FRAME     = 0x80,
    /* Error response */
    GSP_CMD_ERROR           = 0xFF
} GSP_CMD_ID_T;

/* Error codes (payload of GSP_CMD_ERROR response) */
typedef enum {
    GSP_ERR_UNKNOWN_CMD      = 0x01,
    GSP_ERR_BAD_LENGTH       = 0x02,
    GSP_ERR_BUSY             = 0x03,
    GSP_ERR_WRONG_STATE      = 0x04,
    GSP_ERR_OUT_OF_RANGE     = 0x05,
    GSP_ERR_UNKNOWN_PARAM    = 0x06,
    GSP_ERR_CROSS_VALIDATION = 0x07,
    GSP_ERR_EEPROM_THROTTLED = 0x08
} GSP_ERR_CODE_T;

/* GSP_INFO_T — 20 bytes, returned by GET_INFO */
typedef struct __attribute__((packed)) {
    uint8_t  protocolVersion;
    uint8_t  fwMajor;
    uint8_t  fwMinor;
    uint8_t  fwPatch;
    uint16_t boardId;
    uint8_t  motorProfile;
    uint8_t  motorPolePairs;
    uint32_t featureFlags;
    uint32_t pwmFrequency;
    uint16_t maxErpm;
    uint16_t reserved;
} GSP_INFO_T;

_Static_assert(sizeof(GSP_INFO_T) == 20, "GSP_INFO_T wire size mismatch");

/* GSP_SNAPSHOT_T — 68 bytes, returned by GET_SNAPSHOT */
typedef struct __attribute__((packed)) {
    /* Core state (8B) */
    uint8_t  state;
    uint8_t  faultCode;
    uint8_t  currentStep;
    uint8_t  direction;
    uint16_t throttle;
    uint8_t  dutyPct;           /* duty as 0-100% (duty * 100 / LOOPTIME_TCY) */
    uint8_t  pad0;

    /* Bus (6B) */
    uint16_t vbusRaw;
    uint16_t ibusRaw;
    uint16_t ibusMax;

    /* BEMF/ZC (8B) */
    uint16_t bemfRaw;
    uint16_t zcThreshold;
    uint16_t stepPeriod;
    uint16_t goodZcCount;

    /* ZC flags (3B) */
    uint8_t  risingZcWorks;
    uint8_t  fallingZcWorks;
    uint8_t  zcSynced;

    /* ZC diag (4B) — pad to keep alignment clean */
    uint8_t  pad1;
    uint16_t zcConfirmedCount;
    uint16_t zcTimeoutForceCount;

    /* HWZC (18B) */
    uint8_t  hwzcEnabled;
    uint8_t  hwzcPhase;
    uint32_t hwzcTotalZcCount;
    uint32_t hwzcTotalMissCount;
    uint32_t hwzcStepPeriodHR;
    uint8_t  hwzcDbgLatchDisable;
    uint8_t  pad2;

    /* Morph (6B) */
    uint8_t  morphSubPhase;
    uint8_t  morphStep;
    uint16_t morphZcCount;
    uint16_t morphAlpha;

    /* Overcurrent (8B) */
    uint32_t clpciTripCount;
    uint32_t fpciTripCount;

    /* System (8B) */
    uint32_t systemTick;
    uint32_t uptimeSec;
} GSP_SNAPSHOT_T;

_Static_assert(sizeof(GSP_SNAPSHOT_T) == 68, "GSP_SNAPSHOT_T wire size mismatch");

/* Command handler prototype */
typedef void (*GSP_CMD_HANDLER_T)(const uint8_t *payload, uint8_t payloadLen);

/* Dispatch a received command (called by parser) */
void GSP_DispatchCommand(uint8_t cmdId, const uint8_t *payload, uint8_t payloadLen);

/* Telemetry streaming tick — called from GSP_Service() */
void GSP_TelemTick(void);

#ifdef __cplusplus
}
#endif

#endif /* GSP_COMMANDS_H */
