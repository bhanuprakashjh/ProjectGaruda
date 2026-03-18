/**
 * @file gsp_commands.h
 * @brief GSP v2 command IDs and CK board wire-format structures.
 *
 * Protocol-compatible with dsPIC33AK board. Same packet format, CRC,
 * and command IDs. Different snapshot struct (CK 6-step vs AK FOC).
 * Board detection via boardId field in GSP_INFO_T.
 */

#ifndef GSP_COMMANDS_H
#define GSP_COMMANDS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Firmware version */
#define GSP_FW_MAJOR    1
#define GSP_FW_MINOR    0
#define GSP_FW_PATCH    0

/* GSP protocol version (same as AK board) */
#define GSP_PROTOCOL_VERSION  2

/* Board IDs — GUI uses this to select CK vs AK UI */
#define GSP_BOARD_MCLV48V300W  0x0001   /* dsPIC33AK + MCLV-48V-300W */
#define GSP_BOARD_EV43F54A     0x0002   /* dsPIC33CK + ATA6847 (this board) */

/* Command IDs — same as AK board for protocol compatibility */
typedef enum {
    GSP_CMD_PING            = 0x00,
    GSP_CMD_GET_INFO        = 0x01,
    GSP_CMD_GET_SNAPSHOT    = 0x02,
    GSP_CMD_START_MOTOR     = 0x03,
    GSP_CMD_STOP_MOTOR      = 0x04,
    GSP_CMD_CLEAR_FAULT     = 0x05,
    GSP_CMD_SET_THROTTLE    = 0x06,
    GSP_CMD_HEARTBEAT       = 0x08,
    GSP_CMD_GET_PARAM       = 0x10,
    GSP_CMD_SET_PARAM       = 0x11,
    GSP_CMD_SAVE_CONFIG     = 0x12,
    GSP_CMD_LOAD_DEFAULTS   = 0x13,
    GSP_CMD_TELEM_START     = 0x14,
    GSP_CMD_TELEM_STOP      = 0x15,
    GSP_CMD_TELEM_FRAME     = 0x80,
    GSP_CMD_ERROR           = 0xFF
} GSP_CMD_ID_T;

/* Error codes */
typedef enum {
    GSP_ERR_UNKNOWN_CMD      = 0x01,
    GSP_ERR_BAD_LENGTH       = 0x02,
    GSP_ERR_BUSY             = 0x03,
    GSP_ERR_WRONG_STATE      = 0x04,
    GSP_ERR_OUT_OF_RANGE     = 0x05,
} GSP_ERR_CODE_T;

/* GSP_INFO_T — 20 bytes, same as AK board (protocol-compatible) */
typedef struct __attribute__((packed)) {
    uint8_t  protocolVersion;
    uint8_t  fwMajor;
    uint8_t  fwMinor;
    uint8_t  fwPatch;
    uint16_t boardId;           /* 0x0002 for CK board */
    uint8_t  motorProfile;      /* 0=Hurst, 1=A2212 */
    uint8_t  motorPolePairs;
    uint32_t featureFlags;      /* CK-specific feature bits */
    uint32_t pwmFrequency;
    uint32_t maxErpm;
} GSP_INFO_T;

/* CK Feature flag bits */
#define GSP_FEATURE_IC_ZC       (1UL << 0)   /* SCCP1 fast poll ZC */
#define GSP_FEATURE_VBUS_FAULT  (1UL << 1)   /* Vbus OV/UV monitoring */
#define GSP_FEATURE_DESYNC_REC  (1UL << 2)   /* Desync auto-recovery */
#define GSP_FEATURE_DUTY_SLEW   (1UL << 3)   /* Asymmetric duty slew */
#define GSP_FEATURE_TIM_ADVANCE (1UL << 4)   /* Linear timing advance */
#define GSP_FEATURE_ATA6847     (1UL << 25)  /* ATA6847 gate driver */
#define GSP_FEATURE_ILIM_HW     (1UL << 26)  /* Hardware current limit */
#define GSP_FEATURE_CURRENT_SNS (1UL << 27)  /* Phase current sensing */
#define GSP_FEATURE_GSP         (1UL << 16)  /* GSP protocol active */

/**
 * GSP_CK_SNAPSHOT_T — 48 bytes, CK board telemetry snapshot.
 *
 * Different from AK board's 170-byte snapshot (FOC fields).
 * GUI detects CK board via boardId=0x0002 and uses CK decoder.
 */
typedef struct __attribute__((packed)) {
    /* Core state (8B) */
    uint8_t  state;             /* ESC_STATE_T */
    uint8_t  faultCode;         /* FAULT_CODE_T */
    uint8_t  currentStep;       /* 0-5 */
    uint8_t  ataStatus;         /* SIR1 summary + ILIM flag */
    uint16_t potRaw;            /* Pot ADC (speed reference) */
    uint8_t  dutyPct;           /* 0-100% */
    uint8_t  zcSynced;          /* 1=SYN, 0=--- */

    /* Electrical (12B) */
    uint16_t vbusRaw;           /* Bus voltage ADC */
    int16_t  iaRaw;             /* Phase A current (signed, fractional) */
    int16_t  ibRaw;             /* Phase B current (signed, fractional) */
    int16_t  ibusRaw;           /* Reconstructed bus current (abs value) */
    uint16_t duty;              /* Raw duty count */
    uint16_t pad0;

    /* Speed/timing (12B) */
    uint16_t stepPeriod;        /* Timer1 ticks */
    uint16_t stepPeriodHR;      /* SCCP4 HR ticks (640ns) */
    uint16_t eRpm;              /* Computed eRPM (capped 65535) */
    uint16_t goodZcCount;
    uint16_t zcInterval;
    uint16_t prevZcInterval;

    /* ZC diagnostics (8B) */
    uint16_t icAccepted;        /* Fast poll accepted ZCs */
    uint16_t icFalse;           /* Rejected ZCs */
    uint8_t  filterLevel;       /* Current deglitch FL */
    uint8_t  missedSteps;       /* consecutiveMissedSteps */
    uint8_t  forcedSteps;       /* stepsSinceLastZc */
    uint8_t  ilimActive;        /* ATA6847 ILIM chopping */

    /* System (8B) */
    uint32_t systemTick;        /* 1ms counter */
    uint32_t uptimeSec;
} GSP_CK_SNAPSHOT_T;

/* XC16 doesn't support _Static_assert. Verify size at compile time:
 * sizeof(GSP_CK_SNAPSHOT_T) should be 48 bytes. */

/* Command handler prototype */
typedef void (*GSP_CMD_HANDLER_T)(const uint8_t *payload, uint8_t payloadLen);

void GSP_DispatchCommand(uint8_t cmdId, const uint8_t *payload, uint8_t payloadLen);
void GSP_TelemTick(void);

#ifdef __cplusplus
}
#endif

#endif /* GSP_COMMANDS_H */
