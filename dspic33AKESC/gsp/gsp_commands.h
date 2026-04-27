/**
 * @file gsp_commands.h
 *
 * @brief GSP v2 command IDs, error codes, and wire-format structures.
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

/* Firmware version — bump on meaningful changes so the logger
 * unambiguously shows what's running on the chip. */
#define GSP_FW_MAJOR    0
#define GSP_FW_MINOR    2
#define GSP_FW_PATCH    0

/* GSP protocol version — bump when GSP_INFO_T or other wire formats
 * change.  V3 = added 4-byte buildHash to GSP_INFO_T. */
#define GSP_PROTOCOL_VERSION  3

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
    /* Phase 1.5: profiles */
    GSP_CMD_LOAD_PROFILE    = 0x17,
    /* Auto-commissioning: measure Rs, Ls, λ_pm */
    GSP_CMD_AUTO_DETECT     = 0x20,
    /* Phase H: RX status + test injection */
    GSP_CMD_GET_RX_STATUS   = 0x26,
    GSP_CMD_RX_INJECT       = 0x27,
    /* Burst scope */
    GSP_CMD_SCOPE_ARM       = 0x30,
    GSP_CMD_SCOPE_STATUS    = 0x31,
    GSP_CMD_SCOPE_READ      = 0x32,
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

/* GSP_INFO_T — 24 bytes, returned by GET_INFO.
 *
 * V3 ADDED buildHash: a djb2 hash of __DATE__ " " __TIME__ computed at
 * firmware boot.  Every recompile produces a new buildHash, so the host
 * tool can unambiguously identify which binary is on the chip. */
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
    uint32_t maxErpm;           /* V2: widened from u16+reserved to u32 */
    uint32_t buildHash;         /* V3: hash of __DATE__ __TIME__ */
} GSP_INFO_T;

_Static_assert(sizeof(GSP_INFO_T) == 24, "GSP_INFO_T wire size mismatch");

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

    /* FOC telemetry (38B) — zero when FEATURE_FOC not enabled */
    float    focIdMeas;       /* D-axis current (A) — should be ~0 */
    float    focIqMeas;       /* Q-axis current (A) — torque */
    float    focTheta;        /* Commutation angle (rad) */
    float    focOmega;        /* Electrical speed (rad/s) */
    float    focVbus;         /* Bus voltage (V, float) */
    float    focIa;           /* Phase A current (A) */
    float    focIb;           /* Phase B current (A) */
    float    focThetaObs;     /* Observer angle (rad) */
    float    focVd;           /* D-axis voltage command (V) */
    float    focVq;           /* Q-axis voltage command (V) */
    /* Observer internals (16B) */
    float    focFluxAlpha;    /* Observer flux alpha state (V·s) */
    float    focFluxBeta;     /* Observer flux beta state (V·s) */
    float    focLambdaEst;    /* Adaptive flux linkage estimate (V·s/rad) */
    float    focObsGain;      /* Observer scheduled gain */
    /* PI controller internals (12B) */
    float    focPidDInteg;    /* D-axis PI integrator */
    float    focPidQInteg;    /* Q-axis PI integrator */
    float    focPidSpdInteg;  /* Speed PI integrator */
    /* Derived diagnostics (8B) */
    float    focModIndex;     /* Modulation index 0-1 */
    float    focObsConfidence;/* Observer confidence 0-1 */
    uint8_t  focSubState;     /* 0=idle, 1=armed, 2=align, 3=if, 4=cl */
    uint8_t  focPad;
    uint16_t focOffsetIa;     /* Calibrated ADC offset Ia */
    uint16_t focOffsetIb;     /* Calibrated ADC offset Ib */
    /* V4 observer diagnostics (22B) */
    float    smoResidual;     /* LP-filtered current estimation error (A) */
    float    pllInnovation;   /* LP-filtered PLL phase error (rad) */
    float    pllOmega;        /* Raw PLL speed estimate (rad/s) */
    float    omegaOl;         /* OL forced speed / CL filtered speed (rad/s) */
    uint16_t handoffCtr;      /* Handoff dwell counter (ticks) */
    uint8_t  smoObservable;   /* Observer health flag (0/1) */
    uint8_t  pad3;

    /* HWZC phantom-ZC rejection diagnostics (4B) */
    uint32_t hwzcNoiseReject; /* Cumulative ZCs rejected (off-time / interval-min gate) */

    /* 6-step phase-current monitor (24B) — raw ADC counts, bias ~2048,
     * ~93 counts/A. iaMin/ibMin are 0xFFFF until the first sample.
     *
     * Window fields (iaMax/iaMin/ibMax/ibMin) are reset atomically on each
     * snapshot read, so each telemetry row shows the peaks in the most
     * recent ~20 ms window.
     *
     * The ...AtFault fields are frozen at the first BOARD_PCI transition
     * of each CL run — they hold the pre-trip 20 ms window envelope so we
     * can see what the phase currents actually did right before the board
     * latched the fault. */
    uint16_t iaRaw;
    uint16_t ibRaw;
    uint16_t iaMax;           /* max in last window (reset each snapshot) */
    uint16_t iaMin;
    uint16_t ibMax;
    uint16_t ibMin;
    uint16_t iaAtFault;       /* Ia sample at moment of first BOARD_PCI */
    uint16_t ibAtFault;
    uint16_t iaMaxAtFault;    /* 20 ms-window Ia max captured at fault */
    uint16_t iaMinAtFault;
    uint16_t ibMaxAtFault;
    uint16_t ibMinAtFault;
    /* Bus-current (OA3/M1_IBUS) windowed + at-fault tracking (10B) */
    uint16_t ibusWinMax;
    uint16_t ibusWinMin;
    uint16_t ibusAtFault;
    uint16_t ibusMaxAtFault;
    uint16_t ibusMinAtFault;
} GSP_SNAPSHOT_T;

_Static_assert(sizeof(GSP_SNAPSHOT_T) == 208, "GSP_SNAPSHOT_T wire size mismatch");

/* GSP_RX_STATUS_T — 12 bytes, returned by GET_RX_STATUS */
typedef struct __attribute__((packed)) {
    uint8_t  linkState;
    uint8_t  protocol;
    uint8_t  dshotRate;
    uint8_t  pad0;
    uint16_t throttle;
    uint16_t pulseUs;
    uint16_t crcErrors;
    uint16_t droppedFrames;
} GSP_RX_STATUS_T;

_Static_assert(sizeof(GSP_RX_STATUS_T) == 12, "RX status wire size");

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
