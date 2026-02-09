/**
 * @file garuda_types.h
 *
 * @brief Core data structures and enumerations for Project Garuda ESC firmware.
 *
 * Component: TYPES
 */

#ifndef GARUDA_TYPES_H
#define GARUDA_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include "garuda_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ESC state machine states */
typedef enum
{
    ESC_IDLE = 0,
    ESC_ARMED,
    ESC_ALIGN,
    ESC_OL_RAMP,
    ESC_CLOSED_LOOP,
    ESC_BRAKING,
    ESC_FAULT
} ESC_STATE_T;

/* Phase output states for 6-step commutation */
typedef enum
{
    PHASE_PWM_ACTIVE = 0,   /* PWM driven (H-side modulated, L-side complementary) */
    PHASE_LOW,              /* Forced LOW (both H and L off, L pulled low) */
    PHASE_FLOAT             /* Floating (both H and L off via override) */
} PHASE_STATE_T;

/* Commutation step descriptor */
typedef struct
{
    PHASE_STATE_T phaseA;
    PHASE_STATE_T phaseB;
    PHASE_STATE_T phaseC;
    uint8_t floatingPhase;  /* 0=A, 1=B, 2=C */
    int8_t zcPolarity;      /* +1 = rising ZC, -1 = falling ZC */
} COMMUTATION_STEP_T;

/* BEMF sensing state (stubbed for Phase 1, filled in Phase 2) */
typedef struct
{
    uint16_t bemfRaw;
    uint16_t vbusHalf;
    bool zeroCrossDetected;
} BEMF_STATE_T;

/* Commutation timing state (stubbed for Phase 1, filled in Phase 2) */
typedef struct
{
    uint32_t stepPeriod;        /* current commutation step period in timer counts */
    uint32_t lastZcTimestamp;
    uint32_t zcInterval;
    uint16_t filterCount;
} TIMING_STATE_T;

/* Telemetry fault flag bitmasks */
#define TELEM_FLAG_ZC_MISSED       0x01
#define TELEM_FLAG_ZC_TIMEOUT      0x02
#define TELEM_FLAG_FORCED_COMM     0x04
#define TELEM_FLAG_DESYNC_RECOVERY 0x08
#define TELEM_FLAG_FALSE_CROSS     0x10  /* ZC fired but polarity wrong (noise) */

/* Telemetry sample — ring buffer element (ISR → main loop) */
typedef struct __attribute__((packed))
{
    uint32_t timestamp;         /* systemTick (1ms) */
    uint8_t  step;              /* commutation step 0-5 */
    int8_t   zcPolarity;        /* +1=rising, -1=falling, 0=missed */
    uint16_t zcLatencyTicks;    /* Timer1 ticks from commutation to ZC */
    uint16_t duty;              /* PWM duty (lower 16 bits) */
    uint16_t vbusRaw;           /* DC bus voltage raw ADC */
    uint8_t  faultFlags;        /* bitmask: TELEM_FLAG_* */
} TELEM_SAMPLE_T;              /* 13 bytes packed */

/* ZC quality metrics (computed from telemetry window) */
typedef struct
{
    /* Live accumulation counters (reset each window) */
    uint16_t zcMissCount;
    uint16_t zcTotalCount;
    uint16_t zcTimeoutCount;
    uint16_t falseCrossCount;
    uint16_t desyncRecoveries;
    /* Computed at end of each window (persist across resets) */
    uint16_t zcMissRateQ8;      /* Q8.8 fixed-point */
    uint16_t falseRateQ8;       /* Q8.8 */
    uint16_t timeoutRateQ8;     /* Q8.8 */
    uint16_t zcJitterQ8;        /* ZC latency std dev, Q8.8 */
    uint8_t  confidenceScore;   /* 0-255 */
    uint16_t lastWindowTotal;   /* zcTotalCount from last completed window */
    uint32_t windowStartTick;
    uint16_t windowSizeTicks;
} QUALITY_METRICS_T;

/* Adaptation action types */
typedef enum
{
    ADAPT_NO_CHANGE = 0,
    ADAPT_TIMING_INCREASE,
    ADAPT_TIMING_DECREASE,
    ADAPT_STARTUP_DUTY_UP,
    ADAPT_STARTUP_DUTY_DOWN,
    ADAPT_RAMP_ACCEL_UP,
    ADAPT_RAMP_ACCEL_DOWN,
    ADAPT_ROLLBACK
} ADAPT_ACTION_T;

/* Bounded adaptation parameters */
typedef struct
{
    uint8_t  timingAdvanceDeg;          /* 0-30 active */
    uint16_t startupDutyPercent;        /* Q8.8 */
    uint16_t rampAccelErpmPerS;
    uint16_t alignTimeMs;
    uint8_t  timingAdvanceMin;          /* envelope min */
    uint8_t  timingAdvanceMax;          /* envelope max */
    uint16_t startupDutyMin;
    uint16_t startupDutyMax;
    uint8_t  lkgTimingAdvanceDeg;       /* last-known-good */
    uint16_t lkgStartupDutyPercent;
    uint16_t lkgRampAccelErpmPerS;
    uint16_t lkgAlignTimeMs;
    uint8_t  rollbackCount;
    uint8_t  consecutiveFailures;
    bool     adaptationLocked;
} ADAPT_PARAMS_T;

/* Self-commissioning state machine states */
typedef enum
{
    COMM_IDLE = 0,
    COMM_STATIC_R,
    COMM_STATIC_L,
    COMM_DYNAMIC_KV,
    COMM_DYNAMIC_POLES,
    COMM_DYNAMIC_TIMING,
    COMM_VALIDATE,
    COMM_COMMIT,
    COMM_COMPLETE,
    COMM_ERROR
} COMMISSION_STATE_T;

/* Self-commissioning data */
typedef struct
{
    COMMISSION_STATE_T state;
    COMMISSION_STATE_T prevState;
    uint16_t phaseResistanceMilliOhm;
    uint16_t phaseInductanceMicroH;
    uint16_t motorKv;
    uint8_t  detectedPolePairs;
    uint8_t  optimalTimingAdvDeg;
    uint16_t sampleCount;
    uint16_t targetSamples;
    uint32_t accumulator;
    uint32_t stateEntryTick;
    uint16_t timeoutMs;
    uint8_t  validationConfidence;
    bool     validationPassed;
} COMMISSION_DATA_T;

/* Health sub-scores and composite */
typedef struct
{
    uint8_t  bearingScore;      /* vibration indicator (ZC jitter vs baseline) */
    uint8_t  balanceScore;      /* step timing asymmetry */
    uint8_t  connectionScore;   /* resistance drift */
    uint8_t  thermalScore;      /* temperature trend */
    uint8_t  electricalScore;   /* Vbus ripple / current anomalies */
    uint8_t  compositeHealth;   /* weighted average */
    int8_t   trend;             /* +1/0/-1 */
    uint16_t operatingHours;
    uint16_t baselineJitterQ8;
    uint16_t baselineAsymmetryQ8;
    uint16_t baselineResistanceMilliOhm;
} HEALTH_STATE_T;

/* 64-byte packed configuration structure (for flash storage / GSP protocol) */
typedef struct __attribute__((packed))
{
    uint16_t pwmFrequency;      /* Hz */
    uint16_t deadtimeNs;        /* nanoseconds */
    uint8_t motorPolePairs;
    uint8_t direction;          /* 0=CW, 1=CCW */
    uint16_t alignTimeMs;
    uint8_t alignDutyPercent;
    uint8_t dshotRate;          /* 0=DShot150..4=DShot1200 */
    uint16_t initialErpm;
    uint16_t rampTargetErpm;
    uint16_t rampAccelErpmPerS;
    uint8_t reserved[48];       /* pad to 64 bytes (16 used + 48 reserved) */
} GARUDA_CONFIG_T;

_Static_assert(sizeof(GARUDA_CONFIG_T) == 64, "GARUDA_CONFIG_T must be 64 bytes");

/* Learned motor parameters (persisted to EEPROM) */
typedef struct __attribute__((packed))
{
    uint16_t phaseResistanceMilliOhm;
    uint16_t phaseInductanceMicroH;
    uint16_t motorKv;
    uint8_t  polePairs;
    uint8_t  optimalTimingAdvDeg;
    uint16_t startupDutyPercent;    /* Q8.8 */
} LEARNED_PARAMS_T;                 /* 10 bytes */

/* Active adapted parameters (persisted to EEPROM) */
typedef struct __attribute__((packed))
{
    uint8_t  timingAdvanceDeg;
    uint16_t startupDutyPercent;    /* Q8.8 */
    uint16_t rampAccelErpmPerS;
    uint8_t  confidence;
} ADAPTED_SNAPSHOT_T;               /* 6 bytes */

/* Health baselines (persisted to EEPROM) */
typedef struct __attribute__((packed))
{
    uint16_t jitterQ8;
    uint16_t asymmetryQ8;
    uint16_t resistanceMilliOhm;
} HEALTH_BASELINES_T;               /* 6 bytes */

/* Wear/stats counters (persisted to EEPROM) */
typedef struct __attribute__((packed))
{
    uint16_t writeCount;
    uint8_t  rollbackCount;
    uint8_t  commissionCount;
    uint16_t operatingHours;
    uint16_t startupCount;
} WEAR_STATS_T;                     /* 8 bytes */

/* EEPROM v2 learned data block (bytes 64-127) */
typedef struct __attribute__((packed))
{
    uint16_t          magic;            /* 0x4752 ('GR') */
    uint8_t           schemaVersion;    /* 2 */
    uint8_t           flags;
    LEARNED_PARAMS_T  commissioned;     /* 10 bytes */
    ADAPTED_SNAPSHOT_T active;          /* 6 bytes */
    ADAPTED_SNAPSHOT_T lastKnownGood;   /* 6 bytes */
    HEALTH_BASELINES_T baselines;       /* 6 bytes */
    WEAR_STATS_T      wear;            /* 8 bytes */
    uint8_t           reserved[22];     /* future expansion */
    uint16_t          crc16;            /* CRC-16-CCITT over bytes 64-125 */
} EEPROM_LEARNED_T;                     /* 64 bytes */

_Static_assert(sizeof(EEPROM_LEARNED_T) == 64, "EEPROM_LEARNED_T must be 64 bytes");

/* Full EEPROM image (128 bytes) */
typedef struct __attribute__((packed))
{
    GARUDA_CONFIG_T   config;           /* bytes 0-63 */
    EEPROM_LEARNED_T  learned;          /* bytes 64-127 */
} EEPROM_IMAGE_T;

_Static_assert(sizeof(EEPROM_IMAGE_T) == 128, "EEPROM_IMAGE_T must be 128 bytes");

/* Fault codes */
typedef enum
{
    FAULT_NONE = 0,
    FAULT_OVERVOLTAGE,
    FAULT_UNDERVOLTAGE,
    FAULT_OVERCURRENT,
    FAULT_STALL,
    FAULT_DESYNC
} FAULT_CODE_T;

/* Main ESC runtime data */
typedef struct
{
    ESC_STATE_T state;
    uint16_t throttle;          /* 0-2000 (DShot range), or ADC pot value */
    uint8_t currentStep;        /* 0-5 commutation step index */
    uint8_t direction;          /* 0=CW, 1=CCW */
    uint32_t duty;              /* PWM duty cycle count */
    uint16_t vbusRaw;           /* raw ADC reading of DC bus voltage */
    uint16_t potRaw;            /* raw ADC reading of potentiometer */
    FAULT_CODE_T faultCode;

    /* Timing counters */
    uint32_t alignCounter;      /* counts down during alignment phase */
    uint32_t rampStepPeriod;    /* current forced commutation period during OL ramp */
    uint32_t rampCounter;       /* counts down within current step */
    uint32_t systemTick;        /* 1ms tick counter */
    uint16_t armCounter;        /* counts up during arming phase */

    /* Sub-structures */
    BEMF_STATE_T bemf;
    TIMING_STATE_T timing;

#if FEATURE_LEARN_MODULES
    QUALITY_METRICS_T quality;
    ADAPT_PARAMS_T    adapt;
    HEALTH_STATE_T    health;
#endif
#if FEATURE_COMMISSION
    COMMISSION_DATA_T commission;
#endif
} GARUDA_DATA_T;

#ifdef __cplusplus
}
#endif

#endif /* GARUDA_TYPES_H */
