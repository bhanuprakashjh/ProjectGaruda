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
    ESC_RECOVERY,       /* Desync coast-down before restart attempt */
    ESC_FAULT
} ESC_STATE_T;

/* Enum ordering guards — voltage fault check uses >= / <= on state enum */
_Static_assert(ESC_IDLE < ESC_ARMED, "State enum ordering violated");
_Static_assert(ESC_ARMED < ESC_ALIGN, "State enum ordering violated");
_Static_assert(ESC_ALIGN < ESC_CLOSED_LOOP, "State enum ordering violated");
_Static_assert(ESC_CLOSED_LOOP < ESC_RECOVERY, "State enum ordering violated");
_Static_assert(ESC_RECOVERY < ESC_FAULT, "State enum ordering violated");

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

/* BEMF sensing state */
typedef struct
{
    uint16_t bemfRaw;           /* Floating phase ADC reading (0-4095) */
    uint16_t zcThreshold;       /* ZC comparison threshold (scaled from Vbus) */
    bool     zeroCrossDetected; /* ZC event detected this step */
    uint8_t  cmpPrev;           /* Previous computed state (0 or 1, 0xFF=unknown) */
    uint8_t  cmpExpected;       /* Expected post-ZC state (0 or 1) */
    uint8_t  filterCount;       /* Consecutive reads matching cmpExpected */
    uint8_t  ad2SettleCount;    /* Samples to discard after AD2 PINSEL change (0=settled) */
    bool     bemfSampleValid;   /* false when bemfRaw is stale after mux switch */
} BEMF_STATE_T;

/* ZC timeout result enum */
typedef enum
{
    ZC_TIMEOUT_NONE = 0,        /* No timeout — keep waiting */
    ZC_TIMEOUT_FORCE_STEP,      /* One forced step needed (re-armed internally) */
    ZC_TIMEOUT_DESYNC           /* Too many misses — fault */
} ZC_TIMEOUT_RESULT_T;

/* Commutation timing state */
typedef struct
{
    uint16_t stepPeriod;            /* Current step period in adcIsrTick units */
    uint16_t lastCommTick;          /* adcIsrTick at last commutation (timeout basis) */
    uint16_t lastZcTick;            /* adcIsrTick at last confirmed ZC */
    uint16_t prevZcTick;            /* adcIsrTick at ZC before lastZcTick (for interval) */
    uint16_t zcInterval;            /* Ticks between last two ZCs */
    uint16_t commDeadline;          /* adcIsrTick when next commutation should fire */
    uint16_t forcedCountdown;       /* Ticks remaining for forced commutation (pre-sync) */
    uint16_t goodZcCount;           /* Consecutive valid ZC events */
    uint8_t  consecutiveMissedSteps;/* Steps where ZC was not detected */
    uint8_t  stepsSinceLastZc;      /* Forced steps since last confirmed ZC (interval compensation) */
    uint8_t  stepMissCount[6];      /* Per-step consecutive miss counter for fallback scheduling */
    bool     risingZcWorks;         /* True after any rising-ZC step is confirmed */
    bool     fallingZcWorks;        /* True after any falling-ZC step is confirmed */
    bool     zcSynced;              /* True = ZC-driven, False = forced */
    bool     deadlineActive;        /* True when commDeadline is valid */
    bool     hasPrevZc;             /* True after first post-sync ZC (guards zcInterval calc) */
} TIMING_STATE_T;

/* Diagnostic counters for ZC bring-up debugging (ADC threshold method).
 * Readable via debugger halt. All uint16_t/uint8_t to avoid atomicity issues. */
#if FEATURE_BEMF_CLOSED_LOOP
typedef struct
{
    /* Event counters */
    uint16_t zcConfirmedCount;
    uint16_t zcTimeoutForceCount;
    uint16_t zcDesyncCount;
    uint16_t forcedStepPresyncCount;
    /* Signal diagnostics */
    uint16_t zcPollTotal;           /* Polls that passed blanking AND were valid */
    uint16_t zcAboveTotal;          /* Polls where computed state = 1 */
    uint16_t blankSkipTotal;
    uint16_t invalidSampleTotal;    /* Polls skipped due to stale AD2 mux data */
    uint16_t deadbandHoldTotal;
    /* Blanking-aware transition tracking */
    uint16_t blankTransitionTotal;  /* Correct-polarity edges detected during blanking */
    uint16_t wrongEdgeTotal;        /* Transitions opposite to expected polarity */
    /* Last-sample snapshot */
    uint16_t lastFloatAdc;
    uint16_t lastZcThreshold;
    uint8_t  lastStateNow;
    uint8_t  lastCmpExpected;
    uint8_t  lastFloatPhase;
    uint8_t  lastEdgeCount;
    /* Per-phase min/max (detect stuck channels / scaling mismatch) */
    uint16_t floatAdcMin[3];        /* Init to 4095 */
    uint16_t floatAdcMax[3];        /* Init to 0 */
    uint16_t zcThresholdMin;
    uint16_t zcThresholdMax;
    /* Per-step ZC confirmed count (which of the 6 steps fire?) */
    uint16_t zcPerStep[6];
#if FEATURE_TIMING_ADVANCE
    uint8_t  lastAdvanceDeg;        /* Last computed timing advance in degrees */
#endif
} ZC_DIAG_T;
#endif

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
    float zcMissRate;           /* 0.0-1.0 */
    float falseRate;            /* 0.0-1.0 */
    float timeoutRate;          /* 0.0-1.0 */
    float zcJitter;             /* ZC latency std dev in timer ticks */
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
    float    startupDutyPercent;         /* percent (0.0-100.0) */
    uint16_t rampAccelErpmPerS;
    uint16_t alignTimeMs;
    uint8_t  timingAdvanceMin;          /* envelope min */
    uint8_t  timingAdvanceMax;          /* envelope max */
    float    startupDutyMin;            /* percent */
    float    startupDutyMax;            /* percent */
    uint8_t  lkgTimingAdvanceDeg;       /* last-known-good */
    float    lkgStartupDutyPercent;
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
    float    baselineJitter;             /* std dev in timer ticks */
    float    baselineAsymmetry;
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

    /* Run command and recovery */
    bool     runCommandActive;       /* SW1-driven latch: true = user wants motor running */
    uint8_t  desyncRestartAttempts;  /* Reset on successful sync lock */
    uint32_t recoveryCounter;        /* Timer1 countdown during coast-down */

    /* Sub-structures */
    BEMF_STATE_T bemf;
    TIMING_STATE_T timing;

#if FEATURE_BEMF_CLOSED_LOOP
    ZC_DIAG_T zcDiag;
#endif

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
