/**
 * @file gsp_params.h
 *
 * @brief Runtime parameter system for GSP Phase 1.5.
 *
 * Provides typed parameter struct (GSP_PARAMS_T), precomputed derived
 * values for ISR use (GSP_DERIVED_T), and table-driven validation.
 *
 * 31 tunables: 8 Stage 1 + 11 motor profile + 12 tuning params.
 * 3 built-in profiles (Hurst, A2212, 5010) + 1 custom.
 *
 * Component: GSP
 */

#ifndef GSP_PARAMS_H
#define GSP_PARAMS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Parameter IDs (stable across firmware versions) ─────────────────── */

/* Stage 1 (existing 8) */
#define PARAM_ID_RAMP_TARGET_ERPM       0x15
#define PARAM_ID_RAMP_ACCEL_ERPM_PER_S  0x16
#define PARAM_ID_RAMP_DUTY_PCT          0x17
#define PARAM_ID_CL_IDLE_DUTY_PCT       0x20
#define PARAM_ID_TIMING_ADV_MAX_DEG     0x22
#define PARAM_ID_HWZC_CROSSOVER_ERPM    0x30
#define PARAM_ID_OC_FAULT_MA            0x41
#define PARAM_ID_OC_SW_LIMIT_MA         0x42

/* Motor profile params (11 new) */
#define PARAM_ID_MOTOR_POLE_PAIRS       0x50
#define PARAM_ID_ALIGN_DUTY_PCT         0x51
#define PARAM_ID_INITIAL_ERPM           0x52
#define PARAM_ID_MAX_CL_ERPM            0x53
#define PARAM_ID_SINE_ALIGN_MOD_PCT     0x54
#define PARAM_ID_SINE_RAMP_MOD_PCT      0x55
#define PARAM_ID_ZC_DEMAG_DUTY_THRESH   0x56
#define PARAM_ID_ZC_DEMAG_BLANK_EXTRA   0x57
#define PARAM_ID_OC_LIMIT_MA            0x58
#define PARAM_ID_OC_STARTUP_MA          0x59
#define PARAM_ID_RAMP_CURRENT_GATE_MA   0x5A

/* Tuning params (12 new) */
#define PARAM_ID_DUTY_SLEW_UP           0x60
#define PARAM_ID_DUTY_SLEW_DOWN         0x61
#define PARAM_ID_POST_SYNC_SETTLE_MS    0x62
#define PARAM_ID_POST_SYNC_SLEW_DIV     0x63
#define PARAM_ID_ZC_BLANKING_PCT        0x64
#define PARAM_ID_ZC_ADC_DEADBAND        0x65
#define PARAM_ID_ZC_SYNC_THRESHOLD      0x66
#define PARAM_ID_ZC_FILTER_THRESHOLD    0x67
#define PARAM_ID_VBUS_OV_ADC            0x68
#define PARAM_ID_VBUS_UV_ADC            0x69
#define PARAM_ID_DESYNC_COAST_MS        0x6A
#define PARAM_ID_DESYNC_MAX_RESTARTS    0x6B

/* Parameter type codes (for descriptor table) */
#define PARAM_TYPE_U8   0
#define PARAM_TYPE_U16  1
#define PARAM_TYPE_U32  2

/* Parameter group codes (8 groups) */
#define PARAM_GROUP_STARTUP      0
#define PARAM_GROUP_CLOSED_LOOP  1
#define PARAM_GROUP_OVERCURRENT  2
#define PARAM_GROUP_ZC_DETECT    3
#define PARAM_GROUP_DUTY_SLEW    4
#define PARAM_GROUP_VOLTAGE      5
#define PARAM_GROUP_RECOVERY     6
#define PARAM_GROUP_MOTOR_HW     7

/* ── Runtime parameter struct (31 fields) ────────────────────────────── */

typedef struct {
    /* Stage 1 (existing 8) */
    uint16_t rampTargetErpm;
    uint16_t rampAccelErpmPerS;
    uint8_t  rampDutyPct;
    uint8_t  clIdleDutyPct;
    uint8_t  timingAdvMaxDeg;
    uint8_t  _pad0;
    uint16_t hwzcCrossoverErpm;
    uint16_t ocSwLimitMa;
    uint16_t ocFaultMa;
    /* Motor profile params (11 new) */
    uint8_t  motorPolePairs;
    uint8_t  alignDutyPct;
    uint16_t initialErpm;
    uint32_t maxClosedLoopErpm;
    uint8_t  sineAlignModPct;
    uint8_t  sineRampModPct;
    uint8_t  zcDemagDutyThresh;
    uint8_t  zcDemagBlankExtraPct;
    uint16_t ocLimitMa;
    uint16_t ocStartupMa;
    uint16_t rampCurrentGateMa;
    /* Tuning params (12 new) */
    uint8_t  dutySlewUpPctPerMs;
    uint8_t  dutySlewDownPctPerMs;
    uint16_t postSyncSettleMs;
    uint8_t  postSyncSlewDivisor;
    uint8_t  zcBlankingPercent;
    uint8_t  zcAdcDeadband;
    uint8_t  zcSyncThreshold;
    uint8_t  zcFilterThreshold;
    uint8_t  _pad1;
    uint16_t vbusOvAdc;
    uint16_t vbusUvAdc;
    uint16_t desyncCoastMs;
    uint8_t  desyncMaxRestarts;
    uint8_t  _pad2;
} GSP_PARAMS_T;

/* ── Derived values (precomputed from params, ISR reads these) ───────── */

typedef struct {
    /* Existing 7 */
    uint32_t rampDutyCap;
    uint32_t clIdleDuty;
    uint32_t sineErpmRampRateQ16;
    uint16_t minStepPeriod;
    uint16_t minAdcStepPeriod;
    uint16_t ocSwLimitAdc;
    uint16_t ocFaultAdcVal;
    /* New derived values */
    uint32_t alignDuty;
    uint32_t sineMinAmplitude;
    uint32_t sineMaxAmplitude;
    uint32_t dutySlewUpRate;
    uint32_t dutySlewDownRate;
    uint32_t desyncCoastCounts;
    uint32_t hwzcMinStepTicks;
    uint32_t hwzcNoiseFloorTicks;
    uint16_t initialStepPeriod;
    uint16_t initialAdcStepPeriod;
    uint16_t minClAdcStepPeriod;
    uint16_t postSyncSettleTicks;
    uint16_t ocCmp3DacVal;
    uint16_t ocCmp3StartupDac;
    uint16_t rampCurrentGateAdc;
    uint16_t vbusUvStartupAdc;
} GSP_DERIVED_T;

/* ── Param descriptor for table-driven validation ────────────────────── */

typedef struct {
    uint16_t id;
    uint8_t  type;          /* PARAM_TYPE_U8 / U16 / U32 */
    uint8_t  group;
    uint32_t minVal;        /* V2: widened to u32 (was u16) */
    uint32_t maxVal;
    uint8_t  offsetInParams; /* byte offset into GSP_PARAMS_T */
    uint8_t  fieldSize;      /* 1, 2, or 4 */
} PARAM_DESCRIPTOR_T;

/* Validation result codes */
typedef enum {
    PARAM_OK = 0,
    PARAM_ERR_UNKNOWN_ID,
    PARAM_ERR_OUT_OF_RANGE,
    PARAM_ERR_CROSS_VALIDATION,
} PARAM_RESULT_T;

/* Motor profile IDs */
#define GSP_PROFILE_HURST   0
#define GSP_PROFILE_A2212   1
#define GSP_PROFILE_5010    2
#define GSP_PROFILE_CUSTOM  3
#define GSP_PROFILE_COUNT   3  /* built-in profiles (excl. Custom) */

/* Global instances (defined in gsp_params.c) */
extern GSP_PARAMS_T  gspParams;
extern GSP_DERIVED_T gspDerived;

/**
 * Initialize all params to compile-time defaults from motor profile.
 * Call once at boot before GARUDA_ServiceInit().
 */
void GSP_ParamsInitDefaults(void);

/**
 * Set a parameter by ID with bounds + cross-parameter validation.
 * Calls GSP_RecomputeDerived() on success.
 *
 * @param id    Parameter ID (PARAM_ID_*)
 * @param value New value (u32 container, truncated to field size)
 * @return PARAM_OK on success, error code on failure
 */
PARAM_RESULT_T GSP_ParamSet(uint16_t id, uint32_t value);

/**
 * Get a parameter value by ID.
 *
 * @param id   Parameter ID
 * @param out  Output value (widened to u32)
 * @return true if parameter found, false if unknown ID
 */
bool GSP_ParamGet(uint16_t id, uint32_t *out);

/**
 * Recompute all ISR-facing derived values from current gspParams.
 * Called automatically by GSP_ParamSet(); also call after bulk load.
 */
void GSP_RecomputeDerived(void);

/**
 * @return Number of parameters in the descriptor table.
 */
uint8_t GSP_ParamGetCount(void);

/**
 * Get descriptor by index (0..count-1).
 * @param idx  Table index
 * @return Pointer to descriptor, or NULL if out of range
 */
const PARAM_DESCRIPTOR_T *GSP_ParamGetDescriptor(uint8_t idx);

/**
 * Load a built-in profile (0-2) or adopt current values as custom (3).
 * Recomputes derived values. Returns false if id is out of range.
 */
bool GSP_ParamsLoadProfile(uint8_t profileId);

/**
 * @return Currently active profile ID (0-3).
 */
uint8_t GSP_ParamsGetActiveProfile(void);

/* ── EEPROM V2 persistence ───────────────────────────────────────────── */

#define GSP_PERSIST_V1_MARKER  0xA1
#define GSP_PERSIST_V2_MARKER  0xA2

/* V2 packed persist struct — 48 bytes, fills GARUDA_CONFIG_T.reserved */
typedef struct __attribute__((packed)) {
    /* Header (2 bytes) */
    uint8_t  schemaMarker;          /* 0xA2 = V2 */
    uint8_t  activeProfile;         /* 0-3 */
    /* Stage 1 params (13 bytes, offsets 2-14) */
    uint8_t  rampDutyPct;           /* [2] */
    uint8_t  clIdleDutyPct;         /* [3] */
    uint8_t  timingAdvMaxDeg;       /* [4] */
    uint16_t rampTargetErpm;        /* [5-6] */
    uint16_t rampAccelErpmPerS;     /* [7-8] */
    uint16_t hwzcCrossoverErpm;     /* [9-10] */
    uint16_t ocSwLimitMa;           /* [11-12] */
    uint16_t ocFaultMa;             /* [13-14] */
    /* Motor profile params (16 bytes, offsets 15-30) */
    uint8_t  motorPolePairs;        /* [15] */
    uint8_t  alignDutyPct;          /* [16] */
    uint16_t initialErpm;           /* [17-18] */
    uint8_t  sineAlignModPct;       /* [19] */
    uint8_t  sineRampModPct;        /* [20] */
    uint8_t  zcDemagDutyThresh;     /* [21] */
    uint8_t  zcDemagBlankExtraPct;  /* [22] */
    uint16_t ocLimitMa;             /* [23-24] */
    uint16_t ocStartupMa;           /* [25-26] */
    uint16_t rampCurrentGateMa;     /* [27-28] */
    uint16_t maxClosedLoopErpmLo;   /* [29-30] low 16 bits */
    /* Tuning params (16 bytes, offsets 31-46) */
    uint8_t  dutySlewUpPctPerMs;    /* [31] */
    uint8_t  dutySlewDownPctPerMs;  /* [32] */
    uint16_t postSyncSettleMs;      /* [33-34] */
    uint8_t  postSyncSlewDivisor;   /* [35] */
    uint8_t  zcBlankingPercent;     /* [36] */
    uint8_t  zcAdcDeadband;         /* [37] */
    uint8_t  zcSyncThreshold;       /* [38] */
    uint8_t  zcFilterThreshold;     /* [39] */
    uint16_t vbusOvAdc;             /* [40-41] */
    uint16_t vbusUvAdc;             /* [42-43] */
    uint16_t desyncCoastMs;         /* [44-45] */
    uint8_t  desyncMaxRestarts;     /* [46] */
    /* High byte of maxClosedLoopErpm (1 byte, offset 47) */
    uint8_t  maxClErpmHi;           /* [47] bits [23:16] */
} GSP_CONFIG_PERSIST_V2_T;

_Static_assert(sizeof(GSP_CONFIG_PERSIST_V2_T) == 48, "V2 persist must be 48 bytes");

/* V1 persist (for backward compat read) */
typedef struct __attribute__((packed)) {
    uint8_t  schemaV1Marker;
    uint8_t  rampDutyPct;
    uint8_t  clIdleDutyPct;
    uint8_t  timingAdvMaxDeg;
    uint16_t rampTargetErpm;
    uint16_t rampAccelErpmPerS;
    uint16_t hwzcCrossoverErpm;
    uint16_t ocSwLimitMa;
    uint16_t ocFaultMa;
    uint8_t  reserved2[18];
} GSP_CONFIG_PERSIST_V1_T;

_Static_assert(sizeof(GSP_CONFIG_PERSIST_V1_T) == 32, "V1 persist must be 32 bytes");

/**
 * Load params from EEPROM config reserved bytes.
 * Supports V1 (0xA1) and V2 (0xA2) schemas.
 * V1: loads 8 Stage 1 params, 23 new get defaults from active profile.
 * V2: loads all 31 params + activeProfile.
 *
 * @param cfg  Pointer to loaded GARUDA_CONFIG_T
 */
void GSP_ParamsLoadFromConfig(const void *cfg);

/**
 * Pack current params into GARUDA_CONFIG_T reserved bytes (V2 schema).
 *
 * @param cfg  Pointer to GARUDA_CONFIG_T to update
 */
void GSP_ParamsSaveToConfig(void *cfg);

#ifdef __cplusplus
}
#endif

#endif /* GSP_PARAMS_H */
