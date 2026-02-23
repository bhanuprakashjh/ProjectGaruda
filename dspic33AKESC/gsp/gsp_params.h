/**
 * @file gsp_params.h
 *
 * @brief Runtime parameter system for GSP Phase 1.
 *
 * Provides typed parameter struct (GSP_PARAMS_T), precomputed derived
 * values for ISR use (GSP_DERIVED_T), and table-driven validation.
 *
 * 8 tunables in Stage 1: ramp target/accel/duty, CL idle duty,
 * timing advance, HWZC crossover, overcurrent soft/fault limits.
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

/* Parameter IDs (stable across firmware versions) */
#define PARAM_ID_RAMP_TARGET_ERPM       0x15
#define PARAM_ID_RAMP_ACCEL_ERPM_PER_S  0x16
#define PARAM_ID_RAMP_DUTY_PCT          0x17
#define PARAM_ID_CL_IDLE_DUTY_PCT       0x20
#define PARAM_ID_TIMING_ADV_MAX_DEG     0x22
#define PARAM_ID_HWZC_CROSSOVER_ERPM    0x30
#define PARAM_ID_OC_FAULT_MA            0x41
#define PARAM_ID_OC_SW_LIMIT_MA         0x42

/* Parameter type codes (for descriptor table) */
#define PARAM_TYPE_U8   0
#define PARAM_TYPE_U16  1
#define PARAM_TYPE_U32  2

/* Parameter group codes */
#define PARAM_GROUP_STARTUP     0
#define PARAM_GROUP_CLOSED_LOOP 1
#define PARAM_GROUP_OVERCURRENT 2

/* Runtime parameter struct — typed fields, NOT all-u32 */
typedef struct {
    uint16_t rampTargetErpm;
    uint16_t rampAccelErpmPerS;
    uint8_t  rampDutyPct;
    uint8_t  clIdleDutyPct;
    uint8_t  timingAdvMaxDeg;
    uint8_t  _pad0;              /* alignment padding */
    uint16_t hwzcCrossoverErpm;
    uint16_t ocSwLimitMa;
    uint16_t ocFaultMa;
} GSP_PARAMS_T;  /* 14 bytes + 1 pad = 15 bytes (packed would be 14) */

/* Derived values precomputed from params — ISR reads these.
 * hwzcCrossoverErpm is NOT derived (read directly from gspParams,
 * 16-bit atomic on dsPIC33AK). */
typedef struct {
    uint32_t rampDutyCap;           /* rampDutyPct * LOOPTIME_TCY / 100 */
    uint32_t clIdleDuty;            /* clIdleDutyPct * LOOPTIME_TCY / 100 */
    uint32_t sineErpmRampRateQ16;   /* rampAccelErpmPerS * 65536 / 10000 */
    uint16_t minStepPeriod;         /* ERPM_TO_STEP_TICKS(rampTargetErpm) */
    uint16_t minAdcStepPeriod;      /* TIMER1_TO_ADC_TICKS(minStepPeriod) */
    uint16_t ocSwLimitAdc;          /* from ocSwLimitMa */
    uint16_t ocFaultAdcVal;         /* from ocFaultMa */
} GSP_DERIVED_T;  /* 20 bytes */

/* Param descriptor for table-driven validation */
typedef struct {
    uint16_t id;
    uint8_t  type;          /* PARAM_TYPE_U8 / U16 / U32 */
    uint8_t  group;
    uint16_t minVal;
    uint16_t maxVal;
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

/* EEPROM persistence — packed struct for GARUDA_CONFIG_T reserved bytes */
typedef struct __attribute__((packed)) {
    uint8_t  schemaV1Marker;        /* 0xA1 = GSP params present */
    uint8_t  rampDutyPct;
    uint8_t  clIdleDutyPct;
    uint8_t  timingAdvMaxDeg;
    uint16_t rampTargetErpm;
    uint16_t rampAccelErpmPerS;
    uint16_t hwzcCrossoverErpm;
    uint16_t ocSwLimitMa;
    uint16_t ocFaultMa;
    uint8_t  reserved2[18];         /* future Stage 2+ params */
} GSP_CONFIG_PERSIST_T;             /* 32 bytes */

_Static_assert(sizeof(GSP_CONFIG_PERSIST_T) == 32, "must fit EEPROM reserved");

/**
 * Load params from EEPROM config reserved bytes.
 * If schemaV1Marker != 0xA1, keeps compile-time defaults.
 *
 * @param cfg  Pointer to loaded GARUDA_CONFIG_T
 */
void GSP_ParamsLoadFromConfig(const void *cfg);

/**
 * Pack current params into GARUDA_CONFIG_T reserved bytes.
 *
 * @param cfg  Pointer to GARUDA_CONFIG_T to update
 */
void GSP_ParamsSaveToConfig(void *cfg);

#ifdef __cplusplus
}
#endif

#endif /* GSP_PARAMS_H */
