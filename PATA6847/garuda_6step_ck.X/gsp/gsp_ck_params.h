/**
 * @file gsp_ck_params.h
 * @brief CK board runtime parameters — struct, derived values, API.
 *
 * All motor, ATA6847, and ZC settings that were compile-time macros are
 * now runtime-adjustable via GSP GET_PARAM/SET_PARAM commands.
 *
 * Architecture: always-allocate. ckParams/ckDerived exist regardless of
 * FEATURE_GSP. All code reads from runtime structs (single code path).
 * When FEATURE_GSP=0, params are just compile-time defaults with no way
 * to change them. When FEATURE_GSP=1, GUI can modify via GSP commands.
 */

#ifndef GSP_CK_PARAMS_H
#define GSP_CK_PARAMS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── CK Parameter IDs (start at 0xC0 to avoid collision with AK params) ── */

/* Group 0: Motor Identity */
#define CK_PARAM_POLE_PAIRS          0xC0
#define CK_PARAM_MOTOR_KV            0xC1
#define CK_PARAM_MOTOR_RS            0xC2
#define CK_PARAM_MOTOR_LS            0xC3

/* Group 1: Startup & Ramp */
#define CK_PARAM_ALIGN_TIME_MS       0xC4
#define CK_PARAM_ALIGN_DUTY_PCT      0xC5  /* x10: 25 = 2.5% */
#define CK_PARAM_INITIAL_STEP_PERIOD 0xC6
#define CK_PARAM_MIN_STEP_PERIOD     0xC7
#define CK_PARAM_RAMP_ACCEL          0xC8
#define CK_PARAM_RAMP_DUTY_PCT       0xC9  /* x10: 170 = 17.0% */
#define CK_PARAM_RAMP_TARGET_ERPM    0xCA

/* Group 2: Closed-Loop */
#define CK_PARAM_CL_IDLE_DUTY_PCT    0xCB
#define CK_PARAM_TIM_ADV_MIN_DEG     0xCC
#define CK_PARAM_TIM_ADV_MAX_DEG     0xCD
#define CK_PARAM_TIM_ADV_START_ERPM  0xCE
#define CK_PARAM_MAX_CL_ERPM         0xCF
#define CK_PARAM_DUTY_SLEW_UP        0xD0
#define CK_PARAM_DUTY_SLEW_DOWN      0xD1

/* Group 3: ATA6847 Protection */
#define CK_PARAM_ILIM_ENABLE         0xD2
#define CK_PARAM_ILIM_SHUTDOWN       0xD3
#define CK_PARAM_ILIM_DAC            0xD4
#define CK_PARAM_ILIM_FILTER         0xD5
#define CK_PARAM_SC_ENABLE           0xD6
#define CK_PARAM_SC_THRESHOLD        0xD7
#define CK_PARAM_SC_FILTER           0xD8

/* Group 4: ATA6847 GDU */
#define CK_PARAM_BEMF_ENABLE         0xD9
#define CK_PARAM_CROSS_COND_TIME     0xDA
#define CK_PARAM_EDGE_BLANKING       0xDB
#define CK_PARAM_CSA_GAIN            0xDC

/* Group 5: ZC Detection */
#define CK_PARAM_ZC_DEGLITCH_MIN     0xDD
#define CK_PARAM_ZC_DEGLITCH_MAX     0xDE
#define CK_PARAM_ZC_TIMEOUT_MULT     0xDF
#define CK_PARAM_ZC_DESYNC_THRESH    0xE0
#define CK_PARAM_ZC_MISS_LIMIT       0xE1

/* Group 6: Voltage Protection */
#define CK_PARAM_VBUS_OV             0xE2
#define CK_PARAM_VBUS_UV             0xE3

/* Group 7: Recovery */
#define CK_PARAM_DESYNC_RESTART_MAX  0xE4
#define CK_PARAM_RECOVERY_TIME_MS    0xE5

#define CK_PARAM_COUNT  38

/* ── Group IDs ── */
#define CK_GROUP_MOTOR       0
#define CK_GROUP_STARTUP     1
#define CK_GROUP_CL          2
#define CK_GROUP_ATA_PROT    3
#define CK_GROUP_ATA_GDU     4
#define CK_GROUP_ZC          5
#define CK_GROUP_VOLTAGE     6
#define CK_GROUP_RECOVERY    7
#define CK_GROUP_COUNT       8

/* ── Type codes (for descriptor wire format) ── */
#define CK_TYPE_U8   0
#define CK_TYPE_U16  1
#define CK_TYPE_U32  2

/* ── Profile IDs ── */
#define CK_PROFILE_HURST  0
#define CK_PROFILE_A2212  1
#define CK_PROFILE_2810   2
#define CK_PROFILE_COUNT  3

/* ── Runtime Parameter Struct (~60 bytes) ── */
typedef struct {
    /* Group 0: Motor Identity */
    uint8_t  polePairs;
    uint16_t motorKv;
    uint16_t motorRsMilliOhm;
    uint16_t motorLsMicroH;

    /* Group 1: Startup & Ramp */
    uint16_t alignTimeMs;
    uint16_t alignDutyPctX10;    /* 0.1% units: 25 = 2.5% */
    uint16_t initialStepPeriod;  /* Timer1 ticks */
    uint16_t minStepPeriod;      /* Timer1 ticks */
    uint16_t rampAccelErpmS;
    uint16_t rampDutyPctX10;     /* 0.1% units: 170 = 17.0% */
    uint16_t rampTargetErpm;

    /* Group 2: Closed-Loop */
    uint8_t  clIdleDutyPct;
    uint8_t  timingAdvMinDeg;
    uint8_t  timingAdvMaxDeg;
    uint16_t timingAdvStartErpm;
    uint32_t maxClosedLoopErpm;
    uint8_t  dutySlewUp;         /* PWM counts/tick */
    uint8_t  dutySlewDown;

    /* Group 3: ATA6847 Protection */
    uint8_t  ilimEnable;
    uint8_t  ilimShutdownEnable;
    uint8_t  ilimDac;            /* 0-127 */
    uint8_t  ilimFilterTime;     /* 0-7 */
    uint8_t  scEnable;
    uint8_t  scThreshold;        /* 0-7 */
    uint8_t  scFilterTime;       /* 0-7 */

    /* Group 4: ATA6847 GDU */
    uint8_t  bemfEnable;
    uint8_t  crossConductionTime; /* 0-7 */
    uint8_t  edgeBlankingTime;    /* 0-3 */
    uint8_t  csaGain;             /* 0-3: 8x/16x/32x/64x */

    /* Group 5: ZC Detection */
    uint8_t  zcDeglitchMin;
    uint8_t  zcDeglitchMax;
    uint8_t  zcTimeoutMult;
    uint8_t  zcDesyncThresh;
    uint8_t  zcMissLimit;

    /* Group 6: Voltage Protection */
    uint16_t vbusOvThreshold;    /* Raw ADC (16-bit scaled) */
    uint16_t vbusUvThreshold;

    /* Group 7: Recovery */
    uint8_t  desyncRestartMax;
    uint16_t recoveryTimeMs;

    /* Internal — not in descriptor table */
    uint8_t  activeProfile;
} CK_PARAMS_T;

/* ── Derived Values (~16 bytes) — precomputed for ISR use ── */
typedef struct {
    /* Pre-built ATA6847 register bytes */
    uint8_t  ataIlimcrVal;
    uint8_t  ataScpcrVal;
    uint8_t  ataGducr1Val;
    uint8_t  ataGducr3Val;
    uint8_t  ataCscrVal;       /* Gain + offset only; CSA enables added at runtime */

    /* PWM-derived values */
    uint16_t alignDuty;        /* PWM counts */
    uint16_t alignTimeCounts;  /* Timer1 ticks */
    uint16_t clIdleDuty;       /* PWM counts */
    uint16_t rampDutyCap;      /* PWM counts */
    uint16_t recoveryCounts;   /* Timer1 ticks */
} CK_DERIVED_T;

/* ── Descriptor table entry (matches AK 12-byte wire format) ── */
typedef struct {
    uint16_t id;
    uint8_t  type;
    uint8_t  group;
    uint32_t min;
    uint32_t max;
} CK_PARAM_DESC_T;

/* ── Global instances ── */
extern CK_PARAMS_T  ckParams;
extern CK_DERIVED_T ckDerived;

/* ── API ── */

/** Initialize params from compile-time defaults (current MOTOR_PROFILE) */
void CK_ParamsInitDefaults(void);

/** Load a specific profile's defaults */
void CK_ParamsLoadProfile(uint8_t profileId);

/** Get a parameter value by ID. Returns 0 on unknown ID, sets *ok=false. */
uint32_t CK_ParamGet(uint16_t paramId, bool *ok);

/** Set a parameter value by ID. Returns false if ID unknown or out of range. */
bool CK_ParamSet(uint16_t paramId, uint32_t value);

/** Recompute derived values from current params. Call after any param change. */
void CK_RecomputeDerived(void);

/** Cross-validate params. Returns true if valid, false if constraint violated. */
bool CK_ParamsCrossValidate(void);

/** Get descriptor table pointer and count (for GET_PARAM_LIST). */
const CK_PARAM_DESC_T* CK_GetDescriptorTable(uint8_t *count);

#ifdef __cplusplus
}
#endif

#endif /* GSP_CK_PARAMS_H */
