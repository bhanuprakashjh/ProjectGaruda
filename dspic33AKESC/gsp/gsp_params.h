/**
 * @file gsp_params.h
 *
 * @brief Runtime parameter system for GSP Phase 1.5.
 *
 * Provides typed parameter struct (GSP_PARAMS_T), precomputed derived
 * values for ISR use (GSP_DERIVED_T), and table-driven validation.
 *
 * 48 tunables: 8 Stage 1 + 11 motor profile + 12 tuning + 17 FOC params.
 * 4 built-in profiles (Hurst, A2212, 5010, 5055) + 1 custom.
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
#define PARAM_ID_ZC_DEMAG_BLANK_PER_A   0x5B   /* WS1: load-adaptive demag blank, % sector per 256 cts ibus */
#define PARAM_ID_ZC_DEMAG_BLANK_IBUS_DB 0x5C   /* WS1: ibus deadband (raw counts) before current-blank engages */
#define PARAM_ID_STALL_IPHASE_ADC       0x5D   /* WS2: |phase current| (raw counts over bias) to call stall */
#define PARAM_ID_STALL_DEBOUNCE_MS      0x5E   /* WS2: sustained-ms above threshold before FAULT_STALL */
#define PARAM_ID_STALL_ARM_ERPM         0x5F   /* WS2: eRPM the motor must reach once before WS2 can fire */
/* WS4: nested speed→current→duty cascade (FEATURE_SPEED_CASCADE) */
#define PARAM_ID_CASCADE_SPEED_KP_MILLI 0x94   /* outer speed PI Kp ×1000 */
#define PARAM_ID_CASCADE_SPEED_KI_MICRO 0x95   /* outer speed PI Ki ×1e6 */
#define PARAM_ID_CASCADE_CURR_KP_MILLI  0x96   /* inner current PI Kp ×1000 */
#define PARAM_ID_CASCADE_CURR_KI_MICRO  0x97   /* inner current PI Ki ×1e6 */
#define PARAM_ID_CASCADE_IREF_CEILING   0x98   /* current ceiling (raw counts over bias) = the safety clamp */
#define PARAM_ID_CASCADE_TGT_ERPM_IDLE  0x99   /* throttle=0 target eRPM */
#define PARAM_ID_CASCADE_TGT_ERPM_MAX   0x9A   /* throttle=full target eRPM */
/* WS3: duty-adaptive BEMF trigger (FEATURE_VARIABLE_BEMF_TRIGGER) */
#define PARAM_ID_BEMF_TRIG_BASE_PCT     0x9B   /* base sample point as % of LOOPTIME_TCY */
#define PARAM_ID_BEMF_TRIG_DUTY_THRESH  0x9C   /* duty% above which the shift engages */
#define PARAM_ID_BEMF_TRIG_SHIFT_Q      0x9D   /* duty-proportional shift gain (0 = fixed) */
/* WS1 (extended): cap on total HW-ZC blanking, % of commutation period (FEATURE_ZC_CURRENT_BLANK) */
#define PARAM_ID_ZC_DEMAG_BLANK_MAX_PCT 0x9E   /* total blank cap as % of period (25 = legacy period/4) */
#define PARAM_ID_DBG_MIN_STEP_PERIOD    0xA0   /* RO mirror of gspDerived.minStepPeriod */
#define PARAM_ID_DBG_MISTIME_EVENTS     0xA1   /* mistime watchdog: +1 strike, +100 trip */
#define PARAM_ID_DBG_FE_SAMPLES         0xA2   /* softneutral FE: fast-lane samples/sector */
#define PARAM_ID_DBG_FE_VOTERESETS      0xA3   /* softneutral FE: cumulative vote resets */
#define PARAM_ID_DBG_FE_D3MIN           0xA4   /* FE diag: d3 min in probe sector, +32768 offset */
#define PARAM_ID_DBG_FE_D3MAX           0xA5   /* FE diag: d3 max in probe sector, +32768 offset */

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
/* Morph→CL lock gate (live-tunable) */
#define PARAM_ID_MORPH_LOCK_ZC_COUNT    0x6C
#define PARAM_ID_MORPH_LOCK_TOL_PCT     0x6D
/* I-f spin-up (live-tunable) */
#define PARAM_ID_IF_CURRENT_CA          0x6E
#define PARAM_ID_IF_RAMP_ERPM_PER_S     0x6F

/* FOC motor model params (17 new — runtime-configurable motor profiles) */
#define PARAM_ID_FOC_RS_MOHM            0x70
#define PARAM_ID_FOC_LS_UH              0x71
#define PARAM_ID_FOC_KE_UV_S_RAD        0x72
#define PARAM_ID_FOC_VBUS_NOM_CV        0x73
#define PARAM_ID_FOC_MAX_CURRENT_CA     0x74
#define PARAM_ID_FOC_MAX_ELEC_RAD_S     0x75
#define PARAM_ID_FOC_KP_DQ_MILLI        0x76
#define PARAM_ID_FOC_KI_DQ              0x77
#define PARAM_ID_FOC_OBS_LPF_MILLI      0x78
#define PARAM_ID_FOC_ALIGN_IQ_CA        0x79
#define PARAM_ID_FOC_RAMP_IQ_CA         0x7A
#define PARAM_ID_FOC_ALIGN_TIME_MS      0x7B
#define PARAM_ID_FOC_IQ_RAMP_TIME_MS    0x7C
#define PARAM_ID_FOC_RAMP_RATE_RPS2     0x7D
#define PARAM_ID_FOC_HANDOFF_RAD_S      0x7E
#define PARAM_ID_FOC_FAULT_OC_CA        0x7F
#define PARAM_ID_FOC_FAULT_STALL_DRS    0x80

/* AN1078 SMC tuning IDs — live-update on SET_PARAM */
#define PARAM_ID_AN1078_THETA_BASE_DEGX10  0x90
#define PARAM_ID_AN1078_THETA_K_E7         0x91
#define PARAM_ID_AN1078_KSLIDE_MV          0x92
#define PARAM_ID_AN1078_ID_FW_MAX_DECIA    0x93

/* Parameter type codes (for descriptor table) */
#define PARAM_TYPE_U8   0
#define PARAM_TYPE_U16  1
#define PARAM_TYPE_U32  2

/* Parameter group codes (11 groups) */
#define PARAM_GROUP_STARTUP      0
#define PARAM_GROUP_CLOSED_LOOP  1
#define PARAM_GROUP_OVERCURRENT  2
#define PARAM_GROUP_ZC_DETECT    3
#define PARAM_GROUP_DUTY_SLEW    4
#define PARAM_GROUP_VOLTAGE      5
#define PARAM_GROUP_RECOVERY     6
#define PARAM_GROUP_MOTOR_HW     7
#define PARAM_GROUP_FOC_MOTOR    8
#define PARAM_GROUP_FOC_TUNING   9
#define PARAM_GROUP_FOC_STARTUP  10
#define PARAM_GROUP_AN1078       11

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
    uint8_t  morphLockZcCount;    /* (was _pad1) consecutive STABLE Hi-Z ZC intervals
                                   * required to exit morph → CL (strict lock gate) */
    uint16_t vbusOvAdc;
    uint16_t vbusUvAdc;
    uint16_t desyncCoastMs;
    uint8_t  desyncMaxRestarts;
    uint8_t  morphLockTolPct;     /* (was _pad2) max % deviation between consecutive
                                   * Hi-Z ZC intervals to count as "stable" */
    /* FOC motor model params (17 new) — u16 integer-scaled */
    uint16_t focRsMilliOhm;          /* Rs × 1000 (mΩ) */
    uint16_t focLsMicroH;            /* Ls × 1e6 (µH) */
    uint16_t focKeUvSRad;            /* Ke × 1e6 (µV·s/rad) */
    uint16_t focVbusNomCentiV;       /* Vbus × 100 (cV) */
    uint16_t focMaxCurrentCentiA;    /* Imax × 100 (cA) */
    uint16_t focMaxElecRadS;         /* max ωe (rad/s) */
    uint16_t focKpDqMilli;           /* Kp_dq × 1000 */
    uint16_t focKiDq;                /* Ki_dq (integer) */
    uint16_t focObsLpfAlphaMilli;    /* OBS_LPF_ALPHA × 1000 */
    uint16_t focAlignIqCentiA;       /* Align Iq × 100 (cA) */
    uint16_t focRampIqCentiA;        /* Ramp Iq × 100 (cA) */
    uint16_t focAlignTimeMs;         /* Alignment dwell (ms) */
    uint16_t focIqRampTimeMs;        /* Iq ramp duration (ms) */
    uint16_t focRampRateRps2;        /* Ramp acceleration (rad/s²) */
    uint16_t focHandoffRadS;         /* CL handoff speed (rad/s) */
    uint16_t focFaultOcCentiA;       /* OC fault threshold × 100 (cA) */
    uint16_t focFaultStallDeciRadS;  /* Stall threshold × 10 (drad/s) */
    /* AN1078 SMC observer tuning (RAM-only, not persisted to V3 EEPROM —
     * defaults loaded from per-profile init in gsp_params.c at boot.
     * Add to V4 persist when ready to bake validated values). */
    uint16_t an1078ThetaBaseDegX10;  /* SMC theta offset base × 10 (deg) — 0-3600 */
    uint16_t an1078ThetaKE7;         /* SMC theta offset K × 1e7 — 0-1000 */
    uint16_t an1078KslideMv;         /* SMC sliding gain × 1000 (mV) — 100-30000 */
    uint16_t an1078IdFwMaxDecia;     /* |Id_fw_max| × 10 (dA) — 0-200, sign forced negative */
    /* I-f spin-up (FEATURE_IF_STARTUP) — appended at end (no layout shift above) */
    uint16_t ifCurrentCa;            /* I-f forced current × 100 (cA) — the spin-up cap */
    uint16_t ifRampErpmPerS;         /* I-f open-loop accel (eRPM/s) — rotor-follow knob */
    /* WS1: load-adaptive demag blanking (FEATURE_ZC_CURRENT_BLANK) — appended at
     * end, no layout shift above. RAM-only (not in V3 EEPROM/snapshot packed
     * structs); reset to profile default on reboot, live-settable for tuning. */
    uint8_t  zcDemagBlankPerA;       /* extra HW-ZC blank: % of sector per 256 cts ibus over deadband */
    uint8_t  zcDemagBlankIbusDb;     /* ibus deadband (raw counts) before the current term engages */
    /* WS2: phase-current stall fault (FEATURE_PHASE_STALL_FAULT) — RAM-only, appended at end. */
    uint16_t stallIphaseAdc;         /* |phase current| magnitude (raw counts over 2048) to call stall */
    uint8_t  stallDebounceMs;        /* sustained-ms above threshold before latching FAULT_STALL */
    uint16_t stallArmErpm;           /* WS2 arm gate: eRPM the motor must cross once (latched) before stall can fire */
    /* WS4: nested speed→current→duty cascade (FEATURE_SPEED_CASCADE) — RAM-only, appended at end. */
    uint16_t cascadeSpeedKpMilli;    /* outer speed PI Kp ×1000 */
    uint16_t cascadeSpeedKiMicro;    /* outer speed PI Ki ×1e6 */
    uint16_t cascadeCurrKpMilli;     /* inner current PI Kp ×1000 */
    uint16_t cascadeCurrKiMicro;     /* inner current PI Ki ×1e6 */
    uint16_t cascadeIrefCeilingAdc;  /* current ceiling (raw counts over bias) — the safety clamp */
    uint16_t cascadeTgtErpmIdle;     /* throttle=0 target eRPM */
    uint32_t cascadeTgtErpmMax;      /* throttle=full target eRPM */
    /* WS3: duty-adaptive BEMF trigger (FEATURE_VARIABLE_BEMF_TRIGGER) — RAM-only, appended at end. */
    uint8_t  bemfTrigBasePct;        /* base sample point as % of LOOPTIME_TCY (50 = MPER/2) */
    uint8_t  bemfTrigDutyThreshPct;  /* duty% above which the duty-proportional shift engages */
    uint8_t  bemfTrigShiftQ;         /* shift gain (0 = fixed at basePct) */
    /* WS1 (extended): cap on total HW-ZC blanking as % of period — RAM-only, appended at end.
     * 25 reproduces the legacy period/4 cap; raise (33 = period/3) for more high-current
     * demag-blank headroom when zcDemagBlankPerA saturates above the ~5.7A knee. */
    uint8_t  zcDemagBlankMaxPct;
    /* Debug observability (RAM-only, appended): dbgMinStepPeriod mirrors
     * gspDerived.minStepPeriod after every recompute (proves live param
     * plumbing on the bench); dbgMistimeEvents = mistimed-lock watchdog
     * strikes (+1 each) and trips (+100 each). */
    uint16_t dbgMinStepPeriod;
    uint16_t dbgMistimeEvents;
    uint16_t dbgFeSamples;      /* softneutral FE: fast-lane samples/sector (latched) */
    uint16_t dbgFeVoteResets;   /* softneutral FE: cumulative wrong-sample vote resets */
    uint16_t dbgFeD3Min;        /* FE diag: probe-sector d3 min, offset-binary +32768 */
    uint16_t dbgFeD3Max;        /* FE diag: probe-sector d3 max, offset-binary +32768 */
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
#define GSP_PROFILE_5055    3
#define GSP_PROFILE_COBRA   4  /* Cobra CM-2814/36 470KV (12N14P, 7PP, heavy, high-R) */
#define GSP_PROFILE_XROTOR  5  /* Hobbywing XRotor 3110 1150KV (12N14P, 7PP, ~2810 regime) */
#define GSP_PROFILE_VEX     6  /* VEX 14mm 4000KV (12N?, 6PP, 7.4V rated / 10V max, micro) */
#define GSP_PROFILE_1407_2S 7  /* 1407 4000KV 9N12P (6PP) @ 2S (8.4V) — FPV 3" */
#define GSP_PROFILE_1407_3S 8  /* 1407 4000KV 9N12P (6PP) @ 3S (12.6V) — FPV 3" */
#define GSP_PROFILE_U3      9  /* T-Motor U3 KV700 12N14P (7PP) @ 3-4S (~16V) — heavy 97g, propped bench */
#define GSP_PROFILE_CUSTOM  10
#define GSP_PROFILE_COUNT   10 /* built-in profiles (excl. Custom) */

/* Global instances (defined in gsp_params.c) */
extern GSP_PARAMS_T  gspParams;
extern GSP_DERIVED_T gspDerived;
extern const GSP_PARAMS_T profileDefaults[10];  /* factory image (const, in .hex) */
extern GSP_PARAMS_T gspParamTable[GSP_PROFILE_COUNT];  /* live RAM table, all profiles */

/**
 * Boot entry: load user-saved table (or factory) via the param store into
 * gspParamTable, activate the boot profile into gspParams, and apply
 * compile-time feature overrides.
 */
void GSP_ParamsInit(void);

/**
 * Persist the current gspParamTable (with the active profile's live
 * values folded in) plus activeProfile to the user flash area.
 * @return true on success, false on any NVM failure.
 */
bool GSP_ParamsSaveAll(void);

/**
 * Erase the user flash area and reload gspParamTable/gspParams from the
 * compiled factory defaults.
 * @return true on success, false on any NVM failure.
 */
bool GSP_ParamsFactoryReset(void);

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
 * @return Currently active profile ID (0-4).
 */
uint8_t GSP_ParamsGetActiveProfile(void);

#ifdef __cplusplus
}
#endif

#endif /* GSP_PARAMS_H */
