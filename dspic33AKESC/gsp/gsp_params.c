/**
 * @file gsp_params.c
 *
 * @brief Runtime parameter system implementation (Phase 1.5).
 *
 * Table-driven validation, cross-parameter checks, derived value
 * precomputation, profile defaults, and EEPROM V2 persistence.
 *
 * Component: GSP
 */

#include "garuda_config.h"

#if FEATURE_GSP

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "gsp_params.h"
#include "garuda_calc_params.h"

/* ── Global instances ────────────────────────────────────────────────── */

GSP_PARAMS_T  gspParams;
GSP_DERIVED_T gspDerived;

static uint8_t activeProfile;

/* ── Profile defaults (hardcoded, not macro-derived) ─────────────────
 * garuda_config.h only exposes one profile at compile time via #if.
 * These const arrays duplicate the literal values from each profile
 * block — intentional: stable per-motor constants that rarely change. */

/* Shared tuning defaults (same for all profiles) */
#define TUNING_DEFAULTS \
    .dutySlewUpPctPerMs   = 2,  \
    .dutySlewDownPctPerMs = 5,  \
    .postSyncSettleMs     = 1000, \
    .postSyncSlewDivisor  = 4,  \
    .zcBlankingPercent    = 3,  \
    .zcAdcDeadband        = 4,  \
    .zcSyncThreshold      = 6,  \
    .zcFilterThreshold    = 2,  \
    .vbusOvAdc            = 3600, \
    .vbusUvAdc            = 500,  \
    .desyncCoastMs        = 200,  \
    .desyncMaxRestarts    = 3

static const GSP_PARAMS_T profileDefaults[4] = {
    [GSP_PROFILE_HURST] = {
        .rampTargetErpm     = 2000,
        .rampAccelErpmPerS  = 1000,
        .rampDutyPct        = 40,
        .clIdleDutyPct      = 0,
        .timingAdvMaxDeg    = 15,
        .hwzcCrossoverErpm  = 5000,
        .ocSwLimitMa        = 1500,
        .ocFaultMa          = 3000,
        .motorPolePairs     = 5,
        .alignDutyPct       = 20,
        .initialErpm        = 300,
        .maxClosedLoopErpm  = 20000,
        .sineAlignModPct    = 15,
        .sineRampModPct     = 35,
        .zcDemagDutyThresh  = 70,
        .zcDemagBlankExtraPct = 12,
        .ocLimitMa          = 1800,
        .ocStartupMa        = 18000,
        .rampCurrentGateMa  = 0,
        TUNING_DEFAULTS,
        /* FOC motor model: Hurst DMB2424B10002 (Long Hurst / Hurst300)
         * Rs=0.534Ω (measured), Ls=359µH, Ke=0.00742 V·s/rad, 5PP, 24V */
        .focRsMilliOhm       = 534,    /* 0.534 Ω (auto-detect measured) */
        .focLsMicroH          = 471,    /* 0.471 mH (auto-detect measured) */
        .focKeUvSRad          = 7420,   /* 0.00742 V·s/rad (per-phase λ_pm) */
        .focVbusNomCentiV     = 2400,   /* 24.0V */
        .focMaxCurrentCentiA  = 1000,   /* 10.0A (2x rated peak) */
        .focMaxElecRadS       = 2000,
        .focKpDqMilli         = 1570,   /* 1.57 (ωbw×Ls = 2π×530×0.000471) */
        .focKiDq              = 1778,   /* Ki = ωbw × Rs = 2π×530 × 0.534 */
        .focObsLpfAlphaMilli  = 80,     /* 0.08 */
        .focAlignIqCentiA     = 100,    /* 1.0A (Microchip LOCK_CURRENT) */
        .focRampIqCentiA      = 150,    /* 1.5A — margin for SMO angle error */
        .focAlignTimeMs       = 500,
        .focIqRampTimeMs      = 200,
        .focRampRateRps2      = 500,
        .focHandoffRadS       = 262,    /* 500 RPM mech × 5PP × 2π/60 */
        .focFaultOcCentiA     = 1000,   /* 10.0A */
        .focFaultStallDeciRadS = 100,   /* 10.0 rad/s */
    },
    [GSP_PROFILE_A2212] = {
        .rampTargetErpm     = 3000,
        .rampAccelErpmPerS  = 300,
        .rampDutyPct        = 15,
        .clIdleDutyPct      = 12,
        .timingAdvMaxDeg    = 15,
        .hwzcCrossoverErpm  = 1500,
        .ocSwLimitMa        = 8000,
        .ocFaultMa          = 18000,
        .motorPolePairs     = 7,
        .alignDutyPct       = 8,
        .initialErpm        = 200,
        .maxClosedLoopErpm  = 120000,
        .sineAlignModPct    = 4,
        .sineRampModPct     = 12,
        .zcDemagDutyThresh  = 40,
        .zcDemagBlankExtraPct = 18,
        .ocLimitMa          = 12000,
        .ocStartupMa        = 22000,
        .rampCurrentGateMa  = 5000,
        TUNING_DEFAULTS,
        /* FOC motor model: A2212 1400KV (7PP, 12V, low-Rs) */
        .focRsMilliOhm       = 65,     /* 0.065 Ω */
        .focLsMicroH          = 30,     /* 30 µH */
        .focKeUvSRad          = 563,    /* 0.000563 V·s/rad */
        .focVbusNomCentiV     = 1200,   /* 12.0V */
        .focMaxCurrentCentiA  = 2000,   /* 20.0A */
        .focMaxElecRadS       = 12000,
        .focKpDqMilli         = 190,    /* 0.19 */
        .focKiDq              = 408,
        .focObsLpfAlphaMilli  = 350,    /* 0.35 */
        .focAlignIqCentiA     = 250,    /* 2.5A */
        .focRampIqCentiA      = 400,    /* 4.0A — 5A trips U25B during startup */
        .focAlignTimeMs       = 300,
        .focIqRampTimeMs      = 100,
        .focRampRateRps2      = 500,
        .focHandoffRadS       = 500,    /* 500 rad/s — proven reliable on A2212 */
        .focFaultOcCentiA     = 2500,   /* 25.0A */
        .focFaultStallDeciRadS = 500,   /* 50.0 rad/s */
    },
    [GSP_PROFILE_5010] = {
        .rampTargetErpm     = 2500,
        .rampAccelErpmPerS  = 200,
        .rampDutyPct        = 12,
        .clIdleDutyPct      = 10,
        .timingAdvMaxDeg    = 15,
        .hwzcCrossoverErpm  = 1500,
        .ocSwLimitMa        = 12000,
        .ocFaultMa          = 20000,
        .motorPolePairs     = 7,
        .alignDutyPct       = 6,
        .initialErpm        = 150,
        .maxClosedLoopErpm  = 80000,
        .sineAlignModPct    = 5,
        .sineRampModPct     = 10,
        .zcDemagDutyThresh  = 45,
        .zcDemagBlankExtraPct = 16,
        .ocLimitMa          = 15000,
        .ocStartupMa        = 22000,
        .rampCurrentGateMa  = 8000,
        TUNING_DEFAULTS,
        /* FOC motor model: Flycat 5010-750KV (7PP, 14.8V) */
        .focRsMilliOhm       = 80,     /* 0.080 Ω */
        .focLsMicroH          = 30,     /* 30 µH */
        .focKeUvSRad          = 1050,   /* 0.001050 V·s/rad */
        .focVbusNomCentiV     = 1480,   /* 14.8V */
        .focMaxCurrentCentiA  = 3000,   /* 30.0A */
        .focMaxElecRadS       = 8500,
        .focKpDqMilli         = 190,    /* 0.19 */
        .focKiDq              = 503,
        .focObsLpfAlphaMilli  = 200,    /* 0.20 */
        .focAlignIqCentiA     = 300,    /* 3.0A */
        .focRampIqCentiA      = 300,    /* 3.0A */
        .focAlignTimeMs       = 750,
        .focIqRampTimeMs      = 500,
        .focRampRateRps2      = 100,
        .focHandoffRadS       = 1000,
        .focFaultOcCentiA     = 3500,   /* 35.0A */
        .focFaultStallDeciRadS = 500,   /* 50.0 rad/s */
    },
    [GSP_PROFILE_5055] = {
        .rampTargetErpm     = 2000,
        .rampAccelErpmPerS  = 150,
        .rampDutyPct        = 8,
        .clIdleDutyPct      = 8,
        .timingAdvMaxDeg    = 15,
        .hwzcCrossoverErpm  = 1500,
        .ocSwLimitMa        = 10000,
        .ocFaultMa          = 20000,
        .motorPolePairs     = 7,
        .alignDutyPct       = 4,
        .initialErpm        = 100,
        .maxClosedLoopErpm  = 80000,
        .sineAlignModPct    = 4,
        .sineRampModPct     = 8,
        .zcDemagDutyThresh  = 45,
        .zcDemagBlankExtraPct = 16,
        .ocLimitMa          = 15000,
        .ocStartupMa        = 22000,
        .rampCurrentGateMa  = 6000,
        TUNING_DEFAULTS,
        /* FOC motor model: Generic 5055 ~580KV (7PP, 14.8V, very-low-Rs) */
        .focRsMilliOhm       = 50,     /* 0.050 Ω */
        .focLsMicroH          = 18,     /* 17.5 µH → 18 */
        .focKeUvSRad          = 1355,   /* 0.001355 V·s/rad */
        .focVbusNomCentiV     = 1480,   /* 14.8V */
        .focMaxCurrentCentiA  = 2500,   /* 25.0A */
        .focMaxElecRadS       = 7000,
        .focKpDqMilli         = 88,     /* 0.088 */
        .focKiDq              = 251,
        .focObsLpfAlphaMilli  = 200,    /* 0.20 */
        .focAlignIqCentiA     = 400,    /* 4.0A */
        .focRampIqCentiA      = 300,    /* 3.0A */
        .focAlignTimeMs       = 1000,
        .focIqRampTimeMs      = 500,
        .focRampRateRps2      = 80,
        .focHandoffRadS       = 800,
        .focFaultOcCentiA     = 3000,   /* 30.0A */
        .focFaultStallDeciRadS = 300,   /* 30.0 rad/s */
    },
};

/* Max safe mA for OC params: DAC ceiling 4095 counts = 3299 mV */
#define OC_MAX_SAFE_MA  22000

/* ── Descriptor table (31 entries) ───────────────────────────────────── */

static const PARAM_DESCRIPTOR_T paramDescriptors[] = {
    /* Stage 1: Startup & Ramp (group 0) */
    { PARAM_ID_RAMP_TARGET_ERPM,      PARAM_TYPE_U16, PARAM_GROUP_STARTUP,    500,    10000, offsetof(GSP_PARAMS_T, rampTargetErpm),     2 },
    { PARAM_ID_RAMP_ACCEL_ERPM_PER_S, PARAM_TYPE_U16, PARAM_GROUP_STARTUP,     50,     5000, offsetof(GSP_PARAMS_T, rampAccelErpmPerS),  2 },
    { PARAM_ID_RAMP_DUTY_PCT,         PARAM_TYPE_U8,  PARAM_GROUP_STARTUP,      5,       80, offsetof(GSP_PARAMS_T, rampDutyPct),        1 },
    { PARAM_ID_ALIGN_DUTY_PCT,        PARAM_TYPE_U8,  PARAM_GROUP_STARTUP,      2,       50, offsetof(GSP_PARAMS_T, alignDutyPct),       1 },
    { PARAM_ID_INITIAL_ERPM,          PARAM_TYPE_U16, PARAM_GROUP_STARTUP,     50,     1000, offsetof(GSP_PARAMS_T, initialErpm),        2 },
    { PARAM_ID_SINE_ALIGN_MOD_PCT,    PARAM_TYPE_U8,  PARAM_GROUP_STARTUP,      2,       50, offsetof(GSP_PARAMS_T, sineAlignModPct),    1 },
    { PARAM_ID_SINE_RAMP_MOD_PCT,     PARAM_TYPE_U8,  PARAM_GROUP_STARTUP,      5,       80, offsetof(GSP_PARAMS_T, sineRampModPct),     1 },
    /* Stage 1: Closed-Loop Control (group 1) */
    { PARAM_ID_CL_IDLE_DUTY_PCT,      PARAM_TYPE_U8,  PARAM_GROUP_CLOSED_LOOP,  0,       30, offsetof(GSP_PARAMS_T, clIdleDutyPct),      1 },
    { PARAM_ID_TIMING_ADV_MAX_DEG,    PARAM_TYPE_U8,  PARAM_GROUP_CLOSED_LOOP,  0,       25, offsetof(GSP_PARAMS_T, timingAdvMaxDeg),     1 },
    { PARAM_ID_HWZC_CROSSOVER_ERPM,   PARAM_TYPE_U16, PARAM_GROUP_CLOSED_LOOP, 500,   20000, offsetof(GSP_PARAMS_T, hwzcCrossoverErpm),  2 },
    { PARAM_ID_MAX_CL_ERPM,           PARAM_TYPE_U32, PARAM_GROUP_CLOSED_LOOP, 5000, 150000, offsetof(GSP_PARAMS_T, maxClosedLoopErpm),  4 },
    { PARAM_ID_ZC_DEMAG_DUTY_THRESH,  PARAM_TYPE_U8,  PARAM_GROUP_CLOSED_LOOP,  20,      90, offsetof(GSP_PARAMS_T, zcDemagDutyThresh),  1 },
    { PARAM_ID_ZC_DEMAG_BLANK_EXTRA,  PARAM_TYPE_U8,  PARAM_GROUP_CLOSED_LOOP,   0,      30, offsetof(GSP_PARAMS_T, zcDemagBlankExtraPct), 1 },
    /* Current Protection (group 2) */
    { PARAM_ID_OC_SW_LIMIT_MA,        PARAM_TYPE_U16, PARAM_GROUP_OVERCURRENT,  500, OC_MAX_SAFE_MA, offsetof(GSP_PARAMS_T, ocSwLimitMa),      2 },
    { PARAM_ID_OC_FAULT_MA,           PARAM_TYPE_U16, PARAM_GROUP_OVERCURRENT, 1000, OC_MAX_SAFE_MA, offsetof(GSP_PARAMS_T, ocFaultMa),        2 },
    { PARAM_ID_OC_LIMIT_MA,           PARAM_TYPE_U16, PARAM_GROUP_OVERCURRENT,  501, OC_MAX_SAFE_MA, offsetof(GSP_PARAMS_T, ocLimitMa),        2 },
    { PARAM_ID_OC_STARTUP_MA,         PARAM_TYPE_U16, PARAM_GROUP_OVERCURRENT, 5000, OC_MAX_SAFE_MA, offsetof(GSP_PARAMS_T, ocStartupMa),      2 },
    { PARAM_ID_RAMP_CURRENT_GATE_MA,  PARAM_TYPE_U16, PARAM_GROUP_OVERCURRENT,    0, OC_MAX_SAFE_MA, offsetof(GSP_PARAMS_T, rampCurrentGateMa), 2 },
    /* ZC Detection (group 3) */
    { PARAM_ID_ZC_BLANKING_PCT,       PARAM_TYPE_U8,  PARAM_GROUP_ZC_DETECT,    1,    15, offsetof(GSP_PARAMS_T, zcBlankingPercent),   1 },
    { PARAM_ID_ZC_ADC_DEADBAND,       PARAM_TYPE_U8,  PARAM_GROUP_ZC_DETECT,    0,    20, offsetof(GSP_PARAMS_T, zcAdcDeadband),       1 },
    { PARAM_ID_ZC_SYNC_THRESHOLD,     PARAM_TYPE_U8,  PARAM_GROUP_ZC_DETECT,    4,    20, offsetof(GSP_PARAMS_T, zcSyncThreshold),     1 },
    { PARAM_ID_ZC_FILTER_THRESHOLD,   PARAM_TYPE_U8,  PARAM_GROUP_ZC_DETECT,    1,    10, offsetof(GSP_PARAMS_T, zcFilterThreshold),   1 },
    /* Duty Slew (group 4) */
    { PARAM_ID_DUTY_SLEW_UP,          PARAM_TYPE_U8,  PARAM_GROUP_DUTY_SLEW,   1,    20, offsetof(GSP_PARAMS_T, dutySlewUpPctPerMs),  1 },
    { PARAM_ID_DUTY_SLEW_DOWN,        PARAM_TYPE_U8,  PARAM_GROUP_DUTY_SLEW,   1,    50, offsetof(GSP_PARAMS_T, dutySlewDownPctPerMs), 1 },
    { PARAM_ID_POST_SYNC_SETTLE_MS,   PARAM_TYPE_U16, PARAM_GROUP_DUTY_SLEW, 100,  5000, offsetof(GSP_PARAMS_T, postSyncSettleMs),    2 },
    { PARAM_ID_POST_SYNC_SLEW_DIV,    PARAM_TYPE_U8,  PARAM_GROUP_DUTY_SLEW,   1,    16, offsetof(GSP_PARAMS_T, postSyncSlewDivisor), 1 },
    /* Voltage Protection (group 5) */
    { PARAM_ID_VBUS_OV_ADC,           PARAM_TYPE_U16, PARAM_GROUP_VOLTAGE,   2000,  4000, offsetof(GSP_PARAMS_T, vbusOvAdc),          2 },
    { PARAM_ID_VBUS_UV_ADC,           PARAM_TYPE_U16, PARAM_GROUP_VOLTAGE,    200,  2000, offsetof(GSP_PARAMS_T, vbusUvAdc),          2 },
    /* Recovery (group 6) */
    { PARAM_ID_DESYNC_COAST_MS,       PARAM_TYPE_U16, PARAM_GROUP_RECOVERY,    50,  1000, offsetof(GSP_PARAMS_T, desyncCoastMs),      2 },
    { PARAM_ID_DESYNC_MAX_RESTARTS,   PARAM_TYPE_U8,  PARAM_GROUP_RECOVERY,     0,    10, offsetof(GSP_PARAMS_T, desyncMaxRestarts),  1 },
    /* Motor Hardware (group 7) */
    { PARAM_ID_MOTOR_POLE_PAIRS,      PARAM_TYPE_U8,  PARAM_GROUP_MOTOR_HW,    1,    20, offsetof(GSP_PARAMS_T, motorPolePairs),     1 },
    /* FOC Motor Model (group 8) */
    { PARAM_ID_FOC_RS_MOHM,           PARAM_TYPE_U16, PARAM_GROUP_FOC_MOTOR,   10, 10000, offsetof(GSP_PARAMS_T, focRsMilliOhm),     2 },
    { PARAM_ID_FOC_LS_UH,             PARAM_TYPE_U16, PARAM_GROUP_FOC_MOTOR,    1, 10000, offsetof(GSP_PARAMS_T, focLsMicroH),        2 },
    { PARAM_ID_FOC_KE_UV_S_RAD,       PARAM_TYPE_U16, PARAM_GROUP_FOC_MOTOR,    1, 65000, offsetof(GSP_PARAMS_T, focKeUvSRad),        2 },
    { PARAM_ID_FOC_VBUS_NOM_CV,       PARAM_TYPE_U16, PARAM_GROUP_FOC_MOTOR,  500,  6000, offsetof(GSP_PARAMS_T, focVbusNomCentiV),   2 },
    { PARAM_ID_FOC_MAX_CURRENT_CA,    PARAM_TYPE_U16, PARAM_GROUP_FOC_MOTOR,   50,  5000, offsetof(GSP_PARAMS_T, focMaxCurrentCentiA), 2 },
    { PARAM_ID_FOC_MAX_ELEC_RAD_S,    PARAM_TYPE_U16, PARAM_GROUP_FOC_MOTOR,  500, 30000, offsetof(GSP_PARAMS_T, focMaxElecRadS),     2 },
    /* FOC Tuning (group 9) */
    { PARAM_ID_FOC_KP_DQ_MILLI,       PARAM_TYPE_U16, PARAM_GROUP_FOC_TUNING,   1, 60000, offsetof(GSP_PARAMS_T, focKpDqMilli),       2 },
    { PARAM_ID_FOC_KI_DQ,             PARAM_TYPE_U16, PARAM_GROUP_FOC_TUNING,   1, 60000, offsetof(GSP_PARAMS_T, focKiDq),            2 },
    { PARAM_ID_FOC_OBS_LPF_MILLI,     PARAM_TYPE_U16, PARAM_GROUP_FOC_TUNING,  10,   900, offsetof(GSP_PARAMS_T, focObsLpfAlphaMilli), 2 },
    /* FOC Startup (group 10) */
    { PARAM_ID_FOC_ALIGN_IQ_CA,       PARAM_TYPE_U16, PARAM_GROUP_FOC_STARTUP,  1,  5000, offsetof(GSP_PARAMS_T, focAlignIqCentiA),   2 },
    { PARAM_ID_FOC_RAMP_IQ_CA,        PARAM_TYPE_U16, PARAM_GROUP_FOC_STARTUP,  1,  5000, offsetof(GSP_PARAMS_T, focRampIqCentiA),    2 },
    { PARAM_ID_FOC_ALIGN_TIME_MS,     PARAM_TYPE_U16, PARAM_GROUP_FOC_STARTUP, 100, 5000, offsetof(GSP_PARAMS_T, focAlignTimeMs),     2 },
    { PARAM_ID_FOC_IQ_RAMP_TIME_MS,   PARAM_TYPE_U16, PARAM_GROUP_FOC_STARTUP,  50, 2000, offsetof(GSP_PARAMS_T, focIqRampTimeMs),    2 },
    { PARAM_ID_FOC_RAMP_RATE_RPS2,    PARAM_TYPE_U16, PARAM_GROUP_FOC_STARTUP,  10, 5000, offsetof(GSP_PARAMS_T, focRampRateRps2),    2 },
    { PARAM_ID_FOC_HANDOFF_RAD_S,     PARAM_TYPE_U16, PARAM_GROUP_FOC_STARTUP,  50, 10000, offsetof(GSP_PARAMS_T, focHandoffRadS),    2 },
    { PARAM_ID_FOC_FAULT_OC_CA,       PARAM_TYPE_U16, PARAM_GROUP_FOC_STARTUP, 100, 5000, offsetof(GSP_PARAMS_T, focFaultOcCentiA),   2 },
    { PARAM_ID_FOC_FAULT_STALL_DRS,   PARAM_TYPE_U16, PARAM_GROUP_FOC_STARTUP,  10, 1000, offsetof(GSP_PARAMS_T, focFaultStallDeciRadS), 2 },
};

#define PARAM_COUNT (sizeof(paramDescriptors) / sizeof(paramDescriptors[0]))

/* ── Helpers ─────────────────────────────────────────────────────────── */

static const PARAM_DESCRIPTOR_T *FindDescriptor(uint16_t id)
{
    for (uint8_t i = 0; i < PARAM_COUNT; i++) {
        if (paramDescriptors[i].id == id)
            return &paramDescriptors[i];
    }
    return NULL;
}

static uint32_t ReadField(const PARAM_DESCRIPTOR_T *desc)
{
    const uint8_t *base = (const uint8_t *)&gspParams;
    if (desc->fieldSize == 1)
        return *(const uint8_t *)(base + desc->offsetInParams);
    else if (desc->fieldSize == 2) {
        uint16_t v;
        memcpy(&v, base + desc->offsetInParams, 2);
        return v;
    } else {
        uint32_t v;
        memcpy(&v, base + desc->offsetInParams, 4);
        return v;
    }
}

static void WriteField(const PARAM_DESCRIPTOR_T *desc, uint32_t value)
{
    uint8_t *base = (uint8_t *)&gspParams;
    if (desc->fieldSize == 1) {
        uint8_t v = (uint8_t)value;
        *(base + desc->offsetInParams) = v;
    } else if (desc->fieldSize == 2) {
        uint16_t v = (uint16_t)value;
        memcpy(base + desc->offsetInParams, &v, 2);
    } else {
        memcpy(base + desc->offsetInParams, &value, 4);
    }
}

/* ── OC mA to ADC conversion ────────────────────────────────────────── */

static uint16_t OcMaToAdcCounts(uint16_t ma)
{
#if FEATURE_HW_OVERCURRENT
    uint32_t tripMv = OC_VREF_MV +
        ((uint32_t)ma * OC_SHUNT_MOHM * OC_GAIN_X100 / 100000);
    return (uint16_t)((uint32_t)tripMv * 4096 / OC_VADC_MV);
#else
    (void)ma;
    return 0;
#endif
}

/* ── Public API ──────────────────────────────────────────────────────── */

void GSP_ParamsInitDefaults(void)
{
    /* Load from compile-time MOTOR_PROFILE */
    activeProfile = MOTOR_PROFILE;

#if MOTOR_PROFILE < GSP_PROFILE_COUNT
    memcpy(&gspParams, &profileDefaults[MOTOR_PROFILE], sizeof(gspParams));
#else
    /* Custom profile at compile time — use A2212 as base */
    memcpy(&gspParams, &profileDefaults[GSP_PROFILE_A2212], sizeof(gspParams));
#endif

    /* Override from compile-time config for features that may be disabled */
#if !FEATURE_TIMING_ADVANCE
    gspParams.timingAdvMaxDeg = 0;
#endif
#if !FEATURE_ADC_CMP_ZC
    gspParams.hwzcCrossoverErpm = 0;
#endif
#if !FEATURE_HW_OVERCURRENT
    gspParams.ocSwLimitMa = 0;
    gspParams.ocFaultMa = 0;
    gspParams.ocLimitMa = 0;
    gspParams.ocStartupMa = 0;
    gspParams.rampCurrentGateMa = 0;
#endif

#if FEATURE_FOC_V2
    extern volatile bool gspFocReinitNeeded;
    gspFocReinitNeeded = true;
#endif
}

void GSP_RecomputeDerived(void)
{
    GSP_PARAMS_T *p = &gspParams;
    GSP_DERIVED_T *d = &gspDerived;

    /* Ramp duty cap in PWM counts */
    d->rampDutyCap = (uint32_t)(p->rampDutyPct / 100.0f * LOOPTIME_TCY);

    /* CL idle duty floor */
    if (p->clIdleDutyPct > 0)
        d->clIdleDuty = (uint32_t)(p->clIdleDutyPct / 100.0f * LOOPTIME_TCY);
    else
        d->clIdleDuty = MIN_DUTY;

    /* Sine eRPM ramp rate (Q16 fractional per Timer1 tick) */
    d->sineErpmRampRateQ16 =
        (uint32_t)((uint64_t)p->rampAccelErpmPerS * 65536UL / 10000UL);

    /* Min step period from ramp target eRPM (Timer1 ticks) */
    if (p->rampTargetErpm > 0) {
        d->minStepPeriod = (uint16_t)(100000UL / p->rampTargetErpm);
        if (d->minStepPeriod < 1) d->minStepPeriod = 1;
    } else {
        d->minStepPeriod = 1;
    }

    /* Convert to ADC ISR ticks: Timer1_ticks * 12 / 5 */
#if FEATURE_BEMF_CLOSED_LOOP
    d->minAdcStepPeriod = (uint16_t)(((uint32_t)d->minStepPeriod * 12) / 5);
    if (d->minAdcStepPeriod < 1) d->minAdcStepPeriod = 1;
#else
    d->minAdcStepPeriod = 1;
#endif

    /* OC thresholds: mA → ADC counts */
    d->ocSwLimitAdc  = OcMaToAdcCounts(p->ocSwLimitMa);
    d->ocFaultAdcVal = OcMaToAdcCounts(p->ocFaultMa);

    /* ── New derived values (Phase 1.5) ─────────────────────────────── */

    /* Align duty in PWM counts */
    d->alignDuty = (uint32_t)(p->alignDutyPct / 100.0f * LOOPTIME_TCY);

    /* Initial step period from initial eRPM */
    if (p->initialErpm > 0) {
        d->initialStepPeriod = (uint16_t)(100000UL / p->initialErpm);
        if (d->initialStepPeriod < 1) d->initialStepPeriod = 1;
    } else {
        d->initialStepPeriod = 1;
    }

    /* Initial ADC step period */
#if FEATURE_BEMF_CLOSED_LOOP
    d->initialAdcStepPeriod = (uint16_t)(((uint32_t)d->initialStepPeriod * 12) / 5);
    if (d->initialAdcStepPeriod < 1) d->initialAdcStepPeriod = 1;
#else
    d->initialAdcStepPeriod = 1;
#endif

    /* Minimum CL ADC step period from max closed-loop eRPM */
#if FEATURE_BEMF_CLOSED_LOOP
    if (p->maxClosedLoopErpm > 0) {
        uint32_t maxClStepT1 = 100000UL / p->maxClosedLoopErpm;
        if (maxClStepT1 < 1) maxClStepT1 = 1;
        d->minClAdcStepPeriod = (uint16_t)(((uint32_t)maxClStepT1 * 12) / 5);
        if (d->minClAdcStepPeriod < 1) d->minClAdcStepPeriod = 1;
    } else {
        d->minClAdcStepPeriod = 1;
    }
#else
    d->minClAdcStepPeriod = 1;
#endif

    /* Sine amplitude limits in PWM counts */
#if FEATURE_SINE_STARTUP
    d->sineMinAmplitude = (uint32_t)(LOOPTIME_TCY * p->sineAlignModPct / 200);
    d->sineMaxAmplitude = (uint32_t)(LOOPTIME_TCY * p->sineRampModPct / 200);
#else
    d->sineMinAmplitude = 0;
    d->sineMaxAmplitude = 0;
#endif

    /* Duty slew rates (per ADC ISR tick) */
#if FEATURE_DUTY_SLEW
    d->dutySlewUpRate = (uint32_t)(
        (uint64_t)MAX_DUTY * p->dutySlewUpPctPerMs / 100
        / (PWMFREQUENCY_HZ / 1000));
    d->dutySlewDownRate = (uint32_t)(
        (uint64_t)MAX_DUTY * p->dutySlewDownPctPerMs / 100
        / (PWMFREQUENCY_HZ / 1000));
    d->postSyncSettleTicks = (uint16_t)(
        (uint32_t)p->postSyncSettleMs * PWMFREQUENCY_HZ / 1000);
#else
    d->dutySlewUpRate = 0;
    d->dutySlewDownRate = 0;
    d->postSyncSettleTicks = 0;
#endif

    /* OC CMP3 thresholds with hardware safety clamp */
#if FEATURE_HW_OVERCURRENT
    d->ocCmp3DacVal = OcMaToAdcCounts(p->ocLimitMa);
    if (d->ocCmp3DacVal >= 4096) d->ocCmp3DacVal = 4095;

    d->ocCmp3StartupDac = OcMaToAdcCounts(p->ocStartupMa);
    if (d->ocCmp3StartupDac >= 4096) d->ocCmp3StartupDac = 4095;

    if (p->rampCurrentGateMa > 0)
        d->rampCurrentGateAdc = OcMaToAdcCounts(p->rampCurrentGateMa);
    else
        d->rampCurrentGateAdc = 0;
#else
    d->ocCmp3DacVal = 0;
    d->ocCmp3StartupDac = 0;
    d->rampCurrentGateAdc = 0;
#endif

    /* Desync coast-down counts (Timer1 = 100us ticks) */
#if FEATURE_DESYNC_RECOVERY
    d->desyncCoastCounts = (uint32_t)(p->desyncCoastMs * 10);
#else
    d->desyncCoastCounts = 0;
#endif

    /* HWZC step period limits + noise floor clamping */
#if FEATURE_ADC_CMP_ZC
    if (p->maxClosedLoopErpm > 0)
        d->hwzcMinStepTicks = 1000000000UL / p->maxClosedLoopErpm;
    else
        d->hwzcMinStepTicks = 1000000000UL;

    d->hwzcNoiseFloorTicks = d->hwzcMinStepTicks * 2 / 3;
    /* Fix 1: clamp noise floor to PWM noise interval + 20% margin */
    {
        uint32_t pwmNoiseInterval = HWZC_TIMER_HZ / PWMFREQUENCY_HZ; /* 4167 */
        uint32_t minFloor = pwmNoiseInterval + pwmNoiseInterval / 5;  /* +20% */
        if (d->hwzcNoiseFloorTicks < minFloor)
            d->hwzcNoiseFloorTicks = minFloor;
    }
#else
    d->hwzcMinStepTicks = 0;
    d->hwzcNoiseFloorTicks = 0;
#endif

    /* Relaxed UV threshold during pre-sync startup (80% of normal UV) */
    d->vbusUvStartupAdc = (uint16_t)(p->vbusUvAdc * 4 / 5);
    if (d->vbusUvStartupAdc < 200) d->vbusUvStartupAdc = 200;
}

/* ── Cross-parameter validation (bilateral, 10 checks) ──────────────── */

static PARAM_RESULT_T CrossValidate(uint16_t id, uint32_t value)
{
    GSP_PARAMS_T *p = &gspParams;

    switch (id) {
    /* rampTargetErpm > initialErpm (bilateral) */
    case PARAM_ID_RAMP_TARGET_ERPM:
        if (value <= p->initialErpm)
            return PARAM_ERR_CROSS_VALIDATION;
        if (value >= p->maxClosedLoopErpm)
            return PARAM_ERR_CROSS_VALIDATION;
        break;
    case PARAM_ID_INITIAL_ERPM:
        if (value >= p->rampTargetErpm)
            return PARAM_ERR_CROSS_VALIDATION;
        break;

    /* maxClosedLoopErpm > rampTargetErpm (bilateral) */
    case PARAM_ID_MAX_CL_ERPM:
        if (value <= p->rampTargetErpm)
            return PARAM_ERR_CROSS_VALIDATION;
        break;

    /* zcSyncThreshold >= MORPH_ZC_THRESHOLD (4) */
    case PARAM_ID_ZC_SYNC_THRESHOLD:
#if FEATURE_SINE_STARTUP
        if (value < MORPH_ZC_THRESHOLD)
            return PARAM_ERR_CROSS_VALIDATION;
#endif
        if (value <= p->zcFilterThreshold)
            return PARAM_ERR_CROSS_VALIDATION;
        break;

    /* zcFilterThreshold < zcSyncThreshold (bilateral) */
    case PARAM_ID_ZC_FILTER_THRESHOLD:
        if (value >= p->zcSyncThreshold)
            return PARAM_ERR_CROSS_VALIDATION;
        break;

    /* OC chain: ocSwLimitMa < ocLimitMa <= ocFaultMa */
    case PARAM_ID_OC_SW_LIMIT_MA:
        if (value >= p->ocLimitMa)
            return PARAM_ERR_CROSS_VALIDATION;
        if (value > p->ocFaultMa)
            return PARAM_ERR_CROSS_VALIDATION;
        break;
    case PARAM_ID_OC_LIMIT_MA:
        if (value <= p->ocSwLimitMa)
            return PARAM_ERR_CROSS_VALIDATION;
        if (value > p->ocFaultMa)
            return PARAM_ERR_CROSS_VALIDATION;
        /* ocStartupMa >= ocLimitMa */
        if (p->ocStartupMa < value)
            return PARAM_ERR_CROSS_VALIDATION;
        /* rampCurrentGateMa < ocLimitMa (if nonzero) */
        if (p->rampCurrentGateMa != 0 && p->rampCurrentGateMa >= value)
            return PARAM_ERR_CROSS_VALIDATION;
        break;
    case PARAM_ID_OC_FAULT_MA:
        if (value < p->ocLimitMa)
            return PARAM_ERR_CROSS_VALIDATION;
        if (value < p->ocSwLimitMa)
            return PARAM_ERR_CROSS_VALIDATION;
        break;
    case PARAM_ID_OC_STARTUP_MA:
        if (value < p->ocLimitMa)
            return PARAM_ERR_CROSS_VALIDATION;
        break;
    case PARAM_ID_RAMP_CURRENT_GATE_MA:
        if (value != 0 && value >= p->ocLimitMa)
            return PARAM_ERR_CROSS_VALIDATION;
        break;

    /* vbusOvAdc > vbusUvAdc (bilateral) */
    case PARAM_ID_VBUS_OV_ADC:
        if (value <= p->vbusUvAdc)
            return PARAM_ERR_CROSS_VALIDATION;
        break;
    case PARAM_ID_VBUS_UV_ADC:
        if (value >= p->vbusOvAdc)
            return PARAM_ERR_CROSS_VALIDATION;
        break;

    default:
        break;
    }

    return PARAM_OK;
}

PARAM_RESULT_T GSP_ParamSet(uint16_t id, uint32_t value)
{
    const PARAM_DESCRIPTOR_T *desc = FindDescriptor(id);
    if (desc == NULL)
        return PARAM_ERR_UNKNOWN_ID;

    /* Bounds check */
    if (value < desc->minVal || value > desc->maxVal)
        return PARAM_ERR_OUT_OF_RANGE;

    /* Cross-parameter validation */
    PARAM_RESULT_T cv = CrossValidate(id, value);
    if (cv != PARAM_OK)
        return cv;

    /* Write field and recompute derived */
    WriteField(desc, value);
    GSP_RecomputeDerived();

    /* Signal FOC re-init if a FOC param was changed */
#if FEATURE_FOC_V2
    if (id >= PARAM_ID_FOC_RS_MOHM && id <= PARAM_ID_FOC_FAULT_STALL_DRS) {
        extern volatile bool gspFocReinitNeeded;
        gspFocReinitNeeded = true;
    }
#endif

    return PARAM_OK;
}

bool GSP_ParamGet(uint16_t id, uint32_t *out)
{
    const PARAM_DESCRIPTOR_T *desc = FindDescriptor(id);
    if (desc == NULL)
        return false;

    *out = ReadField(desc);
    return true;
}

uint8_t GSP_ParamGetCount(void)
{
    return (uint8_t)PARAM_COUNT;
}

const PARAM_DESCRIPTOR_T *GSP_ParamGetDescriptor(uint8_t idx)
{
    if (idx >= PARAM_COUNT)
        return NULL;
    return &paramDescriptors[idx];
}

/* ── Profile management ──────────────────────────────────────────────── */

bool GSP_ParamsLoadProfile(uint8_t profileId)
{
    if (profileId < GSP_PROFILE_COUNT) {
        /* Built-in profile: copy defaults */
        memcpy(&gspParams, &profileDefaults[profileId], sizeof(gspParams));
        activeProfile = profileId;
        GSP_RecomputeDerived();
        /* Signal FOC re-init needed (checked by ISR when IDLE) */
#if FEATURE_FOC_V2
        extern volatile bool gspFocReinitNeeded;
        gspFocReinitNeeded = true;
#endif
        return true;
    } else if (profileId == GSP_PROFILE_CUSTOM) {
        /* Custom: adopt current values as-is, just mark profile */
        activeProfile = GSP_PROFILE_CUSTOM;
        return true;
    }
    return false;
}

uint8_t GSP_ParamsGetActiveProfile(void)
{
    return activeProfile;
}

/* ── EEPROM persistence (V2/V3) ──────────────────────────────────────── */

#define GSP_PERSIST_OFFSET  16  /* Byte offset within GARUDA_CONFIG_T.reserved */

/**
 * Clamp all gspParams fields to descriptor bounds, then verify
 * cross-parameter invariants.  Returns true if all invariants hold
 * (possibly after clamping), false if invariants are violated
 * and the caller should fall back to profile defaults.
 */
static bool SanitizeLoadedParams(void)
{
    /* Validate activeProfile */
    if (activeProfile > GSP_PROFILE_CUSTOM)
        return false;

    /* Pass 1: clamp every field to its descriptor [min, max] */
    for (uint8_t i = 0; i < PARAM_COUNT; i++) {
        const PARAM_DESCRIPTOR_T *d = &paramDescriptors[i];
        uint32_t v = ReadField(d);
        if (v < d->minVal)      { WriteField(d, d->minVal); }
        else if (v > d->maxVal) { WriteField(d, d->maxVal); }
    }

    /* Pass 2: verify cross-parameter invariants.
     * If any fails, the entire param set is suspect. */
    const GSP_PARAMS_T *p = &gspParams;

    if (p->rampTargetErpm <= p->initialErpm)            return false;
    if (p->maxClosedLoopErpm <= p->rampTargetErpm)      return false;
#if FEATURE_SINE_STARTUP
    if (p->zcSyncThreshold < MORPH_ZC_THRESHOLD)        return false;
#endif
    if (p->zcFilterThreshold >= p->zcSyncThreshold)     return false;
    if (p->ocSwLimitMa >= p->ocLimitMa)                 return false;
    if (p->ocLimitMa > p->ocFaultMa)                    return false;
    if (p->ocStartupMa < p->ocLimitMa)                  return false;
    if (p->rampCurrentGateMa != 0 &&
        p->rampCurrentGateMa >= p->ocLimitMa)           return false;
    if (p->vbusOvAdc <= p->vbusUvAdc)                   return false;

    return true;
}

/** Fall back to profile defaults when sanitization fails. */
static void FallbackToProfileDefaults(void)
{
    uint8_t fallback = (activeProfile < GSP_PROFILE_COUNT)
                        ? activeProfile : (uint8_t)MOTOR_PROFILE;
    if (fallback >= GSP_PROFILE_COUNT)
        fallback = GSP_PROFILE_A2212;
    memcpy(&gspParams, &profileDefaults[fallback], sizeof(gspParams));
    activeProfile = fallback;
}

void GSP_ParamsLoadFromConfig(const void *cfg)
{
    const uint8_t *base = (const uint8_t *)cfg;
    uint8_t marker;
    memcpy(&marker, base + GSP_PERSIST_OFFSET, 1);

    if (marker == GSP_PERSIST_V3_MARKER) {
        /* V3 schema: V2 fields + 17 FOC params */
        GSP_CONFIG_PERSIST_V3_T persist;
        memcpy(&persist, base + GSP_PERSIST_OFFSET, sizeof(persist));

        activeProfile                   = persist.activeProfile;
        gspParams.rampDutyPct           = persist.rampDutyPct;
        gspParams.clIdleDutyPct         = persist.clIdleDutyPct;
        gspParams.timingAdvMaxDeg       = persist.timingAdvMaxDeg;
        gspParams.rampTargetErpm        = persist.rampTargetErpm;
        gspParams.rampAccelErpmPerS     = persist.rampAccelErpmPerS;
        gspParams.hwzcCrossoverErpm     = persist.hwzcCrossoverErpm;
        gspParams.ocSwLimitMa           = persist.ocSwLimitMa;
        gspParams.ocFaultMa             = persist.ocFaultMa;
        gspParams.motorPolePairs        = persist.motorPolePairs;
        gspParams.alignDutyPct          = persist.alignDutyPct;
        gspParams.initialErpm           = persist.initialErpm;
        gspParams.sineAlignModPct       = persist.sineAlignModPct;
        gspParams.sineRampModPct        = persist.sineRampModPct;
        gspParams.zcDemagDutyThresh     = persist.zcDemagDutyThresh;
        gspParams.zcDemagBlankExtraPct  = persist.zcDemagBlankExtraPct;
        gspParams.ocLimitMa             = persist.ocLimitMa;
        gspParams.ocStartupMa           = persist.ocStartupMa;
        gspParams.rampCurrentGateMa     = persist.rampCurrentGateMa;
        gspParams.maxClosedLoopErpm     = ((uint32_t)persist.maxClErpmHi << 16) |
                                           persist.maxClosedLoopErpmLo;
        gspParams.dutySlewUpPctPerMs    = persist.dutySlewUpPctPerMs;
        gspParams.dutySlewDownPctPerMs  = persist.dutySlewDownPctPerMs;
        gspParams.postSyncSettleMs      = persist.postSyncSettleMs;
        gspParams.postSyncSlewDivisor   = persist.postSyncSlewDivisor;
        gspParams.zcBlankingPercent     = persist.zcBlankingPercent;
        gspParams.zcAdcDeadband         = persist.zcAdcDeadband;
        gspParams.zcSyncThreshold       = persist.zcSyncThreshold;
        gspParams.zcFilterThreshold     = persist.zcFilterThreshold;
        gspParams.vbusOvAdc             = persist.vbusOvAdc;
        gspParams.vbusUvAdc             = persist.vbusUvAdc;
        gspParams.desyncCoastMs         = persist.desyncCoastMs;
        gspParams.desyncMaxRestarts     = persist.desyncMaxRestarts;
        /* V3 FOC fields */
        gspParams.focRsMilliOhm        = persist.focRsMilliOhm;
        gspParams.focLsMicroH           = persist.focLsMicroH;
        gspParams.focKeUvSRad           = persist.focKeUvSRad;
        gspParams.focVbusNomCentiV      = persist.focVbusNomCentiV;
        gspParams.focMaxCurrentCentiA   = persist.focMaxCurrentCentiA;
        gspParams.focMaxElecRadS        = persist.focMaxElecRadS;
        gspParams.focKpDqMilli          = persist.focKpDqMilli;
        gspParams.focKiDq               = persist.focKiDq;
        gspParams.focObsLpfAlphaMilli   = persist.focObsLpfAlphaMilli;
        gspParams.focAlignIqCentiA      = persist.focAlignIqCentiA;
        gspParams.focRampIqCentiA       = persist.focRampIqCentiA;
        gspParams.focAlignTimeMs        = persist.focAlignTimeMs;
        gspParams.focIqRampTimeMs       = persist.focIqRampTimeMs;
        gspParams.focRampRateRps2       = persist.focRampRateRps2;
        gspParams.focHandoffRadS        = persist.focHandoffRadS;
        gspParams.focFaultOcCentiA      = persist.focFaultOcCentiA;
        gspParams.focFaultStallDeciRadS = persist.focFaultStallDeciRadS;

        if (!SanitizeLoadedParams())
            FallbackToProfileDefaults();

    } else if (marker == GSP_PERSIST_V2_MARKER) {
        /* V2 schema: load 31 6-step params, FOC params get profile defaults */
        GSP_CONFIG_PERSIST_V2_T persist;
        memcpy(&persist, base + GSP_PERSIST_OFFSET, sizeof(persist));

        activeProfile                   = persist.activeProfile;
        gspParams.rampDutyPct           = persist.rampDutyPct;
        gspParams.clIdleDutyPct         = persist.clIdleDutyPct;
        gspParams.timingAdvMaxDeg       = persist.timingAdvMaxDeg;
        gspParams.rampTargetErpm        = persist.rampTargetErpm;
        gspParams.rampAccelErpmPerS     = persist.rampAccelErpmPerS;
        gspParams.hwzcCrossoverErpm     = persist.hwzcCrossoverErpm;
        gspParams.ocSwLimitMa           = persist.ocSwLimitMa;
        gspParams.ocFaultMa             = persist.ocFaultMa;
        gspParams.motorPolePairs        = persist.motorPolePairs;
        gspParams.alignDutyPct          = persist.alignDutyPct;
        gspParams.initialErpm           = persist.initialErpm;
        gspParams.sineAlignModPct       = persist.sineAlignModPct;
        gspParams.sineRampModPct        = persist.sineRampModPct;
        gspParams.zcDemagDutyThresh     = persist.zcDemagDutyThresh;
        gspParams.zcDemagBlankExtraPct  = persist.zcDemagBlankExtraPct;
        gspParams.ocLimitMa             = persist.ocLimitMa;
        gspParams.ocStartupMa           = persist.ocStartupMa;
        gspParams.rampCurrentGateMa     = persist.rampCurrentGateMa;
        gspParams.maxClosedLoopErpm     = ((uint32_t)persist.maxClErpmHi << 16) |
                                           persist.maxClosedLoopErpmLo;
        gspParams.dutySlewUpPctPerMs    = persist.dutySlewUpPctPerMs;
        gspParams.dutySlewDownPctPerMs  = persist.dutySlewDownPctPerMs;
        gspParams.postSyncSettleMs      = persist.postSyncSettleMs;
        gspParams.postSyncSlewDivisor   = persist.postSyncSlewDivisor;
        gspParams.zcBlankingPercent     = persist.zcBlankingPercent;
        gspParams.zcAdcDeadband         = persist.zcAdcDeadband;
        gspParams.zcSyncThreshold       = persist.zcSyncThreshold;
        gspParams.zcFilterThreshold     = persist.zcFilterThreshold;
        gspParams.vbusOvAdc             = persist.vbusOvAdc;
        gspParams.vbusUvAdc             = persist.vbusUvAdc;
        gspParams.desyncCoastMs         = persist.desyncCoastMs;
        gspParams.desyncMaxRestarts     = persist.desyncMaxRestarts;
        /* FOC params keep their profile defaults from InitDefaults() */

        if (!SanitizeLoadedParams())
            FallbackToProfileDefaults();

    } else if (marker == GSP_PERSIST_V1_MARKER) {
        /* V1 schema: load 8 Stage 1 params, keep profile defaults for rest.
         * activeProfile = MOTOR_PROFILE (compile-time, V1 migration rule). */
        GSP_CONFIG_PERSIST_V1_T persist;
        memcpy(&persist, base + GSP_PERSIST_OFFSET, sizeof(persist));

        gspParams.rampDutyPct       = persist.rampDutyPct;
        gspParams.clIdleDutyPct     = persist.clIdleDutyPct;
        gspParams.timingAdvMaxDeg   = persist.timingAdvMaxDeg;
        gspParams.rampTargetErpm    = persist.rampTargetErpm;
        gspParams.rampAccelErpmPerS = persist.rampAccelErpmPerS;
        gspParams.hwzcCrossoverErpm = persist.hwzcCrossoverErpm;
        gspParams.ocSwLimitMa       = persist.ocSwLimitMa;
        gspParams.ocFaultMa         = persist.ocFaultMa;
        /* 23 new params keep their profile defaults from InitDefaults() */

        /* Sanitize: clamp to bounds + check cross-parameter invariants */
        if (!SanitizeLoadedParams())
            FallbackToProfileDefaults();
    }
    /* else: unknown marker — keep compile-time defaults */
}

void GSP_ParamsSaveToConfig(void *cfg)
{
    uint8_t *base = (uint8_t *)cfg;
    GSP_CONFIG_PERSIST_V3_T persist;
    memset(&persist, 0, sizeof(persist));

    persist.schemaMarker           = GSP_PERSIST_V3_MARKER;
    persist.activeProfile          = activeProfile;
    persist.rampDutyPct            = gspParams.rampDutyPct;
    persist.clIdleDutyPct          = gspParams.clIdleDutyPct;
    persist.timingAdvMaxDeg        = gspParams.timingAdvMaxDeg;
    persist.rampTargetErpm         = gspParams.rampTargetErpm;
    persist.rampAccelErpmPerS      = gspParams.rampAccelErpmPerS;
    persist.hwzcCrossoverErpm      = gspParams.hwzcCrossoverErpm;
    persist.ocSwLimitMa            = gspParams.ocSwLimitMa;
    persist.ocFaultMa              = gspParams.ocFaultMa;
    persist.motorPolePairs         = gspParams.motorPolePairs;
    persist.alignDutyPct           = gspParams.alignDutyPct;
    persist.initialErpm            = gspParams.initialErpm;
    persist.sineAlignModPct        = gspParams.sineAlignModPct;
    persist.sineRampModPct         = gspParams.sineRampModPct;
    persist.zcDemagDutyThresh      = gspParams.zcDemagDutyThresh;
    persist.zcDemagBlankExtraPct   = gspParams.zcDemagBlankExtraPct;
    persist.ocLimitMa              = gspParams.ocLimitMa;
    persist.ocStartupMa            = gspParams.ocStartupMa;
    persist.rampCurrentGateMa      = gspParams.rampCurrentGateMa;
    persist.maxClosedLoopErpmLo    = (uint16_t)(gspParams.maxClosedLoopErpm & 0xFFFF);
    persist.maxClErpmHi            = (uint8_t)((gspParams.maxClosedLoopErpm >> 16) & 0xFF);
    persist.dutySlewUpPctPerMs     = gspParams.dutySlewUpPctPerMs;
    persist.dutySlewDownPctPerMs   = gspParams.dutySlewDownPctPerMs;
    persist.postSyncSettleMs       = gspParams.postSyncSettleMs;
    persist.postSyncSlewDivisor    = gspParams.postSyncSlewDivisor;
    persist.zcBlankingPercent      = gspParams.zcBlankingPercent;
    persist.zcAdcDeadband          = gspParams.zcAdcDeadband;
    persist.zcSyncThreshold        = gspParams.zcSyncThreshold;
    persist.zcFilterThreshold      = gspParams.zcFilterThreshold;
    persist.vbusOvAdc              = gspParams.vbusOvAdc;
    persist.vbusUvAdc              = gspParams.vbusUvAdc;
    persist.desyncCoastMs          = gspParams.desyncCoastMs;
    persist.desyncMaxRestarts      = gspParams.desyncMaxRestarts;
    /* V3 FOC fields */
    persist.focRsMilliOhm          = gspParams.focRsMilliOhm;
    persist.focLsMicroH            = gspParams.focLsMicroH;
    persist.focKeUvSRad            = gspParams.focKeUvSRad;
    persist.focVbusNomCentiV       = gspParams.focVbusNomCentiV;
    persist.focMaxCurrentCentiA    = gspParams.focMaxCurrentCentiA;
    persist.focMaxElecRadS         = gspParams.focMaxElecRadS;
    persist.focKpDqMilli           = gspParams.focKpDqMilli;
    persist.focKiDq                = gspParams.focKiDq;
    persist.focObsLpfAlphaMilli    = gspParams.focObsLpfAlphaMilli;
    persist.focAlignIqCentiA       = gspParams.focAlignIqCentiA;
    persist.focRampIqCentiA        = gspParams.focRampIqCentiA;
    persist.focAlignTimeMs         = gspParams.focAlignTimeMs;
    persist.focIqRampTimeMs        = gspParams.focIqRampTimeMs;
    persist.focRampRateRps2        = gspParams.focRampRateRps2;
    persist.focHandoffRadS         = gspParams.focHandoffRadS;
    persist.focFaultOcCentiA       = gspParams.focFaultOcCentiA;
    persist.focFaultStallDeciRadS  = gspParams.focFaultStallDeciRadS;

    memcpy(base + GSP_PERSIST_OFFSET, &persist, sizeof(persist));
}

#endif /* FEATURE_GSP */
