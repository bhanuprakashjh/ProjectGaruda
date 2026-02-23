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

static const GSP_PARAMS_T profileDefaults[3] = {
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
        /* Shared tuning defaults */
        .dutySlewUpPctPerMs   = 2,
        .dutySlewDownPctPerMs = 5,
        .postSyncSettleMs     = 1000,
        .postSyncSlewDivisor  = 4,
        .zcBlankingPercent    = 3,
        .zcAdcDeadband        = 4,
        .zcSyncThreshold      = 6,
        .zcFilterThreshold    = 2,
        .vbusOvAdc            = 3600,
        .vbusUvAdc            = 500,
        .desyncCoastMs        = 200,
        .desyncMaxRestarts    = 3,
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
        .dutySlewUpPctPerMs   = 2,
        .dutySlewDownPctPerMs = 5,
        .postSyncSettleMs     = 1000,
        .postSyncSlewDivisor  = 4,
        .zcBlankingPercent    = 3,
        .zcAdcDeadband        = 4,
        .zcSyncThreshold      = 6,
        .zcFilterThreshold    = 2,
        .vbusOvAdc            = 3600,
        .vbusUvAdc            = 500,
        .desyncCoastMs        = 200,
        .desyncMaxRestarts    = 3,
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
        .dutySlewUpPctPerMs   = 2,
        .dutySlewDownPctPerMs = 5,
        .postSyncSettleMs     = 1000,
        .postSyncSlewDivisor  = 4,
        .zcBlankingPercent    = 3,
        .zcAdcDeadband        = 4,
        .zcSyncThreshold      = 6,
        .zcFilterThreshold    = 2,
        .vbusOvAdc            = 3600,
        .vbusUvAdc            = 500,
        .desyncCoastMs        = 200,
        .desyncMaxRestarts    = 3,
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

/* ── EEPROM persistence (V2) ─────────────────────────────────────────── */

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

    if (marker == GSP_PERSIST_V2_MARKER) {
        /* V2 schema: load all 31 params + activeProfile */
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

        /* Sanitize: clamp to bounds + check cross-parameter invariants */
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
    GSP_CONFIG_PERSIST_V2_T persist;
    memset(&persist, 0, sizeof(persist));

    persist.schemaMarker           = GSP_PERSIST_V2_MARKER;
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

    memcpy(base + GSP_PERSIST_OFFSET, &persist, sizeof(persist));
}

#endif /* FEATURE_GSP */
