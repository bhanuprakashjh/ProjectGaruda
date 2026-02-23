/**
 * @file gsp_params.c
 *
 * @brief Runtime parameter system implementation.
 *
 * Table-driven validation, cross-parameter checks, and derived value
 * precomputation for ISR consumption. Uses FPU for derived calculations.
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

/* ── Descriptor table ────────────────────────────────────────────────── */

static const PARAM_DESCRIPTOR_T paramDescriptors[] = {
    {
        .id = PARAM_ID_RAMP_TARGET_ERPM,
        .type = PARAM_TYPE_U16,
        .group = PARAM_GROUP_STARTUP,
        .minVal = 500,
        .maxVal = 10000,
        .offsetInParams = offsetof(GSP_PARAMS_T, rampTargetErpm),
        .fieldSize = 2,
    },
    {
        .id = PARAM_ID_RAMP_ACCEL_ERPM_PER_S,
        .type = PARAM_TYPE_U16,
        .group = PARAM_GROUP_STARTUP,
        .minVal = 50,
        .maxVal = 5000,
        .offsetInParams = offsetof(GSP_PARAMS_T, rampAccelErpmPerS),
        .fieldSize = 2,
    },
    {
        .id = PARAM_ID_RAMP_DUTY_PCT,
        .type = PARAM_TYPE_U8,
        .group = PARAM_GROUP_STARTUP,
        .minVal = 5,
        .maxVal = 80,
        .offsetInParams = offsetof(GSP_PARAMS_T, rampDutyPct),
        .fieldSize = 1,
    },
    {
        .id = PARAM_ID_CL_IDLE_DUTY_PCT,
        .type = PARAM_TYPE_U8,
        .group = PARAM_GROUP_CLOSED_LOOP,
        .minVal = 0,
        .maxVal = 30,
        .offsetInParams = offsetof(GSP_PARAMS_T, clIdleDutyPct),
        .fieldSize = 1,
    },
    {
        .id = PARAM_ID_TIMING_ADV_MAX_DEG,
        .type = PARAM_TYPE_U8,
        .group = PARAM_GROUP_CLOSED_LOOP,
        .minVal = 0,
        .maxVal = 25,
        .offsetInParams = offsetof(GSP_PARAMS_T, timingAdvMaxDeg),
        .fieldSize = 1,
    },
    {
        .id = PARAM_ID_HWZC_CROSSOVER_ERPM,
        .type = PARAM_TYPE_U16,
        .group = PARAM_GROUP_CLOSED_LOOP,
        .minVal = 500,
        .maxVal = 20000,
        .offsetInParams = offsetof(GSP_PARAMS_T, hwzcCrossoverErpm),
        .fieldSize = 2,
    },
    {
        .id = PARAM_ID_OC_SW_LIMIT_MA,
        .type = PARAM_TYPE_U16,
        .group = PARAM_GROUP_OVERCURRENT,
        .minVal = 500,
        .maxVal = 30000,
        .offsetInParams = offsetof(GSP_PARAMS_T, ocSwLimitMa),
        .fieldSize = 2,
    },
    {
        .id = PARAM_ID_OC_FAULT_MA,
        .type = PARAM_TYPE_U16,
        .group = PARAM_GROUP_OVERCURRENT,
        .minVal = 1000,
        .maxVal = 30000,
        .offsetInParams = offsetof(GSP_PARAMS_T, ocFaultMa),
        .fieldSize = 2,
    },
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

/* ── OC mA to ADC conversion (matches garuda_calc_params.h OC_MV_TO_COUNTS) ── */

static uint16_t OcMaToAdcCounts(uint16_t ma)
{
    /* V_trip_mV = OC_VREF_MV + (mA * OC_SHUNT_MOHM * OC_GAIN_X100 / 100000)
     * ADC_counts = V_trip_mV * 4096 / OC_VADC_MV */
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
    memset(&gspParams, 0, sizeof(gspParams));

    gspParams.rampTargetErpm     = RAMP_TARGET_ERPM;
    gspParams.rampAccelErpmPerS  = RAMP_ACCEL_ERPM_PER_S;
    gspParams.rampDutyPct        = RAMP_DUTY_PERCENT;
    gspParams.clIdleDutyPct      = CL_IDLE_DUTY_PERCENT;
#if FEATURE_TIMING_ADVANCE
    gspParams.timingAdvMaxDeg    = TIMING_ADVANCE_MAX_DEG;
#else
    gspParams.timingAdvMaxDeg    = 0;
#endif
#if FEATURE_ADC_CMP_ZC
    gspParams.hwzcCrossoverErpm  = HWZC_CROSSOVER_ERPM;
#else
    gspParams.hwzcCrossoverErpm  = 0;
#endif
#if FEATURE_HW_OVERCURRENT
    gspParams.ocSwLimitMa        = OC_SW_LIMIT_MA;
    gspParams.ocFaultMa          = OC_FAULT_MA;
#else
    gspParams.ocSwLimitMa        = 0;
    gspParams.ocFaultMa          = 0;
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
}

PARAM_RESULT_T GSP_ParamSet(uint16_t id, uint32_t value)
{
    const PARAM_DESCRIPTOR_T *desc = FindDescriptor(id);
    if (desc == NULL)
        return PARAM_ERR_UNKNOWN_ID;

    /* Bounds check */
    if (value < desc->minVal || value > desc->maxVal)
        return PARAM_ERR_OUT_OF_RANGE;

    /* Cross-parameter validation: ocFaultMa must be >= ocSwLimitMa */
    if (id == PARAM_ID_OC_FAULT_MA && value < gspParams.ocSwLimitMa)
        return PARAM_ERR_CROSS_VALIDATION;
    if (id == PARAM_ID_OC_SW_LIMIT_MA && value > gspParams.ocFaultMa)
        return PARAM_ERR_CROSS_VALIDATION;

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

/* ── EEPROM persistence ──────────────────────────────────────────────── */

#define GSP_PERSIST_MARKER  0xA1
#define GSP_PERSIST_OFFSET  16  /* Byte offset within GARUDA_CONFIG_T.reserved */

void GSP_ParamsLoadFromConfig(const void *cfg)
{
    /* GARUDA_CONFIG_T.reserved starts at byte offset 16 in the struct.
     * We use memcpy to avoid alignment/aliasing issues. */
    const uint8_t *base = (const uint8_t *)cfg;
    GSP_CONFIG_PERSIST_T persist;
    memcpy(&persist, base + GSP_PERSIST_OFFSET, sizeof(persist));

    if (persist.schemaV1Marker != GSP_PERSIST_MARKER)
        return;  /* No saved params — keep compile-time defaults */

    /* Overlay persisted values. Validate each before applying. */
    gspParams.rampDutyPct       = persist.rampDutyPct;
    gspParams.clIdleDutyPct     = persist.clIdleDutyPct;
    gspParams.timingAdvMaxDeg   = persist.timingAdvMaxDeg;
    gspParams.rampTargetErpm    = persist.rampTargetErpm;
    gspParams.rampAccelErpmPerS = persist.rampAccelErpmPerS;
    gspParams.hwzcCrossoverErpm = persist.hwzcCrossoverErpm;
    gspParams.ocSwLimitMa       = persist.ocSwLimitMa;
    gspParams.ocFaultMa         = persist.ocFaultMa;
}

void GSP_ParamsSaveToConfig(void *cfg)
{
    uint8_t *base = (uint8_t *)cfg;
    GSP_CONFIG_PERSIST_T persist;
    memset(&persist, 0, sizeof(persist));

    persist.schemaV1Marker   = GSP_PERSIST_MARKER;
    persist.rampDutyPct      = gspParams.rampDutyPct;
    persist.clIdleDutyPct    = gspParams.clIdleDutyPct;
    persist.timingAdvMaxDeg  = gspParams.timingAdvMaxDeg;
    persist.rampTargetErpm   = gspParams.rampTargetErpm;
    persist.rampAccelErpmPerS= gspParams.rampAccelErpmPerS;
    persist.hwzcCrossoverErpm= gspParams.hwzcCrossoverErpm;
    persist.ocSwLimitMa      = gspParams.ocSwLimitMa;
    persist.ocFaultMa        = gspParams.ocFaultMa;

    memcpy(base + GSP_PERSIST_OFFSET, &persist, sizeof(persist));
}

#endif /* FEATURE_GSP */
