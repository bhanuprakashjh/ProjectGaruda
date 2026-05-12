/**
 * @file gsp_ck_params.c
 * @brief CK board runtime parameters — profiles, descriptors, get/set, derived.
 */

#include "../garuda_config.h"
#include "gsp_ck_params.h"

/* ── Global instances ── */
CK_PARAMS_T  ckParams;
CK_DERIVED_T ckDerived;

/* ── Helper: compute percent x10 from compile-time duty expression ── */
/* LOOPTIME_TCY/N → pct = 100*10/N = 1000/N (in x10 units) */

/* ── Profile defaults ── */

static void LoadHurstDefaults(void)
{
    ckParams.polePairs         = 5;
    ckParams.motorKv           = 149;
    ckParams.motorRsMilliOhm   = 534;
    ckParams.motorLsMicroH     = 471;
    ckParams.alignTimeMs       = 200;
    ckParams.alignDutyPctX10   = 50;   /* 5.0% → LOOPTIME_TCY/20 */
    ckParams.initialStepPeriod = 1000;
    ckParams.minStepPeriod     = 66;
    ckParams.rampAccelErpmS    = 1500;
    ckParams.rampDutyPctX10    = 167;  /* ~16.7% → LOOPTIME_TCY/6 */
    ckParams.rampTargetErpm    = 3000;
    ckParams.clIdleDutyPct     = 8;
    ckParams.timingAdvMinDeg   = 0;
    ckParams.timingAdvMaxDeg   = 8;
    ckParams.timingAdvStartErpm = 6000;
    ckParams.maxClosedLoopErpm = 15000UL;
    ckParams.dutySlewUp        = 25;
    ckParams.dutySlewDown      = 10;
    ckParams.ilimEnable        = 1;
    ckParams.ilimShutdownEnable = 0;
    ckParams.ilimDac           = 80;
    ckParams.ilimFilterTime    = 6;
    ckParams.scEnable          = 1;
    ckParams.scThreshold       = 7;
    ckParams.scFilterTime      = 7;
    ckParams.bemfEnable        = 1;
    ckParams.crossConductionTime = 7;
    ckParams.edgeBlankingTime  = 1;
    ckParams.csaGain           = 1;    /* 16x */
    ckParams.zcDeglitchMin     = 3;
    ckParams.zcDeglitchMax     = 8;
    ckParams.zcTimeoutMult     = 2;
    ckParams.zcDesyncThresh    = 3;
    ckParams.zcMissLimit       = 12;
    ckParams.vbusOvThreshold   = 51200U;  /* 3200*16 ~30V */
    ckParams.vbusUvThreshold   = 11200U;  /* 700*16  ~7V */
    ckParams.desyncRestartMax  = 3;
    ckParams.recoveryTimeMs    = 200;
    ckParams.activeProfile     = CK_PROFILE_HURST;
}

static void LoadA2212Defaults(void)
{
    ckParams.polePairs         = 7;
    ckParams.motorKv           = 1400;
    ckParams.motorRsMilliOhm   = 65;
    ckParams.motorLsMicroH     = 30;
    ckParams.alignTimeMs       = 150;
    ckParams.alignDutyPctX10   = 25;   /* 2.5% → LOOPTIME_TCY/40 */
    ckParams.initialStepPeriod = 800;
    ckParams.minStepPeriod     = 50;
    ckParams.rampAccelErpmS    = 1500;
    ckParams.rampDutyPctX10    = 167;  /* ~16.7% → LOOPTIME_TCY/6 */
    ckParams.rampTargetErpm    = 4000;
    ckParams.clIdleDutyPct     = 10;
    ckParams.timingAdvMinDeg   = 0;
    ckParams.timingAdvMaxDeg   = 20;
    ckParams.timingAdvStartErpm = 5000;
    ckParams.maxClosedLoopErpm = 100000UL;
    ckParams.dutySlewUp        = 25;
    ckParams.dutySlewDown      = 10;
    ckParams.ilimEnable        = 1;
    ckParams.ilimShutdownEnable = 0;
    ckParams.ilimDac           = 85;
    ckParams.ilimFilterTime    = 6;
    ckParams.scEnable          = 1;
    ckParams.scThreshold       = 7;
    ckParams.scFilterTime      = 7;
    ckParams.bemfEnable        = 1;
    ckParams.crossConductionTime = 7;
    ckParams.edgeBlankingTime  = 1;
    ckParams.csaGain           = 1;    /* 16x */
    ckParams.zcDeglitchMin     = 3;
    ckParams.zcDeglitchMax     = 8;
    ckParams.zcTimeoutMult     = 2;
    ckParams.zcDesyncThresh    = 3;
    ckParams.zcMissLimit       = 12;
    ckParams.vbusOvThreshold   = 18176U;  /* 1136*16 ~15V */
    ckParams.vbusUvThreshold   = 9696U;   /* 606*16  ~8V */
    ckParams.desyncRestartMax  = 3;
    ckParams.recoveryTimeMs    = 200;
    ckParams.activeProfile     = CK_PROFILE_A2212;
}

static void Load2810Defaults(void)
{
    ckParams.polePairs         = 7;
    ckParams.motorKv           = 1350;
    ckParams.motorRsMilliOhm   = 50;
    ckParams.motorLsMicroH     = 25;
    ckParams.alignTimeMs       = 200;
    ckParams.alignDutyPctX10   = 25;   /* 2.5% → LOOPTIME_TCY/40 */
    ckParams.initialStepPeriod = 1000;
    ckParams.minStepPeriod     = 50;
    ckParams.rampAccelErpmS    = 500;
    ckParams.rampDutyPctX10    = 67;   /* ~6.7% → LOOPTIME_TCY/15 */
    ckParams.rampTargetErpm    = 3000;
    ckParams.clIdleDutyPct     = 10;
    ckParams.timingAdvMinDeg   = 0;
    ckParams.timingAdvMaxDeg   = 20;
    ckParams.timingAdvStartErpm = 5000;
    ckParams.maxClosedLoopErpm = 150000UL;
    ckParams.dutySlewUp        = 25;
    ckParams.dutySlewDown      = 10;
    ckParams.ilimEnable        = 1;
    ckParams.ilimShutdownEnable = 0;
    ckParams.ilimDac           = 95;
    ckParams.ilimFilterTime    = 6;
    ckParams.scEnable          = 1;
    ckParams.scThreshold       = 7;
    ckParams.scFilterTime      = 7;
    ckParams.bemfEnable        = 1;
    ckParams.crossConductionTime = 7;
    ckParams.edgeBlankingTime  = 1;
    ckParams.csaGain           = 1;    /* 16x */
    ckParams.zcDeglitchMin     = 3;
    ckParams.zcDeglitchMax     = 8;
    ckParams.zcTimeoutMult     = 2;
    ckParams.zcDesyncThresh    = 3;
    ckParams.zcMissLimit       = 12;
    ckParams.vbusOvThreshold   = 33908U;  /* ~28V */
    ckParams.vbusUvThreshold   = 14532U;  /* ~12V */
    ckParams.desyncRestartMax  = 3;
    ckParams.recoveryTimeMs    = 200;
    ckParams.activeProfile     = CK_PROFILE_2810;
}

/* ── Descriptor table ── */

static const CK_PARAM_DESC_T descriptorTable[CK_PARAM_COUNT] = {
    /* Group 0: Motor Identity */
    { CK_PARAM_POLE_PAIRS,          CK_TYPE_U8,  CK_GROUP_MOTOR,    1,     12 },
    { CK_PARAM_MOTOR_KV,            CK_TYPE_U16, CK_GROUP_MOTOR,    10,    5000 },
    { CK_PARAM_MOTOR_RS,            CK_TYPE_U16, CK_GROUP_MOTOR,    1,     10000 },
    { CK_PARAM_MOTOR_LS,            CK_TYPE_U16, CK_GROUP_MOTOR,    1,     5000 },

    /* Group 1: Startup & Ramp */
    { CK_PARAM_ALIGN_TIME_MS,       CK_TYPE_U16, CK_GROUP_STARTUP,  50,    2000 },
    { CK_PARAM_ALIGN_DUTY_PCT,      CK_TYPE_U16, CK_GROUP_STARTUP,  10,    200 },   /* x10: 1.0%-20.0% */
    { CK_PARAM_INITIAL_STEP_PERIOD, CK_TYPE_U16, CK_GROUP_STARTUP,  100,   5000 },
    { CK_PARAM_MIN_STEP_PERIOD,     CK_TYPE_U16, CK_GROUP_STARTUP,  2,     500 },
    { CK_PARAM_RAMP_ACCEL,          CK_TYPE_U16, CK_GROUP_STARTUP,  100,   10000 },
    { CK_PARAM_RAMP_DUTY_PCT,       CK_TYPE_U16, CK_GROUP_STARTUP,  20,    500 },   /* x10: 2.0%-50.0% */
    { CK_PARAM_RAMP_TARGET_ERPM,    CK_TYPE_U16, CK_GROUP_STARTUP,  1000,  20000 },

    /* Group 2: Closed-Loop */
    { CK_PARAM_CL_IDLE_DUTY_PCT,    CK_TYPE_U8,  CK_GROUP_CL,       1,    30 },
    { CK_PARAM_TIM_ADV_MIN_DEG,     CK_TYPE_U8,  CK_GROUP_CL,       0,    25 },
    { CK_PARAM_TIM_ADV_MAX_DEG,     CK_TYPE_U8,  CK_GROUP_CL,       0,    30 },
    { CK_PARAM_TIM_ADV_START_ERPM,  CK_TYPE_U16, CK_GROUP_CL,       500,  50000 },
    { CK_PARAM_MAX_CL_ERPM,         CK_TYPE_U32, CK_GROUP_CL,       5000, 200000 },
    { CK_PARAM_DUTY_SLEW_UP,        CK_TYPE_U8,  CK_GROUP_CL,       1,    100 },
    { CK_PARAM_DUTY_SLEW_DOWN,      CK_TYPE_U8,  CK_GROUP_CL,       1,    100 },

    /* Group 3: ATA6847 Protection */
    { CK_PARAM_ILIM_ENABLE,         CK_TYPE_U8,  CK_GROUP_ATA_PROT, 0,    1 },
    { CK_PARAM_ILIM_SHUTDOWN,       CK_TYPE_U8,  CK_GROUP_ATA_PROT, 0,    1 },
    { CK_PARAM_ILIM_DAC,            CK_TYPE_U8,  CK_GROUP_ATA_PROT, 0,    127 },
    { CK_PARAM_ILIM_FILTER,         CK_TYPE_U8,  CK_GROUP_ATA_PROT, 0,    7 },
    { CK_PARAM_SC_ENABLE,           CK_TYPE_U8,  CK_GROUP_ATA_PROT, 0,    1 },
    { CK_PARAM_SC_THRESHOLD,        CK_TYPE_U8,  CK_GROUP_ATA_PROT, 0,    7 },
    { CK_PARAM_SC_FILTER,           CK_TYPE_U8,  CK_GROUP_ATA_PROT, 0,    7 },

    /* Group 4: ATA6847 GDU */
    { CK_PARAM_BEMF_ENABLE,         CK_TYPE_U8,  CK_GROUP_ATA_GDU,  0,    1 },
    { CK_PARAM_CROSS_COND_TIME,     CK_TYPE_U8,  CK_GROUP_ATA_GDU,  0,    7 },
    { CK_PARAM_EDGE_BLANKING,       CK_TYPE_U8,  CK_GROUP_ATA_GDU,  0,    3 },
    { CK_PARAM_CSA_GAIN,            CK_TYPE_U8,  CK_GROUP_ATA_GDU,  0,    3 },

    /* Group 5: ZC Detection */
    { CK_PARAM_ZC_DEGLITCH_MIN,     CK_TYPE_U8,  CK_GROUP_ZC,       1,    15 },
    { CK_PARAM_ZC_DEGLITCH_MAX,     CK_TYPE_U8,  CK_GROUP_ZC,       1,    30 },
    { CK_PARAM_ZC_TIMEOUT_MULT,     CK_TYPE_U8,  CK_GROUP_ZC,       1,    5 },
    { CK_PARAM_ZC_DESYNC_THRESH,    CK_TYPE_U8,  CK_GROUP_ZC,       1,    20 },
    { CK_PARAM_ZC_MISS_LIMIT,       CK_TYPE_U8,  CK_GROUP_ZC,       3,    50 },

    /* Group 6: Voltage Protection */
    { CK_PARAM_VBUS_OV,             CK_TYPE_U16, CK_GROUP_VOLTAGE,  1000,  65000 },
    { CK_PARAM_VBUS_UV,             CK_TYPE_U16, CK_GROUP_VOLTAGE,  1000,  65000 },

    /* Group 7: Recovery */
    { CK_PARAM_DESYNC_RESTART_MAX,  CK_TYPE_U8,  CK_GROUP_RECOVERY, 0,    10 },
    { CK_PARAM_RECOVERY_TIME_MS,    CK_TYPE_U16, CK_GROUP_RECOVERY, 50,   5000 },
};

/* ── Public API ── */

void CK_ParamsLoadProfile(uint8_t profileId)
{
    switch (profileId) {
        case CK_PROFILE_HURST: LoadHurstDefaults(); break;
        case CK_PROFILE_A2212: LoadA2212Defaults(); break;
        case CK_PROFILE_2810:  Load2810Defaults();  break;
        default:               LoadA2212Defaults();  break;
    }
    CK_RecomputeDerived();
}

void CK_ParamsInitDefaults(void)
{
    CK_ParamsLoadProfile(MOTOR_PROFILE);
}

const CK_PARAM_DESC_T* CK_GetDescriptorTable(uint8_t *count)
{
    *count = CK_PARAM_COUNT;
    return descriptorTable;
}

uint32_t CK_ParamGet(uint16_t paramId, bool *ok)
{
    *ok = true;
    switch (paramId) {
        /* Group 0 */
        case CK_PARAM_POLE_PAIRS:          return ckParams.polePairs;
        case CK_PARAM_MOTOR_KV:            return ckParams.motorKv;
        case CK_PARAM_MOTOR_RS:            return ckParams.motorRsMilliOhm;
        case CK_PARAM_MOTOR_LS:            return ckParams.motorLsMicroH;
        /* Group 1 */
        case CK_PARAM_ALIGN_TIME_MS:       return ckParams.alignTimeMs;
        case CK_PARAM_ALIGN_DUTY_PCT:      return ckParams.alignDutyPctX10;
        case CK_PARAM_INITIAL_STEP_PERIOD: return ckParams.initialStepPeriod;
        case CK_PARAM_MIN_STEP_PERIOD:     return ckParams.minStepPeriod;
        case CK_PARAM_RAMP_ACCEL:          return ckParams.rampAccelErpmS;
        case CK_PARAM_RAMP_DUTY_PCT:       return ckParams.rampDutyPctX10;
        case CK_PARAM_RAMP_TARGET_ERPM:    return ckParams.rampTargetErpm;
        /* Group 2 */
        case CK_PARAM_CL_IDLE_DUTY_PCT:    return ckParams.clIdleDutyPct;
        case CK_PARAM_TIM_ADV_MIN_DEG:     return ckParams.timingAdvMinDeg;
        case CK_PARAM_TIM_ADV_MAX_DEG:     return ckParams.timingAdvMaxDeg;
        case CK_PARAM_TIM_ADV_START_ERPM:  return ckParams.timingAdvStartErpm;
        case CK_PARAM_MAX_CL_ERPM:         return ckParams.maxClosedLoopErpm;
        case CK_PARAM_DUTY_SLEW_UP:        return ckParams.dutySlewUp;
        case CK_PARAM_DUTY_SLEW_DOWN:      return ckParams.dutySlewDown;
        /* Group 3 */
        case CK_PARAM_ILIM_ENABLE:         return ckParams.ilimEnable;
        case CK_PARAM_ILIM_SHUTDOWN:       return ckParams.ilimShutdownEnable;
        case CK_PARAM_ILIM_DAC:            return ckParams.ilimDac;
        case CK_PARAM_ILIM_FILTER:         return ckParams.ilimFilterTime;
        case CK_PARAM_SC_ENABLE:           return ckParams.scEnable;
        case CK_PARAM_SC_THRESHOLD:        return ckParams.scThreshold;
        case CK_PARAM_SC_FILTER:           return ckParams.scFilterTime;
        /* Group 4 */
        case CK_PARAM_BEMF_ENABLE:         return ckParams.bemfEnable;
        case CK_PARAM_CROSS_COND_TIME:     return ckParams.crossConductionTime;
        case CK_PARAM_EDGE_BLANKING:       return ckParams.edgeBlankingTime;
        case CK_PARAM_CSA_GAIN:            return ckParams.csaGain;
        /* Group 5 */
        case CK_PARAM_ZC_DEGLITCH_MIN:     return ckParams.zcDeglitchMin;
        case CK_PARAM_ZC_DEGLITCH_MAX:     return ckParams.zcDeglitchMax;
        case CK_PARAM_ZC_TIMEOUT_MULT:     return ckParams.zcTimeoutMult;
        case CK_PARAM_ZC_DESYNC_THRESH:    return ckParams.zcDesyncThresh;
        case CK_PARAM_ZC_MISS_LIMIT:       return ckParams.zcMissLimit;
        /* Group 6 */
        case CK_PARAM_VBUS_OV:             return ckParams.vbusOvThreshold;
        case CK_PARAM_VBUS_UV:             return ckParams.vbusUvThreshold;
        /* Group 7 */
        case CK_PARAM_DESYNC_RESTART_MAX:  return ckParams.desyncRestartMax;
        case CK_PARAM_RECOVERY_TIME_MS:    return ckParams.recoveryTimeMs;

        default:
            *ok = false;
            return 0;
    }
}

bool CK_ParamSet(uint16_t paramId, uint32_t value)
{
    /* Find descriptor for bounds checking */
    uint8_t i;
    const CK_PARAM_DESC_T *desc = 0;
    for (i = 0; i < CK_PARAM_COUNT; i++) {
        if (descriptorTable[i].id == paramId) {
            desc = &descriptorTable[i];
            break;
        }
    }
    if (!desc)
        return false;  /* Unknown param */

    if (value < desc->min || value > desc->max)
        return false;  /* Out of range */

    switch (paramId) {
        /* Group 0 */
        case CK_PARAM_POLE_PAIRS:          ckParams.polePairs = (uint8_t)value; break;
        case CK_PARAM_MOTOR_KV:            ckParams.motorKv = (uint16_t)value; break;
        case CK_PARAM_MOTOR_RS:            ckParams.motorRsMilliOhm = (uint16_t)value; break;
        case CK_PARAM_MOTOR_LS:            ckParams.motorLsMicroH = (uint16_t)value; break;
        /* Group 1 */
        case CK_PARAM_ALIGN_TIME_MS:       ckParams.alignTimeMs = (uint16_t)value; break;
        case CK_PARAM_ALIGN_DUTY_PCT:      ckParams.alignDutyPctX10 = (uint16_t)value; break;
        case CK_PARAM_INITIAL_STEP_PERIOD: ckParams.initialStepPeriod = (uint16_t)value; break;
        case CK_PARAM_MIN_STEP_PERIOD:     ckParams.minStepPeriod = (uint16_t)value; break;
        case CK_PARAM_RAMP_ACCEL:          ckParams.rampAccelErpmS = (uint16_t)value; break;
        case CK_PARAM_RAMP_DUTY_PCT:       ckParams.rampDutyPctX10 = (uint16_t)value; break;
        case CK_PARAM_RAMP_TARGET_ERPM:    ckParams.rampTargetErpm = (uint16_t)value; break;
        /* Group 2 */
        case CK_PARAM_CL_IDLE_DUTY_PCT:    ckParams.clIdleDutyPct = (uint8_t)value; break;
        case CK_PARAM_TIM_ADV_MIN_DEG:     ckParams.timingAdvMinDeg = (uint8_t)value; break;
        case CK_PARAM_TIM_ADV_MAX_DEG:     ckParams.timingAdvMaxDeg = (uint8_t)value; break;
        case CK_PARAM_TIM_ADV_START_ERPM:  ckParams.timingAdvStartErpm = (uint16_t)value; break;
        case CK_PARAM_MAX_CL_ERPM:         ckParams.maxClosedLoopErpm = value; break;
        case CK_PARAM_DUTY_SLEW_UP:        ckParams.dutySlewUp = (uint8_t)value; break;
        case CK_PARAM_DUTY_SLEW_DOWN:      ckParams.dutySlewDown = (uint8_t)value; break;
        /* Group 3 */
        case CK_PARAM_ILIM_ENABLE:         ckParams.ilimEnable = (uint8_t)value; break;
        case CK_PARAM_ILIM_SHUTDOWN:       ckParams.ilimShutdownEnable = (uint8_t)value; break;
        case CK_PARAM_ILIM_DAC:            ckParams.ilimDac = (uint8_t)value; break;
        case CK_PARAM_ILIM_FILTER:         ckParams.ilimFilterTime = (uint8_t)value; break;
        case CK_PARAM_SC_ENABLE:           ckParams.scEnable = (uint8_t)value; break;
        case CK_PARAM_SC_THRESHOLD:        ckParams.scThreshold = (uint8_t)value; break;
        case CK_PARAM_SC_FILTER:           ckParams.scFilterTime = (uint8_t)value; break;
        /* Group 4 */
        case CK_PARAM_BEMF_ENABLE:         ckParams.bemfEnable = (uint8_t)value; break;
        case CK_PARAM_CROSS_COND_TIME:     ckParams.crossConductionTime = (uint8_t)value; break;
        case CK_PARAM_EDGE_BLANKING:       ckParams.edgeBlankingTime = (uint8_t)value; break;
        case CK_PARAM_CSA_GAIN:            ckParams.csaGain = (uint8_t)value; break;
        /* Group 5 */
        case CK_PARAM_ZC_DEGLITCH_MIN:     ckParams.zcDeglitchMin = (uint8_t)value; break;
        case CK_PARAM_ZC_DEGLITCH_MAX:     ckParams.zcDeglitchMax = (uint8_t)value; break;
        case CK_PARAM_ZC_TIMEOUT_MULT:     ckParams.zcTimeoutMult = (uint8_t)value; break;
        case CK_PARAM_ZC_DESYNC_THRESH:    ckParams.zcDesyncThresh = (uint8_t)value; break;
        case CK_PARAM_ZC_MISS_LIMIT:       ckParams.zcMissLimit = (uint8_t)value; break;
        /* Group 6 */
        case CK_PARAM_VBUS_OV:             ckParams.vbusOvThreshold = (uint16_t)value; break;
        case CK_PARAM_VBUS_UV:             ckParams.vbusUvThreshold = (uint16_t)value; break;
        /* Group 7 */
        case CK_PARAM_DESYNC_RESTART_MAX:  ckParams.desyncRestartMax = (uint8_t)value; break;
        case CK_PARAM_RECOVERY_TIME_MS:    ckParams.recoveryTimeMs = (uint16_t)value; break;

        default: return false;
    }
    return true;
}

void CK_RecomputeDerived(void)
{
    /* ATA6847 register bytes */
    ckDerived.ataIlimcrVal = (uint8_t)(
        (ckParams.ilimEnable << 7) |
        (ckParams.ilimFilterTime << 3) |
        (ckParams.ilimShutdownEnable << 2));

    ckDerived.ataScpcrVal = (uint8_t)(
        (ckParams.scEnable << 7) |
        (ckParams.scFilterTime << 3) |
        ckParams.scThreshold);

    ckDerived.ataGducr1Val = (uint8_t)(
        (ckParams.crossConductionTime << 2) |
        (ckParams.bemfEnable << 1) |
        1);  /* bit 0 constant */

    ckDerived.ataGducr3Val = (uint8_t)(ckParams.edgeBlankingTime << 2);

    ckDerived.ataCscrVal = (uint8_t)((ckParams.csaGain << 2) | 0x01);

    /* PWM-derived values */
    ckDerived.alignDuty = (uint16_t)(
        (uint32_t)ckParams.alignDutyPctX10 * LOOPTIME_TCY / 1000);
    if (ckDerived.alignDuty < MIN_DUTY)
        ckDerived.alignDuty = MIN_DUTY;

    ckDerived.alignTimeCounts = (uint16_t)(
        (uint32_t)ckParams.alignTimeMs * TIMER1_FREQ_HZ / 1000);

    ckDerived.clIdleDuty = (uint16_t)(
        (uint32_t)ckParams.clIdleDutyPct * LOOPTIME_TCY / 100);
    if (ckDerived.clIdleDuty < MIN_DUTY)
        ckDerived.clIdleDuty = MIN_DUTY;

    ckDerived.rampDutyCap = (uint16_t)(
        (uint32_t)ckParams.rampDutyPctX10 * LOOPTIME_TCY / 1000);

    ckDerived.recoveryCounts = (uint16_t)(
        (uint32_t)ckParams.recoveryTimeMs * TIMER1_FREQ_HZ / 1000);
}

bool CK_ParamsCrossValidate(void)
{
    if (ckParams.vbusOvThreshold <= ckParams.vbusUvThreshold)
        return false;
    if (ckParams.timingAdvMaxDeg < ckParams.timingAdvMinDeg)
        return false;
    if (ckParams.zcDeglitchMax < ckParams.zcDeglitchMin)
        return false;
    return true;
}
