/**
 * @file motor_params.c
 * @brief Sector-PI runtime tunable parameters — implementation.
 */

#include "motor_params.h"


#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <xc.h>     /* PG1TRIGA register access */

/* ── Globals ───────────────────────────────────────────────────── */
volatile ESC_PARAMS_T  escParams;
volatile ESC_DERIVED_T escDerived;

/* ── Descriptor table — drives GET_PARAM_LIST ──────────────────── */
/* Wire format matches V3 (12 bytes per entry: id, type, group, min, max).
 * Min/max values are inclusive bounds. */
static const PARAM_DESC_T descTable[PARAM_COUNT] = {
    /* PI loop tuning */
    { PARAM_PHASE_ADVANCE_X10, TYPE_U16, GROUP_PI_LOOP,    0, 300 },
    /* Kp/Ki shift floors at 1 — shift=0 collapses the gain to 1 (vs the
     * calibrated 1/4 and 1/16), which combined with the ±25% delta clamp
     * still allows large single-capture period jumps. Validated range
     * is roughly 2..6 for both; widen at your own risk. */
    { PARAM_PI_KP_SHIFT,       TYPE_U8,  GROUP_PI_LOOP,    1,   8 },
    { PARAM_PI_KI_SHIFT,       TYPE_U8,  GROUP_PI_LOOP,    1,   8 },

    /* Capture / blanking */
    { PARAM_BLANKING_PCT,      TYPE_U8,  GROUP_CAPTURE,   10,  60 },

    /* Limits */
    { PARAM_MIN_PERIOD_HR,     TYPE_U16, GROUP_LIMITS,     5, 500 },

    /* AK ADC trigger position — sweep target for cF lockout diagnostic.
     * Bit 15 = CAHALF, bits 0-14 = TRIGA value. Range covers both cycles. */
    { PARAM_TRIGA_POS,         TYPE_U16, GROUP_LIMITS,     0, 0xFFFF },
};

/* ── Implementation ────────────────────────────────────────────── */

void Params_InitDefaults(void)
{
    /* Defaults pulled from compile-time #defines — preserves existing
     * baseline behavior on first boot before any GUI tuning happens. */
    escParams.phaseAdvanceDegX10 = (uint16_t)(PHASE_ADVANCE_DEG * 10.0f + 0.5f);
    escParams.piKpShift          = PI_KP_SHIFT;
    escParams.piKiShift          = PI_KI_SHIFT;
    /* Bench tested 15% with FEATURE_IOC_BEMF=1 on 2026-05-21 — desync
     * within 1 second because at high speed (Tp ≈ 350 HR ticks) 15%
     * blanking shrinks to ~30 µs which is less than demag time under
     * load. Stay at 25%. */
    escParams.blankingPct        = 25;
    escParams.minPeriodHr        = MIN_PERIOD_HR;

    Params_RecomputeDerived();
}

void Params_RecomputeDerived(void)
{
    /* Phase advance → AVR-style 8.8 fixed-point factor used by setValue.
     *   factor = (advance° + 30°) * 256 / 60°
     *   setValue = (factor * timerPeriod) >> 8
     * Compute once here so the ISR hot path stays a single multiply+shift. */
    uint32_t advTimes10Plus300 = (uint32_t)escParams.phaseAdvanceDegX10 + 300UL;  /* (adv+30)*10 */
    /* (adv+30)/60 × 256 = (adv*10+300)*256 / 600 */
    escDerived.advancePlus30Fp8 = (uint16_t)((advTimes10Plus300 * 256UL + 300UL) / 600UL);
}

uint32_t Params_Get(uint16_t paramId, bool *ok)
{
    if (ok) *ok = true;
    switch (paramId)
    {
        case PARAM_PHASE_ADVANCE_X10: return escParams.phaseAdvanceDegX10;
        case PARAM_PI_KP_SHIFT:       return escParams.piKpShift;
        case PARAM_PI_KI_SHIFT:       return escParams.piKiShift;
        case PARAM_BLANKING_PCT:      return escParams.blankingPct;
        case PARAM_MIN_PERIOD_HR:     return escParams.minPeriodHr;
        case PARAM_TRIGA_POS: {
            uint32_t reg = PG1TRIGA;
            uint16_t pos = (uint16_t)(reg & 0x7FFFu);
            if (reg & (1UL << 31)) pos |= 0x8000u;
            return pos;
        }
        default:
            if (ok) *ok = false;
            return 0;
    }
}

/* Helper: walk descriptor table, return entry by ID or NULL. */
static const PARAM_DESC_T* findDesc(uint16_t paramId)
{
    uint8_t i;
    for (i = 0; i < PARAM_COUNT; i++) {
        if (descTable[i].id == paramId) return &descTable[i];
    }
    return NULL;
}

bool Params_Set(uint16_t paramId, uint32_t value)
{
    const PARAM_DESC_T *d = findDesc(paramId);
    if (!d) return false;
    if (value < d->min || value > d->max) return false;

    switch (paramId)
    {
        case PARAM_PHASE_ADVANCE_X10: escParams.phaseAdvanceDegX10 = (uint16_t)value; break;
        case PARAM_PI_KP_SHIFT:       escParams.piKpShift          = (uint8_t)value;  break;
        case PARAM_PI_KI_SHIFT:       escParams.piKiShift          = (uint8_t)value;  break;
        case PARAM_BLANKING_PCT:      escParams.blankingPct        = (uint8_t)value;  break;
        case PARAM_MIN_PERIOD_HR:     escParams.minPeriodHr        = (uint16_t)value; break;
        case PARAM_TRIGA_POS: {
            uint32_t v = (uint32_t)((uint16_t)value & 0x7FFFu);
            if ((uint16_t)value & 0x8000u) v |= (1UL << 31);
            PG1TRIGA = v;
            break;
        }
        default: return false;
    }

    Params_RecomputeDerived();
    return true;
}

const PARAM_DESC_T* Params_GetDescriptorTable(uint8_t *count)
{
    if (count) *count = PARAM_COUNT;
    return descTable;
}

