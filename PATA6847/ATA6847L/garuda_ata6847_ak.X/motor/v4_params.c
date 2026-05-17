/**
 * @file v4_params.c
 * @brief V4 sector-PI runtime tunable parameters — implementation.
 */

#include "v4_params.h"


#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <xc.h>     /* PG1TRIGA register access */

/* ── Globals ───────────────────────────────────────────────────── */
volatile V4_PARAMS_T  v4Params;
volatile V4_DERIVED_T v4Derived;

/* ── Descriptor table — drives GET_PARAM_LIST ──────────────────── */
/* Wire format matches V3 (12 bytes per entry: id, type, group, min, max).
 * Min/max values are inclusive bounds. */
static const V4_PARAM_DESC_T descTable[V4_PARAM_COUNT] = {
    /* PI loop tuning */
    { V4_PARAM_PHASE_ADVANCE_X10, V4_TYPE_U16, V4_GROUP_PI_LOOP,    0, 300 },
    { V4_PARAM_PI_KP_SHIFT,       V4_TYPE_U8,  V4_GROUP_PI_LOOP,    0,   8 },
    { V4_PARAM_PI_KI_SHIFT,       V4_TYPE_U8,  V4_GROUP_PI_LOOP,    0,   8 },

    /* Capture / blanking */
    { V4_PARAM_BLANKING_PCT,      V4_TYPE_U8,  V4_GROUP_CAPTURE,   10,  60 },
    { V4_PARAM_PI_FEED_POLARITY,  V4_TYPE_U8,  V4_GROUP_CAPTURE,    0,   2 },

    /* Limits */
    { V4_PARAM_MIN_PERIOD_HR,     V4_TYPE_U16, V4_GROUP_LIMITS,     5, 500 },

    /* AK ADC trigger position — sweep target for cF lockout diagnostic.
     * Bit 15 = CAHALF, bits 0-14 = TRIGA value. Range covers both cycles. */
    { V4_PARAM_TRIGA_POS,         V4_TYPE_U16, V4_GROUP_LIMITS,     0, 0xFFFF },
};

/* ── Implementation ────────────────────────────────────────────── */

void V4Params_InitDefaults(void)
{
    /* Defaults pulled from compile-time #defines — preserves existing
     * baseline behavior on first boot before any GUI tuning happens. */
    v4Params.phaseAdvanceDegX10 = (uint16_t)(V4_PHASE_ADVANCE_DEG * 10.0f + 0.5f);
    v4Params.piKpShift          = V4_KP_SHIFT;
    v4Params.piKiShift          = V4_KI_SHIFT;
    v4Params.blankingPct        = 25;
    v4Params.piFeedPolarity     = 0;           /* 2026-05-14 diagnostic run: feed BOTH polarities.
                                                * With OWN=0 (legacy V4 PRE-ZC), this routes both
                                                * rising and falling captures through to the PI
                                                * via the `default: feedPi = true` branch in
                                                * garuda_service.c. The new diagPiFed{Rising,
                                                * Falling}/Miss{Rising,Falling} counters in the
                                                * snapshot will tell us per-polarity which sectors
                                                * actually reach the PI through the plausibility
                                                * filter. cR=44k/cF=3417 frozen at 1 proved Phase
                                                * 1 (piFeedPolarity=1) was structurally
                                                * rising-only-fed — 3 captures/cycle vs CK's 6. */
    v4Params.minPeriodHr        = V4_MIN_PERIOD;

    V4Params_RecomputeDerived();
}

void V4Params_RecomputeDerived(void)
{
    /* Phase advance → AVR-style 8.8 fixed-point factor used by setValue.
     *   factor = (advance° + 30°) * 256 / 60°
     *   setValue = (factor * timerPeriod) >> 8
     * Compute once here so the ISR hot path stays a single multiply+shift. */
    uint32_t advTimes10Plus300 = (uint32_t)v4Params.phaseAdvanceDegX10 + 300UL;  /* (adv+30)*10 */
    /* (adv+30)/60 × 256 = (adv*10+300)*256 / 600 */
    v4Derived.advancePlus30Fp8 = (uint16_t)((advTimes10Plus300 * 256UL + 300UL) / 600UL);
}

uint32_t V4Params_Get(uint16_t paramId, bool *ok)
{
    if (ok) *ok = true;
    switch (paramId)
    {
        case V4_PARAM_PHASE_ADVANCE_X10: return v4Params.phaseAdvanceDegX10;
        case V4_PARAM_PI_KP_SHIFT:       return v4Params.piKpShift;
        case V4_PARAM_PI_KI_SHIFT:       return v4Params.piKiShift;
        case V4_PARAM_BLANKING_PCT:      return v4Params.blankingPct;
        case V4_PARAM_PI_FEED_POLARITY:  return v4Params.piFeedPolarity;
        case V4_PARAM_MIN_PERIOD_HR:     return v4Params.minPeriodHr;
        case V4_PARAM_TRIGA_POS: {
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
static const V4_PARAM_DESC_T* findDesc(uint16_t paramId)
{
    uint8_t i;
    for (i = 0; i < V4_PARAM_COUNT; i++) {
        if (descTable[i].id == paramId) return &descTable[i];
    }
    return NULL;
}

bool V4Params_Set(uint16_t paramId, uint32_t value)
{
    const V4_PARAM_DESC_T *d = findDesc(paramId);
    if (!d) return false;
    if (value < d->min || value > d->max) return false;

    switch (paramId)
    {
        case V4_PARAM_PHASE_ADVANCE_X10: v4Params.phaseAdvanceDegX10 = (uint16_t)value; break;
        case V4_PARAM_PI_KP_SHIFT:       v4Params.piKpShift          = (uint8_t)value;  break;
        case V4_PARAM_PI_KI_SHIFT:       v4Params.piKiShift          = (uint8_t)value;  break;
        case V4_PARAM_BLANKING_PCT:      v4Params.blankingPct        = (uint8_t)value;  break;
        case V4_PARAM_PI_FEED_POLARITY:  v4Params.piFeedPolarity     = (uint8_t)value;  break;
        case V4_PARAM_MIN_PERIOD_HR:     v4Params.minPeriodHr        = (uint16_t)value; break;
        case V4_PARAM_TRIGA_POS: {
            uint32_t v = (uint32_t)((uint16_t)value & 0x7FFFu);
            if ((uint16_t)value & 0x8000u) v |= (1UL << 31);
            PG1TRIGA = v;
            break;
        }
        default: return false;
    }

    V4Params_RecomputeDerived();
    return true;
}

const V4_PARAM_DESC_T* V4Params_GetDescriptorTable(uint8_t *count)
{
    if (count) *count = V4_PARAM_COUNT;
    return descTable;
}

