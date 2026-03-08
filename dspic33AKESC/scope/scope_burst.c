/**
 * @file scope_burst.c
 * @brief Triggered high-rate burst scope implementation.
 *
 * 128-sample ring buffer at 24kHz ISR rate with trigger detection.
 * Total RAM: 128 × 26 = 3328 bytes + ~20 bytes state = ~3.4KB.
 *
 * Component: SCOPE
 */

#include "garuda_config.h"

#if FEATURE_BURST_SCOPE

#include <string.h>
#include "scope_burst.h"

/* ── Ring buffer ───────────────────────────────────────────────── */

static SCOPE_SAMPLE_T s_buf[SCOPE_BUF_SIZE];
static uint8_t  s_head;            /* Next write index */
static uint8_t  s_count;           /* Valid samples in buffer */

/* ── State ─────────────────────────────────────────────────────── */

static SCOPE_State_t     s_state;
static SCOPE_ArmConfig_t s_cfg;
static uint8_t           s_trigIdx;     /* Buffer index where trigger fired */
static uint8_t           s_postCount;   /* Post-trigger samples written */
static uint8_t           s_postTarget;  /* Post-trigger samples needed */
static int16_t           s_prevChVal;   /* Previous channel value (threshold) */
static uint8_t           s_prevState;   /* Previous ESC state (state-change) */
static bool              s_firstSample; /* Skip first sample for edge detect */

/* ── Helpers ───────────────────────────────────────────────────── */

/**
 * Extract the value of the configured trigger channel from a sample.
 */
static int16_t GetChannelValue(const SCOPE_SAMPLE_T *s, SCOPE_Channel_t ch)
{
    switch (ch) {
        case SCOPE_CH_IA:    return s->ia;
        case SCOPE_CH_IB:    return s->ib;
        case SCOPE_CH_ID:    return s->id;
        case SCOPE_CH_IQ:    return s->iq;
        case SCOPE_CH_VD:    return s->vd;
        case SCOPE_CH_VQ:    return s->vq;
        case SCOPE_CH_THETA: return s->theta;
        case SCOPE_CH_OMEGA: return s->omega;
        case SCOPE_CH_MOD:   return s->mod_index;
        default:             return 0;
    }
}

/**
 * Check if trigger condition is met for the current sample.
 */
static bool CheckTrigger(const SCOPE_SAMPLE_T *sample)
{
    switch (s_cfg.trigMode) {
        case SCOPE_TRIG_MANUAL:
            return false;  /* Only via Scope_ForceTrigger() */

        case SCOPE_TRIG_FAULT:
            return (sample->flags & 0x02) != 0;  /* bit1 = fault */

        case SCOPE_TRIG_STATE_CHANGE: {
            uint8_t cur = sample->state;
            if (s_firstSample) {
                s_firstSample = false;
                s_prevState = cur;
                return false;  /* Seed — don't trigger */
            }
            bool changed = (cur != s_prevState);
            s_prevState = cur;
            return changed;
        }

        case SCOPE_TRIG_THRESHOLD: {
            int16_t val = GetChannelValue(sample, s_cfg.trigChannel);
            if (s_firstSample) {
                s_firstSample = false;
                s_prevChVal = val;
                return false;  /* Seed — don't trigger */
            }
            bool triggered = false;
            if (s_cfg.trigEdge == SCOPE_EDGE_RISING) {
                triggered = (s_prevChVal < s_cfg.threshold &&
                             val >= s_cfg.threshold);
            } else {
                triggered = (s_prevChVal > s_cfg.threshold &&
                             val <= s_cfg.threshold);
            }
            s_prevChVal = val;
            return triggered;
        }

        default:
            return false;
    }
}

/* ── Public API ────────────────────────────────────────────────── */

void Scope_Init(void)
{
    s_state = SCOPE_IDLE;
    s_head = 0;
    s_count = 0;
    s_trigIdx = 0;
    s_postCount = 0;
    s_postTarget = 0;
    s_prevChVal = 0;
    s_prevState = 0;
    s_firstSample = true;
}

void Scope_Arm(const SCOPE_ArmConfig_t *cfg)
{
    if (s_state != SCOPE_IDLE && s_state != SCOPE_READY)
        return;

    s_cfg = *cfg;

    /* Clamp pre-trigger percentage */
    if (s_cfg.preTrigPct > 100)
        s_cfg.preTrigPct = 50;

    /* Post-trigger samples = buffer × (100 - prePct) / 100 */
    s_postTarget = (uint8_t)((uint16_t)SCOPE_BUF_SIZE *
                             (100 - s_cfg.preTrigPct) / 100);
    if (s_postTarget == 0)
        s_postTarget = 1;

    /* Reset buffer */
    s_head = 0;
    s_count = 0;
    s_trigIdx = 0;
    s_postCount = 0;
    s_prevChVal = 0;
    s_prevState = 0;
    s_firstSample = true;

    s_state = SCOPE_ARMED;
}

void Scope_Disarm(void)
{
    s_state = SCOPE_IDLE;
}

void Scope_ForceTrigger(void)
{
    if (s_state == SCOPE_ARMED) {
        s_trigIdx = s_head;
        s_postCount = 0;
        s_state = SCOPE_FILLING;
    }
}

void Scope_WriteSample(const SCOPE_SAMPLE_T *sample)
{
    /* Fast exit for non-active states */
    if (s_state != SCOPE_ARMED && s_state != SCOPE_FILLING)
        return;

    /* Write sample to ring buffer */
    s_buf[s_head] = *sample;
    s_head = (s_head + 1) & SCOPE_BUF_MASK;
    if (s_count < SCOPE_BUF_SIZE)
        s_count++;

    if (s_state == SCOPE_ARMED) {
        /* Check trigger condition */
        if (CheckTrigger(sample)) {
            s_trigIdx = (s_head - 1) & SCOPE_BUF_MASK;
            s_postCount = 0;
            s_state = SCOPE_FILLING;
        }
    } else if (s_state == SCOPE_FILLING) {
        s_postCount++;
        if (s_postCount >= s_postTarget) {
            s_state = SCOPE_READY;  /* Buffer frozen */
        }
    }
}

SCOPE_Status_t Scope_GetStatus(void)
{
    SCOPE_Status_t st;
    st.state       = s_state;
    st.trigMode    = s_cfg.trigMode;
    st.preTrigPct  = s_cfg.preTrigPct;
    st.trigIdx     = s_trigIdx;
    st.sampleCount = s_count;
    st.sampleSize  = SCOPE_SAMPLE_SIZE;
    return st;
}

uint8_t Scope_ReadSamples(uint8_t offset, uint8_t count,
                          SCOPE_SAMPLE_T *out)
{
    if (s_state != SCOPE_READY || s_count == 0)
        return 0;

    if (count > SCOPE_MAX_CHUNK)
        count = SCOPE_MAX_CHUNK;

    if (offset >= s_count)
        return 0;

    if (offset + count > s_count)
        count = s_count - offset;

    /* Oldest sample index: head has wrapped past count samples */
    uint8_t oldest = (s_head - s_count) & SCOPE_BUF_MASK;

    for (uint8_t i = 0; i < count; i++) {
        uint8_t idx = (oldest + offset + i) & SCOPE_BUF_MASK;
        out[i] = s_buf[idx];
    }

    return count;
}

#endif /* FEATURE_BURST_SCOPE */
