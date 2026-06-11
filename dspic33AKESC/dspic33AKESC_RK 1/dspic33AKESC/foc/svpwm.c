/**
 * @file svpwm.c
 * @brief Space-Vector PWM using min-max (mid-clamp) injection.
 *
 * Algorithm:
 *  1. Inverse Clarke: αβ → abc phase voltages.
 *  2. Compute neutral-shift offset = -(max + min) / 2.
 *  3. Add offset to each phase and normalise by Vbus → duty [0,1].
 *
 * This is mathematically equivalent to the 6-sector T1/T2 method but
 * requires no sector lookup, no switch/case, and no sign tables —
 * eliminating an entire class of mapping bugs.
 *
 * Component: FOC SVPWM
 */

#include "svpwm.h"

#define SQRT3_HALF  0.86602540378443864676f   /* √3 / 2 */

static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void svpwm_update(float v_alpha, float v_beta, float vbus,
                  float *duty_a, float *duty_b, float *duty_c)
{
    /* Protect against zero-bus edge case */
    if (vbus < 1.0f) vbus = 1.0f;

    /* ── 1. Inverse Clarke: αβ → abc phase voltages ──────────────────── */
    float va =  v_alpha;
    float vb = -0.5f * v_alpha + SQRT3_HALF * v_beta;
    float vc = -0.5f * v_alpha - SQRT3_HALF * v_beta;

    /* ── 2. Min-max neutral injection (SVPWM-equivalent) ─────────────── */
    float v_max = va;
    if (vb > v_max) v_max = vb;
    if (vc > v_max) v_max = vc;

    float v_min = va;
    if (vb < v_min) v_min = vb;
    if (vc < v_min) v_min = vc;

    float v_offset = -0.5f * (v_max + v_min);

    /* ── 3. Normalise to duty cycles [0, 1] ──────────────────────────── */
    float inv_vbus = 1.0f / vbus;

    *duty_a = clampf((va + v_offset) * inv_vbus + 0.5f, 0.0f, 1.0f);
    *duty_b = clampf((vb + v_offset) * inv_vbus + 0.5f, 0.0f, 1.0f);
    *duty_c = clampf((vc + v_offset) * inv_vbus + 0.5f, 0.0f, 1.0f);
}
