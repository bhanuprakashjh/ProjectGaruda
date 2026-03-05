/**
 * @file foc_v2_math.h
 * @brief Inline math transforms for FOC v2 pipeline.
 *
 * All functions are static inline — zero call overhead, single compilation unit.
 * dsPIC33AK hardware FPU: sinf, cosf, sqrtf, atan2f are fast.
 *
 * Component: FOC V2
 */

#ifndef FOC_V2_MATH_H
#define FOC_V2_MATH_H

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FOC_TWO_PI   6.28318530718f
#define FOC_PI_F     3.14159265359f
#define FOC_SQRT3    1.73205080757f
#define FOC_INV_SQRT3 0.57735026919f  /* 1/sqrt(3) */

/* ── Utility ──────────────────────────────────────────────────── */

static inline float foc_clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline void foc_clamp_abs(float *x, float limit)
{
    if (*x > limit)  *x = limit;
    if (*x < -limit) *x = -limit;
}

static inline float foc_angle_wrap(float angle)
{
    if (angle >= FOC_TWO_PI) angle -= FOC_TWO_PI;
    if (angle < 0.0f)       angle += FOC_TWO_PI;
    return angle;
}

/**
 * Soft sign function: smooth transition around zero.
 * Returns x / (|x| + eps).  Avoids division by zero.
 */
static inline float foc_soft_sign(float x, float eps)
{
    float ax = (x >= 0.0f) ? x : -x;
    return x / (ax + eps);
}

/* ── Clarke transform (two-phase measurement) ────────────────── */
/* Ia + Ib + Ic = 0 → Ic reconstructed from Ia, Ib */
static inline void foc_clarke(float ia, float ib,
                              float *alpha, float *beta)
{
    *alpha = ia;
    *beta  = (ia + 2.0f * ib) * FOC_INV_SQRT3;
}

/* ── Park transform (αβ → dq) ────────────────────────────────── */
static inline void foc_park(float alpha, float beta,
                            float sin_t, float cos_t,
                            float *d, float *q)
{
    *d =  alpha * cos_t + beta * sin_t;
    *q = -alpha * sin_t + beta * cos_t;
}

/* ── Inverse Park transform (dq → αβ) ───────────────────────── */
static inline void foc_inv_park(float d, float q,
                                float sin_t, float cos_t,
                                float *alpha, float *beta)
{
    *alpha = d * cos_t - q * sin_t;
    *beta  = d * sin_t + q * cos_t;
}

/* ── Min-max (third harmonic injection) SVPWM ────────────────── */
/* Converts Vα, Vβ to three duty cycles [0..1].
 * Uses min-max injection — simpler than 6-sector, same Vdc utilization. */
static inline void foc_svpwm(float v_alpha, float v_beta, float vbus,
                              float *da, float *db, float *dc)
{
    if (vbus < 1.0f) vbus = 1.0f;  /* Safety clamp */
    float inv_vbus = 1.0f / vbus;

    /* αβ → abc (inverse Clarke) */
    float va = v_alpha * inv_vbus;
    float vb = (-0.5f * v_alpha + FOC_SQRT3 * 0.5f * v_beta) * inv_vbus;
    float vc = (-0.5f * v_alpha - FOC_SQRT3 * 0.5f * v_beta) * inv_vbus;

    /* Min-max injection (third harmonic equivalent) */
    float vmin = va;
    if (vb < vmin) vmin = vb;
    if (vc < vmin) vmin = vc;
    float vmax = va;
    if (vb > vmax) vmax = vb;
    if (vc > vmax) vmax = vc;
    float voff = -0.5f * (vmin + vmax);

    /* Shift to [0..1] range */
    *da = foc_clampf(va + voff + 0.5f, 0.0f, 1.0f);
    *db = foc_clampf(vb + voff + 0.5f, 0.0f, 1.0f);
    *dc = foc_clampf(vc + voff + 0.5f, 0.0f, 1.0f);
}

#ifdef __cplusplus
}
#endif

#endif /* FOC_V2_MATH_H */
