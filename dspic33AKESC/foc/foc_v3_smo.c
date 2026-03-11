/**
 * @file foc_v3_smo.c
 * @brief Sliding Mode Observer implementation — correct classical SMO.
 *
 * Reference: Microchip AN1078 (dsPIC33CK SMO), adapted to float
 * for dsPIC33AK hardware FPU.
 *
 * Key differences from AN1078 Q15 implementation:
 *   - All float (no Q15 normalization headaches)
 *   - Sigmoid switching instead of linear+saturate
 *   - Speed-adaptive LPF bandwidth (AN1078 uses speed-proportional Kslf)
 *   - PLL for speed instead of delta-theta accumulation
 *   - Explicit health metrics: residual, confidence, observable flag
 *   - Phase delay model: v3_smo_phase_delay() for commutation comp
 *
 * Component: FOC V3
 */

#include "foc_v3_smo.h"
#include "foc_v2_math.h"      /* foc_clampf, FOC_TWO_PI, FOC_PI_F */
#include <math.h>

/* ── Sigmoid switching function ──────────────────────────────── */

/**
 * Sigmoid: F(x) = x / (|x| + φ)
 * Smooth approximation of sign(x) with boundary layer φ.
 * |F(x)| < 1 always, F(x) → sign(x) as |x| >> φ.
 */
static inline float sigmoid(float x, float phi)
{
    float ax = (x >= 0.0f) ? x : -x;
    return x / (ax + phi);
}

/* ── SMO Init / Reset ────────────────────────────────────────── */

void v3_smo_init(SMO_Observer_t *smo, const SMO_Config_t *cfg)
{
    /* Discrete plant model: forward Euler
     *   i[k+1] = (1 - Rs*dt/Ls)*i[k] + (dt/Ls)*(V - E - Z)
     * F must be < 1 for stability.  At 24kHz, A2212:
     *   F = 1 - 0.065*41.67e-6/30e-6 = 1 - 0.0903 = 0.910 (stable) */
    smo->F = 1.0f - cfg->Rs * cfg->dt / cfg->Ls;
    smo->G = cfg->dt / cfg->Ls;

    /* Rs adaptation: store initial Rs and plant params */
    smo->Rs_est    = cfg->Rs;
    smo->Ls        = cfg->Ls;
    smo->dt        = cfg->dt;
    smo->lambda_pm = cfg->lambda_pm;

    /* Sliding gain state */
    smo->k_base        = cfg->k_base;
    smo->k_bemf_scale  = cfg->k_bemf_scale;
    smo->k_max         = cfg->k_max;
    smo->k_adapt_alpha = cfg->k_adapt_alpha;
    smo->phi           = cfg->phi;

    /* LPF state */
    smo->alpha_base = cfg->alpha_base;
    smo->omega_ref  = cfg->omega_ref;
    smo->alpha_min  = cfg->alpha_min;
    smo->alpha_max  = cfg->alpha_max;

    /* Health thresholds */
    smo->conf_min     = cfg->conf_min;
    smo->residual_max = cfg->residual_max;

    v3_smo_reset(smo);
}

void v3_smo_reset(SMO_Observer_t *smo)
{
    smo->i_hat_alpha = 0.0f;
    smo->i_hat_beta  = 0.0f;
    smo->z_alpha     = 0.0f;
    smo->z_beta      = 0.0f;
    smo->e1_alpha    = 0.0f;
    smo->e1_beta     = 0.0f;
    smo->e2_alpha    = 0.0f;
    smo->e2_beta     = 0.0f;
    smo->theta_est   = 0.0f;
    smo->omega_est   = 0.0f;
    smo->theta_prev  = 0.0f;
    smo->omega_lpf   = 0.0f;
    smo->k_adapt     = smo->k_base;
    smo->alpha_now   = smo->alpha_base;
    smo->tau_lpf     = 0.0f;
    smo->err_alpha   = 0.0f;
    smo->err_beta    = 0.0f;
    smo->err_mag_filt = 0.0f;
    smo->bemf_mag_filt = 0.0f;
    smo->confidence  = 0.0f;
    smo->residual    = 0.0f;
    smo->observable  = false;
}

/* ── SMO Update (one tick at 24 kHz) ─────────────────────────── */

void v3_smo_update(SMO_Observer_t *smo,
                float v_alpha, float v_beta,
                float i_alpha, float i_beta,
                float omega_est)
{
    /* 0. Absolute speed (used by adaptive K and adaptive LPF) */
    float abs_omega = (omega_est >= 0.0f) ? omega_est : -omega_est;

    /* 1. Current estimation error */
    float err_alpha = smo->i_hat_alpha - i_alpha;
    float err_beta  = smo->i_hat_beta  - i_beta;
    smo->err_alpha = err_alpha;
    smo->err_beta  = err_beta;

    /* 2. Sliding mode switching function (sigmoid).
     *
     * Adaptive K: LP-filtered from error magnitude.
     * K_now = max(K_bemf, K_adapt), where K_adapt tracks:
     *   K_target = k_base + k_base × |err|
     * LP filter (k_adapt_alpha) prevents K oscillation. */
    float err_mag = sqrtf(err_alpha * err_alpha + err_beta * err_beta);
    smo->err_mag_filt += smo->k_adapt_alpha * (err_mag - smo->err_mag_filt);

    float K_target = smo->k_base * 0.5f + smo->k_base * smo->err_mag_filt;
    smo->k_adapt += smo->k_adapt_alpha * (K_target - smo->k_adapt);

    float K_bemf = smo->k_bemf_scale * smo->lambda_pm * abs_omega;
    float K_now = (K_bemf > smo->k_adapt) ? K_bemf : smo->k_adapt;
    if (K_now > smo->k_max) K_now = smo->k_max;

    smo->z_alpha = K_now * sigmoid(err_alpha, smo->phi);
    smo->z_beta  = K_now * sigmoid(err_beta,  smo->phi);

    /* 3. Current model update (forward Euler)
     *    Î[k+1] = F·Î[k] + G·(V - E_stage1 - Z) */
    smo->i_hat_alpha = smo->F * smo->i_hat_alpha
                     + smo->G * (v_alpha - smo->e1_alpha - smo->z_alpha);
    smo->i_hat_beta  = smo->F * smo->i_hat_beta
                     + smo->G * (v_beta  - smo->e1_beta  - smo->z_beta);

    /* 4. Back-EMF extraction via cascaded LPF on switching signal Z.
     *
     * Speed-adaptive cutoff: alpha = base × |omega/omega_ref|,
     * clamped [alpha_min, alpha_max]. */
    float alpha_scale = abs_omega / smo->omega_ref;
    if (alpha_scale < 0.1f) alpha_scale = 0.1f;
    float alpha = smo->alpha_base * alpha_scale;
    if (alpha < smo->alpha_min) alpha = smo->alpha_min;
    if (alpha > smo->alpha_max) alpha = smo->alpha_max;
    smo->alpha_now = alpha;

    /* Stage 1: Z → E1 (first LPF, fed back to current model) */
    smo->e1_alpha += alpha * (smo->z_alpha - smo->e1_alpha);
    smo->e1_beta  += alpha * (smo->z_beta  - smo->e1_beta);

    /* Stage 2: E1 → E2 (second LPF, used for angle) */
    smo->e2_alpha += alpha * (smo->e1_alpha - smo->e2_alpha);
    smo->e2_beta  += alpha * (smo->e1_beta  - smo->e2_beta);

    /* Store LPF cutoff for accurate arctan phase compensation.
     * ωc = α·fs / (1-α).  Phase delay per stage = arctan(ω/ωc). */
    smo->tau_lpf = alpha / ((1.0f - alpha) * smo->dt);  /* = ωc */

    /* 5. Angle extraction from filtered back-EMF.
     *    eα = -λ·ω·sin(θ),  eβ = +λ·ω·cos(θ)
     *    θ = atan2(-eα, eβ) */
    float theta_raw = atan2f(-smo->e2_alpha, smo->e2_beta);
    if (theta_raw < 0.0f)
        theta_raw += FOC_TWO_PI;
    smo->theta_est = theta_raw;

    /* 5b. Health metrics */
    {
        float bemf_mag = sqrtf(smo->e2_alpha * smo->e2_alpha
                             + smo->e2_beta  * smo->e2_beta);
        smo->bemf_mag_filt += 0.005f * (bemf_mag - smo->bemf_mag_filt);

        /* Residual: normalized current estimation error (0-1 scale).
         * Raw err_mag ≈ G×K (inherent SMO chattering). Normalizing by
         * G×K_now gives a convergence quality metric where ~0.5-0.7 is
         * normal (current model absorbs ~40% of switching signal). */
        float GK = smo->G * K_now;
        float res_norm = (GK > 0.1f) ? (err_mag / GK) : 0.0f;
        smo->residual += 0.005f * (res_norm - smo->residual);

        /* Confidence: observed / expected BEMF ratio */
        float bemf_expected = smo->lambda_pm * abs_omega;
        if (bemf_expected > 0.01f) {
            float conf = smo->bemf_mag_filt / bemf_expected;
            if (conf > 1.0f) conf = 1.0f;
            if (conf < 0.0f) conf = 0.0f;
            smo->confidence = conf;
        } else {
            smo->confidence = 0.0f;
        }

        /* Observable flag: confidence-based (residual is inherent SMO
         * switching chattering at G×K ≈ 16A/tick, not a convergence metric) */
        smo->observable = (smo->confidence >= smo->conf_min);
    }

    /* 6. Simple speed estimate from delta-theta (for telemetry).
     *    The PLL provides the smooth speed used for control. */
    float dtheta = smo->theta_est - smo->theta_prev;
    if (dtheta >  FOC_PI_F) dtheta -= FOC_TWO_PI;
    if (dtheta < -FOC_PI_F) dtheta += FOC_TWO_PI;
    smo->theta_prev = smo->theta_est;

    float omega_raw = dtheta / smo->dt;
    smo->omega_lpf += 0.05f * (omega_raw - smo->omega_lpf);
    smo->omega_est = smo->omega_lpf;
}

/* ── SMO Phase Delay ─────────────────────────────────────────── */

float v3_smo_phase_delay(const SMO_Observer_t *smo, float omega)
{
    /* Exact phase delay for 2 cascaded 1st-order IIR stages + transport delay.
     * φ = 2·arctan(ω/ωc) + ω·dt
     * tau_lpf field stores ωc (LPF cutoff in rad/s). */
    float wc = smo->tau_lpf;
    if (wc < 1.0f) wc = 1.0f;  /* safety */
    return 2.0f * atanf(omega / wc) + omega * smo->dt;
}

/* ── Rs Online Adaptation ────────────────────────────────────── */

void v3_smo_adapt_rs(SMO_Observer_t *smo,
                     float i_alpha, float i_beta,
                     float Rs_init)
{
    const float adapt_gain = 0.1f;

    float err_alpha = smo->i_hat_alpha - i_alpha;
    float err_beta  = smo->i_hat_beta  - i_beta;
    float err_proj  = err_alpha * i_alpha + err_beta * i_beta;

    float i_sq = i_alpha * i_alpha + i_beta * i_beta;
    if (i_sq < 0.25f) return;

    float dRs = adapt_gain * err_proj / i_sq * smo->dt;

    smo->Rs_est += dRs;

    float Rs_min = Rs_init * 0.5f;
    float Rs_max = Rs_init * 1.5f;
    if (smo->Rs_est < Rs_min) smo->Rs_est = Rs_min;
    if (smo->Rs_est > Rs_max) smo->Rs_est = Rs_max;

    smo->F = 1.0f - smo->Rs_est * smo->dt / smo->Ls;
}

/* ── PLL for smooth speed estimation ─────────────────────────── */

void v3_smo_pll_init(SMO_PLL_t *pll,
                     float bw_min_hz, float bw_max_hz,
                     float omega_bw_ref, float omega_max)
{
    pll->bw_min_hz    = bw_min_hz;
    pll->bw_max_hz    = bw_max_hz;
    pll->omega_bw_ref = omega_bw_ref;

    float bw_init = pll->bw_min_hz;
    pll->kp = 2.0f * FOC_TWO_PI * bw_init;
    pll->ki = pll->kp * pll->kp * 0.25f;
    pll->theta_est = 0.0f;
    pll->omega_est = 0.0f;
    pll->omega_max = omega_max;
    pll->innovation = 0.0f;
    pll->innovation_lpf = 0.0f;
}

void v3_smo_pll_reset(SMO_PLL_t *pll)
{
    pll->theta_est = 0.0f;
    pll->omega_est = 0.0f;
    pll->innovation = 0.0f;
    pll->innovation_lpf = 0.0f;
}

float v3_smo_pll_update(SMO_PLL_t *pll, float theta_meas, float dt)
{
    /* Speed-adaptive bandwidth */
    {
        float abs_w = pll->omega_est;
        if (abs_w < 0.0f) abs_w = -abs_w;
        float frac = abs_w / pll->omega_bw_ref;
        if (frac > 1.0f) frac = 1.0f;

        float bw = pll->bw_min_hz + frac * (pll->bw_max_hz - pll->bw_min_hz);
        float w_n = FOC_TWO_PI * bw;
        pll->kp = 2.0f * 1.2f * w_n;       /* 2·ζ·ωn (ζ=1.2) */
        pll->ki = w_n * w_n;                 /* ωn² */
    }

    /* Phase error */
    float delta = theta_meas - pll->theta_est;
    if (delta >  FOC_PI_F) delta -= FOC_TWO_PI;
    if (delta < -FOC_PI_F) delta += FOC_TWO_PI;

    /* Track innovation for health monitoring */
    float abs_delta = (delta >= 0.0f) ? delta : -delta;
    pll->innovation = abs_delta;
    pll->innovation_lpf += 0.01f * (abs_delta - pll->innovation_lpf);

    /* PI update */
    pll->omega_est += pll->ki * delta * dt;

    /* Clamp speed: unidirectional */
    if (pll->omega_est > pll->omega_max) pll->omega_est = pll->omega_max;
    if (pll->omega_est < 0.0f) pll->omega_est = 0.0f;

    /* Advance angle */
    pll->theta_est += (pll->omega_est + pll->kp * delta) * dt;
    pll->theta_est = foc_angle_wrap(pll->theta_est);

    return pll->omega_est;
}
