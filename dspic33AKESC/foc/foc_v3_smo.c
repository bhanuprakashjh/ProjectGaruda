/**
 * @file foc_v3_smo.c
 * @brief Sliding Mode Observer implementation.
 *
 * Reference: Microchip AN1078 (dsPIC33CK SMO), adapted to float
 * for dsPIC33AK hardware FPU.
 *
 * Key differences from AN1078 Q15 implementation:
 *   - All float (no Q15 normalization headaches)
 *   - Sigmoid switching instead of linear+saturate
 *   - Speed-adaptive LPF bandwidth (AN1078 uses speed-proportional Kslf)
 *   - PLL for speed instead of delta-theta accumulation
 *
 * Component: FOC V3
 */

#include "foc_v3_smo.h"
#include "foc_v2_math.h"      /* foc_clampf, FOC_TWO_PI, FOC_PI_F */
#include <math.h>
#include "../garuda_foc_params.h"

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

void v3_smo_init(SMO_Observer_t *smo,
              float Rs, float Ls, float lambda_pm,
              float vbus_nom, float dt)
{
    /* Discrete plant model: forward Euler
     *   i[k+1] = (1 - Rs*dt/Ls)*i[k] + (dt/Ls)*(V - E - Z)
     * F must be < 1 for stability.  At 24kHz, A2212:
     *   F = 1 - 0.065*41.67e-6/30e-6 = 1 - 0.0903 = 0.910 (stable) */
    smo->F = 1.0f - Rs * dt / Ls;
    smo->G = dt / Ls;

    /* Speed-adaptive K: balance convergence vs chattering.
     * K_base = 1.5·Ls/dt — keeps G×K ≈ 1.5A (moderate chattering).
     * K_max = Vbus — ceiling for safety.
     * At runtime: K = max(K_base, 1.5·λ·ω) — scales with BEMF at speed.
     *
     * For A2212 (Ls=30µH): K_base = 1.5×30e-6/41.67e-6 = 1.08V
     * For Hurst  (Ls=359µH): K_base = 1.5×359e-6/41.67e-6 = 12.9V
     *
     * With K=1.08V: G×K = 1.5A. Sigmoid partially saturates.
     * Duty cycle properly encodes BEMF. LPF extracts clean angle.
     * (K=Vbus=12V gave G×K=16.7A → complete saturation → no BEMF info) */
    smo->K_base = 1.5f * Ls / dt;
    smo->K_max = vbus_nom;
    smo->lambda_pm = lambda_pm;

    /* Sigmoid boundary layer: controls chattering vs tracking.
     * 0.1A: sharp enough for good tracking, smooth enough to
     * suppress chattering at ADC noise floor (~0.04A). */
    smo->phi = SMO_SIGMOID_PHI;

    /* BEMF LPF alpha: per-profile tuning.
     * This is the base value — smo_update() scales it with speed
     * to maintain constant phase lag across the speed range. */
    smo->lpf_alpha = SMO_LPF_ALPHA;

    v3_smo_reset(smo);
}

void v3_smo_reset(SMO_Observer_t *smo)
{
    smo->i_hat_alpha = 0.0f;
    smo->i_hat_beta  = 0.0f;
    smo->z_alpha     = 0.0f;
    smo->z_beta      = 0.0f;
    smo->e_alpha     = 0.0f;
    smo->e_beta      = 0.0f;
    smo->e_alpha_filt = 0.0f;
    smo->e_beta_filt  = 0.0f;
    smo->theta_est   = 0.0f;
    smo->omega_est   = 0.0f;
    smo->theta_prev  = 0.0f;
    smo->omega_lpf   = 0.0f;
}

/* ── SMO Update (one tick at 24 kHz) ─────────────────────────── */

void v3_smo_update(SMO_Observer_t *smo,
                float v_alpha, float v_beta,
                float i_alpha, float i_beta,
                float omega_est, float dt)
{
    /* 0. Absolute speed (used by adaptive K and adaptive LPF) */
    float abs_omega = (omega_est >= 0.0f) ? omega_est : -omega_est;

    /* 1. Current estimation error */
    float err_alpha = smo->i_hat_alpha - i_alpha;
    float err_beta  = smo->i_hat_beta  - i_beta;

    /* 2. Sliding mode switching function (sigmoid).
     *    Speed-adaptive K: max(K_base, 1.5·λ·ω).
     *    K_base = 1.5·Ls/dt: controls current model chattering.
     *    At high speed, K scales with BEMF to maintain sliding condition.
     *    This keeps G×K proportional and sigmoid partially saturated
     *    so the switching duty cycle properly encodes BEMF. */
    float K_bemf = 1.5f * smo->lambda_pm * abs_omega;
    float K_now = (K_bemf > smo->K_base) ? K_bemf : smo->K_base;
    if (K_now > smo->K_max) K_now = smo->K_max;

    smo->z_alpha = K_now * sigmoid(err_alpha, smo->phi);
    smo->z_beta  = K_now * sigmoid(err_beta,  smo->phi);

    /* 3. Current model update (forward Euler)
     *    Î[k+1] = F·Î[k] + G·(V - E_stage1 - Z)
     *
     * E_stage1 (first LPF output) is fed back to the current model.
     * This is the AN1078 architecture: the observer sees the filtered
     * BEMF estimate as the "actual" BEMF for current prediction. */
    smo->i_hat_alpha = smo->F * smo->i_hat_alpha
                     + smo->G * (v_alpha - smo->e_alpha - smo->z_alpha);
    smo->i_hat_beta  = smo->F * smo->i_hat_beta
                     + smo->G * (v_beta  - smo->e_beta  - smo->z_beta);

    /* 4. Back-EMF extraction via cascaded LPF on switching signal Z.
     *
     * Speed-adaptive cutoff: scale alpha with |omega|.
     * At low speed, use minimum alpha (heavy filtering).
     * At high speed, increase alpha (less filtering, less lag).
     *
     * AN1078: Kslf = omega * THETA_FILTER_CNST, clamped to min.
     * We use: alpha = base_alpha * |omega/omega_ref|, clamped [0.02, 0.95].
     * omega_ref = handoff speed — at handoff, alpha = base_alpha. */

    /* Scale factor: alpha grows linearly with speed.
     * At handoff (1000 rad/s for A2212), alpha = SMO_LPF_ALPHA.
     * At 2x handoff, alpha = 2*SMO_LPF_ALPHA. Clamp to [0.02, 0.95]. */
    float omega_ref = STARTUP_HANDOFF_RAD_S;
    if (omega_ref < 100.0f) omega_ref = 100.0f;
    float alpha_scale = abs_omega / omega_ref;
    if (alpha_scale < 0.1f) alpha_scale = 0.1f;  /* minimum at very low speed */
    float alpha = smo->lpf_alpha * alpha_scale;
    alpha = foc_clampf(alpha, 0.02f, 0.95f);

    /* Stage 1: Z → E (first LPF) */
    smo->e_alpha += alpha * (smo->z_alpha - smo->e_alpha);
    smo->e_beta  += alpha * (smo->z_beta  - smo->e_beta);

    /* Stage 2: E → E_filt (second LPF, used for angle) */
    smo->e_alpha_filt += alpha * (smo->e_alpha - smo->e_alpha_filt);
    smo->e_beta_filt  += alpha * (smo->e_beta  - smo->e_beta_filt);

    /* 5. Angle extraction from filtered back-EMF.
     *
     * Motor back-EMF in αβ:
     *   eα = -λ·ω·sin(θ),  eβ = +λ·ω·cos(θ)
     *
     * So: θ = atan2(-eα, eβ) = atan2(λ·ω·sin(θ), λ·ω·cos(θ))
     *
     * The SMO switching signal Z converges to the actual BEMF,
     * so after filtering: θ = atan2(-E_α_filt, E_β_filt).
     *
     * Phase compensation: the cascaded 2nd-order LPF introduces
     * phase lag. AN1078 uses a constant 90° offset, but that only
     * works at one speed. We use the PLL to track the true angle
     * with implicit lag compensation. */
    float theta_raw = atan2f(-smo->e_alpha_filt, smo->e_beta_filt);
    if (theta_raw < 0.0f)
        theta_raw += FOC_TWO_PI;
    smo->theta_est = theta_raw;

    /* 6. Simple speed estimate from delta-theta (for telemetry).
     *    The PLL provides the smooth speed used for control. */
    float dtheta = smo->theta_est - smo->theta_prev;
    if (dtheta >  FOC_PI_F) dtheta -= FOC_TWO_PI;
    if (dtheta < -FOC_PI_F) dtheta += FOC_TWO_PI;
    smo->theta_prev = smo->theta_est;

    float omega_raw = dtheta / dt;
    /* LP filter the raw speed (EMA, alpha=0.05 → ~20Hz BW) */
    smo->omega_lpf += 0.05f * (omega_raw - smo->omega_lpf);
    smo->omega_est = smo->omega_lpf;
}

/* ── PLL for smooth speed estimation ─────────────────────────── */

void v3_smo_pll_init(SMO_PLL_t *pll, float bw_hz, float omega_max)
{
    /* Critically damped 2nd-order PLL:
     * Kp = 2 × 2π × BW
     * Ki = Kp² / 4 */
    pll->kp = 2.0f * FOC_TWO_PI * bw_hz;
    pll->ki = pll->kp * pll->kp * 0.25f;
    pll->theta_est = 0.0f;
    pll->omega_est = 0.0f;
    pll->omega_max = omega_max;
}

void v3_smo_pll_reset(SMO_PLL_t *pll)
{
    pll->theta_est = 0.0f;
    pll->omega_est = 0.0f;
}

float v3_smo_pll_update(SMO_PLL_t *pll, float theta_meas, float dt)
{
    /* Phase error */
    float delta = theta_meas - pll->theta_est;
    if (delta >  FOC_PI_F) delta -= FOC_TWO_PI;
    if (delta < -FOC_PI_F) delta += FOC_TWO_PI;

    /* PI update */
    pll->omega_est += pll->ki * delta * dt;

    /* Clamp speed */
    if (pll->omega_est >  pll->omega_max) pll->omega_est =  pll->omega_max;
    if (pll->omega_est < -pll->omega_max) pll->omega_est = -pll->omega_max;

    /* Advance angle */
    pll->theta_est += (pll->omega_est + pll->kp * delta) * dt;
    pll->theta_est = foc_angle_wrap(pll->theta_est);

    return pll->omega_est;
}
