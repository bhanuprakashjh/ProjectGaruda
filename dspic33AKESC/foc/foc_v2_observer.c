/**
 * @file foc_v2_observer.c
 * @brief MXLEMMING flux observer + PLL speed estimator.
 *
 * The MXLEMMING observer integrates back-EMF voltage to estimate rotor flux,
 * then extracts angle via atan2.  Key features:
 *   - L·dI cancellation (no Ls model error at high di/dt)
 *   - Adaptive flux linkage tracking (handles motor-to-motor variation)
 *   - Gain scheduling by duty cycle (low gain at low duty = less noise)
 *   - Dead-time compensation on input voltages
 *   - Upward flux correction (prevents zero-flux collapse at low speed)
 *
 * The PLL runs on the observer angle output for smooth speed estimation.
 * The observer angle is used directly for commutation (no PLL lag).
 *
 * Component: FOC V2
 */

#include "foc_v2_observer.h"
#include "foc_v2_math.h"

/* ── Observer ─────────────────────────────────────────────────── */

void foc_observer_reset(FOC_Observer_t *obs)
{
    obs->x1 = 0.0f;
    obs->x2 = 0.0f;
    obs->lambda_est = 0.0f;
    obs->i_alpha_last = 0.0f;
    obs->i_beta_last  = 0.0f;
    obs->theta_est = 0.0f;
    obs->gain = 0.0f;
}

void foc_observer_seed(FOC_Observer_t *obs, float lambda, float theta)
{
    obs->x1 = lambda * cosf(theta);
    obs->x2 = lambda * sinf(theta);
    obs->lambda_est = lambda;
}

void foc_observer_update(FOC_Observer_t *obs, FOC_PLL_t *pll,
                         float v_alpha, float v_beta,
                         float i_alpha, float i_beta,
                         float Rs, float Ls, float lambda,
                         float dt_comp_v, float duty_abs,
                         float dt)
{
    /* Dead-time compensated voltages */
    float v_a_eff = v_alpha + dt_comp_v * foc_soft_sign(i_alpha, 0.1f);
    float v_b_eff = v_beta  + dt_comp_v * foc_soft_sign(i_beta,  0.1f);

    /* Flux integration with L·dI cancellation.
     * dx = (V - Rs·I)·dt - Ls·(I[n] - I[n-1])
     * The Ls term removes the inductive voltage component directly
     * from current measurements, avoiding Ls model sensitivity. */
    obs->x1 += (v_a_eff - Rs * i_alpha) * dt - Ls * (i_alpha - obs->i_alpha_last);
    obs->x2 += (v_b_eff - Rs * i_beta)  * dt - Ls * (i_beta  - obs->i_beta_last);
    obs->i_alpha_last = i_alpha;
    obs->i_beta_last  = i_beta;

    /* Adaptive lambda tracking (VESC MXLEMMING_LAMBDA_COMP).
     * Adjusts flux estimate to match observed flux magnitude. */
    float mag_sq = obs->x1 * obs->x1 + obs->x2 * obs->x2;
    float err = obs->lambda_est * obs->lambda_est - mag_sq;
    obs->lambda_est += 0.1f * obs->gain * obs->lambda_est * (-err) * dt;
    obs->lambda_est = foc_clampf(obs->lambda_est, lambda * 0.3f, lambda * 2.5f);

    /* Circular normalization: constrain flux magnitude to lambda_est
     * while preserving angle.  Per-axis clamping (foc_clamp_abs) creates
     * a square boundary that distorts the angle when the per-tick
     * integration step is a large fraction of lambda — critical for
     * high-KV motors (A2212: step = 42% of lambda at 10k RPM).
     *
     * Also handles upward flux correction: if magnitude drops below
     * 50% of nominal lambda, gently push it up to prevent zero-flux
     * collapse at low speed. */
    {
        float mag_c = sqrtf(obs->x1 * obs->x1 + obs->x2 * obs->x2);
        if (mag_c > 1e-12f) {
            if (mag_c > obs->lambda_est) {
                /* Overshoot: normalize to lambda_est (preserves angle) */
                float sc = obs->lambda_est / mag_c;
                obs->x1 *= sc;
                obs->x2 *= sc;
            } else if (mag_c < lambda * 0.5f) {
                /* Collapse: gently boost ~10%/tick at 24kHz */
                float sc = 1.0f + 0.1f * dt * 24000.0f;
                obs->x1 *= sc;
                obs->x2 *= sc;
            }
        }
    }

    /* Angle extraction — direct atan2, no PLL needed for angle */
    obs->theta_est = atan2f(obs->x2, obs->x1);
    if (obs->theta_est < 0.0f)
        obs->theta_est += FOC_TWO_PI;

    /* Observer gain scheduling: scale with duty, floor at 5%.
     * Low duty = low applied voltage = noisy flux → reduce gain.
     * High duty = strong flux signal → aggressive correction. */
    float base_gain = 1e-3f / (lambda * lambda + 1e-12f);
    float duty_factor = (duty_abs > 0.05f) ? duty_abs : 0.05f;
    obs->gain = duty_factor * base_gain;

    /* PLL for speed only (smooth speed estimate for speed PI).
     * The observer atan2 angle has derivative noise — PLL filters it. */
    float delta = obs->theta_est - pll->theta_est;
    if (delta >  FOC_PI_F) delta -= FOC_TWO_PI;
    if (delta < -FOC_PI_F) delta += FOC_TWO_PI;

    pll->omega_est += pll->ki * delta * dt;
    pll->theta_est += (pll->omega_est + pll->kp * delta) * dt;
    pll->theta_est = foc_angle_wrap(pll->theta_est);
}

/* ── PLL ──────────────────────────────────────────────────────── */

void foc_pll_reset(FOC_PLL_t *pll)
{
    /* Reset state only — preserve Kp/Ki gains set by foc_pll_init().
     * reset_startup() calls this before each ALIGN cycle; zeroing gains
     * here killed the PLL (omega_est stuck at 0 → handoff never fired). */
    pll->theta_est = 0.0f;
    pll->omega_est = 0.0f;
}

void foc_pll_init(FOC_PLL_t *pll, float bw_hz)
{
    /* Critically damped 2nd-order PLL:
     * Kp = 2 * 2π * BW
     * Ki = Kp² / 4 */
    pll->kp = 2.0f * FOC_TWO_PI * bw_hz;
    pll->ki = pll->kp * pll->kp * 0.25f;
    pll->theta_est = 0.0f;
    pll->omega_est = 0.0f;
}
