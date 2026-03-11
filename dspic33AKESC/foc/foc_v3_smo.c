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
     * Cap K_base at 25% Vbus to prevent BEMF drowning on high-Ls motors:
     *   Hurst (Ls=359µH): uncapped=12.9V ≈ Vbus → capped to 6.0V
     *   A2212 (Ls=30µH):  uncapped=1.08V → unchanged (< 3V cap)
     *
     * K too high → sigmoid always saturated → switching duty cycle
     * doesn't encode BEMF → LPF output is noise, not BEMF angle. */
    smo->K_base = 1.5f * Ls / dt;
    {
        float k_cap = vbus_nom * 0.25f;
        if (smo->K_base > k_cap) smo->K_base = k_cap;
    }
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

    /* Rs adaptation: store initial Rs and plant params for recomputation */
    smo->Rs_est = Rs;
    smo->Ls_val = Ls;
    smo->dt_val = dt;

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
    smo->confidence  = 0.0f;
    smo->bemf_mag_filt = 0.0f;
    smo->K_adapt     = smo->K_base;
    smo->err_mag_filt = 0.0f;
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
     *
     * Adaptive K (Phase 3): K tracks current estimation error magnitude.
     * When error is large (transient, startup): K rises → maintain sliding.
     * When error is small (converged): K falls → less chattering → cleaner BEMF.
     *
     * K_now = max(K_bemf, K_adapt), where K_adapt is LP-filtered from:
     *   K_target = K_base + K_err_scale × |err|
     * The LP filter (alpha=0.01) prevents K from oscillating with the
     * switching noise — K changes slowly over ~100 ticks (4ms).
     *
     * Fallback: K_bemf = 1.5·λ·ω ensures sliding condition at high speed
     * even if adaptive K hasn't caught up yet. */
    float err_mag = sqrtf(err_alpha * err_alpha + err_beta * err_beta);
    smo->err_mag_filt += 0.01f * (err_mag - smo->err_mag_filt);

    /* K_err_scale: how much K increases per amp of error.
     * At 1A error: adds K_base worth of gain (doubles K).
     * At 0.1A (converged): adds 10% of K_base (near minimum). */
    float K_err_scale = smo->K_base;
    float K_target = smo->K_base * 0.5f + K_err_scale * smo->err_mag_filt;
    smo->K_adapt += 0.01f * (K_target - smo->K_adapt);

    float K_bemf = 1.5f * smo->lambda_pm * abs_omega;
    float K_now = (K_bemf > smo->K_adapt) ? K_bemf : smo->K_adapt;
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
     * Note: AN1078 adds a constant +90° here, but our SMO already
     * LEADS the forced angle by ~70° (sigmoid + adaptive K + Ls
     * model error). The residual offset is captured at handoff and
     * subtracted during CL commutation. No constant shift needed. */
    float theta_raw = atan2f(-smo->e_alpha_filt, smo->e_beta_filt);
    if (theta_raw < 0.0f)
        theta_raw += FOC_TWO_PI;
    smo->theta_est = theta_raw;

    /* 5b. Confidence metric (Phase 3).
     *
     * Confidence = ratio of observed BEMF magnitude to expected BEMF.
     * Expected BEMF = λ_pm × |ω|.  Observed = |E_filt|.
     *
     * When SMO has converged, E_filt tracks the real BEMF, so the
     * ratio approaches 1.0.  During startup or desync, E_filt is
     * noise or has wrong magnitude, so confidence is low.
     *
     * LP-filter the BEMF magnitude (alpha=0.005 → ~20Hz BW) to
     * reject per-cycle ripple. Clamp confidence to [0, 1]. */
    {
        float bemf_mag = sqrtf(smo->e_alpha_filt * smo->e_alpha_filt
                             + smo->e_beta_filt  * smo->e_beta_filt);
        smo->bemf_mag_filt += 0.005f * (bemf_mag - smo->bemf_mag_filt);

        float bemf_expected = smo->lambda_pm * abs_omega;
        if (bemf_expected > 0.01f) {
            float conf = smo->bemf_mag_filt / bemf_expected;
            if (conf > 1.0f) conf = 1.0f;
            if (conf < 0.0f) conf = 0.0f;
            smo->confidence = conf;
        } else {
            smo->confidence = 0.0f;
        }
    }

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

/* ── Rs Online Adaptation ────────────────────────────────────── */

/**
 * Adapt Rs estimate based on current prediction error.
 *
 * The SMO current model: Î[k+1] = (1 - Rs*dt/Ls)·Î[k] + ...
 * If Rs_est < true Rs: model overestimates current → err = Î - I > 0
 * If Rs_est > true Rs: model underestimates current → err < 0
 *
 * Adaptation law: dRs = gain × (err · i_meas) — projects error onto
 * current direction to get sign-correct Rs update.
 *
 * Only call during closed-loop at moderate-to-high speed (good SNR).
 * Clamp Rs_est to ±50% of initial value to prevent divergence.
 *
 * @param smo       Observer state
 * @param i_alpha   Measured current α (A)
 * @param i_beta    Measured current β (A)
 * @param Rs_init   Initial Rs from motor profile (for clamping)
 */
void v3_smo_adapt_rs(SMO_Observer_t *smo,
                     float i_alpha, float i_beta,
                     float Rs_init)
{
    /* Adaptation gain: very slow (0.1 Ω/s per A² of error×current).
     * At 1A current with 0.05Ω Rs error: dRs/dt = 0.1 × 0.05 × 1 = 5mΩ/s.
     * Takes ~10s to correct — intentionally slow for stability. */
    const float adapt_gain = 0.1f;

    /* Project current error onto current direction:
     *   err_Rs = err_α·iα + err_β·iβ  (dot product)
     * Positive when model over-predicts → Rs too low → increase. */
    float err_alpha = smo->i_hat_alpha - i_alpha;
    float err_beta  = smo->i_hat_beta  - i_beta;
    float err_proj  = err_alpha * i_alpha + err_beta * i_beta;

    /* Normalize by |I|² to make gain independent of current magnitude.
     * Avoid division by zero at low current. */
    float i_sq = i_alpha * i_alpha + i_beta * i_beta;
    if (i_sq < 0.25f) return;  /* Skip below 0.5A — poor SNR */

    float dRs = adapt_gain * err_proj / i_sq * smo->dt_val;

    /* Update Rs estimate */
    smo->Rs_est += dRs;

    /* Clamp to ±50% of initial value */
    float Rs_min = Rs_init * 0.5f;
    float Rs_max = Rs_init * 1.5f;
    if (smo->Rs_est < Rs_min) smo->Rs_est = Rs_min;
    if (smo->Rs_est > Rs_max) smo->Rs_est = Rs_max;

    /* Recompute plant coefficients with updated Rs */
    smo->F = 1.0f - smo->Rs_est * smo->dt_val / smo->Ls_val;
    /* G = dt/Ls — doesn't depend on Rs, no update needed */
}

/* ── PLL for smooth speed estimation ─────────────────────────── */

void v3_smo_pll_init(SMO_PLL_t *pll, float bw_hz, float omega_max)
{
    /* Speed-adaptive PLL: BW scales linearly with speed.
     * At low speed (handoff), BW = bw_min → heavy filtering.
     * At high speed, BW = bw_max → fast tracking.
     *
     * Rule of thumb: PLL BW should be ~10-25% of electrical freq.
     * Hurst: f_elec_handoff = 262/(2π) = 42 Hz → BW_min = 10 Hz.
     *         f_elec_max = 1500/(2π) = 239 Hz → BW_max = 60 Hz.
     *
     * bw_hz parameter becomes bw_max. */
    pll->bw_min_hz  = bw_hz * 0.33f;   /* 33% of max BW = 20 Hz min */
    pll->bw_max_hz  = bw_hz;
    pll->omega_bw_ref = omega_max * 0.5f;  /* BW=max at 50% max speed */

    /* Initialize gains at min BW (startup) */
    float bw_init = pll->bw_min_hz;
    pll->kp = 2.0f * FOC_TWO_PI * bw_init;
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
    /* Speed-adaptive bandwidth: scale BW linearly with |omega|.
     * Low speed → narrow BW → reject SMO noise.
     * High speed → wide BW → track speed changes quickly.
     * Recompute Kp/Ki every tick (cheap: 2 multiplies + 1 clamp). */
    {
        float abs_w = pll->omega_est;
        if (abs_w < 0.0f) abs_w = -abs_w;
        float frac = abs_w / pll->omega_bw_ref;
        if (frac > 1.0f) frac = 1.0f;

        /* Slightly overdamped (ζ=1.2) for smoother transients */
        float bw = pll->bw_min_hz + frac * (pll->bw_max_hz - pll->bw_min_hz);
        float w_n = FOC_TWO_PI * bw;
        pll->kp = 2.0f * 1.2f * w_n;       /* 2·ζ·ωn */
        pll->ki = w_n * w_n;                 /* ωn² */
    }

    /* Phase error */
    float delta = theta_meas - pll->theta_est;
    if (delta >  FOC_PI_F) delta -= FOC_TWO_PI;
    if (delta < -FOC_PI_F) delta += FOC_TWO_PI;

    /* PI update */
    pll->omega_est += pll->ki * delta * dt;

    /* Clamp speed: unidirectional drive — motor doesn't reverse.
     * Allowing negative omega_est causes PLL to lock onto the wrong
     * direction at low speed, creating a positive feedback desync. */
    if (pll->omega_est > pll->omega_max) pll->omega_est = pll->omega_max;
    if (pll->omega_est < 0.0f) pll->omega_est = 0.0f;

    /* Advance angle */
    pll->theta_est += (pll->omega_est + pll->kp * delta) * dt;
    pll->theta_est = foc_angle_wrap(pll->theta_est);

    return pll->omega_est;
}
