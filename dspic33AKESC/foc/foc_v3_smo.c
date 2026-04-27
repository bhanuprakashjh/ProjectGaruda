/**
 * @file foc_v3_smo.c
 * @brief Sliding Mode Observer — AN1078 float port.
 *
 * Direct translation of Microchip AN1078 `smcpos.c` (Q15 fixed-point)
 * to IEEE float, preserving the algorithm exactly.  Reference:
 *   lvmc-dspic33ck256mp508-an1078/smcpos.c
 *
 * Algorithm (per tick at FOC_TS_FAST_S = 1/f_pwm):
 *
 *   1. Current-model update (forward Euler):
 *        î[k+1] = F·î[k] + G·(V − E1 − Z)
 *      where F = 1 − Rs·dt/Ls,  G = dt/Ls.
 *
 *   2. Current error:  e = î − i_meas.
 *
 *   3. Sliding control (linear-in-boundary, saturated outside):
 *        if |e| < MaxSMCError:  Z = Kslide · e / MaxSMCError
 *        else:                  Z = ±Kslide   (sign of e)
 *
 *   4. Back-EMF extraction via two cascaded 1st-order IIRs on Z:
 *        E1 += Kslf · (Z  − E1)      (fed back into current model)
 *        E2 += Kslf · (E1 − E2)      (used for angle)
 *      Kslf = |ω_elec| · THETA_FILTER_CNST, floored at kslf_min.
 *
 *   5. Angle:  θ = atan2(−E2α, E2β) + CONSTANT_PHASE_SHIFT.
 *
 *   6. Speed:  LP-filtered delta-θ per tick.
 *
 * Phase 1 of the AN1078-port plan.  The runtime/handoff logic in
 * foc_v3_control.c still calls `v3_smo_pll_*` — those remain untouched
 * until Phase 4 removes the PLL.
 *
 * Component: FOC V3
 */

#include "foc_v3_smo.h"
#include "foc_v2_math.h"      /* FOC_TWO_PI, FOC_PI_F, foc_angle_wrap */
#include <math.h>

/* ── SMO Init / Reset ────────────────────────────────────────── */

void v3_smo_init(SMO_Observer_t *smo, const SMO_Config_t *cfg)
{
    /* Discrete plant model (forward Euler):
     *   F = 1 − Rs·dt/Ls,  G = dt/Ls.
     * Stability requires F ∈ (0, 1) — at 24 kHz, 2810 (22mΩ, 10µH):
     *   F = 1 − 0.022·4.17e-5/1e-5 = 1 − 0.0917 = 0.908. */
    smo->F = 1.0f - cfg->Rs * cfg->dt / cfg->Ls;
    smo->G = cfg->dt / cfg->Ls;

    /* Stored motor + timing */
    smo->Rs_est    = cfg->Rs;
    smo->Ls        = cfg->Ls;
    smo->dt        = cfg->dt;
    smo->lambda_pm = cfg->lambda_pm;

    /* Sliding controller */
    smo->Kslide       = cfg->Kslide;
    smo->MaxSMCError  = cfg->MaxSMCError;

    /* BEMF LPF */
    smo->theta_filter_cnst = cfg->theta_filter_cnst;
    smo->kslf_min          = cfg->kslf_min;

    /* Angle offset (AN1078 CONSTANT_PHASE_SHIFT) */
    smo->theta_offset = cfg->theta_offset;

    /* Back-compat alias for focObsGain snapshot field */
    smo->k_base = cfg->Kslide;

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
    smo->alpha_now   = smo->kslf_min;
    smo->err_alpha   = 0.0f;
    smo->err_beta    = 0.0f;
    smo->bemf_mag_filt = 0.0f;
    smo->confidence  = 0.0f;
    smo->residual    = 0.0f;
    smo->observable  = false;
}

/* ── SMO Update (one tick at f_pwm) ──────────────────────────── */

void v3_smo_update(SMO_Observer_t *smo,
                float v_alpha, float v_beta,
                float i_alpha, float i_beta,
                float omega_est)
{
    float abs_omega = (omega_est >= 0.0f) ? omega_est : -omega_est;

    /* ── 1. Current-model update (forward Euler) ───────────────
     * î[k+1] = F·î[k] + G·(V − E1 − Z).  Uses E1 and Z from the
     * previous tick, as in AN1078's CalcEstI. */
    smo->i_hat_alpha = smo->F * smo->i_hat_alpha
                     + smo->G * (v_alpha - smo->e1_alpha - smo->z_alpha);
    smo->i_hat_beta  = smo->F * smo->i_hat_beta
                     + smo->G * (v_beta  - smo->e1_beta  - smo->z_beta);

    /* ── 2. Current estimation error ──────────────────────────── */
    smo->err_alpha = smo->i_hat_alpha - i_alpha;
    smo->err_beta  = smo->i_hat_beta  - i_beta;

    /* ── 3. Sliding switching law — linear in boundary, saturate
     *      outside.  Direct port of AN1078 CalcEstI. */
    {
        float abs_a = (smo->err_alpha >= 0.0f) ? smo->err_alpha : -smo->err_alpha;
        if (abs_a < smo->MaxSMCError) {
            smo->z_alpha = smo->Kslide * smo->err_alpha / smo->MaxSMCError;
        } else {
            smo->z_alpha = (smo->err_alpha > 0.0f) ? smo->Kslide : -smo->Kslide;
        }

        float abs_b = (smo->err_beta >= 0.0f) ? smo->err_beta : -smo->err_beta;
        if (abs_b < smo->MaxSMCError) {
            smo->z_beta = smo->Kslide * smo->err_beta / smo->MaxSMCError;
        } else {
            smo->z_beta = (smo->err_beta > 0.0f) ? smo->Kslide : -smo->Kslide;
        }
    }

    /* ── 4. Back-EMF extraction — cascaded 1st-order IIR on Z.
     * AN1078 uses single Kslf = KslfFinal, speed-proportional, floored.
     * Kslf = |ω|·THETA_FILTER_CNST, clamped at kslf_min. */
    {
        float kslf = abs_omega * smo->theta_filter_cnst;
        if (kslf < smo->kslf_min) kslf = smo->kslf_min;
        if (kslf > 1.0f)          kslf = 1.0f;
        smo->alpha_now = kslf;

        /* Stage 1 — E1 fed back into current model next tick */
        smo->e1_alpha += kslf * (smo->z_alpha - smo->e1_alpha);
        smo->e1_beta  += kslf * (smo->z_beta  - smo->e1_beta );

        /* Stage 2 — E2 used for angle extraction */
        smo->e2_alpha += kslf * (smo->e1_alpha - smo->e2_alpha);
        smo->e2_beta  += kslf * (smo->e1_beta  - smo->e2_beta );
    }

    /* ── 5. Angle from filtered back-EMF.
     *      Eα = −λω sin(θ), Eβ = +λω cos(θ)
     *   → θ = atan2(−Eα, Eβ) + CONSTANT_PHASE_SHIFT. */
    {
        float theta_raw = atan2f(-smo->e2_alpha, smo->e2_beta) + smo->theta_offset;
        /* Wrap to [0, 2π) */
        while (theta_raw >= FOC_TWO_PI) theta_raw -= FOC_TWO_PI;
        while (theta_raw < 0.0f)        theta_raw += FOC_TWO_PI;
        smo->theta_est = theta_raw;
    }

    /* ── 6. Speed estimate from delta-θ, LP-filtered ─────────── */
    {
        float dtheta = smo->theta_est - smo->theta_prev;
        if (dtheta >  FOC_PI_F) dtheta -= FOC_TWO_PI;
        if (dtheta < -FOC_PI_F) dtheta += FOC_TWO_PI;
        smo->theta_prev = smo->theta_est;

        float omega_raw = dtheta / smo->dt;
        smo->omega_lpf += 0.05f * (omega_raw - smo->omega_lpf);
        smo->omega_est = smo->omega_lpf;
    }

    /* ── 7. Diagnostics (do NOT affect the observer math) ─────
     * BEMF magnitude, confidence = observed/expected, residual =
     * LP-filtered |err|, observable flag. */
    {
        float bemf_mag = sqrtf(smo->e2_alpha * smo->e2_alpha
                             + smo->e2_beta  * smo->e2_beta);
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

        float err_mag = sqrtf(smo->err_alpha * smo->err_alpha
                            + smo->err_beta  * smo->err_beta);
        smo->residual += 0.005f * (err_mag - smo->residual);

        smo->observable = (smo->confidence >= smo->conf_min);
    }
}

/* ── SMO Phase Delay ─────────────────────────────────────────────
 *
 * AN1078 bakes the observer's bulk angle correction into a constant
 * CONSTANT_PHASE_SHIFT applied inside SMO (see step 5 above), and does
 * NOT compute dynamic group-delay compensation.  We preserve the API
 * for call-site compatibility but return 0 — all offset is already
 * applied to `theta_est`. */
float v3_smo_phase_delay(const SMO_Observer_t *smo, float omega)
{
    (void)smo;
    (void)omega;
    return 0.0f;
}

/* ── Rs Online Adaptation — unchanged from pre-AN1078 port ─────
 *
 * Not called from foc_v3_control.c today; kept as reusable utility.
 * Projects current error onto the measured current vector and nudges
 * Rs_est by an LP-filtered amount, with ±50% of initial Rs as hard
 * clamps.  Recomputes F. */
void v3_smo_adapt_rs(SMO_Observer_t *smo,
                     float i_alpha, float i_beta,
                     float Rs_init)
{
    const float adapt_gain = 0.1f;

    float err_proj = smo->err_alpha * i_alpha + smo->err_beta * i_beta;

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

/* ── PLL for smooth speed estimation ─────────────────────────────
 *
 * Retained verbatim pending Phase 4 removal.  AN1078 does not use a
 * separate PLL — it accumulates delta-θ over N ticks to compute ω.
 * Our existing foc_v3_control.c still calls these, so keeping them
 * callable avoids touching the control loop in this phase. */

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

        float bw  = pll->bw_min_hz + frac * (pll->bw_max_hz - pll->bw_min_hz);
        float w_n = FOC_TWO_PI * bw;
        pll->kp = 2.0f * 1.2f * w_n;       /* 2·ζ·ωn (ζ = 1.2) */
        pll->ki = w_n * w_n;
    }

    float delta = theta_meas - pll->theta_est;
    if (delta >  FOC_PI_F) delta -= FOC_TWO_PI;
    if (delta < -FOC_PI_F) delta += FOC_TWO_PI;

    float abs_delta = (delta >= 0.0f) ? delta : -delta;
    pll->innovation = abs_delta;
    pll->innovation_lpf += 0.01f * (abs_delta - pll->innovation_lpf);

    pll->omega_est += pll->ki * delta * dt;
    if (pll->omega_est > pll->omega_max) pll->omega_est = pll->omega_max;
    if (pll->omega_est < 0.0f)           pll->omega_est = 0.0f;

    pll->theta_est += (pll->omega_est + pll->kp * delta) * dt;
    pll->theta_est = foc_angle_wrap(pll->theta_est);

    return pll->omega_est;
}
