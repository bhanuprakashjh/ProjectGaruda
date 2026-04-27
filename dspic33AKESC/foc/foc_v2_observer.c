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
    obs->bemf_int = 0.0f;
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
    /* Dead-time compensated voltages.
     * Dead time makes actual voltage LOWER than commanded for positive
     * current (body diode clamps to rail during dead interval).
     * Observer needs actual voltage, so SUBTRACT the dead-time error.
     * VESC: x1 -= dt_comp * sign(I) * dt  (same effect).
     * Previous code had WRONG sign (+), which doubled the DT error
     * and was the reason DT comp was disabled ("caused distortion"). */
    /* Soft-sign threshold = 2.0A: below this, DT comp scales linearly
     * to zero.  At no-load (~0.03A from ADC noise), DT comp is negligible
     * → no random noise injection into observer.  At 2A+, full correction.
     * Previous 0.1A threshold caused ±0.086V random walk at no-load. */
    float v_a_eff = v_alpha - dt_comp_v * foc_soft_sign(i_alpha, 2.0f);
    float v_b_eff = v_beta  - dt_comp_v * foc_soft_sign(i_beta,  2.0f);

    /* Flux integration with L·dI cancellation.
     * dx = (V - Rs·I)·dt - Ls·(I[n] - I[n-1])
     * Rs·I uses unfiltered current (no lag on resistive term).
     *
     * L·dI noise: accumulates at Ls/λ per tick from ADC noise.
     * For high-Ls motors (Hurst: Ls/λ=0.29), this dominates at low speed.
     * For low-Ls motors (A2212: Ls/λ=0.053), noise coupling is small.
     * No dI filtering applied here — raw per-sample difference like MESC.
     * High-Ls motors need higher handoff speed instead (SNR > 1). */
    obs->x1 += (v_a_eff - Rs * i_alpha) * dt
             - Ls * (i_alpha - obs->i_alpha_last);
    obs->x2 += (v_b_eff - Rs * i_beta)  * dt
             - Ls * (i_beta  - obs->i_beta_last);
    obs->i_alpha_last = i_alpha;
    obs->i_beta_last  = i_beta;

    /* 2026-04-23: Adaptive lambda DISABLED for 2810@24V debugging.
     * Suspected cause of observer drift that gives Vd ~1.8V steady-state
     * bias across all speeds. Force lambda_est = configured lambda so
     * the flux magnitude clamp uses a fixed reference.
     * Low-speed upward boost (×1.1/tick when |flux|<λ/2) also disabled
     * — at 2000 rad/s flux should be well above λ/2 but any transient
     * dip would cause explosive inflation. */
    float mag_sq = obs->x1 * obs->x1 + obs->x2 * obs->x2;
    (void)mag_sq;
    obs->lambda_est = lambda;    /* fixed, no adaptation */

    /* Circular normalization: downward clamp only, no upward boost. */
    {
        float mag_c = sqrtf(obs->x1 * obs->x1 + obs->x2 * obs->x2);
        if (mag_c > lambda) {
            float sc = lambda / mag_c;
            obs->x1 *= sc;
            obs->x2 *= sc;
        }
    }

    /* Angle extraction — direct atan2, no PLL needed for angle */
    obs->theta_est = atan2f(obs->x2, obs->x1);
    if (obs->theta_est < 0.0f)
        obs->theta_est += FOC_TWO_PI;

    /* Observer gain scheduling: scale with duty, floor at 5%. */
    float base_gain = 1e-3f / (lambda * lambda + 1e-12f);
    float duty_factor = (duty_abs > 0.05f) ? duty_abs : 0.05f;
    obs->gain = duty_factor * base_gain;

    /* PLL for speed only (smooth speed estimate for speed PI). */
    float delta = obs->theta_est - pll->theta_est;
    if (delta >  FOC_PI_F) delta -= FOC_TWO_PI;
    if (delta < -FOC_PI_F) delta += FOC_TWO_PI;

    pll->omega_est += pll->ki * delta * dt;
    /* Clamp speed to prevent runaway from noisy observer angle */
    if (pll->omega_est >  pll->omega_max) pll->omega_est =  pll->omega_max;
    if (pll->omega_est < -pll->omega_max) pll->omega_est = -pll->omega_max;
    pll->theta_est += (pll->omega_est + pll->kp * delta) * dt;
    pll->theta_est = foc_angle_wrap(pll->theta_est);
}

/* ── BEMF angle correction (PLL_OBS hybrid) ──────────────────── */

void foc_observer_bemf_correct(FOC_Observer_t *obs,
                                float vd, float vq,
                                float id, float iq,
                                float omega,
                                float Rs, float Ls, float lambda)
{
    /* Speed gate with gain ramp.
     * Below 2000 rad/s: no correction (startup current creates
     *   systematic BEMFd from Ls*dId/dt → false angle error).
     * 2000-4000 rad/s: linear ramp from 0 to BEMF_KP_MAX.
     * Above 4000 rad/s: full correction strength.
     *
     * The 180° attractor problem occurs at high speed with zero
     * load current.  The ramp ensures gentle activation after
     * the high-current startup regime. */
    float abs_omega = (omega >= 0.0f) ? omega : -omega;
    /* 2026-04-23: BEMF correction re-enabled with a wide/gentle ramp.
     * Earlier disable (threshold=10000) caused 50° observer angle drift
     * because flux integrator had no angle feedback. Motor was running
     * open-loop and PI had to absorb the full offset via Vd integrator,
     * which saturated at higher modIdx → BOARD_PCI.
     * Original 2000 rad/s gate also problematic (discontinuous activation).
     * New: gate at 200 rad/s (well below handoff), ramp Kp from 0 → full
     * over a wide range so observer gets progressive feedback from the
     * very start of CL. */
    if (abs_omega < 200.0f) {
        obs->bemf_int = 0.0f;
        return;
    }

    /* d-axis BEMF residual (motor voltage equation).
     * Standard d-q model: Vd = Rs*Id + Ld*dId/dt - ω*Lq*Iq
     * In steady state (dId/dt ≈ 0):
     *   BEMFd = Vd - Rs*Id + ω*Ls*Iq ≈ 0  (correct angle)
     *   BEMFd ≈ ω*λ*sin(δθ)               (angle error δθ)
     *
     * Non-salient motor (Ld ≈ Lq = Ls), use measured ω for
     * cross-coupling. */
    float bemfd = vd - Rs * id + omega * Ls * iq;

    /* Normalize by ω*λ to get angle error in radians. */
    float denom = omega * lambda;
    if (denom > -0.5f && denom < 0.5f)
        return;

    float angle_err = bemfd / denom;

    /* Clamp to ±0.5 rad (~29°) — allows strong correction for
     * large errors while preventing wild overshoot. */
    angle_err = foc_clampf(angle_err, -0.5f, 0.5f);

    /* Speed-ramped P-only correction.
     * Kp ramps from 0 at 2000 rad/s to 0.15 at 4000 rad/s.
     *
     * At 5500 rad/s, 21° error (Vd=1.12V, BEMFd=1.3V):
     *   angle_err = 0.42 (clamped to 0.5)
     *   correction = 0.15 * 0.42 = 0.063 rad/tick = 1512 rad/s
     *   Settling time: ~7 ticks (0.3ms) — fast enough to
     *   prevent the 180° flip before HW OC (3ms window).
     *
     * At 3000 rad/s (mid-ramp), Kp = 0.075:
     *   Systematic BEMFd from 2A Iq: 0.1V
     *   angle_err = 0.1/1.69 = 0.059 rad
     *   correction = 0.075 * 0.059 = 0.0044 rad/tick
     *   vs ω*dt = 0.125 rad/tick → 3.5% perturbation (safe). */
    /* Gentler correction + wider ramp (2026-04-23).
     * KP_MAX halved from 0.15 to prevent any single-step kick being large.
     * Ramp starts at 500 rad/s (just above gate, below OL handoff) and
     * reaches full gain by 3000 rad/s — 2.5 krad/s of gentle engagement. */
    const float BEMF_KP_MAX  = 0.075f;    /* was 0.15 — halved for smoother engagement */
    const float RAMP_LO      = 500.0f;    /* was 10000 — engage very early */
    const float RAMP_HI      = 3000.0f;   /* was 12000 — full gain by typical cruise speed */

    float kp_scale = (abs_omega - RAMP_LO) / (RAMP_HI - RAMP_LO);
    if (kp_scale > 1.0f) kp_scale = 1.0f;
    float kp = BEMF_KP_MAX * kp_scale;

    float correction = kp * angle_err;

    /* Rotate flux vector (x1, x2) by correction angle.
     * Positive correction = rotate forward (increase angle).
     * For small corrections (<0.01 rad): use first-order approx. */
    float x1_new, x2_new;
    if (correction > -0.01f && correction < 0.01f) {
        /* Small angle: cos≈1, sin≈correction */
        x1_new = obs->x1 - obs->x2 * correction;
        x2_new = obs->x1 * correction + obs->x2;
    } else {
        float cos_c = cosf(correction);
        float sin_c = sinf(correction);
        x1_new = obs->x1 * cos_c - obs->x2 * sin_c;
        x2_new = obs->x1 * sin_c + obs->x2 * cos_c;
    }
    obs->x1 = x1_new;
    obs->x2 = x2_new;

    /* Recompute angle after correction */
    obs->theta_est = atan2f(obs->x2, obs->x1);
    if (obs->theta_est < 0.0f)
        obs->theta_est += FOC_TWO_PI;
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
    /* omega_max must be set by caller after init (default safe value) */
    pll->omega_max = 20000.0f;
}
