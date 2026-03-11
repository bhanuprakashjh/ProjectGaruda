/**
 * @file foc_v3_control.c
 * @brief FOC v3 control loop — SMO observer + open-loop ramp startup.
 *
 * State machine:
 *   IDLE → ARMED → ALIGN → OL_RAMP → CLOSED_LOOP → FAULT
 *                                          ↓ (throttle=0 >1s)
 *                                        ARMED (clean re-arm)
 *
 * Fast loop (24 kHz): Clarke → SMO → Park → PI_d/PI_q
 *   → circular clamp → InvPark → SVPWM → duty output
 *
 * Slow loop (1 kHz): Speed PI → Iq_ref, arming, fault detect
 *
 * The SMO runs free during OL ramp (receives real I, V but its
 * angle is not used for commutation). At handoff speed, if the
 * PLL tracks consistently, commutation switches to SMO angle.
 *
 * Component: FOC V3
 */

#include <xc.h>
#include "foc_v3_control.h"
#include "foc_v3_smo.h"
#include "foc_v2_math.h"
#include "foc_v2_pi.h"
#include "../garuda_foc_params.h"
#include "../garuda_types.h"

/* ── V/f Open-Loop Startup ───────────────────────────────────
 * Voltage-mode startup with proper V/f curve.
 * Vq = VF_BOOST + Ke × ω — voltage tracks BEMF plus a torque
 * margin.  Boost provides lock-in torque at standstill, Ke×ω
 * keeps V above BEMF as speed rises.
 *
 * A2212 (Ke=0.000563): at 500 rad/s → 0.5+0.28=0.78V (old: 1.0V)
 * Hurst (Ke=0.00742):  at 262 rad/s → 0.5+1.94=2.44V (old: 1.0V → desync!)
 *
 * Handoff at profile STARTUP_HANDOFF_RAD_S via st->handoff_rad_s. */
#define VF_BOOST_V           0.5f   /* Starting torque voltage (V) */
#define VF_OL_RAMP_RATE      200.0f /* Electrical rad/s² during V/f */

/* ── Constants ───────────────────────────────────────────────── */
#define DT_FAST          FOC_TS_FAST_S
#define DT_SLOW          FOC_TS_SLOW_S
#define SLOW_DIV         FOC_SLOW_DIV

/* ADC calibration */
#define CAL_SAMPLES      1024U
#define CAL_SHIFT         10U

/* Overcurrent debounce (2ms at 24kHz) */
#define OC_DEBOUNCE_TICKS 48U

/* Board fault pin: RB11 = U25A(OV)+U25B(OC), active-low */
#define BOARD_FAULT_PIN      PORTBbits.RB11
#define BOARD_FAULT_ACTIVE   0
#define BOARD_FAULT_DEBOUNCE 48U

/* Handoff: dwell at handoff speed before CL entry.
 * 6000 ticks = 250ms — let SMO converge. */
#define HANDOFF_DWELL_TICKS 6000U

/* CL holdoff: after CL entry, hold initial Iq while PLL/SMO stabilize.
 * Speed PI takes over after holdoff.
 * 20 slow-loop ticks = 20ms — just enough for PLL to lock. */
#define CL_HOLDOFF_SLOW_TICKS 20U

/* Re-arm: throttle=0 for 1s in slow loop ticks */
#define REARM_TIMEOUT_TICKS 1000U

/* ── Helpers ─────────────────────────────────────────────────── */

static inline float raw_to_amps(uint16_t raw, float offset)
{
    return -((float)raw - offset) * CURRENT_SCALE_A_PER_COUNT;
}

static inline float raw_to_vbus(uint16_t raw)
{
    return (float)raw * VBUS_SCALE_V_PER_COUNT;
}

static void reset_startup(V3_State_t *st)
{
    st->omega_ol    = 0.0f;
    st->theta_ol    = 0.0f;
    st->align_ctr   = 0;
    st->ramp_ctr    = 0;
    st->handoff_ctr  = 0;
    st->angle_ok_ctr    = 0;
    st->cl_active       = false;
    st->blend_ctr       = 0;
    st->handoff_offset  = 0.0f;
    st->iq_ref      = 0.0f;
    st->id_ref      = 0.0f;
    st->vd          = 0.0f;
    st->vq          = 0.0f;
    st->da_prev     = 0.5f;
    st->db_prev     = 0.5f;
    st->dc_prev     = 0.5f;
    st->oc_debounce_ctr = 0;
    st->sub_state   = 0;

    foc_pi_reset(&st->pid_d);
    foc_pi_reset(&st->pid_q);
    foc_pi_reset(&st->pid_spd);
    v3_smo_reset(&st->smo);
    v3_smo_pll_reset(&st->pll);
}

/* ── Public API ──────────────────────────────────────────────── */

void foc_v3_init(V3_State_t *st, const FOC_MotorParams_t *params)
{
    /* Zero entire state */
    V3_State_t zero = {0};
    *st = zero;

    st->mode = V3_IDLE;

    /* Copy motor params */
    st->Rs        = params->Rs;
    st->Ls        = params->Ls;
    st->lambda_pm = params->lambda_pm;
    st->Ke        = params->Ke;

    /* Dead-time compensation */
    st->dt_comp = params->vbus_nom_v * FOC_DT_COMP_FRAC;

    /* Copy runtime startup params */
    st->align_iq       = params->align_iq_a;
    st->ramp_iq        = params->ramp_iq_a;
    st->align_ticks    = params->align_ticks;
    st->iq_ramp_ticks  = params->iq_ramp_ticks;
    st->ramp_rate      = params->ramp_rate_rps2;
    st->handoff_rad_s  = params->handoff_rad_s;
    st->fault_oc_a     = params->fault_oc_a;
    st->max_elec_rad_s = params->max_elec_rad_s;
    st->use_vf_startup = STARTUP_USE_VF;

    /* Initialize SMO observer */
    v3_smo_init(&st->smo, params->Rs, params->Ls, params->lambda_pm,
             params->vbus_nom_v, DT_FAST);

    /* Initialize PLL (100 Hz BW for speed estimation) */
    /* PLL max BW = 60 Hz. Speed-adaptive: scales from 9→60 Hz.
     * At handoff (262 rad/s): BW ≈ 9 Hz (clean speed estimate).
     * At max (2000 rad/s): BW = 60 Hz (fast tracking). */
    v3_smo_pll_init(&st->pll, 60.0f, params->max_elec_rad_s);

    /* Current PI controllers */
    float vmax = params->vbus_nom_v * 0.95f * FOC_INV_SQRT3;
    foc_pi_init(&st->pid_d, params->kp_dq, params->ki_dq, -vmax, vmax);
    foc_pi_init(&st->pid_q, params->kp_dq, params->ki_dq, -vmax, vmax);

    /* Speed PI controller → Iq reference.
     * Allow negative Iq for active braking (regenerative) so the
     * motor can decelerate when speed > target. Limit braking to
     * -1A to avoid excessive regen current into bus caps.
     * Positive clamp at 50% max current for OC safety. */
    float iq_clamp = MOTOR_MAX_CURRENT_A * 0.5f;
    foc_pi_init(&st->pid_spd, KP_SPD, KI_SPD, -1.0f, iq_clamp);
}

void foc_v3_start(V3_State_t *st)
{
    if (st->mode == V3_ARMED) {
        reset_startup(st);
        /* Reset PI integrators — stale values from previous run
         * cause alignment failure (wrong voltage at theta=0). */
        foc_pi_preload(&st->pid_d, 0.0f);
        foc_pi_preload(&st->pid_q, 0.0f);
        foc_pi_preload(&st->pid_spd, 0.0f);
        st->mode = V3_ALIGN;
    }
}

void foc_v3_stop(V3_State_t *st)
{
    st->mode = V3_ARMED;
    reset_startup(st);
}

void foc_v3_fault(V3_State_t *st, uint16_t fault_code)
{
    st->mode = V3_FAULT;
    st->fault_code = fault_code;
}

/* ── Fast-loop tick (24 kHz) ─────────────────────────────────── */

void foc_v3_fast_tick(V3_State_t *st,
                      uint16_t ia_raw, uint16_t ib_raw,
                      uint16_t vbus_raw, uint16_t throttle,
                      float *da_out, float *db_out, float *dc_out)
{
    /* Default: 50% duty (no net voltage) */
    *da_out = 0.5f;
    *db_out = 0.5f;
    *dc_out = 0.5f;

    /* Vbus conversion + dynamic dt_comp */
    st->vbus = raw_to_vbus(vbus_raw);
    st->dt_comp = st->vbus * FOC_DT_COMP_FRAC;

    /* ── IDLE / ARMED: ADC offset calibration ───────────────── */
    if (st->mode == V3_IDLE || st->mode == V3_ARMED) {
        if (!st->cal_done) {
            st->cal_accum_a += ia_raw;
            st->cal_accum_b += ib_raw;
            st->cal_count++;
            if (st->cal_count >= CAL_SAMPLES) {
                st->ia_offset = (float)(st->cal_accum_a >> CAL_SHIFT);
                st->ib_offset = (float)(st->cal_accum_b >> CAL_SHIFT);
                st->cal_done  = true;
            }
        }
        st->sub_state = (st->mode == V3_IDLE) ? 0 : 1;
        goto slow_loop;
    }

    /* ── FAULT: outputs disabled ─────────────────────────────── */
    if (st->mode == V3_FAULT) {
        st->sub_state = 0;
        *da_out = 0.5f;
        *db_out = 0.5f;
        *dc_out = 0.5f;
        goto slow_loop;
    }

    /* ── Motor running: ALIGN / OL_RAMP / CLOSED_LOOP ──────── */

    /* Dynamic PI voltage clamp — tracks Vbus (matches v2) */
    {
        float vclamp = st->vbus * 0.95f * FOC_INV_SQRT3;
        st->pid_d.out_max =  vclamp;
        st->pid_d.out_min = -vclamp;
        st->pid_q.out_max =  vclamp;
        st->pid_q.out_min = -vclamp;
    }

    /* 1. Convert ADC to physical units */
    float ia = raw_to_amps(ia_raw, st->ia_offset);
    float ib = raw_to_amps(ib_raw, st->ib_offset);

    /* 2. Clarke transform */
    float i_alpha, i_beta;
    foc_clarke(ia, ib, &i_alpha, &i_beta);

    /* 3. Reconstruct applied voltage from previous PWM duty cycles.
     *    Using actual duties (post-SVPWM) instead of commanded Vd/Vq
     *    accounts for SVM zero-sequence injection and duty clipping.
     *    duties are [0..1], center is 0.5. Convert back to αβ volts:
     *      va = (da - 0.5) * Vbus,  vb = (db - 0.5) * Vbus, etc.
     *    Then inverse Clarke: vα = va, vβ = (va + 2·vb) / √3
     *    But SVM adds common-mode offset, which Clarke cancels:
     *      vα = (2·da - db - dc) / 3 · Vbus
     *      vβ = (db - dc) / √3 · Vbus                              */
    float v_alpha, v_beta;
    {
        float da = st->da_prev, db = st->db_prev, dc = st->dc_prev;
        v_alpha = (2.0f * da - db - dc) * (1.0f / 3.0f) * st->vbus;
        v_beta  = (db - dc) * FOC_INV_SQRT3 * st->vbus;
    }

    /* 4. Run SMO observer (always, even during OL — let it converge).
     * Speed hint for adaptive LPF: during V/f we know the real speed
     * (omega_ol), so use it directly. Using PLL omega creates a
     * chicken-and-egg: PLL needs SMO → SMO needs correct LPF → LPF
     * needs real speed → PLL is garbage. */
    float smo_omega_hint;
    if (st->mode == V3_ALIGN || st->mode == V3_OL_RAMP)
        smo_omega_hint = st->omega_ol;
    else
        smo_omega_hint = st->pll.omega_est;

    v3_smo_update(&st->smo,
               v_alpha, v_beta,
               i_alpha, i_beta,
               smo_omega_hint, DT_FAST);

    /* 5. Run PLL on SMO angle */
    v3_smo_pll_update(&st->pll, st->smo.theta_est, DT_FAST);

    /* 5b. Rs online adaptation (Phase 3).
     * Only during CL at moderate+ speed where current SNR is good.
     * Updates smo.F coefficient to track temperature-dependent Rs. */
    if (st->mode == V3_CLOSED_LOOP && st->pll.omega_est > st->handoff_rad_s) {
        v3_smo_adapt_rs(&st->smo, i_alpha, i_beta, st->Rs);
    }

    /* 6. Select commutation angle based on mode */
    float theta_drive;
    float iq_ref, id_ref;

    switch (st->mode) {
    case V3_ALIGN: {
        /* Alignment: hold θ=0, lock rotor to d-axis.
         * I/f: inject Id=align_iq with PI → accurate voltage for SMO.
         * V/f: ramp Vq from 0→VF_BOOST (for low-Rs motors). */
        theta_drive = 0.0f;
        st->align_ctr++;

        if (st->use_vf_startup) {
            id_ref = 0.0f;
            iq_ref = 0.0f;  /* V/f: refs unused, set for telemetry */
        } else {
            /* I/f: d-axis current injection at θ=0 locks rotor */
            id_ref = st->align_iq;
            iq_ref = 0.0f;
        }

        if (st->align_ctr >= st->align_ticks) {
            st->mode = V3_OL_RAMP;
            st->ramp_ctr = 0;
            st->omega_ol = 0.0f;
            st->theta_ol = 0.0f;

            /* Seed SMO with alignment angle */
            v3_smo_reset(&st->smo);
            v3_smo_pll_reset(&st->pll);
        }

        st->sub_state = 2;  /* ALIGN */
        break;
    }

    case V3_OL_RAMP: {
        /* Open-loop forced angle ramp to handoff speed.
         * I/f: PI controls current (Iq=ramp_iq), voltage auto-generated.
         * V/f: fixed Vq at forced angle (for low-Rs motors). */
        st->ramp_ctr++;

        if (st->use_vf_startup) {
            iq_ref = 0.0f;
            id_ref = 0.0f;  /* V/f: refs unused, set for telemetry */
        } else {
            /* I/f: PI drives q-axis current at forced angle */
            iq_ref = st->ramp_iq;
            id_ref = 0.0f;
        }

        /* Speed ramp — cap at profile handoff speed */
        st->omega_ol += st->ramp_rate * DT_FAST;
        if (st->omega_ol > st->handoff_rad_s)
            st->omega_ol = st->handoff_rad_s;

        /* Advance forced angle */
        st->theta_ol += st->omega_ol * DT_FAST;
        st->theta_ol = foc_angle_wrap(st->theta_ol);

        theta_drive = st->theta_ol;

        /* Check handoff condition: confidence-based (Phase 3).
         *
         * FOUR checks must all pass simultaneously:
         *  1) PLL speed within ±50% of OL speed (PLL alive)
         *  2) SMO angle within ±120° of forced angle (angle coherence)
         *  3) SMO confidence > 0.3 (BEMF magnitude reasonable)
         *  4) All sustained for HANDOFF_DWELL_TICKS
         *
         * The confidence metric (smo.confidence) measures observed BEMF
         * magnitude vs expected (λ·ω).  When converged, confidence ≈ 0.5-1.0.
         * Threshold 0.3 is low enough to not block handoff on motors with
         * parameter errors, but high enough to reject random noise. */
        if (st->omega_ol >= st->handoff_rad_s) {
            /* Speed check: PLL tracking OL speed */
            float pll_speed = st->pll.omega_est;
            float ol_speed = st->omega_ol;
            bool pll_tracking = (pll_speed > ol_speed * 0.5f) &&
                                (pll_speed < ol_speed * 1.5f);

            /* Angle coherence */
            float angle_err = st->smo.theta_est - st->theta_ol;
            if (angle_err >  FOC_PI_F) angle_err -= FOC_TWO_PI;
            if (angle_err < -FOC_PI_F) angle_err += FOC_TWO_PI;
            float abs_angle_err = (angle_err >= 0.0f) ? angle_err : -angle_err;
            bool angle_ok = (abs_angle_err < 2.094f); /* 2π/3 = 120° */

            /* Confidence check (Phase 3) */
            bool conf_ok = (st->smo.confidence > 0.3f);

            if (pll_tracking && angle_ok && conf_ok) {
                st->handoff_ctr++;
                st->angle_ok_ctr++;
            } else {
                st->handoff_ctr = 0;
                st->angle_ok_ctr = 0;
            }

            if (st->handoff_ctr >= HANDOFF_DWELL_TICKS) {
                /* Transition to closed loop.
                 *
                 * With the AN1078 +90° phase shift in the SMO, the
                 * angle offset should be small (< 30°). Record any
                 * residual offset for compensation. The speed PI is
                 * held for CL_HOLDOFF_SLOW_TICKS to prevent braking
                 * from speed mismatch (handoff speed > throttle ref). */
                st->mode = V3_CLOSED_LOOP;
                st->cl_active = true;
                st->blend_ctr = 0;  /* repurposed as CL holdoff counter (slow loop) */

                /* Record angle offset (wrapped to ±π) */
                float off = st->smo.theta_est - st->theta_ol;
                if (off >  FOC_PI_F) off -= FOC_TWO_PI;
                if (off < -FOC_PI_F) off += FOC_TWO_PI;
                st->handoff_offset = off;

                /* Seed PLL with OL speed so speed feedback is correct
                 * immediately at CL entry (prevents speed PI mismatch). */
                st->pll.omega_est = st->omega_ol;
                st->pll.theta_est = st->smo.theta_est;

                /* Preload current PI integrators with expected steady-state
                 * voltages to prevent voltage transient at CL entry.
                 * V/f was applying Vq = VF_BOOST + Ke*ω, Vd = 0.
                 * Without preload, PI starts from 0 → voltage drops →
                 * SMO sees transient → PLL diverges → omega_ol runaway. */
                if (st->use_vf_startup) {
                    float vq_ss = VF_BOOST_V + st->Ke * st->omega_ol
                                + st->Rs * 0.3f;  /* match preloaded Iq */
                    foc_pi_preload(&st->pid_q, vq_ss);
                    foc_pi_preload(&st->pid_d, 0.0f);
                }

                /* Preload speed PI with modest Iq.
                 * ramp_iq=1.5A is too much for no-load — drives motor
                 * to voltage limit instantly. Use 0.3A: enough to keep
                 * the motor spinning during CL transition but won't
                 * cause overshoot. Speed PI adjusts from here. */
                foc_pi_preload(&st->pid_spd, 0.3f);
                st->iq_ref = 0.3f;
            }
        }

        st->sub_state = 3;  /* OL_RAMP */
        break;
    }

    case V3_CLOSED_LOOP: {
        /* Direct SMO angle commutation.
         *
         * Use the observer angle directly — no dead-reckoning.
         * Dead-reckon + PLL correction had a positive feedback loop:
         *   PLL noise → omega_ol drift → angle error → worse PLL.
         *
         * Phase compensation: the 2-stage cascaded LPF in the SMO
         * introduces a group delay that lags the angle estimate.
         * For a single IIR stage y[n] = (1-α)·y[n-1] + α·x[n]:
         *   τ_stage = (1 - α) / (α · fs)
         * Two cascaded stages: τ_lpf ≈ 2 · τ_stage.
         * Plus 1-sample computation delay (DT_FAST).
         *
         * The SMO uses adaptive alpha = base × |ω|/ω_ref, so we
         * must replicate that calculation here for correct comp. */

        /* Use PLL speed for phase comp (LP-filtered, less noisy) */
        float omega_est = st->pll.omega_est;
        if (omega_est < 0.0f) omega_est = 0.0f;

        /* Replicate SMO adaptive alpha calculation */
        float omega_ref = (float)STARTUP_HANDOFF_RAD_S;
        if (omega_ref < 100.0f) omega_ref = 100.0f;
        float alpha_scale = omega_est / omega_ref;
        if (alpha_scale < 0.1f) alpha_scale = 0.1f;
        float alpha_rt = st->smo.lpf_alpha * alpha_scale;
        if (alpha_rt < 0.02f) alpha_rt = 0.02f;
        if (alpha_rt > 0.95f) alpha_rt = 0.95f;

        /* Group delay: 2 cascaded IIR stages + 1 sample transport */
        float fs = 1.0f / DT_FAST;
        float tau_stage = (1.0f - alpha_rt) / (alpha_rt * fs);
        float tau_total = 2.0f * tau_stage + DT_FAST;

        float phase_comp = omega_est * tau_total;

        /* Direct SMO angle + phase lead compensation */
        theta_drive = st->smo.theta_est + phase_comp;
        theta_drive = foc_angle_wrap(theta_drive);

        /* Speed/current references from slow loop */
        iq_ref = st->iq_ref;
        id_ref = 0.0f;

        /* Track speed for telemetry (LP-filtered PLL) */
        st->omega += 0.01f * (omega_est - st->omega);

        st->sub_state = 4;  /* CL */
        break;
    }

    default:
        return;
    }

    /* ── Common FOC pipeline (all running modes) ─────────────── */

    /* 7. Park transform at drive angle */
    float sin_t = sinf(theta_drive);
    float cos_t = cosf(theta_drive);
    float id_meas, iq_meas;
    foc_park(i_alpha, i_beta, sin_t, cos_t, &id_meas, &iq_meas);

    /* Store for telemetry */
    st->id_meas = id_meas;
    st->iq_meas = iq_meas;
    st->theta   = theta_drive;
    st->theta_obs = st->smo.theta_est;
    st->omega_pll = st->pll.omega_est;

    /* 8. Voltage generation */
    float vd_tot, vq_tot;
    bool use_pi = (st->mode == V3_CLOSED_LOOP)
               || ((st->mode == V3_ALIGN || st->mode == V3_OL_RAMP)
                   && !st->use_vf_startup);

    if (use_pi) {
        /* PI current control — used for CL and I/f startup.
         * During ALIGN/OL_RAMP with I/f: PI drives current at forced angle.
         * The PI output IS the actual applied voltage → SMO sees accurate V. */

        /* During ALIGN: hold q-axis PI at zero. iq_ref=0 but noise causes
         * integrator windup. With circular clamp (q priority), Vq steals
         * voltage budget from Vd, starving the d-axis alignment current.
         * Fix: zero q PI integrator each tick → Vq stays ~0 → full Vd. */
        if (st->mode == V3_ALIGN) {
            foc_pi_preload(&st->pid_q, 0.0f);
        }

        float vd_pi = foc_pi_run(&st->pid_d, id_ref - id_meas, DT_FAST);
        float vq_pi = foc_pi_run(&st->pid_q, iq_ref - iq_meas, DT_FAST);

        vd_tot = vd_pi;
        vq_tot = vq_pi;

        /* Circular voltage clamp.
         * ALIGN: d-axis priority (need Vd for rotor lock-in).
         * Other modes: q-axis priority (torque production). */
        float vmax = st->vbus * 0.95f * FOC_INV_SQRT3;
        if (st->mode == V3_ALIGN) {
            /* D-axis priority: clamp Vd first, Vq gets remainder */
            vd_tot = foc_clampf(vd_tot, -vmax, vmax);
            float sq = vmax * vmax - vd_tot * vd_tot;
            if (sq < 0.0f) sq = 0.0f;
            float vq_lim = sqrtf(sq);
            vq_tot = foc_clampf(vq_tot, -vq_lim, vq_lim);
        } else {
            /* Q-axis priority: clamp Vq first, Vd gets remainder */
            vq_tot = foc_clampf(vq_tot, -vmax, vmax);
            float sq = vmax * vmax - vq_tot * vq_tot;
            if (sq < 0.0f) sq = 0.0f;
            float vd_lim = sqrtf(sq);
            vd_tot = foc_clampf(vd_tot, -vd_lim, vd_lim);
        }
    } else {
        /* V/f: direct voltage at forced angle (for low-Rs motors).
         * ALIGN: ramp 0→VF_BOOST. OL_RAMP: Vq = VF_BOOST + Ke×ω. */
        vd_tot = 0.0f;
        if (st->mode == V3_ALIGN) {
            float vfrac = (float)st->align_ctr / (float)st->align_ticks;
            if (vfrac > 1.0f) vfrac = 1.0f;
            vq_tot = VF_BOOST_V * vfrac;
        } else {
            vq_tot = VF_BOOST_V + st->Ke * st->omega_ol;
            float vq_max = st->vbus * 0.95f * FOC_INV_SQRT3;
            if (vq_tot > vq_max) vq_tot = vq_max;
        }
    }

    /* Store for telemetry and next-tick voltage reconstruction */
    st->vd = vd_tot;
    st->vq = vq_tot;

    /* 10. Inverse Park → αβ */
    float v_alpha_out, v_beta_out;
    foc_inv_park(vd_tot, vq_tot, sin_t, cos_t, &v_alpha_out, &v_beta_out);

    /* 11. SVPWM → duty cycles */
    foc_svpwm(v_alpha_out, v_beta_out, st->vbus, da_out, db_out, dc_out);

    /* Store actual duties for next-tick voltage reconstruction */
    st->da_prev = *da_out;
    st->db_prev = *db_out;
    st->dc_prev = *dc_out;

    /* ── Fast-loop protections ───────────────────────────────── */

    /* 12. Software overcurrent check */
    {
        float i_sq = id_meas * id_meas + iq_meas * iq_meas;
        float oc_sq = st->fault_oc_a * st->fault_oc_a;
        if (i_sq > oc_sq) {
            st->oc_debounce_ctr++;
            if (st->oc_debounce_ctr >= OC_DEBOUNCE_TICKS) {
                st->mode = V3_FAULT;
                st->fault_code = FAULT_FOC_INTERNAL;
                *da_out = 0.5f;
                *db_out = 0.5f;
                *dc_out = 0.5f;
            }
        } else {
            st->oc_debounce_ctr = 0;
        }
    }

    /* 13. Board fault pin check (U25B OC, active-low) */
    {
        static uint16_t board_fault_ctr = 0;
        if (BOARD_FAULT_PIN == BOARD_FAULT_ACTIVE) {
            board_fault_ctr++;
            if (board_fault_ctr >= BOARD_FAULT_DEBOUNCE) {
                st->mode = V3_FAULT;
                st->fault_code = FAULT_BOARD_PCI;
                *da_out = 0.5f;
                *db_out = 0.5f;
                *dc_out = 0.5f;
            }
        } else {
            board_fault_ctr = 0;
        }
    }

    /* ── Slow loop (every SLOW_DIV ticks → 1 kHz) ───────────── */
slow_loop:
    st->slow_div_ctr++;
    if (st->slow_div_ctr < SLOW_DIV) return;
    st->slow_div_ctr = 0;

    /* Arming logic: garuda_service.c ISR syncs ESC_ARMED → V3_ARMED.
     * Main loop (SW1 or GSP) sets garudaData.state = ESC_ARMED,
     * ISR picks it up at the top of the v3 block.
     * Do NOT auto-arm here — that bypasses the user's start command. */

    /* ARMED → ALIGN: hold zero throttle for FOC_ARM_TIME_MS, then start.
     * Same as v2 / 6-step: SW1 arms, arm counter verifies zero throttle,
     * then auto-transitions to ALIGN. */
    if (st->mode == V3_ARMED) {
        if (throttle <= THROTTLE_DEADBAND) {
            st->arm_ctr++;
            if (st->arm_ctr >= FOC_ARM_TIME_MS) {
                foc_v3_start(st);
            }
        } else {
            st->arm_ctr = 0;
        }
    }

    /* Closed-loop slow loop: ramp speed + speed PI for Iq.
     *
     * omega_ol ramps toward speed_ref (throttle command).
     * Speed PI compares speed_ref vs omega_ol (which tracks actual
     * motor speed via fast-loop PLL correction, alpha=0.002).
     *
     * Key insight: use omega_ol as speed feedback, NOT raw PLL.
     * omega_ol is already LP-filtered (PLL alpha=0.002 + ramp limiter)
     * so it's smooth even at high speed. Previous OC faults were caused
     * by using raw pll.omega_est which oscillates ±12% at 1300 rad/s.
     *
     * Under load: motor slows → PLL correction pulls omega_ol down →
     * speed_ref - omega_ol grows → PI increases Iq → more torque.
     * No-load at target: omega_ol ≈ speed_ref → small error → low Iq. */
    if (st->mode == V3_CLOSED_LOOP) {
        st->blend_ctr++;  /* CL ticks (slow loop) */

        if (st->blend_ctr <= CL_HOLDOFF_SLOW_TICKS) {
            /* Holdoff: keep ramp_iq, let PLL/SMO stabilize. */
        } else {
            /* Map throttle linearly to [handoff_speed, voltage_limit].
             * This uses the full pot range for the achievable speed range,
             * avoiding dead zones at top (voltage limit) or bottom. */
            float omega_max_v = 0.80f * st->vbus / (FOC_SQRT3 * st->Ke + 1e-9f);
            if (omega_max_v > st->max_elec_rad_s)
                omega_max_v = st->max_elec_rad_s;

            float speed_ref;
            if (throttle <= THROTTLE_DEADBAND) {
                speed_ref = st->handoff_rad_s;
            } else {
                float pot_frac = (float)(throttle - THROTTLE_DEADBAND)
                               / (4095.0f - (float)THROTTLE_DEADBAND);
                speed_ref = st->handoff_rad_s
                          + pot_frac * (omega_max_v - st->handoff_rad_s);
            }

            /* Speed PI: compare throttle-commanded speed vs actual.
             * Feedback = st->omega (LP-filtered PLL from fast loop).
             * With direct SMO angle commutation, omega_ol is unused
             * for commutation — speed control is purely via Iq. */
            float speed_err = speed_ref - st->omega;
            st->iq_ref = foc_pi_run(&st->pid_spd, speed_err, DT_SLOW);

            /* No Iq floor — speed PI handles it all including
             * negative Iq for active braking (regen limited to -1A).
             * At low speed, BEMF is small but SMO angle isn't used
             * critically since speed PI naturally reduces demand. */
        }

        /* Desync/stall detection: excessive |Id| indicates angle error.
         * When motor is in sync, the commutation angle aligns with the
         * rotor — all torque current flows in q-axis, Id ≈ 0.
         * When desynced, the angle is wrong and current spills into d-axis.
         * Data shows |Id| jumps from ±0.1A (normal) to 0.5-0.7A at desync.
         * Threshold: |Id| > 0.4A for 100ms → desync fault.
         * Only active after holdoff and above idle speed. */
        #define DESYNC_ID_THRESH   2.0f    /* |Id| > 2.0A → suspect desync */
        #define DESYNC_DEBOUNCE    200U    /* 200ms at 1kHz slow loop */
        {
            static uint16_t desync_ctr = 0;
            if (st->blend_ctr > CL_HOLDOFF_SLOW_TICKS
                && st->omega > st->handoff_rad_s * 1.5f)
            {
                float abs_id = st->id_meas;
                if (abs_id < 0.0f) abs_id = -abs_id;
                if (abs_id > DESYNC_ID_THRESH) {
                    desync_ctr++;
                    if (desync_ctr >= DESYNC_DEBOUNCE) {
                        st->mode = V3_FAULT;
                        st->fault_code = FAULT_DESYNC;
                        desync_ctr = 0;
                    }
                } else {
                    desync_ctr = 0;
                }
            } else {
                desync_ctr = 0;
            }
        }

        /* Low-speed CL exit: SMO has poor SNR below handoff speed.
         * If speed drops below handoff/2, stop immediately rather
         * than risking desync from noisy observer angle. */
        if (st->blend_ctr > CL_HOLDOFF_SLOW_TICKS
            && st->omega < st->handoff_rad_s * 0.5f)
        {
            foc_v3_stop(st);
            return;
        }

        /* Re-arm: throttle=0 and low speed for 1s → back to ARMED */
        static uint32_t rearm_ctr = 0;
        float abs_omega = st->omega;

        if (throttle <= THROTTLE_DEADBAND && abs_omega < 200.0f) {
            rearm_ctr++;
            if (rearm_ctr >= REARM_TIMEOUT_TICKS) {
                foc_v3_stop(st);
                rearm_ctr = 0;
            }
        } else {
            rearm_ctr = 0;
        }
    }
}
