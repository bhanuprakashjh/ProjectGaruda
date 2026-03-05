/**
 * @file foc_v2_control.c
 * @brief FOC v2 control loop — state machine, startup, closed-loop pipeline.
 *
 * State machine:
 *   IDLE → ARMED → ALIGN → IF_RAMP → CLOSED_LOOP → FAULT
 *                                          ↓ (throttle=0 >1s)
 *                                        ARMED (clean re-arm)
 *
 * Fast loop (24 kHz): Clarke → Observer → Park → PI_d/PI_q → decoupling
 *   → circular clamp → InvPark → SVPWM → duty output
 *
 * Slow loop (1 kHz): Speed PI → Iq_ref, arming, stall detect
 *
 * Component: FOC V2
 */

#include "foc_v2_control.h"
#include "foc_v2_math.h"
#include "foc_v2_pi.h"
#include "foc_v2_observer.h"
#include "../garuda_foc_params.h"

/* ── Constants from garuda_foc_params.h ──────────────────────── */
#define DT_FAST          FOC_TS_FAST_S
#define DT_SLOW          FOC_TS_SLOW_S
#define SLOW_DIV         FOC_SLOW_DIV

/* ADC calibration */
#define CAL_SAMPLES      1024U
#define CAL_SHIFT         10U

/* Overcurrent debounce (2ms at 24kHz) */
#define OC_DEBOUNCE_TICKS 48U

/* Handoff speed tolerance: PLL ω within 20% of forced ω.
 * Speed-based handoff is immune to load angle (prop drag causes
 * 20-30° rotor lag in I/f mode — angle comparison always fails). */
#define HANDOFF_SPEED_TOL  0.20f

/* Lock count for handoff (20ms at 24kHz) */
#define HANDOFF_LOCK_TICKS 480U

/* Re-arm: throttle=0 for 1s in slow loop ticks */
#define REARM_TIMEOUT_TICKS 1000U

/* ── Helpers ─────────────────────────────────────────────────── */

static inline float raw_to_amps(uint16_t raw, float offset)
{
    /* Negate: OA1/OA2 inverting topology on MCLV-48V-300W DIM */
    return -((float)raw - offset) * CURRENT_SCALE_A_PER_COUNT;
}

static inline float raw_to_vbus(uint16_t raw)
{
    return (float)raw * VBUS_SCALE_V_PER_COUNT;
}

static void reset_startup(FOC_State_t *foc)
{
    foc->omega_if    = 0.0f;
    foc->theta_if    = 0.0f;
    foc->align_ctr   = 0;
    foc->ramp_ctr    = 0;
    foc->lock_ctr    = 0;
    foc->cl_active   = false;
    foc->iq_ref      = 0.0f;
    foc->id_ref      = 0.0f;
    foc->vd          = 0.0f;
    foc->vq          = 0.0f;
    foc->oc_debounce_ctr = 0;
    foc->stall_ctr   = 0;
    foc->sub_state   = 0;

    foc_pi_reset(&foc->pid_d);
    foc_pi_reset(&foc->pid_q);
    foc_pi_reset(&foc->pid_spd);
    foc_observer_reset(&foc->obs);
    foc_pll_reset(&foc->pll);
}

/* ── Public API ──────────────────────────────────────────────── */

void foc_v2_init(FOC_State_t *foc, const FOC_MotorParams_t *params)
{
    /* Zero entire state */
    FOC_State_t zero = {0};
    *foc = zero;

    foc->mode = FOC_IDLE;

    /* Copy motor params */
    foc->Rs        = params->Rs;
    foc->Ls        = params->Ls;
    foc->lambda_pm = params->lambda_pm;
    foc->Ke        = params->Ke;

    /* Dead-time compensation: Vbus * (t_dead / T_pwm) */
    float t_dead_s = (float)DEADTIME_NS * 1e-9f;
    float t_pwm_s  = 1.0f / (float)PWMFREQUENCY_HZ;
    foc->dt_comp   = params->vbus_nom_v * (t_dead_s / t_pwm_s);

    /* Init PI controllers */
    float vclamp = params->vbus_nom_v * 0.95f * FOC_INV_SQRT3;
    foc_pi_init(&foc->pid_d, KP_DQ, KI_DQ, -vclamp, vclamp);
    foc_pi_init(&foc->pid_q, KP_DQ, KI_DQ, -vclamp, vclamp);
    foc_pi_init(&foc->pid_spd, KP_SPD, KI_SPD,
                -params->max_current_a, params->max_current_a);

    /* Init PLL — 200 Hz BW for A2212 (max ~955 Hz electrical).
     * 50 Hz was too low: phase lag at high speed caused observer→Park
     * feedback instability. 200 Hz gives <15° lag at 6000 rad/s. */
    foc_pll_init(&foc->pll, 200.0f);

    /* Init observer */
    foc_observer_reset(&foc->obs);
    foc->obs.lambda_est = params->lambda_pm;

    /* ADC calibration */
    foc->ia_offset = (float)ADC_MIDPOINT;
    foc->ib_offset = (float)ADC_MIDPOINT;
    foc->cal_accum_a = 0;
    foc->cal_accum_b = 0;
    foc->cal_count = 0;
    foc->cal_done  = false;

    foc->vbus = params->vbus_nom_v;
    foc->slow_div_ctr = 0;
    foc->arm_ctr = 0;
}

void foc_v2_start(FOC_State_t *foc)
{
    if (foc->mode == FOC_ARMED) {
        reset_startup(foc);
        foc->mode = FOC_ALIGN;
    }
}

void foc_v2_stop(FOC_State_t *foc)
{
    reset_startup(foc);
    foc->mode = FOC_ARMED;
    foc->arm_ctr = 0;
}

void foc_v2_fault(FOC_State_t *foc, uint16_t fault_code)
{
    foc->mode = FOC_FAULT;
    (void)fault_code;  /* Fault code stored in garudaData.faultCode by caller */
}

/* ── Fast Loop ───────────────────────────────────────────────── */

void foc_v2_fast_tick(FOC_State_t *foc,
                      uint16_t ia_raw, uint16_t ib_raw,
                      uint16_t vbus_raw, uint16_t throttle,
                      float *da_out, float *db_out, float *dc_out)
{
    /* Default: center duty (safe) */
    *da_out = 0.5f;
    *db_out = 0.5f;
    *dc_out = 0.5f;

    /* Vbus conversion */
    foc->vbus = raw_to_vbus(vbus_raw);

    /* ── IDLE / ARMED: ADC offset calibration, no motor drive ── */
    if (foc->mode == FOC_IDLE || foc->mode == FOC_ARMED) {
        /* ADC zero-current offset calibration (motor off → no current) */
        if (!foc->cal_done) {
            foc->cal_accum_a += ia_raw;
            foc->cal_accum_b += ib_raw;
            if (++foc->cal_count >= CAL_SAMPLES) {
                foc->ia_offset = (float)(foc->cal_accum_a >> CAL_SHIFT);
                foc->ib_offset = (float)(foc->cal_accum_b >> CAL_SHIFT);
                foc->cal_done  = true;
            }
        }
        goto slow_loop;
    }

    /* ── FAULT: outputs disabled ─────────────────────────────── */
    if (foc->mode == FOC_FAULT) {
        goto slow_loop;
    }

    /* ── Active modes: ALIGN, IF_RAMP, CLOSED_LOOP ──────────── */
    {
        /* Convert ADC to physical units */
        float ia = raw_to_amps(ia_raw, foc->ia_offset);
        float ib = raw_to_amps(ib_raw, foc->ib_offset);

        /* Dynamic PI voltage clamp — tracks Vbus.
         * Reduce Q-axis PI budget by estimated BEMF feedforward so the
         * integrator can't wind up against the voltage ceiling. */
        float vclamp = foc->vbus * 0.95f * FOC_INV_SQRT3;
        float omega_est = foc->cl_active ? foc->pll.omega_est : foc->omega_if;
        float bemf_ff = omega_est * foc->Ke;
        float vq_pi_budget = vclamp - bemf_ff;
        if (vq_pi_budget < 0.5f) vq_pi_budget = 0.5f;  /* min 0.5V for PI */
        foc->pid_d.out_max =  vclamp;
        foc->pid_d.out_min = -vclamp;
        foc->pid_q.out_max =  vq_pi_budget;
        foc->pid_q.out_min = -vclamp;  /* reverse: BEMF helps, full budget */

        /* 1. Clarke transform */
        float i_alpha, i_beta;
        foc_clarke(ia, ib, &i_alpha, &i_beta);

        /* 2. Observer update (uses previous tick's applied voltage) */
        if (foc->mode == FOC_CLOSED_LOOP || foc->mode == FOC_IF_RAMP) {
            /* Use reconstructed applied voltage for observer */
            float sin_t = sinf(foc->theta);
            float cos_t = cosf(foc->theta);
            float v_alpha, v_beta;
            foc_inv_park(foc->vd, foc->vq, sin_t, cos_t, &v_alpha, &v_beta);

            /* Approximate duty for observer gain scheduling:
             * |V| / Vbus gives effective modulation index. */
            float v_mag = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
            float duty_abs = (foc->vbus > 1.0f) ? (v_mag / foc->vbus) : 0.05f;

            foc_observer_update(&foc->obs, &foc->pll,
                                v_alpha, v_beta,
                                i_alpha, i_beta,
                                foc->Rs, foc->Ls, foc->lambda_pm,
                                foc->dt_comp, duty_abs,
                                DT_FAST);
        }

        /* 3. Select commutation angle based on mode */
        float theta_drive;
        float id_ref = 0.0f;
        float iq_ref;

        switch (foc->mode) {
        case FOC_ALIGN: {
            /* Fixed angle θ=0, ramp Id from 0 to ALIGN_IQ.
             * MUST use Id (not Iq) for alignment!  At θ=0:
             *   Id → field along α-axis → rotor PM aligns at θ=0 ✓
             *   Iq → field along β-axis → rotor PM aligns at θ=90° ✗
             * Using Iq puts rotor 90° from expected angle — I/f ramp
             * then has zero real torque and loses sync under any load. */
            foc->align_ctr++;
            theta_drive = 0.0f;

            float align_frac = (float)foc->align_ctr / (float)STARTUP_ALIGN_TICKS;
            if (align_frac > 1.0f) align_frac = 1.0f;
            id_ref = STARTUP_ALIGN_IQ_A * align_frac;
            iq_ref = 0.0f;

            /* Transition: alignment complete → IF_RAMP */
            if (foc->align_ctr >= STARTUP_ALIGN_TICKS) {
                /* Seed observer: rotor aligned at θ=0 */
                foc_observer_seed(&foc->obs, foc->lambda_pm, 0.0f);
                foc->pll.theta_est = 0.0f;
                foc->pll.omega_est = 0.0f;

                foc->theta_if = 0.0f;
                foc->omega_if = 0.0f;
                foc->ramp_ctr = 0;
                foc->mode = FOC_IF_RAMP;
            }
            break;
        }

        case FOC_IF_RAMP: {
            /* I/f open-loop: forced angle + PI current control */
            foc->ramp_ctr++;

            /* Pot → speed target */
            float pot_target;
            if (throttle < THROTTLE_DEADBAND) {
                pot_target = 0.0f;
            } else {
                float pot_frac = (float)(throttle - THROTTLE_DEADBAND)
                               / (4095.0f - (float)THROTTLE_DEADBAND);
                pot_target = pot_frac * STARTUP_MAX_OL_RAD_S;
            }

            /* Slew-rate limited speed ramp */
            float max_delta = STARTUP_RAMP_RATE_RPS2 * DT_FAST;
            if (foc->omega_if < pot_target) {
                foc->omega_if += max_delta;
                if (foc->omega_if > pot_target) foc->omega_if = pot_target;
            } else if (foc->omega_if > pot_target) {
                foc->omega_if -= max_delta;
                if (foc->omega_if < pot_target) foc->omega_if = pot_target;
            }

            /* Advance forced angle */
            foc->theta_if += foc->omega_if * DT_FAST;
            foc->theta_if = foc_angle_wrap(foc->theta_if);
            theta_drive = foc->theta_if;

            /* I/f current: ramp from ALIGN_IQ to RAMP_IQ */
            if (foc->ramp_ctr < STARTUP_IQ_RAMP_TICKS) {
                float frac = (float)foc->ramp_ctr / (float)STARTUP_IQ_RAMP_TICKS;
                iq_ref = STARTUP_ALIGN_IQ_A
                       + (STARTUP_RAMP_IQ_A - STARTUP_ALIGN_IQ_A) * frac;
            } else {
                iq_ref = STARTUP_RAMP_IQ_A;
            }

            /* Handoff check: PLL speed matches forced speed.
             * Speed-based (not angle-based) — immune to load angle from
             * prop drag, which causes 20-30° rotor lag in I/f mode. */
            if (foc->omega_if >= STARTUP_HANDOFF_RAD_S) {
                float spd_err = foc->pll.omega_est - foc->omega_if;
                if (spd_err < 0.0f) spd_err = -spd_err;
                float spd_tol = HANDOFF_SPEED_TOL * foc->omega_if;

                if (spd_err < spd_tol) {
                    if (++foc->lock_ctr >= HANDOFF_LOCK_TICKS) {
                        /* Bumpless transfer: preload PI with steady-state values.
                         * Feedforward (Ke*ω) is now added outside PI, so PI only
                         * needs the resistive drop Rs*Iq for smooth handoff. */
                        foc_pi_preload(&foc->pid_q, foc->Rs * iq_ref);
                        foc_pi_preload(&foc->pid_d, 0.0f);
                        foc_pi_preload(&foc->pid_spd, iq_ref);

                        foc->cl_active = true;
                        foc->mode = FOC_CLOSED_LOOP;
                    }
                } else {
                    foc->lock_ctr = 0;
                }
            }
            break;
        }

        case FOC_CLOSED_LOOP: {
            /* PLL angle for commutation — smoother than raw atan2.
             * Raw observer atan2 has derivative noise from flux integrator;
             * PLL is 2nd-order filtered. */
            theta_drive = foc->pll.theta_est;
            float cl_omega = foc->pll.omega_est;

            /* Speed reference from pot (handled in slow loop via pid_spd) */
            iq_ref = foc->iq_ref;  /* Set by slow loop */

            /* Track speed for telemetry */
            foc->omega = cl_omega;
            break;
        }

        default:
            theta_drive = 0.0f;
            iq_ref = 0.0f;
            break;
        }

        /* 4. Park transform */
        float sin_t = sinf(theta_drive);
        float cos_t = cosf(theta_drive);
        float id_meas, iq_meas;
        foc_park(i_alpha, i_beta, sin_t, cos_t, &id_meas, &iq_meas);

        /* 5-6. PI controllers */
        float vd_pi = foc_pi_run(&foc->pid_d, id_ref - id_meas, DT_FAST);
        float vq_pi = foc_pi_run(&foc->pid_q, iq_ref - iq_meas, DT_FAST);

        /* 7. Cross-coupling decoupling + BEMF feedforward */
        float omega_ff = foc->cl_active ? foc->pll.omega_est : foc->omega_if;
        float ff_d = -omega_ff * foc->Ls * iq_meas;
        float ff_q =  omega_ff * foc->Ls * id_meas + omega_ff * foc->Ke;
        float vd_tot = vd_pi + ff_d;
        float vq_tot = vq_pi + ff_q;

        /* 8. Circular voltage clamp (d-axis priority) */
        float vmax = foc->vbus * 0.95f * FOC_INV_SQRT3;
        vd_tot = foc_clampf(vd_tot, -vmax, vmax);
        float sq = vmax * vmax - vd_tot * vd_tot;
        if (sq < 0.0f) sq = 0.0f;
        float vq_lim = sqrtf(sq);
        vq_tot = foc_clampf(vq_tot, -vq_lim, vq_lim);

        /* Back-calculate effective PI output for anti-windup:
         * If clamp reduced total, feed the clamped PI value back so
         * the integrator doesn't wind up against the voltage ceiling. */
        vd_pi = vd_tot - ff_d;
        vq_pi = vq_tot - ff_q;

        /* Save total voltage for observer on next tick */
        foc->vd = vd_tot;
        foc->vq = vq_tot;

        /* 9. Inverse Park (total voltage = PI + feedforward, clamped) */
        float v_alpha, v_beta;
        foc_inv_park(vd_tot, vq_tot, sin_t, cos_t, &v_alpha, &v_beta);

        /* 10-11. SVPWM */
        foc_svpwm(v_alpha, v_beta, foc->vbus, da_out, db_out, dc_out);

        /* 12. Update commutation angle for next tick */
        foc->theta = theta_drive;

        /* 13. Telemetry */
        foc->id_meas   = id_meas;
        foc->iq_meas   = iq_meas;
        foc->theta_obs = foc->obs.theta_est;
        foc->omega_pll = foc->pll.omega_est;
        foc->sub_state = (foc->mode == FOC_ALIGN)       ? 2 :
                         (foc->mode == FOC_IF_RAMP)      ? 3 :
                         (foc->mode == FOC_CLOSED_LOOP)  ? 4 : 0;

        /* 14. Software overcurrent check */
        {
            float i_sq = id_meas * id_meas + iq_meas * iq_meas;
            if (i_sq > FAULT_OC_A * FAULT_OC_A) {
                if (++foc->oc_debounce_ctr >= OC_DEBOUNCE_TICKS) {
                    foc->mode = FOC_FAULT;
                    foc->oc_debounce_ctr = 0;
                    *da_out = 0.5f;
                    *db_out = 0.5f;
                    *dc_out = 0.5f;
                }
            } else {
                foc->oc_debounce_ctr = 0;
            }
        }

        /* 15. Stall detection (CL only) */
        if (foc->mode == FOC_CLOSED_LOOP &&
            throttle >= THROTTLE_DEADBAND &&
            foc->iq_ref > 0.1f) {
            float abs_omega = foc->pll.omega_est;
            if (abs_omega < 0.0f) abs_omega = -abs_omega;
            if (abs_omega < FAULT_STALL_RAD_S) {
                foc->stall_ctr++;
                if (foc->stall_ctr >= (uint32_t)(FAULT_STALL_TIMEOUT_MS * SLOW_DIV)) {
                    foc->mode = FOC_FAULT;
                    foc->stall_ctr = 0;
                    *da_out = 0.5f;
                    *db_out = 0.5f;
                    *dc_out = 0.5f;
                }
            } else {
                foc->stall_ctr = 0;
            }
        }
    }

    /* ── Slow loop (every SLOW_DIV ticks → 1 kHz) ───────────── */
slow_loop:
    if (++foc->slow_div_ctr >= SLOW_DIV) {
        foc->slow_div_ctr = 0;

        if (foc->mode == FOC_ARMED) {
            /* Arming: throttle at zero for ARM_TIME_MS → ALIGN */
            if (throttle < THROTTLE_DEADBAND) {
                if (++foc->arm_ctr >= (uint32_t)FOC_ARM_TIME_MS) {
                    reset_startup(foc);
                    foc->mode = FOC_ALIGN;
                }
            } else {
                foc->arm_ctr = 0;
            }
        }
        else if (foc->mode == FOC_CLOSED_LOOP) {
            /* Speed PI: pot → speed ref → Iq ref */
            float speed_ref;
            if (throttle < THROTTLE_DEADBAND) {
                speed_ref = 0.0f;
            } else {
                float pot_frac = (float)(throttle - THROTTLE_DEADBAND)
                               / (4095.0f - (float)THROTTLE_DEADBAND);
                speed_ref = pot_frac * MOTOR_MAX_ELEC_RAD_S;
            }

            /* Voltage-limited speed cap: prevent commanding speeds that
             * require more BEMF voltage than the bus can deliver.
             * Reserve 15% of Vq budget for current control headroom. */
            float vq_avail = foc->vbus * 0.85f * FOC_INV_SQRT3;
            float omega_max_v = vq_avail / foc->Ke;
            if (speed_ref > omega_max_v)
                speed_ref = omega_max_v;

            foc->iq_ref = foc_pi_run(&foc->pid_spd,
                                      speed_ref - foc->pll.omega_est,
                                      DT_SLOW);

            /* Re-arm: throttle=0 and low speed for 1s → back to ARMED */
            static uint32_t rearm_ctr = 0;
            float abs_omega = foc->pll.omega_est;
            if (abs_omega < 0.0f) abs_omega = -abs_omega;
            if (throttle < THROTTLE_DEADBAND && abs_omega < FAULT_STALL_RAD_S * 2.0f) {
                if (++rearm_ctr >= REARM_TIMEOUT_TICKS) {
                    foc_v2_stop(foc);
                    rearm_ctr = 0;
                }
            } else {
                rearm_ctr = 0;
            }
        }
    }
}
