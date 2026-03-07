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
#include "foc_v2_detect.h"
#include "../garuda_foc_params.h"

/* ── Constants from garuda_foc_params.h ──────────────────────── */
#define DT_FAST          FOC_TS_FAST_S
#define DT_SLOW          FOC_TS_SLOW_S
#define SLOW_DIV         FOC_SLOW_DIV

/* Dead-time compensation disabled — see note at line ~460.
 * Microchip AN1292 reference achieves clean waveforms without it. */

/* ADC calibration */
#define CAL_SAMPLES      1024U
#define CAL_SHIFT         10U

/* Overcurrent debounce (2ms at 24kHz) */
#define OC_DEBOUNCE_TICKS 48U

/* Handoff speed tolerance: PLL ω within 20% of forced ω.
 * Speed-based handoff is immune to load angle (prop drag causes
 * 20-30° rotor lag in I/f mode — angle comparison always fails). */
#define HANDOFF_SPEED_TOL  0.20f

/* Lock count for handoff (10ms at 24kHz — reduced for faster CL entry) */
#define HANDOFF_LOCK_TICKS 240U

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

    foc->dt_comp   = 0.0f;  /* DT comp disabled */

    /* Copy runtime startup/tuning params */
    foc->align_iq        = params->align_iq_a;
    foc->ramp_iq         = params->ramp_iq_a;
    foc->align_ticks     = params->align_ticks;
    foc->iq_ramp_ticks   = params->iq_ramp_ticks;
    foc->ramp_rate       = params->ramp_rate_rps2;
    foc->handoff_rad_s   = params->handoff_rad_s;
    foc->fault_oc_a      = params->fault_oc_a;
    foc->fault_stall_rad_s = params->fault_stall_rad_s;
    foc->obs_lpf_alpha   = params->obs_lpf_alpha;
    foc->max_elec_rad_s  = params->max_elec_rad_s;

    /* Init PI controllers */
    float vclamp = params->vbus_nom_v * 0.95f * FOC_INV_SQRT3;
    foc_pi_init(&foc->pid_d, params->kp_dq, params->ki_dq, -vclamp, vclamp);
    foc_pi_init(&foc->pid_q, params->kp_dq, params->ki_dq, -vclamp, vclamp);
    foc_pi_init(&foc->pid_spd, KP_SPD, KI_SPD,
                -params->max_current_a, params->max_current_a);

    /* Init PLL — 100 Hz BW, used for speed estimation only.
     * Observer atan2 angle drives commutation (zero lag).
     * PLL smooths speed for the speed PI and handoff check. */
    foc_pll_init(&foc->pll, 100.0f);
    foc->pll.omega_max = params->max_elec_rad_s;

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

    /* ── MOTOR_DETECT: auto-commissioning ────────────────────── */
    if (foc->mode == FOC_MOTOR_DETECT) {
        float ia = raw_to_amps(ia_raw, foc->ia_offset);
        float ib = raw_to_amps(ib_raw, foc->ib_offset);
        float i_alpha, i_beta;
        foc_clarke(ia, ib, &i_alpha, &i_beta);

        bool done = foc_detect_fast_tick(foc, ia, ib, i_alpha, i_beta,
                                         da_out, db_out, dc_out);
        if (done) {
            if (foc->detect_state == DETECT_DONE) {
                /* Apply measured params and transition to ARMED */
                foc_detect_apply(foc);
                reset_startup(foc);
                foc->mode = FOC_ARMED;
                foc->arm_ctr = 0;
            } else {
                /* Detection failed */
                foc->mode = FOC_FAULT;
            }
            *da_out = 0.5f;
            *db_out = 0.5f;
            *dc_out = 0.5f;
        }
        goto slow_loop;
    }

    /* ── Active modes: ALIGN, IF_RAMP, CLOSED_LOOP ──────────── */
    {
        /* Convert ADC to physical units */
        float ia = raw_to_amps(ia_raw, foc->ia_offset);
        float ib = raw_to_amps(ib_raw, foc->ib_offset);

        /* Dynamic PI voltage clamp — tracks Vbus.
         * No feedforward → PI gets full symmetric voltage budget.
         * PI will naturally produce BEMF voltage + correction. */
        float vclamp = foc->vbus * 0.95f * FOC_INV_SQRT3;
        foc->pid_d.out_max =  vclamp;
        foc->pid_d.out_min = -vclamp;
        foc->pid_q.out_max =  vclamp;
        foc->pid_q.out_min = -vclamp;

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
                                0.0f, duty_abs,  /* DT comp applied on output duties, not observer */
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

            float align_frac = (float)foc->align_ctr / (float)foc->align_ticks;
            if (align_frac > 1.0f) align_frac = 1.0f;
            id_ref = foc->align_iq * align_frac;
            iq_ref = 0.0f;

            /* Transition: alignment complete → IF_RAMP */
            if (foc->align_ctr >= foc->align_ticks) {
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
            /* I/f open-loop: forced angle + PI current control.
             * Ramp speed is POT-INDEPENDENT — always accelerate to handoff
             * speed as fast as possible, then CL speed PI takes over.
             * This minimizes time in I/f (high fixed current) and avoids
             * humming at pot=0 during ramp. */
            foc->ramp_ctr++;

            /* Always ramp toward handoff speed at max rate */
            float max_delta = foc->ramp_rate * DT_FAST;
            foc->omega_if += max_delta;
            if (foc->omega_if > foc->handoff_rad_s * 1.1f)
                foc->omega_if = foc->handoff_rad_s * 1.1f;

            /* Advance forced angle */
            foc->theta_if += foc->omega_if * DT_FAST;
            foc->theta_if = foc_angle_wrap(foc->theta_if);
            theta_drive = foc->theta_if;

            /* Let observer free-run during I/f ramp.  It was seeded at
             * the end of ALIGN (line 304) and now tracks independently.
             * The handoff check below compares the observer's independent
             * estimate against the forced angle — only fires when the
             * observer has genuinely converged.  Previous code anchored
             * the observer here, making the handoff check trivially true
             * and causing immediate angle drift at CL entry. */

            /* Smooth d→q current transition during I/f ramp.
             * At ALIGN end: Id=align_iq, Iq=0.  Jumping to Id=0, Iq=X
             * in one tick creates a 90° torque impulse → loud thump.
             * Instead, linearly fade Id→0 while ramping Iq→ramp_iq
             * over iq_ramp_ticks, giving a smooth current vector rotation. */
            if (foc->ramp_ctr < foc->iq_ramp_ticks) {
                float frac = (float)foc->ramp_ctr / (float)foc->iq_ramp_ticks;
                iq_ref = foc->ramp_iq * frac;
                id_ref = foc->align_iq * (1.0f - frac);
            } else {
                iq_ref = foc->ramp_iq;
            }

            /* Handoff: Microchip AN1292 approach — transition when forced
             * speed reaches handoff threshold and observer sees rotation.
             * Frame transform handles any angle difference at transition.
             * Previous complex speed+angle matching failed because Rs
             * mismatch (model vs actual) causes continuous observer drift. */
            if (foc->omega_if >= foc->handoff_rad_s) {
                /* Simple check: PLL sees positive rotation above 50% of
                 * handoff speed.  This confirms the observer is alive and
                 * tracking in the correct direction.  The frame transform
                 * and speed PI handle any residual mismatch. */
                float pll_spd = foc->pll.omega_est;
                bool obs_tracking = (pll_spd > foc->handoff_rad_s * 0.5f);

                if (obs_tracking) {
                    if (++foc->lock_ctr >= HANDOFF_LOCK_TICKS) {
                        /* Bumpless transfer: transform PI integrator state
                         * from OL frame (theta_if) to CL frame (obs.theta_est).
                         * Rotates voltage vector to new d-q frame so PI
                         * integrators don't see a discontinuity. */
                        float delta_th = foc->theta_if - foc->obs.theta_est;
                        float cos_d = cosf(delta_th);
                        float sin_d = sinf(delta_th);
                        float vd_new =  foc->vd * cos_d + foc->vq * sin_d;
                        float vq_new = -foc->vd * sin_d + foc->vq * cos_d;
                        foc_pi_preload(&foc->pid_d, vd_new);
                        foc_pi_preload(&foc->pid_q, vq_new);

                        /* Speed PI preload with measured Iq */
                        float iq_actual = foc->iq_meas > 0.0f ? foc->iq_meas : 0.0f;
                        foc_pi_preload(&foc->pid_spd, iq_actual);

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
            /* Observer atan2 angle for commutation — zero phase lag.
             * PLL has phase lead at high electrical frequency (200Hz BW
             * overshoots at 933Hz/8kRPM → 16° lead → desync).
             * PLL is used only for smooth speed estimation (speed PI). */
            theta_drive = foc->obs.theta_est;
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

        /* 7. No feedforward — let PI controllers handle everything.
         * Feedforward (BEMF + cross-coupling) amplifies parameter errors
         * into systematic voltage distortion.  Microchip AN1292 reference
         * uses no feedforward and achieves clean sinusoidal currents.
         * Can be re-enabled once motor params are well-characterized. */
        float vd_tot = vd_pi;
        float vq_tot = vq_pi;

        /* 8. Circular voltage clamp (d-axis priority) */
        float vmax = foc->vbus * 0.95f * FOC_INV_SQRT3;
        vd_tot = foc_clampf(vd_tot, -vmax, vmax);
        float sq = vmax * vmax - vd_tot * vd_tot;
        if (sq < 0.0f) sq = 0.0f;
        float vq_lim = sqrtf(sq);
        vq_tot = foc_clampf(vq_tot, -vq_lim, vq_lim);

        /* NOTE: Current PI anti-windup relies on the dynamic PI limits
         * set at lines 215-224 (vq_pi_budget tracks BEMF feedforward).
         * The circular voltage clamp may occasionally clip beyond these
         * limits, but the PI's internal Tustin anti-windup prevents
         * severe accumulation.  Speed PI windup (the main issue at the
         * voltage ceiling) is handled by the dynamic speed PI clamp
         * in the slow loop. */

        /* Save total voltage for observer on next tick */
        foc->vd = vd_tot;
        foc->vq = vq_tot;

        /* 9. Inverse Park (total voltage = PI + feedforward, clamped) */
        float v_alpha, v_beta;
        foc_inv_park(vd_tot, vq_tot, sin_t, cos_t, &v_alpha, &v_beta);

        /* 10-11. SVPWM */
        foc_svpwm(v_alpha, v_beta, foc->vbus, da_out, db_out, dc_out);

        /* Dead-time compensation disabled — Microchip AN1292 reference
         * does not use software DT comp and achieves clean waveforms.
         * DT comp (1.8% duty = 0.43V) was larger than Rs*I at low current
         * and any sign error created more distortion than it fixed. */

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
            if (i_sq > foc->fault_oc_a * foc->fault_oc_a) {
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
            if (abs_omega < foc->fault_stall_rad_s) {
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
            /* Speed PI: pot → speed ref → Iq ref.
             * Minimum idle speed = handoff speed to keep observer tracking.
             * Below this speed, BEMF is too weak for reliable sensorless. */
            float idle_speed = foc->handoff_rad_s;
            float speed_ref;
            if (throttle < THROTTLE_DEADBAND) {
                speed_ref = idle_speed;
            } else {
                float pot_frac = (float)(throttle - THROTTLE_DEADBAND)
                               / (4095.0f - (float)THROTTLE_DEADBAND);
                speed_ref = idle_speed + pot_frac * (foc->max_elec_rad_s - idle_speed);
            }

            /* Voltage-limited speed cap: prevent commanding speeds that
             * require more BEMF voltage than the bus can deliver.
             * Reserve 20% of Vq budget for current control headroom. */
            float vq_avail = foc->vbus * 0.80f * FOC_INV_SQRT3;
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
            if (throttle < THROTTLE_DEADBAND && abs_omega < foc->fault_stall_rad_s * 2.0f) {
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
