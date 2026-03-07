/**
 * @file foc_v2_detect.c
 * @brief Motor auto-commissioning: Rs, Ls, λ_pm measurement + PI auto-tune.
 *
 * Runs inside the 24 kHz fast loop using the existing PI + SVPWM pipeline.
 *
 * Phase 1 — Rs (resistance):
 *   Command Id=TEST_CURRENT at θ=0, PI drives Vd to maintain it.
 *   After settling, average Vd/Id over SAMPLE window → Rs.
 *
 * Phase 2 — Ls (inductance):
 *   At θ=0, apply a voltage step (bypass PI, direct Vd command).
 *   Measure initial dI/dt: L = V_step / (dI/dt).
 *   Multiple pulses averaged for noise rejection.
 *
 * Phase 2.5 — Re-align:
 *   Re-apply Id at θ=0 to lock rotor (may have drifted during Ls pulses).
 *   Pure Id only — d→q transition happens in Phase 3 while θ advances.
 *
 * Phase 3 — λ_pm (flux linkage):
 *   Spin motor with I/f at known ω, PI holds Id=0, Iq=test current.
 *   At steady state: Vq = Rs*Iq + ω*λ_pm  →  λ_pm = (Vq - Rs*Iq) / ω.
 *
 * Phase 4 — Auto-tune:
 *   Kp_dq = ωbw * Ls,  Ki_dq = ωbw * Rs  (1 kHz BW, pole cancellation).
 *
 * Component: FOC V2
 */

#include "foc_v2_detect.h"
#include "foc_v2_math.h"
#include "foc_v2_pi.h"
#include "../garuda_foc_params.h"

/* ── Detect configuration ─────────────────────────────────────── */

/* Test current for Rs measurement (A).
 * Must be enough to get measurable voltage but not trip U25B.
 * 0.5A is conservative — U25B on MCLV-48V-300W has no LEB and trips
 * on SVM switching transients at higher currents. */
#define DETECT_TEST_CURRENT_A   0.5f

/* Rs timing (24 kHz ticks) */
#define RS_RAMP_TICKS       12000U  /* 500ms: very slow ramp to avoid transient */
#define RS_SETTLE_TICKS     4800U   /* 200ms: let PI settle */
#define RS_SAMPLE_TICKS     12000U  /* 500ms: accumulate Vd/Id */

/* Ls timing */
#define LS_SETTLE_TICKS     2400U   /* 100ms: PI holds Id=0 first */
#define LS_STEP_TICKS       6U      /* 0.25ms: early inductive region only */
#define LS_COAST_TICKS      960U    /* 40ms: let current fully decay + U25B recover */
#define LS_NUM_PULSES       10U     /* Average over N pulses */
#define LS_TARGET_PEAK_A    0.5f    /* Target peak current during step */

/* Re-alignment between Ls and Lambda phases */
#define REALIGN_TICKS       4800U   /* 200ms: lock rotor at θ=0 before spinning */

/* d→q transition at start of Lambda (WHILE θ is advancing).
 * Must NOT happen at static θ — commanding Iq at θ=0 creates a static
 * torque that pushes the rotor ~90° ahead of the reference angle,
 * causing all BEMF to appear in Vd instead of Vq → λ_measured ≈ 0. */
#define LAMBDA_DQ_TICKS     2880U   /* 120ms: smooth d→q while spinning */

/* Lambda timing */
#define LAMBDA_RAMP_TICKS   36000U  /* 1.5s: I/f spin-up (150/100=1.5s to target) */
#define LAMBDA_SETTLE_TICKS 24000U  /* 1s: speed settle (longer for stability) */
#define LAMBDA_SAMPLE_TICKS 24000U  /* 1s: accumulate Vq/Iq/ω */
#define LAMBDA_TEST_SPEED   150.0f  /* rad/s elec: conservative speed for sync margin */
#define LAMBDA_RAMP_RATE    100.0f  /* rad/s²: gentle acceleration */
#define LAMBDA_IQ_A         1.0f    /* Iq during spin — higher for reliable rotor sync */

/* Auto-tune */
#define TUNE_CURRENT_BW_HZ  1000.0f
#define TUNE_CURRENT_BW_RAD (2.0f * FOC_PI_F * TUNE_CURRENT_BW_HZ)

/* Sanity limits */
#define RS_MIN  0.005f    /* 5 mohm */
#define RS_MAX  50.0f     /* 50 ohm */
#define LS_MIN  1e-6f     /* 1 uH */
#define LS_MAX  0.1f      /* 100 mH */
#define KE_MIN  1e-5f     /* Tiny motor */
#define KE_MAX  0.5f      /* Large motor */

#define DT_FAST FOC_TS_FAST_S

/* ── Internal sub-phase tracking ──────────────────────────────── */

/* Within each DETECT_x state, detect_ctr counts ticks.
 * detect_v_accum / detect_i_accum accumulate measurements.
 * detect_samples counts valid samples. */

/* Ls pulse state machine (within DETECT_L) */
typedef enum {
    LS_PHASE_SETTLE = 0,
    LS_PHASE_STEP,
    LS_PHASE_COAST
} ls_phase_t;

/* File-scope Ls pulse tracking */
static uint16_t ls_pulse_count;
static ls_phase_t ls_phase;
static uint32_t ls_phase_ctr;
static float ls_i_start;       /* Current at start of voltage step */
static float ls_step_voltage;  /* Adaptive step voltage based on measured Rs */
static float lambda_test_speed; /* Adaptive test speed for lambda phase */

void foc_detect_start(FOC_State_t *foc)
{
    foc->detect_state   = DETECT_R;
    foc->detect_ctr     = 0;
    foc->detect_v_accum = 0.0f;
    foc->detect_i_accum = 0.0f;
    foc->detect_samples = 0;
    foc->detect_Rs      = 0.0f;
    foc->detect_Ls      = 0.0f;
    foc->detect_lambda  = 0.0f;
    foc->mode           = FOC_MOTOR_DETECT;
    foc->sub_state      = 1;  /* Telemetry: detect active */

    /* Reset I/f state for lambda phase */
    foc->theta_if = 0.0f;
    foc->omega_if = 0.0f;

    /* Init PI for detect — soft gains, bus-scaled voltage clamp.
     * Vclamp = 0.15*Vbus (≈1.8V@12V, ≈3.6V@24V) — enough for high-Rs
     * motors (Hurst: 2.35Ω×0.5A = 1.18V) while limiting peak current
     * on low-Rs motors (A2212: 1.8V/0.065Ω = 27A, but slow ramp
     * + Kp=0.05 keeps peak well below that).
     * Ki=50 → slow integral, won't overshoot. */
    float vclamp = foc->vbus * 0.15f;
    if (vclamp < 1.5f) vclamp = 1.5f;  /* min 1.5V for very low Vbus */
    foc_pi_init(&foc->pid_d, 0.05f, 50.0f, -vclamp, vclamp);
    foc_pi_init(&foc->pid_q, 0.05f, 50.0f, -vclamp, vclamp);

    /* Reset Ls pulse state */
    ls_pulse_count = 0;
    ls_phase = LS_PHASE_SETTLE;
    ls_phase_ctr = 0;
    ls_i_start = 0.0f;
}

bool foc_detect_fast_tick(FOC_State_t *foc,
                          float ia, float ib,
                          float i_alpha, float i_beta,
                          float *da_out, float *db_out, float *dc_out)
{
    foc->detect_ctr++;

    /* Default: safe center duty */
    *da_out = 0.5f;
    *db_out = 0.5f;
    *dc_out = 0.5f;

    switch (foc->detect_state) {

    /* ════════════════════════════════════════════════════════════
     * PHASE 1: Rs — Command Id at θ=0, measure Vd/Id at steady state
     * ════════════════════════════════════════════════════════════ */
    case DETECT_R: {
        float sin_t = 0.0f;  /* sin(0) */
        float cos_t = 1.0f;  /* cos(0) */

        /* Park transform at θ=0 */
        float id_meas, iq_meas;
        foc_park(i_alpha, i_beta, sin_t, cos_t, &id_meas, &iq_meas);

        /* Ramp Id reference */
        float id_ref;
        if (foc->detect_ctr <= RS_RAMP_TICKS) {
            float frac = (float)foc->detect_ctr / (float)RS_RAMP_TICKS;
            id_ref = DETECT_TEST_CURRENT_A * frac;
        } else {
            id_ref = DETECT_TEST_CURRENT_A;
        }

        /* PI current control */
        float vd = foc_pi_run(&foc->pid_d, id_ref - id_meas, DT_FAST);
        float vq = foc_pi_run(&foc->pid_q, 0.0f - iq_meas, DT_FAST);

        /* Sampling phase: after ramp + settle */
        uint32_t sample_start = RS_RAMP_TICKS + RS_SETTLE_TICKS;
        uint32_t sample_end   = sample_start + RS_SAMPLE_TICKS;

        if (foc->detect_ctr > sample_start && foc->detect_ctr <= sample_end) {
            /* Accumulate Vd and Id for averaging.
             * At θ=0 steady state with no rotation: Vd = Rs * Id */
            foc->detect_v_accum += vd;
            foc->detect_i_accum += id_meas;
            foc->detect_samples++;
        }

        /* Inverse Park + SVPWM */
        float v_alpha, v_beta;
        foc_inv_park(vd, vq, sin_t, cos_t, &v_alpha, &v_beta);
        foc_svpwm(v_alpha, v_beta, foc->vbus, da_out, db_out, dc_out);

        /* Telemetry */
        foc->id_meas = id_meas;
        foc->iq_meas = iq_meas;
        foc->vd = vd;
        foc->vq = vq;

        /* Phase complete? */
        if (foc->detect_ctr > sample_end) {
            if (foc->detect_samples > 0 && foc->detect_i_accum > 0.1f) {
                foc->detect_Rs = foc->detect_v_accum / foc->detect_i_accum;
                foc->detect_Rs = foc_clampf(foc->detect_Rs, RS_MIN, RS_MAX);
            } else {
                foc->detect_state = DETECT_FAIL;
                return true;
            }

            /* Transition to Ls */
            foc->detect_state   = DETECT_L;
            foc->detect_ctr     = 0;
            foc->detect_v_accum = 0.0f;
            foc->detect_i_accum = 0.0f;
            foc->detect_samples = 0;
            foc->sub_state      = 2;

            /* Compute adaptive Ls step voltage: V = Rs * Ipeak.
             * Targets LS_TARGET_PEAK_A steady-state current.
             * Clamp [0.1V, 3V] for safety. */
            ls_step_voltage = foc->detect_Rs * LS_TARGET_PEAK_A;
            ls_step_voltage = foc_clampf(ls_step_voltage, 0.1f, 3.0f);

            /* Reset PI and Ls pulse state */
            foc_pi_reset(&foc->pid_d);
            foc_pi_reset(&foc->pid_q);
            ls_pulse_count = 0;
            ls_phase = LS_PHASE_SETTLE;
            ls_phase_ctr = 0;
        }
        break;
    }

    /* ════════════════════════════════════════════════════════════
     * PHASE 2: Ls — Voltage step, measure dI/dt
     * ════════════════════════════════════════════════════════════ */
    case DETECT_L: {
        float sin_t = 0.0f;
        float cos_t = 1.0f;

        float id_meas, iq_meas;
        foc_park(i_alpha, i_beta, sin_t, cos_t, &id_meas, &iq_meas);

        float vd = 0.0f;
        float vq = 0.0f;

        ls_phase_ctr++;

        switch (ls_phase) {
        case LS_PHASE_SETTLE:
            /* Let current decay to ~0 (PI holds Id=0) */
            vd = foc_pi_run(&foc->pid_d, 0.0f - id_meas, DT_FAST);
            vq = foc_pi_run(&foc->pid_q, 0.0f - iq_meas, DT_FAST);
            if (ls_phase_ctr >= (ls_pulse_count == 0 ? LS_SETTLE_TICKS : LS_COAST_TICKS)) {
                ls_phase = LS_PHASE_STEP;
                ls_phase_ctr = 0;
                ls_i_start = id_meas;
                foc_pi_reset(&foc->pid_d);
                foc_pi_reset(&foc->pid_q);
            }
            break;

        case LS_PHASE_STEP:
            /* Apply constant voltage step on d-axis, bypass PI.
             * L = V * dt / ΔI.  We measure ΔI over LS_STEP_TICKS. */
            vd = ls_step_voltage;
            vq = 0.0f;

            if (ls_phase_ctr >= LS_STEP_TICKS) {
                /* Measure current change */
                float di = id_meas - ls_i_start;
                if (di > 0.005f) {
                    /* L = (V - R*I_avg) * t / ΔI — corrected for R drop */
                    float t_step = (float)LS_STEP_TICKS * DT_FAST;
                    float i_avg = (ls_i_start + id_meas) * 0.5f;
                    float v_net = ls_step_voltage - foc->detect_Rs * i_avg;
                    if (v_net > 0.01f) {
                        float l_sample = v_net * t_step / di;
                        if (l_sample > LS_MIN && l_sample < LS_MAX) {
                            foc->detect_v_accum += l_sample;
                            foc->detect_samples++;
                        }
                    }
                }

                ls_pulse_count++;
                if (ls_pulse_count >= LS_NUM_PULSES) {
                    /* All pulses done — compute average Ls */
                    if (foc->detect_samples >= 2) {
                        foc->detect_Ls = foc->detect_v_accum / (float)foc->detect_samples;
                        foc->detect_Ls = foc_clampf(foc->detect_Ls, LS_MIN, LS_MAX);
                    } else {
                        foc->detect_state = DETECT_FAIL;
                        return true;
                    }

                    /* Lambda test speed: use default. */
                    lambda_test_speed = LAMBDA_TEST_SPEED;

                    /* Update PI gains with measured Rs, Ls for align + lambda.
                     * Vclamp = 0.45 × Vbus/√3 — leaves headroom vs linear limit. */
                    float vclamp = foc->vbus * 0.45f * FOC_INV_SQRT3;
                    float kp = foc->detect_Ls * TUNE_CURRENT_BW_RAD;
                    float ki = foc->detect_Rs * TUNE_CURRENT_BW_RAD;
                    foc_pi_init(&foc->pid_d, kp, ki, -vclamp, vclamp);
                    foc_pi_init(&foc->pid_q, kp, ki, -vclamp, vclamp);

                    /* Transition to re-alignment (NOT directly to lambda).
                     * After Ls pulses, rotor may have drifted from θ=0.
                     * Re-align firmly before spinning. */
                    foc->detect_state   = DETECT_ALIGN;
                    foc->detect_ctr     = 0;
                    foc->detect_v_accum = 0.0f;
                    foc->detect_i_accum = 0.0f;
                    foc->detect_samples = 0;
                    foc->sub_state      = 3;
                    foc->theta_if       = 0.0f;
                    foc->omega_if       = 0.0f;
                } else {
                    /* More pulses to go — coast and repeat */
                    ls_phase = LS_PHASE_SETTLE;
                    ls_phase_ctr = 0;
                }
            }
            break;

        case LS_PHASE_COAST:
            /* Handled by LS_PHASE_SETTLE with coast timing */
            break;
        }

        /* Inverse Park + SVPWM */
        float v_alpha, v_beta;
        foc_inv_park(vd, vq, sin_t, cos_t, &v_alpha, &v_beta);
        foc_svpwm(v_alpha, v_beta, foc->vbus, da_out, db_out, dc_out);

        foc->id_meas = id_meas;
        foc->iq_meas = iq_meas;
        foc->vd = vd;
        foc->vq = vq;
        break;
    }

    /* ════════════════════════════════════════════════════════════
     * PHASE 2.5: Re-align rotor at θ=0 before spinning.
     *
     * After Ls pulses, rotor may have drifted from θ=0 (no holding
     * current during coast periods).  Re-applying Id for 200ms
     * firmly locks the rotor position.  Then a 100ms smooth d→q
     * transition (like normal IF_RAMP) ensures the torque vector
     * rotates gradually — avoids the 90° current jump that could
     * cause rotor sync loss at lambda start.
     * ════════════════════════════════════════════════════════════ */
    case DETECT_ALIGN: {
        /* Pure Id alignment at θ=0 — NO d→q transition here.
         * The d→q transition must happen in DETECT_LAMBDA while θ
         * is advancing, otherwise Iq at static θ=0 pushes the rotor
         * 90° ahead, putting all BEMF into Vd instead of Vq. */
        float sin_t = 0.0f;
        float cos_t = 1.0f;

        float id_meas, iq_meas;
        foc_park(i_alpha, i_beta, sin_t, cos_t, &id_meas, &iq_meas);

        /* Ramp Id to LAMBDA_IQ_A for firm alignment */
        float ramp = (float)foc->detect_ctr / (float)(REALIGN_TICKS / 2);
        if (ramp > 1.0f) ramp = 1.0f;
        float id_ref = LAMBDA_IQ_A * ramp;

        float vd = foc_pi_run(&foc->pid_d, id_ref - id_meas, DT_FAST);
        float vq = foc_pi_run(&foc->pid_q, 0.0f - iq_meas, DT_FAST);

        float v_alpha, v_beta;
        foc_inv_park(vd, vq, sin_t, cos_t, &v_alpha, &v_beta);
        foc_svpwm(v_alpha, v_beta, foc->vbus, da_out, db_out, dc_out);

        foc->id_meas = id_meas;
        foc->iq_meas = iq_meas;
        foc->vd = vd;
        foc->vq = vq;

        /* Transition to lambda after alignment hold */
        if (foc->detect_ctr >= REALIGN_TICKS) {
            foc->detect_state   = DETECT_LAMBDA;
            foc->detect_ctr     = 0;
            foc->detect_v_accum = 0.0f;
            foc->detect_i_accum = 0.0f;
            foc->detect_samples = 0;
            foc->sub_state      = 4;
            foc->theta_if       = 0.0f;
            foc->omega_if       = 0.0f;
            /* Don't reset PI — Id integral helps bumpless d→q start */
        }
        break;
    }

    /* ════════════════════════════════════════════════════════════
     * PHASE 3: λ_pm — Spin with I/f, measure back-EMF from Vq
     * ════════════════════════════════════════════════════════════ */
    case DETECT_LAMBDA: {
        /* I/f ramp: accelerate to adaptive test speed */
        if (foc->omega_if < lambda_test_speed) {
            foc->omega_if += LAMBDA_RAMP_RATE * DT_FAST;
            if (foc->omega_if > lambda_test_speed)
                foc->omega_if = lambda_test_speed;
        }

        /* Advance forced angle — MUST start from tick 1 */
        foc->theta_if += foc->omega_if * DT_FAST;
        foc->theta_if = foc_angle_wrap(foc->theta_if);

        float sin_t = sinf(foc->theta_if);
        float cos_t = cosf(foc->theta_if);

        /* Park transform */
        float id_meas, iq_meas;
        foc_park(i_alpha, i_beta, sin_t, cos_t, &id_meas, &iq_meas);

        /* d→q transition WHILE θ is advancing (like normal IF_RAMP).
         * This is critical: transitioning to Iq at static θ creates a
         * static torque that pushes the rotor ~90° ahead, causing BEMF
         * to appear in Vd instead of Vq → λ_measured ≈ 0.
         * By advancing θ simultaneously, the rotating field drags the
         * rotor along smoothly, maintaining synchronization. */
        float id_ref, iq_ref;
        if (foc->detect_ctr <= LAMBDA_DQ_TICKS) {
            float frac = (float)foc->detect_ctr / (float)LAMBDA_DQ_TICKS;
            id_ref = LAMBDA_IQ_A * (1.0f - frac);
            iq_ref = LAMBDA_IQ_A * frac;
        } else {
            id_ref = 0.0f;
            iq_ref = LAMBDA_IQ_A;
        }

        float vd = foc_pi_run(&foc->pid_d, id_ref - id_meas, DT_FAST);
        float vq = foc_pi_run(&foc->pid_q, iq_ref - iq_meas, DT_FAST);

        /* Cross-coupling feedforward (uses measured Rs, Ls) */
        float ff_d = -foc->omega_if * foc->detect_Ls * iq_meas;
        float ff_q =  foc->omega_if * foc->detect_Ls * id_meas;
        float vd_tot = vd + ff_d;
        float vq_tot = vq + ff_q;

        /* Sampling: after d→q transition + speed ramp + settle */
        uint32_t sample_start = LAMBDA_DQ_TICKS + LAMBDA_RAMP_TICKS + LAMBDA_SETTLE_TICKS;
        uint32_t sample_end   = sample_start + LAMBDA_SAMPLE_TICKS;

        if (foc->detect_ctr > sample_start && foc->detect_ctr <= sample_end) {
            /* Accumulate total Vq and Iq.
             * At steady state with Id≈0:
             *   Vq_total = Rs*Iq + ω*λ_pm
             *   λ_pm = (Vq_total - Rs*Iq) / ω */
            foc->detect_v_accum += vq_tot;
            foc->detect_i_accum += iq_meas;
            foc->detect_samples++;
        }

        /* Inverse Park + SVPWM */
        float v_alpha, v_beta;
        foc_inv_park(vd_tot, vq_tot, sin_t, cos_t, &v_alpha, &v_beta);
        foc_svpwm(v_alpha, v_beta, foc->vbus, da_out, db_out, dc_out);

        foc->id_meas = id_meas;
        foc->iq_meas = iq_meas;
        foc->vd = vd_tot;
        foc->vq = vq_tot;
        foc->omega = foc->omega_if;

        /* Phase complete? */
        if (foc->detect_ctr > sample_end) {
            if (foc->detect_samples > 0 && foc->omega_if > 10.0f) {
                float avg_vq = foc->detect_v_accum / (float)foc->detect_samples;
                float avg_iq = foc->detect_i_accum / (float)foc->detect_samples;

                foc->detect_lambda = (avg_vq - foc->detect_Rs * avg_iq)
                                   / foc->omega_if;
                foc->detect_lambda = foc_clampf(foc->detect_lambda, KE_MIN, KE_MAX);
            } else {
                foc->detect_state = DETECT_FAIL;
                return true;
            }

            /* Transition to auto-tune */
            foc->detect_state = DETECT_TUNE;
            foc->detect_ctr   = 0;
            foc->sub_state    = 5;
        }
        break;
    }

    /* ════════════════════════════════════════════════════════════
     * PHASE 4: Auto-tune — Compute PI gains, apply all params
     * ════════════════════════════════════════════════════════════ */
    case DETECT_TUNE: {
        /* Single-tick computation */
        foc->detect_state = DETECT_DONE;
        foc->sub_state    = 6;
        return true;
    }

    case DETECT_DONE:
    case DETECT_FAIL:
        return true;

    default:
        foc->detect_state = DETECT_FAIL;
        return true;
    }

    return false;
}

void foc_detect_apply(FOC_State_t *foc)
{
    if (foc->detect_state != DETECT_DONE) return;

    /* Apply measured motor params */
    foc->Rs        = foc->detect_Rs;
    foc->Ls        = foc->detect_Ls;
    foc->lambda_pm = foc->detect_lambda;
    foc->Ke        = foc->detect_lambda;

    /* Compute and apply PI gains: pole cancellation at 1 kHz BW */
    float kp_dq = foc->detect_Ls * TUNE_CURRENT_BW_RAD;
    float ki_dq = foc->detect_Rs * TUNE_CURRENT_BW_RAD;
    float vclamp = foc->vbus * 0.95f * FOC_INV_SQRT3;
    foc_pi_init(&foc->pid_d, kp_dq, ki_dq, -vclamp, vclamp);
    foc_pi_init(&foc->pid_q, kp_dq, ki_dq, -vclamp, vclamp);

    /* Update observer initial lambda */
    foc->obs.lambda_est = foc->detect_lambda;

    /* Compute max electrical speed from Vbus and Ke */
    if (foc->detect_lambda > KE_MIN) {
        float omega_max = foc->vbus * 0.85f * FOC_INV_SQRT3 / foc->detect_lambda;
        foc->max_elec_rad_s = omega_max;
        foc->pll.omega_max  = omega_max;
    }

    /* Set reasonable startup params based on detected motor */
    foc->handoff_rad_s = lambda_test_speed;  /* Proven spin speed */
    foc->ramp_rate     = LAMBDA_RAMP_RATE;   /* Proven ramp rate */
    foc->align_iq      = DETECT_TEST_CURRENT_A;
    foc->ramp_iq       = LAMBDA_IQ_A;

    /* Conservative overcurrent limit: 4× max detect current.
     * Uses whichever is higher: Rs test (0.5A) or lambda spin (1.0A).
     * User can raise via GSP if needed. */
    float max_test_i = DETECT_TEST_CURRENT_A > LAMBDA_IQ_A
                     ? DETECT_TEST_CURRENT_A : LAMBDA_IQ_A;
    foc->fault_oc_a    = max_test_i * 4.0f;
}
