/**
 * @file foc_v2_detect.c
 * @brief Motor auto-detection: R, L, lambda measurement and PI auto-tune.
 *
 * Detection sequence (all at θ=0, rotor locked):
 *   1. DETECT_R:      Ramp current to test level, measure V/I → Rs
 *   2. DETECT_L:      Apply voltage steps, measure ΔI → Ls
 *   3. DETECT_LAMBDA: Spin up I/f, measure BEMF → lambda
 *   4. DETECT_TUNE:   Compute PI gains from measured params
 *
 * Component: FOC V2
 */

#include "foc_v2_detect.h"
#include "foc_v2_math.h"
#include "../garuda_foc_params.h"

/* Detection test current (A) — moderate, safe for most motors */
#define DETECT_TEST_CURRENT_A   1.0f

/* Phase timing (slow loop ticks = ms) */
#define DETECT_R_RAMP_MS        200U
#define DETECT_R_SETTLE_MS      100U
#define DETECT_R_SAMPLE_MS      500U
#define DETECT_R_TOTAL_MS       (DETECT_R_RAMP_MS + DETECT_R_SETTLE_MS + DETECT_R_SAMPLE_MS)

#define DETECT_L_PULSES         100U
#define DETECT_L_PULSE_MS       5U
#define DETECT_L_TOTAL_MS       (DETECT_L_PULSES * DETECT_L_PULSE_MS)

#define DETECT_LAMBDA_SPINUP_MS 3000U
#define DETECT_LAMBDA_SAMPLE_MS 1000U
#define DETECT_LAMBDA_TOTAL_MS  (DETECT_LAMBDA_SPINUP_MS + DETECT_LAMBDA_SAMPLE_MS)

/* Auto-tune bandwidth targets */
#define TUNE_CURRENT_BW_RAD_S   (2.0f * FOC_PI_F * 1000.0f)  /* 1kHz */
#define TUNE_PLL_BW_HZ          50.0f

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
}

bool foc_detect_tick(FOC_State_t *foc, float ia, float ib)
{
    foc->detect_ctr++;

    switch (foc->detect_state) {
    case DETECT_R: {
        /* Phase 1: Resistance measurement.
         * Lock rotor at θ=0, ramp current, measure V/I at steady state. */
        if (foc->detect_ctr <= DETECT_R_RAMP_MS) {
            /* Ramping — let PI settle */
        } else if (foc->detect_ctr <= (DETECT_R_RAMP_MS + DETECT_R_SETTLE_MS)) {
            /* Settling */
        } else if (foc->detect_ctr <= DETECT_R_TOTAL_MS) {
            /* Sampling: accumulate Vq and Iq */
            float iq_abs = (ia >= 0.0f) ? ia : -ia;
            foc->detect_i_accum += iq_abs;
            foc->detect_v_accum += foc->vq;
            foc->detect_samples++;
        } else {
            /* Compute Rs */
            if (foc->detect_samples > 0 && foc->detect_i_accum > 0.1f) {
                foc->detect_Rs = foc->detect_v_accum / foc->detect_i_accum;
                if (foc->detect_Rs < 0.001f) foc->detect_Rs = 0.001f;
                if (foc->detect_Rs > 100.0f) foc->detect_Rs = 100.0f;
            } else {
                foc->detect_state = DETECT_FAIL;
                return true;
            }
            /* Transition to L measurement */
            foc->detect_state   = DETECT_L;
            foc->detect_ctr     = 0;
            foc->detect_v_accum = 0.0f;
            foc->detect_i_accum = 0.0f;
            foc->detect_samples = 0;
        }
        break;
    }

    case DETECT_L: {
        /* Phase 2: Inductance measurement.
         * Apply voltage pulses, measure ΔI over known time.
         * L = V_applied * dt / ΔI */
        if (foc->detect_ctr <= DETECT_L_TOTAL_MS) {
            /* Accumulate peak current from each pulse */
            float i_mag = (ia >= 0.0f) ? ia : -ia;
            if (i_mag > 0.01f) {
                /* V_applied approximated from Vbus * duty_step */
                float v_step = foc->vbus * 0.05f;  /* 5% duty step */
                float dt_step = 1.0f / (float)PWMFREQUENCY_HZ;  /* One PWM cycle */
                float l_sample = v_step * dt_step / i_mag;
                foc->detect_v_accum += l_sample;
                foc->detect_samples++;
            }
        } else {
            /* Average L, derate by 0.9 (VESC practice) */
            if (foc->detect_samples > 10) {
                foc->detect_Ls = (foc->detect_v_accum / (float)foc->detect_samples) * 0.9f;
                if (foc->detect_Ls < 1e-7f) foc->detect_Ls = 1e-7f;
                if (foc->detect_Ls > 0.1f)  foc->detect_Ls = 0.1f;
            } else {
                foc->detect_state = DETECT_FAIL;
                return true;
            }
            /* Transition to lambda measurement */
            foc->detect_state   = DETECT_LAMBDA;
            foc->detect_ctr     = 0;
            foc->detect_v_accum = 0.0f;
            foc->detect_i_accum = 0.0f;
            foc->detect_samples = 0;
        }
        break;
    }

    case DETECT_LAMBDA: {
        /* Phase 3: Flux linkage measurement.
         * Spin motor with I/f, measure Vq and Iq at known speed.
         * λ = (Vq - Rs*Iq) / ω - Ls*Iq (approximate) */
        if (foc->detect_ctr <= DETECT_LAMBDA_SPINUP_MS) {
            /* Spinning up — let I/f ramp handle this */
        } else if (foc->detect_ctr <= DETECT_LAMBDA_TOTAL_MS) {
            /* Sampling: accumulate Vq, Iq, and ω */
            foc->detect_v_accum += foc->vq;
            foc->detect_i_accum += (ia >= 0.0f) ? ia : -ia;
            foc->detect_samples++;
        } else {
            if (foc->detect_samples > 0 && foc->omega_if > 10.0f) {
                float avg_vq = foc->detect_v_accum / (float)foc->detect_samples;
                float avg_iq = foc->detect_i_accum / (float)foc->detect_samples;
                float omega = foc->omega_if;

                foc->detect_lambda = (avg_vq - foc->detect_Rs * avg_iq) / omega
                                   - foc->detect_Ls * avg_iq;
                if (foc->detect_lambda < 1e-5f) foc->detect_lambda = 1e-5f;
                if (foc->detect_lambda > 1.0f)  foc->detect_lambda = 1.0f;
            } else {
                foc->detect_state = DETECT_FAIL;
                return true;
            }
            /* Transition to auto-tune */
            foc->detect_state = DETECT_TUNE;
            foc->detect_ctr   = 0;
        }
        break;
    }

    case DETECT_TUNE: {
        /* Phase 4: Auto-tune PI gains from measured params.
         * Kp = Ls * ωbw,  Ki = Rs * ωbw */
        float kp_dq = foc->detect_Ls * TUNE_CURRENT_BW_RAD_S;
        float ki_dq = foc->detect_Rs * TUNE_CURRENT_BW_RAD_S;

        /* Apply to PI controllers */
        foc->pid_d.kp = kp_dq;
        foc->pid_d.ki = ki_dq;
        foc->pid_q.kp = kp_dq;
        foc->pid_q.ki = ki_dq;

        /* Update motor params in state */
        foc->Rs        = foc->detect_Rs;
        foc->Ls        = foc->detect_Ls;
        foc->lambda_pm = foc->detect_lambda;
        foc->Ke        = foc->detect_lambda;

        /* Update observer initial lambda */
        foc->obs.lambda_est = foc->detect_lambda;

        foc->detect_state = DETECT_DONE;
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

FOC_MotorParams_t foc_detect_get_results(const FOC_State_t *foc)
{
    FOC_MotorParams_t p;
    p.Rs             = foc->detect_Rs;
    p.Ls             = foc->detect_Ls;
    p.lambda_pm      = foc->detect_lambda;
    p.Ke             = foc->detect_lambda;
    p.pole_pairs     = MOTOR_POLE_PAIRS_FOC;
    p.max_current_a  = MOTOR_MAX_CURRENT_A;
    p.max_elec_rad_s = MOTOR_MAX_ELEC_RAD_S;
    p.vbus_nom_v     = MOTOR_VBUS_NOM_V;
    return p;
}
