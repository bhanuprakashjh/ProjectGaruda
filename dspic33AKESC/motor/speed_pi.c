/**
 * @file speed_pi.c
 *
 * @brief Per-ZC interval-based speed PID — v2 architecture.
 *
 * Architecture (bench-tuned 2026-05-29):
 *
 *   throttle → target_eRPM ──┬─→ feedforward (linear duty model)
 *                            │
 *   measured_eRPM ←── 1/T ───┤
 *                            │
 *                            └─→ error_eRPM ─→ deadband ─→ PI ─→ correction
 *
 *   output_duty = feedforward + correction (clamped CL_IDLE..MAX)
 *
 * Why this design (vs the v1 period-error approach):
 *
 *   1. eRPM is the natural control variable. Plant gain in (eRPM, duty)
 *      is CONSTANT at ~3.2 eRPM/PWM-tick across the full 14k..230k range
 *      (bench fit). Plant gain in (period, duty) varies as 1/ω² → 16×
 *      across the range → impossible to tune PI uniformly.
 *
 *   2. Feedforward eliminates the integrator's role in building up steady-
 *      state duty. duty_ff ≈ 0.305 × target_eRPM + 700 ticks covers ~95%
 *      of the steady-state duty. PI only trims the residual.
 *
 *   3. Deadband on eRPM error kills per-event HWZC capture noise (3-4%
 *      in the mid range, up to 10% at extremes). Without deadband,
 *      Kp × noise drives a small but visible limit cycle.
 *
 *   4. Gains tuned in eRPM units — operating-point-independent.
 *
 * Component: SPEED_PI
 */
#include "speed_pi.h"
#include "../garuda_config.h"
#include "../garuda_calc_params.h"

void SPEED_PI_Init(volatile GARUDA_DATA_T *pData)
{
    pData->speedPi.enabled         = false;
    pData->speedPi.zcsSinceEnable  = 0;
    pData->speedPi.integratorF     = 0.0f;
    pData->speedPi.outputDuty      = MIN_DUTY;
    pData->speedPi.lastError       = 0;
    pData->speedPi.lastTarget      = 0;
}

#if FEATURE_SPEED_PI

void SPEED_PI_Enable(volatile GARUDA_DATA_T *pData)
{
    /* Bumpless transfer: seed integrator with the residual between the
     * current duty and the feedforward prediction for the (likely-low)
     * rotor speed at handoff. Since FF is based on target_eRPM, and at
     * enable we don't yet know what target the user is commanding, we
     * just zero the integrator and let it converge after the integral-
     * disable window.
     *
     * The output we emit on the very first tick (before SPEED_PI_OnZcEvent
     * runs) is the existing pData->duty, so there's no immediate jump. */
    pData->speedPi.enabled        = true;
    pData->speedPi.zcsSinceEnable = 0;
    pData->speedPi.integratorF    = 0.0f;
    pData->speedPi.outputDuty     = pData->duty;
    pData->speedPi.lastError      = 0;
    pData->speedPi.lastTarget     = 0;
}

void SPEED_PI_Disable(volatile GARUDA_DATA_T *pData)
{
    pData->speedPi.enabled = false;
    /* Leave integratorF / outputDuty as-is for telemetry. */
}

void SPEED_PI_OnZcEvent(volatile GARUDA_DATA_T *pData)
{
    if (!pData->speedPi.enabled)
        return;

    /* Saturating ZC counter for the integral-disable gate. */
    if (pData->speedPi.zcsSinceEnable < UINT16_MAX)
        pData->speedPi.zcsSinceEnable++;

    /* ===== Stage 1: Throttle → target eRPM ===== *
     * Linear above the throttle deadband, clamped to IDLE below.
     * The deadband (= ARM_THROTTLE_ZERO_ADC for consistency with the
     * arming logic) means that small throttle ADC noise / bench pot
     * jitter at "zero throttle" doesn't drift the target above idle. */
    uint32_t thr = pData->throttle;
    if (thr > 4095u) thr = 4095u;
    uint32_t target_erpm;
    if (thr <= SPEED_PI_THROTTLE_DEADBAND) {
        target_erpm = SPEED_PI_TARGET_ERPM_IDLE;
    } else {
        uint32_t thr_active = thr - SPEED_PI_THROTTLE_DEADBAND;
        uint32_t thr_span   = 4096u - SPEED_PI_THROTTLE_DEADBAND;
        target_erpm = SPEED_PI_TARGET_ERPM_IDLE
            + (uint32_t)((uint64_t)(SPEED_PI_TARGET_ERPM_MAX
                                    - SPEED_PI_TARGET_ERPM_IDLE)
                         * thr_active / thr_span);
    }
    pData->speedPi.lastTarget = target_erpm;

    /* ===== Stage 2: Measured period → measured eRPM ===== *
     * eRPM = 1e9 / stepPeriodHR (matches host-side parser scaling).
     * Single FPU division — ~10 cycles on dsPIC33AK FPU.
     * Guard against stepPeriodHR=0 at startup. */
    uint32_t measured_erpm = 0;
    if (pData->hwzc.stepPeriodHR > 0) {
        measured_erpm = SPEED_PI_ERPM_FROM_TICKS / pData->hwzc.stepPeriodHR;
    }

    /* ===== Stage 3: Error in eRPM units (constant-plant-gain space) ===== *
     * Positive: motor too slow → need more duty.
     * Negative: motor too fast → reduce duty. */
    int32_t error = (int32_t)target_erpm - (int32_t)measured_erpm;
    pData->speedPi.lastError = error;

    /* ===== Stage 4: Deadband ===== *
     * Suppress per-event PI action for errors within ±DEADBAND_PCT of
     * target. HWZC capture noise (typically 3-4% in mid range) sits
     * inside this band and gets ignored. Real load disturbances larger
     * than the deadband pass through to the PI (shifted, so there's no
     * discontinuity at the deadband edge). */
    int32_t deadband = (int32_t)((uint64_t)target_erpm
                                  * SPEED_PI_DEADBAND_PCT / 100u);
    int32_t err_for_pi;
    if (error > deadband)         err_for_pi = error - deadband;
    else if (error < -deadband)   err_for_pi = error + deadband;
    else                           err_for_pi = 0;
    float errorF = (float)err_for_pi;

    /* ===== Stage 5: Feedforward — open-loop duty estimate ===== *
     * Linear fit duty_ticks = FF_SLOPE × target_eRPM + FF_OFFSET.
     * Captures ~95% of the steady-state duty for the target speed.
     * PI then trims the residual ~5%. */
    float ff_dutyF = SPEED_PI_FF_SLOPE * (float)target_erpm
                   + SPEED_PI_FF_OFFSET;

    /* ===== Stage 6: PI on the residual ===== *
     * Integrator gated for the first N events after CL entry so it
     * doesn't wind up during the rotor's initial speed catch-up. */
    if (pData->speedPi.zcsSinceEnable >= SPEED_PI_INTEGRAL_DISABLE_ZCS) {
        pData->speedPi.integratorF += SPEED_PI_KI_FLOAT * errorF;

        /* Tight integrator clamp — see config header for why ±8000.
         * Without this, slow drift over 50+ seconds at top RPM built
         * up to ±18% LOOPTIME of correction → throttle changes caused
         * catastrophic 18% duty jumps and OC_SW faults. */
        if (pData->speedPi.integratorF >  SPEED_PI_INTEGRATOR_CLAMP)
            pData->speedPi.integratorF =  SPEED_PI_INTEGRATOR_CLAMP;
        if (pData->speedPi.integratorF < -SPEED_PI_INTEGRATOR_CLAMP)
            pData->speedPi.integratorF = -SPEED_PI_INTEGRATOR_CLAMP;
    }

    /* Correction = I-term + P-term. Centered around 0 (the FF carries
     * the steady-state duty). */
    float correctionF = pData->speedPi.integratorF
                      + SPEED_PI_KP_FLOAT * errorF;

    /* ===== Stage 7: Combine and clamp ===== */
    float floor_F = (float)RT_CL_IDLE_DUTY;
    float output_unclampedF = ff_dutyF + correctionF;
    float outputF = output_unclampedF;
    if (outputF < floor_F)         outputF = floor_F;
    if (outputF > (float)MAX_DUTY) outputF = (float)MAX_DUTY;

    /* ===== Stage 8: Back-calc anti-windup ===== *
     * If output saturated, pull the integrator back by K_aw × overshoot. */
    float satExcess = output_unclampedF - outputF;
    if (satExcess != 0.0f) {
        pData->speedPi.integratorF -= SPEED_PI_AW_KBC * satExcess;
        /* Re-clamp after back-calc (same ±SPEED_PI_INTEGRATOR_CLAMP). */
        if (pData->speedPi.integratorF >  SPEED_PI_INTEGRATOR_CLAMP)
            pData->speedPi.integratorF =  SPEED_PI_INTEGRATOR_CLAMP;
        if (pData->speedPi.integratorF < -SPEED_PI_INTEGRATOR_CLAMP)
            pData->speedPi.integratorF = -SPEED_PI_INTEGRATOR_CLAMP;
    }

    pData->speedPi.outputDuty = (uint32_t)outputF;
}

#else /* !FEATURE_SPEED_PI */

void SPEED_PI_Enable(volatile GARUDA_DATA_T *pData)  { (void)pData; }
void SPEED_PI_Disable(volatile GARUDA_DATA_T *pData) { (void)pData; }
void SPEED_PI_OnZcEvent(volatile GARUDA_DATA_T *pData) { (void)pData; }

#endif /* FEATURE_SPEED_PI */
