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
#include "pi.h"   /* generic MC_PI_T — reused by the WS4 cascade below */

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

/* ======================================================================
 * WS4: Nested speed→current→duty cascade (FEATURE_SPEED_CASCADE).
 *
 * A FRESH implementation, distinct from SPEED_PI above — it does NOT use
 * the speed_pi logic (that one outputs duty directly and had high-RPM OV
 * issues). It mirrors the Microchip/Atomberg reference cascade:
 *
 *   throttle → target eRPM ─►[outer speed PI]─► current ref (clamped to
 *                                                a ceiling)
 *   phase current (counts) ──►[inner current PI]─► duty
 *
 * The ceiling on the outer PI output (= the inner loop's current setpoint
 * cap) is THE safety: it bounds how hard the loop pushes to hold speed,
 * kept below the WS1 demag desync current and the WS2 stall threshold.
 *
 * NOTE: the inner loop regulates PHASE current magnitude (max|iaRaw|,|ibRaw|
 * about bias), NOT bus current. Bench data shows ibusRaw reads BELOW bias
 * under load (freewheel valley-sampling artifact) → a bus-keyed inner loop
 * would see "current << ref" and slam duty to max, driving straight into the
 * desync. Phase current is the clean, positive, torque-tracking signal. The
 * outer ceiling (counts) is therefore a PHASE-current ceiling.
 *
 * Co-located in this file ONLY to avoid adding a new translation unit to
 * the MPLAB project; it shares no state with SPEED_PI. Outer runs per-ZC
 * (from hwzc.c); inner runs per ADC ISR (from the CL duty block). Gains
 * are re-read every call (live-tunable). All current quantities are RAW
 * ADC counts above the 2048 bias → gain-independent.
 * ====================================================================== */
#if FEATURE_SPEED_CASCADE

static MC_PI_T  s_casSpeed;              /* outer: eRPM err  -> current ref (counts) */
static MC_PI_T  s_casCurr;               /* inner: current err -> duty (ticks)       */
static bool     s_casEnabled    = false;  /* loop actively driving duty (engaged)     */
static float    s_casCurrentRef = 0.0f;  /* SLEWED outer output = inner setpoint     */
static float    s_casRefTarget  = 0.0f;  /* raw outer PI output (slew source)        */
static float    s_casIfilt      = 0.0f;  /* IIR-filtered inner phase current         */
static float    s_casDuty       = 0.0f;  /* SLEWED applied duty (slew accumulator)   */

static inline void casLoadGains(void)
{
    s_casSpeed.param.kp     = (float)RT_CASCADE_SPEED_KP_MILLI / 1000.0f;
    s_casSpeed.param.ki     = (float)RT_CASCADE_SPEED_KI_MICRO / 1000000.0f;
    s_casSpeed.param.outMax = (float)RT_CASCADE_IREF_CEILING_ADC;  /* the ceiling */
    s_casSpeed.param.outMin = 0.0f;

    s_casCurr.param.kp      = (float)RT_CASCADE_CURR_KP_MILLI / 1000.0f;
    s_casCurr.param.ki      = (float)RT_CASCADE_CURR_KI_MICRO / 1000000.0f;
    /* inner outMax/outMin set per step from the live duty cap. */
}

/* Phase-current magnitude in raw ADC counts above the 2048 bias — the larger
 * of the two measured phase legs. The clean, positive, load-tracking signal
 * the inner loop regulates (see header note on the ibusRaw freewheel artifact). */
static inline float casPhaseMag(volatile GARUDA_DATA_T *pData)
{
    int32_t iaDev = (int32_t)pData->phaseCurrent.iaRaw - 2048;
    int32_t ibDev = (int32_t)pData->phaseCurrent.ibRaw - 2048;
    uint32_t iaMag = (uint32_t)(iaDev < 0 ? -iaDev : iaDev);
    uint32_t ibMag = (uint32_t)(ibDev < 0 ? -ibDev : ibDev);
    return (float)((iaMag > ibMag) ? iaMag : ibMag);
}

static inline uint32_t casMeasuredErpm(volatile GARUDA_DATA_T *pData)
{
    if (pData->hwzc.stepPeriodHR > 0)
        return HWZC_TICKS_TO_ERPM(pData->hwzc.stepPeriodHR);
    return 0;
}

/* Seed every state bumplessly from the present operating point so engaging the
 * loop produces ZERO step: ref = present current, duty accumulator = applied
 * duty, both integrators back-calculated to those values. */
static void casEngage(volatile GARUDA_DATA_T *pData)
{
    casLoadGains();
    float iMag = casPhaseMag(pData);
    if (iMag > (float)RT_CASCADE_IREF_CEILING_ADC)
        iMag = (float)RT_CASCADE_IREF_CEILING_ADC;
    s_casCurrentRef = iMag;
    s_casRefTarget  = iMag;
    s_casIfilt      = iMag;
    s_casDuty       = (float)pData->duty;
    MC_ControllerPIReset(&s_casSpeed, iMag);
    MC_ControllerPIReset(&s_casCurr,  (float)pData->duty);
    s_casEnabled = true;
}

void SPEED_CASCADE_Init(void)
{
    s_casEnabled    = false;
    s_casCurrentRef = 0.0f;
    s_casRefTarget  = 0.0f;
    s_casIfilt      = 0.0f;
    s_casDuty       = 0.0f;
    MC_ControllerPIReset(&s_casSpeed, 0.0f);
    MC_ControllerPIReset(&s_casCurr,  0.0f);
}

/* Called on CL entry — force the loop disengaged so the normal throttle→duty
 * map drives startup and the sync-fragile low-speed accel. The loop re-engages
 * itself only once measured eRPM climbs past the engage gate (see InnerStep). */
void SPEED_CASCADE_Reset(void) { s_casEnabled = false; }

void SPEED_CASCADE_Disable(void) { s_casEnabled = false; }

bool SPEED_CASCADE_Enabled(void) { return s_casEnabled; }

/* Outer speed PI — per ZC. throttle→target eRPM; PI output = current ref,
 * SLEW-LIMITED into s_casCurrentRef so the inner setpoint can't step. */
void SPEED_CASCADE_OnZcEvent(volatile GARUDA_DATA_T *pData)
{
    if (!s_casEnabled) return;
    casLoadGains();   /* live re-tune */

    uint32_t thr = pData->throttle;
    if (thr > 4095u) thr = 4095u;
    uint32_t idle = RT_CASCADE_TGT_ERPM_IDLE;
    uint32_t maxe = RT_CASCADE_TGT_ERPM_MAX;
    uint32_t span = (maxe > idle) ? (maxe - idle) : 0u;
    uint32_t target_erpm = idle + (uint32_t)((uint64_t)span * thr / 4096u);
    if (target_erpm < CASCADE_ERPM_FLOOR) target_erpm = CASCADE_ERPM_FLOOR;

    s_casSpeed.inReference = (float)target_erpm;
    s_casSpeed.inMeasure   = (float)casMeasuredErpm(pData);
    MC_ControllerPIUpdate(&s_casSpeed);     /* output clamped to [0, ceiling] */
    s_casRefTarget = s_casSpeed.output;

    /* Slew the inner setpoint toward the PI output — bounds how fast we can
     * demand more current, so a big speed error can't instantly call ceiling. */
    float step = (float)CASCADE_IREF_SLEW_PER_ZC;
    if      (s_casRefTarget > s_casCurrentRef + step) s_casCurrentRef += step;
    else if (s_casRefTarget < s_casCurrentRef - step) s_casCurrentRef -= step;
    else                                              s_casCurrentRef  = s_casRefTarget;
}

/* Inner current PI — per ADC ISR. Below the engage gate, returns fallbackDuty
 * (the normal map) untouched. Once engaged, regulates phase current→duty with a
 * per-ISR DUTY SLEW limit + instantaneous over-current foldback. */
uint32_t SPEED_CASCADE_InnerStep(volatile GARUDA_DATA_T *pData,
                                 uint32_t cap, uint32_t fallbackDuty)
{
    uint32_t erpm = casMeasuredErpm(pData);

    if (!s_casEnabled)
    {
        /* Engage only once the motor is established ABOVE the sync-fragile zone.
         * Until then the proven throttle→duty map runs — the loop never forces
         * current through the low-speed region where pushing it desyncs. */
        if (erpm >= CASCADE_ENGAGE_ERPM)
            casEngage(pData);
        else
            return fallbackDuty;
    }
    else if (erpm < CASCADE_DISENGAGE_ERPM)
    {
        /* Dropped back below the stable band (load too heavy to hold) — hand
         * control back to the normal map rather than keep forcing current. */
        s_casEnabled = false;
        return fallbackDuty;
    }

    float magNow = casPhaseMag(pData);
    s_casIfilt += (magNow - s_casIfilt) * CASCADE_IFILT_ALPHA;

    s_casCurr.param.outMax = (float)cap;
    s_casCurr.param.outMin = (float)MIN_DUTY;
    s_casCurr.inReference  = s_casCurrentRef;
    s_casCurr.inMeasure    = s_casIfilt;
    MC_ControllerPIUpdate(&s_casCurr);
    float piDuty = s_casCurr.output;

    /* Per-ISR duty slew limit — the actual applied duty can move at most
     * CASCADE_DUTY_SLEW_PER_ISR ticks/ISR toward the PI output. This is the
     * structural guarantee against a one-ISR slam: no matter what the PI (or a
     * wound-up integrator) commands, duty ramps, current can't step to a rail. */
    float dStep = (float)CASCADE_DUTY_SLEW_PER_ISR;
    if      (piDuty > s_casDuty + dStep) s_casDuty += dStep;
    else if (piDuty < s_casDuty - dStep) s_casDuty -= dStep;
    else                                 s_casDuty  = piDuty;

    uint32_t duty = (uint32_t)s_casDuty;

    /* Hard foldback on INSTANTANEOUS current — bypasses the slew, collapses duty
     * immediately and drags every accumulator down with it. Last-resort bound on
     * ACTUAL current if the loop ever goes unstable. */
    float iHard = (float)RT_CASCADE_IREF_CEILING_ADC + (float)CASCADE_IHARD_MARGIN_ADC;
    if (magNow > iHard || s_casIfilt > iHard)
    {
        uint32_t fold = (duty * CASCADE_FOLDBACK_NUM) / CASCADE_FOLDBACK_DEN;
        if (fold < MIN_DUTY) fold = MIN_DUTY;
        duty      = fold;
        s_casDuty = (float)fold;
        MC_ControllerPIReset(&s_casCurr, (float)fold);
    }
    if (duty < MIN_DUTY) duty = MIN_DUTY;
    if (duty > cap)      duty = cap;
    return duty;
}

#else  /* !FEATURE_SPEED_CASCADE */

void SPEED_CASCADE_Init(void) {}
void SPEED_CASCADE_Reset(void) {}
void SPEED_CASCADE_Disable(void) {}
bool SPEED_CASCADE_Enabled(void) { return false; }
void SPEED_CASCADE_OnZcEvent(volatile GARUDA_DATA_T *pData) { (void)pData; }
uint32_t SPEED_CASCADE_InnerStep(volatile GARUDA_DATA_T *pData,
                                 uint32_t cap, uint32_t fallbackDuty)
{ (void)pData; (void)cap; return fallbackDuty; }

#endif /* FEATURE_SPEED_CASCADE */
