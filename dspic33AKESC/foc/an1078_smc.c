/**
 * @file  an1078_smc.c
 * @brief AN1078 Sliding-Mode Observer — float port (implementation).
 *
 * Direct line-by-line translation of AN1078 `smcpos.c` to single-
 * precision float.  Algorithm preserved exactly:
 *
 *  CalcEstI:   î[k+1] = Fsmopos·î[k] + Gsmopos·(V − E1 − Z),
 *              IerrorErr = î - i_meas
 *              Z = Kslide·(err / MaxSMCError)  if |err| < MaxSMCError
 *              Z = ±Kslide                     otherwise
 *
 *  CalcBEMF:   E1 += Kslf · (Z  - E1)          first-order LPF on Z
 *              E2 += Kslf · (E1 - E2)          second LPF stage
 *
 *  Position:   Theta = atan2(-E2α, E2β) + CONSTANT_PHASE_SHIFT
 *              Omega = (Σ ΔTheta) / IRP_PERCALC   every IRP_PERCALC ticks
 *              OmegaFltred = LPF(Omega)
 *              Kslf = OmegaFltred × THETA_FILTER_CNST  (clamped at min)
 *
 * No CORCON / Q15 saturation needed — float handles range natively.
 */

#include "an1078_smc.h"
#include "an1078_params.h"
#include "pll_estimator.h"
#include "../garuda_foc_params.h"   /* PLL_ANGLE_OFFSET */
#include <math.h>

/* 2π for angle wrap (avoid relying on M_TWOPI portability). */
#define AN_TWO_PI    6.28318530717958647692f
#define AN_PI        3.14159265358979323846f

/* ── Init / Reset ─────────────────────────────────────────── */

void AN_SMCInit(AN_SMC_T *s)
{
    /* Plant model coefficients — float, no scaling factors. */
    s->Fsmopos = AN_F_PLANT;        /* 1 - Rs·Ts/Ls */
    s->Gsmopos = AN_G_PLANT;        /* Ts/Ls        */

    s->Kslide      = AN_SMC_KSLIDE;
    s->MaxSMCError = AN_SMC_MAX_LINEAR_ERR;

    s->KslfScale = AN_SMC_KSLF_SCALE;
    s->KslfMin   = AN_SMC_KSLF_MIN;
    s->Kslf      = s->KslfMin;
    s->KslfFinal = s->KslfMin;

    s->ThetaOffset = AN_SMC_THETA_OFFSET;

    AN_SMCReset(s);
}

void AN_SMCReset(AN_SMC_T *s)
{
    /* Zero all integrator state.  Mirror of pmsm.c:319 reset block.
     * Tuning fields (Fsmopos, Gsmopos, Kslide, MaxSMCError, KslfScale,
     * KslfMin, ThetaOffset) are preserved. */
    s->Valpha = 0.0f;
    s->Ealpha = 0.0f;
    s->EalphaFinal = 0.0f;
    s->Zalpha = 0.0f;
    s->EstIalpha = 0.0f;
    s->Vbeta = 0.0f;
    s->Ebeta = 0.0f;
    s->EbetaFinal = 0.0f;
    s->Zbeta = 0.0f;
    s->EstIbeta = 0.0f;
    s->Ialpha = 0.0f;
    s->IalphaError = 0.0f;
    s->Ibeta = 0.0f;
    s->IbetaError = 0.0f;
    s->Theta = 0.0f;
    s->Omega = 0.0f;
    s->OmegaFltred = 0.0f;

    s->PrevTheta = 0.0f;
    s->AccumTheta = 0.0f;
    s->AccumThetaCnt = 0;

    s->Kslf      = s->KslfMin;
    s->KslfFinal = s->KslfMin;

    pll_reset(&s->pll);
}

/* ── Helpers (file-local) ─────────────────────────────────── */

static inline float an_abs(float x) { return (x >= 0.0f) ? x : -x; }

/* Wrap angle into [0, 2π).  Used for Theta. */
static inline float an_wrap_2pi(float th)
{
    while (th >= AN_TWO_PI) th -= AN_TWO_PI;
    while (th <  0.0f)      th += AN_TWO_PI;
    return th;
}

/* Wrap delta angle into (-π, π].  Used for Δθ accumulation. */
static inline float an_wrap_delta(float dth)
{
    if (dth >  AN_PI) dth -= AN_TWO_PI;
    if (dth < -AN_PI) dth += AN_TWO_PI;
    return dth;
}

/* ── CalcEstI (port of smcpos.c CalcEstI) ─────────────────────
 *
 * Original Q15 form:
 *   EstIalpha = G·V - G·E - G·Z + F·EstIalpha
 *
 * The G·E and G·Z subtractions are computed independently in Q15
 * to avoid intermediate overflow.  In float we just compute it
 * directly: the parenthesized form is equivalent. */
static inline void CalcEstI(AN_SMC_T *s)
{
    s->EstIalpha = s->Fsmopos * s->EstIalpha
                 + s->Gsmopos * (s->Valpha - s->Ealpha - s->Zalpha);
    s->EstIbeta  = s->Fsmopos * s->EstIbeta
                 + s->Gsmopos * (s->Vbeta  - s->Ebeta  - s->Zbeta);

    s->IalphaError = s->EstIalpha - s->Ialpha;
    s->IbetaError  = s->EstIbeta  - s->Ibeta;

    /* α: linear in boundary, saturated outside.
     * AN1078 calls CalcZalpha() which performs (Kslide × err) / MaxSMCError
     * in Q15 (with __builtin_mulss + divsd).  Direct float port: */
    if (an_abs(s->IalphaError) < s->MaxSMCError) {
        s->Zalpha = (s->Kslide * s->IalphaError) / s->MaxSMCError;
    } else if (s->IalphaError > 0.0f) {
        s->Zalpha =  s->Kslide;
    } else {
        s->Zalpha = -s->Kslide;
    }

    /* β: same logic */
    if (an_abs(s->IbetaError) < s->MaxSMCError) {
        s->Zbeta = (s->Kslide * s->IbetaError) / s->MaxSMCError;
    } else if (s->IbetaError > 0.0f) {
        s->Zbeta =  s->Kslide;
    } else {
        s->Zbeta = -s->Kslide;
    }
}

/* ── CalcBEMF (port of smcpos.c CalcBEMF) ─────────────────────
 *
 * Two cascaded first-order IIR lowpass filters on the Z signal:
 *   E1 += Kslf      · (Z  - E1)
 *   E2 += KslfFinal · (E1 - E2)
 *
 * Both coefficients = same speed-adaptive value.  The original code
 * reuses Ealpha and EalphaFinal as the LPF state. */
static inline void CalcBEMF(AN_SMC_T *s)
{
    /* α channel */
    s->Ealpha      += s->Kslf      * (s->Zalpha - s->Ealpha);
    s->EalphaFinal += s->KslfFinal * (s->Ealpha - s->EalphaFinal);

    /* β channel */
    s->Ebeta       += s->Kslf      * (s->Zbeta  - s->Ebeta);
    s->EbetaFinal  += s->KslfFinal * (s->Ebeta  - s->EbetaFinal);
}

/* ── SMC_Position_Estimation (port of smcpos.c inline version) ─
 *
 * One-tick observer step.  Order:
 *   1. CalcEstI  — current model + sliding switching
 *   2. CalcBEMF  — extract back-EMF via 2-stage LPF on Z
 *   3. Theta = atan2(-Eα_filt, Eβ_filt) + offset
 *   4. Accumulate ΔTheta; every IRP_PERCALC ticks compute Omega and
 *      LP-filter into OmegaFltred (used for next-tick Kslf).
 */
void AN_SMC_Position_Estimation(AN_SMC_T *s)
{
    CalcEstI(s);
    CalcBEMF(s);

    /* ── Angle PLL (AN1292-style) ─────────────────────────────────
     *
     * Replaces AN1078's raw atan2(BEMF) with a phase-locked loop on
     * (EalphaFinal, EbetaFinal).  PLL tracks the BEMF angle smoothly
     * via cross-product discriminator + PI loop filter.  Per-tick
     * atan2 noise that previously caused angle wobble at high speed
     * is rejected by the PLL's loop-filter bandwidth (~21 Hz at
     * |E|=0.2V, ~1 kHz at |E|=10V — self-adapting because the
     * discriminator amplitude scales with |E|).
     *
     * pll.theta_est tracks the BEMF vector angle θ_E.  PMSM convention:
     * BEMF leads rotor by π/2.  Subtract PLL_ANGLE_OFFSET to get rotor
     * angle, then add the empirical AN_SMC_THETA_OFFSET (BASE + K·ω)
     * which compensates LPF group delay in (Eα,Eβ).
     *
     * pll.omega_est is a clean instantaneous speed (no IRP averaging
     * latency) — feeds OmegaFltred for speed PI and Kslf scaling. */
    pll_update(&s->pll, s->EalphaFinal, s->EbetaFinal, AN_TS);

    {
        float dyn_offset = AN_SMC_THETA_OFFSET_BASE
                         + AN_SMC_THETA_OFFSET_K * an_abs(s->pll.omega_est);
        s->Theta = s->pll.theta_est - PLL_ANGLE_OFFSET + dyn_offset;
        s->Theta = an_wrap_2pi(s->Theta);
    }

    /* Keep ΔTheta accumulator running for telemetry / IRP-averaged
     * Omega field, but the live speed feedback comes from PLL. */
    {
        float dth = an_wrap_delta(s->Theta - s->PrevTheta);
        s->AccumTheta += dth;
        s->PrevTheta = s->Theta;
    }
    s->AccumThetaCnt++;
    if (s->AccumThetaCnt >= AN_IRP_PERCALC) {
        s->Omega = s->AccumTheta / ((float)AN_IRP_PERCALC * AN_TS);
        s->AccumTheta = 0.0f;
        s->AccumThetaCnt = 0;
    }

    /* OmegaFltred = PLL output directly.  PLL already provides smooth
     * speed estimate; no additional LPF needed.  Used by Kslf scaling
     * and (in motor.c) by the speed PI when CL is active. */
    s->OmegaFltred = s->pll.omega_est;

    /* ── Speed-adaptive LPF coefficient.
     *    Kslf = |OmegaFltred| × KslfScale, floored at KslfMin, capped
     *    at AN_SMC_KSLF_MAX so LPF doesn't degenerate to passthrough
     *    (Kslf=1 means y[n] = x[n] = no filtering — observer breaks). */
    {
        float o = an_abs(s->OmegaFltred);
        float k = o * s->KslfScale;
        if (k < s->KslfMin)      k = s->KslfMin;
        if (k > AN_SMC_KSLF_MAX) k = AN_SMC_KSLF_MAX;
        s->Kslf      = k;
        s->KslfFinal = k;
    }
}
