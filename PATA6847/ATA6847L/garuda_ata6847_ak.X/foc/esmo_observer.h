/**
 * @file  esmo_observer.h
 * @brief Enhanced SMO observer — drop-in replacement for AN1078 SMC.
 *
 * Port of TI C2000 motor-control-sdk ESMO (libraries/observers/esmo/).
 * Reuses AN_SMC_T struct for binary-compatible substitution; all
 * direct field reads in an1078_motor.c (m->smc.OmegaFltred,
 * m->smc.EalphaFinal, m->smc.Theta, etc.) work unchanged.
 *
 * Differences vs AN1078 (built incrementally — see Phase 1 task list):
 *
 *   1. PLL discriminator: Ed/|E| projection with Eq-sign flip
 *      (full-quadrant pull-in) replaces cross-product
 *      Eβ·cos − Eα·sin (1st-quadrant only).
 *
 *   2. Adaptive Kp PLL: Kp = clamp(KpMin + KpSF·|ωref|, KpMin, KpMax).
 *      Constant gain at the upper end, but slewed up from a soft
 *      start so the PLL stays stable at low BEMF.
 *
 *   3. Closed-form theta offset: thetaOffset = atan2(speedRef·offsetSF, Kslf).
 *      Replaces the empirical BASE + K·|ω| linear model — the atan2
 *      form is the exact phase lag of the 1st-order LPF.
 *
 *   4. Adaptive Kslide: ramps from KslideMin to KslideMax over startup
 *      (TI uses +0.000002f per tick). Low Kslide at startup avoids Z
 *      chattering when BEMF is small; high Kslide at speed sharpens
 *      tracking.
 *
 *   5. Speed LPF after PLL: omega_est = lpf_b0·pll_Out + lpf_a1·omega_est.
 *      First-order LPF before omega feeds theta integration. Smoother
 *      angle, less speed jitter into the speed loop.
 *
 * For step 0 (skeleton), the math is identical to AN1078 — these are
 * stubs that delegate, just to prove the plumbing. Each subsequent
 * task swaps one piece (Ed/|E|, then adaptive Kp, …).
 */
#ifndef ESMO_OBSERVER_H
#define ESMO_OBSERVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "an1078_smc.h"   /* AN_SMC_T type — shared between AN1078 and ESMO */

/** Initialize ESMO state (Fsmopos, Gsmopos from motor params + ESMO
 *  extras: KpMin/Max, KslideMin/Max, offsetSF, lpf coefs). Same call
 *  signature as AN_SMCInit so the runtime can switch via macro. */
void ESMO_Init(AN_SMC_T *s);

/** Zero estimator state (preserves tuning). Same as AN_SMCReset. */
void ESMO_Reset(AN_SMC_T *s);

/** One-tick observer update. Caller fills Valpha, Vbeta, Ialpha, Ibeta
 *  before calling. Updates EstI*, IerrorErr*, Z*, E*, EFinal*, Theta,
 *  Omega, OmegaFltred. Same contract as AN_SMC_Position_Estimation. */
void ESMO_Update(AN_SMC_T *s);

#ifdef __cplusplus
}
#endif

#endif /* ESMO_OBSERVER_H */
