/**
 * @file  fwc.h
 * @brief Field Weakening Control — voltage-margin PI on FW depth angle.
 *
 * Concept (clean convention — not TI's "π/2 - pi_out" form which is
 * ambiguous about sign):
 *
 *   angle_fw ∈ [0, angle_fw_max]   — FW rotation amount, 0 = no FW
 *   Is_ref                         — total stator current magnitude
 *   Id_ref = -Is_ref · sin(angle_fw)   (negative for FW)
 *   Iq_ref = +Is_ref · cos(angle_fw)   (≤ Is_ref, decreases as FW deepens)
 *
 *   At angle_fw = 0:    Id_ref = 0,           Iq_ref = Is_ref      (no FW)
 *   At angle_fw = π/4:  Id_ref = -0.71·Is,    Iq_ref = +0.71·Is    (~50%)
 *   At angle_fw = π/3:  Id_ref = -0.87·Is,    Iq_ref = +0.50·Is    (heavy)
 *
 * PI runs on voltage margin:
 *   err = |Vs| - VsRef             (positive when over-voltage)
 *   angle_fw = clamp(Kp·err + ∫Ki·err, 0, angle_fw_max)
 *
 * Default behavior:
 *   Vs < VsRef:  err < 0 → integrator decays → angle_fw → 0 → no FW
 *   Vs > VsRef:  err > 0 → integrator rises → angle_fw climbs → FW engages
 *   Steady state at Vs ≈ VsRef:  PI integrator holds whatever angle_fw
 *                                keeps Vs at the threshold.
 *
 * Defaulted to DISABLED — flip FEATURE_FWC=1 to compile in.  Disabled
 * mode returns angle_fw = 0 (no FW) so wiring code can call it
 * unconditionally.
 */
#ifndef FWC_H
#define FWC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "../garuda_config.h"

#if FEATURE_FWC

#include "foc_pi.h"        /* AN_PI_T + an_pi_run() */
#include <stdbool.h>
#include <math.h>          /* sinf, cosf for FWC_SplitCurrent */

typedef struct {
    AN_PI_T pi;              /* PI on voltage margin */
    float   angle_fw;        /* last computed FW angle (rad), 0 = no FW */
    float   angle_fw_max;    /* upper clamp on angle_fw, rad */
    bool    enabled;
} FWC_T;

/** Initialize FWC.
 *  @param kp           Proportional gain (rad / V).
 *  @param ki           Integral gain (rad / V·s).
 *  @param angle_fw_max Max FW angle (rad). π/4 (45°) is a reasonable
 *                      starting point; π/3 for deeper FW. */
void FWC_Init(FWC_T *fwc, float kp, float ki, float angle_fw_max);

/** Enable / disable. */
static inline void FWC_Enable (FWC_T *fwc) { fwc->enabled = true;  }
static inline void FWC_Disable(FWC_T *fwc) { fwc->enabled = false; }

/** Reset integrator to zero. Call at motor stop / fault. */
void FWC_Reset(FWC_T *fwc);

/** Run FWC controller.
 *  @param vs     |Vs| this tick (V).
 *  @param vsRef  Max allowed |Vs| (V).  Typically 0.95 · Vbus / √3.
 *  @param dt     Tick period (s).
 *  @return       FW angle in rad, in [0, angle_fw_max].  Use:
 *                  Id_ref = -Is_ref · sinf(angle)
 *                  Iq_ref = +Is_ref · cosf(angle) */
float FWC_RunAngle(FWC_T *fwc, float vs, float vsRef, float dt);

/** Split a stator current command into (Id_ref, Iq_ref) using the last
 *  computed angle.  Call this after FWC_RunAngle. */
static inline void FWC_SplitCurrent(const FWC_T *fwc, float Is_ref,
                                    float *Id_ref, float *Iq_ref)
{
    *Id_ref = -Is_ref * sinf(fwc->angle_fw);
    *Iq_ref = +Is_ref * cosf(fwc->angle_fw);
}

#endif /* FEATURE_FWC */

#ifdef __cplusplus
}
#endif

#endif /* FWC_H */
