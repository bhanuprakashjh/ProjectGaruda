/**
 * @file  fwc.c
 * @brief Field Weakening Control — implementation.
 */
#include "../garuda_config.h"

#if FEATURE_FWC

#include "fwc.h"
#include <math.h>

void FWC_Init(FWC_T *fwc, float kp, float ki, float angle_fw_max)
{
    fwc->pi.kp         = kp;
    fwc->pi.ki         = ki;
    fwc->pi.kc         = 0.999f;        /* match AN1078 PI anti-windup */
    fwc->pi.outMin     = 0.0f;          /* never negative — that'd be field-strengthening */
    fwc->pi.outMax     = angle_fw_max;  /* upper clamp on FW depth */
    fwc->pi.integrator = 0.0f;
    fwc->angle_fw      = 0.0f;
    fwc->angle_fw_max  = angle_fw_max;
    fwc->enabled       = false;
}

void FWC_Reset(FWC_T *fwc)
{
    fwc->pi.integrator = 0.0f;
    fwc->angle_fw      = 0.0f;
}

float FWC_RunAngle(FWC_T *fwc, float vs, float vsRef, float dt)
{
    if (!fwc->enabled) {
        /* Disabled — pin at no-FW, drain integrator so re-enable is clean. */
        fwc->pi.integrator = 0.0f;
        fwc->angle_fw      = 0.0f;
        return 0.0f;
    }

    /* an_pi_run computes err = ref - meas = Vs - VsRef.
     *  Vs > VsRef → err > 0 → PI output climbs → angle_fw climbs → FW engages.
     *  Vs < VsRef → err < 0 → integrator drains → angle_fw → 0 → no FW.
     * Output clamped to [0, angle_fw_max] by the PI's outMin/outMax. */
    fwc->angle_fw = an_pi_run(&fwc->pi, vs, vsRef, dt);
    return fwc->angle_fw;
}

#endif /* FEATURE_FWC */
