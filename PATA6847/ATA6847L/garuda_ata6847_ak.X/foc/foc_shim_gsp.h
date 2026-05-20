/**
 * @file foc_shim_gsp.h
 * @brief Stub for the FOC pipeline's runtime tuning params.
 *
 * The AN1078 SMO sources are ported from dspic33AKESC, which has a GUI-
 * editable `gspParams` struct (V3 EEPROM schema) carrying motor R/L/λ
 * overrides and observer gains. The AK board's own gsp/gsp_params.h
 * uses a different scheme (`ckParams`) and doesn't have FOC fields.
 *
 * For the initial compile-only port we don't want to disturb the AK
 * param system or its EEPROM layout. This shim defines a minimal
 * `gspParams` global with the same field names the FOC sources expect,
 * all zeros. The SMC code reads each field through a `(v > 0) ? scaled
 * : compile_time_default` gate, so zeros activate the AN_MOTOR_RS /
 * AN_MOTOR_LS / AN_SMC_KSLIDE etc. fallbacks. Effect: the port runs
 * purely off the constants in an1078_params.h until someone wires
 * runtime tuning back in.
 *
 * Only built when FEATURE_FOC_AN1078 is enabled.
 */
#ifndef FOC_SHIM_GSP_H
#define FOC_SHIM_GSP_H

#include <stdint.h>

typedef struct {
    /* Observer tuning */
    uint16_t an1078ThetaBaseDegX10;   /* base theta offset, deg×10  */
    uint16_t an1078ThetaKE7;          /* speed-dependent K, ×1e7    */
    uint16_t an1078KslideMv;          /* SMC sliding gain, mV       */
    uint16_t an1078IdFwMaxDecia;      /* field-weak |Id| max, dA    */

    /* Motor plant — Rs, Ls, λ, Kv, max speed */
    uint16_t focRsMilliOhm;           /* phase Rs in mΩ             */
    uint16_t focLsMicroH;             /* phase Ls in µH             */
    uint16_t focKeUvSRad;             /* λ in µV·s/rad              */
    uint16_t focMaxElecRadS;          /* speed cap in elec rad/s    */
} AN1078_GspShim_T;

/* All-zero default → SMC code falls back to compile-time AN_MOTOR_RS /
 * AN_MOTOR_LS / AN_SMC_KSLIDE / etc. defined in an1078_params.h. */
static const AN1078_GspShim_T gspParams = { 0 };

/* From dspic33AKESC's garuda_foc_params.h. Pulled into the shim so the
 * FOC code doesn't have to depend on the source board's whole config
 * header. Values match the source board's MOTOR_PROFILE=2 (2810). */
#ifndef PLL_ANGLE_OFFSET
#define PLL_ANGLE_OFFSET   1.5707963f      /* π/2 = 90° */
#endif

#ifndef PLL_KP
#define PLL_KP             628.0f
#endif
#ifndef PLL_KI
#define PLL_KI             98600.0f
#endif
#ifndef PLL_SPEED_CLAMP
#define PLL_SPEED_CLAMP    25000.0f        /* ~239k eRPM elec rad/s, 2810 max */
#endif

#endif /* FOC_SHIM_GSP_H */
