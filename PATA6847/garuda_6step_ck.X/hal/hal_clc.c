/**
 * @file hal_clc.c
 * @brief CLC D-Flip-Flop BEMF blanking implementation.
 *
 * CLC D-FF (MODE=100) gate assignments:
 *   Gate 1 = CLK  (rising edge triggers sampling)
 *   Gate 2 = D    (data input = BEMF comparator output)
 *   Gate 3 = R    (async reset — disabled, held low)
 *   Gate 4 = S    (async set — disabled, held low)
 *
 * CLC input MUX (CLCxSEL) data source assignments:
 *   DS1 = CLCINA (PPS-mapped to BEMF_A) — used by CLC1 Gate2
 *   DS2 = CLCINB (PPS-mapped to BEMF_B) — used by CLC2 Gate2
 *   DS3 = CLCINC (PPS-mapped to BEMF_C) — used by CLC3 Gate2
 *   DS4 = PWM Event A (internal, no PPS) — used by all CLCs Gate1
 *
 * CLCxGLSL bit layout (Gate Logic Select Low):
 *   Bits 7:0  = Gate 1: G1D1N(0) G1D1T(1) G1D2N(2) G1D2T(3)
 *                        G1D3N(4) G1D3T(5) G1D4N(6) G1D4T(7)
 *   Bits 15:8 = Gate 2: G2D1N(8) G2D1T(9) G2D2N(10) G2D2T(11)
 *                        G2D3N(12) G2D3T(13) G2D4N(14) G2D4T(15)
 *
 * CLCxGLSH: Gate 3 and Gate 4 — all bits zero (R and S disabled).
 */

#include "hal_clc.h"

#if FEATURE_IC_ZC && FEATURE_CLC_BLANKING

#include <xc.h>

void HAL_CLC_Init(void)
{
    /* ── CLC1: BEMF_A — D from CLCINA (DS1), CLK from PWM Event A (DS4) ── */
    CLC1CONL = 0;                       /* Disable while configuring */
    CLC1CONH = 0x0000;                  /* No gate polarity inversions */
    CLC1SEL  = (0b000u << 12)           /* DS4 = PWM Event A */
             | (0b000u << 8)            /* DS3 = don't care */
             | (0b000u << 4)            /* DS2 = don't care */
             | (0b000u);               /* DS1 = CLCINA (BEMF_A) */
    CLC1GLSL = (1u << 7)               /* G1D4T: Gate1 CLK = DS4 true (PWM Event A) */
             | (1u << 9);              /* G2D1T: Gate2 D   = DS1 true (CLCINA = BEMF_A) */
    CLC1GLSH = 0x0000;                  /* Gate3 R = 0 (inactive), Gate4 S = 0 (inactive) */
    CLC1CONL = (1u << 15)              /* LCEN = 1: enable module */
             | (0b100u);               /* MODE = 100: 1-input D-FF with S and R */

    /* ── CLC2: BEMF_B — D from CLCINB (DS2), CLK from PWM Event A (DS4) ── */
    CLC2CONL = 0;
    CLC2CONH = 0x0000;
    CLC2SEL  = (0b000u << 12)           /* DS4 = PWM Event A */
             | (0b000u << 8)
             | (0b000u << 4)            /* DS2 = CLCINB (BEMF_B) */
             | (0b000u);
    CLC2GLSL = (1u << 7)               /* G1D4T: Gate1 CLK = DS4 true (PWM Event A) */
             | (1u << 11);             /* G2D2T: Gate2 D   = DS2 true (CLCINB = BEMF_B) */
    CLC2GLSH = 0x0000;
    CLC2CONL = (1u << 15)
             | (0b100u);

    /* ── CLC3: BEMF_C — D from CLCINC (DS3), CLK from PWM Event A (DS4) ── */
    CLC3CONL = 0;
    CLC3CONH = 0x0000;
    CLC3SEL  = (0b000u << 12)           /* DS4 = PWM Event A */
             | (0b000u << 8)            /* DS3 = CLCINC (BEMF_C) */
             | (0b000u << 4)
             | (0b000u);
    CLC3GLSL = (1u << 7)               /* G1D4T: Gate1 CLK = DS4 true (PWM Event A) */
             | (1u << 13);             /* G2D3T: Gate2 D   = DS3 true (CLCINC = BEMF_C) */
    CLC3GLSH = 0x0000;
    CLC3CONL = (1u << 15)
             | (0b100u);
}

#endif /* FEATURE_IC_ZC && FEATURE_CLC_BLANKING */
