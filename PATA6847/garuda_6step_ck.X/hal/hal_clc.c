/**
 * @file hal_clc.c
 * @brief CLC D-Flip-Flop BEMF noise filter — 3 independent CLCs.
 *
 * CLC1/2/3 each sample one BEMF phase at PWM mid-ON, hold between clocks.
 * Always running — no per-step reconfiguration needed.
 * Poll reads the CLC output for the current floating phase.
 *
 * Gate assignments (D-FF MODE=100):
 *   Gate1 = CLK  (PWM Event A = mid-ON)
 *   Gate2 = D    (BEMF comparator output)
 *   Gate3 = R    (async reset — for ForcePreZcState)
 *   Gate4 = S    (async set — for ForcePreZcState)
 */

#include "hal_clc.h"

#if FEATURE_IC_ZC && FEATURE_CLC_BLANKING

#include <xc.h>

void HAL_CLC_Init(void)
{
    /* CLC1: BEMF_A — D from CLCINA (DS1), CLK from PWM Event A (DS4) */
    CLC1CONL = 0;
    CLC1CONH = 0x0000;
    CLC1SEL  = (0b000u << 12)          /* DS4 = PWM Event A */
             | (0b000u << 8)
             | (0b000u << 4)
             | (0b000u);              /* DS1 = CLCINA (BEMF_A) */
    CLC1GLSL = (1u << 7)              /* G1D4T: Gate1 CLK = DS4 (PWM Event A) */
             | (1u << 9);             /* G2D1T: Gate2 D = DS1 (CLCINA = BEMF_A) */
    CLC1GLSH = 0x0000;
    CLC1CONL = (1u << 15) | (0b100u); /* LCEN=1, MODE=D-FF */

    /* CLC2: BEMF_B — D from CLCINB (DS2), CLK from PWM Event A (DS4) */
    CLC2CONL = 0;
    CLC2CONH = 0x0000;
    CLC2SEL  = (0b000u << 12)
             | (0b000u << 8)
             | (0b000u << 4)
             | (0b000u);
    CLC2GLSL = (1u << 7)              /* G1D4T: CLK = DS4 */
             | (1u << 11);            /* G2D2T: D = DS2 (CLCINB = BEMF_B) */
    CLC2GLSH = 0x0000;
    CLC2CONL = (1u << 15) | (0b100u);

    /* CLC3: BEMF_C — D from CLCINC (DS3), CLK from PWM Event A (DS4) */
    CLC3CONL = 0;
    CLC3CONH = 0x0000;
    CLC3SEL  = (0b000u << 12)
             | (0b000u << 8)
             | (0b000u << 4)
             | (0b000u);
    CLC3GLSL = (1u << 7)              /* G1D4T: CLK = DS4 */
             | (1u << 13);            /* G2D3T: D = DS3 (CLCINC = BEMF_C) */
    CLC3GLSH = 0x0000;
    CLC3CONL = (1u << 15) | (0b100u);
}

void HAL_CLC_ConfigureStep(uint8_t floatingPhase, uint8_t pwmPhase)
{
    /* 3-CLC approach: all always running, no reconfiguration needed.
     * ReadBEMFComparator selects the right CLC output per phase. */
    (void)floatingPhase;
    (void)pwmPhase;
}

#endif /* FEATURE_IC_ZC && FEATURE_CLC_BLANKING */
