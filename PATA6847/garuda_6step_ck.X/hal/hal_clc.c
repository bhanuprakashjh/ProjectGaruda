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

#if FEATURE_IC_ZC && FEATURE_CLC_BLANKING && !FEATURE_V4_SECTOR_PI

#include <xc.h>

void HAL_CLC_Init(void)
{
    /* CLC1: BEMF_A — D from CLCINA (DS1)
     * PTG mode: no clock (PTG ISR updates via R/S at edge-relative timing)
     * Normal mode: CLK from PWM Event A (DS4, fires at PG1TRIGA=0) */
    CLC1CONL = 0;
    CLC1CONH = 0x0000;
    CLC1SEL  = (0b000u << 12)          /* DS4 = PWM Event A */
             | (0b000u << 8)
             | (0b000u << 4)
             | (0b000u);              /* DS1 = CLCINA (BEMF_A) */
#if FEATURE_PTG_ZC
    /* PTG controls CLC timing via R/S. Disable PWMEVTA clock so they
     * don't fight — PWMEVTA (counter-relative) and PTG (edge-relative)
     * would race, causing CLC to toggle → flip storm → false ZCs. */
    CLC1GLSL = (1u << 9);             /* G2D1T only: D = BEMF_A. No clock. */
#else
    CLC1GLSL = (1u << 7)              /* G1D4N: Gate1 CLK = !DS4 (!PWM Event A) */
             | (1u << 9);             /* G2D1N: Gate2 D = !DS1 (!CLCINA = !BEMF_A) */
#endif
    CLC1GLSH = 0x0000;
    CLC1CONL = (1u << 15) | (0b100u); /* LCEN=1, MODE=D-FF (edge-triggered).
                                       * Was MODE=100 (D-FF, edge-triggered):
                                       * sampled once per PWM cycle (24kHz) → only
                                       * 1.3 updates in detection window at 85k+ eRPM
                                       * → speed bottleneck causing timeouts.
                                       * MODE=111 (D-Latch): transparent when CLK=1,
                                       * tracks comparator in real-time. Poll reads
                                       * at 210.5kHz → no bottleneck at any speed.
                                       * Noise handled by blanking + 50% rejection
                                       * + deglitch filter. */

    /* CLC2: BEMF_B — D from CLCINB (DS2) */
    CLC2CONL = 0;
    CLC2CONH = 0x0000;
    CLC2SEL  = (0b000u << 12)
             | (0b000u << 8)
             | (0b000u << 4)
             | (0b000u);
#if FEATURE_PTG_ZC
    CLC2GLSL = (1u << 11);
#else
    CLC2GLSL = (1u << 7)
             | (1u << 11);
#endif
    CLC2GLSH = 0x0000;
    CLC2CONL = (1u << 15) | (0b100u);

    /* CLC3: BEMF_C — D from CLCINC (DS3) */
    CLC3CONL = 0;
    CLC3CONH = 0x0000;
    CLC3SEL  = (0b000u << 12)
             | (0b000u << 8)
             | (0b000u << 4)
             | (0b000u);
#if FEATURE_PTG_ZC
    CLC3GLSL = (1u << 13);
#else
    CLC3GLSL = (1u << 7)
             | (1u << 13);
#endif
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

#endif /* FEATURE_IC_ZC && FEATURE_CLC_BLANKING && !FEATURE_V4_SECTOR_PI */
