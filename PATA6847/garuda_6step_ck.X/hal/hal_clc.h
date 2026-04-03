/**
 * @file hal_clc.h
 * @brief CLC D-Flip-Flop hardware BEMF noise filter.
 *
 * CLC1/2/3: one per phase, always running. Each D-FF samples its BEMF
 * at PWM mid-ON and holds the clean value. No per-step reconfiguration.
 */

#ifndef HAL_CLC_H
#define HAL_CLC_H

#include "../garuda_config.h"

#if FEATURE_IC_ZC && FEATURE_CLC_BLANKING

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

void HAL_CLC_Init(void);
void HAL_CLC_ConfigureStep(uint8_t floatingPhase, uint8_t pwmPhase);

static inline uint8_t HAL_CLC_ReadOutput(uint8_t channel)
{
    switch (channel)
    {
        case 0: return CLC1CONLbits.LCOUT ? 1 : 0;
        case 1: return CLC2CONLbits.LCOUT ? 1 : 0;
        case 2: return CLC3CONLbits.LCOUT ? 1 : 0;
        default: return 0;
    }
}

static inline void HAL_CLC_ForcePreZcState(uint8_t channel, bool risingZc)
{
    volatile uint16_t *conh;
    switch (channel)
    {
        case 0: conh = (volatile uint16_t *)&CLC1CONH; break;
        case 1: conh = (volatile uint16_t *)&CLC2CONH; break;
        case 2: conh = (volatile uint16_t *)&CLC3CONH; break;
        default: return;
    }
    if (risingZc)
    {
        *conh |= (1u << 2);   /* G3POL=1 → R asserted → Q=0 */
        __asm__ volatile ("nop");
        __asm__ volatile ("nop");
        *conh &= ~(1u << 2);
    }
    else
    {
        *conh |= (1u << 3);   /* G4POL=1 → S asserted → Q=1 */
        __asm__ volatile ("nop");
        __asm__ volatile ("nop");
        *conh &= ~(1u << 3);
    }
}

#endif
#endif
