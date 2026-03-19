/**
 * @file hal_clc.h
 * @brief CLC (Configurable Logic Cell) HAL for hardware BEMF blanking.
 *
 * Uses CLC1/2/3 in D-Flip-Flop mode to sample the ATA6847 BEMF comparator
 * outputs at mid-PWM (center of ON-time), eliminating all PWM switching noise.
 *
 * Signal chain:
 *   BEMF_A (RC6/RP54) → CLCINA → CLC1 D-FF → CLC1OUT → RPV0 → SCCP1 IC
 *   BEMF_B (RC7/RP55) → CLCINB → CLC2 D-FF → CLC2OUT → RPV1 → SCCP2 IC
 *   BEMF_C (RD10/RP74) → CLCINC → CLC3 D-FF → CLC3OUT → RPV2 → SCCP3 IC
 *
 * Clock: PWM Event A (ADC Trigger 1 from PG1TRIGA=0 = center of ON-time).
 * The D-FF samples the comparator once per PWM cycle during PWM-ON, so the
 * output has at most one clean edge per zero crossing — no PWM noise edges.
 *
 * Max ZC timing jitter: ±1 PWM period (50µs at 20kHz).
 *
 * Enabled by FEATURE_CLC_BLANKING=1 in garuda_config.h.
 * Requires FEATURE_IC_ZC=1.
 */

#ifndef HAL_CLC_H
#define HAL_CLC_H

#include "../garuda_config.h"

#if FEATURE_IC_ZC && FEATURE_CLC_BLANKING

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize CLC1/2/3 as D-Flip-Flops for BEMF blanking.
 * Must be called AFTER HAL_PWM_Init() (needs PWMEVTA configured).
 * PPS routing for CLCINA/B/C and CLC→IC is done in port_config.c.
 */
void HAL_CLC_Init(void);

/**
 * @brief Read the current CLC output state for a BEMF phase.
 * @param channel 0=CLC1(BEMF_A), 1=CLC2(BEMF_B), 2=CLC3(BEMF_C)
 * @return 1 if CLC output is HIGH, 0 if LOW
 */
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

/**
 * @brief Force CLC D-FF to pre-ZC state by asserting async reset or set.
 *
 * After commutation, the CLC D-FF may hold a stale output from when the
 * phase was driven (not floating). This forces the output to the state
 * BEFORE the expected ZC transition, so the IC will see a clean edge
 * when the actual ZC occurs.
 *
 * @param channel 0=CLC1, 1=CLC2, 2=CLC3
 * @param risingZc true if expecting rising ZC (force LOW), false for falling (force HIGH)
 */
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
        /* Force Q=0 (LOW) so IC catches the LOW→HIGH rising edge.
         * G3POL=1 inverts Gate3 output: gate has no inputs selected so
         * base output is 0, inverted to 1, asserting Reset (Q→0).
         * Reset is dominant over Set. */
        *conh |= (1u << 2);   /* G3POL = 1 → R asserted → Q=0 */
        __asm__ volatile ("nop");
        __asm__ volatile ("nop");
        *conh &= ~(1u << 2);  /* G3POL = 0 → R released */
    }
    else
    {
        /* Force Q=1 (HIGH) so IC catches the HIGH→LOW falling edge.
         * G4POL=1 inverts Gate4 output, asserting Set (Q→1).
         * Must ensure R is NOT asserted (G3POL=0) since R is dominant. */
        *conh |= (1u << 3);   /* G4POL = 1 → S asserted → Q=1 */
        __asm__ volatile ("nop");
        __asm__ volatile ("nop");
        *conh &= ~(1u << 3);  /* G4POL = 0 → S released */
    }
}

#endif /* FEATURE_IC_ZC && FEATURE_CLC_BLANKING */
#endif /* HAL_CLC_H */
