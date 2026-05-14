/**
 * @file hal_ptg.h
 * @brief Peripheral Trigger Generator — PTG-driven BEMF detection (AK port).
 *
 * Phase 1 (current): heartbeat ISR.  PTG fires _PTG0Interrupt at every
 * PWM mid-OFF (PG1TRIGB match), ISR just increments v5_ptgFires.  This
 * proves the PTG → ADC-Trigger-2 → CPU path before any BEMF logic moves
 * into it.
 *
 * Phase 2: BEMF GPIO read + sector-PI update will move from the ADC ISR
 * into _PTG0Interrupt, decoupling BEMF detection from ADC scan
 * completion (~1.5 µs latency improvement at 60 kHz PWM).
 *
 * PTG input wiring on dsPIC33AK128MC106 (DS70005539 Table 26-2):
 *   PTG Trigger Input 0 = PWM1 ADC Trigger 2  (= PG1TRIGB match).
 * Hardwired — no PPS routing.  PG1TRIGB defaults to 0x00 in hal_pwm.c,
 * which on Double-Update Center-Aligned PWM (mode 4) means "period
 * boundary" = mid-OFF — the same instant the BEMF GPIO is stable today.
 *
 * Step queue (PTGQUE0):
 *   STEP0: PTGWHI | 0      wait for PTG Input 0 rising edge
 *   STEP1: PTGIRQ | 0      generate _PTG0Interrupt
 *   STEP2: PTGJMP | 0      loop back to STEP0
 */
#ifndef HAL_PTG_H
#define HAL_PTG_H

#include <stdint.h>

/* Heartbeat / fire counter — exposed via the existing v5_ptgFires
 * snapshot path (gated by FEATURE_V5_PTG_ZC, which Phase 1 turns on so
 * the host can see the count). */
extern volatile uint32_t v5_ptgFires;

/* Per-sector expected post-ZC comparator state. Written by Commutate
 * (sector_pi.c) when V5_POST_ZC_ACCEPT or V5_PTG_ZC is enabled.  Phase 2
 * will read this inside _PTG0Interrupt to classify accept vs reject. */
extern volatile uint8_t  v5_ptgExpectedComp;

/* Polarity-split accept/reject counters — defined here for Phase 2.
 * Phase 1 leaves them at zero. */
extern volatile uint32_t v5_ptgRisingAcc;
extern volatile uint32_t v5_ptgRisingRej;
extern volatile uint32_t v5_ptgFallingAcc;
extern volatile uint32_t v5_ptgFallingRej;

void HAL_PTG_Init(void);
void HAL_PTG_Start(void);
void HAL_PTG_Stop(void);

/* Placeholders kept for source compatibility with existing call sites.
 * Phase 2 will wire them to PTGT0LIM / PTGT1LIM if we add a delay step. */
static inline void HAL_PTG_SetPeakDelay(uint16_t ticks)   { (void)ticks; }
static inline void HAL_PTG_SetValleyDelay(uint16_t ticks) { (void)ticks; }

#endif /* HAL_PTG_H */
