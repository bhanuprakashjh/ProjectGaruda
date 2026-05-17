/**
 * @file hal_ptg.h
 * @brief Peripheral Trigger Generator — PTG-driven BEMF detection (AK port).
 *
 * _PTG0Interrupt fires at every PG1TRIGB match (PWM mid-OFF or mid-ON,
 * selected per-fire by the duty-adaptive sampler in hal_ptg.c) and calls
 * ProcessBemfSample() for BEMF GPIO read + sector-PI update.
 *
 * PTG input wiring on dsPIC33AK128MC106 (DS70005539 Table 26-2):
 *   PTG Trigger Input 0 = PWM1 ADC Trigger 2  (= PG1TRIGB match).
 * Hardwired — no PPS routing.
 *
 * Step queue (PTGQUE0):
 *   STEP0: PTGWHI | 0      wait for PTG Input 0 rising edge
 *   STEP1: PTGIRQ | 0      generate _PTG0Interrupt
 *   STEP2: PTGJMP | 0      loop back to STEP0
 */
#ifndef HAL_PTG_H
#define HAL_PTG_H

#include <stdint.h>

/* Heartbeat / fire counter — exposed via the snapshot for host diagnostics. */
extern volatile uint32_t ptgFires;

/* Per-sector expected post-ZC comparator state. Written by Commutate
 * (sector_pi.c) when POST_ZC_ACCEPT is enabled. Read in the BEMF ISR
 * paths to classify accept vs reject. */
extern volatile uint8_t  ptgExpectedComp;

/* PTG postscaler skipped-fire count — gated fires that bypass
 * ProcessBemfSample() (see PTG_POSTSCALE_N in garuda_config.h). */
extern volatile uint32_t ptgSkipped;

void HAL_PTG_Init(void);
void HAL_PTG_Start(void);
void HAL_PTG_Stop(void);

#endif /* HAL_PTG_H */
