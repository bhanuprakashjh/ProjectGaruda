/**
 * @file hal_ptg.h
 * @brief V5.0 Peripheral Trigger Generator for per-sector BEMF sampling.
 *
 * The PTG is a core-independent state machine. We use it to fire an
 * interrupt at a hardware-exact delay from each PWM trigger so the BEMF
 * comparator GPIO can be sampled at an offset that software polling
 * (SCCP1, ADC ISR) cannot reach. Zero CPU jitter.
 *
 * Gate: FEATURE_V5_PTG_ZC. When 0, every function is a no-op and the
 * PTG module stays off. When 1, HAL_PTG_Start runs the queue and
 * _PTG0Interrupt fires at (valley + PTGT0LIM ticks) every PWM cycle.
 *
 * A prior V3-era attempt lived in this same file with wrong opcodes
 * (confirmed against DS70005349 Table 24-1). It has been removed;
 * the V3 PTG path was never proven on bench.
 */

#ifndef HAL_PTG_H
#define HAL_PTG_H

#include "../garuda_config.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if FEATURE_V5_PTG_ZC

/** Initialise PTG to a stopped, known-safe state. Idempotent. */
void     HAL_PTG_Init(void);

/** Load the step queue and start the PTG state machine. */
void     HAL_PTG_Start(void);

/** Stop the PTG state machine. Register values are preserved. */
void     HAL_PTG_Stop(void);

/** Update the trigger-to-sample delay (PTG ticks, ~10 ns each at default).
 *  Safe while running — the new value is read on the next step iteration. */
void     HAL_PTG_SetDelay(uint16_t ptgTicks);

/** Raw fire counter — incremented unconditionally in _PTG0Interrupt.
 *  First sanity check that PTG is running at all. */
extern volatile uint32_t v5_ptgFires;

#else  /* !FEATURE_V5_PTG_ZC — fold calls away at compile time */

static inline void HAL_PTG_Init(void)                { }
static inline void HAL_PTG_Start(void)               { }
static inline void HAL_PTG_Stop(void)                { }
static inline void HAL_PTG_SetDelay(uint16_t t)      { (void)t; }

#endif

#ifdef __cplusplus
}
#endif

#endif /* HAL_PTG_H */
