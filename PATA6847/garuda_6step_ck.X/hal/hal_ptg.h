/**
 * @file hal_ptg.h
 * @brief PTG (Peripheral Trigger Generator) for edge-relative BEMF sampling.
 *
 * The CLC D-FF samples BEMF at PG1TRIGA=0 (counter-relative). At certain
 * duty ratios, this position lands near switching edges → noise.
 *
 * PTG fires an ISR at a FIXED DELAY after the switching edge, guaranteeing
 * a clean read regardless of duty ratio. This supplements the CLC path.
 *
 * Chain: PG1TRIGB (at duty value) → PTG WAIT → Timer0 delay → PTG0 ISR
 *        → reads raw GPIO → stores ptgSample for FastPoll to consume.
 */

#ifndef HAL_PTG_H
#define HAL_PTG_H

#include <xc.h>
#include "../garuda_config.h"

#if FEATURE_PTG_ZC

/**
 * @brief Initialize PTG registers and step queue.
 * Does NOT start execution — call HAL_PTG_Start() when entering CL.
 */
void HAL_PTG_Init(void);

/**
 * @brief Start PTG step queue execution.
 * Call when entering closed-loop.
 */
void HAL_PTG_Start(void);

/**
 * @brief Stop PTG and disable interrupt.
 * Call on motor stop, recovery, or fault.
 */
void HAL_PTG_Stop(void);

#endif /* FEATURE_PTG_ZC */
#endif /* HAL_PTG_H */
