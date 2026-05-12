/**
 * @file hal_ptg.c
 * @brief PTG variable definitions (AK port stub).
 *
 * Real PTG init/start/stop are no-ops in `hal_ptg.h`. This file holds
 * the shared variable `v5_ptgExpectedComp` so the V4 sector PI can
 * link without compiling the full PTG state machine.
 */
#include <stdint.h>

/* Expected post-ZC comparator state, written by Commutate per sector,
 * read by ADC ISR. 0 for rising sectors (BEMF crosses neutral going UP
 * → comp goes 1→0 on the inverted ATA6847L), 1 for falling sectors. */
volatile uint8_t v5_ptgExpectedComp = 0;
