/**
 * @file hal_opa.h
 * @brief Op-amp HAL stub (AK port — 6-step ATA6847L does not use internal OAs).
 * Phase current sensing on EV92R69A goes through ATA6847L CSAs, not MCU OAs.
 * Kept as empty stub so files including it still compile.
 */
#ifndef HAL_OPA_H
#define HAL_OPA_H
static inline void HAL_OPA_Init(void) { }
static inline void HAL_OPA_Enable(void) { }
static inline void HAL_OPA_Disable(void) { }
#endif
