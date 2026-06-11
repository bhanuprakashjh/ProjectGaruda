/**
 * sil/stubs.c — minimal no-op stubs for firmware symbols referenced by the
 * compiled set but not part of the SIL behavioral surface.
 *
 * Every stub here is recorded in NOTES.md.
 */
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "garuda_config.h"

/* ── RX input (input/rx_decode.h) ────────────────────────────────────────
 * FEATURE_RX_* are =1 in the active config, so the ADC ISR's throttle mux
 * compiles the RX cases and references these cached values. The SIL uses
 * THROTTLE_SRC_ADC (pot) — keep RX permanently unlocked/zero. */
volatile uint16_t rxCachedThrottleAdc = 0;
volatile bool rxCachedLocked = false;

/* ── burst scope (scope/scope_burst.h) ───────────────────────────────────
 * FEATURE_BURST_SCOPE=1: the ADC ISR streams one SCOPE_SAMPLE_T per tick.
 * SIL discards them (telemetry comes from garudaData directly). */
#if FEATURE_BURST_SCOPE
#include "scope/scope_burst.h"
void Scope_Init(void) { }
void Scope_WriteSample(const SCOPE_SAMPLE_T *s) { (void)s; }
#endif

/* ── EEPROM (hal/eeprom.h) ────────────────────────────────────────────────
 * Only main.c touches EEPROM_* (not compiled in SIL). Provide RAM no-ops in
 * case a future compile-set addition links them. Load reports failure so
 * code defaults always win. */
typedef struct { uint8_t dummy; } SIL_EEPROM_OPAQUE;
void EEPROM_Init(void *img) { (void)img; }
bool EEPROM_LoadConfig(void *cfg) { (void)cfg; return false; }
bool EEPROM_SaveConfig(const void *cfg) { (void)cfg; return false; }
