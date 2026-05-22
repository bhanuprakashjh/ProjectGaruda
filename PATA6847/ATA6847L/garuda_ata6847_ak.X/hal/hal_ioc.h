/**
 * @file hal_ioc.h
 * @brief Change-Notification (Interrupt-on-Change) BEMF zero-crossing
 *        detection — experimental edge-triggered alternative to the
 *        PTG level-sample path.
 *
 * Only built when FEATURE_IOC_BEMF=1 in garuda_config.h. See
 * docs/ioc_bemf_detection_plan.md for the full architecture (five-layer
 * rejection: demag interval, static blanking, multi-sample consensus,
 * polarity gate, speed-adaptive widths).
 *
 * BEMF pin layout on the AK board (verified):
 *   BEMF_A  RB9   (Port B, bit 9)   — RP26
 *   BEMF_B  RB8   (Port B, bit 8)   — RP25
 *   BEMF_C  RA10  (Port A, bit 10)  — RP11
 *
 * Two ISR vectors: _CNAInterrupt for RA10, _CNBInterrupt for RB8/9.
 * At any sector only one port's _CNxIE is enabled.
 *
 * ATA6847L comparator output is INVERTED — rising-BEMF crosses as a
 * falling edge on the comp pin. The arming code translates between the
 * commutation table's zcPolarity (+1/-1, in BEMF terms) and the
 * physical edge (CNEN0=rising, CNEN1=falling).
 */
#ifndef HAL_IOC_H
#define HAL_IOC_H

#include "../garuda_config.h"

#if FEATURE_IOC_BEMF

#include <stdint.h>
#include <stdbool.h>

/* Public API ─────────────────────────────────────────────────────── */

/** One-time init at boot. Clears all CN enables on both ports, sets
 *  interrupt priority, leaves _CNAIE/_CNBIE disarmed. */
void HAL_Ioc_Init(void);

/** Disarm both ports — masks port-level IE and clears all per-pin
 *  enables. Called from SectorPI_Stop and from the IOC ISR after a
 *  successful capture. */
void HAL_Ioc_Disarm(void);

/** Per-sector arm. Reads commutationTable[sector] for floatingPhase
 *  + zcPolarity, enables CN on the right port + bit + edge, leaves
 *  the other port disabled. Called from SectorPI_Commutate. */
void HAL_Ioc_ArmForSector(uint8_t sector);

/* Diagnostic counters — incremented by the IOC ISR. Exposed for the
 * GSP snapshot / pot_capture.py decoder. */
extern volatile uint32_t iocAcceptCount;
extern volatile uint32_t iocDemagReject;
extern volatile uint32_t iocBlankReject;
extern volatile uint32_t iocFilterReject;
extern volatile uint32_t iocPolarityReject;
extern volatile uint32_t iocIntervalReject;   /* Layer 6 — short-ZC-interval reject */
extern volatile uint32_t iocPolUnanimous;     /* L4 subset: all-3 reads agreed wrong polarity
                                                 (real edge in wrong direction → sector mismatch
                                                 signal, distinct from split-vote comparator noise) */
extern volatile uint32_t iocFiresPortA;
extern volatile uint32_t iocFiresPortB;

#endif /* FEATURE_IOC_BEMF */
#endif /* HAL_IOC_H */
