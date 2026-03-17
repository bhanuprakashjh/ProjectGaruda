/**
 * @file hal_opa.c
 * @brief Internal op-amp initialization for current sensing.
 *
 * EV43F54A current sensing topology (from schematic sheets 2, 3, 8):
 *   OA1: DC bus current (IBus) — ATA6847 OPO3 (Gt=8) → OA1 → AN0
 *   OA2: Phase A current (IS1) — shunt RS1 (3mΩ) → OA2 (Gt=16) → AN1
 *   OA3: Phase B current (IS2) — shunt RS2 (3mΩ) → OA3 (Gt=16) → AN4
 */

#include <xc.h>
#include "hal_opa.h"

void HAL_OPA_Init(void)
{
    /* AMPCON1L: Enable OA2 and OA3 for phase current sensing.
     * OA1 is NOT used: R46 (12k) feedback with the Vbus divider on
     * OA1IN- creates ~7000x gain, saturating the ADC. The OA1 circuit
     * was designed for the ATA6847 external op-amp topology which
     * doesn't map to a simple dsPIC OA1 non-inverting amp.
     * IBus is computed from phase currents instead: the active PWM
     * phase carries the full DC bus current.
     *
     * Bit 1: AMPEN2 (OA2 — Phase A, IS1, Gt=16)
     * Bit 2: AMPEN3 (OA3 — Phase B, IS2, Gt=16)
     * Bit 15: AMPON (master enable, set separately) */
    AMPCON1L = (0x8006 & 0x7FFF);  /* Configure but don't enable yet */

    /* AMPCON1H: Wide input voltage range for all channels */
    AMPCON1H = 0x00;
}

void HAL_OPA_Enable(void)
{
    AMPCON1Lbits.AMPON = 1;
}

void HAL_OPA_Disable(void)
{
    AMPCON1Lbits.AMPON = 0;
}
