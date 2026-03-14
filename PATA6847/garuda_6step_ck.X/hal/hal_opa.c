/**
 * @file hal_opa.c
 * @brief Internal op-amp initialization for current sensing.
 *
 * Register values from EV43F54A reference firmware.
 * OPA2 and OPA3 are enabled for phase current amplification.
 * OPA1 is not used on EV43F54A board.
 */

#include <xc.h>
#include "hal_opa.h"

void HAL_OPA_Init(void)
{
    /* AMPCON1L: AMPEN2=1, AMPEN3=1, AMPON initially disabled
     * Reference: 0x8006 = AMPON=1, AMPEN3=1, AMPEN2=1 */
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
