/**
 * @file hal_spi.c
 * @brief SPI1 driver for dsPIC33CK — ATA6847 communication.
 *
 * SPI1 in 16-bit master mode, CKP=1 (idle high), CKE=1 (active→idle).
 * BRGL=14 → ~6.67 MHz SPI clock.
 * nCS managed by caller (hal_ata6847.c).
 */

#include <xc.h>
#include "hal_spi.h"
#include "port_config.h"

void HAL_SPI_Init(void)
{
    SPI1CON1H = 0x00;
    SPI1CON2L = 0x00;
    SPI1STATL = 0x00;
    SPI1BRGL = 0x0E;       /* Baud = FOSC/2 / (BRGL+1) = 100/15 ≈ 6.67 MHz */
    SPI1IMSKL = 0x00;
    SPI1IMSKH = 0x00;
    SPI1URDTL = 0x00;
    SPI1URDTH = 0x00;

    /* SPIEN=1, MODE16=1, MSTEN=1, CKP=1, CKE=1, SMP=End, ENHBUF=1 */
    SPI1CON1L = 0x8621;
}

uint16_t HAL_SPI_Exchange16(uint16_t data)
{
    while (SPI1STATLbits.SPITBF);
    nCS_Enable();
    SPI1BUFL = data;
    while (SPI1STATLbits.SPIRBE);
    nCS_Disable();
    return SPI1BUFL;
}
