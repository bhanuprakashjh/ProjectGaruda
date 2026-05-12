/**
 * @file hal_spi.c
 * @brief SPI driver for dsPIC33AK128MC106 — ATA6847L communication.
 *
 * Forked from CK `../../garuda_6step_ck.X/hal/hal_spi.c`. SFR layout on
 * dsPIC33AK matches dsPIC33CK for the SPI peripheral (SPIxCON1L/H,
 * SPIxSTATL, SPIxBUFL, SPIxBRGL, SPIxIMSKL/H), so the byte values
 * carry over unchanged.  Only the PPS routing in port_config.c differs.
 *
 * Peripheral choice (see ATA6847L+DIM evaluation board user guide
 * appendix A.3 + plan §4 pin map):
 *   SPI1 or SPI3 — DO NOT use SPI2, because its native SDI input
 *   shares the same DIM pin (40, AK RB11) as our nIRQ GPIO.
 *
 * Pin map (EV92R69A carrier ↔ EV68M17A DIM ↔ AK128MC106):
 *   nCS / GATE_DVR_ENABLE : DIM 6 or 8  → AK RC2/RP35 or RC5/RP38 (GPIO out)
 *   nIRQ                  : DIM 40      → AK RB11/RP28          (GPIO in + IOC)
 *   MOSI                  : DIM 42      → AK RC6/RP39           (PPS → SDOx)
 *   MISO                  : DIM 44      → AK RC7/RP40           (PPS → SDIx)
 *   SCK                   : DIM 46      → AK RC8/RP41           (PPS → SCKxOUT)
 *
 * SPI mode: 16-bit master, CKP=1 (idle high), CKE=1 (active→idle).
 * BRG=14 at AK SPI clock (Standard Speed Peripheral = 100 MHz):
 *   F_SCK = F_periph / (2 × (BRG+1)) = 100e6 / 30 = 3.33 MHz
 * ATA6847L SPI max clock is 10 MHz per DS50003904 §5.2 — 3.33 MHz
 * leaves wide margin and tolerates board parasitics on bring-up.
 * Lower BRG (e.g. 4 → 10 MHz) once signal integrity is verified.
 *
 * nCS is managed by caller (hal_ata6847.c) via nCS_Enable/Disable
 * macros declared in port_config.h.
 */

#include <xc.h>
#include "hal_spi.h"
#include "port_config.h"

void HAL_SPI_Init(void)
{
    /* AK SPI SFRs are single 32-bit (no L/H split like CK).  Bit fields
     * are accessed via `SPI1CON1bits.FIELDNAME`. */
    SPI1CON2 = 0x00000000;
    SPI1STAT = 0x00000000;
    SPI1BRG  = 14;       /* Baud = Fp / 2 / (BRG+1) = 100/30 ≈ 3.33 MHz
                          * (AK Fp from SPI clock chain — verify on bring-up) */
    SPI1IMSK = 0x00000000;
    SPI1URDT = 0x00000000;

    /* Match the EV94U98A_CK_x05 working ATA6847L driver SPI mode:
     * SPI1CON1L = 0x8621 → SPIEN, MODE16, SMP, MSTEN, ENHBUF (CKP=0, CKE=0).
     * AK port previously used CKP=1/CKE=1, which made the bit phase shift
     * so DSR1 read back 0x21 instead of the real 0x05 — VDD1OTPWS appeared
     * set when it wasn't and GDUS appeared clear when GDU was actually
     * ready. Bring the mode in line with the chip's requirement. */
    SPI1CON1bits.MODE16 = 1;   /* 16-bit transfer */
    SPI1CON1bits.MSTEN  = 1;   /* Host mode */
    SPI1CON1bits.CKP    = 0;   /* Clock idle LOW (ATA6847L) */
    SPI1CON1bits.CKE    = 0;   /* Output on idle→active (Microchip Mode 1) */
    SPI1CON1bits.SMP    = 1;   /* Sample at end */
    SPI1CON1bits.ENHBUF = 1;   /* Enhanced (FIFO) buffer */
    SPI1CON1bits.ON     = 1;   /* AK: ON (CK used SPIEN) */
}

uint16_t HAL_SPI_Exchange16(uint16_t data)
{
    volatile uint16_t timeout;

    timeout = 10000;
    while (SPI1STATbits.SPITBF && --timeout);
    if (!timeout) return 0xFFFF;

    nCS_Enable();
    SPI1BUF = data;

    timeout = 10000;
    while (SPI1STATbits.SPIRBE && --timeout);
    nCS_Disable();

    if (!timeout) return 0xFFFF;
    return (uint16_t)SPI1BUF;
}
