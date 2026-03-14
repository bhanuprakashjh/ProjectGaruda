/**
 * @file clock.c
 * @brief dsPIC33CK256MP503 clock initialization.
 *
 * FRC (8 MHz) → PLL → FOSC = 200 MHz, FCY = 100 MHz.
 * Register values from EV43F54A reference (verified working).
 */

#include <xc.h>
#include "clock.h"

/* ── Configuration Bits ─────────────────────────────────────────────── */

/* FSEC */
#pragma config BWRP = OFF
#pragma config BSS = DISABLED
#pragma config BSEN = OFF
#pragma config GWRP = OFF
#pragma config GSS = DISABLED
#pragma config CWRP = OFF
#pragma config CSS = DISABLED
#pragma config AIVTDIS = OFF

/* FBSLIM */
#pragma config BSLIM = 8191

/* FOSCSEL */
#pragma config FNOSC = FRC
#pragma config IESO = OFF

/* FOSC */
#pragma config POSCMD = NONE
#pragma config OSCIOFNC = ON
#pragma config FCKSM = CSECME
#pragma config PLLKEN = ON
#pragma config XTCFG = G3
#pragma config XTBST = ENABLE

/* FWDT */
#pragma config RWDTPS = PS2147483648
#pragma config RCLKSEL = LPRC
#pragma config WINDIS = OFF
#pragma config WDTWIN = WIN25
#pragma config SWDTPS = PS2147483648
#pragma config FWDTEN = ON_SW

/* FPOR */
#pragma config BISTDIS = DISABLED

/* FICD */
#pragma config ICS = PGD3
#pragma config JTAGEN = OFF
#pragma config NOBTSWP = DISABLED

/* FDMTIVTL / FDMTIVTH */
#pragma config DMTIVTL = 0
#pragma config DMTIVTH = 0

/* FDMTCNTL / FDMTCNTH */
#pragma config DMTCNTL = 0
#pragma config DMTCNTH = 0

/* FDMT */
#pragma config DMTDIS = OFF

/* FDEVOPT */
#pragma config ALTI2C1 = OFF
#pragma config ALTI2C2 = OFF
#pragma config ALTI2C3 = OFF
#pragma config SMBEN = SMBUS
#pragma config SPI2PIN = PPS

/* FALTREG */
#pragma config CTXT1 = OFF
#pragma config CTXT2 = OFF
#pragma config CTXT3 = OFF
#pragma config CTXT4 = OFF

/* FBTSEQ */
#pragma config BSEQ = 4095
#pragma config IBSEQ = 4095

/* FBOOT */
#pragma config BTMODE = SINGLE


void CLOCK_Initialize(void)
{
    /* FRC → PLL: FOSC = FRC * PLLFBD / (PLLPRE * POST1 * POST2)
     * = 8 MHz * 50 / (1 * 1 * 1) = 400 MHz VCO, /2 = 200 MHz FOSC */
    CLKDIV = 0x3001;    /* FRCDIV FRC/1, PLLPRE 1, DOZE 1:8 */
    PLLFBD = 0x32;      /* PLLFBDIV 50 */
    OSCTUN = 0x00;
    PLLDIV = 0x311;     /* POST1DIV 1:1, POST2DIV 1:1, VCODIV FVCO */

    /* Auxiliary PLL (not used for motor, but configured per reference) */
    ACLKCON1 = 0x101;
    APLLFBD1 = 0x96;
    APLLDIV1 = 0x41;

    REFOCONL = 0x00;
    REFOCONH = 0x00;
    REFOTRIMH = 0x00;
    RPCON = 0x00;
    PMDCON = 0x00;

    /* Enable all peripheral modules */
    PMD1 = 0x00;
    PMD2 = 0x00;
    PMD3 = 0x00;
    PMD4 = 0x00;
    PMD6 = 0x00;
    PMD7 = 0x00;
    PMD8 = 0x00;

    /* Switch to FRCPLL */
    __builtin_write_OSCCONH((uint8_t)0x01);
    __builtin_write_OSCCONL((uint8_t)0x01);
    while (OSCCONbits.OSWEN != 0);
    while (OSCCONbits.LOCK  != 1);
}
