/**
 * @file port_config.c
 * @brief GPIO + PPS init for EV92R69A (ATA6847L) + EV68M17A (dsPIC33AK128MC106).
 *
 * Adapted from AK FOC sibling `../../dspic33AKESC/hal/port_config.c` and
 * CK source-of-truth `../../garuda_6step_ck.X/hal/port_config.c`.
 *
 * Pin assignments are documented inline.  TODO markers flag items that
 * still need probe-confirmation against the EV92R69A schematic.
 */

#include <xc.h>
#include "port_config.h"
#include "../garuda_config.h"

static void MapGPIOHWFunction(void);

void SetupGPIOPorts(void)
{
    /* Reset all ports — every pin defaults to digital input. */
    #ifdef TRISA
        TRISA = 0xFFFF;  LATA = 0x0000;
    #endif
    #ifdef ANSELA
        ANSELA = 0x0000;
    #endif
    #ifdef TRISB
        TRISB = 0xFFFF;  LATB = 0x0000;
    #endif
    #ifdef ANSELB
        ANSELB = 0x0000;
    #endif
    #ifdef TRISC
        TRISC = 0xFFFF;  LATC = 0x0000;
    #endif
    #ifdef ANSELC
        ANSELC = 0x0000;
    #endif
    #ifdef TRISD
        TRISD = 0xFFFF;  LATD = 0x0000;
    #endif
    #ifdef ANSELD
        ANSELD = 0x0000;
    #endif

    MapGPIOHWFunction();
}

static void MapGPIOHWFunction(void)
{
    /* ── Analog inputs (POT + VBUS) ──────────────────────────────── */
    /* POT  : DIM 28 → RA11 (AD1AN10).  Speed reference. */
    ANSELAbits.ANSELA11 = 1;
    TRISAbits.TRISA11   = 1;

    /* VBUS : DIM 39 → RA7  (AD1AN6).   DC bus voltage. */
    ANSELAbits.ANSELA7  = 1;
    TRISAbits.TRISA7    = 1;

    /* ── PWM outputs (PG1/PG2/PG3 complementary, 6-step) ─────────── */
    /* PWM1H : DIM 1 → RD2 (pin 53)   PWM1L : DIM 3 → RD3 (pin 54)
     * PWM2H : DIM 5 → RD0 (pin 51)   PWM2L : DIM 7 → RD1 (pin 52)
     * PWM3H : DIM 2 → RC3 (pin 43)   PWM3L : DIM 4 → RC4 (pin 44) */
    TRISDbits.TRISD2 = 0;  TRISDbits.TRISD3 = 0;
    TRISDbits.TRISD0 = 0;  TRISDbits.TRISD1 = 0;
    TRISCbits.TRISC3 = 0;  TRISCbits.TRISC4 = 0;

    /* ── LEDs ─────────────────────────────────────────────────────
     * RUN   : DIM 30 → RD5    FAULT : DIM 32 → RC9
     * (Inherits CK names LED_RUN/LED_FAULT via port_config.h.) */
    TRISDbits.TRISD5 = 0;   /* LED_RUN  → RD5 */
    TRISCbits.TRISC9 = 0;   /* LED_FAULT → RC9 (TODO: confirm against
                             * EV92R69A — placeholder mirrors AK FOC). */

    /* ── ATA6847L SPI1 (master) ──────────────────────────────────
     * nCS  : DIM 6 OR 8 (TODO probe) → RC2/RP35 (default) GPIO output
     * nIRQ : DIM 40 → RB11/RP28      GPIO input
     * MOSI : DIM 42 → RC6/RP39       PPS OUT  SDO1
     * MISO : DIM 44 → RC7/RP40       PPS IN   SDI1
     * SCK  : DIM 46 → RC8/RP41       PPS OUT  SCK1OUT
     * PPS function codes below are taken from dsPIC33AK128MC106 PPS
     * table — verify against DS70005527 §11.4 once datasheet handy. */
    /* nCS = DIM 8 → RC5 (resolved from AN6285 + DS70005527; was RC2 placeholder) */
    TRISCbits.TRISC5  = 0;  LATCbits.LATC5 = 1;   /* nCS idle high */
    TRISBbits.TRISB11 = 1;                        /* nIRQ input    */

    /* SPI1 PPS routing — function codes verified against DS70005539
     * §11 Table 11-13 (Output Selection for Remappable Pins). */
    _SDI1R  = 40;       /* SDI1   ← RP40 (RC7) — input PPS */
    _RP39R  = 13;       /* RP39  → SDO1     (table 11-13: SDO1 = 13) */
    _RP41R  = 14;       /* RP41  → SCK1OUT  (table 11-13: SCK1OUT = 14) */

    /* ── UART1 (GSP / debug) ─────────────────────────────────────
     * RX : DIM 54 → RC11/RP44 (input)
     * TX : DIM 52 → RC10/RP43 (output) */
    _U1RXR  = 44;       /* U1RX ← RP44 (RC11) */
    _RP43R  =  9;       /* RP43 → U1TX  (table 11-13: U1TX = 9, verified) */

    /* ── ATA6847L BEMF comparator GPIO inputs (3 phases) ─────────
     * Resolved via AN6285 + DS70005527 — no probe needed.
     *   BEMF_A (BEMF1) : DIM 9  → RB9  (AD2AN10/RP26)
     *   BEMF_B (BEMF2) : DIM 11 → RB8  (AD1AN11/RP25)
     *   BEMF_C (BEMF3) : DIM 22 → RA10 (AD2AN7/RP11)
     * GDUCR1.BEMFEN=1 (set in hal_ata6847.c) makes these DIGITAL outputs
     * from the ATA6847L internal comparators — disable ANSEL so the pins
     * read as digital GPIO. */
    BEMF_A_SetInput();
    BEMF_B_SetInput();
    BEMF_C_SetInput();
}
