/**
 * @file port_config.c
 * @brief GPIO, PPS, and analog/digital pin configuration for EV43F54A.
 *
 * Pin mapping (from EV43F54A schematic):
 *   PWM1H/L = RB10/RB11, PWM2H/L = RB12/RB13, PWM3H/L = RB14/RB15
 *   SPI1: SDI=RC4, SCK=RC5, SDO=RC10, nCS=RC11
 *   UART1: RX=RC8, TX=RC9
 *   BEMF: A=RC6, B=RC7, C=RD10
 *   LEDs: Fault=RC3, Run=RC13
 *   Buttons: SW1=RC0, SW2=RD13
 *   nIRQ: RD1
 */

#include <xc.h>
#include "port_config.h"

void SetupGPIOPorts(void)
{
    /* Output latches low */
    LATA = 0x0000;
    LATB = 0x0000;
    LATC = 0x0000;

    /* No open drain */
    ODCA = 0x0000;
    ODCB = 0x0000;
    ODCC = 0x0000;

    /* Analog/Digital: only ADC channels are analog, everything else digital.
     * RA0,RA1,RA2,RA4 = analog (current sense inputs from ATA6847 op-amps)
     * RB0-RB4 = analog (additional ADC channels)
     * RC1,RC2 = analog
     * All port D = digital */
    ANSELA = 0x0017;
    ANSELB = 0x001F;
    ANSELC = 0x0006;
    ANSELD = 0x0000;

    /* GPIO direction */
    nCS_SetOutput();
    nCS_Disable();
    nIRQ_SetInput();
    _TRISC5 = 0;           /* SCK output (for SPI debug) */
    LED_FAULT_SetOutput();
    LED_RUN_SetOutput();
    BTN1_SetInput();
    BTN2_SetInput();
    BEMF_A_SetInput();
    BEMF_B_SetInput();
    BEMF_C_SetInput();

    /* Peripheral Pin Select (PPS) */
    __builtin_write_RPCON(0x0000);  /* unlock */

    /* SPI1 */
    RPINR20bits.SDI1R = 0x0034;     /* RC4 → SPI1:SDI1 */
    RPINR20bits.SCK1R = 0x0035;     /* RC5 → SPI1:SCK1 input */
    RPOR13bits.RP58R  = 0x0005;     /* RC10 → SPI1:SDO1 */
    RPOR10bits.RP53R  = 0x0006;     /* RC5 → SPI1:SCK1 output */

    /* UART1 */
    RPINR18bits.U1RXR = 0x0038;     /* RC8 → UART1:U1RX */
    RPOR12bits.RP57R  = 0x0001;     /* RC9 → UART1:U1TX */

    /* External interrupt: nIRQ from ATA6847 */
    RPINR0bits.INT1R  = 0x0041;     /* RD1 → EXT_INT:INT1 */

    /* Button interrupts */
    RPINR1bits.INT3R  = 0x0030;     /* RC0 → EXT_INT:INT3 (SW1) */
    RPINR1bits.INT2R  = 0x004D;     /* RD13 → EXT_INT:INT2 (SW2) */

    /* SCCP1 is in timer mode (fast poll) — no PPS routing needed.
     * BEMF comparator pins (RC6/RC7/RD10) are read as GPIO directly. */

    __builtin_write_RPCON(0x0800);  /* lock */
}
