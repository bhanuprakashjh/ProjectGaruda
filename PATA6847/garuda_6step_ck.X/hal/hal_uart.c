/**
 * @file hal_uart.c
 * @brief UART1 debug output for dsPIC33CK on EV43F54A.
 *
 * UART1 pins: RC8(RX) / RC9(TX) via PPS (configured in port_config.c).
 * Connected to onboard USB-UART converter (USB micro-B connector).
 *
 * 115200 baud, 8N1, BRGH=1.
 * BRG = FCY / (4 * BAUD) - 1 = 100000000 / (4 * 115200) - 1 = 216 = 0xD8
 *
 * Register values adapted from EV43F54A reference (uart1.c), baud changed
 * from 9600 to 115200 for faster debug output.
 */

#include <xc.h>
#include "hal_uart.h"

void HAL_UART_Init(void)
{
    /* U1MODE: Async 8-bit, BRGH=1 (high-speed), disabled initially */
    U1MODE = (0x8080 & ~(1 << 15));  /* BRGH=1, UARTEN=0 */
    U1MODEH = 0x00;

    U1STA = 0x00;
    U1STAH = 0x22;   /* URXISEL=RX_ONE_WORD, UTXISEL=TX_BUF_EMPTY */

    /* BRG for 115200 baud at FCY=100MHz, BRGH=1:
     * BRG = FCY / (4 * BAUD) - 1 = 100000000 / 460800 - 1 = 216 */
    U1BRG = 0x00D8;
    U1BRGH = 0x00;

    U1P1 = 0x00;
    U1P2 = 0x00;
    U1P3 = 0x00;
    U1P3H = 0x00;
    U1TXCHK = 0x00;
    U1RXCHK = 0x00;
    U1SCCON = 0x00;
    U1SCINT = 0x00;
    U1INT = 0x00;

    HAL_UART_Enable();
}

void HAL_UART_Enable(void)
{
    U1MODEbits.UARTEN = 1;
    U1MODEbits.UTXEN = 1;
    U1MODEbits.URXEN = 1;
}

void HAL_UART_Disable(void)
{
    U1MODEbits.UARTEN = 0;
    U1MODEbits.UTXEN = 0;
    U1MODEbits.URXEN = 0;
}

void HAL_UART_WriteByte(uint8_t data)
{
    while (U1STAHbits.UTXBF);
    U1TXREG = data;
}

void HAL_UART_WriteString(const char *str)
{
    while (*str)
        HAL_UART_WriteByte((uint8_t)*str++);
}

static void WriteHexNibble(uint8_t val)
{
    val &= 0x0F;
    HAL_UART_WriteByte((uint8_t)((val < 10) ? ('0' + val) : ('A' + val - 10)));
}

void HAL_UART_WriteHex8(uint8_t val)
{
    WriteHexNibble((uint8_t)(val >> 4));
    WriteHexNibble(val);
}

void HAL_UART_WriteHex16(uint16_t val)
{
    HAL_UART_WriteHex8((uint8_t)(val >> 8));
    HAL_UART_WriteHex8((uint8_t)val);
}

void HAL_UART_WriteU16(uint16_t val)
{
    char buf[6];
    int i = 0;

    if (val == 0)
    {
        HAL_UART_WriteByte('0');
        return;
    }

    while (val > 0)
    {
        buf[i++] = (char)('0' + (val % 10));
        val /= 10;
    }
    while (i > 0)
        HAL_UART_WriteByte((uint8_t)buf[--i]);
}

void HAL_UART_WriteU32(uint32_t val)
{
    char buf[11];
    int i = 0;

    if (val == 0)
    {
        HAL_UART_WriteByte('0');
        return;
    }

    while (val > 0)
    {
        buf[i++] = (char)('0' + (val % 10));
        val /= 10;
    }
    while (i > 0)
        HAL_UART_WriteByte((uint8_t)buf[--i]);
}

void HAL_UART_NewLine(void)
{
    HAL_UART_WriteByte('\r');
    HAL_UART_WriteByte('\n');
}

bool HAL_UART_IsRxReady(void)
{
    return (U1STAHbits.URXBE == 0);
}

uint8_t HAL_UART_ReadByte(void)
{
    while (U1STAHbits.URXBE);

    if (U1STAbits.OERR)
        U1STAbits.OERR = 0;

    return (uint8_t)U1RXREG;
}
