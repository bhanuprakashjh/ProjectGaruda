/**
 * @file hal_uart.c
 * @brief UART1 driver for dsPIC33AK128MC106 — debug output + GSP transport.
 *
 * AK UART peripheral: U1CON + U1STAT + U1TXB/U1RXB layout.
 *
 * Clock: AK CLK8 (UART clock domain) is configured to 100 MHz in
 * clock.c. With CLKSEL=01 (Clock Gen 8) and BRGS=0 (low-speed div-16):
 *   actual_baud = CLK8 / (16 × (BRG + 1)) = 100e6 / (16 × 55) = 113 636
 * Target 115 200 → 1.36 % low (within ±2 % UART spec, but on the edge
 * for ratty PC USB-UART converters).  If GSP throws frame errors,
 * switch to BRGS=1 (high-speed div-4):
 *   BRG = (100e6 / (4 × 115200)) − 1 ≈ 216 → 0.16 % error.
 *
 * Pins are PPS-routed in port_config.c.
 *
 * When FEATURE_GSP=1 the write functions compile as empty stubs so the
 * GSP binary protocol owns UART1 traffic.  Init still runs.
 */

#include <xc.h>
#include "hal_uart.h"
#include "../garuda_config.h"

void HAL_UART_Init(void)
{
    /* Disable while configuring */
    U1CONbits.ON   = 0;
    U1CONbits.TXEN = 0;
    U1CONbits.RXEN = 0;

    U1CON = 0;
    U1CONbits.MODE   = 0;   /* Asynchronous 8-bit, no parity */
    U1CONbits.CLKSEL = 1;   /* Clock Gen 8 (CLK8 = 100 MHz) */
    U1CONbits.BRGS   = 0;   /* low-speed: BRG = CLK8 / (16 × Baud) − 1 */
    U1CONbits.STP    = 0;   /* 1 stop bit */
    U1CONbits.FLO    = 0;   /* no flow control */

    U1STAT = 0;
    U1STATbits.TXWM = 7;    /* TX FIFO interrupt when 1 slot empty */
    U1STATbits.RXWM = 0;    /* RX FIFO interrupt when ≥1 word */

    /* 115200 target at CLK8 = 100 MHz, BRGS=0:
     *   BRG = 100e6 / (16 × 115200) − 1 ≈ 53.25 → use 54
     *   Actual: 100e6 / (16 × 55) = 113 636 (1.36 % low — see header) */
    U1BRG = 54;

    HAL_UART_Enable();
}

void HAL_UART_Enable(void)
{
    U1CONbits.ON   = 1;
    U1CONbits.TXEN = 1;
    U1CONbits.RXEN = 1;
}

void HAL_UART_Disable(void)
{
    U1CONbits.TXEN = 0;
    U1CONbits.RXEN = 0;
    U1CONbits.ON   = 0;
}

#if FEATURE_GSP
/* ── GSP mode: empty stubs (UART1 reserved for binary protocol) ──── */
void HAL_UART_WriteByte(uint8_t data) { (void)data; }
void HAL_UART_WriteString(const char *str) { (void)str; }
void HAL_UART_WriteHex8(uint8_t val) { (void)val; }
void HAL_UART_WriteHex16(uint16_t val) { (void)val; }
void HAL_UART_WriteU16(uint16_t val) { (void)val; }
void HAL_UART_WriteS16(int16_t val) { (void)val; }
void HAL_UART_WriteU32(uint32_t val) { (void)val; }
void HAL_UART_NewLine(void) { }
bool HAL_UART_IsRxReady(void) { return false; }
uint8_t HAL_UART_ReadByte(void) { return 0; }

#else
/* ── Debug mode: normal UART output ── */

void HAL_UART_WriteByte(uint8_t data)
{
    while (U1STATbits.TXBF);   /* wait while TX FIFO full */
    U1TXB = data;
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

void HAL_UART_WriteS16(int16_t val)
{
    if (val < 0)
    {
        HAL_UART_WriteByte('-');
        HAL_UART_WriteU16((uint16_t)(-val));
    }
    else
    {
        HAL_UART_WriteU16((uint16_t)val);
    }
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
    return (U1STATbits.RXBE == 0);
}

uint8_t HAL_UART_ReadByte(void)
{
    while (U1STATbits.RXBE);
    return (uint8_t)U1RXB;
}

#endif /* !FEATURE_GSP */
