/**
 * @file hal_uart.h
 * @brief UART1 debug output for dsPIC33CK on EV43F54A.
 *
 * UART1 on RC8(RX)/RC9(TX) → onboard USB-UART converter.
 * 115200 baud, 8N1.
 */
#ifndef HAL_UART_H
#define HAL_UART_H

#include <stdint.h>
#include <stdbool.h>

void     HAL_UART_Init(void);
void     HAL_UART_Enable(void);
void     HAL_UART_Disable(void);
void     HAL_UART_WriteByte(uint8_t data);
void     HAL_UART_WriteString(const char *str);
void     HAL_UART_WriteHex8(uint8_t val);
void     HAL_UART_WriteHex16(uint16_t val);
void     HAL_UART_WriteU16(uint16_t val);
void     HAL_UART_WriteU32(uint32_t val);
void     HAL_UART_NewLine(void);
bool     HAL_UART_IsRxReady(void);
uint8_t  HAL_UART_ReadByte(void);

#endif
