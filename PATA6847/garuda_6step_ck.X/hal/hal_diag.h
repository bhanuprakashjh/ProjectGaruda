/**
 * @file hal_diag.h
 * @brief Interactive diagnostic commands via UART.
 *
 * Single-character commands from terminal trigger register dumps,
 * pin state reads, and manual test operations.
 */
#ifndef HAL_DIAG_H
#define HAL_DIAG_H

#include <stdint.h>
#include <stdbool.h>

void DIAG_ProcessCommand(uint8_t cmd);
void DIAG_DumpAll(void);
bool DIAG_IsVerbose(void);

#endif
