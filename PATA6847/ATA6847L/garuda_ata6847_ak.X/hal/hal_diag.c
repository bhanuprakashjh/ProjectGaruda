/**
 * @file hal_diag.c
 * @brief Interactive diagnostic commands via UART (stub — 'v' toggle, 'h' help).
 */

#include <stdint.h>
#include <stdbool.h>
#include "hal_diag.h"
#include "hal_uart.h"

static bool verboseMode = false;
bool DIAG_IsVerbose(void) { return verboseMode; }

void DIAG_ProcessCommand(uint8_t cmd)
{
    if (cmd == 'v') verboseMode = !verboseMode;
    else if (cmd == 'h')
    {
        HAL_UART_WriteString("diag commands TBD\r\n");
    }
}

