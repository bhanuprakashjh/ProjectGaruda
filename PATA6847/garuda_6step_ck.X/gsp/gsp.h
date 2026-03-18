/**
 * @file gsp.h
 * @brief Garuda Serial Protocol (GSP) v2 — public API for CK board.
 *
 * Same protocol as dsPIC33AK board (binary framed, CRC16-CCITT).
 * Mutually exclusive with debug UART (both use UART1).
 * Controlled by FEATURE_GSP in garuda_config.h.
 */

#ifndef GSP_H
#define GSP_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void GSP_Init(void);
void GSP_Service(void);
bool GSP_SendResponse(uint8_t cmdId, const uint8_t *payload, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* GSP_H */
