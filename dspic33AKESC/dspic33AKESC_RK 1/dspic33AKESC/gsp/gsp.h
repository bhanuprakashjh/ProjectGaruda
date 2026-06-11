/**
 * @file gsp.h
 *
 * @brief Garuda Serial Protocol (GSP) v1 — public API.
 *
 * GSP provides a binary protocol over UART1 (PKoB4 CDC) for host
 * communication: ping, firmware info query, and telemetry snapshots.
 *
 * Mutually exclusive with X2CScope (both use UART1).
 * Controlled by FEATURE_GSP in garuda_config.h.
 *
 * Component: GSP
 */

#ifndef GSP_H
#define GSP_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize UART1 for GSP (115.2kbps) and reset ring buffers + parser.
 * Call once from main() before entering the main loop.
 */
void GSP_Init(void);

/**
 * Poll-driven service routine. Call from main loop.
 * - Drains UART1 HW FIFO into RX ring buffer
 * - Runs parser state machine
 * - Dispatches up to 1 command per call
 * - Pumps TX ring buffer to UART1 HW TX register
 */
void GSP_Service(void);

/**
 * Enqueue a response packet into the TX ring buffer.
 * Frames the response with start byte, length, CRC.
 *
 * @param cmdId    Command ID for the response
 * @param payload  Pointer to response payload (NULL if none)
 * @param len      Payload length in bytes (0 for empty response)
 * @return true if packet was enqueued, false if TX ring full
 */
bool GSP_SendResponse(uint8_t cmdId, const uint8_t *payload, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* GSP_H */
