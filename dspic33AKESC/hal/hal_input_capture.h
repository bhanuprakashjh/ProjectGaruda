/**
 * @file hal_input_capture.h
 *
 * @brief SCCP4 Input Capture HAL for RX input (PWM / DShot).
 * Pin: RD8/RP57 -> ICM4R PPS.
 * SCCP4 captures every rise/fall edge with 32-bit timer @ 100 MHz.
 *
 * Component: HAL
 */

#ifndef HAL_INPUT_CAPTURE_H
#define HAL_INPUT_CAPTURE_H

#include <stdint.h>
#include <stdbool.h>
#include "garuda_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#if (FEATURE_RX_PWM || FEATURE_RX_DSHOT || FEATURE_RX_AUTO)

/**
 * @brief Initialize SCCP4 for input capture on RD8/RP57.
 * Configures PPS, SCCP4 IC mode (every rise/fall, 32-bit timer).
 * Does NOT enable the IC — call HAL_IC4_Enable() separately.
 */
void HAL_IC4_Init(void);

/**
 * @brief Enable IC4 — starts capturing edges, enables ISR.
 */
void HAL_IC4_Enable(void);

/**
 * @brief Disable IC4 — stops capturing, disables ISR.
 */
void HAL_IC4_Disable(void);

/**
 * @brief Switch IC4 to PWM capture mode (rise/fall pairing).
 */
void HAL_IC4_SetModePwm(void);

/**
 * @brief Switch IC4 to detection mode (edge buffering for classification).
 */
void HAL_IC4_SetModeDetect(void);

/**
 * @brief Get number of captured edges in detection buffer.
 */
uint8_t HAL_IC4_GetDetectEdgeCount(void);

/**
 * @brief Copy detection edges to caller buffer and reset.
 * @param buf    Output buffer (at least 16 entries).
 * @param count  Output: number of edges copied.
 */
void HAL_IC4_GetDetectEdges(uint32_t *buf, uint8_t *count);

#if FEATURE_RX_DSHOT
/**
 * @brief Decode a 16-bit DShot frame from 32 edge timestamps in ring buffer.
 * @param edges   64-edge ring buffer.
 * @param offset  Bit offset (0-15) within the ring.
 * @param[out] throttle  Decoded throttle 0-2047.
 * @param[out] telemetry Telemetry request bit.
 * @return true if CRC passes.
 */
bool DshotDecodeFrame(const volatile uint32_t *edges, uint8_t offset,
                       uint16_t *throttle, uint8_t *telemetry);

/**
 * @brief Configure DMA channel 0 for DShot ping-pong capture.
 * Source: CCP4BUF, Dest: ping-pong buffers, Count: RX_DSHOT_DMA_COUNT.
 * Disables IC4 ISR (DMA handles captures instead).
 */
void HAL_IC4_ConfigDmaDshot(void);

/**
 * @brief Disable DMA mode, re-enable IC4 ISR for detection phase.
 */
void HAL_IC4_DisableDma(void);
#endif

#endif /* FEATURE_RX_PWM || FEATURE_RX_DSHOT || FEATURE_RX_AUTO */

#ifdef __cplusplus
}
#endif

#endif /* HAL_INPUT_CAPTURE_H */
