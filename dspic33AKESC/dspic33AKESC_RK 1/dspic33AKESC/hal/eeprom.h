/**
 * @file eeprom.h
 *
 * @brief NVM persistent storage with dual-page ping-pong and CRC-16.
 *
 * 128-byte EEPROM image:
 *   Bytes 0-63:   GARUDA_CONFIG_T (user config)
 *   Bytes 64-127: EEPROM_LEARNED_T (commissioned + adapted + health)
 *
 * Features:
 *   - Dual-page ping-pong for power-loss safety
 *   - CRC-16-CCITT validation on load
 *   - Write throttle: minimum 60s between writes
 *
 * Component: HAL / EEPROM
 */

#ifndef EEPROM_H
#define EEPROM_H

#include <stdint.h>
#include <stdbool.h>
#include "../garuda_types.h"
#include "../garuda_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#if FEATURE_EEPROM_V2

/* EEPROM v2 magic number ('GR') */
#define EEPROM_MAGIC            0x4752
#define EEPROM_SCHEMA_VERSION   2

/* Minimum interval between writes (milliseconds) */
#define EEPROM_WRITE_THROTTLE_MS 60000

/**
 * @brief Initialize EEPROM subsystem and load image from NVM.
 *
 * Reads both pages, validates CRC, selects the latest valid page.
 * If no valid page exists, initializes to factory defaults.
 *
 * @param image  Pointer to RAM shadow of EEPROM image
 * @return true if a valid image was loaded, false if factory defaults used
 */
bool EEPROM_Init(EEPROM_IMAGE_T *image);

/**
 * @brief Save the full EEPROM image to NVM.
 *
 * Writes to the inactive page (ping-pong), computes and appends CRC.
 * Enforces write throttle (minimum 60s between writes).
 *
 * @param image  Pointer to RAM image to save
 * @param now    Current systemTick (1ms) for write throttle
 * @return true if written, false if throttled or write failed
 */
bool EEPROM_Save(const EEPROM_IMAGE_T *image, uint32_t now);

/**
 * @brief Save only the learned data block (bytes 64-127).
 *
 * Convenience wrapper: updates learned block in current image and saves.
 *
 * @param learned  Pointer to learned params to save
 * @param now      Current systemTick (1ms)
 * @return true if written, false if throttled or write failed
 */
bool EEPROM_SaveLearned(const LEARNED_PARAMS_T *learned, uint32_t now);

/**
 * @brief Save user config block (bytes 0-63).
 *
 * Convenience wrapper: updates config in current image and saves.
 *
 * @param config  Pointer to config to save
 * @param now     Current systemTick (1ms)
 * @return true if written, false if throttled or write failed
 */
bool EEPROM_SaveConfig(const GARUDA_CONFIG_T *config, uint32_t now);

/**
 * @brief Load user config from EEPROM image.
 * @param config  Pointer to destination config struct
 * @return true if loaded successfully
 */
bool EEPROM_LoadConfig(GARUDA_CONFIG_T *config);

/**
 * @brief Load learned motor params from EEPROM image.
 * @param learned  Pointer to destination learned struct
 * @return true if loaded and CRC valid
 */
bool EEPROM_LoadLearned(LEARNED_PARAMS_T *learned);

/**
 * @brief Erase all EEPROM data and restore factory defaults.
 * @return true if factory reset completed
 */
bool EEPROM_FactoryReset(void);

/**
 * @brief Compute CRC-16-CCITT over a data block.
 * @param data  Pointer to data
 * @param len   Length in bytes
 * @return CRC-16 value
 */
uint16_t EEPROM_ComputeCRC16(const uint8_t *data, uint16_t len);

/**
 * @brief Get remaining write cooldown time.
 * @param now  Current systemTick (1ms)
 * @return 0 if write allowed, else remaining cooldown in ms
 */
uint32_t EEPROM_GetCooldownRemainingMs(uint32_t now);

#endif /* FEATURE_EEPROM_V2 */

#ifdef __cplusplus
}
#endif

#endif /* EEPROM_H */
