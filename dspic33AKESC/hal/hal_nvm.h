/**
 * @file hal_nvm.h
 * @brief Policy-free dsPIC33AK512 flash primitives: page erase, row-chunked
 *        image write, memory-mapped read, and a destructive self-test.
 *        All addresses are absolute flash byte addresses.
 * Component: HAL / NVM
 */
#ifndef HAL_NVM_H
#define HAL_NVM_H

#include <stdint.h>
#include <stdbool.h>

/* Erase nPages consecutive flash pages starting at page-aligned addr. */
bool NVMFLASH_ErasePages(uint32_t addr, uint16_t nPages);

/* Write len bytes from src to flash at row-aligned addr (rows padded 0xFF).
 * Target range must be erased first. */
bool NVMFLASH_WriteImage(uint32_t addr, const uint8_t *src, uint16_t len);

/* Read len bytes from memory-mapped flash. */
void NVMFLASH_Read(uint32_t addr, uint8_t *dst, uint16_t len);

/* Destructive test on [addr, addr + nPages*_FLASH_PAGE):
 * erase → verify 0xFF → write pattern → readback-verify → erase.
 * Returns 0 = pass; 1 = erase fail; 2 = not blank after erase;
 * 3 = write fail; 4 = readback mismatch; 5 = final erase fail. */
uint16_t NVMFLASH_SelfTest(uint32_t addr, uint16_t nPages);

/* Result of the boot-time self-test (0xFFFF = not run). */
extern volatile uint16_t g_nvmSelfTestResult;

#endif /* HAL_NVM_H */
