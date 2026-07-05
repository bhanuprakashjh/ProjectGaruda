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

/* Erase/write floor: the only legitimate mutable region is the parameter
 * user area in the last pages of flash. Guards in ErasePages/WriteImage
 * reject anything below this or past the end of flash — a caller bug can
 * then never touch firmware code. Raise deliberately if a second mutable
 * region is ever added. */
#define NVMFLASH_WRITE_FLOOR  0x87F000UL
#define NVMFLASH_FLASH_END    0x880000UL

/* Erase nPages consecutive flash pages starting at page-aligned addr. */
bool NVMFLASH_ErasePages(uint32_t addr, uint16_t nPages);

/* Write len bytes from src to flash at row-aligned addr (rows padded 0xFF).
 * Target range must be erased first. */
bool NVMFLASH_WriteImage(uint32_t addr, const uint8_t *src, uint16_t len);

/* Read len bytes from memory-mapped flash. ONLY for regions known to be
 * programmed (ECC-valid) — reading raw-erased flash is undocumented on
 * this ECC part and hung the first bench boot. */
void NVMFLASH_Read(uint32_t addr, uint8_t *dst, uint16_t len);

/* Device geometry in BYTES. THE DFP'S _FLASH_PAGE/_FLASH_ROW ARE IN
 * INSTRUCTIONS (×4 bytes each) — treating them as bytes was the 2026-07-05
 * root cause of every save failure and trap-blink: 1KB erase strides and
 * 128-byte "rows" made 3 of 4 row writes misaligned (WRERR → BUSY) and left
 * pages part-raw-erased (later read → ECC trap → blink loop). Ground truth:
 * Microchip's dspic33a-curiosity-data-eeprom-emulation-demo MCC driver —
 * FLASH_ERASE_PAGE_SIZE_IN_INSTRUCTIONS 1024 → 4096-byte page; row = 128
 * instructions → 512 bytes. Asserted against _FLASH_PAGE/_FLASH_ROW ×4 in
 * hal_nvm.c. */
#define NVMFLASH_PAGE_BYTES 0x1000u
#define NVMFLASH_ROW_BYTES  0x200u

/* Program one all-zero row at row-aligned addr (target must be erased):
 * an ECC-valid "committed blank" stamp so an erased region is never left
 * raw-erased for a later read to trip on. Zeros can't alias a valid image
 * header. */
bool NVMFLASH_ZeroRow(uint32_t addr);

/* Destructive test on [addr, addr + nPages*_FLASH_PAGE):
 * erase → write pattern page → readback-verify → erase → zero-row stamp.
 * Never reads raw-erased flash (no blank-check read — see hal_nvm.c).
 * Returns 0 = pass; 1 = erase fail; 2 = retired (was blank-check);
 * 3 = write fail; 4 = readback mismatch; 5 = final erase fail;
 * 6 = commit-stamp write fail. */
uint16_t NVMFLASH_SelfTest(uint32_t addr, uint16_t nPages);

/* Result of the boot-time self-test (0xFFFF = not run). */
extern volatile uint16_t g_nvmSelfTestResult;

#endif /* HAL_NVM_H */
