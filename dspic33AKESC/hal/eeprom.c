/**
 * @file eeprom.c
 *
 * @brief NVM persistent storage implementation.
 *
 * Uses dsPIC33AK data flash (DEE emulation or direct NVM row writes).
 * Dual-page ping-pong: writes alternate between two 128-byte regions.
 * CRC-16-CCITT validates the learned data block (bytes 64-125).
 *
 * Component: HAL / EEPROM
 */

#include "eeprom.h"

#if FEATURE_EEPROM_V2

#include <xc.h>
#include <string.h>

/* RAM shadow of the EEPROM image */
static EEPROM_IMAGE_T eepromShadow;

/* Active page index (0 or 1) for ping-pong */
static uint8_t activePage;

/* Timestamp of last write (for throttling) */
static uint32_t lastWriteTick;

/* NVM page indices */
#define NVM_PAGE0  0
#define NVM_PAGE1  1
#define NVM_PAGES  2

/* ── dsPIC33AK flash NVM storage ─────────────────────────────────────
 *
 * dsPIC33AK128MC106 flash layout:
 *   Program flash: 0x800004 – 0x81FFFF (128KB address space)
 *   _FLASH_PAGE = 0x400 (1024 bytes erase granularity)
 *   _FLASH_ROW  = 0x80  (128 bytes write granularity)
 *
 * We reserve the last 2 pages for EEPROM ping-pong storage.
 * The linker program region must be shortened by 0x800 bytes
 * (add -Wl,--defsym=__PROGRAM_LENGTH=0x1F7FC to linker flags).
 *
 * Each page stores one EEPROM_IMAGE_T (192 bytes = 2 rows).
 * Data is packed 2 bytes per instruction word (low 16 bits only)
 * since dsPIC33AK instruction words are 24-bit (4 byte addresses).
 *
 * NVM write sequence (dsPIC33AK):
 *   1. Set NVMADR to target flash address
 *   2. Set NVMSRCADR to RAM source buffer address
 *   3. Set NVMCONbits.NVMOP for operation (0x03=page erase, 0x02=row write)
 *   4. Set NVMCONbits.WREN = 1
 *   5. __builtin_write_NVM() — compiler handles unlock sequence + WR=1
 *   6. Wait for NVMCONbits.WR == 0
 *   7. Clear NVMCONbits.WREN
 */

/* Flash addresses for the two EEPROM pages (last 2 pages of 128KB flash) */
#define NVM_PAGE0_FLASH_ADDR  0x81F800UL  /* Page 126 */
#define NVM_PAGE1_FLASH_ADDR  0x81FC00UL  /* Page 127 */
#define NVM_FLASH_PAGE_SIZE   _FLASH_PAGE /* 1024 bytes */
#define NVM_FLASH_ROW_SIZE    _FLASH_ROW  /* 128 bytes */

/* Number of rows needed to store one EEPROM_IMAGE_T (192 bytes → 2 rows) */
#define NVM_ROWS_PER_IMAGE    ((sizeof(EEPROM_IMAGE_T) + NVM_FLASH_ROW_SIZE - 1) / NVM_FLASH_ROW_SIZE)

/* Set FEATURE_NVM_FLASH=1 in garuda_config.h to enable real flash
 * persistence across resets.  Default 0 = RAM stubs (data lost on reset
 * but MOTOR_PROFILE compile-time default provides correct boot values).
 * The flash driver uses __builtin_write_NVM() which may need adaptation
 * for the dsPIC33AK PAC unlock sequence. */
#ifndef FEATURE_NVM_FLASH
#define FEATURE_NVM_FLASH  0
#endif

#if FEATURE_NVM_FLASH

/* No linker reservation needed — the NVM pages are at the end of flash
 * beyond the normal program region.  If the linker places code in the
 * last 2 pages, add -Wl,--defsym=__PROGRAM_LENGTH=0x1F7FC to shorten
 * the program region by 0x800 bytes. */

static uint32_t NVM_PageAddr(uint8_t page)
{
    return (page == 0) ? NVM_PAGE0_FLASH_ADDR : NVM_PAGE1_FLASH_ADDR;
}

static bool NVM_ErasePage(uint32_t addr)
{
    NVMADR = addr;
    NVMCONbits.NVMOP = 0x03;  /* Page erase */
    NVMCONbits.WREN = 1;
    __builtin_write_NVM();     /* Unlock + WR=1 */
    while (NVMCONbits.WR);     /* Wait for completion */
    NVMCONbits.WREN = 0;
    return !NVMCONbits.WRERR;
}

static bool NVM_WriteRow(uint32_t flash_addr, const uint8_t *ram_src)
{
    NVMADR = flash_addr;
    NVMSRCADR = (uint32_t)ram_src;
    NVMCONbits.NVMOP = 0x02;  /* Row program */
    NVMCONbits.WREN = 1;
    __builtin_write_NVM();
    while (NVMCONbits.WR);
    NVMCONbits.WREN = 0;
    return !NVMCONbits.WRERR;
}

static bool NVM_ReadPage(uint8_t page, uint8_t *dest, uint16_t len)
{
    if (page >= NVM_PAGES || len > NVM_FLASH_PAGE_SIZE)
        return false;

    /* dsPIC33AK unified memory: read flash directly via pointer.
     * Each 4-byte address holds a 24-bit instruction word.
     * We stored data in the low 16 bits of each word, so we read
     * 2 bytes from every 4-byte boundary. */
    uint32_t addr = NVM_PageAddr(page);
    const volatile uint8_t *src = (const volatile uint8_t *)addr;

    /* Try direct byte read first — dsPIC33AK unified memory should
     * allow this.  Data is stored packed (memcpy-compatible) in rows. */
    memcpy(dest, (const void *)src, len);
    return true;
}

static bool NVM_WritePage(uint8_t page, const uint8_t *src, uint16_t len)
{
    if (page >= NVM_PAGES || len > NVM_FLASH_PAGE_SIZE)
        return false;

    uint32_t base = NVM_PageAddr(page);

    /* Step 1: Erase the target page (sets all bits to 1) */
    if (!NVM_ErasePage(base))
        return false;

    /* Step 2: Write data row by row.
     * NVM row write copies NVM_FLASH_ROW_SIZE bytes from RAM to flash. */
    uint16_t offset = 0;
    uint8_t rowBuf[NVM_FLASH_ROW_SIZE];

    for (uint16_t row = 0; row < NVM_ROWS_PER_IMAGE; row++) {
        uint16_t chunk = len - offset;
        if (chunk > NVM_FLASH_ROW_SIZE)
            chunk = NVM_FLASH_ROW_SIZE;

        /* Fill row buffer: data + 0xFF padding */
        memcpy(rowBuf, src + offset, chunk);
        if (chunk < NVM_FLASH_ROW_SIZE)
            memset(rowBuf + chunk, 0xFF, NVM_FLASH_ROW_SIZE - chunk);

        if (!NVM_WriteRow(base + offset, rowBuf))
            return false;

        offset += NVM_FLASH_ROW_SIZE;
        if (offset >= len)
            break;
    }

    return true;
}

#else /* !FEATURE_NVM_FLASH — RAM fallback for simulation/testing */

static uint8_t nvmPages[NVM_PAGES][sizeof(EEPROM_IMAGE_T)];
static bool    nvmPageValid[NVM_PAGES];

static bool NVM_ReadPage(uint8_t page, uint8_t *dest, uint16_t len)
{
    if (page >= NVM_PAGES || len > sizeof(EEPROM_IMAGE_T))
        return false;
    if (!nvmPageValid[page])
        return false;
    memcpy(dest, nvmPages[page], len);
    return true;
}

static bool NVM_WritePage(uint8_t page, const uint8_t *src, uint16_t len)
{
    if (page >= NVM_PAGES || len > sizeof(EEPROM_IMAGE_T))
        return false;
    memcpy(nvmPages[page], src, len);
    nvmPageValid[page] = true;
    return true;
}

#endif /* FEATURE_NVM_FLASH */

uint16_t EEPROM_ComputeCRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

static bool ValidateLearnedBlock(const EEPROM_LEARNED_T *learned)
{
    if (learned->magic != EEPROM_MAGIC)
        return false;
    if (learned->schemaVersion != EEPROM_SCHEMA_VERSION)
        return false;

    /* CRC covers bytes 0-61 of the learned block (offset 64-125 in image),
     * i.e., everything except the CRC field itself */
    uint16_t computedCRC = EEPROM_ComputeCRC16(
        (const uint8_t *)learned,
        sizeof(EEPROM_LEARNED_T) - sizeof(uint16_t));

    return (computedCRC == learned->crc16);
}

static void InitLearnedDefaults(EEPROM_LEARNED_T *learned)
{
    memset(learned, 0, sizeof(EEPROM_LEARNED_T));
    learned->magic = EEPROM_MAGIC;
    learned->schemaVersion = EEPROM_SCHEMA_VERSION;
    learned->flags = 0;
    /* CRC will be computed on save */
}

bool EEPROM_Init(EEPROM_IMAGE_T *image)
{
    EEPROM_IMAGE_T page0, page1;
    bool valid0, valid1;

    valid0 = NVM_ReadPage(NVM_PAGE0, (uint8_t *)&page0, sizeof(EEPROM_IMAGE_T));
    valid1 = NVM_ReadPage(NVM_PAGE1, (uint8_t *)&page1, sizeof(EEPROM_IMAGE_T));

    if (valid0)
        valid0 = ValidateLearnedBlock(&page0.learned);
    if (valid1)
        valid1 = ValidateLearnedBlock(&page1.learned);

    if (valid0 && valid1)
    {
        /* Both valid — pick the one with higher write count */
        if (page1.learned.wear.writeCount > page0.learned.wear.writeCount)
        {
            memcpy(image, &page1, sizeof(EEPROM_IMAGE_T));
            activePage = 1;
        }
        else
        {
            memcpy(image, &page0, sizeof(EEPROM_IMAGE_T));
            activePage = 0;
        }
    }
    else if (valid0)
    {
        memcpy(image, &page0, sizeof(EEPROM_IMAGE_T));
        activePage = 0;
    }
    else if (valid1)
    {
        memcpy(image, &page1, sizeof(EEPROM_IMAGE_T));
        activePage = 1;
    }
    else
    {
        /* No valid pages — factory defaults */
        memset(image, 0, sizeof(EEPROM_IMAGE_T));
        InitLearnedDefaults(&image->learned);
        activePage = 0;
        memcpy(&eepromShadow, image, sizeof(EEPROM_IMAGE_T));
        lastWriteTick = 0;
        return false;
    }

    memcpy(&eepromShadow, image, sizeof(EEPROM_IMAGE_T));
    lastWriteTick = 0;
    return true;
}

bool EEPROM_Save(const EEPROM_IMAGE_T *image, uint32_t now)
{
    /* Write throttle */
    if (lastWriteTick > 0 && (now - lastWriteTick) < EEPROM_WRITE_THROTTLE_MS)
        return false;

    /* Prepare image with updated CRC */
    EEPROM_IMAGE_T writeImage;
    memcpy(&writeImage, image, sizeof(EEPROM_IMAGE_T));

    writeImage.learned.wear.writeCount++;
    writeImage.learned.crc16 = EEPROM_ComputeCRC16(
        (const uint8_t *)&writeImage.learned,
        sizeof(EEPROM_LEARNED_T) - sizeof(uint16_t));

    /* Write to inactive page (ping-pong) */
    uint8_t targetPage = activePage ^ 1;

    if (!NVM_WritePage(targetPage, (const uint8_t *)&writeImage,
                       sizeof(EEPROM_IMAGE_T)))
        return false;

    activePage = targetPage;
    memcpy(&eepromShadow, &writeImage, sizeof(EEPROM_IMAGE_T));
    lastWriteTick = now;
    return true;
}

bool EEPROM_SaveLearned(const LEARNED_PARAMS_T *learned, uint32_t now)
{
    memcpy(&eepromShadow.learned.commissioned, learned,
           sizeof(LEARNED_PARAMS_T));
    return EEPROM_Save(&eepromShadow, now);
}

bool EEPROM_SaveConfig(const GARUDA_CONFIG_T *config, uint32_t now)
{
    memcpy(&eepromShadow.config, config, sizeof(GARUDA_CONFIG_T));
    return EEPROM_Save(&eepromShadow, now);
}

bool EEPROM_LoadConfig(GARUDA_CONFIG_T *config)
{
    memcpy(config, &eepromShadow.config, sizeof(GARUDA_CONFIG_T));
    return true;
}

bool EEPROM_LoadLearned(LEARNED_PARAMS_T *learned)
{
    if (!ValidateLearnedBlock(&eepromShadow.learned))
        return false;
    memcpy(learned, &eepromShadow.learned.commissioned,
           sizeof(LEARNED_PARAMS_T));
    return true;
}

bool EEPROM_FactoryReset(void)
{
    memset(&eepromShadow, 0, sizeof(EEPROM_IMAGE_T));
    InitLearnedDefaults(&eepromShadow.learned);

    /* Write factory defaults to both pages */
    eepromShadow.learned.crc16 = EEPROM_ComputeCRC16(
        (const uint8_t *)&eepromShadow.learned,
        sizeof(EEPROM_LEARNED_T) - sizeof(uint16_t));

    bool ok = true;
    ok &= NVM_WritePage(NVM_PAGE0, (const uint8_t *)&eepromShadow,
                        sizeof(EEPROM_IMAGE_T));
    ok &= NVM_WritePage(NVM_PAGE1, (const uint8_t *)&eepromShadow,
                        sizeof(EEPROM_IMAGE_T));
    activePage = 0;
    lastWriteTick = 0;
    return ok;
}

uint32_t EEPROM_GetCooldownRemainingMs(uint32_t now)
{
    if (lastWriteTick == 0)
        return 0;  /* Never written — no cooldown */
    uint32_t elapsed = now - lastWriteTick;
    if (elapsed >= EEPROM_WRITE_THROTTLE_MS)
        return 0;
    return EEPROM_WRITE_THROTTLE_MS - elapsed;
}

#endif /* FEATURE_EEPROM_V2 */
