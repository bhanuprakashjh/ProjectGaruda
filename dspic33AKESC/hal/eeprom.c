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

/* RAM-backed NVM simulation. Allows full EEPROM logic (ping-pong, CRC,
 * throttle) to be tested before the real dsPIC33AK data-flash driver is
 * integrated. Replace these two functions with actual NVM row erase +
 * program when bringing up on hardware. */
static uint8_t nvmPages[NVM_PAGES][sizeof(EEPROM_IMAGE_T)];
static bool    nvmPageValid[NVM_PAGES];  /* tracks whether page has been written */

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
