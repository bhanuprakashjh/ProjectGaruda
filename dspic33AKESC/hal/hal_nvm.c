#include "hal_nvm.h"
#include <xc.h>
#include <string.h>

volatile uint16_t g_nvmSelfTestResult = 0xFFFFu;

/* NVM sequence per dsPIC33AK FRM. This device has no NVMKEY-based unlock
 * builtin (__builtin_write_NVM() does not exist on the dsPIC33A/xc-dsc
 * toolchain -- confirmed against the Microchip-generated dsPIC33A flash
 * driver template, which writes the opcode word straight to NVMCON and
 * triggers via NVMCONbits.WR). The opcode word packs WREN (bit 14) and
 * NVMOP (bits 3:0) together per the device pack (ATDF):
 *   page erase = 0x4003, row write = 0x4002.
 * Sequence: NVMADR = target, NVMSRCADR = RAM source (row write only),
 * NVMCON = opcode, WR = 1, wait !WR, check WRERR. */
#define NVM_OP_PAGE_ERASE  0x4003u
#define NVM_OP_ROW_WRITE   0x4002u

static bool NvmOp(uint32_t addr, const uint8_t *src, uint32_t opcode)
{
    NVMADR = addr;
    if (src != NULL)
        NVMSRCADR = (uint32_t)(uintptr_t)src;
    NVMCON = opcode;
    NVMCONbits.WR = 1;
    while (NVMCONbits.WR);
    return !NVMCONbits.WRERR;
}

bool NVMFLASH_ErasePages(uint32_t addr, uint16_t nPages)
{
    if (addr % _FLASH_PAGE)
        return false;
    for (uint16_t p = 0; p < nPages; p++) {
        if (!NvmOp(addr + (uint32_t)p * _FLASH_PAGE, NULL, NVM_OP_PAGE_ERASE))
            return false;
    }
    return true;
}

bool NVMFLASH_WriteImage(uint32_t addr, const uint8_t *src, uint16_t len)
{
    if (addr % _FLASH_ROW)
        return false;
    uint8_t rowBuf[_FLASH_ROW];
    uint16_t offset = 0;
    while (offset < len) {
        uint16_t chunk = (uint16_t)(len - offset);
        if (chunk > _FLASH_ROW)
            chunk = _FLASH_ROW;
        memcpy(rowBuf, src + offset, chunk);
        if (chunk < _FLASH_ROW)
            memset(rowBuf + chunk, 0xFF, _FLASH_ROW - chunk);
        if (!NvmOp(addr + offset, rowBuf, NVM_OP_ROW_WRITE))
            return false;
        offset += _FLASH_ROW;
    }
    return true;
}

void NVMFLASH_Read(uint32_t addr, uint8_t *dst, uint16_t len)
{
    /* dsPIC33AK flash is memory-mapped and byte-readable (the const
     * profileDefaults[] array is read the same way). */
    memcpy(dst, (const void *)(uintptr_t)addr, len);
}

uint16_t NVMFLASH_SelfTest(uint32_t addr, uint16_t nPages)
{
    if (!NVMFLASH_ErasePages(addr, nPages))
        return 1;

    const uint8_t *flash = (const uint8_t *)(uintptr_t)addr;
    uint32_t nBytes = (uint32_t)nPages * _FLASH_PAGE;
    for (uint32_t i = 0; i < nBytes; i++)
        if (flash[i] != 0xFFu)
            return 2;

    /* Pattern: one page worth, index-derived so shifts are detectable. */
    static uint8_t pat[_FLASH_PAGE];
    for (uint16_t i = 0; i < _FLASH_PAGE; i++)
        pat[i] = (uint8_t)(i * 7u + 3u);
    if (!NVMFLASH_WriteImage(addr, pat, _FLASH_PAGE))
        return 3;

    for (uint16_t i = 0; i < _FLASH_PAGE; i++)
        if (flash[i] != (uint8_t)(i * 7u + 3u))
            return 4;

    if (!NVMFLASH_ErasePages(addr, nPages))
        return 5;
    return 0;
}
