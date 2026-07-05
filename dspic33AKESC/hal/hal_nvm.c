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

/* RAM-RESIDENT (2026-07-05 bench, trap-blink boot): NO instruction may be
 * fetched from flash while WR is active. The selftest build trapped at boot
 * (LED2 fast-blink loop) — the busy-wait below, executing from flash,
 * faults the moment an op engages. Traps are unmaskable, so the GIE guard
 * alone can't save it; the op + wait must run from RAM. ramfunc: startup
 * copies this function to data RAM; it touches only SFRs and stack.
 * noinline: inlining into a flash-resident caller would silently undo the
 * whole point. */
static bool __attribute__((ramfunc, noinline))
NvmOp(uint32_t addr, const uint8_t *src, uint32_t opcode)
{
    /* GIE off for the whole op (2026-07-05 bench: first-ever runtime save
     * returned BUSY, retry hard-hung the CPU; boot-context ops were the only
     * ones ever proven). An ISR taken while WR is active fetches vectors and
     * code from the busy flash panel — fatal (see ramfunc note). Every
     * caller is motor-stopped; the blackout is bounded by one op (~ms row
     * write, ~tens of ms page erase). Interrupts breathe between ops so
     * UART/heartbeat survive a full save. */
    uint32_t gie = INTCON1bits.GIE;
    INTCON1bits.GIE = 0;
    NVMADR = addr;
    if (src != NULL)
        NVMSRCADR = (uint32_t)(uintptr_t)src;
    NVMCON = opcode;
    NVMCONbits.WR = 1;
    /* Bounded wait: a stuck WR must degrade to a reported failure, not a
     * silent forever-hang (WDT is fused off on this board). ~100M iterations
     * is seconds at 200 MHz — orders of magnitude past a worst-case erase. */
    uint32_t spins = 100000000u;
    while (NVMCONbits.WR && --spins);
    NVMCONbits.WREN = 0;   /* defense-in-depth: don't leave WE latched */
    bool ok = (spins != 0u) && !NVMCONbits.WRERR;
    INTCON1bits.GIE = gie;
    return ok;
}

bool NVMFLASH_ErasePages(uint32_t addr, uint16_t nPages)
{
    if (addr % _FLASH_PAGE)
        return false;
    if (addr < NVMFLASH_WRITE_FLOOR || addr >= NVMFLASH_FLASH_END)
        return false;
    /* Subtraction form: addr < FLASH_END here, so END - addr cannot wrap;
     * nPages*_FLASH_PAGE maxes at 0x3FFFC00 — no overflow either side. */
    if ((uint32_t)nPages * _FLASH_PAGE > NVMFLASH_FLASH_END - addr)
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
    if (addr < NVMFLASH_WRITE_FLOOR || addr >= NVMFLASH_FLASH_END)
        return false;
    if ((uint32_t)len > NVMFLASH_FLASH_END - addr)
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

_Static_assert(NVMFLASH_ROW_BYTES == _FLASH_ROW,
               "exported row size must match the device row");

bool NVMFLASH_ZeroRow(uint32_t addr)
{
    /* Program one all-zero row: an ECC-valid "committed blank" stamp so a
     * just-erased region is never left in the raw-erased state (reading
     * erased-never-reprogrammed flash is UNDOCUMENTED on dsPIC33A ECC
     * flash — DS70005509 only defines the corrupted-checksum read as
     * "interrupt or trap" — and the first bench boot that byte-read a
     * fresh erase hung pre-heartbeat). Zeros also can never alias a valid
     * image header (magic 0x0000 != 'GP'). */
    static const uint8_t zeroRow[_FLASH_ROW] = {0};
    return NVMFLASH_WriteImage(addr, zeroRow, _FLASH_ROW);
}

uint16_t NVMFLASH_SelfTest(uint32_t addr, uint16_t nPages)
{
    /* ECC-safe sequence (2026-07-04 bench hang): NEVER read raw-erased
     * flash. The old post-erase byte blank-check is gone (step code 2
     * retired) — erase success is proven implicitly: flash programming can
     * only clear bits 1->0, so if the erase didn't happen the pattern
     * write over old data yields WRERR or a readback mismatch (code 3/4). */
    if (!NVMFLASH_ErasePages(addr, nPages))
        return 1;

    /* Pattern: one page worth, index-derived so shifts are detectable. */
    static uint8_t pat[_FLASH_PAGE];
    for (uint16_t i = 0; i < _FLASH_PAGE; i++)
        pat[i] = (uint8_t)(i * 7u + 3u);
    if (!NVMFLASH_WriteImage(addr, pat, _FLASH_PAGE))
        return 3;

    {
        const volatile uint8_t *flash = (const volatile uint8_t *)(uintptr_t)addr;
        for (uint16_t i = 0; i < _FLASH_PAGE; i++)
            if (flash[i] != (uint8_t)(i * 7u + 3u))
                return 4;
    }

    if (!NVMFLASH_ErasePages(addr, nPages))
        return 5;

    /* Leave the area COMMITTED, never raw-erased: the next boot's param
     * load reads this region. */
    if (!NVMFLASH_ZeroRow(addr))
        return 6;
    return 0;
}
