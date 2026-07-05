#include "hal_nvm.h"
#include <xc.h>
#include <string.h>

volatile uint16_t g_nvmSelfTestResult = 0xFFFFu;

/* NVM sequence per dsPIC33AK FRM and Microchip's own MCC flash driver
 * (dspic33a-curiosity-data-eeprom-emulation-demo, cloned 2026-07-05):
 * NVMADR = target, NVMSRCADR = RAM source (row write only), NVMCON =
 * opcode, WR = 1, wait !WR, check WRERR/WREC. No NVMKEY exists on
 * dsPIC33A (the demo's FLASH_UNLOCK_KEY is a software-API token, never
 * written to hardware); the MCC driver busy-waits FROM FLASH with no
 * interrupt masking, so neither RAM residency nor an unlock dance is
 * required. The opcode word packs WREN (bit 14) and NVMOP (bits 3:0):
 *   page erase = 0x4003, row write = 0x4002  (same literals as MCC).
 *
 * GEOMETRY (the 2026-07-05 root cause): the DFP's _FLASH_PAGE (0x400) and
 * _FLASH_ROW (0x80) are INSTRUCTION counts — ×4 for bytes. This driver
 * originally used them as byte counts: erases strode 1KB (3 of 4 landed
 * mid-page) and "rows" were 128 B at 128-B offsets, so 3 of 4 row writes
 * were misaligned → WRERR → every save returned BUSY, and aborted
 * sequences left pages part-raw-erased → later reads took the ECC trap
 * (LED2 blink loop / boot hang family). All sizes below are BYTES via the
 * NVMFLASH_*_BYTES macros; the asserts pin them to the DFP. */
#define NVM_OP_PAGE_ERASE  0x4003u
#define NVM_OP_ROW_WRITE   0x4002u

_Static_assert(NVMFLASH_PAGE_BYTES == (uint32_t)_FLASH_PAGE * 4u,
               "page bytes must be _FLASH_PAGE instructions x 4");
_Static_assert(NVMFLASH_ROW_BYTES == (uint32_t)_FLASH_ROW * 4u,
               "row bytes must be _FLASH_ROW instructions x 4");

static bool NvmOp(uint32_t addr, const uint8_t *src, uint32_t opcode)
{
    /* GIE off for the op: an ISR taken mid-op executes with the flash
     * controller busy — MCC tolerates this in a bare demo, but this
     * firmware has a 20 kHz ADC ISR + UART + SCCP load, and every caller
     * is motor-stopped, so the bounded blackout (~ms row write, ~tens of
     * ms page erase) is free insurance. Interrupts breathe between ops. */
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
    if (addr % NVMFLASH_PAGE_BYTES)
        return false;
    if (addr < NVMFLASH_WRITE_FLOOR || addr >= NVMFLASH_FLASH_END)
        return false;
    /* Subtraction form: addr < FLASH_END here, so END - addr cannot wrap. */
    if ((uint32_t)nPages * NVMFLASH_PAGE_BYTES > NVMFLASH_FLASH_END - addr)
        return false;
    for (uint16_t p = 0; p < nPages; p++) {
        if (!NvmOp(addr + (uint32_t)p * NVMFLASH_PAGE_BYTES, NULL,
                   NVM_OP_PAGE_ERASE))
            return false;
    }
    return true;
}

bool NVMFLASH_WriteImage(uint32_t addr, const uint8_t *src, uint16_t len)
{
    if (addr % NVMFLASH_ROW_BYTES)
        return false;
    if (addr < NVMFLASH_WRITE_FLOOR || addr >= NVMFLASH_FLASH_END)
        return false;
    if ((uint32_t)len > NVMFLASH_FLASH_END - addr)
        return false;
    /* Static + 4-aligned: the controller DMAs a FULL 512-byte row from
     * NVMSRCADR (MCC passes uint32_t*), and 512 B is too heavy for the
     * ISR-shared stack anyway. Callers are main-loop only — no reentry. */
    static uint8_t rowBuf[NVMFLASH_ROW_BYTES] __attribute__((aligned(4)));
    uint16_t offset = 0;
    while (offset < len) {
        uint16_t chunk = (uint16_t)(len - offset);
        if (chunk > NVMFLASH_ROW_BYTES)
            chunk = NVMFLASH_ROW_BYTES;
        memcpy(rowBuf, src + offset, chunk);
        if (chunk < NVMFLASH_ROW_BYTES)
            memset(rowBuf + chunk, 0xFF, NVMFLASH_ROW_BYTES - chunk);
        if (!NvmOp(addr + offset, rowBuf, NVM_OP_ROW_WRITE))
            return false;
        offset += NVMFLASH_ROW_BYTES;
    }
    return true;
}

void NVMFLASH_Read(uint32_t addr, uint8_t *dst, uint16_t len)
{
    /* dsPIC33AK flash is memory-mapped and byte-readable (the const
     * profileDefaults[] array is read the same way). */
    memcpy(dst, (const void *)(uintptr_t)addr, len);
}

bool NVMFLASH_ZeroRow(uint32_t addr)
{
    /* Program one all-zero row: an ECC-valid "committed blank" stamp so a
     * just-erased region is never left in the raw-erased state (reading
     * erased-never-reprogrammed flash is UNDOCUMENTED on dsPIC33A ECC
     * flash — DS70005509 only defines the corrupted-checksum read as
     * "interrupt or trap" — and the first bench boot that byte-read a
     * fresh erase hung pre-heartbeat). Zeros also can never alias a valid
     * image header (magic 0x0000 != 'GP'). */
    static const uint8_t zeroRow[NVMFLASH_ROW_BYTES] = {0};
    return NVMFLASH_WriteImage(addr, zeroRow, NVMFLASH_ROW_BYTES);
}

uint16_t NVMFLASH_SelfTest(uint32_t addr, uint16_t nPages)
{
    /* ECC-safe sequence (2026-07-04 bench hang): NEVER read raw-erased
     * flash. The old post-erase byte blank-check is gone (step code 2
     * retired) — erase success is proven implicitly: flash programming can
     * only clear bits 1->0, so if the erase didn't happen the pattern
     * write over old data yields WRERR or a readback mismatch (code 3/4).
     * Pattern covers TWO rows (1 KB) — exercises the row loop without a
     * 4 KB RAM buffer. The rest of the erased page is left raw, which is
     * fine: the final erase + zero-row stamp recommits the whole page and
     * nothing reads past the stamp row before then. */
#define NVM_SELFTEST_PAT_BYTES (2u * NVMFLASH_ROW_BYTES)
    if (!NVMFLASH_ErasePages(addr, nPages))
        return 1;

    /* Index-derived pattern so shifts are detectable. */
    static uint8_t pat[NVM_SELFTEST_PAT_BYTES];
    for (uint16_t i = 0; i < NVM_SELFTEST_PAT_BYTES; i++)
        pat[i] = (uint8_t)(i * 7u + 3u);
    if (!NVMFLASH_WriteImage(addr, pat, NVM_SELFTEST_PAT_BYTES))
        return 3;

    {
        const volatile uint8_t *flash = (const volatile uint8_t *)(uintptr_t)addr;
        for (uint16_t i = 0; i < NVM_SELFTEST_PAT_BYTES; i++)
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
