#include "gsp_param_store.h"
#include "../hal/hal_nvm.h"
#include "../hal/eeprom.h"      /* EEPROM_ComputeCRC16 */
#include <string.h>

_Static_assert(PARAM_STORE_ADDR >= NVMFLASH_WRITE_FLOOR,
               "user area must sit inside the NVM write window");

/* Reserve + hex-emit the user area: 0xFF filler pinned at PARAM_STORE_ADDR.
 * This (a) keeps the linker from placing code there and (b) guarantees
 * programming rewrites the area to the erased pattern → reflash = factory
 * reset regardless of IDE erase settings. */
static const uint8_t s_userArea[PARAM_STORE_PAGES * 0x400UL]
    __attribute__((section("gsp_param_store_userarea"), address(PARAM_STORE_ADDR), keep))
    = { [0 ... (PARAM_STORE_PAGES * 0x400UL) - 1] = 0xFF };

_Static_assert(sizeof(GSP_PARAM_IMAGE_T) <= sizeof(s_userArea),
               "param image must fit the reserved user area");
_Static_assert(PARAM_STORE_ADDR % 0x400UL == 0,
               "user area must be page-aligned");

static PARAM_SOURCE_T s_source = PARAM_SOURCE_FACTORY;

static bool UserImageValid(const GSP_PARAM_IMAGE_T *img)
{
    if (img->magic != GSP_PARAM_MAGIC)
        return false;
    if (img->schema != (uint16_t)sizeof(GSP_PARAMS_T))
        return false;
    /* CUSTOM (== GSP_PROFILE_COUNT) is a legitimate saved active profile —
     * it has no table slot but the table itself is still good. Only values
     * beyond CUSTOM indicate corruption. */
    if (img->activeProfile > GSP_PROFILE_CUSTOM)
        return false;
    uint16_t crc = EEPROM_ComputeCRC16((const uint8_t *)img->table,
                                       (uint16_t)sizeof(img->table));
    return crc == img->crc16;
}

PARAM_SOURCE_T GSP_ParamStore_Load(GSP_PARAMS_T *table, uint8_t *activeProfileOut)
{
    const GSP_PARAM_IMAGE_T *img = (const GSP_PARAM_IMAGE_T *)s_userArea;

    if (UserImageValid(img)) {
        memcpy(table, img->table, sizeof(img->table));
        /* CUSTOM has no table slot: load the table but keep the caller's
         * compile-time default as the active selection. */
        if (img->activeProfile < GSP_PROFILE_COUNT)
            *activeProfileOut = img->activeProfile;
        s_source = PARAM_SOURCE_USER;
    } else {
        memcpy(table, profileDefaults,
               sizeof(GSP_PARAMS_T) * GSP_PROFILE_COUNT);
        /* caller keeps its compile-time default for activeProfileOut */
        s_source = PARAM_SOURCE_FACTORY;
    }
    return s_source;
}

bool GSP_ParamStore_Save(const GSP_PARAMS_T *table, uint8_t activeProfileNow)
{
    static GSP_PARAM_IMAGE_T img;   /* static: too big for ISR-shared stack */

    img.magic         = GSP_PARAM_MAGIC;
    img.schema        = (uint16_t)sizeof(GSP_PARAMS_T);
    img.activeProfile = activeProfileNow;
    img.reserved      = 0;
    memcpy(img.table, table, sizeof(img.table));
    img.crc16 = EEPROM_ComputeCRC16((const uint8_t *)img.table,
                                    (uint16_t)sizeof(img.table));

    if (!NVMFLASH_ErasePages(PARAM_STORE_ADDR, PARAM_STORE_PAGES))
        return false;
    if (!NVMFLASH_WriteImage(PARAM_STORE_ADDR, (const uint8_t *)&img,
                             (uint16_t)sizeof(img)))
        return false;

    /* Readback verify via the memory-mapped area itself. */
    if (memcmp(s_userArea, &img, sizeof(img)) != 0)
        return false;
    return true;
}

bool GSP_ParamStore_EraseUser(void)
{
    return NVMFLASH_ErasePages(PARAM_STORE_ADDR, PARAM_STORE_PAGES);
}

PARAM_SOURCE_T GSP_ParamStore_GetSource(void)
{
    return s_source;
}
