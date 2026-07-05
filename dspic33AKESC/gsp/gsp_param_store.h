/**
 * @file gsp_param_store.h
 * @brief GSP parameter persistence: factory area = the const
 *        profileDefaults[] array (baked into the .hex, never written at
 *        runtime); user area = PARAM_STORE_PAGES flash pages at
 *        PARAM_STORE_ADDR, emitted in the .hex as 0xFF filler so every
 *        reflash factory-resets. Whole-table image with magic + sizeof
 *        fingerprint + CRC-16. Spec: 2026-07-04-param-storage-design.md.
 * Component: GSP / param store
 */
#ifndef GSP_PARAM_STORE_H
#define GSP_PARAM_STORE_H

#include <stdint.h>
#include <stdbool.h>
#include "gsp_params.h"
#include "../hal/hal_nvm.h"   /* NVMFLASH_PAGE_BYTES — real 4096-byte pages */

/* GEOMETRY FIX 2026-07-05: the area was declared as "4 pages of 0x400"
 * back when the driver believed pages were 1 KB — the DFP's _FLASH_PAGE is
 * in INSTRUCTIONS and a real dsPIC33AK page is 4096 BYTES. Total size and
 * address are UNCHANGED (4 KB at 0x87F000); it is simply ONE real page. */
#define PARAM_STORE_PAGES  1u
#define PARAM_STORE_ADDR   (0x880000UL - (uint32_t)PARAM_STORE_PAGES * NVMFLASH_PAGE_BYTES) /* 0x87F000 */
#define GSP_PARAM_MAGIC    0x4750u  /* 'GP' */

typedef enum {
    PARAM_SOURCE_FACTORY = 0,
    PARAM_SOURCE_USER    = 1,
} PARAM_SOURCE_T;

typedef struct {
    uint16_t magic;          /* GSP_PARAM_MAGIC */
    uint16_t schema;         /* sizeof(GSP_PARAMS_T) — layout fingerprint */
    uint8_t  activeProfile;
    uint8_t  reserved;
    uint16_t crc16;          /* CRC-16-CCITT over table[] only */
    GSP_PARAMS_T table[GSP_PROFILE_COUNT];
} GSP_PARAM_IMAGE_T;

/* Fill table[GSP_PROFILE_COUNT] + activeProfileOut from the user area if it
 * validates (magic, schema, CRC), else from factory. Returns the source. */
PARAM_SOURCE_T GSP_ParamStore_Load(GSP_PARAMS_T *table, uint8_t *activeProfileOut);

/* Erase user area and write header + table. Returns false on any NVM
 * failure (RAM state untouched by caller contract). Readback-verified. */
bool GSP_ParamStore_Save(const GSP_PARAMS_T *table, uint8_t activeProfileNow);

/* Erase the user area (factory becomes the boot source). */
bool GSP_ParamStore_EraseUser(void);

/* Source chosen at the last GSP_ParamStore_Load(). */
PARAM_SOURCE_T GSP_ParamStore_GetSource(void);

#endif /* GSP_PARAM_STORE_H */
