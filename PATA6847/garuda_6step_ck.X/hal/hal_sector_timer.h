/**
 * @file hal_sector_timer.h
 * @brief V4 Sector Timer HAL — SCCP3 periodic + SCCP4 HR timestamp.
 *
 * SCCP3: Periodic sector timer at 640 ns/tick (Fp/64). Counts 0 → PRL,
 * fires CCT3IF, auto-reloads to 0. The PI writes new period each sector.
 *
 * SCCP4: Free-running HR timestamp source at 640 ns/tick (unchanged
 * from V3). Provides absolute timestamps for telemetry.
 *
 * Both share the same Fp/64 clock — tick values are interchangeable.
 */

#ifndef HAL_SECTOR_TIMER_H
#define HAL_SECTOR_TIMER_H

#include "../garuda_config.h"

#if FEATURE_V4_SECTOR_PI

#include <stdint.h>
#include <xc.h>

void HAL_SectorTimer_Init(void);
void HAL_SectorTimer_Start(void);
void HAL_SectorTimer_Stop(void);

static inline void HAL_SectorTimer_SetPeriod(uint16_t period)
{
    /* CCPON must be cycled for PRL change to take effect on dsPIC33CK */
    CCP3CON1Lbits.CCPON = 0;
    CCP3PRL = period;
    CCP3TMRL = 0;
    CCP3CON1Lbits.CCPON = 1;
}

static inline uint16_t HAL_SectorTimer_ReadCounter(void)
{
    return CCP3TMRL;
}

static inline uint16_t HAL_SectorTimer_ReadHR(void)
{
    return CCP4TMRL;
}

#endif /* FEATURE_V4_SECTOR_PI */
#endif /* HAL_SECTOR_TIMER_H */
