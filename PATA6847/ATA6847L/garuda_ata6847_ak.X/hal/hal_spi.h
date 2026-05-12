/**
 * @file hal_spi.h
 * @brief SPI driver for ATA6847L communication (AK port).
 *
 * Forked from CK `../../garuda_6step_ck.X/hal/hal_spi.h`.  Interface is
 * unchanged — port differences are in hal_spi.c (SFR names + PPS
 * routing) and port_config.c (pin direction + PPS function codes).
 */
#ifndef HAL_SPI_H
#define HAL_SPI_H

#include <stdint.h>

void     HAL_SPI_Init(void);
uint16_t HAL_SPI_Exchange16(uint16_t data);

#endif
