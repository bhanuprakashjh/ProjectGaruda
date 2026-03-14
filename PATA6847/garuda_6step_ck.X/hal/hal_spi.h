/**
 * @file hal_spi.h
 * @brief SPI1 driver for ATA6847 communication.
 */
#ifndef HAL_SPI_H
#define HAL_SPI_H

#include <stdint.h>

void     HAL_SPI_Init(void);
uint16_t HAL_SPI_Exchange16(uint16_t data);

#endif
