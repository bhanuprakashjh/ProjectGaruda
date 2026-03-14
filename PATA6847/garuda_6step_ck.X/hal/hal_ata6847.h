/**
 * @file hal_ata6847.h
 * @brief ATA6847 gate driver interface.
 */
#ifndef HAL_ATA6847_H
#define HAL_ATA6847_H

#include <stdint.h>
#include <stdbool.h>

/* ATA6847 register addresses */
#define ATA_DOPMCR   0x01
#define ATA_LOPMCR   0x02
#define ATA_GOPMCR   0x03
#define ATA_WUCR     0x04
#define ATA_GDUCR1   0x05
#define ATA_GDUCR2   0x06
#define ATA_GDUCR3   0x07
#define ATA_GDUCR4   0x08
#define ATA_ILIMCR   0x09
#define ATA_ILIMTH   0x0A
#define ATA_SCPCR    0x0B
#define ATA_CSCR     0x0C
#define ATA_MLDCR    0x0E
#define ATA_RWPCR    0x0F
#define ATA_DSR1     0x10
#define ATA_DSR2     0x11
#define ATA_SIR1     0x13
#define ATA_SIR2     0x14
#define ATA_SIR3     0x15
#define ATA_SIR4     0x16
#define ATA_SIR5     0x17
#define ATA_SIECER1  0x18
#define ATA_SIECER2  0x19
#define ATA_WDTRIG   0x20
#define ATA_WDCR1    0x21

/* GDU modes */
#define GDU_OFF      0x01
#define GDU_STANDBY  0x04
#define GDU_NORMAL   0x07

void    HAL_ATA6847_Init(void);
bool    HAL_ATA6847_EnterGduNormal(void);
void    HAL_ATA6847_EnterGduStandby(void);
void    HAL_ATA6847_ClearFaults(void);
void    HAL_ATA6847_WriteReg(uint8_t addr, uint8_t data);
uint8_t HAL_ATA6847_ReadReg(uint8_t addr);
void    HAL_ATA6847_ReadDiag(uint8_t diag[8]);

#endif
