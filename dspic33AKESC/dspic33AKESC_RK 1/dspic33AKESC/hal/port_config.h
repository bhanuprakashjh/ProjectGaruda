/**
 * @file port_config.h
 *
 * @brief GPIO pin definitions and interface functions for Project Garuda ESC.
 * Adapted from AN1292 reference — adds DShot IC pin.
 * Op-amp init (HAL_OA3_Init) gated by USE_INTERNAL_OPAMP in garuda_config.h.
 *
 * Definitions in this file are for dsPIC33AK128MC106
 *
 * Component: PORTS
 */

#ifndef _PORTCONFIG_H
#define _PORTCONFIG_H

#include <xc.h>
#include "../garuda_config.h"   /* Feature flags for conditional declarations */

#ifdef __cplusplus
extern "C" {
#endif

/* Push buttons */
/* SW1 : DIM:034 - PIN #49 : RP58/IOMF7/RD9 */
#define SW1                     PORTDbits.RD9
/* SW2 : DIM:036 - PIN #50 : RP59/RD10 */
#define SW2                     PORTDbits.RD10

/* Used as START/STOP button of Motor */
#define BUTTON_START_STOP       SW1
/* Used as Direction Change button of Motor */
#define BUTTON_DIRECTION_CHANGE SW2

/* Debug LEDs */
/* LED1(LD2) : DIM:030 - PIN #55 : RP54/ASCL1/RD5 */
#define LED1                    LATDbits.LATD5
/* LED2(LD3) : DIM:032 - PIN #34 : RP42/IOMD10/SDO2/IOMF10/PCI19/RC9 */
#define LED2                    LATCbits.LATC9

void SetupGPIOPorts(void);
void MapGPIOHWFunction(void);

/* Initialize OA1, OA2, OA3 for 3-phase current sensing (FOC) */
void HAL_OA3_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* _PORTCONFIG_H */
