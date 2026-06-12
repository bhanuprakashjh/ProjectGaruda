/**
 * @file port_config.h
 *
 * @brief GPIO pin definitions and interface functions for Project Garuda ESC.
 * Adapted from AN1292 reference — adds BEMF ADC pins, DShot IC pin,
 * removes op-amp configuration.
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

#if GARUDA_TARGET_AK512
/* MC510 DIM (EV67N21A) — same DIM positions, new device pins (AN957
 * port_config.c is the authority for this DIM on the MCLV-48V-300W). */
/* Push buttons */
/* SW1 : DIM:034 - PIN #03 : CVDAN12/RP13/RA12 */
#define SW1                     PORTAbits.RA12
/* SW2 : DIM:036 - PIN #02 : CVDTX20/RP66/RE1 */
#define SW2                     PORTEbits.RE1

/* Used as START/STOP button of Motor */
#define BUTTON_START_STOP       SW1
/* Used as Direction Change button of Motor */
#define BUTTON_DIRECTION_CHANGE SW2

/* Debug LEDs (board LEDs routed through the DIM, per AN957).
 * NOTE: the DIM itself also carries a local yellow LED on RF1 (pin 38,
 * CVDTX30/RP82/RF1, "DIM_LED1/LD2") — that is NOT DIM:032; we keep the
 * board LED mapping AN957 uses. */
/* LED1 : DIM:030 - PIN #10 : CVDTX21/RP67/RE2 */
#define LED1                    LATEbits.LATE2
/* LED2 : DIM:032 - PIN #06 : CVDTX22/RP68/RE3 */
#define LED2                    LATEbits.LATE3
#else
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
#endif /* GARUDA_TARGET_AK512 */

void SetupGPIOPorts(void);
void MapGPIOHWFunction(void);

void HAL_OA12_Init(void);

#if FEATURE_HW_OVERCURRENT
void HAL_OA3_Init(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _PORTCONFIG_H */
