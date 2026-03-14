/**
 * @file port_config.h
 * @brief GPIO and PPS configuration for EV43F54A board.
 */
#ifndef HAL_PORT_CONFIG_H
#define HAL_PORT_CONFIG_H

#include <xc.h>
#include "../garuda_config.h"

/* ── Pin Macros ────────────────────────────────────────────────────── */

/* SPI chip select (ATA6847) */
#define nCS_SetOutput()     (_TRISC11 = 0)
#define nCS                 _LATC11
#define nCS_Enable()        (nCS = 0)
#define nCS_Disable()       (nCS = 1)

/* nIRQ from ATA6847 */
#define nIRQ_SetInput()     (_TRISD1 = 1)
#define nIRQ_GetValue()     (_RD1)

/* LEDs */
#define LED_FAULT_SetOutput()  (_TRISC3 = 0)
#define LED_FAULT              _LATC3
#define LED_RUN_SetOutput()    (_TRISC13 = 0)
#define LED_RUN                _LATC13

/* Buttons (active low) */
#define BTN1_SetInput()     (_TRISC0 = 1)
#define BTN1_GetValue()     (_RC0)
#define BTN2_SetInput()     (_TRISD13 = 1)
#define BTN2_GetValue()     (_RD13)

/* BEMF comparator outputs from ATA6847 */
#define BEMF_A_SetInput()   (_TRISC6 = 1)
#define BEMF_B_SetInput()   (_TRISC7 = 1)
#define BEMF_C_SetInput()   (_TRISD10 = 1)
#define BEMF_A_GetValue()   _RC6
#define BEMF_B_GetValue()   _RC7
#define BEMF_C_GetValue()   _RD10

/* PWM module control */
#define PWM_EnableAll()     (PG1CONLbits.ON = PG2CONLbits.ON = PG3CONLbits.ON = 1)
#define PWM_DisableAll()    (PG1CONLbits.ON = PG2CONLbits.ON = PG3CONLbits.ON = 0)

void SetupGPIOPorts(void);

#endif
