/**
 * @file port_config.h
 * @brief GPIO and PPS configuration for EV92R69A + EV68M17A.
 *
 * dsPIC33AK128MC106 pin assignments derived from:
 *   - EV92R69A user guide (DS50003904) appendix A.3 — DIM connector
 *     schematic shows which DIM pins carry ATA6847L SPI/nIRQ + BEMF.
 *   - EV68M17A DIM info sheet (DS70005527) §5 — DIM↔AK pin map.
 *
 * PWM: PG1H/L=RD2/RD3, PG2H/L=RD0/RD1, PG3H/L=RC3/RC4.
 */
#ifndef HAL_PORT_CONFIG_H
#define HAL_PORT_CONFIG_H

#include <xc.h>
#include "../garuda_config.h"

/* ── SPI chip select (ATA6847L nCS == GATE_DVR_ENABLE) ────────────── */
/* AN6285 table 3-1 row "GATE_DVR_ENABLE": DIM 8 → CK RC12 (RP60).
 * DS70005527 (AK DIM info sheet) DIM 8 row: RC5/RP38. So nCS = RC5. */
#define nCS_SetOutput()     (_TRISC5 = 0)
#define nCS                 _LATC5
#define nCS_Enable()        (nCS = 0)
#define nCS_Disable()       (nCS = 1)

/* ── nIRQ from ATA6847L ──────────────────────────────────────────── */
/* AN6285: nIRQ on DIM 40 → CK RC13 (RP61).
 * DS70005527 DIM 40 row: RB11/RP28. GPIO input + IOC.
 * MUST be GPIO not SPI2 SDI — see hal_spi.c header comment. */
#define nIRQ_SetInput()     (_TRISB11 = 1)
#define nIRQ_GetValue()     (_RB11)

/* ── LEDs (placeholders — verify on EV92R69A) ─────────────────────── */
#define LED_FAULT_SetOutput()  (_TRISC9 = 0)
#define LED_FAULT              _LATC9
#define LED_RUN_SetOutput()    (_TRISD5 = 0)
#define LED_RUN                _LATD5

/* ── Push buttons on EV68M17A DIM (active-low) ────────────────────── */
/* SW1 : DIM 34 → RD9   SW2 : DIM 36 → RD10
 * Source: dspic33AKESC/hal/port_config.c (same DIM). */
#define BTN1_SetInput()     (_TRISD9 = 1)
#define BTN1_GetValue()     (_RD9)
#define BTN2_SetInput()     (_TRISD10 = 1)
#define BTN2_GetValue()     (_RD10)

/* ── BEMF outputs from ATA6847L (3 phases) ────────────────────────
 * Per AN6285 §2 (M1_VA/B/C ↔ BEMF1/2/3 pin map):
 *   DIM 9  → BEMF1 (Phase A)
 *   DIM 11 → BEMF2 (Phase B)
 *   DIM 22 → BEMF3 (Phase C)
 * Per DS70005527 (AK DIM info sheet):
 *   DIM 9  → RB9  (AD2AN10, RP26)
 *   DIM 11 → RB8  (AD1AN11, RP25)
 *   DIM 22 → RA10 (AD2AN7,  RP11)
 *
 * ATA6847L drives these pins as DIGITAL comparator outputs when
 * GDUCR1.BEMFEN=1 (set in hal_ata6847.c). ANSEL must be 0 → digital. */
#define BEMF_A_SetInput()   do { ANSELBbits.ANSELB9  = 0; _TRISB9  = 1; } while(0)
#define BEMF_B_SetInput()   do { ANSELBbits.ANSELB8  = 0; _TRISB8  = 1; } while(0)
#define BEMF_C_SetInput()   do { ANSELAbits.ANSELA10 = 0; _TRISA10 = 1; } while(0)
#define BEMF_A_GetValue()   _RB9
#define BEMF_B_GetValue()   _RB8
#define BEMF_C_GetValue()   _RA10

/* RP numbers for PPS routing of BEMF GPIO → CCPx IC input (V4 CCP
 * diag path; off by default but sector_pi.c references the constants). */
#define BEMF_A_RP           26U
#define BEMF_B_RP           25U
#define BEMF_C_RP           11U

/* ── PWM module control (AK PG1/2/3, identical to CK API) ─────────── */
#define PWM_EnableAll()     (PG1CONLbits.ON = PG2CONLbits.ON = PG3CONLbits.ON = 1)
#define PWM_DisableAll()    (PG1CONLbits.ON = PG2CONLbits.ON = PG3CONLbits.ON = 0)

void SetupGPIOPorts(void);

#endif
