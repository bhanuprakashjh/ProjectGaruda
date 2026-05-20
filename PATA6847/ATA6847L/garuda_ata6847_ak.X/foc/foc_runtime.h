/**
 * @file foc_runtime.h
 * @brief AN1078 SMO FOC runtime — boot + ADC-ISR glue.
 *
 * When FEATURE_FOC_AN1078 is enabled this module owns the single
 * AN_Motor_T instance and provides four entry points:
 *
 *   FOC_Init()         once at boot, after HAL_PWM_Init / HAL_ADC_Init
 *   FOC_StartMotor()   SW1 / GSP run — bring up gate driver + ADC ISR
 *   FOC_StopMotor()    SW2 / GSP stop — safe shutdown
 *   FOC_AdcIsrTick()   called from _AD1CH4Interrupt every PWM cycle
 *
 * When FEATURE_FOC_AN1078=0 the whole file is empty and the AK 6-step
 * code path runs unchanged.
 */
#ifndef FOC_RUNTIME_H
#define FOC_RUNTIME_H

#include "../garuda_config.h"

#if FEATURE_FOC_AN1078

#include <stdint.h>
#include "an1078_motor.h"

/* Single AN1078 instance — exposed so GSP / diagnostics can read state. */
extern AN_Motor_T s_foc_an;

void FOC_Init(void);
void FOC_StartMotor(void);
void FOC_StopMotor(void);

/* Fast-loop hook — called from the ADC ISR with the four raw values it
 * already read. Arms AN_MotorStart on the ESC_ARMED → cal_done edge,
 * runs AN_MotorFastTick, and writes the resulting duties to PG1/2/3. */
void FOC_AdcIsrTick(uint16_t raw_ia, uint16_t raw_ib,
                    uint16_t raw_vbus, uint16_t throttle);

/* Main-loop debug — prints one line per call. Rate-limit at the caller
 * (~10 Hz). Cheap UART; only meaningful when motor is armed. */
void FOC_DebugPrint(void);

/* Main-loop visual indicator — toggles LED_RUN at a mode-dependent rate
 * so the AN1078 state is readable without UART. Call at the same ~10 Hz
 * cadence as FOC_DebugPrint.  */
void FOC_BlinkTick(void);

#endif /* FEATURE_FOC_AN1078 */

#endif /* FOC_RUNTIME_H */
