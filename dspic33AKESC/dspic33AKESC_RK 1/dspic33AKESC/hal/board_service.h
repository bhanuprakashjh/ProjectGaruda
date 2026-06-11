/**
 * @file board_service.h
 *
 * @brief Board service interface — button handling, heartbeat LED,
 * peripheral initialization, PWM output control.
 * Adapted from AN1292 reference for Project Garuda ESC.
 *
 * Component: BOARD SERVICE
 */

#ifndef __BOARD_SERVICE_H
#define __BOARD_SERVICE_H

#include <stdint.h>
#include <stdbool.h>
#include <xc.h>

#include "clock.h"
#include "hal_comparator.h"
#include "hal_pwm.h"
#include "hal_adc.h"
#include "port_config.h"
#include "timer1.h"
#include "delay.h"
#include "../garuda_config.h"
#include "../garuda_calc_params.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Button scanning states */
typedef enum tagBUTTON_STATE
{
    BUTTON_NOT_PRESSED = 0,
    BUTTON_PRESSED = 1,
    BUTTON_DEBOUNCE = 2
} BUTTON_STATE;

typedef struct
{
    BUTTON_STATE state;
    uint16_t debounceCount;
    bool logicState;
    bool status;
} BUTTON_T;

/* Timing definitions */
#define HEART_BEAT_LED_mSec         250
#define BOARD_SERVICE_TICK_mSec     1
#define BUTTON_DEBOUNCE_mSec        30

/* Timer1 period is 100us */
#define HEART_BEAT_LED_COUNT        (HEART_BEAT_LED_mSec * 1000 / TIMER1_PERIOD_uSec)
#define BOARD_SERVICE_TICK_COUNT    (BOARD_SERVICE_TICK_mSec * 1000 / TIMER1_PERIOD_uSec)
#define BUTTON_DEBOUNCE_COUNT       (BUTTON_DEBOUNCE_mSec / BOARD_SERVICE_TICK_mSec)

void BoardServiceInit(void);
void BoardServiceStepIsr(void);
void BoardService(void);
bool IsPressed_Button1(void);
bool IsPressed_Button2(void);
void HAL_InitPeripherals(void);
void HAL_ResetPeripherals(void);
void HAL_MC1PWMDisableOutputs(void);
void HAL_MC1PWMEnableOutputs(void);
void HAL_MC1ClearPWMPCIFault(void);
void HAL_TrapHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_SERVICE_H */
