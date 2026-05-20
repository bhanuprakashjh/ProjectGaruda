/**
 * @file foc_runtime.c
 * @brief AN1078 SMO FOC runtime — boot + ADC-ISR glue (see header).
 *
 * State flow:
 *   boot:  FOC_Init()
 *   SW1:   FOC_StartMotor() → ATA6847 normal + PWM enabled + overrides
 *          asserted (FETs OFF) + ADC ISR enabled. gEscState = ESC_ARMED.
 *   ISR:   first ESC_ARMED tick with cal_done → AN_MotorStart →
 *          AN_MODE_LOCK, release overrides, duties drive FETs.
 *   SW2:   FOC_StopMotor() → AN_MotorStop → override reassert + PWM off.
 *
 * ADC ISR runs while gEscState ≥ ESC_ARMED. Calibration (cal_done) is
 * accumulated by AN_MotorFastTick during the first ~64 ticks of zero
 * currents (ATA6847 in standby → no drive → clean offset measurement).
 */

#include "../garuda_config.h"

#if FEATURE_FOC_AN1078

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "foc_runtime.h"
#include "an1078_motor.h"
#include "../garuda_types.h"
#include "../hal/hal_pwm.h"
#include "../hal/hal_adc.h"
#include "../hal/hal_opa.h"
#include "../hal/hal_ata6847.h"
#include "../hal/hal_uart.h"
#include "../hal/port_config.h"      /* LED_RUN / LED_FAULT */

extern volatile ESC_STATE_T gEscState;

/* Single global motor instance. Non-static so future GSP hooks can poke
 * SMC gains without re-init. */
AN_Motor_T s_foc_an;

void FOC_Init(void)
{
    AN_MotorInit(&s_foc_an);
}

void FOC_StartMotor(void)
{
    if (gEscState != ESC_IDLE) return;

    HAL_UART_WriteString("FOC:clr ");
    HAL_ATA6847_ClearFaults();
    { volatile uint32_t d; for (d = 0; d < 50000UL; d++); }
    HAL_ATA6847_ClearFaults();

    HAL_UART_WriteString("gdu ");
    if (!HAL_ATA6847_EnterGduNormal())
    {
        { volatile uint32_t d; for (d = 0; d < 100000UL; d++); }
        HAL_ATA6847_ClearFaults();
        { volatile uint32_t d; for (d = 0; d < 50000UL; d++); }
    }
    if (!HAL_ATA6847_EnterGduNormal())
    {
        HAL_UART_WriteString("FAIL!\r\n");
        gEscState = ESC_FAULT;
        return;
    }

    /* PWM hardware ON with overrides asserted = all FETs OFF (PG1IOCONL
     * was set to 0x3000 with OVRDAT=00 in HAL_PWM_Init). Safe state
     * until AN_MotorStart's LOCK transition releases the overrides. */
    HAL_UART_WriteString("pwm ");
    HAL_PWM_ForceAllFloat();        /* belt-and-braces: re-assert overrides */
    HAL_PWM_EnableOutputs();
    HAL_PWM_ChargeBootstrap();
    { volatile uint32_t d; for (d = 0; d < 200000UL; d++); }  /* ~20ms */
    HAL_PWM_ForceAllFloat();        /* drop bootstrap, FETs OFF for cal */
    HAL_ATA6847_ClearFaults();
    { volatile uint32_t d; for (d = 0; d < 50000UL; d++); }

    HAL_UART_WriteString("adc ");
    HAL_OPA_Enable();
    HAL_ADC_InterruptEnable();

    LED_RUN = 1;
    LED_FAULT = 0;

    /* Hand off to the ISR — it observes ESC_ARMED and starts AN1078
     * once cal_done flips true (about 2 ms at 30 kHz). */
    gEscState = ESC_ARMED;
    HAL_UART_WriteString("FOC:ARMED\r\n");
}

void FOC_StopMotor(void)
{
    AN_MotorStop(&s_foc_an);
    HAL_ADC_InterruptDisable();
    HAL_PWM_DisableOutputs();
    HAL_OPA_Disable();
    HAL_ATA6847_EnterGduStandby();
    gEscState = ESC_IDLE;
    LED_RUN = 0;
}

void FOC_AdcIsrTick(uint16_t raw_ia, uint16_t raw_ib,
                    uint16_t raw_vbus, uint16_t throttle)
{
    /* Fault propagation — main loop / button / GSP can set ESC_FAULT.
     * Stop the algorithm but keep ticking through it so cal can re-arm
     * after a clear. */
    if (gEscState == ESC_FAULT && s_foc_an.mode != AN_MODE_STOPPED) {
        AN_MotorStop(&s_foc_an);
    }
    if (gEscState == ESC_IDLE && s_foc_an.mode != AN_MODE_STOPPED) {
        AN_MotorStop(&s_foc_an);
    }
    if (gEscState == ESC_ARMED &&
        s_foc_an.mode == AN_MODE_STOPPED &&
        s_foc_an.cal_done) {
        AN_MotorStart(&s_foc_an);
    }

    float da, db, dc;
    AN_MotorFastTick(&s_foc_an,
                     raw_ia, raw_ib,
                     raw_vbus, throttle,
                     &da, &db, &dc);

    /* PWM output management. Mirror of dspic33AKESC pattern:
     *   - first LOCK tick → release overrides, FETs follow duty
     *   - FAULT          → zero-vector (0.5/0.5/0.5)
     *   - STOPPED        → re-assert overrides, FETs OFF */
    static bool an_ovr_released = false;
    static AN_Mode_t last_mode_for_log = AN_MODE_STOPPED;
    if (s_foc_an.mode >= AN_MODE_LOCK && s_foc_an.mode <= AN_MODE_CLOSED_LOOP) {
        if (!an_ovr_released) {
            HAL_PWM_ReleaseAllOverrides();
            an_ovr_released = true;
        }
        HAL_PWM_SetDutyFloat3Phase(da, db, dc);
    } else if (s_foc_an.mode == AN_MODE_FAULT && an_ovr_released) {
        HAL_PWM_SetDutyFloat3Phase(0.5f, 0.5f, 0.5f);
    } else if (s_foc_an.mode == AN_MODE_STOPPED && an_ovr_released) {
        HAL_PWM_ForceAllFloat();
        an_ovr_released = false;
        HAL_UART_WriteString("FOC:STOPPED prev=");
        HAL_UART_WriteByte('0' + (uint8_t)last_mode_for_log);
        HAL_UART_WriteString("\r\n");
    }
    last_mode_for_log = s_foc_an.mode;

    /* ESC state mirror so the rest of the firmware sees something sane. */
    if (gEscState != ESC_FAULT) {
        switch (s_foc_an.mode) {
            case AN_MODE_STOPPED:
                /* Stay in ESC_ARMED until cal_done flips us into LOCK. */
                break;
            case AN_MODE_LOCK:        gEscState = ESC_ALIGN;         break;
            case AN_MODE_OPEN_LOOP:   gEscState = ESC_OL_RAMP;       break;
            case AN_MODE_CLOSED_LOOP: gEscState = ESC_CLOSED_LOOP;   break;
            case AN_MODE_FAULT:      gEscState = ESC_FAULT;         break;
        }
    }

    if (s_foc_an.mode == AN_MODE_FAULT) {
        HAL_PWM_ForceAllFloat();
        LED_FAULT = 1;
    }
}

/* LED_RUN blink pattern lets you read the AN1078 mode without UART.
 * Called from main loop ~10 Hz (same rate as FOC_DebugPrint).
 *   STOPPED   → off
 *   LOCK      → solid on
 *   OPEN_LOOP → blink slow  (toggle every 5 calls = ~500 ms period)
 *   CL        → blink fast  (toggle every call = ~100 ms period)
 *   FAULT     → blink very fast + LED_FAULT solid
 * Watch the run LED while pressing SW1 — if you see slow blink that
 * never speeds up, handoff isn't firing. Fast blink = CL reached. */
void FOC_BlinkTick(void);
void FOC_BlinkTick(void)
{
    static uint8_t cnt = 0;
    cnt++;
    switch (s_foc_an.mode) {
        case AN_MODE_STOPPED:    LED_RUN = 0;                       break;
        case AN_MODE_LOCK:       LED_RUN = 1;                       break;
        case AN_MODE_OPEN_LOOP:  if ((cnt % 5U) == 0) LED_RUN ^= 1; break;
        case AN_MODE_CLOSED_LOOP:                    LED_RUN ^= 1;  break;
        case AN_MODE_FAULT:      LED_RUN ^= 1; LED_FAULT = 1;       break;
        default: break;
    }
}

/* One-line snapshot of observer state. Reads the AN1078 instance
 * non-atomically — fine for diagnostics, the values are floats updated
 * far slower than the read. Format:
 *     F m=<mode> ω=<rad/s> |E|²=<scaled> dw=<dwell> cal=<0|1> th=<x10>
 * Print on mode change OR forced (every call). */
void FOC_DebugPrint(void)
{
    static AN_Mode_t last_mode = (AN_Mode_t)0xFF;
    int mode_changed = (s_foc_an.mode != last_mode);
    last_mode = s_foc_an.mode;

    /* BEMF magnitude squared, scaled 1000× — keeps integer math.
     * EalphaFinal/EbetaFinal are in volts; squared sum scaled by 1000
     * fits comfortably in uint32 across full range. */
    float ea = s_foc_an.smc.EalphaFinal;
    float eb = s_foc_an.smc.EbetaFinal;
    uint32_t bemf_sq_x1k = (uint32_t)((ea * ea + eb * eb) * 1000.0f);

    /* OmegaFltred is in elec rad/s. Clamp to int16 range for print. */
    int32_t omega = (int32_t)s_foc_an.smc.OmegaFltred;
    if (omega >  32000) omega =  32000;
    if (omega < -32000) omega = -32000;

    /* Throttle for context */
    uint16_t throttle = s_foc_an.throttle;

    HAL_UART_WriteString(mode_changed ? "*F m=" : " F m=");
    HAL_UART_WriteU16((uint16_t)s_foc_an.mode);
    HAL_UART_WriteString(" w=");
    HAL_UART_WriteS16((int16_t)omega);
    HAL_UART_WriteString(" |E|2x1k=");
    HAL_UART_WriteU32(bemf_sq_x1k);
    HAL_UART_WriteString(" dw=");
    HAL_UART_WriteU16((uint16_t)s_foc_an.handoff_dwell);
    HAL_UART_WriteString(" cal=");
    HAL_UART_WriteByte(s_foc_an.cal_done ? '1' : '0');
    HAL_UART_WriteString(" th=");
    HAL_UART_WriteU16(throttle);
    HAL_UART_WriteString("\r\n");
}

#endif /* FEATURE_FOC_AN1078 */
