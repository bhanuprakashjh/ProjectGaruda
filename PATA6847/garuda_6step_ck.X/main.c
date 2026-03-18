/**
 * @file main.c
 * @brief Entry point for 6-step BLDC ESC on dsPIC33CK + ATA6847.
 *
 * Target: EV43F54A board (dsPIC33CK64MP205 + ATA6847)
 * Motor:  Selected by MOTOR_PROFILE in garuda_config.h
 *         0=Hurst DMB2424B10002 (5PP, 24V), 1=A2212 1400KV (7PP, 12V)
 *
 * Debug UART: 115200 baud on RC8(RX)/RC9(TX) via USB-UART converter.
 * Type 'h' in terminal for diagnostic command list.
 *
 * Button 1: Start motor
 * Button 2: Stop motor
 */

#include <xc.h>
#include "garuda_config.h"
#include "garuda_service.h"
#include "hal/clock.h"
#include "hal/port_config.h"
#include "hal/hal_uart.h"
#include "hal/hal_diag.h"
#include "hal/hal_spi.h"
#include "hal/hal_ata6847.h"
#include "hal/hal_adc.h"
#include "hal/hal_opa.h"
#include "hal/hal_pwm.h"
#include "hal/hal_timer1.h"
#if FEATURE_IC_ZC
#include "hal/hal_ic.h"
#include "hal/hal_com_timer.h"
#endif
#include "hal/board_service.h"
#if FEATURE_GSP
#include "gsp/gsp.h"
#endif

/* Debug print rate limiter */
static volatile uint32_t lastDebugTick = 0;
#define DEBUG_INTERVAL_MS       500
#define DEBUG_INTERVAL_IDLE_MS  5000   /* Slow prints during IDLE */
#define DEBUG_INTERVAL_VERBOSE  100

/* Heartbeat: blinks LED_RUN ~1Hz when IDLE, solid when running */
static volatile uint32_t heartbeatCounter = 0;
#define HEARTBEAT_PERIOD  50000UL  /* main loop iterations (~500ms at 100MHz) */

static void PrintStatus(void)
{
    uint32_t interval;
    if (DIAG_IsVerbose())
        interval = DEBUG_INTERVAL_VERBOSE;
    else if (gData.state == ESC_IDLE || gData.state == ESC_FAULT)
        interval = DEBUG_INTERVAL_IDLE_MS;
    else
        interval = DEBUG_INTERVAL_MS;
    if ((gData.systemTick - lastDebugTick) < interval)
        return;
    lastDebugTick = gData.systemTick;

    static const char *stateNames[] = {
        "IDLE", "ARMED", "ALIGN", "OL_RAMP", "CL", "RECOV", "FAULT"
    };

    HAL_UART_WriteString("S:");
    if (gData.state <= ESC_FAULT)
        HAL_UART_WriteString(stateNames[gData.state]);
    else
        HAL_UART_WriteU16(gData.state);

    HAL_UART_WriteString(" P:");
    HAL_UART_WriteU16(gData.potRaw);
    HAL_UART_WriteString(" V:");
    HAL_UART_WriteU16(gData.vbusRaw);
    HAL_UART_WriteString(" D:");
    HAL_UART_WriteU32(gData.duty);
    HAL_UART_WriteString(" St:");
    HAL_UART_WriteU16(gData.currentStep);

    if (gData.state >= ESC_OL_RAMP)
    {
        HAL_UART_WriteString(" Tp:");
        HAL_UART_WriteU16(gData.timing.stepPeriod);
        HAL_UART_WriteString(" ZC:");
        HAL_UART_WriteU16(gData.timing.goodZcCount);
        HAL_UART_WriteString(gData.timing.zcSynced ? " SYN" : " ---");
        HAL_UART_WriteString(" Miss:");
        HAL_UART_WriteU16(gData.timing.consecutiveMissedSteps);
        HAL_UART_WriteString(" B:");
        HAL_UART_WriteByte(BEMF_A_GetValue() ? '1' : '0');
        HAL_UART_WriteByte(BEMF_B_GetValue() ? '1' : '0');
        HAL_UART_WriteByte(BEMF_C_GetValue() ? '1' : '0');

        /* HW register readback for PWM debug */
        HAL_UART_WriteString(" DC1:");
        HAL_UART_WriteU16(PG1DC);
        HAL_UART_WriteString(" IO:");
        HAL_UART_WriteHex16(PG1IOCONL);
        HAL_UART_WriteByte('/');
        HAL_UART_WriteHex16(PG2IOCONL);
        HAL_UART_WriteByte('/');
        HAL_UART_WriteHex16(PG3IOCONL);

#if FEATURE_IC_ZC
        if (DIAG_IsVerbose())
        {
            HAL_UART_WriteString(" FP:");
            HAL_UART_WriteU16(gData.icZc.diagAccepted);
            HAL_UART_WriteString("/BK:");
            HAL_UART_WriteU16(gData.icZc.diagLcoutAccepted);
            HAL_UART_WriteByte('/');
            HAL_UART_WriteU16(gData.icZc.diagFalseZc);
            HAL_UART_WriteString(" Cyc:");
            HAL_UART_WriteU16(gData.icZc.diagPollCycles);

            /* High-resolution timing */
            HAL_UART_WriteString(" TpHR:");
            HAL_UART_WriteU16(gData.timing.stepPeriodHR);
            HAL_UART_WriteString(" ZcI:");
            HAL_UART_WriteU16(gData.timing.zcInterval);
            HAL_UART_WriteString(" ZcIHR:");
            HAL_UART_WriteU16(gData.timing.zcIntervalHR);
            HAL_UART_WriteString(" PZcI:");
            HAL_UART_WriteU16(gData.timing.prevZcInterval);

            /* Poll state machine + forced step tracking */
            HAL_UART_WriteString(" Ph:");
            HAL_UART_WriteU16(gData.icZc.phase);
            HAL_UART_WriteString(" FL:");
            HAL_UART_WriteU16(gData.icZc.filterLevel);
            HAL_UART_WriteString(" Frc:");
            HAL_UART_WriteU16(gData.timing.stepsSinceLastZc);

            /* eRPM: prefer HR when available (78x better resolution) */
            {
                uint32_t eRPM = 0;
#if FEATURE_IC_ZC
                if (gData.timing.stepPeriodHR > 0 &&
                    gData.timing.hasPrevZcHR)
                {
                    /* HR: eRPM = 10 / (stepPeriodHR × 640ns)
                     * = 10 / (stepPeriodHR × 640e-9)
                     * = 15625000 / stepPeriodHR */
                    eRPM = 15625000UL / gData.timing.stepPeriodHR;
                }
                else
#endif
                if (gData.timing.stepPeriod > 0)
                {
                    eRPM = (uint32_t)TIMER1_FREQ_HZ * 10UL /
                           gData.timing.stepPeriod;
                }
                if (eRPM > 0)
                {
                    HAL_UART_WriteString(" eRPM:");
                    HAL_UART_WriteU32(eRPM);
                }
            }

            /* Duty as percent (×10 for 1 decimal place) */
            {
                uint32_t dutyPct10 = gData.duty * 1000UL / LOOPTIME_TCY;
                HAL_UART_WriteString(" D%:");
                HAL_UART_WriteU16((uint16_t)dutyPct10);
            }

            /* Current sensing: raw signed ADC values.
             * Ia=Phase A (OA2, Gt=16), Ib=Phase B (OA3, Gt=16),
             * Ibus=DC bus (OA1, ATA6847 Gt=8). All via 3mΩ shunts.
             * Convert to mA: phase_mA = raw * 3300 / 4096 / 0.048
             *                ibus_mA  = raw * 3300 / 4096 / 0.024 */
            HAL_UART_WriteString(" Ia:");
            HAL_UART_WriteS16(gData.iaRaw);
            HAL_UART_WriteString(" Ib:");
            HAL_UART_WriteS16(gData.ibRaw);
            HAL_UART_WriteString(" Ibus:");
            HAL_UART_WriteS16(gData.ibusRaw);
            if (gData.ataIlimActive)
                HAL_UART_WriteString(" ILIM!");
        }
#endif
    }

    if (gData.state == ESC_FAULT)
    {
        HAL_UART_WriteString(" F:");
        HAL_UART_WriteU16(gData.faultCode);
    }

    HAL_UART_NewLine();
}

int main(void)
{
    /* 1. Clock — FRC → PLL → 200 MHz */
    CLOCK_Initialize();

    /* 2. GPIO — PPS assignments, analog/digital, I/O direction */
    SetupGPIOPorts();

    /* 3. UART — debug output via onboard USB-UART */
    HAL_UART_Init();

    HAL_UART_NewLine();
    HAL_UART_WriteString("=============================");
    HAL_UART_NewLine();
    HAL_UART_WriteString("Garuda 6-Step CK v1.0");
    HAL_UART_NewLine();
    HAL_UART_WriteString("dsPIC33CK64MP205 + ATA6847");
    HAL_UART_NewLine();
    HAL_UART_WriteString("=============================");
    HAL_UART_NewLine();
    HAL_UART_WriteString("Init: ");

    /* 4. SPI — for ATA6847 communication */
    HAL_SPI_Init();
    HAL_UART_WriteString("SPI.");

    /* 5. ATA6847 — gate driver register configuration */
    HAL_ATA6847_Init();
    HAL_UART_WriteString("ATA.");

    /* Verify ATA6847 communication by reading DSR1 */
    {
        uint8_t dsr1 = HAL_ATA6847_ReadReg(ATA_DSR1);
        HAL_UART_WriteString("DSR1=0x");
        HAL_UART_WriteHex8(dsr1);
        if (dsr1 == 0xFF)
        {
            HAL_UART_WriteString(" !!SPI_ERR!! ");
        }
        else if (dsr1 == 0x00)
        {
            HAL_UART_WriteString(" (GDU off-OK) ");
        }
        HAL_UART_WriteByte(' ');
    }

    /* 6a. Internal op-amps for current sensing */
    HAL_OPA_Init();

    /* 6b. ADC — phase currents, pot, Vbus */
    HAL_ADC_Init();
    HAL_UART_WriteString("ADC.");

    /* 7. PWM — center-aligned complementary, 20 kHz */
    HAL_PWM_Init();
    HAL_UART_WriteString("PWM.");

    /* 8. Timer1 — 50 µs tick for state machine */
    HAL_Timer1_Init();

#if FEATURE_IC_ZC
    /* 8b. SCCP1 fast poll timer for BEMF ZC detection */
    HAL_ZcTimer_Init();
    HAL_UART_WriteString("ZC.");

    /* 8c. SCCP4 commutation timer (hardware-timed commutation) */
    HAL_ComTimer_Init();
    HAL_UART_WriteString("CT.");
#endif

    /* 9. Board service + ESC service */
    BoardServiceInit();
    GarudaService_Init();

#if FEATURE_GSP
    GSP_Init();
#endif

    HAL_UART_WriteString("OK");
    HAL_UART_NewLine();

    /* Print computed constants for verification */
    HAL_UART_WriteString("MPER=");
    HAL_UART_WriteU16(LOOPTIME_TCY);
    HAL_UART_WriteString(" DT=");
    HAL_UART_WriteU16(DEADTIME_TCY);
    HAL_UART_WriteString(" MaxDuty=");
    HAL_UART_WriteU16(MAX_DUTY);
    HAL_UART_WriteString(" AlignDuty=");
    HAL_UART_WriteU16(ALIGN_DUTY);
    HAL_UART_NewLine();

    HAL_UART_WriteString("AlignTime=");
    HAL_UART_WriteU16(ALIGN_TIME_COUNTS);
    HAL_UART_WriteString(" InitStep=");
    HAL_UART_WriteU16(INITIAL_STEP_PERIOD);
    HAL_UART_WriteString(" MinStep=");
    HAL_UART_WriteU16(MIN_STEP_PERIOD);
    HAL_UART_WriteString(" RampCap=");
    HAL_UART_WriteU16(RAMP_DUTY_CAP);
    HAL_UART_NewLine();

    HAL_UART_WriteString("PLL LOCK=");
    HAL_UART_WriteByte(OSCCONbits.LOCK ? '1' : '0');
    HAL_UART_WriteString(" COSC=");
    HAL_UART_WriteU16(OSCCONbits.COSC);
    HAL_UART_NewLine();

    HAL_UART_NewLine();
    HAL_UART_WriteString("BTN1=Start BTN2=Stop  h=Diag help");
    HAL_UART_NewLine();

    /* LEDs off at startup */
    LED_RUN = 0;
    LED_FAULT = 0;

    /* Main loop */
    while (1)
    {
        BoardService();

        /* Poll pot/Vbus via software trigger during IDLE (no PWM triggers) */
        if (gData.state == ESC_IDLE)
        {
            HAL_ADC_PollPotVbus((uint16_t *)&gData.potRaw,
                                (uint16_t *)&gData.vbusRaw);
        }

#if !FEATURE_GSP
        /* Process UART commands (debug mode only) */
        if (HAL_UART_IsRxReady())
        {
            uint8_t cmd = HAL_UART_ReadByte();
            DIAG_ProcessCommand(cmd);
        }
#endif

        /* Button 1: Start motor */
        if (IsPressed_Button1())
        {
            if (gData.state == ESC_IDLE)
            {
#if !FEATURE_GSP
                HAL_UART_WriteString(">> START");
                HAL_UART_NewLine();
#endif
                GarudaService_StartMotor();
            }
        }

        /* Button 2: Stop motor */
        if (IsPressed_Button2())
        {
#if !FEATURE_GSP
            HAL_UART_WriteString(">> STOP");
            HAL_UART_NewLine();
#endif
            GarudaService_StopMotor();
        }

#if !FEATURE_GSP
        /* Log state transitions immediately */
        if (gStateChanged)
        {
            gStateChanged = false;
            HAL_UART_WriteString("-> ");
            PrintStatus();
            lastDebugTick = gData.systemTick;
        }

        /* Periodic status output (IDLE: every 5s, active: every 500ms) */
        PrintStatus();
#endif

        /* Heartbeat LED — blink when IDLE, solid ON when running */
        heartbeatCounter++;
        if (heartbeatCounter >= HEARTBEAT_PERIOD)
        {
            heartbeatCounter = 0;
            if (gData.state == ESC_IDLE)
                LED_RUN ^= 1;   /* toggle ~1Hz blink */
        }

        /* Housekeeping */
        GarudaService_MainLoop();

#if FEATURE_GSP
        GSP_Service();
#endif
    }

    return 0;
}
