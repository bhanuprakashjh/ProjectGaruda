/**
 * @file main.c
 * @brief Entry point for 6-step BLDC ESC on dsPIC33AK + ATA6847L.
 *
 * Init order (each step depends on the previous):
 *   clock → GPIO → UART → SPI → ATA6847L → PWM/ADC/Timer1 → IRQ enable.
 *
 * Target: EV92R69A (ATA6847L) + EV68M17A (dsPIC33AK128MC106).
 * Motor:  Selected by MOTOR_PROFILE in garuda_config.h
 *         0=Hurst, 1=A2212, 2=2810, 3=HiZ1460.
 *
 * Debug UART: 115200 baud on RC11(RX)/RC10(TX) via USB-UART converter.
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
#include "hal/hal_ptg.h"
#include "hal/board_service.h"
#include "motor/sector_pi.h"
extern volatile ESC_STATE_T gEscState;
#if FEATURE_GSP
#include "gsp/gsp.h"
#include "gsp/gsp_params.h"
#endif
#if FEATURE_FOC_AN1078
#include "foc/foc_runtime.h"
#endif

/* Debug print rate limiter */
static volatile uint32_t lastDebugTick = 0;
#define DEBUG_INTERVAL_MS       500
#define DEBUG_INTERVAL_IDLE_MS  5000   /* Slow prints during IDLE */
#define DEBUG_INTERVAL_VERBOSE  100

/* Heartbeat: blinks LED_RUN ~1Hz when IDLE, solid when running */
static volatile uint32_t heartbeatCounter = 0;
#define HEARTBEAT_PERIOD  50000UL  /* main loop iterations (~500ms at 100MHz) */

/* Free-running main-loop iteration counter — exposed via telemetry to
 * diagnose main-loop starvation (frames stuck at fixed RPM, etc).
 * If telemetry stalls while this counter still grows between frames,
 * main loop is fine and TelemTick is being skipped or the GSP TX path
 * is blocked. If counter freezes between frames, main loop itself is
 * blocked by an ISR storm. Reset on motor start in GarudaService. */
volatile uint32_t gMainLoopHb = 0;


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
    HAL_UART_WriteString("dsPIC33AK128MC106 + ATA6847L");
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

    /* V4: sector timer + capture timer init happens in SectorPI_Init() */
    HAL_UART_WriteString("V4.");

    /* PTG: fires _PTG0Interrupt at every PG1TRIGB match. Started here
     * once and left running for the lifetime of the firmware —
     * SectorPI_Start/Stop call HAL_PTG_Start/Stop which just reset counters.
     *
     * Skipped in FOC mode — PTG drives the 6-step BEMF sampler that the
     * AN1078 path doesn't need. */
#if !FEATURE_FOC_AN1078
    HAL_PTG_Init();
    HAL_PTG_Start();
    HAL_UART_WriteString("PTG.");
#endif

    /* 9. Board service + ESC service */
    BoardServiceInit();
    GarudaService_Init();

#if FEATURE_FOC_AN1078
    FOC_Init();
    HAL_UART_WriteString("FOC.");
#endif

#if FEATURE_GSP
    CK_ParamsInitDefaults();
    GSP_Init();
#endif

    HAL_UART_WriteString("OK");
    HAL_UART_NewLine();

    HAL_UART_WriteString("ESC — MPER=");
    HAL_UART_WriteU16(LOOPTIME_TCY);
    HAL_UART_WriteString(" InitSector=");
    HAL_UART_WriteU16(ERPM_TO_PERIOD(STARTUP_SPEED_ERPM));
    HAL_UART_NewLine();

    /* AK clock status lives in PLL1CONbits / CLK1CONbits (no OSCCONbits
     * .LOCK/COSC on this MCU). Print PLL1.CLKRDY + CLK1 NOSC. */
    HAL_UART_WriteString("PLL RDY=");
    HAL_UART_WriteByte(PLL1CONbits.CLKRDY ? '1' : '0');
    HAL_UART_WriteString(" NOSC=");
    HAL_UART_WriteU16(CLK1CONbits.NOSC);
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
        gMainLoopHb++;
        BoardService();

        /* V4: pot/Vbus updated from the ADC ISR while motor runs. In IDLE
         * they stay at 0; press the run button with the pot held where
         * you want and the OL_RAMP samples it after entering active. */

#if !FEATURE_GSP
        if (HAL_UART_IsRxReady())
        {
            uint8_t cmd = HAL_UART_ReadByte();
            DIAG_ProcessCommand(cmd);
        }
#endif

        if (IsPressed_Button1())
        {
            if (gEscState == ESC_IDLE) {
#if FEATURE_FOC_AN1078
                FOC_StartMotor();
#else
                GarudaService_StartMotor();
#endif
            }
        }
        if (IsPressed_Button2()) {
#if FEATURE_FOC_AN1078
            FOC_StopMotor();
#else
            GarudaService_StopMotor();
#endif
        }

        heartbeatCounter++;
        if (heartbeatCounter >= HEARTBEAT_PERIOD)
        {
            heartbeatCounter = 0;
            if (gEscState == ESC_IDLE)
                LED_RUN ^= 1;
        }

#if FEATURE_FOC_AN1078
        /* FOC observer debug print + LED mode-blink at ~10 Hz once armed.
         * Tied to the heartbeat counter — fires N×finer than the LED
         * heartbeat toggle. The LED_RUN blink rate is the primary
         * visual indicator (no UART required): off=STOPPED, solid=LOCK,
         * slow=OL, fast=CL, frantic=FAULT. */
        {
            static uint32_t focDbgCnt = 0;
            if (++focDbgCnt >= (HEARTBEAT_PERIOD / 5UL)) {
                focDbgCnt = 0;
                if (gEscState != ESC_IDLE) {
                    FOC_BlinkTick();
                    FOC_DebugPrint();
                }
            }
        }
#endif

        /* Housekeeping */
        GarudaService_MainLoop();

#if FEATURE_GSP
        GSP_Service();
#endif
    }

    return 0;
}
