/**
 * @file garuda_service.c
 * @brief Main ESC service: state machine, ADC ISR, Timer1 ISR.
 *
 * Timer1 ISR (20 kHz / 50 µs):
 *   - State machine: IDLE→ARMED→ALIGN→OL_RAMP→CLOSED_LOOP
 *   - BEMF ZC timeout checking
 *   - Duty slew rate control
 *   - systemTick increment (1 ms from divide-by-20)
 *
 * ADC ISR (20 kHz, triggered by PWM):
 *   - Read pot, Vbus, phase currents
 *   - BEMF ZC polling (digital comparator)
 *   - Commutation deadline checking (50 µs resolution)
 *   - Vbus fault checking
 *
 * Target: dsPIC33CK64MP205 on EV43F54A board.
 */

#include <xc.h>
#include "garuda_service.h"
#include "garuda_config.h"
#include "hal/hal_pwm.h"
#include "hal/hal_adc.h"
#include "hal/hal_ata6847.h"
#include "hal/hal_timer1.h"
#include "hal/hal_opa.h"
#include "hal/hal_uart.h"
#include "hal/port_config.h"
#include "motor/commutation.h"
#include "motor/startup.h"
#include "motor/bemf_zc.h"
#if FEATURE_IC_ZC
#include "hal/hal_ic.h"
#include "hal/hal_com_timer.h"
#endif

/* Global ESC runtime data */
volatile GARUDA_DATA_T gData;

/* State change flag — set in ISR, consumed in main loop for debug print */
volatile bool gStateChanged = false;
volatile ESC_STATE_T gPrevState = ESC_IDLE;

/* Deferred flags — ISR sets, main loop executes (no SPI in ISR) */
static volatile bool deferredStop = false;
static volatile bool deferredRestart = false;
static volatile FAULT_CODE_T deferredFault = FAULT_NONE;

/* System tick counter for 1ms from Timer1 (divides 20kHz by 20) */
static volatile uint8_t tickDiv = 0;

/* Vbus UV debounce: require 10 consecutive UV readings (500µs at 20kHz) */
#define VBUS_UV_DEBOUNCE  10
static volatile uint8_t uvDebounce = 0;

/* Duty slew rate limiter: max duty change per Timer1 tick (50µs @ 20kHz).
 * Asymmetric: fast UP for responsive throttle, slow DOWN for gentle decel.
 * UP: 25 counts/tick → full range in ~20ms
 * DOWN: 10 counts/tick → ramp-to-idle in ~17ms
 *   This prevents ZC desync during CL entry and pot-down sweeps. */
#define DUTY_SLEW_UP     3U   /* ~200ms full range (was 25 = 20ms).
                              * Slow enough for stepPeriod IIR to track.
                              * Fast pot sweep at 25 caused acceleration
                              * faster than ZC Layer 2 could follow →
                              * real ZCs rejected as "too early" → desync. */
#define DUTY_SLEW_DOWN   5U   /* ~100ms ramp down (was 10 = 50ms) */
static volatile uint32_t slewedDuty = 0;
static volatile uint32_t rampExitDuty = 0;  /* duty at OL→CL transition, used as ACQUIRE cap */

/* CL settle: hold ramp duty for N ticks after CL entry before switching
 * to pot control. Lets ZC tracking stabilize after OL→CL handoff. */
#define CL_SETTLE_TICKS 2000U  /* 100ms at 20kHz Timer1 */
static volatile uint16_t clSettleCounter = 0;

/* ── Throttle Mapping ─────────────────────────────────────────────── */

/**
 * @brief Map pot ADC reading (0-4095) to duty cycle.
 * Dead zone at low end (pot < 200 → zero throttle).
 * Linear mapping from dead zone to MAX_DUTY.
 */
static uint32_t MapThrottleToDuty(uint16_t potRaw)
{
    /* ADC fractional format: 12-bit left-justified in 16-bit → 0..65520.
     * Dead zone below ~3% pot travel → returns CL_IDLE_DUTY (not zero).
     * Linear map from CL_IDLE_DUTY to MAX_DUTY above dead zone. */
    #define POT_DEADZONE  2000u
    #define POT_MAX       64000u

    if (potRaw < POT_DEADZONE)
        return CL_IDLE_DUTY;

    uint32_t range = POT_MAX - POT_DEADZONE;
    uint32_t dutyRange = MAX_DUTY - CL_IDLE_DUTY;
    uint32_t adj = (potRaw > POT_MAX) ? POT_MAX - POT_DEADZONE
                                      : potRaw - POT_DEADZONE;
    uint32_t duty = CL_IDLE_DUTY + (adj * dutyRange) / range;

    if (duty > MAX_DUTY) duty = MAX_DUTY;
    return duty;
}

/* ── Fault Handling ───────────────────────────────────────────────── */

/* ISR-safe fault entry — just kills PWM and sets state, no SPI */
static void EnterFaultISR(FAULT_CODE_T code)
{
    gData.state = ESC_FAULT;
    gData.faultCode = code;
    HAL_PWM_DisableOutputs();
#if FEATURE_IC_ZC
    HAL_ZcTimer_Stop();
    HAL_ComTimer_Cancel();
#endif
    deferredFault = code;  /* main loop will do SPI standby */
    LED_FAULT = 1;
    LED_RUN = 0;
}

/* Full fault entry — safe to call from main loop only */
static void EnterFault(FAULT_CODE_T code)
{
    gData.state = ESC_FAULT;
    gData.faultCode = code;
    HAL_PWM_DisableOutputs();
    HAL_ATA6847_EnterGduStandby();
    LED_FAULT = 1;
    LED_RUN = 0;
}

static void EnterRecovery(void)
{
    HAL_PWM_DisableOutputs();
#if FEATURE_IC_ZC
    HAL_ZcTimer_Stop();
    HAL_ComTimer_Cancel();
#endif
    gData.state = ESC_RECOVERY;
    gData.recoveryCounter = RECOVERY_COUNTS;
}

/* ── Initialization ───────────────────────────────────────────────── */

void GarudaService_Init(void)
{
    gData.state = ESC_IDLE;
    gData.currentStep = 0;
    gData.direction = 0;   /* CW */
    gData.duty = 0;
    gData.vbusRaw = 0;
    gData.potRaw = 0;
    gData.throttle = 0;
    gData.faultCode = FAULT_NONE;
    gData.systemTick = 0;
    gData.timer1Tick = 0;
    gData.armCounter = 0;
    gData.runCommandActive = false;
    gData.gspThrottleActive = false;
    gData.gspThrottleValue = 0;
    gData.desyncRestartAttempts = 0;
    gData.recoveryCounter = 0;
    BEMF_ZC_Init((volatile GARUDA_DATA_T *)&gData);

    /* Start Timer1 for systemTick — always runs */
    HAL_Timer1_Start();
}

/* ── Motor Start/Stop ─────────────────────────────────────────────── */

void GarudaService_StartMotor(void)
{
    if (gData.state != ESC_IDLE)
        return;

    gData.runCommandActive = true;
    gData.desyncRestartAttempts = 0;

    HAL_UART_WriteString("SM:clr ");

    /* Clear ATA6847 faults */
    HAL_ATA6847_ClearFaults();

    HAL_UART_WriteString("gdu ");

    /* Power up GDU */
    if (!HAL_ATA6847_EnterGduNormal())
    {
        HAL_UART_WriteString("FAIL!");
        HAL_UART_NewLine();
        /* Dump fault registers to diagnose why GDU won't enter Normal */
        uint8_t diag[8];
        HAL_ATA6847_ReadDiag(diag);
        HAL_UART_WriteString("DSR1=");
        HAL_UART_WriteHex8(diag[0]);
        HAL_UART_WriteString(" DSR2=");
        HAL_UART_WriteHex8(diag[1]);
        HAL_UART_WriteString(" SIR1=");
        HAL_UART_WriteHex8(diag[2]);
        HAL_UART_WriteString(" SIR2=");
        HAL_UART_WriteHex8(diag[3]);
        HAL_UART_WriteString(" SIR3=");
        HAL_UART_WriteHex8(diag[4]);
        HAL_UART_WriteString(" SIR4=");
        HAL_UART_WriteHex8(diag[5]);
        HAL_UART_WriteString(" SIR5=");
        HAL_UART_WriteHex8(diag[6]);
        HAL_UART_WriteString(" GOPMCR=");
        HAL_UART_WriteHex8(diag[7]);
        HAL_UART_NewLine();
        EnterFault(FAULT_ATA6847);
        return;
    }

    /* Read back GDU status for debug */
    {
        uint8_t dsr1 = HAL_ATA6847_ReadReg(ATA_DSR1);
        HAL_UART_WriteString("DSR1=0x");
        HAL_UART_WriteHex8(dsr1);
        HAL_UART_WriteByte(' ');
    }

    HAL_UART_WriteString("pwm ");

    /* Charge bootstrap caps */
    HAL_PWM_EnableOutputs();
    HAL_PWM_ChargeBootstrap();

    /* Small delay for bootstrap charge */
    /* The Timer1 ISR will handle the actual ARM→ALIGN transition */

    HAL_UART_WriteString("adc ");

    /* Enable op-amps and ADC ISR (ADC module already ON from init) */
    HAL_OPA_Enable();
    HAL_ADC_InterruptEnable();

    /* Timer1 already running (started in Init) */

    /* Enter armed state */
    gData.state = ESC_ARMED;
    gData.armCounter = ARM_TIME_COUNTS;
    uvDebounce = 0;

    HAL_UART_WriteString("ARMED");
    HAL_UART_NewLine();

    LED_RUN = 1;
    LED_FAULT = 0;
}

void GarudaService_StopMotor(void)
{
    gData.runCommandActive = false;

    HAL_PWM_DisableOutputs();
#if FEATURE_IC_ZC
    HAL_ZcTimer_Stop();
    HAL_ComTimer_Cancel();
#endif
    /* Don't stop Timer1 — systemTick must keep running for UART debug */
    /* Don't disable ADC — pot/Vbus polling needs it during IDLE */
    HAL_ADC_InterruptDisable();
    HAL_OPA_Disable();
    HAL_ATA6847_EnterGduStandby();

    gData.state = ESC_IDLE;
    gData.duty = 0;
    gData.faultCode = FAULT_NONE;
    gData.gspThrottleActive = false;
    gData.gspThrottleValue = 0;

    LED_RUN = 0;
    LED_FAULT = 0;
}

void GarudaService_ClearFault(void)
{
    if (gData.state == ESC_FAULT)
    {
        gData.state = ESC_IDLE;
        gData.faultCode = FAULT_NONE;
        gData.runCommandActive = false;
        LED_FAULT = 0;
    }
}

/* ── Main Loop (called from while(1) in main.c) ──────────────────── */

void GarudaService_MainLoop(void)
{
    /* Track state changes from ISR for main-loop debug output */
    if (gData.state != gPrevState)
    {
        gStateChanged = true;
        gPrevState = gData.state;
    }

    /* Handle deferred stop (ISR requested, main loop executes SPI) */
    if (deferredStop)
    {
        deferredStop = false;
        GarudaService_StopMotor();
    }

    /* Handle deferred restart (recovery complete, ISR requested restart) */
    if (deferredRestart)
    {
        deferredRestart = false;
        if (gData.runCommandActive && gData.state == ESC_IDLE)
            GarudaService_StartMotor();
    }

    /* Handle deferred fault — do the SPI standby that ISR couldn't */
    if (deferredFault != FAULT_NONE)
    {
        deferredFault = FAULT_NONE;
        HAL_ATA6847_EnterGduStandby();
    }

    /* ATA6847 fault decode — PWM is already OFF (killed by ISR),
     * so SPI reads are safe (no EMI from switching). */
    if (gData.ataFaultPending)
    {
        gData.ataFaultPending = false;

        /* Put GDU to standby (SPI write — safe, PWM is off) */
        HAL_ATA6847_EnterGduStandby();

        /* Now read fault details via SPI */
        uint8_t sir1 = HAL_ATA6847_ReadReg(ATA_SIR1);
        gData.ataLastSIR1 = sir1;

        /* Decode and log specific fault */
        if (sir1 != 0xFF)  /* 0xFF = SPI timeout, skip decode */
        {
            if (sir1 & 0x02)  /* VDSSC — VDS short circuit */
            {
                gData.faultCode = FAULT_ATA6847;
                HAL_UART_WriteString("\r\n!ATA SC SIR3=");
                HAL_UART_WriteHex8(HAL_ATA6847_ReadReg(ATA_SIR3));
                HAL_UART_NewLine();
            }
            else if (sir1 & 0x04)  /* OVTF — over-temperature */
            {
                gData.faultCode = FAULT_ATA6847;
                HAL_UART_WriteString("\r\n!ATA OVT\r\n");
            }
            else if (sir1 & 0x80)  /* VSUPF — supply failure */
            {
                gData.faultCode = FAULT_ATA6847;
                HAL_UART_WriteString("\r\n!ATA VSUP\r\n");
            }
            else if (sir1 & 0x01)  /* VGSUV — gate under-voltage */
            {
                gData.faultCode = FAULT_ATA6847;
                HAL_UART_WriteString("\r\n!ATA VGSUV\r\n");
            }
            else if (sir1 & 0x10)  /* ILIM — was just chopping, not critical */
            {
                /* ILIM triggered but not a hard fault — motor was stopped
                 * by ISR as precaution. Could be spurious from EMI. */
                gData.faultCode = FAULT_ATA6847;
                HAL_UART_WriteString("\r\n!ATA ILIM\r\n");
            }
        }

        /* Clear ATA6847 fault registers */
        HAL_ATA6847_ClearFaults();
    }

    /* Fault state: do NOT auto-clear. User must press Clear Fault
     * (GUI button or BTN2) to acknowledge and return to IDLE. */
}

/* ── Timer1 ISR (20 kHz = 50 µs) ─────────────────────────────────── */

void __attribute__((interrupt, auto_psv)) _T1Interrupt(void)
{
    gData.timer1Tick++;

    /* 1 ms system tick (divide by 20 at 20 kHz Timer1) */
    tickDiv++;
    if (tickDiv >= 20)
    {
        tickDiv = 0;
        gData.systemTick++;
    }

    /* ATA6847 nIRQ — active low when fault occurs.
     * Strategy: kill PWM IMMEDIATELY in ISR (no SPI — EMI would hang it).
     * Main loop reads SPI fault details AFTER PWM is off (safe). */
    {
        static uint16_t nirqDiv = 0;
        if (++nirqDiv >= 200)  /* Check every 10ms (rate-limited for EMI) */
        {
            nirqDiv = 0;
            if (!nIRQ_GetValue() && gData.state >= ESC_OL_RAMP)
            {
                /* Kill PWM immediately — no SPI here, just registers */
                HAL_PWM_DisableOutputs();
#if FEATURE_IC_ZC
                HAL_ZcTimer_Stop();
                HAL_ComTimer_Cancel();
#endif
                gData.state = ESC_FAULT;
                gData.faultCode = FAULT_ATA6847;
                gData.ataFaultPending = true;  /* Main loop decodes via SPI */
                LED_FAULT = 1;
                LED_RUN = 0;
            }
        }
    }

    switch (gData.state)
    {
        case ESC_IDLE:
            break;

        case ESC_ARMED:
            if (gData.armCounter > 0)
            {
                gData.armCounter--;
            }
            else
            {
                /* ARM complete → enter ALIGN */
                STARTUP_Init((volatile GARUDA_DATA_T *)&gData);
                BEMF_ZC_Init((volatile GARUDA_DATA_T *)&gData);
                gData.state = ESC_ALIGN;
            }
            break;

        case ESC_ALIGN:
            if (STARTUP_Align((volatile GARUDA_DATA_T *)&gData))
            {
                /* Alignment done → enter OL ramp */
                gData.state = ESC_OL_RAMP;
                gData.rampCounter = gData.rampStepPeriod;
                BEMF_ZC_OnCommutation((volatile GARUDA_DATA_T *)&gData);
            }
            break;

        case ESC_OL_RAMP:
        {
            uint8_t prevStep = gData.currentStep;
            bool rampDone = STARTUP_OpenLoopRamp((volatile GARUDA_DATA_T *)&gData);

            /* If ramp commutated, reset ZC tracker for the new step */
            if (gData.currentStep != prevStep)
            {
                BEMF_ZC_OnCommutation((volatile GARUDA_DATA_T *)&gData);
                /* Keep timing.stepPeriod in sync with ramp for ZC timeout */
                gData.timing.stepPeriod = gData.rampStepPeriod;
            }

            /* NOTE: Do NOT call BEMF_ZC_CheckTimeout during OL ramp.
             * The ramp forces commutation on its own schedule.
             * ZC timeout would reset zcSynced, blocking CL transition. */

            /* Transition to closed-loop when ramp target reached AND
             * BEMF ZC is synchronized */
            if (rampDone && gData.timing.zcSynced)
            {
                gData.state = ESC_CLOSED_LOOP;
                gData.timing.consecutiveMissedSteps = 0;
                gData.timing.stepsSinceLastZc = 0;
                gData.zcCtrl.mode = ZC_MODE_ACQUIRE;
                gData.zcCtrl.acquireGoodCount = 0;
                gData.zcCtrl.recoverAttempts = 0;
                /* Phase 4: seed refIntervalHR from stepPeriodHR */
                gData.zcCtrl.refIntervalHR = gData.timing.stepPeriodHR;
                /* Capture ramp exit duty for ACQUIRE mode cap */
                rampExitDuty = gData.duty;
                gData.timing.bypassSuppressed = true;  /* Suppress bypass
                    * until first confirmed CL ZC. Prevents polarity-
                    * blind acceptance during CL entry when the estimator
                    * has no valid reference yet. */
                slewedDuty = gData.duty;  /* Seed slew from ramp exit duty */
                clSettleCounter = CL_SETTLE_TICKS;

#if FEATURE_IC_ZC
                /* Carry polling ZC history into CL.
                 * Seed stepPeriodHR from Timer1 stepPeriod for initial
                 * HR blanking/scheduling. First real HR ZC will replace. */
                gData.timing.hasPrevZcHR = false;
                {
                    uint32_t seedHR = (uint32_t)gData.timing.stepPeriod *
                                      COM_TIMER_T1_NUMER / COM_TIMER_T1_DENOM;
                    gData.timing.stepPeriodHR =
                        (seedHR <= 65535U) ? (uint16_t)seedHR : 0;
                }
                /* Seed HR checkpoint and lastZcTickHR so Layer 2 (50%
                 * interval rejection in FastPoll) and Layer 3 (50%
                 * checkpoint rejection in RecordZcTiming) are active
                 * from the first CL step.
                 *
                 * Without this, lastZcTickHR is stale (0 from init),
                 * causing elapsed to wrap large → Layer 2 bypassed →
                 * demag tail accepted as false ZC → IIR poisoned.
                 *
                 * Seed lastZcTickHR = now - stepPeriodHR so the 50%
                 * window opens at the right time: first valid ZC at
                 * ~50% of step period will pass, demag at ~10-30% won't. */
                gData.timing.checkpointStepPeriodHR =
                    gData.timing.stepPeriodHR;
                gData.timing.lastZcTickHR =
                    HAL_ComTimer_ReadTimer() - gData.timing.stepPeriodHR;
                /* Set up fast poll for the current step */
                BEMF_ZC_OnCommutation((volatile GARUDA_DATA_T *)&gData);
                /* Start SCCP1 fast poll timer */
                HAL_ZcTimer_Start();
#endif
            }
            else if (rampDone && !gData.timing.zcSynced)
            {
                /* Ramp done but no ZC sync — continue forced commutation
                 * at min step period. Check timeout only NOW (post-ramp). */
                ZC_TIMEOUT_RESULT_T tout = BEMF_ZC_CheckTimeout(
                    (volatile GARUDA_DATA_T *)&gData);
                if (tout == ZC_TIMEOUT_DESYNC)
                {
                    if (gData.desyncRestartAttempts < DESYNC_RESTART_MAX)
                    {
                        gData.desyncRestartAttempts++;
                        EnterRecovery();
                    }
                    else
                    {
                        EnterFaultISR(FAULT_DESYNC);
                    }
                }
            }
            break;
        }

        case ESC_CLOSED_LOOP:
        {
            /* CL settle: hold ramp duty for 100ms after CL entry */
            if (clSettleCounter > 0)
            {
                clSettleCounter--;
                HAL_PWM_SetDutyCycle(gData.duty);
            }
            else
            {
                uint32_t target;

                if (gData.timing.zcSynced)
                {
                    /* Synced: pot-mapped with idle floor.
                     * GSP throttle override for GUI motor testing. */
                    target = MapThrottleToDuty(
                        gData.gspThrottleActive ? gData.gspThrottleValue : gData.potRaw);

                    /* Phase 7: mode-specific duty limiting.
                     * ACQUIRE: don't increase beyond current duty (let estimator catch up)
                     * RECOVER: hold duty — don't change during recovery
                     * TRACK: normal operation */
                    switch (gData.zcCtrl.mode)
                    {
                        case ZC_MODE_ACQUIRE:
                            /* Cap duty at ramp exit level — don't accelerate
                             * until ACQUIRE → TRACK confirms estimator locked. */
                            if (target > rampExitDuty)
                                target = rampExitDuty;
                            break;
                        case ZC_MODE_RECOVER:
                            /* Hold current duty — don't accelerate during recovery */
                            if (target > slewedDuty)
                                target = slewedDuty;
                            break;
                        case ZC_MODE_TRACK:
                            /* Normal — no restriction */
                            break;
                    }
                }
                else
                {
                    /* NOT synced (per-revolution check or missed steps).
                     * Cut duty aggressively to reduce current and let motor
                     * decelerate to a speed where ZC can re-acquire.
                     * AM32-style: halve duty immediately, don't rely on slew. */
                    target = CL_IDLE_DUTY;
                    if (slewedDuty > CL_IDLE_DUTY * 2)
                        slewedDuty = slewedDuty / 2;  /* Fast cut, bypass slew */
                }

                /* Phase 9: Speed governance (ESCape32-style duty_ramp).
                 * Limit max duty based on measured eRPM so the motor can't
                 * be driven faster than the estimator can track.
                 * duty_max ramps linearly from CL_IDLE_DUTY at 0 eRPM to
                 * MAX_DUTY at DUTY_RAMP_ERPM. Above DUTY_RAMP_ERPM: no cap.
                 *
                 * This prevents the desync-on-fast-pot failure: the motor
                 * can only get more duty as it proves it can commutate at
                 * the current speed. */
#if FEATURE_IC_ZC
                {
                    uint32_t measuredErpm = 0;
                    if (gData.zcCtrl.refIntervalHR > 0)
                        measuredErpm = 15625000UL / gData.zcCtrl.refIntervalHR;

                    if (measuredErpm < DUTY_RAMP_ERPM)
                    {
                        uint32_t maxDuty = CL_IDLE_DUTY +
                            (uint32_t)(MAX_DUTY - CL_IDLE_DUTY)
                            * measuredErpm / DUTY_RAMP_ERPM;
                        if (target > maxDuty)
                            target = maxDuty;
                    }
                }
#endif

                /* Asymmetric slew: fast up, slow down */
                if (target > slewedDuty)
                {
                    uint32_t delta = target - slewedDuty;
                    slewedDuty += (delta > DUTY_SLEW_UP) ? DUTY_SLEW_UP : delta;
                }
                else if (slewedDuty > target)
                {
                    uint32_t delta = slewedDuty - target;
                    slewedDuty -= (delta > DUTY_SLEW_DOWN) ? DUTY_SLEW_DOWN : delta;
                }

                gData.duty = slewedDuty;
                HAL_PWM_SetDutyCycle(gData.duty);
            }

            /* Commutation deadline is checked in ADC ISR (20 kHz)
             * for better timing resolution. See _ADCInterrupt. */

            /* Check for ZC timeout */
            ZC_TIMEOUT_RESULT_T tout = BEMF_ZC_CheckTimeout(
                (volatile GARUDA_DATA_T *)&gData);

            if (tout == ZC_TIMEOUT_DESYNC)
            {
                /* Too many consecutive missed ZCs — desync */
                if (gData.desyncRestartAttempts < DESYNC_RESTART_MAX)
                {
                    gData.desyncRestartAttempts++;
                    EnterRecovery();
                }
                else
                {
                    EnterFaultISR(FAULT_DESYNC);
                }
            }

            /* Bypass-count desync detection. In healthy operation, bypass
             * fires 0-1 times (CL entry only). During a false-ZC cascade,
             * bypass fires hundreds of times per second as wrong-polarity
             * ZCs are accepted. Detect via rate: if bypass count grows
             * by >20 over a 10ms window, trigger recovery.
             *
             * Don't use DESYNC_RESTART_MAX — that causes permanent fault
             * after 3 attempts. Instead always recover and let the user
             * stop the motor via button if it keeps failing. */
            {
                static uint16_t prevBypassCount = 0;
                static uint16_t bypassCheckDiv = 0;
                if (++bypassCheckDiv >= 200)  /* Check every 10ms (200 × 50µs) */
                {
                    bypassCheckDiv = 0;
                    uint16_t bypassDelta = gData.zcDiag.diagBypassAccepted
                                         - prevBypassCount;
                    prevBypassCount = gData.zcDiag.diagBypassAccepted;
                    if (bypassDelta > 20)
                    {
                        EnterRecovery();
                    }
                }
            }

            /* Reset desync restart counter when stable in CL for 2 seconds */
            if (gData.timing.zcSynced && gData.systemTick > 2000 &&
                gData.desyncRestartAttempts > 0)
            {
                static uint32_t stableStartTick = 0;
                if (stableStartTick == 0)
                    stableStartTick = gData.systemTick;
                else if (gData.systemTick - stableStartTick > 2000)
                {
                    gData.desyncRestartAttempts = 0;
                    stableStartTick = 0;
                }
            }

            if (tout == ZC_TIMEOUT_FORCE_STEP)
            {
                /* Force a commutation step.
                 * Gate out higher-priority ISRs (SCCP1/SCCP4) during the
                 * transition to prevent race conditions where they see
                 * partially-updated state (old cmpExpected + new step). */
#if FEATURE_IC_ZC
                gData.bemf.zeroCrossDetected = true;  /* Block SCCP1 polling */
                gData.icZc.phase = IC_ZC_DONE;        /* Block FastPoll */
                HAL_ComTimer_Cancel();                 /* Cancel pending SCCP4 */
                gData.timing.deadlineActive = false;
#endif
                COMMUTATION_AdvanceStep((volatile GARUDA_DATA_T *)&gData);
                BEMF_ZC_OnCommutation((volatile GARUDA_DATA_T *)&gData);
            }

            /* Throttle-zero shutdown: only after pot has been raised once,
             * then dropped back to zero for 50ms */
            /* (Button stop via BTN2 is the primary stop mechanism for now) */
            break;
        }

        case ESC_RECOVERY:
            if (gData.recoveryCounter > 0)
            {
                gData.recoveryCounter--;
            }
            else
            {
                /* Retry: go back to IDLE, main loop will restart */
                HAL_PWM_DisableOutputs();
                gData.state = ESC_IDLE;
                if (gData.runCommandActive)
                    deferredRestart = true;
            }
            break;

        case ESC_FAULT:
            /* Stay in fault until cleared by main loop */
            break;
    }

    IFS0bits.T1IF = 0;
}

/* ── ADC ISR (20 kHz, triggered by PWM center) ───────────────────── */

void __attribute__((interrupt, auto_psv)) _ADCInterrupt(void)
{
    /* Read ALL ADC buffers to clear data-ready flags.
     * If any enabled channel's flag stays set, ADCIF re-asserts → ISR loops. */
    gData.potRaw = (uint16_t)ADCBUF_POT;    /* AN6 */
    gData.vbusRaw = (uint16_t)ADCBUF_VBUS;  /* AN9 */

    /* Current sensing — read all buffers (clears data-ready flags).
     * EV43F54A: RS1/RS2/RS3 = 3mΩ shunts.
     * AN0 (ADCBUF0) = DC bus current via ATA6847 OPO3 (Gt=8) → OA1
     * AN1 (ADCBUF1) = Phase A current via OA2 (Gt=16)
     * AN4 (ADCBUF4) = Phase B current via OA3 (Gt=16)
     * Signed 12-bit fractional format. */
    /* Current sensing — read all buffers (clears data-ready flags).
     * AN0 (ADCBUF0): not usable for IBus — ATA6847 CSA3 is internal only,
     *   no external pin. OA1 R46=12k feedback creates ~7000x gain → saturates.
     *   Microchip reference (EV43F54A_SMO_Lib) also doesn't use OA1 (AMPEN1=0).
     * AN1 (ADCBUF1): Phase A current via OA2 (Gt=16, 3mΩ shunt RS1)
     * AN4 (ADCBUF4): Phase B current via OA3 (Gt=16, 3mΩ shunt RS2)
     * Phase C has no external shunt — reconstructed as Ic = -(Ia + Ib).
     *
     * IBus: computed per commutation step — the PWM-active phase carries
     * the full DC bus current:
     *   Steps 0,5: A=PWM → IBus = |Ia|
     *   Steps 3,4: B=PWM → IBus = |Ib|
     *   Steps 1,2: C=PWM → IBus = |-(Ia+Ib)| = |Ia+Ib| */
    (void)ADCBUF0;                      /* AN0: read to clear, not used */
    gData.iaRaw = (int16_t)ADCBUF1;    /* AN1: Phase A (IS1) */
    gData.ibRaw = (int16_t)ADCBUF4;    /* AN4: Phase B (IS2) */

    /* Compute IBus from the active PWM phase per commutation step. */
    {
        int16_t ibus;
        switch (gData.currentStep)
        {
            case 0: case 5:  /* Phase A is PWM → Ia = IBus */
                ibus = gData.iaRaw;
                break;
            case 3: case 4:  /* Phase B is PWM → Ib = IBus */
                ibus = gData.ibRaw;
                break;
            default:         /* Steps 1,2: Phase C is PWM → Ic = -(Ia+Ib) */
                ibus = -(gData.iaRaw + gData.ibRaw);
                break;
        }
        gData.ibusRaw = ibus < 0 ? -ibus : ibus;
    }

    /* BEMF zero-crossing detection */
    if (gData.state == ESC_OL_RAMP || gData.state == ESC_CLOSED_LOOP)
    {
#if FEATURE_IC_ZC
        /* Hybrid: polling during OL_RAMP (robust at low BEMF),
         * IC + PWM-center poll during CL. */
        if (gData.state == ESC_OL_RAMP)
        {
            if (BEMF_ZC_Poll((volatile GARUDA_DATA_T *)&gData))
                BEMF_ZC_ScheduleCommutation((volatile GARUDA_DATA_T *)&gData);
        }
        else if (gData.state == ESC_CLOSED_LOOP)
        {
            /* ADC ISR backup poll at 20 kHz (PWM center).
             * Primary ZC detection is the fast poll timer (100 kHz).
             * This catches rare cases where the fast poll misses ZC
             * (e.g., ISR preemption). Uses the same bounce-tolerant
             * filter as OL_RAMP polling. */
            if (!gData.bemf.zeroCrossDetected &&
                gData.icZc.phase == IC_ZC_ARMED &&
                BEMF_ZC_Poll((volatile GARUDA_DATA_T *)&gData))
            {
                gData.icZc.diagLcoutAccepted++;
                gData.icZc.phase = IC_ZC_DONE;
                BEMF_ZC_ScheduleCommutation((volatile GARUDA_DATA_T *)&gData);
            }
        }
#else
        /* Polling mode: read digital comparators at 20 kHz */
        if (BEMF_ZC_Poll((volatile GARUDA_DATA_T *)&gData))
        {
            /* ZC detected — schedule next commutation */
            BEMF_ZC_ScheduleCommutation((volatile GARUDA_DATA_T *)&gData);
        }
#endif

        /* Commutation deadline check at 20 kHz (50 us resolution).
         * With FEATURE_IC_ZC: com timer (640 ns) fires first in CL, but
         * this serves as a safety fallback if com timer hasn't fired yet
         * (e.g., delay already elapsed when scheduled). Uses absolute
         * deadline timestamp so it correctly handles elapsed time. */
        if (gData.timing.deadlineActive)
        {
            int16_t dt = (int16_t)(gData.timer1Tick - gData.timing.commDeadline);
            if (dt >= 0)
            {
#if FEATURE_IC_ZC
                /* Gate out SCCP1/SCCP4 during transition */
                gData.icZc.phase = IC_ZC_DONE;
                HAL_ComTimer_Cancel();
#endif
                gData.timing.deadlineActive = false;
                COMMUTATION_AdvanceStep((volatile GARUDA_DATA_T *)&gData);
                BEMF_ZC_OnCommutation((volatile GARUDA_DATA_T *)&gData);
            }
        }
    }

    /* Vbus fault checking (ISR-safe — no SPI).
     * Skip during ARMED/ALIGN — ADC readings settle over first few ms. */
    if (gData.state >= ESC_OL_RAMP)
    {
        if (gData.vbusRaw > VBUS_OV_THRESHOLD)
        {
            EnterFaultISR(FAULT_OVERVOLTAGE);
        }
        else if (gData.vbusRaw < VBUS_UV_THRESHOLD && gData.vbusRaw > 0)
        {
            if (++uvDebounce >= VBUS_UV_DEBOUNCE)
                EnterFaultISR(FAULT_UNDERVOLTAGE);
        }
        else
        {
            uvDebounce = 0;
        }
    }

    /* Clear ADC interrupt flag at END of ISR */
    IFS5bits.ADCIF = 0;
}

/* ── SCCP1 Fast Poll Timer ISR (FEATURE_IC_ZC) ───────────────────── */
#if FEATURE_IC_ZC

/**
 * @brief SCCP1 timer period ISR — fires at ZC_POLL_FREQ_HZ (100 kHz).
 *
 * Polls the ATA6847 BEMF comparator for the floating phase every 10µs.
 * Adaptive deglitch filter confirms ZC after 2-8 consecutive matching reads.
 * SCCP4-based blanking provides 640ns resolution (vs 50µs with Timer1).
 *
 * Only active during closed-loop. OL_RAMP uses ADC ISR polling.
 *
 * CPU budget: ~15-20 instructions per call when blanking/idle (150-200ns),
 * plus ~50-100 instructions once per step for RecordZcTiming + scheduling.
 * At 100kHz: ~2% CPU idle, <3% peak. Replaces 3 IC ISRs that were
 * consuming up to 30% CPU from 14000+ bounce captures per run.
 */
void __attribute__((interrupt, no_auto_psv)) _CCT1Interrupt(void)
{
    _CCT1IF = 0;

    if (gData.state != ESC_CLOSED_LOOP)
        return;

    if (BEMF_ZC_FastPoll((volatile GARUDA_DATA_T *)&gData))
    {
        /* ZC confirmed — schedule commutation via SCCP4 output compare */
        BEMF_ZC_ScheduleCommutation((volatile GARUDA_DATA_T *)&gData);
    }
}

/**
 * @brief SCCP4 output compare ISR — fires at exact ZC+delay moment.
 *
 * SCCP4 runs as free-running timer (640 ns/tick). When ZC is detected,
 * ScheduleCommutation sets CCP4RA to the target tick. When CCP4TMRL
 * matches CCP4RA, this ISR fires at the precise commutation point.
 *
 * One-shot: disable CCP4IE to prevent re-fire on next period wrap.
 *
 * Resolution: 640 ns vs 50 us = 78x improvement.
 */
void __attribute__((interrupt, no_auto_psv)) _CCP4Interrupt(void)
{
    /* One-shot: disable compare interrupt (timer keeps running) */
    _CCP4IE = 0;

    /* Fire commutation — but only if no other ISR has already handled
     * this step (deadlineActive is the one-shot gate). */
    if (gData.timing.deadlineActive)
    {
        /* Gate out SCCP1 fast poll during transition */
        gData.icZc.phase = IC_ZC_DONE;
        gData.timing.deadlineActive = false;
        COMMUTATION_AdvanceStep((volatile GARUDA_DATA_T *)&gData);
        BEMF_ZC_OnCommutation((volatile GARUDA_DATA_T *)&gData);
    }

    _CCP4IF = 0;
}

#endif /* FEATURE_IC_ZC */
