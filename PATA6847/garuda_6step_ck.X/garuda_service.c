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

#if FEATURE_V4_SECTOR_PI
#include "motor/sector_pi.h"
#include "hal/hal_com_timer.h"
#include "hal/hal_capture.h"
#else
#include "motor/startup.h"
#include "motor/bemf_zc.h"
#if FEATURE_IC_ZC
#include "hal/hal_ic.h"
#include "hal/hal_com_timer.h"
#endif
#if FEATURE_IC_DMA_SHADOW
#include "hal/hal_ic_dma.h"
#endif
#if FEATURE_PTG_ZC
#include "hal/hal_ptg.h"
#endif
#endif /* !FEATURE_V4_SECTOR_PI */

#if !FEATURE_V4_SECTOR_PI
/* Global ESC runtime data (V3 only) */
volatile GARUDA_DATA_T gData;
#endif

#if FEATURE_V4_SECTOR_PI
/* ====================================================================
 * V4 SECTOR PI ARCHITECTURE
 * ==================================================================== */

/* Minimal state for V4 (sector_pi.c owns motor state) */
volatile ESC_STATE_T gV4State = ESC_IDLE;
volatile bool gStateChanged = false;
volatile ESC_STATE_T gPrevState = ESC_IDLE;
volatile uint16_t gV4PotRaw = 0;
volatile uint16_t gV4VbusRaw = 0;
volatile int16_t  gV4IaRaw = 0;
volatile int16_t  gV4IbRaw = 0;
static volatile uint8_t  gV4TickDiv = 0;
volatile uint32_t gV4SystemTick = 0;

/* ATA6847 fault check interval */
static volatile uint8_t gV4AtaCheckDiv = 0;
static volatile uint32_t gV4StartTick = 0;  /* systemTick when motor started */

void GarudaService_Init(void)
{
    gV4State = ESC_IDLE;
    SectorPI_Init();
    HAL_Timer1_Start();
}

void GarudaService_StartMotor(void)
{
    if (gV4State != ESC_IDLE) return;

    HAL_UART_WriteString("V4:clr ");
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
        gV4State = ESC_FAULT;
        return;
    }

    HAL_UART_WriteString("pwm ");
    HAL_PWM_EnableOutputs();
    HAL_PWM_ChargeBootstrap();

    /* Settle delay after GDU power-up + bootstrap charge.
     * ATA6847 needs time for VDS monitors to clear after
     * bootstrap charging. V3 uses 200ms ARM state for this. */
    { volatile uint32_t d; for (d = 0; d < 200000UL; d++); }  /* ~20ms */

    /* Clear any transient faults from bootstrap charging */
    HAL_ATA6847_ClearFaults();
    { volatile uint32_t d; for (d = 0; d < 50000UL; d++); }

    HAL_UART_WriteString("adc ");
    HAL_OPA_Enable();
    HAL_ADC_InterruptEnable();

    LED_RUN = 1;
    LED_FAULT = 0;

    HAL_UART_WriteString("V4:START\r\n");

    /* SectorPI_Start is NON-BLOCKING. It sets up alignment state.
     * Timer1 ISR drives ALIGN + OL_RAMP via SectorPI_OlTick().
     * When ramp completes, sector_pi.c starts SCCP3 + CCP2 for CL. */
    SectorPI_Start(gV4VbusRaw);

    gV4State = ESC_OL_RAMP;  /* V3-style state during ramp */
    gStateChanged = true;
    gPrevState = ESC_IDLE;
    gV4AtaCheckDiv = 0;
    gV4StartTick = gV4SystemTick;
}

void GarudaService_StopMotor(void)
{
    SectorPI_Stop();
    HAL_PWM_DisableOutputs();
    HAL_ADC_InterruptDisable();
    HAL_OPA_Disable();
    HAL_ATA6847_EnterGduStandby();
    gV4State = ESC_IDLE;
    gStateChanged = true;
    LED_RUN = 0;
}

/* ── V4 Timer1 ISR ────────────────────────────────────────────────── */
void __attribute__((interrupt, auto_psv)) _T1Interrupt(void)
{
    IFS0bits.T1IF = 0;

    /* OL ramp: Timer1 drives commutation at 20kHz during ALIGN+OL_RAMP.
     * This is the V3-proven approach. Every 50µs tick, the ramp countdown
     * decrements. When it hits 0, the next commutation step fires. */
    if (SectorPI_IsRunning())
        SectorPI_OlTick();

    /* 1ms tick (divide 20kHz by 20) */
    if (++gV4TickDiv >= 20)
    {
        gV4TickDiv = 0;
        gV4SystemTick++;

        if (SectorPI_IsRunning())
        {
            SectorPI_TimeTick();

            /* Throttle */
            uint16_t amp = gV4PotRaw >> 1;
            if (amp < 1500) amp = 0;
            SectorPI_CommandSet(amp);

            /* Track phase transitions for telemetry state */
            if (gV4State == ESC_OL_RAMP && SectorPI_GetPhase() == 3)
            {
                gV4State = ESC_CLOSED_LOOP;
                gV4StartTick = gV4SystemTick;
                gStateChanged = true;
            }
        }
    }

    /* ATA6847 nIRQ check — DISABLED for now.
     * At high speed/duty the ATA6847 VDS monitor triggers spurious
     * nIRQ faults. Need to read the actual fault register and
     * distinguish real faults (short circuit) from transient VDS
     * trips before re-enabling. */
}

/* Forward declarations for ZC detection */
static inline uint8_t ReadBEMFComp(void);
extern volatile uint8_t v4_floatingPhase;
extern volatile uint16_t v4_blankingEndHR;
extern volatile uint16_t v4_timerPeriod;  /* from sector_pi.c */

#if FEATURE_V4_MIDPOINT_ZC >= 1
/* (midpoint modes also need these) */
#endif

/* Mode 2 hybrid: midpoint confirms ZC state, CCP provides timestamp */
#if FEATURE_V4_MIDPOINT_ZC == 2
static volatile bool v4_zcConfirmed = false;  /* Set by ADC ISR, consumed by CCP ISR */
#endif

/* ── V4 ADC ISR ───────────────────────────────────────────────────── */
void __attribute__((interrupt, auto_psv)) _ADCInterrupt(void)
{
    gV4PotRaw  = ADCBUF_POT;
    gV4VbusRaw = ADCBUF_VBUS;
    gV4IaRaw   = (int16_t)ADCBUF1;
    gV4IbRaw   = (int16_t)ADCBUF4;
    (void)ADCBUF0;  /* Must read to clear data-ready */

#if FEATURE_V4_MIDPOINT_ZC == 1
    /* Mode 1: Pure midpoint — ADC ISR owns ZC detection entirely */
    if (SectorPI_IsRunning() && SectorPI_GetPhase() == 3 && !v4_captureValid)
    {
        uint16_t nowHR = CCP4TMRL;
        if ((int16_t)(nowHR - v4_blankingEndHR) >= 0)
        {
            uint8_t comp = ReadBEMFComp();
            uint8_t expected = HAL_Capture_IsRisingZc() ? 1 : 0;
            if (comp == expected)
            {
                v4_lastCaptureHR = nowHR;
                v4_captureValid = true;
            }
        }
    }
#elif FEATURE_V4_MIDPOINT_ZC == 2
    /* Mode 2: Hybrid — midpoint confirms ZC state, CCP provides timestamp.
     * Read comparator at PWM center. If post-ZC state detected, set
     * confirmation flag. Next CCP edge that fires will be accepted as
     * the real ZC with hardware-accurate timestamp.
     * Like AM32 maskPhaseInterrupts but inverted: we GATE the CCP
     * acceptance on midpoint confirmation instead of masking. */
    if (SectorPI_IsRunning() && SectorPI_GetPhase() == 3
        && !v4_captureValid && !v4_zcConfirmed)
    {
        uint16_t nowHR = CCP4TMRL;
        if ((int16_t)(nowHR - v4_blankingEndHR) >= 0)
        {
            uint8_t comp = ReadBEMFComp();
            uint8_t expected = HAL_Capture_IsRisingZc() ? 1 : 0;
            if (comp == expected)
            {
                v4_zcConfirmed = true;  /* CCP ISR can now accept next edge */
            }
        }
    }
#endif

    IFS5bits.ADCIF = 0;
}

/* ── V4 Commutation ISR (SCCP3 sector timer period match) ─────── */
void __attribute__((interrupt, no_auto_psv)) _CCT3Interrupt(void)
{
#if FEATURE_V4_MIDPOINT_ZC == 2
    v4_zcConfirmed = false;  /* Reset for new sector */
#endif
    /* One-shot guard: disable interrupt + push PRL to prevent
     * stray re-trigger while SectorPI_Commutate runs.
     * Commutate will call ScheduleAbsolute to arm the next one. */
    _CCT3IE = 0;
    _CCT3IF = 0;
    CCP3PRL = 0xFFFF;
    SectorPI_Commutate();
}

/* ── V4 CCP ISRs — drain FIFO, store last edge unconditionally ── */
/* No blanking here — blanking is in the commutation ISR.
 * These ISRs prevent FIFO overflow (4-deep FIFO loses new
 * captures when full). By draining on every edge, the real
 * ZC capture survives instead of being discarded. */

/* ── Cluster detection state (per-polarity) ──────────────────── */
/* A ZC produces a burst of 2+ comparator edges within ~10 HR ticks.
 * Single PWM noise edges are spaced ~25µs (39 HR ticks) apart.
 * Detect cluster: current edge within CLUSTER_GAP of previous → ZC.
 * This is V3's DMA cluster detection running in the ISR. */
#define CLUSTER_GAP_HR  15u   /* ~10µs — wide enough for ZC burst,
                               * narrow enough to reject PWM noise */
uint16_t prevEdgeCCP2 = 0;
uint16_t prevEdgeCCP5 = 0;

/* ── ZC detection with deglitch ──────────────────────────────── */
/* CCP ISR fires on every comparator edge. After blanking, read
 * the comparator GPIO 3 times with ~500ns delays. All 3 must
 * match the expected post-ZC state. This rejects PWM ringing
 * (brief pulses that bounce back) and detects real ZC (sustained
 * state change). Once detected, lock v4_captureValid. */

/* Floating phase GPIO readers */
volatile uint8_t v4_floatingPhase = 0;

static inline uint8_t ReadBEMFComp(void)
{
    switch (v4_floatingPhase) {
        case 0: return _RC6;
        case 1: return _RC7;
        case 2: return _RD10;
        default: return 0;
    }
}

/* Blanking state */
volatile uint16_t v4_blankingEndHR = 0;

void __attribute__((interrupt, no_auto_psv)) _CCP2Interrupt(void)
{
    uint16_t ts = 0;
    bool got = false;
    while (CCP2STATLbits.ICBNE) { ts = CCP2BUFL; got = true; }

#if FEATURE_V4_MIDPOINT_ZC == 0
    if (got && HAL_Capture_IsRisingZc() && !v4_captureValid)
    {
        uint16_t hr = (uint16_t)(ts + HAL_Capture_GetCcp2Offset());
        if ((int16_t)(hr - v4_blankingEndHR) >= 0)
        {
            /* Speed-adaptive deglitch + AM32 mask.
             * Low speed (<60k): 3 reads — reject noise, BEMF weak.
             * High speed (>60k): 1 read — BEMF strong, minimize
             * ISR time, avoid spanning a PWM edge. */
            bool accept;
            if (v4_timerPeriod < 260) {
                /* High speed: single read */
                accept = (ReadBEMFComp() == 1);
            } else {
                /* Low speed: 3-read deglitch */
                uint8_t r1 = ReadBEMFComp();
                Nop(); Nop(); Nop(); Nop(); Nop();
                Nop(); Nop(); Nop(); Nop(); Nop();
                uint8_t r2 = ReadBEMFComp();
                Nop(); Nop(); Nop(); Nop(); Nop();
                Nop(); Nop(); Nop(); Nop(); Nop();
                uint8_t r3 = ReadBEMFComp();
                accept = (r1 == 1 && r2 == 1 && r3 == 1);
            }
            if (accept)
            {
                v4_lastCaptureHR = hr;
                v4_captureValid = true;
                _CCP2IE = 0;
                _CCP5IE = 0;
            }
        }
    }
#elif FEATURE_V4_MIDPOINT_ZC == 2
    if (got && HAL_Capture_IsRisingZc() && !v4_captureValid && v4_zcConfirmed)
    {
        uint16_t hr = (uint16_t)(ts + HAL_Capture_GetCcp2Offset());
        v4_lastCaptureHR = hr;
        v4_captureValid = true;
        v4_zcConfirmed = false;
        _CCP2IE = 0;
        _CCP5IE = 0;
    }
#endif
    _CCP2IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _CCP5Interrupt(void)
{
    uint16_t ts = 0;
    bool got = false;
    while (CCP5STATLbits.ICBNE) { ts = CCP5BUFL; got = true; }

#if FEATURE_V4_MIDPOINT_ZC == 0
    if (got && !HAL_Capture_IsRisingZc() && !v4_captureValid)
    {
        uint16_t hr = (uint16_t)(ts + HAL_Capture_GetCcp5Offset());
        if ((int16_t)(hr - v4_blankingEndHR) >= 0)
        {
            bool accept;
            if (v4_timerPeriod < 260) {
                accept = (ReadBEMFComp() == 0);
            } else {
                uint8_t r1 = ReadBEMFComp();
                Nop(); Nop(); Nop(); Nop(); Nop();
                Nop(); Nop(); Nop(); Nop(); Nop();
                uint8_t r2 = ReadBEMFComp();
                Nop(); Nop(); Nop(); Nop(); Nop();
                Nop(); Nop(); Nop(); Nop(); Nop();
                uint8_t r3 = ReadBEMFComp();
                accept = (r1 == 0 && r2 == 0 && r3 == 0);
            }
            if (accept)
            {
                v4_lastCaptureHR = hr;
                v4_captureValid = true;
                _CCP2IE = 0;
                _CCP5IE = 0;
            }
        }
    }
#elif FEATURE_V4_MIDPOINT_ZC == 2
    if (got && !HAL_Capture_IsRisingZc() && !v4_captureValid && v4_zcConfirmed)
    {
        uint16_t hr = (uint16_t)(ts + HAL_Capture_GetCcp5Offset());
        v4_lastCaptureHR = hr;
        v4_captureValid = true;
        v4_zcConfirmed = false;
        _CCP2IE = 0;
        _CCP5IE = 0;
    }
#endif
    _CCP5IF = 0;
}

/* ── V4 service tick (called from main loop) ─────────────────── */
void GarudaService_Tasks(void)
{
    /* Check for stall → restart */
    if (gV4State == ESC_CLOSED_LOOP && !SectorPI_IsRunning())
    {
        HAL_UART_WriteString("V4:STALL\r\n");
        GarudaService_StopMotor();
    }
}

void GarudaService_ClearFault(void)
{
    if (gV4State == ESC_FAULT)
    {
        gV4State = ESC_IDLE;
        LED_FAULT = 0;
    }
}

void GarudaService_MainLoop(void)
{
    if (gV4State != gPrevState)
    {
        gStateChanged = true;
        gPrevState = gV4State;
    }
    GarudaService_Tasks();
}

#else /* !FEATURE_V4_SECTOR_PI — V3 code below */

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
#if FEATURE_PTG_ZC
    HAL_PTG_Stop();
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
#if FEATURE_PTG_ZC
    HAL_PTG_Stop();
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

    /* Clear ATA6847 faults — may need multiple attempts after desync.
     * VDS/SC faults latch until cleared AND the fault condition is gone. */
    HAL_ATA6847_ClearFaults();
    { volatile uint32_t d; for (d = 0; d < 50000UL; d++); }  /* ~5ms */
    HAL_ATA6847_ClearFaults();

    HAL_UART_WriteString("gdu ");

    /* Power up GDU — retry once if first attempt fails */
    if (!HAL_ATA6847_EnterGduNormal())
    {
        { volatile uint32_t d; for (d = 0; d < 100000UL; d++); }  /* ~10ms */
        HAL_ATA6847_ClearFaults();
        { volatile uint32_t d; for (d = 0; d < 50000UL; d++); }
    }
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
#if FEATURE_PTG_ZC
    HAL_PTG_Stop();
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
        if (gData.state == ESC_IDLE)
            GarudaService_StartMotor();
    }

    /* FAULT state: clear fault on SW1 press (same as normal start).
     * Don't auto-restart — ATA6847 faults need the motor to stop
     * completely before GDU can re-enter Normal mode. */
    if (gData.state == ESC_FAULT && BTN1_GetValue() == 0)
    {
        gData.state = ESC_IDLE;
        gData.faultCode = FAULT_NONE;
        gData.desyncRestartAttempts = 0;
        LED_FAULT = 0;
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
#if FEATURE_PTG_ZC
                HAL_PTG_Stop();
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
#if FEATURE_SPEED_PD
                /* Seed PD override from ramp exit duty */
                gData.speedPd.override = (int32_t)gData.duty;
                gData.speedPd.lastError = 0;
                gData.speedPd.targetErpm = 0;
#endif
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
#if FEATURE_PTG_ZC
                HAL_PTG_Start();
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
#if FEATURE_SPEED_PD
                    /* Speed PD controller — AM32-style accumulated override.
                     * pot → target_eRPM → PD → duty override.
                     * Runs at 1ms rate (every 20 Timer1 ticks). */
                    {
                        static uint16_t pdDiv = 0;
                        if (++pdDiv >= 20)
                        {
                            pdDiv = 0;

                            /* 1. Target eRPM from pot */
                            uint16_t potVal = gData.gspThrottleActive
                                ? gData.gspThrottleValue : gData.potRaw;
                            uint32_t tgtErpm;
                            if (potVal < POT_DEADZONE)
                                tgtErpm = MIN_TARGET_ERPM;
                            else if (potVal > POT_MAX)
                                tgtErpm = MAX_TARGET_ERPM;
                            else
                                tgtErpm = MIN_TARGET_ERPM +
                                    (uint32_t)(MAX_TARGET_ERPM - MIN_TARGET_ERPM)
                                    * (potVal - POT_DEADZONE)
                                    / (POT_MAX - POT_DEADZONE);
                            gData.speedPd.targetErpm = tgtErpm;

                            /* 2. Measured eRPM from protected estimator */
                            bool fbValid = (gData.zcCtrl.refIntervalHR > 0 &&
                                            gData.timing.zcSynced);
                            uint32_t measErpm = 0;
                            if (fbValid)
                                measErpm = 15625000UL / gData.zcCtrl.refIntervalHR;

                            /* 3. PD computation (only with valid feedback) */
                            if (fbValid)
                            {
                                int32_t error = (int32_t)tgtErpm - (int32_t)measErpm;
                                int32_t dError = error - gData.speedPd.lastError;
                                gData.speedPd.lastError = error;

                                gData.speedPd.override +=
                                    (error * (int32_t)SPEED_PD_KP +
                                     dError * (int32_t)SPEED_PD_KD)
                                    / (int32_t)SPEED_PD_SCALE;

                                /* Clamp */
                                if (gData.speedPd.override > (int32_t)MAX_DUTY)
                                    gData.speedPd.override = (int32_t)MAX_DUTY;
                                if (gData.speedPd.override < (int32_t)CL_IDLE_DUTY)
                                    gData.speedPd.override = (int32_t)CL_IDLE_DUTY;
                            }
                            /* If feedback invalid: freeze override */
                        }
                        target = (uint32_t)gData.speedPd.override;
                    }
#else
                    /* Open-loop: pot → duty */
                    target = MapThrottleToDuty(
                        gData.gspThrottleActive ? gData.gspThrottleValue : gData.potRaw);
#endif

                    /* Mode-specific duty limiting (applies to both PD and open-loop) */
                    switch (gData.zcCtrl.mode)
                    {
                        case ZC_MODE_ACQUIRE:
                            if (target > rampExitDuty)
                                target = rampExitDuty;
                            break;
                        case ZC_MODE_RECOVER:
                            if (target > slewedDuty)
                                target = slewedDuty;
                            break;
                        case ZC_MODE_TRACK:
                            break;
                    }
                }
                else
                {
                    /* NOT synced — reduce duty to idle via normal slew. */
                    target = CL_IDLE_DUTY;
#if FEATURE_SPEED_PD
                    gData.speedPd.override = (int32_t)CL_IDLE_DUTY;
                    gData.speedPd.lastError = 0;
#endif
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
/* Phase 9 duty governance DISABLED for SCCP3 bench validation.
 * Re-enable for prop testing where it prevents desync-on-fast-pot.
 * On bench without load the linear ramp releases at exactly 60k eRPM
 * (DUTY_RAMP_ERPM) and the duty step from cap to 100% causes a
 * violent jerk → desync. Prop load smooths this naturally. */
#if 0 && FEATURE_IC_ZC
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
                /* Force a commutation step at current duty.
                 * Duty cut was tested but causes ATA6847 VDS fault from regen
                 * voltage spike (motor acts as generator when duty drops).
                 * The remaining timeouts are mostly PWM-step aliasing artifacts
                 * (integer ratio at specific eRPM) — single forced steps at
                 * full duty are harmless and the motor recovers immediately. */

                /* Gate out higher-priority ISRs (SCCP1/SCCP4) during the
                 * transition to prevent race conditions where they see
                 * partially-updated state (old cmpExpected + new step). */
#if FEATURE_IC_ZC
                gData.bemf.zeroCrossDetected = true;  /* Block SCCP1 polling */
                gData.icZc.phase = IC_ZC_DONE;        /* Block FastPoll */
#if FEATURE_SECTOR_PI
                if (gData.zcSync.mode != 2)  /* Don't kill PI's timer */
#endif
                {
                    HAL_ComTimer_Cancel();
                    gData.timing.deadlineActive = false;
                }
                gData.zcPred.lastCommHR = HAL_ComTimer_ReadTimer();
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
                /* Retry: go back to IDLE, auto-restart.
                 * Always restart after desync recovery — if pot is
                 * non-zero the user wants the motor running. The old
                 * runCommandActive check caused hangs when SW1 wasn't
                 * held during recovery. */
                HAL_PWM_DisableOutputs();
                gData.state = ESC_IDLE;
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

        /* Timer1 commutation backup — defers to SCCP3 one-shot scheduler.
         * If `_CCT3IE` is set, SCCP3 is armed and will fire its own
         * (higher-priority IPL=6) ISR on the precise HR target. The
         * Timer1 deadline lives in 50µs ticks while SCCP3 fires inside
         * that tick at 640ns precision, so an unconditional dt>=0
         * check would beat SCCP3 to the cancel by up to 50µs and starve
         * the period-match ISR. */
        if (!gData.zcPred.predictiveMode &&
            gData.timing.deadlineActive &&
            !_CCT3IE)
        {
            int16_t dt = (int16_t)(gData.timer1Tick - gData.timing.commDeadline);
            if (dt >= 0)
            {
#if FEATURE_IC_ZC
                gData.icZc.phase = IC_ZC_DONE;
#if FEATURE_SECTOR_PI
                if (gData.zcSync.mode != 2)
#endif
                    HAL_ComTimer_Cancel();
#endif
#if FEATURE_SECTOR_PI
                if (gData.zcSync.mode != 2)
#endif
                gData.timing.deadlineActive = false;
                gData.zcPred.lastCommHR = HAL_ComTimer_ReadTimer();
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
 * @brief SCCP3 one-shot timer period ISR — commutation deadline fire.
 *
 * SCCP3 runs in Time Base mode as a one-shot scheduler. After ZC,
 * HAL_ComTimer_ScheduleAbsolute() sets CCP3PRL = (target − now) and
 * starts SCCP3 from zero — when CCP3TMRL reaches PRL, this ISR fires
 * the commutation. CCT3IE is cleared in the ISR to make it one-shot.
 *
 * Resolution: 640 ns/tick (same prescaler as SCCP4 HR timer), so
 * scheduling accuracy is 78× better than the Timer1 backup path.
 *
 * NOTE: previously this lived on _CCP4Interrupt with SCCP4 in OC mode,
 * but the dsPIC33CK SCCP OC compare match never raised CCP4IF on this
 * device (verified empirically with OCAEN=0 and OCAEN=1). The Time
 * Base CCTxIF path is the proven working pattern (SCCP1 fast poll
 * uses it at 100kHz reliably).
 */
void __attribute__((interrupt, no_auto_psv)) _CCT3Interrupt(void)
{
    /* One-shot: disable interrupt + push PRL back to 0xFFFF so the
     * always-on free-running timer can't generate a stray match
     * before the next ScheduleAbsolute updates PRL. */
    _CCT3IE = 0;
    _CCT3IF = 0;
    CCP3PRL = 0xFFFF;

    /* DIAGNOSTIC: count EVERY ISR entry, before any gating. */
    gData.zcPred.diagPredIsrEntries++;

    /* Fire commutation — but only if no other ISR has already handled
     * this step (deadlineActive is the one-shot gate). */
    if (gData.timing.deadlineActive)
    {
        if (gData.zcPred.predictiveMode)
            gData.zcPred.diagPredIsrFired++;

        /* Capture actual commutation moment from the free-running
         * SCCP4 HR timer (replaces the old `CCP4RA` read which was
         * the *scheduled* target — actual fire time is `now` plus
         * a small ISR latency, which is what the predictor needs). */
        uint16_t thisCommHR = HAL_ComTimer_ReadTimer();
        gData.zcPred.lastCommHR = thisCommHR;

        /* Step 3: Latch handoff decision and PROGRAM SCCP4 IMMEDIATELY,
         * before the expensive AdvanceStep/OnCommutation work.
         * OnCommutation calls HAL_ComTimer_Cancel() which would
         * destroy the freshly programmed target — so we set
         * deadlineActive=true and predictiveMode=true FIRST to
         * guard against that. */
        /* Pure period-based handoff: predictor takes over reactive's
         * scheduling pattern (comm + predStepHR), no advance change.
         * Codex: separate observer (period) from control (advance).
         * predZcOffsetHR is observer/supervision only, not used here. */
        bool doHandoff = gData.zcPred.handoffPending &&
            gData.zcPred.predStepHR > 0;

        if (doHandoff)
        {
            /* Seed first predictive target from the LAST REACTIVE TARGET,
             * not from thisCommHR + predStepHR. The reactive path computed
             * targetHR = lastZcTickHR + delayHR using AdaptiveTimingAdvance,
             * so this preserves the exact phase/advance that was scheduled.
             *
             * Using thisCommHR + predStepHR guarantees a phase jump because:
             * (1) predStepHR uses a different time-base than reactive's delay
             * (2) advanceCmdHR may differ from reactive's TAL-based advance
             *
             * lastReactiveTargetHR is set every reactive scheduling call,
             * so it always reflects the most recent reactive decision. */
            uint16_t nowHR = HAL_ComTimer_ReadTimer();
            uint16_t nominalTargetHR = gData.zcPred.lastReactiveTargetHR;
            /* Fallback if lastReactiveTargetHR was never set (shouldn't
             * happen — reactive runs before handoff can be requested) */
            if (nominalTargetHR == 0)
                nominalTargetHR = thisCommHR + gData.zcPred.predStepHR;
            /* Handoff guard: target must be >= now + 24 ticks (~15µs).
             * Codex: 8 ticks was too close — hardware can miss the
             * compare match if the timer ticks past before the
             * register updates, causing a 42ms wrap wait. */
            uint16_t minTargetHR = nowHR + 24;
            uint16_t nextTargetHR =
                ((int16_t)(nominalTargetHR - minTargetHR) > 0)
                ? nominalTargetHR : minTargetHR;
            int16_t margin = (int16_t)(nextTargetHR - nowHR);
            if (margin > 2)
            {
                /* Program SCCP4 NOW — before OnCommutation can cancel it */
                HAL_ComTimer_ScheduleAbsolute(nextTargetHR);
                gData.zcPred.predictiveMode = true;
                gData.zcPred.handoffPending = false;
#if FEATURE_6STEP_DPLL
                gData.zcPred.graceCount = 24;  /* suppress phaseErr exits
                                                * while bias IIR absorbs
                                                * the entry discontinuity.
                                                * 24 steps ≈ 4 revolutions,
                                                * absorbs 95% of error. */
                /* Re-seed phaseBias from the shadow predictor's current
                 * estimate. Without this, re-entries after a fallback
                 * use a stale bias from a different speed, causing the
                 * bias to swing wildly during the grace period. */
                if (gData.zcPred.predZcOffsetHR > 0 &&
                    gData.zcPred.predStepHR > 0)
                {
                    gData.zcPred.phaseBiasHR = (int16_t)(
                        gData.zcPred.predZcOffsetHR
                        - (gData.zcPred.predStepHR >> 1)
                        - gData.zcPred.advanceCmdHR);
                }
#endif
                gData.zcPred.lastPredCommHR = thisCommHR;
                gData.zcPred.pendingPredCommHR = nextTargetHR;
                gData.zcPred.pendingPredValid = true;
                gData.zcPred.diagPredCommOwned++;
                gData.zcPred.diagPredEnter++;
                /* Clear missCount — OnCommutation will increment it
                 * but we just entered, so reset the counter to give
                 * the predictor a clean window to operate. */
                gData.zcPred.missCount = 0;
                /* deadlineActive stays true — guards OnCommutation's Cancel */
                gData.timing.deadlineActive = true;
            }
            else
            {
                gData.zcPred.handoffPending = false;
                gData.zcPred.diagPredEntryLate++;
            }
        }

        /* Steady-state predictive: schedule next comm BEFORE OnCommutation */
        if (!doHandoff && gData.zcPred.predictiveMode &&
            gData.zcPred.predStepHR > 0)
        {
            uint16_t nextTargetHR = thisCommHR + gData.zcPred.predStepHR;
            HAL_ComTimer_ScheduleAbsolute(nextTargetHR);
            gData.zcPred.lastPredCommHR = thisCommHR;
            gData.zcPred.pendingPredCommHR = nextTargetHR;
            gData.zcPred.pendingPredValid = true;
            gData.zcPred.diagPredCommOwned++;
            gData.timing.deadlineActive = true;
        }

#if FEATURE_SECTOR_PI && FEATURE_IC_DMA_SHADOW
        /* ── Sector PI ownership: query DMA + schedule from T_hat ──
         * MUST run BEFORE AdvanceStep/OnCommutation because those
         * update the DMA commHead markers for the NEW step. The DMA
         * query needs markers pointing to the PREVIOUS step's edges.
         *
         * When mode==2 (OWNED): PI drives commutation scheduling.
         * Reactive scheduling is skipped (guard in ScheduleCommutation).
         * Poll detection still runs for supervision and fallback. */
        if (gData.zcSync.mode == 2)
        {
            /* DEBUG: prove this block executes — count in clusterWidthHR */
            gData.zcSync.clusterWidthHR++;

            bool risingZc = gData.zcSync.prevStepRisingZc;
            uint16_t windowOpenHR = gData.zcSync.lastCommHR
                                  + (gData.zcSync.T_hatHR >> 2);
            /* Use blanking end if later */
            if ((int16_t)(gData.icZc.blankingEndHR - windowOpenHR) > 0)
                windowOpenHR = gData.icZc.blankingEndHR;
            uint16_t windowCloseHR = thisCommHR;  /* up to this commutation */
            uint16_t dmaZcHR = 0;

            bool zcFound = HAL_ZcDma_DetectZc(
                windowOpenHR, windowCloseHR, risingZc, &dmaZcHR);

            /* DEBUG: store window for telemetry diagnosis */
            gData.zcSync.capValueHR = (uint16_t)(windowCloseHR - windowOpenHR);
            gData.zcSync.setValueHR = zcFound ? (uint16_t)(dmaZcHR - windowOpenHR) : 0;

            if (zcFound)
            {
                int16_t dmaVsClose = (int16_t)(dmaZcHR - windowCloseHR);
                /* Same gate as shadow: DMA must be within [-40, 0]
                 * of the close bound. The old [-200, 0] was too loose
                 * and admitted wrong PWM clusters. */
                if (dmaVsClose >= -40 && dmaVsClose <= 0)
                {
                    uint16_t capValueHR = dmaZcHR - gData.zcSync.lastCommHR;
                    uint8_t searchTal = gData.zcPred.lastReactiveTAL;
                    uint16_t modelAdvHR = (gData.zcSync.T_hatHR >> 3) * searchTal;
                    uint16_t setValueHR = (gData.zcSync.T_hatHR >> 1)
                                        + modelAdvHR
                                        - gData.zcSync.detDelayHR;
                    int16_t errHR = (int16_t)(capValueHR - setValueHR);

                    /* PI update — symmetric truncation */
                    int16_t kiCorr = (errHR >= 0)
                        ? (errHR >> ZC_SYNC_KI_SHIFT)
                        : -((-errHR) >> ZC_SYNC_KI_SHIFT);
                    int16_t kpCorr = (errHR >= 0)
                        ? (errHR >> ZC_SYNC_KP_SHIFT)
                        : -((-errHR) >> ZC_SYNC_KP_SHIFT);

                    int32_t newInt = (int32_t)gData.zcSync.syncIntHR + kiCorr;
                    if (newInt < 50) newInt = 50;
                    if (newInt > 2000) newInt = 2000;
                    gData.zcSync.syncIntHR = (uint16_t)newInt;

                    int32_t newT = (int32_t)gData.zcSync.syncIntHR + kpCorr;
                    if (newT < 50) newT = 50;
                    if (newT > 2000) newT = 2000;
                    gData.zcSync.T_hatHR = (uint16_t)newT;

                    gData.zcSync.syncErrHR = errHR;
                    gData.zcSync.lastMeasHR = dmaZcHR;
                    gData.zcSync.diagSyncAccepts++;
                    gData.zcSync.missStreak = 0;
                }
                else
                {
                    gData.zcSync.missStreak++;
                    gData.zcSync.diagSyncMisses++;
                }
            }
            else
            {
                gData.zcSync.missStreak++;
                gData.zcSync.diagSyncMisses++;
            }

            /* Schedule next commutation from PI */
            uint16_t nextTargetHR = thisCommHR + gData.zcSync.T_hatHR;
            uint16_t nowHR = HAL_ComTimer_ReadTimer();
            int16_t margin = (int16_t)(nextTargetHR - nowHR);
            if (margin > 2)
            {
                HAL_ComTimer_ScheduleAbsolute(nextTargetHR);
                gData.timing.deadlineActive = true;
            }
            else
            {
                /* Target already past — fire ASAP */
                HAL_ComTimer_ScheduleAbsolute(nowHR + 4);
                gData.timing.deadlineActive = true;
            }

            /* Update lastCommHR for next sector's PI.
             * In owned mode, use the PI's own commutation time —
             * NOT lastReactiveTargetHR, which stops updating when
             * reactive scheduling is skipped. */
            gData.zcSync.lastCommHR = thisCommHR;
            /* prevStepRisingZc updated in OnCommutation below */

            /* Exit conditions — with grace period after entry.
             * The PI needs ~24 sectors to absorb the entry transient
             * (model offset + timing discontinuity). During grace,
             * only missStreak exit is active (safety). */
            if (gData.zcSync.missStreak > 3)
            {
                gData.zcSync.mode = 1;  /* SHADOW */
                gData.zcSync.fallbackReason = 1;
                gData.zcSync.diagSyncExits++;
                gData.zcSync.goodStreak = 0;
            }
            else if (gData.zcSync.goodStreak > 0)
            {
                /* Grace period: goodStreak counts down from entry.
                 * Repurpose it as a grace counter — decrement each
                 * owned sector. Only check syncErr exit after grace
                 * expires (goodStreak reaches 0). */
                gData.zcSync.goodStreak--;
            }
            else if (gData.zcSync.T_hatHR > 0)
            {
                int16_t absErr = (gData.zcSync.syncErrHR >= 0)
                    ? gData.zcSync.syncErrHR
                    : -gData.zcSync.syncErrHR;
                if (absErr > (int16_t)(gData.zcSync.T_hatHR >> 1))
                {
                    gData.zcSync.mode = 1;
                    gData.zcSync.fallbackReason = 2;
                    gData.zcSync.diagSyncExits++;
                }
            }

            if (gData.state != ESC_CLOSED_LOOP)
            {
                gData.zcSync.mode = 0;  /* OFF */
                gData.zcSync.fallbackReason = 4;
                gData.zcSync.diagSyncExits++;
            }
        }
#endif /* FEATURE_SECTOR_PI */

        /* Gate out SCCP1 fast poll during transition.
         * Only clear deadlineActive if predictor hasn't scheduled
         * the next compare already. */
        gData.icZc.phase = IC_ZC_DONE;
        if (!gData.zcPred.predictiveMode && gData.zcSync.mode != 2)
            gData.timing.deadlineActive = false;
        COMMUTATION_AdvanceStep((volatile GARUDA_DATA_T *)&gData);
        BEMF_ZC_OnCommutation((volatile GARUDA_DATA_T *)&gData);

        /* Re-assert deadlineActive after OnCommutation (which clears
         * it unconditionally at bemf_zc.c:371). */
        if (gData.zcPred.predictiveMode && gData.zcPred.pendingPredValid)
            gData.timing.deadlineActive = true;
#if FEATURE_SECTOR_PI
        if (gData.zcSync.mode == 2)
            gData.timing.deadlineActive = true;
#endif
    }
}

#if FEATURE_IC_ZC_CAPTURE
/**
 * @brief SCCP2 Input Capture ISR — pure IC-driven ZC detection.
 *
 * Replaces the poll path for ZC detection. On the first comparator edge
 * after blanking, validates the interval, records timing, and schedules
 * commutation directly. No deglitch filter — single edge = ZC.
 *
 * The poll path (FastPoll) still runs but serves only as a fallback
 * if the IC misses an edge (e.g. no transition occurs).
 */
void __attribute__((interrupt, no_auto_psv)) _CCP2Interrupt(void)
{
    /* One-shot: disarm immediately to prevent bounce re-triggers */
    _CCP2IE = 0;
    _CCP2IF = 0;

    /* Only process during closed-loop AND when IC was properly armed. */
    if (gData.state != ESC_CLOSED_LOOP || !gData.icZc.icArmed)
        return;

    /* If poll path already got this ZC, stop */
    if (gData.bemf.zeroCrossDetected)
        return;

    /* HYBRID: IC provides precise timestamp, poll validates.
     * IC latches timestamp ONLY if the edge passes 50% interval check.
     * This prevents noise edges (before 50%) from providing a wrong
     * early timestamp that the poll later confirms and schedules from. */

    /* Backdate IC timestamp to SCCP4 domain. */
    uint16_t icCapture = HAL_ZcIC_ReadCapture();
    uint16_t icNow     = HAL_ZcIC_ReadTimer();
    uint16_t s4Now     = HAL_ComTimer_ReadTimer();
    uint16_t elapsed   = icNow - icCapture;
    uint16_t zcTickHR  = s4Now - elapsed;

    /* 50% interval rejection — same as poll path Layer 2.
     * Reject noise edges that arrive before 50% of expected interval.
     * Without this, IC latches an early timestamp that the poll later
     * confirms (comparator stayed in expected state), causing early
     * commutation → rough operation → current spikes → OV fault. */
    {
        uint16_t sinceLastZc = zcTickHR - gData.timing.lastZcTickHR;
        /* Fix B: prefer the *most recent* measured ZC interval over the
         * IIR-averaged refIntervalHR. The IIR has a 25% shrink clamp
         * (bemf_zc.c:1190) plus a 3:1 IIR (~6% effective shrink/ZC),
         * so during acceleration it lags reality by many ZCs. The 50%
         * gate built from a stale (too large) ref then rejects the
         * actual ZC capture as if it were a bounce. zcIntervalHR is
         * the raw last-step measurement — current to within one ZC.
         * Falls back to refIntervalHR, then stepPeriodHR. */
        uint16_t ref = gData.timing.zcIntervalHR;
        if (ref == 0) ref = gData.zcCtrl.refIntervalHR;
        if (ref == 0) ref = gData.timing.stepPeriodHR;
        uint16_t halfInterval = ref >> 1;
        if (halfInterval > 0 && (int16_t)(sinceLastZc - halfInterval) < 0)
        {
            /* Too early — noise edge. Don't latch timestamp.
             * Poll will use its own timestamp when it confirms. */
            gData.icZc.diagIcBounce++;
            return;
        }
    }

    gData.icZc.zcCandidateHR = zcTickHR;
    {
        uint16_t elapsedT1 = (uint16_t)((uint32_t)elapsed * COM_TIMER_T1_DENOM
                                         / COM_TIMER_T1_NUMER);
        gData.icZc.zcCandidateT1 = gData.timer1Tick - elapsedT1;
    }
    gData.icZc.icArmed = false;
    gData.icZc.icCandidateValid = true;
    gData.icZc.diagIcAccepted++;

    /* IC only stores the precise timestamp. Does NOT call
     * RecordZcTiming or ScheduleCommutation — that corrupts the
     * estimator if this edge is noise.
     *
     * The poll path reads CLC D-FF output (clean, PWM-sync sampled).
     * When CLC confirms expected state, poll calls RecordZcTiming
     * using the IC's stored timestamp → precise + validated. */
}
#endif /* FEATURE_IC_ZC_CAPTURE */

#endif /* FEATURE_IC_ZC */

#endif /* !FEATURE_V4_SECTOR_PI — end of V3 code */
