/**
 * @file garuda_service.c
 * @brief Main ESC service (AK port — verbatim CK copy + ISR vector
 *        re-name pass pending).
 *
 * Forked from CK `../../garuda_6step_ck.X/garuda_service.c`.  ISR
 * vector names verified against DS70005539 §6 IRQ table:
 *
 *   _ADCAN5Interrupt   ✓ same on AK (verify exact ADCANn slot)
 *   _T1Interrupt       ✓ same on AK
 *   _CCP1..4Interrupt  ✓ same on AK (NO `_S` prefix — datasheet line
 *                        45565 ff. uses _CCP1Interrupt etc. despite
 *                        the module being called SCCP)
 *   _CCP5Interrupt     ✗ NOT on AK — only SCCP1-4 (datasheet line 4601).
 *                       Any CCP5 reference must be removed or re-mapped
 *                       to SCCP3 for falling-edge diag.
 *
 * `_CCP5Interrupt` references are gated behind FEATURE_V4_CCP_DIAG
 * (default off in garuda_config.h) so the AK build links.  Production
 * motor control runs entirely on the ADC ISR midpoint sampler, which
 * needs no CCP capture.
 *
 * State machine, ADC ISR (PWM-triggered midpoint sampler used for ZC)
 * and Timer1 ISR (50 µs system tick) — all algorithm-side, unchanged.
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
#include "hal/hal_ak_compat.h"   /* CK L/H → AK 32-bit SFR shims */
#include "hal/hal_pwm.h"
#include "hal/hal_adc.h"
#include "hal/hal_ata6847.h"
#include "hal/hal_timer1.h"
#include "hal/hal_opa.h"
#include "hal/hal_uart.h"
#include "hal/port_config.h"
#include "motor/commutation.h"

#include "motor/sector_pi.h"
#include "motor/v4_params.h"
#include "hal/hal_com_timer.h"
#include "hal/hal_capture.h"

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
volatile int16_t  gV4IbusRaw = 0;       /* direct DC-bus shunt read (OA3OUT) */

/* Calibrated electrical values — produced in the ADC ISR by applying the
 * board-specific scale factors from garuda_config.h.  These are what the
 * GSP telemetry ships to the host, so the Python tool only has to display
 * them; it never knows the shunt/gain/divider numbers itself. */
volatile int16_t  gV4Ia_mA   = 0;       /* signed milliamps */
volatile int16_t  gV4Ib_mA   = 0;
volatile int16_t  gV4Ibus_mA = 0;
volatile uint16_t gV4Vbus_mV = 0;       /* unsigned millivolts */

/* Current peak tracking (rolling window, reset on snapshot read).
 * Units: SIGNED MILLIAMPS, written by the ADC ISR after the count→mA
 * conversion.  Previously these held raw ADC counts; switched to mA so
 * the host doesn't need to know the shunt/gain calibration.  Range is
 * ±32 A in int16, well above the ±22 A ADC hardware ceiling. */
volatile int16_t  gV4IaPkMax   = 0, gV4IaPkMin   = 0;   /* mA */
volatile int16_t  gV4IbPkMax   = 0, gV4IbPkMin   = 0;   /* mA */
volatile int16_t  gV4IbusPkMax = 0, gV4IbusPkMin = 0;   /* mA */

/* At-fault frozen snapshot — populated when V4 enters FAULT, preserved
 * until motor restart. Captures exact peaks at the moment of trip.
 * Units: milliamps (same as the live peaks above). */
volatile int16_t  gV4IaAtFaultMax = 0, gV4IaAtFaultMin = 0;
volatile int16_t  gV4IbAtFaultMax = 0, gV4IbAtFaultMin = 0;
volatile int16_t  gV4IbusAtFaultMax = 0, gV4IbusAtFaultMin = 0;
volatile int16_t  gV4IaAtFaultInst = 0, gV4IbAtFaultInst = 0, gV4IbusAtFaultInst = 0;
volatile uint8_t  gV4FaultSnapshotValid = 0;

static inline void V4FreezeAtFaultPeaks(void)
{
    gV4IaAtFaultMax   = gV4IaPkMax;   gV4IaAtFaultMin   = gV4IaPkMin;
    gV4IbAtFaultMax   = gV4IbPkMax;   gV4IbAtFaultMin   = gV4IbPkMin;
    gV4IbusAtFaultMax = gV4IbusPkMax; gV4IbusAtFaultMin = gV4IbusPkMin;
    gV4IaAtFaultInst   = gV4Ia_mA;
    gV4IbAtFaultInst   = gV4Ib_mA;
    gV4IbusAtFaultInst = gV4Ibus_mA;
    gV4FaultSnapshotValid = 1;
}
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
    _T1IF = 0;

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

            /* Throttle — scale 12-bit pot (0..4095) to Q15 amplitude
             * (0..32768). Original code did `>> 1` which capped at 2047
             * — always below V4_MIN_AMPLITUDE=5000 floor, so pot had no
             * effect. `<< 3` maps potRaw=0..4095 to amp=0..32760, full
             * Q15 range. Dead-zone at potRaw < 200 (~5% pot) → amp=0 →
             * SectorPI_CommandSet floors to MIN_AMPLITUDE (15% idle). */
            uint16_t amp = (gV4PotRaw < 200u) ? 0u
                                              : (uint16_t)(gV4PotRaw << 3);
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
extern volatile bool     v4_spActive;     /* from sector_pi.c: SP mode flag */

#if FEATURE_V5_SCHEDULER
/* V5.3 scheduler state (from sector_pi.c). CCP ISRs update these
 * on ZC capture and schedule next Commutate. */
extern volatile uint16_t sched_prevCaptureHR;
extern volatile uint16_t sched_thisCaptureHR;
extern volatile uint16_t sched_Tsector;
extern volatile uint8_t  sched_captureSource;
extern volatile uint32_t sched_diagRisingAcc;
extern volatile uint32_t sched_diagFallingAcc;
extern volatile uint32_t sched_diagScheduleLate;
extern volatile uint32_t sched_diagScheduleOk;
extern volatile uint32_t sched_diagImplausible;
extern volatile bool     sched_firstCapture;
#define V5_SCHED_FORWARD_MARGIN_HR  100u     /* min 64µs → >2 PWM cycles */

/* Shared V5.3 capture accept helper — called from both CCP2 and CCP5
 * ISRs after polarity/deglitch validation. Handles first-capture seed,
 * dt plausibility, EMA update, and scheduling with forward margin. */
static inline void V5_AcceptCapture(uint16_t hr, uint8_t source)
{
    if (sched_firstCapture) {
        /* Bootstrap: use the ramp-seeded sched_Tsector for the first
         * schedule. Skip dt computation (prev values are zero). */
        sched_thisCaptureHR = hr;
        sched_prevCaptureHR = hr;
        sched_captureSource = source;
        sched_firstCapture = false;
    } else {
        uint16_t dt = (uint16_t)(hr - sched_thisCaptureHR);
        /* Plausibility: absolute noise floor only (100 HR = 64µs,
         * above PWM switching settle). No upper bound from T — motor
         * acceleration must be allowed (dt shrinking is legal).
         * If dt > 4T we probably missed multiple sectors — reject and
         * let fallback fire so we don't corrupt T upward. */
        if (dt < 100u) {
            sched_diagImplausible++;
            return;        /* noise — no state update */
        }
        uint16_t hiLimit = (sched_Tsector < 0x4000u) ? (sched_Tsector << 2) : 0xFFFFu;
        if (dt > hiLimit) {
            sched_diagImplausible++;
            return;        /* too slow — probably missed multiple sectors */
        }
        /* Accept — advance capture history, EMA T. If dt is much
         * larger than T (1.5T-4T range) it may be a missed sector
         * (dt = 2× real period); apply weaker EMA (α=1/8) so T doesn't
         * balloon. Otherwise use α=1/4 for normal tracking. */
        sched_prevCaptureHR = sched_thisCaptureHR;
        sched_thisCaptureHR = hr;
        sched_captureSource = source;
        int32_t err = (int32_t)dt - (int32_t)sched_Tsector;
        int32_t nt;
        if (dt > (uint16_t)(sched_Tsector + (sched_Tsector >> 1))) {
            nt = (int32_t)sched_Tsector + (err >> 3);   /* weak: might be 2×T */
        } else {
            nt = (int32_t)sched_Tsector + (err >> 2);   /* normal */
        }
        if (nt < 10)     nt = 10;
        if (nt > 0xFFFF) nt = 0xFFFF;
        sched_Tsector = (uint16_t)nt;
    }
    /* Compute target = hr + (T/2 - advance). */
    uint16_t halfT = sched_Tsector >> 1;
    uint16_t tal   = (sched_Tsector < 260u) ? 3u : 2u;
    uint16_t advHR = (uint16_t)((sched_Tsector >> 3) * tal);
    uint16_t delayHR = (halfT > advHR) ? (halfT - advHR) : V5_SCHED_FORWARD_MARGIN_HR;
    uint16_t target = (uint16_t)(hr + delayHR);
    /* Forward margin: max(100 HR, T/4). Never fire Commutate sooner
     * than blanking-width after now — prevents chatter cascades. */
    uint16_t minMargin = sched_Tsector >> 2;
    if (minMargin < V5_SCHED_FORWARD_MARGIN_HR) minMargin = V5_SCHED_FORWARD_MARGIN_HR;
    uint16_t nowHR = HAL_ComTimer_ReadTimer();
    int16_t margin = (int16_t)(target - nowHR);
    if (margin < (int16_t)minMargin) {
        target = (uint16_t)(nowHR + minMargin);
        sched_diagScheduleLate++;
    } else {
        sched_diagScheduleOk++;
    }
    HAL_ComTimer_ScheduleAbsolute(target);
    v4_captureValid = true;
    v4_lastCaptureHR = hr;
    /* Disable CCP ISRs — next Commutate re-enables after reconfig. */
    _CCP2IE = 0;
    _CCP5IE = 0;
}
#endif

/* ── ADC midpoint ZC diagnostic counters (Mode 1) ─────────────── */
/* Used to diagnose why capture rate sits at ~50% at high speed.
 * Reset on motor start in SectorPI_Start().
 * 32-bit — at 40kHz ADC rate, uint16 wraps every ~1.6s making
 * multi-second tests unusable. */
volatile uint32_t v4_adcBlankReject  = 0;  /* ADC fired pre-blanking-end */
volatile uint32_t v4_adcStateMismatch = 0; /* past blanking, wrong GPIO  */
volatile uint32_t v4_adcCaptureSet   = 0;  /* set v4_captureValid       */
volatile uint32_t v4_adcAlreadySet   = 0;  /* skipped — already true    */
/* Polarity split: sets that happened on rising-ZC sectors (0,2,4).
 * Falling portion is (v4_adcCaptureSet - v4_adcSetRising).
 * Uint32 data shows capture rate is a flat 49% across all speeds —
 * structural, not speed-dependent. This counter tests the hypothesis
 * that one polarity class (rising vs falling) misses every time. */
volatile uint32_t v4_adcSetRising    = 0;

/* 2026-05-14 capture-layer probe: tally what the PTG sample finds at the
 * mid-ON instant, binned by sector polarity. Used to decide if
 * rising-sector misses are a *physics asymmetry* (comp stuck in pre-state
 * at the sample point — no transition ever observed) or a *software*
 * problem (comp shows post-state but accept logic discards it).
 *
 * On the inverted ATA6847:
 *   rising-BEMF sector: pre-ZC comp=1, post-ZC comp=0
 *   falling-BEMF sector: pre-ZC comp=0, post-ZC comp=1
 *
 * Healthy capture requires comp == post-ZC by the time we sample. If
 * compRising_High >> compRising_Low across the run, the rising-sector
 * BEMF isn't reaching the (mid-ON-shifted) virtual neutral and the
 * capture layer can never see the transition. */
volatile uint32_t v4_compRising_High  = 0;  /* rising sector,  comp=1 (pre-ZC state)  */
volatile uint32_t v4_compRising_Low   = 0;  /* rising sector,  comp=0 (post-ZC state) */
volatile uint32_t v4_compFalling_High = 0;  /* falling sector, comp=1 (post-ZC state) */
volatile uint32_t v4_compFalling_Low  = 0;  /* falling sector, comp=0 (pre-ZC state)  */

/* SCCP1 off-mid falling ZC diagnostic (fires at PWM peak) */
volatile uint32_t v4_offMidCapture   = 0;
volatile uint32_t v4_offMidMismatch  = 0;

/* V5.1 post-ZC shadow counters. Incremented in _ADCInterrupt every
 * past-blanking sample (no v4_captureValid sticky gate, so per-sample
 * rate matches what the PTG diagnostic measured). When
 * FEATURE_V5_POST_ZC_ACCEPT=0 these stay at 0.
 *   v5_postZcRisingAcc  — rising sector + comp == 0 (post-ZC for rising)
 *   v5_postZcRisingRej  — rising sector + comp != 0
 *   v5_postZcFallingAcc — falling sector + comp == 1 (post-ZC for falling)
 *   v5_postZcFallingRej — falling sector + comp != 1
 * Expected from PTG comparison: per-polarity Acc/(Acc+Rej) ≈ 67%. */
volatile uint32_t v5_postZcRisingAcc  = 0;
volatile uint32_t v5_postZcRisingRej  = 0;
volatile uint32_t v5_postZcFallingAcc = 0;
volatile uint32_t v5_postZcFallingRej = 0;

/* B1 mid-ON diagnostic (FEATURE_MIDON_DIAG_PROBE).
 * Falling-sector second-read counters. PTG ISR fires at mid-OFF
 * (PG1TRIGB = duty match) and runs V4_ProcessBemfSample. When this
 * feature is on, the ISR busy-waits ~half a PWM period after the
 * normal mid-OFF read, takes a second comp read at mid-ON, and
 * classifies it for falling sectors only. Tests the hypothesis
 * that mid-ON has cleaner falling-sector BEMF than mid-OFF. */
volatile uint32_t v5_midOnFallingAcc = 0;
volatile uint32_t v5_midOnFallingRej = 0;

/* Edge-detection state for FEATURE_V5_POST_ZC_OWN.
 *
 * AK bring-up showed that pure "comp == expectedPost" acceptance phantoms
 * out on this carrier: 3 of 6 sectors find the idle comparator state
 * already matching expectedPost the moment blanking ends, so they
 * phantom-accept before any actual rotor crossing. The CK board didn't
 * see this because its idle BEMF bias was different.
 *
 * Fix: require the comp to be observed in the PRE-ZC state at least once
 * after blanking before accepting the post-ZC state. Real rotor rotation
 * always sweeps comp through pre → ZC → post. Idle bias starts at post
 * and never produces a pre observation, so the gate stays armed forever
 * and no phantom accepts.
 *
 * Reset to 0 at every commutation (sector_pi.c Commutate). */
volatile uint8_t  v5_sawPreZc = 0;

#if FEATURE_V4_MIDPOINT_ZC >= 1
/* (midpoint modes also need these) */
#endif

/* Mode 2 hybrid: midpoint confirms ZC state, CCP provides timestamp */
#if FEATURE_V4_MIDPOINT_ZC == 2
static volatile bool v4_zcConfirmed = false;  /* Set by ADC ISR, consumed by CCP ISR */
#endif

/* ── V4 ADC ISR ───────────────────────────────────────────────────── */
void __attribute__((interrupt, auto_psv)) _AD1CH4Interrupt(void)
{
    gV4PotRaw  = ADCBUF_POT;
    gV4VbusRaw = ADCBUF_VBUS;
    /* Phase + bus currents: unsigned 12-bit ADC reads, zero-current bias
     * subtracted to give a signed int16 swing around 0.  Raw counts are
     * kept for internal use; the host-facing values are converted to
     * milliamps below using the calibration constants from garuda_config.h. */
    gV4IaRaw   = (int16_t)((int16_t)ADCBUF_IA   - ADC_CURRENT_BIAS);
    gV4IbRaw   = (int16_t)((int16_t)ADCBUF_IB   - ADC_CURRENT_BIAS);
    gV4IbusRaw = (int16_t)((int16_t)ADCBUF_IBUS - ADC_CURRENT_BIAS);

    /* Convert ADC counts → physical units once, at the source.  All
     * downstream consumers (snapshot builder, fault snapshot, peak
     * tracker, future current limiter) read the calibrated globals so
     * that swapping the DIM gain resistors only requires updating the
     * two _Q8 constants in garuda_config.h.  Q8 fixed-point keeps the
     * multiply in int32 and the result in int16 with no FP. */
    gV4Vbus_mV = (uint16_t)(((uint32_t)gV4VbusRaw * ADC_VBUS_MV_PER_COUNT_Q8) >> 8);
    gV4Ia_mA   = (int16_t)(((int32_t)gV4IaRaw   * ADC_MA_PER_COUNT_Q8) >> 8);
    gV4Ib_mA   = (int16_t)(((int32_t)gV4IbRaw   * ADC_MA_PER_COUNT_Q8) >> 8);
    gV4Ibus_mA = (int16_t)(((int32_t)gV4IbusRaw * ADC_MA_PER_COUNT_Q8) >> 8);

    /* Peak tracking in milliamps (rolling window reset on snapshot read).
     * Direct OA3 read of the DC-bus shunt → ibusPk straight from gV4Ibus_mA
     * instead of being host-reconstructed from |Ia|/|Ib| extrema (which
     * underestimated by √3 in C-PWM sectors). */
    if (gV4Ia_mA   > gV4IaPkMax)   gV4IaPkMax   = gV4Ia_mA;
    if (gV4Ia_mA   < gV4IaPkMin)   gV4IaPkMin   = gV4Ia_mA;
    if (gV4Ib_mA   > gV4IbPkMax)   gV4IbPkMax   = gV4Ib_mA;
    if (gV4Ib_mA   < gV4IbPkMin)   gV4IbPkMin   = gV4Ib_mA;
    if (gV4Ibus_mA > gV4IbusPkMax) gV4IbusPkMax = gV4Ibus_mA;
    if (gV4Ibus_mA < gV4IbusPkMin) gV4IbusPkMin = gV4Ibus_mA;

#if FEATURE_V4_MIDPOINT_ZC == 1
#if !FEATURE_BEMF_VIA_PTG
    /* Default path: BEMF detection runs inside the ADC ISR, right after
     * the current/Vbus reads.  When FEATURE_BEMF_VIA_PTG=1 the same body
     * runs from the PTG ISR instead, ~1.5 µs earlier and decoupled from
     * the ADC scan.  See V4_ProcessBemfSample() below for the actual
     * logic — kept as one function so both call sites stay byte-equal. */
    V4_ProcessBemfSample();
#endif
#elif FEATURE_V4_MIDPOINT_ZC == 2
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
                v4_zcConfirmed = true;
            }
        }
    }
#endif

    /* [AK PORT] CK had a shared ADCIF; AK uses per-channel flags.
     * Clear AD1CH4 (VBUS) since it triggered this ISR. */
    _AD1CH4IF = 0;
}

#if FEATURE_V4_MIDPOINT_ZC == 1
#if FEATURE_V4_PTG_RESCHEDULE
/* ── V4_NextTargetHR — capture-anchored next-Commutate target ──────
 *
 * What it does:
 *   Given a fresh capture timestamp (in SCCP4 HR-tick domain) and the
 *   PI's current sector-period estimate, compute the HR time at which
 *   the next Commutate should fire to land the PWM step at the correct
 *   electrical advance relative to the rotor's physical ZC.
 *
 * The formula:
 *     halfHR  = T/2                    // half a sector
 *     tal     = speed-banded multiplier (2..5)
 *     advHR   = (T/8) * tal             // advance in HR ticks
 *     delayHR = max(halfHR - advHR, 2)  // time from capture to target
 *     target  = captureHR + delayHR
 *
 * The 2-tick clamp engages above ~156 k eRPM when advHR > T/2 — at
 * that point we fire essentially immediately after the capture. This
 * is part of the 2T:ε pacing the legacy V4 scheduler relies on; see
 * docs/v4_ak_port_scheduler.md §7.
 *
 * Two callers must agree on this formula:
 *   1. PTG ISR (this file, V4_ProcessBemfSample) — pulls CCP3 forward
 *      the instant a capture is accepted, so CCP3 fires at the
 *      capture-anchored target instead of the previously-armed
 *      fallback. ONLY when FEATURE_V4_PTG_RESCHEDULE=1.
 *   2. SectorPI_Commutate tail (sector_pi.c ~line 1094-1116) — when
 *      consuming the capture, computes the same target.
 *
 * Lifted into an inline so the two callers can never drift. */
static inline uint16_t V4_NextTargetHR(uint16_t captureHR,
                                       uint16_t schedPeriod)
{
    uint16_t halfHR = (uint16_t)(schedPeriod >> 1);
    uint16_t tal    = (schedPeriod >= 260U) ? 2U   /* ≤60 k eRPM  → 15.0° */
                    : (schedPeriod >= 130U) ? 3U   /* 60-120 k    → 22.5° */
                    : (schedPeriod >= 100U) ? 4U   /* 120-156 k   → 30.0° */
                    :                         5U;  /* >156 k      → 37.5° */
    uint16_t advHR   = (uint16_t)((schedPeriod >> 3) * tal);
    uint16_t delayHR = (halfHR > advHR) ? (uint16_t)(halfHR - advHR) : 2U;
    return (uint16_t)(captureHR + delayHR);
}

/* Diagnostic counter — incremented every PTG fire that called
 * HAL_ComTimer_ScheduleAbsolute via the reschedule path. Useful to
 * confirm at the bench that the feature is actually engaged. */
volatile uint32_t v4_ptgRescheduleCount = 0;
#endif /* FEATURE_V4_PTG_RESCHEDULE */

/* ── V4_ProcessBemfSample — runs once per PTG fire, decides ZC ─────
 *
 * Called from _PTG0Interrupt (hal_ptg.c) when FEATURE_BEMF_VIA_PTG=1.
 * Historically also called from the ADC ISR — hence the `v4_adc*`
 * counter names. Active path is PTG.
 *
 * Decision flow:
 *   Gate 1: motor must be in CL  → else return.
 *   Gate 2: v4_captureValid already set this sector? → just bump
 *           v4_adcAlreadySet and return (one accepted ZC per sector).
 *   Gate 3: are we past v4_blankingEndHR? → else v4_adcBlankReject++.
 *           Blanking rejects the post-Commutate ringing window.
 *   Probe: tally per-(sector,phase) comp=1 counts in v4_bemfTally[].
 *          Tally is reset on each telemetry snapshot send so the
 *          ratios are a fresh ~50 ms window.
 *   Deglitch: read the comparator 3 times with NOPs between (~400 ns
 *             total). Majority vote rejects single-sample bounces.
 *   Polarity gate: compare `comp` to `expected`. Expected is currently
 *                  hard-coded to 0 (catches comp=0 events). Sectors
 *                  matching → set captureValid + lastCaptureHR. Other
 *                  polarity → just count v4_adcStateMismatch.
 *   Optional reschedule (FEATURE_V4_PTG_RESCHEDULE): pull CCP3
 *                 forward to fire at captureHR+delayHR. OFF by default
 *                 on this branch — see config.h comment.
 *
 * The two key globals it writes:
 *   v4_captureValid — "there's a fresh capture waiting" flag.
 *                     Consumed by SectorPI_Commutate.
 *   v4_lastCaptureHR — timestamp of that capture, in SCCP4 domain.
 *
 * NOT marked inline: called from two ISRs in different .o files.
 * Out-of-line is correct. */
void V4_ProcessBemfSample(void)
{
    /* Same scaffold the ADC ISR used to host directly.  Original
     * comments preserved verbatim because they document hard-won
     * tuning history (single-read vs 3-read regression, isRising
     * stuck-true workarounds, PI-feed polarity rationale, etc). */
    if (!v4_spActive
        && SectorPI_IsRunning() && SectorPI_GetPhase() == 3)
    {
        if (v4_captureValid)
        {
            v4_adcAlreadySet++;
        }
        else
        {
            uint16_t nowHR = CCP4TMRL;
            if ((int16_t)(nowHR - v4_blankingEndHR) < 0)
            {
                v4_adcBlankReject++;
            }
            else
            {
                /* 2026-05-15 — multi-phase tally probe.  Read ALL three
                 * BEMF GPIOs once and tally per-sector comp=1 counts per
                 * phase.  This runs BEFORE the deglitch and capture
                 * logic, costs ~10 cycles, and tells us whether the
                 * floatingPhase mapping points at the actually-floating
                 * pin in each sector. */
                {
                    extern volatile uint8_t  v4_currentSector;
                    extern volatile uint16_t v4_bemfTally[6][3];
                    extern volatile uint16_t v4_bemfTallyTotal[6];
                    extern volatile uint16_t v4_fpStaleCount;
                    uint8_t sect = v4_currentSector;
                    if (sect < 6u) {
                        uint8_t bA = BEMF_A_GetValue();
                        uint8_t bB = BEMF_B_GetValue();
                        uint8_t bC = BEMF_C_GetValue();
                        v4_bemfTallyTotal[sect]++;
                        if (bA) v4_bemfTally[sect][0]++;
                        if (bB) v4_bemfTally[sect][1]++;
                        if (bC) v4_bemfTally[sect][2]++;

                        /* Stale-fp probe: does v4_floatingPhase match the
                         * table's belief for v4_currentSector? Both are
                         * written by SectorPI_Commutate but at different
                         * statements — drift here means the deglitch
                         * (which uses v4_floatingPhase) is reading the
                         * wrong pin. */
                        uint8_t fp_table = commutationTable[sect].floatingPhase;
                        if (fp_table != v4_floatingPhase) v4_fpStaleCount++;
                    }
                }

                /* 3-read deglitch: reject single-sample GPIO bounce /
                 * comparator chatter near threshold. Majority vote of
                 * 3 reads with short gaps (~400 ns total). Tested
                 * single-read above 150k 2026-04-28 — regressed peak
                 * 195k→178k, so 3-read is load-bearing for noise
                 * rejection at all speeds. */
                uint8_t r1 = ReadBEMFComp();
                Nop(); Nop(); Nop(); Nop();
                uint8_t r2 = ReadBEMFComp();
                Nop(); Nop(); Nop(); Nop();
                uint8_t r3 = ReadBEMFComp();
                uint8_t comp = ((uint8_t)(r1 + r2 + r3) >= 2u) ? 1u : 0u;

                bool isRising = HAL_Capture_IsRisingZc();

                /* Capture-layer probe (2026-05-14): tally comp value per
                 * sector polarity, past blanking. Runs in BOTH detection
                 * paths (V4 PRE-ZC and V5 POST_ZC_OWN) so the same probe
                 * can compare what mid-OFF vs mid-ON sees regardless of
                 * which accept logic is active. Uses v5_ptgExpectedComp
                 * (reliable per-sector flag) not isRising (stuck-true). */
                {
                    extern volatile uint8_t v5_ptgExpectedComp;
                    bool sectorRising_probe = (v5_ptgExpectedComp == 0u);
                    if (sectorRising_probe) {
                        if (comp) v4_compRising_High++;
                        else      v4_compRising_Low++;
                    } else {
                        if (comp) v4_compFalling_High++;
                        else      v4_compFalling_Low++;
                    }
                }

#if FEATURE_V5_POST_ZC_ACCEPT
                /* V5.1 shadow: count what post-ZC accept logic would do.
                 * Reads v5_ptgExpectedComp (written by Commutate) instead
                 * of HAL_Capture_IsRisingZc() — the function call exhibits
                 * a "stuck true" behavior in ISR context that the volatile
                 * global bypasses. First run of this shadow with the
                 * function call showed p2F=0% (impossible); this read
                 * is what PTG ISR successfully used to get pR/pF ≈ 67/67.
                 * No v4_captureValid sticky gate — per-sample rate matches
                 * the PTG diagnostic for direct comparison. */
                {
                    extern volatile uint8_t v5_ptgExpectedComp;
                    uint8_t expectedPostZc = v5_ptgExpectedComp;
                    bool    sectorRising   = (expectedPostZc == 0u);
                    if (comp == expectedPostZc) {
                        if (sectorRising) v5_postZcRisingAcc++;
                        else              v5_postZcFallingAcc++;
                    } else {
                        if (sectorRising) v5_postZcRisingRej++;
                        else              v5_postZcFallingRej++;
                    }
                }
#endif

#if FEATURE_V5_POST_ZC_OWN
                /* V5.1-step2 post-ZC convention.  Accept when comp matches
                 * the POST-ZC state for this sector — the inverted ATA6847
                 * settles to 0 after rising-BEMF ZC and to 1 after
                 * falling-BEMF ZC, so v5_ptgExpectedComp (0 for even/rising,
                 * 1 for odd/falling sectors) is exactly the post-ZC state.
                 *
                 * Reads v5_ptgExpectedComp instead of HAL_Capture_IsRisingZc
                 * to avoid the stuck-TRUE behaviour the diagnostic ISRs
                 * documented around 2026-04-18 — the volatile global is
                 * written cleanly by the Commutate ISR and is the same
                 * source the working PTG/post-ZC shadow paths use.
                 *
                 * Replaces the legacy V4 pre-ZC logic which only ever
                 * caught one polarity and fed the PI bimodal capValues
                 * (one cluster = real ZC at ~50% T, other = first
                 * post-blanking sample where pre-ZC state still held). */
                /* 2026-05-14 reverted to edge-gated form. CK-aligned version
                 * (no edge gate, no piFeedPolarity) caused cR cascade on AK
                 * — accepted-first-match-of-expectedPost is a positive-
                 * feedback loop at CL entry on AK hardware (idle bias
                 * matches expectedPost every PTG fire). The v5_sawPreZc
                 * gate is an AK-specific cascade protection. CK GitHub
                 * baseline uses V5_OWN=0 anyway, so V5_OWN=1 on AK is
                 * untested even in concept. */
                {
                    extern volatile uint8_t v5_ptgExpectedComp;
                    extern volatile uint8_t v5_sawPreZc;
                    uint8_t expectedPost = v5_ptgExpectedComp;
                    bool    sectorRising = (expectedPost == 0u);
                    if (comp != expectedPost)
                    {
                        v5_sawPreZc = 1u;
                        v4_adcStateMismatch++;
                    }
                    else if (v5_sawPreZc)
                    {
                        v4_adcCaptureSet++;
                        if (sectorRising) v4_adcSetRising++;

                        bool feedPi;
                        switch (v4Params.piFeedPolarity)
                        {
                            case 1:  feedPi = sectorRising;   break;
                            case 2:  feedPi = !sectorRising;  break;
                            default: feedPi = true;           break;
                        }
                        if (feedPi)
                        {
                            v4_lastCaptureHR = nowHR;
                            v4_captureValid  = true;
#if FEATURE_V4_PTG_RESCHEDULE
                            if (v4_timerPeriod >= V4_MIN_PERIOD) {
                                HAL_ComTimer_ScheduleAbsolute(
                                    V4_NextTargetHR(nowHR, v4_timerPeriod));
                                v4_ptgRescheduleCount++;
                            }
#endif
                        }
                        v5_sawPreZc = 0u;
                    }
                }
#else
                /* Legacy V4 PRE-ZC detection.
                 *
                 * Polarity gate uses v5_ptgExpectedComp (written by Commutate
                 * per sector — reliable) instead of HAL_Capture_IsRisingZc()
                 * (function call exhibits "stuck-true" behavior in ISR context,
                 * see comments at ~line 535 and CCP2 ISR for history).
                 * `isRising` is still used for state-match counting because
                 * that's tied to the inverted-ATA6847 comp polarity rules
                 * and is independent of which sector flag we trust. */
                extern volatile uint8_t v5_ptgExpectedComp;
                bool sectorRising_v5 = (v5_ptgExpectedComp == 0u);

                /* 2026-05-15 — falling-only PI feed.
                 *
                 * `expected = 0` means: accept comp=0.  On inverted ATA6847
                 * this is PRE-ZC of falling (BEMF still above neutral).
                 * Rising sectors stay at comp=1 (preR=100% across the whole
                 * speed range, idle → 226k BC) so rising never reaches the
                 * gate.  Net effect: PI is fed from falling-sector captures
                 * only, at every speed.
                 *
                 * Bench (2026-05-15): high-speed current improved slightly
                 * vs the prior legacy gate (`isRising ? 1 : 0`).  Idle is
                 * a touch higher in current/speed.  Keeping this until a
                 * fix for the rising-sector floating-phase asymmetry lands. */
                uint8_t expected = 0;
                if (comp != expected)
                {
                    v4_adcStateMismatch++;
                }
                else
                {
                    v4_adcCaptureSet++;
                    if (sectorRising_v5) v4_adcSetRising++;

                    bool feedPi;
                    switch (v4Params.piFeedPolarity)
                    {
                        case 1:  feedPi = sectorRising_v5;   break;
                        case 2:  feedPi = !sectorRising_v5;  break;
                        default: feedPi = true;              break;
                    }
                    if (feedPi)
                    {
                        /* Publish the capture for Commutate to consume.
                         * nowHR is the SCCP4 free-running tick at the
                         * moment the deglitch sample passed. */
                        v4_lastCaptureHR = nowHR;
                        v4_captureValid = true;
#if FEATURE_V4_PTG_RESCHEDULE
                        /* Experimental (default OFF) — reschedule CCP3
                         * from this PTG fire so the next Commutate
                         * lands at the capture-anchored target instead
                         * of waiting for the previously-armed fallback.
                         *
                         * Architecturally correct: kills the 2T:ε
                         * ASAP-pair, gives 1:1 sector-to-Commutate
                         * mapping, both polarities feed PI.
                         *
                         * Practically broken: V4 PI was tuned to the
                         * 2T:ε scale; with 1:1 the measured period
                         * doubles, PI sees a 2× speedup, slams
                         * timerPeriod down by half, motor desyncs.
                         * See docs/v4_ak_port_scheduler.md §7 and
                         * memory/ak_v4_ptg_reschedule_2026_05_15.md.
                         *
                         * Floor at V4_MIN_PERIOD: skip the reschedule
                         * during very first commutations of CL when
                         * timerPeriod hasn't been seeded yet. The
                         * pre-armed fallback fires safely. */
                        if (v4_timerPeriod >= V4_MIN_PERIOD) {
                            HAL_ComTimer_ScheduleAbsolute(
                                V4_NextTargetHR(nowHR, v4_timerPeriod));
                            v4_ptgRescheduleCount++;
                        }
#endif
                    }
                }
#endif
            }
        }

#if FEATURE_MIDON_DIAG_PROBE
        /* B1 mid-ON diagnostic. Runs every time the PTG ISR enters with
         * sector 3 + PI active, regardless of blanking or captureValid.
         * Busy-waits ~half a PWM period, then re-reads the BEMF comp.
         * Classifies falling-sector samples only — testing whether
         * mid-ON has a cleaner signal than mid-OFF for falling sectors
         * (where cF=0 / pF=0% under the normal mid-OFF read).
         *
         * Cost: ~MIDON_DIAG_LOOPS × 6 cycles inside the PTG ISR. At
         * 60kHz PWM that's ~8µs busy-wait, ~50% of PWM period burned
         * in ISR. Disable for normal operation — diagnostic only. */
        {
            extern volatile uint8_t v5_ptgExpectedComp;
            uint8_t expectedPostZc = v5_ptgExpectedComp;
            bool sectorRising = (expectedPostZc == 0u);
            if (!sectorRising) {
                volatile uint16_t spinIdx;
                for (spinIdx = 0; spinIdx < MIDON_DIAG_LOOPS; spinIdx++) {
                    /* spin */
                }
                uint8_t comp_midOn = ReadBEMFComp();
                if (comp_midOn == expectedPostZc) v5_midOnFallingAcc++;
                else                              v5_midOnFallingRej++;
            }
        }
#endif
    }
}
#endif /* FEATURE_V4_MIDPOINT_ZC == 1 */

/* ── SCCP1 ISR: falling ZC level check at PWM OFF-mid ────────────
 * Fires at 40 kHz, phase-offset from ADC ISR by 12.5 µs (PWM peak).
 * Independent timer — no PWM trigger chain involvement. */
void __attribute__((interrupt, no_auto_psv)) _CCT1Interrupt(void)
{
    _CCT1IF = 0;
#if FEATURE_V4_MIDPOINT_ZC == 1
    if (!v4_spActive
        && SectorPI_IsRunning() && SectorPI_GetPhase() == 3
        && !HAL_Capture_IsRisingZc())
    {
        uint16_t nowHR = CCP4TMRL;
        if ((int16_t)(nowHR - v4_blankingEndHR) >= 0)
        {
            if (ReadBEMFComp() == 1)
                v4_offMidCapture++;
            else
                v4_offMidMismatch++;
        }
    }
#endif
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
#if FEATURE_V4_CCP_DIAG
uint16_t prevEdgeCCP2 = 0;
uint16_t prevEdgeCCP5 = 0;
#endif

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
    /* [AK PORT] CK had BEMF on _RC6/_RC7/_RD10; AK routes the same
     * ATA6847L digital comparator outputs to RB9/RB8/RA10 (per AN6285
     * + DS70005527 DIM pin map). BEMF_x_GetValue() macros wrap the
     * board-specific GPIO so this code is identical across boards. */
    switch (v4_floatingPhase) {
        case 0: return BEMF_A_GetValue();
        case 1: return BEMF_B_GetValue();
        case 2: return BEMF_C_GetValue();
        default: return 0;
    }
}

/* Blanking state */
volatile uint16_t v4_blankingEndHR = 0;
/* Expected ZC HR timestamp (center of corridor for SP CCP ISR) */
volatile uint16_t v4_expectedZcHR = 0;

/* 2026-05-15 — Multi-phase BEMF tally probe.
 *
 * In every post-blanking PTG fire, read ALL three BEMF GPIOs (A,B,C)
 * and count comp=1 occurrences per (sector, phase). Plus total
 * post-blanking fires per sector. Goal: definitively answer whether
 * `preR=100%` (rising sectors never show comp=0 on the floating phase)
 * is a phase-mapping bug or true motor/comparator physics.
 *
 * Reading: in sector S where code thinks phase P floats, the floating
 * phase should show a TRANSITION — its comp=1 ratio should fall in
 * (10..90)%. A ratio at 0% or 100% means we're reading a driven phase.
 * If the driven phase happens to be the one our floatingPhase index
 * points at, the phase-mapping is bugged. */
volatile uint16_t v4_bemfTally[6][3]  = {{0}};   /* [sector][phase] count of comp=1 */
volatile uint16_t v4_bemfTallyTotal[6] = {0};    /* per-sector post-blanking fires */

/* 2026-05-15 — stale-floatingPhase diagnostic.
 *
 * Counts PTG ISR fires (past blanking) where v4_floatingPhase doesn't
 * match commutationTable[v4_currentSector].floatingPhase. Should be 0
 * in steady state because the Commutate ISR (priority 6) atomically
 * sets both v4_currentSector and v4_floatingPhase, and PTG (priority
 * 5) can't preempt it. Non-zero = a write to one of those globals is
 * being missed, which would explain why the deglitched comp (via
 * v4_floatingPhase) reports 100% comp=1 on rising sectors while the
 * raw multi-phase tally (via direct pin reads) reports 99% comp=0.
 * Reset by the snapshot send so each ~50 ms frame shows the delta. */
volatile uint16_t v4_fpStaleCount = 0;
/* (v4_adc* counters declared earlier — used by ADC ISR above) */

#if FEATURE_V4_CCP_DIAG
/* [AK PORT] CCP capture diagnostic path — off by default because:
 *  (a) V4 milestone drives motor from ADC-ISR midpoint sampler;
 *  (b) `_CCP5Interrupt` doesn't exist on AK (no SCCP5 — only SCCP1-4).
 * Re-enable only after re-targeting the CCP5 path to SCCP3 on AK and
 * verifying SCCP2 IC FIFO semantics. */

void __attribute__((interrupt, no_auto_psv)) _CCP2Interrupt(void)
{
    uint16_t ts = 0;
    bool got = false;
    while (CCP2STATLbits.ICBNE) { ts = CCP2BUFL; got = true; }

#if FEATURE_V5_SCHEDULER
    /* V5.3 path: validate rising-ZC capture, hand off to shared helper. */
    if (got && HAL_Capture_IsRisingZc() && !v4_captureValid) {
        uint16_t hr = (uint16_t)(ts + HAL_Capture_GetCcp2Offset());
        if ((int16_t)(hr - v4_blankingEndHR) >= 0) {
            /* Rising ZC → comp settles at 0. Deglitch. */
            uint8_t r1 = ReadBEMFComp();
            bool accept;
            if (sched_Tsector < 260) {
                accept = (r1 == 0);
            } else {
                Nop(); Nop(); Nop(); Nop(); Nop();
                uint8_t r2 = ReadBEMFComp();
                Nop(); Nop(); Nop(); Nop(); Nop();
                uint8_t r3 = ReadBEMFComp();
                accept = (r1 == 0 && r2 == 0 && r3 == 0);
            }
            if (accept) {
                sched_diagRisingAcc++;
                V5_AcceptCapture(hr, 0);
            }
        }
    }
    _CCP2IF = 0;
    return;
#endif

    /* SP mode: hardware comparator IC owns ZC. Dynamic blanking in the
     * commutation ISR physically rejects the pulse turn-off edge
     * (blanking extends past amp×T + settle). A single GPIO state read
     * is the remaining discriminator — no ISR busy-wait. */
    if (v4_spActive && got && HAL_Capture_IsRisingZc() && !v4_captureValid)
    {
        uint16_t hr = (uint16_t)(ts + HAL_Capture_GetCcp2Offset());
        if ((int16_t)(hr - v4_blankingEndHR) >= 0
            && ReadBEMFComp() == 1)
        {
            v4_lastCaptureHR = hr;
            v4_captureValid = true;
            _CCP2IE = 0;
            _CCP5IE = 0;
        }
        _CCP2IF = 0;
        return;
    }

#if FEATURE_V4_MIDPOINT_ZC == 0
    if (got && HAL_Capture_IsRisingZc() && !v4_captureValid)
    {
        uint16_t hr = (uint16_t)(ts + HAL_Capture_GetCcp2Offset());
        if ((int16_t)(hr - v4_blankingEndHR) >= 0)
        {
            /* CCP2 catches FALLING comp edges, which on the inverted
             * ATA6847 = real rising-ZC (BEMF crosses neutral going UP →
             * comp 1→0). After a real rising-ZC the comp settles at 0.
             * Deglitch confirms post-edge state by reading 0; if the
             * edge was a noise spike that bounced back to 1, the check
             * fails and we reject.
             *
             * Bug fix 2026-04-16: original code checked `==1` here,
             * which rejected every real ZC and only accepted noise
             * bounces. Mode 0 R/F=0/100 + 49% Cap% confirmed the
             * inversion. With the corrected `==0` check Mode 0 should
             * capture both polarities at high rate. */
            bool accept;
            if (v4_timerPeriod < 260) {
                /* High speed: single read */
                accept = (ReadBEMFComp() == 0);
            } else {
                /* Low speed: 3-read deglitch */
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
                /* Polarity diagnostic — CCP2 fires on rising-ZC sectors. */
                v4_adcCaptureSet++;
                v4_adcSetRising++;
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

#if FEATURE_V5_SCHEDULER
    /* V5.3 path: validate falling-ZC capture, hand off to shared helper. */
    if (got && !HAL_Capture_IsRisingZc() && !v4_captureValid) {
        uint16_t hr = (uint16_t)(ts + HAL_Capture_GetCcp5Offset());
        if ((int16_t)(hr - v4_blankingEndHR) >= 0) {
            /* Falling ZC → comp settles at 1. Deglitch. */
            uint8_t r1 = ReadBEMFComp();
            bool accept;
            if (sched_Tsector < 260) {
                accept = (r1 == 1);
            } else {
                Nop(); Nop(); Nop(); Nop(); Nop();
                uint8_t r2 = ReadBEMFComp();
                Nop(); Nop(); Nop(); Nop(); Nop();
                uint8_t r3 = ReadBEMFComp();
                accept = (r1 == 1 && r2 == 1 && r3 == 1);
            }
            if (accept) {
                sched_diagFallingAcc++;
                V5_AcceptCapture(hr, 1);
            }
        }
    }
    _CCP5IF = 0;
    return;
#endif

    /* SP mode: falling-ZC hardware capture. Dynamic blanking +
     * single GPIO read. Mirror of CCP2 path. */
    if (v4_spActive && got && !HAL_Capture_IsRisingZc() && !v4_captureValid)
    {
        uint16_t hr = (uint16_t)(ts + HAL_Capture_GetCcp5Offset());
        if ((int16_t)(hr - v4_blankingEndHR) >= 0
            && ReadBEMFComp() == 0)
        {
            v4_lastCaptureHR = hr;
            v4_captureValid = true;
            _CCP2IE = 0;
            _CCP5IE = 0;
        }
        _CCP5IF = 0;
        return;
    }

#if FEATURE_V4_MIDPOINT_ZC == 0
    if (got && !HAL_Capture_IsRisingZc() && !v4_captureValid)
    {
        uint16_t hr = (uint16_t)(ts + HAL_Capture_GetCcp5Offset());
        if ((int16_t)(hr - v4_blankingEndHR) >= 0)
        {
            /* CCP5 catches RISING comp edges, which on the inverted
             * ATA6847 = real falling-ZC (BEMF crosses neutral going
             * DOWN → comp 0→1). After a real falling-ZC the comp
             * settles at 1. Deglitch confirms by reading 1.
             * (See CCP2 comment for the inversion bug fix history.) */
            bool accept;
            if (v4_timerPeriod < 260) {
                accept = (ReadBEMFComp() == 1);
            } else {
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
                /* EXPERIMENT 2026-04-16: Mode 0 with corrected deglitch
                 * captured both polarities but PI desynced because rising
                 * and falling have different relative capture timing.
                 * Bench data showed sustained 200k+ eRPM peaks but no
                 * stability. Disable falling-sector PI feeding by NOT
                 * setting v4_captureValid here — falling captures still
                 * count for diagnostics but don't drive the loop. PI sees
                 * rising-only (like proven Mode 1) but with hardware-edge
                 * precision instead of PWM-valley sample noise. */
                v4_adcCaptureSet++;       /* diagnostic only */
                /* v4_lastCaptureHR = hr;  // intentionally not set */
                /* v4_captureValid = true; // intentionally not set */
                /* _CCP2IE = 0; _CCP5IE = 0;  // keep ISRs running */
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

#endif /* FEATURE_V4_CCP_DIAG */

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

