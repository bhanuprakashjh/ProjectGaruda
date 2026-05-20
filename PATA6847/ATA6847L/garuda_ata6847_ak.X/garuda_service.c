/**
 * @file garuda_service.c
 * @brief Main ESC service — state machine, Timer1 + ADC + PTG ISRs.
 *
 * BEMF sampling runs in the PTG ISR (hal_ptg.c), which calls
 * ProcessBemfSample here once per fire for per-fire classification.
 * The ADC ISR retains POT/Vbus/current responsibilities only.
 *
 * Timer1 ISR (20 kHz / 50 µs):
 *   - State machine: IDLE→ARMED→ALIGN→OL_RAMP→CLOSED_LOOP
 *   - BEMF ZC timeout checking
 *   - Duty slew rate control
 *   - systemTick increment (1 ms from divide-by-20)
 *
 * ADC ISR (20 kHz, PWM-triggered):
 *   - Read pot, Vbus, phase currents
 *   - Vbus fault checking
 *
 * Target: dsPIC33AK128MC106 on EV92R69A (ATA6847L) + EV68M17A.
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
#include "motor/motor_params.h"
#include "hal/hal_com_timer.h"

/* ====================================================================
 * SECTOR PI ARCHITECTURE
 * ==================================================================== */

/* Minimal global state (sector_pi.c owns motor state) */
volatile ESC_STATE_T gEscState = ESC_IDLE;
volatile bool gStateChanged = false;
volatile ESC_STATE_T gPrevState = ESC_IDLE;
volatile uint16_t gPotRaw = 0;
volatile uint16_t gVbusRaw = 0;
volatile int16_t  gIaRaw = 0;
volatile int16_t  gIbRaw = 0;
volatile int16_t  gIbusRaw = 0;       /* direct DC-bus shunt read (OA3OUT) */

/* Calibrated electrical values — produced in the ADC ISR by applying the
 * board-specific scale factors from garuda_config.h.  These are what the
 * GSP telemetry ships to the host, so the Python tool only has to display
 * them; it never knows the shunt/gain/divider numbers itself. */
volatile int16_t  gIa_mA   = 0;       /* signed milliamps */
volatile int16_t  gIb_mA   = 0;
volatile int16_t  gIbus_mA = 0;
volatile uint16_t gVbus_mV = 0;       /* unsigned millivolts */

/* Current peak tracking (rolling window, reset on snapshot read).
 * Units: SIGNED MILLIAMPS, written by the ADC ISR after the count→mA
 * conversion.  Previously these held raw ADC counts; switched to mA so
 * the host doesn't need to know the shunt/gain calibration.  Range is
 * ±32 A in int16, well above the ±22 A ADC hardware ceiling. */
volatile int16_t  gIaPkMax   = 0, gIaPkMin   = 0;   /* mA */
volatile int16_t  gIbPkMax   = 0, gIbPkMin   = 0;   /* mA */
volatile int16_t  gIbusPkMax = 0, gIbusPkMin = 0;   /* mA */

/* At-fault frozen snapshot — populated when ESC enters FAULT, preserved
 * until motor restart. Captures exact peaks at the moment of trip.
 * Units: milliamps (same as the live peaks above). */
volatile int16_t  gIaAtFaultMax = 0, gIaAtFaultMin = 0;
volatile int16_t  gIbAtFaultMax = 0, gIbAtFaultMin = 0;
volatile int16_t  gIbusAtFaultMax = 0, gIbusAtFaultMin = 0;
volatile int16_t  gIaAtFaultInst = 0, gIbAtFaultInst = 0, gIbusAtFaultInst = 0;
volatile uint8_t  gFaultSnapshotValid = 0;

static inline void FreezeAtFaultPeaks(void)
{
    gIaAtFaultMax   = gIaPkMax;   gIaAtFaultMin   = gIaPkMin;
    gIbAtFaultMax   = gIbPkMax;   gIbAtFaultMin   = gIbPkMin;
    gIbusAtFaultMax = gIbusPkMax; gIbusAtFaultMin = gIbusPkMin;
    gIaAtFaultInst   = gIa_mA;
    gIbAtFaultInst   = gIb_mA;
    gIbusAtFaultInst = gIbus_mA;
    gFaultSnapshotValid = 1;
}
static volatile uint8_t  gTickDiv = 0;
volatile uint32_t gSystemTick = 0;

/* ATA6847 fault check interval */
static volatile uint8_t gAtaCheckDiv = 0;
static volatile uint32_t gStartTick = 0;  /* systemTick when motor started */

void GarudaService_Init(void)
{
    gEscState = ESC_IDLE;
    SectorPI_Init();
    HAL_Timer1_Start();
}

void GarudaService_StartMotor(void)
{
    if (gEscState != ESC_IDLE) return;

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
        gEscState = ESC_FAULT;
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
    SectorPI_Start(gVbusRaw);

    gEscState = ESC_OL_RAMP;  /* V3-style state during ramp */
    gStateChanged = true;
    gPrevState = ESC_IDLE;
    gAtaCheckDiv = 0;
    gStartTick = gSystemTick;
}

void GarudaService_StopMotor(void)
{
    SectorPI_Stop();
    HAL_PWM_DisableOutputs();
    HAL_ADC_InterruptDisable();
    HAL_OPA_Disable();
    HAL_ATA6847_EnterGduStandby();
    gEscState = ESC_IDLE;
    gStateChanged = true;
    LED_RUN = 0;
}

/* ── Timer1 ISR ────────────────────────────────────────────────── */
void __attribute__((interrupt, auto_psv)) _T1Interrupt(void)
{
    _T1IF = 0;

    /* OL ramp: Timer1 drives commutation at 20kHz during ALIGN+OL_RAMP.
     * This is the V3-proven approach. Every 50µs tick, the ramp countdown
     * decrements. When it hits 0, the next commutation step fires. */
    if (SectorPI_IsRunning())
        SectorPI_OlTick();

    /* 1ms tick (divide 20kHz by 20) */
    if (++gTickDiv >= 20)
    {
        gTickDiv = 0;
        gSystemTick++;

        if (SectorPI_IsRunning())
        {
            SectorPI_TimeTick();

            /* Throttle — scale 12-bit pot (0..4095) to Q15 amplitude
             * (0..32768). Original code did `>> 1` which capped at 2047
             * — always below MIN_AMPLITUDE=5000 floor, so pot had no
             * effect. `<< 3` maps potRaw=0..4095 to amp=0..32760, full
             * Q15 range. Dead-zone at potRaw < 200 (~5% pot) → amp=0 →
             * SectorPI_CommandSet floors to MIN_AMPLITUDE (15% idle). */
            uint16_t amp = (gPotRaw < 200u) ? 0u
                                              : (uint16_t)(gPotRaw << 3);
            SectorPI_CommandSet(amp);

            /* Track phase transitions for telemetry state */
            if (gEscState == ESC_OL_RAMP && SectorPI_GetPhase() == 3)
            {
                gEscState = ESC_CLOSED_LOOP;
                gStartTick = gSystemTick;
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
static inline uint8_t ReadBEMFComp(uint8_t fp);
extern volatile uint8_t floatingPhase;
extern volatile uint16_t blankingEndHR;
extern volatile uint16_t timerPeriod_g;  /* from sector_pi.c */


/* ── ADC midpoint ZC diagnostic counters (Mode 1) ─────────────── */
/* Used to diagnose why capture rate sits at ~50% at high speed.
 * Reset on motor start in SectorPI_Start().
 * 32-bit — at 40kHz ADC rate, uint16 wraps every ~1.6s making
 * multi-second tests unusable. */
volatile uint32_t adcBlankReject  = 0;  /* ADC fired pre-blanking-end */
volatile uint32_t adcStateMismatch = 0; /* past blanking, wrong GPIO  */
volatile uint32_t adcCaptureSet   = 0;  /* set captureValid       */

/* Capture-layer probe — falling-sector counters only. compFalling_Low =
 * comp=0 at sample (PRE-ZC on inverted ATA6847), High = comp=1 (POST-ZC).
 * The rising-sector counterparts were always ~100% Low / 0% High on
 * this hardware (sample lands POST-ZC of rising) — pure noise, removed. */
volatile uint32_t compFalling_High = 0;  /* falling sector, comp=1 (post-ZC state) */
volatile uint32_t compFalling_Low  = 0;  /* falling sector, comp=0 (pre-ZC state)  */

/* Post-ZC shadow counters — per-sample (no captureValid sticky gate),
 * gated by FEATURE_POST_ZC_ACCEPT. Diagnostic only, doesn't drive
 * commutation.
 *   postZcRisingAcc/Rej  — rising sector, comp == / != 0
 *   postZcFallingAcc/Rej — falling sector, comp == / != 1 */
volatile uint32_t postZcRisingAcc  = 0;
volatile uint32_t postZcRisingRej  = 0;
volatile uint32_t postZcFallingAcc = 0;
volatile uint32_t postZcFallingRej = 0;

/* ── ADC ISR ───────────────────────────────────────────────────── */
void __attribute__((interrupt, auto_psv)) _AD1CH4Interrupt(void)
{
    gPotRaw  = ADCBUF_POT;
    gVbusRaw = ADCBUF_VBUS;
    /* Phase + bus currents: unsigned 12-bit ADC reads, zero-current bias
     * subtracted to give a signed int16 swing around 0.  Raw counts are
     * kept for internal use; the host-facing values are converted to
     * milliamps below using the calibration constants from garuda_config.h. */
    gIaRaw   = (int16_t)((int16_t)ADCBUF_IA   - ADC_CURRENT_BIAS);
    gIbRaw   = (int16_t)((int16_t)ADCBUF_IB   - ADC_CURRENT_BIAS);
    gIbusRaw = (int16_t)((int16_t)ADCBUF_IBUS - ADC_CURRENT_BIAS);

    /* Convert ADC counts → physical units once, at the source.  All
     * downstream consumers (snapshot builder, fault snapshot, peak
     * tracker, future current limiter) read the calibrated globals so
     * that swapping the DIM gain resistors only requires updating the
     * two _Q8 constants in garuda_config.h.  Q8 fixed-point keeps the
     * multiply in int32 and the result in int16 with no FP. */
    gVbus_mV = (uint16_t)(((uint32_t)gVbusRaw * ADC_VBUS_MV_PER_COUNT_Q8) >> 8);
    gIa_mA   = (int16_t)(((int32_t)gIaRaw   * ADC_MA_PER_COUNT_Q8) >> 8);
    gIb_mA   = (int16_t)(((int32_t)gIbRaw   * ADC_MA_PER_COUNT_Q8) >> 8);
    gIbus_mA = (int16_t)(((int32_t)gIbusRaw * ADC_MA_PER_COUNT_Q8) >> 8);

    /* Peak tracking in milliamps (rolling window reset on snapshot read).
     * Direct OA3 read of the DC-bus shunt → ibusPk straight from gIbus_mA
     * instead of being host-reconstructed from |Ia|/|Ib| extrema (which
     * underestimated by √3 in C-PWM sectors). */
    if (gIa_mA   > gIaPkMax)   gIaPkMax   = gIa_mA;
    if (gIa_mA   < gIaPkMin)   gIaPkMin   = gIa_mA;
    if (gIb_mA   > gIbPkMax)   gIbPkMax   = gIb_mA;
    if (gIb_mA   < gIbPkMin)   gIbPkMin   = gIb_mA;
    if (gIbus_mA > gIbusPkMax) gIbusPkMax = gIbus_mA;
    if (gIbus_mA < gIbusPkMin) gIbusPkMin = gIbus_mA;

    /* AK ADC uses per-channel flags. Clear AD1CH4 (VBUS) since it
     * triggered this ISR. BEMF detection runs in ProcessBemfSample
     * (called from the PTG ISR), not here. */
    _AD1CH4IF = 0;
}

/* ── ProcessBemfSample — runs once per PTG fire, decides ZC ─────
 *
 * Called from _PTG0Interrupt (hal_ptg.c). The `adc*` counter names
 * are historical — they aren't tied to the ADC peripheral anymore.
 *
 * Decision flow:
 *   Gate 1: motor must be in CL  → else return.
 *   Gate 2: captureValid already set this sector? → return (one
 *           accepted ZC per sector).
 *   Gate 3: are we past blankingEndHR? → else adcBlankReject++.
 *           Blanking rejects the post-Commutate ringing window.
 *   Probe: tally per-(sector,phase) comp=1 counts in bemfTally[].
 *          Tally is reset on each telemetry snapshot send so the
 *          ratios are a fresh ~50 ms window.
 *   Deglitch: read the comparator 3 times with NOPs between (~400 ns
 *             total). Majority vote rejects single-sample bounces.
 *   Polarity gate: compare `comp` to `expected`. Expected is currently
 *                  hard-coded to 0 (catches comp=0 events). Sectors
 *                  matching → set captureValid + lastCaptureHR. Other
 *                  polarity → just count adcStateMismatch.
 *
 * The two key globals it writes:
 *   captureValid — "there's a fresh capture waiting" flag.
 *                     Consumed by SectorPI_Commutate.
 *   lastCaptureHR_g — timestamp of that capture, in SCCP4 domain.
 *
 * NOT marked inline: called from two ISRs in different .o files.
 * Out-of-line is correct. */
void ProcessBemfSample(void)
{
    /* Same scaffold the ADC ISR used to host directly.  Original
     * comments preserved verbatim because they document hard-won
     * tuning history (single-read vs 3-read regression, isRising
     * stuck-true workarounds, PI-feed polarity rationale, etc). */
    /* Stage A: gate widened from "phase == 3 (CL only)" to "phase >= 2
     * (OL_RAMP or CL)" so ProcessBemfSample writes captureValid during
     * the open-loop ramp too. sector_pi.c's OlTick harvests these to
     * populate the olCapInCorridor / olCapOutOfCorridor diag. ALIGN
     * (phase 1) and OFF (0) still short-circuit — no commutations happen
     * in those, so BEMF samples have no rotor-position meaning. */
    if (SectorPI_IsRunning() && SectorPI_GetPhase() >= 2)
    {
        if (!captureValid)
        {
            /* Atomic snapshot — single 16-bit load of all three
             * per-sector globals. Commutate (IPL 6) cannot interleave a
             * partial update with our reads after this point. */
            uint16_t snap   = sectorSnap;
            uint8_t  fp     = SECTOR_SNAP_FP(snap);
            uint8_t  exp    = SECTOR_SNAP_EXP(snap);

            uint16_t nowHR = CCP4TMRL;
            if ((int16_t)(nowHR - blankingEndHR) < 0)
            {
                adcBlankReject++;
            }
            else
            {
                /* 3-read deglitch with majority vote rejects single-
                 * sample GPIO bounce + comparator chatter near threshold
                 * (~400 ns total). Single-read regresses high-speed
                 * peak — load-bearing for noise rejection. */
                uint8_t r1 = ReadBEMFComp(fp);
                Nop(); Nop(); Nop(); Nop();
                uint8_t r2 = ReadBEMFComp(fp);
                Nop(); Nop(); Nop(); Nop();
                uint8_t r3 = ReadBEMFComp(fp);
                uint8_t comp = ((uint8_t)(r1 + r2 + r3) >= 2u) ? 1u : 0u;

                /* Capture-layer probe — falling-sector comp tally only.
                 * Feeds `postF` in telemetry. Polarity comes from the
                 * atomic sector snapshot (`exp`). */
                if (exp != 0u) {
                    if (comp) compFalling_High++;
                    else      compFalling_Low++;
                }

#if FEATURE_POST_ZC_ACCEPT
                /* Post-ZC shadow — count what the post-ZC accept logic
                 * would do if it were live. Same `exp` source as the
                 * probe above; sticky-free per-sample rate. */
                {
                    bool sectorRising = (exp == 0u);
                    if (comp == exp) {
                        if (sectorRising) postZcRisingAcc++;
                        else              postZcFallingAcc++;
                    } else {
                        if (sectorRising) postZcRisingRej++;
                        else              postZcFallingRej++;
                    }
                }
#endif

                /* PRE-ZC detection. Polarity from the same snapshot. */
                bool sectorRising_v5 = (exp == 0u);

                /* Polarity-symmetric PRE-ZC gate, matching CK reference
                 * (garuda_service.c:453: `expected = isRising ? 1 : 0`).
                 * On inverted ATA6847:
                 *   Rising sectors: BEMF crosses up → comp 1→0, so PRE-ZC = 1.
                 *   Falling sectors: BEMF crosses down → comp 0→1, so PRE-ZC = 0.
                 * Old code hardcoded expected=0 and starved the PI of
                 * rising-sector captures (preR≈100% but never accepted).
                 * That asymmetry kept the loop alive at 60 kHz but walls
                 * the motor at 40 kHz where single-polarity sample noise
                 * dominates. CK runs both polarities and is stable at
                 * 60 + 40 kHz on multiple motors. */
                uint8_t expected = sectorRising_v5 ? 1u : 0u;
                if (comp != expected)
                {
                    adcStateMismatch++;
                }
                else
                {
                    adcCaptureSet++;

                    /* Publish the capture for Commutate to consume.
                     * nowHR is the SCCP4 free-running tick at the
                     * moment the deglitch sample passed. */
                    lastCaptureHR_g = nowHR;
                    captureValid = true;
                }
            }
        }
    }
}

/* ── Commutation ISR (SCCP3 sector timer period match) ─────── */
void __attribute__((interrupt, no_auto_psv)) _CCT3Interrupt(void)
{
    /* One-shot guard: disable interrupt + push PRL to prevent
     * stray re-trigger while SectorPI_Commutate runs.
     * Commutate will call ScheduleAbsolute to arm the next one. */
    _CCT3IE = 0;
    _CCT3IF = 0;
    CCP3PRL = 0xFFFF;
    SectorPI_Commutate();
}

/* CCP ISRs drain the FIFO unconditionally — blanking lives in the
 * commutation ISR. Draining on every edge prevents FIFO overflow
 * (the 4-deep FIFO drops new captures when full). */

/* Cluster detection: a real ZC produces a burst of 2+ comparator
 * edges within ~10 HR ticks; PWM noise edges are spaced ~25µs (39 HR
 * ticks) apart. Current edge within CLUSTER_GAP of previous = ZC. */
#define CLUSTER_GAP_HR  15u   /* ~10µs — covers ZC burst, rejects PWM noise */

/* Floating phase GPIO readers */
volatile uint8_t floatingPhase = 0;

static inline uint8_t ReadBEMFComp(uint8_t fp)
{
    /* BEMF GPIO routing — see port_config.c. fp is passed in (not read
     * from the global) so the caller can use a single coherent snapshot
     * of the per-sector state without race exposure. */
    switch (fp) {
        case 0: return BEMF_A_GetValue();
        case 1: return BEMF_B_GetValue();
        case 2: return BEMF_C_GetValue();
        default: return 0;
    }
}

/* Blanking state */
volatile uint16_t blankingEndHR = 0;
/* Expected ZC HR timestamp (center of corridor for SP CCP ISR) */
volatile uint16_t expectedZcHR = 0;

/* (adc* counters declared earlier — used by ADC ISR above) */


/* ── Service tick (called from main loop) ─────────────────── */
void GarudaService_Tasks(void)
{
    /* Check for stall → restart */
    if (gEscState == ESC_CLOSED_LOOP && !SectorPI_IsRunning())
    {
        HAL_UART_WriteString("V4:STALL\r\n");
        GarudaService_StopMotor();
    }
}

void GarudaService_ClearFault(void)
{
    if (gEscState == ESC_FAULT)
    {
        gEscState = ESC_IDLE;
        LED_FAULT = 0;
    }
}

void GarudaService_MainLoop(void)
{
    if (gEscState != gPrevState)
    {
        gStateChanged = true;
        gPrevState = gEscState;
    }
    GarudaService_Tasks();
}

