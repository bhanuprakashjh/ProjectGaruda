/**
 * @file garuda_service.c
 * @brief Main ESC service — state machine, Timer1 + ADC + PTG ISRs.
 *
 * BEMF sampling runs in the PTG ISR (hal_ptg.c), which calls
 * V4_ProcessBemfSample here once per fire for per-fire classification.
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


/* ── ADC midpoint ZC diagnostic counters (Mode 1) ─────────────── */
/* Used to diagnose why capture rate sits at ~50% at high speed.
 * Reset on motor start in SectorPI_Start().
 * 32-bit — at 40kHz ADC rate, uint16 wraps every ~1.6s making
 * multi-second tests unusable. */
volatile uint32_t v4_adcBlankReject  = 0;  /* ADC fired pre-blanking-end */
volatile uint32_t v4_adcStateMismatch = 0; /* past blanking, wrong GPIO  */
volatile uint32_t v4_adcCaptureSet   = 0;  /* set v4_captureValid       */
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

/* V5.1 post-ZC shadow counters. Incremented in _ADCInterrupt every
 * past-blanking sample (no v4_captureValid sticky gate, so per-sample
 * rate matches what the PTG diagnostic measured). When
 * FEATURE_POST_ZC_ACCEPT=0 these stay at 0.
 *   postZcRisingAcc  — rising sector + comp == 0 (post-ZC for rising)
 *   postZcRisingRej  — rising sector + comp != 0
 *   postZcFallingAcc — falling sector + comp == 1 (post-ZC for falling)
 *   postZcFallingRej — falling sector + comp != 1
 * Expected from PTG comparison: per-polarity Acc/(Acc+Rej) ≈ 67%. */
volatile uint32_t postZcRisingAcc  = 0;
volatile uint32_t postZcRisingRej  = 0;
volatile uint32_t postZcFallingAcc = 0;
volatile uint32_t postZcFallingRej = 0;

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

    /* AK ADC uses per-channel flags. Clear AD1CH4 (VBUS) since it
     * triggered this ISR. BEMF detection runs in V4_ProcessBemfSample
     * (called from the PTG ISR), not here. */
    _AD1CH4IF = 0;
}

/* ── V4_ProcessBemfSample — runs once per PTG fire, decides ZC ─────
 *
 * Called from _PTG0Interrupt (hal_ptg.c). The `v4_adc*` counter names
 * are historical — they aren't tied to the ADC peripheral anymore.
 *
 * Decision flow:
 *   Gate 1: motor must be in CL  → else return.
 *   Gate 2: v4_captureValid already set this sector? → return (one
 *           accepted ZC per sector).
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
    if (SectorPI_IsRunning() && SectorPI_GetPhase() == 3)
    {
        if (!v4_captureValid)
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
                 * which accept logic is active. Uses ptgExpectedComp
                 * (reliable per-sector flag) not isRising (stuck-true). */
                {
                    extern volatile uint8_t ptgExpectedComp;
                    bool sectorRising_probe = (ptgExpectedComp == 0u);
                    if (sectorRising_probe) {
                        if (comp) v4_compRising_High++;
                        else      v4_compRising_Low++;
                    } else {
                        if (comp) v4_compFalling_High++;
                        else      v4_compFalling_Low++;
                    }
                }

#if FEATURE_POST_ZC_ACCEPT
                /* V5.1 shadow: count what post-ZC accept logic would do.
                 * Reads ptgExpectedComp (written by Commutate) instead
                 * of HAL_Capture_IsRisingZc() — the function call exhibits
                 * a "stuck true" behavior in ISR context that the volatile
                 * global bypasses. First run of this shadow with the
                 * function call showed p2F=0% (impossible); this read
                 * is what PTG ISR successfully used to get pR/pF ≈ 67/67.
                 * No v4_captureValid sticky gate — per-sample rate matches
                 * the PTG diagnostic for direct comparison. */
                {
                    extern volatile uint8_t ptgExpectedComp;
                    uint8_t expectedPostZc = ptgExpectedComp;
                    bool    sectorRising   = (expectedPostZc == 0u);
                    if (comp == expectedPostZc) {
                        if (sectorRising) postZcRisingAcc++;
                        else              postZcFallingAcc++;
                    } else {
                        if (sectorRising) postZcRisingRej++;
                        else              postZcFallingRej++;
                    }
                }
#endif

                /* V4 PRE-ZC detection.
                 *
                 * Polarity gate uses ptgExpectedComp (written by Commutate
                 * per sector — reliable) instead of HAL_Capture_IsRisingZc()
                 * (function call exhibits "stuck-true" behavior in ISR context,
                 * see comments at ~line 535 and CCP2 ISR for history).
                 * `isRising` is still used for state-match counting because
                 * that's tied to the inverted-ATA6847 comp polarity rules
                 * and is independent of which sector flag we trust. */
                extern volatile uint8_t ptgExpectedComp;
                bool sectorRising_v5 = (ptgExpectedComp == 0u);

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
                    }
                }
            }
        }
    }
}

/* ── V4 Commutation ISR (SCCP3 sector timer period match) ─────── */
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
    /* BEMF GPIO routing — see port_config.c for the actual
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

