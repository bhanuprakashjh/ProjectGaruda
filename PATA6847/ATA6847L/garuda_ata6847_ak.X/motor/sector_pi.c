/**
 * @file sector_pi.c
 * @brief Sector PI motor control.
 *
 * Startup: V3-style Timer1 countdown for ALIGN + OL_RAMP (proven).
 * Closed-loop: AVR-style PI synchronizer on SCCP3 + CCP2 capture.
 *
 * OL ramp is driven by Timer1 ISR calling SectorPI_OlTick() at 20kHz.
 * When ramp reaches MIN_STEP_PERIOD, SectorPI_EnterCL() starts the
 * SCCP3 sector timer and CCP2 capture. PI owns commutation from there.
 */

#include "sector_pi.h"


#include <xc.h>
#include "../garuda_config.h"
#include "../hal/hal_ak_compat.h"
#include "../hal/hal_pwm.h"
#include "../hal/hal_com_timer.h"
#include "../hal/hal_capture.h"
#include "../hal/hal_ptg.h"
#include "../hal/port_config.h"
#include "commutation.h"
#include "motor_params.h"

/* ── Sentinel for "no comparator edge this sector" ──────────────── */
#define CAP_SENTINEL    0xFFFFU

/* ── PPS lookup: floatingPhase index → RP pin number ────────────── */
static const uint8_t bemfRpPin[3] = {
    (uint8_t)BEMF_A_RP,
    (uint8_t)BEMF_B_RP,
    (uint8_t)BEMF_C_RP
};

/* ── State ──────────────────────────────────────────────────────── */
typedef enum {
    SECTOR_PHASE_OFF = 0,
    SECTOR_PHASE_ALIGN,
    SECTOR_PHASE_OL_RAMP,
    SECTOR_PHASE_CL
} SECTOR_PHASE_T;

static volatile SECTOR_PHASE_T phase;
static          uint8_t    position;        /* 0..5 */
static volatile bool       running;
static volatile uint16_t   statusEvents;
static volatile uint32_t   sectorCount;

/* Per-sector hit counter — written from BOTH Commutate paths to tell
 * whether all 6 positions actually fire. Diagnostic for "snapshot only
 * shows odd sectors" anomaly: if hits[0,2,4] stay at 0 while [1,3,5]
 * climb, position is incrementing by 2 somewhere and rising-sector
 * BEMF detection literally never runs. Exposed via GSP snapshot tail. */
volatile uint32_t sectorHits[6] = {0,0,0,0,0,0};

/* Per-sector current position mirror — written from both Commutate paths
 * to give the PTG/ProcessBemfSample probe a sector index without
 * exposing the static `position`. Read-only outside sector_pi.c. */
volatile uint8_t currentSector = 0;

/* Atomic per-sector snapshot read by the PTG ISR. Packs the same three
 * fields written individually below (currentSector, floatingPhase,
 * ptgExpectedComp) into a single uint16 so PTG can read them with one
 * 16-bit load and avoid a torn snapshot if Commutate (IPL 6) preempts
 * mid-read. Layout — see SECTOR_SNAP_* helpers in sector_pi.h:
 *   bits 0..2 = currentSector (0..5)
 *   bits 3..4 = floatingPhase (0..2)
 *   bit  5    = ptgExpectedComp (0..1) */
volatile uint16_t sectorSnap = 0;

/* ── OL ramp state (Timer1 driven, matches V3 startup.c) ───────── */
static volatile uint16_t   alignCounter;
static volatile uint16_t   rampStepPeriod;  /* Timer1 ticks per step */
static volatile uint16_t   rampCounter;     /* Timer1 countdown */
static volatile uint32_t   rampDuty;
/* OL handoff settle: after rampStepPeriod reaches MIN_STEP_PERIOD, hold
 * the motor at the forced rate for OL_HANDOFF_SETTLE_TICKS Timer1 ticks
 * (50µs each) before EnterCL. Without this, the rotor lags commanded
 * speed at handoff → BEMF stays on the wrong side of neutral all
 * sector → post-ZC edge gate never triggers → stall. */
#ifndef OL_HANDOFF_SETTLE_MS
#define OL_HANDOFF_SETTLE_MS  200U
#endif
#define OL_HANDOFF_SETTLE_TICKS \
    ((uint16_t)((uint32_t)OL_HANDOFF_SETTLE_MS * TIMER1_FREQ_HZ / 1000U))
static volatile uint16_t   olHandoffCounter;

/* ── PI state (SCCP3 driven, active in CL only) ────────────────── */
static volatile uint16_t   timerPeriod;     /* sector period (PI output) */
volatile uint16_t timerPeriod_g;           /* exposed for CCP ISR speed check */
static volatile uint16_t   integrator;
/* PI integrator floor — set in EnterCL() to the OL handoff seed value.
 * Until commandEnabled goes true (rotor has had CL_SETTLE_MS to lock),
 * the PI cannot drive timerPeriod below the seed. Reason: early-CL
 * captures arrive with capValue<setValue (rotor lags forced commutation
 * rate). The PI reads that as "rotor is ahead" → shrinks timerPeriod →
 * commutates faster → rotor falls further behind → death spiral. The
 * floor blocks this until the loop has settled. */
static volatile uint16_t   seedIntegrator;
static          uint8_t    stallCounter;
static volatile bool       commandEnabled;
/* Updated in Commutate ISR; exposed for telemetry.              */
volatile uint16_t g_pwmPer = LOOPTIME_TCY;
static volatile uint16_t   actualAmplitude;
static volatile uint16_t   targetAmplitude;
static volatile uint16_t   targetPeriod;    /* pot-commanded speed */
static volatile uint16_t   basePeriod;      /* ramped toward targetPeriod */
static volatile uint16_t   lastCommHR;      /* HR time of last commutation */
static volatile uint16_t   actualStepPeriodHR; /* measured comm-to-comm HR */

/* ── Speed measurement ──────────────────────────────────────────── */
#define SPEED_MEAS_WINDOW   20U
static volatile uint16_t   speedCounter;
static volatile uint16_t   speedBase;
static volatile uint16_t   measuredSpeed;

/* CL settle counter — declared early so SectorPI_Start can reset it.
 * MUST be reset on each motor start; otherwise it stays at CL_SETTLE_MS
 * after the first run and the next CL entry skips the settle window. */
#define CL_SETTLE_MS  500U
static volatile uint16_t clSettleCounter = 0;

/* Measurement-based period tracker.
 *   tMeasHR       — raw halved measuredCommPeriod, written by Commutate ISR.
 *                   Fast path: single write, no branches. Noisy.
 *   tMeasHRSmooth — spike-filtered + EMA smoothed, written by TimeTick
 *                   (1 kHz rate where cycles are cheap). Shipped in
 *                   telemetry and used by MEAS_PI_OWN.
 * Defined unconditionally so telemetry stays valid when MEAS_PI is off. */
volatile uint16_t tMeasHR       = 0;
volatile uint16_t tMeasHRSmooth = 0;

/* ── Diagnostics ────────────────────────────────────────────────── */
volatile uint16_t diagCaptures;
volatile uint16_t diagPiRuns;
volatile uint16_t diagLastCapValue;
volatile int16_t  diagDelta;
/* Commutate ISR: capValue == SENTINEL (no valid capture this sector) */
volatile uint32_t diagCommutateNoCapture;

/* Per-polarity PI-feed accounting. Counted at the PI-feed point (after
 * plausibility filter) so the GUI can show which sector polarity is
 * actually contributing captures vs falling through to scheduler-only. */
volatile uint32_t diagPiFedRising   = 0;
volatile uint32_t diagPiFedFalling  = 0;
volatile uint32_t diagPiMissRising  = 0;  /* sector was rising, no PI feed */
volatile uint32_t diagPiMissFalling = 0;  /* sector was falling, no PI feed */

/* Per-polarity elapsed snapshots for the GUI mechanism-probe. Used to
 * diagnose why one polarity's captures get filtered out:
 *   - rejectRising ≈ filterHRLast + small : sector ran slightly long
 *   - rejectRising near 65535             : capture is stale (uncleared)
 *   - rejectRising ≈ 0..tiny              : capture fired at the
 *                                           commutate boundary, before
 *                                           this sector's BEMF settled */
volatile uint16_t diagElapsedAcceptRising  = 0;
volatile uint16_t diagElapsedAcceptFalling = 0;
volatile uint16_t diagElapsedRejectRising  = 0;
volatile uint16_t diagElapsedRejectFalling = 0;
volatile uint16_t diagFilterHRLast         = 0;

/* Corridor good-streak: PI only runs after 12 consecutive
 * sectors with a valid corridor capture. */
#define CORRIDOR_GOOD_THRESHOLD  12
static uint8_t corridorGoodStreak = 0;
static uint8_t corridorMissStreak = 0;     /* consecutive sentinels — block-comm exit */
static bool piEnabled = false;        /* PI error: capValue - setValue */


/* Stability counter for block-comm entry — incremented every TimeTick
 * (1 kHz) while eRPM ≥ 150k. Cleared when speed drops. */
static uint16_t blockCommStableTicks = 0;

/* Block-comm exit cooldown — counts speed-windows (20 ms each).
 * After exit, prevents re-entry until the motor settles on PWM,
 * which kills the BLK↔PWM thrashing seen on small pot jitter at
 * the entry threshold (each toggle is a 100%↔88% drive perturbation
 * that disturbs the BEMF measurement and can cascade into desync). */
static uint16_t blockExitCooldown = 0;

/* PI capture log — snapshots the first PI_LOG_ENTRIES iterations
 * after CL entry. Frozen once full, re-armed by EnterCL, read via GSP.
 * Used when runaway/desync happens too fast for streamed telemetry. */
#define PI_LOG_ENTRIES   30U
typedef struct __attribute__((packed)) {
    uint16_t timerPeriod;   /* before PI update */
    uint16_t setValue;
    uint16_t capValue;
    int16_t  deltaClamped;
} PI_LOG_ENTRY_T;

static volatile PI_LOG_ENTRY_T piLog[PI_LOG_ENTRIES];
static volatile uint8_t        piLogCount = 0;

/* ── Helpers ────────────────────────────────────────────────────── */

static inline uint16_t PhaseAdvance(uint16_t fp8, uint16_t period)
{
    return (uint16_t)(((uint32_t)fp8 * period) >> 8);
}

static inline uint16_t FixpMulU16(uint16_t a, uint16_t b)
{
    return (uint16_t)(((uint32_t)a * (uint32_t)b) >> 15);
}

static void ShutOff(void)
{
    HAL_Capture_Stop();
    HAL_PTG_Stop();
    HAL_ComTimer_Cancel();
    HAL_PWM_ForceAllFloat();
    HAL_PWM_SetDutyCycle(0);
    running = false;
    phase = SECTOR_PHASE_OFF;
    commandEnabled = false;
    actualAmplitude = 0;
    position = 0;
    speedCounter = 0;
    speedBase = 0;
    measuredSpeed = 0;
}

/* ── Enter closed-loop: start SCCP3 sector timer + CCP2 capture ── */
static void EnterCL(void)
{
    /* Seed PI from current ramp speed.
     * Convert Timer1 step period to SCCP3 ticks (640ns).
     * T1 tick = 50µs = 78.125 SCCP ticks. */
    uint32_t hrPeriod = (uint32_t)rampStepPeriod * 625UL / 8UL;
    if (hrPeriod > 0xFFFF) hrPeriod = 0xFFFF;
    if (hrPeriod < escParams.minPeriodHr) hrPeriod = escParams.minPeriodHr;

    timerPeriod = (uint16_t)hrPeriod;
    integrator  = timerPeriod;
    seedIntegrator = timerPeriod;   /* floor during early CL */
    stallCounter = 0;
    commandEnabled = false;
    diagCaptures = 0;
    diagPiRuns = 0;
    diagLastCapValue = 0xFFFF;
    corridorGoodStreak = 0;
    corridorMissStreak = 0;
    piEnabled = false;
    piLogCount = 0;

    targetPeriod = timerPeriod;
    basePeriod = timerPeriod;
    actualStepPeriodHR = timerPeriod;

    /* Keep current duty from ramp */
    actualAmplitude = (uint16_t)((rampDuty * 32768UL) / LOOPTIME_TCY);
    targetAmplitude = actualAmplitude;

    /* Configure CCP2 for current floating phase.
     * Flush FIFOs and clear flags BEFORE enabling ISRs to prevent
     * stale noise edges from setting captureValid. */
    const COMMUTATION_STEP_T *step = &commutationTable[position];
    HAL_Capture_Configure(bemfRpPin[step->floatingPhase],
                          step->zcPolarity > 0);
    HAL_Capture_SetBlanking(timerPeriod);

    /* Flush any accumulated edges before enabling capture ISRs */
    while (CCP2STATbits.ICBNE) (void)CCP2BUF;
    _CCP2IF = 0;
    captureValid = false;

    HAL_Capture_Start();
    HAL_PTG_Start();

    /* Seed lastCommHR for PI elapsed calculation */
    lastCommHR = HAL_ComTimer_ReadTimer();

    /* Seed blankingEndHR so the ADC ISR's blanking gate is sane
     * during the first CL sector — before Commutate has a chance to
     * write it.  Without this, a stale 0 from init lets every ADC
     * sample past the gate. */
    {
        extern volatile uint16_t blankingEndHR;
        uint8_t  pct      = escParams.blankingPct;
        uint16_t blankHR;
        if (pct == 0U || pct > 100U) {
            blankHR = timerPeriod >> 2;
        } else {
            blankHR = (uint16_t)(((uint32_t)timerPeriod * pct) / 100U);
        }
        blankingEndHR = (uint16_t)(lastCommHR + blankHR);
    }


    /* Schedule first CL commutation via one-shot SCCP3 */
    HAL_ComTimer_ScheduleAbsolute(lastCommHR + timerPeriod);

    phase = SECTOR_PHASE_CL;

    /* Enable throttle after a short delay (handled in TimeTick) */
}

/* ────────────────────────────────────────────────────────────────── */
/* PUBLIC API                                                        */
/* ────────────────────────────────────────────────────────────────── */

void SectorPI_Init(void)
{
    /* Load runtime tunables from compile-time defaults BEFORE the HAL
     * inits run — HAL_Capture_SetBlanking and the PI loop both consume
     * escParams on first use. */
    Params_InitDefaults();

    HAL_ComTimer_Init();   /* SCCP3 one-shot + SCCP4 HR (proven V3 pattern) */
    HAL_Capture_Init();
    HAL_PTG_Init();
    running = false;
    phase = SECTOR_PHASE_OFF;
    statusEvents = 0;
    sectorCount = 0;
}

void SectorPI_Start(uint16_t vbusRaw)
{
    /* Defensive: if ShutOff was missed or running flag stuck, force a
     * clean stop before re-arming. Otherwise this no-ops and the user's
     * SW1 press is silently ignored. */
    if (running) ShutOff();
    (void)vbusRaw;

    /* Reset all carry-over static state so a previous run's residue
     * (especially clSettleCounter, which would otherwise let the PI
     * take pot input on the very first CL TimeTick) doesn't break the
     * fresh start. */
    clSettleCounter = 0;

    position = 0;
    running = true;
    statusEvents = 0;
    sectorCount = 0;
    speedCounter = 0;
    speedBase = 0;
    measuredSpeed = 0;

    /* Reset capture-rate diagnostic counters (defined in garuda_service.c
     * for the ADC ISR side, sector_pi.c for the Commutate side). Lets us
     * read fresh ratios per run instead of accumulating across restarts. */
    diagCaptures = 0;
    diagPiRuns = 0;
    diagCommutateNoCapture = 0;
    extern volatile uint32_t adcBlankReject;
    extern volatile uint32_t adcStateMismatch;
    extern volatile uint32_t adcCaptureSet;
    extern volatile uint32_t adcSetRising;
    adcBlankReject = 0;
    adcStateMismatch = 0;
    adcCaptureSet = 0;
    adcSetRising = 0;
    extern volatile uint32_t postZcRisingAcc;
    extern volatile uint32_t postZcRisingRej;
    extern volatile uint32_t postZcFallingAcc;
    extern volatile uint32_t postZcFallingRej;
    postZcRisingAcc  = 0;
    postZcRisingRej  = 0;
    postZcFallingAcc = 0;
    postZcFallingRej = 0;
    /* Reset measurement tracker to unseeded. Raw value seeds on the
     * first valid commutation interval in Commutate; smoothed value
     * seeds from raw in TimeTick. */
    tMeasHR       = 0;
    tMeasHRSmooth = 0;

    /* Start alignment — Timer1 ISR drives via SectorPI_OlTick() */
    alignCounter = ALIGN_TIME_COUNTS;
    rampStepPeriod = INITIAL_STEP_PERIOD;
    rampCounter = INITIAL_STEP_PERIOD;
    rampDuty = ALIGN_DUTY;
    olHandoffCounter = OL_HANDOFF_SETTLE_TICKS;

    HAL_PWM_SetCommutationStep(0);
    HAL_PWM_SetDutyCycle(MIN_DUTY);

    phase = SECTOR_PHASE_ALIGN;
}

void SectorPI_Stop(void)
{
    ShutOff();
}

/* ── Called from Timer1 ISR at 20kHz (50µs) during ALIGN + OL_RAMP ── */
void SectorPI_OlTick(void)
{
    if (!running) return;

    if (phase == SECTOR_PHASE_ALIGN)
    {
        if (alignCounter > 0)
        {
            alignCounter--;
            /* Gradual duty ramp during alignment */
            uint32_t duty;
            if (ALIGN_DUTY > MIN_DUTY)
            {
                uint32_t elapsed = ALIGN_TIME_COUNTS - alignCounter;
                duty = MIN_DUTY +
                    ((uint32_t)(ALIGN_DUTY - MIN_DUTY) * elapsed) / ALIGN_TIME_COUNTS;
            }
            else
            {
                duty = ALIGN_DUTY;
            }
            rampDuty = duty;
            HAL_PWM_SetDutyCycle(duty);
        }
        else
        {
            /* Alignment done → start OL ramp */
            phase = SECTOR_PHASE_OL_RAMP;
        }
        return;
    }

    if (phase == SECTOR_PHASE_OL_RAMP)
    {
        if (rampCounter > 0)
            rampCounter--;

        if (rampCounter == 0)
        {
            /* Advance to next commutation step */
            position++;
            if (position > 5) position = 0;
            HAL_PWM_SetCommutationStep(position);

            /* Compute new step period (V3 acceleration formula) */
            #define ERPM_CONST ((uint32_t)TIMER1_FREQ_HZ * 10UL)
            uint32_t curPeriod = rampStepPeriod;
            if (curPeriod == 0) curPeriod = INITIAL_STEP_PERIOD;
            uint32_t curErpm = ERPM_CONST / curPeriod;
            uint32_t deltaErpm = ((uint32_t)RAMP_ACCEL_ERPM_S * curPeriod)
                                 / TIMER1_FREQ_HZ;
            if (deltaErpm < 1) deltaErpm = 1;
            uint32_t newErpm = curErpm + deltaErpm;
            uint32_t newPeriod = ERPM_CONST / newErpm;
            if (newPeriod < MIN_STEP_PERIOD)
                newPeriod = MIN_STEP_PERIOD;
            rampStepPeriod = (uint16_t)newPeriod;
            rampCounter = rampStepPeriod;

            /* Gradually increase duty toward RAMP_DUTY_CAP */
            if (rampDuty < RAMP_DUTY_CAP)
            {
                rampDuty += (LOOPTIME_TCY / 200);
                if (rampDuty > RAMP_DUTY_CAP)
                    rampDuty = RAMP_DUTY_CAP;
            }
            HAL_PWM_SetDutyCycle(rampDuty);

            /* Speed counting for telemetry */
            speedCounter++;
            sectorCount++;
        }

        /* Ramp complete → transition to closed-loop.
         *
         * Settle window: once rampStepPeriod reaches MIN_STEP_PERIOD,
         * keep firing at that forced rate for OL_HANDOFF_SETTLE_TICKS so
         * the rotor (which lags the commanded rate during the ramp) has
         * time to physically reach the target speed. Entering CL before
         * the rotor catches up leaves the PI seeded with a timerPeriod
         * faster than the rotor — every Commutate fires before BEMF can
         * cross neutral and stallCounter trips out in ~1s. Decremented
         * in Timer1 (1 tick per OlTick) only after the speed target is
         * reached. */
        if (rampStepPeriod <= MIN_STEP_PERIOD)
        {
            if (olHandoffCounter > 0u)
            {
                olHandoffCounter--;
            }
            else
            {
                EnterCL();
            }
        }
        return;
    }

    /* CL phase: CCP ISRs handle FIFO drain continuously */
}


/* ── Called from SCCP3 ISR (sector timer match) — CL only ───────── */
void SectorPI_Commutate(void)
{
    if (phase != SECTOR_PHASE_CL) return;

    /* 1. Disable CCP ISRs during critical section */
    _CCP2IE = 0;

    /* 2. Consume best corridor candidate from previous sector */
    uint16_t thisCommHR = HAL_ComTimer_ReadTimer();
    uint16_t prevCommHR = lastCommHR;

    /* 2.0 Close the cross-sector ADC race.  The ADC ISR (40 kHz, not
     * disabled here) can fire between `captureValid = false` below
     * and the new-sector blanking write further down.  With OLD
     * blankingEnd in the past, its `(now - blankingEnd) >= 0` gate
     * always passes → it captures the start of the new sector and
     * the next Commutate sees a tiny capValue.
     *
     * Move the new blankingEnd into place *now*, before clearing
     * captureValid, so the gate rejects every ADC sample until real
     * blanking expires.  Width re-uses actualStepPeriodHR (sector
     * width is independent of position parity). */
    {
        extern volatile uint16_t blankingEndHR;
        uint16_t guardSectorHR = (actualStepPeriodHR >= MIN_PERIOD_HR)
                                 ? actualStepPeriodHR : timerPeriod;
        uint8_t  guardPct      = escParams.blankingPct;
        uint16_t guardBlankHR;
        if (guardPct == 0U || guardPct > 100U) {
            guardBlankHR = guardSectorHR >> 2;
        } else {
            guardBlankHR = (uint16_t)(((uint32_t)guardSectorHR * guardPct) / 100U);
        }
        blankingEndHR = (uint16_t)(thisCommHR + guardBlankHR);
    }
    uint16_t measuredCommPeriod = (uint16_t)(thisCommHR - prevCommHR);
    if (measuredCommPeriod >= MIN_PERIOD_HR)
    {
        actualStepPeriodHR = measuredCommPeriod;
#if FEATURE_MEAS_PI
        /* Halve measuredCommPeriod — raw reads 2× the real sector
         * period (root cause unresolved). Single write only: extra
         * ISR cycles here destabilize Commutate timing. */
        tMeasHR = (uint16_t)(measuredCommPeriod >> 1);
        if (tMeasHR < MIN_PERIOD_HR) tMeasHR = MIN_PERIOD_HR;
#endif
    }
    uint16_t capValue = CAP_SENTINEL;

    /* Per-polarity PI-feed accounting. position holds the JUST-CONSUMED
     * sector index (incremented after this block). Even = rising-BEMF,
     * odd = falling. */
    bool sectorWasRising = ((position & 1u) == 0u);

    if (captureValid)
    {
        uint16_t elapsed = (uint16_t)(lastCaptureHR_g - prevCommHR);
        /* Filter against the actual measured rotor sector, not the PI
         * estimate. timerPeriod can settle ~2× T_rotor, which lets cross-
         * sector captures pass and locks the system into an every-other-
         * sector miss pattern. Use the truth: actualStepPeriodHR. Fall
         * back to timerPeriod only on the very first commutation when
         * actualStepPeriodHR isn't initialized yet. */
        uint16_t filterHR = (actualStepPeriodHR >= MIN_PERIOD_HR)
                            ? actualStepPeriodHR : timerPeriod;
        diagFilterHRLast = filterHR;
        if (elapsed < filterHR)
        {
            capValue = elapsed;
            diagCaptures++;
            if (sectorWasRising) { diagPiFedRising++;  diagElapsedAcceptRising  = elapsed; }
            else                 { diagPiFedFalling++; diagElapsedAcceptFalling = elapsed; }
        }
        else
        {
            diagCommutateNoCapture++;  /* capture present but past filter */
            if (sectorWasRising) { diagPiMissRising++;  diagElapsedRejectRising  = elapsed; }
            else                 { diagPiMissFalling++; diagElapsedRejectFalling = elapsed; }
        }
        captureValid = false;
    }
    else
    {
        diagCommutateNoCapture++;      /* no capture at all this sector */
        if (sectorWasRising) diagPiMissRising++;
        else                 diagPiMissFalling++;
    }
    lastCommHR = thisCommHR;

    /* 3. Advance to next commutation step.
     *
     * INVARIANT: the three per-sector globals
     *   currentSector  — software sector index 0..5
     *   floatingPhase  — which BEMF GPIO PTG should read (0=A 1=B 2=C)
     *   ptgExpectedComp   — post-ZC comp state (0=rising, 1=falling)
     * MUST be written back-to-back inside this block.
     *
     * Commutate runs at IPL 6, PTG at IPL 5 — PTG cannot preempt
     * Commutate, but Commutate can preempt PTG mid-flight. PTG reads
     * these globals across several statements. If Commutate's writes
     * were spread out, PTG could see a partial update (e.g. old sector
     * index, new expected-comp) and misbin the sample.
     *
     * If you add a new per-sector global written by Commutate and read
     * by PTG, write it inside this same block. */
    position++;
    if (position > 5) position = 0;
    sectorHits[position]++;
    {
        extern volatile uint8_t floatingPhase;
#if FEATURE_POST_ZC_ACCEPT
        extern volatile uint8_t ptgExpectedComp;
#endif
        /* Compute new values BEFORE writing any volatile global.
         * If PTG preempts mid-compute we don't care — none of the
         * globals have been updated yet, PTG sees the previous
         * sector consistently. */
        uint8_t newFp       = commutationTable[position].floatingPhase;
        uint8_t newExpected = (uint8_t)(position & 1u);
        uint16_t newSnap    = (uint16_t)(position
                                       | ((uint16_t)newFp       << 3)
                                       | ((uint16_t)newExpected << 5));
        /* Publish all four. The single 16-bit sectorSnap write is the
         * atomic one PTG actually reads. The three byte-wide globals
         * are retained for callers that legitimately need just one
         * (e.g. ReadBEMFComp's static-inline switch). */
        currentSector  = position;
        floatingPhase  = newFp;
#if FEATURE_POST_ZC_ACCEPT
        ptgExpectedComp = newExpected;
#endif
        sectorSnap     = newSnap;
    }
    HAL_PWM_SetCommutationStep(position);

    /* 4. Configure CCP for new phase, flush FIFOs */
    {
        const COMMUTATION_STEP_T *step = &commutationTable[position];
        HAL_Capture_Configure(bemfRpPin[step->floatingPhase],
                              step->zcPolarity > 0);
        while (CCP2STATbits.ICBNE) (void)CCP2BUF;
        _CCP2IF = 0;
    }

    /* 5. Set blanking + expected ZC for CCP ISR.
     * floatingPhase is written atomically in step-3 above. */
    {
        extern volatile uint16_t blankingEndHR;
        extern volatile uint16_t expectedZcHR;

        /* Blanking: 25% of measured rotor sector. Always use
         * actualStepPeriodHR (truth) instead of timerPeriod (PI estimate
         * which can settle at 2× T_rotor and reject valid early ZCs).
         * Fall back to timerPeriod only on first commutation before
         * actualStepPeriodHR is initialized.
         * In SP mode, extend past pulse turn-off + 10µs settle. */
        uint16_t sectorHR = (actualStepPeriodHR >= MIN_PERIOD_HR)
                            ? actualStepPeriodHR : timerPeriod;
        /* Runtime blanking — read escParams.blankingPct (GUI-tunable
         * via SET_PARAM 0xF3). Fallback to 25% if pct is invalid. */
        uint8_t  blankPct = escParams.blankingPct;
        uint16_t blankHR;
        if (blankPct == 0U || blankPct > 100U) {
            blankHR = sectorHR >> 2;   /* safe fallback */
        } else {
            blankHR = (uint16_t)(((uint32_t)sectorHR * blankPct) / 100U);
        }
        blankingEndHR = (uint16_t)(thisCommHR + blankHR);

        /* Expected ZC = prev_comm + T/2 + advance (physical ZC position).
         * Matches the reactive scheduler: targetHR = ZC + (T/2 - advance),
         * so ZC = target - T/2 + advance = this_comm + T/2 + advance.
         * tal selection mirrors the reactive path (3 at high speed). */
        uint16_t halfHR_exp = sectorHR >> 1;
        uint16_t tal_exp    = (sectorHR < 260) ? 3U : 2U;
        uint16_t advHR_exp  = (uint16_t)((sectorHR >> 3) * tal_exp);
        expectedZcHR = (uint16_t)(thisCommHR + halfHR_exp + advHR_exp);

    }

    /* 6. Re-enable CCP ISRs. */
    _CCP2IE = 1;

    /* Track good-capture streak for diagnostics, and consecutive-miss
     * streak for block-commutation exit (V4 captures only rising-edge
     * sectors, so cap rate is naturally ~50% at steady state — counting
     * misses-in-a-row is a more meaningful "still locked" indicator
     * than counting consecutive hits). */
    if (capValue != CAP_SENTINEL)
    {
        if (corridorGoodStreak < 255) corridorGoodStreak++;
        corridorMissStreak = 0;
    }
    else
    {
        corridorGoodStreak = 0;
        if (corridorMissStreak < 255) corridorMissStreak++;
    }

    /* Reactive scheduling — IIR + reactive from first valid capture.
     * Safety against rapid-fire is the one-shot guard in CCT3 ISR
     * (disables CCT3IE + resets PRL before calling Commutate).
     * Pot controls duty. Speed follows motor naturally. */
    diagLastCapValue = capValue;
    if (capValue != CAP_SENTINEL)
    {
        diagPiRuns++;

        /* AVR-style set-point PI synchronizer (matches motor.c:441-450).
         *   setValue = (advance + 30°) × T / 60° + RC_DELAY  (expected ZC pos)
         *   delta    = capValue - setValue                   (signed phase error)
         *   integrator += delta >> Ki_SHIFT                  (Ki = 1/16)
         *   timerPeriod = integrator + (delta >> Kp_SHIFT)   (Kp = 1/4)
         *
         * Replaces the IIR ratio formula `timerPeriod = (3*old + 1.5*cap)/4`
         * which had a stable 3-cycle limit attractor instead of a single
         * fixed-point equilibrium (CSV evidence: timerPeriod cycled
         * 224→270→253 indefinitely at 120k eRPM). Set-point PI converges
         * to a single value, removing the limit-cycle jitter that was
         * destabilizing high-speed operation. */
        {
            /* Snapshot runtime params once per cycle. They're written by
             * the GSP set-param path on a low-priority code path; reading
             * to locals here keeps the ISR consistent within a cycle even
             * if the GUI changes a value mid-update. */
            uint16_t advFp8     = escDerived.advancePlus30Fp8;
            uint8_t  kpShift    = escParams.piKpShift;
            uint8_t  kiShift    = escParams.piKiShift;
            uint16_t minPeriod  = escParams.minPeriodHr;

            uint16_t setValue = (uint16_t)(((uint32_t)advFp8 * timerPeriod) >> 8)
                                + RC_DELAY_HR;
            int32_t delta = (int32_t)capValue - (int32_t)setValue;

            /* Per-capture delta clamp at ±25% of timerPeriod prevents
             * runaway from a single noisy capture (first post-handoff
             * captures arrive at unpredictable timestamps under prop
             * load; unclamped PI can shrink timerPeriod into phantom
             * 700k+ eRPM). Steady-state deltas are <5% so the clamp is
             * invisible in normal operation. */
            int32_t deltaCap = (int32_t)timerPeriod >> 2;
            if (delta >  deltaCap) delta =  deltaCap;
            if (delta < -deltaCap) delta = -deltaCap;

            /* Diagnostic snapshot — first PI_LOG_ENTRIES PI runs after CL.
             * timerPeriod is the value PI is operating ON (pre-update).
             * delta is post-clamp (what PI actually applies). */
            if (piLogCount < PI_LOG_ENTRIES) {
                uint8_t i = piLogCount;
                piLog[i].timerPeriod  = timerPeriod;
                piLog[i].setValue     = setValue;
                piLog[i].capValue     = capValue;
                piLog[i].deltaClamped = (int16_t)delta;
                piLogCount = (uint8_t)(i + 1U);
            }

            /* Seed floor during CL_SETTLE — gives the PI 500ms to
             * stabilize at handoff before letting it shrink Tp freely.
             * Without this, PI can shrink Tp to minPeriod in a few
             * captures at CL entry and blow stallCounter in one tick.
             * Floor lifts when commandEnabled goes true. */
            int32_t newInt = (int32_t)integrator + (delta >> kiShift);
            if (newInt < minPeriod) newInt = minPeriod;
            if (!commandEnabled && newInt < (int32_t)seedIntegrator)
                newInt = (int32_t)seedIntegrator;
            if (newInt > 0xFFFF) newInt = 0xFFFF;
            integrator = (uint16_t)newInt;

            int32_t newPer = (int32_t)integrator + (delta >> kpShift);
            if (newPer < minPeriod) newPer = minPeriod;
            if (!commandEnabled && newPer < (int32_t)seedIntegrator)
                newPer = (int32_t)seedIntegrator;
            if (newPer > 0xFFFF) newPer = 0xFFFF;
            timerPeriod = (uint16_t)newPer;
            timerPeriod_g = timerPeriod;  /* expose for CCP ISR */
        }

        /* Reactive target: ZC timestamp + delay-to-next-commutation.
         * PI's setValue stays at fixed advance (its own model);
         * scheduler uses speed-adaptive TAL ramp below.
         *
         * Unifying PI and scheduler advance (using advancePlus30Fp8
         * for both) walls the motor at 130k. Predicted scheduling
         * anchored to thisCommHR with >30° advance regresses peak
         * because timerPeriod saturates at minPeriodHr. Keep separate. */
        uint16_t schedPeriod = timerPeriod;
        uint16_t halfHR = schedPeriod >> 1;
        uint16_t tal;
        if      (schedPeriod >= 260U) tal = 2U;     /* ≤60k:    15° */
        else if (schedPeriod >= 130U) tal = 3U;     /* 60-120k: 22.5° */
        else if (schedPeriod >= 100U) tal = 4U;     /* 120-156k:30° */
        else                          tal = 5U;     /* >156k:   37.5° */
        uint16_t advHR  = (uint16_t)((schedPeriod >> 3) * tal);
        uint16_t delayHR = (halfHR > advHR) ? (uint16_t)(halfHR - advHR) : 2U;
        uint16_t targetHR = (uint16_t)(lastCaptureHR_g + delayHR);

        diagDelta = (int16_t)(targetHR - thisCommHR);  /* margin */
        HAL_ComTimer_ScheduleAbsolute(targetHR);
    }
    else
    {
        /* No ZC this sector — forced commutation at the measured sector
         * period in SP. `timerPeriod` is a detector model in normal mode
         * and can be ~2x actual sector near 70k, which immediately
         * desyncs SP if used as the fallback period. */
        uint16_t schedPeriod = timerPeriod;
        diagDelta = 0;
        HAL_ComTimer_ScheduleAbsolute(thisCommHR + schedPeriod);
    }

    /* 7. Stall detection.
     * Weighted decrement (-2 on capture, +1 on miss) tolerates ~33%
     * capture-rate floor. With piFeedPolarity=1 the PI only sees
     * rising sectors, so per-commutation capture rate caps at 50%
     * even at 100% rising hit rate — weighting keeps the counter
     * balanced at this structural rate. */
    if (commandEnabled)
    {
        if (capValue == CAP_SENTINEL)
        {
            stallCounter++;
            if (stallCounter > STALL_THRESHOLD)
            {
                ShutOff();
                statusEvents |= EVENT_STALL;
            }
        }
        else
        {
            if (stallCounter >= 2) stallCounter -= 2;
            else                   stallCounter = 0;
        }
    }

    /* 6. PWM duty — AVR-style per-sector scaling.
     * Normal mode: per = LOOPTIME_TCY (fixed 40kHz carrier).
     * SP mode: per = timerPeriod << 7 (HR→PWM ticks). MPER stays at
     * 0xFFFF, but the duty compare is bounded by `per` so the ON pulse
     * never runs longer than the sector duration. On this dsPIC PWM
     * setup MPER=0xFFFF is much longer than one electrical sector, and
     * the active phase changes every commutation, so force SOC every
     * SP sector. Otherwise the newly active phase can wait for the long
     * master frame before producing torque. */
    uint16_t per = LOOPTIME_TCY;
    g_pwmPer = per;
    HAL_PWM_SetDutyCyclePeriod((uint32_t)FixpMulU16(actualAmplitude, per), per);

    /* 7. Speed counting */
    speedCounter++;
    sectorCount++;
}

/* ── Called from Timer1 at 1ms (divide by 20 in caller) ─────────── */
/* clSettleCounter + CL_SETTLE_MS now declared at file top so they're
 * visible to SectorPI_Start (which resets the counter). */

void SectorPI_TimeTick(void)
{
    if (!running) return;

    /* CL settle: hold ramp duty for 2s after CL entry before
     * allowing pot to take over. Lets PI stabilize. */
    if (phase == SECTOR_PHASE_CL && !commandEnabled)
    {
        if (clSettleCounter < CL_SETTLE_MS)
            clSettleCounter++;
        else
            commandEnabled = true;
    }

#if FEATURE_MEAS_PI
    /* Spike-filter + EMA over tMeasHR at 1 kHz. Commutate writes raw
     * halved period (noisy, occasional near-zero samples from back-to-
     * back fires); filtering here keeps the hot path minimal. Reject
     * samples below half the current smoother (back-to-back spike
     * signature), EMA-smooth the rest with α=1/4. */
    {
        uint16_t sample = tMeasHR;
        if (sample >= MIN_PERIOD_HR) {
            if (tMeasHRSmooth < MIN_PERIOD_HR) {
                tMeasHRSmooth = sample;           /* seed */
            } else if (sample >= (tMeasHRSmooth >> 1)) {
                int32_t err = (int32_t)sample - (int32_t)tMeasHRSmooth;
                int32_t nt  = (int32_t)tMeasHRSmooth
                              + (err >> MEAS_PI_ALPHA_SHIFT);
                if (nt < MIN_PERIOD_HR) nt = MIN_PERIOD_HR;
                if (nt > 0xFFFF)        nt = 0xFFFF;
                tMeasHRSmooth = (uint16_t)nt;
            }
            /* else: spike below half of smoother — drop sample */
        }
    }
#endif

    /* Speed measurement */
    speedBase++;
    if (speedBase >= SPEED_MEAS_WINDOW)
    {
        measuredSpeed = speedCounter;
        speedCounter = 0;
        speedBase = 0;

        uint32_t erpm = (uint32_t)measuredSpeed * 500UL;

        /* Block-commutation auto-engagement (Option A: duty saturation).
         * At ≥95% Q15 the PWM is already clipping near the period cap,
         * so further "switching" gives no torque headroom. Override the
         * active phase H gate solid-ON for the whole sector — no PWM
         * chopping. Equivalent to infinite switching frequency.
         *
         * We capture rising-edge sectors only — falling sectors return
         * CAP_SENTINEL by design — so corridorGoodStreak alternates near
         * 1 even at perfect lock. The first version gated entry on
         * `goodStreak > 50`, which was unreachable. Replaced with a
         * "stable at speed" gauge (10 speed-windows × 20 ms = 200 ms
         * above 150k) for entry, and the inverse `corridorMissStreak`
         * (consecutive sentinels) for the exit lock-loss check. */
        if (erpm >= BLOCK_ENTER_ERPM)
        {
            if (blockCommStableTicks < 0xFFFFu) blockCommStableTicks++;
        }
        else
        {
            blockCommStableTicks = 0;
        }

        if (blockExitCooldown > 0U) blockExitCooldown--;

        if (g_blockCommActive)
        {
            if (actualAmplitude < 29490U                 /* < 90% Q15 */
                || erpm < BLOCK_EXIT_ERPM
                || corridorMissStreak > 5U)
            {
                g_blockCommActive = false;
                blockExitCooldown = 25U;                 /* 500 ms re-entry lockout */
            }
        }
        else
        {
            if (actualAmplitude >= 31130U                /* ≥ 95% Q15 */
                && erpm >= BLOCK_ENTER_ERPM
                && blockCommStableTicks >= 10U           /* ≥ 200 ms above enter eRPM */
                && commandEnabled
                && blockExitCooldown == 0U)
            {
                g_blockCommActive = true;
            }
        }
    }

    /* Pot→duty direct. Reactive scheduling in Commutate handles
     * speed — more duty = more torque = faster = earlier ZC. */
    if (commandEnabled)
    {
        if (actualAmplitude < targetAmplitude)
        {
            actualAmplitude += 13;
            if (actualAmplitude > targetAmplitude)
                actualAmplitude = targetAmplitude;
        }
        else if (actualAmplitude > targetAmplitude)
        {
            if (actualAmplitude > 13)
                actualAmplitude -= 13;
            else
                actualAmplitude = 0;
        }
    }
}

#ifdef MIN_AMPLITUDE_PROFILE
#define MIN_AMPLITUDE  MIN_AMPLITUDE_PROFILE   /* per-profile override (e.g. HiZ1460) */
#else
#define MIN_AMPLITUDE  5000U  /* ~15.3% Q15. At 60 kHz the per-cycle
                                  * switching overhead eats more of the
                                  * 16.7µs cycle, so 12.2% Q15 only
                                  * delivers ~10% effective duty — below
                                  * the desync threshold. */
#endif

void SectorPI_CommandSet(uint16_t amplitude)
{
    if (commandEnabled)
    {
        if (amplitude < MIN_AMPLITUDE)
            amplitude = MIN_AMPLITUDE;
        targetAmplitude = amplitude;
    }
}

uint32_t SectorPI_ErpmGet(void)
{
    return (uint32_t)measuredSpeed * (1000UL / SPEED_MEAS_WINDOW) * 60UL / 6UL;
}

bool SectorPI_IsRunning(void)
{
    return running;
}

uint8_t SectorPI_GetPhase(void)
{
    return (uint8_t)phase;
}

void SectorPI_GetCaptureLog(uint8_t *buf, uint8_t *entriesOut)
{
    uint8_t n = piLogCount;
    if (n > PI_LOG_ENTRIES) n = PI_LOG_ENTRIES;
    /* Snapshot under no-interrupt window — entries are 8B and the PI
     * could write a partial one mid-copy.  Cheap copy: 30 × 8 = 240B. */
    for (uint8_t i = 0; i < n; i++) {
        PI_LOG_ENTRY_T e = piLog[i];   /* struct copy from volatile */
        buf[i*8 + 0] = (uint8_t)(e.timerPeriod  & 0xFF);
        buf[i*8 + 1] = (uint8_t)(e.timerPeriod  >> 8);
        buf[i*8 + 2] = (uint8_t)(e.setValue     & 0xFF);
        buf[i*8 + 3] = (uint8_t)(e.setValue     >> 8);
        buf[i*8 + 4] = (uint8_t)(e.capValue     & 0xFF);
        buf[i*8 + 5] = (uint8_t)(e.capValue     >> 8);
        buf[i*8 + 6] = (uint8_t)((uint16_t)e.deltaClamped & 0xFF);
        buf[i*8 + 7] = (uint8_t)((uint16_t)e.deltaClamped >> 8);
    }
    *entriesOut = n;
}

void SectorPI_TelemGet(TELEM_T *out)
{
    out->timerPeriod     = timerPeriod;
    out->integrator      = integrator;
    out->lastCapValue    = lastCaptureHR_g;
    out->actualAmplitude = actualAmplitude;
    out->measuredSpeed   = measuredSpeed;
    out->position        = position;
    out->stallCounter    = stallCounter;
    out->sectorCount     = sectorCount;
    out->statusEvents    = statusEvents;
    out->running         = running;
    out->commandEnabled  = commandEnabled;
    out->diagCaptures    = diagCaptures;
    out->diagPiRuns      = diagPiRuns;
    out->diagLastCapValue = diagLastCapValue;
    out->diagDelta       = diagDelta;
    out->spMode          = false;
    out->spRequest       = false;
    {
        uint16_t erpmPeriod = actualStepPeriodHR ? actualStepPeriodHR : timerPeriod;
        out->erpmNow = (erpmPeriod > 0) ?
            (60UL * SECTOR_TIMER_FREQ_HZ / (6UL * (uint32_t)erpmPeriod)) : 0;
    }
    /* Capture-rate diagnostics — pull from garuda_service.c (ADC ISR side)
     * and the local Commutate counter. Lets the GUI/CSV diagnose where the
     * 50% capture loss is happening. */
    {
        extern volatile uint32_t adcBlankReject;
        extern volatile uint32_t adcStateMismatch;
        extern volatile uint32_t adcCaptureSet;
        extern volatile uint32_t adcSetRising;
        out->adcBlankReject     = adcBlankReject;
        out->adcStateMismatch   = adcStateMismatch;
        out->adcCaptureSet      = adcCaptureSet;
        out->adcSetRising       = adcSetRising;
        out->commutateNoCapture = diagCommutateNoCapture;
        extern volatile uint32_t ptgFires;
        out->ptgFires = ptgFires;
        out->ptgRisingAcc  = 0;
        out->ptgRisingRej  = 0;
        out->ptgFallingAcc = 0;
        out->ptgFallingRej = 0;
        extern volatile uint32_t postZcRisingAcc;
        extern volatile uint32_t postZcRisingRej;
        extern volatile uint32_t postZcFallingAcc;
        extern volatile uint32_t postZcFallingRej;
        out->postZcRisingAcc  = postZcRisingAcc;   /* 0 when POST_ZC_ACCEPT=0 */
        out->postZcRisingRej  = postZcRisingRej;
        out->postZcFallingAcc = postZcFallingAcc;
        out->postZcFallingRej = postZcFallingRej;
        out->tMeasHR          = tMeasHRSmooth;      /* smoothed; 0 when MEAS_PI=0 */
    }
}

