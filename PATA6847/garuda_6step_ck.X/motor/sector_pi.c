/**
 * @file sector_pi.c
 * @brief V4 Sector PI motor control.
 *
 * Startup: V3-style Timer1 countdown for ALIGN + OL_RAMP (proven).
 * Closed-loop: AVR-style PI synchronizer on SCCP3 + CCP2 capture.
 *
 * OL ramp is driven by Timer1 ISR calling SectorPI_OlTick() at 20kHz.
 * When ramp reaches MIN_STEP_PERIOD, SectorPI_EnterCL() starts the
 * SCCP3 sector timer and CCP2 capture. PI owns commutation from there.
 */

#include "sector_pi.h"

#if FEATURE_V4_SECTOR_PI

#include <xc.h>
#include "../garuda_config.h"
#include "../hal/hal_pwm.h"
#include "../hal/hal_com_timer.h"
#include "../hal/hal_capture.h"
#include "../hal/hal_ptg.h"
#include "../hal/port_config.h"
#include "commutation.h"
#include "v4_params.h"

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
    V4_OFF = 0,
    V4_ALIGN,
    V4_OL_RAMP,
    V4_CLOSED_LOOP
} V4_PHASE_T;

static volatile V4_PHASE_T phase;
static          uint8_t    position;        /* 0..5 */
static volatile bool       running;
static volatile uint16_t   statusEvents;
static volatile uint32_t   sectorCount;

/* ── OL ramp state (Timer1 driven, matches V3 startup.c) ───────── */
static volatile uint16_t   alignCounter;
static volatile uint16_t   rampStepPeriod;  /* Timer1 ticks per step */
static volatile uint16_t   rampCounter;     /* Timer1 countdown */
static volatile uint32_t   rampDuty;

/* ── PI state (SCCP3 driven, active in CL only) ────────────────── */
static volatile uint16_t   timerPeriod;     /* sector period (PI output) */
volatile uint16_t v4_timerPeriod;           /* exposed for CCP ISR speed check */
static volatile uint16_t   integrator;
static          uint8_t    stallCounter;
static volatile bool       commandEnabled;
/* SP mode flags.
 *   v4_spRequest: set in TimeTick based on eRPM threshold + hysteresis.
 *   v4_spActive : set in Commutate ISR when transition is actually applied
 *                 (MPER write + capture flush). Read by CCP ISRs in
 *                 garuda_service.c to switch ZC source from ADC midpoint
 *                 (invalid when MPER=0xFFFF) to hardware comparator IC.
 *                 Decoupling prevents mid-sector MPER changes that can
 *                 drop the transition sector's ZC. */
static volatile bool       v4_spRequest = false;
volatile bool              v4_spActive  = false;

/* ── Single-pulse mode: effective PWM period for duty scaling ─ */
/* HR tick = 640ns = 128 PWM ticks (Fosc/2=200MHz, 5ns/tick).   */
/* SP mode: per = timerPeriod << 7 (pulse width = amp × per).    */
/* Normal:  per = LOOPTIME_TCY (fixed 40kHz).                    */
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

/* V5.2 measurement-based period tracker.
 *   v5_tMeasHR       — raw halved measuredCommPeriod, written by Commutate ISR.
 *                       Fast path: single write, no branches. Noisy.
 *   v5_tMeasHRSmooth — spike-filtered + EMA smoothed, written by TimeTick
 *                       (1 ms rate where cycles are cheap). Clean value
 *                       shipped in telemetry and used by MEAS_PI_OWN.
 * Both defined unconditionally so telemetry works with flag off. */
volatile uint16_t v5_tMeasHR       = 0;
volatile uint16_t v5_tMeasHRSmooth = 0;

/* ── V5.3 scheduler state ───────────────────────────────────────
 * Used only when FEATURE_V5_SCHEDULER=1. Defined unconditionally so
 * telemetry and tooling can reference fields without flag gating.
 *
 * sched_prevCaptureHR / sched_thisCaptureHR:
 *   Capture timestamps from the last two ZC events (either CCP2
 *   rising or CCP5 falling). T_sector = this - prev. Unlike V4's
 *   measuredCommPeriod (Commutate-to-Commutate, which reads 2×
 *   because of ASAP pair firing), this is 1 physical sector.
 *
 * sched_Tsector:
 *   Raw per-sector period in HR ticks. Updated on each new
 *   capture. No EMA here — the scheduler uses this directly.
 *
 * sched_captureSource:
 *   0 = last capture was from rising (CCP2)
 *   1 = last capture was from falling (CCP5)
 *   Used to sanity-check that captures alternate (rising-falling).
 *
 * sched_diag* counters:
 *   Accepted captures per polarity, schedule misses, etc. */
volatile uint16_t sched_prevCaptureHR   = 0;
volatile uint16_t sched_thisCaptureHR   = 0;
volatile uint16_t sched_Tsector         = 0;
volatile uint8_t  sched_captureSource   = 0;
volatile bool     sched_firstCapture    = true;   /* true after EnterCL */
volatile uint32_t sched_diagRisingAcc   = 0;
volatile uint32_t sched_diagFallingAcc  = 0;
volatile uint32_t sched_diagImplausible = 0;    /* dt outside [T/2, 1.5T] */
volatile uint32_t sched_diagScheduleLate = 0;   /* target was past */
volatile uint32_t sched_diagScheduleOk  = 0;    /* target was future */

/* ── Speed regulation (outer loop) ─────────────────────────────── */
/* pot → targetSpeed, PI on (targetSpeed - measuredSpeed) → amplitude.
 * This is the AVR's __Speed_Control. Runs at 1ms in TimeTick.
 * Separate from the sector PI (inner loop) which tracks ZC phase. */
static volatile uint16_t   targetSpeed;     /* in measuredSpeed units */
static volatile int32_t    speedIntegrator; /* fixp16 amplitude integrator */
#define SPEED_KP            1U              /* proportional gain */
#define SPEED_KI_DT         3U              /* Ki * dt (integral step) */
#define SPEED_MIN_RPM       4000UL          /* minimum target eRPM */
#define SPEED_MAX_RPM       100000UL        /* maximum target eRPM */

/* ── Diagnostics ────────────────────────────────────────────────── */
volatile uint16_t diagCaptures;
volatile uint16_t diagPiRuns;
volatile uint16_t diagLastCapValue;
volatile int16_t  diagDelta;
/* Commutate ISR: capValue == SENTINEL (no valid capture this sector) */
volatile uint32_t diagCommutateNoCapture;

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
    HAL_PTG_Stop();        /* V5.0 — no-op unless FEATURE_V5_PTG_ZC=1 */
    HAL_ComTimer_Cancel();
    /* Exit SP mode if active — restores MPER = LOOPTIME_TCY. Reset
     * BOTH v4_spRequest and v4_spActive so the next start doesn't
     * inherit a latched request from the previous run. */
    if (v4_spActive)
    {
        MPER = LOOPTIME_TCY;
        PG1TRIGA = 0x0000U;
        PG1STATbits.UPDREQ = 1;
        PG2STATbits.UPDREQ = 1;
        PG3STATbits.UPDREQ = 1;
        g_pwmPer = LOOPTIME_TCY;
    }
    v4_spRequest = false;
    v4_spActive = false;
    HAL_PWM_SetSPFlag(false);
    HAL_PWM_ForceAllFloat();
    HAL_PWM_SetDutyCycle(0);
    running = false;
    phase = V4_OFF;
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
    if (hrPeriod < v4Params.minPeriodHr) hrPeriod = v4Params.minPeriodHr;

    timerPeriod = (uint16_t)hrPeriod;
    integrator  = timerPeriod;
    stallCounter = 0;
    commandEnabled = false;
    diagCaptures = 0;
    diagPiRuns = 0;
    diagLastCapValue = 0xFFFF;
    corridorGoodStreak = 0;
    corridorMissStreak = 0;
    piEnabled = false;

    targetPeriod = timerPeriod;
    basePeriod = timerPeriod;
    actualStepPeriodHR = timerPeriod;

    /* Keep current duty from ramp */
    actualAmplitude = (uint16_t)((rampDuty * 32768UL) / LOOPTIME_TCY);
    targetAmplitude = actualAmplitude;

    /* Configure CCP2 for current floating phase.
     * Flush FIFOs and clear flags BEFORE enabling ISRs to prevent
     * stale noise edges from setting v4_captureValid. */
    const COMMUTATION_STEP_T *step = &commutationTable[position];
    HAL_Capture_Configure(bemfRpPin[step->floatingPhase],
                          step->zcPolarity > 0);
    HAL_Capture_SetBlanking(timerPeriod);

    /* Flush any accumulated edges before enabling capture ISRs */
    while (CCP2STATLbits.ICBNE) (void)CCP2BUFL;
    while (CCP5STATLbits.ICBNE) (void)CCP5BUFL;
    _CCP2IF = 0;
    _CCP5IF = 0;
    v4_captureValid = false;

    HAL_Capture_Start();
    HAL_PTG_Start();       /* V5.0 — no-op unless FEATURE_V5_PTG_ZC=1 */

    /* Seed lastCommHR for PI elapsed calculation */
    lastCommHR = HAL_ComTimer_ReadTimer();

#if FEATURE_V5_SCHEDULER
    /* V5.3: seed scheduler state from ramp period. sched_Tsector is
     * the per-physical-sector period (not 2× like V4's measured).
     * First-capture flag tells CCP ISR to accept the first capture
     * unconditionally and use the ramp-derived sched_Tsector for
     * its initial schedule — avoids computing dt from stale seeds. */
    sched_Tsector       = (uint16_t)hrPeriod;
    sched_thisCaptureHR = 0;
    sched_prevCaptureHR = 0;
    sched_captureSource = 0;
    sched_firstCapture  = true;
    sched_diagRisingAcc    = 0;
    sched_diagFallingAcc   = 0;
    sched_diagImplausible  = 0;
    sched_diagScheduleLate = 0;
    sched_diagScheduleOk   = 0;
#endif

    /* Schedule first CL commutation via one-shot SCCP3 */
    HAL_ComTimer_ScheduleAbsolute(lastCommHR + timerPeriod);

    phase = V4_CLOSED_LOOP;

    /* Enable throttle after a short delay (handled in TimeTick) */
}

/* ────────────────────────────────────────────────────────────────── */
/* PUBLIC API                                                        */
/* ────────────────────────────────────────────────────────────────── */

void SectorPI_Init(void)
{
    /* Load runtime tunables from compile-time defaults BEFORE the HAL
     * inits run — HAL_Capture_SetBlanking and the PI loop both consume
     * v4Params on first use. */
    V4Params_InitDefaults();

    HAL_ComTimer_Init();   /* SCCP3 one-shot + SCCP4 HR (proven V3 pattern) */
    HAL_Capture_Init();
    HAL_PTG_Init();        /* V5.0 — no-op unless FEATURE_V5_PTG_ZC=1 */
    running = false;
    phase = V4_OFF;
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
    /* Reset SP state so a previous run's latched request doesn't
     * carry over into the next start. */
    v4_spRequest = false;
    v4_spActive = false;
    HAL_PWM_SetSPFlag(false);

    /* Reset capture-rate diagnostic counters (defined in garuda_service.c
     * for the ADC ISR side, sector_pi.c for the Commutate side). Lets us
     * read fresh ratios per run instead of accumulating across restarts. */
    diagCaptures = 0;
    diagPiRuns = 0;
    diagCommutateNoCapture = 0;
    extern volatile uint32_t v4_adcBlankReject;
    extern volatile uint32_t v4_adcStateMismatch;
    extern volatile uint32_t v4_adcCaptureSet;
    extern volatile uint32_t v4_adcSetRising;
    extern volatile uint32_t v4_adcAlreadySet;
    v4_adcBlankReject = 0;
    v4_adcStateMismatch = 0;
    v4_adcCaptureSet = 0;
    v4_adcSetRising = 0;
    v4_adcAlreadySet = 0;
    extern volatile uint32_t v4_offMidCapture;
    extern volatile uint32_t v4_offMidMismatch;
    v4_offMidCapture = 0;
    v4_offMidMismatch = 0;
    extern volatile uint32_t v5_postZcRisingAcc;
    extern volatile uint32_t v5_postZcRisingRej;
    extern volatile uint32_t v5_postZcFallingAcc;
    extern volatile uint32_t v5_postZcFallingRej;
    v5_postZcRisingAcc  = 0;
    v5_postZcRisingRej  = 0;
    v5_postZcFallingAcc = 0;
    v5_postZcFallingRej = 0;
    /* V5.2: reset measurement tracker to unseeded. It seeds on first
     * valid commutation interval in Commutate. Smoothed companion is
     * seeded from the raw value in TimeTick. */
    v5_tMeasHR       = 0;
    v5_tMeasHRSmooth = 0;

    /* Start alignment — Timer1 ISR drives via SectorPI_OlTick() */
    alignCounter = ALIGN_TIME_COUNTS;
    rampStepPeriod = INITIAL_STEP_PERIOD;
    rampCounter = INITIAL_STEP_PERIOD;
    rampDuty = ALIGN_DUTY;

    HAL_PWM_SetCommutationStep(0);
    HAL_PWM_SetDutyCycle(MIN_DUTY);

    phase = V4_ALIGN;
}

void SectorPI_Stop(void)
{
    ShutOff();
}

/* ── Called from Timer1 ISR at 20kHz (50µs) during ALIGN + OL_RAMP ── */
void SectorPI_OlTick(void)
{
    if (!running) return;

    if (phase == V4_ALIGN)
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
            phase = V4_OL_RAMP;
        }
        return;
    }

    if (phase == V4_OL_RAMP)
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

        /* Ramp complete → transition to closed-loop */
        if (rampStepPeriod <= MIN_STEP_PERIOD)
        {
            EnterCL();
        }
        return;
    }

    /* CL phase: CCP ISRs handle FIFO drain continuously */
}

/* ── V5.3 scheduler: one Commutate per physical sector ──────────
 * Called from SCCP3 ISR dispatch when FEATURE_V5_SCHEDULER=1. This
 * Commutate only advances PWM state — the scheduling of the NEXT
 * Commutate happens in the CCP2/CCP5 ISR when a new ZC capture
 * arrives. If no capture arrives, we fall back to a timer-based
 * re-fire at (thisCommHR + sched_Tsector) so the motor doesn't
 * stall.
 *
 * Position advances 1 per call, so Commutate rate = 6 per elec rev
 * = physical sector rate. Software position tracks physical rotor.
 *
 * Gated entirely by FEATURE_V5_SCHEDULER so when off, this function
 * doesn't exist in the binary.
 *
 * Diagnostics: sched_diag* counters, plus for backwards compat with
 * existing telemetry we populate actualStepPeriodHR/timerPeriod/
 * v4_timerPeriod from sched_Tsector so eTP reads match reality. */
#if FEATURE_V5_SCHEDULER

#define V5_SCHED_FORWARD_MARGIN_HR  10u    /* min forward margin on schedule */
#define V5_SCHED_BLANKING_MIN_HR    10u    /* min blanking after commutation */

static void CommutateV5_3(void)
{
    /* 1. Disable CCP ISRs during reconfiguration. */
    _CCP2IE = 0;
    _CCP5IE = 0;

    uint16_t thisCommHR = HAL_ComTimer_ReadTimer();

    /* 2. Advance position 1:1 with physical rotor. */
    position++;
    if (position > 5) position = 0;
    HAL_PWM_SetCommutationStep(position);

    /* 3. Configure CCP for NEW sector's floating phase and ZC polarity. */
    const COMMUTATION_STEP_T *step = &commutationTable[position];
    HAL_Capture_Configure(bemfRpPin[step->floatingPhase],
                          step->zcPolarity > 0);

    extern volatile uint8_t v4_floatingPhase;
    v4_floatingPhase = step->floatingPhase;

    /* 4. Flush FIFOs, clear flags, reset captureValid. */
    while (CCP2STATLbits.ICBNE) (void)CCP2BUFL;
    while (CCP5STATLbits.ICBNE) (void)CCP5BUFL;
    _CCP2IF = 0;
    _CCP5IF = 0;
    v4_captureValid = false;

    /* 5. Blanking: 25% of sector period (minimum safety floor). */
    uint16_t blankHR = sched_Tsector >> 2;
    if (blankHR < V5_SCHED_BLANKING_MIN_HR) blankHR = V5_SCHED_BLANKING_MIN_HR;
    extern volatile uint16_t v4_blankingEndHR;
    v4_blankingEndHR = (uint16_t)(thisCommHR + blankHR);

    /* 6. PTG ISR expected comp state (same position-parity logic). */
#if FEATURE_V5_PTG_ZC || FEATURE_V5_POST_ZC_ACCEPT
    {
        extern volatile uint8_t v5_ptgExpectedComp;
        v5_ptgExpectedComp = (uint8_t)(position & 1u);
    }
#endif

    /* 7. Fallback schedule: if no capture arrives, re-fire at
     * thisCommHR + sched_Tsector so the motor keeps moving. The
     * common case is CCP ISR re-schedules us at (capture + T/2 -
     * advance) before this fires. */
    HAL_ComTimer_ScheduleAbsolute((uint16_t)(thisCommHR + sched_Tsector));

    /* 8. Re-enable CCP ISRs. */
    _CCP2IE = 1;
    _CCP5IE = 1;

    /* 9. Expose to existing telemetry paths so eTP/eRpm read correct. */
    actualStepPeriodHR = sched_Tsector;
    timerPeriod        = sched_Tsector;
    v4_timerPeriod     = sched_Tsector;
    lastCommHR         = thisCommHR;

    /* 10. Speed counting. */
    speedCounter++;
    sectorCount++;
}
#endif /* FEATURE_V5_SCHEDULER */

/* ── Called from SCCP3 ISR (sector timer match) — CL only ───────── */
void SectorPI_Commutate(void)
{
    if (phase != V4_CLOSED_LOOP) return;

#if FEATURE_V5_SCHEDULER
    CommutateV5_3();
    return;
#endif

    /* V4 path below — unchanged when FEATURE_V5_SCHEDULER=0. */

    /* 1. Disable CCP ISRs during critical section */
    _CCP2IE = 0;
    _CCP5IE = 0;

    /* 2. Consume best corridor candidate from previous sector */
    uint16_t thisCommHR = HAL_ComTimer_ReadTimer();
    uint16_t prevCommHR = lastCommHR;
    uint16_t measuredCommPeriod = (uint16_t)(thisCommHR - prevCommHR);
    if (measuredCommPeriod >= V4_MIN_PERIOD)
    {
        actualStepPeriodHR = measuredCommPeriod;
#if FEATURE_V5_MEAS_PI
        /* V5.2 tracker: halve measuredCommPeriod (raw reads 2× real sector
         * period, root cause unresolved). Minimal single write — any
         * additional ISR cycles destabilize the V4 Commutate timing. */
        v5_tMeasHR = (uint16_t)(measuredCommPeriod >> 1);
        if (v5_tMeasHR < V4_MIN_PERIOD) v5_tMeasHR = V4_MIN_PERIOD;
#endif
#if FEATURE_V5_MEAS_PI_OWN
        /* V5.2 ownership: drive timerPeriod off the SMOOTHED tracker
         * (v5_tMeasHRSmooth, filtered in TimeTick) — never the noisy
         * raw v5_tMeasHR. One volatile read + 2 writes, no math, no
         * branches beyond the SP gate. The reactive scheduler and
         * fallback both read timerPeriod later in this ISR, so the
         * overwrite steers commutation off measurement truth. The
         * set-point PI block below still runs but its timerPeriod
         * writes get overwritten by this one. Pre-seed guard: if the
         * smoother hasn't seeded yet (early CL entry, first few
         * Commutates), skip the overwrite so the ramp-seeded value
         * from EnterCL stays intact. */
        if (!v4_spActive && v5_tMeasHRSmooth >= V4_MIN_PERIOD) {
            uint16_t t = v5_tMeasHRSmooth;
            timerPeriod    = t;
            integrator     = t;
            v4_timerPeriod = t;
        }
#endif
    }
    uint16_t capValue = CAP_SENTINEL;

    if (v4_captureValid)
    {
        uint16_t elapsed = (uint16_t)(v4_lastCaptureHR - prevCommHR);
        /* Filter against the actual measured rotor sector, not the PI
         * estimate. timerPeriod can settle ~2× T_rotor, which lets cross-
         * sector captures pass and locks the system into an every-other-
         * sector miss pattern. Use the truth: actualStepPeriodHR. Fall
         * back to timerPeriod only on the very first commutation when
         * actualStepPeriodHR isn't initialized yet. */
        uint16_t filterHR = (actualStepPeriodHR >= V4_MIN_PERIOD)
                            ? actualStepPeriodHR : timerPeriod;
        if (elapsed < filterHR)
        {
            capValue = elapsed;
            diagCaptures++;
        }
        else
        {
            diagCommutateNoCapture++;  /* capture present but past filter */
        }
        v4_captureValid = false;
    }
    else
    {
        diagCommutateNoCapture++;      /* no capture at all this sector */
    }
    lastCommHR = thisCommHR;

    /* 2.5. SP mode transition — applied at sector boundary so the
     * previous sector's ZC (consumed above) isn't dropped by a
     * mid-sector MPER swap. ZC source switches between ADC midpoint
     * (normal) and CCP hardware capture (SP) via v4_spActive. */
    if (v4_spActive != v4_spRequest)
    {
        while (CCP2STATLbits.ICBNE) (void)CCP2BUFL;
        while (CCP5STATLbits.ICBNE) (void)CCP5BUFL;
        _CCP2IF = 0;
        _CCP5IF = 0;
        /* v4_captureValid already cleared above after consumption */

        if (v4_spRequest)
        {
            MPER = 0xFFFFU;
        }
        else
        {
            MPER = LOOPTIME_TCY;
            PG1TRIGA = 0x0000U;
            g_pwmPer = LOOPTIME_TCY;
        }
        PG1STATbits.UPDREQ = 1;
        PG2STATbits.UPDREQ = 1;
        PG3STATbits.UPDREQ = 1;
        v4_spActive = v4_spRequest;
        HAL_PWM_SetSPFlag(v4_spActive);
    }

    /* 3. Advance to next commutation step */
    position++;
    if (position > 5) position = 0;
    HAL_PWM_SetCommutationStep(position);

    /* 4. Configure CCP for new phase, flush FIFOs */
    {
        const COMMUTATION_STEP_T *step = &commutationTable[position];
        HAL_Capture_Configure(bemfRpPin[step->floatingPhase],
                              step->zcPolarity > 0);
        while (CCP2STATLbits.ICBNE) (void)CCP2BUFL;
        while (CCP5STATLbits.ICBNE) (void)CCP5BUFL;
        _CCP2IF = 0;
        _CCP5IF = 0;
    }

    /* 5. Set blanking + floating phase + expected ZC for CCP ISR */
    {
        extern volatile uint16_t v4_blankingEndHR;
        extern volatile uint16_t v4_expectedZcHR;
        extern volatile uint8_t v4_floatingPhase;

        /* Blanking: 25% of measured rotor sector. Always use
         * actualStepPeriodHR (truth) instead of timerPeriod (PI estimate
         * which can settle at 2× T_rotor and reject valid early ZCs).
         * Fall back to timerPeriod only on first commutation before
         * actualStepPeriodHR is initialized.
         * In SP mode, extend past pulse turn-off + 10µs settle. */
        uint16_t sectorHR = (actualStepPeriodHR >= V4_MIN_PERIOD)
                            ? actualStepPeriodHR : timerPeriod;
        /* Runtime blanking (V4 Commutate ISR) — was hardcoded `>> 2`
         * (25%), ignoring v4Params.blankingPct.  Wired up 2026-04-28
         * so SET_PARAM 0xF3 actually takes effect.  Fallback to 25%
         * if pct is zero or out of range. */
        uint8_t  blankPct = v4Params.blankingPct;
        uint16_t blankHR;
        if (blankPct == 0U || blankPct > 100U) {
            blankHR = sectorHR >> 2;   /* safe fallback */
        } else {
            blankHR = (uint16_t)(((uint32_t)sectorHR * blankPct) / 100U);
        }
        if (v4_spActive)
        {
            uint16_t dutyHR = (uint16_t)(((uint32_t)actualAmplitude
                                          * sectorHR) >> 15);
            uint16_t spBlankHR = dutyHR + 16U;   /* +10.2µs settle */
            if (spBlankHR > blankHR) blankHR = spBlankHR;
        }
        v4_blankingEndHR = (uint16_t)(thisCommHR + blankHR);

        /* Expected ZC = prev_comm + T/2 + advance (physical ZC position).
         * Matches the reactive scheduler: targetHR = ZC + (T/2 - advance),
         * so ZC = target - T/2 + advance = this_comm + T/2 + advance.
         * tal selection mirrors the reactive path (3 at high speed). */
        uint16_t halfHR_exp = sectorHR >> 1;
        uint16_t tal_exp    = (sectorHR < 260) ? 3U : 2U;
        uint16_t advHR_exp  = (uint16_t)((sectorHR >> 3) * tal_exp);
        v4_expectedZcHR = (uint16_t)(thisCommHR + halfHR_exp + advHR_exp);

        /* Floating phase for GPIO deglitch reads */
        const COMMUTATION_STEP_T *step = &commutationTable[position];
        v4_floatingPhase = step->floatingPhase;

        /* Per-sector expected post-ZC comp state, written here so any
         * ISR that samples BEMF can decide accept/reject without
         * going through HAL_Capture_IsRisingZc() (which gets stuck at
         * true via a mechanism we still don't fully understand — see
         * Apr 18 investigation notes). Used by:
         *   V5.0 PTG diagnostic ISR (when FEATURE_V5_PTG_ZC=1)
         *   V5.1 ADC post-ZC shadow (when FEATURE_V5_POST_ZC_ACCEPT=1)
         * Variable name is v5_ptgExpectedComp for historical reasons;
         * semantics are "comp state expected AFTER ZC in this sector". */
#if FEATURE_V5_PTG_ZC || FEATURE_V5_POST_ZC_ACCEPT
        {
            extern volatile uint8_t v5_ptgExpectedComp;
            /* Expected post-ZC comp state from position parity
             * (even steps 0,2,4 = rising sectors → comp drops to 0;
             *  odd  steps 1,3,5 = falling sectors → comp rises to 1).
             * NOTE: this value is biased 2T:ε in wallclock due to the
             * V4 scheduler's ASAP+fallback pairing — see V5.3 scheduler
             * rewrite proposal. The flag itself is correct per-sector;
             * the software state just isn't 1:1 with physical rotor. */
            v5_ptgExpectedComp = (uint8_t)(position & 1u);
        }
#endif
        /* HAL_PTG_SetDelay call intentionally removed for this test. */
    }

    /* 6. Re-enable CCP ISRs (always).
     *
     * Phase C attempt (2026-04-21) gated this on v4_spActive —
     * tripped at ~100k eRPM even with Phase A inline drain in place.
     * The CCP ISR preemption (pri 4 preempts ADC ISR pri 3, fires
     * on every edge, not batched to PWM center) is load-bearing in
     * a way inline drain can't replicate. Reverted; CCP ISRs stay
     * always-on. */
    _CCP2IE = 1;
    _CCP5IE = 1;

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
         * destabilizing high-speed operation.
         *
         * SP gate kept: SP mode freezes timerPeriod since CCP captures
         * have different timing semantics than midpoint-ADC. */
        if (!v4_spActive)
        {
            /* Snapshot runtime params once per cycle. They're written by
             * the GSP set-param path on a low-priority code path; reading
             * to locals here keeps the ISR consistent within a cycle even
             * if the GUI changes a value mid-update. */
            uint16_t advFp8     = v4Derived.advancePlus30Fp8;
            uint8_t  kpShift    = v4Params.piKpShift;
            uint8_t  kiShift    = v4Params.piKiShift;
            uint16_t minPeriod  = v4Params.minPeriodHr;

            uint16_t setValue = (uint16_t)(((uint32_t)advFp8 * timerPeriod) >> 8)
                                + V4_RC_DELAY_HR;
            int32_t delta = (int32_t)capValue - (int32_t)setValue;

            int32_t newInt = (int32_t)integrator + (delta >> kiShift);
            if (newInt < minPeriod) newInt = minPeriod;
            if (newInt > 0xFFFF) newInt = 0xFFFF;
            integrator = (uint16_t)newInt;

            int32_t newPer = (int32_t)integrator + (delta >> kpShift);
            if (newPer < minPeriod) newPer = minPeriod;
            if (newPer > 0xFFFF) newPer = 0xFFFF;
            timerPeriod = (uint16_t)newPer;
            v4_timerPeriod = timerPeriod;  /* expose for CCP ISR */
        }

        /* Reactive target: ZC timestamp + delay-to-next-commutation.
         *
         * Speed-adaptive advance (proven 196k baseline):
         *   schedPeriod >= 260 HR (≤60k eRPM): TAL=2 → 15° advance
         *   schedPeriod <  260 HR (>60k eRPM): TAL=3 → 22.5° advance
         *
         * The 2026-04-28 unification (using v4Derived.advancePlus30Fp8
         * for both PI and scheduler) removed this ramp and motor walled
         * at 130k.  Restored: PI's setValue stays at fixed 15° (its own
         * model); scheduler keeps speed-adaptive ramp. */
        uint16_t schedPeriod = v4_spActive ? actualStepPeriodHR : timerPeriod;
        uint16_t halfHR = schedPeriod >> 1;
        /* Speed-adaptive TAL ramp:
         *   schedPeriod >= 260 HR (≤60k eRPM):  TAL=2 → 15°
         *   130-260   (60-120k eRPM):           TAL=3 → 22.5°
         *   schedPeriod < 130 HR (>120k eRPM):  TAL=4 → 30°
         * 4th band added 2026-04-28 to push past 178k wall.
         *
         * Predicted-scheduling experiment 2026-04-28: tried thisCommHR-
         * anchored target above 175k for >30° advance. Regressed peak
         * 195k→178k because timerPeriod saturates at minPeriodHr giving
         * a bad period estimate. Reverted; current TAL=4 floor at 195k
         * may be a real BEMF/timing limit, not a scheduler one. */
        uint16_t tal;
        if      (schedPeriod >= 260U) tal = 2U;
        else if (schedPeriod >= 130U) tal = 3U;
        else                          tal = 4U;
        uint16_t advHR  = (uint16_t)((schedPeriod >> 3) * tal);
        uint16_t delayHR = (halfHR > advHR) ? (uint16_t)(halfHR - advHR) : 2U;
        uint16_t targetHR = (uint16_t)(v4_lastCaptureHR + delayHR);

        diagDelta = (int16_t)(targetHR - thisCommHR);  /* margin */
        HAL_ComTimer_ScheduleAbsolute(targetHR);
    }
    else
    {
        /* No ZC this sector — forced commutation at the measured sector
         * period in SP. `timerPeriod` is a detector model in normal mode
         * and can be ~2x actual sector near 70k, which immediately
         * desyncs SP if used as the fallback period. */
        uint16_t schedPeriod = v4_spActive ? actualStepPeriodHR : timerPeriod;
        diagDelta = 0;
        HAL_ComTimer_ScheduleAbsolute(thisCommHR + schedPeriod);
    }

    /* 7. Stall detection */
    if (commandEnabled)
    {
        if (capValue == CAP_SENTINEL)
        {
            stallCounter++;
            if (stallCounter > V4_STALL_THRESHOLD)
            {
                ShutOff();
                statusEvents |= V4_EVENT_STALL;
            }
        }
        else
        {
            if (stallCounter > 0) stallCounter--;
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
    if (v4_spActive)
    {
        uint32_t spPer = (uint32_t)actualStepPeriodHR << 7;
        if (spPer > 0xFFFFU) spPer = 0xFFFFU;
        per = (uint16_t)spPer;
    }
    g_pwmPer = per;
    HAL_PWM_SetDutyCyclePeriod((uint32_t)FixpMulU16(actualAmplitude, per), per);
    if (v4_spActive)
    {
        PG1STATbits.TRSET = 1;
    }

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
    if (phase == V4_CLOSED_LOOP && !commandEnabled)
    {
        if (clSettleCounter < CL_SETTLE_MS)
            clSettleCounter++;
        else
            commandEnabled = true;
    }

#if FEATURE_V5_MEAS_PI
    /* V5.2: spike-filter + EMA over v5_tMeasHR at 1 kHz. The Commutate
     * ISR writes raw halved period (noisy, occasional near-zero samples
     * from back-to-back fires). Filtering here keeps the hot path
     * minimal — TimeTick has microseconds of slack.
     * Reject any sample less than half the current smoother (the
     * back-to-back spike signature), and EMA-smooth the rest with
     * α=1/4 (V5_MEAS_PI_ALPHA_SHIFT). MEAS_PI_OWN reads v5_tMeasHRSmooth. */
    {
        uint16_t sample = v5_tMeasHR;
        if (sample >= V4_MIN_PERIOD) {
            if (v5_tMeasHRSmooth < V4_MIN_PERIOD) {
                v5_tMeasHRSmooth = sample;           /* seed */
            } else if (sample >= (v5_tMeasHRSmooth >> 1)) {
                int32_t err = (int32_t)sample - (int32_t)v5_tMeasHRSmooth;
                int32_t nt  = (int32_t)v5_tMeasHRSmooth
                              + (err >> V5_MEAS_PI_ALPHA_SHIFT);
                if (nt < V4_MIN_PERIOD) nt = V4_MIN_PERIOD;
                if (nt > 0xFFFF)        nt = 0xFFFF;
                v5_tMeasHRSmooth = (uint16_t)nt;
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

        /* SP mode request — hysteresis based on measured eRPM.
         * Actual transition (MPER write, capture flush) happens in
         * Commutate ISR at a sector boundary, so the transition sector
         * consumes its ZC before the ZC source changes. */
        uint32_t erpm = (uint32_t)measuredSpeed * 500UL;
        if (!v4_spRequest && erpm >= SP_ENTER_ERPM)
        {
            v4_spRequest = true;
        }
        else if (v4_spRequest && erpm <= SP_EXIT_ERPM)
        {
            v4_spRequest = false;
        }

        /* Block-commutation auto-engagement (Option A: duty saturation).
         * At ≥95% Q15 the PWM is already clipping near the period cap,
         * so further "switching" gives no torque headroom. Override the
         * active phase H gate solid-ON for the whole sector — no PWM
         * chopping. Equivalent to infinite switching frequency.
         *
         * V4 captures rising-edge sectors only — falling sectors return
         * CAP_SENTINEL by design — so corridorGoodStreak alternates near
         * 1 even at perfect lock. The first version gated entry on
         * `goodStreak > 50`, which was unreachable. Replaced with a
         * "stable at speed" gauge (10 speed-windows × 20 ms = 200 ms
         * above 150k) for entry, and the inverse `corridorMissStreak`
         * (consecutive sentinels) for the exit lock-loss check. */
        if (erpm >= V4_BLOCK_ENTER_ERPM)
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
                || erpm < V4_BLOCK_EXIT_ERPM
                || corridorMissStreak > 5U)
            {
                g_blockCommActive = false;
                blockExitCooldown = 25U;                 /* 500 ms re-entry lockout */
            }
        }
        else
        {
            if (actualAmplitude >= 31130U                /* ≥ 95% Q15 */
                && erpm >= V4_BLOCK_ENTER_ERPM
                && blockCommStableTicks >= 10U           /* ≥ 200 ms above enter eRPM */
                && commandEnabled
                && !v4_spActive
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

#ifdef V4_MIN_AMPLITUDE_PROFILE
#define V4_MIN_AMPLITUDE  V4_MIN_AMPLITUDE_PROFILE   /* per-profile override (e.g. HiZ1460) */
#else
#define V4_MIN_AMPLITUDE  5000U  /* ~15.3% duty — bumped from 4000 (12.2%) on
                                  * 2026-04-29 because at 60 kHz the per-cycle
                                  * switching overhead eats more of the shorter
                                  * 16.7µs cycle, so 12.2% Q15 only delivers
                                  * ~10% effective. Empirical desync threshold
                                  * was ~12% commanded; 15.3% gives margin. */
#endif

void SectorPI_CommandSet(uint16_t amplitude)
{
    if (commandEnabled)
    {
        if (amplitude < V4_MIN_AMPLITUDE)
            amplitude = V4_MIN_AMPLITUDE;
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

void SectorPI_TelemGet(V4_TELEM_T *out)
{
    out->timerPeriod     = timerPeriod;
    out->integrator      = integrator;
    out->lastCapValue    = v4_lastCaptureHR;
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
    out->spMode          = v4_spActive;
    out->spRequest       = v4_spRequest;
    {
        uint16_t erpmPeriod = actualStepPeriodHR ? actualStepPeriodHR : timerPeriod;
        out->erpmNow = (erpmPeriod > 0) ?
            (60UL * V4_TIMER_FREQ_HZ / (6UL * (uint32_t)erpmPeriod)) : 0;
    }
    /* Capture-rate diagnostics — pull from garuda_service.c (ADC ISR side)
     * and the local Commutate counter. Lets the GUI/CSV diagnose where the
     * 50% capture loss is happening. */
    {
        extern volatile uint32_t v4_adcBlankReject;
        extern volatile uint32_t v4_adcStateMismatch;
        extern volatile uint32_t v4_adcCaptureSet;
        extern volatile uint32_t v4_adcSetRising;
        extern volatile uint32_t v4_adcAlreadySet;
        out->adcBlankReject     = v4_adcBlankReject;
        out->adcStateMismatch   = v4_adcStateMismatch;
        out->adcCaptureSet      = v4_adcCaptureSet;
        out->adcSetRising       = v4_adcSetRising;
        out->adcAlreadySet      = v4_adcAlreadySet;
        out->commutateNoCapture = diagCommutateNoCapture;
        extern volatile uint32_t v4_offMidCapture;
        extern volatile uint32_t v4_offMidMismatch;
        out->offMidCapture  = v4_offMidCapture;
        out->offMidMismatch = v4_offMidMismatch;
        out->ptgFires       = v5_ptgFires;       /* 0 when V5_PTG=0 */
        out->ptgRisingAcc   = v5_ptgRisingAcc;
        out->ptgRisingRej   = v5_ptgRisingRej;
        out->ptgFallingAcc  = v5_ptgFallingAcc;
        out->ptgFallingRej  = v5_ptgFallingRej;
        extern volatile uint32_t v5_postZcRisingAcc;
        extern volatile uint32_t v5_postZcRisingRej;
        extern volatile uint32_t v5_postZcFallingAcc;
        extern volatile uint32_t v5_postZcFallingRej;
        out->postZcRisingAcc  = v5_postZcRisingAcc;   /* 0 when V5_POST_ZC=0 */
        out->postZcRisingRej  = v5_postZcRisingRej;
        out->postZcFallingAcc = v5_postZcFallingAcc;
        out->postZcFallingRej = v5_postZcFallingRej;
        out->tMeasHR          = v5_tMeasHRSmooth;      /* smoothed; 0 when V5_MEAS=0 */
    }
}

#endif /* FEATURE_V4_SECTOR_PI */
