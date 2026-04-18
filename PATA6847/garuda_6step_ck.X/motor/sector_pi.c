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
static bool piEnabled = false;        /* PI error: capValue - setValue */

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

    /* Seed lastCommHR for PI elapsed calculation */
    lastCommHR = HAL_ComTimer_ReadTimer();

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
    running = false;
    phase = V4_OFF;
    statusEvents = 0;
    sectorCount = 0;
}

void SectorPI_Start(uint16_t vbusRaw)
{
    if (running) return;
    (void)vbusRaw;

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

/* ── Called from SCCP3 ISR (sector timer match) — CL only ───────── */
void SectorPI_Commutate(void)
{
    if (phase != V4_CLOSED_LOOP) return;

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
        uint16_t blankHR = sectorHR >> 2;
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
    }

    /* 6. Re-enable CCP ISRs */
    _CCP2IE = 1;
    _CCP5IE = 1;

    /* Track good-capture streak for diagnostics */
    if (capValue != CAP_SENTINEL)
    {
        if (corridorGoodStreak < 255) corridorGoodStreak++;
    }
    else
    {
        corridorGoodStreak = 0;
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

        /* Reactive target: ZC timestamp + halfPeriod - advance.
         * TAL=2 (15°) at low speed for stable startup.
         * TAL=3 (22.5°) above 60k eRPM for high-speed margin.
         * 60k eRPM = 15.6M/60000 = 260 ticks. */
        uint16_t schedPeriod = v4_spActive ? actualStepPeriodHR : timerPeriod;
        uint16_t halfHR = schedPeriod >> 1;
        uint16_t tal = (schedPeriod < 260) ? 3 : 2;
        uint16_t advHR = (schedPeriod >> 3) * tal;
        uint16_t delayHR = (halfHR > advHR) ? (halfHR - advHR) : 2;
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
static uint16_t clSettleCounter = 0;
#define CL_SETTLE_MS  500U   /* Hold ramp duty + suppress PI for 500ms.
                               * Lets motor stabilize at OL ramp speed
                               * before PI starts adjusting timerPeriod. */

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

#define V4_MIN_AMPLITUDE  4000U  /* ~12% duty — enough BEMF at CL entry */

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
    }
}

#endif /* FEATURE_V4_SECTOR_PI */
