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
static volatile uint16_t   integrator;
static          uint8_t    stallCounter;
static volatile bool       commandEnabled;
static volatile uint16_t   actualAmplitude;
static volatile uint16_t   targetAmplitude;
static volatile uint16_t   targetPeriod;    /* pot-commanded speed */
static volatile uint16_t   basePeriod;      /* ramped toward targetPeriod */
static volatile uint16_t   lastCommHR;      /* HR time of last commutation */

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
    if (hrPeriod < V4_MIN_PERIOD) hrPeriod = V4_MIN_PERIOD;

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

    /* Keep current duty from ramp */
    actualAmplitude = (uint16_t)((rampDuty * 32768UL) / LOOPTIME_TCY);
    targetAmplitude = actualAmplitude;

    /* Configure CCP2 for current floating phase */
    const COMMUTATION_STEP_T *step = &commutationTable[position];
    HAL_Capture_Configure(bemfRpPin[step->floatingPhase],
                          step->zcPolarity > 0);
    HAL_Capture_SetBlanking(timerPeriod);
    HAL_Capture_Start();
    /* Flush FIFOs + clear IFs after start to prevent bogus captures */
    while (CCP2STATLbits.ICBNE) (void)CCP2BUFL;
    while (CCP5STATLbits.ICBNE) (void)CCP5BUFL;
    _CCP2IF = 0;
    _CCP5IF = 0;

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
    uint16_t capValue = CAP_SENTINEL;

    if (v4_captureValid)
    {
        uint16_t elapsed = (uint16_t)(v4_lastCaptureHR - lastCommHR);
        if (elapsed < timerPeriod)
        {
            capValue = elapsed;
            diagCaptures++;
        }
        v4_captureValid = false;
    }
    lastCommHR = thisCommHR;

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

    /* 5. Set blanking + floating phase for deglitch ISR */
    {
        extern volatile uint16_t v4_blankingEndHR;
        extern volatile uint8_t v4_floatingPhase;

        /* Blanking: 25% of sector after commutation */
        v4_blankingEndHR = (uint16_t)(thisCommHR + (timerPeriod >> 2));

        /* Floating phase for GPIO deglitch reads */
        const COMMUTATION_STEP_T *step = &commutationTable[position];
        v4_floatingPhase = step->floatingPhase;
    }

    /* 6. Re-enable CCP ISRs */
    _CCP2IE = 1;
    _CCP5IE = 1;

    /* Track corridor streak — PI only runs after 12 consecutive
     * sectors with valid corridor captures. This ensures the
     * measurement stream is coherent before PI adjusts timing. */
    if (capValue != CAP_SENTINEL)
    {
        if (corridorGoodStreak < 255) corridorGoodStreak++;
        if (corridorGoodStreak >= CORRIDOR_GOOD_THRESHOLD)
            piEnabled = true;
    }
    else
    {
        corridorGoodStreak = 0;
        /* Don't clear piEnabled — once locked, stay locked
         * unless stall detection intervenes. */
    }

    /* Reactive scheduling — like V3 but with hardware timestamp.
     * When ZC is captured: schedule commutation at ZC + delay.
     * When no ZC: use last timerPeriod as timeout (forced comm).
     * Pot controls duty. Speed follows motor naturally. */
    diagLastCapValue = capValue;
    if (capValue != CAP_SENTINEL)
    {
        diagPiRuns++;

        /* Update step period estimate from actual ZC timing.
         * capValue = elapsed from lastComm to ZC.
         * ZC is at (30°+advance)/60° = 40°/60° = 66.7% of sector.
         * stepPeriod = capValue / 0.667 = capValue * 3 / 2.
         * IIR smooth: timerPeriod = (3*old + new) >> 2 */
        uint16_t measuredStep = (uint16_t)((uint32_t)capValue * 3 / 2);
        uint32_t iir = ((uint32_t)timerPeriod * 3 + measuredStep) >> 2;
        if (iir < V4_MIN_PERIOD) iir = V4_MIN_PERIOD;
        if (iir > 0xFFFF) iir = 0xFFFF;
        timerPeriod = (uint16_t)iir;
        integrator = timerPeriod;

        /* V3-style reactive target:
         * target = zcTimestamp + halfPeriod - advance
         * halfPeriod = timerPeriod / 2
         * advance = TAL * timerPeriod / 8 (TAL=2 for 15°) */
        uint16_t halfHR = timerPeriod >> 1;
        uint16_t advHR = (timerPeriod >> 3) * 2;  /* TAL=2 = 15° */
        uint16_t delayHR = (halfHR > advHR) ? (halfHR - advHR) : 2;
        uint16_t targetHR = (uint16_t)(v4_lastCaptureHR + delayHR);

        diagDelta = (int16_t)(targetHR - thisCommHR);  /* margin */
        HAL_ComTimer_ScheduleAbsolute(targetHR);
    }
    else
    {
        /* No ZC this sector — forced commutation at timerPeriod */
        HAL_ComTimer_ScheduleAbsolute(thisCommHR + timerPeriod);
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

    /* 6. PWM duty */
    HAL_PWM_SetDutyCycle((uint32_t)FixpMulU16(actualAmplitude, LOOPTIME_TCY));

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

#define V4_MIN_AMPLITUDE  3000U

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
}

#endif /* FEATURE_V4_SECTOR_PI */
