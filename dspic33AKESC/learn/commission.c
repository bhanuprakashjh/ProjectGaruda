/**
 * @file commission.c
 *
 * @brief Self-commissioning state machine implementation.
 *
 * Static tests measure phase resistance (DC injection) and inductance
 * (voltage step response) with the motor stopped. Dynamic tests perform
 * a protected low-speed spin to determine Kv, pole count, and optimal
 * timing advance.
 *
 * Component: LEARN / COMMISSION
 */

#include "commission.h"

#if FEATURE_COMMISSION

#include "../hal/hal_pwm.h"

/* Timeout and sample targets per state */
#define STATIC_R_TIMEOUT_MS     2000
#define STATIC_R_SAMPLES        100
#define STATIC_L_TIMEOUT_MS     2000
#define STATIC_L_SAMPLES        50
#define DYNAMIC_KV_TIMEOUT_MS   5000
#define DYNAMIC_KV_SAMPLES      200
#define DYNAMIC_POLES_TIMEOUT_MS 5000
#define DYNAMIC_POLES_SAMPLES   100
#define DYNAMIC_TIMING_TIMEOUT_MS 5000
#define DYNAMIC_TIMING_SAMPLES  200
#define VALIDATE_TIMEOUT_MS     3000
#define VALIDATE_SAMPLES        100
#define VALIDATE_MIN_CONFIDENCE 200

/* Progress percentages for each state */
static const uint8_t stateProgress[] = {
    0,   /* COMM_IDLE */
    10,  /* COMM_STATIC_R */
    20,  /* COMM_STATIC_L */
    40,  /* COMM_DYNAMIC_KV */
    55,  /* COMM_DYNAMIC_POLES */
    70,  /* COMM_DYNAMIC_TIMING */
    85,  /* COMM_VALIDATE */
    95,  /* COMM_COMMIT */
    100, /* COMM_COMPLETE */
    0    /* COMM_ERROR */
};

static void EnterState(COMMISSION_DATA_T *comm, COMMISSION_STATE_T newState,
                       uint32_t now, uint16_t timeout, uint16_t targetSamples)
{
    comm->prevState = comm->state;
    comm->state = newState;
    comm->stateEntryTick = now;
    comm->timeoutMs = timeout;
    comm->targetSamples = targetSamples;
    comm->sampleCount = 0;
    comm->accumulator = 0;
}

void COMMISSION_Init(COMMISSION_DATA_T *comm)
{
    comm->state = COMM_IDLE;
    comm->prevState = COMM_IDLE;
    comm->phaseResistanceMilliOhm = 0;
    comm->phaseInductanceMicroH = 0;
    comm->motorKv = 0;
    comm->detectedPolePairs = 0;
    comm->optimalTimingAdvDeg = 0;
    comm->sampleCount = 0;
    comm->targetSamples = 0;
    comm->accumulator = 0;
    comm->stateEntryTick = 0;
    comm->timeoutMs = 0;
    comm->validationConfidence = 0;
    comm->validationPassed = false;
}

bool COMMISSION_Start(COMMISSION_DATA_T *comm, ESC_STATE_T state, uint32_t now)
{
    if (state != ESC_IDLE)
        return false;
    if (comm->state != COMM_IDLE && comm->state != COMM_COMPLETE &&
        comm->state != COMM_ERROR)
        return false;

    COMMISSION_Init(comm);
    EnterState(comm, COMM_STATIC_R, now,
               STATIC_R_TIMEOUT_MS, STATIC_R_SAMPLES);
    return true;
}

bool COMMISSION_Update(COMMISSION_DATA_T *comm, volatile GARUDA_DATA_T *pData,
                       TELEM_RING_T *ring, uint32_t now)
{
    /* Check timeout for active states */
    if (comm->state > COMM_IDLE && comm->state < COMM_COMPLETE)
    {
        if ((now - comm->stateEntryTick) > comm->timeoutMs)
        {
            /* Timeout — check if we have enough data to proceed */
            if (comm->sampleCount < (comm->targetSamples / 2))
            {
                comm->state = COMM_ERROR;
                HAL_MC1PWMDisableOutputs();
                return false;
            }
            /* Proceed with partial data — fall through to state logic */
        }
    }

    switch (comm->state)
    {
        case COMM_IDLE:
        case COMM_COMPLETE:
        case COMM_ERROR:
            return false;

        case COMM_STATIC_R:
            /* Phase 2: DC injection and voltage/current measurement.
             * Stub: collect ADC samples and compute R = V/I */
            comm->sampleCount++;
            if (comm->sampleCount >= comm->targetSamples)
            {
                /* Placeholder: compute R from accumulated samples */
                if (comm->accumulator > 0)
                    comm->phaseResistanceMilliOhm =
                        (uint16_t)(comm->accumulator / comm->sampleCount);
                EnterState(comm, COMM_STATIC_L, now,
                           STATIC_L_TIMEOUT_MS, STATIC_L_SAMPLES);
            }
            break;

        case COMM_STATIC_L:
            /* Phase 2: Voltage step response for inductance measurement.
             * Stub: measure di/dt from step response */
            comm->sampleCount++;
            if (comm->sampleCount >= comm->targetSamples)
            {
                if (comm->accumulator > 0)
                    comm->phaseInductanceMicroH =
                        (uint16_t)(comm->accumulator / comm->sampleCount);
                EnterState(comm, COMM_DYNAMIC_KV, now,
                           DYNAMIC_KV_TIMEOUT_MS, DYNAMIC_KV_SAMPLES);
            }
            break;

        case COMM_DYNAMIC_KV:
        {
            /* Phase 2: Low-speed spin to measure back-EMF constant.
             * Drain ring buffer for BEMF samples */
            TELEM_SAMPLE_T sample;
            while (RingBuffer_Read(ring, &sample))
            {
                comm->sampleCount++;
                comm->accumulator += sample.vbusRaw;
            }
            if (comm->sampleCount >= comm->targetSamples)
            {
                /* Placeholder: Kv = RPM / V_bemf */
                EnterState(comm, COMM_DYNAMIC_POLES, now,
                           DYNAMIC_POLES_TIMEOUT_MS, DYNAMIC_POLES_SAMPLES);
            }
            break;
        }

        case COMM_DYNAMIC_POLES:
        {
            /* Phase 2: Count electrical cycles per mechanical revolution.
             * Requires hall sensor or index pulse for mech reference.
             * detectedPolePairs stays 0 until Phase 2 implements the
             * measurement; COMMISSION_ApplyResults() skips the update
             * when detectedPolePairs == 0. */
            TELEM_SAMPLE_T sample;
            (void)pData;    /* will be used by Phase 2 pole detection */
            while (RingBuffer_Read(ring, &sample))
            {
                comm->sampleCount++;
            }
            if (comm->sampleCount >= comm->targetSamples)
            {
                EnterState(comm, COMM_DYNAMIC_TIMING, now,
                           DYNAMIC_TIMING_TIMEOUT_MS, DYNAMIC_TIMING_SAMPLES);
            }
            break;
        }

        case COMM_DYNAMIC_TIMING:
        {
            /* Phase 2: Sweep timing advance to find optimal ZC alignment */
            TELEM_SAMPLE_T sample;
            while (RingBuffer_Read(ring, &sample))
            {
                comm->sampleCount++;
            }
            if (comm->sampleCount >= comm->targetSamples)
            {
                /* Placeholder: optimal timing from best ZC quality */
                EnterState(comm, COMM_VALIDATE, now,
                           VALIDATE_TIMEOUT_MS, VALIDATE_SAMPLES);
            }
            break;
        }

        case COMM_VALIDATE:
        {
            /* Run with learned params, check quality confidence */
            TELEM_SAMPLE_T sample;
            while (RingBuffer_Read(ring, &sample))
            {
                comm->sampleCount++;
            }
            if (comm->sampleCount >= comm->targetSamples)
            {
                comm->validationConfidence = pData->quality.confidenceScore;
                comm->validationPassed =
                    (comm->validationConfidence >= VALIDATE_MIN_CONFIDENCE);

                if (comm->validationPassed)
                {
                    EnterState(comm, COMM_COMMIT, now, 1000, 0);
                }
                else
                {
                    comm->state = COMM_ERROR;
                    HAL_MC1PWMDisableOutputs();
                    return false;
                }
            }
            break;
        }

        case COMM_COMMIT:
            /* Phase 2: Write results to EEPROM, set health baselines */
            comm->state = COMM_COMPLETE;
            HAL_MC1PWMDisableOutputs();
            return false;
    }

    return true;    /* still in progress */
}

void COMMISSION_Abort(COMMISSION_DATA_T *comm, volatile GARUDA_DATA_T *pData)
{
    HAL_MC1PWMDisableOutputs();
    comm->state = COMM_IDLE;
    pData->state = ESC_IDLE;
}

uint8_t COMMISSION_GetProgress(const COMMISSION_DATA_T *comm)
{
    if (comm->state >= COMM_ERROR)
        return 0;

    uint8_t base = stateProgress[comm->state];
    uint8_t next = (comm->state + 1 < COMM_ERROR)
                       ? stateProgress[comm->state + 1]
                       : 100;

    if (comm->targetSamples == 0)
        return base;

    /* Interpolate within current state */
    uint8_t range = next - base;
    uint16_t pct = (uint16_t)comm->sampleCount * range / comm->targetSamples;
    if (pct > range)
        pct = range;

    return base + (uint8_t)pct;
}

void COMMISSION_ApplyResults(const COMMISSION_DATA_T *comm,
                             GARUDA_CONFIG_T *config, ADAPT_PARAMS_T *adapt)
{
    if (comm->state != COMM_COMPLETE)
        return;

    /* Apply detected pole pairs to config */
    if (comm->detectedPolePairs > 0)
        config->motorPolePairs = comm->detectedPolePairs;

    /* Set adaptation starting point from commissioned values */
    if (comm->optimalTimingAdvDeg > 0)
        adapt->timingAdvanceDeg = comm->optimalTimingAdvDeg;

    /* Snapshot as last-known-good */
    ADAPT_SnapshotLKG(adapt);
}

#endif /* FEATURE_COMMISSION */
