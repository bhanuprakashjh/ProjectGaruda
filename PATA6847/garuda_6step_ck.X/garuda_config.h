/**
 * @file garuda_config.h
 * @brief Configuration for 6-step BLDC on dsPIC33CK + ATA6847.
 *
 * Target: EV43F54A board (dsPIC33CK64MP205 + ATA6847)
 * Motor:  Hurst DMB2424B10002 (Long Hurst), 10 poles, 24V
 */

#ifndef GARUDA_CONFIG_H
#define GARUDA_CONFIG_H

/* ── Clock ─────────────────────────────────────────────────────────── */
#define FOSC                200000000UL  /* 200 MHz */
#define FCY                 100000000UL  /* 100 MHz instruction clock */
#define FOSC_PWM_MHZ        400U         /* PWM timing base (CK: 2x Fosc) */

/* ── PWM ───────────────────────────────────────────────────────────── */
#define PWMFREQUENCY_HZ     20000U       /* 20 kHz switching */
#define LOOPTIME_MICROSEC   (uint16_t)(1000000UL / PWMFREQUENCY_HZ)  /* 50 us */
#define LOOPTIME_TCY        (uint16_t)((uint32_t)LOOPTIME_MICROSEC * FOSC_PWM_MHZ / 2 - 1)
#define MAX_DUTY            (LOOPTIME_TCY - 200U)  /* Min off-time margin */
#define MIN_DUTY            200U
/* Dead time: ATA6847 handles 700ns CCPT internally, so dsPIC dead time
 * is minimal. Reference firmware uses 0x14 = 20 counts = 50ns. */
#define DEADTIME_TCY        0x14U

/* ── Timer1 (50 µs tick) ───────────────────────────────────────────── */
#define TIMER1_PRESCALE     8U
#define TIMER1_FREQ_HZ      20000U       /* 50 µs period — matches ADC ISR rate */
#define TIMER1_PR           (uint16_t)(FCY / TIMER1_PRESCALE / TIMER1_FREQ_HZ - 1)

/* ── Motor: Hurst Long (DMB2424B10002) ─────────────────────────────── */
#define MOTOR_POLE_PAIRS    5U
#define MOTOR_RS_MILLIOHM   534U         /* Phase resistance */
#define MOTOR_LS_MICROH     471U         /* Phase inductance */
#define MOTOR_KV            149U         /* RPM/V */

/* ── Startup ───────────────────────────────────────────────────────── */
#define ALIGN_TIME_MS       200U
#define ALIGN_TIME_COUNTS   ((uint16_t)((uint32_t)ALIGN_TIME_MS * TIMER1_FREQ_HZ / 1000))
#define ALIGN_DUTY          (LOOPTIME_TCY / 20)    /* ~5% duty (~2.2A on Hurst) */

#define INITIAL_STEP_PERIOD 1000U        /* Timer1 ticks (50 ms per step = ~200 eRPM) */
#define MIN_STEP_PERIOD     66U          /* Timer1 ticks (3.3 ms per step = ~3000 eRPM) */
#define RAMP_ACCEL_ERPM_S   1500U        /* eRPM/s acceleration (6-step OL limited by duty cap) */
#define RAMP_DUTY_CAP       (LOOPTIME_TCY / 6)     /* Max ~17% duty during OL ramp */

/* ── BEMF ZC Detection (ATA6847 digital comparators) ───────────────── */
#define ZC_BLANKING_PERCENT 20U          /* % of step period to blank after commutation */
#define ZC_FILTER_THRESHOLD 3U           /* Net matching reads to confirm ZC (bounce-tolerant) */
#define ZC_SYNC_THRESHOLD   6U           /* Good ZC count to enter post-sync */
#define ZC_TIMEOUT_MULT     3U           /* Forced step at 3x step period (generous) */
#define ZC_DESYNC_THRESH    3U           /* Consecutive misses before clearing zcSynced */
#define ZC_MISS_LIMIT       12U          /* Consecutive misses → desync fault */

/* ── Timing Advance (matches dspic33AKESC 6-step implementation) ───── */
/* Linear eRPM-based advance: 0° below ADVANCE_START_ERPM, linearly ramps to
 * MAX degrees at MAX_CLOSED_LOOP_ERPM. Compensates for winding inductance.
 * Delay formula: stepPeriod * (30 - advDeg) / 60.
 * eRPM = TIMER1_FREQ_HZ * 10 / stepPeriod.
 *
 * Note: ZC filter adds ~3 ticks systematic delay to lastZcTick.
 * At max speed (Tp:11), total delay from real ZC = 3 + sp*(30-adv)/60.
 * With 8° max: 3 + 11*22/60 = 7 ticks. HalfStep = 5.5. OK margin.
 * With 15° max: 3 + 11*15/60 = 5.75 — too tight at Tp:13 (3.25 ticks). */
#define TIMING_ADVANCE_MIN_DEG      0U   /* Degrees advance at low speed */
#define TIMING_ADVANCE_MAX_DEG      8U   /* Degrees advance at max speed */
#define TIMING_ADVANCE_START_ERPM   6000U /* eRPM where advance begins (~1200 mech RPM) */

/* ── Closed-Loop ───────────────────────────────────────────────────── */
#define MAX_CLOSED_LOOP_ERPM 15000U
#define RAMP_TARGET_ERPM     3000U       /* OL→CL handoff speed */
#define CL_IDLE_DUTY_PERCENT 8U          /* Minimum duty in CL (idle floor) */
#define CL_IDLE_DUTY         (((uint32_t)CL_IDLE_DUTY_PERCENT * LOOPTIME_TCY) / 100)

/* ── Vbus Fault Thresholds (16-bit fractional ADC, KVbus=16) ────────── */
/* dsPIC33CK ADC outputs 12-bit left-justified in 16-bit → multiply 12-bit
 * thresholds by 16 to compare against raw ADCBUF values */
#define VBUS_OV_THRESHOLD   (3200U * 16U)  /* ~30V → 51200 in 16-bit */
#define VBUS_UV_THRESHOLD   (700U * 16U)   /* ~7V  → 11200 in 16-bit */

/* ── Desync Recovery ───────────────────────────────────────────────── */
#define DESYNC_RESTART_MAX  3U
#define RECOVERY_TIME_MS    200U
#define RECOVERY_COUNTS     ((uint16_t)((uint32_t)RECOVERY_TIME_MS * TIMER1_FREQ_HZ / 1000))

/* ── ARM ───────────────────────────────────────────────────────────── */
#define ARM_TIME_MS         200U
#define ARM_TIME_COUNTS     ((uint16_t)((uint32_t)ARM_TIME_MS * TIMER1_FREQ_HZ / 1000))

/* ── ADC Channel Mapping (dsPIC33CK on EV43F54A) ──────────────────── */
/* AN6 = Potentiometer (speed ref), AN9 = Vbus voltage */
#define ADCBUF_POT          ADCBUF6
#define ADCBUF_VBUS         ADCBUF9

#endif /* GARUDA_CONFIG_H */
