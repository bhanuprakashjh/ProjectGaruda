/**
 * @file garuda_config.h
 *
 * @brief User-configurable parameters for Project Garuda ESC firmware.
 * These are the "knobs" — change these to tune the ESC for your motor/setup.
 *
 * Component: CONFIGURATION
 */

#ifndef GARUDA_CONFIG_H
#define GARUDA_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Feature Flags (0=disabled, 1=enabled) */
#define FEATURE_LEARN_MODULES   0   /* master: ring buffer + quality + health */
#define FEATURE_ADAPTATION      0   /* requires FEATURE_LEARN_MODULES */
#define FEATURE_COMMISSION      0   /* requires FEATURE_LEARN_MODULES */
#define FEATURE_EEPROM_V2       0   /* requires at least one above */

/* Diagnostic: Manual step mode (1=enabled)
 * SW1: Start motor → align to step 0
 * SW2: Manually advance one commutation step
 * LED2: Toggles on each step advance
 * No automatic ramp — user controls step timing.
 * Set to 0 for normal auto-ramp operation. */
#define DIAGNOSTIC_MANUAL_STEP  0

/* Learn module tuning (only compiled when FEATURE_LEARN_MODULES=1) */
#if FEATURE_LEARN_MODULES
#define TELEM_RING_SIZE             64      /* power of 2, 64*13=832B RAM */
#define TELEM_RING_MASK             (TELEM_RING_SIZE - 1)
#define QUALITY_WINDOW_MS           1000
#define QUALITY_UPDATE_DIVIDER      1       /* every 1ms */
#define HEALTH_UPDATE_DIVIDER       10      /* every 10ms */
#define ADAPT_EVAL_DIVIDER          100     /* every 100ms */
#define TIMING_ADVANCE_MIN_DEG      0
#define TIMING_ADVANCE_MAX_DEG      30
#define ADAPT_MIN_CONFIDENCE        180
#define ADAPT_MAX_CONSECUTIVE_FAIL  3
#define ADAPT_MAX_ROLLBACK_TOTAL    10
#endif

/* PWM Configuration */
#define PWMFREQUENCY_HZ            24000       /* PWM switching frequency */
#define DEADTIME_NS                750         /* Dead time in nanoseconds */

/* Motor Configuration */
#define MOTOR_POLE_PAIRS           5           /* Hurst DMB0224C10002: 10 poles = 5 pairs */
#define DIRECTION_DEFAULT          0           /* 0=CW, 1=CCW */

/* Startup / Alignment */
#define ALIGN_TIME_MS              500         /* Time to hold alignment position */
#define ALIGN_DUTY_PERCENT         20          /* Duty cycle during alignment (must > MIN_DUTY) */

/* Open-Loop Ramp */
#define INITIAL_ERPM               300         /* ~5 steps/sec — slow start */
#define RAMP_TARGET_ERPM           5000        /* 1000 mech RPM (5 pole pairs) */
#define RAMP_ACCEL_ERPM_PER_S      1000        /* Moderate acceleration */
#define RAMP_DUTY_PERCENT          40          /* Duty cap during open-loop ramp */

/* Arming */
#define ARM_TIME_MS                500         /* Throttle must be zero for this long to arm */

/* Bus Voltage */
#define VBUS_OVERVOLTAGE_ADC       3600        /* ~52V with typical divider (tunable) */
#define VBUS_UNDERVOLTAGE_ADC      500         /* ~7V with typical divider (tunable) */

/* Comparator DAC reference for overcurrent fault (from reference) */
#define CMP_REF_DCBUS_FAULT        2048        /* Default DAC reference, midscale */

#ifdef __cplusplus
}
#endif

#endif /* GARUDA_CONFIG_H */
