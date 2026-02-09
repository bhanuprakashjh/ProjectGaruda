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

/* PWM Configuration */
#define PWMFREQUENCY_HZ            24000       /* PWM switching frequency */
#define DEADTIME_NS                750         /* Dead time in nanoseconds */

/* Motor Configuration */
#define MOTOR_POLE_PAIRS           7
#define DIRECTION_DEFAULT          0           /* 0=CW, 1=CCW */

/* Startup / Alignment */
#define ALIGN_TIME_MS              200         /* Time to hold alignment position */
#define ALIGN_DUTY_PERCENT         5           /* Duty cycle during alignment */

/* Open-Loop Ramp */
#define INITIAL_ERPM               2000        /* Starting eRPM for forced commutation */
#define RAMP_TARGET_ERPM           15000       /* eRPM at which to transition to closed-loop */
#define RAMP_ACCEL_ERPM_PER_S      5000        /* Ramp acceleration rate */

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
