/**
 * @file gsp_ak.h
 *
 * @brief Garuda Serial Protocol (GSP) v1 for the MCLV high-speed sensorless
 * trapezoidal reference — makes this firmware speak to the garuda-gui
 * (React/Web-Serial) exactly like the Garuda AK firmware does.
 *
 * Protocol: [0x02][LEN][CMD][PAYLOAD...][CRC16H][CRC16L] over UART1 @115.2k
 * (PKoB4 CDC).  CRC-16-CCITT (0x1021, init 0xFFFF) over [LEN][CMD][PAYLOAD].
 *
 * MUTUALLY EXCLUSIVE with X2CScope (both own UART1).  Enabled by
 * ENABLE_GSP in mc1_user_params.h — main.c swaps DiagnosticsInit/StepMain
 * for GSP_Init/GSP_Service and #includes gsp_ak.c (single-TU, so no MPLAB
 * project-file changes are needed).
 *
 * Commands implemented (garuda-gui contract):
 *   0x00 PING            0x01 GET_INFO         0x02 GET_SNAPSHOT
 *   0x03 START_MOTOR     0x04 STOP_MOTOR       0x05 CLEAR_FAULT
 *   0x06 SET_THROTTLE    0x07 SET_THROTTLE_SRC 0x08 HEARTBEAT
 *   0x14 TELEM_START     0x15 TELEM_STOP       0x16 GET_PARAM_LIST (empty)
 *   0x80 TELEM_FRAME (streamed)                everything else -> ERROR
 *
 * Component: GSP (port)
 */

#ifndef GSP_AK_H
#define GSP_AK_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Init UART1 for GSP (115.2k) + reset rings/parser. Call once from main(). */
void GSP_Init(void);

/* Poll-driven service: drain RX, parse, dispatch, stream telemetry, pump TX.
 * Call every main-loop iteration. */
void GSP_Service(void);

/* Millisecond timebase + GSP failsafe. Call from the PWM-rate ADC ISR
 * (MC1_ADC_INTERRUPT, 40 kHz); divides down to 1 ms internally. */
void GSP_IsrTick(void);

/* Throttle bridge: called from MCAPP_MC1ReceivedDataProcess with the pot
 * value; returns the pot unchanged in ADC mode, or the GSP throttle
 * rescaled to ADC counts (0..4096) in GSP mode. */
float GSP_ThrottleTarget(float potValue);

/* Accepted-commutation counter — defined in mc1_service.c (always compiled),
 * incremented by estim_bemf.c on every sector change, reported as
 * goodZcCount in the snapshot. */
extern volatile uint16_t g_gspZcCount;

/* Accessors provided by mc1_service.c */
uint16_t MC1_GspAppState(void);
void     MC1_GspClearFault(void);
struct MC1APP_DATA_T_TAG;   /* opaque here; gsp_ak.c sees the real type */

#ifdef __cplusplus
}
#endif

#endif /* GSP_AK_H */
