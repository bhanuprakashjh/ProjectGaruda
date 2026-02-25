/**
 * @file rx_decode.h
 *
 * @brief RX input decode: seqlock mailbox consumer, PWM/DShot decode,
 * lock FSM, cached throttle for ADC ISR mux.
 *
 * RX_Service() runs in main.c while(1) loop (not ISR).
 *
 * Component: INPUT
 */

#ifndef RX_DECODE_H
#define RX_DECODE_H

#include <stdint.h>
#include "garuda_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#if (FEATURE_RX_PWM || FEATURE_RX_DSHOT || FEATURE_RX_AUTO)

/* Seqlock mailbox structure (ISR writes, RX_Service reads) */
typedef struct {
    volatile uint16_t seqNum;   /* odd = writing, even = complete */
    uint16_t value;             /* pulse width (PWM us) or throttle 0-2047 (DShot) */
    uint8_t  valid;             /* 1 = CRC ok / width in range */
    uint8_t  pad0;
} RX_MAILBOX_T;

/* Mailbox — declared in hal_input_capture.c, extern here */
extern volatile RX_MAILBOX_T rxMailbox;

/* Cached throttle for ADC ISR consumption (write-ordered by RX_Service).
 * ADC ISR reads rxCachedLocked FIRST, then rxCachedThrottleAdc. */
extern volatile uint16_t rxCachedThrottleAdc;  /* 0-4095 scaled */
extern volatile uint8_t  rxCachedLocked;       /* 0 or 1 */

/**
 * @brief Initialize RX decode state. Called once at startup.
 */
void RX_Init(void);

/**
 * @brief Main-loop RX service: read mailbox, decode, lock FSM, cache.
 * Call from main.c while(1) loop alongside GSP_Service()/BoardService().
 */
void RX_Service(void);

#endif /* FEATURE_RX_PWM || FEATURE_RX_DSHOT || FEATURE_RX_AUTO */

#ifdef __cplusplus
}
#endif

#endif /* RX_DECODE_H */
