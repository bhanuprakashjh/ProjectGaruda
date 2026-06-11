/**
 * @file ring_buffer.h
 *
 * @brief Lock-free SPSC telemetry ring buffer.
 *
 * Single-producer (ISR) / single-consumer (main loop) ring buffer for
 * passing TELEM_SAMPLE_T data from the commutation ISR to the learning
 * loop without disabling interrupts.
 *
 * Component: LEARN / RING BUFFER
 */

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include "../garuda_types.h"
#include "../garuda_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#if FEATURE_LEARN_MODULES

/* Ring buffer structure */
typedef struct
{
    TELEM_SAMPLE_T buffer[TELEM_RING_SIZE];
    volatile uint16_t head;         /* written by ISR (producer) */
    volatile uint16_t tail;         /* written by main loop (consumer) */
    uint16_t overflowCount;         /* diagnostic: dropped samples */
} TELEM_RING_T;

/**
 * @brief Initialize ring buffer to empty state.
 * @param ring  Pointer to ring buffer instance
 */
void RingBuffer_Init(TELEM_RING_T *ring);

/**
 * @brief Write a sample into the ring buffer (ISR context, <1us).
 * @param ring    Pointer to ring buffer instance
 * @param sample  Pointer to sample to copy in
 * @return true if written, false if buffer full (sample dropped)
 */
bool RingBuffer_Write(TELEM_RING_T *ring, const TELEM_SAMPLE_T *sample);

/**
 * @brief Read a sample from the ring buffer (main loop context).
 * @param ring    Pointer to ring buffer instance
 * @param sample  Pointer to destination for the sample
 * @return true if a sample was read, false if buffer empty
 */
bool RingBuffer_Read(TELEM_RING_T *ring, TELEM_SAMPLE_T *sample);

/**
 * @brief Get the number of samples currently in the buffer.
 * @param ring  Pointer to ring buffer instance
 * @return Number of unread samples
 */
uint16_t RingBuffer_Count(const TELEM_RING_T *ring);

#endif /* FEATURE_LEARN_MODULES */

#ifdef __cplusplus
}
#endif

#endif /* RING_BUFFER_H */
