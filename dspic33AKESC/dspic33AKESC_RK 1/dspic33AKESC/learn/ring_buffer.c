/**
 * @file ring_buffer.c
 *
 * @brief Lock-free SPSC telemetry ring buffer implementation.
 *
 * Power-of-2 sizing with mask wrapping. No interrupt disable needed —
 * head is only written by the ISR producer, tail only by the main loop
 * consumer. On overflow, newest sample is dropped.
 *
 * Component: LEARN / RING BUFFER
 */

#include "ring_buffer.h"

#if FEATURE_LEARN_MODULES

#include <string.h>

void RingBuffer_Init(TELEM_RING_T *ring)
{
    ring->head = 0;
    ring->tail = 0;
    ring->overflowCount = 0;
    memset(ring->buffer, 0, sizeof(ring->buffer));
}

bool RingBuffer_Write(TELEM_RING_T *ring, const TELEM_SAMPLE_T *sample)
{
    uint16_t nextHead = (ring->head + 1) & TELEM_RING_MASK;

    if (nextHead == ring->tail)
    {
        /* Buffer full — drop newest sample */
        ring->overflowCount++;
        return false;
    }

    memcpy(&ring->buffer[ring->head], sample, sizeof(TELEM_SAMPLE_T));
    ring->head = nextHead;
    return true;
}

bool RingBuffer_Read(TELEM_RING_T *ring, TELEM_SAMPLE_T *sample)
{
    if (ring->head == ring->tail)
    {
        return false;   /* empty */
    }

    memcpy(sample, &ring->buffer[ring->tail], sizeof(TELEM_SAMPLE_T));
    ring->tail = (ring->tail + 1) & TELEM_RING_MASK;
    return true;
}

uint16_t RingBuffer_Count(const TELEM_RING_T *ring)
{
    return (ring->head - ring->tail) & TELEM_RING_MASK;
}

#endif /* FEATURE_LEARN_MODULES */
