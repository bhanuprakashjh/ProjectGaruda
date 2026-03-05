/**
 * @file clarke.c
 * @brief Amplitude-invariant Clarke transform.
 *
 * 1/√3 literal avoids a runtime reciprocal; FPU executes
 * the two float multiplies in 2 cycles each.
 */

#include "clarke.h"

#define ONE_OVER_SQRT3  0.57735026918962576451f   /* 1/√3 */

void clarke_transform(float ia, float ib, AlphaBeta_t *ab)
{
    ab->alpha = ia;
    ab->beta  = (ia + 2.0f * ib) * ONE_OVER_SQRT3;
}
