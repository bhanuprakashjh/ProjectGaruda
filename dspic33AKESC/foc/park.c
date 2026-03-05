/**
 * @file park.c
 * @brief Park and Inverse Park using dsPIC33AK hardware FPU sinf/cosf.
 */

#include "park.h"
#include <math.h>   /* sinf, cosf — mapped to FSIN/FCOS instructions by XC-DSC */

void park_transform(const AlphaBeta_t *ab, float theta, DQ_t *dq)
{
    float s = sinf(theta);
    float c = cosf(theta);
    dq->d =  ab->alpha * c + ab->beta * s;
    dq->q = -ab->alpha * s + ab->beta * c;
}

void inv_park_transform(const DQ_t *dq, float theta, AlphaBeta_t *ab)
{
    float s = sinf(theta);
    float c = cosf(theta);
    ab->alpha = dq->d * c - dq->q * s;
    ab->beta  = dq->d * s + dq->q * c;
}
