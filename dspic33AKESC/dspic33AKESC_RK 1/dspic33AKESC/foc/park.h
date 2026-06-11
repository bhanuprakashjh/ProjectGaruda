/**
 * @file park.h
 * @brief Park (αβ→dq) and Inverse Park (dq→αβ) transforms.
 *
 * Forward Park:
 *   Id =  Iα·cos(θ) + Iβ·sin(θ)
 *   Iq = −Iα·sin(θ) + Iβ·cos(θ)
 *
 * Inverse Park:
 *   Vα = Vd·cos(θ) − Vq·sin(θ)
 *   Vβ = Vd·sin(θ) + Vq·cos(θ)
 *
 * sinf/cosf are native HW FPU instructions on dsPIC33AK128MC106.
 * Both functions share one sin/cos pair (2 hardware calls per
 * transform, not 4).
 */

#ifndef PARK_H
#define PARK_H

#include "foc_types.h"

/**
 * @brief Park transform: stationary αβ → rotating dq.
 * @param ab     Iα, Iβ (A)
 * @param theta  Electrical angle (rad)
 * @param dq     Output: Id, Iq (A)
 */
void park_transform(const AlphaBeta_t *ab, float theta, DQ_t *dq);

/**
 * @brief Inverse Park transform: rotating dq → stationary αβ.
 * @param dq     Vd, Vq (V)
 * @param theta  Electrical angle (rad)
 * @param ab     Output: Vα, Vβ (V)
 */
void inv_park_transform(const DQ_t *dq, float theta, AlphaBeta_t *ab);

#endif /* PARK_H */
