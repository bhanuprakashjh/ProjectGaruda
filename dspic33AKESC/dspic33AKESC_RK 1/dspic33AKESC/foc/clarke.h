/**
 * @file clarke.h
 * @brief Clarke transform: abc → αβ (amplitude-invariant form)
 *
 *   Iα = Ia
 *   Iβ = (Ia + 2·Ib) / √3
 *
 * Assumes Kirchhoff: Ia + Ib + Ic = 0, so Ic is not needed.
 * No trigonometry — two multiplies, zero sinf/cosf calls.
 */

#ifndef CLARKE_H
#define CLARKE_H

#include "foc_types.h"

/**
 * @brief Clarke transform.
 * @param ia   Phase A current (A)
 * @param ib   Phase B current (A)
 * @param ab   Output: Iα, Iβ (A)
 */
void clarke_transform(float ia, float ib, AlphaBeta_t *ab);

#endif /* CLARKE_H */
