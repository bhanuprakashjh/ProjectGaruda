/**
 * @file svpwm.h
 * @brief Space-Vector PWM — αβ voltage reference → 3-phase duty cycles.
 *
 * Min-max (mid-clamp) injection method: inverse Clarke → neutral shift → duty.
 * Mathematically equivalent to 6-sector T1/T2 but simpler and bug-free.
 *
 * Linear modulation limit: |V| ≤ Vbus / √3 ≈ 0.577·Vbus (hexagon inscribed circle).
 * Overmodulation is handled naturally by output clamping.
 *
 * Inputs:
 *   v_alpha, v_beta  — voltage reference in stationary frame (V)
 *   vbus             — DC bus voltage (V)
 *
 * Outputs:
 *   duty_a/b/c       — normalized duties [0.0, 1.0] for center-aligned PWM
 */

#ifndef SVPWM_H
#define SVPWM_H

/**
 * @brief Compute SVPWM duty cycles.
 * @param v_alpha  Vα reference (V)
 * @param v_beta   Vβ reference (V)
 * @param vbus     DC bus voltage (V)
 * @param duty_a   Phase A duty [0.0, 1.0]
 * @param duty_b   Phase B duty [0.0, 1.0]
 * @param duty_c   Phase C duty [0.0, 1.0]
 */
void svpwm_update(float v_alpha, float v_beta, float vbus,
                  float *duty_a, float *duty_b, float *duty_c);

#endif /* SVPWM_H */
