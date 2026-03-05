/**
 * @file smo_observer.h
 * @brief Sliding Mode Observer (SMO) for sensorless PMSM back-EMF estimation.
 *
 * Implements a current-model SMO with sigmoid switching function
 * for chattering reduction. Runs in parallel with the voltage-model
 * observer for comparison and robustness evaluation.
 *
 * Algorithm (discrete-time, forward Euler):
 *   err = i_hat - i_measured
 *   Z   = k * sigmoid(err)        (switching term)
 *   i_hat[k+1] = A*i_hat + B*(V - Z)   (current observer)
 *   e_est = LPF(Z)                (back-EMF extraction)
 *
 * When the observer converges (err -> 0), Z converges to the
 * actual back-EMF. The sigmoid function (x/(|x|+phi)) replaces
 * the classic signum to eliminate high-frequency chattering while
 * preserving the sliding mode convergence property.
 *
 * Key advantage over voltage-model observer:
 *   - Robust to Rs/Ls parameter errors (within bounded uncertainty)
 *   - No di/dt computation (avoids noise amplification from backward difference)
 *   - Finite-time convergence via sliding mode theory
 *
 * Component: FOC
 */

#ifndef SMO_OBSERVER_H
#define SMO_OBSERVER_H

#include <stdint.h>

typedef struct {
    float i_hat_alpha;     /* Estimated current alpha (A) */
    float i_hat_beta;      /* Estimated current beta  (A) */
    float e_alpha;         /* Filtered back-EMF alpha (V) -- output to PLL */
    float e_beta;          /* Filtered back-EMF beta  (V) -- output to PLL */
    float z_alpha;         /* Raw switching signal alpha (V) -- pre-filter */
    float z_beta;          /* Raw switching signal beta  (V) -- pre-filter */
} SMO_t;

/** @brief Reset observer state to zero (call on startup/fault clear). */
void smo_reset(SMO_t *smo);

/**
 * @brief Run one SMO step (call every fast-loop tick).
 * @param smo      Observer state
 * @param v_alpha  Applied voltage alpha (V) -- from inverse Park output
 * @param v_beta   Applied voltage beta  (V)
 * @param i_alpha  Measured current alpha (A) -- from Clarke output
 * @param i_beta   Measured current beta  (A)
 */
void smo_update(SMO_t *smo,
                float v_alpha, float v_beta,
                float i_alpha, float i_beta);

#endif /* SMO_OBSERVER_H */
