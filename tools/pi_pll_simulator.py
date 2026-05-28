#!/usr/bin/env python3
"""
PI/PLL Simulator for Garuda ESC Sector PI Controller

Models the sector PI from motor/hwzc.c as a discrete-time PLL acting
on a simplified BLDC plant. Lets you compare the current integer
firmware implementation against proposed float-based variants
(feedforward, velocity form, etc.) without bench time.

LIMITATIONS:
  - Plant uses "ideal commutation" assumption — controller's commanded
    sector does NOT feed back into rotor torque. Real firmware
    bench-validates the wrong-angle penalty separately; modeling that
    here makes the sim oscillate badly until the controller locks,
    obscuring the controller dynamics we actually want to see.
  - ZC events fire at true rotor sector midpoints (with filter delay
    and noise) regardless of what the controller is doing — so the
    PI's "phase error" reflects only its own clock vs rotor truth.
  - No model of phase current ripple from PWM. PWM-noise effects on
    captures are approximated by the noise_std_HR knob on the
    EventDetector.

This is enough to study:
  - Lock acquisition speed
  - Step response to throttle change
  - Disturbance rejection (load step)
  - Noise rejection (high σ + missed captures)
  - Feedforward vs no-feedforward
  - Float vs integer (no behavior diff expected — sanity check)

Companion to:
  - docs/pi_controller_research.md
  - docs/bemf_filter_compensation.md
  - docs/pwm_frequency_effects.md

Usage:
  python3 pi_pll_simulator.py                                # all scenarios, show
  python3 pi_pll_simulator.py --scenario lock                # one scenario
  python3 pi_pll_simulator.py --save /tmp/sim                # save PNG, no GUI

Requires: numpy, matplotlib
"""

import os
# Allow running without a DISPLAY when saving plots
if "--save" in os.sys.argv:
    os.environ.setdefault("MPLBACKEND", "Agg")

import argparse
from dataclasses import dataclass, field
from typing import Callable, List, Tuple

import matplotlib.pyplot as plt
import numpy as np

# ----------------------------------------------------------------------
# Physical constants (must match firmware)
# ----------------------------------------------------------------------
HR_TICK_HZ = 100_000_000      # 100 MHz SCCP2 timer
NS_PER_TICK = 1e9 / HR_TICK_HZ
SECTORS_PER_REV = 6

# ----------------------------------------------------------------------
# Motor and filter parameters
# ----------------------------------------------------------------------
@dataclass
class MotorParams:
    """BLDC motor parameters. Defaults mirror 2810 1350KV from gsp_params.c."""
    name: str = "2810 1350KV"
    KV: float = 1350           # RPM per volt
    pole_pairs: int = 7
    Rs_ohm: float = 0.022      # phase resistance
    Ls_uH: float = 25          # phase inductance
    J_kgm2: float = 1.5e-6     # rotor moment of inertia (estimated)
    B_Nms: float = 5e-6        # viscous friction (estimated)

    @property
    def ke_Vs_per_rad(self) -> float:
        """Back-EMF constant (V·s/rad mechanical) = 60/(2π·KV)."""
        return 60.0 / (2.0 * np.pi * self.KV)

    @property
    def kt_Nm_per_A(self) -> float:
        """Torque constant — equals ke in SI for ideal motor."""
        return self.ke_Vs_per_rad

@dataclass
class FilterParams:
    """PCB BEMF RC low-pass filter."""
    R_ohm: float = 3000.0
    C_F: float = 10e-9

    @property
    def tau_s(self) -> float:
        return self.R_ohm * self.C_F

    def phase_lag_rad(self, omega_elec: float) -> float:
        return np.arctan(omega_elec * self.tau_s)

    def time_delay_s(self, omega_elec: float) -> float:
        if omega_elec < 1e-9:
            return 0.0
        return self.phase_lag_rad(omega_elec) / omega_elec

# ----------------------------------------------------------------------
# Motor plant simulator
# ----------------------------------------------------------------------
class MotorPlant:
    """
    Simplified BLDC plant.

    State: (theta_elec, omega_elec)
    Input: applied voltage (Vbus × duty), commanded commutation sector.
    Output: rotor mechanical angle / velocity, BEMF on floating phase.

    Torque model: T_em = kt × I_phase × cos(error_angle)
      where error_angle = commanded_sector_midpoint − rotor_angle
      This captures the dominant 6-step phenomenon: torque drops as
      commutation drifts from the ideal angle.
    """

    def __init__(self, motor: MotorParams, Vbus: float = 24.0):
        self.motor = motor
        self.Vbus = Vbus
        self.theta_rotor = 0.0          # absolute rotor electrical angle [0, 2π)
        self.total_rotation = 0.0       # cumulative rotation, for ZC event counting
        self.omega_elec = 0.0
        self.duty = 0.0
        self.load_Nm = 0.0
        self.commanded_sector = 0
        self.theta_commanded = np.pi / 6
        # Diagnostics
        self.last_lead_rad = 0.0
        self.last_torque_eff = 1.0
        self.last_I_phase = 0.0

    def on_commutation(self):
        """Bridge advances to next sector. Rotor angle is unchanged
        (rotor evolves continuously per physics; bridge just changes
        which phases are energized)."""
        self.theta_commanded = (self.theta_commanded + np.pi / 3) % (2 * np.pi)
        self._zc_emitted_this_sector = False

    def step(self, dt: float):
        """
        Realistic BLDC plant with commutation-phase-angle torque penalty.

          - Rotor angle evolves continuously at omega_elec
          - Bridge commands a target angle (commanded_sector midpoint)
          - Torque efficiency = cos(lead), where lead = θ_commanded − θ_rotor
            (signed, wrapped to [-π, π])
          - Best torque at lead = 0 (rotor exactly where bridge expects)
          - Torque drops as |lead| grows: 0 at ±π/2, braking beyond
          - Phase current still drawn even at 0 torque (I²R loss only)
          - Soft current limit at 22 A mirrors firmware OC_SW

        This captures the failure mode bench saw on 2026-05-28: when the
        controller commands sectors faster than the rotor can follow,
        lead grows past π/2, torque collapses, current pegs at 22 A,
        rotor stays stationary.
        """
        V_app = self.Vbus * self.duty
        omega_mech = self.omega_elec / self.motor.pole_pairs
        E_phase = self.motor.ke_Vs_per_rad * omega_mech

        # Phase current
        I_phase = (V_app - E_phase) / self.motor.Rs_ohm
        I_LIMIT = 22.0
        if I_phase >  I_LIMIT: I_phase =  I_LIMIT
        if I_phase < -I_LIMIT: I_phase = -I_LIMIT
        self.last_I_phase = I_phase

        # Lead angle: commanded − rotor, wrapped to [-π, π]
        lead = self.theta_commanded - self.theta_rotor
        while lead >  np.pi: lead -= 2 * np.pi
        while lead < -np.pi: lead += 2 * np.pi
        self.last_lead_rad = lead

        # Torque efficiency = cos(lead). Smooth, captures wrong-angle stall.
        torque_eff = np.cos(lead)
        self.last_torque_eff = torque_eff

        T_em = self.motor.kt_Nm_per_A * I_phase * torque_eff
        T_net = T_em - self.load_Nm - self.motor.B_Nms * omega_mech
        domega_mech = T_net / self.motor.J_kgm2

        # Update omega (Euler)
        self.omega_elec += domega_mech * self.motor.pole_pairs * dt
        if self.omega_elec < 0:
            self.omega_elec = 0
        # Soft cap at no-load BEMF ceiling × 1.1
        omega_cap = self.motor.KV * self.Vbus * 2 * np.pi / 60 * self.motor.pole_pairs * 1.1
        if self.omega_elec > omega_cap:
            self.omega_elec = omega_cap

        # Advance absolute rotor angle (continuous)
        dtheta = self.omega_elec * dt
        self.theta_rotor = (self.theta_rotor + dtheta) % (2 * np.pi)
        self.total_rotation += dtheta

    @property
    def theta_elec(self) -> float:
        """Compatibility shim for older code/event-detector references."""
        return self.theta_rotor

    @property
    def eRPM(self) -> float:
        return self.omega_elec * 60.0 / (2.0 * np.pi)

    @property
    def rotor_sector(self) -> int:
        return int(self.theta_elec / (np.pi / 3)) % 6

# ----------------------------------------------------------------------
# Event detection (sector boundaries, ZC captures)
# ----------------------------------------------------------------------
class EventDetector:
    """
    Watches the plant for:
      - Rotor crossing a sector boundary (every 60° elec)
      - True ZC at sector midpoint (30° from sector start)

    The ZC event has filter delay + measurement noise + occasional miss.
    """

    def __init__(self, plant: MotorPlant, filt: FilterParams,
                 noise_std_HR: float = 5.0,
                 miss_probability: float = 0.0,
                 advance_deg: float = 15.0,
                 rng_seed: int = 42):
        self.plant = plant
        self.filt = filt
        self.noise_std_HR = noise_std_HR
        self.miss_prob = miss_probability
        # ZC fires every π/3 of rotor rotation, at offset (30+advance)°
        # into each sector. With advance=0°, ZC is at the sector midpoint;
        # with advance=15° it's at 45° in.
        self.zc_fire_rad = (advance_deg + 30.0) * np.pi / 180.0
        self.rng = np.random.default_rng(rng_seed)
        self._prev_sector = plant.commanded_sector
        # Count how many ZC events the rotor has passed through cumulatively.
        # ZC #N happens when total_rotation crosses zc_fire_rad + N×(π/3).
        self._last_zc_index = 0

    def check(self, t_now_s: float, dt_s: float = 0.0) -> Tuple[bool, bool, float, int]:
        """
        Returns (rotor_sector_changed, zc_captured, zc_HR_time, new_rotor_sector).

        ZC time is linearly interpolated within the dt step to remove the
        sampling-time bias that would otherwise show up as ~dt/2 systematic
        lateness on every capture.
        """
        rotor_changed = False
        zc_captured = False
        zc_HR = 0.0

        cur_sector = self.plant.commanded_sector
        if cur_sector != self._prev_sector:
            rotor_changed = True
            self._prev_sector = cur_sector

        # ZC events occur every π/3 of cumulative rotor rotation, at offset
        # zc_fire_rad. Crossing detection is wrap-safe via the cumulative
        # total_rotation counter.
        cur_index = int((self.plant.total_rotation - self.zc_fire_rad)
                        / (np.pi / 3)) if self.plant.total_rotation >= self.zc_fire_rad else -1

        if cur_index > self._last_zc_index:
            self._last_zc_index = cur_index
            if self.rng.uniform() > self.miss_prob:
                # Linear interpolation: where in this dt step did we cross?
                # Total rotation went from (total - dtheta) to total.
                # ZC at total = zc_fire_rad + cur_index × π/3.
                target_rot = self.zc_fire_rad + cur_index * (np.pi / 3)
                dtheta_step = self.plant.omega_elec * dt_s
                if dtheta_step > 1e-12:
                    frac = (target_rot - (self.plant.total_rotation - dtheta_step)) / dtheta_step
                    frac = max(0.0, min(1.0, frac))
                else:
                    frac = 1.0
                t_cross_s = (t_now_s - dt_s) + frac * dt_s

                # Filter delay assumed compensated by firmware CMPLO pre-distortion
                delay_s = 0.0
                noise_s = self.rng.normal(0, self.noise_std_HR * NS_PER_TICK / 1e9)
                cap_t_s = t_cross_s + delay_s + noise_s
                zc_HR = cap_t_s * HR_TICK_HZ
                zc_captured = True

        return rotor_changed, zc_captured, zc_HR, cur_sector

# ----------------------------------------------------------------------
# Controllers
# ----------------------------------------------------------------------
class ControllerBase:
    """Base class for sector PI variants."""
    def __init__(self, name: str):
        self.name = name
        self.timer_period_HR = 0.0
        self.integrator_HR = 0.0
        self.commanded_sector = 0
        self.last_comm_HR = 0.0
        self.last_capture_HR = 0.0
        self.capture_valid = False

        # Logs for plotting
        self.log_t = []
        self.log_timer_period = []
        self.log_delta = []
        self.log_eRPM_est = []
        self.log_commanded_sector = []

    def seed(self, initial_period_HR: float, t_now_HR: float):
        self.timer_period_HR = initial_period_HR
        self.integrator_HR = initial_period_HR
        self.last_comm_HR = t_now_HR

    def on_capture(self, zc_HR: float):
        """Accept only the FIRST capture per inter-commutation interval.
        Mirrors firmware's blank-then-arm semantics: after a commutation,
        the HW comparator is disabled, then re-armed; the first edge
        becomes the valid capture for that sector. Subsequent edges
        (from rotor running ahead of controller) are ignored."""
        if not self.capture_valid:
            self.last_capture_HR = zc_HR
            self.capture_valid = True

    def on_commutation(self, t_now_HR: float) -> int:
        """Run PI math; advance commanded sector; return new sector."""
        raise NotImplementedError

    def get_eRPM(self) -> float:
        if self.timer_period_HR < 1:
            return 0
        return HR_TICK_HZ * 60.0 / (self.timer_period_HR * SECTORS_PER_REV)

    def log(self, t_s: float, delta: float):
        self.log_t.append(t_s)
        self.log_timer_period.append(self.timer_period_HR)
        self.log_delta.append(delta)
        self.log_eRPM_est.append(self.get_eRPM())
        self.log_commanded_sector.append(self.commanded_sector)


class CurrentFirmwarePI(ControllerBase):
    """
    Faithful port of motor/hwzc.c OnPiPeriodExpired (integer + bit-shifts).

    KP_SHIFT = 2  (Kp = 1/4)
    KI_SHIFT = 4  (Ki = 1/16)
    Clamp ±T/8 normally, asymmetric +T/16/-T/32 at high RPM.
    """

    KP_SHIFT = 2
    KI_SHIFT = 4
    CLAMP_SHIFT = 3            # ±T/8
    CLAMP_HIGH_SHIFT = 4        # +T/16 (positive)
    CLAMP_HIGH_NEG_SHIFT = 5    # -T/32 (negative)
    CLAMP_HIGH_TICKS = int(1e9 / 150_000)  # T at 150k eRPM
    MIN_PERIOD_TICKS = int(1e9 / 260_000)  # T at 260k eRPM
    ADVANCE_DEG = 15            # fixed for this simulator

    def __init__(self):
        super().__init__("Current (integer bit-shift)")

    def on_commutation(self, t_now_HR: float) -> int:
        T = self.timer_period_HR

        # advancePlus30Fp8 = (30 + advDeg) × 256 / 60
        adv_fp8 = int((30 + self.ADVANCE_DEG) * 256 / 60)

        delta = 0.0
        if self.capture_valid:
            cap_value = self.last_capture_HR - self.last_comm_HR
            set_value = (adv_fp8 * T) / 256

            if 0 < cap_value <= T:
                delta = cap_value - set_value

                # Asymmetric clamp at high RPM
                if T < self.CLAMP_HIGH_TICKS:
                    pos_cap = T / (1 << self.CLAMP_HIGH_SHIFT)
                    neg_cap = T / (1 << self.CLAMP_HIGH_NEG_SHIFT)
                else:
                    pos_cap = T / (1 << self.CLAMP_SHIFT)
                    neg_cap = pos_cap
                if delta >  pos_cap: delta =  pos_cap
                if delta < -neg_cap: delta = -neg_cap

                # PI math (bit-shift gains)
                self.integrator_HR += int(delta) >> self.KI_SHIFT
                if self.integrator_HR < self.MIN_PERIOD_TICKS:
                    self.integrator_HR = self.MIN_PERIOD_TICKS
                new_per = self.integrator_HR + (int(delta) >> self.KP_SHIFT)
                if new_per < self.MIN_PERIOD_TICKS:
                    new_per = self.MIN_PERIOD_TICKS
                self.timer_period_HR = new_per

            self.capture_valid = False

        # Advance commanded sector
        self.commanded_sector = (self.commanded_sector + 1) % 6
        self.last_comm_HR = t_now_HR
        self.log(t_now_HR / HR_TICK_HZ, delta)
        return self.commanded_sector


class FloatPI(ControllerBase):
    """
    Same algorithm as CurrentFirmwarePI but in float. No behavior change
    expected — purpose is to validate the harness and serve as base for
    further improvements.
    """

    Kp = 0.25
    Ki = 0.0625
    CLAMP_POS_FRAC_HIGH = 1/16
    CLAMP_NEG_FRAC_HIGH = 1/32
    CLAMP_FRAC = 1/8
    CLAMP_HIGH_TICKS = 1e9 / 150_000
    MIN_PERIOD_TICKS = 1e9 / 260_000
    ADVANCE_DEG = 15

    def __init__(self):
        super().__init__("Float (no algorithm change)")

    def on_commutation(self, t_now_HR: float) -> int:
        T = self.timer_period_HR
        adv_frac = (30 + self.ADVANCE_DEG) / 60.0

        delta = 0.0
        if self.capture_valid:
            cap_value = self.last_capture_HR - self.last_comm_HR
            set_value = adv_frac * T

            if 0 < cap_value <= T:
                delta = cap_value - set_value

                if T < self.CLAMP_HIGH_TICKS:
                    pos_cap = T * self.CLAMP_POS_FRAC_HIGH
                    neg_cap = T * self.CLAMP_NEG_FRAC_HIGH
                else:
                    pos_cap = T * self.CLAMP_FRAC
                    neg_cap = pos_cap
                if delta >  pos_cap: delta =  pos_cap
                if delta < -neg_cap: delta = -neg_cap

                self.integrator_HR += self.Ki * delta
                if self.integrator_HR < self.MIN_PERIOD_TICKS:
                    self.integrator_HR = self.MIN_PERIOD_TICKS
                new_per = self.integrator_HR + self.Kp * delta
                if new_per < self.MIN_PERIOD_TICKS:
                    new_per = self.MIN_PERIOD_TICKS
                self.timer_period_HR = new_per

            self.capture_valid = False

        self.commanded_sector = (self.commanded_sector + 1) % 6
        self.last_comm_HR = t_now_HR
        self.log(t_now_HR / HR_TICK_HZ, delta)
        return self.commanded_sector


class FloatPIWithFeedforward(ControllerBase):
    """
    Float PI + physics-based feedforward.

    P_ff = 1e9 / (KV × Vbus × duty × n_PP)        [HR ticks per sector]

    Derivation:
      eRPM_no_load = KV × V × D × polepairs
      sector_seconds = 10 / eRPM
      sector_HR_ticks = sector_seconds × 1e8 = 1e9 / eRPM

    The PI only corrects the residual error. Integrator stays smaller,
    lock acquisition is faster, less wind-up risk.
    """

    Kp = 0.25
    Ki = 0.0625
    CLAMP_FRAC = 1/8
    MIN_PERIOD_TICKS = 1e9 / 260_000
    ADVANCE_DEG = 15

    def __init__(self, motor: MotorParams, Vbus: float):
        super().__init__("Float + feedforward")
        self.motor = motor
        self.Vbus = Vbus
        self.duty = 0.0    # set externally

    def feedforward_period(self) -> float:
        if self.duty < 0.01:
            return self.MIN_PERIOD_TICKS * 10
        return 1e9 / (self.motor.KV * self.Vbus * self.duty
                      * self.motor.pole_pairs)

    def seed(self, initial_period_HR: float, t_now_HR: float):
        # Seed from feedforward, not from external initial guess
        P_ff = self.feedforward_period()
        self.timer_period_HR = P_ff
        self.integrator_HR = 0.0   # residual integrator starts at zero
        self.last_comm_HR = t_now_HR

    def on_commutation(self, t_now_HR: float) -> int:
        T = self.timer_period_HR
        adv_frac = (30 + self.ADVANCE_DEG) / 60.0
        P_ff = self.feedforward_period()

        delta = 0.0
        if self.capture_valid:
            cap_value = self.last_capture_HR - self.last_comm_HR
            set_value = adv_frac * T

            if 0 < cap_value <= T:
                delta = cap_value - set_value

                clamp = T * self.CLAMP_FRAC
                if delta >  clamp: delta =  clamp
                if delta < -clamp: delta = -clamp

                # Residual integrator + PI on the correction
                self.integrator_HR += self.Ki * delta
                # Anti-windup: clamp integrator to ±20% of feedforward
                max_residual = P_ff * 0.2
                if self.integrator_HR >  max_residual: self.integrator_HR =  max_residual
                if self.integrator_HR < -max_residual: self.integrator_HR = -max_residual

                correction = self.integrator_HR + self.Kp * delta
                new_per = P_ff + correction
                if new_per < self.MIN_PERIOD_TICKS:
                    new_per = self.MIN_PERIOD_TICKS
                self.timer_period_HR = new_per

            self.capture_valid = False

        self.commanded_sector = (self.commanded_sector + 1) % 6
        self.last_comm_HR = t_now_HR
        self.log(t_now_HR / HR_TICK_HZ, delta)
        return self.commanded_sector


class FloatPIWithSpeedLoop(FloatPIWithFeedforward):
    """
    Cascaded control: sector PI (inner) + speed PI (outer).

    Inner loop: float PI + feedforward (same as FloatPIWithFeedforward).
    Outer loop: speed PI that maps target eRPM to duty cycle.

    Throttle command is interpreted as target eRPM fraction
    (0..max_eRPM) instead of duty fraction. The speed PI then drives
    duty to make actual eRPM track target.

    Robustness features:
      - Back-calculation anti-windup (not just clamp)
      - Per-event slew limit on duty (prevents step cliffs)
      - Only integrates when sector PI is locked (no integrator
        runaway during desync / startup transient)
      - Minimum duty floor (idle / arming safety)
      - Maximum duty ceiling (MAX_DUTY safety)
      - Stall floor: if actual eRPM crashes far below target, boost
        duty proportionally faster (asymmetric P gain)
    """

    # Speed PI gains — designed for normalized duty (0..1) and eRPM in actual units
    KP_SPD_BASE = 4e-6        # duty fraction per eRPM error (full step at 250k err)
    KI_SPD_BASE = 8e-8        # 50× slower than P
    DUTY_SLEW_PER_EVENT = 0.01   # max 1% duty change per ZC event
    DUTY_MIN = 0.05           # 5% idle floor
    DUTY_MAX = 0.99
    IDLE_TARGET_ERPM = 8000   # what "throttle=zero" means in speed mode
    LOCK_DETECT_ERPM = 5000   # need >this much eRPM before speed loop engages
    STALL_BOOST_FACTOR = 2.0  # P-gain multiplier when actual << target

    def __init__(self, motor: MotorParams, Vbus: float, max_eRPM: float = 240_000):
        super().__init__(motor, Vbus)
        self.name = "Float + FF + Speed PI"
        self.max_eRPM = max_eRPM
        self.target_eRPM = 0.0
        self.duty_cmd = self.DUTY_MIN
        self.speed_integrator = 0.0
        self._was_locked = False
        # Logs
        self.log_target_eRPM = []
        self.log_duty_cmd = []

    def set_target_from_throttle(self, throttle_frac: float):
        """Map [0,1] throttle to target eRPM with idle floor."""
        if throttle_frac < 0.02:
            self.target_eRPM = self.IDLE_TARGET_ERPM
        else:
            self.target_eRPM = (self.IDLE_TARGET_ERPM
                                + throttle_frac * (self.max_eRPM - self.IDLE_TARGET_ERPM))

    def seed(self, initial_period_HR: float, t_now_HR: float):
        super().seed(initial_period_HR, t_now_HR)
        # Seed speed-loop integrator at the current duty so output doesn't jump
        self.speed_integrator = self.duty
        self.duty_cmd = self.duty

    def on_commutation(self, t_now_HR: float) -> int:
        # --- INNER LOOP: sector PI (period tracking) ---
        new_sector = super().on_commutation(t_now_HR)

        # --- OUTER LOOP: speed PI (duty adjustment) ---
        actual_eRPM = self.get_eRPM()

        # Lock detection: only engage speed loop after sector PI has locked
        if actual_eRPM > self.LOCK_DETECT_ERPM:
            self._was_locked = True

        if self._was_locked:
            err = self.target_eRPM - actual_eRPM

            # Asymmetric P gain for stall recovery (positive err = need more speed)
            kp_eff = self.KP_SPD_BASE
            if err > self.target_eRPM * 0.3:   # > 30% below target → stall regime
                kp_eff *= self.STALL_BOOST_FACTOR

            # 1) Unconstrained tentative output
            self.speed_integrator += self.KI_SPD_BASE * err
            duty_unsat = self.speed_integrator + kp_eff * err

            # 2) Apply slew limit (rate-of-change on duty)
            slew = self.DUTY_SLEW_PER_EVENT
            duty_slewed = max(self.duty_cmd - slew,
                              min(self.duty_cmd + slew, duty_unsat))

            # 3) Apply saturation (duty floor/ceiling)
            duty_sat = max(self.DUTY_MIN, min(self.DUTY_MAX, duty_slewed))

            # 4) Back-calculation anti-windup: if ANY limit (slew OR
            #    saturation) constrained the output below what the math
            #    asked for, pull the integrator back so it doesn't wind
            #    up. Kb is the back-calc gain — too high causes
            #    oscillation, too low lets some windup through.
            if duty_sat != duty_unsat:
                Kb = 0.5   # back-calc strength
                self.speed_integrator -= Kb * (duty_unsat - duty_sat)

            self.duty_cmd = duty_sat
            self.duty = duty_sat   # also update feedforward's duty input

        # Log speed-loop state
        self.log_target_eRPM.append(self.target_eRPM)
        self.log_duty_cmd.append(self.duty_cmd)

        return new_sector


# ----------------------------------------------------------------------
# Simulator engine
# ----------------------------------------------------------------------
@dataclass
class SimConfig:
    duration_s: float = 1.0
    dt_s: float = 10e-6         # 10 µs plant integration step
                                # (motor dynamics τ_mech ~50 ms; this is plenty)
    Vbus: float = 24.0
    initial_duty: float = 0.5
    duty_schedule: Callable[[float], float] = None  # t -> duty
    load_schedule: Callable[[float], float] = None  # t -> load_Nm
    initial_omega_elec: float = 0.0  # start from rest unless overridden
    noise_std_HR: float = 5.0
    miss_probability: float = 0.0


def run_simulation(controller: ControllerBase, motor: MotorParams,
                   cfg: SimConfig) -> Tuple[dict, MotorPlant]:
    """
    Single simulation run with given controller.
    Returns (motor_trace_dict, plant_object).
    """
    plant = MotorPlant(motor, Vbus=cfg.Vbus)
    plant.duty = cfg.initial_duty
    plant.omega_elec = cfg.initial_omega_elec

    filt = FilterParams()
    # Detector needs to know the controller's advance so the ZC fires at
    # the correct (advance + 30)° angle within the inter-commutation interval.
    advance = getattr(controller, 'ADVANCE_DEG', 15.0)
    events = EventDetector(plant, filt,
                           noise_std_HR=cfg.noise_std_HR,
                           miss_probability=cfg.miss_probability,
                           advance_deg=advance)

    # Seed the controller with a slightly-wrong guess. Real firmware
    # hands off from the SW path with a similar-magnitude error.
    if isinstance(controller, FloatPIWithFeedforward):
        controller.duty = cfg.initial_duty
    seed_period_HR = 1e9 / (motor.KV * cfg.Vbus * max(cfg.initial_duty, 0.05)
                             * motor.pole_pairs) * 1.1   # 10% slow
    controller.seed(seed_period_HR, 0.0)
    plant.commanded_sector = 0

    # Trace storage
    trace = {
        't': [], 'duty': [], 'load': [],
        'rotor_eRPM': [], 'estimated_eRPM': [],
        'rotor_sector': [], 'commanded_sector': [],
    }

    # Main loop
    t = 0.0
    next_comm_HR = controller.timer_period_HR   # first commutation time
    sample_every = max(1, int(200e-6 / cfg.dt_s))  # log every 200 µs
    step_idx = 0

    while t < cfg.duration_s:
        # Apply schedules
        if cfg.duty_schedule is not None:
            throttle_cmd = cfg.duty_schedule(t)
            if isinstance(controller, FloatPIWithSpeedLoop):
                # Speed mode: throttle command → target eRPM, controller owns duty
                controller.set_target_from_throttle(throttle_cmd)
                plant.duty = controller.duty_cmd   # speed PI's output
            elif isinstance(controller, FloatPIWithFeedforward):
                plant.duty = throttle_cmd
                controller.duty = throttle_cmd     # for feedforward calc
            else:
                plant.duty = throttle_cmd
        if cfg.load_schedule is not None:
            plant.load_Nm = cfg.load_schedule(t)

        plant.step(cfg.dt_s)
        t += cfg.dt_s
        t_HR = t * HR_TICK_HZ

        # Detect events
        _, zc_captured, zc_HR, _ = events.check(t, cfg.dt_s)
        if zc_captured:
            controller.on_capture(zc_HR)

        # Controller-driven commutation
        if t_HR >= next_comm_HR:
            new_sector = controller.on_commutation(t_HR)
            plant.commanded_sector = new_sector
            plant.on_commutation()    # reset intra-sector phase
            next_comm_HR = t_HR + controller.timer_period_HR

        # Trace
        if step_idx % sample_every == 0:
            trace['t'].append(t)
            trace['duty'].append(plant.duty)
            trace['load'].append(plant.load_Nm)
            trace['rotor_eRPM'].append(plant.eRPM)
            trace['estimated_eRPM'].append(controller.get_eRPM())
            trace['rotor_sector'].append(plant.rotor_sector)
            trace['commanded_sector'].append(controller.commanded_sector)
        step_idx += 1

    return trace, plant


# ----------------------------------------------------------------------
# Scenarios
# ----------------------------------------------------------------------
def scenario_lock_acquisition(motor: MotorParams) -> dict:
    """Step from rest to 50% duty. Watch lock acquisition."""
    cfg = SimConfig(
        duration_s=0.5,
        initial_duty=0.5,
        initial_omega_elec=2 * np.pi * 14000 / 60,  # seed at 14k eRPM
        duty_schedule=lambda t: 0.5,
        noise_std_HR=5.0,
    )
    return {'cfg': cfg, 'title': 'Lock acquisition (step duty 0→50% at rest)'}


def scenario_step_response(motor: MotorParams) -> dict:
    """Steady at 30%, step to 70% at 0.3s."""
    cfg = SimConfig(
        duration_s=0.8,
        initial_duty=0.3,
        initial_omega_elec=2 * np.pi * 50_000 / 60,
        duty_schedule=lambda t: 0.3 if t < 0.3 else 0.7,
        noise_std_HR=5.0,
    )
    return {'cfg': cfg, 'title': 'Throttle step 30% → 70% at t=0.3s'}


def scenario_disturbance(motor: MotorParams) -> dict:
    """Hold 50% duty, apply load impulse at 0.3s."""
    cfg = SimConfig(
        duration_s=0.8,
        initial_duty=0.5,
        initial_omega_elec=2 * np.pi * 100_000 / 60,
        duty_schedule=lambda t: 0.5,
        load_schedule=lambda t: 0.02 if 0.3 <= t < 0.5 else 0.0,
        noise_std_HR=5.0,
    )
    return {'cfg': cfg, 'title': 'Load disturbance (0.02 Nm) from t=0.3 to 0.5s'}


def scenario_noisy_captures(motor: MotorParams) -> dict:
    """High noise + occasional miss to test PI robustness."""
    cfg = SimConfig(
        duration_s=0.5,
        initial_duty=0.5,
        initial_omega_elec=2 * np.pi * 80_000 / 60,
        duty_schedule=lambda t: 0.5,
        noise_std_HR=20.0,         # 4x normal noise
        miss_probability=0.15,     # 15% missed captures
    )
    return {'cfg': cfg, 'title': 'High noise (σ=20 HR) + 15% missed captures'}


SCENARIOS = {
    'lock': scenario_lock_acquisition,
    'step': scenario_step_response,
    'disturbance': scenario_disturbance,
    'noisy': scenario_noisy_captures,
}


# ----------------------------------------------------------------------
# Plotting
# ----------------------------------------------------------------------
def plot_comparison(scenario_name: str, scenario_title: str,
                    motor: MotorParams, results: List[Tuple[str, dict, ControllerBase]]):
    """Side-by-side plot of multiple controllers on the same scenario."""
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

    for label, trace, ctrl in results:
        t_arr = np.array(trace['t'])
        eRPM_rotor = np.array(trace['rotor_eRPM'])
        eRPM_est = np.array(trace['estimated_eRPM'])
        delta_arr = np.array(ctrl.log_delta)
        t_ctrl = np.array(ctrl.log_t)

        axes[0].plot(t_arr, eRPM_rotor, label=f'{label} — rotor', lw=1.5)
        axes[0].plot(t_arr, eRPM_est, '--', label=f'{label} — estimated', lw=1, alpha=0.7)

        axes[1].plot(t_arr, eRPM_rotor - eRPM_est, label=label, lw=1.5)

        if len(t_ctrl) > 0:
            axes[2].plot(t_ctrl, delta_arr, '.', label=label, ms=2, alpha=0.6)

    axes[0].set_ylabel('eRPM')
    axes[0].set_title(f'{scenario_title}\n(rotor truth = solid, controller estimate = dashed)')
    axes[0].legend(loc='best', fontsize=8)
    axes[0].grid(True, alpha=0.3)

    axes[1].set_ylabel('eRPM error\n(rotor − estimate)')
    axes[1].axhline(0, color='k', lw=0.5)
    axes[1].legend(loc='best', fontsize=8)
    axes[1].grid(True, alpha=0.3)

    axes[2].set_ylabel('Phase error δ\n(HR ticks)')
    axes[2].set_xlabel('Time (s)')
    axes[2].axhline(0, color='k', lw=0.5)
    axes[2].legend(loc='best', fontsize=8)
    axes[2].grid(True, alpha=0.3)

    fig.tight_layout()
    return fig


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------
def make_controllers(motor: MotorParams, Vbus: float) -> List[ControllerBase]:
    """Default list of controllers for CLI comparisons."""
    return [
        CurrentFirmwarePI(),
        FloatPI(),
        FloatPIWithFeedforward(motor=motor, Vbus=Vbus),
        FloatPIWithSpeedLoop(motor=motor, Vbus=Vbus, max_eRPM=240_000),
    ]


def dump_text_summary(scenario_name: str, scenario_title: str,
                      motor: MotorParams, Vbus: float,
                      results: List[Tuple[str, dict, ControllerBase]]) -> str:
    """Compact text summary of a scenario — copy-paste friendly."""
    out = []
    out.append("=" * 78)
    out.append(f"SCENARIO: {scenario_name}  —  {scenario_title}")
    out.append("-" * 78)
    out.append(f"Motor: {motor.name}  KV={motor.KV}  PP={motor.pole_pairs}  "
               f"Rs={motor.Rs_ohm*1000:.0f}mΩ  Ls={motor.Ls_uH}µH")
    out.append(f"Vbus: {Vbus} V    No-load max eRPM = {motor.KV*Vbus*motor.pole_pairs:,}")
    out.append("")
    out.append(f"{'Controller':<35} {'SS RMS err':>11} {'Lock time':>10} {'Peak err':>10}")
    out.append("-" * 78)
    for label, trace, ctrl in results:
        t_arr = np.array(trace['t'])
        err = np.array(trace['rotor_eRPM']) - np.array(trace['estimated_eRPM'])
        ss_mask = t_arr > t_arr[-1] * 0.7
        ss_rms = float(np.sqrt(np.mean(err[ss_mask] ** 2))) if ss_mask.any() else 0
        abs_err = np.abs(err)
        lock_idx = None
        for i in range(len(abs_err)):
            if abs_err[i] < 5000 and i + 5 < len(abs_err) and (abs_err[i:i+5] < 5000).all():
                lock_idx = i
                break
        lock_str = f"{t_arr[lock_idx]*1000:.0f}ms" if lock_idx else "no lock"
        peak = float(np.max(abs_err))
        out.append(f"{label:<35} {ss_rms:>9,.0f}eRPM {lock_str:>10} {peak:>8,.0f}")

    # Sparse timeline — 10 evenly-spaced rows of key metrics from the first controller
    if results:
        out.append("")
        out.append("Sample timeline (first controller, every ~duration/10):")
        out.append(f"{'t(s)':>6} {'duty%':>6} {'load':>5} {'rotor':>8} {'estim':>8} {'err':>7}")
        first_trace = results[0][1]
        n = len(first_trace['t'])
        for i in np.linspace(0, n - 1, 11).astype(int):
            t_arr = first_trace['t']
            err = first_trace['rotor_eRPM'][i] - first_trace['estimated_eRPM'][i]
            out.append(
                f"{t_arr[i]:6.3f} "
                f"{first_trace['duty'][i]*100:5.1f}% "
                f"{first_trace['load'][i]*1000:4.1f}m "
                f"{first_trace['rotor_eRPM'][i]:8,.0f} "
                f"{first_trace['estimated_eRPM'][i]:8,.0f} "
                f"{err:+7,.0f}"
            )

    # Per-controller final state
    out.append("")
    out.append("Final controller state:")
    for label, trace, ctrl in results:
        final_period = ctrl.timer_period_HR
        final_integ = ctrl.integrator_HR
        final_est = ctrl.get_eRPM()
        extra = ""
        if isinstance(ctrl, FloatPIWithSpeedLoop):
            extra = (f"  target={ctrl.target_eRPM:,.0f}  "
                     f"duty_cmd={ctrl.duty_cmd:.3f}  "
                     f"spd_integ={ctrl.speed_integrator:.3f}")
        out.append(f"  {label:<35} period={final_period:>8,.0f}HR  "
                   f"integ={final_integ:>8,.0f}  est={final_est:,.0f}eRPM{extra}")
    out.append("=" * 78)
    return "\n".join(out)


def dump_csv(scenario_name: str, results: List[Tuple[str, dict, ControllerBase]]) -> str:
    """One-row-per-sample CSV. Wide format with one column per controller per metric."""
    if not results:
        return ""
    first_trace = results[0][1]
    n = len(first_trace['t'])

    header = ['t_s', 'duty', 'load_Nm']
    for label, _, _ in results:
        safe = label.replace(' ', '_').replace('+', 'plus').replace('(', '').replace(')', '')
        header.extend([f'{safe}__rotor_eRPM', f'{safe}__est_eRPM', f'{safe}__err_eRPM'])

    rows = [','.join(header)]
    for i in range(n):
        row = [f"{first_trace['t'][i]:.6f}",
               f"{first_trace['duty'][i]:.4f}",
               f"{first_trace['load'][i]:.4f}"]
        for label, trace, _ in results:
            r = trace['rotor_eRPM'][i] if i < len(trace['rotor_eRPM']) else 0
            e = trace['estimated_eRPM'][i] if i < len(trace['estimated_eRPM']) else 0
            row.extend([f"{r:.0f}", f"{e:.0f}", f"{r-e:.0f}"])
        rows.append(','.join(row))
    return '\n'.join(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', default='all', choices=list(SCENARIOS.keys()) + ['all'],
                        help='Which scenario to run')
    parser.add_argument('--save', metavar='PREFIX', default=None,
                        help='Save plots to PREFIX-<scenario>.png instead of showing')
    parser.add_argument('--text', action='store_true',
                        help='Print text summary to stdout (always on if --no-plot)')
    parser.add_argument('--csv', metavar='PREFIX', default=None,
                        help='Save CSV trace data to PREFIX-<scenario>.csv')
    parser.add_argument('--no-plot', action='store_true',
                        help='Skip plotting entirely (faster, text-only)')
    args = parser.parse_args()

    motor = MotorParams()
    Vbus = 24.0

    scenarios_to_run = list(SCENARIOS.keys()) if args.scenario == 'all' else [args.scenario]

    for sc_name in scenarios_to_run:
        sc = SCENARIOS[sc_name](motor)
        print(f"\n=== Running scenario: {sc['title']} ===")

        results = []
        for ctrl in make_controllers(motor, Vbus):
            print(f"  Controller: {ctrl.name}")
            trace, _ = run_simulation(ctrl, motor, sc['cfg'])
            results.append((ctrl.name, trace, ctrl))

            # Quick stats
            t_arr = np.array(trace['t'])
            err_arr = np.array(trace['rotor_eRPM']) - np.array(trace['estimated_eRPM'])
            # Steady-state (last 20% of run)
            ss_mask = t_arr > t_arr[-1] * 0.8
            ss_rms_err = float(np.sqrt(np.mean(err_arr[ss_mask] ** 2)))
            print(f"    Steady-state RMS eRPM error: {ss_rms_err:.0f}")

        # Text summary
        if args.text or args.no_plot:
            print()
            print(dump_text_summary(sc_name, sc['title'], motor, Vbus, results))

        # CSV trace
        if args.csv:
            csv_path = f"{args.csv}-{sc_name}.csv"
            with open(csv_path, 'w') as f:
                f.write(dump_csv(sc_name, results))
            print(f"  Saved CSV: {csv_path}")

        # Plots
        if not args.no_plot:
            fig = plot_comparison(sc_name, sc['title'], motor, results)
            if args.save:
                fname = f"{args.save}-{sc_name}.png"
                fig.savefig(fname, dpi=120)
                print(f"  Saved plot: {fname}")
                plt.close(fig)
            else:
                plt.show()


if __name__ == '__main__':
    main()
