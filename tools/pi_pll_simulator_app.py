#!/usr/bin/env python3
"""
Interactive PI/PLL simulator — Streamlit UI

Browser-based playground for the sector PI controller. Move sliders,
see live plots of how the controller tracks the rotor.

Setup (one-time):
    pip install streamlit numpy matplotlib

Run:
    streamlit run tools/pi_pll_simulator_app.py

This will open a local browser window (typically http://localhost:8501).
Move the sliders in the sidebar and the plots update live.

Companion to:
  - tools/pi_pll_simulator.py — core simulation engine (this is the UI)
  - docs/pi_controller_research.md — what we're studying and why
"""

import os
import sys
from pathlib import Path

# Force matplotlib backend before any other imports
os.environ.setdefault("MPLBACKEND", "Agg")

# Make the core simulator importable
THIS_DIR = Path(__file__).parent
sys.path.insert(0, str(THIS_DIR))

import numpy as np
import matplotlib.pyplot as plt
import streamlit as st

from pi_pll_simulator import (
    HR_TICK_HZ, NS_PER_TICK, SECTORS_PER_REV,
    MotorParams, FilterParams, MotorPlant, EventDetector,
    CurrentFirmwarePI, FloatPI, FloatPIWithFeedforward, FloatPIWithSpeedLoop,
    SimConfig, run_simulation,
)


# ---------------------------------------------------------------
# Page config
# ---------------------------------------------------------------
st.set_page_config(
    page_title="Garuda Sector PI Simulator",
    page_icon="🌀",
    layout="wide",
)

st.title("🌀 Sector PI / PLL Controller Simulator")
st.caption(
    "Garuda ESC firmware — move the sliders to see how the sector PI "
    "tracks the rotor under different motors, gains, noise, and "
    "throttle scenarios. Companion to "
    "[`docs/pi_controller_research.md`](https://github.com/bhanuprakashjh/ProjectGaruda/blob/main/dspic33AKESC/docs/pi_controller_research.md)."
)


# ---------------------------------------------------------------
# Motor profile presets
# ---------------------------------------------------------------
MOTOR_PROFILES = {
    "2810 1350KV (default)":   dict(KV=1350, pole_pairs=7, Rs_ohm=0.022, Ls_uH=25,  J_kgm2=1.5e-6, B_Nms=5e-6),
    "A2212 1400KV":            dict(KV=1400, pole_pairs=7, Rs_ohm=0.065, Ls_uH=30,  J_kgm2=1.0e-6, B_Nms=8e-6),
    "Hurst 280KV (gimbal)":    dict(KV=280,  pole_pairs=5, Rs_ohm=0.534, Ls_uH=471, J_kgm2=8.0e-6, B_Nms=2e-5),
    "Custom":                  None,
}


# ---------------------------------------------------------------
# Sidebar — all the knobs
# ---------------------------------------------------------------
with st.sidebar:
    st.header("📋 Motor")
    profile_name = st.selectbox("Profile", list(MOTOR_PROFILES.keys()))

    if MOTOR_PROFILES[profile_name] is not None:
        params = MOTOR_PROFILES[profile_name]
        KV = params["KV"]
        pole_pairs = params["pole_pairs"]
        Rs_ohm = params["Rs_ohm"]
        Ls_uH = params["Ls_uH"]
        J_kgm2 = params["J_kgm2"]
        B_Nms = params["B_Nms"]
        st.caption(f"KV={KV}, PP={pole_pairs}, Rs={Rs_ohm*1000:.0f}mΩ, Ls={Ls_uH}µH")
    else:
        KV = st.slider("KV (RPM/V)", 100, 4000, 1350, 50)
        pole_pairs = st.slider("Pole pairs", 2, 14, 7, 1)
        Rs_ohm = st.slider("Rs (mΩ)", 5, 1000, 22, 1) / 1000.0
        Ls_uH = st.slider("Ls (µH)", 5, 500, 25, 1)
        J_kgm2 = st.slider("Inertia J (µkg·m²)", 1, 100, 15, 1) * 1e-7
        B_Nms = st.slider("Friction B (µN·m·s)", 1, 100, 5, 1) * 1e-6

    motor = MotorParams(
        name=profile_name,
        KV=KV, pole_pairs=pole_pairs, Rs_ohm=Rs_ohm,
        Ls_uH=Ls_uH, J_kgm2=J_kgm2, B_Nms=B_Nms,
    )

    st.header("⚡ Bus")
    Vbus = st.slider("Vbus (V)", 6.0, 50.0, 24.0, 0.5)

    no_load_eRPM = KV * Vbus * pole_pairs
    st.caption(f"No-load eRPM ceiling: **{no_load_eRPM:,.0f}**")

    st.header("🎛️ Controllers")
    controllers_enabled = st.multiselect(
        "Variants to compare",
        ["Current (int bit-shift)", "Float (no algo change)",
         "Float + feedforward", "Float + FF + Speed PI"],
        default=["Current (int bit-shift)", "Float + feedforward",
                 "Float + FF + Speed PI"],
    )
    has_speed_pi = "Float + FF + Speed PI" in controllers_enabled
    if has_speed_pi:
        st.caption("ℹ️ With Speed PI: throttle slider sets target **eRPM**, "
                   "not duty. Speed loop adjusts duty to track.")

    with st.expander("Sector PI gains (inner loop)"):
        Kp_value = st.select_slider(
            "Kp (proportional)",
            options=[1/16, 1/8, 1/4, 1/2, 0.31, 0.4, 0.6],
            value=0.25,
        )
        Ki_value = st.select_slider(
            "Ki (integral)",
            options=[1/64, 1/32, 1/16, 1/8, 0.1, 0.15],
            value=1/16,
        )
        advance_deg = st.slider("Timing advance (deg)", 0, 30, 15)

    with st.expander("Speed PI gains (outer loop)", expanded=has_speed_pi):
        max_target_eRPM = st.slider(
            "Max target eRPM", 50_000, 300_000, 240_000, 10_000)
        Kp_spd_scale = st.slider(
            "Kp_spd scale", 0.1, 5.0, 1.0, 0.1,
            help="Multiplier on default Kp_spd=4e-6 (duty per eRPM err)")
        Ki_spd_scale = st.slider(
            "Ki_spd scale", 0.1, 5.0, 1.0, 0.1,
            help="Multiplier on default Ki_spd=8e-8")
        duty_slew_pct = st.slider(
            "Duty slew limit (% per ZC event)", 0.1, 10.0, 1.0, 0.1)
        idle_target_eRPM = st.slider(
            "Idle target eRPM (throttle=0)", 0, 30_000, 8_000, 500)
        stall_boost = st.slider(
            "Stall boost (P-gain ×)", 1.0, 5.0, 2.0, 0.1,
            help="When actual << target, multiply Kp by this for faster recovery")

    st.header("🔍 Measurement")
    noise_HR = st.slider("ZC noise σ (HR ticks)", 0, 50, 5)
    miss_pct = st.slider("Missed-capture rate (%)", 0, 50, 0)

    st.header("📈 Scenario")
    scenario_kind = st.radio(
        "Throttle profile",
        ["Constant", "Step", "Ramp", "Sinusoidal"],
    )
    duration_s = st.slider("Duration (s)", 0.1, 2.0, 0.5, 0.05)
    initial_duty = st.slider("Initial duty (%)", 5, 99, 50) / 100.0

    if scenario_kind == "Step":
        step_duty = st.slider("Step to duty (%)", 5, 99, 70) / 100.0
        step_time = st.slider("Step at t= (s)", 0.05, duration_s - 0.05,
                              duration_s / 2, 0.05)
    elif scenario_kind == "Ramp":
        end_duty = st.slider("Final duty (%)", 5, 99, 80) / 100.0
        ramp_start = st.slider("Ramp start (s)", 0.0, duration_s - 0.05, 0.1, 0.05)
        ramp_end = st.slider("Ramp end (s)", ramp_start + 0.05, duration_s,
                             min(duration_s, ramp_start + 0.3), 0.05)
    elif scenario_kind == "Sinusoidal":
        sine_amp = st.slider("Sine amplitude (% duty)", 0, 50, 20) / 100.0
        sine_freq = st.slider("Sine frequency (Hz)", 0.5, 50.0, 5.0, 0.5)

    enable_load = st.checkbox("Add load disturbance")
    if enable_load:
        load_Nm = st.slider("Load (N·m)", 0.0, 0.1, 0.02, 0.005)
        load_start = st.slider("Load start (s)", 0.0, duration_s - 0.05, duration_s / 3, 0.05)
        load_end = st.slider("Load end (s)", load_start, duration_s,
                             min(duration_s, load_start + 0.2), 0.05)
    else:
        load_Nm = 0.0

    seed_error_pct = st.slider("Initial seed error (%)", -50, 100, 10, 5,
                               help="How far off is the controller's initial guess?")


# ---------------------------------------------------------------
# Build duty schedule
# ---------------------------------------------------------------
def make_duty_schedule():
    if scenario_kind == "Constant":
        return lambda t: initial_duty
    elif scenario_kind == "Step":
        return lambda t: initial_duty if t < step_time else step_duty
    elif scenario_kind == "Ramp":
        def ramp(t):
            if t < ramp_start:
                return initial_duty
            if t > ramp_end:
                return end_duty
            return initial_duty + (end_duty - initial_duty) * (t - ramp_start) / (ramp_end - ramp_start)
        return ramp
    elif scenario_kind == "Sinusoidal":
        return lambda t: max(0.05, min(0.99,
                              initial_duty + sine_amp * np.sin(2 * np.pi * sine_freq * t)))


def make_load_schedule():
    if not enable_load:
        return None
    return lambda t: load_Nm if load_start <= t < load_end else 0.0


# ---------------------------------------------------------------
# Build controllers
# ---------------------------------------------------------------
def build_controllers():
    ctrls = []
    for name in controllers_enabled:
        if name == "Current (int bit-shift)":
            c = CurrentFirmwarePI()
        elif name == "Float (no algo change)":
            c = FloatPI()
            c.Kp = Kp_value
            c.Ki = Ki_value
            c.ADVANCE_DEG = advance_deg
        elif name == "Float + feedforward":
            c = FloatPIWithFeedforward(motor=motor, Vbus=Vbus)
            c.Kp = Kp_value
            c.Ki = Ki_value
            c.ADVANCE_DEG = advance_deg
        else:
            continue
        ctrls.append(c)
    return ctrls


# ---------------------------------------------------------------
# Run sim (cached so unchanged inputs don't re-run)
# ---------------------------------------------------------------
@st.cache_data(show_spinner=False)
def run_one(ctrl_factory_key, motor_dict, Vbus, duration_s, initial_duty,
            duty_kind, duty_extra, load_kind, load_extra, noise_HR, miss_pct,
            seed_error_pct, Kp_value, Ki_value, advance_deg,
            max_target_eRPM, Kp_spd_scale, Ki_spd_scale,
            duty_slew_pct, idle_target_eRPM, stall_boost):
    """Cached single-controller run. Args are hashable; controllers built inside.

    Note: arg names must NOT start with underscore — Streamlit skips
    underscore-prefixed args during cache-key hashing, which would
    cause different controllers to collide to the same cache entry.
    """
    # Reconstruct motor
    m = MotorParams(**motor_dict)

    # Build duty schedule from kind/extra
    if duty_kind == "Constant":
        duty_sched = lambda t: initial_duty
    elif duty_kind == "Step":
        step_time, step_duty = duty_extra
        duty_sched = lambda t: initial_duty if t < step_time else step_duty
    elif duty_kind == "Ramp":
        ramp_start, ramp_end, end_duty = duty_extra
        def duty_sched(t):
            if t < ramp_start: return initial_duty
            if t > ramp_end: return end_duty
            return initial_duty + (end_duty - initial_duty) * (t - ramp_start) / (ramp_end - ramp_start)
    elif duty_kind == "Sinusoidal":
        sine_amp, sine_freq = duty_extra
        duty_sched = lambda t: max(0.05, min(0.99,
                                  initial_duty + sine_amp * np.sin(2 * np.pi * sine_freq * t)))

    if load_kind == "off":
        load_sched = None
    else:
        load_Nm, load_start, load_end = load_extra
        load_sched = lambda t: load_Nm if load_start <= t < load_end else 0.0

    # Build the controller
    if ctrl_factory_key == "Current (int bit-shift)":
        c = CurrentFirmwarePI()
        c.ADVANCE_DEG = advance_deg
    elif ctrl_factory_key == "Float (no algo change)":
        c = FloatPI()
        c.Kp = Kp_value
        c.Ki = Ki_value
        c.ADVANCE_DEG = advance_deg
    elif ctrl_factory_key == "Float + feedforward":
        c = FloatPIWithFeedforward(motor=m, Vbus=Vbus)
        c.Kp = Kp_value
        c.Ki = Ki_value
        c.ADVANCE_DEG = advance_deg
    elif ctrl_factory_key == "Float + FF + Speed PI":
        c = FloatPIWithSpeedLoop(motor=m, Vbus=Vbus, max_eRPM=max_target_eRPM)
        c.Kp = Kp_value
        c.Ki = Ki_value
        c.ADVANCE_DEG = advance_deg
        # Speed-loop tuning
        c.KP_SPD_BASE = 4e-6 * Kp_spd_scale
        c.KI_SPD_BASE = 8e-8 * Ki_spd_scale
        c.DUTY_SLEW_PER_EVENT = duty_slew_pct / 100.0
        c.IDLE_TARGET_ERPM = idle_target_eRPM
        c.STALL_BOOST_FACTOR = stall_boost

    # Apply seed error knob (the run_simulation helper uses fixed 1.1× now;
    # we just patch it by setting the controller's seed manually below)
    cfg = SimConfig(
        duration_s=duration_s,
        dt_s=10e-6,
        Vbus=Vbus,
        initial_duty=initial_duty,
        duty_schedule=duty_sched,
        load_schedule=load_sched,
        initial_omega_elec=0.0,
        noise_std_HR=noise_HR,
        miss_probability=miss_pct / 100.0,
    )

    # Manually override seed (run_simulation seeds at 1.1× — overwrite after)
    # Cleaner approach would be to pass seed_factor; for now this works.
    seed_factor = 1.0 + seed_error_pct / 100.0
    plant = MotorPlant(m, Vbus=Vbus)
    plant.duty = initial_duty
    plant.omega_elec = 0.0
    filt = FilterParams()
    events = EventDetector(plant, filt, noise_std_HR=noise_HR,
                           miss_probability=miss_pct / 100.0)

    seed_period_HR = 1e9 / (m.KV * Vbus * max(initial_duty, 0.05)
                            * m.pole_pairs) * seed_factor
    if isinstance(c, FloatPIWithFeedforward):
        c.duty = initial_duty
    c.seed(seed_period_HR, 0.0)
    plant.commanded_sector = 0

    trace = {
        't': [], 'duty': [], 'load': [],
        'rotor_eRPM': [], 'estimated_eRPM': [],
        'rotor_sector': [], 'commanded_sector': [],
    }

    t = 0.0
    next_comm_HR = c.timer_period_HR
    sample_every = max(1, int(200e-6 / cfg.dt_s))
    step_idx = 0

    while t < cfg.duration_s:
        throttle_cmd = duty_sched(t)
        if isinstance(c, FloatPIWithSpeedLoop):
            # Speed mode: throttle command → target eRPM; controller owns duty
            c.set_target_from_throttle(throttle_cmd)
            plant.duty = c.duty_cmd
        elif isinstance(c, FloatPIWithFeedforward):
            plant.duty = throttle_cmd
            c.duty = throttle_cmd
        else:
            plant.duty = throttle_cmd
        if load_sched is not None:
            plant.load_Nm = load_sched(t)

        plant.step(cfg.dt_s)
        t += cfg.dt_s
        t_HR = t * HR_TICK_HZ

        _, zc_captured, zc_HR, _ = events.check(t)
        if zc_captured:
            c.on_capture(zc_HR)

        if t_HR >= next_comm_HR:
            new_sector = c.on_commutation(t_HR)
            plant.commanded_sector = new_sector
            next_comm_HR = t_HR + c.timer_period_HR

        if step_idx % sample_every == 0:
            trace['t'].append(t)
            trace['duty'].append(plant.duty)
            trace['load'].append(plant.load_Nm)
            trace['rotor_eRPM'].append(plant.eRPM)
            trace['estimated_eRPM'].append(c.get_eRPM())
            trace['rotor_sector'].append(plant.rotor_sector)
            trace['commanded_sector'].append(c.commanded_sector)
        step_idx += 1

    return {
        'trace': trace,
        'log_t': list(c.log_t),
        'log_delta': list(c.log_delta),
        'log_timer_period': list(c.log_timer_period),
        'log_eRPM_est': list(c.log_eRPM_est),
    }


# Hashable args
motor_dict = dict(name=motor.name, KV=KV, pole_pairs=pole_pairs,
                  Rs_ohm=Rs_ohm, Ls_uH=Ls_uH, J_kgm2=J_kgm2, B_Nms=B_Nms)

if scenario_kind == "Step":
    duty_kind = "Step"
    duty_extra = (step_time, step_duty)
elif scenario_kind == "Ramp":
    duty_kind = "Ramp"
    duty_extra = (ramp_start, ramp_end, end_duty)
elif scenario_kind == "Sinusoidal":
    duty_kind = "Sinusoidal"
    duty_extra = (sine_amp, sine_freq)
else:
    duty_kind = "Constant"
    duty_extra = ()

if enable_load:
    load_kind = "on"
    load_extra = (load_Nm, load_start, load_end)
else:
    load_kind = "off"
    load_extra = ()


# ---------------------------------------------------------------
# Run all enabled controllers
# ---------------------------------------------------------------
if not controllers_enabled:
    st.warning("Select at least one controller variant in the sidebar.")
    st.stop()

with st.spinner("Simulating..."):
    results = {}
    for name in controllers_enabled:
        results[name] = run_one(
            name, motor_dict, Vbus, duration_s, initial_duty,
            duty_kind, duty_extra, load_kind, load_extra,
            noise_HR, miss_pct, seed_error_pct,
            Kp_value, Ki_value, advance_deg,
            max_target_eRPM, Kp_spd_scale, Ki_spd_scale,
            duty_slew_pct, idle_target_eRPM, stall_boost,
        )


# ---------------------------------------------------------------
# Top metrics row
# ---------------------------------------------------------------
def stats_for(result):
    t = np.array(result['trace']['t'])
    err = np.array(result['trace']['rotor_eRPM']) - np.array(result['trace']['estimated_eRPM'])
    # Steady-state = last 30% of run
    ss_mask = t > t[-1] * 0.7
    ss_rms = float(np.sqrt(np.mean(err[ss_mask] ** 2))) if ss_mask.any() else 0.0
    # Lock time = first time abs(error) < 5000 eRPM continuously
    abs_err = np.abs(err)
    lock_idx = None
    for i in range(len(abs_err)):
        if abs_err[i] < 5000 and i + 5 < len(abs_err) and (abs_err[i:i+5] < 5000).all():
            lock_idx = i
            break
    lock_time = float(t[lock_idx]) if lock_idx else None
    # Peak error
    peak_err = float(np.max(np.abs(err)))
    return ss_rms, lock_time, peak_err

cols = st.columns(len(results))
for col, (name, result) in zip(cols, results.items()):
    ss_rms, lock_time, peak_err = stats_for(result)
    with col:
        st.subheader(name)
        st.metric("Steady-state RMS error", f"{ss_rms:,.0f} eRPM")
        st.metric("Lock time", f"{lock_time*1000:.0f} ms" if lock_time else "did not lock")
        st.metric("Peak error", f"{peak_err:,.0f} eRPM")


# ---------------------------------------------------------------
# Plots
# ---------------------------------------------------------------
def make_plot():
    fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True)

    # Duty / load
    if results:
        first_trace = next(iter(results.values()))['trace']
        t_arr = np.array(first_trace['t'])
        axes[0].plot(t_arr, np.array(first_trace['duty']) * 100, 'k-', lw=1.5, label='duty %')
        axes[0].set_ylabel('Duty (%)')
        axes[0].set_ylim(0, 105)
        axes[0].grid(True, alpha=0.3)
        if enable_load:
            ax_load = axes[0].twinx()
            ax_load.plot(t_arr, np.array(first_trace['load']) * 1000, 'r--', lw=1, label='load (mN·m)')
            ax_load.set_ylabel('Load (mN·m)', color='r')
            ax_load.tick_params(axis='y', colors='r')
        axes[0].set_title('Throttle and Load Schedule')

    # eRPM tracking
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
    for (name, result), color in zip(results.items(), colors):
        trace = result['trace']
        t_arr = np.array(trace['t'])
        axes[1].plot(t_arr, trace['rotor_eRPM'], '-', lw=1.5, color=color,
                     label=f'{name} — rotor')
        axes[1].plot(t_arr, trace['estimated_eRPM'], '--', lw=1, color=color,
                     alpha=0.7, label=f'{name} — estimated')
    axes[1].set_ylabel('eRPM')
    axes[1].set_title('Rotor (solid) vs Controller Estimate (dashed)')
    axes[1].legend(loc='best', fontsize=8, ncol=2)
    axes[1].grid(True, alpha=0.3)

    # eRPM error
    for (name, result), color in zip(results.items(), colors):
        trace = result['trace']
        t_arr = np.array(trace['t'])
        err = np.array(trace['rotor_eRPM']) - np.array(trace['estimated_eRPM'])
        axes[2].plot(t_arr, err, '-', lw=1.5, color=color, label=name)
    axes[2].axhline(0, color='k', lw=0.5)
    axes[2].set_ylabel('Tracking error\n(rotor − estimate, eRPM)')
    axes[2].set_title('Tracking Error')
    axes[2].legend(loc='best', fontsize=8)
    axes[2].grid(True, alpha=0.3)

    # Phase delta
    for (name, result), color in zip(results.items(), colors):
        t_ctrl = np.array(result['log_t'])
        delta = np.array(result['log_delta'])
        if len(t_ctrl) > 0:
            axes[3].plot(t_ctrl, delta, '.', ms=3, color=color, alpha=0.6, label=name)
    axes[3].axhline(0, color='k', lw=0.5)
    axes[3].set_ylabel('Phase error δ\n(HR ticks)')
    axes[3].set_title('PI Phase Error (per-sector capture−setpoint)')
    axes[3].set_xlabel('Time (s)')
    axes[3].legend(loc='best', fontsize=8)
    axes[3].grid(True, alpha=0.3)

    fig.tight_layout()
    return fig

st.pyplot(make_plot())


# ---------------------------------------------------------------
# Diagnostics
# ---------------------------------------------------------------
with st.expander("🔬 Diagnostics & derived values"):
    filt = FilterParams()
    st.write(f"**Motor**: {motor.name}")
    st.write(f"- KV = {motor.KV}, pole pairs = {motor.pole_pairs}")
    st.write(f"- Rs = {motor.Rs_ohm*1000:.1f} mΩ, Ls = {motor.Ls_uH} µH")
    st.write(f"- ke = {motor.ke_Vs_per_rad*1000:.3f} mV·s/rad mech")
    st.write(f"- τ_electrical (Ls/Rs) = {motor.Ls_uH / motor.Rs_ohm:.0f} µs")
    st.write("")
    st.write(f"**Filter**: R = {filt.R_ohm:.0f} Ω, C = {filt.C_F*1e9:.1f} nF, τ = {filt.tau_s*1e6:.1f} µs")
    st.write(f"- Cutoff = {1/(2*np.pi*filt.tau_s):,.0f} Hz")
    st.write(f"- Phase lag at no-load eRPM = {np.degrees(filt.phase_lag_rad(no_load_eRPM*2*np.pi/60)):.1f}°")
    st.write("")
    st.write(f"**Bus + duty**: {Vbus} V × {initial_duty*100:.0f}% = {Vbus*initial_duty:.1f} V applied")
    st.write(f"**No-load steady-state eRPM**: {KV*Vbus*initial_duty*pole_pairs:,.0f}")
    st.write("")
    st.write(f"**Controller seed**: {seed_error_pct:+d}% off ideal")
    st.write(f"**ZC noise**: σ = {noise_HR} HR ticks = {noise_HR*NS_PER_TICK:.0f} ns")
    if miss_pct:
        st.write(f"**Missed captures**: {miss_pct}%")


# ---------------------------------------------------------------
# Footer / help
# ---------------------------------------------------------------
st.divider()
with st.expander("ℹ️ About this simulator / what to look for"):
    st.markdown("""
**What you're looking at**:
- **Top panel**: the throttle (duty) profile you chose, plus any load
- **eRPM tracking**: solid = actual rotor speed, dashed = what the controller *thinks* the rotor is doing
- **Tracking error**: difference between the two. Should converge to zero for a working controller
- **Phase error δ**: the per-capture phase error feeding the PI (HR ticks = 10 ns)

**Things to try**:
1. Default settings → step throttle 30%→70% → see how `Float + feedforward` tracks instantly while `Current` lags badly
2. High noise (σ = 30+) → see noise rejection between variants
3. Miss probability 30%+ → see robustness to dropped captures
4. Switch to Hurst (low KV, high Rs) → see how the seed and feedforward predict differently
5. Sinusoidal duty at 5-10 Hz → see bandwidth limits

**What's modeled**:
- Simplified first-order BLDC plant (no commutation-misalignment torque coupling)
- ZC events at true rotor sector midpoints with filter delay + noise + miss
- Three controller variants matching the firmware + roadmap

**What's NOT modeled**:
- PWM ripple (approximated by the noise knob)
- Wrong-angle torque penalty (would oscillate while controller catches up)
- Phase current saturation / OC events

This is a **controller-dynamics** simulator. For hardware-fidelity tests, the bench is still authoritative.
""")
