"""garuda zcsim - 6-step sensorless BEMF zero-cross simulator (model only, numpy).

Reproduces, from physics, the falling-polarity ZC masking measured on the bench:
the floating-phase BEMF is clean in the PWM-OFF (freewheel) window but, in the
PWM-ON window, gets swamped by switching transients (the patent's "parasitic
capacitance resonates with motor inductance during PWM-off" + PWM dv/dt). The
1 MHz comparator (gated to ON) therefore sees corrupted data on falling sectors,
while the RC-filtered OFF-center sample stays clean.

Two layers, one knob set:
  * Layer 1 (ideal): spike_amp=0, ring_amp=0  -> both polarities detect in BOTH
    windows (no masking). Proves the masking is NOT an algorithm/threshold bug.
  * Layer 2 (parasitic): nonzero spike/ring  -> ON-window comparator corrupts,
    falling masking appears, speed-progressive. Tune to the bench envelopes.

Bench validation targets (2810@24V, gui_auto_*):
  OFF-center bemfRaw @>180k:   54..1282   (clean, neutral ~609)
  comparator ON-window @>180k: 12..2559   (corrupted ~2x Vbus)
  falling capture: ~88% @50-90k -> 0% @>180k

Pure numpy; no Qt. Imported by the Studio "ZC Sim" tab, and runnable standalone.
"""
from __future__ import annotations
import numpy as np

# ---- motor / board constants (2810 1350KV on MCLV-48V @ 24V) ----------------
VBUS_DEFAULT = 24.0
KV           = 1350.0                 # rpm/V
POLES_PP     = 7                      # pole pairs (14-pole 2810)
KE           = 60.0 / (2 * np.pi * KV)   # V*s/rad mech (line-neutral peak = KE*w_mech)
PWM_HZ       = 45000.0
ADC_PER_V    = 625.0 / 12.0           # divider: Vbus/2 = 12V -> ~625 ADC counts
CMP_HZ       = 1.0e6                  # comparator sample rate
DEADBAND     = 4                      # ADC counts (HWZC_CMP_DEADBAND)
ADC_MAX      = 4095


def _trap(theta_deg):
    """Normalized trapezoidal BEMF: +1 flat 120 deg, ramp 60, -1 flat 120, ramp 60."""
    t = np.mod(theta_deg, 360.0)
    if t < 120.0:
        return 1.0
    if t < 180.0:
        return 1.0 - 2.0 * (t - 120.0) / 60.0
    if t < 300.0:
        return -1.0
    return -1.0 + 2.0 * (t - 300.0) / 60.0


def bemf_volts(theta_e_deg, amp):
    """Three line-neutral BEMFs (volts), phases at 0/120/240 deg, amplitude amp."""
    return (amp * _trap(theta_e_deg),
            amp * _trap(theta_e_deg - 120.0),
            amp * _trap(theta_e_deg - 240.0))


# floating phase + zc polarity per 60 deg sector (matches firmware commutationTable)
SECTORS = [(2, +1), (0, -1), (1, +1), (2, -1), (0, +1), (1, -1)]

# ---- motor library (approx params; KV/poles drive the eRPM range + BEMF) -----
MOTORS = {
    "2810 1350KV (24V)":        dict(kv=1350, poles_pp=7, R=0.05, L=30e-6,
                                     vbus=24.0, erpm_max=235000),
    "Cobra CM-2814 470KV (24V)":dict(kv=470,  poles_pp=7, R=0.09, L=70e-6,
                                     vbus=24.0, erpm_max=92000),
    "XRotor 3110 1150KV (24V)": dict(kv=1150, poles_pp=7, R=0.04, L=24e-6,
                                     vbus=24.0, erpm_max=205000),
    "A2212 1000KV (12V)":       dict(kv=1000, poles_pp=7, R=0.11, L=55e-6,
                                     vbus=12.0, erpm_max=85000),
}


def vf_duty(erpm, motor):
    """Physical-ish V/f duty: duty ~ BEMF_peak / (Vbus/2), clamped."""
    w_m = (erpm / motor["poles_pp"]) / 60.0 * 2 * np.pi
    bemf_pk = KE_of(motor["kv"]) * w_m
    return float(np.clip(bemf_pk / (motor["vbus"] * 0.5), 0.05, 0.98))


def KE_of(kv):
    return 60.0 / (2 * np.pi * kv)


def capture_model(erpm, motor):
    """Mechanism model of ZC detection, calibrated to the 2810 bench sweep.

    rising sectors (even 0/2/4): detected by the ON-window HW comparator -> ~100%
      across the CL range (bench: rising never missed).
    falling sectors (odd 1/3/5): detected by the RC-filtered OFF-center SW sample.
      Limited by samples-per-sector (OFF-center sampler runs at PWM rate) AND the
      ~30us RC lag eating the post-crossing window. Logistic fit to the bench:
      88% @50-90k, 12% @90-130k, 0% >=130k  (samples/sector 6.4 -> 4.1 -> <=2.9).
    Returns capture fractions + the per-sector array (matches firmware sectors).
    """
    if erpm < 1500:
        return dict(spp=0, duty=0.05, rising=0.0, falling=0.0, measured=0.0,
                    per_sector=[0.0]*6)
    spp = PWM_HZ * (10.0 / erpm)             # OFF-center samples per 60deg sector
    duty = vf_duty(erpm, motor)
    rising = 1.0                              # ON-window comparator: always (in CL)
    # falling: logistic in samples/sector (center 5.0, width 0.6 -> fits bench)
    falling = 1.0 / (1.0 + np.exp(-(spp - 5.0) / 0.6))
    falling = float(np.clip(falling, 0.0, 1.0))
    measured = 0.5 * rising + 0.5 * falling
    per = [rising, falling, rising, falling, rising, falling]  # even=rising,odd=falling
    return dict(spp=float(spp), duty=duty, rising=rising, falling=falling,
                measured=float(measured), per_sector=per)


def bemf_curves(erpm, motor, n=720):
    """One electrical cycle of the 3 normalized BEMFs + the ZC angles, for plotting."""
    th = np.linspace(0, 360, n)
    a = np.array([_trap(x) for x in th])
    b = np.array([_trap(x - 120) for x in th])
    c = np.array([_trap(x - 240) for x in th])
    return th, a, b, c


def simulate(erpm, duty_frac, *, vbus=VBUS_DEFAULT, rc_fc=5500.0,
             spike_amp=0.0, spike_tau_us=0.6, ring_amp=0.0, ring_fhz=120e3,
             ring_tau_us=3.0, n_sub=64, sector_index=1, seed=0):
    """Simulate ONE sector's floating-phase signal + detection.

    sector_index: which of the 6 sectors (default 1 = a falling sector).
    Returns dict with time series (us) and detection results for both the
    OFF-center sampler and the ON-gated 1 MHz comparator.
    """
    rng = np.random.default_rng(seed)
    w_e = erpm / 60.0 * 2.0 * np.pi          # elec rad/s
    w_mech = w_e / POLES_PP
    E = KE * w_mech                          # BEMF amplitude (V)
    T_e = 1.0 / (erpm / 60.0)                # elec period (s)
    sector_T = T_e / 6.0                      # sector duration (s)
    T_pwm = 1.0 / PWM_HZ
    dt = T_pwm / n_sub
    n = max(8, int(round(sector_T / dt)))
    fphase, pol = SECTORS[sector_index % 6]

    rc = 1.0 / (2.0 * np.pi * rc_fc)
    alpha = dt / (rc + dt)                    # 1st-order LP coeff

    thr_adc = duty_frac * (vbus / 2.0) * ADC_PER_V   # firmware: duty*Vbus/2
    # sector spans the floating phase's BEMF ramp; center the ZC at mid-sector.
    # the floating phase's electrical angle sweeps its 60-deg ramp across the sector.
    theta0 = (120.0 if pol > 0 else 300.0)   # ramp start (rising at 120, falling at 300)

    t_us = np.empty(n); v_node = np.empty(n); v_sense = np.empty(n)
    e_float_arr = np.empty(n)
    filt = thr_adc
    spike = 0.0
    prev_on = None
    comp_fired_t = None      # first valid ON-window comparator detection (us)
    offc_fired_t = None      # first valid OFF-center detection (us)
    comp_on_min = 1e9; comp_on_max = -1e9     # comparator ON-window envelope (ADC)
    offc_min = 1e9; offc_max = -1e9           # OFF-center sample envelope (ADC)

    for k in range(n):
        t = k * dt
        frac = (t / sector_T)                  # 0..1 through the ramp
        theta_e = theta0 + 60.0 * frac         # floating phase sweeps its 60-deg ramp
        e = E * _trap(theta_e)                 # floating-phase BEMF (V), crosses 0 mid-ramp
        e_float_arr[k] = e

        # center-aligned PWM: ON in the central 'duty' fraction; OFF at the edges
        # (period boundary = OFF-center, where bemfRaw samples).
        ph = (t % T_pwm) / T_pwm               # 0..1 within PWM period
        on = abs(ph - 0.5) < (duty_frac / 2.0)

        # ideal floating-node voltage (V, ref to ground): ON ~ Vbus/2 + 3/2 e ;
        # OFF (freewheel) ~ 3/2 e around 0 (body-diode clamp at ~ -0.7V).
        if on:
            vn = vbus / 2.0 + 1.5 * e
        else:
            vn = 1.5 * e
            if vn < -0.7:
                vn = -0.7

        # ---- Layer 2 parasitics ----
        # switching-edge dv/dt spike: injected at each ON<->OFF transition, decays.
        if prev_on is not None and on != prev_on:
            spike = spike_amp * (1.0 if on else -0.6)   # rising edge bigger
        spike *= np.exp(-dt * 1e6 / max(0.05, spike_tau_us))
        # freewheel L-C resonance during OFF (patent: parasitic C * motor L)
        ring = 0.0
        if not on and ring_amp > 0:
            ring = ring_amp * np.exp(-(t % T_pwm) * 1e6 / max(0.1, ring_tau_us)) \
                   * np.sin(2 * np.pi * ring_fhz * (t % T_pwm))
        prev_on = on

        vnode_total = vn + spike + ring
        adc = np.clip(vnode_total * ADC_PER_V, 0, ADC_MAX)
        v_node[k] = adc
        filt = filt + alpha * (adc - filt)     # RC-filtered sense line
        v_sense[k] = filt
        t_us[k] = t * 1e6

        # ---- detectors ----
        # comparator: 1 MHz, gated to ON, reads the (lightly) filtered node.
        # model "lightly filtered" as the sense line (post-RC) + residual spike
        # that the RC can't fully kill at 1 MHz timescales.
        if on:
            comp_in = filt + 0.5 * spike * ADC_PER_V    # residual edge that passes RC
            comp_on_min = min(comp_on_min, comp_in)
            comp_on_max = max(comp_on_max, comp_in)
            if comp_fired_t is None:
                crossed = (comp_in > thr_adc + DEADBAND) if pol > 0 \
                          else (comp_in < thr_adc - DEADBAND)
                # plausibility: only accept past 1/4 of the sector (firmware floor)
                if crossed and frac > 0.25 and frac > 0.5:  # past the true ZC (mid)
                    comp_fired_t = t_us[k]

        # OFF-center sampler: once per PWM period at the boundary (ph wrap),
        # reads the RC-filtered line (clean average).
        if k > 0 and (t % T_pwm) < dt:
            offc_min = min(offc_min, filt)
            offc_max = max(offc_max, filt)
            if offc_fired_t is None and frac > 0.25:
                crossed = (filt > thr_adc + DEADBAND) if pol > 0 \
                          else (filt < thr_adc - DEADBAND)
                if crossed and frac > 0.5:
                    offc_fired_t = t_us[k]

    return {
        "t_us": t_us, "v_node": v_node, "v_sense": v_sense, "e_float": e_float_arr,
        "thr_adc": thr_adc, "polarity": pol, "fphase": fphase,
        "E_volts": E, "neutral_adc": thr_adc, "sector_us": sector_T * 1e6,
        "comp_fired_us": comp_fired_t, "offc_fired_us": offc_fired_t,
        "comp_on_env": (comp_on_min, comp_on_max),
        "offc_env": (offc_min, offc_max),
    }


def sweep(erpm_list, duty_fn, **kw):
    """Sweep eRPM; return per-band rising/falling capture% for both detectors.

    duty_fn(erpm)->duty_frac models the throttle/duty schedule.
    Capture = the detector fired a valid crossing within the sector.
    """
    out = []
    for erpm in erpm_list:
        duty = duty_fn(erpm)
        # rising sector (even, e.g. index 0) and falling sector (odd, index 1)
        r = simulate(erpm, duty, sector_index=0, **kw)
        f = simulate(erpm, duty, sector_index=1, **kw)
        out.append({
            "erpm": erpm, "duty": duty,
            "comp_rise": r["comp_fired_us"] is not None,
            "comp_fall": f["comp_fired_us"] is not None,
            "offc_rise": r["offc_fired_us"] is not None,
            "offc_fall": f["offc_fired_us"] is not None,
            "comp_on_env_fall": f["comp_on_env"],
            "offc_env_fall": f["offc_env"],
        })
    return out


def default_duty(erpm):
    """Rough V/f duty schedule matching the 2810 bench sweep."""
    return float(np.clip(0.06 + erpm / 240000.0, 0.05, 0.98))


if __name__ == "__main__":
    # quick standalone check: ideal (no parasitics) should detect BOTH polarities
    # in BOTH windows; turning on parasitics should mask falling in the comparator.
    for label, kw in [("IDEAL", dict(spike_amp=0.0, ring_amp=0.0)),
                      ("PARASITIC", dict(spike_amp=30.0, ring_amp=8.0))]:
        print(f"\n=== {label} ===")
        print(f"{'eRPM':>8} {'duty':>5} {'comp_rise':>10} {'comp_fall':>10} "
              f"{'offc_rise':>10} {'offc_fall':>10} {'comp_fall_env':>16}")
        for row in sweep([15000, 50000, 90000, 130000, 200000], default_duty, **kw):
            ce = row["comp_on_env_fall"]
            print(f"{row['erpm']:>8} {row['duty']:>5.2f} {str(row['comp_rise']):>10} "
                  f"{str(row['comp_fall']):>10} {str(row['offc_rise']):>10} "
                  f"{str(row['offc_fall']):>10}   {ce[0]:6.0f}..{ce[1]:6.0f}")
