"""garuda_sil — Python harness for the REAL dsPIC33AK ESC firmware compiled
to x86 (libgaruda_sil.so) behind a mock HAL, plus a simple simulated 2810.

Time model (DESIGN.md): logical, deterministic. Master tick = 1 PWM period
(1/45000 s). Each tick is sub-stepped; priority-7 events (SCCP1 deadline,
ADC digital comparator) fire inside sub-steps, then the ADC ISR runs once,
then Timer1 (100 us) from its own accumulator.

BEMF / terminal-voltage synthesis follows tools/garuda_debug/garuda_gui/
zcsim.py: divider ~52 ADC counts/V (Vbus/2 = 12 V -> ~625 counts), the
sense line is the 5.5 kHz RC-filtered node, PWM-ON node = Vn + e_float with
Vn the conduction-pair virtual neutral (≈ duty*Vbus/2).
"""
from __future__ import annotations

import math
import os
import shutil
import tempfile

try:
    import cffi
    _HAVE_CFFI = True
except ImportError:                                  # pragma: no cover
    _HAVE_CFFI = False
    import ctypes

HERE = os.path.dirname(os.path.abspath(__file__))
SO_PATH = os.path.join(HERE, "..", "libgaruda_sil.so")

PWM_HZ = 45000.0
SCCP2_HZ = 100e6
TICKS_PER_PWM = SCCP2_HZ / PWM_HZ                    # 2222.22
T1_PERIOD_TICKS = 10000                              # 100 us @ 100 MHz

ADC_PER_V = 625.0 / 12.0                             # zcsim ADC scaling
IBUS_COUNTS_PER_A = 93.0                             # fw scope convention
IBUS_BIAS = 2048

# ESC_STATE_T (garuda_types.h)
STATES = ["IDLE", "ARMED", "DETECT", "ALIGN", "OL_RAMP", "MORPH",
          "CLOSED_LOOP", "BRAKING", "RECOVERY", "FAULT", "IF_RAMP"]
FAULTS = ["NONE", "OV", "UV", "OC", "BOARD_PCI", "STALL", "DESYNC",
          "STARTUP_TIMEOUT", "MORPH_TIMEOUT", "RX_LOSS", "FOC_INTERNAL",
          "OBSERVER", "FOC_BUSLOSS", "TRAP_BUS", "TRAP_ILLEGAL", "TRAP_ADDR"]

LEG_PWM, LEG_LOW, LEG_HIZ = 0, 1, 2

CDEF = r"""
typedef struct {
    uint32_t pdc[3];
    uint8_t  leg_mode[3];
    uint8_t  bemf_mux;
    uint8_t  hs_pinsel;
    uint16_t cmp_thresh[3];
    uint8_t  cmp_rising[3];
    uint16_t cmp3_dac;
    uint8_t  sccp1_armed;
    uint64_t sccp1_deadline;
    uint64_t sccp2_now;
    uint32_t sccp3_period;
    uint8_t  sccp3_running;
    uint32_t writes_set_step;
    uint32_t writes_duty;
    uint8_t  last_step;
} SilHw;

void sil_init(void);
void sil_adc_isr(void);
void sil_timer1_isr(void);
void sil_cct1_isr(void);
void sil_cmp_isr(int core);
SilHw *sil_hw(void);
void sil_set_time(uint64_t sccp2_ticks);
void sil_set_adc(uint16_t phaseB, uint16_t phaseAC, uint16_t pot,
                 uint16_t vbus, uint16_t ibus, uint16_t ia, uint16_t ib);
void sil_set_cmp_adc(uint16_t ad1ch5, uint16_t ad2ch1);
void sil_set_pwm_on(int on);
int      sil_leg_mode(int leg);
uint32_t sil_pdc(int leg);
int      sil_bemf_mux(void);
int      sil_hs_pinsel(void);
int      sil_cmp_ie(int core);
int      sil_cmp_rising(int core);
uint16_t sil_cmp_thresh(int core);
int      sil_sccp1_armed(void);
uint64_t sil_sccp1_deadline(void);
int      sil_cct1_ie(void);
int      sil_state(void);
int      sil_fault(void);
uint32_t sil_duty(void);
int      sil_step(void);
int      sil_direction(void);
int      sil_zc_synced(void);
uint16_t sil_step_period(void);
uint32_t sil_hwzc_period(void);
int      sil_hwzc_enabled(void);
int      sil_hwzc_phase(void);
uint16_t sil_good_zc(void);
uint32_t sil_hwzc_total_zc(void);
uint32_t sil_erpm(void);
uint32_t sil_systick(void);
uint16_t sil_throttle(void);
uint16_t sil_zc_threshold(void);
uint16_t sil_bemf_raw(void);
int      sil_morph_subphase(void);
uint16_t sil_morph_alpha(void);
uint16_t sil_sw_good_zc(void);
uint16_t sil_sine_amplitude(void);
uint32_t sil_sine_erpm(void);
int      sil_run_command(void);
uint16_t sil_arm_counter(void);
uint16_t sil_ramp_step_period(void);
uint32_t sil_min_duty(void);
uint32_t sil_looptime_tcy(void);
volatile uint8_t  *sil_cmp_ie_ptr(int core);
volatile uint8_t  *sil_cct1_ie_ptr(void);
volatile uint32_t *sil_cmp_data_ptr(int core);
void sil_cmd_start(void);
void sil_cmd_stop(void);
void sil_cmd_fault_clear(void);
"""


def _trap(t):
    """zcsim trapezoid: +1 flat 0..120, ramp down 120..180 (zero at 150),
    -1 flat 180..300, ramp up 300..360 (zero at 330)."""
    t = t % 360.0
    if t < 120.0:
        return 1.0
    if t < 180.0:
        return 1.0 - 2.0 * (t - 120.0) / 60.0
    if t < 300.0:
        return -1.0
    return -1.0 + 2.0 * (t - 300.0) / 60.0


class Plant2810:
    """Quasi-static 2810 1350KV plant (M1 fidelity, DESIGN.md).

    BEMF: trapezoid per phase from rotor electrical angle, with
      e_A = E*g(th), e_B = E*g(th+120), e_C = E*g(th-120)
    where g rises through zero at th=0 (sector centers then line up with
    the firmware commutationTable: step 4 = A rising, spans [-30,+30)).

    Electrical: R-only conduction (L/R ~ 0.3 ms >> PWM period is handled
    by using AVERAGE duty voltage per sub-step — DESIGN.md sanctions a
    quasi-static torque model for M1).
    """

    def __init__(self, vbus=24.0, kv=1350.0, pp=7, r_ll=0.05,
                 j=2.0e-5, b_visc=1.1573e-4, tau_c=0.0, v_drop_a=0.0847, v_drop_c=0.0,
                 rc_fc=5500.0, phase_fix_deg=0.0, theta0_deg=200.0):
        self.vbus = vbus
        self.pp = pp
        self.ke_ll = 60.0 / (2.0 * math.pi * kv)     # V*s/rad mech, line-line
        self.ke_e = self.ke_ll / 2.0                 # line-neutral amplitude coeff
        self.r_ll = r_ll
        self.r_ln = r_ll / 2.0
        self.j = j
        self.b_visc = b_visc
        self.tau_c = tau_c
        # drive volt-loss = a + c*Vbus: 'a' = diode/deadtime floor, the
        # c*Vbus term = switching/deadtime loss scaling with the bus.
        # (a,c) fit EXACTLY to the 2026-06-11 bench equilibrium ladder
        # (24.3->10.4k, 14.2->5.75k, 13.1->5.3k) with b_visc small enough
        # for a realistic seconds-scale coast-down (J/b ~ 1.5 s).
        self.v_drop_a = v_drop_a
        self.v_drop_c = v_drop_c
        self.rc_tau = 1.0 / (2.0 * math.pi * rc_fc)
        self.phase_fix = phase_fix_deg
        # state
        self.theta_e = math.radians(theta0_deg)      # electrical angle (rad)
        self.omega_m = 0.0                           # mech rad/s
        self.filt = [0.0, 0.0, 0.0]                  # RC-filtered terminal volts
        self.i_ph = [0.0, 0.0, 0.0]
        self.ibus = 0.0
        self.ia_pk = 0.0

    # -- helpers ----------------------------------------------------------
    def erpm(self):
        return abs(self.omega_m) * 60.0 / (2.0 * math.pi) * self.pp

    def bemf(self):
        th = math.degrees(self.theta_e) + self.phase_fix
        e = self.ke_e * self.omega_m
        return (e * _trap(th), e * _trap(th + 120.0), e * _trap(th - 120.0)), th

    def substep(self, modes, duties_v, dt):
        """modes: 3x LEG_*, duties_v: 3x average leg volts (PWM legs only).
        Returns nothing; updates internal state."""
        (eA, eB, eC), th = self.bemf()
        e = (eA, eB, eC)
        g = (_trap(th), _trap(th + 120.0), _trap(th - 120.0))

        v = [0.0, 0.0, 0.0]
        driven = []
        for k in range(3):
            m = modes[k]
            if m == LEG_PWM:
                v[k] = duties_v[k]
                driven.append(k)
            elif m == LEG_LOW:
                v[k] = 0.0
                driven.append(k)

        i = [0.0, 0.0, 0.0]
        if len(driven) == 3:
            vn = (v[0] + v[1] + v[2]) / 3.0 - (e[0] + e[1] + e[2]) / 3.0
            for k in range(3):
                i[k] = (v[k] - e[k] - vn) / self.r_ln
        elif len(driven) == 2:
            a, c = driven
            drive = (v[a] - v[c]) - (e[a] - e[c])
            # series conduction drop (FET/diode) — bench-fit 0.078 V
            vdrop = self.v_drop_a + self.v_drop_c * self.vbus
            if drive > vdrop:
                drive -= vdrop
            elif drive < -vdrop:
                drive += vdrop
            else:
                drive = 0.0
            cur = drive / self.r_ll
            i[a] = cur
            i[c] = -cur
            vn = (v[a] + v[c]) / 2.0 - (e[a] + e[c]) / 2.0
        else:
            vn = 0.0

        # torque + mechanics
        t_e = self.ke_e * (g[0] * i[0] + g[1] * i[1] + g[2] * i[2])
        t_fr = self.b_visc * self.omega_m + (
            self.tau_c if self.omega_m > 0.5 else
            (-self.tau_c if self.omega_m < -0.5 else 0.0))
        self.omega_m += (t_e - t_fr) / self.j * dt
        self.theta_e += self.omega_m * self.pp * dt
        if self.theta_e > 1e6:                       # keep bounded
            self.theta_e = math.fmod(self.theta_e, 2.0 * math.pi)

        # terminal voltages (floating phase = Vn + e, diode-clamped)
        vt = [0.0, 0.0, 0.0]
        for k in range(3):
            if modes[k] == LEG_HIZ:
                x = vn + e[k]
                if x < -0.7:
                    x = -0.7
                elif x > self.vbus + 0.7:
                    x = self.vbus + 0.7
                vt[k] = x
            else:
                vt[k] = v[k]

        a = dt / (self.rc_tau + dt)
        f = self.filt
        f[0] += a * (vt[0] - f[0])
        f[1] += a * (vt[1] - f[1])
        f[2] += a * (vt[2] - f[2])

        self.i_ph = i
        self.ibus = (v[0] * i[0] + v[1] * i[1] + v[2] * i[2]) / self.vbus
        m = abs(i[0])
        if m > self.ia_pk:
            self.ia_pk = m


def _adc(volts):
    c = int(volts * ADC_PER_V)
    return 0 if c < 0 else (4095 if c > 4095 else c)


class Sim:
    """One deterministic SIL instance: fresh copy of libgaruda_sil.so
    (private statics), Plant2810, logical-time loop."""

    def __init__(self, plant=None, pot=40, n_sub=4, so_path=SO_PATH):
        self.plant = plant or Plant2810()
        self.pot = pot
        self.n_sub = n_sub
        # private dlopen so firmware statics are per-instance
        self._tmp = tempfile.NamedTemporaryFile(
            prefix="garuda_sil_", suffix=".so", delete=False)
        shutil.copyfile(so_path, self._tmp.name)
        if _HAVE_CFFI:
            self.ffi = cffi.FFI()
            self.ffi.cdef(CDEF)
            self.lib = self.ffi.dlopen(self._tmp.name)
        else:                                        # pragma: no cover
            raise RuntimeError("ctypes fallback not wired — install cffi")

        self.lib.sil_init()
        self.hw = self.lib.sil_hw()
        self._cmp_ie = (None, self.lib.sil_cmp_ie_ptr(1),
                        self.lib.sil_cmp_ie_ptr(2))
        self._cmp_data = (None, self.lib.sil_cmp_data_ptr(1),
                          self.lib.sil_cmp_data_ptr(2))
        self._cct1_ie = self.lib.sil_cct1_ie_ptr()
        self.looptime = float(self.lib.sil_looptime_tcy())
        self.min_duty = self.lib.sil_min_duty()

        self.tick = 0
        self.t_ticks = 0.0                           # sccp2 ticks (float)
        self.t1_next = float(T1_PERIOD_TICKS)
        self.trace = []                              # (tick, state) transitions
        self._last_state = self.lib.sil_state()
        self.trace.append((0, self._last_state))
        self.lib.sil_set_pwm_on(1)                   # HWZC PWM gate always passes
        self.vbus_counts = _adc(self.plant.vbus)

    def close(self):
        try:
            os.unlink(self._tmp.name)
        except OSError:
            pass

    # ------------------------------------------------------------------
    def cmd_start(self):
        self.lib.sil_cmd_start()

    def cmd_stop(self):
        self.lib.sil_cmd_stop()

    def state(self):
        return self.lib.sil_state()

    def state_name(self):
        return STATES[self.lib.sil_state()]

    def fault_name(self):
        f = self.lib.sil_fault()
        return FAULTS[f] if f < len(FAULTS) else str(f)

    # ------------------------------------------------------------------
    def _fire_comparators(self):
        """Level-sensitive digital comparator model: fires when the watched
        RC-filtered phase crosses the configured threshold while IE is on."""
        hw = self.hw
        p = self.plant
        for core in (1, 2):
            if not self._cmp_ie[core][0]:
                continue
            if core == 1:
                val = _adc(p.filt[1])                # AD1CH5 = phase B
            else:
                ph = 0 if hw.hs_pinsel == 10 else 2  # AD2CH1 pinsel: 10=A, 7=C
                val = _adc(p.filt[ph])
            rising = hw.cmp_rising[core]
            thr = hw.cmp_thresh[core]
            if (val > thr) if rising else (val < thr):
                self._cmp_data[core][0] = val        # verify-reads see this
                self.lib.sil_cmp_isr(core)

    def step_tick(self):
        """Advance one PWM period."""
        hw = self.hw
        p = self.plant
        lib = self.lib
        dt = 1.0 / (PWM_HZ * self.n_sub)
        sub_ticks = TICKS_PER_PWM / self.n_sub
        inv_loop = self.plant.vbus / self.looptime

        for _ in range(self.n_sub):
            self.t_ticks += sub_ticks
            now = int(self.t_ticks)
            hw.sccp2_now = now

            modes = (hw.leg_mode[0], hw.leg_mode[1], hw.leg_mode[2])
            duties_v = (hw.pdc[0] * inv_loop,
                        hw.pdc[1] * inv_loop,
                        hw.pdc[2] * inv_loop)
            p.substep(modes, duties_v, dt)

            # SCCP1 one-shot deadline(s) — may chain (blanking can be 1 us)
            guard = 0
            while (hw.sccp1_armed and self._cct1_ie[0]
                   and hw.sccp1_deadline <= now and guard < 8):
                hw.sccp2_now = hw.sccp1_deadline     # exact event timestamp
                lib.sil_cct1_isr()
                hw.sccp2_now = now
                guard += 1

            # ADC digital comparator events
            if self._cmp_ie[1][0] or self._cmp_ie[2][0]:
                self._fire_comparators()

        # tick boundary: ADC ISR consumes the RC-filtered samples
        phaseB = _adc(p.filt[1])
        ph_ac = 0 if hw.bemf_mux == 10 else 2
        phaseAC = _adc(p.filt[ph_ac])
        ibus = int(IBUS_BIAS + p.ibus * IBUS_COUNTS_PER_A)
        ibus = 0 if ibus < 0 else (4095 if ibus > 4095 else ibus)
        ia = int(IBUS_BIAS + p.i_ph[0] * IBUS_COUNTS_PER_A) & 0xFFF
        ib = int(IBUS_BIAS + p.i_ph[1] * IBUS_COUNTS_PER_A) & 0xFFF
        lib.sil_set_adc(phaseB, phaseAC, self.pot, self.vbus_counts,
                        ibus, ia, ib)
        lib.sil_set_cmp_adc(phaseB, _adc(p.filt[0 if hw.hs_pinsel == 10 else 2]))
        lib.sil_adc_isr()

        # Timer1 from its own 100 us accumulator (4.5 PWM ticks average)
        while self.t_ticks >= self.t1_next:
            lib.sil_timer1_isr()
            self.t1_next += T1_PERIOD_TICKS

        self.tick += 1
        s = lib.sil_state()
        if s != self._last_state:
            self.trace.append((self.tick, s))
            self._last_state = s

    def run(self, n_ticks, until_state=None, fail_state=None):
        for _ in range(n_ticks):
            self.step_tick()
            if until_state is not None and self.lib.sil_state() == until_state:
                return True
            if fail_state is not None and self.lib.sil_state() == fail_state:
                return False
        return until_state is None

    # ------------------------------------------------------------------
    def snapshot(self):
        lib = self.lib
        return dict(
            tick=self.tick,
            t_ms=self.tick / PWM_HZ * 1e3,
            state=STATES[lib.sil_state()],
            fault=self.fault_name(),
            duty=lib.sil_duty(),
            duty_pct=100.0 * lib.sil_duty() / self.looptime,
            step=lib.sil_step(),
            zc_synced=lib.sil_zc_synced(),
            step_period=lib.sil_step_period(),
            hwzc_en=lib.sil_hwzc_enabled(),
            hwzc_period=lib.sil_hwzc_period(),
            fw_erpm=lib.sil_erpm(),
            plant_erpm=self.plant.erpm(),
            throttle=lib.sil_throttle(),
            zc_thr=lib.sil_zc_threshold(),
            bemf_raw=lib.sil_bemf_raw(),
            morph_sub=lib.sil_morph_subphase(),
            ibus=self.plant.ibus,
            ia=self.plant.i_ph[0],
        )

    def trace_named(self):
        return [(t, STATES[s]) for (t, s) in self.trace]
