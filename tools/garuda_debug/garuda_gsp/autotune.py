"""autotune.py — guarded closed-loop parameter tuning on the LIVE motor.

Drives the board over GSP (throttle source -> GSP, start motor, heartbeat) and
sweeps ONE whitelisted parameter to minimise a cost function measured from
telemetry, then restores the original value. The motor is real, so this is
SUPERVISED, not fire-and-forget — keep a hand on the bench supply.

Hard safety rails (any trip -> emergency stop + restore):
  * heartbeat dead-man — a background thread pings the firmware watchdog; if this
    process dies the firmware safe-stops the motor on its own.
  * abort on ANY fault, |Ibus| over limit, or eRPM over the cap.
  * only SAFE_PARAMS may be tuned — never the protection limits (OC/UV/OV/maxRPM).
  * the original param value (and a stop) are always restored on exit/abort.

Ia is NOT used for safety (the phase-current offset issue) — Ibus is.
"""
import threading
import time

from . import protocol as P

# Params that are safe to sweep — they shape behaviour but cannot disable
# protection. The protection/limit params (ocFaultMa, ocSwLimitMa, ocLimitMa,
# ocStartupMa, vbusOvAdc, vbusUvAdc, maxClosedLoopErpm, motorPolePairs,
# hwzcCrossoverErpm, desyncMaxRestarts, rampCurrentGateMa) are deliberately
# excluded — the tuner refuses them.
SAFE_PARAMS = {
    "rampTargetErpm", "rampAccelErpmPerS", "rampDutyPct", "clIdleDutyPct",
    "timingAdvMaxDeg", "alignDutyPct", "initialErpm", "sineAlignModPct",
    "sineRampModPct", "zcDemagDutyThresh", "zcDemagBlankExtraPct",
    "dutySlewUpPctPerMs", "dutySlewDownPctPerMs", "postSyncSettleMs",
    "postSyncSlewDivisor", "zcBlankingPercent", "zcAdcDeadband",
    "zcSyncThreshold", "zcFilterThreshold",
}

THROTTLE_SRC_GSP = 1
THROTTLE_SRC_ADC = 0
HEARTBEAT_S = 0.1                 # well under any firmware timeout
THROTTLE_STEP = 50               # ramp rate (units per ~50 ms) — no slamming


def _ibus(s):
    return abs(s.get("ibus_win_A", s.get("ibus_A", 0)) or 0)


class MotorSession:
    """Take GSP control with a heartbeat dead-man; guarantee a safe stop on exit.

    with MotorSession(client) as m:
        m.ramp_to(throttle); ... ; m.check(snap)
    """
    def __init__(self, client, ibus_abort=20.0, erpm_abort=245000):
        self.c = client
        self.ibus_abort = ibus_abort
        self.erpm_abort = erpm_abort
        self._hb_run = False
        self._hb = None
        self.throttle = 0

    def __enter__(self):
        self.c.stop_motor()                 # ensure IDLE so source switch is legal
        time.sleep(0.2)
        self.c.clear_fault()
        self.c.set_throttle_src(THROTTLE_SRC_GSP)
        self.c.set_throttle(0)
        self._hb_run = True
        self._hb = threading.Thread(target=self._beat, daemon=True)
        self._hb.start()
        self.c.start_motor()
        return self

    def _beat(self):
        while self._hb_run:
            try:
                self.c.heartbeat()
            except Exception:
                pass
            time.sleep(HEARTBEAT_S)

    def ramp_to(self, target, step=THROTTLE_STEP):
        target = max(0, min(2000, int(target)))
        while self.throttle != target:
            if self.throttle < target:
                self.throttle = min(target, self.throttle + step)
            else:
                self.throttle = max(target, self.throttle - step)
            self.c.set_throttle(self.throttle)
            s = self.c.get_snapshot()
            r = self.check(s)
            if r:
                return r
            time.sleep(0.05)
        return None

    def check(self, snap):
        """Return an abort reason string if unsafe, else None."""
        f = (snap.get("fault_name") or "NONE").strip()
        if f not in ("NONE", ""):
            return f"fault:{f}"
        if _ibus(snap) > self.ibus_abort:
            return f"ibus>{self.ibus_abort}A ({_ibus(snap):.1f})"
        e = snap.get("eRPM", 0) or 0
        if e > self.erpm_abort:
            return f"eRPM>{self.erpm_abort} ({e})"
        return None

    def emergency_stop(self):
        try:
            self.c.set_throttle(0)
            self.c.stop_motor()
        except Exception:
            pass

    def __exit__(self, *a):
        self._hb_run = False
        self.emergency_stop()
        time.sleep(0.2)
        try:                                 # best-effort restore of pot control
            self.c.set_throttle_src(THROTTLE_SRC_ADC)
        except Exception:
            pass


# ── cost functions over a window of snapshots (lower = better) ─────────────
def _delta(samples, key):
    a = samples[0].get(key, 0) or 0
    b = samples[-1].get(key, 0) or 0
    return max(0, b - a)


def cost_miss_rate(samples):
    acc = _delta(samples, "hwzc_zc")
    miss = _delta(samples, "hwzc_miss")
    tot = acc + miss
    return (miss / tot) if tot else 1.0


def cost_erpm_cv(samples):
    es = [s.get("eRPM", 0) or 0 for s in samples if (s.get("eRPM", 0) or 0) > 0]
    if len(es) < 3:
        return 1.0
    mu = sum(es) / len(es)
    if mu <= 0:
        return 1.0
    var = sum((e - mu) ** 2 for e in es) / len(es)
    return (var ** 0.5) / mu


def cost_neg_top_speed(samples):
    return -max((s.get("eRPM", 0) or 0) for s in samples)   # maximise eRPM


COSTS = {
    "miss_rate": cost_miss_rate,
    "erpm_stability": cost_erpm_cv,
    "top_speed": cost_neg_top_speed,
}


def _summary(samples):
    if not samples:
        return {}
    e = [s.get("eRPM", 0) or 0 for s in samples]
    return {"eRPM_mean": round(sum(e) / len(e)), "eRPM_peak": round(max(e)),
            "ibus_peak": round(max(_ibus(s) for s in samples), 1)}


# ── the guarded sweep ──────────────────────────────────────────────────────
def sweep_param(client, param, lo, hi, steps=5, throttle=300, settle_s=2.5,
                cost="miss_rate", ibus_abort=20.0, erpm_abort=245000, on_step=None):
    """Sweep `param` over [lo,hi] in `steps`, spin at `throttle` (0..2000), and
    measure `cost` after settling. Restores the original value. Returns the cost
    curve, the best value, and any safety abort. SUPERVISE this — it spins the
    motor."""
    if param not in SAFE_PARAMS:
        raise ValueError(f"'{param}' is not in the safe-to-tune whitelist "
                         f"(protection/limit params are refused)")
    if cost not in COSTS:
        raise ValueError(f"unknown cost '{cost}'; use {sorted(COSTS)}")
    pid = P.PARAM_IDS[param]
    cost_fn = COSTS[cost]
    orig = client.get_param(pid)
    vals = [int(round(lo + (hi - lo) * i / max(1, steps - 1))) for i in range(steps)]

    results, aborted = [], None
    with MotorSession(client, ibus_abort, erpm_abort) as m:
        r = m.ramp_to(throttle)
        if r:
            aborted = {"value": "ramp", "reason": r}
        else:
            for v in vals:
                client.set_param(pid, v)
                samples, t_end = [], time.monotonic() + settle_s
                while time.monotonic() < t_end:
                    s = client.get_snapshot()
                    reason = m.check(s)
                    if reason:
                        aborted = {"value": v, "reason": reason}
                        break
                    samples.append(s)
                    time.sleep(0.05)
                if aborted:
                    break
                c = cost_fn(samples)
                row = {"value": v, "cost": round(c, 4), "summary": _summary(samples)}
                results.append(row)
                if on_step:
                    on_step(row)

    client.set_param(pid, orig)                  # always restore the original
    best = min(results, key=lambda r: r["cost"]) if results else None
    return {"param": param, "cost_metric": cost, "throttle": throttle,
            "original": orig, "results": results, "best": best,
            "aborted": aborted, "applied": False,
            "note": "original value restored; apply 'best' via set_param + "
                    "save_config if you want to keep it"}
