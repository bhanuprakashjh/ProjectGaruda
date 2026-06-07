"""garuda zcml - offline ZC-position estimator (small MLP) for sensorless 6-step.

Idea (after Energies 2023 16(10):4027): instead of thresholding a single ZC edge,
regress the rotor position within each floating sector from the conditioned
phase-voltage trajectory. Trained on burst-scope captures (the un-aliased 24 kHz
waveform) with the TRUE ZC extracted offline as the label -> learns the falling
crossing the live comparator can't cleanly threshold.

Pipeline:
  GUI "Collect ML" -> sessions/ml_*.jsonl (per-capture bemf/zcthr/sector + erpm/duty)
  extract_examples()  -> per floating-sector trajectory (resampled) + true ZC frac
  MLP (pure numpy)    -> predict ZC fraction in [0,1]  (x60deg = elec position)
  evaluate()          -> MAE in elec degrees vs eRPM band, rising vs falling

Pure numpy, no sklearn/torch. `python zcml.py` runs a synthetic self-test.
"""
from __future__ import annotations
import glob
import json
import numpy as np

SEG_LEN = 16          # resample each sector trajectory to this many points
MIN_RUN = 5           # min samples in a sector-run to use it


# ---- dataset I/O -----------------------------------------------------------
def load_jsonl(paths):
    caps = []
    for p in (paths if isinstance(paths, (list, tuple)) else glob.glob(paths)):
        with open(p) as f:
            for line in f:
                line = line.strip()
                if line:
                    caps.append(json.loads(line))
    return caps


def _resample(arr, n):
    arr = np.asarray(arr, float)
    if arr.size == n:
        return arr
    xp = np.linspace(0, 1, arr.size)
    return np.interp(np.linspace(0, 1, n), xp, arr)


def _find_zc(run, pol_sign):
    """Robust ZC = the polarity-correct, SUSTAINED crossing of (bemf - zcthr).
    pol_sign +1 (rising) wants a -> + transition; -1 (falling) wants + -> -.
    Requires the run to actually be on the expected side before and after, so a
    lone PWM-ON spike that briefly crosses doesn't get mistaken for the ZC.
    Returns the fraction (0..1) of the crossing within the run, or None."""
    n = run.size
    pre = max(1, n // 4)
    for k in range(1, n):
        if pol_sign > 0:
            cross = run[k - 1] <= 0 < run[k]
        else:
            cross = run[k - 1] >= 0 > run[k]
        if not cross:
            continue
        # sustained: the samples after the crossing must stay on the new side
        tail = run[k:min(n, k + pre)]
        if pol_sign > 0 and np.mean(tail) <= 0:
            continue
        if pol_sign < 0 and np.mean(tail) >= 0:
            continue
        f = run[k - 1] / (run[k - 1] - run[k] + 1e-9)
        return (k - 1 + f) / (n - 1)
    return None


def extract_examples(captures):
    """Each capture -> per floating-sector trajectory. Returns X, y, meta.
      X    : [SEG_LEN normalized (bemf-zcthr) samples, duty_frac, polarity] per sector
      y    : true ZC fraction within the sector (0..1)
      meta : (erpm, polarity) per example  (polarity +1 rising / -1 falling)
    """
    X, y, meta = [], [], []
    for c in captures:
        bemf = np.asarray(c["bemf"], float)
        zcthr = np.asarray(c["zcthr"], float)
        sector = np.asarray(c["sector"], float).astype(int)
        erpm = float(c.get("erpm", 0))
        duty = float(c.get("duty", 0)) / 100.0
        d = bemf - zcthr                       # BEMF relative to threshold
        # split into runs of constant sector
        i = 0
        n = len(sector)
        while i < n:
            j = i
            while j < n and sector[j] == sector[i]:
                j += 1
            run = d[i:j]
            if (j - i) >= MIN_RUN:
                pol_sign = +1 if (sector[i] % 2 == 0) else -1
                zc_frac = _find_zc(run, pol_sign)
                if zc_frac is not None:
                    pol = +1.0 if (sector[i] % 2 == 0) else -1.0  # even=rising
                    traj = _resample(run, SEG_LEN)
                    sc = np.max(np.abs(traj)) + 1e-6
                    feat = np.concatenate([traj / sc, [duty, pol]])
                    X.append(feat); y.append(zc_frac); meta.append((erpm, pol))
            i = j
    return np.array(X), np.array(y), meta


# ---- tiny MLP (pure numpy) -------------------------------------------------
class MLP:
    def __init__(self, n_in, n_hid=24, seed=0):
        rng = np.random.default_rng(seed)
        self.W1 = rng.normal(0, 1 / np.sqrt(n_in), (n_in, n_hid))
        self.b1 = np.zeros(n_hid)
        self.W2 = rng.normal(0, 1 / np.sqrt(n_hid), (n_hid, 1))
        self.b2 = np.zeros(1)
        self.mu = None; self.sd = None

    def _norm(self, X):
        return (X - self.mu) / self.sd

    def fit(self, X, y, epochs=800, lr=0.08):
        self.mu = X.mean(0); self.sd = X.std(0) + 1e-6
        Xn = self._norm(X); yy = y.reshape(-1, 1)
        n = len(Xn)
        for _ in range(epochs):
            z1 = Xn @ self.W1 + self.b1
            a1 = np.tanh(z1)
            pred = a1 @ self.W2 + self.b2
            err = pred - yy
            dW2 = a1.T @ err / n; db2 = err.mean(0)
            da1 = (err @ self.W2.T) * (1 - a1 ** 2)
            dW1 = Xn.T @ da1 / n; db1 = da1.mean(0)
            self.W2 -= lr * dW2; self.b2 -= lr * db2
            self.W1 -= lr * dW1; self.b1 -= lr * db1
        return self

    def predict(self, X):
        a1 = np.tanh(self._norm(X) @ self.W1 + self.b1)
        return (a1 @ self.W2 + self.b2)[:, 0]


# ---- evaluation ------------------------------------------------------------
def evaluate(model, X, y, meta):
    pred = model.predict(X)
    err_deg = np.abs(pred - y) * 60.0          # sector = 60 elec deg
    erpm = np.array([m[0] for m in meta])
    pol = np.array([m[1] for m in meta])
    bands = [(0, 50000), (50000, 90000), (90000, 130000), (130000, 999999)]
    # The honest baseline = best possible CONSTANT guess (always predict the mean
    # ZC fraction of that subset). 'floor' = its MAE; no constant can beat it.
    # 'win' = floor - MLP. win>0 means the MLP predicts each crossing, not just
    # the average offset. win~0 means it only memorised the mean (useless).
    print(f"{'eRPM band':>11} {'n':>4} | "
          f"{'rise MLP':>8} {'rise floor':>10} {'win':>5} | "
          f"{'fall MLP':>8} {'fall floor':>10} {'win':>5}   (elec deg)")
    for lo, hi in bands:
        m = (erpm >= lo) & (erpm < hi)
        if not m.any():
            continue
        lab = f"{lo//1000}-{hi//1000}k" if hi < 999999 else f">{lo//1000}k"
        cells = []
        for sgn in (+1, -1):
            ms = m & (pol == sgn)
            if ms.any():
                mae = err_deg[ms].mean()
                floor = (np.abs(y[ms] - y[ms].mean()) * 60.0).mean()
                cells.append(f"{mae:>8.2f} {floor:>10.2f} {floor-mae:>+5.1f}")
            else:
                cells.append(f"{'—':>8} {'—':>10} {'—':>5}")
        print(f"{lab:>11} {m.sum():>4} | {cells[0]} | {cells[1]}")
    return err_deg


# ---- synthetic self-test (no bench data needed) ----------------------------
def _synth_captures(n_caps=400, seed=1):
    """Trapezoidal floating-phase BEMF crossing threshold at a known fraction,
    sampled at a rate that drops with eRPM (samples/sector), with falling-sector
    ON-window corruption (the masking) added as noise. Lets us prove the MLP can
    still recover the ZC from the trajectory where a simple threshold struggles."""
    rng = np.random.default_rng(seed)
    caps = []
    for _ in range(n_caps):
        erpm = float(rng.uniform(15000, 180000))
        duty = float(np.clip(6 + erpm / 2700, 5, 98))
        spp = max(4, int(450000 / erpm))       # samples per sector ~ like the bench
        seq = []
        sect = int(rng.integers(0, 6))
        bemf, zcthr, sector = [], [], []
        for _s in range(6):                    # 6 sectors per capture
            zc_frac = float(rng.uniform(0.4, 0.6))
            pol = +1 if (sect % 2 == 0) else -1
            thr = 600 * duty / 100.0
            for k in range(spp):
                fr = k / max(1, spp - 1)
                # BEMF ramps through neutral at zc_frac (pol sets direction)
                val = thr + pol * (fr - zc_frac) * 1400
                if pol < 0:                    # falling: add ON-window corruption
                    val += rng.normal(0, 120) + (300 if rng.random() < 0.25 else 0)
                else:
                    val += rng.normal(0, 30)
                bemf.append(val); zcthr.append(thr); sector.append(sect)
            sect = (sect + 1) % 6
        caps.append({"erpm": erpm, "duty": duty,
                     "bemf": bemf, "zcthr": zcthr, "sector": sector})
    return caps


if __name__ == "__main__":
    print("=== zcml synthetic self-test ===")
    caps = _synth_captures()
    X, y, meta = extract_examples(caps)
    print(f"examples: {len(X)}  feature dim: {X.shape[1] if len(X) else 0}")
    ntr = int(0.8 * len(X))
    idx = np.random.default_rng(0).permutation(len(X))
    tr, te = idx[:ntr], idx[ntr:]
    model = MLP(X.shape[1]).fit(X[tr], y[tr])
    # baseline: predict 0.5 (mid-sector) always
    base = np.abs(0.5 - y[te]).mean() * 60.0
    print(f"\nbaseline (always mid-sector) MAE: {base:.2f} elec deg")
    print("MLP held-out test:")
    evaluate(model, X[te], y[te], [meta[i] for i in te])
