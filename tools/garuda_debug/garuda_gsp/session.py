"""
Session — the shareable, self-describing unit of a bench run.

A bundle captures everything needed to diagnose a run offline, so a colleague
can run a test on any motor and just send the bundle:

    <name>/
      meta.json        identity: fw, buildHash, board, profile, PP, features, host, times
      params.json      full GET_PARAM dump (value + min/max) at capture time
      telemetry.csv    the timeseries (one row per snapshot)
      faults.json      fault transitions with the sample at the moment of fault

The diagnostic engine (v2) consumes a Session and appends diagnosis.json.
"""
import csv
import json
import os

SCHEMA_VERSION = 1

# Stable column order for telemetry.csv (extra keys appended deterministically).
CORE_COLS = [
    "t", "state_name", "fault_name", "throttle", "duty", "vbus_V", "ibus_A",
    "eRPM", "good_zc", "synced", "hwzc_zc", "hwzc_miss", "hwzc_reject",
    "ia_pk_mag", "ib_pk_mag", "ibus_pk_mag", "step_period", "uptime",
    "spi_en", "spi_target", "spi_error", "spi_output", "spi_integ",
    "cpu_load_pct",
    "miss_s0", "miss_s1", "miss_s2", "miss_s3", "miss_s4", "miss_s5",
    "zc_thresh", "fall_off_min", "fall_off_max",
]


class Session:
    def __init__(self, info=None, params=None, host=None, tool_version=None,
                 started_at=None):
        self.info = info or {}
        self.params = params or {}
        self.samples = []          # list[dict] decoded snapshots
        self.faults = []           # list[dict] {t, fault_name, sample}
        self.host = host or {}
        self.tool_version = tool_version
        self.started_at = started_at
        self.notes = ""

    # ── building ────────────────────────────────────────────────────────
    def add(self, sample: dict):
        """Append a sample; auto-detect a fault edge for the fault log."""
        if not sample or "state" not in sample:
            return
        prev = self.samples[-1] if self.samples else None
        self.samples.append(sample)
        was = prev["fault"] if prev else 0
        now = sample.get("fault", 0)
        if now != 0 and now != was:
            self.faults.append({"t": sample.get("t"),
                                "fault_name": sample.get("fault_name"),
                                "sample": sample})

    # ── persistence ─────────────────────────────────────────────────────
    def save(self, path: str):
        os.makedirs(path, exist_ok=True)
        meta = {
            "schemaVersion": SCHEMA_VERSION,
            "info": self.info,
            "host": self.host,
            "toolVersion": self.tool_version,
            "startedAt": self.started_at,
            "sampleCount": len(self.samples),
            "durationS": (self.samples[-1]["t"] - self.samples[0]["t"])
                         if len(self.samples) >= 2 else 0.0,
            "notes": self.notes,
        }
        with open(os.path.join(path, "meta.json"), "w") as f:
            json.dump(meta, f, indent=2)
        with open(os.path.join(path, "params.json"), "w") as f:
            json.dump(self.params, f, indent=2)
        with open(os.path.join(path, "faults.json"), "w") as f:
            json.dump(self.faults, f, indent=2)
        # Auto-extra columns: scalar keys only (skip list/dict-valued fields
        # like miss_by_sector, which we flatten into miss_s0..5 below).
        extra = {k for s in self.samples for k, v in s.items()
                 if not isinstance(v, (list, dict, tuple))}
        cols = CORE_COLS + sorted(
            extra - set(CORE_COLS)
            - {"state", "fault", "state_name", "fault_name"})
        with open(os.path.join(path, "telemetry.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
            w.writeheader()
            for s in self.samples:
                mbs = s.get("miss_by_sector")
                if mbs:
                    row = dict(s)
                    for i in range(min(6, len(mbs))):
                        row[f"miss_s{i}"] = mbs[i]
                    w.writerow(row)
                else:
                    w.writerow(s)
        return path

    @classmethod
    def load(cls, path: str) -> "Session":
        with open(os.path.join(path, "meta.json")) as f:
            meta = json.load(f)
        s = cls(info=meta.get("info"), host=meta.get("host"),
                tool_version=meta.get("toolVersion"),
                started_at=meta.get("startedAt"))
        s.notes = meta.get("notes", "")
        pf = os.path.join(path, "params.json")
        if os.path.exists(pf):
            with open(pf) as f:
                s.params = json.load(f)
        ff = os.path.join(path, "faults.json")
        if os.path.exists(ff):
            with open(ff) as f:
                s.faults = json.load(f)
        with open(os.path.join(path, "telemetry.csv")) as f:
            for row in csv.DictReader(f):
                conv = {}
                for k, v in row.items():
                    if v == "" or v is None:
                        continue
                    try:
                        conv[k] = int(v)
                    except ValueError:
                        try:
                            conv[k] = float(v)
                        except ValueError:
                            conv[k] = v
                s.samples.append(conv)
        return s

    # ── convenience ─────────────────────────────────────────────────────
    def state_timeline(self):
        """[(state_name, t_start, t_end, next_state)] from the samples."""
        if not self.samples:
            return []
        out, cur, t0 = [], self.samples[0]["state_name"], self.samples[0]["t"]
        for s in self.samples[1:]:
            if s["state_name"] != cur:
                out.append((cur, t0, s["t"], s["state_name"]))
                cur, t0 = s["state_name"], s["t"]
        out.append((cur, t0, self.samples[-1]["t"], None))
        return out
