"""
diagnose.py — the diagnostic brain.

Two tiers:
  1. Local rule engine (always available, no deps) — the seed of the v2 engine,
     encoding the fault→cause→fix table from bench experience.
  2. Claude deepening (optional) — packages the run + the knowledge base and asks
     claude-opus-4-8 for a structured diagnosis. Used only if `anthropic` is
     installed and ANTHROPIC_API_KEY is set; otherwise we return the local result.

Both consume a garuda_gsp.Session (live or loaded bundle) and return the same
shape, so the UI renders them identically.
"""
import json
import os

# ── Compact run summary (shared by both tiers; keeps Claude tokens small) ──
def summarize(session) -> dict:
    s = session.samples
    if not s:
        return {"info": session.info, "params": _params(session), "empty": True}

    def peak(key):
        return max((x.get(key, 0) or 0) for x in s)

    # commutation-health: reject rate over the run
    zc0, zc1 = s[0], s[-1]
    span = max(zc1.get("t", 0) - zc0.get("t", 0), 1e-3)
    acc = max(0, zc1.get("hwzc_zc", 0) - zc0.get("hwzc_zc", 0))
    rej = max(0, zc1.get("hwzc_reject", 0) - zc0.get("hwzc_reject", 0))
    rej_pct = min(100.0, 100.0 * rej / max(acc + rej, 1))   # can't exceed 100%

    per_state = {}
    for x in s:
        st = x.get("state_name", "?")
        d = per_state.setdefault(st, {"n": 0, "peak_eRPM": 0, "peak_Ia": 0.0,
                                       "t0": x["t"], "t1": x["t"]})
        d["n"] += 1
        d["peak_eRPM"] = max(d["peak_eRPM"], x.get("eRPM", 0))
        d["peak_Ia"] = max(d["peak_Ia"], x.get("ia_pk_mag", 0.0))
        d["t1"] = x["t"]

    return {
        "info": session.info,
        "params": _params(session),
        "duration_s": round(span, 1),
        "samples": len(s),
        "timeline": [(st, round(t0, 2), round(t1, 2), nxt)
                     for st, t0, t1, nxt in session.state_timeline()],
        "faults": [{"t": round(f.get("t", 0), 2), "fault": f.get("fault_name")}
                   for f in session.faults],
        "peak_eRPM": peak("eRPM"),
        "peak_Ia_A": round(peak("ia_pk_mag"), 1),
        "peak_Ibus_A": round(peak("ibus_pk_mag"), 1),
        "hwzc_accept": acc, "hwzc_reject": rej,
        "hwzc_reject_pct": round(rej_pct, 1),
        "per_state": {k: {"peak_eRPM": v["peak_eRPM"],
                          "peak_Ia": round(v["peak_Ia"], 1),
                          "dur_s": round(v["t1"] - v["t0"], 2)}
                      for k, v in per_state.items()},
    }


def _params(session):
    out = {}
    for k, v in (session.params or {}).items():
        out[k] = v.get("value") if isinstance(v, dict) else v
    return out


# ── Tier 1: local rule engine ──────────────────────────────────────────
def local_diagnose(session) -> dict:
    d = summarize(session)
    findings = []
    p = d.get("params", {})
    faults = {f["fault"] for f in d.get("faults", [])}
    last_state = d.get("timeline", [("?",)])[-1][0] if d.get("timeline") else "?"
    rejp = d.get("hwzc_reject_pct", 0)
    peak_ia = d.get("peak_Ia_A", 0)

    def add(cause, conf, evid, fixes):
        findings.append({"cause": cause, "confidence": conf,
                         "evidence": evid, "fixes": fixes})

    # config: param outside descriptor range (EEPROM/profileDefaults mismatch)
    for name, v in (session.params or {}).items():
        if isinstance(v, dict) and v.get("value") is not None \
           and v.get("min") is not None and v.get("max") is not None \
           and not (v["min"] <= v["value"] <= v["max"]):
            add(f"{name} is outside its descriptor range",
                "high", f"{name}={v['value']} vs [{v['min']}..{v['max']}]",
                [{"param": name, "current": str(v["value"]),
                  "suggested": "verify intended; profileDefaults bypasses SET_PARAM limits",
                  "why": "value the firmware would reject via SET_PARAM is active"}])

    # startup never left ramp
    if ("START_TO" in faults or "MORPH_TO" in faults) or \
       (last_state in ("ALIGN", "OL_RAMP", "MORPH") and d.get("peak_eRPM", 0) < 1000):
        add("Startup never reached closed loop (torque starvation or ramp too fast)",
            "high", f"ended in {last_state}, peak {d.get('peak_eRPM',0)} eRPM, faults {faults or 'none'}",
            [{"param": "sineRampModPct", "current": str(p.get("sineRampModPct")),
              "suggested": "raise (scale ∝ Rs vs 2810's 5%)", "why": "open-loop sine amplitude is the real startup torque knob"},
             {"param": "rampAccelErpmPerS", "current": str(p.get("rampAccelErpmPerS")),
              "suggested": "lower if it slips after moving", "why": "heavy/high-inertia rotor can't follow a fast ramp"}])

    # low-speed idle desync → OC_SW
    if "OC_SW" in faults and last_state in ("CL", "FAULT", "IDLE") and rejp > 80:
        add("Low-speed ZC-noise desync → wrong-angle commutation → OC_SW",
            "high", f"OC_SW with HWZC reject {rejp:.0f}%, peak Ia {peak_ia}A",
            [{"param": "zcBlankingPercent", "current": str(p.get("zcBlankingPercent")),
              "suggested": "raise", "why": "longer blanking rejects post-commutation PWM-noise ZCs"},
             {"param": "zcAdcDeadband", "current": str(p.get("zcAdcDeadband")),
              "suggested": "raise", "why": "wider deadband ignores low-eRPM switching noise"}])

    # overcurrent during startup
    if ("OC_SW" in faults or "BOARD_PCI" in faults) and last_state in ("ALIGN", "OL_RAMP"):
        add("Overcurrent during startup — amplitude too high for this Rs",
            "medium", f"OC in {last_state}, peak Ia {peak_ia}A",
            [{"param": "sineRampModPct", "current": str(p.get("sineRampModPct")),
              "suggested": "lower", "why": "low-Rs motor turns small amplitude into large current"}])

    # in-CL behaviour: regen, oscillation, intermittent lock (from raw samples)
    import statistics
    cl = [x for x in session.samples if x.get("state_name") == "CL"]
    if len(cl) >= 10:
        # windowed bus current — NOT the valley-sampled ibus_A (phantom -20A artifact)
        ibus_mean = sum(x.get("ibus_win_A", x.get("ibus_A", 0)) for x in cl) / len(cl)
        erpms = [x.get("eRPM", 0) for x in cl]
        emean = sum(erpms) / len(erpms)
        estd = statistics.pstdev(erpms) if len(erpms) > 1 else 0
        if ibus_mean < -10:
            add("Sustained regen in CL — commutation mistimed at this speed/duty (BEMF≈applied balance zone)",
                "high", f"mean Ibus in CL = {ibus_mean:.1f}A (heavy regen / shunt near rail)",
                [{"param": "timingAdvMaxDeg", "current": str(p.get("timingAdvMaxDeg")),
                  "suggested": "review advance at this eRPM band",
                  "why": "when BEMF≈applied V, a small late-commutation error flips motoring→regen"}])
        if emean > 0 and estd / emean > 0.03:
            add("eRPM oscillation — marginal sensorless lock",
                "medium", f"eRPM σ/μ = {100*estd/emean:.0f}% (μ={emean:,.0f})",
                [{"param": "zcFilterThreshold", "current": str(p.get("zcFilterThreshold")),
                  "suggested": "raise slightly", "why": "period estimate dithering between two values"}])
        notsync = sum(1 for x in cl if not x.get("synced", 1))
        if notsync > 0.3 * len(cl):
            add("BEMF lock intermittent in CL",
                "medium", f"{100*notsync/len(cl):.0f}% of CL samples report not-synced",
                [{"param": "hwzcCrossoverErpm", "current": str(p.get("hwzcCrossoverErpm")),
                  "suggested": "review crossover / blanking", "why": "sync flag dropping during CL"}])

    # regen OV
    if "OV" in faults:
        add("Regen overvoltage on throttle-down",
            "medium", "OV fault present",
            [{"param": "dutySlewDownPctPerMs", "current": str(p.get("dutySlewDownPctPerMs")),
              "suggested": "lower (slower down-slew)", "why": "limits regen energy dumped to the bus"}])

    sev = "critical" if faults else ("warning" if findings else "ok")
    summary = (f"{d.get('samples',0)} samples / {d.get('duration_s',0)}s, "
               f"peak {d.get('peak_eRPM',0):,} eRPM, HWZC reject {rejp:.0f}%. "
               + ("Faults: " + ", ".join(faults) if faults else "No faults.")) \
              if not d.get("empty") else "No telemetry captured."
    return {"summary": summary, "severity": sev, "findings": findings,
            "engine": "local-rules", "raw": d}


# ── Tier 2: Claude deepening (optional) ─────────────────────────────────
KNOWLEDGE = """You are an expert sensorless 6-step BLDC ESC firmware debugger for the
Garuda ESC (dsPIC33AK, MCLV-48V-300W). Startup is sine-amplitude driven:
IDLE→ARMED→ALIGN→OL_RAMP(sine V/f)→MORPH(sine→trap blend, needs >=4 real BEMF ZCs)→CL.
The REAL open-loop startup torque knob is sineAlignModPct/sineRampModPct (sine amplitude),
NOT the trap ALIGN_DUTY/RAMP_DUTY. Known fault→fix mappings:
- Stuck ALIGN/OL_RAMP, no rotation, START_TO/MORPH_TO: torque starvation → raise sineRampModPct (scale ~∝ Rs).
- Moves then desyncs / phantom eRPM in ramp: ramp too fast → lower rampAccelErpmPerS / rampTargetErpm.
- Idle/low-speed OC_SW with high HWZC reject%: PWM-noise phantom ZC desync → raise zcBlankingPercent/zcAdcDeadband/zcFilterThreshold.
- OC during ALIGN/OL_RAMP: amplitude too high (low Rs) → lower sineAlign/RampModPct.
- High-speed desync: BEMF filter phase lag → filter comp / PWM gate.
- Regen OV on throttle-down → lower dutySlewDownPctPerMs / Vbus emergency hold.
- maxClosedLoopErpm cap plateau → raise maxClosedLoopErpm.
- Param outside descriptor range → stale EEPROM shadowing code (use FEATURE_PARAMS_FORCE_DEFAULTS or reset EEPROM).
Enforce invariants: rampTarget>initial; maxCL>rampTarget; ocSwLimit<ocLimit<=ocFault; ocStartup>=ocLimit.
Do NOT suggest dead-ends: raising rampTargetErpm past the OL-slip ceiling; in-loop I-smoothing below loop bandwidth."""

DIAGNOSIS_SCHEMA = {
    "type": "object",
    "properties": {
        "summary": {"type": "string"},
        "severity": {"type": "string", "enum": ["ok", "warning", "critical"]},
        "findings": {"type": "array", "items": {
            "type": "object",
            "properties": {
                "cause": {"type": "string"},
                "confidence": {"type": "string", "enum": ["low", "medium", "high"]},
                "evidence": {"type": "string"},
                "fixes": {"type": "array", "items": {
                    "type": "object",
                    "properties": {
                        "param": {"type": "string"},
                        "current": {"type": "string"},
                        "suggested": {"type": "string"},
                        "why": {"type": "string"},
                    },
                    "required": ["param", "current", "suggested", "why"],
                    "additionalProperties": False,
                }},
            },
            "required": ["cause", "confidence", "evidence", "fixes"],
            "additionalProperties": False,
        }},
    },
    "required": ["summary", "severity", "findings"],
    "additionalProperties": False,
}


def claude_available() -> bool:
    if not os.environ.get("ANTHROPIC_API_KEY"):
        return False
    try:
        import anthropic  # noqa
        return True
    except ImportError:
        return False


def claude_diagnose(session) -> dict:
    """Structured diagnosis from claude-opus-4-8. Raises if unavailable."""
    import anthropic
    client = anthropic.Anthropic()
    payload = json.dumps(summarize(session), default=str)
    resp = client.messages.create(
        model="claude-opus-4-8",
        max_tokens=16000,
        thinking={"type": "adaptive"},
        output_config={
            "effort": "high",
            "format": {"type": "json_schema", "schema": DIAGNOSIS_SCHEMA},
        },
        system=KNOWLEDGE,
        messages=[{"role": "user", "content":
                   "Diagnose this Garuda ESC bench run. Return ranked root causes "
                   "with the exact parameter changes to make.\n\nRUN:\n" + payload}],
    )
    text = next((b.text for b in resp.content if b.type == "text"), "{}")
    out = json.loads(text)
    out["engine"] = "claude-opus-4-8"
    return out


def diagnose(session, prefer_claude=True) -> dict:
    """Entry point: Claude if configured, else local rules. Never raises."""
    if prefer_claude and claude_available():
        try:
            return claude_diagnose(session)
        except Exception as e:  # noqa — degrade gracefully to local
            local = local_diagnose(session)
            local["summary"] = f"[Claude unavailable: {e}] " + local["summary"]
            return local
    return local_diagnose(session)
