# Session 2026-07-03 — The U3 Load-Desync Root Cause Hunt

**Motor:** T-Motor U3 KV700 (7pp), profile 9, 16V bench (later 20–24V), prop-less hand/towel load
**Starting symptom (months old):** motor desyncs under ~2A of load; full speed no-load fine
**Ending state:** 11.6A peak real load clean at 52% duty; 96,581 eRPM record @20.9V;
root cause captured on the burst scope, fixed with one constant; full protection stack live.
**Commits:** `7a12de7..98df2a0` (this session), building on `bea206b/7552233/20bef27` (gate + confounder strips).

---

## TL;DR — what the "load desync" actually was

Three stacked problems, peeled in order:

1. **Vbus sag limiter mis-calibrated** (`20bef27`): engage threshold was effectively 16.8V
   on a 16V bench — always on, strangled duty under any load. Confounder; stripped.
2. **PWM-OFF capture gate disabled** for the U3 (`bea206b` re-enabled): genuine Mode-A
   slips (one bad ZC capture in a PWM-OFF window → 60° slip). Re-enabling moved the
   ceiling 4.6A → 6.5A. Real, but not the core disease.
3. **THE ROOT CAUSE** (`651a3a8`): the 2026-06-26 "WS5 correction" changed
   `OC_GAIN_X100` 2495 → 830, silently rescaling every OC threshold to ~1/3 of its
   label. The **software OC soft limiter** (`OC_SW_LIMIT_MA` 18000, proportional
   per-cycle duty cut) therefore engaged at **~6A real** instead of 18A. Load ripple
   peaks crossing ~6A → duty chopped mid-sector → BEMF dead at the ZC sample point →
   one corrupted capture → 60° slip → re-lock onto a false slow rhythm (~1/8 speed
   "phantom lock") → 21A circulating → PSU collapse → UNDERVOLTAGE. The 53↔5% duty
   flap seen in every phantom tail all campaign *was* the limiter chopping.

### The gain proof (resolves the DIM contradiction permanently)
- A PSU CC-limited at 10A ran a steady load for 30+ s that the 830-gain scale read as
  10.5A bus — impossible. At 2495 (93 counts/A) the same raw reads 3.49A = telemetry
  = PSU actual.
- The phantom-lock rail always latched 21.4A on the 93 scale = exact ADC clip at a
  22A full scale — physically consistent (66A FS would imply dead FETs).
- **93 counts/A / 24.95 gain / 22A FS is correct for this board.** The Microchip
  reference project's 8.3 datasheet value does not match this hardware.

---

## Forensic chain (20 bench runs, all in `esc_ak.py` logs)

| Run | Config | Result |
|-----|--------|--------|
| 1–2 | baseline full FW | cascade oscillations; sag limiter convicted (duty 62→5% autonomously) |
| 3 | sag+cascade off | **original disease clean**: healthy droop to ~4.6A, then one-sample 21.4A rail → UV |
| 4–5 | PWM gate on | full-pot loaded 15 s clean; 52% duty survived to 5.6A (was 4.6) |
| 6 | PSU limit 10A | OCP tripped at the slam → PSU exonerated, event draws >10A |
| 7 | (perA set failed) | full slam anatomy on 10 Hz telemetry: phantom lock 0.6 s, duty flap 53↔5 |
| 8–9 | perA=8 flashed | low-duty CL regression (eRPM 3k↔7k thrash) → WS1 duty gate (`c820e9a`) |
| 10–11 | perA 8→12, blankmax 33 | slams persist, trend *backwards* → demag hypothesis dead |
| 12 | auto-diag | **miss table frozen, bias trendless vs load** → discrete event, not degradation |
| 13 | WS2 live | phantom executed in 50 ms, Vbus never sagged — but 2 false stalls at 12%/4% duty |
| 14 | scope capture #1 | flawless 128-sample recording while WS2 stall-faulted at full speed → **WS2 threshold was ~3.9A real, not 12A** → gain contradiction cracked open |
| 15 | scope capture #2 | **root cause on camera**: first anomaly is a duty cut at ripple ≈ 6A real; chop → bemf=0 → slip → phantom |
| 16 | gain 2495 flashed | **confirmation**: 9.7A avg / 11.6A peak real at 52% duty, 15+ s, zero anomalies; full-pot fault = pure PSU 10A foldback with sector sequence perfect through the collapse |
| 17 | towel drag 22% duty | eRPM 17k→7.8k→recovered, no fault; NEW: odd-sector-only miss growth at low BEMF (polarity asymmetry, backlog) |
| 18 | 12–17.8A real load | correctly-scaled limiter engaged at 18A and chop-slipped (same mechanism at the right threshold) → chop gate (`71b160c`) |
| 19 | startup cycles | 7 consecutive textbook startups; pot-0 auto-disarm proved noise-triggered → reverted (`98df2a0`) |
| 20 | 20.9V | **96,581 eRPM record**, miss table frozen 82–96k, rode live 15→24V PSU swings, pot-0 idle hold confirmed |

---

## Fixes landed (chronological)

| Commit | Change |
|--------|--------|
| `bea206b` | FEATURE_HWZC_PWM_GATE 1 for profile 9 (Mode-A killer, from the 2810 playbook) |
| `7a12de7`/`ed8072f` | esc_ak.py GSP get/set param commands (live tuning from bench console) |
| `f1b87b9` | quiet-idle console: settings at connect, telemetry streams only when running |
| `0feab56` | WS1 zcDemagBlankPerA=8 baked (later proven not the fix, kept with duty gate) |
| `c820e9a` | WS1 duty gate: load-adaptive blank engages only ≥ zcDemagDutyThresh (40%) — phase-current samples are garbage at low duty |
| `4622739` | auto-DIAG at 1 Hz in closed loop (nine runs had zero manual diag under load) |
| `e40bc09` | WS2 phase-stall backstop ON for profile 9 + burst-scope auto-capture in console (auto-arm on CL entry, threshold trigger on bus current, auto-dump through fault) |
| `c6d703c` | WS2 duty-gated debounce (freeze not reset — phantom flaps duty); scope read flow control (one chunk in flight; burst overflowed fw TX ring) |
| `83eab13` | stallIphaseAdc 360→1300 (~14A real); scope units → real amps |
| `651a3a8` | **OC_GAIN_X100 830→2495** — the root-cause fix; all OC labels true amps again |
| `71b160c` | HWZC chop gate: limiter sets `hwzc.chopBlank=2` on every duty cut; capture ISR rejects ZC in chopped windows (missed > false; towel run proved 60% misses survivable) |
| `3b1499d` | startup smoothing: FEATURE_CL_SOFT_ENTRY on (500 ms, /64 slew), MORPH_CONVERGE_SECTORS 12→24, live ramptarget/rampaccel knobs |
| `8a7f1a8`→`98df2a0` | pot-0 auto-disarm tried, then reverted: hasSeenThrottle latches on ONE noisy ISR pot sample ≥200 (idle pot ~40 spikes past 200 between 10 Hz frames) → self-disarm ~1.5 s after CL entry. Pot 0 = idle speed until the latch is debounced (sustained ≥THROTTLE_START_ADC 400 for ~100 ms). |

## Protection stack (all bench-proven today)

- **PWM-OFF gate**: rejects OFF-window comparator fires (Mode-A killer).
- **Chop gate**: rejects captures in SW-limiter-chopped cycles (18A-event killer).
- **SW OC soft limiter**: proportional duty cut from 18A real → CMP3 DAC (20A, parked).
- **WS2 phase-stall**: ≥1300 counts (~14A) phase for 50 ms at ≥40% duty → FAULT_STALL
  in 50 ms. Turned the 21A/0.8s/8V-collapse phantom into a benign blip (Vbus min 13V).
- **SW OC hard fault** 21A, **UV/OV**, morph timeout, desync recovery: unchanged.

## Tooling (esc_ak.py) end state

Quiet-idle console; settings + frozen-capture recovery at connect; auto-DIAG 1 Hz in CL
(bias, per-sector caps/miss); burst scope auto-arm (bus ≥15A real, 75% pre-trigger,
128 samples @~24 kHz: sector, duty, eRPM, ia, ibus, vbus, zcTh, bemf) with auto-dump
that survives the fault; live params: pera, peradb, blankmax, blankextra, stalladc,
stallarm, ramptarget, rampaccel. Scope/telemetry both in real amps (93 counts/A).

## Records & data points

- **96,581 eRPM @ 20.9V** (~13.8k mech) — miss table frozen through 82–96k cruise.
- **11.6A peak real bus** at 52% duty, 15+ s — was dying at 2–4.5A before.
- Rode live PSU swings 15→24V without a capture hiccup; zcTh tracks Vbus correctly.
- Towel drag: 22% duty dragged 17k→7.8k eRPM and back, no fault.
- ADC current channel clips at ~22A (2048 counts / 93 per amp).

## Backlog (in priority order)

1. **>100k eRPM exploration**: maxClosedLoopErpm clamp is 98000 (20V anchor), grazed
   at 96.5k. Live param 0x11; raise in steps (110k first) with scope armed. 24V wants
   ~117k at full duty.
2. **SW limiter chop→slew redesign**: chop gate protects detection, but a slewed CL
   duty command would keep the motor *driving* through an 18A event instead of
   chattering. (Chop gate is the mitigation, not the endgame.)
3. **Rising/falling ZC polarity asymmetry** at low BEMF (towel run: odd sectors only
   missed, 9→101 while evens frozen) — comparator/threshold offset compensation.
4. **Pot-0 stop, done right**: debounce hasSeenThrottle (sustained ≥400 for ~100 ms),
   optional pot filtering; then FEATURE_THROTTLE_ZERO_AUTO_DISARM or full
   FEATURE_POT_START_STOP.
5. **OC DAC audit post-gain-revert**: CMP3 thresholds now mean true amps — verify DAC
   derivations end-to-end (CLPCI currently disabled, so latent).
6. **WS4 speed cascade revival** on this now-solid detection foundation; sag limiter
   re-tune with measured-Vbus thresholds.
7. React GUI Vbus scale 19.8 → 23.2 (esc_ak.py already corrected).

## Cross-references

- `docs/akesc_session_2026_05_26_191k_to_243k.md` — the 2810 campaign that produced
  the PWM gate and failure taxonomy reused here.
- `garuda_config.h` OC_GAIN_X100 comment block — the bench proof, inline.
- Memory: `project-u3-load-desync-campaign.md` (full run-by-run log, 20 entries).
