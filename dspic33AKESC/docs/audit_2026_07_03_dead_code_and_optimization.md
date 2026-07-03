# Full Code Audit — Dead Code & Optimization (2026-07-03)

Build audited: flagship `dspic33AK512MC510.X`, MOTOR_PROFILE=9 (U3), GARUDA_TARGET_AK512,
HWZC sector-PI engine, PWM 45 kHz (ADC tick 22.2 µs), CPU 200 MHz.
Method: 4 parallel read-only audit passes (config flags, service loop, module/file inventory,
ISR cycle costs), conflicts re-verified against source.

## Verdict in one line

CPU is NOT the bottleneck (~10–12 % total, not the 96 % one pass claimed — that pass used the
wrong tick period); the codebase's real burdens are ~7,000 lines of dead/superseded code, a
noise-storm interrupt load at low duty, and a handful of overlapping mechanisms that make every
new feature harder to reason about.

## A. Dead weight — safe to remove now (zero behavioral risk)

| Item | Size | Evidence |
|---|---|---|
| `dspic33AKESC_RK 1/` backup dir | 880 K | not referenced by any project |
| `dspic33AKESC.X/` stale MPLAB project | 3.5 M | superseded by dspic33AK512MC510.X |
| FOC stack: `foc/` (17 .c), `garuda_foc_params.h`, FEATURE_FOC/V2/V3/AN1078/SMO/MXLEMMING | ~4,085 gated LOC / 328 K + 39 K | all flags 0 since 2026-05-25; unreachable |
| `learn/` (6 .c) FEATURE_LEARN_MODULES | ~1,185 gated LOC / 72 K | flag 0; FOC-era commissioning |
| x2cscope lib + diagnostics.c | 152 K | GSP is the protocol; mutually exclusive |
| `motor/pi.c` MC_ControllerPIUpdate/Reset | 19 LOC | never called (FOC had inline PI) |
| `BuildFocMotorParams()` in garuda_service.c | ~30 LOC | marked `__attribute__((unused))` |
| 17 orphan constants (ARM_BEEP_*×6, IF_BRIDGE_*×5, CL_ENTRY_START_PCT/RAMP_MS, HANDOFF_CHOP_MS, OC_CMP3_HANDOFF_MA…) | — | gated by flags that are 0 |
| Falsified experiments: FEATURE_IF_BRIDGE (59), FEATURE_HWZC_FALLING_SW (95), FEATURE_HWZC_FREEWHEEL_GATE (29), FEATURE_HANDOFF_CHOP (32), FEATURE_ARM_BEEP (31), FEATURE_HWZC_LOWSPD_OFFCTR (42), FEATURE_THROTTLE_ZERO_AUTO_DISARM (60 — superseded by planned debounced version) | ~350 LOC | bench-falsified or superseded; git history preserves them |

KEEP despite being off (documented, promising, or pending re-integration):
FEATURE_CL_COAST_VERIFY (261 LOC, bench 6/6), FEATURE_SPEED_CASCADE (222, WS4 revival planned),
FEATURE_PLL_STARTUP (138), FEATURE_IBUS_PROBE (145), FEATURE_VBUS_SAG_LIMIT (61, re-tune planned),
FEATURE_HWZC_FALLING_OFFWIN (39, valid under CCM/load — DCM-falsified prop-less only).

KEEP the legacy SW-ZC path (~309 LOC in service + motor/bemf_zc.c ~850 LOC) for now: it is the
miss-limit fallback and recovery re-seed. Retire it only when the software-neutral detection
engine (Atomberg-style) lands and proves itself — then ~1,160 LOC goes at once.

## B. Defects / redundancies found (functional, not cosmetic)

1. **Two soft-start mechanisms compiled simultaneously**: FEATURE_CL_ENTRY_SOFTSTART=1
   (CL_ENTRY_* ramp + slew baseline at service ~4069/4109/4136) AND FEATURE_CL_SOFT_ENTRY=1
   (softCap ramp, added 2026-07-03). They overlap on CL entry; behavior is the intersection of
   two caps. Pick one (CL_SOFT_ENTRY is the bench-tuned one) and retire the other.
2. **Duplicate duty-floor block**: service ~4341–4350 re-implements ~4099–4105 (3-way
   MIN_DUTY/CL_DIFF_IDLE/CL_LOW_IDLE floor). Extract one helper.
3. **Write-only shadow observer**: FEATURE_BEMF_INTEGRATION=1 runs 27 lines + 12 struct fields
   at 45 kHz inside the HWZC branch; nothing reads them (telemetry only, and not surfaced in GSP
   console). Flip to 0 for free cycles + clarity; re-enable when actually A/B-ing the observer.
4. **DIAGNOSTIC_MANUAL_STEP dead twin case** for ESC_CLOSED_LOOP (~3252) — compile-out clutter.
5. ~25 diag fields written at tick rate, consumed at ≤10 Hz. Harmless individually; batch or
   gate the expensive ones if the ISR ever tightens.

## C. Performance — measured shape and ranked wins

Total CPU today: ~10–12 % (idle and top speed similar; ADC ISR dominates).
There is headroom; optimize for architectural cleanliness and the noise storm, not for survival.

| # | Win | Mechanism | Est. saving | Risk |
|---|---|---|---|---|
| 1 | **Comparator noise-storm suppression** | At low duty the comparator re-fires ~10⁵–10⁶/s during PWM-OFF; each rejected fire costs an ISR entry + gate checks. Instead of clear-flag+re-arm on every reject, leave IE masked until the next PWM-ON edge (rising sectors accept only in ON anyway); today the same rejection outcome is bought with 20× more interrupts | ~2–3 % CPU at idle, and removes ISR-jitter coupling into ZC latency | MEDIUM — touches the detection core; A/B on bench with cap=/diag; keep flag-gated |
| 2 | **Spindown-floor batching** | Floor block runs 3–4 divisions every 22 µs tick; floor dynamics are mechanical (ms-scale). Compute every 8th tick (or at commutation) and cache; also early-skip when mappedDuty ≥ cached floor | ~1.3–1.5 % CPU | LOW |
| 3 | **Shadow observer off** (B.3) | flag flip | ~0.5 % + RAM | NONE |
| 4 | **Cache eRPM once per tick** | hwzc and service both derive speed (independent divides) | ~0.3 % | LOW |
| 5 | Timing-advance LUT; verify-wait tuning; integer PI | minor cycle trims | ~0.3–1 % combined | LOW–HIGH (integer PI is HIGH: float PI is bench-validated at 203k — don't touch without a campaign) |

## D. Suggested sequencing

- **Phase 0 (hygiene, one commit, no hex change expected in behavior):** delete archive dir +
  stale project; remove falsified-experiment blocks + orphan constants + pi.c dead fns +
  BuildFocMotorParams; flip FEATURE_BEMF_INTEGRATION=0; resolve the dual soft-start (keep
  CL_SOFT_ENTRY); dedupe the floor block. Re-run full bench regression card once.
- **Phase 1 (perf, cheap):** floor batching + eRPM cache (one commit, verify pot-0 glide still
  clean at 16 V and 20 V).
- **Phase 2 (perf, guarded):** comparator masking behind FEATURE_HWZC_OFF_MASK, A/B with cap=
  telemetry and per-sector diag at idle/mid/top.
- **Phase 3 (structural, later):** extract the ESC_CLOSED_LOOP duty pipeline from
  garuda_service.c (5,188 lines) into its own module; retire FOC/learn directories from the
  project file; retire SW-ZC after the software-neutral engine proves out.

## Cross-refs
- Session docs: `session_2026_07_03_u3_load_desync_root_cause.md`
- Memory: project-u3-load-desync-campaign (run log incl. today's floor latch + caprate watchdog)
