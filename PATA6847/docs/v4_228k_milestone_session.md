# V4 Sector-PI: 228k eRPM Milestone Session Log

**Date:** 2026-04-29
**Hardware:** EV43F54A board, dsPIC33CK64MP205 + ATA6847, A2212 1400KV motor @ 24-25V
**Result:** Peak 228k eRPM @ 96% effective duty, stable for tens of seconds, no desync.
**Starting point:** 130k eRPM @ 56% duty wall.

This document captures every bug fix, what worked, what didn't, and where to tune for further work. All references use `file:line` notation against commits ending at `9df971b`.

---

## 1. Final keeper configuration

| Knob | File:line | Value | Why |
|------|-----------|-------|-----|
| PWM frequency | `garuda_6step_ck.X/garuda_config.h:26` | `60000U` | Lower current ripple → cleaner BEMF on float phase. Verified vs 40/50 kHz. |
| MIN_OFF clamp (PWM duty cap) | `garuda_6step_ck.X/hal/hal_pwm.c:234` | `per - 100U` (~96.9% effective at 60 kHz) | ATA6847 has charge pumps; bootstrap-refresh argument doesn't apply. |
| V4 minimum amplitude (idle floor) | `garuda_6step_ck.X/motor/sector_pi.c:1001` | `5000U` (~15.3% Q15) | Pot-zero stable, no desync. Was 4000 (12.2%) — desyncs at 60 kHz. |
| Phase advance — PI model | `garuda_6step_ck.X/garuda_config.h:736` | `V4_PHASE_ADVANCE_DEG = 10.0f` | The proven 196k baseline ratio with scheduler ramp. |
| Phase advance — scheduler | `garuda_6step_ck.X/motor/sector_pi.c:830-836` | TAL=2 (≥260 HR), TAL=3 (130-260), TAL=4 (<130) | Speed-adaptive 15° / 22.5° / 30°. |
| Blanking percent | `garuda_6step_ck.X/motor/v4_params.c:44` | `25` (% of sector) | Matches old hardcoded `>> 2`. |
| PI Kp shift | `garuda_6step_ck.X/garuda_config.h:748` | `2` (Kp = 1/4) | Proven 196k baseline. Doubling didn't help peak; made it twitchy. |
| PI Ki shift | `garuda_6step_ck.X/garuda_config.h:749` | `4` (Ki = 1/16) | Unchanged from baseline. |
| CCP2 ISR | `garuda_6step_ck.X/hal/hal_capture.c:102` | `_CCP2IE = 0` | V4 Mode 1 doesn't use this capture path. |
| CCP5 ISR | `garuda_6step_ck.X/hal/hal_capture.c:108` | `_CCP5IE = 0` | Same — only FIFO drain cost. |

---

## 2. Where to change things (tuning guide)

### 2.1 Maximum duty cap (the most-asked knob)

**File:** `garuda_6step_ck.X/hal/hal_pwm.c:234`
```c
uint32_t maxD = (per > 100U) ? (uint32_t)(per - 100U) : (uint32_t)per;
```

Change the literal `100U` to set MIN_OFF in PWM ticks (5 ns each):

| Value | MIN_OFF time | Effective duty cap @ 60 kHz | Notes |
|-------|--------------|------------------------------|-------|
| `200U` | 1.0 µs | 93.7% | Old conservative bootstrap-safe value |
| `100U` | 500 ns | **96.9% (current keeper)** | ATA6847 charge pump can do this |
| `50U` | 250 ns | 98.4% | Tight but datasheet allows |
| `0U` | 0 ns | 100% | True 100% — only the dsPIC deadtime remains |

**Note:** The macro `MAX_DUTY` in `garuda_config.h:50,54` is **dead in V4** — it's only used by `HAL_PWM_SetDutyCycle()` which V4 doesn't call. The V4 path goes through `HAL_PWM_SetDutyCyclePeriod()` which has its own inline clamp at line 234.

### 2.2 PWM frequency

**File:** `garuda_6step_ck.X/garuda_config.h:26`
```c
#define PWMFREQUENCY_HZ     60000U
```

Tested values:
- 40 kHz: walls at 195k. Sample undersampling above ~178k.
- 50 kHz: 220k. Right at ATA6847 spec'd switching limit with 100 nC FETs.
- **60 kHz: 228k (keeper).**

Going higher needs verification of FET gate-charge headroom. ATA6847 datasheet caps at 50 kHz with 100 nC FETs.

### 2.3 Phase advance

**File:** `garuda_6step_ck.X/garuda_config.h:736`
```c
#define V4_PHASE_ADVANCE_DEG    10.0f
```

This is the PI's `setValue` model. **The scheduler ramp (`sector_pi.c:830-836`) is intentionally different (15°/22.5°/30° based on speed)** — the PI/scheduler mismatch is load-bearing. Don't unify them.

### 2.4 Idle / minimum running duty

**File:** `garuda_6step_ck.X/motor/sector_pi.c:1001`
```c
#define V4_MIN_AMPLITUDE  5000U  /* ~15.3% duty */
```

Below this the motor desyncs at low speed. Higher = faster idle, smoother but loses creep range. Lower = risk of pot-zero desync at 60 kHz.

### 2.5 Blanking (post-commutation BEMF mask)

**File:** `garuda_6step_ck.X/motor/v4_params.c:44` (default)
```c
v4Params.blankingPct = 25;
```

Live-tunable via `SET_PARAM 0xF3`. Range 10-60%. Smaller = more detection window at speed, but lets demag transients through.

### 2.6 PI gains

**File:** `garuda_6step_ck.X/garuda_config.h:748,749`
```c
#define V4_KP_SHIFT             2       /* Kp = 1/(2^2) = 1/4 */
#define V4_KI_SHIFT             4       /* Ki = 1/(2^4) = 1/16 */
```

Live-tunable via `SET_PARAM 0xF1` (Kp shift) and `0xF2` (Ki shift). Smaller shift = larger gain.

---

## 3. Bug fixes (chronological with file:line)

### Fix 1: Build hash in telemetry — defeats "wrong hex flashed" confusion

**Commit:** `0248478`
**File:** `garuda_6step_ck.X/gsp/gsp_commands.c` (`HandleGetInfo`), `gsp/gsp_commands.h`
**Bug:** No way to verify firmware identity from telemetry — "is this the hex I just built?" had no answer.
**Fix:** djb2 hash of `__DATE__ " " __TIME__` XOR'd with key tunables (advance, blank%, kp, ki, minPeriod). Bumped `GSP_PROTOCOL_VERSION 2 → 3`. Tool side: `tools/pot_capture.py` prints `Build: 0xXXXXXXXX` at startup.

### Fix 2: Wired runtime blanking parameter

**Commit:** `0248478`
**File:** `garuda_6step_ck.X/motor/sector_pi.c:693-703`
**Bug:** `v4Params.blankingPct` was a runtime-tunable parameter, but the V4 commutate ISR ignored it and used hardcoded `sectorHR >> 2` (25%). `SET_PARAM 0xF3` had no effect.
**Fix:**
```c
uint8_t  blankPct = v4Params.blankingPct;
uint16_t blankHR;
if (blankPct == 0U || blankPct > 100U) {
    blankHR = sectorHR >> 2;       /* safe fallback */
} else {
    blankHR = (uint16_t)(((uint32_t)sectorHR * blankPct) / 100U);
}
```
**File:** `garuda_6step_ck.X/motor/v4_params.c:44` — default changed `40 → 25` to match old hardcoded behavior.

### Fix 3: Added TAL=4 (30°) advance band above 120k eRPM

**Commit:** `0248478`
**File:** `garuda_6step_ck.X/motor/sector_pi.c:830-836`
**Bug:** Scheduler advance ramp topped out at TAL=3 (22.5°) — by 178k eRPM the loop was effectively at the delayHR=2 floor and had no way to add more advance.
**Fix:**
```c
uint16_t tal;
if      (schedPeriod >= 260U) tal = 2U;       /* ≤60k eRPM:   15° */
else if (schedPeriod >= 130U) tal = 3U;       /* 60-120k:   22.5° */
else                          tal = 4U;       /* >120k:       30° */
```
**Result:** 178k → 195k peak.

### Fix 4: PWM 40 → 60 kHz

**Commit:** `0248478`
**File:** `garuda_6step_ck.X/garuda_config.h:26`
**Bug/observation:** At 40 kHz with sector ≈ 80 HR at 195k, only ~2 PWM cycles per sector → undersampled BEMF + larger current ripple coupling onto float phase.
**Fix:** `PWMFREQUENCY_HZ 40000U → 60000U`. **Result:** 195k → 213k peak.

### Fix 5: MIN_OFF clamp 200 → 100 ticks

**Commit:** `b3e68e7`
**File:** `garuda_6step_ck.X/hal/hal_pwm.c:234`
**Bug:** `HAL_PWM_SetDutyCyclePeriod` clamped to `per - 200U` (1 µs MIN_OFF), assuming bootstrap topology. ATA6847 datasheet confirms it has *charge pumps* with native 100% duty support — the bootstrap argument is invalid.
**Fix:** `per - 200U → per - 100U`. Effective duty cap raised 93.7% → 96.9% at 60 kHz. **Result:** 220k → 228k peak.

### Fix 6: Idle floor V4_MIN_AMPLITUDE 4000 → 5000

**Commit:** `b3e68e7`
**File:** `garuda_6step_ck.X/motor/sector_pi.c:1001`
**Bug:** At 60 kHz the shorter 16.7 µs cycle made FET switching overhead a larger fraction of the ON pulse. The 12.2% Q15 amplitude that worked at 40 kHz now delivered only ~10% effective voltage → motor desync'd at pot-zero.
**Fix:** `4000 → 5000` (12.2% → 15.3% Q15). Empirical desync threshold was at 12% commanded; 15.3% gives margin.

### Fix 7: Telemetry duty% read the wrong value

**Commit:** `9df971b`
**File:** `garuda_6step_ck.X/gsp/gsp_commands.c:746-757`
**Bug:** Displayed duty% read `t.actualAmplitude * 100 / 32768` — the *commanded* Q15 value, NOT what the hardware actually delivered. Showed "99%" even when the per-100 clamp pinned PG[123]DC at ~96%. Confusing — looked like there was duty headroom that didn't exist.
**Fix:**
```c
extern volatile uint16_t g_pwmActualDuty;
extern volatile uint16_t g_pwmPer;
uint8_t dutyPct = 0;
if (g_pwmPer > 0U && t.actualAmplitude > 0U)
    dutyPct = (uint8_t)((uint32_t)g_pwmActualDuty * 100UL / g_pwmPer);
d[6] = dutyPct;
```
Plus added `g_pwmActualDuty` global in `hal_pwm.c:32, 211, 244` (set after the clamp in both `HAL_PWM_SetDutyCycle` and `HAL_PWM_SetDutyCyclePeriod`) and declared in `hal_pwm.h:22`.
**Effect:** Telemetry now plateaus at 96% at full pot, matching the actual gate signal and matching the speed plateau.

### Fix 8: CCP2 ISR disabled (FIFO drain only — dead code)

**Commit:** `9df971b`
**File:** `garuda_6step_ck.X/hal/hal_capture.c:102`
**Bug:** CCP2 capture ISR was enabled (`_CCP2IE = 1`) and fired on every comparator edge, but its capture-write path is gated to `FEATURE_V4_MIDPOINT_ZC == 0` (we run Mode 1). The ISR only drained the FIFO — pure overhead.
**Fix:** `_CCP2IE = 0`. Frees ~1 µs CPU per FIFO event. **No regression** at 60 kHz / 228k (old comment had warned of "regressed to ~100k trip" which was a 40 kHz baseline artifact).

### Fix 9: CCP5 ISR also disabled

**Commit:** `9df971b`
**File:** `garuda_6step_ck.X/hal/hal_capture.c:108`
**Same reason as CCP2.** Fix: `_CCP5IE = 0`. No regression.

---

## 4. What didn't work (and why) — preserved as warnings

These were tested in this session and rolled back. Don't re-try without re-reading the failure mode.

### "Fix" A: Unifying PI and scheduler advance to one knob

**Symptom:** Tried setting both PI's `setValue = advFp8 × T / 256` and the scheduler's `delayHR = (256-advFp8) × T / 256` from the same `phaseAdvanceDegX10` parameter, with default 15°. Motor regressed from 196k baseline to 130k wall.
**Why it failed:** The PI/scheduler advance mismatch is *load-bearing*. The PI's `setValue` model uses fixed 10°; the scheduler ramps 15°/22.5°/30°. The integrator pins at `minPeriodHr=10` and the IF-branch schedules from real `lastCaptureHR + delayHR` rather than from `timerPeriod`. Unifying broke that delicate equilibrium.
**Lesson:** Keep PI `setValue` model and scheduler delayHR independent. The mismatch is what makes the system work.

### "Fix" B: Single-read deglitch above 150k eRPM

**Symptom:** `garuda_service.c:420-425` does a 3-read majority vote on the comparator GPIO with ~400 ns total. Tried bypassing to single-read above 150k for ISR speedup. Regressed peak 195k → 178k.
**Why it failed:** The 3 reads aren't redundant — they're catching real switching/comparator chatter. Single-read accepts noise as ZC.
**Lesson:** 3-read deglitch is load-bearing at all speeds.

### "Fix" C: Predicted scheduling at very high speed

**Symptom:** When `schedPeriod < 90 HR`, tried switching the commutation target to `thisCommHR + schedPeriod - extraAdvHR` (anchored to current commutation, not last ZC). Regressed 195k → 178k.
**Why it failed:** `schedPeriod` reads `timerPeriod` from the PI integrator, which is saturated at `minPeriodHr=10` (eTP=65535 in telemetry). The "predicted" target is computed from a wrong period.
**Lesson:** Don't anchor scheduling to PI output when PI is saturated.

### "Fix" D: Kp doubled (1/4 → 1/2)

**Symptom:** Reduced KP_SHIFT 2→1 hoping for faster transient tracking. No peak gain (still 178k at the time), made acceleration twitchy and pot-sensitive.
**Lesson:** Steady-state is dominated by the integrator; Kp tuning doesn't help peak speed in this architecture.

### "Fix" E: Doubled ADC sampling rate

**Symptom:** Set `PG1TRIGB = 200` so ADC fires twice per PWM cycle (~100 kHz effective rate at 50 kHz PWM). Tested whether sampling-rate or PWM-frequency is the dominant factor for top speed. Result: **220k at 50 kHz with 2× ADC, same as 50 kHz with 1×.** 60 kHz at 1× still beats it (228k).
**Lesson:** Sampling rate is **not** the bottleneck. The 60 kHz advantage is current-ripple / float-phase BEMF cleanliness.

---

## 5. Build & flash (so the cloned repo compiles)

### CLI build (Linux, with XC16 v2.10 installed)

```bash
cd PATA6847/garuda_6step_ck.X
make -f Makefile-cli.mk MOTOR_PROFILE=1   # 0=Hurst, 1=A2212, 2=2810
```

Output: `dist/default/production/garuda_6step_ck.X.production.hex`

### MPLAB X build

```
File → Open Project → PATA6847/garuda_6step_ck.X/
Set Configuration → "default"
Build → "Clean & Build"
```

The default `MOTOR_PROFILE` value comes from `garuda_config.h:17`. To change profile at MPLAB build time, edit that line — the IDE does NOT pick up `-D` flags from `Makefile-cli.mk`.

### Flash

```
PKoB 4 (USB VID:PID 04d8:810b) → MPLAB Make and Program Device
```

Or via MDB CLI:
```bash
/media/.../MPLABX/v6.30/mplab_platform/bin/mdb.sh
> hwtool PKOBSKDEPLAT
> device dsPIC33CK64MP205
> program dist/default/production/garuda_6step_ck.X.production.hex
> run
> quit
```

### Verify flash

After flash, run `python3 PATA6847/tools/pot_capture.py /dev/ttyACM1`. The first lines print:
```
Build: 0xXXXXXXXX
```
If the hash differs from your last build, you flashed the wrong file. (See [Fix 1](#fix-1-build-hash-in-telemetry).)

### Tracked project files

`git ls-files PATA6847/garuda_6step_ck.X/nbproject/` shows what MPLAB needs:
- `configurations.xml`
- `Makefile-default.mk`
- `Makefile-genesis.properties`
- `Makefile-impl.mk`
- `Makefile-variables.mk`
- `project.xml`

Plus `Makefile-cli.mk` at project root for command-line builds. All committed.

---

## 6. Telemetry/diagnostics column reference

When watching `pot_capture.py`:

| Column | Meaning | Healthy at 228k peak |
|--------|---------|----------------------|
| Duty | Real PWM duty (post-clamp) | 96% |
| eRPM | Display motor eRPM | 226-228k |
| Vbus | DC bus voltage | ~24.7V |
| Cap% | ADC ISR capture-set rate | ~49% (rising-only, every other sector) |
| R/F | Rising/falling capture ratio | 100/0 (rising-only mode) |
| Mis% | Polls without finding ZC | 81-83% (normal) |
| eTP | eRPM-from-PI-timerPeriod | 65535 (saturated; PI integrator pinned at floor — expected, not a bug) |
| eTPm | eRPM-from-measurement | tracks eRPM (~14000 → 14754 at peak; the smoother decimates) |

**Don't be alarmed by `eTP = 65535`.** The PI integrator is intentionally saturated at `minPeriodHr=10` — the IF-branch in `OnCommutation` uses `lastCaptureHR + delayHR` directly (real ZC timestamp), not `timerPeriod`. PI is bypassed in steady state by design.

---

## 7. Quick history (commit hashes)

| Commit | Speed unlock | Fix summary |
|--------|--------------|-------------|
| `0248478` | 130k → 213k | TAL=4 band + PWM 60 kHz + buildHash + runtime blanking |
| `b3e68e7` | 213k → 228k | MIN_OFF 200→100 + idle floor 4000→5000 |
| `9df971b` | 228k (no change) | Telemetry honest + CCP2/CCP5 disable + 60 kHz confirmed |

All three commits are on `main`. Run `git show <hash>` for the full diff of each.

---

## 8. The remaining wall — where to look next if you want >230k

We're at ~93% of the theoretical no-load BEMF max (`KV × Vbus × PP = 1400 × 25 × 7 ≈ 245k eRPM`). The remaining 7% is normally lost to:

1. **FET on-resistance + wire resistance** — measured 130 mΩ total path. Could potentially halve by upgrading FETs / thicker phase wires.
2. **Switching loss** — 60 kHz on this gate driver eats real watts. Could reduce by lowering PWM, but that costs detection cleanliness.
3. **Trapezoidal commutation inefficiency** — torque ripple wastes energy at the harmonics. Field-oriented control (FOC) would close ~5% of this gap, but it's a different control architecture.
4. **Higher Vbus** — physically the simplest. 30V → ~294k theoretical, 36V → ~353k. Need to verify ATA6847 VDH headroom (32V max per datasheet).

Nothing in the firmware will reach the next milestone — physics or hardware change is required.
