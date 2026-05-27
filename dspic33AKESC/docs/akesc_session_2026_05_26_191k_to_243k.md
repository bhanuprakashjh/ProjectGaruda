# AKESC Session 2026-05-26 — From 191k Desync to 243k Stable

**Motor under test**: Surpass Hobby C2810 1350KV, 14×4.7" load tested no-load
**Board**: dsPIC33AK128MC106 on MCLV-48V-300W dev board
**Supply**: 25V bench (lab supply)
**Starting commit**: `ff0dbe3` (AK FOC observer skeleton — 6-step path was at 191k peak with 79% wall)
**Ending commit**: `343d6be` (MAX_DUTY 99%, 243k peak)

## Executive Summary

This session moved peak eRPM from **~191k (with frequent desyncs at 79% duty)** to **242,777 eRPM (stable at 98% duty)** — a +27% gain — through five independent firmware changes, each addressing a specific physical failure mode identified from bench data.

The session followed a strict empirical loop: hypothesize → minimal firmware change → bench test → analyze CSV telemetry → iterate. Every claim in this document is backed by specific CSV log files.

### Final config delta vs starting commit
| Parameter | Old | New | Purpose |
|---|---|---|---|
| `HWZC_MIN_INTERVAL_PCT` | 75 | 50 | Accept PWM-ripple-shifted captures |
| `FEATURE_HWZC_PWM_GATE` | (didn't exist) | 1 | Reject OFF-time false captures at BEMF crossover |
| `HWZC_PI_NEG_DELTA_CLAMP_SHIFT_HIGH` | (didn't exist; symmetric T/16) | 5 (= T/32) | Asymmetric clamp blocks period-shrink drift |
| `maxClosedLoopErpm` (profile 2) | 220000 | 260000 | Old cap was masking real BEMF headroom |
| `MAX_DUTY` formula | `LOOPTIME_TCY - 2×DT` (94.6%) | `99/100 × LOOPTIME_TCY` (99%) | Allow degenerate BC at the top |

### Final config delta — paths tried but reverted
| Tried | Result | Reverted reason |
|---|---|---|
| `HWZC_FILTER_AMP_PCT = 35` | Helped 74-78% but worse at 79% wall | Wrong knob — wall wasn't filter-comp-driven |
| `HWZC_FILTER_AMP_PCT = 0` (disabled) | Same 79% wall, no help | Wall is not CMPLO position |
| `HWZC_BLANKING_PERCENT = 14` (from 8) | Marginal effect | Not the cause |
| `maxClosedLoopErpm = 250000` (first try) | Looked like startup runaway | False alarm — coasting rotor from prior fault |
| `PWMFREQUENCY_HZ = 24000` | New wall at 54% duty / 142k eRPM | Lower PWM = sector/PWM ratio degrades; wall pulled down |

---

## Session Narrative — Step by Step

### Stage 0: Starting Position
Previous session had ported the PATA6847 sector PI synchronizer (HWZC IRQ timestamps + autonomous SCCP1 timer driving commutation + PI on phase error). Filter compensation Path 1 was active at `AMP_PCT=75`. The motor was making 191-203k eRPM but consistently desyncing at 79% duty with UV faults from regen pulses sagging Vbus.

**The "79% wall" was the central problem of this session.**

### Stage 1 — `HWZC_MIN_INTERVAL_PCT 75 → 50`
**Log**: `step6_20260526_184502.csv`
**Hypothesis**: At 135k+ eRPM, the sector is only ~3 PWM cycles long. The 5.5kHz BEMF RC filter passes ~125 counts of PWM ripple to the ADC. The comparator can fire on any PWM cycle that ripples the signal across CMPLO, so the firing time can shift by up to one PWM period (~30% of T). The OLD 75% interval gate then rejects these real-but-shifted captures.

**Math**: At lock with 22° advance, setValue ≈ 87% of T. A real capture shifted earlier by one PWM cycle gives capValue ≈ 57% of T. Interval between consecutive accepts = (T - 87%) + 57% = 70% of T < 75% min → REJECTED.

**Bench result**: Motor reached **214,961 eRPM** on first attempt — but later attempts still desynced at 79%. **Inconsistent**. The change clearly improved efficiency below the wall (ibusPkNeg dropped from -3.5A to -1.5A at 78-82%) but didn't break the wall reliably.

**Lesson**: Loosening the gate was a real improvement but not the root fix.

### Stage 2 — Bench analysis: regen pulses, not detection
**Logs**: `step6_20260526_184348/184611/185104.csv`
**Investigation**: Examined 4 desync events in detail. Found:
- Phase currents (iaPkNeg) were IDENTICAL between success and desync runs at same speed (~-5.6A)
- Bus current went 2-4× more negative in desyncs vs success
- PI's `hwzcStepPeriodHR` at same eRPM was 5191 (desync) vs 5438 (success) — **PI was commutating 5% ahead of rotor in failures**

**Interpretation**: At BEMF ceiling, motor can't accelerate. If PI's timer keeps shrinking the period, it commutates ahead of rotor → over-advanced → regen pulses → Vbus sag → UV trip. Filter comp magnitude was first suspected.

### Stage 3 — `HWZC_FILTER_AMP_PCT 50 → 35` (and back)
**Logs**: `step6_20260526_190639.csv`
**Hypothesis**: Filter compensation was over-shifting CMPLO at the wall, causing the comparator to fire too early, biasing PI toward shorter periods.

**Bench result**:
- At 74-78% duty: ibusPkNeg improved to -0.61A (best ever)
- At 78-82% duty: ibusPkNeg WORSE (-4.87A vs -3.5A at AMP_PCT=50)
- PI period at 78-82% got SHORTER (5217 vs 5266) → still desynced

**Lesson**: Filter comp magnitude wasn't the dominant driver of the wall. Reverted to AMP_PCT=50.

### Stage 4 — Asymmetric PI delta clamp
**Hypothesis**: PI integrator drift at the BEMF ceiling is dominated by negative deltas ("rotor ahead" interpretations). At a true BEMF ceiling, the motor physically CANNOT accelerate, so any "rotor ahead" reading is either noise or the start of a runaway.

**Change** (`hwzc.c`): At high RPM (> 150k eRPM), introduce asymmetric per-event clamp:
- Positive delta (capValue > setValue, rotor lagging): clamp at +T/16 (unchanged)
- Negative delta (capValue < setValue, rotor leading): clamp at **-T/32** (twice as tight)

**Bench result** (`step6_20260526_191240.csv`): Bench Ibus at 200k dropped to ~2.1A (best efficiency yet) but **wall still failed at 79% with the SAME UV signature**. The clamp helped efficiency but didn't break the wall.

**Lesson**: The wall isn't about gradual PI drift. It's about **single-event slips**. Looking at the data, one 22A phase current spike preceded each UV by ~200ms.

### Stage 5 — The breakthrough: `FEATURE_HWZC_PWM_GATE`
**Log**: `step6_20260526_194510.csv`

**Critical insight**: At the BEMF crossover (Vapp ≈ Vbemf around 79% duty / 188k eRPM), the floating-phase signal swings differently in PWM ON vs OFF windows:
- **During PWM ON**: motor neutral = duty × Vbus/2 (~10V). Comparator fires when ADC crosses CMPLO at neutral + filter-comp offset → captures correspond to true rotor ZC.
- **During PWM OFF**: all driven phases at GND, motor neutral = 0V, floating phase reads pure BEMF (centered on 0V). Comparator threshold (set for ON-time neutral) is now ABOVE the OFF-time signal, so the comparator triggers IMMEDIATELY at the OFF-time signal floor → captures at arbitrary rotor angles.

PI ingesting BOTH regimes produced ambiguous rotor angles. One bad capture per ~50 sectors → **60° rotor slip → 22A phase spike → bus regen → UV**.

**Implementation**: In `HWZC_OnZcDetected`, immediately after timestamp:
```c
const COMMUTATION_STEP_T *cs = &commutationTable[pData->currentStep];
bool pwmOn;
if      (cs->phaseA == PHASE_PWM_ACTIVE) pwmOn = (PORTD & (1u << 2)) != 0; /* RD2 */
else if (cs->phaseB == PHASE_PWM_ACTIVE) pwmOn = (PORTD & (1u << 0)) != 0; /* RD0 */
else                                     pwmOn = (PORTC & (1u << 3)) != 0; /* RC3 */
if (!pwmOn) { /* reject — re-enable IE, return */ }
```

**ADC sampling stayed at 1 MHz, asynchronous to PWM.** Not coupled. Just an in-ISR filter.

**Bench result**:
- 220,022 eRPM cleanly reached at 91% duty
- 209 samples (4.9 sec) in the 78-82% danger zone — passed cleanly (previously 100% failure)
- 474 samples at 84-88%, 482 samples at 88-92% — sustained ~16 seconds at BEMF ceiling
- 649,000 total rejections over the run (11k/sec) — OFF-time captures filtered out
- Failure mode shifted from **UV (desync)** to **OC_SW** (BEMF-ceiling phase saturation at 91%)

**This was the actual fix for the 79% wall.** The user said: "eureka".

### Stage 6 — Recognized that 220k is firmware cap, not BEMF
**Log**: `step6_20260526_200150.csv`

User noticed: above 90% duty, no speed increase but current rises. Investigated and found:
- `hwzcStepPeriodHR` was locked at exactly 4545 ticks at 91%+ duty
- 4545 = `1e9 / 220000` = `HWZC_ERPM_TO_TICKS(MAX_CLOSED_LOOP_ERPM)` for profile 2
- `gsp_params.c` line 170: `.maxClosedLoopErpm = 220000`

The motor was hitting the FIRMWARE cap, not the physical BEMF ceiling. The cap had been set to "just above" theoretical BEMF (1350 × 24 × 7 = 226.8k) but was actually clamping the motor's real top speed.

### Stage 7 — Cap raise (initially looked broken)
**Log**: `step6_20260526_200818.csv`
**Change**: `gsp_params.c` profile 2: `.maxClosedLoopErpm = 250000`
**Initial bench result**: Startup looked broken — PI period collapsed to 4054 ticks within 400ms of CL handoff, motor reporting 246k eRPM at 6% duty (impossible).

**False alarm root cause**: Comparing align currents:
- Working run: align Ia(pk 14.1) — normal stall current
- Broken run: align Ia(pk 4.4) — 1/3 normal

The motor was COASTING from a prior fault when ALIGN began. The "PI runaway" was actually CL handoff with rotor at unknown state. Reverted briefly to debug, then realized this was a contamination issue, not a cap problem.

### Stage 8 — Cap raise (validated)
**Log**: `step6_20260526_202126.csv`
**Change**: `gsp_params.c` profile 2: `.maxClosedLoopErpm = 260000` (with clean power-cycle before test)

**Bench result**:
- **Peak: 232,504 eRPM** at 94% duty
- **109.5 seconds sustained at 94% duty / 229k avg**
- Bench Ibus at 94%: **-3.12A avg** (better than -5.7A at 88%!)
- Single UV transient at 127s, motor recovered cleanly on restart
- 1.27M total ZC captures, no desync UV during high-duty operation

The "BEMF ceiling at 220k" claim from earlier in the session was wrong. The motor really had ~12k of headroom that the old cap was hiding.

**Physics check**: Theoretical ceiling at 94% × 25V × 1350 × 7 = 222k. Measured: 232k. The motor exceeds the simple BEMF formula by ~10k — explanation is filter compensation gives effective extra advance (~3-5°) at high RPM.

### Stage 9 — PWM frequency experiment (FAILED)
**Log**: `step6_20260526_205445.csv`
**Hypothesis** (user-driven): Lower PWM frequency might give cleaner signals because the BEMF RC filter (5.5kHz) attenuates less ripple at higher PWM.
**Change**: `PWMFREQUENCY_HZ 45000 → 24000`

**Bench result**:
- Motor failed at 54% duty / 142k eRPM (first attempt)
- Motor failed at 76% duty / 170k eRPM (second attempt with Vbus collapse to 6V)
- **The wall moved DOWN, not up**

**Mechanism**: The wall isn't at a fixed RPM — it's at a fixed **sector/PWM-cycle ratio**:

| PWM | Wall location | Sector | Cycles/sector |
|---|---|---|---|
| 45 kHz | 188k eRPM (old wall) | 53 µs | 2.4 |
| 24 kHz | 142k eRPM | 70 µs | 1.7 |
| 24 kHz (full loss) | 170k eRPM | 59 µs | 1.4 |

At 24kHz × 232k eRPM, sector ≈ 1.0 PWM cycle — the PWM gate has too few capture windows to work with.

**Also**: BEMF filter ripple attenuation went from 12% (at 45kHz) to 23% (at 24kHz) → 2× more PWM ripple on the signal.

**Lesson**: Higher PWM is better for high-speed operation. Lower PWM trades MOSFET switching losses for detection headroom — but we're not switching-loss-limited.

### Stage 10 — `MAX_DUTY 94.6% → 99%`
**Log**: `step6_20260526_205854.csv`
**Reverted**: PWM back to 45 kHz.

**Change**: `garuda_calc_params.h`:
```c
// Old (94.6% at 45kHz × 300ns deadtime):
#define MAX_DUTY (LOOPTIME_TCY - 2*DEADTIME_COUNTS)

// New:
#define MAX_DUTY ((LOOPTIME_TCY * 99UL) / 100UL)
```

**What happens between 94% and 99%**:
- PWM peripheral handles deadtime internally
- When requested H-pulse leaves less than 2×deadtime (600ns) for L-FET to switch, the **module suppresses L entirely**
- H-FET is solid ON with brief deadtime gaps (~220ns OFF at 99%)
- L-FET stops switching — held off
- **This is effectively block commutation for the active phase**

**Bench result**:
- **Peak: 242,777 eRPM at 98% duty**
- 1085 samples (38.7s) sustained at 98% duty / 239k avg
- Bench Ibus at 98%: **-3.30A** (less PWM switching = less bus ripple)
- iaPkNeg at 98%: -6.68A (clean commutation)
- No desync, no UV across the full throttle range

**Surprising bonus**: ibusPkNeg actually IMPROVES at 98% vs 88% (-3.3A vs -7.3A). The degenerate BC eliminates PWM-coupled bus ripple on the active phase.

---

## What Each Failure Mode Looked Like

For future debugging — three distinct failure modes appeared this session:

### Mode A: 79% wall desync (FIXED by PWM gate)
**Symptoms**:
- Phase Ia spike to 22A (saturation) ONE sample
- Phase Ia returns to ~6A
- 100-200ms later: UV trip (Vbus dipped enough to satisfy 3-sample filter at 9.3V)
- `hwzcStepPeriodHR` shrinking slowly toward floor before the spike

**Root cause**: Single 60° rotor slip from PWM ON/OFF capture ambiguity at BEMF crossover.

**Diagnostic**: `iaPkPos` jumping from ~6A to 22A in one telemetry sample.

### Mode B: BEMF-ceiling regen → UV cascade (mitigated by cap raise)
**Symptoms**:
- Phase currents stay clean
- Bus current goes increasingly negative (regen pulses)
- Vbus sags as bench supply can't absorb regen
- Eventually Vbus < 9.3V for 30+ µs → UV

**Root cause**: Motor speed locked at firmware cap, applied voltage exceeds BEMF for that speed → current builds up.

**Diagnostic**: `ibusPkNeg < -5A` and `hwzcStepPeriodHR` stuck at exactly the cap floor (4545 for 220k, 3846 for 260k, etc.).

### Mode C: Catastrophic PI runaway (only observed once with prop load)
**Symptoms**:
- `hwzcStepPeriodHR` halves in one telemetry sample (e.g., 12977 → 6614 in 21ms)
- Motor speed jumps to capped maximum
- Massive current draw → bench supply collapses
- UV cascade

**Root cause**: Sustained negative-delta saturation drives PI integrator to floor over ~100ms.

**Diagnostic**: `hwzcStepPeriodHR` collapsing across consecutive samples while throttle/eRPM look stable.

---

## Key Technical Discoveries This Session

### 1. The 79% wall was capture ambiguity, not BEMF ceiling
The wall ALWAYS appeared at 78-80% duty / 186-194k eRPM regardless of:
- PWM frequency (within reason)
- Throttle ramp rate
- Bench supply state
- Filter compensation magnitude

What it actually was: **dual-regime comparator firings** at the BEMF crossover. PWM ON gives clean ZC detection; PWM OFF gives pseudo-crossings at arbitrary angles. PI got confused.

### 2. Sector/PWM-cycle ratio is the relevant detection metric, not absolute frequency
The wall happens when sector duration approaches ~2 PWM cycles. This is why:
- 45 kHz × 188k eRPM = wall (2.4 cycles/sector)
- 24 kHz × 142k eRPM = wall (1.7 cycles/sector)
- 60 kHz would push the wall further up

### 3. PWM-state gating is software-only, not architecture change
The implementation is a GPIO read + branch in the ISR. ADC sampling stays at 1 MHz. Not coupled to PWM frequency. Not a return to ATA-style PWM-synchronous sampling.

Above 50% duty, the ON-only gate works. Below 50% duty, ON window is brief and OFF gating might be needed — but bench data shows current ON-only gate works fine at startup (98% accept rate at 6% duty).

### 4. Firmware caps can hide BEMF headroom
The `maxClosedLoopErpm` value was set to "just above" theoretical BEMF ceiling, but the motor can actually exceed that ceiling by ~5% due to:
- Effective extra advance from filter compensation
- Transient operation above steady-state BEMF
- KV variation across motor units

Setting cap = 1.15× theoretical works well. Setting cap = 1× theoretical clamps the motor short.

### 5. Above 94% duty, complementary PWM degenerates to BC naturally
The PWM peripheral handles this gracefully. The H-FET stays solid ON with brief deadtime gaps; the L-FET stops switching. We get most of BC's benefits (lower switching loss, lower bus ripple) without any explicit BC mode code.

### 6. Failed startup ≠ broken firmware
The "broken startup" at the first cap raise test was rotor-state contamination from a prior fault — the rotor was coasting when ALIGN began. ALWAYS check align current peaks (should be ~13A on this motor) before drawing conclusions about firmware changes.

---

## Test Methodology That Worked

1. **One change per build.** Multi-variable tests would have been impossible to interpret.

2. **Single-axis hypothesis per test.** When a fix worked partially (interval gate), the temptation was to call it a fix. Discipline: did it explain ALL the data? If not, keep looking.

3. **Bench data over theory.** When my "BEMF ceiling at 220k" theory said the motor couldn't go faster, the user pushed back. They were right. The cap was the limit, not BEMF.

4. **Look at the raw CSV, not just the live telemetry.** Many subtle patterns (the 50%/50% accept/miss split, the hwHR drift across samples, the phase current asymmetry between success and desync) only became visible reading the full CSV with Pandas.

5. **Trust counters over hypotheses.** When `rej` stopped growing during high RPM, that meant comparator wasn't firing multiple times per sector — different from what I assumed.

6. **Power-cycle on transition.** Rotor state contamination cost us a wrong conclusion once. Clean power-cycle between major firmware changes prevents this.

---

## Bench Log Index

All logs in `/media/bhanu1234/Development/ProjectGaruda/gui/logs/`:

| File | What it shows |
|---|---|
| `step6_20260526_184348.csv` | First desync at 79% / 192k (AMP_PCT=50, no PWM gate) |
| `step6_20260526_184502.csv` | "Success" to 220k — actually a fluke due to fast ramp |
| `step6_20260526_184611.csv` | Multiple desyncs + one accidental 220k run |
| `step6_20260526_185104.csv` | 4 desyncs at 79% — same firmware as 184348 |
| `step6_20260526_190639.csv` | AMP_PCT=35 test — helped <78% but not at wall |
| `step6_20260526_191240.csv` | Prop run + no-load with asymmetric clamp; prop runaway observed |
| `step6_20260526_191448.csv` | No-load run, first attempt desync at 81%, second to 92% |
| `step6_20260526_191752.csv` | Multiple desyncs with asymmetric clamp + AMP_PCT=50 |
| `step6_20260526_194510.csv` | **PWM gate first run — 91% / 220k reached, broke the wall** |
| `step6_20260526_200150.csv` | Sustained 220k operation, fault was OC_SW not desync |
| `step6_20260526_200818.csv` | "Broken" startup — actually coasting rotor state |
| `step6_20260526_201338.csv` | Cap reverted to 220k — clean operation, confirmed cap=BEMF behavior |
| `step6_20260526_202126.csv` | **Cap raised to 260k — 232k peak, 109s sustained at 94%** |
| `step6_20260526_205445.csv` | 24 kHz PWM — wall pulled DOWN to 54% / 142k |
| `step6_20260526_205854.csv` | **45 kHz + MAX_DUTY=99% — 242k peak at 98% duty** |

## Git Commits This Session

| Hash | Subject |
|---|---|
| `3d67c44` | AK: PWM-state gate + cap raise — 232k eRPM milestone |
| `343d6be` | AK: MAX_DUTY 94.6% → 99% — 243k eRPM milestone |

## What's Still Open

1. **Throttle rollback regen → fake UV** — see Appendix B. Backing off throttle from very high RPM causes Vbus to RISE (regen brake into bench supply) up to 32V before dipping and triggering a fault labeled UV. Real fix: duty-down slew limiter and/or brake resistor.
2. **OC_SW threshold tuning** — currently 18A trip. The 22A saturation events during fast throttle bumps trip this. Could relax or implement smarter current limiting.
3. **Symmetric OFF-time gate for very low duty** — not needed at startup (98% accept rate confirmed), but might help if/when adding prop load operation where sustained low-duty matters.
4. **Period floor from measured rotor period** — the safest catch for PI runaway, deferred since PWM gate solved the actual problem.
5. **Why motor exceeds BEMF formula by ~5%** — believed to be filter comp's effective advance, but not rigorously characterized.
6. **Filter comp linear approximation** — see appendix below. Theoretically correct form is `sin(arctan(ω·τ))`, current code uses linear `ω·τ` with a magic clamp + 50% fudge factor. Replacing with the algebraic identity `ω·τ / √(1+(ω·τ)²)` would be more correct without calling actual sin/arctan. Not implementing now since we're BEMF-limited at 24V; matters at higher Vbus or different motors where `ω·τ` enters the nonlinear region of arctan.

---

## Appendix A — Filter Compensation: Linear vs True `sin(arctan(ω·τ))`

### Why the offset formula has sin and arctan in it

The PCB BEMF lines run through a passive RC low-pass filter (5.5 kHz cutoff, τ ≈ 30 µs).
At electrical frequency ω, this filter produces:

- **Phase lag**: φ = arctan(ω·τ)
- **Amplitude attenuation**: 1 / √(1 + (ω·τ)²)

The comparator fires when the filtered signal crosses CMPLO. We want it to fire at the time that corresponds to the TRUE (pre-filter) zero crossing. To compensate the phase lag φ:

- The pre-filter signal at the comparator-fire moment was at value `amp × sin(φ)` above/below zero
- So we shift CMPLO by **`amp × sin(φ)`** from neutral — this pulls the comparator firing time back to the true ZC

Combining: **`offset = amp × sin(arctan(ω·τ))`**

That's the theoretically correct form. Sin and arctan live here.

### How the current code approximates it

`hwzc.h:HWZC_ApplyFilterComp()`:
```c
uint32_t omegaTauQ15 = HWZC_FILTER_K_Q15 / stepP;
if (omegaTauQ15 > HWZC_FILTER_MAX_OMEGA_Q15)
    omegaTauQ15 = HWZC_FILTER_MAX_OMEGA_Q15;     // <-- magic clamp
uint32_t offset = ((uint32_t)amp * omegaTauQ15) >> 15;
offset = (offset * HWZC_FILTER_AMP_PCT) / 100UL; // <-- AMP_PCT=50 magic factor
```

This uses just **`amp × ω·τ × 0.50`** — the linear part of the Taylor series around ω·τ = 0, scaled by 50%.

### Why linear overshoots — and why the clamp exists

The trig identity:
- **sin(arctan(x)) = x / √(1 + x²)**  (right-triangle identity, no actual sin/arctan calls needed)

| ω·τ | True `x/√(1+x²)` | Linear `x` | Linear overshoot |
|---|---|---|---|
| 0.05 (low RPM) | 0.0499 | 0.050 | +0.2% |
| 0.1 | 0.0995 | 0.100 | +0.5% |
| 0.3 | 0.287 | 0.300 | +4.5% |
| 0.5 | 0.447 | 0.500 | +12% |
| 0.73 (= `MAX_OMEGA_Q15`) | 0.591 | 0.730 | +24% |
| 1.0 | 0.707 | 1.000 | +41% |
| 2.0 | 0.894 | 2.000 | +124% |
| ∞ | **1.0 (saturates)** | ∞ | — |

The linear form is fine for low ω·τ (low RPM) but **diverges to infinity** as RPM climbs, while the true function **asymptotes at 1.0**.

`HWZC_FILTER_MAX_OMEGA_Q15 = 24000` (= ω·τ ≈ 0.73 rad = 42°) was added as a clamp to prevent the linear value from exploding. `HWZC_FILTER_AMP_PCT = 50` was added as a fudge factor to scale the whole thing down because the linear values were too big.

Together, the clamp + fudge accidentally produce an effective curve that's close to `sin(arctan)` in OUR operating range — which is why the code works.

### The correct implementation (deferred)

Replace the linear form with:

```c
/* offset = amp × sin(arctan(ω·τ)) = amp × ω·τ / √(1 + (ω·τ)²)
 * Saturates naturally at amp × 1.0 (filter phase → 90° max).
 * No magic clamp needed. AMP_PCT can be 100 (theoretical).             */
uint32_t omegaTauQ15 = HWZC_FILTER_K_Q15 / stepP;
uint64_t omegaTau2 = ((uint64_t)omegaTauQ15 * omegaTauQ15) >> 15;        // (ω·τ)²  in Q15
uint32_t denom_sq = (1UL << 15) + (uint32_t)omegaTau2;                   // 1 + (ω·τ)²
uint32_t denom = isqrt_q15(denom_sq);                                    // √(1+(ω·τ)²)
uint32_t shift = ((uint64_t)omegaTauQ15 << 15) / denom;                  // ω·τ / √(1+(ω·τ)²)
uint32_t offset = ((uint32_t)amp * shift) >> 15;
offset = offset * HWZC_FILTER_AMP_PCT / 100UL;
```

CPU cost: one 64-bit multiply, one division, one integer sqrt — ~50-100 cycles on dsPIC33AK at 200 MHz vs ~5 cycles for the linear form. Per-event in the ISR, this is acceptable.

### Why this is NOT being implemented today

1. **We're BEMF-limited at 24V**, not detection-limited. The motor reaches 243k regardless of which approximation is used.
2. **The fudge-factor combination happens to work in our operating range** — the clamp + 50% factor accidentally lands in the right neighborhood for `sin(arctan)` between ω·τ = 0.3 and 0.73.
3. **Risk of subtle regression with no measurable upside** — getting the algebra right would require re-tuning `AMP_PCT` and removing the clamp, with potential to upset the carefully-tuned-at-24V behavior.

### When this WOULD become important

- **6S LiPo testing (22V)** — pushes ω·τ further into the nonlinear region. The linear form's overshoot grows, the fudge factor would need re-tuning.
- **Different motors** with much higher τ (longer filter time constant) — same issue at lower RPM.
- **Removing magic constants for code clarity** — the algebraically-correct form is self-contained: no MAX_OMEGA clamp, no fudge factor, just the physics.

### Alternatives to `x/√(1+x²)`

| Form | Pros | Cons |
|---|---|---|
| Current linear `x` | Cheap | Diverges, needs clamp + fudge |
| `x / √(1+x²)` (exact) | Self-saturating, correct | sqrt cost in ISR |
| `tanh(x)` table lookup | Cheap (~20 cycles) | Slightly different shape; ~10% mismatch at boundary |
| `arctan(x)` table → then sin | Most accurate decomposition | Two table lookups, more memory |
| Polynomial fit (Chebyshev or Padé) | No table memory, parameterizable | Tuning effort |

For our use case, `x/√(1+x²)` is the natural choice — it's the exact algebraic equivalent of sin(arctan) and removes the need for any magic constants.

---

## Appendix B — Throttle Rollback Regen → "UV Fault"

Two bench runs (`step6_20260526_215456.csv` and `step6_20260526_215904.csv`) tested rapid throttle modulation at 45 kHz / MAX_DUTY=99%. Both ran 250+ samples at 98% duty cleanly. Both showed the same pattern when throttle was reduced from high RPM: **Vbus RISES** (not drops) as the motor regenerates into the supply.

### The actual mechanism

When throttle suddenly drops from 98% → 56% at 240k eRPM:
1. Applied voltage drops from 24.5V → 14V instantly
2. Motor still spinning at 240k eRPM (rotor inertia, takes hundreds of ms to slow)
3. **BEMF at 240k eRPM ≈ 22-23V** (matches the no-load top speed at full duty)
4. BEMF (22V) >> Applied (14V) → motor regenerates HARD
5. Energy flows back into the Vbus capacitor → Vbus rises
6. If the bench supply can't absorb the regen current, Vbus keeps climbing

### What's happening at the supply

Most bench supplies (and many flight LiPo packs) are **sources, not sinks** for current. They can't absorb reverse current — only resist it via internal impedance. So when the motor regenerates:
- Bus capacitor charges up rapidly
- Supply input current goes to zero, then becomes briefly negative if supply tolerates it
- If supply has a reverse-current protection diode, Vbus keeps rising unbounded
- Eventually FET reverse-recovery or bridge limits clip something → current spike → fault

### Bench data from two runs

**Run 1 (`215456.csv`) — fast rollback → fault**:
```
t=32.495s  76% / 218k  Vbus=28.38V  ibusPkNeg=-11.1A
t=32.519s  70% / 209k  Vbus=29.67V  ibusPkNeg=-1.0A
t=32.544s  60% / 192k  Vbus=32.31V  ibusPkNeg=-1.0A   ← Vbus +7V above nominal
t=32.567s  56% / FAULT Vbus=20.08V  iaPkPos=22.0 (sat)  ← spike then collapse
```
Vbus rose 7V above nominal in 50ms. Then a 22A phase current saturation (likely from FET reverse-recovery or bus cap clamping) triggered the fault. Label "UV" is misleading — root cause is regen overvoltage.

**Run 2 (`215904.csv`) — slower rollback → survived**:
```
t=39.053s  77% / 201k  Vbus=26.16V  ibusPkNeg=-7.1A
t=39.218s  53% / 143k  Vbus=28.77V  ibusPkNeg=-1.0A
t=39.266s  51% / 125k  Vbus=26.12V (recovered)
```
Same direction (Vbus rising) but smaller peak (28.77V vs 32.31V). Throttle drop was spread over 165ms instead of 70ms. Bus capacitor handled it.

### Why the previous "BEMF ceiling at 220k" intuition is consistent with this

At 240k eRPM no-load, BEMF ≈ 23V — almost equal to Vbus. The motor barely needs any duty to maintain that speed. When duty drops to 56% (= 14V applied), the difference (23V - 14V = 9V) drives regen current through the bridge inductors. Standard back-EMF braking.

### Three fixes (in order of preference)

**(1) Firmware duty-down slew limiter** — best
Limit how fast duty can decrease, especially at high RPM. At 200k+ eRPM, force throttle-down to take at least 500ms even if pot changes faster. Motor decelerates naturally through wind resistance and bearing friction; BEMF drops with it; regen pulses stay manageable.

```c
/* Pseudocode in the throttle → duty mapping */
uint16_t maxDutyChange = (currentERPM > 150000)
                        ? MAX_DUTY * 5 / 1000   /* 0.5% per ms at high RPM */
                        : MAX_DUTY * 50 / 1000; /* 5% per ms at low RPM */
if (targetDuty < currentDuty - maxDutyChange)
    targetDuty = currentDuty - maxDutyChange;
```

This is what every flight ESC does. It mimics realistic flight-controller throttle behavior.

**(2) Brake resistor + comparator** — hardware fix
Add an external resistor (a few ohms, high-wattage) switched by a MOSFET across the bus, triggered by a comparator at, say, 28V. Dumps regen energy as heat. Common in industrial drives, rare in flight ESCs because of weight/cost.

**(3) Bigger bus capacitor** — hardware fix
Absorb more transient energy. Adds weight but reliable. Diminishing returns — at the energies we're seeing, would need huge caps.

### Why we're not implementing (1) right now

User testing is intentionally pushing the motor hard to validate the high-RPM detection chain. Adding a slew limiter while testing detection would mask the data. Once detection is fully validated (probably done now), slew limiter is a one-evening implementation.

### What this means for future testing

When testing throttle response or rapid maneuvers, expect Vbus to spike up to ~30V on the bench supply during decelerations from 240k+ eRPM. The "UV fault" reported in this regime is **not** a real undervoltage event — it's the bridge saturation after regen overvoltage. Don't chase it as a sync issue.

---

## Appendix C: Rapid pot-zero intermittent stall (2026-05-27)

### Symptom

Holding pot at zero idles cleanly at ~14k eRPM / 6% duty. Bumping the
pot briefly and letting it snap back works most of the time. But a
**fast, hard pot release** sometimes pushes the firmware into a state
where:

- `hwzc.totalZcCount` stops incrementing (no new real BEMF captures)
- `hwzcStepPeriodHR` freezes at the last value before the release
- The autonomous SCCP1 timer keeps commutating on that stale period
- `BEMF_ZC_CheckTimeout` doesn't see DESYNC because the timer's own
  callback keeps `lastCommTick` fresh
- Motor buzzes for several hundred ms and may or may not recover on
  its own

### What was added this session (commit `0de7d93`)

A second watchdog in the CL ADC ISR slow path that monitors
`hwzc.totalZcCount` directly. If it doesn't increment for
`HWZC_NO_CAPTURE_MS` (150 ms default) while `zcSynced=true`, the code
mirrors the desync handler:

- Disable PWM
- Re-arm `runCommandActive` under `AUTO_DISARM=0`
- `state = ESC_RECOVERY`, `recoveryCounter = RT_DESYNC_COAST_COUNTS`

This catches the runaway case where the previous fix could not.

### What still fails (known limitation, parked)

Even with the watchdog, **rapid** pot-zero events occasionally produce
a brief stall before the 150 ms threshold fires. Bench log
`step6_20260527_103134.csv` shows:

| Time | State | Notes |
|---|---|---|
| `t=20.069` | CL @ 17836 eRPM, hwT=56066 — last good capture |
| `t=20.092` | **RECOVERY** | new watchdog tripped, +35 hwMs in 23 ms |
| `t=20.282` | ALIGN | restart begins |
| `t=20.708` | back at 246k eRPM (PI-runaway again) |
| `t=21.442` | IDLE | a different exit path took over |
| `t=22.341` | ARMED | auto-rearm |
| ... cycle continues ... | | motor recovered via the standard cycle |

It is **not** a permanent stall. The motor always recovers via either
the new no-capture watchdog or the existing auto-rearm chain. The
visible behavior is "motor pauses then restarts" within a couple of
seconds — the user can immediately throttle up to recover faster.

### Why we're parking this

- Threshold tuning (150 ms) is a tradeoff against false-trip on
  legitimate low-signal transients near startup. Lowering it risks
  tripping during normal CL entry where the first few captures are
  sparse.
- The rapid-release stall is a corner case of bench testing — a
  flight ESC would never see human-speed throttle slams; it would see
  a slew-limited DShot stream.
- The real cure is the **duty-down slew limiter** described in
  Appendix B. With slew-limited duty drops, BEMF stays measurable
  during the deceleration and HWZC never goes silent in the first
  place. That single fix solves both the regen overvoltage and the
  rapid-pot stall.

### When we come back to it

The slew limiter from Appendix B is the priority. After that's in,
re-run this test. The watchdog stays as a belt-and-suspenders for the
case where the motor stalls for some other reason (e.g. mechanical
load spike).
