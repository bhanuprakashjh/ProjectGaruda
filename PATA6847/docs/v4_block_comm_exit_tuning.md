# Tuning V4 Block-Commutation Exit on a New Motor

**Audience:** firmware engineer porting the V4 block-commutation feature to a motor not yet validated (anything other than Hurst, A2212, 2810, or HiZ1460).
**Goal:** dial the BLK→PWM exit so the motor (a) doesn't thrash near the entry threshold, (b) doesn't desync when pot is snapped to zero from full throttle, (c) doesn't hang in BLK while genuinely losing lock.
**Required reading before this guide:** `v4_block_commutation.md` (concept + state machine), `v4_architecture.md` (sector PI overview).

This file is **the playbook** — follow the sections in order. Each tier has a diagnostic, the knob, the file:line, and a pass/fail gate before moving on.

---

## 1. Bench setup (10 minutes)

You need all of this before tuning starts:

| Item | Why |
|---|---|
| Motor mechanically clamped to bench fixture | Block-comm exit failures can cascade into desync; an unsecured motor will lift off the bench at high eRPM |
| Bench supply matched to motor target voltage, **2× motor's rated current** | Regen spikes on BLK exit at top speed need supply headroom; under-rated supplies sag and confuse the test |
| Vbus tap to scope channel 1 | Watch for the regen spike at the moment of BLK exit |
| GSP telemetry running, `pot_capture.py` at 100 Hz | Captures `dutyPct`, `eRPM`, `d[37]` (block-comm flag bit 2), `actualForcedComm`, `goodZcCount` |
| Optional: PWM1H pin (RB10) to scope channel 2 | Visual confirmation when BLK engages — pin sits flat LOW during BLK, chops at 60 kHz otherwise |
| Spare bare motor of the same SKU on the bench | If a tuning attempt cooks magnets or windings, swap in a fresh one rather than running with damaged inductance/Rs |

Have `v4_block_commutation.md` open in another tab — it has the canonical entry/exit conditions you're tweaking.

---

## 2. Establish a baseline first (mandatory — do not skip)

You cannot tune BLK exit until you know the motor's PWM-only ceiling. Otherwise you're chasing a moving target.

**Step 2.1.** In `garuda_config.h`, set the new profile's `V4_BLOCK_ENTER_ERPM` to an unreachable value:

```c
#define V4_BLOCK_ENTER_ERPM_PROFILE_N  99999999UL  /* disable BLK entirely */
```

Build, flash, run a slow pot sweep 0% → 100% over 30 seconds. Capture telemetry.

**Step 2.2.** Identify two numbers from the capture:

- **Plateau eRPM**: the eRPM where `dutyPct` first reaches 95 or 96 and stays there as you continue raising the pot. This is the motor's natural PWM-only ceiling. Call this `ERPM_PLATEAU`.
- **Drop-off eRPM** during pot release: the eRPM where `dutyPct` falls back below 90 on a slow pot-down sweep. Call this `ERPM_DROPOFF`.

**Step 2.3.** Sanity-check the gap. `ERPM_PLATEAU − ERPM_DROPOFF` should be **≥ 10% of `ERPM_PLATEAU`**. If smaller, the motor has very abrupt duty-saturation behavior and BLK exit will be twitchy — plan to use the most conservative settings in this guide.

**Gate to leave this section:** baseline `ERPM_PLATEAU` and `ERPM_DROPOFF` recorded for the new motor at the target Vbus. PWM-only operation is stable to full throttle (no desync, no fault, no thrashing). Write both numbers in your bench log.

---

## 3. Pick initial entry/exit thresholds

These are per-profile macros in `garuda_config.h` around lines 824–859. Add a block for your new motor:

```c
#elif MOTOR_PROFILE == N    /* your new motor */
#define V4_BLOCK_ENTER_ERPM_PROFILE_N  <ENTER>
#define V4_BLOCK_EXIT_ERPM_PROFILE_N   <EXIT>
```

**Starting values:**

```
ENTER = ERPM_PLATEAU * 1.05   (5% above plateau)
EXIT  = ERPM_PLATEAU * 0.85   (15% below plateau)
```

Worked example: if `ERPM_PLATEAU = 120k`, set `ENTER = 126k`, `EXIT = 102k`.

**Why these ratios:** ENTER above the plateau guarantees BLK only engages when PWM is truly saturated (not just dipping near 95%). EXIT well below plateau guarantees you don't bounce back to BLK on small eRPM wobble. A 20%-of-plateau gap is the safety baseline; you can tighten later if the motor proves docile.

For reference, the four validated motors use these gaps:

| Profile | Motor | Vbus | ENTER | EXIT | Gap | Gap / Plateau |
|---|---|---|---|---|---|---|
| 0 | Hurst | 24 V | 30k | 25k | 5k | 17% |
| 1 | A2212 1400 KV | 12 V | 100k | 85k | 15k | 15% |
| 2 | 2810 1350 KV | 25 V | 150k | 130k | 20k | 13% |
| 3 | HiZ1460 | 30 V | 250k | 220k | 30k | 12% |

Higher-Vbus / higher-eRPM motors tolerate tighter gaps in absolute terms because the regen-spike energy scales with V², so the per-percent risk is lower at high voltage. Start conservative; narrow later.

---

## 4. Three failure modes — diagnose first, fix second

After flashing your initial thresholds, do a slow pot sweep up and a snap pot release. **Watch telemetry and the Vbus scope simultaneously.** You will see one of these four outcomes:

### 4a. BLK engages and exits cleanly, no thrash, Vbus spike < 1 V

You got lucky. Run the validation procedure in §9. If it passes, ship it.

### 4b. Thrashing — `d[37] bit 2` toggles rapidly near full pot

Telemetry signature: bit-2 of `d[37]` flips on/off many times per second when pot is held near full throttle. The motor may sound rough or shudder.

**Root cause**: amplitude / eRPM dips below exit threshold on momentary noise, motor exits BLK, regen kick + PI re-converge bounces it back to entry conditions within < 500 ms.

**Fix path:** §5 (widen hysteresis) and §6 (lengthen cooldown). Do §5 first.

### 4c. Desync on pot release from full throttle

Telemetry signature: at the moment `d[37]` bit 2 clears (BLK→PWM exit), `actualForcedComm` jumps by 5+ within a few ms, `goodZcCount` rate craters, motor enters fault or coast.

Scope signature: Vbus channel shows a > 1.5 V spike at the moment of exit; pin RB10 (PWM1H) shows the abrupt transition from flat-LOW to chopped-at-some-duty.

**Root cause**: H gate steps from solid-ON to PWM at ~88–89% effective duty in one ISR period. The 11% off-time engages complementary L-FET → active braking → kinetic energy dumps into Vbus → spike → BEMF comparator picks up noise during the spike → ZC pattern breaks → desync.

**Fix path:** §7 (slew rate softening) **and**/**or** §8 (graceful exit ramp, not yet built — needed for top-speed motors).

### 4d. BLK never engages — motor caps at ~96% duty, peak eRPM = `ERPM_PLATEAU`

Telemetry signature: `dutyPct` clamps at 100 once it reaches the duty ceiling, `d[37]` bit 2 stays 0 forever, peak eRPM matches baseline.

**Root causes (in order of likelihood):**
1. `ENTER` threshold set too high — motor can't reach it (verify: peak eRPM < `ENTER`). **Fix:** lower `ENTER` to ~5% above the actual peak eRPM you observed.
2. Amplitude doesn't reach 95% Q15 = 31130. This happens with small motors / low Vbus where Q15 amplitude saturates lower. **Fix:** drop the amplitude entry threshold in `sector_pi.c` (see §5).
3. eRPM-stable counter never reaches 10 ticks — motor speed too noisy. **Fix:** widen the `blockCommStableTicks` window or accept more eRPM jitter (see §5b).

Apply the relevant fix from §5, re-run §2 to re-baseline.

---

## 5. Tier 1 — Hysteresis adjustments (widen the dead band)

### 5a. eRPM hysteresis (per-profile, `garuda_config.h`)

If thrashing: **raise `ENTER`, lower `EXIT`** in 5% steps. Each step grows the gap.

```c
/* Example: profile started at ENTER=100k, EXIT=85k, thrashing observed */
#define V4_BLOCK_ENTER_ERPM_PROFILE_N  105k   /* raised 5% */
#define V4_BLOCK_EXIT_ERPM_PROFILE_N    80k   /* lowered 6% */
```

**Re-flash, re-test.** If thrashing persists after one step, do another. Stop once thrashing is gone, then move to §6.

### 5b. Amplitude hysteresis (board-global, `sector_pi.c:1003–1010`)

The Q15 amplitude thresholds live in code, not config:

```c
/* Current values */
actualAmplitude >= 31130U   /* 95% Q15 → entry */
actualAmplitude <  29490U   /* 90% Q15 → exit */
```

For thrashing: drop the exit threshold to **27852** (85% Q15) or **26214** (80% Q15). Keep entry at 31130. This widens the amplitude gap to 10–15% without touching entry behavior.

For "BLK never engages" (case 4d, root cause 2): drop **entry** to 29490 (90% Q15) and exit to 26214 (80% Q15). Understand that you're engaging BLK at lower duty where PWM still has switching headroom — the milestone benefit shrinks, but the feature works.

### 5c. eRPM-stable window (`sector_pi.c:1003`, `blockCommStableTicks`)

```c
blockCommStableTicks >= 10U   /* eRPM held above ENTER for 200 ms */
```

For "noisy speed prevents entry" (case 4d, root cause 3): drop to **5** (100 ms). Don't go below 3 — at the 50 Hz speed-window tick rate, fewer than 3 means random spikes can trigger entry.

For "thrashing": raise to **20** (400 ms) so entry requires a sustained operating point.

**Gate to leave §5:** slow full-pot sweep up shows BLK engaging cleanly once, no toggling at the top. Move to §6.

---

## 6. Tier 2 — Cooldown adjustment (`sector_pi.c`, `blockExitCooldown`)

```c
blockExitCooldown = 25;   /* 25 × 20ms = 500ms re-entry lockout */
```

The cooldown is what specifically prevents BLK↔PWM oscillation right after an exit. **Tune this second**, after hysteresis is set, because hysteresis prevents most thrashing on its own.

**Symptom: motor exits BLK on regen, almost-immediately re-enters, then exits again — repeated within ~1 second.**

Raise cooldown:
- High-voltage / large-rotor motors (≥ 30 V, ≥ 6S LiPo, large inrunners): **50 (1 s)** or **100 (2 s)**
- Medium-voltage drone motors (4S, 14–18 V): **25–50 (500 ms – 1 s)** — the validated default
- Low-voltage small motors (3S, ~12 V): **15 (300 ms)** may be acceptable

**Symptom: pot snapped to zero, then back to full — motor takes too long to spool back into BLK.**

Drop cooldown stepwise: 25 → 15 → 10. Don't go below 5 (100 ms) — needs 2–3 speed-window ticks to confirm conditions on re-entry.

**Method:** flash the candidate cooldown, run 20 full-pot pump cycles (slow ramp up, full pot for ~2 s, snap to zero, wait 1 s, repeat). Watch `d[37]` bit 2 transitions in telemetry. Count BLK engagements and exits. **Each engagement should have exactly one matching exit** with no extra bounces. If any cycle shows ≥ 2 engagements per pump, raise cooldown one step.

**Gate to leave §6:** 20 pump-cycle sequence shows exactly 20 clean engage/disengage events, no extra bounces. Move to §7 if you also saw desync (case 4c); skip to §9 otherwise.

---

## 7. Tier 3 — Lock-loss exit sensitivity (`sector_pi.c`, `corridorMissStreak`)

```c
if (corridorMissStreak > 5U) { /* force-exit on true lock-loss */ }
```

**This knob trades off two failure modes against each other.**

| If you see... | Knob direction | Try |
|---|---|---|
| Motor stays in BLK while audibly losing lock or shuddering before exit | Lower (more sensitive) | `> 3` |
| BLK exits prematurely on transient noise (bench supply ripple, prop airflow imbalance) | Raise (less sensitive) | `> 10` |
| Validated motors all use 5 | Default | `> 5` (do not change unless symptomatic) |

V4 captures rising-edge sectors only, so `corridorMissStreak` counts consecutive `CAP_SENTINEL` returns from rising sectors. At 200k eRPM that's ~10 µs per sector → `> 5` ≈ 50 µs of confirmed lock-loss → fast enough to catch real desync, slow enough to ignore single-noise events.

**Force-stall test method:** at full BLK throttle, gradually brake the motor with a finger or load tool. Watch `d[37]` bit 2 and `actualForcedComm`:
- Bit 2 should clear **before** `actualForcedComm` jumps by 10+ (i.e., BLK exits before deep desync)
- If bit 2 clears too early (still able to spin freely), the threshold is too sensitive — raise it
- If bit 2 clears too late (motor stalls hard in BLK, `actualForcedComm` jumps by 50+ before exit), the threshold is too lax — lower it

**Gate to leave §7:** force-stall test shows BLK exits cleanly under load with `actualForcedComm` increase < 10 between the start of stall and the exit.

---

## 8. Tier 4 — Soften the regen spike (Vbus-side mitigations)

Only do this section if you saw case 4c (desync on exit at top speed) and §5–§7 didn't fully fix it.

### 8a. ATA6847 high-side slew rate

`hal_ata6847.c` init, register `GDUCR3`, field `HSSRC`:

```c
/* Current validated setting */
ATA6847_REGS.GDUCR3_CFG.HSSRC = FullS_12_5;   /* 12.5% of full slew */
```

Options (slower → faster):
- `FullS_12_5` (12.5% — slowest, current default)
- `FullS_25_0` (25%)
- `FullS_50_0` (50%)
- `FullSpeed` (100% — fastest)

**Counter-intuitive note:** we're **already at the slowest setting**. The CK board's regen spike happens despite 12.5% slew. There is no way to make slew slower on this gate driver. So this knob is **only relevant if a different motor is doing fine on 12.5% but the regen spike on exit is large enough to cause desync** — in which case the answer is to keep 12.5% slew and tackle §8b or §8c.

Don't be tempted to raise slew rate to "fix" regen — faster slew makes the spike worse, not better. Slower slew = smaller dV/dt = smaller spike. We're already at the floor.

### 8b. Vbus over-voltage margin (firmware OV threshold)

The Vbus OV trip threshold is a GSP param (`VBUS_OV`, ID `0xE2`) backed by a per-profile default. If a regen spike of 1.5 V hits the OV threshold, the firmware faults and stops the motor — looks like a desync but is actually OV.

Quick check: in telemetry after a BLK exit fault, read `faultCode`. If `FAULT_VBUS_OV`, raise `VBUS_OV` to give margin:

```
target threshold = (Vbus_nominal + regen_spike_amplitude) + 1 V safety
```

Example: 25 V supply + 1.5 V regen spike + 1 V margin = 27.5 V threshold. **Do not exceed the ATA6847L hardware OV (40 V firmware-allowed, but stay 2 V below the configured `VBUS_OV_HW`).**

If lowering OV is unsafe (motor really does sit near supply OV), drop `V4_BLOCK_ENTER_ERPM` so BLK engages at a lower energy point — smaller regen spike on exit.

### 8c. Graceful exit ramp (not yet built — implement on AK port)

The proper fix for desync-on-exit at top speed is implementing the **graceful exit ramp** mentioned as known-limitation #1 in `v4_block_commutation.md`. Instead of stepping from solid-ON to PWM in one ISR period, ramp duty from 100% → target over ~50 ms.

**Effort estimate:** ~2 hours. Pseudocode in the porting plan `docs/garuda_6step_ak_port_plan.md` §7 / tier 6 of the previous tuning playbook in conversation.

**Recommended:** if you're hitting desync-on-exit despite §5–§8b being tuned, **stop tuning and implement the exit ramp**. It's a structural fix that supersedes all of §8. Tracked as a Phase 4 task in the AK port plan.

---

## 9. Validation procedure (declares success)

Pass criteria for "BLK exit is stable on this motor":

1. **Slow pot sweep up (0 → 100% over 30 s):**
   - BLK engages exactly once
   - `d[37]` bit 2 transitions 0 → 1 once, no toggles after
   - Peak eRPM ≥ `ERPM_PLATEAU × 1.03` (you actually got the BLK speed bonus)

2. **Hold full pot for 10 s:**
   - `d[37]` bit 2 stays 1 throughout
   - `actualForcedComm` does not increase
   - Vbus stays within ± 0.5 V of nominal supply

3. **Snap pot to zero (instant release):**
   - `d[37]` bit 2 clears within 50 ms
   - Vbus spike < 1.5 V peak on scope
   - `actualForcedComm` increase < 3 within 200 ms of exit
   - Motor decelerates smoothly to zero, no fault, no coast-with-spinup-retry

4. **20-cycle pump test** (per §6 method):
   - Exactly 20 BLK engagements and 20 exits
   - No spurious mid-pump toggles
   - No faults

5. **Force-stall under BLK** (per §7 method):
   - BLK exits before deep desync (`actualForcedComm` increase < 10)
   - Motor enters fault or coast cleanly after exit
   - No PI runaway or phantom-eRPM telemetry

**If all five pass: BLK exit is stable for this motor. Record the final per-profile values in `garuda_config.h`, the amplitude/cooldown/miss-streak values in `sector_pi.c`, and any ATA register changes in `hal_ata6847.c`. Add a row to the validated-motors table at the top of `v4_block_commutation.md`.**

---

## 10. Knob reference card

Pin this to your bench wall.

| Knob | File:Line | Default (validated) | Tuning direction for thrash | for desync-on-exit | for fail-to-engage |
|---|---|---|---|---|---|
| `V4_BLOCK_ENTER_ERPM` | `garuda_config.h:824–859` | per profile | raise | — | lower (closer to plateau) |
| `V4_BLOCK_EXIT_ERPM` | `garuda_config.h:824–859` | per profile | lower | — | — |
| Amplitude entry Q15 | `sector_pi.c:1003` | 31130 (95%) | — | — | lower to 29490 (90%) |
| Amplitude exit Q15 | `sector_pi.c:1010` | 29490 (90%) | lower (27852 = 85%) | — | — |
| `blockCommStableTicks` | `sector_pi.c:1003` | ≥10 (200 ms) | raise (20 = 400 ms) | — | lower (5 = 100 ms) |
| `blockExitCooldown` | `sector_pi.c:166` | 25 (500 ms) | raise (50 = 1 s) | raise | lower (15 = 300 ms) |
| `corridorMissStreak` limit | `sector_pi.c:165` | > 5 | — | — | raise (>10) |
| ATA6847 `HSSRC` | `hal_ata6847.c` init | `FullS_12_5` | — | already floor | — |
| `VBUS_OV` GSP param | per profile defaults | profile-specific | — | raise (if margin allows) | — |
| Graceful exit ramp | not yet built | — | — | implement | — |

---

## 11. Common mistakes

- **Changing multiple knobs in one flash.** Always change one at a time, test, decide, then change the next. Otherwise you can't tell which knob fixed what.
- **Tuning amplitude thresholds before eRPM thresholds.** Set the per-profile eRPM first — that's the dominant safety net. Amplitude tightening comes after.
- **Lowering cooldown to "make BLK feel snappier."** The cooldown protects against regen-induced re-entry. Tightening it past 15 ticks (300 ms) almost always introduces edge-case desync. Live with 500 ms.
- **Believing `dutyPct` in telemetry during BLK.** It always reads 100 in BLK because the override bypasses PG[123]DC. Use `d[37]` bit 2 to determine BLK state, not duty.
- **Forgetting to re-baseline after firmware changes that affect PWM behavior.** If you touch `V4_MIN_OFF_HR`, `LOOPTIME_TCY`, deadtime, or anything in `hal_pwm.c`, re-run §2 — `ERPM_PLATEAU` may shift.
- **Tuning at the wrong Vbus.** BLK regen spike scales with Vbus². Tune at the production Vbus, not a bench-convenient lower voltage. A motor that passes §9 at 12 V can desync on exit at 18 V.

---

## 12. When to escalate (don't ship, fix the architecture)

If after 4+ hours of tuning on a motor you still can't pass §9, the issue is structural, not tunable. Stop tuning and either:

- **Implement the graceful exit ramp** (§8c). 2 hours of code, supersedes most §5–§8 tuning for top-speed motors.
- **Disable BLK for this motor** (`V4_BLOCK_ENTER_ERPM = 99999999`). The motor will cap at `ERPM_PLATEAU`. Acceptable for motors where the extra 4% is not worth the desync risk.
- **Lower the BLK operating window** — set `V4_BLOCK_ENTER_ERPM` well below the motor's true peak so BLK runs at lower kinetic energy. Smaller regen spike, easier exit.
- **Tighten `V4_MIN_OFF_HR`** to raise PWM's own duty ceiling toward 98–99%, narrowing the gap BLK was filling. Reduces the benefit but eliminates the BLK→PWM transition entirely.

Document which path you took and why in the validated-motors row.

---

## Related documents

- `v4_block_commutation.md` — concept, state machine, history of the two fixes
- `v4_architecture.md` — sector PI overview
- `v4_motor_tuning_guide.md` — symptom-to-knob lookup for the broader V4 control loop (not just BLK)
- `v4_228k_milestone_session.md` — the pre-BLK 228k PWM-only baseline that defines what we're improving over
- `garuda_6step_ak_port_plan.md` §Phase 4 — where the graceful exit ramp is scheduled to be implemented
