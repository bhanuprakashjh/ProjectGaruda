# Overnight Review C — Simplified vs Flagship: is the complexity the disease?

**Date:** 2026-07-04 (overnight)
**Scope:** READ-ONLY comparative review of the two sibling sensorless 6-step ESC trees for dsPIC33AK512MC510.
**Question:** the flagship fights a reproducible PLL alias wall — tracked speed snaps to 1.5x at constant duty in the ~40–65k eRPM band during throttle-up, and a 2x alias at 5%-duty idle. Would this disease exist at all in the Simplified architecture, and what is the smallest port in either direction?

Trees reviewed:
- **FLAGSHIP** `/media/bhanu1234/Development/ProjectGaruda-ak512/dspic33AKESC/` — sector-PI PLL (`motor/hwzc.c`, 1361 lines), HW-comparator ZC, 13 distinct gates/watchdogs (`garuda_config.h`, 137 KB).
- **SIMPLIFIED** `/media/bhanu1234/Development/ProjectGaruda-ak512/dspic33AKESC-Simplified/` — deliberate clean rebuild (`src/control.c` 1395 lines, `src/adc.c` 366 lines, `src/config.h`, `src/motors.h`), bench-confirmed closed-loop at 16 V. Two engines behind `ZC_METHOD`: **ZC_PERIOD_TRACK (default)** and an interval-anchored ZC_PLL.

---

## TL;DR

The flagship's sector-PI owns the commutation period through a **free-running integral of phase error** (`integratorF += Ki*delta`, hwzc.c:952) and commutates from an **autonomous timer** that fires whether or not a crossing was heard (hwzc.c:461-474). Frequency — the actual ZC-to-ZC interval — is never measured in PI mode; only in-sector *position* is. A phase detector modulo the sector is inherently ambiguous between harmonics: a 1.5x lock produces near-zero deltas on the captures it does hear and blanks/rejects the rest, so it is a **stable fixed point** that every self-referenced gate then defends. The Simplified period-tracker cannot express this failure: its period IS the measured interval (IIR, 25% weight), its commutation grid is **re-anchored to every accepted crossing**, and — decisively — it **accepts late crossings and divides them back** (`interval/steps`) instead of rejecting them (`cap > T` → discard) the way the flagship does. Late-rejection makes a too-fast hypothesis self-defending; late-acceptance makes it self-correcting.

The Simplified tree already ran this exact experiment on itself: its ZC_PLL engine hit "the ~150k current wall (inst 192k vs interval-true 150k)" — the same PI-owned-period drift — and was fixed on 2026-07-01 by **interval-anchoring** (`s_integratorF = (float)T` each tick, PI demoted to a bounded ±~3% phase trim, control.c:452-511). That fix is the port the flagship needs.

---

## 1. Simplified tree: ZC detection + tracking architecture

### 1.1 Sampling front-end (src/adc.c)

- Each floating phase has a **dedicated high-speed ADC channel** with a built-in digital comparator: VA=AD1CH1, VB=AD1CH2, VC=AD2CH2 (adc.c:11-13).
- The active floating phase is triggered by **SCCP3 free-running at 1 MHz** (`CCP3PR=100` @100 MHz Fcy, adc.c:113-122). The comparator is **level-evaluated on every 1 µs sample** (CMPMOD 0b011 = sample>CMPLO rising, 0b100 = sample≤CMPLO falling, adc.c:217).
- **No PWM trigger anywhere in the ZC path.** The two driven phases are either gated off or sampled at freewheel-center for the measured-neutral diagnostics (adc.c:202-210).
- The comparator ISR clears the flag but **leaves IE enabled** so it re-fires every 1 µs while the signal stays crossed — that continuous refiring is what feeds the persistence verify (adc.c:347-365).
- Timebases: SCCP2 free-runs at 100 MHz as the timestamp clock (10 ns); SCCP1 is a one-shot that fires the commutation instant. Timer1 at 45 kHz runs only the state machine, never commutation (control.c:29-35).

### 1.2 Threshold (control.c:311-384, `zcThreshFor`)

Default config (`ZC_DIFFERENTIAL=0`, `NEUTRAL_METHOD=NEUTRAL_DUTY_MODEL`, config.h:296,345):
- Freewheel window: neutral = `Vbus*duty/(2*PWM_PERIOD)` (duty model).
- PWM-ON window (rising only, above the 75% duty knee): neutral = `Vbus/2`. Falling is always freewheel-referenced (control.c:347-361).
- RC filter-lag compensation shifts the threshold so the comparator fires at the TRUE crossing: measured-amplitude form `off = amp * ωτ * pct` with the ωτ period fed from a **slow decoupled IIR** (`s_periodFiltHR`, shift from config) precisely to break the shrink→bigger-offset→earlier-fire→more-shrink positive feedback (control.c:246-277). Deadband ±4 counts (config.h:136).

### 1.3 The accept pipeline (`Control_OnZc`, control.c:1075-1319) — shared by BOTH engines

A comparator fire must pass, in order:

1. **Window gate** (control.c:1083-1098): accept only in the window the current threshold is valid for (freewheel vs PWM-ON per `s_onWindow`). *Rejection is not starvation*: the comparator keeps firing every 1 µs, so a crossing rejected mid-ON is re-heard within microseconds of the next freewheel window.
2. **Demag blank by timestamp** (control.c:1102-1106): `now < s_blankUntilHR` → ignore. The comparator was armed **at the commutation instant** (control.c:546-593) — the blank is a timestamp comparison, not an arming delay, so there is zero arming latency after the blank ends. Blank = max(28% of period, current-adaptive floor with two separately-bounded terms: load term ≤ period/4, RC-settle term ≤ period/2, two-tier at the top band) (control.c:185-239, config.h:96-119).
3. **Persistence verify** (control.c:1108-1125): a real crossing persists across consecutive 1 MHz samples; a switching phantom is a single blip. Requires `ZC_VERIFY_N=2` fires ≤5 µs apart (`ZC_VERIFY_GAP_HR=500`, config.h:135,653). This gate is **absolute** (signal-shape-referenced), not period-referenced.
4. **Phantom/interval reject** (control.c:1217-1238): `interval < 55% * period * steps` → reject and **re-arm** (`ZC_MIN_PCT=55`, config.h:101). `steps` (commutations since the last accept) is **capped at 2** — the file's own comment records why: letting the dead-reckoned step count scale the reject bar diverged ("never scale an accept gate by the system's own unvalidated hypothesis", control.c:1226-1229). **There is no maximum-interval reject** — a late crossing is always accepted.
5. **One accept per sector** (control.c:1314-1318): disarm after accept; `commutate()` re-arms cleanly at the next step.

### 1.4 Period update law — ZC_PERIOD_TRACK (default; control.c:1286-1311)

```c
steps   = s_stepsSinceZc ? s_stepsSinceZc : 1;      /* commutations since last accept */
perStep = interval / steps;                          /* divide multi-step gap back    */
p = s_stepPeriodHR - (s_stepPeriodHR >> ZC_IIR_SHIFT) + (perStep >> ZC_IIR_SHIFT);
/* ZC_IIR_SHIFT=2 -> new = 3/4 old + 1/4 measured; clamp [STEP_PERIOD_MIN_HR, MAX] */
s_stepPeriodHR = p;
HAL_HR_ScheduleComm(commDelayHR());                  /* RE-ANCHOR: comm = ZC + T*(30-adv)/60 */
```

Three properties matter:

- **The period is a direct frequency measurement.** Every update is an IIR toward a measured ZC-to-ZC interval. One bad sample moves the estimate ≤25% toward the lie; the next true interval pulls it straight back. There is no integrator, hence no momentum and no wind-up during acceleration.
- **The commutation grid is slaved to the rotor.** Every accepted ZC reschedules SCCP1 to `ZC + (30-adv)/60·T` (control.c:406-411, 1310). The blind full-period fallback in `Control_CommFire` (control.c:1058-1072) only free-runs across genuinely silent sectors, and the next accept immediately re-phases the grid. An alias grid cannot persist because it is re-phased by every true crossing it hears.
- **Missed polarities self-heal.** A silent sector makes the next interval span 2 steps; `interval/steps` divides it back to one step, so the frequency estimate is unbiased by miss patterns (this is exactly what keeps rising-only topologies honest).

### 1.5 Period update law — ZC_PLL variant (control.c:413-543, 1249-1284) and its history

The Simplified PLL is a port of the flagship's sector-PI — and it **reproduced the flagship disease**, then fixed it:

> "Root cause of the ~150k wall (bench): the old free-integrating position PI (`s_integratorF += Ki*delta`) wound the period ~28% too SHORT at high duty — telemetry showed instantaneous eRPM ~192k while the interval-true avg was ~150k — so the bridge over-commutated into current with no speed." (control.c:457-463)

The 2026-07-01 fix ("INTERVAL-ANCHORED"):
- `s_stepPeriodHR` is IIR'd from the **measured interval** in `Control_OnZc`, identically to the period-tracker (control.c:1250-1267).
- `pllTick()` pins `s_integratorF = (float)T` **every tick** and applies only `Kp*delta` as a bounded phase trim, |trim| ≤ Kp·T/8 ≈ T/32 ≈ 3% (control.c:504-511).
- Each real ZC also re-anchors the commutation via `commDelayHR()` — the PI no longer trusts a free-running period for a full blind step (control.c:1249-1256, 398-405).
- Defenses on top: asymmetric RPM-tiered delta clamps (±T/8 low; +T/16/−T/32 above 150k, config.h:440-443), CL-entry handoff damp, defensive grow on 6-miss silence streaks, and the duty-tracking **ABS_FLOOR** no-load ceiling (`P_ff = 181380·λ·PWM_PERIOD/(Vbus_mV·duty)`, overspeed tier 130/150%, control.c:434-449, config.h:467-481).

### 1.6 Protection layer (hypothesis-independent; control.c:692-838)

The file states the design law learned from the phantom-lock campaigns:

> "Every accept gate in the ZC pipeline is referenced to the system's OWN period estimate, so a false lock defends itself… Current is the one witness a false lock cannot fool. And a COASTING bridge is the one window phantoms cannot exist in." (control.c:693-701)

- **OC latch** `FC_OC`: Ibus EMA > 12 A sustained → latched fault (config.h:615-616).
- **Coast-resync**: 8 A distress in RUN → bridge hi-Z, listen for true crossings against the measured virtual neutral, validate the coast measurement against no-load physics (>115% of rated = lie → `FC_VALIDATE`), re-seed everything from truth, resume at torque-neutral duty under a back-off ceiling (control.c:721-838).
- Governor (when SPEED_LOOP=1): advance-referenced zc% health band, consecutive-low = variance detector (control.c:1154-1188).

### 1.7 What keeps the Simplified tracker from alias-locking (and honest limits)

**Implicit robustness the flagship PI lacks:**
1. Frequency is measured, not inferred from phase — the harmonic ambiguity of a phase detector never enters.
2. Late crossings are **accepted and divided back**, never rejected — a too-fast hypothesis is corrected by the very evidence the flagship throws away (`cap > T` reject, hwzc.c:904-909).
3. The commutation grid is re-anchored per accept — a wrong-frequency grid cannot geometrically self-sustain.
4. No arming latency (armed at commutation, blank by timestamp) and no PWM-trigger dependency (1 MHz free-run) — no whole-sector capture starvation from window phase.
5. Bounded influence: IIR 25% per sample + min/max period clamps; a bad ZC "mistimes ONE step — it cannot ratchet into runaway" (control.c:5-8).

**Honest limits — the Simplified tree is NOT unconditionally alias-proof:**
- Its blank, ZC_MIN_PCT, and window gates ARE self-referenced to its own period; its own history includes the 45k self-lock, the 9:8 ratio-lock and the 196k desync slam (comments at control.c:693-701), all driven by **period-spaced demag phantoms** — a different disease (poisoned measurements) than the flagship's (unmeasured frequency).
- In a contrived geometry, `interval/steps` can confirm a wrong period (e.g. accepts at 0.5T and 3.5T of a 2/3-true grid give perStep = T_cmd). But because each accept re-phases the grid onto the true rotor and the min-gate steps cap forbids the reject bar from outrunning true ZCs, such a state has no restoring force holding it together — unlike the flagship PI, where the phase loop actively regulates INTO the alias.
- Its measured ceilings are elsewhere: the demag-blank/settle-phantom war ~150k (control.c:196-233) and the falling-sector mute topology above the duty knee (control.c:577-590).

---

## 2. End-to-end capture-path contrast

### 2.1 Sampling and threshold

| Aspect | FLAGSHIP (sector-PI mode) | SIMPLIFIED (both engines) |
|---|---|---|
| BEMF sampling | ADC digital comparator on the floating phase, SCCP3-triggered at 1 MHz — but **IE enabled only after the blanking one-shot expires** (hwzc.c:434-455) and **disabled after one accept** (hwzc.c:739) | Same silicon comparator, SCCP3 1 MHz free-run — **armed at the commutation instant**, blank enforced by timestamp, IE stays on through rejects (adc.c:194-227, control.c:546-593, 1102-1106) |
| ZC timestamp | SCCP2 100 MHz (10 ns) | SCCP2 100 MHz (10 ns) — identical |
| Threshold | duty-model neutral + filter-comp + deadband 4 (2 for high-KV profiles) (hwzc.c:346-372, garuda_config.h:977-983) | duty-model neutral + measured-amp filter-comp + deadband 4 (control.c:311-384) — same family |
| ωτ decoupling | `stepPeriodForFilterComp` slow IIR shift 11 (hwzc.c:1099-1115) | `s_periodFiltHR` slow IIR (control.c:253-262) — same idea, both trees converged |
| Commutation source | **Autonomous**: SCCP1 fires at `lastComm + timerPeriod` regardless of capture (hwzc.c:461-474); captures only timestamp `lastCaptureHR` (hwzc.c:722-740) | **Reactive**: every accepted ZC reschedules SCCP1 to `ZC + (30-adv)/60·T`; full-T autonomous fire is only a silent-sector fallback (control.c:1058-1072, 1310) |

### 2.2 Validation gates

| # | Gate | FLAGSHIP | SIMPLIFIED | Self-referenced? | Can starve a Simplified-style engine? |
|---|---|---|---|---|---|
| 1 | Demag blank | 14% of period + phase-current-adaptive, cap 25% (hwzc.c:246-290, cfg:965) — comparator **not armed** until it expires | max(28%, adaptive floor), timestamp-enforced, comparator armed throughout (control.c:185-239) | Yes (both) | Blank itself: no. Flagship's arm-after-blank adds latency the Simplified path doesn't have |
| 2 | Persistence / verify | 1 verify read ~1 µs after fire, **skipped above 80k eRPM** (hwzc.c:653-719, cfg `HWZC_VERIFY_SKIP_ERPM=80000`) | 2 consecutive fires ≤5 µs, active at all speeds (`ZC_VERIFY_SKIP=0`, config.h:253) | No (absolute) | No — continuous 1 MHz refiring makes persistence free |
| 3 | PWM-window gate | GPIO read in ISR, reject OFF-time fires (`FEATURE_HWZC_PWM_GATE=1`, hwzc.c:569-603) | Window gate on accept, comparator keeps firing (control.c:1083-1098) | No (absolute) | **No** — with a level-triggered 1 MHz comparator a rejected fire is retried next µs; the flagship's gate + edge-armed comparator can eat a whole sector |
| 4 | Chop blank | Rejects captures for 2 PWM cycles after an OC-limiter chop (~44 µs; at 150k = 2/3 of a sector — the tree itself documents the starvation feedback loop, hwzc.c:605-630) | No equivalent (no SW OC chop; OC is a latch) | No | **Cannot exist** — nothing chops duty mid-sector |
| 5 | Min-interval (phantom) | `interval < 50% * stepPeriodHR` reject (`HWZC_MIN_INTERVAL_PCT=50`, hwzc.c:632-651, cfg:998) | `interval < 55% * period * min(steps,2)` (control.c:1230-1238) | Yes (both) | No — equivalent, but Simplified caps the steps multiplier (anti-divergence) |
| 6 | **Late/cross-sector** | **`capValue > T` → capture DISCARDED** (hwzc.c:904-909) | **No late reject — late intervals accepted, divided by steps** (control.c:1258-1267) | Yes (flagship) | **This is the decisive asymmetry** — see §3 |
| 7 | Falling mute | IE never enabled for falling sectors above 70k (profile 2 only; cfg:1234) | Falling muted only above the ~75% duty knee (differential mode) (control.c:577-590) | Yes | Deliberate starvation in both; Simplified's period math divides the gap back |
| 8 | Delta clamp | ±T/8; +T/16/−T/32 above 150k (hwzc.c:911-940, cfg:1523-1556) | PLL variant: identical scheme (control.c:484-501); tracker: N/A (IIR bounds influence) | Yes | N/A |
| 9 | Handoff damp | First-N-sector shrink limit (hwzc.c:922-933) | PLL variant: identical (control.c:495-500); tracker: `s_firstZc` anchor-only (control.c:1196-1215) | Yes | N/A |
| 10 | ABS_FLOOR | `P_ff` no-load ceiling, shrink-only, 130/130% tiers (hwzc.c:982-1022, cfg:1575-1595) | PLL variant: same, 130/150% tiers (control.c:434-449, 517-526); tracker: doesn't need it (nothing can wind the period down) | No (physics) | N/A |
| 11 | Defensive grow | 6 silent sectors in / 2 out, +1%/event (hwzc.c:1050-1096) — **counts only true silence; gate-rejected captures don't count** | PLL variant: identical (control.c:528-539); tracker: blind full-T fallback + interval division | Yes | The flagship's blind spot: gate starvation is invisible to it |
| 12 | Fault/recovery | Miss-limit 3 → fallback to SW ZC + latch (hwzc.c:1200-1209); mistime watchdog (115% physics, 8 strikes); phase-clip trip; spin-catch | OC latch 12 A; coast-resync at 8 A with physics-validated re-seed + back-off ceiling (control.c:702-838) | Mixed | Simplified's coast-listen is the only phantom-free measurement window in either tree |

### 2.3 Period update law (the heart of it)

| | FLAGSHIP sector-PI (hwzc.c:952-960) | SIMPLIFIED tracker (control.c:1304-1308) | SIMPLIFIED PLL post-fix (control.c:504-511) |
|---|---|---|---|
| Law | `I += Ki·delta; T = I + Kp·delta` (Ki=0.0625, Kp=0.25 float) | `T = ¾T + ¼(interval/steps)` | `I = T_interval; T = I + Kp·delta` (trim ≤ ~3%) |
| What it measures | **Phase only** (cap position in sector) | **Frequency** (ZC-to-ZC interval) | Frequency; phase as bounded trim |
| Restoring force to true frequency | None — only phase error, which re-aliases | Every accepted interval | Every accepted interval |
| Behaviour on missed captures | Non-update; T flywheels on PI value (hwzc.c:1038-1048) | Next interval spans k steps, divided back — frequency re-measured | Same as tracker |
| Accel behaviour | Deltas integrate → momentum → overshoot | IIR tracks measured truth, no momentum | Same as tracker |

---

## 3. Verdict: is the flagship's alias wall plausibly absent in the Simplified architecture?

**Yes — the specific disease (stochastic 1.5x/2x snap during throttle-up, 2x at idle) is a property of the free-integrating phase-PI plus autonomous commutation grid, and the Simplified period-tracker has neither ingredient.** Reasoning:

1. **The alias is a stable fixed point of the flagship's loop.** Put the autonomous grid at T_cmd = ⅔·T_true (a 1.5x lock). True ZCs land in-sector at a repeating position for two of every three commanded sectors; the third lands at a sector boundary where it is blanked or `cap > T`-rejected. Accepted deltas ≈ 0 → the PI is *satisfied*. The defensive-grow never triggers (the miss streak never reaches 6 — two of three sectors capture, and a rejected `cap > T` explicitly "is NOT silence", hwzc.c:869-877). The 50% min-interval gate passes 1.5T spacings trivially. ABS_FLOOR only catches it if 1.5x exceeds 130% of no-load *at the present duty* — during throttle-up duty is rising, so P_ff is falling and the floor chases downward ahead of the alias; that is exactly why the failure is **accel-triggered and stochastic**: it needs a transient where integrator momentum (accumulated shrink from a run of early-arriving captures during genuine acceleration) carries T past truth into the alias basin while the floor is out of position. At 5%-duty idle the low-duty tier (130%, was 100 until 2026-07-04) leaves a 2x-alias-sized gap for low-KV operating points.
2. **Every flagship containment is a clamp on the PI, not a measurement.** Delta clamps, handoff damp, defensive grow, ABS_FLOOR — all bound *how fast* the integrator can move or *how far* it can go; none of them re-anchors it to a measured frequency. The flagship's own config comment concedes the point: *"The harmonic-lock disease itself (gear-slip to 1.5x under capture starvation, also 2x at 5%-duty idle) is a PLL dynamics problem"* (garuda_config.h:1225-1233).
3. **The Simplified tracker removes the disease at the root, not the symptom.** Period = IIR of measured interval; commutation re-anchored to every accept; late crossings accepted-and-divided instead of rejected. A 1.5x hypothesis lasts at most until the next accepted true crossing, whose interval reads 1.5·T_cmd and pulls the period 25% toward truth *while simultaneously re-phasing the grid onto the rotor*. There is no loop trying to hold the alias together.
4. **Strongest evidence: the same disease already occurred and was cured inside the Simplified tree.** Its ported sector-PI wound the period 28% short into a current wall (inst 192k vs interval-true 150k) — the identical signature, at a different speed — and the interval-anchor fix eliminated that failure class on the bench (control.c:9-17, 452-468). Two independent implementations, same PI law → same disease; same interval law → same cure.
5. **Caveat, stated plainly:** the Simplified tree has its *own* wall family (demag/settle phantom poisoning around ~150k, falling-window topology at the duty knee). Adopting it does not buy a free ride to 243k; it swaps an unfixable-by-clamps dynamics problem for a fixable-by-blanking measurement problem.

### The smallest port (recommended direction: Simplified → flagship)

Mirror the 2026-07-01 interval-anchor into `HWZC_OnPiPeriodExpired` + `HWZC_OnZcDetected` (~30-40 lines, all in hwzc.c, every gate untouched):

1. In the PI-mode branch of `HWZC_OnZcDetected` (hwzc.c:722-740): compute `interval = zcStamp - lastZcStamp`, `perStep = interval / max(stepsSinceLastCapture,1)`, IIR it into a new `measuredPeriodHR` (¾/¼), and — the second half of the fix — **reschedule SCCP1 to `zcStamp + commDelay`** (re-anchor, exactly what the reactive path at hwzc.c:780-796 already does).
2. In `HWZC_OnPiPeriodExpired` (hwzc.c:952-960): replace `integratorF += Ki*deltaF` with `integratorF = (float)measuredPeriodHR`; keep `timerPeriod = integratorF + Kp*deltaF` with the existing clamps as the bounded phase trim.
3. Delete nothing. ABS_FLOOR, defensive grow, clamps, gates all remain as belt-and-braces.

The reverse port (flagship gates → Simplified) is NOT recommended as the primary move: the gates are treatments for the PI's disease; the tracker doesn't have the disease. The one flagship idea worth importing later is the per-sector capture diagnostics (`dbgPiCapBySector`/`dbgPiMissBySector`) for bench visibility.

Note also: the flagship's **reactive mode** (`FEATURE_HWZC_SECTOR_PI=0`) already *is* essentially the Simplified tracker — IIR `(3T+interval)/4` (hwzc.c:759) + reschedule-on-ZC — which makes a config-only A/B possible (option a below). Two differences from the Simplified tracker to keep in mind when reading that A/B: reactive mode only updates the IIR on single-step intervals (`stepsSinceLastHwZc == 1`, hwzc.c:755 — no divide-back, so muted/missed sectors freeze the estimate) and it falls back to SW ZC after only `HWZC_MISS_LIMIT=3` misses with a latch (hwzc.c:1200-1209).

---

## 4. Morning options, ranked

### Option 1 (do first) — (a) config-only "simplicity build" on the flagship
**Flip `FEATURE_HWZC_SECTOR_PI` 1 → 0 in `garuda_config.h` (line 1166), rebuild, flash.** (Per bench workflow: config-flip instruction only, user builds in MPLAB X.)
- Cheapest possible test of the "complexity is the disease" hypothesis on the exact hardware + profile that shows the 63k wall: reactive mode replaces the phase-PI with the interval IIR + ZC re-anchor.
- **Predicted outcome if the diagnosis is right:** the 1.5x snap at constant duty disappears; the 2x idle alias disappears. New failure modes to watch instead: SW-ZC fallback latching (miss-limit 3 is tight — consider also raising `HWZC_MISS_LIMIT` 3 → 8 in the same flip), and a possibly lower top end (reactive path is less tuned above ~120k).
- 10 minutes of build time; the single most information-dense experiment available.

### Option 2 — (c) bench the Simplified tree on the 2810 directly
**Edit `src/motors.h:57`: `#define MOTOR_PROFILE PROFILE_2810`** (currently `PROFILE_VEX`). The profile already exists and is self-contained (motors.h:107-126): 7pp, 1350KV, `LAMBDA_UVSRAD=583`, align 3%, ramp to 3000 eRPM @8% duty, advance 20° across 45k–85k, CL idle 4%, `SETPOINT_MAX_ERPM=270000`.
- **24 V:** nothing else to change — Vbus is measured; the rated-speed cap, ABS_FLOOR and resync validation all scale from `ADC_VbusMv()` live. OC latch 12 A / resync 8 A stand as-is for a first spin.
- Leave `ZC_METHOD=ZC_PERIOD_TRACK` (default) and `SPEED_LOOP=0` (current open-loop config) for the first run; second run flip `SPEED_LOOP` to 1 if throttle-holding is wanted.
- Watch items at 24 V: `CL_IDLE_DUTY_PCT=4` may sit at MIN_DUTY (the profile comment flags this, motors.h:124); and expect the tree's own ~150k blank-ceiling territory — irrelevant to the 63k question but don't confuse it for the alias.
- Value: independent confirmation on the *same motor* that the alias band 40–65k is clean under an interval-anchored engine. Also directly answers "adopt Simplified wholesale for the 2810?"

### Option 3 — (b) targeted port of the interval anchor into the flagship
The §3 "smallest port": interval-IIR + `integratorF = measured T` + SCCP1 re-anchor in hwzc.c. Do this **after** options 1/2 have produced evidence — it is the keeper fix (retains the flagship's gates, telemetry, WS layers and 243k tuning), but it is code on the PLL path, and the campaign law in force says PLL dynamics changes go through simulation first. Options 1 and 2 are precisely the cheap physical evidence that either licenses the port or falsifies the diagnosis; the port itself is then a bench-proven transplant (Simplified 2026-07-01) rather than new dynamics. Estimated diff: ~30-40 lines, one file.

### Explicitly not recommended for the morning
- More clamp/floor/gate tuning on the flagship PI: three campaigns of history (delta clamps, ABS_FLOOR tiers, defensive mode, handoff damp) show each clamp narrows but never removes the alias basin, because none of them adds a frequency measurement.
- Porting flagship gates into the Simplified tree: importing treatments for a disease it doesn't have.

---

## Appendix: key file/line index

| Item | Flagship | Simplified |
|---|---|---|
| PI / period law | hwzc.c:952-960 (float), 966-979 (int) | control.c:1304-1308 (tracker), 504-511 (PLL) |
| Cross-sector / late handling | hwzc.c:904-909 (reject) | control.c:1258-1267 (accept + divide) |
| Autonomous vs re-anchored comm | hwzc.c:461-474 | control.c:1310, 1058-1072 |
| Delta clamps | hwzc.c:911-940; cfg 1523-1556 | control.c:484-501; cfg 440-443 |
| ABS_FLOOR | hwzc.c:982-1022; cfg 1575-1595 | control.c:434-449; cfg 467-481 |
| Defensive mode | hwzc.c:1050-1096; cfg 1518-1521 | control.c:528-539 |
| Blank | hwzc.c:246-290; cfg 965 | control.c:185-239; cfg 96-119 |
| Verify | hwzc.c:653-719; cfg 941-963 | control.c:1108-1125; cfg 135, 653 |
| Interval gate | hwzc.c:632-651; cfg 998 (50%) | control.c:1230-1238; cfg 101 (55%, steps≤2) |
| PWM gate / chop gate | hwzc.c:569-603 / 605-630 | control.c:1083-1098 (window only) |
| Falling mute | hwzc.c:441-453; cfg 1234 (70k, profile 2) | control.c:577-590 (duty-knee) |
| 1 MHz front-end | (comparator armed post-blank) | adc.c:113-122, 189-227, 347-365 |
| Coast-resync | (recovery/spin-catch, service layer) | control.c:721-838 |
| The disease, in the flagship's own words | garuda_config.h:1225-1233 | control.c:9-17, 452-468 (same disease, cured) |
| 2810 profile for option 2 | — | motors.h:57, 107-126 |
