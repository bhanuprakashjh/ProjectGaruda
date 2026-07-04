# Overnight-A: Detection/Tracking-Core Archaeology — Proven-Fast vs HEAD

**Date:** 2026-07-04
**Repo:** `/media/bhanu1234/Development/ProjectGaruda-ak512` (branch `ak512-port`)
**Mission:** Enumerate every behavioral delta in the capture→validate→PI→commutate chain
between the historically proven sensorless 6-step ESC and current HEAD, and answer:
*did the proven code have any anti-alias mechanism the current code lost?*

## Anchors (verified against git history)

| Ref | Date | Profile | Proven result | Role |
|---|---|---|---|---|
| **`bd85e9b`** | 2026-06-17 | **6 (VEX 4000KV)** | last **pushed** state; neighbors "214k–285k real ceiling", "advance decouple" | **primary known-good** (coordinator) |
| **`93ca4a4`** | 2026-06-13 | **2 (2810)** | "filter-comp measured BEMF → ZC centered, **260k**" | apples-to-apples **profile-2** proven anchor |
| `ca8d941` | 2026-07-03 | 9 (U3) | U3 load-desync campaign baseline | context |
| **`db8cd0f`** | 2026-07-04 21:04 | **2 (2810)** | current HEAD — the failing build | target |

`93ca4a4` is an ancestor of `bd85e9b` (bd85e9b is newer). Both anchors predate the entire
2810-campaign / WS1–WS4 / U3-port layer. **bd85e9b ran 285k on profile 6; 93ca4a4 ran 260k on
profile 2.** Because HEAD builds profile 2, `93ca4a4` is the truest apples-to-apples comparison,
while `bd85e9b` proves the *architecture* (hybrid per-polarity detection + model neutral +
per-profile SW cap + PLL-start consistency gate) that ran clean at the very top.

## Files in scope — where the change actually is

| File | 93ca4a4 → HEAD | bd85e9b → HEAD | Verdict |
|---|---|---|---|
| `motor/commutation.c` | **0 lines** (identical) | 0 | not touched |
| `motor/bemf_zc.c` | **0 lines** (identical) | 0 | not touched |
| `motor/hwzc.c` | 334 lines | **318 lines (196+/122−)** | **detection core — primary** |
| `garuda_service.c` | 1128 lines | ~1087 | ISR/CL machine — additive watchdogs |
| `garuda_config.h` | 1230 lines | ~1185 | detection flags/thresholds |
| `gsp/gsp_params.c` | 736 lines | — | profile-2 tuning |

**The capture/validate/PI/commutate math lives in `hwzc.c`.** `commutation.c` and `bemf_zc.c`
are byte-identical to the proven era — commutation-table geometry and the software ZC path are
NOT the regression. The `garuda_service.c` diff is almost entirely *added watchdogs* plus deleted
startup experiments; its speed-PI consumption block is byte-identical.

## Bench symptoms this report addresses

- **[a] PLL alias / gear-slip during acceleration.** Ratios vary **1.5–2.0×**, always during
  **throttle-up (never steady cruise)**, never below ~14k real, and the tracked speed after the
  snap **always lands ABOVE the λ-formula no-load for that duty.** Observed onsets on the current
  build (mute=70k): 39.7k@22%, 46.7k@24% (46.7→65.2→75.4k), 63k@33% (×2). On the 20k-mute build:
  46–49k. Idle: 8k→15.9k (2×) and 13k→23k (1.77×) at ~5% duty. `clIdleDutyPct=7` gives multi-second
  stable real idle ~13k; mid-range 26–42k is rock stable.
- **[b]** sector S4 (phase A floating, RISING) captures nothing (GUI blue) while others capture.
- **[c]** sector S3 (C falling) timing offset walks −3° → +30° rail with speed.

---

# THE ANSWER TO THE CRITICAL QUESTION

**Yes. The proven code had TWO acceleration-time mechanisms that HEAD lost, and BOTH are exactly
in the capture-availability / capture-attribution domain the coordinator asked about. HEAD also
ADDED two new capture-rejection windows that can *induce* the capture starvation the authors
themselves blame for the gear-slip. The steady-state PI clamps and interval math are unchanged.**

The regression is **not** in the PI period math — it is in **which captures reach the PI, and from
which polarity, during acceleration near the mute edge.**

---

## LOST MECHANISM #1 — Hybrid per-polarity detection (`FEATURE_HWZC_FALLING_SW`)

**This is the largest structural departure and the top suspect.**

### Proven code (both bd85e9b and 93ca4a4): falling on a SEPARATE path, HW comparator RISING-ONLY

`hwzc.c :: HWZC_OnBlankingExpired`, bd85e9b lines 341–356 (identical at 93ca4a4):

```c
#if FEATURE_HWZC_FALLING_SW
    /* Hybrid per-polarity: arm the HW comparator ONLY for rising sectors.
     * Falling sectors are detected via the OFF-center SW compare in the ADC ISR
     * (the HW comparator is silent on falling at speed). Leaving its IE off for
     * falling avoids phantom ON-time fires competing with the SW capture. */
    if (commutationTable[pData->currentStep].zcPolarity > 0)
    {
        HAL_ADC_ClearComparatorFlag(core);
        HAL_ADC_EnableComparatorIE(core);
    }
#else
    HAL_ADC_ClearComparatorFlag(core);
    HAL_ADC_EnableComparatorIE(core);
#endif
```

At both anchors `FEATURE_HWZC_FALLING_SW = 1`. Falling ZC was detected by the **RC-filtered
OFF-center software sample** in the ADC ISR (into the same sector-PI), and the HW comparator IE
was left **off for falling** entirely — so a falling sector never competed with a phantom ON-time
comparator fire. The falling-SW path was **per-profile speed-capped**:

```
HWZC_FALLING_SW_MAX_ERPM  35000   (A2212)
HWZC_FALLING_SW_MAX_ERPM  45000   (VEX profile 6 — the 285k runner)
HWZC_FALLING_SW_MAX_ERPM  50000   (1407)
HWZC_FALLING_SW_MAX_ERPM  70000   (2810/AK512)
```

Above the cap, falling coasted and **rising-only carried the top end** (the config comment records
"the AK128 ran 234k rising-only above its ceiling"). This is the proven top-end architecture.

### HEAD: `FEATURE_HWZC_FALLING_SW` PURGED — falling on HW comparator, armed at ALL speeds, then muted

`hwzc.c :: HWZC_OnBlankingExpired`, HEAD lines ~449–460:

```c
    HAL_ADC_ClearComparatorFlag(core);
#if HWZC_FALLING_MUTE_ERPM > 0
    if (commutationTable[pData->currentStep].zcPolarity < 0
        && pData->hwzc.timerPeriod
               < (1000000000UL / (uint32_t)HWZC_FALLING_MUTE_ERPM))
    {
        /* falling muted — no capture this sector */
    }
    else
#endif
    HAL_ADC_EnableComparatorIE(core);
```

`grep -c FEATURE_HWZC_FALLING_SW hwzc.c` (HEAD) = **0**. The entire hybrid path is gone. Falling is
now the **HW comparator, armed unconditionally below the mute** — precisely the regime the proven
comment warns against ("the HW comparator is silent on falling at speed… phantom ON-time fires
competing"). `HWZC_FALLING_MUTE_ERPM` did **not exist anywhere at either anchor** (grep = 0).

### Attribution
- Purge: `9ec01c8` (2026-07-04) "Purge dead code: startup consolidation … + falsified experiments".
- Replacement mute: `0d3d164` (2026-07-04) "Rising-only top end for the 2810: mute falling
  captures above 70k eRPM".
- `83834a1` (2026-07-04): mute 70k→20k, "S3 C-falling poison drove 1.5x harmonic locks".
- `db8cd0f` HEAD (2026-07-04): reverted 20k→70k, **"20k A/B falsified on bench (harmonic snap
  moved 62-68k down to 46-49k)"**.

### Why this is the smoking gun for symptom [a]
HEAD's own config comment (`garuda_config.h:1205–1233`) states verbatim:

> "The 260k/234k tops ran RISING-ONLY above the cap; **the cap was LOST when FALLING_SW was purged**
> (falling-HW arms at all speeds) … falling captures in the 20k–70k band are net **STABILIZERS**
> against the harmonic alias (6 spokes beat 3) … **The harmonic-lock disease itself (gear-slip to
> 1.5x under capture starvation, also 2x at 5%-duty idle) is a PLL dynamics problem.**"

Three independent facts nail this:
1. The snap sits **just below the 70000 mute edge** (63k @33%) — the hysteresis boundary where
   falling captures stop feeding the PI, dropping the capture pattern from 6 spokes to 3.
2. Commit `83834a1` proves the mute boundary **causally moves the snap**: dropping the mute 70k→20k
   moved the snap 62–68k → 46–49k. The alias tracks the threshold.
3. The proven code had NO such boundary in this band — falling-SW ran continuously to the
   per-profile cap and the top was carried rising-only, so there was no mid-band transition from a
   6-spoke to a 3-spoke capture pattern for a nascent harmonic to lock onto.

The 1.5–2.0× ratios and "always lands above no-load" are the classic PLL-locking-to-a-subharmonic
signature: with half the sectors muted, a plausible-but-early capture pattern reinforces itself.

---

## LOST MECHANISM #2 — PLL-start consistency gate (`HWZC_PllStartTick`)

The proven code (both anchors) carried a blind-startup PLL tick with a **capture-to-capture slew
limit and a sector-plausibility window** that HEAD deleted wholesale (`hwzc.c` old lines 836–915,
removed by the `FEATURE_PLL_STARTUP` purge in `9ec01c8`):

```c
/* CONSISTENCY gate (twin iteration #5). A locked rotor puts the ZC at the SAME
 * sector fraction every sector; phantoms land randomly. … Gate instead on
 * sector-plausible (T/8..7T/8, outside blanking, inside sector) AND stable vs
 * the previous capture (|Δ| < T/4 …). */
uint32_t prevCap = pData->hwzc.pllPrevCap;
pData->hwzc.pllPrevCap = cap;
if (cap > (T >> 3) && cap < (T - (T >> 3))) {          /* T/8..7T/8 window */
    uint32_t d = (cap > prevCap) ? cap - prevCap : prevCap - cap;
    if (prevCap != 0u && d < (T >> 2)) {                /* |Δcap| < T/4 slew */
        if (++pData->hwzc.pllStartGood >= PLL_START_SYNC_CAPS) { ... }
```

**Caveat:** this gate ran only in the *blind-startup* phase (`pllStartActive`), and both anchors
shipped with `FEATURE_PLL_STARTUP = 0` (off by default). So it was not active in the proven runs
either — it is a **lost design pattern, not a lost active guard.** But it is the ONLY place in the
codebase that ever implemented a **capture-to-capture slew limit** (`|Δcap| < T/4`) and a
**sector-fraction plausibility window** (`T/8..7T/8`) — the two mechanisms most directly capable of
rejecting a harmonic capture. Its deletion means those patterns are now absent everywhere and would
have to be re-created to attack the alias. Documented here because the mission asks specifically
for "capture-count gates before big corrections" and "different rejection windows."

---

## ADDED HARM — two new capture-rejection windows active BELOW the mute

Both are new since the proven era and both are **ON at 63k** (below 70k). They can *cause* the
capture starvation the config comment blames for the gear-slip.

### Added #1 — Chop gate (`chopBlank`)
`hwzc.c :: HWZC_OnZcDetected`, HEAD lines ~602–632 (introduced `71b160c`, speed-gated `f745ec8`):

```c
/* Chop gate: the SW OC limiter cut duty within the last 2 PWM cycles - the
 * applied-voltage step corrupts the BEMF crossing … Reject; … a false capture
 * here is a 60-degree slip (scope run 18 @18A). */
if (pData->hwzc.chopBlank
    && pData->hwzc.timerPeriod >= (1000000000UL / (uint32_t)HWZC_FALLING_MUTE_ERPM))
{
    pData->hwzc.noiseRejectCount++; ... return;   /* capture discarded */
}
```

`garuda_service.c:~4245` sets `chopBlank = 2` in the OC-limiter branch; the field is new
(`garuda_types.h`). Did NOT exist at either anchor (flat blank only). At 63k (below 70k) the gate
is ACTIVE: if the OC limiter chops during a throttle-up (higher current), it **rejects real
captures for 2 PWM cycles per chop → capture starvation → the exact "gear-slip under capture
starvation" the config names.** This is a rejection window the proven code never had.

### Added #2 — Load-adaptive demag blank (WS1, `FEATURE_ZC_CURRENT_BLANK`)
`hwzc.c :: HWZC_BlankTicks`, HEAD lines 60–104 (introduced `c820e9a`; `FEATURE_ZC_CURRENT_BLANK=1`
at HEAD). Proven code used a flat `blankTicks = period * HWZC_BLANKING_PERCENT / 100`. HEAD grows
the blank with measured phase current above a duty floor, capped at `period/3` (profile-2
`zcDemagBlankMaxPct=40`, `zcDemagBlankPerA=8` — all NEW fields). A longer blank on a *shrinking*
sector near 63k pushes the true (especially falling) ZC toward/out of the detection window →
miss → starvation. Bench note in the profile records this term "moved the desync wall ~90k→114k"
— i.e. it materially reshapes the detection window at speed, and did not exist at the proven anchors.

---

## MECHANISMS VERIFIED UNCHANGED (rule these OUT as the regression)

All confirmed byte-identical between 93ca4a4 and HEAD (line refs old / new):

| Mechanism | Location | Value |
|---|---|---|
| Interval-check lower bound | `hwzc.c` 497–508 / 631–651 | `HWZC_MIN_INTERVAL_PCT = 50` (both). Floored at `RT_HWZC_MIN_STEP_TICKS`. |
| Per-capture delta clamp | `hwzc.c` 851–874 / 912–935 | `±T/8` low RPM; asymmetric `+T/16 / −T/32` above `HWZC_PI_CLAMP_HIGH_ERPM=150000` (both). |
| Cross-sector reject | `hwzc.c` `if (capValue > T) drop` | identical |
| IIR-freeze gate | `hwzc.c` old 604 / new 756 | `HWZC_IIR_FREEZE_ZC_COUNT = 3` (both) |
| PI gains / integrator floor | config | `KP_SHIFT=2, KI_SHIFT=4`, integ floored at `RT_HWZC_MIN_STEP_TICKS` (both) |
| ABS_FLOOR overspeed | `hwzc.c` 934 / ~944 | `HWZC_ABS_FLOOR_OVERSPEED_PCT = 130` in both; only change = low-duty tier (below) |
| `HWZC_CMP_DEADBAND` (profile 2) | config | `4` in both (reverted to 260k value by `f5c3bcd`) |
| `HWZC_BLANKING_PERCENT` | config | `14` (both) |
| Advance schedule anchor (profile 2) | `RT_TIMING_ADV_FULL_ERPM` | new symbol, but for profile 2 falls back to `RT_MAX_CLOSED_LOOP_ERPM=260000` → behaviorally identical |

### The two standing holes (present in BOTH versions — not regressions, but they let the alias in)
1. **The interval gate is a lower bound only.** It rejects captures arriving *faster* than 50% of
   the current period. A **1.5× harmonic sits at 0.67× interval — ABOVE 50% — and is ACCEPTED.**
   Only the 2× octave (0.5× interval) is near the reject edge. This is why the 1.5× snap is admitted
   in both builds; the proven build simply never *presented* the 3-spoke pattern in this band
   (falling-SW was feeding the other 3 spokes), so the harmonic had nothing to lock to.
2. **There is NO upper-interval reject and NO explicit period slew-rate limit** in either version
   beyond the per-capture `±T/8` delta clamp. A harmonic capture that survives the T/8 clamp will
   still ratchet the period toward the alias over several sectors. The proven code tolerated this
   because capture *availability* (6 spokes) kept the true fundamental dominant.

---

## CONFIG-LEVEL DELTAS ALSO IMPLICATED (secondary)

### Measured neutral turned ON (was model neutral at BOTH anchors)
- bd85e9b & 93ca4a4: `FEATURE_VIRTUAL_NEUTRAL = 0` → comparator threshold = clean `duty·Vbus/2` model.
- HEAD: measured `(VA+VB+VC)/3` neutral (introduced `31d0276`).

The anchor's own A/B comment predicted the failure: VA/VB share AD1 and convert **sequentially**
(~0.3–1 µs skew), so the measured neutral is not a simultaneous snapshot and "the skew→threshold-
error **grows at high duty** … the same high-speed regime that fails." A duty/speed-dependent
threshold offset is a direct mechanism for **symptom [c] (S3 offset walking with speed)** and can
bias a crossing enough to seed **[a]**. This is a real behavioral change from the proven runs.

### WS3 BEMF-trigger defaults — fixed sample point (`bemfTrigShiftQ = 0`)
New `TUNING_DEFAULTS` fields (`gsp_params.c`, commit `4291293`/`9608de3`):
`bemfTrigBasePct=50, bemfTrigDutyThreshPct=50, bemfTrigShiftQ=0`. Did not exist at anchors
(implicit C-zero). With `ShiftQ=0` the sample point is fixed at MPER/2 with no duty compensation;
the code comment notes it "degrades above ~50% duty as the ON pulse swallows that instant." Combined
with the newly-enabled measured neutral, a sample landing in the switching spike is a plausible
generator of **[b] (a sector capturing nothing)** and **[c] (walking offset)**. Profile-2 snaps at
22–33% duty are below the >50% swallow, so this is a contributing interaction, not the prime driver.

### ABS_FLOOR duty-tiered (relevant to the 5%-idle 2× octave)
HEAD split the overspeed ceiling: `overPct = (dutyFrac < HWZC_ABS_FLOOR_LOW_DUTYFRAC 0.12) ?
OVERSPEED_PCT_LOW : OVERSPEED_PCT` (both = 130 now; `_PCT_LOW` was 100 for non-U3 before `5130a3c`).
ABS_FLOOR is the mechanism that *should* clamp a faster-than-physics phantom at idle. It is
SHRINK-ONLY (`timerPeriod < floorP && timerPeriod < T`), so during throttle-up (T legitimately
shrinking) it can engage — but a 130% ceiling admits an eRPM up to 1.3× no-load, which does **not**
block a 1.5× or 2× reading. Memory `project-a2212-idle-lock-solved` records the low-duty ABS_FLOOR
ceiling was itself an attractor. This governs the 8k→15.9k idle octave regime and is a retuned,
not-identical, mechanism vs the proven era.

### New reactive watchdogs (do NOT prevent the alias — they only tear down a bad lock)
All new since anchors, all in `garuda_service.c`, all trip into `ESC_RECOVERY` (coast-resync):
`FEATURE_HWZC_CAPRATE_WATCHDOG` (`0848eea`), `FEATURE_HWZC_MISTIME_WATCHDOG` (`738c51c`),
`FEATURE_IPHASE_CLIP_TRIP` (`2168bfe`), `RECOVERY_SPINCATCH` (`12b842b`),
`FEATURE_PHASE_STALL_FAULT` (`31d0276`). The **mistimed-lock watchdog** is the closest thing to a
speed-vs-duty sanity check (`eRPM > (vbus·duty)·(1e9/(181.38·λ))·(1+MARGIN 15%)`, non-decel), but it
only *coasts to recovery*, it does not reject the aliased capture, and with `MARGIN=15%` a
sub-threshold alias sits under it. `desyncMaxRestarts=0` (profile 2, `4f37fbc`) + `vbusUvAdc 500→300`
(`31e7ff7`) mean the alias no longer trips UV early or restarts — it **coasts and latches**, so it
is now *more observable/persistent* than in the proven build.

---

# RANKED — deltas most likely to matter for the alias wall

| # | Delta | Old ref (proven) | New ref (HEAD db8cd0f) | Commit(s) | Verdict |
|---|---|---|---|---|---|
| **1** | **Hybrid per-polarity detection LOST → falling on HW comparator + 70k mute** | `hwzc.c:341–356` (bd85e9b/93ca4a4, `FEATURE_HWZC_FALLING_SW`); `HWZC_FALLING_SW_MAX_ERPM` per-profile (config); **no mute define existed** | `hwzc.c:~449–460`; `garuda_config.h:1234` `HWZC_FALLING_MUTE_ERPM=70000` | `9ec01c8`, `0d3d164`, `83834a1`, `db8cd0f` | **PRIME.** Snap sits just under 70k; commit `83834a1` proves the boundary moves the snap; authors' own comment names this band's falling captures as anti-harmonic stabilizers. The exact mechanism the proven code had and this code lost. |
| **2** | **Chop gate (`chopBlank`) — NEW capture-rejection window, active below 70k** | absent | `hwzc.c:~602–632`; `garuda_service.c:~4245`; field in `garuda_types.h` | `71b160c`, `f745ec8` | Adds a rejection window the proven code never had; at 63k it is ON. OC-limiter chops during accel → capture starvation → gear-slip. |
| **3** | **Load-adaptive demag blank (WS1) — NEW, shrinks detection window under load** | flat `period*HWZC_BLANKING_PERCENT/100` (`hwzc.c` old ~385) | `hwzc.c:60–104` `HWZC_BlankTicks()`, cap `period/3`; profile-2 `zcDemagBlank* ` fields NEW | `c820e9a` | Longer blank on a shrinking sector near 63k pushes the true (falling) ZC out of window → miss → starvation. |
| **4** | **Measured neutral ON (was model neutral at both anchors)** | `FEATURE_VIRTUAL_NEUTRAL=0` (config, both anchors) | measured `(VA+VB+VC)/3` | `31d0276` | Sequential-AD skew → duty/speed-dependent threshold offset; anchor comment predicts it "grows at high duty." Prime driver for **[c]** and a seed for **[a]**. |
| **5** | **PLL-start consistency gate DELETED (`|Δcap|<T/4` slew + `T/8..7T/8` window)** | `hwzc.c` old 836–915 (`FEATURE_PLL_STARTUP`, off by default) | removed entirely | `9ec01c8` | Not active in proven runs, but the ONLY capture-to-capture slew limit + sector-plausibility window ever written. Its loss = no template to attack the alias. |
| **6** | **WS3 fixed BEMF sample point (`bemfTrigShiftQ=0`)** | implicit 0 (fields absent) | `gsp_params.c` `TUNING_DEFAULTS` bemfTrig* | `4291293`/`9608de3` | Fixed MPER/2 + measured neutral; degrades >50% duty. Contributor to **[b]/[c]**; below the swallow at 22–33% so secondary. |
| **7** | **ABS_FLOOR duty-tiered low-duty ceiling (retuned)** | flat `OVERSPEED_PCT=130` (`hwzc.c` old ~934) | tiered `_PCT_LOW` below `LOW_DUTYFRAC=0.12` (`hwzc.c:~944`) | `31d0276`, `5130a3c` | Governs the 5%-idle 2× octave; 130% admits a 2× reading; memory records low-duty ceiling as an attractor. |
| — | Interval-lower-bound 50%, per-capture `±T/8` clamp, IIR-freeze, cross-sector reject, PI gains, deadband, blanking% | `hwzc.c`/config | **identical** | — | **NOT the regression.** But: 50% gate cannot reject a 1.5× (0.67× interval), and neither version has an upper-interval reject or period slew limit — standing holes the proven build survived via capture *availability*. |

## Bottom line for the coordinator's question — "what did the proven code do during ACCELERATION that bounded corrections or capture attribution, that today's code lost?"

It did **not** bound corrections with a different clamp — the clamps are identical. What it did was
keep **all six sectors feeding the PI throughout the accel band via a separate software falling-ZC
path (`FEATURE_HWZC_FALLING_SW`), with the HW comparator armed rising-only**, and carry the top
end rising-only above a per-profile SW cap. HEAD deleted that path, arms the HW comparator on
falling everywhere below a brand-new 70k mute, and added two new capture-rejection windows
(chop gate, load-adaptive demag) that fire in the same band. The net effect during throttle-up is
**capture starvation on the falling polarity right where the alias appears** — dropping the
capture pattern from 6 spokes to 3 and giving a 1.5–2.0× harmonic a stable pattern to lock onto,
with no upper-interval / slew guard to reject it. The authors' own HEAD config comment already
diagnoses this ("the cap was LOST when FALLING_SW was purged … gear-slip under capture starvation …
a PLL dynamics problem") and commit `83834a1` provides the causal proof that the mute boundary
directly moves the snap speed.

**Highest-value next experiments (archaeology-derived, non-prescriptive):** (i) restore the
`FEATURE_HWZC_FALLING_SW` hybrid path for profile 2 and A/B against the current HW+mute; (ii) with
the mute in place, temporarily disable the chop gate and WS1 load-adaptive blank below the mute to
test the capture-starvation hypothesis; (iii) A/B measured-neutral OFF (model neutral) as at the
proven anchors to isolate the [c] offset-walk; (iv) if the harmonic persists with 6-spoke capture
restored, it is a genuine PLL-dynamics alias for the Wolfram/Modelica sim — add an upper-interval
reject or a capture-to-capture slew gate (resurrect the deleted consistency-gate pattern).
