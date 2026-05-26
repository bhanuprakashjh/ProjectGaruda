# Debugging V4 High-Speed Ceiling on a New Motor

**Symptom this guide addresses:** your motor walls at some peak eRPM below what a reference ESC achieves on the same motor and the same bench supply. Common observation: 2–5% gap to the reference at the very top of the throttle range.

**Audience:** firmware engineer on a remote bench with the EV43F54A board, our `garuda_6step_ck.X/` build, and a motor that isn't one of the four validated profiles.

**What this guide replaces:** ad-hoc tuning that chases symptoms. Follow the diagnostic-first procedure — don't change knobs until you've captured the four telemetry channels in §3.

---

## 1. Pre-flight (10 minutes — eliminate the easy stuff)

Before touching firmware, prove these four things. Skip any of them and you'll waste days chasing the wrong cause.

| Check | How | Pass criterion |
|---|---|---|
| **Bench supply is not sagging** | Multimeter on `+VBus`/`GND` at the ESC input while running at full pot | Vbus stays within ±0.3 V of nominal at peak load |
| **Block-commutation is actually engaging** | Scope on PWM1H (RB10). Or read GSP snapshot `d[37]` bit 2 | Pin sits **statically LOW** during peak / bit 2 reads `1` |
| **Reference ESC reaches its peak on this same bench** | Swap ESC, leave motor/supply/load identical, re-test | Reference ESC reaches its claimed peak on this supply (rules out supply/wiring) |
| **Motor mechanically secure, no thermal throttle** | Touch test motor case after 30 s peak; verify mount | Motor < 60 °C, no movement of mount |

**If any of these fail, fix that first.** The rest of the guide assumes all four pass.

---

## 2. Identify the speed gap and translate to telemetry units

Convert your observation into eRPM and sector period:

```
eRPM_motor   = mech_RPM × pole_pairs
schedPeriod  = 60e6 / (eRPM × 6) / 0.64        (in HR ticks, 640 ns/tick)
              ≈ 15625000 / eRPM
```

Worked example with the user's motor (24300 mech RPM, 7 PP):

```
eRPM_motor   = 24300 × 7 = 170100 eRPM
schedPeriod  = 15625000 / 170100 ≈ 92 HR ticks
```

For the reference ESC's 25000 mech RPM target:

```
eRPM_ref     = 25000 × 7 = 175000 eRPM
schedPeriod  = 15625000 / 175000 ≈ 89 HR ticks
```

So the gap is ~3 HR ticks of sector period. **Keep these two numbers (your peak eRPM, your peak schedPeriod) in your bench notes** — every step below refers back to them.

---

## 3. Mandatory telemetry capture (the diagnostic gate)

Do **not** start tuning until you've captured these. The whole rest of the guide depends on what you see here.

### 3a. Tool setup

```bash
cd PATA6847
python3 tools/pot_capture.py --rate 100 --duration 30 \
    --output captures/highspeed_baseline.csv
```

During the capture: slow pot sweep 0 → 100% over the first 15 seconds, hold at 100% for the next 10 seconds, snap to 0 for the last 5 seconds.

### 3b. Channels to extract from the CSV

| Channel | What it tells you |
|---|---|
| `eRpm` | Confirmed peak; should match your math from §2 |
| `stepPeriodHR` | Firmware's measurement of actual sector duration from comm timestamps |
| `timerPeriod` (via GSP CMD_PI_LOG 0x30) | What the V4 PI thinks the period is; updates only on captured ZCs |
| `actualForcedComm` | Sectors that got forced-commutated because no ZC was captured. Should be ~0 at steady-state peak. Climbing = BEMF detection is failing. |
| `zcLatencyPct` | Where in the sector the ZC arrived (0–100%, ~50% is ideal). Tells you if ZCs are landing inside the blanking window. |
| `dutyPct` and `d[37]` bit 2 | Confirms BLK is engaged at peak (bit 2 = 1, dutyPct = 100). |
| `goodZcCount` (delta over 1 s) | Rate of successful ZC detections. Compare against expected 2×sectors/s = 2 × peak eRPM × 6 / 60. |
| `vbusRaw` | Vbus stability throughout the run. Should not droop by more than ~1% at peak. |

### 3c. Save the PI log

After hitting peak and holding for 5 seconds, before pot-release:

```bash
python3 tools/gsp_test.py --command pi_log --output captures/pi_log_at_peak.csv
```

This dumps the first 30 PI iterations after CL entry — useful for seeing if the PI is converging or oscillating.

---

## 4. Diagnostic matrix — match what you see to a mechanism

From the captured data, fill this table. The matching mechanism determines which section below to read next.

| Observation at peak throttle | Mechanism in play | Go to |
|---|---|---|
| `timerPeriod ≈ stepPeriodHR ≈ math-predicted value` (e.g., both ~92 for your motor); `actualForcedComm` not climbing; `zcLatencyPct` stable around 30–70% | **No diagnostic signal here.** Motor is at a physics limit OR the reference ESC uses a different commutation algorithm | §5 |
| `timerPeriod` is significantly **longer** than `stepPeriodHR` (e.g., timerPeriod=110, stepPeriodHR=92) | **PI is biased late** — captured ZCs are arriving later than the PI's model expects, dragging timerPeriod up. Latency bias. | §6 |
| `timerPeriod` and `stepPeriodHR` both oscillate (e.g., bouncing 80 ↔ 110) | **PI is not converging** — gain mismatch with this motor's BEMF characteristics | §7 |
| `actualForcedComm` climbing at peak (>5 per second sustained); `goodZcCount` rate below expected sector rate | **BEMF detection is dropping captures** — blanking window or comparator can't keep up | §8 |
| `zcLatencyPct` drifts toward 0 or 100 at peak (instead of staying near 50) | **ZCs are landing at the sector boundaries** — blanking is eating them, or timing advance is off | §8 |
| `dutyPct = 100` and bit 2 = 0 (BLK not engaging but duty is at the firmware ceiling) | **BLK never activates for this motor** — entry threshold too high | §9 |
| `dutyPct = 100`, bit 2 = 1, but `stepPeriodHR` won't drop below some value as you increase pot | **Possibly a real motor-physics limit** OR algorithmic | §5, then §6 |

If multiple rows match, work them in the order they appear in the table above. The PI itself must be healthy before downstream tuning means anything.

---

## 5. If telemetry shows no diagnostic signal (motor is at a real limit)

You've eliminated all the in-firmware suspects. The 2–5% gap may genuinely be the **commutation algorithm difference** between trapezoidal 6-step (this firmware) and whatever the reference ESC uses.

### 5a. Check what the reference ESC is doing

If you can scope the reference ESC's phase outputs at peak:
- **Trapezoidal hard-switching like us**: each phase shows ~120° on, ~60° off, hard edges. Reference probably has a tighter advance algorithm than ours.
- **Sinusoidal at top end**: phases show smooth sinusoids, not hard edges. Reference is using sine modulation above some eRPM threshold. This is an architectural difference, not a tunable parameter.
- **Trapezoidal with timing-compensated zero-crossing**: same as us in shape, but with smarter ZC interpolation. Closer to our path.

### 5b. What you can try in our firmware

If the reference is doing more aggressive advance:

1. **Add a TAL=5 band**. Open `motor/sector_pi.c` around line 952, change the speed-adaptive TAL ramp:
   ```c
   if      (schedPeriod >= 260U) tal = 2U;
   else if (schedPeriod >= 130U) tal = 3U;
   else if (schedPeriod >=  95U) tal = 4U;     // was: < 130 → 4
   else                          tal = 5U;     // new band, only useful above ~165k eRPM
   ```
   Flash, retest. If peak eRPM rises by 1–3%, advance was indeed the bottleneck.
   If peak drops, you've gone past the optimal advance angle for this motor — BEMF measurements get unreliable. Revert.

2. **Try predicted scheduling for sectors above the TAL=4 threshold.** Open `motor/sector_pi.c` around line 957. The current code is `targetHR = v4_lastCaptureHR + delayHR;`. Replace it with a thisCommHR-anchored variant when delayHR is at the floor:
   ```c
   uint16_t targetHR;
   if (delayHR <= 3U && schedPeriod < 100U) {
       /* Floor of delayHR — switch to predicted from this commutation */
       targetHR = (uint16_t)(thisCommHR + schedPeriod - advHR);
   } else {
       targetHR = (uint16_t)(v4_lastCaptureHR + delayHR);
   }
   ```
   This was tried in 2026-04-28 and reverted due to a parameter bug at the time (`v4Params.minPeriodHr` was wrong). With it at the current value of 10, the experiment is worth re-running.

3. **Sinusoidal commutation at top end.** Major architectural change; not a tuning knob. Out of scope for this guide. If 1 and 2 don't close the gap and the reference is sinusoidal, the gap may not be closeable without that work.

---

## 6. If `timerPeriod` is biased long (PI sees ZCs late)

The PI is converging on a sector period that's **longer** than reality. Mechanism: BEMF detection latency (comparator + filter + IC capture) is fixed in absolute time but grows as a fraction of the sector as speed climbs. The captured ZC arrives "late" relative to the PI's model, the PI corrects by lengthening timerPeriod, and the motor settles slower than it could.

### 6a. Reduce ATA6847 edge blanking (EGBLT)

Lower EGBLT through GSP at runtime: param `EDGE_BLANKING` (current default 15 = 3.75 µs). Try **10 (2.5 µs)** then **7 (1.75 µs)**. Re-capture each time, look for `timerPeriod` to drop closer to `stepPeriodHR`.

If `actualForcedComm` starts climbing as you lower EGBLT, you've gone below the noise floor — back off one step.

### 6b. Lower software blanking percentage

GSP param `ZC_BLANK_PCT_FAST` (per-profile, 8–15 typical). Try **5–7**. Watch the same metrics.

### 6c. Reduce `V4_RC_DELAY_HR`

Open `motor/v4_params.h`, look for `V4_RC_DELAY_HR` (the constant added to `setValue` in the PI). This is a model-side correction for RC filter delay. If it's larger than the actual hardware delay, the PI applies negative bias. Try halving it.

### 6d. Verify with PI log

After each change, dump the PI log via GSP CMD 0x30. Look at the `deltaClamped` column for steady-state captures. **It should bounce around zero**. If it's consistently positive (5–20 HR ticks), the PI is still seeing late ZCs — keep tuning. If it's consistently negative, you've overshot — back off.

---

## 7. If PI is oscillating

`timerPeriod` and `stepPeriodHR` both swinging means the PI's gain is mismatched to this motor's dynamics. At high speed, the BEMF response is faster and the PI needs different gains.

### 7a. Lower Kp

GSP param `V4_KP_SHIFT` (default 2 = Kp of 1/4). Try **3 (Kp = 1/8)**, then **4 (Kp = 1/16)**. Lower Kp = slower response, less oscillation.

### 7b. Lower Ki

GSP param `V4_KI_SHIFT` (default 4 = Ki of 1/16). Try **5 (Ki = 1/32)**, then **6 (Ki = 1/64)**. Reduces integrator wind-up.

### 7c. Re-verify with PI log

Same procedure as §6d. Steady-state `deltaClamped` should be ~zero with low variance.

### 7d. Tune in coarse-then-fine sequence

- First make sure Kp and Ki together stop the oscillation, even if response is slow
- Then bring Kp back up one step at a time until oscillation re-emerges
- Then back off one step

Typical good values for high-KV drone motors: KpShift = 3, KiShift = 4 or 5.

---

## 8. If BEMF captures are being dropped at high speed

`actualForcedComm` climbing, `goodZcCount` rate below expected, or `zcLatencyPct` drifting toward 0 or 100.

### 8a. Identify which edge of the sector is failing

`zcLatencyPct` near **0** = ZC arrives right at the start of the sector and gets blanked by software blanking. **Reduce `ZC_BLANK_PCT_FAST`** (per §6b).

`zcLatencyPct` near **100** = ZC arrives at the end of the sector and the commutation fires before the comparator can capture it. **Reduce advance** for this speed band (lower TAL=4 to TAL=3 in `sector_pi.c:954`).

### 8b. Tighten ATA6847 short-circuit and current-limit windows

If high-speed transient currents are tripping ILIM or SC protection mid-sector, the comparator goes into recovery and ZCs get lost. Through GSP:
- `ILIM_ENABLE` = 0 (disable cycle-by-cycle ILIM during high-speed runs; rely on the MCU CMP3 for protection)
- `SC_ENABLE` = 0 with caution; only if you can scope-confirm no real shorts

These are diagnostic-only — don't ship with both disabled. Re-enable after the experiment.

### 8c. Comparator gain / offset (advanced)

ATA6847 register `CSCR` controls CSA gain and offset. Default gain (`Gain_32`) and offset (`VRef_2`) are tuned for the validated motors. If your motor has very different BEMF amplitude, you may need:
- Higher KV motors: lower gain (`Gain_16`)
- Lower KV motors: higher gain (`Gain_64`)

Change via GSP `CSA_GAIN` param. Verify in scope that the BEMF comparator output is clean square waves at peak speed.

---

## 9. If BLK never engages (`d[37]` bit 2 stays 0)

Your motor's natural duty-saturation eRPM is **below** the per-profile `V4_BLOCK_ENTER_ERPM` threshold. BLK never triggers, so you stay at the PWM 96% ceiling.

### 9a. Lower the entry threshold

Open `garuda_config.h:824–859`, find the per-profile `V4_BLOCK_ENTER_ERPM_PROFILE_N` define for your active profile. Drop it by 20% and re-flash.

Worked example: if your `MOTOR_PROFILE = 2` and `V4_BLOCK_ENTER_ERPM_PROFILE_2 = 150000`, try **120000**. If BLK now engages at peak (bit 2 = 1), keep it. If it engages mid-throttle and causes thrashing, raise back stepwise.

### 9b. Make sure the amplitude threshold is reachable

`sector_pi.c:1003` requires `actualAmplitude ≥ 31130U` (95% Q15) for BLK entry. If your throttle path saturates at 90% Q15 (e.g., pot range limited), entry is unreachable.

Workaround for testing: lower the entry amplitude threshold to **29490U (90% Q15)** as a one-time bench experiment. If BLK now engages, the issue is in the throttle scaling, not the entry threshold.

### 9c. Then re-run §3 telemetry capture

With BLK now engaging, re-do the diagnostic matrix in §4. The wall may shift to one of the other mechanisms.

---

## 10. Re-validation after every change

After **every** parameter change in §5–§9, re-run the §3 capture procedure. Don't stack multiple changes — you won't know which one helped or hurt.

For each test:
1. Reset the board (power cycle or RESET button — don't just stop+start the motor)
2. Slow pot sweep up
3. Hold full pot 10 s
4. Read peak eRPM, peak stepPeriodHR, `actualForcedComm` delta, `d[37]` bit 2 at peak
5. Snap to zero, watch for clean exit (no fault, no desync)
6. Decide: keep, revert, or move to next knob

---

## 11. Knob reference card

| Knob | File / Param | Default | Increase if... | Decrease if... |
|---|---|---|---|---|
| `V4_KP_SHIFT` | GSP / `v4_params` | 2 | PI oscillates | PI too slow |
| `V4_KI_SHIFT` | GSP / `v4_params` | 4 | Integrator winds up | Steady-state error |
| `V4_RC_DELAY_HR` | `v4_params.h` constant | per profile | ZCs arrive early in steady state | ZCs arrive late |
| `EDGE_BLANKING` (ATA EGBLT) | GSP `0xDB` | 15 (3.75 µs) | Phantom edges from FET noise | Real ZCs being blanked |
| `ZC_BLANK_PCT_FAST` | GSP per profile | 8–15 | Forced comms from early ZCs | `zcLatencyPct` near 0 |
| `TIMING_ADVANCE_LEVEL` (per profile band) | `sector_pi.c:952–954` | TAL=2/3/4 | Motor wants more advance at top speed | Going past optimal angle (BEMF unstable) |
| `V4_BLOCK_ENTER_ERPM` (per profile) | `garuda_config.h:824` | per profile | BLK engages too early | BLK never engages |
| Predicted scheduling | `sector_pi.c:957` | not used | Want >30° effective advance | (architectural; risky) |
| `CSA_GAIN` | GSP `0xDC` | 32 | BEMF signal too small | Saturating comparator |

---

## 12. When to escalate (give up tuning, change architecture)

If you've done a full pass through §3–§9 and you're still 2%+ short of the reference ESC, and the diagnostic matrix shows "no diagnostic signal" (§5), the gap is likely **architectural**, not tunable.

Decision tree:
- **Reference ESC is trapezoidal too** but uses predicted scheduling → re-try the §5b code change, accept the risk of bench-time tuning to make it stable
- **Reference ESC switches to sinusoidal at top end** → out of scope for this firmware; not closeable without a sine-mode implementation (several weeks of work)
- **Reference ESC matches you on peak eRPM but you're losing on prop torque or throttle response** → not a peak-eRPM problem; different debug guide needed

Either way, document the findings in your bench notes and surface them — the firmware team needs to know which architectural path closes the gap before committing the work.

---

## 13. Common mistakes

- **Changing multiple knobs before re-capturing telemetry.** Always one change, one capture, one decision.
- **Trusting the comments in `sector_pi.c` as authoritative.** Some of them are stale or describe failed experiments under different parameter conditions. Verify with telemetry, not narrative.
- **Tuning to match the reference ESC's peak number without first confirming Vbus parity.** A 0.3 V Vbus difference can produce a 1% eRPM difference at the K·V ceiling.
- **Assuming BLK engaging = problem solved.** BLK raises voltage by ~4% but does **not** by itself raise sector rate. The PI still has to shorten `timerPeriod` to use the extra voltage as more speed.
- **Lowering `v4Params.minPeriodHr` from 10 to chase higher peak.** It's already at the physical safety floor; lowering further has no effect and risks tripping watchdog on a noisy capture.

---

## 14. Related documents

- `v4_architecture.md` — V4 sector PI architecture overview (read first if you haven't)
- `v4_motor_tuning_guide.md` — broader symptom → knob lookup for non-peak-related issues
- `v4_block_commutation.md` — concept and state machine for BLK
- `v4_block_comm_exit_tuning.md` — BLK→PWM exit tuning (different problem space)
- `v4_228k_milestone_session.md` — pre-BLK 228k PWM-only baseline and the bug fixes that got it there (good prior art if you suspect a regression)
