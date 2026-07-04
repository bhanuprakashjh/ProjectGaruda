# Overnight-B Complexity / Interaction Audit — Zero-Crossing Capture Path

**Repo:** `/media/bhanu1234/Development/ProjectGaruda-ak512/dspic33AKESC`
**Branch / HEAD:** `ak512-port` @ `db8cd0f`
**Mode:** READ-ONLY audit. No source edited.
**Active config:** `MOTOR_PROFILE = 2` (2810 1350KV, 7pp, 24V), classic startup (`FEATURE_SINE_STARTUP=0`), HW ADC comparator (`FEATURE_ADC_CMP_ZC=1`), sector-PI (`FEATURE_HWZC_SECTOR_PI=1`), duty-model threshold (`FEATURE_VIRTUAL_NEUTRAL=0`, `FEATURE_HWZC_MEASURED_NEUTRAL=0`).
**Runtime param store LIVE:** `FEATURE_GSP=1` — the values that actually run are `gspParams` (gsp_params.c profile-2 block, lines 175-303), **not** the compile-time `#define`s. This distinction changes several key numbers (see the box below).

---

## 0. Ground-truth constants (verified, not assumed)

| Quantity | Value | Source |
|---|---|---|
| PWM frequency | 45 kHz → **PWM period 22.22 µs** | `garuda_config.h:510` |
| SCCP1/SCCP2/SCCP3 clock | 100 MHz → **1 HR tick = 10 ns** | `garuda_calc_params.h:20,410`; hal_timer.c |
| eRPM ↔ HR ticks | `eRPM = 1e9 / stepPeriodHR`; `ticks = 1e9/eRPM` | `garuda_calc_params.h:415-416` |
| 1 PWM period in HR ticks | 100e6/45e3 = **2222 HR ticks** | `HWZC_ADC_TO_SCCP2`, calc_params:424 |
| Comparator ISRs | priority 7, disable own IE + clear CMPSTAT on entry, call `HWZC_OnZcDetected` | garuda_service.c:4773-4823 |
| SCCP1 ISR (`_CCT1Interrupt`) | priority 7, dispatches on `hwzc.phase` | garuda_service.c:4830-4861 |
| ADC/control ISR | 24 kHz? **No — one conversion per PWM period = 45 kHz**, priority 6 | garuda_service.c |
| SCCP3 BEMF burst | **1 MHz** (100-tick period) → 1 µs verify cadence | garuda_config.h:1005; calc_params:455 |

> **GSP-store overrides that matter (these are what the board runs):**
> - `timingAdvMaxDeg = 25`, `maxClosedLoopErpm = 260000` → **`RT_TIMING_ADV_FULL_ERPM = 260000`** (advance *anchor*, NOT a speed cap). So the advance schedule ramps 0°→25° over 3000→260000 eRPM. **At 40-65k the commanded advance is only ~4-6°.** (gsp_params.c:205, 227; calc_params:712-714)
> - `zcDemagBlankPerA = 8`, `zcDemagBlankMaxPct = 40`, `zcDemagBlankIbusDb = 30`, `zcDemagDutyThresh = 40` → the WS1 load-adaptive demag term is **ACTIVE above 40% duty**, up to a 40%-of-period blank cap. (The compile-time fallback `ZC_DEMAG_BLANK_PER_A=0` is NOT in force.) (gsp_params.c:221-226)
> - `RT_HWZC_MIN_STEP_TICKS = 1e9/260000 ≈ 3846 HR ticks` (gspDerived, gsp_params.c:1115) — the interval-gate hard floor and PI period floor.
> - **`FEATURE_SPEED_PI = 0` and `FEATURE_SPEED_CASCADE = 0`** (config:1750, 239) → **there is no outer speed loop.** Speed is set by a **direct throttle→duty map**, slew-limited. The sector-PI is the *only* closed loop, and it controls *commutation timing only*, never duty. This is central to the acceleration-snap analysis.
> - `desyncMaxRestarts = 0` (gsp_params.c:249) → recovery coasts then latches `FAULT_DESYNC`; no restart.

---

## 1. The full mechanism catalog (everything that can create / reject / delay / modify / reinterpret a capture, or change commutation timing)

Grouped by where they live in the pipeline. `[F]` = controlling flag/param. Each is real in the profile-2 build unless marked INACTIVE.

### A. Signal formation / thresholding (before the comparator ever fires)

| # | Name | Location | Trigger | Effect | Controls |
|---|---|---|---|---|---|
| A1 | **Duty-model threshold** | garuda_service.c:1070-1071 | every PWM ISR in CL | `zcThreshold = vbusRaw·duty / ZC_DUTY_DIVISOR` (AK512 divisor `=2.6·LOOPTIME_TCY`, calc:193). Models motor neutral = duty·Vbus/2. | `FEATURE_VIRTUAL_NEUTRAL=0` selects this path |
| A2 | **Threshold IIR smoothing** | garuda_service.c:1148-1154 | every PWM ISR, CL only | symmetric ¼-gain IIR, τ≈4 ticks. Lags fast duty ramps. | (always on in CL) |
| A3 | **Live CMPLO refresh** | garuda_service.c:1217-1246 → hal_adc.c:576-602 | every PWM ISR while `phase==WATCHING` | rewrites comparator CMPLO = threshold ± filter-offset ± deadband. Keeps threshold current mid-sector but re-injects A5/A6 every cycle. | `FEATURE_ADC_CMP_ZC` |
| A4 | **zcAmpForFilterComp IIR** | garuda_service.c:1173-1195 | per-sector, on step change | slow IIR (τ≈32 sectors) of measured BEMF peak; feeds A5 amplitude. | `GARUDA_TARGET_AK512` |
| A5 | **Filter comp (RC-lag pre-distortion)** | hwzc.h:48-103 (`HWZC_ApplyFilterComp`) | at OnCommutation + every CMPLO refresh | shifts threshold toward earlier fire by `amp·ωτ·AMP_PCT`; `ωτ=K_Q15/stepP`, capped. Rising & falling both 50%. Makes comparator fire *earlier* as speed rises. | `FEATURE_HWZC_FILTER_COMP=1`; `HWZC_FILTER_K_Q15=102943706` (τ=30µs), `AMP_PCT=50`, `MAX_OMEGA=24000`, `MAX_OFFSET=600` |
| A6 | **Comparator deadband** | hwzc.c:366-369 | at OnCommutation | rising: +4 counts, falling: −4 counts hysteresis on CMPLO. | `HWZC_CMP_DEADBAND=4` |
| A7 | **WS3 variable BEMF trigger** | hal_pwm.c (`HAL_PWM_SetBemfTrigger`) | on duty change | moves the PG-trigger ADC sample instant within the PWM period by duty. | `FEATURE_VARIABLE_BEMF_TRIGGER=1` (config:285) |
| A8 | **Falling OFF-window epsilon** | hwzc.c:352-360 | falling sector, low duty | INACTIVE — `FEATURE_HWZC_FALLING_OFFWIN=0` (bench-falsified). | `FEATURE_HWZC_FALLING_OFFWIN=0` |

### B. Capture gating (comparator fired — accept or reject?)

| # | Name | Location | Trigger | Effect | Controls |
|---|---|---|---|---|---|
| B1 | **Demag/ZC blanking (base + WS1)** | hwzc.c:246-290 (`HWZC_BlankTicks`) | after every commutation | comparator IE stays OFF for `blankPct·period` + load term. Base 14%; WS1 adds `perA·(Iexcess/256)` % above 40% duty & 30-count deadband, capped at 40% of period. **The whole front of every sector is deaf.** | `HWZC_BLANKING_PERCENT=14`; `FEATURE_ZC_CURRENT_BLANK=1`; `perA=8,maxPct=40,db=30,dutyThresh=40` |
| B2 | **PWM-edge gate** | hwzc.c:569-603 | on each comparator fire | reads H-side GPIO of PWMing phase; rejects capture unless PWM is ON. **Rejects the entire PWM-OFF fraction of every sector.** Increments `noiseRejectCount` (this is why rej% telemetry reads 92-99%). | `FEATURE_HWZC_PWM_GATE=1` |
| B3 | **Chop gate** | hwzc.c:610-630 | on fire, if `chopBlank>0` | after the SW OC limiter cuts duty, blanks ~2 PWM cycles (≈44 µs) of captures. | set at garuda_service.c:4248 (`chopBlank=2`); speed-gated OFF below the mute speed |
| B3s | **Chop-gate speed gate** | hwzc.c:620 | chop gate only above mute speed | only applies chop gate when `timerPeriod ≥ 1e9/70000`. Below 70k the chop gate is DISABLED. | `HWZC_FALLING_MUTE_ERPM=70000` |
| B4 | **Falling mute** | hwzc.c:441-454 | at OnBlankingExpired, falling sector | above 70k eRPM, leaves comparator IE OFF for odd sectors — falling half never captured; PI dead-reckons those 3 sectors. | `HWZC_FALLING_MUTE_ERPM=70000` |
| B5 | **Interval / plausibility gate** | hwzc.c:636-651 | on fire | reject if `interval < 50%·stepPeriodHR` (floored at `RT_HWZC_MIN_STEP_TICKS≈3846`). | `HWZC_MIN_INTERVAL_PCT=50` |
| B6 | **Verify reads (coherence)** | hwzc.c:653-720 | on fire, if `stepPeriodHR > 12500` (<80k) | waits ~1 µs, re-reads ADC; reject if signal flipped back. Adds ~1 µs detection latency below 80k. | `FEATURE_HWZC_VERIFY_READS=1`, `HWZC_VERIFY_READS=1`, `HWZC_VERIFY_SKIP_ERPM=80000` |
| B7 | **One-accept-per-sector** | hwzc.c:739 | after accept | disables comparator IE for the rest of the sector. | (structural in SECTOR_PI) |
| B8 | **First-ZC bypass** | hwzc.c:527-566 | first ZC after Enable | bypasses B5 and IIR; just anchors timestamp. | (structural) |

### C. Timing / PI interpretation (accepted capture → commutation instant)

| # | Name | Location | Trigger | Effect | Controls |
|---|---|---|---|---|---|
| C1 | **Sector-PI period control** | hwzc.c:840-1138 (`HWZC_OnPiPeriodExpired`) | autonomous SCCP1 fire at `lastComm+timerPeriod` | `delta=capValue−setValue`; `integrator += delta·Ki`; `timerPeriod = integrator + delta·Kp`. Commutation is driven by `timerPeriod`, NOT by the capture directly. | `FEATURE_HWZC_PI_FLOAT=1`, `Kp=0.25,Ki=0.0625` |
| C2 | **setValue (advance schedule)** | hwzc.c:844-891 | each PI fire | `setValue=(30+advDeg)/60·T`; advDeg interpolated 0→25° over 3000→**260000** eRPM ⇒ ~4-6° at 40-65k. This sets where in the sector the capture is *expected*. | `FEATURE_TIMING_ADVANCE=1`, `timingAdvMaxDeg=25`, `RT_TIMING_ADV_FULL_ERPM=260000` |
| C3 | **Cross-sector reject** | hwzc.c:904-909 | PI, `capValue > T` | discard capture, skip PI update, still commutate. | (structural) |
| C4 | **Per-capture min/max plausibility** | (config knobs) hwzc.c region + config:1663-1666 | PI | intended ±50%/150%-of-setValue bound (`CAP_MIN=1/2`, `CAP_MAX=3/2`). NOTE the *live* gate in hwzc.c is only the `capValue>T` reject (C3); the min-fraction bound is defined but the effective floor is C3+B5. | `HWZC_PI_CAP_MIN/MAX_*` |
| C5 | **Asymmetric per-capture delta clamp** | hwzc.c:913-936 | PI | clamp delta to ±T/8 (`SHIFT=3`) below 150k; above 150k pos=T/16, neg=T/32. At 40-65k the symmetric ±T/8 clamp applies. | `HWZC_PI_DELTA_CLAMP_SHIFT=3`, high-shifts 4/5, `CLAMP_HIGH_ERPM=150000` |
| C6 | **Handoff damp** | hwzc.c:922-933 | first 24 events after CL entry | limits period *shrink* to ≤ T/64 per event. | `FEATURE_HWZC_HANDOFF_DAMP=1`, `EVENTS=24`, `NEG_SHIFT=6` |
| C7 | **IIR freeze gate** | hwzc.c:755-756 | reactive-path IIR only | freeze `stepPeriodHR` IIR until `goodZcCount≥3`. (In PI mode the IIR is mirrored from `timerPeriod`, so this mostly gates the reactive fallback.) | `HWZC_IIR_FREEZE_ZC_COUNT=3` |
| C8 | **stepPeriodForFilterComp slow IIR** | hwzc.c:1099-1115 | each PI commutate | shift-11 IIR feeds A5's ωτ, decoupled from `timerPeriod` to break the shrink→earlier-fire→shrink loop. | (structural, SECTOR_PI) |
| C9 | **PI defensive mode** | hwzc.c:1050-1096 | 6 consecutive silent sectors | grows `timerPeriod` +1%/event until 2 good captures. | `FEATURE_HWZC_PI_DEFENSIVE=1`, trigger 6/exit 2/grow 1% |
| C10 | **ABS_FLOOR (harmonic-lock clamp)** | hwzc.c:982-1022 | PI, duty 3-100% | period can't be shorter than `P_ff·100/overspeed`. Shrink-only. `overspeed=130%` both tiers. `P_ff=181.38·λ/(Vbus·dutyFrac)`, λ=583µV·s/rad. | `FEATURE_HWZC_ABS_FLOOR=1`, `OVERSPEED_PCT=130`, `LOW=130`, `MIN_DUTYFRAC=0.03`, `LOW_DUTYFRAC=0.12` |
| C11 | **FF seed** | hwzc.c:166-188 | at Enable only | one-shot integrator seed. `HWZC_PI_FF_ENABLE=0` → INACTIVE. | `HWZC_PI_FF_ENABLE=0` |

### D. Commutation-timing modifiers outside the PI

| # | Name | Location | Trigger | Effect | Controls |
|---|---|---|---|---|---|
| D1 | **Reactive commDelay** | hwzc.c:401-428 | reactive path only | INACTIVE in PI mode (PI owns timing). | `FEATURE_HWZC_SECTOR_PI=1` disables |
| D2 | **Timeout / forced step** | hwzc.c:1146-1217 | reactive `WATCHING` timeout | INACTIVE in PI mode (SCCP1 in WATCHING → PI, not timeout). | ditto |
| D3 | **Duty slew limiter** | garuda_service.c:4006-4113 | every PWM ISR | up ≤ **2%/ms**, down ≤ 5%/ms of MAX_DUTY. Bounds how fast throttle→duty can climb → bounds accel rate. `up_rate ≈ MAX_DUTY·2/100/45 ≈ 31 ticks/ISR`. | `FEATURE_DUTY_SLEW=1`, `UP=2%/ms`, `DOWN=5%/ms` |
| D4 | **CL soft-entry duty cap** | garuda_service.c:4254+ | 500 ms after CL entry | separate gentle duty ramp at entry. | `FEATURE_CL_SOFT_ENTRY=1`, 500ms, /64 |
| D5 | **CAP_DUTY_HOLD** | config:1656-1658, service | eRPM near maxClosedLoopErpm | holds/lowers duty near the period floor. (Anchor=260k, so effectively never bites at 40-65k.) | `FEATURE_HWZC_CAP_DUTY_HOLD=1`, `MAX_PCT=86` |

### E. Current-side actuators that perturb the capture stream

| # | Name | Location | Trigger | Effect | Controls |
|---|---|---|---|---|---|
| E1 | **SW OC limiter** | garuda_service.c:4228-4251 | `ibusRaw>OC_SW_LIMIT` (18A) & duty>MIN | proportional duty cut toward MIN_DUTY; sets `chopBlank=2` (→B3). | `ocSwLimitMa=18000`, `ocFaultMa=21000` |
| E2 | **HW CMP3/CLPCI chop** | hal_pwm | cycle-by-cycle | INACTIVE — `OC_CLPCI_ENABLE=0`. | `OC_CLPCI_ENABLE=0` |
| E3 | **bemfSampleValid / AD2 settle** | garuda_service.c:1259-1270; hwzc.c:1237 | mux switch to A/C | gates SW-sample path for `ZC_AD2_SETTLE_SAMPLES=2` ticks. (Minor in HW-comparator mode.) | `ZC_AD2_SETTLE_SAMPLES=2` |
| E4 | **BemfBurstOff** | hal_adc.c:498-503; hwzc.c:219 | on HWZC disable | stops 1 MHz burst so it can't starve control ISR. | (structural) |

### F. Watchdogs (force RECOVERY / coast-resync / fault)

| # | Name | Location | Trigger | Effect | Controls |
|---|---|---|---|---|---|
| WS1 | **Load-adaptive demag** (also B1) | hwzc.c:246-290 | phase-I dependent | grows blanking under load. | see B1 |
| WS2 | **Phase-stall fault** | gsp params + service | phase-I ≥ `stallIphaseAdc=1800` for `stallDebounceMs=150`, above `stallArmErpm=5000` | `FAULT_STALL`. | `FEATURE_PHASE_STALL_FAULT=(prof 2\|9)` |
| — | **Caprate watchdog** | garuda_service.c ~3715 | 50 ms window | accepts < 25% of comms for 3 windows (150 ms) → RECOVERY. **But B2 counts PWM-OFF fires as rejects; rej%=92-99% here is the PWM gate, not true blindness** — caprate looks at *accepts vs comms*, which is a different ratio. | `FEATURE_HWZC_CAPRATE_WATCHDOG=1`, `MIN_PCT=25`, `MIN_COMMS=20`, `STRIKES=3` |
| — | **Mistime watchdog** | garuda_service.c ~3780 | 50 ms, duty 3-12% | eRPM > (no-load·1.15) sustained 8 windows (400 ms), not falling → coast-resync. | `FEATURE_HWZC_MISTIME_WATCHDOG=1`, `MARGIN_PCT=15`, `STRIKES=8`, `DECEL_EXCL=5` |
| — | **Phase-clip trip** | garuda_service.c ~3846 | 50 ms, duty>15% | window-max |Ia| ≥ 20A for 3 windows → coast-resync. | `FEATURE_IPHASE_CLIP_TRIP=1`, `20000mA`, `3`, `0.15` |
| — | **Coast-resync + spin-catch** | garuda_service.c 4666-4724 | recovery entry | coast; if `vbusRaw>1390` (~26V regen) re-arm coast; then latch `FAULT_DESYNC` (`desyncMaxRestarts=0`). | `RECOVERY_SPINCATCH_VBUS_ADC=1390` |

**Total: ~35 distinct mechanisms** touch a capture or its timing in the profile-2 build. Even excluding the inactive ones (A8, C11, D1, D2, E2), **~28 are live.**

---

## 2. Interaction matrix — which pairs fight or compound

The dangerous couplings, ranked by how directly they bear on the accel-snap:

| Pair | Interaction | Sign |
|---|---|---|
| **B1 (blank) × B2 (PWM gate)** | Blank kills sector front; PWM gate kills every OFF window in what remains. They **stack multiplicatively** on the listenable window (see §3). | COMPOUND (harmful) |
| **B2 (PWM gate) × A1 (duty threshold)** | The true ON-window crossing sits at neutral = duty·Vbus/2. As duty rises the ON window narrows *and* the crossing moves; the gate throws away the OFF-window crossing that would have disambiguated. | COMPOUND |
| **A5 (filter comp) × C1 (PI shrink)** | Filter comp fires earlier as ωτ grows; a shrinking period grows ωτ → earlier fire → PI reads capture early → shrinks more. **C8 (slow-IIR ωτ source) exists specifically to break this loop** — evidence the loop is real and known. | POSITIVE FEEDBACK (mitigated by C8) |
| **B4 (falling mute) × C1 (PI)** | Above 70k, 3 of 6 sectors feed the PI *nothing*; PI dead-reckons half the electrical cycle. Below 70k all 6 feed it. **The 40-65k band is entirely below the mute**, so falling captures ARE present there — but they arrive progressively late (RC lag = growing fraction of a shrinking sector), see §4. | REINTERPRET |
| **B3 (chop gate) × E1 (OC limiter) × C1** | OC cut → chopBlank=2 → 44 µs deaf → PI starved → worse timing → more current → more OC. Classic positive-feedback; B3s speed-gates it OFF below 70k to break it. | POSITIVE FEEDBACK (gated below 70k) |
| **C5 (delta clamp) × acceleration** | During accel the rotor genuinely arrives early each sector (negative delta). The ±T/8 clamp *caps how fast the PI can shorten the period.* If the real accel exceeds T/8 per sector, the PI **cannot keep up** and the captures walk earlier and earlier until they cross a gate. This is the accel-starvation seed (see §4). | RATE LIMIT (harmful under accel) |
| **C2 (advance anchor 260k) × 40-65k operation** | Advance is only ~4-6° at 40-65k, so setValue ≈ 0.55·T. Captures that arrive earlier than that (accel) produce negative delta that C5 clamps. A *correct* anchor at the operating speed would raise advance and move setValue earlier, giving more negative-delta headroom. **The 260k anchor starves advance exactly in the operating band.** | MISCONFIG (harmful) |
| **B5 (interval 50%) × C5 (clamp) × accel** | B5 rejects captures earlier than 50% of period. Under hard accel the real crossing can legitimately fall below 50% of the *stale* period → rejected → PI starved → period stays long → next crossing even earlier → cascade. | COMPOUND (harmful under accel) |
| **D3 (duty slew 2%/ms) × C1** | Duty slew bounds the *cause* of accel. 2%/ms is slow, so accel is gentle — but there is **no outer speed loop** (SPEED_PI off) to close the loop on eRPM; duty is open-loop to speed, and the PI only tracks timing. A duty step keeps accelerating the rotor while the PI chases. | STRUCTURAL |
| **A2 (threshold IIR) × D3 (duty ramp)** | During a duty ramp the ¼-gain threshold IIR lags the true neutral, biasing the crossing point transiently. | COMPOUND (transient) |
| **C10 (ABS_FLOOR) × mistime WD** | Two independent overspeed guards (period-shrink clamp + operating-point watchdog). Redundant coverage of the same phantom, different detection basis. | REDUNDANT (intentional belt-and-suspenders) |
| **Caprate WD × B2** | B2 inflates `noiseRejectCount` with every OFF fire, but caprate judges *accepts/comms*, so it is not fooled by rej%. However the rej% *telemetry* the user reads is dominated by B2 and is nearly useless as a health signal (92-99% at all speeds). | INSTRUMENTATION CONFUSION |

---

## 3. Timing budget across the 40-65k accel band — how much of each sector is LISTENABLE

Electrical cycle = 6 sectors. Sector period `T = 1e9 / eRPM` HR ticks = `1e6/(6·eRPM_mech...)` — in µs: **T_µs = 1e6/eRPM · (1/... )**; simplest: `T_µs = 1e9/eRPM / 100` = `1e7/eRPM`.

| eRPM | Sector T (µs) | PWM periods/sector (T/22.22) |
|---|---|---|
| 40,000 | **25.0 µs** | 1.13 |
| 46,700 | 21.4 µs | 0.96 |
| 55,000 | 18.2 µs | 0.82 |
| 63,000 | **15.9 µs** | 0.71 |
| 65,000 | 15.4 µs | 0.69 |

**This is the headline fact: at 46.7k the sector is already ≈ one PWM period, and by 55-65k the sector is SHORTER than a single 22.22 µs PWM period.**

### Per-sector subtraction (worst reasonable case, at 63k, 33% duty)

Sector T = 15.9 µs = 1590 HR ticks. Budget consumers:

1. **Base blanking B1 = 14% of T** = 2.23 µs. WS1 demag adds nothing at 33% duty (gate is 40% duty), so blank = **2.23 µs**.
2. **RC settle after commutation / mux** ≈ the PCB τ = 30 µs. The BEMF signal itself has a 30 µs time constant — *longer than the entire sector.* Even the "≈30 µs RC settle" you cited is **~2× the sector**. The comparator is watching a signal that is still slewing from the previous sector's step for the whole sector.
3. **PWM-ON fraction (B2 gate)** = duty = **33%** of each PWM period is listenable; 67% (OFF) is rejected outright. Over a 15.9 µs sector that spans ~0.71 of a PWM period, the ON time totals ≈ 0.33 × 15.9 = **5.2 µs**, but it is chopped into ≤1 ON pulse of ~7.3 µs that may or may not overlap the crossing.
4. **Verify reads B6** = +1 µs latency per accepted fire (active <80k, so YES here).

**Listenable arithmetic at 63k / 33%:**
```
Sector                         15.9 µs        (100%)
− base blank (14%)             −2.23 µs
= post-blank window            13.7 µs
× PWM-ON fraction (33%)        ×0.33
= raw listenable time           4.5 µs         (28% of sector)
```
So even before RC lag, **~28% of the sector is nominally listenable.** But layer in the RC time constant (30 µs > T): the crossing edge is smeared over more than a full sector, so the *effective* window where a clean, monotone, ON-time crossing sits is a fraction of that 4.5 µs. And the ONE ON-pulse in the sector has to *contain* the crossing instant. If the crossing lands in the 67% OFF fraction (which happens ~2/3 of the time by pure phase luck), the sector yields **zero** ON-window captures → PI dead-reckons that sector.

### Is the wall "the gates ate the whole sector"?
**Partly, but that is the steady-state framing and the user's correction rules it out as the primary cause.** At *steady* 63k the ~4.5 µs listenable window plus the falling-mute-inactive (63k<70k) means enough sectors land an ON crossing to hold lock — and indeed the bench shows steady cruise 30-42k and 26-28k stable, 13k idle stable. **The gates do NOT eat the whole sector at steady state in this band.** The wall is the *acceleration* case, analyzed next.

---

## 4. The acceleration case — early-capture starvation / misattribution

**Setup:** No outer speed loop. Throttle→duty (slew ≤2%/ms). A duty step makes the rotor accelerate. The sector-PI's `timerPeriod` (T) lags the true rotor period because the PI can only shorten T by a clamped delta per accepted capture.

**Per-sector the rotor "gains":** if the rotor accelerates at rate `a` (eRPM/s), over one sector (T seconds) the true period shrinks by roughly `dT/T ≈ a·T/eRPM · (T)`... concretely the *phase* the capture arrives early, relative to the stale T, is:

```
early_fraction ≈ (T_stale − T_true) / T_true
```

The PI's correction authority per sector is the **delta clamp C5 = T/8** (below 150k). So the PI can only track an acceleration where the per-sector period change stays under **T/8 = 12.5% of the period per sector.**

**When does the accepted-capture stream break?** Two failure gates trip, in this order, as accel rises:

1. **B5 interval gate (50%)** — as the rotor runs ahead, the *measured interval* between accepted captures shrinks. Once a real crossing arrives earlier than 50% of the *stale* stepPeriodHR, B5 rejects it as noise. Because the PI period lags (C5-clamped), stepPeriodHR stays long, so the 50% floor is measured against a too-long period → **legit early crossings get rejected.** Each rejection starves the PI → period stays long → next crossing even earlier → **positive-feedback starvation.**

2. **B2 PWM gate + shrinking ON window** — as duty rises during throttle-up AND the sector shrinks, the probability that a given sector's crossing lands in the ON fraction drops. Missed sectors → PI dead-reckons at the stale (too-slow) period → commutation lags the rotor further → the *next* capture arrives even earlier relative to setValue → larger negative delta → clamped → still can't catch up.

**The misattribution / alias:** once the accepted-capture stream is sparse and biased-early, the PI is receiving captures spaced at ~½ (or ⅔) of the true rotor period whenever it *does* accept — because it accepts one crossing, dead-reckons a sector or two, then accepts a crossing that is actually **one rotor sector later than it thinks.** The PI cannot distinguish "rotor is one full sector ahead" from "rotor crossed early in this sector." It locks onto the sub-harmonic that is consistent with the captures it managed to keep — landing at 1.5-2.0× the commanded speed, i.e. the observed **snap to a ratio above the duty's no-load formula.** That is exactly the "gear-slip to 1.5×/2× under capture starvation" the memory notes describe, and it is a **PLL-dynamics false-lock, not a fixed-speed ceiling** — consistent with the stochastic 39.7k/46.7k/63k onsets.

**Approximate accel-rate breakdown point.** The PI tracks cleanly while per-sector period change < T/8 (12.5%). Per-sector period change as a fraction ≈ `(a / eRPM²) · 1e9`... in practical terms, at 50k (T=20µs) the PI can absorb up to ~12.5% period change per 20 µs sector = a *sustained* dω/dt of order `0.125 · 50000 / 20µs ≈ 3.1e8 eRPM/s` in the *ideal* accept-every-sector case. **But** the accept rate in the 40-65k band is throttled by B2 (≈1/3 of sectors land an ON crossing) and B6 latency, so the *effective* tracking authority is ~⅓ of that. The break is therefore not the raw clamp — it is **clamp authority × accept probability.** When throttle-up drives duty (and thus torque and accel) past the point where `accel > (T/8 per sector) × P(accept)`, the accepted stream can no longer pull setValue in fast enough, B5 starts rejecting the early crossings, and the loop slips to the sub-harmonic within a few sectors. Because `P(accept)` is stochastic (phase luck of crossing vs ON window), the onset speed is stochastic in the 40-65k band — matching the bench.

**Which gates eat the EARLY crossings first (ordered):**
1. **B5 interval gate** — the primary killer of early-arriving captures (rejects <50% of stale period). *This is the single most accel-hostile gate.*
2. **B2 PWM gate** — reduces accept probability, compounding B5 by keeping the period stale.
3. **C5 delta clamp** — even accepted early captures can only move the period by T/8, so accepted-but-clamped captures still lag a fast ramp.
4. **B6 verify** — 1 µs latency per accept, minor but real below 80k.
5. **A5 filter comp** — biases the accepted crossing earlier as period shrinks; benign in the tracking direction but adds to the early-attribution.

---

## 5. Redundant / subsumed mechanisms in the profile-2 config

- **C10 ABS_FLOOR vs Mistime watchdog** — both guard the same low-duty harmonic phantom by the same λ-formula; different detection basis (per-event clamp vs 400 ms operating-point). Redundant on purpose; ABS_FLOOR is the cheaper one and mostly does the job at duty>3%. In the 40-65k accel band **neither is the mechanism at fault** (ABS_FLOOR is shrink-only and the snap is a *growth* to a higher speed above no-load; mistime WD is gated to duty<12%). They can be left as-is but they are *not* protecting against the accel-snap.
- **C4 per-capture min/max plausibility** — the min-fraction bound (setValue/2) is largely **subsumed by B5** (50% interval) + **C3** (`capValue>T`). It adds little.
- **C7 IIR-freeze** — in PI mode `stepPeriodHR` is a mirror of `timerPeriod`; the freeze gate mostly matters for the reactive fallback path, which is disabled. Largely inert.
- **A6 deadband (±4) vs A5 filter comp** — both shift the effective threshold; the deadband is a small fixed hysteresis on top of the dynamic comp. Not redundant but overlapping.
- **B3 chop gate below 70k** — already speed-gated OFF below the mute (B3s), so in the 40-65k band it is **inert.** Good.
- **D1/D2 reactive commDelay & timeout, C11 FF seed, E2 CLPCI, A8 OFF-window** — **dead in this build.** Pure carrying-cost / config noise.
- **WS3 variable BEMF trigger (A7)** — moves the sample instant; overlaps with the whole PWM-gate rationale. Its benefit at 40-65k (sector ≈ 1 PWM period) is questionable — there is barely a period to place a trigger within.

---

## 6. Proposed MINIMAL CORE (what a correct 6-step needs 0-100k)

A textbook sensorless 6-step at 0-100k needs exactly:
1. **A threshold** — the duty-model (A1) + light smoothing (A2). Keep.
2. **Blanking** — fixed % post-commutation to skip the demag spike (B1 base, no WS1 load term). Keep base, drop WS1 in the A/B.
3. **One crossing detector** — the HW comparator, watching continuously after blank (i.e. **B2 OFF**), one accept per sector (B7). Keep detector.
4. **A period estimator** — either the sector-PI (C1) with a **sane advance anchor** and a **wider delta clamp**, or the simpler reactive IIR + commDelay. Keep PI but retune C2/C5.
5. **A commutation timer** — SCCP1 autonomous (structural). Keep.
6. **One safety net** — coast-resync on sustained blindness (caprate WD). Keep one watchdog.

Everything else is defense-in-depth accreted against specific bench failures, and several of those defenses (B5 at 50%, B2, C5 at T/8, the 260k advance anchor) are **precisely what strangles the accel case.**

### Strip-list for an A/B "simplicity build" — config flips only, ranked by expected benefit vs risk

Ranked most-likely-to-help-the-accel-snap first. All are single flips; none touch the detector or the PI structure.

| Rank | Flip | File | Rationale (accel-snap) | Risk |
|---|---|---|---|---|
| **1** | **Fix the advance anchor: set `maxClosedLoopErpm` (advance anchor) or add `HWZC_ADV_FULL_ERPM` = ~70000** | gsp_params.c:205 / config | Restores real advance (→~15° at 55k) → setValue moves earlier → **more negative-delta headroom for the PI to track accel** without B5 rejecting. Directly attacks the starvation. | LOW-MED (changes top-end advance too; but top end already runs its own regime) |
| **2** | **Loosen B5: `HWZC_MIN_INTERVAL_PCT` 50 → 30** | config:998 | The 50% floor is the primary killer of legit early captures during accel. 30 was the known-good value; note says it was raised only for a λ=230 ABS_FLOOR artifact that no longer applies here. **Highest direct leverage on the reject cascade.** | MED (30 admits more phantoms at steady state — but bench had 30 working) |
| **3** | **Widen C5: `HWZC_PI_DELTA_CLAMP_SHIFT` 3 → 2** (±T/8 → ±T/4) | config:1525 | Doubles the PI's per-sector catch-up authority so it can follow a real ramp instead of falling behind and slipping. | MED (more responsive → can chase a phantom faster too; pair with #1) |
| **4** | **Disable B2 PWM gate: `FEATURE_HWZC_PWM_GATE` 1 → 0** | config:883 | In the 40-65k band the sector ≈ 1 PWM period, so the gate throws away ~⅔ of candidate crossings and keeps the period stale. Continuous comparator restores accept probability. **This is the biggest single un-gating.** History says it was THE fix for a *different* (one-shot 60° slip / 22A slam) failure, so this is the riskiest-to-regress. | HIGH (may reintroduce the load-slip it was added for — but that failure is a steady-state slip, not the accel snap; worth an isolated A/B) |
| **5** | **Disable WS1 load-demag: `zcDemagBlankPerA` 8 → 0** | gsp_params.c:223 | Removes the load-adaptive blank growth. At 33% duty it is already inert (40% gate), so this is mostly a top-band simplification; low benefit in the exact snap band but removes a moving part. | LOW |
| **6** | **Disable verify: `FEATURE_HWZC_VERIFY_READS` 1 → 0** | config:943 | Removes 1 µs/accept latency and one reject path below 80k. Modest. | LOW |
| **7** | **Drop redundant guards: `FEATURE_HWZC_MISTIME_WATCHDOG` 0, keep only caprate** | config:1608 | Neither guard fires in the accel band; removing one reduces surface area. Cosmetic for this bug. | LOW |

**Recommended first A/B (single-variable discipline):** flip **#1 alone** (advance anchor) — it is the lowest-risk change that directly widens the PI's ability to track a real acceleration, and it corrects a genuine misconfiguration (advance starved by a 260k anchor while operating at 40-65k). If that doesn't move the snap onset up, flip **#2** (interval 50→30) next. Reserve **#4** (PWM gate off) for a dedicated isolated run because it has the clearest regression history.

**What NOT to touch first:** the falling-mute (70k) does not bear on 40-65k (falling captures are present and, per the config note, are net *stabilizers* against the alias in this band — muting them at 20k made the snap *worse*). ABS_FLOOR/mistime are shrink/low-duty guards orthogonal to a growth-to-higher-speed snap.

---

## 6a. Cross-check against the proven-fast build (bd85e9b, 2026-06-17, 214k-285k VEX era)

Pristine clone at `/media/bhanu1234/Development/ProjectGaruda-github/dspic33AKESC` @ `bd85e9b`. **Caveat:** that tree was configured for `MOTOR_PROFILE 6` (VEX, λ=230, 6PP) — a *different motor* than the current profile-2 2810 — so profile-*gated* values differ. But it is the **same codebase**, so a mechanism's *existence* (its code/flag being present at all) is a clean, date-grounded signal: **anything absent from bd85e9b was added June 18 – July 4** and is a prime strip candidate; anything present in both is load-bearing across the 285k run.

| Mechanism | In bd85e9b (285k)? | In current (db8cd0f)? | Verdict |
|---|---|---|---|
| HW comparator + SECTOR_PI (C1) | YES (`FEATURE_HWZC_SECTOR_PI=1`) | YES | **LOAD-BEARING — core** |
| Base blanking 14% (B1 base) | YES (`HWZC_BLANKING_PERCENT=14`) | YES | **LOAD-BEARING** |
| Interval gate (B5) | YES (`HWZC_MIN_INTERVAL_PCT=50`, same "VEX restored 30→50" note) | YES | **LOAD-BEARING** (but 50 is the accel-hostile value even here) |
| PWM-edge gate (B2) | YES (`FEATURE_HWZC_PWM_GATE=1`) | YES | **LOAD-BEARING** — present in the 285k build. Lowers strip priority of #4. |
| Filter comp (A5) + slow-IIR ωτ (C8) | YES (`FEATURE_HWZC_FILTER_COMP=1`, `stepPeriodForFilterComp` present) | YES | **LOAD-BEARING** |
| Deadband (A6) | YES (`=2` for VEX, `=4` here) | YES | LOAD-BEARING (value profile-tuned) |
| PI defensive (C9) | YES | YES | LOAD-BEARING |
| Handoff damp (C6) | YES | YES | LOAD-BEARING |
| ABS_FLOOR (C10) | YES | YES | LOAD-BEARING |
| Delta clamp T/8 (C5) | YES (`SHIFT=3`) | YES | present in 285k → the *clamp* is load-bearing; but the 285k run used advance anchored at its own operating speed (see below), so the clamp was not starved |
| Falling detection | **`FEATURE_HWZC_FALLING_SW=1`** (SW off-center) | **falling-HW + `HWZC_FALLING_MUTE_ERPM=70000`** | **CHANGED** — the mute is a post-6/17 addition; the proven build used SW off-center falling, not HW+mute |
| **Chop gate (B3/B3s) + `chopBlank`** | **ABSENT — field does not exist anywhere in the tree** | YES | **ADDED after 6/17 → PRIME STRIP CANDIDATE** |
| **WS1 load-adaptive demag (`FEATURE_ZC_CURRENT_BLANK`)** | **ABSENT** | YES (perA=8) | **ADDED after 6/17 → PRIME STRIP CANDIDATE** |
| **Caprate watchdog** | **ABSENT** | YES | **ADDED after 6/17** |
| **Mistime watchdog** | **ABSENT** | YES | **ADDED after 6/17 → strip candidate** |
| **Phase-clip trip (`FEATURE_IPHASE_CLIP_TRIP`)** | **ABSENT** | YES | **ADDED after 6/17** |
| **Phase-stall fault (WS2)** | **ABSENT** | YES | **ADDED after 6/17** |
| **CAP_DUTY_HOLD (D5)** | **ABSENT** | YES | **ADDED after 6/17** |
| **Spin-catch (`RECOVERY_SPINCATCH_VBUS_ADC`)** | **ABSENT** | YES | **ADDED after 6/17** |
| **WS3 variable BEMF trigger (A7)** | **ABSENT (`FEATURE_VARIABLE_BEMF_TRIGGER` undefined)** | YES | **ADDED after 6/17 → strip candidate** |
| Falling OFF-window (A8) | ABSENT | present-but-0 | added then disabled |

### What this does to the strip-list

The proven-fast build **is** an empirical minimal-enough core: HW comparator + sector-PI + base blank + interval gate + PWM gate + filter comp + PI defensive + handoff damp + ABS_FLOOR, with **SW off-center falling** (no mute) and **no chop gate, no WS1 demag, no WS2/WS3, no caprate/mistime/clip watchdogs, no CAP_DUTY_HOLD, no spin-catch.**

This **re-ranks the strip-list with bench evidence**, and it *demotes* my judgment-only ranking of the PWM gate:

1. **B2 PWM gate — DEMOTE from #4 to "keep."** It was present in the 285k build, so it is not the accel-snap culprit by itself. Do NOT strip it first.
2. **Chop gate (B3/B3s) — PROMOTE to top strip candidate.** It did not exist in the proven build. Flip: it's controlled by the `chopBlank` set at garuda_service.c:4248 (guarded by `FEATURE_HW_OVERCURRENT`). Cleanest A/B is to neutralize it — since B3 is already speed-gated OFF below 70k (B3s), it is inert in the 40-65k *band*, so stripping it is low-benefit for the snap specifically but removes a proven-absent moving part. **Low benefit for the snap, but pure carrying cost — safe to remove for the simplicity build.**
3. **WS1 load-demag (`zcDemagBlankPerA` → 0) — confirmed strip candidate** (absent from 285k). Inert at 33% duty but proven-unnecessary.
4. **WS3 variable BEMF trigger (`FEATURE_VARIABLE_BEMF_TRIGGER` → 0) — confirmed strip candidate** (absent from 285k; questionable value when sector ≈ 1 PWM period).
5. **Falling-HW+mute → revert to `FEATURE_HWZC_FALLING_SW=1` (the proven mechanism).** This is the single largest *architectural* divergence from the proven build in the falling path. The config note already records that muting falling *worsened* the alias; the proven build never muted — it captured falling on the SW off-center sample. This is a strong, evidence-grounded A/B: **restore the proven falling path.** (Higher risk / larger change; run isolated.)
6. **Redundant watchdogs (caprate/mistime/clip/stall) — all post-6/17.** None fire in the accel band and the 285k build ran without any of them. Safe to disable for the simplicity build to shrink surface area; keep one (caprate) as the single net.

### Where my §6 flips #1-#3 stand against the evidence

- **#1 advance anchor** — bd85e9b profile 6 has its own anchor; the *decoupling knob* `HWZC_ADV_FULL_ERPM` exists in both trees. The principle (anchor at operating speed, not 260k) is not contradicted by the proven build — profile 6 operated much closer to its anchor. **Still the recommended first flip.**
- **#2 interval 50→30** — bd85e9b *also* runs 50 with the identical "VEX restored 30→50" comment, so 50 was load-bearing at 285k. This **weakens** the case for 30 as a blind flip; treat #2 as second-line, and note the proven build tolerated 50 *because it wasn't accel-starved* (its advance/clamp let the PI track). Prefer fixing the anchor (#1) first.
- **#3 delta clamp T/8→T/4** — bd85e9b runs T/8 (`SHIFT=3`) too. So the clamp value is proven at 285k; again, the proven build wasn't starved because the anchor gave it headroom. **Fix the anchor before touching the clamp.**

**Net:** the empirical minimal core says the accel-snap is *most likely* an interaction that the post-6/17 additions introduced or a profile-2-specific misconfiguration (the 260k anchor), **not** the PWM gate / interval / clamp values that were all present and healthy at 285k. The highest-confidence, evidence-grounded moves are: **(1) fix the advance anchor**, and **(5) restore the proven `FEATURE_HWZC_FALLING_SW` falling path in place of falling-HW+mute** — then strip the proven-absent accretions (chop gate, WS1, WS3, extra watchdogs) to shrink the system back toward the shape that ran 285k.

---

## 7. Bottom line

The 40-65k accel snap is not "the gates ate the whole sector" at steady state — steady cruise in that band is stable. It is an **acceleration-triggered PLL false-lock**: under throttle-up the rotor accelerates faster than the sector-PI can shorten its period, because (a) the advance anchor at 260k starves setValue advance in the operating band, (b) the ±T/8 delta clamp caps catch-up authority, and (c) the 50% interval gate + PWM-ON gate reject the *legitimately-early* captures that would let the PI catch up — starving the loop until it locks onto the ½/⅔ sub-harmonic at 1.5-2× the commanded speed. The fix path is to give the PI enough authority and enough accepted early captures to track real acceleration (flips #1-#3), not to add another gate.
