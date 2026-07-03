# Reference Codebase Study — Atomberg "highSpeed_v1" Trapezoidal 6-Step
### Microchip MCLV-48V-300W + dsPIC33AK512MC510, T-Motor U3 KV700

> Source tree studied:
> `…/atomberg-drone-support-2026-feature-highSpeed_v1/mclv48v300w-33ak512mc510-bldc-highspeed-sensorless-trapezoidal/project/`
>
> Studied 2026-06-26 to inform our U3 load-desync / speed-under-load work on
> `dspic33AKESC` (branch `ak512-port`, MOTOR_PROFILE=9 / GSP_PROFILE_U3).
> All findings are read directly from the reference source (file:line cited).
> **Nothing here is committed; this is a study/port-notes document.**

---

## 0. TL;DR — the things that matter for us

1. **HARDWARE SCALING MISMATCH (read this first).** Same board, same 3 mΩ shunt,
   but the reference uses **op-amp gain 8.3 → 66.265 A peak**, while our firmware
   uses **gain 24.95 → 22.06 A peak**. The reference *explicitly migrated* from
   24.95→8.3 (`MC1_PEAK_CURRENT 66.265f//22.0f`, the `//22.0f` is the old gain-24.95
   value). If 8.3 is correct for this DIM (strong evidence it is), **our current
   readings under-report by 3×** and our "21.8 A rail" phantom current is really
   **~65 A**. See §2.
2. **The reference is the architecture we want**: outer **speed PI → inner
   bus-current PI → duty** cascade. Throttle commands *speed*; the loop holds RPM
   by pushing current up to a clamp. This is what "push more current and maintain
   speed" means in code. See §4–§5.
3. **The "highSpeed" feature = variable BEMF trigger** (`variable_trigger.h`):
   move the ADC sample to 0.6/0.8/0.9× duty as duty rises + multi-sampling. This
   is the most portable idea for our high-speed ceiling. See §6.
4. **Neither firmware adapts blanking to current** → both desync under load the
   same way. Current-scaled demag blanking is genuinely unsolved everywhere and is
   *our* piece to add. See §6.4.
5. **Both are blind to a freewheeling stall** (DC-link single shunt, phase current
   compiled out, bus-only OC at 53 A). Our planned phase-current stall fault is a
   real improvement over the reference, not just parity. See §7.

---

## 1. What this codebase is

The **stock Microchip MCLV-48V-300W trapezoidal sensorless demo**, lightly adapted
by Atomberg, on the **same silicon as ours** (dsPIC33AK512MC510). It is **not**
Garuda/GSP-derived — different lineage, different conventions, MCC-generated HAL.

- BEMF zero-cross sensorless, **bus-current** inner loop (no per-phase current in
  any control path).
- Active build: `MOTOR 4` (U3 KV700), `CLOSED_LOOP 1` (speed), `ENABLE_CURRENT_CONTROL`
  defined → cascaded speed+current control (`mc1_user_params.h:62,70,132`).
- It is a **work-in-progress branch**: `variable_trigger.*` stamped "Created
  May 20, 2026"; several `#warning remove this line` debug toggles remain in the
  commutation timing path (`trapezoidal_control.c:210,218`). Don't trust any single
  constant as "final."

Layout:
```
project/
  main.c                      entry
  mc1_init.c / .h             param + PI gain computation, wiring
  mc1_service.c / .h          app FSM, ADC ISRs, throttle ingress, duty write
  mc1_user_params.h           USER knobs (board scaling, motor select, loop mode)
  mc1_calc_params.h           DERIVED constants (all the real numbers)
  control/
    trapezoidal_control.c     control FSM + speed/current loops + commutation
    estim_bemf.c / .h         the live ZC engine (neutral + sector decode)
    estim_interface.c         thin shim → estim_bemf
    variable_trigger.h        duty-adaptive ADC sample point (the "highSpeed" bit)
    pi.c / .h                 PI controller (parallel form, cond-integration AW)
  hal/
    pwm.c (59 KB)             PWM/timer config, commutation override registers
    adc.c                     ADC channel/trigger map
    measure.c                 offset cal + counts→amps/volts scaling
    port_config.c             GPIO/PPS + op-amp (OpampConfig)
    cmp.c                     CMP3 + DAC3 (bus OC comparator; OFF in this build)
    board_service.c, uart1.c, clock.c, sccp2.c, delay.c
  motor/
    tmotor_u3_kv700.h         OUR motor profile
    sparrow_3115-kv900.h, alex_2807-kv1300.h, linix36zwn24.h
  x2cscope/diagnostics.c      X2CScope telemetry over UART1
```

---

## 2. HARDWARE & SCALING — "same board, did the shunt change?"

> **RESOLVED 2026-06-26:** the board is the **stock/unmodified AK512MC510 DIM**, so
> the reference's **gain 8.3 is correct**; our 24.95 was the inherited error. Fixed
> in firmware (WS5): `OC_GAIN_X100 830`, derived `COUNTS_PER_AMP`, telemetry + FOC
> scales updated. Baseline now reads/limits in REAL amps (OC bites ~3× earlier than
> the old wrong-gain behavior — re-validate startup inrush vs `ocStartupMa`).

**Shunt is the same (3 mΩ). The op-amp GAIN differs, and that is the whole story.**

### Reference (`mc1_user_params.h:154-160`)
```c
#define MC1_PEAK_VOLTAGE   75.9f
#define MC1_PEAK_CURRENT   66.265f//22.0f   // Vref/(Gain*Rshunt) = 1.65/(8.3*0.003)
#define DC_LINK_VOLTAGE    24.0f
```
- Gain **8.3**, Vref 1.65 V, Rshunt **0.003 Ω** → peak ±66.265 A at full ADC swing.
- The trailing `//22.0f` is the **previous value**, which is exactly
  `1.65/(24.95·0.003) = 22.06`. They **corrected the gain from 24.95 to 8.3** for
  this DIM.

### Ours (`garuda_config.h:2003-2006`, `hal/hal_adc.c:263`, `hal/port_config.c:368`)
```c
#define OC_SHUNT_MOHM   3        /* 3 mΩ — SAME */
#define OC_GAIN_X100    2495     /* 24.95 — DIFFERENT */
#define OC_VREF_MV      1650     /* 1.65 V — SAME */
#define OC_VADC_MV      3300     /* 3.3 V — SAME */
// "Shunt = 3 mΩ, op-amp gain = 24.95 default … ~93 ADC counts/amp … 22 A → ~4094/2"
```

### The scaling math (12-bit ADC, 2048 = bias)

| Quantity | Reference (gain 8.3) | Ours (gain 24.95) |
|---|---|---|
| V/A at op-amp out | 0.003·8.3 = **24.9 mV/A** | 0.003·24.95 = **74.85 mV/A** |
| Full-swing current (1.65 V) | **±66.27 A** | **±22.06 A** |
| 12-bit counts/amp | 2048/66.27 = **30.9** | 2048/22.06 = **92.8** |
| Q15 scale `A/LSB` | 66.265/32768 = **2.022 mA** | (n/a, we use integer) |

### Why this matters — the op-amp railing is a *physical* event
The op-amp output saturates near the supply rail (~1.65 V swing → ~2047 counts).
The **physical current** that drives it to rail depends on the **real** gain:
- If real gain = **8.3**: rail at **66 A** real. Our firmware (92.8 counts/A) reads
  2047 counts and reports **22 A** — under-reporting by 3×.
- If real gain = **24.95**: rail at **22 A** real, and our reading is correct.

**Evidence strongly favors 8.3 for this DIM:** the reference is a Microchip-authored
project *for the dsPIC33AK512MC510 DIM on this exact board*, and it deliberately
deleted the 24.95-derived 22.0 in favor of the 8.3-derived 66.265. Our 24.95 is
the dsPIC33CK MCLV DIM value and was most likely carried over verbatim during the
AK512 port without re-checking the DIM's gain resistors.

### Consequences if 8.3 is right (to verify on bench)
- **"21.8 A rail" phantom current ≈ 65 A real.** The load-desync end state is far
  more violent than our telemetry suggests; the rotor/FETs see ~3× the current.
- **Every "amps" threshold we set is 3× off**: `ocLimitMa`, `ocFaultMa`, the CMP3
  DAC reference, any current clamp we add for the speed loop.
- **The 22 A U25B board-comparator note** (`hal/hal_adc.c:251-271`) was reasoning
  about a 22 A trip in *our* (wrong-gain) frame — re-examine it in the 66 A frame.

### ACTION (cheap, decisive)
Measure the DIM's current-amp gain: known DC load current through a phase/bus,
read OA1OUT/OA3OUT (or the reported counts), compute mV/A. Or read the DIM
schematic gain resistors. If gain ≈ 8.3, set `OC_GAIN_X100 = 830` and re-derive all
current thresholds — this likely changes the load story materially.

### Other board scalars (agree, sanity check)
- **Voltage:** reference `MC1_PEAK_VOLTAGE 75.9 V`, `ADC_VOLTAGE_SCALE = 75.9/4096
  = 18.53 mV/count` (`mc1_calc_params.h:67`). Cross-check our `vbusRaw` scale
  (~53.8 counts/V ≈ 18.6 mV/count → **matches**, voltage divider is the same).
- **DC link:** reference hard-codes **24 V**; we run **16 V**. In the reference this
  mis-scales MaxSpeed and clamps (see §8). For us it's a porting caveat, not a live
  bug (we don't use their constants), but any borrowed formula must use 16.

---

## 3. Peripheral configuration & rationale

### 3.1 PWM / commutation (`pwm.c`, `pwm.h`)
| Param | Value | Why |
|---|---|---|
| Frequency | **40 kHz** (25 µs) | More samples per electrical cycle for high eRPM |
| Resolution | High-res, 400 MHz PWM clock, 156.25 ps LSB; `MPER=159984` | Fine duty at 40 kHz |
| Mode | **Independent Edge-aligned** (`MODSEL=0`), **Independent output** (`PMOD=1`) | 6-step needs per-leg override, not complementary SVM |
| Dead time | 4800 cnt = **750 ns** | FET shoot-through guard |
| Generators | PG1 master, PG2/PG3 slaved; shared period | Synchronized 3-leg timing + ADC triggers off PG1/PG2 |

**Commutation = register override**, not duty steering (`board_service.h:112-116`,
`trapezoidal_control.c:866-945`):
- `PWM_ON  0x00100000` → high-side chops, low-side off (active/driven phase)
- `PWM_OFF 0x00301000` → high-side off, low-side forced ON (return phase → DC−)
- `FLOATING 0x00300000` → both off, phase high-Z (BEMF sense phase)

So **active phase high-side PWM, return phase low-side held ON, third floats.**
During the chop OFF-time the current freewheels through low-side devices — **this
freewheel current does NOT pass through the DC-link shunt** (the root of the
phantom blind spot, §7).

Commutation tables (`trapezoidal_control.c:51-53`): FWD `{1,5,4,6,2,3}`, REV `{4,5,1,3,2,6}`.

### 3.2 ADC (`adc.c`) — 3 cores, role-partitioned
| Signal | Core/Chan | PINSEL | Trigger | Note |
|---|---|---|---|---|
| IA / IB / IC | AD1CH2 / AD2CH1 / AD3CH1 | OA1 / OA2 / ext | PWM (src 4) | **#ifdef'd OUT — never triggered** |
| **IBUS** | **AD3CH0** | OA3OUT (0) | **PWM src 7** | sampled IN the ON window |
| VBUS | AD3CH2 | RF0 (4) | PWM src 5 | **its conv-done = the 40 kHz control ISR** |
| POT | AD2CH2 | RB15 (5) | PWM src 5 | speed reference |
| VA / VB / VC | AD1CH0 / AD1CH1 / AD2CH0 | RA9/RA10/RB8 | **software (CH0TRG)** | fired by variable_trigger |

Two ISRs (decoupled): **`_AD3CH2Interrupt` (VBUS)** = the 40 kHz control loop
(reads IBUS/VBUS/POT, runs FSM, writes duty, `mc1_service.c:236-256`); **`_AD1CH1Interrupt`
(VB)** = the BEMF sample ISR (latches VA/VB/VC, runs the ZC estimator,
`mc1_service.c:258-281`). Why split: the ZC estimate runs at the duty-adaptive
sample instant, independent of the control cadence.

Note IC is the **only external-amplifier** channel (`AD3CH1 PINSEL=3` = AD3AN3/RB13,
`port_config.c:184-187`); IA/IB/IBUS use the internal op-amps. (All moot here since
phase currents are compiled out.)

### 3.3 Op-amps (`port_config.c:281-413` `OpampConfig`)
- OA1=IA, OA2=IB, OA3=IBUS, all `AMPEN=1`, **`HPEN=1` (high-power/high-bandwidth)**,
  `UGE=0` (external-resistor gain — the 8.3 gain lives in DIM resistors), `DIFFCON=0`
  (both diff pairs), all offset-correction fields 0.
- `INTERNAL_OPAMP_CONFIG` defined → op-amp OUT pins drive the ADC internally
  (TRIS=0 on RA2/RB0/RA5). Note this DIM/header has **no `OMONEN` field** — same
  thing our AK512 port discovered (`hal/port_config.c:340` TODO). The reference
  simply doesn't reference OMONEN, consistent with our finding that MC510 lacks it.

### 3.4 Comparator CMP3 + DAC3 (`cmp.c`) — bus OC, **DISABLED in this build**
- CMP3A = OA3OUT (`INPSEL=0`), ref from DAC3, 45 mV hysteresis (`HYSSEL=0b11`).
- Intended for cycle-by-cycle **bus**-current PCI limiting, but `ENABLE_PWM_CURRENT_LIMITER`
  is auto-`#undef`'d whenever `ENABLE_CURRENT_CONTROL` is set (`mc1_user_params.h:91-94`)
  → CMP3 left disabled, all `PGxCLPCI1=0`. So in the live U3 build **there is no
  hardware chop**; current is bounded only by the software loop + 53 A latch.

### 3.5 Clocks/timer
FCY 200 MHz; commutation interval measured by **SCCP2 free-running @ 100 MHz**
(`StandardClockFrequencyGet`). `SPEED_MULTIPLIER` uses FCY/2 = 100 MHz.

---

## 4. Control architecture (the cascade)

```
POT(ADC) --slew 2s--> controlInput --map--> targetSpeed(rpm)
                                              │
 CONTROL_INIT → ROTOR_LOCK(0.5s) → FORCED_COMMUTATION_STARTUP(V/f ramp 1s)
   align@Ilock     (current PI @ rotorLockCurrentRef)   integrate angle, ramp freq,
                                                          in parallel: BEMF phase-map
                                                          + 5 sectors within ±5% speed
                                              │ handoff (closedLoopStartingFlag)
                                              ▼
                    ┌──────────── SPEED_CONTROL_LOOP (live) ────────────┐
   estimator ──────►│ 1 kHz: speedRef ramp ±1.5 rpm/tick                │
   (async ISR):     │ piSpeed(omegaFiltered vs speedRef) → Iref ≤35.4A  │
   VA,VB,VC →       │ 40 kHz: piCurrent(Ibus vs Iref)    → Vcmd ≤22.8V  │
   neutral, sector, │ duty = (Vcmd/Vdc)·pwmPeriod                       │
   60° interval,    │ commutate 30° after ZC sector edge                │
   commPeriod=int/2 └───────────────────────────────────────────────────┘
   Ibus>53A → MCAPP_FAULT (latched). Desync at low Ibus: NO fault.
```

State machines: app FSM `MC1APP_StateMachine` (`mc1_service.c:87-217`) runs the
control core inside the VBUS ADC ISR; control FSM `MCAPP_TrapezoidalControlStateMachine`
(`trapezoidal_control.c:240-572`). Startup = align → V/f forced commutation →
two-gate handoff (phase-mapping, then 5 consecutive sectors within ±5% of target).

---

## 5. CURRENT LOOP — what we can borrow (the core of this study)

This is the part worth importing conceptually. Our `FEATURE_SPEED_PI` is compiled
off and throttle maps straight to duty; this is the missing structure.

### 5.1 PI controller (`pi.c:59-86`) — clean, portable
Parallel form with **conditional-integration anti-windup** (integrator frozen while
output is saturated — no separate clamp/back-calc needed):
```c
error    = inReference - inMeasure;
outUnsat = integrator + kp*error;
if      (outUnsat > outMax) output = outMax;          // saturate, freeze I
else if (outUnsat < outMin) output = outMin;          // saturate, freeze I
else  { integrator += ki*error*samTime; output = outUnsat; }  // integrate
```
No FF in the PI itself; the only feedforward is the startup V/f term.

### 5.2 The cascade (`trapezoidal_control.c:465-543`)
- **Outer speed PI @ 1 kHz**: `inMeasure = omegaFiltered`, `inReference = speedRef`,
  **output is a CURRENT reference**, clamped to `RatedCurrent` (35.4 A for U3).
  *This clamp is the current ceiling that "holds speed by pushing current up to a
  limit."*
- **Inner current PI @ 40 kHz**: `inReference = piSpeed.output`,
  `inMeasure = Ibus` (bus current), output = voltage cmd, clamped to
  `maxVoltage = 0.95·Vdc`. Then `voltageRaw = out/Vdc`, `pwmDuty = voltageRaw·period`.

### 5.3 Gains are auto-computed from motor params (`mc1_init.c:162-188`) — borrowable
- Current loop (modulus-optimum-style):
  `Kp = 2·Ls/(4·ζ²·1.5·Tpwm)`, `Ki = 2·Rs/(4·ζ²·1.5·Tpwm)`, ζ=1.0.
- Speed loop (pole-placement from inertia, Ke, pole pairs, 50 Hz feedback LPF, 85°
  PM). Full closed-form in `mc1_init.c:165-188`.
- Output clamps: speed-PI `outMax = RatedCurrent`; current-PI `outMax = 0.95·Vdc`.

### 5.4 Throttle ingress (`mc1_service.c:392-410`)
Pot → `RateLimitSym` slew over 2 s → `controlInput` → mapped to
`targetSpeed ∈ [MinSpeed, MaxSpeed]` → second ramp at 1.5 rpm/tick. Two-stage
smoothing prevents step demands from slamming the loops.

### 5.5 Offset calibration (`measure.c:107-167`)
1024-sample average at run-entry (bridge configured), then per-sample subtract.
Same idea as our OC-autozero. Q15 conversion via `(raw−2048)<<4` then `·ADC_CURRENT_SCALE`.

### What to borrow for `dspic33AKESC`
- **The cascade shape**: speed PI output = current ref (clamped) → current PI →
  duty. Enable/port `FEATURE_SPEED_PI` this way rather than throttle→duty.
- **The current clamp as the speed loop's safety**: cap the demanded current so
  "hold speed under load" can't run away into desync (gated below our ZC current
  ceiling — see §6.4 / §9).
- **Conditional-integration AW** if our PI doesn't already use it.
- **Auto-gain formulas** from Ls/Rs/inertia as a starting point (then bench-tune).
- ⚠ **All current numbers must be re-based to the correct gain (§2) and 16 V.**

---

## 6. BEMF / ZC + the "highSpeed" feature

### 6.1 ZC method (`estim_bemf.c:118-143`) — different from ours
Not edge-catching. Reconstructs virtual neutral `(Va+Vb+Vc)/3` and decodes the
full 6-step sector from **signs of (phase − neutral)** — a software 3-comparator
(Hall-like). ADC-sampled **during PWM-ON** at a duty-scaled instant (opposite of
our freewheel-gated approach).

### 6.2 Robustness gates
- Post-commutation **blanking** (speed-scaled only, §6.4).
- **2-of-3 majority filter** on the decoded sector (`estim_bemf.c:142`).
- **Interval glitch reject**: drop intervals implying >1.5× expected speed
  (`estim_bemf.c:189`).
- **Sector-sequence plausibility hold**: if estimator sector ≠ expected next, set
  `commutationSectorMissmatched` and **hold previous sector** (no resync, no fault)
  (`trapezoidal_control.c:866-892`).

### 6.3 The "highSpeed" feature = variable trigger (`variable_trigger.h`)
There is **no `#ifdef HIGHSPEED`**; the feature is the duty-adaptive BEMF sample:
- 3-state hysteretic machine on `pwmDuty`, writes `PG2TRIGA = SCALE·pwmDuty`:
  STATE_1 **0.60**, STATE_2 **0.80**, STATE_3 **0.90** of the ON pulse.
- Thresholds anchored to `BEMF_RC_RISE_TIME_COUNTS` (2 µs RC settle):
  T1→T2 at 1.25×, T2→T3 at 3× (hysteresis to avoid chatter).
- **Multi-sampling**: at high duty (STATE_3) extra conversions are software-triggered
  per period (`mc1_service.c:283-316`) so the floating phase is sampled multiple
  times when the ON window is short.

**Why**: at high eRPM the sector shrinks and the RC-filtered BEMF lags; pushing the
sample deeper into the ON pulse (and sampling more often) keeps a valid reading.
**This is the single most portable idea for our high-speed ceiling.**

### 6.4 Demag / load — CONFIRMED ABSENT (the gap we must fill)
- Blanking is **speed/frequency-only, fixed at compile time** (`OL_BLANKING_TIME`,
  `CL_BLANKING_TIME`, `mc1_calc_params.h:124-131`). No `Ibus`, no current, no
  measured demag anywhere. Counter reset on commutation, incremented per ISR,
  compared to the static limit (`estim_bemf.c:124`).
- Worse: above **65% of MaxSpeed blanking is disabled entirely**
  (`UpdateBlankingTime`, `estim_bemf.h:149-160`).
- The estimator struct has **no current field** — current never reaches blanking.

⇒ Under heavy load the demag interval lengthens with current while
`blankingTimeLimit` stays fixed → decode window opens while the freewheel diode
still masks BEMF → wrong/late sector → the desync we see. **Even this reference
desyncs the same way.** Natural insertion point for *us*:
`blankingTimeLimit = closedLoopBlankingTime + f(measured current)` at
`estim_bemf.c:124` (equivalent to our `zcDemagBlankExtraPct` scaled by measured
amps — which our prior Modelica work already pointed at).

### 6.5 Commutation timing — fixed 30°, no advance knob
`commutationPeriod = commutationInterval/2` (`estim_bemf.c:200-203`); commutation
scheduled when within one PWM period (`trapezoidal_control.c:192-226`). **No
speed/load timing-advance term exists** — the reference itself flags this as the
missing lever for high-speed desync. Our profile-based `timingAdvMaxDeg` is ahead
of the reference here.

### 6.6 Speed estimate (`estim_bemf.h:107-146`)
`speed = SPEED_MULTIPLIER/avgPeriod`, two filters: leaky moving-average (1/16) on
the period + bilinear 50 Hz LPF on speed → `omegaFiltered` (the speed-PI feedback).
On missed ZC, speed bleeds down slowly via `motorStallCounter` — it never freezes,
but also never faults.

---

## 7. Faults / protection — and the shared blind spot

Three fault states (`fault.h:69-75`): NONE, OVERCURRENT_PHASE (misnomer),
OVERCURRENT_DCBUS.

- **Active in U3 build: ONLY software bus-current latch** `Ibus_actual > ~53 A`
  (`fault.c:79-96`, `mc1_init.c:145-146`). Latched, no recovery. At our load
  current it never trips → **no fault code, motor sits locked** (matches our
  symptom exactly).
- **No UV/OV at all** (Vdc only normalizes duty). (We have UV — keep ours.)
- **HW chop disabled** (CMP3 off when current-control on, §3.4).
- **Stall "detection"** only decays the speed estimate (`estim_bemf.h:128-141`) —
  not a trip.
- **Phase current compiled out** (`ENABLE_PHASE_CURRENT_MEASUREMENT` undef) → no
  phase-referenced limit exists.

**Shared blind spot**: the DC-link single shunt only sees current sourced through a
conducting high-side switch. In a freewheeling/phantom stall the current
recirculates through the low-side devices and **bypasses the shunt → bus reads ~0
while the phase carries the real (large) current.** Bus-only OC/limit cannot see
it. **Our planned phase-current stall fault is a genuine improvement over the
reference.** (And per §2, the "real" phantom current is likely ~3× our reading.)

---

## 8. The 24 V vs 16 V issue (reference-internal; porting caveat for us)

Reference hard-codes `DC_LINK_VOLTAGE 24.0`. Effects if run at 16 V:
- `MAXIMUM_SPEED_RPM = 24·KV = 16,800 rpm` — unreachable at 16 V → permanent speed
  error at high throttle → speed PI pins current demand → duty saturates regardless
  of load.
- `maxVoltage = 0.95·24 = 22.8 V` clamp vs `voltageRaw = out/16` → duty hits 100%
  while the current-PI output is only ~16 V (well under its clamp) → integrator
  keeps winding with no actuator headroom.
- The 65%-blanking-off threshold is referenced to the 24 V MaxSpeed → engages at a
  higher fraction of our *actual* attainable speed.

For us this is a "don't copy their constant" note: any borrowed formula uses 16 V
and a real attainable MaxSpeed.

---

## 9. Side-by-side: reference vs our `dspic33AKESC`

| Aspect | Reference (Atomberg highSpeed) | Ours (dspic33AKESC ak512-port) |
|---|---|---|
| Board / MCU | MCLV-48V-300W / AK512MC510 | **same** |
| Shunt | 3 mΩ | 3 mΩ (**same**) |
| **Op-amp gain** | **8.3 → 66.265 A peak** | **24.95 → 22.06 A peak** ⚠ suspect |
| Voltage scale | 18.53 mV/count | ~18.6 mV/count (**match**) |
| PWM | 40 kHz, edge-aligned, independent, override | 6-step override (verify freq) |
| Bus voltage assumed | 24 V (hard-coded) | 16 V (real) |
| Speed control | **outer speed PI → inner current PI → duty** | throttle→duty (`FEATURE_SPEED_PI` OFF) |
| Current feedback | bus only | bus only (phase compiled out) |
| ZC method | neutral + sign decode, PWM-ON, ADC | freewheel-gated HW comparator (+ falling SW) |
| High-speed ZC | **variable trigger 0.6/0.8/0.9× duty + multi-sample** | fixed sample (none) |
| Demag/load blanking | **none** (speed-only, off >65%) | none yet (the gap to fill) |
| Timing advance | **none** (fixed 30°) | per-profile `timingAdvMaxDeg` (ahead) |
| Faults | bus OC 53 A latch only | UV + OC + (chop infra) |
| Stall protection | soft speed decay, no trip | none yet (planned phase fault) |
| Phantom-stall visibility | **blind** | **blind** (both) |

---

## 10. Borrow list / recommended order

1. **FIRST: resolve the gain (§2).** Bench-measure or schematic-check the DIM
   current-amp gain. If 8.3, set `OC_GAIN_X100=830` and re-base all current
   thresholds. Everything below depends on knowing real amps.
2. **Load-adaptive demag blanking (§6.4)** — the actual desync root cause; absent
   in *both* firmwares; our novel piece. Scale `zcDemagBlankExtra` by measured
   current. Bench-verify. *Do this before the speed loop* — otherwise holding speed
   just pushes current and triggers desync sooner.
3. **Enable + tune the speed→current→duty cascade (§5)** with the speed-PI output
   clamped to a current ceiling **below** the ZC current limit from step 2. This is
   "push current, hold speed" done safely. Borrow `pi.c` AW + the auto-gain formulas
   (re-based to 16 V and correct amps).
4. **Variable trigger (§6.3)** — duty-adaptive BEMF sample point + multi-sampling
   for the high-speed ceiling. Independent of the load fix; can land anytime.
5. **Phase-current stall fault (§7)** — backstop the phantom lock that *neither*
   firmware catches; doubles as the speed loop's hard current limit.

> Honest caveat (unchanged): 6-step sensorless has a hard low-speed/high-torque
> ceiling. Steps 2–5 raise and protect it; they don't eliminate it. The reference
> confirms there is no magic in the Microchip design we're missing — its only edge
> over us is the variable trigger; its gaps (no load blanking, no advance, bus-only
> protection) are exactly where our work already aims.

---

## Appendix — key file:line index

- Board scaling: `mc1_user_params.h:154-160`; ours `garuda_config.h:2003-2006`,
  `hal/hal_adc.c:263`, `hal/port_config.c:368`.
- Derived constants: `mc1_calc_params.h` (all formulas).
- PI: `control/pi.c:59-86`. Cascade: `control/trapezoidal_control.c:465-543`.
- Gains: `mc1_init.c:162-211`. Clamps: `:191,196,202-203`.
- Startup FSM: `trapezoidal_control.c:240-572`; handoff `:624-695`.
- ZC decode: `estim_bemf.c:118-143`; blanking `:124` + `mc1_calc_params.h:124-131`
  + `estim_bemf.h:149-160`; interval/30° `:182-204`; gates
  `trapezoidal_control.c:866-892`.
- Variable trigger: `control/variable_trigger.h:24-119`; ISRs `mc1_service.c:283-316`.
- PWM/commutation: `pwm.c` (config), `trapezoidal_control.c:51-53,866-945`,
  `board_service.h:112-116`.
- ADC: `adc.c:62-190`. Op-amps: `port_config.c:281-413`. CMP3: `cmp.c`.
- Faults: `fault.c:79-96`; limit `mc1_init.c:145-146`.
- Throttle/ISRs/duty: `mc1_service.c:236-316,392-410`.
