# V3 Global ZC Architecture Research

## Purpose

This document captures the clean-sheet V3 zero-crossing detector architecture
research for `garuda_6step_ck.X`.

It is intentionally separate from the current ZC V2 work. The goal here is not
to patch the existing `SCCP1 fast poll + software filter` path. The goal is to
define a future detector architecture that uses the dsPIC33CK peripherals more
coherently:

- `CLC` for signal conditioning
- `SCCP capture` for clean edge detection
- `PTG` for deterministic window sequencing
- `SCCP4` for commutation scheduling

This document is based on:

- the current Garuda implementation
- the 2810 / A2212 bench captures and per-step analysis
- the local reference firmware trees (`AM32`, `ESCape32`, `BLHeli`, `Bluejay`)
- the local `dsPIC33CK256MP508` family datasheet

No runtime code changes are implied by this document.

## Executive Summary

The current Garuda ZC detector is:

- robust against raw ATA6847 chatter compared with old edge capture
- still vulnerable to step-vector-specific aliasing because it samples the
  comparator asynchronously
- strongest when the signal is already clean
- weakest when the floating phase only looks valid during a narrow part of the
  PWM cycle

The most important recent finding is that the failure is not generic
"rising/falling" behavior and not a permanently bad comparator pin. The bad
regime is dominated by a specific drive vector, especially step 2.

That points away from a simple polarity bug and toward a detector timing
problem:

- the ATA6847 BEMF signal is valid
- but the current detector is often looking at the wrong phase of the PWM cycle

The clean-sheet V3 answer is:

1. sample the comparator at a known, repeatable PWM phase
2. turn the sampled result into a clean digital edge
3. use hardware capture to detect that edge
4. use PTG to open and close the valid detection window
5. keep commutation scheduling on the existing high-resolution SCCP4 path

## Why V2 Is Not Enough For This

The current V2 detector works like this:

- ATA6847 digital comparator output on `RC6/RC7/RD10`
- `SCCP1` periodic timer interrupt at `ZC_POLL_FREQ_HZ`
- software reads the active floating phase as GPIO
- software applies:
  - blanking
  - half-interval rejection
  - consecutive-read filter
- accepted ZC is scheduled on `SCCP4`

This is a reasonable design for a noisy external comparator output, and it was
already better than the old edge-driven capture path. But it still has one
structural weakness:

- the detector reads the comparator at arbitrary points relative to PWM

That makes it sensitive to:

- switching ripple
- commutation settling
- vector-specific floating-phase behavior
- aliasing when the usable signal window is narrow

For the high-speed trouble region, the issue is no longer "how many microseconds
of blanking should we use?" The issue is "when are we observing the waveform?"

## Research Findings That Motivate V3

### 1. The failure is vector-specific

Per-step analysis showed that the bad regime is dominated by step 2 rather than
all rising steps, all falling steps, or all uses of one comparator pin.

That means the important variable is not only polarity. It is the whole
commutation vector:

- which phase is PWM-driven
- which phase is LOW
- which phase is floating
- when the floating phase is actually clean enough to observe

### 2. Raw edge-driven capture already failed once

The current `hal_ic.c` explicitly documents why the old input-capture approach
was removed:

- PWM-induced comparator bounce created many false edges
- capture ISRs were flooded
- software deglitch could not keep up

That matters because it rules out "just use the raw BEMF pins as interrupts"
unless the signal is conditioned first.

### 3. CLC is relevant because the problem is observation timing

The existing `hal_clc.h` / `hal_clc.c` path was designed to:

- sample the ATA6847 comparator at a fixed PWM phase
- hold the sampled value
- eliminate PWM-edge chatter from the observed signal

That is exactly the right direction for a vector-dependent aliasing problem.

### 4. PTG is useful as a sequencer, not as the detector

On this dsPIC33CK family, PTG can:

- wait on selected trigger inputs
- insert deterministic delays
- raise interrupts
- emit trigger outputs

But PTG is not a three-channel ZC detector by itself. Its best role here is to
sequence:

- blanking
- arming
- optional confirmation
- timeout

### 5. PWM-synchronous detection has a real resolution tradeoff

If the detector only gets one clean observation per PWM cycle, then its timing
is quantized to the PWM grid.

At high eRPM this becomes a first-order design constraint, not a detail.

## Relevant dsPIC33CK Hardware Facts

### Virtual PPS pins

The dsPIC33CK family provides internal virtual RP pins `RP176..RP181`
(`RPV0..RPV5`).

These can connect peripherals internally without consuming a physical package
pin. This is what makes `CLC output -> SCCP capture input` practical without a
board-level loopback.

### PTG inputs on this device

The local family datasheet shows PTG inputs including:

- `PWM1 ADC Trigger 2`
- `PWM2 ADC Trigger 2`
- `PWM3 ADC Trigger 2`
- `SCCP4`
- `SCCP5`
- `Comparator 1/2/3`
- `CLC1`
- `ADC Done Group Interrupt`
- `INT2 PPS`

The key implication is:

- PTG does not directly observe all three ATA6847 GPIO comparator outputs
- PTG is best used as a timing sequencer driven by PWM cadence and/or internal
  detector state

### SCCP capture role

`SCCP1..3` can be used in Input Capture mode with input selected through PPS.
That makes them good clean edge detectors when driven from `CLC1OUT..CLC3OUT`.

Important caveat:

- their capture time base is their own module time base
- they should be treated as relative capture timers for the current step
- `SCCP4` should remain the global absolute commutation scheduler

### PWM trigger role

The PWM module already provides internal compare events (`PGxTRIGA/B/C`) and
ADC trigger routing. That is the natural cadence source for both:

- the CLC sampling clock
- the PTG detection-window sequencer

## Proposed V3 Architecture

### High-level block diagram

```text
ATA6847 BEMF_A/B/C
        |
        v
CLC1/2/3 D-FFs
sampled at a controlled PWM phase
        |
        v
CLC1OUT/2OUT/3OUT -> RPV0/1/2 -> SCCP1/2/3 capture
        |
        +--> capture ISR / event handler
                |
                +--> Record ZC timing
                +--> update interval estimator
                +--> schedule next commutation on SCCP4

PTG runs in parallel as the detector window sequencer:
commutation -> blank -> arm -> optional confirm -> timeout
```

### Functional split

#### CLC

Job:

- clean and re-time the ATA6847 comparator output
- produce at most one stable digital state update per PWM cycle

CLC is not the detector decision engine. It is the signal conditioner.

#### SCCP1/2/3 capture

Job:

- detect transitions on the conditioned CLC outputs
- record the time of the transition relative to the current step

This replaces:

- high-rate polling of raw GPIO
- software "consecutive read" deglitch for the primary path

#### PTG

Job:

- sequence the detector window in hardware
- reduce CPU dependence on exact window timing

PTG should manage:

- blanking duration
- the instant at which the detector is armed
- optional confirmation timing
- timeout timing

PTG is not the ZC detector. PTG is the window controller.

#### SCCP4

Job:

- remain the one-shot absolute commutation timer
- schedule `ZC + delay` commutation as in the current firmware

This should stay unchanged conceptually.

## Detection Timeline

For one commutation step:

1. `SCCP4` commutation ISR fires.
2. CPU advances the commutation state.
3. CPU selects:
   - floating phase
   - expected ZC polarity
   - per-step sample offset
   - blanking and timeout values
4. CPU forces the active CLC D-FF to the expected pre-ZC state.
5. CPU resets and disarms capture state.
6. CPU starts a PTG sequence for the step.
7. PTG waits through `T_blank`.
8. PTG arms the active capture channel.
9. CLC updates at the programmed PWM sample phase each cycle.
10. On a valid CLC edge, the active SCCP capture channel records the event.
11. ISR validates the capture:
    - after blanking
    - after half-interval gate
    - optional confirm if in `ACQUIRE` / `RECOVER`
12. Accepted ZC is recorded.
13. CPU schedules the next commutation on `SCCP4`.
14. PTG timeout is canceled or ignored for this step.

## Detector Modes

The clean-sheet detector should still be mode-based:

- `ACQUIRE`
- `TRACK`
- `RECOVER`

The difference is that V3 changes the hardware front end as well as the mode
logic.

### ACQUIRE

Conservative mode:

- larger blanking
- optional one-PWM-cycle confirmation
- lower advance
- wider timeout

### TRACK

Normal running mode:

- shortest blanking compatible with stable operation
- no extra confirmation
- direct accept on first valid capture

### RECOVER

Conservative re-lock mode:

- similar detector policy to `ACQUIRE`
- widened timeout
- reduced advance
- explicit escape if re-lock fails repeatedly

## Per-Step Tuning

The V3 detector should be tuned per commutation step, not just globally by
motor profile.

The key table is:

- `zc_sample_offset[6]`

Optional supporting tables:

- `zc_blank_ticks[6]`
- `zc_confirm_cycles[6]`
- `zc_timeout_scale[6]`

This is important because the observed problem is vector-specific. A single
motor-wide blanking percentage cannot express "step 2 needs a later observation
point than steps 0 or 4".

## Why Raw BEMF Interrupt Is Not The V3 Answer

Using raw ATA6847 BEMF pins as GPIO/IC interrupts is attractive in theory, but
the current code history already shows why it is not the preferred path:

- the raw signal chatters under PWM conditions
- one real ZC can produce many spurious edges
- ISR load and false timestamps become the problem

If interrupts are used again, they should be used only after the signal is
conditioned by CLC or another synchronous front end.

## Why CLC-Only Is Not Enough

Using only `CLC + poll` is simpler, but it wastes what the hardware can do:

- CLC gives a clean edge
- polling turns that back into repeated software reads of a held state

That means:

- no hardware timestamp of the sampled edge
- software filter becomes mostly duplicate-read latency
- the CPU still spends time polling

If CLC is adopted, capture should be used with it.

## Why PTG Matters In V3

Without PTG, the CPU still owns all the sequencing details:

- when blanking ends
- when capture is armed
- when confirm happens
- when timeout expires

With PTG:

- the CPU sets up the step policy
- hardware executes the step timing deterministically

This is especially useful when:

- sample offset changes by step
- blanking changes by mode
- confirmation exists only in `ACQUIRE` / `RECOVER`

PTG is what turns this from "a clean edge detector" into "a structured detector
state machine with hardware timing support".

## PWM Frequency As A First-Order Design Parameter

Because the CLC updates once per PWM cycle, the PWM frequency directly sets the
detector sampling grid.

For a 6-step BLDC:

- step period = `10 / eRPM` seconds
- samples per step = `PWM_freq * step_period`

### Step period table

| eRPM | Step period |
|---|---:|
| 30k | 333.3 us |
| 60k | 166.7 us |
| 100k | 100.0 us |

### Samples per step

| eRPM | 20 kHz PWM | 40 kHz PWM | 60 kHz PWM |
|---|---:|---:|---:|
| 30k | 6.67 | 13.33 | 20.00 |
| 60k | 3.33 | 6.67 | 10.00 |
| 100k | 2.00 | 4.00 | 6.00 |

### Timing quantization from PWM-synchronous sampling

| PWM | PWM period | Max ZC delay | Avg ZC delay |
|---|---:|---:|---:|
| 20 kHz | 50.0 us | 50.0 us | 25.0 us |
| 40 kHz | 25.0 us | 25.0 us | 12.5 us |
| 60 kHz | 16.7 us | 16.7 us | 8.3 us |

### Equivalent electrical angle error

`angle error = timing error / step period * 60 degrees`

| eRPM | 20 kHz max / avg | 40 kHz max / avg | 60 kHz max / avg |
|---|---:|---:|---:|
| 30k | 9.0 deg / 4.5 deg | 4.5 deg / 2.25 deg | 3.0 deg / 1.5 deg |
| 60k | 18.0 deg / 9.0 deg | 9.0 deg / 4.5 deg | 6.0 deg / 3.0 deg |
| 100k | 30.0 deg / 15.0 deg | 15.0 deg / 7.5 deg | 10.0 deg / 5.0 deg |

## Practical Meaning Of The PWM Table

At `20 kHz`, a PWM-synchronous detector is likely good enough for the current
30k-50k eRPM problem region. It is much less convincing as the only detector at
100k+ eRPM.

That leads to two credible V3 directions:

### Option A: 20 kHz PWM, dual detector strategy

- use `CLC + SCCP + PTG` where the raw detector is weak
- keep a separate high-speed detector path for the extreme top end

### Option B: 40 kHz+ PWM, unified detector strategy

- raise PWM frequency
- let the CLC-based detector become the main detector across a wider speed
  range

Option B is architecturally cleaner, but requires power-stage revalidation.

## Recommended Clean-Sheet V3 Strategy

### Near-term V3 target

Design the detector around:

- `CLC + SCCP capture + PTG + SCCP4`

but allow two operational policies:

1. `robust mode`
   - confirmation enabled
   - used in `ACQUIRE` / `RECOVER`
2. `fast mode`
   - no confirmation
   - used in `TRACK`

### Suggested default operating assumptions

- keep `SCCP4` as the authoritative commutation timer
- use `SCCP1/2/3` only as relative edge detectors
- use PTG only for window sequencing, not as the detector
- tune sample offset per step
- plan for 40 kHz PWM if the architecture is intended to become globally
  primary

## How V3 Compares With AM32 And BLHeli / Bluejay

This V3 design is not a copy of either AM32 or BLHeli. It is closer to:

- AM32 in control philosophy
- BLHeli / Bluejay in detector-window structure
- but uses dsPIC33CK-specific hardware to implement those ideas more directly

### Comparison Summary

| Topic | AM32 | BLHeli / Bluejay | Proposed V3 |
|---|---|---|---|
| Primary detector | Comparator interrupt in normal running | Windowed scan of comparator state | `CLC -> SCCP capture` |
| Fallback / weak-signal mode | Software polling fallback (`old_routine`) | Startup and demag-aware staged scan | `ACQUIRE` / `RECOVER` with optional confirm |
| Blanking model | Early reject + timer blanking | Explicit staged waits and scan timeout | PTG-controlled blank / arm / confirm / timeout |
| Signal conditioning | Mostly comparator hardware plus timer blanking | Mostly timing discipline and demag logic | Hardware retiming in CLC |
| Estimator protection | Conservative interval update | Window-state driven timing | protected interval estimator from V2 carried forward |
| Per-polarity / per-direction handling | Separate up/down polling thresholds | Demag-aware state machine | per-step sample offset and per-mode policy |
| Hardware style | simple timers + comparator + ISR | simple MCU + tight timing state machine | heavier use of dsPIC peripherals |

### Compared with AM32

AM32's normal path is:

- commutate
- wait until half-interval has passed
- enable comparator interrupt
- accept only the expected polarity
- schedule next commutation from `interval / 2 - advance`

When AM32 does not trust that path, it drops to a polling fallback and uses
separate up/down thresholds.

The important similarities between AM32 and V3 are:

- strict polarity in normal tracking
- conservative recovery instead of permissive bypass
- explicit distinction between trusted and untrusted detector states
- interval protection so one bad event does not fully retrain the estimator

The important difference is implementation:

- AM32 does this mostly with simple timers, comparator interrupt, and software
  fallback
- V3 does it with:
  - `CLC` to clean the external ATA6847 comparator outputs
  - `SCCP capture` to timestamp conditioned edges
  - `PTG` to sequence the detector window

So V3 is best understood as:

- **AM32-style control policy implemented with stronger dsPIC hardware
  orchestration**

### Compared with BLHeli / Bluejay

BLHeli / Bluejay are conceptually closer to V3 on the detector side.

Those firmware families do not treat the detector as:

- one blanking percentage
- one filter constant
- one always-on acceptance loop

Instead they structure detection as a sequence:

1. commutate
2. wait through demag / post-commutation settling
3. open a ZC search window
4. reject until the waveform is valid
5. close the search window on timeout
6. delay from ZC to commutation

That is very close to what PTG enables in V3.

The key difference is again implementation style:

- BLHeli / Bluejay do the timing-state machine mostly in firmware
- V3 moves much of that sequencing into hardware:
  - CLC handles synchronous sampling
  - PTG handles the staged timing window
  - SCCP capture handles edge detection

So V3 is best understood as:

- **BLHeli / Bluejay-style windowed detector structure implemented with dsPIC
  hardware sequencing**

### What V3 Does Better Than AM32 / BLHeli On This Hardware

The proposed V3 architecture is specifically attractive because Garuda is not
using an internal MCU comparator for BEMF. It is using external ATA6847 digital
comparator outputs.

That matters because the external comparator outputs have already shown:

- PWM-related chatter on raw edge detection
- vector-specific observation problems
- sensitivity to exactly when the comparator is observed

`CLC -> SCCP capture` is a better fit for that signal source than trying to
imitate AM32 literally with raw comparator interrupts.

So for this board and chip combination, V3 has one major architectural
advantage:

- it can apply **hardware synchronous retiming** before detection

That is not something AM32 or classic BLHeli rely on in the same way.

### What AM32 / BLHeli Still Do Better By Default

AM32 and BLHeli avoid one problem that V3 must still manage carefully:

- PWM-synchronous detection trades signal cleanliness for timing quantization

If V3 stays at `20 kHz` PWM, it may be cleaner than the current detector but
still coarser than the ideal comparator-interrupt timing seen in AM32.

So relative to AM32 / BLHeli:

- V3 may be **more robust against waveform aliasing**
- but can be **worse in raw timing resolution** unless PWM frequency increases
  or a dual-detector strategy is used

### Practical Positioning

The cleanest way to position V3 is:

- **Compared with AM32**:
  - same conservative control philosophy
  - more hardware-heavy front end because the signal source is worse

- **Compared with BLHeli / Bluejay**:
  - same staged detector philosophy
  - more hardware-assisted execution of that staged window

In one sentence:

- **V3 is an AM32-like recovery policy plus a BLHeli-like scan-window model,
  implemented using dsPIC33CK hardware blocks that neither firmware family had
  in the same form.**

## Questions That Must Be Answered Before V3 Implementation

1. Can `PWM Event A` and the ADC trigger allocation be cleanly split so that
   one event becomes the CLC sample strobe and another remains available for
   ADC timing?
2. Does the best sample point differ only by step, or also materially by duty
   and speed?
3. Is one-cycle confirmation needed in `TRACK`, or only in `ACQUIRE` /
   `RECOVER`?
4. Is `20 kHz` PWM acceptable for the intended maximum eRPM, or is a PWM
   frequency increase required for V3 to become the main detector?
5. Are the `CLC -> RPV -> SCCP capture` routes fully supported in the exact
   dsPIC33CK64MP205 silicon and XC device headers used by this project?

## What This Means For Current Firmware Work

This document does not say the current V2 work should stop. It says:

- V2 should continue as the practical firmware path
- V3 should be treated as a different architecture with different constraints

In other words:

- do not force V3 ideas into V2 piecemeal without acknowledging the timing
  tradeoffs
- but also do not keep tuning the asynchronous raw poller forever if the real
  problem is fundamentally sample-phase dependent

## Bottom Line

The strongest V3 architecture for this hardware is:

- condition the external ATA6847 comparator outputs with `CLC`
- detect clean edges with `SCCP capture`
- sequence detector timing with `PTG`
- keep commutation scheduling on `SCCP4`

That is the most coherent use of the dsPIC33CK peripherals for a future global
ZC detector.

The main design tension is no longer "how much blanking?" It is:

- signal cleanliness versus timing resolution

`CLC + SCCP + PTG` solves the cleanliness problem very well.
Whether it also solves the timing problem globally depends primarily on PWM
frequency and whether V3 remains dual-path or becomes the single detector.
