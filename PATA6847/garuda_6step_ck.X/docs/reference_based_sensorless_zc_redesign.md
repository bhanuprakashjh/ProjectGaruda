# Reference-Based Sensorless ZC Redesign

## Purpose

This document defines a replacement sensorless zero-crossing architecture for
`garuda_6step_ck.X` based on patterns observed in the local reference firmware
trees:

- `../../AM32`
- `../../ESCape32`
- `../../bluejay`
- `../../BLHeli`

The intent is not to add another patch on top of the current detector. The
intent is to replace the current "single detector plus permissive recovery"
model with a stateful design that follows the control patterns used by the
reference ESCs.

This document is driven by:

- the current Garuda implementation in `motor/bemf_zc.c`
- the 2810 bench investigation in `2810_zc_investigation_session_20260326.md`
- the captured telemetry CSVs referenced in that investigation

## Problem Statement

The 2810 path has improved substantially after:

- HR seeding at CL entry
- reduced high-speed blanking
- larger ATA6847 edge blanking
- added ZC diagnostics

However, the remaining failure mode is still structural:

1. `forcedSteps` is overloaded as a telemetry signal and does not mean "forced
   commutation happened this step".
2. Closed-loop entry can begin with bypass already eligible.
3. The current recovery path can drop polarity discrimination.
4. A false ZC near half-step can still enter the interval estimator.
5. Once the estimator shrinks, the next schedule/timeout gets tighter and the
   failure becomes self-reinforcing.

The reference firmware families do not solve this by tuning a better blanking
curve. They solve it by:

- using explicit ZC scan windows
- enforcing polarity in normal running
- limiting estimator movement
- switching recovery mode instead of accepting weaker ZC evidence

## Research Summary

### Garuda Current Behavior

Current Garuda `FEATURE_IC_ZC` behavior:

- Layer 1 blanking after commutation in `motor/bemf_zc.c`
- half-interval rejection in `BEMF_ZC_FastPoll()`
- half-interval rejection again in `RecordZcTiming()`
- 3:1 IIR for `stepPeriod` and `stepPeriodHR`
- optional adaptive blanking for the 2810 profile
- permissive bypass when `stepsSinceLastZc >= 2`

Strengths:

- 200 kHz fast poll is high resolution
- SCCP4 HR timer gives a strong time base
- interval-based rejection is directionally correct
- board-specific ATA6847 tuning is already valuable

Weaknesses:

- recovery is permissive instead of conservative
- detector mode is implicit instead of explicit
- interval estimator is still vulnerable to one legal-but-wrong short ZC
- timing windows are expressed as "blanking + half interval" rather than a
  structured scan model

### AM32

Local source: `../../AM32/Src/main.c`

Relevant patterns:

- strict rising/falling polarity tracking by step
- hard reject before half of the current interval
- separate normal comparator-interrupt mode and polling fallback mode
- separate rising/falling BEMF count thresholds in polling mode
- duty/speed governance in the main control loop

Important observations:

- AM32 does not have a Garuda-style "accept any polarity after N misses"
  mechanism.
- AM32 keeps recovery conservative: when it cannot trust the fast path, it
  falls back to polling logic rather than lowering acceptance criteria.
- AM32 changelog explicitly mentions reducing maximum allowed interval change
  and adding early-zero-cross protection.

### ESCape32

Local source: `../../ESCape32/src/main.c`

Relevant patterns:

- reject any ZC before half-interval
- update interval estimate conservatively
- reset sync when interval movement becomes too large

Important observation:

- ESCape32 treats the estimator as protected state. It does not allow an
  obviously suspicious ZC to re-teach the running interval freely.

### BLHeli / Bluejay

Local sources:

- `../../bluejay/src/Bluejay.asm`
- `../../bluejay/src/Modules/Timing.asm`
- `../../BLHeli/Atmel/BLHeli.asm`

Relevant patterns:

- explicit staged timing:
  - wait after commutation
  - wait before ZC scan
  - bounded scan timeout
  - wait from ZC to commutation
- explicit demag detection state/metric
- do not accept even "correct" comparator output while demag is still active
- startup and initial-run phases have wider scan windows and longer timeouts

Important observation:

- BLHeli / Bluejay do not model the entire detector as a single blanking
  percentage. They model the scan as a windowed state machine.

## Reference Comparison Matrix

| Topic | Garuda Current | AM32 | ESCape32 | BLHeli / Bluejay | Redesign Direction |
|---|---|---|---|---|---|
| Normal ZC acceptance | Fast poll + HR timestamp + software filter | Comparator interrupt with strict expected polarity | Capture/ISR with strict interval gate | Windowed scan with explicit comparator state checks | Keep fast poll, keep strict polarity |
| Early-ZC rejection | Half-interval checks in poll and record paths | Reject before `interval / 2` | Reject before `ival >> 1` | Do not scan yet, then reject until scan phase | Keep half-interval rule and make scan window explicit |
| Recovery strategy | Historically permissive bypass after misses | Fall back to polling mode | Lose sync and re-acquire | Extend timeout / re-enter staged scan | Recover by mode change, not weaker evidence |
| Demag handling | Implied via blanking and filter | Implicit, helped by strict timing and fallback | Mostly implicit | Explicit demag flag / timeout extension / do-not-accept | Add explicit demag classification |
| Estimator update | IIR plus current guards, but still tied closely to acceptance path | IIR with bounded movement | Conservative IIR | Timing derived from staged waits | Split raw interval from protected reference interval |
| Startup / handoff | Same detector with extra seeding and blanking | Conservative startup and fallback | Sync before trusting interval | Wider windows and startup timeout extension | Add explicit acquire mode |
| Acceleration control | Mostly throttle-to-duty | Speed and duty governance | Conservative sync loss behavior | Timing-state-based survival | Add speed governance later |

## Current Garuda Gaps To Close

Compared with the reference firmware families, the remaining gaps are:

- detector modes are implicit rather than explicit
- recovery still depends too much on relaxed acceptance rules
- telemetry overloads internal state (`stepsSinceLastZc`) as an operator-facing
  forced-comm indicator
- estimator protection exists, but still sits too close to the acceptance path
- demag is inferred indirectly instead of tracked explicitly
- scheduling is still more aggressive than the reference firmware families

## Shared Design Pattern Across Reference ESCs

The shared pattern is:

1. Do not look for ZC immediately after commutation.
2. Do not accept any ZC before half of the expected interval.
3. Keep polarity strict in normal running.
4. Protect the estimator from one bad interval.
5. Recover by changing detector mode, not by accepting weaker evidence.
6. Use acceleration governance so the motor is not pushed straight into the
   unstable speed region.

This should become the new Garuda design center.

## Design Goals

1. Eliminate permissive any-polarity recovery from normal closed-loop running.
2. Prevent one false ZC from shrinking `stepPeriodHR` by 2x.
3. Make closed-loop entry explicit and conservative.
4. Preserve the strengths of the current implementation:
   - 200 kHz polling
   - SCCP4 HR timing
   - ATA6847 hardware edge blanking
5. Keep the A2212/Hurst path stable while allowing 2810-specific tuning.

## Non-Goals

This redesign does not include:

- ADC-based virtual-neutral ZC in this phase
- phase-current-based demag estimation in this phase
- FOC migration
- ATA6847 hardware changes beyond the existing register tuning

## Proposed Architecture

### Overview

Replace the current implicit detector with an explicit mode-based controller:

- `ZC_ACQUIRE`
- `ZC_TRACK`
- `ZC_RECOVER`

These modes live inside the ZC module and are independent from the top-level
ESC state machine (`ESC_OL_RAMP`, `ESC_CLOSED_LOOP`, etc.).

### State Transition Summary

The mode transitions should be explicit:

| From | Condition | To | Reason |
|---|---|---|---|
| `ZC_ACQUIRE` | `N_acquire_good` valid ZCs, no timeout streak, interval sanity OK | `ZC_TRACK` | Detector is trusted |
| `ZC_ACQUIRE` | timeout streak, demag burst, polarity imbalance | `ZC_RECOVER` | Startup/handoff could not stabilize |
| `ZC_TRACK` | single timeout, suspicious interval streak, repeated wrong-polarity evidence | `ZC_RECOVER` | Preserve estimator before corruption spreads |
| `ZC_RECOVER` | `N_recover_good` valid ZCs with stable interval | `ZC_ACQUIRE` | Recovery succeeded, but trust is not yet full |
| `ZC_RECOVER` | repeated recovery failures | top-level desync / OL restart | Sensorless lock is lost |

This transition model is intentionally conservative. Normal tracking never
widens its evidence criteria. It either stays strict or drops into a slower,
more conservative mode.

### Mode 1: `ZC_ACQUIRE`

Purpose:

- handle OL->CL handoff
- handle resync after desync/recovery

Behavior:

- conservative detector
- no polarity bypass
- lower timing advance
- longer scan timeout
- larger acceptance history requirement
- separate rising/falling quality counters

Entry:

- OL->CL transition
- reset from desync
- repeated suspicious ZC intervals

Exit:

- after `N_acquire_good` valid ZCs with sane intervals and no timeout streak

Reference inspiration:

- AM32 polling fallback
- BLHeli / Bluejay startup and initial-run phases

### Mode 2: `ZC_TRACK`

Purpose:

- normal high-speed closed-loop running

Behavior:

- 200 kHz fast poll remains primary
- HR timestamp remains primary time base
- strict polarity acceptance
- explicit scan window
- protected interval estimator
- standard timing advance

Entry:

- successful completion of `ZC_ACQUIRE`

Exit:

- timeout
- interval anomaly streak
- polarity imbalance event
- demag instability burst

Reference inspiration:

- AM32 interrupt mode
- ESCape32 half-interval reject

### Mode 3: `ZC_RECOVER`

Purpose:

- recover from missed ZCs or estimator corruption without reopening the
  acceptance rules

Behavior:

- lower advance than `ZC_TRACK`
- slower forced commutation if needed
- conservative polling and scan timing
- no any-polarity acceptance
- estimator frozen or only allowed to expand, not shrink aggressively

Entry:

- timeout in `ZC_TRACK`
- repeated suspicious short intervals
- repeated demag/noise classification

Exit:

- after `N_recover_good` valid ZCs, transition to `ZC_ACQUIRE`

Reference inspiration:

- AM32 `old_routine`
- BLHeli/Bluejay demag-driven recovery behavior

## Timing Model

The new detector should stop treating blanking percentage as the main
abstraction. Use an explicit scan window model instead.

### Terms

- `refIntervalHR`: protected reference interval used for scheduling
- `rawIntervalHR`: last accepted raw interval
- `t_switch_blank_hr`: absolute post-commutation blank for switching/ringing
- `t_scan_delay_hr`: time from commutation to start looking for ZC
- `t_scan_timeout_hr`: maximum allowed time from commutation to keep scanning
- `t_comm_delay_hr`: time from accepted ZC to scheduled commutation

### Rules

1. `t_switch_blank_hr` is absolute and hardware-related.
2. `t_scan_delay_hr` is the later of:
   - `t_switch_blank_hr`
   - a small fraction of `refIntervalHR`
3. No candidate can be accepted before `0.5 * refIntervalHR`.
4. `t_scan_timeout_hr` is based on expected interval and mode:
   - tight in `ZC_TRACK`
   - wider in `ZC_ACQUIRE`
   - widest in `ZC_RECOVER`
5. `t_comm_delay_hr = refIntervalHR / 2 - advance_hr`

This model matches BLHeli/Bluejay more closely than "blanking pct + filter".

### Per-Step Flow

Each electrical step should follow this sequence:

1. commutate
2. arm `t_switch_blank_hr`
3. after `t_switch_blank_hr`, enter pre-scan delay
4. when `t_scan_delay_hr` expires, open the ZC scan window
5. while scanning:
   - reject wrong polarity
   - reject candidates before `0.5 * refIntervalHR`
   - run deglitch counter
   - classify early instability as demag/noise
6. on valid ZC:
   - capture raw interval
   - validate against bounds
   - update protected reference interval
   - schedule next commutation from `t_comm_delay_hr`
7. on timeout:
   - count actual forced commutation
   - transition according to mode rules

This sequence is closer to BLHeli / Bluejay than the current Garuda "commutate,
blank, poll forever until timeout" structure.

## Acceptance Rules

In `ZC_TRACK`, a ZC is accepted only if all conditions are true:

1. current mode is armed for scan
2. current time is after `t_scan_delay_hr`
3. current time is before `t_scan_timeout_hr`
4. current time is after `0.5 * refIntervalHR`
5. comparator polarity matches the expected polarity for the current step
6. deglitch counter reaches the required threshold
7. comparator is not currently classified as demag/noise
8. resulting interval is within sanity bounds relative to `refIntervalHR`

There is no permissive any-polarity bypass in `ZC_TRACK`.

## Demag Handling

Demag should become an explicit classification, not an inferred side effect of
larger blanking.

### Demag Metric

Maintain a per-step / rolling metric:

- increment when comparator repeatedly disagrees in the early scan region
- increment when a correct comparator state appears while demag is still
  considered active
- decay when valid ZCs occur cleanly

### Use

- if demag metric is high, stay in `ZC_ACQUIRE` or `ZC_RECOVER`
- optionally extend scan timeout once
- optionally reduce advance temporarily
- log demag events for telemetry and tuning

This is directly aligned with BLHeli / Bluejay.

## Estimator Design

Current Garuda mixes detection, validation, and estimator update in one path.
The redesign should separate them.

### State

- `rawIntervalT1`
- `rawIntervalHR`
- `refIntervalT1`
- `refIntervalHR`
- `checkpointIntervalT1`
- `checkpointIntervalHR`

### Update Policy

On accepted ZC:

1. compute raw interval
2. reject if outside broad sanity bounds
3. clamp interval change relative to `refInterval`:
   - shrink limited to 25%
   - growth limited to a larger bound or unclamped in recover mode
4. update `refInterval` with a conservative IIR
5. update checkpoint only after enough good ZCs

Recommended bounds:

- in `ZC_TRACK`, clamp shrink to `25%` max per accepted interval
- in `ZC_TRACK`, clamp growth to `50%` max unless a recovery handoff is in
  progress
- in `ZC_RECOVER`, allow growth more freely than shrink
- checkpoint should update only after a good streak, not on every accepted ZC

### Scheduling Policy

Schedule from `refInterval`, not from the minimum of:

- IIR period
- last interval
- two-step average

Rationale:

- the minimum-of-recent rule is good for acceleration tracking, but it amplifies
  the damage of one false short interval.
- references prefer a protected central estimate, not the most aggressive one.

Acceleration support should come from the speed controller, not from an overly
eager ZC scheduler.

## Function-Level Migration Map

The redesign should map onto the current Garuda implementation as follows:

### `BEMF_ZC_OnCommutation()`

Current role:

- increments step counters
- computes blanking end
- arms the detector

New role:

- enter the per-step timing state machine
- compute `t_switch_blank_hr`, `t_scan_delay_hr`, `t_scan_timeout_hr`
- arm mode-specific detector behavior
- reset per-step demag / filter state

### `BEMF_ZC_FastPoll()`

Current role:

- software comparator polling
- half-interval rejection
- permissive bypass handling
- deglitch acceptance

New role:

- evaluate only within an explicit open scan window
- enforce strict polarity in `ZC_TRACK`
- apply mode-specific filter depth / acceptance policy
- classify demag/noise events instead of bypassing polarity

### `RecordZcTiming()`

Current role:

- validate interval
- update running interval estimates
- reset miss counters

New role:

- consume already-validated ZC events
- update `rawInterval*`
- clamp and update `refInterval*`
- update checkpoint only after good streaks
- record per-polarity statistics

### `BEMF_ZC_CheckTimeout()`

Current role:

- decrement forced countdown
- trigger forced commutation
- adjust period during unsynced operation

New role:

- own timeout-driven transitions into `ZC_RECOVER`
- count true forced commutations
- apply recover-mode period expansion
- trigger top-level desync if recovery fails repeatedly

### `BEMF_ZC_ScheduleCommutation()`

Current role:

- choose the next delay from recent interval estimates

New role:

- schedule from protected `refIntervalHR`
- apply mode-specific advance
- avoid minimum-of-recent acceleration shortcuts

## Polarity Handling

The new design should assume polarity asymmetry is possible and measure it
directly.

### Required Diagnostics

- rising accepted count
- falling accepted count
- rising timeout count
- falling timeout count
- rising false-ZC reject count
- falling false-ZC reject count
- per-polarity average ZC latency

### Optional Later Features

- per-polarity filter depth
- per-polarity advance trim
- per-polarity scan-delay trim

These are optional follow-ons, not part of the first implementation phase.

## Speed Governance

The reference firmware families do not rely only on the detector to survive.
They also moderate acceleration.

Garuda should add a simple speed governor:

- input maps to target eRPM
- duty is adjusted toward target eRPM
- acceleration is limited
- optionally apply a hard duty cap when `stepPeriod <= threshold`

This follows AM32's control philosophy and reduces entry into the `Tp=2/3`
instability cliff.

## Proposed Data Model

Add a dedicated ZC control structure in `garuda_types.h`:

- detector mode
- window timestamps
- demag metric
- per-polarity statistics
- raw/reference intervals
- anomaly counters
- actual forced-commutation count

Important:

- `stepsSinceLastZc` should remain internal timing state only.
- GSP should expose a true forced-comm counter, not reuse `stepsSinceLastZc`
  as the operator-facing "Frc" signal.

## File-Level Impact

### `motor/bemf_zc.c`

Primary redesign location:

- add `ZC_ACQUIRE`, `ZC_TRACK`, `ZC_RECOVER`
- replace bypass logic
- implement explicit scan windows
- split validation from estimator update
- add demag classification

### `motor/bemf_zc.h`

- expose mode-aware API if needed

### `garuda_types.h`

- add new ZC controller state
- add per-polarity diagnostics
- add true forced comm counter
- add protected interval state separate from raw interval

### `garuda_service.c`

- change OL->CL handoff to enter `ZC_ACQUIRE`
- remove any dependence on permissive bypass
- connect speed governor if implemented

### `garuda_config.h`

- add window timing parameters per profile
- add acquire/recover thresholds
- add speed governor thresholds
- add per-mode filter and timeout knobs

### `gsp/gsp_commands.h` and `gsp/gsp_snapshot.c`

- replace overloaded `forcedSteps` telemetry with:
  - `actualForcedCommCount`
  - `zcMode`
  - per-polarity counters if space allows

## Implementation Phases

### Phase 1: Semantics Cleanup

- stop exposing `stepsSinceLastZc` as "forced comm"
- add true forced-comm counter
- add per-polarity counters
- add ZC mode field to telemetry
- keep existing detector behavior otherwise unchanged

### Phase 2: Detector Modes

- add `ZC_ACQUIRE`, `ZC_TRACK`, `ZC_RECOVER`
- remove permissive bypass from `ZC_TRACK`
- route OL->CL handoff into `ZC_ACQUIRE`
- keep current blanking implementation temporarily

### Phase 3: Windowed Detector

- replace simple blanking-centric model with:
  - switch blank
  - scan start
  - scan timeout
  - comm delay
- add demag classification inside the scan window

### Phase 4: Protected Estimator

- split raw interval from reference interval
- clamp shrink to 25%
- checkpoint only after good-ZC streak
- schedule commutation only from protected reference interval

### Phase 5: Speed Governance

- add target-eRPM loop
- add duty acceleration limit by speed region

## Risks And Tradeoffs

1. This is a structural redesign, not a one-function tweak. It will touch the
   core ZC path and must be staged carefully.
2. The first implementation should preserve current board-level ATA6847 tuning
   and change only the detector logic.
3. Per-polarity asymmetry may turn out to be a measurement problem rather than
   a true comparator limitation; the design therefore measures polarity
   imbalance before adding polarity-specific trims.
4. Speed governance should not be used to hide detector bugs. It is a stabilizer
   after the detector is structurally sound, not a substitute for it.

## Validation Plan

### Bench Metrics

Primary success metrics:

- no 2x collapse of `stepPeriodHR`
- no bypass-like permissive acceptance path in `ZC_TRACK`
- `actualForcedCommCount` low in healthy runs
- 2810 stable through 45-60k eRPM acceleration band
- reduced current spikes at 90-100k eRPM
- no regression on A2212 and Hurst

### Required Telemetry

- `zcMode`
- `actualForcedCommCount`
- `zcTimeoutCount`
- `risingAccepted`
- `fallingAccepted`
- `risingTimeouts`
- `fallingTimeouts`
- `demagMetric`
- `zcLatencyPct` computed from the original step window, not the decremented
  countdown
- `refIntervalHR`
- `rawIntervalHR`

### Test Sequence

1. A2212 regression test at low and mid speed
2. 2810 no-load sweep through the historical 45-60k eRPM failure band
3. 2810 prop-loaded sweep through the same band
4. repeated OL->CL entry tests to validate `ZC_ACQUIRE`
5. deliberate disturbance tests to validate `ZC_RECOVER`
6. comparison of rising vs falling counters to confirm or reject polarity
   asymmetry

## Key Design Decisions

1. Recovery changes mode; it does not weaken polarity acceptance.
2. Blanking is a sub-component of a larger scan-window design.
3. The estimator is protected state and must not be rewritten aggressively by
   one candidate event.
4. Startup / acquisition is a distinct regime and should not share all timing
   rules with steady-state tracking.
5. Speed governance is part of the solution, not an optional extra.

## Recommended First Implementation Slice

If this redesign is implemented incrementally, the first slice should be:

1. telemetry semantics cleanup
2. add `ZC_ACQUIRE` / `ZC_TRACK` / `ZC_RECOVER`
3. remove bypass from `ZC_TRACK`
4. add protected estimator clamp

That provides most of the architectural benefit before introducing the full
windowed BLHeli-style scan model.

## File-By-File Implementation Checklist

This checklist is the concrete execution plan for the redesign. The order is
intentional: keep the firmware compiling and bench-testable after each phase.

### `garuda_types.h`

Phase 1:

- add `ZC_MODE_T` with `ZC_ACQUIRE`, `ZC_TRACK`, `ZC_RECOVER`
- keep the existing `TIMING_STATE_T`, `IC_ZC_STATE_T`, and `ZC_DIAG_T`
  structures, but extend them rather than replacing them
- add a true forced-comm counter (`actualForcedCommCount`)
- add a true timeout counter (`zcTimeoutCount`)
- add per-polarity diagnostic counters:
  - accepted rising
  - accepted falling
  - timeout rising
  - timeout falling
  - reject rising
  - reject falling

Phase 2:

- add `zcMode` to runtime state
- add `modeGoodCount` / `modeAnomalyCount` or equivalent mode-transition
  counters

Phase 3:

- extend `IC_ZC_PHASE_T` from:
  - `IC_ZC_BLANKING`
  - `IC_ZC_ARMED`
  - `IC_ZC_DONE`
- to:
  - `IC_ZC_SWITCH_BLANK`
  - `IC_ZC_SCAN_DELAY`
  - `IC_ZC_SCANNING`
  - `IC_ZC_DONE`
- add explicit scan-window timestamps:
  - `scanStartHR`
  - `scanTimeoutHR`
  - `originalTimeoutHR`

Phase 4:

- add protected estimator state:
  - `rawIntervalT1`
  - `rawIntervalHR`
  - `refIntervalT1`
  - `refIntervalHR`
  - `checkpointIntervalT1`
  - `checkpointIntervalHR`

Phase 5:

- add `demagMetric`
- add `demagEventCount`

Important:

- keep `stepPeriod`, `stepPeriodHR`, `stepsSinceLastZc`, and
  `bypassSuppressed` until the new detector is proven
- do not delete legacy fields until telemetry and scheduling have moved to the
  new state

### `motor/bemf_zc.h`

Phase 2:

- add a closed-loop entry helper prototype, preferably
  `BEMF_ZC_OnClosedLoopEntry()`
- add a reset/recover entry helper prototype if it keeps `garuda_service.c`
  simpler

Notes:

- if the mode helpers remain `static` in `bemf_zc.c`, this header may only need
  the single CL-entry API
- avoid exporting many small helpers; keep the mode machine internal to the ZC
  module

### `motor/bemf_zc.c`

Phase 1:

- update `BEMF_ZC_Init()` to initialize:
  - `zcMode`
  - true forced-comm counters
  - per-polarity counters
  - any new raw/reference interval state
- keep existing blanking and scheduling behavior unchanged in this phase

Phase 2:

- add internal helpers:
  - `ZcEnterAcquire()`
  - `ZcEnterTrack()`
  - `ZcEnterRecover()`
- wire OL->CL entry into `ZC_ACQUIRE`
- remove any normal-running dependence on permissive bypass
- in `BEMF_ZC_FastPoll()`, keep strict polarity whenever `zcMode` is
  `ZC_TRACK`

Phase 3:

- refactor `BEMF_ZC_OnCommutation()` to compute:
  - `t_switch_blank_hr`
  - `t_scan_delay_hr`
  - `t_scan_timeout_hr`
  - mode-specific filter level
- keep the current adaptive blanking logic only as a temporary implementation
  of `t_scan_delay_hr`
- change `BEMF_ZC_FastPoll()` to operate on:
  - switch blank
  - scan delay
  - scan window open
  - scan timeout
- stop treating "blanking end" as the entire detector model

Phase 4:

- split `RecordZcTiming()` into three concerns:
  - candidate validation
  - raw interval capture
  - protected reference-interval update
- clamp shrink in `ZC_TRACK` to 25%
- clamp growth in `ZC_TRACK` to a larger bound
- allow looser growth only in `ZC_RECOVER`
- update checkpoints only after a good streak
- change `BEMF_ZC_ScheduleCommutation()` to schedule from `refInterval*`,
  not the minimum of recent intervals

Phase 5:

- add demag classification in the scan path
- if the comparator is unstable or wrong-polarity in the early scan region,
  increment `demagMetric` instead of trying to reinterpret it as a ZC
- use demag events to hold the detector in `ZC_ACQUIRE` or force
  `ZC_RECOVER`

Phase 6:

- rewrite `BEMF_ZC_CheckTimeout()` to:
  - increment `actualForcedCommCount`
  - increment `zcTimeoutCount`
  - transition mode on timeout
  - apply recover-mode period expansion
  - escalate to desync only after repeated recovery failure

Keep in mind:

- `BEMF_ZC_Poll()` can stay as the OL-ramp and backup path initially
- only align `BEMF_ZC_Poll()` with the new mode semantics after the fast path
  is stable

### `garuda_service.c`

Phase 2:

- replace the direct CL-entry detector setup with one ZC-module entry point
- on OL->CL transition:
  - keep the current HR seeding
  - keep `stepsSinceLastZc = 0`
  - keep bypass suppression through the first trusted CL ZC
  - enter `ZC_ACQUIRE`, not `ZC_TRACK`

Phase 3:

- stop manually manipulating detector-recovery details in the service layer
- let `bemf_zc.c` own:
  - detector mode transitions
  - timeout recovery state
  - trusted-ZC promotion into track mode

Phase 6:

- when top-level desync/restart happens, call back into the ZC module so the
  detector state resets coherently

Phase 7:

- add the speed governor here, not in the ZC module
- map input to target eRPM
- rate-limit duty or apply PI against target eRPM
- keep this change separate from the detector rewrite

### `garuda_config.h`

Phase 1:

- keep the current 2810 adaptive blanking knobs in place
- do not delete working knobs before the explicit scan-window model is proven

Phase 2:

- add mode-transition knobs:
  - `ZC_ACQUIRE_GOOD_ZC`
  - `ZC_RECOVER_GOOD_ZC`
  - `ZC_RECOVER_FAIL_LIMIT`

Phase 3:

- add explicit scan-window knobs:
  - `ZC_SWITCH_BLANK_US`
  - `ZC_SCAN_DELAY_TRACK_PCT`
  - `ZC_SCAN_DELAY_ACQUIRE_PCT`
  - `ZC_SCAN_TIMEOUT_TRACK_MULT`
  - `ZC_SCAN_TIMEOUT_ACQUIRE_MULT`
  - `ZC_SCAN_TIMEOUT_RECOVER_MULT`

Phase 4:

- add estimator protection knobs:
  - `ZC_REF_SHRINK_LIMIT_PCT`
  - `ZC_REF_GROW_LIMIT_TRACK_PCT`
  - `ZC_REF_GROW_LIMIT_RECOVER_PCT`

Phase 5:

- add demag knobs:
  - `ZC_DEMAG_INC`
  - `ZC_DEMAG_DEC`
  - `ZC_DEMAG_RECOVER_THRESH`

Phase 7:

- add speed-governor knobs:
  - `CL_TARGET_ERPM_ENABLE`
  - `CL_ERPM_ACCEL_LIMIT`
  - `CL_TP3_DUTY_CAP`

Configuration rule:

- keep profile-specific tuning in config
- keep detector code free of profile-specific `#if MOTOR_PROFILE` branches

### `gsp/gsp_commands.h`

Phase 1:

- append new fields rather than reordering existing ones
- keep the old `forcedSteps` field temporarily, but mark it as legacy
- add:
  - `zcMode`
  - `actualForcedCommCount`
  - `zcTimeoutCount`
  - `demagMetric`
  - `refIntervalHR`
  - `rawIntervalHR`

Phase 2:

- if packet size changes, update the size comment and all consumers together
- if compatibility is a concern, keep legacy fields until the GUI and scripts
  are updated

### `gsp/gsp_snapshot.c`

Phase 1:

- populate the new telemetry fields from the new runtime state
- keep populating legacy `forcedSteps` until all tooling migrates
- stop using `stepsSinceLastZc` as the only operator-facing forced-comm signal

Phase 3:

- compute `zcLatencyPct` from the original scan window
- do not derive it from the decremented `forcedCountdown`

### External Telemetry Consumers

If the snapshot layout changes, update all consumers in the same phase:

- GUI decode/types
- Python capture scripts
- any CSV tooling or offline analysis scripts

Rules:

- append fields, do not reorder
- keep legacy names until every consumer is updated
- bump any expected payload-length checks together

## Recommended Commit Sequence

1. `garuda_types.h` + `gsp/*` telemetry semantics only
2. `bemf_zc.c` / `bemf_zc.h` mode shell, no timing rewrite yet
3. `garuda_service.c` CL-entry handoff into `ZC_ACQUIRE`
4. `bemf_zc.c` remove permissive bypass from `ZC_TRACK`
5. `bemf_zc.c` protected estimator + scheduler from `refInterval`
6. `bemf_zc.c` explicit scan-window timing
7. `bemf_zc.c` demag metric + recover-mode timeout behavior
8. `garuda_service.c` speed governor

Each commit should bench-test cleanly before the next one.
