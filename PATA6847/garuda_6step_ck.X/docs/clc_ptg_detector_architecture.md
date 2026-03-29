# CLC+SCCP+PTG Detector Architecture Research

## Context

Step 2 (A=LOW, B=FLOAT, C=PWM) accounts for 91% of timeouts at high speed.
Root cause: asynchronous 200kHz poller reads the comparator at random PWM
phase positions — step 2's specific drive vector creates noise that aliases
with the polling. The fix is PWM-synchronous sampling.

## Clean-Sheet Architecture (V3 Target)

```
ATA6847 BEMF pins → CLC D-FF (samples at mid-PWM) → CLC output (clean)
                                                           ↓
                                               SCCP1/2/3 capture (timestamp edge)
                                                           ↓
                                               PTG sequences window:
                                               blank → arm → confirm → timeout
                                                           ↓
                                               SCCP4 schedules commutation
```

### Layer 1: CLC (Signal Conditioner)
- CLC1/2/3 in D-FF mode, clocked from PWM sample event
- Samples each comparator once per PWM cycle at mid-ON
- Output holds between cycles — no switching noise
- Zero CPU cost

### Layer 2: SCCP Capture (Edge Detector)
- SCCP1/2/3 capture edge transitions on CLC outputs
- Timestamps relative to commutation (not absolute)
- Provides `time_since_commutation` for blanking/rejection

### Layer 3: PTG (Window Sequencer)
- Hardware state machine controls detection window
- Sequence: wait T_blank → IRQ0 (arm capture) → wait → IRQ2 (timeout)
- Optional: 1-PWM-cycle confirmation in ACQUIRE/RECOVER

### Layer 4: SCCP4 (Commutation Scheduler)
- Unchanged from current design
- Absolute commutation deadline from refIntervalHR

### Per-Step Tuning
```c
uint16_t zc_sample_offset[6];    // PWM phase for CLC sample per step
uint16_t zc_blank_ticks[6];      // blanking per step
uint8_t  zc_confirm_cycles[6];   // optional PWM-cycle confirmation count
```

Step 2 can sample later in PWM ON-time than other steps.

## PWM Frequency Impact

CLC gives one new observation per PWM cycle. Higher PWM = more samples per step.

### Samples Per Step
| eRPM | 20 kHz | 40 kHz | 60 kHz |
|------|--------|--------|--------|
| 30k  | 6.7    | 13.3   | 20.0   |
| 60k  | 3.3    | 6.7    | 10.0   |
| 100k | 2.0    | 4.0    | 6.0    |

### Max ZC Angle Error (from sampling grid)
| eRPM | 20 kHz | 40 kHz | 60 kHz |
|------|--------|--------|--------|
| 30k  | 9°     | 4.5°   | 3°     |
| 60k  | 18°    | 9°     | 6°     |
| 100k | 30°    | 15°    | 10°    |

### Recommendation
- 20 kHz: usable for 30-60k eRPM (the step-2 problem zone)
- 40 kHz: needed for reliable 100k+ eRPM with CLC detector
- 60 kHz: ideal but increases switching losses

## PTG on dsPIC33CK64MP205

PTG inputs available:
- PWM1 ADC Trigger 2 (not raw PWM edge)
- SCCP4/5
- Comparator 1/2/3
- CLC1
- ADC Done
- INT2

PTG cannot see raw PWM edges. Use PWM1 ADC Trigger2 as cadence source.

## CLC vs PTG vs Raw Poll

| Aspect | Raw 200kHz Poll | CLC D-FF | PTG+ISR |
|--------|----------------|----------|---------|
| Signal quality | Noisy (PWM aliasing) | Clean (mid-PWM sample) | Clean (timed read) |
| CPU cost | ~2% | Zero | ISR overhead |
| Timing precision | 5µs (200kHz) | 50µs (20kHz PWM) | 50µs + ISR jitter |
| Samples/step @100k | 20 | 2 | 1-2 |
| Step-2 fix | No | Yes | Yes |
| Already coded | Yes | Yes (hal_clc.c) | No |

## Current Experiment: CLC Everywhere

Simple replacement: `ReadBEMFComparator()` reads CLC output instead of raw GPIO.
Keeps 200kHz FastPoll for timing. CLC just cleans the input signal.

At 30-60k eRPM (the problem zone): 3-7 CLC updates per step — sufficient.
At 100k eRPM: 2 CLC updates per step — marginal but better than current failure.

The ±50µs jitter concern is overblown: raw GPIO has the same jitter because the
real ZC can happen anywhere in the PWM cycle. CLC doesn't add jitter — it just
quantizes to the PWM grid, which the physics already does.
