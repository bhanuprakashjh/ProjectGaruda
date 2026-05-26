# Plan: CLC Hardware Blanking for IOC BEMF

> **Goal**: route BEMF through a Configurable Logic Cell that filters
> out PWM switching transients in hardware, then drive IOC from the
> clean output. Should let IOC match or exceed PTG (225 k eRPM target).

## Why this is needed

Bench data (2026-05-21) showed software-only IOC filtering hits a hard
ceiling at ~83 k eRPM. Comparator output rings on every PWM switching
event. The ISR sees ~50 k fires/sec at 60 kHz PWM, most rejected by
software gates but enough false-positives slip through near the demag
boundary that the PI eventually loses sync.

The fix every other ESC firmware uses (AVR-DX, AM32, ST) is **hardware
blanking** that masks the comparator signal during the switching window
before it reaches the interrupt source. dsPIC33AK has 4 CLCs — enough
to do this.

## Two topologies

### Topology A: Sample-and-hold (Phase 1, easier)

```
       ┌────────────┐
BEMF ──│ D          │
       │          Q ├──→ CN pin (IOC interrupt)
PG1TRIGB│        ̄  │
("mid-OFF") ─→ CLK│
       └────────────┘
       CLC = D flip-flop
```

CLC latches BEMF only at PWM mid-OFF (same quiet moment PTG samples).
IOC fires only when the SAMPLED value changes — i.e., a real ZC.
Equivalent precision to PTG (1 PWM cycle resolution) but with edge-
interrupt efficiency (no per-PWM-cycle ISR work).

**Expected outcome**: matches PTG (~225 k eRPM), saves CPU vs PTG.

### Topology B: Switching-noise gate (Phase 2, proper fix)

```
                   ┌──────────────┐
BEMF ─────────────→│              │
                   │   A AND ¬B   ├──→ CN pin
switching_window ─→│              │
                   └──────────────┘

switching_window = high during ~1-2 µs around each PWM edge,
                   low otherwise. Driven by SCCP one-shot
                   triggered from PWM compare match.
```

Lets BEMF edges through during the long "quiet" portion of the PWM
cycle, masks only the brief switching transients. Allows **sub-PWM-
cycle ZC detection** — should beat PTG meaningfully.

**Expected outcome**: > 225 k eRPM, smoother CL at all speeds.

Build Phase A first to validate the CLC+IOC chain, then add the
switching-window signal for Phase B.

## Hardware resources

### CLCs (dsPIC33AK128MC106 has 4)

| CLC | Use |
|---|---|
| CLC1 | BEMF_A gate (Phase 1) → BEMF_A_clean |
| CLC2 | BEMF_B gate → BEMF_B_clean |
| CLC3 | BEMF_C gate → BEMF_C_clean |
| CLC4 | Reserved (could host the switching-window signal in Phase 2) |

Each CLC has 4 data inputs (`CLCxSEL`), a gate-logic structure
(`CLCxGLS`), and a mode selector (`CLCxCONbits.MODE`).

Modes relevant to us (per DS70005539 §15):
- `0b0001` D flip-flop with R (Phase 1 — D=BEMF, CLK=PG1TRIGB)
- `0b0010` JK flip-flop with R
- `0b0000` AND-OR (Phase 2 — A1·A2·…·B (AND of BEMF and NOT-switching))

### PWM trigger source

Already wired: PG1TRIGB fires at PWM mid-OFF for the existing PTG
sampler (`hal_pwm.c:148`). Same signal can clock the CLCs — we just
add CLC as another consumer.

For Phase 2 we'd add PG1TRIGC at PWM mid-ON, then OR the two via CLC4
into a "switching-window" signal extended by a SCCP one-shot.

### Pin / PPS budget

CLC outputs can be routed to any RPn pin via PPS. We need 3 free GPIOs
to receive the clean BEMF signals. Cheapest plan:

- Re-use the existing BEMF GPIO pins themselves? No — they're inputs
  from the ATA6847L. Can't drive them as outputs.
- Allocate 3 unused RPn pins as CLC outputs, configure CN on those new
  pins. The original BEMF pins still feed CLC inputs (input mode); the
  CLC outputs drive the new CN-equipped pins.

**Action item**: identify 3 free RPn pins on the EV43F54A board
schematic. Likely on the DIM header but not used by current firmware.
Could use:
- RA0/RA1 (no current use, RP0/RP1)
- RA3/RA4 (no current use)
- Spare pins on the DIM

For Phase 1 we can use TEST pins first (any breakout-accessible RPn)
and confirm the chain works before optimizing.

### CN-capable pins for the CLC output

Any GPIO on Port A/B/C/D supports CN. The destination pin doesn't have
to be on the same port as the BEMF input — we get to choose. To keep
the existing IOC ISR code working, route all three CLC outputs to one
port (e.g., Port C) for single-vector handling.

## Implementation plan

### Phase 1 — sample-and-hold (~1 day)

1. **`hal/hal_clc.{c,h}`** (new) — init CLC1/2/3 as D-FF, D=BEMF input,
   CLK=PG1TRIGB. Route output to 3 chosen RPn pins.
2. **Modify `hal/hal_ioc.c`** — when `FEATURE_IOC_CLC=1`, arm CN on
   the CLC output pins instead of the raw BEMF pins. Per-sector arming
   logic unchanged.
3. **Add `FEATURE_IOC_CLC` flag** in `garuda_config.h`. Mutually-
   exclusive with `FEATURE_IOC_BEMF=1 && FEATURE_IOC_CLC=0` (raw IOC)
   and `FEATURE_IOC_BEMF=0` (PTG path stays untouched).
4. **Bench test**: confirm CN ISRs fire at PTG-like rate (one per real
   ZC, not 50k/s). Confirm motor reaches PTG-equivalent peak speed.

### Phase 2 — switching-noise gate (~2 days)

5. Add SCCP one-shot for PWM switching-window pulse.
6. Reconfigure CLC4 to OR the two trigger sources + extend via SCCP.
7. Reconfigure CLC1/2/3 to AND-NOT mode: BEMF AND NOT switching_window.
8. Bench test for >225 k eRPM peak.

### Risk register

| Risk | Likelihood | Mitigation |
|---|---|---|
| CLC outputs can't reach CN-capable pins via PPS | low | dsPIC33AK PPS is flexible; verify in §11 of datasheet |
| 3 free RPn pins not available on EV43F54A | medium | Check schematic; may need DIM rework or use TEST pads |
| CLC propagation delay too high | very low | < 1 ns typical for combinational, plenty fast |
| D-FF approach catches false ZC if BEMF latches mid-transition | medium | Same risk as PTG today (works at 225 k there); blanking already at 25% |

## Open questions (need answers before coding)

1. **Which 3 RPn pins are physically free?** Need a schematic check.
2. **Does PG1TRIGB drive CLC input via internal mux, or do we need a
   pin loopback?** Check `CLCxSEL` source list.
3. **Can CLC outputs be input to CCP IC for hardware-timestamped
   capture** (a bonus over CN)?

These are the next session's first three tasks before writing code.
