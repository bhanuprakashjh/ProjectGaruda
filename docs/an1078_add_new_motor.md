# Adding a New Motor to AN1078 — Step-by-Step

End-to-end procedure for adding a new motor type to the AN1078 sensorless
FOC firmware.  Pre-2026-04-26, this required ~10 file edits.  After the
GUI live-tune work, most of the iteration is GUI-driven and only a final
"bake-in" recompile is needed to make defaults persist.

You'll need:
- `Rs` (phase-to-neutral resistance, Ω)
- `Ls` (phase-to-neutral inductance, H)
- `KV` (RPM/V) — datasheet, or measure with `GSP_CMD_AUTO_DETECT`
- Pole pair count (= rotor magnet count / 2)
- Nominal Vbus

If you only have line-to-line (phase-to-phase) values, divide by 2 for
phase-to-neutral.

---

## Step 1 — Pick a profile slot

Profile slots in `dspic33AKESC/foc/an1078_params.h` and `gsp_params.c`:

| Slot | Currently | Action |
|------|-----------|--------|
| 0 | Hurst | DON'T overwrite (used by other testing) |
| 1 | A2212 | DON'T overwrite |
| 2 | **2810** | DON'T overwrite (active 200k+ profile) |
| 3 | 5055 | OK to overwrite, OR add a new one |

If your motor is similar to one of the existing slots, you can just
overwrite the values in that slot.  For a totally new motor type, edit
the `5055` slot or add a new enum entry (more invasive).

This guide assumes you're overwriting **slot 3** for now.

---

## Step 2 — Update the firmware motor params

Edit `dspic33AKESC/foc/an1078_params.h`:

```c
#define AN_NOPOLESPAIRS    7              /* your motor's pole pairs */
#define AN_MOTOR_RS        0.022f         /* phase-to-neutral, Ω */
#define AN_MOTOR_LS        10e-6f         /* phase-to-neutral, H */
/* λ_pm = 60 / (√3 · 2π · KV · PP) — for 1350 KV, 7 PP: 0.000583 */
#define AN_MOTOR_LAMBDA    0.000583f      /* V·s/rad_elec */
#define AN_NOMINAL_SPEED_RPM_MECH  30000.0f  /* expected max speed */
```

These are still `#define`s today (FW reads `AN_MOTOR_RS` directly into
`F_PLANT`, not `gspParams.focRs`).  Live-update of these would need a
small additional firmware change (see "C-code changes still needed"
below).

Edit the per-profile defaults in `dspic33AKESC/gsp/gsp_params.c`
(the `[GSP_PROFILE_5055]` block):

```c
[GSP_PROFILE_5055] = {
    /* ... existing 6-step + FOC v3 fields ... */
    .focRsMilliOhm       = 22,       /* same as AN_MOTOR_RS × 1000 */
    .focLsMicroH         = 10,       /* same as AN_MOTOR_LS × 1e6 */
    .focKeUvSRad         = 583,      /* same as AN_MOTOR_LAMBDA × 1e6 */
    .focKpDqMilli        = 63,       /* current PI Kp = bw·Ls (in mΩ units) */
    .focKiDq             = 138,      /* current PI Ki = bw·Rs */
    /* AN1078 SMO defaults (the live-tunable ones, group 11) */
    .an1078ThetaBaseDegX10 = 200,    /* 20° at zero speed */
    .an1078ThetaKE7        = 1000,   /* 1.0e-4 rad/(rad/s elec) */
    .an1078KslideMv        = 2500,   /* 2.5 V — low-Rs motor */
    .an1078IdFwMaxDecia    = 0,      /* start with FW disabled */
    /* ... etc ... */
},
```

For the 4 AN1078 params, sensible starting values for an unknown motor:

| Motor type | THETA_BASE | THETA_K | Kslide | FW |
|-----------|-----------|---------|--------|-----|
| Low-Rs (drone, 10-50 mΩ) | 200 (20°) | 1000 | 2500 mV | 0 |
| Medium-Rs (RC car, 50-200 mΩ) | 100 (10°) | 500 | 4000 mV | 0 |
| High-Rs (Hurst, >300 mΩ) | 0 (0°) | 0 | 8000-20000 mV | 0 |

Field weakening starts at 0 — only enable after observer is rock solid.

---

## Step 3 — Update the GUI profile label

Edit `gui/src/protocol/types.ts`:

```ts
export const PROFILE_NAMES = [
  'Hurst Long (300W)',
  'A2212 1400KV',
  '2810 1350KV',
  'Your Motor Name Here',  // ← was '5055 580KV'
  'Custom',
] as const;
```

---

## Step 4 — Recompile and flash

```bash
cd dspic33AKESC/dspic33AKESC.X
make
```

Hex output: `dist/default/production/dspic33AKESC.X.production.hex`

Flash via MPLAB IDE or MDB CLI:
```bash
/media/bhanu1234/Development/MPLABX/v6.30/mplab_platform/bin/mdb.sh \
  -e "program dist/default/production/dspic33AKESC.X.production.elf"
```

The new build hash should appear in the GUI when you connect.

---

## Step 5 — Verify low-speed operation

In the GUI:
1. **Set active profile** to slot 3 (or wherever you put your motor).
2. **Connect**, confirm `Build Hash` matches what you just flashed.
3. **Open Scope tab → preset "AN1078 Angle Health"**.
4. Spin motor low CL (don't push throttle past 30%).

Look for:
- Drive Theta and Observer Theta rotating at the same rate
- θ Drive − θ Obs settling near a small offset (BASE value)
- focObsConfidence > 0.5

If observer doesn't track:
- Lower **Kslide** (param `0x92`) in 500 mV steps until BEMF can drive Z
- If still no tracking, the BEMF amplitude is too weak — increase
  `AN_END_SPEED_RPM_MECH` slightly (handoff happens at higher BEMF)

---

## Step 6 — Tune at low speed (Theta Offset BASE)

In Motor Setup → "AN1078 SMO Tune" tab:

1. Spin motor low CL (~10× the OL handoff speed)
2. Open Scope → preset "AN1078 SMO Tune". Watch `focVd`.
3. Adjust **Theta Offset BASE** (param `0x90`) using the slider:
   - focVd negative → increase BASE
   - focVd positive → decrease BASE
   - Step 10-20 (= 1-2°), click "Push to ESC"
4. Goal: focVd within ±0.5 V at this speed.

---

## Step 7 — Tune at high speed (Theta Offset K)

1. Push throttle to ~50% of motor's electrical max.
2. Read focVd again.
3. Adjust **Theta Offset K** (param `0x91`):
   - Negative focVd at high speed → INCREASE K (step +100)
   - Positive focVd at high speed → DECREASE K (step -100)
4. Goal: focVd within ±1.5 V across the full speed range.

If `K = 0` works (PLL handles all the speed-dependent lag), great — you
don't need this term.

---

## Step 8 — Push to top speed and tune Field Weakening

1. Push throttle to max.
2. Watch `focModIndex` in Scope.  When it pegs at 0.95 and speed plateaus:
3. Increase **Field Weakening max |Id|** (param `0x93`) in 10 dA (1 A)
   increments.
4. Watch motor speed climb past the voltage plateau.
5. Stop when speed stops growing OR motor heating becomes a concern.

Set 0 to disable FW entirely (motor caps at voltage ceiling).

---

## Step 9 — Bake in the validated values

Once you're happy with the live values:

1. Note them down (BASE, K, Kslide, FW values you settled on)
2. Edit `dspic33AKESC/gsp/gsp_params.c` per-profile block — replace the
   placeholder `.an1078*` defaults with your validated values
3. Recompile + reflash

These now become the boot defaults.  GUI live tuning still works on top
of them.

---

## C-code changes still needed (for full tuning automation)

Today, these AN1078 inputs are still hardcoded `#define`s — changing
them requires recompile + reflash:

| Param | Lives in | Live-tunable? |
|-------|---------|--------------|
| `AN_MOTOR_RS / LS / LAMBDA` | `an1078_params.h` | ❌ |
| `AN_NOPOLESPAIRS` | `an1078_params.h` | ❌ |
| `AN_FS_HZ` (= PWMFREQUENCY_HZ) | `garuda_config.h` + `an1078_params.h` | ❌ |
| `AN_NOMINAL_SPEED_RPM_MECH` | `an1078_params.h` | ❌ |
| `AN_OL_RAMP_RATE_RPS2` | `an1078_params.h` | ❌ |
| `AN_Q_CURRENT_REF_OPENLOOP` | `an1078_params.h` | ❌ |
| `AN_KP_DQ / AN_KI_DQ` | `an1078_params.h` | ❌ |
| `AN_KP_SPD / AN_KI_SPD` | `an1078_params.h` | ❌ |
| `AN_SMC_KSLF_MIN/MAX/SCALE` | `an1078_params.h` | ❌ |
| `AN_SMC_THETA_OFFSET_BASE` | gspParams (live) | ✅ |
| `AN_SMC_THETA_OFFSET_K` | gspParams (live) | ✅ |
| `AN_SMC_KSLIDE` | gspParams (live) | ✅ |
| `ID_FW_MAX_NEG` | gspParams (live) | ✅ |

To make the **motor model** (Rs, Ls, λ) live-tunable too, two things
need to change in firmware:

**1. AN_SMCInit reads from gspParams**.  Replace the constants in
`an1078_smc.c`:

```c
void AN_SMCInit(AN_SMC_T *s)
{
    /* Pull motor params from gspParams (loaded from EEPROM at boot) */
    float Rs = gspParams.focRsMilliOhm * 0.001f;
    float Ls = gspParams.focLsMicroH * 1.0e-6f;
    s->Fsmopos = 1.0f - Rs * AN_TS / Ls;
    s->Gsmopos = AN_TS / Ls;
    /* ... */
}
```

**2. Add a re-init hook on SET_PARAM** for motor model IDs (`0x70`, `0x71`,
`0x72`).  In `an1078_motor.c`, expose `AN_MotorReinitObserver()`, and in
`gsp_commands.c:HandleSetParam`, call it after a motor-model update:

```c
if (paramId >= PARAM_ID_FOC_RS_MOHM && paramId <= PARAM_ID_FOC_KE_UV_S_RAD) {
    AN_MotorReinitObserver(&s_foc_an);  /* recompute F_PLANT/G_PLANT */
}
```

That would make the **entire motor profile** GUI-tunable without
recompile.  Roughly 30 lines of firmware.  Add to backlog if/when needed
— for now, motor params are "set once and forget" since they shouldn't
change unless physical motor changes.

---

## Quick reference — Param IDs in this guide

| ID | Name | Units | Range |
|----|------|-------|-------|
| `0x70` | Rs | mΩ | 10-10000 |
| `0x71` | Ls | µH | 1-10000 |
| `0x72` | Ke | µV·s/rad | 1-65000 |
| `0x90` | THETA_BASE | °×10 | 0-3600 |
| `0x91` | THETA_K | ×1e7 | 0-1000 |
| `0x92` | Kslide | mV | 100-30000 |
| `0x93` | FW max \|Id\| | dA | 0-200 |
