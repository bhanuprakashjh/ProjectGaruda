/**
 * garuda_motors.h — motor selection + per-motor profile data.
 *
 * TO CHANGE MOTOR: edit MOTOR_PROFILE below. Everything motor-specific
 * (pole pairs, Rs/Ls/lambda, voltages, startup targets, OC thresholds,
 * per-profile tuning) lives in this file; engine/feature configuration
 * stays in garuda_config.h. Included by garuda_config.h at the exact
 * point the old inline block occupied, so preprocessor ordering (the
 * per-profile feature overrides that follow) is unchanged.
 *
 * Split out of garuda_config.h 2026-07-04 (config-clarity pass).
 */
#ifndef GARUDA_MOTORS_H
#define GARUDA_MOTORS_H
/* MOTOR_PROFILE selects the motor model + tuning. It MUST be #defined HERE,
 * BEFORE the per-profile AM32-startup #if below. (Bug fixed 2026-06-24: it was
 * #defined ~240 lines later, so the preprocessor saw it as 0 → the #if was
 * always false → AM32 forced ON for ALL profiles, silently defeating the
 * 6/7/8 sine carve-out.) Per-profile motor params are in the section further down.
 * 0=Hurst 1=A2212@12V 2=2810@24V 3=5055 4=Cobra 5=XRotor 6=VEX 7=1407@2S 8=1407@3S 9=U3 KV700@~16V */
#define MOTOR_PROFILE  2   /* 2810 1350KV @24V - multi-motor validation campaign 2026-07-04 */

/* ── Per-profile motor data ── */
/* MOTOR_PROFILE is now #defined ABOVE (before the AM32-startup #if). 2810 @24V bench. Profile 2 carries the correct 2810
                              * motor model (λ=583, 7PP, 24V, Rs=22mΩ, Ls=10µH, OC=20A) — the
                              * VEX profile 6 (λ=230, 6PP, 11.1V) was the 40k decel-floor phantom.
                              * Profile 6 still holds the dialed-in VEX 4000KV tuning; switch back
                              * to 6 for that motor.
                              * 0=Hurst 1=A2212@12V 2=2810@24V 3=5055 4=Cobra 5=XRotor
                              * 6=VEX 4000KV  7=1407 4000KV @2S  8=1407 4000KV @3S */

#if MOTOR_PROFILE == 0
/* === Hurst DMB2424B10002 (long Hurst, MCLV-48V-300W bench motor) ===
 * 10 poles (5PP), 24VDC, Rs=534mΩ, Ls=471µH (auto-detected)
 * NOT the short DMB0224C10002 (Rs=2.54Ω, Ls=2.3mH) */
#define MOTOR_POLE_PAIRS             5
#define DEADTIME_NS                750
#define ALIGN_DUTY_PERCENT          20
#define RAMP_DUTY_PERCENT           40
#define INITIAL_ERPM               300     /* ~5 steps/sec — slow start */
#define RAMP_TARGET_ERPM          2000     /* Step period ~5ms >> L/R=1.14ms */
#define MAX_CLOSED_LOOP_ERPM     20000     /* No-load ~15625 eRPM */
#define RAMP_ACCEL_ERPM_PER_S     1000
#define SINE_ALIGN_MODULATION_PCT   15
#define SINE_RAMP_MODULATION_PCT    35
#define ZC_DEMAG_DUTY_THRESH        70     /* % duty above which extra blanking applies */
#define ZC_DEMAG_BLANK_EXTRA_PERCENT 12    /* Extra blanking % at 100% duty */
#define HWZC_CROSSOVER_ERPM       5000     /* HW ZC activates above this eRPM */
#define CL_IDLE_DUTY_PERCENT         0     /* No idle floor for Hurst */
#define SINE_PHASE_OFFSET_DEG       60     /* Sine-to-trap transition offset (Hurst: 60 works) */
#define OC_LIMIT_MA               1800     /* CMP3 CLPCI chopping (1.8A) */
#define OC_STARTUP_MA            18000     /* Mode 1 only: high CMP3 during startup */
#define OC_FAULT_MA               3000     /* Software hard fault (3.0A, mode 2) */
#define OC_SW_LIMIT_MA            1500     /* Software soft limit (1.5A) */
#define RAMP_CURRENT_GATE_MA         0     /* 0=disabled: Hurst starts easily without gating */
#define OC_CLPCI_ENABLE            0       /* Disabled for FOC: SVPWM incompatible with CLPCI chopping */

#elif MOTOR_PROFILE == 1
/* === A2212 1400KV (drone motor) ===
 * 14 poles, 12V, 0.065 ohm, ~30 uH
 * 1400 KV => ~16800 RPM @ 12V, 117600 eRPM */
#define MOTOR_POLE_PAIRS             7
#define DEADTIME_NS                300     /* Sweet spot (2026-04-21). 500 → 300 ns cut
                                            * Ibus commutation kickback ~55% at top throttle
                                            * (12 A pk → 5 A pk on A2212/12V bare), +4% eRPM
                                            * (103k → 108k). 200 ns tested and marginal:
                                            * Ibus pk flat, top eRPM +1% only, but ALIGN
                                            * current rose 20% (early shoot-through signature).
                                            * 300 ns is the bottom of useful deadtime range
                                            * on this PWM + FET combo. */
#define ALIGN_DUTY_PERCENT           6     /* 2026-06-16 lowered 8->6 to cut startup current (~14A high). */
#define RAMP_DUTY_PERCENT           10     /* 2026-06-16 lowered 15->10 (main startup-current driver). */
#define INITIAL_ERPM               100     /* Very slow start: 100ms per step — prop can follow */
#define RAMP_TARGET_ERPM          2000     /* Lowered from 3000 for prop start. Prop mass
                                            * + inertia cannot accelerate to 3000 eRPM in the
                                            * OL window. 2000 eRPM is still above the 1500
                                            * HWZC crossover (need sufficient BEMF margin) and
                                            * comfortably above the SW ZC floor. */
#define MAX_CLOSED_LOOP_ERPM    120000     /* 1400KV * 12V * 7pp */
#define RAMP_ACCEL_ERPM_PER_S     3000     /* 2026-06-16 BARE-MOTOR 1 s ramp (was 400 for prop inertia). */
#define SINE_ALIGN_MODULATION_PCT    4     /* 2026-06-16 BARE-MOTOR (was 10 for prop). */
#define SINE_RAMP_MODULATION_PCT    12     /* 2026-06-16 BARE-MOTOR (was 25 for prop). */
#define ZC_DEMAG_DUTY_THRESH        40     /* Low-L = more demag */
#define ZC_DEMAG_BLANK_EXTRA_PERCENT 18    /* Aggressive demag blanking */
#define HWZC_CROSSOVER_ERPM       1500     /* Reverted — 868b2ff milestone value.
                                            * Raising to 3000 moved HWZC activation to end
                                            * of ramp but SW ZC hadn't established a clean
                                            * lock yet, seed was wrong. Stick with 1500. */
#define CL_IDLE_DUTY_PERCENT         8     /* 2026-06-16 lowered 12->8: THIS is the startup-current
                                            * driver (CL idle duty vs tiny low-speed BEMF on the 0.065R
                                            * A2212). 12%=~13A inrush; 8% ~8-9A. Idle speed drops too
                                            * (~13k->~9k). If ZC gets rough/desyncs at idle, raise to 10. */
#define SINE_PHASE_OFFSET_DEG       60     /* Sine-to-trap offset (unused when sine disabled) */
#define OC_LIMIT_MA              12000     /* CMP3 CLPCI chopping (12A) */
#define OC_STARTUP_MA            22000     /* High: let 10A supply CC be the limiter, not CMP3 */
#define OC_FAULT_MA              18000     /* Software hard fault (18A, mode 2) */
#define OC_SW_LIMIT_MA            8000     /* Software soft limit (8A, 2A below 10A supply CC) */
#define RAMP_CURRENT_GATE_MA      5000     /* Hold ramp accel when ibus > 5A (prevents overdrive stall).
                                            * At 12V/10% duty the 0.065-ohm A2212 draws ~10A stall.
                                            * 5A gate pauses acceleration when rotor lags, resumes
                                            * when motor catches up and current drops. 10V works
                                            * because lower V/L means slower current rise → less
                                            * braking torque during wrong commutation. */
#define OC_CLPCI_ENABLE            1       /* 2026-06-16 RE-ENABLED: CMP3->CLPCI current-limit chop.
                                            * Prior note (kept): A2212 OA3 ringing (25x gain) caused
                                            * 54-80% false trips, LEB couldn't fix — but that was BEFORE
                                            * the INPSEL=3 fix (CMP3 now watches OA3 output, AN957). If it
                                            * false-trips (stutter / can't accelerate / spurious chop at low
                                            * current), set back to 0. OC_LIMIT_MA=12000 = operational chop. */

#elif MOTOR_PROFILE == 2
/* === 2810 1350KV (7-8" FPV/cine drone motor) ===
 * 12N14P, 14 poles (7PP), 5-6S LiPo (18.5-25.2V), Rs~50mΩ, Ls~25µH.
 * At 24V: no-load max eRPM = 1350 × 24 × 7 = 226,800 eRPM.
 * Bench target: 200k eRPM no-prop.
 *
 * Motor data ported from PATA6847/CK board (garuda_6step_ck.X MOTOR_PROFILE=2).
 * GEPRC EM2810 / T-Motor F100 / BrotherHobby Avenger 2810 range.
 *
 * At 24V supply, peak commutation currents can exceed 30A. Board shunt
 * saturates at ~22A. HW CMP3 is the primary protection. Expect occasional
 * BOARD_PCI at extreme duty — tune down if too aggressive. */
#define MOTOR_POLE_PAIRS             7
#define DEADTIME_NS                300     /* 300 ns (2026-04-21) — match A2212 sweet
                                            * spot. Cuts commutation kickback ~55% on
                                            * A2212 bench. At 24V/2810 the commutation
                                            * energy ½LI² is even higher per event →
                                            * kickback reduction potentially decisive for
                                            * the 22A BOARD_PCI trip threshold that held
                                            * 2810 top speed at 78k eRPM on 500 ns. */
#define ALIGN_DUTY_PERCENT           3     /* 24V * 3% / 0.050Ω = 14.4A stall.
                                            * Half of A2212 (8% at 12V) for same current */
#define RAMP_DUTY_PERCENT            8     /* 24V * 8% / 0.050Ω = 38A stall (briefly).
                                            * Motor spins up quickly so stall current is
                                            * momentary; bench CC limit protects */
#define INITIAL_ERPM               150     /* Slow first step — 2810 low-L picks up fast */
#define RAMP_TARGET_ERPM          3000     /* Same as A2212 — sine startup works well here */
#define MAX_CLOSED_LOOP_ERPM     70000     /* Lowered 220k → 70k (2026-04-20).
                                            * Theoretical no-load is 226.8k but the MCLV
                                            * board U25B trips at ~22 A di/dt, limiting
                                            * bench no-load to ~55 k. This value is also
                                            * the upper anchor of the timing-advance
                                            * interpolation — with 220k, advance at 55 k
                                            * was only 5° (vs the 22° MAX_DEG target) and
                                            * the SW-comparator ADC sampler added another
                                            * ~13° of detection latency, for a net of
                                            * ~-8° of effective advance. 70k anchor gives
                                            * ~17° advance at 55 k, offsetting the SW
                                            * latency. If the motor ever exceeds 70 k the
                                            * advance clamps at 22° (fine). */
#define RAMP_ACCEL_ERPM_PER_S     3000     /* 1 s OL ramp. Reverted 2s → 1s (2026-04-20):
                                            * see A2212 profile comment for the HWZC-IIR-
                                            * collapse bug that 2s exposed. */
#define SINE_ALIGN_MODULATION_PCT    3     /* Conservative — low Rs → current rises fast */
#define SINE_RAMP_MODULATION_PCT     5     /* Lowered 8→5 (2026-05-28): bench showed
                                            * 12.1A end-of-ramp + 21.88A MORPH peak
                                            * (shunt saturated). 24V/5%/0.05Ω = 24A
                                            * stall worst-case, much safer for the
                                            * sine→trap transition on this low-Rs motor */
#define ZC_DEMAG_DUTY_THRESH        40     /* Same as A2212 */
#define ZC_DEMAG_BLANK_EXTRA_PERCENT 20    /* More aggressive than A2212 (18).
                                            * Low L = longer demag tail under switching */
#define HWZC_CROSSOVER_ERPM       1500     /* Enable HWZC at morph handoff (same as A2212) */
#define CL_IDLE_DUTY_PERCENT         8     /* Match RAMP_DUTY_PERCENT so pot-at-zero CL idle
                                            * is the same as the OL→CL handoff duty. Lower
                                            * idle (6%) made the motor cross sectors slowly
                                            * enough at low pot that detection became
                                            * marginal. Sticking to startup duty gives the
                                            * motor consistent torque budget across the
                                            * full throttle range. */
#define SINE_PHASE_OFFSET_DEG       60     /* Same as other profiles */
/* 2026-06-13: HW-chop isolation test. CMP3->CLPCI chop set to 16A and the SW
 * soft-limit pushed ABOVE it (20A) so the SOFTWARE limiter stays inert and the
 * HARDWARE chop is the only thing capping the startup balloon -> a clean read on
 * Ibus of whether the AN957-style CMP3 chop works. Restore (20000/22000/18000)
 * once confirmed. */
#define OC_LIMIT_MA              20000     /* restored to production baseline 2026-06-16 (chop parked) */
#define OC_STARTUP_MA            22000     /* production startup OC */
#define OC_FAULT_MA              21000     /* SW hard fault just below saturation */
#define OC_SW_LIMIT_MA           18000     /* production SW soft limit (below CMP3 operational) */
#define RAMP_CURRENT_GATE_MA     10000     /* production: hold ramp accel when ibus > 10A */
#define OC_CLPCI_ENABLE            0       /* DISABLED 2026-06-24: cycle-by-cycle CMP3->CLPCI chop OFF.
                                            * Overcurrent now relies on the software ADC OC path
                                            * (OC_PROTECT_MODE=2) — no cycle-by-cycle current limiting.
                                            * (Was: armed CLPCI so the handoff chop could current-limit
                                            * the 2810 CL-entry speed-gap pulse.) */

#elif MOTOR_PROFILE == 3
/* === 5055 ~580KV (colleague's motor — ADJUST KV AS NEEDED) ===
 * Rs = 0.1 ohm pp (0.05 L-N), Ls = 35 uH pp (17.5 L-N)
 * Assumed: 14 poles (7PP), 4S/14.8V
 * 580 KV => ~8584 RPM @ 14.8V = ~60088 eRPM
 *
 * KEY: Very low Rs + heavy rotor = MUST ramp slowly. */
#define MOTOR_POLE_PAIRS             7
#define DEADTIME_NS                500     /* Low-L motor */
#define ALIGN_DUTY_PERCENT           4     /* 14.8V*4%/0.05=11.8A stall — low duty critical! */
#define RAMP_DUTY_PERCENT            8     /* Conservative: 14.8V*8%/0.05=23.7A stall.
                                            * Supply CC will limit in practice. */
#define INITIAL_ERPM               100     /* Very slow: heavy rotor needs time per step. */
#define RAMP_TARGET_ERPM          2000     /* Good BEMF at this speed. */
#define MAX_CLOSED_LOOP_ERPM     65000     /* 580KV * 14.8V * 7pp ≈ 60088, rounded up */
#define RAMP_ACCEL_ERPM_PER_S      150     /* Slow: 14.8s to reach ramp target. */
#define SINE_ALIGN_MODULATION_PCT    3     /* Low: 0.05 ohm means high current per % duty */
#define SINE_RAMP_MODULATION_PCT     8     /* Conservative modulation */
#define ZC_DEMAG_DUTY_THRESH        45     /* Low-L = more demag */
#define ZC_DEMAG_BLANK_EXTRA_PERCENT 16    /* Moderate */
#define HWZC_CROSSOVER_ERPM       1500     /* HWZC immediately after morph */
#define CL_IDLE_DUTY_PERCENT        10     /* Maintain ZC at idle */
#define SINE_PHASE_OFFSET_DEG       60     /* Tune if needed */
#define OC_LIMIT_MA              15000     /* CMP3 CLPCI chopping (15A) */
#define OC_STARTUP_MA            22000     /* Let supply CC limit during startup */
#define OC_FAULT_MA              20000     /* Software hard fault (20A) */
#define OC_SW_LIMIT_MA           10000     /* Soft limit (10A) */
#define RAMP_CURRENT_GATE_MA      6000     /* Hold ramp if bus current > 6A.
                                            * Critical for 0.05 ohm: prevents
                                            * runaway current during forced comm. */
#define OC_CLPCI_ENABLE            0       /* CLPCI disabled: OA3 ringing issue */

#elif MOTOR_PROFILE == 4
/* === Cobra CM-2814/36 470KV (12N14P, 7PP, 4-6S, 117g, 36T delta) ===
 * Rs(pp)=0.188Ω, KV=470, max cont 17A, 24V/6S. Opposite regime to the 2810:
 * ~4× R + heavy rotor + low KV → needs much higher sine amplitude + slow ramp.
 * Low KV = strong BEMF = easy ZC. PHYSICS-BASED STARTING values; iterate from
 * the GSP fault code. NOTE: with FEATURE_GSP=1 these are overridden by
 * profileDefaults[GSP_PROFILE_COBRA] in gsp_params.c — keep the two in sync. */
#define MOTOR_POLE_PAIRS             7
#define DEADTIME_NS                400
#define ALIGN_DUTY_PERCENT           8
#define RAMP_DUTY_PERCENT           12
#define INITIAL_ERPM               100
#define RAMP_TARGET_ERPM          3000
#define MAX_CLOSED_LOOP_ERPM     83000     /* 470 * 24V * 7pp ≈ 79k */
#define RAMP_ACCEL_ERPM_PER_S     1000     /* slow — heavy rotor needs dwell */
#define SINE_ALIGN_MODULATION_PCT   15
#define SINE_RAMP_MODULATION_PCT    30     /* high field: 4× R + heavy + low KV */
#define ZC_DEMAG_DUTY_THRESH        45
#define ZC_DEMAG_BLANK_EXTRA_PERCENT 18
#define HWZC_CROSSOVER_ERPM       1500
#define CL_IDLE_DUTY_PERCENT         8
#define SINE_PHASE_OFFSET_DEG       60
#define OC_LIMIT_MA              20000
#define OC_STARTUP_MA            22000
#define OC_FAULT_MA              21000
#define OC_SW_LIMIT_MA           16000     /* ≈ rated 17A continuous */
#define RAMP_CURRENT_GATE_MA     12000
#define OC_CLPCI_ENABLE            0

#elif MOTOR_PROFILE == 5
/* === Hobbywing XRotor 3110 1150KV (12N14P, 7PP, 4-6S, 88g) ===
 * Rs(pp)=0.045Ω, KV=1150. Same regime as the 2810 (profile 2): low R, high KV,
 * light rotor — brings up almost identically. Only KV-driven fields differ.
 * NOTE: with FEATURE_GSP=1 these are overridden by
 * profileDefaults[GSP_PROFILE_XROTOR] in gsp_params.c — keep the two in sync. */
#define MOTOR_POLE_PAIRS             7
#define DEADTIME_NS                300
#define ALIGN_DUTY_PERCENT           3
#define RAMP_DUTY_PERCENT            8
#define INITIAL_ERPM               150
#define RAMP_TARGET_ERPM          3000
#define MAX_CLOSED_LOOP_ERPM    210000     /* 1150 * 24V * 7pp ≈ 193k */
#define RAMP_ACCEL_ERPM_PER_S     3000
#define SINE_ALIGN_MODULATION_PCT    3
#define SINE_RAMP_MODULATION_PCT     5
#define ZC_DEMAG_DUTY_THRESH        40
#define ZC_DEMAG_BLANK_EXTRA_PERCENT 20
#define HWZC_CROSSOVER_ERPM       1500
#define CL_IDLE_DUTY_PERCENT         6
#define SINE_PHASE_OFFSET_DEG       60
#define OC_LIMIT_MA              20000
#define OC_STARTUP_MA            22000
#define OC_FAULT_MA              21000
#define OC_SW_LIMIT_MA           18000
#define RAMP_CURRENT_GATE_MA     10000
#define OC_CLPCI_ENABLE            0

#elif MOTOR_PROFILE == 6
/* === VEX 14mm micro 4000KV (6PP, 7.4V rated / 10V max, ~20g) ===
 * Rs(pp)=0.44Ω, Ld/Lq≈18.4µH(pp), no-load 0.65A, max torque 7.25A, stall 14A.
 * Run at 10V on the MCLV-48V-300W. TWO things make this motor different:
 * (1) 4000KV through 48V-scaled dividers → BEMF at the stock 3k hand-off is
 *     ~6 ADC counts = below the detection floor. Bench-proven 2026-06-11 by
 *     starving the 2810 to the same counts (hand-off 1200): 2/3 starts
 *     fiction-locked then OC'd — the exact reported VEX failure. Hence
 *     RAMP_TARGET 12k / CROSSOVER 6k (only ~2k mech RPM for this KV).
 * (2) stall current is 14A — the 24V-class OC chain (18/20/21A) sits ABOVE
 *     stall and protects nothing; chain scaled to 7.25/9/9.5/10A.
 * NOTE: with FEATURE_GSP=1 these are overridden by
 * profileDefaults[GSP_PROFILE_VEX] in gsp_params.c — keep the two in sync. */
#define MOTOR_POLE_PAIRS             6
#define DEADTIME_NS                300
#define ALIGN_DUTY_PERCENT          14     /* 2026-06-16 25->14: 25% drew ~5.5A -> OC_SW trip at align */
#define RAMP_DUTY_PERCENT           14     /* 2026-06-16 25->14: keep startup current under 7.25A OC */
#define INITIAL_ERPM               300
#define RAMP_TARGET_ERPM         28000     /* 2026-06-16 force OL to where BEMF is real, sustainable @14% */
#define MAX_CLOSED_LOOP_ERPM    252000     /* 4000 * 10V * 6pp ≈ 240k */
#define RAMP_ACCEL_ERPM_PER_S     8000
#define SINE_ALIGN_MODULATION_PCT   28     /* 2026-06-16 50->28: 50% drew ~5.6A -> OC_SW trip */
#define SINE_RAMP_MODULATION_PCT    28     /* 2026-06-16 50->28: keep startup current under 7.25A OC */
#define ZC_DEMAG_DUTY_THRESH        45
#define ZC_DEMAG_BLANK_EXTRA_PERCENT 18
#define HWZC_CROSSOVER_ERPM      24000     /* 2026-06-16 6k->24k: HWZC engages only where BEMF is solid */
#define CL_IDLE_DUTY_PERCENT        14
#define SINE_PHASE_OFFSET_DEG       60
#define OC_LIMIT_MA               9000
#define OC_STARTUP_MA            10000
#define OC_FAULT_MA              14000     /* 2026-06-16 9500->14000: align inrush (9µH spikes in 1 PWM
                                            * cycle) false-tripped OC_SW before the rotor moved. HW CMP3
                                            * chop (9-10A) bounds real current; SW fault sits above it. */
#define OC_SW_LIMIT_MA           13000     /* 2026-06-16 7250->13000: above the HW chop so the inrush
                                            * peak no longer false-trips OC_SW at align (=stall, brief) */
#define RAMP_CURRENT_GATE_MA      7000
#define OC_CLPCI_ENABLE            0

#elif MOTOR_PROFILE == 7
/* === 1407 4000KV 9N12P (6PP) @ 2S (8.4V max), FPV 3" ===
 * Mirror of profileDefaults[GSP_PROFILE_1407_2S] (runtime uses GSP; keep in
 * sync). Based on the VEX 4000KV/6PP profile. A2212 lessons: PP=6, handoff chop
 * ON for the low-L startup pulse, per-profile falling-SW gate (HWZC_FALLING_SW
 * below). All ESTIMATES — bench-tune. */
#define MOTOR_POLE_PAIRS             6
#define DEADTIME_NS                300
#define ALIGN_DUTY_PERCENT          25
#define RAMP_DUTY_PERCENT           25
#define INITIAL_ERPM               300
#define RAMP_TARGET_ERPM         12000
#define MAX_CLOSED_LOOP_ERPM    202000     /* 4000 * 8.4V * 6pp ~ 202k (under ~260k ceiling) */
#define RAMP_ACCEL_ERPM_PER_S     8000
#define SINE_ALIGN_MODULATION_PCT   50
#define SINE_RAMP_MODULATION_PCT    50
#define ZC_DEMAG_DUTY_THRESH        45
#define ZC_DEMAG_BLANK_EXTRA_PERCENT 18
#define HWZC_CROSSOVER_ERPM       6000
#define CL_IDLE_DUTY_PERCENT        16     /* ~32k idle at 8.4V; soft-start ramps into it */
#define SINE_PHASE_OFFSET_DEG       60
#define OC_LIMIT_MA               9000
#define OC_STARTUP_MA            10000
#define OC_FAULT_MA               9500
#define OC_SW_LIMIT_MA            7250
#define RAMP_CURRENT_GATE_MA      7000
#define OC_CLPCI_ENABLE            1       /* handoff chop on for the low-L startup pulse */

#elif MOTOR_PROFILE == 8
/* === 1407 4000KV 9N12P (6PP) @ 3S (12.6V max), FPV 3" ===
 * Same motor as profile 7 at 3S. maxCL capped 260k (no-load ~302k > sensorless
 * ceiling). Lower duties than 2S. Mirror of profileDefaults[GSP_PROFILE_1407_3S]. */
#define MOTOR_POLE_PAIRS             6
#define DEADTIME_NS                300
#define ALIGN_DUTY_PERCENT          20
#define RAMP_DUTY_PERCENT           20
#define INITIAL_ERPM               300
#define RAMP_TARGET_ERPM         12000
#define MAX_CLOSED_LOOP_ERPM    260000     /* capped at sensorless ceiling (no-load ~302k) */
#define RAMP_ACCEL_ERPM_PER_S     8000
#define SINE_ALIGN_MODULATION_PCT   40
#define SINE_RAMP_MODULATION_PCT    40
#define ZC_DEMAG_DUTY_THRESH        45
#define ZC_DEMAG_BLANK_EXTRA_PERCENT 18
#define HWZC_CROSSOVER_ERPM       6000
#define CL_IDLE_DUTY_PERCENT        11     /* ~33k idle at 12.6V */
#define SINE_PHASE_OFFSET_DEG       60
#define OC_LIMIT_MA               9000
#define OC_STARTUP_MA            10000
#define OC_FAULT_MA               9500
#define OC_SW_LIMIT_MA            7250
#define RAMP_CURRENT_GATE_MA      7000
#define OC_CLPCI_ENABLE            1

#elif MOTOR_PROFILE == 9
/* === T-Motor U3 KV700 12N14P (7PP) @ ~16V bench PSU, PROPPED ===
 * Cloned from profile 2 (2810): SAME board, SAME 7PP / ~50 mΩ Rs. KV is ~½ and the
 * bus is ~16V, so this is a flux + duty rescale (see docs/u3_kv700_profile_port.md).
 * With FEATURE_GSP=1 these #defines are INERT — the runtime reads
 * profileDefaults[GSP_PROFILE_U3] in gsp/gsp_params.c. Kept here mirrored for the
 * non-GSP build and the #error guard. The HARDWARE-active ones (deadtime, pole
 * pairs, phase offset, presync, CLPCI) match the 2810 baseline. */
#define MOTOR_POLE_PAIRS             7      /* SAME as 2810 */
#define DEADTIME_NS                300      /* same board/gate timing as 2810 */
#define ALIGN_DUTY_PERCENT           5      /* 3 ×(24/16.8) -> heavier propped rotor lock */
#define RAMP_DUTY_PERCENT           11      /* 8 ×1.43 to hold ramp current at 16V */
#define INITIAL_ERPM               120      /* gentler first step (heavy 97 g rotor) */
#define RAMP_TARGET_ERPM          2500      /* lower KV -> ZC appears earlier than 2810's 3000 */
#define MAX_CLOSED_LOOP_ERPM     98000      /* advance ANCHOR = U3 no-load ceiling @ bench ~20V (700*20*7) */
#define RAMP_ACCEL_ERPM_PER_S     1800      /* heavy rotor + prop can't follow 2810's 3000/s */
#define SINE_ALIGN_MODULATION_PCT    4      /* 3 ×1.43 */
#define SINE_RAMP_MODULATION_PCT     7      /* 5 ×1.43 */
#define ZC_DEMAG_DUTY_THRESH        40
#define ZC_DEMAG_BLANK_EXTRA_PERCENT 24     /* bigger stator -> longer demag tail; prop-desync lever */
#define HWZC_CROSSOVER_ERPM       1500
#define CL_IDLE_DUTY_PERCENT         6      /* lower KV -> more low-speed BEMF; 4 ×1.43 */
#define SINE_PHASE_OFFSET_DEG       60
#define OC_LIMIT_MA              20000      /* board shunt saturates ~22A; U3 25A cont. lands here */
#define OC_STARTUP_MA            22000
#define OC_FAULT_MA              21000
#define OC_SW_LIMIT_MA           18000
#define RAMP_CURRENT_GATE_MA     10000
#define OC_CLPCI_ENABLE            0        /* cycle-by-cycle CMP3 chop OFF (matches 2810 committed baseline;
                                            * SW ADC OC path handles overcurrent) */

#else
#error "Unknown MOTOR_PROFILE — see garuda_config.h"
#endif
#endif /* GARUDA_MOTORS_H */
