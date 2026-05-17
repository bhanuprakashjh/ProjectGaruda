/**
 * @file hal_pwm.c
 * @brief PWM initialization and 6-step commutation (AK port).
 *
 * Forked from CK `../../garuda_6step_ck.X/hal/hal_pwm.c` VERBATIM.
 * dsPIC33AK128MC106 inherits the same PWM Generator peripheral SFR set
 * (MPER, MDC, MPHASE, PGxCONL/H, PGxIOCONL/H, PGxDC, PGxPER, PGxTRIG*)
 * from the same IP block as dsPIC33CK — the bit fields used here are
 * documented in DS70005527 and were spot-checked against the AK FOC
 * sibling `../../dspic33AKESC/hal/hal_pwm.c`.
 *
 * EV92R69A + EV68M17A PWM pin mapping (different from CK EV43F54A):
 *   PG1: RD2(PWM1H) / RD3(PWM1L) — Phase A     (DIM 1 / DIM 3)
 *   PG2: RD0(PWM2H) / RD1(PWM2L) — Phase B     (DIM 5 / DIM 7)
 *   PG3: RC3(PWM3H) / RC4(PWM3L) — Phase C     (DIM 2 / DIM 4)
 * These pins are AK PWM-dedicated (PG1H/L etc are fixed-function on
 * those pins per the AK pinout), so no PPS routing is needed beyond
 * the TRIS=0 in port_config.c.
 *
 * Center-aligned complementary mode, POLH=Active-low (ATA6847L expects
 * active-low high-side inputs), POLL=Active-high — same convention as
 * CK because the ATA6847L gate driver is unchanged.
 *
 * PG1 = master (SOC update), PG2/PG3 = slaves (slaved SOC update).
 * MPER = LOOPTIME_TCY sets the switching period.
 *
 * [AK PORT] Verified against DS70005539 §14:
 *  - MPER/MDC/MPHASE are 16-bit programmable (in a 20-bit field where
 *    the bottom 4 bits are Reserved for AK's High-Resolution PWM mode).
 *    LOOPTIME_TCY @ uint16_t fits 60 kHz / 40 kHz / 20 kHz carriers.
 *  - PG1CONH = 0x4800 decodes correctly: MPERSEL=1 (use MPER), MSTEN=1
 *    (broadcast EOC to clients), UPDMOD=000 (SOC update).
 *  - PG2CONH = PG3CONH = 0x4200 decode correctly: MPERSEL=1,
 *    UPDMOD=010 (Client SOC — wait for master's broadcast).
 *  - PG1CONL/PG2CONL/PG3CONL = 0x000C: CLKSEL=01 (use clock per MCLKSEL),
 *    MODSEL=100 (Center-Aligned PWM).  Same on AK.
 *  - PCLKCON FIXED: AK is 1-bit MCLKSEL (was 2-bit on CK).  Now writes
 *    0x0001 to route CLKGEN5 (400 MHz) to PWM, instead of CK's 0x0002.
 *
 * [AK PORT] Still needs scope-time validation:
 *  1. PG1TRIGA midpoint trigger fires ADC ISR at correct PWM phase
 *     (register exists with same name; bit-field count may differ).
 *  2. OVRENH/OVRENL bypass duty compare in BLK mode.
 *  3. PGxIOCONL/H POLH/POLL pin polarities for ATA6847L (active-low HS).
 *  4. DTH/DTL dead-time register width (may be 20-bit on AK).
 */

#include <xc.h>
#include "hal_pwm.h"
#include "port_config.h"
#include "hal_ak_compat.h"        /* CK L/H → AK 32-bit SFR shims */
#include "../garuda_config.h"

/* SP mode flag from sector_pi.c — read here to switch active-phase drive
 * from complementary to unipolar while SP is engaged. Complementary braking
 * during the long OFF portion of an SP frame decelerates the motor. */
extern volatile bool v4_spActive;

/* Last duty written to PG[123]DC AFTER clamping. Telemetry reads this. */
volatile uint16_t g_pwmActualDuty = 0;

/* Block-commutation flag — see hal_pwm.h. */
volatile bool g_blockCommActive = false;

/* Override data bits: [11:10] = OVRDAT, bit11=H, bit10=L
 *                     [13]    = OVRENH, [12] = OVRENL
 *
 * POLH=1 (active-low) inverts the H output AFTER the override mux.
 *   OVRDAT_H=0 → inverted → pin HIGH → ATA6847 INH=HIGH → HS FET OFF
 *   OVRDAT_H=1 → inverted → pin LOW  → ATA6847 INH=LOW  → HS FET ON
 * POLL=0 (active-high), L output is NOT inverted:
 *   OVRDAT_L=0 → pin LOW  → ATA6847 INL=LOW  → LS FET OFF
 *   OVRDAT_L=1 → pin HIGH → ATA6847 INL=HIGH → LS FET ON
 */

static inline void ApplyPhaseState(volatile uint16_t *ioconl,
                                    PHASE_STATE_T state)
{
    uint16_t val = *ioconl;

    switch (state)
    {
        case PHASE_PWM_ACTIVE:
            if (g_blockCommActive)
            {
                /* Block commutation: H solid ON, L solid OFF.
                 * No PWM chopping during the active sector.
                 *   OVRDAT_H = 1 → POL=active-low inverts → ATA6847 HS pin
                 *                 driven LOW → HS FET ON
                 *   OVRDAT_L = 0 → no invert → ATA6847 LS pin LOW → LS FET OFF
                 * Activated via duty-saturation hysteresis in sector_pi.c
                 * TimeTick. Float phase BEMF stays clean (no switching),
                 * so the existing ADC-midpoint capture path keeps working. */
                val |= 0x3000u;                            /* OVRENH=1, OVRENL=1 */
                val = (val & ~0x0400u) | 0x0800u;          /* OVRDAT = 10 (H=1, L=0) */
                break;
            }
#if PWM_DRIVE_UNIPOLAR
            /* H-PWM / L-OFF (unipolar). 34x fewer ZC timeouts vs
             * complementary — half the switching edges = clean comparator.
             * No braking during OFF → needs lower MAX_DUTY (~25%). */
            val &= ~0x2000u;            /* Clear OVRENH — PWM drives H */
            val |= 0x1000u;             /* Set OVRENL — override L */
            val &= ~0x0400u;            /* OVRDAT_L = 0 — L forced OFF */
#else
            if (v4_spActive)
            {
                /* SP mode needs unipolar drive: MPER=0xFFFF means PWM is
                 * LOW for ~60% of each sector. Under complementary drive
                 * that lights the LS FET → motor brakes continuously →
                 * catastrophic deceleration the instant SP engages. */
                val &= ~0x2000u;        /* Clear OVRENH — PWM drives H */
                val |= 0x1000u;         /* Set OVRENL — override L */
                val &= ~0x0400u;        /* OVRDAT_L = 0 — L forced OFF */
            }
            else
            {
                /* Complementary: H and L alternate with dead time.
                 * Active braking during OFF → needs higher MAX_DUTY. */
                val &= ~(0x3000u);      /* Clear OVRENH and OVRENL */
            }
#endif
            break;
        case PHASE_LOW:
            /* Override: H=OFF, L=ON.
             * OVRDAT_H=0 → POLH inverts → pin HIGH → ATA6847 HS OFF
             * OVRDAT_L=1 → no invert  → pin HIGH → ATA6847 LS ON */
            val |= 0x3000u;             /* Set OVRENH and OVRENL */
            val = (val & ~0x0800u) | 0x0400u;  /* OVRDAT = 01 (H=0,L=1) */
            break;
        case PHASE_FLOAT:
            /* Override: H=OFF, L=OFF.
             * OVRDAT_H=0 → POLH inverts → pin HIGH → ATA6847 HS OFF
             * OVRDAT_L=0 → no invert  → pin LOW  → ATA6847 LS OFF */
            val |= 0x3000u;             /* Set OVRENH and OVRENL */
            val &= ~0x0C00u;            /* OVRDAT = 00 (H=0,L=0) */
            break;
    }

    *ioconl = val;
}

void HAL_PWM_Init(void)
{
    /* [AK PORT] PCLKCON.MCLKSEL = 0 routes Standard Speed Peripheral Clock
     * (FPB/2 = 100 MHz on AK) to the PWM master clock. We DON'T use the
     * 400 MHz CLKGEN5 because at 400 MHz, PGxPER for 40 kHz / 20 kHz
     * carriers overflows uint16_t — see garuda_config.h LOOPTIME_TCY
     * comment. 100 MHz gives 10 ns PWM tick, fits all 20/40/60 kHz in
     * uint16_t, leaves >13-bit duty resolution. */
    PCLKCON = 0x0000;  /* MCLKSEL = 0 → Std Speed Periph Clock = 100 MHz */

    /* Master period, phase, DC */
    MPHASE = 0x00;
    MDC = 0x00;
    MPER = LOOPTIME_TCY;
    LFSR = 0x00;
    FSCL = 0x00;
    FSMINPER = 0x00;

    /* Combinatorial logic — all disabled */
    CMBTRIG = 0x00;       /* AK: single 32-bit SFR (CK had L/H split) */
    LOGCONA = LOGCONB = LOGCONC = LOGCOND = LOGCONE = LOGCONF = 0x00;
    PWMEVTB = PWMEVTC = PWMEVTD = PWMEVTE = PWMEVTF = 0x00;

    PWMEVTA = 0x00;

    /* PG1 (Phase A) — Master */
    PG1CONL = 0x000C;       /* Center-aligned, master clock, ON */
    PG1CONH = 0x4800;       /* UPDMOD=SOC, MPERSEL=enabled (use MPER) */
    PG1STAT = 0x00;
    PG1IOCONL = 0x3000;     /* Start with overrides ON, OVRDAT=00 (float: H off, L off) */
    PG1IOCONH = 0x000E;     /* PENL=1, PENH=1, PMOD=complementary, POLH=active-low */
    /* [AK PORT] On AK, PGxEVTL bit layout differs from CK:
     *   bits 15:11 — ADTR1PS[4:0] (ADC Trigger 1 postscale)
     *   bit  10   — ADTR1EN3 (PGxTRIGC as ADC trig source)
     *   bit   9   — ADTR1EN2 (PGxTRIGB as ADC trig source)
     *   bit   8   — ADTR1EN1 (PGxTRIGA as ADC trig source)
     *   bits  7:5 — PWMPCI[2:0]
     *   bits  4:3 — UPDTRG[1:0]
     *   bits  2:0 — PGTRGSEL[2:0]
     * Source: DS70005539 §14.3.13. Value 0x0118:
     *   ADTR1PS = 0b00000  → 1:1 postscale (ADC ISR every PWM cycle —
     *                       MATCHES V4 milestone: BEMF midpoint sampler
     *                       needs every-cycle samples for >100k eRPM)
     *   ADTR1EN1 = 1        → trigger ADC from PGxTRIGA
     *   UPDTRG = 0b11       → write to TRIGA auto-sets UPDREQ
     *   PGTRGSEL = 0b000    → trigger output = EOC */
    PG1EVTL = 0x0118;
    PG1EVTH = 0x0040;       /* ADTR2EN1=enabled (Trigger2 from TRIGA for Vbus/pot) */
    PG1FPCIL = PG1FPCIH = 0x00;
    PG1CLPCIL = PG1CLPCIH = 0x00;
    PG1FFPCIL = PG1FFPCIH = 0x00;
    PG1SPCIL = PG1SPCIH = 0x00;
    PG1LEBL = PG1LEBH = 0x00;
    PG1PHASE = 0x00;
    /* AK 50%-duty needs (MPER + 16) / 2 because MDC = (MPER+16) × duty. */
    PG1DC = (uint16_t)((LOOPTIME_TCY + 16U) / 2U);
    PG1DCA = 0x00;
    PG1PER = 0x10;
    /* [AK PORT — DS70005539 §14.3.24, §14.4.2.2.4] AK center-aligned
     * PWM uses TWO timer cycles per pulse:
     *   - Cycle 1 (CAHALF=0): rising edge at Timer = PER - DC + 1,
     *     output HIGH to end of cycle 1
     *   - Cycle 2 (CAHALF=1): output HIGH from start of cycle 2,
     *     falling edge at Timer = DC
     * Pulse is geometrically centered on the cycle 1 → cycle 2 wrap.
     *
     * PG1TRIGA is a 32-bit SFR with TRIGA[19:0] at bits 0-19 and a
     * CAHALF bit at bit 31 that selects which cycle the trigger
     * fires in. The trigger fires ONCE per PWM period (in whichever
     * cycle CAHALF selects), not twice.
     *
     * 2026-05-13: switched from MID-ON to MID-OFF sampling to match
     * the proven CK port behaviour.
     *
     * CK uses Mode 4 (double-update center-aligned, MODSEL=100) with
     * PG1TRIGA=0x00 / CAHALF=0 — that fires at counter=0 in cycle 1,
     * which is the PERIOD BOUNDARY, geometrically midway between two
     * adjacent PWM pulses = MID-OFF. In complementary mode, the LS
     * FETs are conducting during OFF time, di/dt has settled, and the
     * floating-phase voltage equals pure BEMF — comp output is a clean
     * polarity indicator.
     *
     * Previous AK mid-ON position (CAHALF=1, TRIGA=0 = cycle 1/2
     * boundary = pulse center) sampled in the middle of the ON pulse,
     * where inductive coupling biased the comp output and we saw
     * pR_pct=90% (POST state dominated, BEMF reading polluted). CK at
     * the same scheduler / detection logic gets pR_pct~50% because it
     * samples mid-OFF instead. Aligning AK to mid-OFF restores the CK
     * behaviour. */
    PG1TRIGA = 0x00000000UL;   /* CAHALF=0, TRIGA=0 — period boundary = MID-OFF (CK-equivalent) */
#if FEATURE_DUAL_POS_PROBE || FEATURE_PER_SECTOR_PTG
    PG1TRIGB = PTG_TRIG_MID_OFF_POS;  /* B2/Phase 1: init at mid-OFF. Commutate
                                       * (PER_SECTOR_PTG) or PTG ISR (DUAL_POS)
                                       * writes this per-sector / per-fire.
                                       * HAL_PWM_SetDutyCycle does NOT update
                                       * PG1TRIGB while either flag is on. */
#else
    PG1TRIGB = 0x00;        /* 2x-ADC experiment (PG1TRIGB=200) on
                             * 2026-04-29 produced no peak-eRPM gain at
                             * 50 kHz PWM (220k → 220k), ruling out sample
                             * rate as the limit. Reverted to single trigger;
                             * the 60 kHz vs 50 kHz peak gap (228k vs 220k)
                             * is current-ripple / float-phase cleanliness. */
#endif
    PG1TRIGC = 0x00;
    PG1DTL = DEADTIME_TCY;
    PG1DTH = DEADTIME_TCY;

    /* PG2 (Phase B) — Slave */
    PG2CONL = 0x000C;
    PG2CONH = 0x4200;       /* UPDMOD=Slaved SOC, MPERSEL=enabled */
    PG2STAT = 0x00;
    PG2IOCONL = 0x3000;
    PG2IOCONH = 0x000E;
    PG2EVTL = 0x0018;       /* UPDTRG=TrigA, PGTRGSEL=EOC */
    PG2EVTH = 0x00;
    PG2FPCIL = PG2FPCIH = 0x00;
    PG2CLPCIL = PG2CLPCIH = 0x00;
    PG2FFPCIL = PG2FFPCIH = 0x00;
    PG2SPCIL = PG2SPCIH = 0x00;
    PG2LEBL = PG2LEBH = 0x00;
    PG2PHASE = 0x00;
    PG2DC = (uint16_t)((LOOPTIME_TCY + 16U) / 2U);
    PG2DCA = 0x00;
    PG2PER = 0x10;
    PG2TRIGA = PG2TRIGB = PG2TRIGC = 0x00;
    PG2DTL = DEADTIME_TCY;
    PG2DTH = DEADTIME_TCY;

    /* PG3 (Phase C) — Slave */
    PG3CONL = 0x000C;
    PG3CONH = 0x4200;
    PG3STAT = 0x00;
    PG3IOCONL = 0x3000;
    PG3IOCONH = 0x000E;
    PG3EVTL = 0x0018;
    PG3EVTH = 0x00;
    PG3FPCIL = PG3FPCIH = 0x00;
    PG3CLPCIL = PG3CLPCIH = 0x00;
    PG3FFPCIL = PG3FFPCIH = 0x00;
    PG3SPCIL = PG3SPCIH = 0x00;
    PG3LEBL = PG3LEBH = 0x00;
    PG3PHASE = 0x00;
    PG3DC = (uint16_t)((LOOPTIME_TCY + 16U) / 2U);
    PG3DCA = 0x00;
    PG3PER = 0x10;
    PG3TRIGA = PG3TRIGB = PG3TRIGC = 0x00;
    PG3DTL = DEADTIME_TCY;
    PG3DTH = DEADTIME_TCY;

    /* PWM interrupt on PG1 — [AK PORT] CK had PWM1IF in IFS4; AK keeps
     * the PWM1IF name but in IFS1/IEC1 (per p33AK128MC106.h struct). */
    IFS1bits.PWM1IF = 0;
    IEC1bits.PWM1IE = 0;     /* Disabled — we use ADC ISR instead */

    /* Start with PWM module OFF (enabled when motor starts).
     * Reverted: keeping PG always-on appeared to interact badly with the
     * ATA6847L EnterGduNormal SPI handshake, stretching motor start from
     * ~15 s into ~3 minutes. IDLE pot reading now uses a software ADC
     * trigger from the main loop (see main.c). */
    PWM_DisableAll();
}

void HAL_PWM_EnableOutputs(void)
{
    PWM_EnableAll();
}

void HAL_PWM_DisableOutputs(void)
{
    HAL_PWM_ForceAllFloat();
    PWM_DisableAll();
}

void HAL_PWM_SetDutyCycle(uint32_t duty)
{
    if (duty > MAX_DUTY) duty = MAX_DUTY;
    if (duty < MIN_DUTY) duty = MIN_DUTY;

    /* Write slaves before master for synchronous update */
    PG2DC = (uint16_t)duty;
    PG3DC = (uint16_t)duty;
    PG1DC = (uint16_t)duty;
    g_pwmActualDuty = (uint16_t)duty;

    /* Request buffer-to-active transfer at next SOC */
    PG1STATbits.UPDREQ = 1;
    PG2STATbits.UPDREQ = 1;
    PG3STATbits.UPDREQ = 1;
}

/* Period-aware variant. Clamps duty to [MIN_DUTY, per - 200], NOT against
 * MPER. In SP mode MPER = 0xFFFF but the intended pulse basis is the
 * sector-matched `per` (timerPeriod << 7). Using MPER would let the ON
 * pulse run wider than the sector. Non-SP callers pass per=LOOPTIME_TCY,
 * so behavior matches HAL_PWM_SetDutyCycle (per-200 ≈ MAX_DUTY). */
void HAL_PWM_SetDutyCyclePeriod(uint32_t duty, uint16_t per)
{
    /* MIN_OFF reduced from 200 → 100 ticks (1µs → 500ns) on 2026-04-29.
     * ATA6847 has charge pumps (datasheet: "100% PWM Duty Cycle Control"),
     * so bootstrap-refresh argument for the 200-tick clamp doesn't apply.
     * 100 ticks gives ~97% effective duty at 60kHz, leaving headroom for
     * adaptive deadtime + CCPT during H↔L transitions when below 100%. */
    uint32_t maxD = (per > 100U) ? (uint32_t)(per - 100U) : (uint32_t)per;
    if (duty > maxD) duty = maxD;
    if (duty < MIN_DUTY) duty = MIN_DUTY;

    PG2DC = (uint16_t)duty;
    PG3DC = (uint16_t)duty;
    PG1DC = (uint16_t)duty;
    g_pwmActualDuty = (uint16_t)duty;

    PG1STATbits.UPDREQ = 1;
    PG2STATbits.UPDREQ = 1;
    PG3STATbits.UPDREQ = 1;
}

/**
 * @brief Apply 6-step commutation pattern.
 * Uses the same commutation table as the AK project.
 */
void HAL_PWM_SetCommutationStep(uint8_t step)
{
    extern const COMMUTATION_STEP_T commutationTable[6];
    if (step >= 6) step %= 6;

    const COMMUTATION_STEP_T *s = &commutationTable[step];

    ApplyPhaseState((volatile uint16_t *)&PG1IOCONL, s->phaseA);
    ApplyPhaseState((volatile uint16_t *)&PG2IOCONL, s->phaseB);
    ApplyPhaseState((volatile uint16_t *)&PG3IOCONL, s->phaseC);
}

void HAL_PWM_ChargeBootstrap(void)
{
    /* Force all low-side ON briefly to charge bootstrap caps.
     * OVRDAT=01: H=0→inverted→HIGH→HS OFF, L=1→HIGH→LS ON */
    PG1IOCONL = (PG1IOCONL | 0x3000u | 0x0400u) & ~0x0800u;
    PG2IOCONL = (PG2IOCONL | 0x3000u | 0x0400u) & ~0x0800u;
    PG3IOCONL = (PG3IOCONL | 0x3000u | 0x0400u) & ~0x0800u;
}

void HAL_PWM_ForceAllFloat(void)
{
    /* Override both H and L OFF on all generators.
     * OVRDAT=00: H=0→inverted→HIGH→HS OFF, L=0→LOW→LS OFF */
    PG1IOCONL = (PG1IOCONL | 0x3000u) & ~0x0C00u;
    PG2IOCONL = (PG2IOCONL | 0x3000u) & ~0x0C00u;
    PG3IOCONL = (PG3IOCONL | 0x3000u) & ~0x0C00u;
}

void HAL_PWM_ForceAllLow(void)
{
    /* Force all low-side ON (same as bootstrap charge).
     * OVRDAT=01: H=0→inverted→HIGH→HS OFF, L=1→HIGH→LS ON */
    PG1IOCONL = (PG1IOCONL | 0x3000u | 0x0400u) & ~0x0800u;
    PG2IOCONL = (PG2IOCONL | 0x3000u | 0x0400u) & ~0x0800u;
    PG3IOCONL = (PG3IOCONL | 0x3000u | 0x0400u) & ~0x0800u;
}

/* ── Single-Pulse Mode (AVR-style) ─────────────────────────────── */
/* Above 90k eRPM, change MPER from fixed 40kHz to match sector
 * duration. One ON + one OFF per sector instead of many PWM pulses.
 * No switching edges mid-sector → clean BEMF comparator output. */

static bool spMode = false;

void HAL_PWM_SetSinglePulse(uint16_t sectorPeriodTCY, uint32_t duty)
{
    /* sectorPeriodTCY is in HR ticks (640ns). Convert to PWM ticks.
     * PWM clock = Fosc/2 = 200MHz, 1 tick = 5ns.
     * HR tick = 640ns = 128 PWM ticks.
     * sectorPWM = sectorPeriodHR × 128. */
    uint32_t sectorPWM = (uint32_t)sectorPeriodTCY * 128UL;
    if (sectorPWM > 0xFFFF) sectorPWM = 0xFFFF;
    if (sectorPWM < 200) sectorPWM = 200;  /* minimum for dead-time */

    /* Set master period to sector duration */
    MPER = (uint16_t)sectorPWM;

    /* Scale duty to new period */
    if (duty > sectorPWM) duty = sectorPWM;

    PG2DC = (uint16_t)duty;
    PG3DC = (uint16_t)duty;
    PG1DC = (uint16_t)duty;

    PG1STATbits.UPDREQ = 1;
    PG2STATbits.UPDREQ = 1;
    PG3STATbits.UPDREQ = 1;

    spMode = true;
}

void HAL_PWM_ExitSinglePulse(void)
{
    if (spMode)
    {
        MPER = LOOPTIME_TCY;
        /* Restore ADC trigger to mid-OFF on AK center-aligned (see init):
         * CAHALF=0, TRIGA=0 → fires at period boundary = MID-OFF, the
         * CK-equivalent sample position. */
        PG1TRIGA = 0x00000000UL;
        PG1STATbits.UPDREQ = 1;
        PG2STATbits.UPDREQ = 1;
        PG3STATbits.UPDREQ = 1;
        spMode = false;
    }
}

bool HAL_PWM_IsSinglePulse(void)
{
    return spMode;
}

void HAL_PWM_SetSPFlag(bool on)
{
    spMode = on;
}
