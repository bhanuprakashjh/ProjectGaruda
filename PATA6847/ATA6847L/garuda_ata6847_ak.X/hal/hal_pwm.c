/**
 * @file hal_pwm.c
 * @brief PWM initialization and 6-step commutation.
 *
 * dsPIC33AK128MC106 PWM Generator (MPER, MDC, MPHASE, PGxCONL/H,
 * PGxIOCONL/H, PGxDC, PGxPER, PGxTRIG*). Bit fields documented in
 * DS70005527.
 *
 * EV92R69A + EV68M17A PWM pin mapping:
 *   PG1: RD2(PWM1H) / RD3(PWM1L) — Phase A     (DIM 1 / DIM 3)
 *   PG2: RD0(PWM2H) / RD1(PWM2L) — Phase B     (DIM 5 / DIM 7)
 *   PG3: RC3(PWM3H) / RC4(PWM3L) — Phase C     (DIM 2 / DIM 4)
 * These pins are AK PWM-dedicated (PG1H/L etc are fixed-function on
 * those pins per the AK pinout), so no PPS routing is needed beyond
 * the TRIS=0 in port_config.c.
 *
 * Center-aligned complementary mode, POLH=Active-low (ATA6847L expects
 * active-low high-side inputs), POLL=Active-high.
 *
 * PG1 = master (SOC update), PG2/PG3 = slaves (slaved SOC update).
 * MPER = LOOPTIME_TCY sets the switching period.
 */

#include <xc.h>
#include "hal_pwm.h"
#include "port_config.h"
#include "hal_ak_compat.h"        /* AK 32-bit SFR ↔ 16-bit L/H shims */
#include "../garuda_config.h"

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
            /* Complementary: H and L alternate with dead time.
             * Active braking during OFF → needs higher MAX_DUTY. */
            val &= ~(0x3000u);      /* Clear OVRENH and OVRENL */
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
    /* PCLKCON.MCLKSEL = 0 routes 100 MHz Std Speed Periph Clock to PWM.
     * 400 MHz CLKGEN5 would overflow uint16_t PGxPER below 60 kHz. */
    PCLKCON = 0x0000;

    /* Master period, phase, DC */
    MPHASE = 0x00;
    MDC = 0x00;
    MPER = LOOPTIME_TCY;
    LFSR = 0x00;
    FSCL = 0x00;
    FSMINPER = 0x00;

    /* Combinatorial logic — all disabled */
    CMBTRIG = 0x00;
    LOGCONA = LOGCONB = LOGCONC = LOGCOND = LOGCONE = LOGCONF = 0x00;
    PWMEVTB = PWMEVTC = PWMEVTD = PWMEVTE = PWMEVTF = 0x00;

    PWMEVTA = 0x00;

    /* PG1 (Phase A) — Master */
    PG1CONL = 0x000C;       /* Center-aligned, master clock, ON */
    PG1CONH = 0x4800;       /* UPDMOD=SOC, MPERSEL=enabled (use MPER) */
    PG1STAT = 0x00;
    PG1IOCONL = 0x3000;     /* Start with overrides ON, OVRDAT=00 (float: H off, L off) */
    PG1IOCONH = 0x000E;     /* PENL=1, PENH=1, PMOD=complementary, POLH=active-low */
    /* PG1EVTL = 0x0118 (DS70005539 §14.3.13):
     *   ADTR1PS=0      1:1 postscale (ADC every PWM cycle, needed for
     *                  the BEMF midpoint sampler at >100k eRPM)
     *   ADTR1EN1=1     trigger ADC from PGxTRIGA
     *   UPDTRG=0b11    writing TRIGA auto-sets UPDREQ
     *   PGTRGSEL=0b000 trigger output = EOC */
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
    /* AK center-aligned PWM uses two timer cycles per pulse (DS70005539
     * §14.3.24, §14.4.2.2.4). PG1TRIGA[19:0] is the compare value and
     * bit 31 = CAHALF selects which cycle the trigger fires in.
     *
     * Why CAHALF=0, TRIGA=0: that fires at counter=0 in cycle 1, the
     * period boundary geometrically midway between adjacent PWM pulses
     * = MID-OFF. In complementary mode the LS FETs are conducting and
     * di/dt has settled — comp output is a clean polarity indicator.
     * Mid-ON sampling polluted the BEMF read (pR_pct=90%, POST state
     * dominated by inductive coupling). */
    PG1TRIGA = 0x00000000UL;   /* CAHALF=0, TRIGA=0 — period boundary = MID-OFF */
    PG1TRIGB = PTG_TRIG_MID_OFF_POS;  /* init at mid-OFF; PTG ISR flips per-fire based on duty */
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

    /* PWM interrupt on PG1 (PWM1IF lives in IFS1/IEC1 on AK). */
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
    /* MIN_OFF = 100 ticks (~500ns).
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

