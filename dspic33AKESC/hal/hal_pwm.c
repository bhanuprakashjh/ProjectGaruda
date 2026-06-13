/**
 * @file hal_pwm.c
 *
 * @brief PWM module configuration for 6-step BLDC commutation.
 * Adapted from AN1292 reference:
 *   - Keeps: PCLKCON, MPER, dead time, complementary mode, PG1 master,
 *            PG2+PG3 slave, bootstrap charging, ADC trigger on PG1TRIGA
 *   - Changes: 24kHz switching frequency, override control for 6-step
 *   - Removes: Single-shunt trigger logic, SVM configuration
 *
 * Definitions in this file are for dsPIC33AK128MC106
 *
 * Component: PWM
 */

#include <xc.h>
#include <stdint.h>

#include "hal_pwm.h"
#include "../garuda_types.h"
#include "../garuda_config.h"

/* External commutation table defined in commutation.c */
extern const COMMUTATION_STEP_T commutationTable[6];

/**
 * @brief Initialize all PWM generators for 6-step BLDC control.
 */
void InitPWMGenerators(void)
{
    PCLKCON      = 0x0000;
    /* PWM Clock Divider: 1:2 */
    PCLKCONbits.DIVSEL = 0;
    /* PWM Master Clock: FPLLO (VCO/2 path) */
    PCLKCONbits.MCLKSEL = 1;
    /* Unlock write-protected registers */
    PCLKCONbits.LOCK = 0;

    /* Initialize Master Phase Register */
    MPHASE       = 0x0000;
    /* Initialize Master Duty Cycle */
    MDC          = 0x0000;
    /* Initialize Master Period Register */
    MPER         = LOOPTIME_TCY;

    /* Initialize FREQUENCY SCALE REGISTER */
    FSCL         = 0x0000;
    FSMINPER     = 0x0000;
    LFSR         = 0x0000;
    CMBTRIG      = 0x0000;
    LOGCONA      = 0x0000;
    LOGCONB      = 0x0000;
    LOGCONC      = 0x0000;
    LOGCOND      = 0x0000;
    LOGCONE      = 0x0000;
    LOGCONF      = 0x0000;
    PWMEVTA      = 0x0000;
    PWMEVTB      = 0x0000;
    PWMEVTC      = 0x0000;
    PWMEVTD      = 0x0000;
    PWMEVTE      = 0x0000;
    PWMEVTF      = 0x0000;

    /* Initialize individual PWM generators */
    InitPWMGenerator1();
    InitPWMGenerator2();
    InitPWMGenerator3();

    /* Start with all outputs overridden LOW */
    InitDutyPWM123Generators();

    /* Clearing and disabling PWM Interrupt */
    _PWM1IF = 0;
    _PWM1IE = 0;
    _PWM1IP = 7;

    /* Enable PWM modules: slaves first, then master */
    PG2CONbits.ON = 1;
    PG3CONbits.ON = 1;
    PG1CONbits.ON = 1;

    /* Charge bootstrap capacitors */
    ChargeBootstrapCapacitors();

    /* Re-assert overrides after bootstrap charging — keep all outputs LOW.
     * Bootstrap leaves overrides released (fine for FOC reference), but our
     * 6-step commutation needs override control: outputs must stay LOW
     * until the state machine enters ESC_ALIGN. */
    InitDutyPWM123Generators();
}

/**
 * @brief Override all PWM outputs LOW at startup.
 */
void InitDutyPWM123Generators(void)
{
#if GARUDA_TARGET_AK512
    /* Set Override Data to 0 (LOW) on all outputs */
    PG3IOCON2bits.OVRDAT = 0;
    PG2IOCON2bits.OVRDAT = 0;
    PG1IOCON2bits.OVRDAT = 0;

    /* Enable override on all H and L outputs */
    PG3IOCON2bits.OVRENH = 1;
    PG3IOCON2bits.OVRENL = 1;
    PG2IOCON2bits.OVRENH = 1;
    PG2IOCON2bits.OVRENL = 1;
    PG1IOCON2bits.OVRENH = 1;
    PG1IOCON2bits.OVRENL = 1;
#else
    /* Set Override Data to 0 (LOW) on all outputs */
    PG3IOCONbits.OVRDAT = 0;
    PG2IOCONbits.OVRDAT = 0;
    PG1IOCONbits.OVRDAT = 0;

    /* Enable override on all H and L outputs */
    PG3IOCONbits.OVRENH = 1;
    PG3IOCONbits.OVRENL = 1;
    PG2IOCONbits.OVRENH = 1;
    PG2IOCONbits.OVRENL = 1;
    PG1IOCONbits.OVRENH = 1;
    PG1IOCONbits.OVRENL = 1;
#endif

    /* Set all duty cycles to zero */
    PG3DC = 0;
    PG2DC = 0;
    PG1DC = 0;
}

/**
 * @brief Charge bootstrap capacitors by pulsing low-side switches.
 */
void ChargeBootstrapCapacitors(void)
{
    uint32_t i = BOOTSTRAP_CHARGING_COUNTS;
    uint8_t prevStatusCAHALF = 0, currStatusCAHALF = 0;

#if GARUDA_TARGET_AK512
    /* Override H-side LOW, let L-side run from PWM generator */
    PG3IOCON2bits.OVRDAT = 0;
    PG2IOCON2bits.OVRDAT = 0;
    PG1IOCON2bits.OVRDAT = 0;

    PG3IOCON2bits.OVRENH = 1;
    PG2IOCON2bits.OVRENH = 1;
    PG1IOCON2bits.OVRENH = 1;

    PG3IOCON2bits.OVRENL = 1;
    PG2IOCON2bits.OVRENL = 1;
    PG1IOCON2bits.OVRENL = 1;
#else
    /* Override H-side LOW, let L-side run from PWM generator */
    PG3IOCONbits.OVRDAT = 0;
    PG2IOCONbits.OVRDAT = 0;
    PG1IOCONbits.OVRDAT = 0;

    PG3IOCONbits.OVRENH = 1;
    PG2IOCONbits.OVRENH = 1;
    PG1IOCONbits.OVRENH = 1;

    PG3IOCONbits.OVRENL = 1;
    PG2IOCONbits.OVRENL = 1;
    PG1IOCONbits.OVRENL = 1;
#endif

    /* Set tickle charge duty */
    PWM_PHASE3 = TICKLE_CHARGE_DUTY;
    PWM_PHASE2 = TICKLE_CHARGE_DUTY;
    PWM_PHASE1 = TICKLE_CHARGE_DUTY;
    PWM_PDC3 = TICKLE_CHARGE_DUTY;
    PWM_PDC2 = TICKLE_CHARGE_DUTY;
    PWM_PDC1 = TICKLE_CHARGE_DUTY;

    /* Release L-side override to allow charging */
#if GARUDA_TARGET_AK512
    PG1IOCON2bits.OVRENL = 0;
    PG2IOCON2bits.OVRENL = 0;
    PG3IOCON2bits.OVRENL = 0;
#else
    PG1IOCONbits.OVRENL = 0;
    PG2IOCONbits.OVRENL = 0;
    PG3IOCONbits.OVRENL = 0;
#endif

    /* Wait for bootstrap charging time */
    while (i)
    {
        prevStatusCAHALF = currStatusCAHALF;
        currStatusCAHALF = PG1STATbits.CAHALF;
        if (prevStatusCAHALF != currStatusCAHALF)
        {
            if (currStatusCAHALF == 0)
            {
                i--;
            }
        }
    }

    /* Reset duty cycles */
    PWM_PHASE3 = 0;
    PWM_PHASE2 = 0;
    PWM_PHASE1 = 0;
    PWM_PDC3 = 0;
    PWM_PDC2 = 0;
    PWM_PDC1 = 0;

    /* Release H-side override */
#if GARUDA_TARGET_AK512
    PG3IOCON2bits.OVRENH = 0;
    PG2IOCON2bits.OVRENH = 0;
    PG1IOCON2bits.OVRENH = 0;
#else
    PG3IOCONbits.OVRENH = 0;
    PG2IOCONbits.OVRENH = 0;
    PG1IOCONbits.OVRENH = 0;
#endif
}

/**
 * @brief Configure PWM Generator 1 (Phase A) — Master.
 */
void InitPWMGenerator1(void)
{
    PG1CON      = 0x0000;
    PG1CONbits.ON = 0;
    PG1CONbits.CLKSEL = 1;         /* Master clock */
    PG1CONbits.MODSEL = 4;         /* Center-Aligned PWM */
    PG1CONbits.TRGCNT = 0;
    PG1CONbits.MDCSEL = 0;         /* Use PG1DC */
    PG1CONbits.MPERSEL = 1;        /* Use MPER */
    PG1CONbits.MPHSEL = 0;
    PG1CONbits.MSTEN = 1;          /* Master — broadcasts EOC */
    PG1CONbits.UPDMOD = 0;
    PG1CONbits.TRGMOD = 0;
    PG1CONbits.SOCS = 0;           /* Local EOC */

    PG1STAT     = 0x0000;
#if GARUDA_TARGET_AK512
    PG1IOCON1   = 0x0000;
    PG1IOCON2   = 0x0000;

    PG1IOCON2bits.CLMOD = 0;
    PG1IOCON1bits.SWAP = 0;
    PG1IOCON2bits.OVRENH = 1;      /* Start with override enabled */
    PG1IOCON2bits.OVRENL = 1;
    PG1IOCON2bits.OVRDAT = 0;      /* Override data = LOW */
    PG1IOCON2bits.OSYNC = 0;
    PG1IOCON2bits.FLT1DAT = 0;
    PG1IOCON2bits.CLDAT = 0;
    PG1IOCON2bits.FFDAT = 0;
    PG1IOCON2bits.DBDAT = 0;

    PG1IOCON1bits.CAPSRC = 0;
    PG1IOCON1bits.DTCMPSEL = 0;
    PG1IOCON1bits.PMOD = 0;        /* Complementary mode */
    PG1IOCON1bits.PENH = 1;
    PG1IOCON1bits.PENL = 1;
    PG1IOCON1bits.POLH = 0;
    PG1IOCON1bits.POLL = 0;

    /* Event registers */
    PG1EVT1     = 0x0000;
    PG1EVT2     = 0x0000;
    PG1EVT1bits.ADTR1PS = 0;
    PG1EVT1bits.ADTR1EN3 = 0;
    PG1EVT1bits.ADTR1EN2 = 0;
    PG1EVT1bits.ADTR1EN1 = 1;      /* PG1TRIGA triggers ADC */
    PG1EVT1bits.UPDTRG = 1;        /* DC write triggers UPDATE */
    PG1EVT1bits.PGTRGSEL = 0;
#ifdef ENABLE_PWM_FAULT_PCI
    PG1EVT1bits.FLT1IEN = 1;       /* Fault interrupt enabled (6-step ISR) */
#endif
    PG1EVT1bits.CLIEN = 0;
    PG1EVT1bits.FFIEN = 0;
    PG1EVT1bits.SIEN = 0;
    PG1EVT1bits.IEVTSEL = 3;       /* Time base interrupts disabled */
    PG1EVT2bits.ADTR2EN3 = 0;
    PG1EVT2bits.ADTR2EN2 = 0;
    PG1EVT2bits.ADTR2EN1 = 0;
    PG1EVT1bits.ADTR1OFS = 0;
#else
    PG1IOCON    = 0x0000;

    PG1IOCONbits.CLMOD = 0;
    PG1IOCONbits.SWAP = 0;
    PG1IOCONbits.OVRENH = 1;       /* Start with override enabled */
    PG1IOCONbits.OVRENL = 1;
    PG1IOCONbits.OVRDAT = 0;       /* Override data = LOW */
    PG1IOCONbits.OSYNC = 0;
    PG1IOCONbits.FLTDAT = 0;
    PG1IOCONbits.CLDAT = 0;
    PG1IOCONbits.FFDAT = 0;
    PG1IOCONbits.DBDAT = 0;

    PG1IOCONbits.CAPSRC = 0;
    PG1IOCONbits.DTCMPSEL = 0;
    PG1IOCONbits.PMOD = 0;         /* Complementary mode */
    PG1IOCONbits.PENH = 1;
    PG1IOCONbits.PENL = 1;
    PG1IOCONbits.POLH = 0;
    PG1IOCONbits.POLL = 0;

    /* Event register */
    PG1EVT      = 0x0000;
    PG1EVTbits.ADTR1PS = 0;
    PG1EVTbits.ADTR1EN3 = 0;
    PG1EVTbits.ADTR1EN2 = 0;
    PG1EVTbits.ADTR1EN1 = 1;       /* PG1TRIGA triggers ADC */
    PG1EVTbits.UPDTRG = 1;         /* DC write triggers UPDATE */
    PG1EVTbits.PGTRGSEL = 0;
#ifdef ENABLE_PWM_FAULT_PCI
    PG1EVTbits.FLTIEN = 1;         /* Fault interrupt enabled (6-step ISR) */
#endif
    PG1EVTbits.CLIEN = 0;
    PG1EVTbits.FFIEN = 0;
    PG1EVTbits.SIEN = 0;
    PG1EVTbits.IEVTSEL = 3;        /* Time base interrupts disabled */
    PG1EVTbits.ADTR2EN3 = 0;
    PG1EVTbits.ADTR2EN2 = 0;
    PG1EVTbits.ADTR2EN1 = 0;
    PG1EVTbits.ADTR1OFS = 0;
#endif

    /* Fault PCI — external overcurrent via PCI8 pin (RB11/RP28).
     *
     * LEB-gated acceptance: the FPCI input is only accepted when LEB is LOW
     * (AQSS=010, AQPS=1). During the LEB window after each PWM edge (see
     * PGxLEB below), the board's U25B analog comparator ringing / FET
     * switching transients are ignored. After LEB expires, FPCI is fully
     * active and latches (ACP=3) on any real overcurrent. This suppresses
     * the BOARD_PCI trips we see on 2810 @ 24V where commutation transients
     * peak above U25B's fixed HW threshold for ~1µs after each edge. */
#ifdef ENABLE_PWM_FAULT_PCI
#if GARUDA_TARGET_AK512
    PG1F1PCI1   = 0x0000;
    PG1F1PCI2   = 0x0000;
    PG1F1PCI1bits.TSYNCDIS = 0;
    PG1F1PCI1bits.TERM = 1;         /* Auto-terminate when qualifier goes false */
    PG1F1PCI1bits.AQPS = 1;         /* Inverted: accept when LEB LOW (post-blanking) */
    PG1F1PCI1bits.AQSS = 0b010;     /* LEB active as acceptance qualifier */
    PG1F1PCI1bits.PSYNC = 0;
    PG1F1PCI1bits.PPS = 1;          /* Inverted polarity */
    PG1F1PCI2 = 0x00000100UL; /* PCI source one-hot: bit8 = PPS PCI8R (board FLTLAT_OC_OV, = AK128 PSS 0b01000) */
    PG1F1PCI1bits.BPEN = 0;
    PG1F1PCI1bits.BPSEL = 0;
    PG1F1PCI1bits.TERMPS = 0;
    PG1F1PCI1bits.ACP = 3;          /* Latched */
    PG1F1PCI1bits.LATMOD = 0;
    PG1F1PCI1bits.TQPS = 0;
    PG1F1PCI1bits.TQSS = 0;
#if FEATURE_HW_OVERCURRENT && OC_PROTECT_MODE == 1
    PG1F1PCI2 = 0x40000000UL; /* PCI source one-hot: bit30 = Comparator 3 output (= AK128 PSS 0b11101 CMP3) */
    PG1F1PCI1bits.PPS = 0;          /* Non-inverted */
#endif
#else
    PG1FPCI     = 0x0000;
    PG1FPCIbits.TSYNCDIS = 0;
    PG1FPCIbits.TERM = 1;           /* Auto-terminate when qualifier goes false */
    PG1FPCIbits.AQPS = 1;           /* Inverted: accept when LEB LOW (post-blanking) */
    PG1FPCIbits.AQSS = 0b010;       /* LEB active as acceptance qualifier */
    PG1FPCIbits.PSYNC = 0;
    PG1FPCIbits.PPS = 1;            /* Inverted polarity */
    PG1FPCIbits.PSS = 0b01000;      /* RPn input (PCI8R = board U25B) */
    PG1FPCIbits.BPEN = 0;
    PG1FPCIbits.BPSEL = 0;
    PG1FPCIbits.TERMPS = 0;
    PG1FPCIbits.ACP = 3;            /* Latched */
    PG1FPCIbits.LATMOD = 0;
    PG1FPCIbits.TQPS = 0;
    PG1FPCIbits.TQSS = 0;
#if FEATURE_HW_OVERCURRENT && OC_PROTECT_MODE == 1
    PG1FPCIbits.PSS = 0b11101;      /* CMP3 (replaces RPn/PCI8R) */
    PG1FPCIbits.PPS = 0;            /* Non-inverted */
#endif
#endif /* GARUDA_TARGET_AK512 */
#else
    /* FOC mode: FPCI truly disabled via acceptance qualifier gating.
     * Board U25B shunt comparator has no LEB — SVM switching transients
     * false-trip U25B → board fault latches → FPCI kills outputs.
     * Fix: ACP=1 (edge-sensitive, requires qualifier), AQSS=010 (LEB as
     * qualifier), AQPS=0. Since PGxLEB=0 in FOC, LEB never asserts →
     * qualifier never met → fault NEVER accepted. Software OC via ADC
     * ibusRaw is the primary protection in FOC mode. */
#if GARUDA_TARGET_AK512
    PG1F1PCI1   = 0x0000;
    PG1F1PCI2   = 0x0000;
    PG1F1PCI1bits.ACP  = 1;     /* Edge-sensitive (requires qualifier) */
    PG1F1PCI1bits.AQSS = 0b010; /* LEB active as acceptance qualifier */
    PG1F1PCI1bits.AQPS = 0;     /* Non-inverted: need LEB HIGH to accept */
    PG1F1PCI1bits.TERM = 1;     /* Safety: auto-terminate if somehow accepted */
#else
    PG1FPCI     = 0x0000;
    PG1FPCIbits.ACP  = 1;       /* Edge-sensitive (requires qualifier) */
    PG1FPCIbits.AQSS = 0b010;   /* LEB active as acceptance qualifier */
    PG1FPCIbits.AQPS = 0;       /* Non-inverted: need LEB HIGH to accept */
    PG1FPCIbits.TERM = 1;       /* Safety: auto-terminate if somehow accepted */
#endif /* GARUDA_TARGET_AK512 */
#endif

#if FEATURE_HW_OVERCURRENT && (OC_PROTECT_MODE == 0 || OC_PROTECT_MODE == 2) && OC_CLPCI_ENABLE
#if GARUDA_TARGET_AK512
    PG1CLPCI1   = 0x0000;
    PG1CLPCI2 = 0x40000000UL; /* PCI source one-hot: bit30 = Comparator 3 output (= AK128 PSS 0b11101 CMP3) */
    PG1CLPCI1bits.PPS = 0;         /* Non-inverted (CMP3 high = overcurrent) */
    PG1CLPCI1bits.TERM = 1;        /* Auto-terminate when PCI source goes inactive */
    PG1CLPCI1bits.ACP = 0b011;    /* Latched acceptance (recommended with LEB) */
    PG1CLPCI1bits.PSYNC = 0;
    PG1CLPCI1bits.AQSS = 0b010;   /* LEB active as acceptance qualifier */
    PG1CLPCI1bits.AQPS = 1;       /* Inverted: accept only when LEB inactive */
#else
    PG1CLPCI    = 0x0000;
    PG1CLPCIbits.PSS = 0b11101;    /* Comparator 3 output */
    PG1CLPCIbits.PPS = 0;          /* Non-inverted (CMP3 high = overcurrent) */
    PG1CLPCIbits.TERM = 1;         /* Auto-terminate when PCI source goes inactive */
    PG1CLPCIbits.ACP = 0b011;     /* Latched acceptance (recommended with LEB) */
    PG1CLPCIbits.PSYNC = 0;
    PG1CLPCIbits.AQSS = 0b010;    /* LEB active as acceptance qualifier */
    PG1CLPCIbits.AQPS = 1;        /* Inverted: accept only when LEB inactive */
#endif /* GARUDA_TARGET_AK512 */
    /* CLIEN stays 0 — no ISR. CLPCI protection is pure hardware.
     * Trip counting uses CLEVT polling in ADC ISR. */
#else
#if GARUDA_TARGET_AK512
    PG1CLPCI1   = 0x0000;
    PG1CLPCI2   = 0x0000;
#else
    PG1CLPCI    = 0x0000;
#endif /* GARUDA_TARGET_AK512 */
#endif
#if GARUDA_TARGET_AK512
    PG1FFPCI1   = 0x0000;
    PG1FFPCI2   = 0x0000;
    PG1SPCI1    = 0x0000;
    PG1SPCI2    = 0x0000;
#else
    PG1FFPCI    = 0x0000;
    PG1SPCI     = 0x0000;
#endif
    /* LEB counter — drives the FPCI qualifier above (and CLPCI if enabled).
     * Runs on every H/L edge, so any CMP3 / board-U25B transient in the
     * first OC_LEB_COUNTS PWM clocks after a switch edge is ignored. */
#if FEATURE_HW_OVERCURRENT
    PG1LEB      = 0x0000;
    PG1LEBbits.PHR = 1;           /* Blank on rising edge of PWMxH */
    PG1LEBbits.PHF = 1;           /* Blank on falling edge of PWMxH */
    PG1LEBbits.PLR = 1;           /* Blank on rising edge of PWMxL */
    PG1LEBbits.PLF = 1;           /* Blank on falling edge of PWMxL */
    PG1LEBbits.LEB = OC_LEB_COUNTS;
#else
    PG1LEB      = 0x0000;
#endif

    PG1PHASEbits.PHASE = MIN_DUTY;
    PG1DCbits.DC = MIN_DUTY;
    PG1DCA      = 0x0000;
    PG1PER      = 0x0000;
    PG1DTbits.DTH = DEADTIME_COUNTS;
    PG1DTbits.DTL = DEADTIME_COUNTS;

    /* CAHALF=0 (cycle-1 half) + TRIGA=MPER/2 = the freewheel-center sample, the
     * empirical optimum (215k). CAHALF=1 trials BOTH made it worse (115k, OC) —
     * see ADC_SAMPLING_POINT for the 4-point fire-time/quality table. */
    PG1TRIGAbits.CAHALF = 0;
    PG1TRIGAbits.TRIGA = ADC_SAMPLING_POINT;
    PG1TRIGB    = 0x0000;
    PG1TRIGC    = 0x0000;
}

/**
 * @brief Configure PWM Generator 2 (Phase B) — Slave to PG1.
 */
void InitPWMGenerator2(void)
{
    PG2CON      = 0x0000;
    PG2CONbits.ON = 0;
    PG2CONbits.CLKSEL = 1;
    PG2CONbits.MODSEL = 4;         /* Center-Aligned PWM */
    PG2CONbits.TRGCNT = 0;
    PG2CONbits.MDCSEL = 0;
    PG2CONbits.MPERSEL = 1;
    PG2CONbits.MPHSEL = 0;
    PG2CONbits.MSTEN = 0;          /* Not master */
    PG2CONbits.UPDMOD = 0b010;     /* Slaved SOC update */
    PG2CONbits.TRGMOD = 0;
    PG2CONbits.SOCS = 1;           /* Triggered by PG1 */

    PG2STAT     = 0x0000;
#if GARUDA_TARGET_AK512
    PG2IOCON1   = 0x0000;
    PG2IOCON2   = 0x0000;

    PG2IOCON2bits.CLMOD = 0;
    PG2IOCON1bits.SWAP = 0;
    PG2IOCON2bits.OVRENH = 1;
    PG2IOCON2bits.OVRENL = 1;
    PG2IOCON2bits.OVRDAT = 0;
    PG2IOCON2bits.OSYNC = 0;
    PG2IOCON2bits.FLT1DAT = 0;
    PG2IOCON2bits.CLDAT = 0;
    PG2IOCON2bits.FFDAT = 0;
    PG2IOCON2bits.DBDAT = 0;

    PG2IOCON1bits.CAPSRC = 0;
    PG2IOCON1bits.DTCMPSEL = 0;
    PG2IOCON1bits.PMOD = 0;
    PG2IOCON1bits.PENH = 1;
    PG2IOCON1bits.PENL = 1;
    PG2IOCON1bits.POLH = 0;
    PG2IOCON1bits.POLL = 0;

    PG2EVT1     = 0x0000;
    PG2EVT2     = 0x0000;
    PG2EVT1bits.ADTR1PS = 0;
    PG2EVT1bits.ADTR1EN3 = 0;
    PG2EVT1bits.ADTR1EN2 = 0;
    PG2EVT1bits.ADTR1EN1 = 0;
    PG2EVT1bits.UPDTRG = 1;        /* DC write triggers UPDATE (fix: slave wasn't loading duty) */
    PG2EVT1bits.PGTRGSEL = 0;
    PG2EVT1bits.FLT1IEN = 0;
    PG2EVT1bits.CLIEN = 0;
    PG2EVT1bits.FFIEN = 0;
    PG2EVT1bits.SIEN = 0;
    PG2EVT1bits.IEVTSEL = 3;
    PG2EVT2bits.ADTR2EN3 = 0;
    PG2EVT2bits.ADTR2EN2 = 0;
    PG2EVT2bits.ADTR2EN1 = 0;
    PG2EVT1bits.ADTR1OFS = 0;
#else
    PG2IOCON    = 0x0000;

    PG2IOCONbits.CLMOD = 0;
    PG2IOCONbits.SWAP = 0;
    PG2IOCONbits.OVRENH = 1;
    PG2IOCONbits.OVRENL = 1;
    PG2IOCONbits.OVRDAT = 0;
    PG2IOCONbits.OSYNC = 0;
    PG2IOCONbits.FLTDAT = 0;
    PG2IOCONbits.CLDAT = 0;
    PG2IOCONbits.FFDAT = 0;
    PG2IOCONbits.DBDAT = 0;

    PG2IOCONbits.CAPSRC = 0;
    PG2IOCONbits.DTCMPSEL = 0;
    PG2IOCONbits.PMOD = 0;
    PG2IOCONbits.PENH = 1;
    PG2IOCONbits.PENL = 1;
    PG2IOCONbits.POLH = 0;
    PG2IOCONbits.POLL = 0;

    PG2EVT      = 0x0000;
    PG2EVTbits.ADTR1PS = 0;
    PG2EVTbits.ADTR1EN3 = 0;
    PG2EVTbits.ADTR1EN2 = 0;
    PG2EVTbits.ADTR1EN1 = 0;
    PG2EVTbits.UPDTRG = 1;         /* DC write triggers UPDATE (fix: slave wasn't loading duty) */
    PG2EVTbits.PGTRGSEL = 0;
    PG2EVTbits.FLTIEN = 0;
    PG2EVTbits.CLIEN = 0;
    PG2EVTbits.FFIEN = 0;
    PG2EVTbits.SIEN = 0;
    PG2EVTbits.IEVTSEL = 3;
    PG2EVTbits.ADTR2EN3 = 0;
    PG2EVTbits.ADTR2EN2 = 0;
    PG2EVTbits.ADTR2EN1 = 0;
    PG2EVTbits.ADTR1OFS = 0;
#endif

#ifdef ENABLE_PWM_FAULT_PCI
    /* LEB-gated FPCI acceptance — see PG1 comment */
#if GARUDA_TARGET_AK512
    PG2F1PCI1   = 0x0000;
    PG2F1PCI2   = 0x0000;
    PG2F1PCI1bits.TSYNCDIS = 0;
    PG2F1PCI1bits.TERM = 1;
    PG2F1PCI1bits.AQPS = 1;         /* Inverted: accept when LEB LOW */
    PG2F1PCI1bits.AQSS = 0b010;     /* LEB as acceptance qualifier */
    PG2F1PCI1bits.PSYNC = 0;
    PG2F1PCI1bits.PPS = 1;
    PG2F1PCI2 = 0x00000100UL; /* PCI source one-hot: bit8 = PPS PCI8R (board FLTLAT_OC_OV, = AK128 PSS 0b01000) */
    PG2F1PCI1bits.BPEN = 0;
    PG2F1PCI1bits.BPSEL = 0;
    PG2F1PCI1bits.TERMPS = 0;
    PG2F1PCI1bits.ACP = 3;
    PG2F1PCI1bits.LATMOD = 0;
    PG2F1PCI1bits.TQPS = 0;
    PG2F1PCI1bits.TQSS = 0;
#if FEATURE_HW_OVERCURRENT && OC_PROTECT_MODE == 1
    PG2F1PCI2 = 0x40000000UL; /* PCI source one-hot: bit30 = Comparator 3 output (= AK128 PSS 0b11101 CMP3) */
    PG2F1PCI1bits.PPS = 0;          /* Non-inverted */
#endif
#else
    PG2FPCI     = 0x0000;
    PG2FPCIbits.TSYNCDIS = 0;
    PG2FPCIbits.TERM = 1;
    PG2FPCIbits.AQPS = 1;           /* Inverted: accept when LEB LOW */
    PG2FPCIbits.AQSS = 0b010;       /* LEB as acceptance qualifier */
    PG2FPCIbits.PSYNC = 0;
    PG2FPCIbits.PPS = 1;
    PG2FPCIbits.PSS = 0b01000;      /* RPn input (PCI8R = board U25B) */
    PG2FPCIbits.BPEN = 0;
    PG2FPCIbits.BPSEL = 0;
    PG2FPCIbits.TERMPS = 0;
    PG2FPCIbits.ACP = 3;
    PG2FPCIbits.LATMOD = 0;
    PG2FPCIbits.TQPS = 0;
    PG2FPCIbits.TQSS = 0;
#if FEATURE_HW_OVERCURRENT && OC_PROTECT_MODE == 1
    PG2FPCIbits.PSS = 0b11101;      /* CMP3 (replaces RPn/PCI8R) */
    PG2FPCIbits.PPS = 0;            /* Non-inverted */
#endif
#endif /* GARUDA_TARGET_AK512 */
#else
    /* FOC: FPCI disabled via LEB qualifier gating (see PG1 comment) */
#if GARUDA_TARGET_AK512
    PG2F1PCI1   = 0x0000;
    PG2F1PCI2   = 0x0000;
    PG2F1PCI1bits.ACP  = 1;
    PG2F1PCI1bits.AQSS = 0b010;
    PG2F1PCI1bits.AQPS = 0;
    PG2F1PCI1bits.TERM = 1;
#else
    PG2FPCI     = 0x0000;
    PG2FPCIbits.ACP  = 1;
    PG2FPCIbits.AQSS = 0b010;
    PG2FPCIbits.AQPS = 0;
    PG2FPCIbits.TERM = 1;
#endif /* GARUDA_TARGET_AK512 */
#endif

#if FEATURE_HW_OVERCURRENT && (OC_PROTECT_MODE == 0 || OC_PROTECT_MODE == 2) && OC_CLPCI_ENABLE
#if GARUDA_TARGET_AK512
    PG2CLPCI1   = 0x0000;
    PG2CLPCI2 = 0x40000000UL; /* PCI source one-hot: bit30 = Comparator 3 output (= AK128 PSS 0b11101 CMP3) */
    PG2CLPCI1bits.PPS = 0;
    PG2CLPCI1bits.TERM = 1;
    PG2CLPCI1bits.ACP = 0b011;    /* Latched acceptance (recommended with LEB) */
    PG2CLPCI1bits.PSYNC = 0;
    PG2CLPCI1bits.AQSS = 0b010;   /* LEB active as acceptance qualifier */
    PG2CLPCI1bits.AQPS = 1;       /* Inverted: accept only when LEB inactive */
#else
    PG2CLPCI    = 0x0000;
    PG2CLPCIbits.PSS = 0b11101;    /* Comparator 3 output */
    PG2CLPCIbits.PPS = 0;
    PG2CLPCIbits.TERM = 1;
    PG2CLPCIbits.ACP = 0b011;     /* Latched acceptance (recommended with LEB) */
    PG2CLPCIbits.PSYNC = 0;
    PG2CLPCIbits.AQSS = 0b010;    /* LEB active as acceptance qualifier */
    PG2CLPCIbits.AQPS = 1;        /* Inverted: accept only when LEB inactive */
#endif /* GARUDA_TARGET_AK512 */
#else
#if GARUDA_TARGET_AK512
    PG2CLPCI1   = 0x0000;
    PG2CLPCI2   = 0x0000;
#else
    PG2CLPCI    = 0x0000;
#endif /* GARUDA_TARGET_AK512 */
#endif
#if GARUDA_TARGET_AK512
    PG2FFPCI1   = 0x0000;
    PG2FFPCI2   = 0x0000;
    PG2SPCI1    = 0x0000;
    PG2SPCI2    = 0x0000;
#else
    PG2FFPCI    = 0x0000;
    PG2SPCI     = 0x0000;
#endif
    /* LEB counter — see PG1 comment. Required for FPCI qualifier. */
#if FEATURE_HW_OVERCURRENT
    PG2LEB      = 0x0000;
    PG2LEBbits.PHR = 1;
    PG2LEBbits.PHF = 1;
    PG2LEBbits.PLR = 1;
    PG2LEBbits.PLF = 1;
    PG2LEBbits.LEB = OC_LEB_COUNTS;
#else
    PG2LEB      = 0x0000;
#endif

    PG2PHASEbits.PHASE = MIN_DUTY;
    PG2DCbits.DC = MIN_DUTY;
    PG2DCA      = 0x0000;
    PG2PER      = 0x0000;
    PG2DTbits.DTH = DEADTIME_COUNTS;
    PG2DTbits.DTL = DEADTIME_COUNTS;

    PG2TRIGA    = 0x0000;
    PG2TRIGB    = 0x0000;
    PG2TRIGC    = 0x0000;
}

/**
 * @brief Configure PWM Generator 3 (Phase C) — Slave to PG1.
 */
void InitPWMGenerator3(void)
{
    PG3CON      = 0x0000;
    PG3CONbits.ON = 0;
    PG3CONbits.CLKSEL = 1;
    PG3CONbits.MODSEL = 4;
    PG3CONbits.TRGCNT = 0;
    PG3CONbits.MDCSEL = 0;
    PG3CONbits.MPERSEL = 1;
    PG3CONbits.MPHSEL = 0;
    PG3CONbits.MSTEN = 0;
    PG3CONbits.UPDMOD = 0b010;
    PG3CONbits.TRGMOD = 0;
    PG3CONbits.SOCS = 1;

    PG3STAT     = 0x0000;
#if GARUDA_TARGET_AK512
    PG3IOCON1   = 0x0000;
    PG3IOCON2   = 0x0000;

    PG3IOCON2bits.CLMOD = 0;
    PG3IOCON1bits.SWAP = 0;
    PG3IOCON2bits.OVRENH = 1;
    PG3IOCON2bits.OVRENL = 1;
    PG3IOCON2bits.OVRDAT = 0;
    PG3IOCON2bits.OSYNC = 0;
    PG3IOCON2bits.FLT1DAT = 0;
    PG3IOCON2bits.CLDAT = 0;
    PG3IOCON2bits.FFDAT = 0;
    PG3IOCON2bits.DBDAT = 0;

    PG3IOCON1bits.CAPSRC = 0;
    PG3IOCON1bits.DTCMPSEL = 0;
    PG3IOCON1bits.PMOD = 0;
    PG3IOCON1bits.PENH = 1;
    PG3IOCON1bits.PENL = 1;
    PG3IOCON1bits.POLH = 0;
    PG3IOCON1bits.POLL = 0;

    PG3EVT1     = 0x0000;
    PG3EVT2     = 0x0000;
    PG3EVT1bits.ADTR1PS = 0;
    PG3EVT1bits.ADTR1EN3 = 0;
    PG3EVT1bits.ADTR1EN2 = 0;
    PG3EVT1bits.ADTR1EN1 = 0;
    PG3EVT1bits.UPDTRG = 1;        /* DC write triggers UPDATE (fix: slave wasn't loading duty) */
    PG3EVT1bits.PGTRGSEL = 0;
    PG3EVT1bits.FLT1IEN = 0;
    PG3EVT1bits.CLIEN = 0;
    PG3EVT1bits.FFIEN = 0;
    PG3EVT1bits.SIEN = 0;
    PG3EVT1bits.IEVTSEL = 3;
    PG3EVT2bits.ADTR2EN3 = 0;
    PG3EVT2bits.ADTR2EN2 = 0;
    PG3EVT2bits.ADTR2EN1 = 0;
    PG3EVT1bits.ADTR1OFS = 0;
#else
    PG3IOCON    = 0x0000;

    PG3IOCONbits.CLMOD = 0;
    PG3IOCONbits.SWAP = 0;
    PG3IOCONbits.OVRENH = 1;
    PG3IOCONbits.OVRENL = 1;
    PG3IOCONbits.OVRDAT = 0;
    PG3IOCONbits.OSYNC = 0;
    PG3IOCONbits.FLTDAT = 0;
    PG3IOCONbits.CLDAT = 0;
    PG3IOCONbits.FFDAT = 0;
    PG3IOCONbits.DBDAT = 0;

    PG3IOCONbits.CAPSRC = 0;
    PG3IOCONbits.DTCMPSEL = 0;
    PG3IOCONbits.PMOD = 0;
    PG3IOCONbits.PENH = 1;
    PG3IOCONbits.PENL = 1;
    PG3IOCONbits.POLH = 0;
    PG3IOCONbits.POLL = 0;

    PG3EVT      = 0x0000;
    PG3EVTbits.ADTR1PS = 0;
    PG3EVTbits.ADTR1EN3 = 0;
    PG3EVTbits.ADTR1EN2 = 0;
    PG3EVTbits.ADTR1EN1 = 0;
    PG3EVTbits.UPDTRG = 1;         /* DC write triggers UPDATE (fix: slave wasn't loading duty) */
    PG3EVTbits.PGTRGSEL = 0;
    PG3EVTbits.FLTIEN = 0;
    PG3EVTbits.CLIEN = 0;
    PG3EVTbits.FFIEN = 0;
    PG3EVTbits.SIEN = 0;
    PG3EVTbits.IEVTSEL = 3;
    PG3EVTbits.ADTR2EN3 = 0;
    PG3EVTbits.ADTR2EN2 = 0;
    PG3EVTbits.ADTR2EN1 = 0;
    PG3EVTbits.ADTR1OFS = 0;
#endif

#ifdef ENABLE_PWM_FAULT_PCI
    /* LEB-gated FPCI acceptance — see PG1 comment */
#if GARUDA_TARGET_AK512
    PG3F1PCI1   = 0x0000;
    PG3F1PCI2   = 0x0000;
    PG3F1PCI1bits.TSYNCDIS = 0;
    PG3F1PCI1bits.TERM = 1;
    PG3F1PCI1bits.AQPS = 1;         /* Inverted: accept when LEB LOW */
    PG3F1PCI1bits.AQSS = 0b010;     /* LEB as acceptance qualifier */
    PG3F1PCI1bits.PSYNC = 0;
    PG3F1PCI1bits.PPS = 1;
    PG3F1PCI2 = 0x00000100UL; /* PCI source one-hot: bit8 = PPS PCI8R (board FLTLAT_OC_OV, = AK128 PSS 0b01000) */
    PG3F1PCI1bits.BPEN = 0;
    PG3F1PCI1bits.BPSEL = 0;
    PG3F1PCI1bits.TERMPS = 0;
    PG3F1PCI1bits.ACP = 3;
    PG3F1PCI1bits.LATMOD = 0;
    PG3F1PCI1bits.TQPS = 0;
    PG3F1PCI1bits.TQSS = 0;
#if FEATURE_HW_OVERCURRENT && OC_PROTECT_MODE == 1
    PG3F1PCI2 = 0x40000000UL; /* PCI source one-hot: bit30 = Comparator 3 output (= AK128 PSS 0b11101 CMP3) */
    PG3F1PCI1bits.PPS = 0;          /* Non-inverted */
#endif
#else
    PG3FPCI     = 0x0000;
    PG3FPCIbits.TSYNCDIS = 0;
    PG3FPCIbits.TERM = 1;
    PG3FPCIbits.AQPS = 1;           /* Inverted: accept when LEB LOW */
    PG3FPCIbits.AQSS = 0b010;       /* LEB as acceptance qualifier */
    PG3FPCIbits.PSYNC = 0;
    PG3FPCIbits.PPS = 1;
    PG3FPCIbits.PSS = 0b01000;      /* RPn input (PCI8R = board U25B) */
    PG3FPCIbits.BPEN = 0;
    PG3FPCIbits.BPSEL = 0;
    PG3FPCIbits.TERMPS = 0;
    PG3FPCIbits.ACP = 3;
    PG3FPCIbits.LATMOD = 0;
    PG3FPCIbits.TQPS = 0;
    PG3FPCIbits.TQSS = 0;
#if FEATURE_HW_OVERCURRENT && OC_PROTECT_MODE == 1
    PG3FPCIbits.PSS = 0b11101;      /* CMP3 (replaces RPn/PCI8R) */
    PG3FPCIbits.PPS = 0;            /* Non-inverted */
#endif
#endif /* GARUDA_TARGET_AK512 */
#else
    /* FOC: FPCI disabled via LEB qualifier gating (see PG1 comment) */
#if GARUDA_TARGET_AK512
    PG3F1PCI1   = 0x0000;
    PG3F1PCI2   = 0x0000;
    PG3F1PCI1bits.ACP  = 1;
    PG3F1PCI1bits.AQSS = 0b010;
    PG3F1PCI1bits.AQPS = 0;
    PG3F1PCI1bits.TERM = 1;
#else
    PG3FPCI     = 0x0000;
    PG3FPCIbits.ACP  = 1;
    PG3FPCIbits.AQSS = 0b010;
    PG3FPCIbits.AQPS = 0;
    PG3FPCIbits.TERM = 1;
#endif /* GARUDA_TARGET_AK512 */
#endif

#if FEATURE_HW_OVERCURRENT && (OC_PROTECT_MODE == 0 || OC_PROTECT_MODE == 2) && OC_CLPCI_ENABLE
#if GARUDA_TARGET_AK512
    PG3CLPCI1   = 0x0000;
    PG3CLPCI2 = 0x40000000UL; /* PCI source one-hot: bit30 = Comparator 3 output (= AK128 PSS 0b11101 CMP3) */
    PG3CLPCI1bits.PPS = 0;
    PG3CLPCI1bits.TERM = 1;
    PG3CLPCI1bits.ACP = 0b011;    /* Latched acceptance (recommended with LEB) */
    PG3CLPCI1bits.PSYNC = 0;
    PG3CLPCI1bits.AQSS = 0b010;   /* LEB active as acceptance qualifier */
    PG3CLPCI1bits.AQPS = 1;       /* Inverted: accept only when LEB inactive */
#else
    PG3CLPCI    = 0x0000;
    PG3CLPCIbits.PSS = 0b11101;    /* Comparator 3 output */
    PG3CLPCIbits.PPS = 0;
    PG3CLPCIbits.TERM = 1;
    PG3CLPCIbits.ACP = 0b011;     /* Latched acceptance (recommended with LEB) */
    PG3CLPCIbits.PSYNC = 0;
    PG3CLPCIbits.AQSS = 0b010;    /* LEB active as acceptance qualifier */
    PG3CLPCIbits.AQPS = 1;        /* Inverted: accept only when LEB inactive */
#endif /* GARUDA_TARGET_AK512 */
#else
#if GARUDA_TARGET_AK512
    PG3CLPCI1   = 0x0000;
    PG3CLPCI2   = 0x0000;
#else
    PG3CLPCI    = 0x0000;
#endif /* GARUDA_TARGET_AK512 */
#endif
#if GARUDA_TARGET_AK512
    PG3FFPCI1   = 0x0000;
    PG3FFPCI2   = 0x0000;
    PG3SPCI1    = 0x0000;
    PG3SPCI2    = 0x0000;
#else
    PG3FFPCI    = 0x0000;
    PG3SPCI     = 0x0000;
#endif
    /* LEB counter — see PG1 comment. Required for FPCI qualifier. */
#if FEATURE_HW_OVERCURRENT
    PG3LEB      = 0x0000;
    PG3LEBbits.PHR = 1;
    PG3LEBbits.PHF = 1;
    PG3LEBbits.PLR = 1;
    PG3LEBbits.PLF = 1;
    PG3LEBbits.LEB = OC_LEB_COUNTS;
#else
    PG3LEB      = 0x0000;
#endif

    PG3PHASEbits.PHASE = MIN_DUTY;
    PG3DCbits.DC = MIN_DUTY;
    PG3DCA      = 0x0000;
    PG3PER      = 0x0000;
    PG3DTbits.DTH = DEADTIME_COUNTS;
    PG3DTbits.DTL = DEADTIME_COUNTS;

    PG3TRIGA    = 0x0000;
    PG3TRIGB    = 0x0000;
    PG3TRIGC    = 0x0000;
}

/* Precomputed full-word PGxIOCON patterns for 6-step commutation.
 *
 * Preserves init-time bits PENH=1 (bit19) and PENL=1 (bit18) → 0x000C0000.
 * Lower 16 bits select the override mode:
 *   OVRENH  = bit13
 *   OVRENL  = bit12
 *   OVRDAT  = bits[11:10]  (bit11 = H data, bit10 = L data)
 *
 * Atomic 32-bit SFR write = 1 instruction cycle (10 ns).
 * Replaces 2-3 read-modify-write bit-field ops (~30 ns each) with a single
 * store, cutting inter-phase skew from ~90 ns to ~30 ns across all three PGs.
 * The reduced skew is what prevents 24 V · (di/dt) transient spikes from
 * crossing the 22 A board fault threshold during a sector transition.
 */
#if GARUDA_TARGET_AK512
/* MC510: PGxIOCON is split — PENH/PENL live in PGxIOCON1 (written once at
 * init, never touched during commutation), while all override fields live in
 * PGxIOCON2: OVRENH=bit21, OVRENL=bit20, OVRDAT=bits[13:12] (bit13=H, bit12=L).
 * A single atomic 32-bit write to PGxIOCON2 preserves the low-skew property. */
#define PG_IOCON_PWM_ACTIVE  0x00000000U                            /* ENH=0 ENL=0 */
#define PG_IOCON_LOW         0x00301000U                            /* ENH=1 ENL=1 OVRDAT=01 (H=LOW, L=HIGH sink) */
#define PG_IOCON_FLOAT       0x00300000U                            /* ENH=1 ENL=1 OVRDAT=00 (both LOW, high-Z) */
#else
#define PG_IOCON_KEEP        0x000C0000U  /* PENH=1, PENL=1 */
#define PG_IOCON_PWM_ACTIVE  (PG_IOCON_KEEP | 0x00000000U)          /* ENH=0 ENL=0 */
#define PG_IOCON_LOW         (PG_IOCON_KEEP | 0x00003400U)          /* ENH=1 ENL=1 OVRDAT=01 (H=LOW, L=HIGH sink) */
#define PG_IOCON_FLOAT       (PG_IOCON_KEEP | 0x00003000U)          /* ENH=1 ENL=1 OVRDAT=00 (both LOW, high-Z) */
#endif

static inline uint32_t pgIoconWord(uint8_t mode)
{
    switch (mode)
    {
        case PHASE_PWM_ACTIVE: return PG_IOCON_PWM_ACTIVE;
        case PHASE_LOW:        return PG_IOCON_LOW;
        case PHASE_FLOAT:      return PG_IOCON_FLOAT;
        default:               return PG_IOCON_FLOAT;
    }
}

/**
 * @brief Apply a 6-step commutation pattern to PWM override registers.
 *
 * Writes PG1/PG2/PG3 IOCON atomically (single 32-bit SFR store each) to
 * minimise the window of intermediate invalid phase configurations during
 * sector changes. See PG_IOCON_* macros for the encoding.
 *
 * @param step Commutation step index 0-5
 */
#if FEATURE_CL_DIFF_IDLE
/* Differential-low CL drive (see garuda_config.h). When set, the LOW phase of
 * the current step runs complementary PWM at MIN_DUTY instead of override-LOW,
 * so the line-line voltage is (duty − base) — controllable below the MIN_DUTY
 * waveform floor. Cleared by HAL_MC1PWMDisableOutputs and STARTUP_Init. */
volatile uint8_t g_pwmDiffLow = 0;
static volatile uint8_t s_diffStep = 0;   /* current step for per-phase PDC writes */
#endif

void HAL_PWM_SetCommutationStep(uint8_t step)
{
    const COMMUTATION_STEP_T *s = &commutationTable[step];

#if FEATURE_CL_DIFF_IDLE
    s_diffStep = step;
    if (g_pwmDiffLow)
    {
        /* LOW phase → complementary PWM (its PDC carries the MIN_DUTY base,
         * written by HAL_PWM_SetDutyCycle). Active/float roles unchanged. */
#if GARUDA_TARGET_AK512
        PG1IOCON2 = pgIoconWord((s->phaseA == PHASE_LOW) ? PHASE_PWM_ACTIVE : s->phaseA);
        PG2IOCON2 = pgIoconWord((s->phaseB == PHASE_LOW) ? PHASE_PWM_ACTIVE : s->phaseB);
        PG3IOCON2 = pgIoconWord((s->phaseC == PHASE_LOW) ? PHASE_PWM_ACTIVE : s->phaseC);
#else
        PG1IOCON = pgIoconWord((s->phaseA == PHASE_LOW) ? PHASE_PWM_ACTIVE : s->phaseA);
        PG2IOCON = pgIoconWord((s->phaseB == PHASE_LOW) ? PHASE_PWM_ACTIVE : s->phaseB);
        PG3IOCON = pgIoconWord((s->phaseC == PHASE_LOW) ? PHASE_PWM_ACTIVE : s->phaseC);
#endif
        return;
    }
#endif
#if GARUDA_TARGET_AK512
    PG1IOCON2 = pgIoconWord(s->phaseA);
    PG2IOCON2 = pgIoconWord(s->phaseB);
    PG3IOCON2 = pgIoconWord(s->phaseC);
#else
    PG1IOCON = pgIoconWord(s->phaseA);
    PG2IOCON = pgIoconWord(s->phaseB);
    PG3IOCON = pgIoconWord(s->phaseC);
#endif
}

/**
 * @brief Set the PWM duty cycle on all three generators.
 * Only the active (PWM-driven) phase will produce output;
 * the other phases are held by override registers.
 *
 * @param duty Duty cycle in PWM clock counts
 */
void HAL_PWM_SetDutyCycle(uint32_t duty)
{
#if FEATURE_CL_DIFF_IDLE
    if (g_pwmDiffLow)
    {
        /* duty = EFFECTIVE duty (may be < MIN_DUTY). Active phase carries
         * MIN_DUTY + duty + deadtime compensation, low phase the MIN_DUTY base.
         * DEADTIME COMPENSATION (bench 2026-06-10): the two driven phases carry
         * OPPOSITE currents (active sources, low sinks — fixed in 6-step), so
         * their deadtime voltage errors ADD against the differential instead of
         * cancelling: ~2×(DT/T)×Vbus ≈ 0.65V eaten — measured as the rotor
         * settling at ~700 eRPM (≈0.07V real) on a 0.53V command. Current
         * directions are known by construction → deterministic comp: +2×DT
         * (= MIN_DUTY) on the active phase restores effective ≈ duty. */
        uint32_t active = duty + 2u * MIN_DUTY;
        if (active > MAX_DUTY) active = MAX_DUTY;
        const COMMUTATION_STEP_T *s = &commutationTable[s_diffStep];
        uint32_t dA = (s->phaseA == PHASE_PWM_ACTIVE) ? active : MIN_DUTY;
        uint32_t dB = (s->phaseB == PHASE_PWM_ACTIVE) ? active : MIN_DUTY;
        uint32_t dC = (s->phaseC == PHASE_PWM_ACTIVE) ? active : MIN_DUTY;
        PWM_PDC3 = dC;   /* slaves first, master (PG1) last — broadcast */
        PWM_PDC2 = dB;
        PWM_PDC1 = dA;
        return;
    }
#endif
    if (duty < MIN_DUTY) duty = MIN_DUTY;
    if (duty > MAX_DUTY) duty = MAX_DUTY;

    /* Write slave generators first, then master.
     * PG1 (master, UPDTRG=1) triggers an update broadcast on DC write.
     * Writing PG2/PG3 first ensures their buffers are set before
     * the master broadcasts the update event. */
    PWM_PDC3 = duty;
    PWM_PDC2 = duty;
    PWM_PDC1 = duty;
}

#if (FEATURE_SINE_STARTUP || FEATURE_FOC || FEATURE_FOC_V2 || FEATURE_FOC_V3 || FEATURE_FOC_AN1078 || FEATURE_IF_STARTUP)
/**
 * @brief Set independent duty cycles on all three PWM generators.
 * Used by sine startup and FOC SVM to drive 3-phase waveforms.
 * Each phase gets its own duty value. Clamps to MIN_DUTY..MAX_DUTY.
 * Write order: slaves (PG3, PG2) before master (PG1).
 */
void HAL_PWM_SetDutyCycle3Phase(uint32_t dutyA, uint32_t dutyB, uint32_t dutyC)
{
    if (dutyA < MIN_DUTY) dutyA = MIN_DUTY;
    if (dutyA > MAX_DUTY) dutyA = MAX_DUTY;
    if (dutyB < MIN_DUTY) dutyB = MIN_DUTY;
    if (dutyB > MAX_DUTY) dutyB = MAX_DUTY;
    if (dutyC < MIN_DUTY) dutyC = MIN_DUTY;
    if (dutyC > MAX_DUTY) dutyC = MAX_DUTY;

    PWM_PDC3 = dutyC;
    PWM_PDC2 = dutyB;
    PWM_PDC1 = dutyA;
}

/**
 * @brief Release all PWM override registers.
 * Sets OVRENH=0, OVRENL=0 on PG1, PG2, PG3, allowing all three
 * generators to drive complementary PWM from their own PGxDC values.
 */
void HAL_PWM_ReleaseAllOverrides(void)
{
#if GARUDA_TARGET_AK512
    PG3IOCON2bits.OVRENH = 0;
    PG3IOCON2bits.OVRENL = 0;
    PG2IOCON2bits.OVRENH = 0;
    PG2IOCON2bits.OVRENL = 0;
    PG1IOCON2bits.OVRENH = 0;
    PG1IOCON2bits.OVRENL = 0;
#else
    PG3IOCONbits.OVRENH = 0;
    PG3IOCONbits.OVRENL = 0;
    PG2IOCONbits.OVRENH = 0;
    PG2IOCONbits.OVRENL = 0;
    PG1IOCONbits.OVRENH = 0;
    PG1IOCONbits.OVRENL = 0;
#endif
}

void HAL_PWM_ReleaseFloatPhase(uint8_t step)
{
    switch (commutationTable[step].floatingPhase)
    {
#if GARUDA_TARGET_AK512
        case 0: PG1IOCON2bits.OVRENH = 0; PG1IOCON2bits.OVRENL = 0; break;
        case 1: PG2IOCON2bits.OVRENH = 0; PG2IOCON2bits.OVRENL = 0; break;
        case 2: PG3IOCON2bits.OVRENH = 0; PG3IOCON2bits.OVRENL = 0; break;
#else
        case 0: PG1IOCONbits.OVRENH = 0; PG1IOCONbits.OVRENL = 0; break;
        case 1: PG2IOCONbits.OVRENH = 0; PG2IOCONbits.OVRENL = 0; break;
        case 2: PG3IOCONbits.OVRENH = 0; PG3IOCONbits.OVRENL = 0; break;
#endif
    }
}

void HAL_PWM_FloatPhaseToHiZ(uint8_t step)
{
    switch (commutationTable[step].floatingPhase)
    {
#if GARUDA_TARGET_AK512
        case 0:
            PG1IOCON2bits.OVRDAT = 0b00;
            PG1IOCON2bits.OVRENH = 1; PG1IOCON2bits.OVRENL = 1; break;
        case 1:
            PG2IOCON2bits.OVRDAT = 0b00;
            PG2IOCON2bits.OVRENH = 1; PG2IOCON2bits.OVRENL = 1; break;
        case 2:
            PG3IOCON2bits.OVRDAT = 0b00;
            PG3IOCON2bits.OVRENH = 1; PG3IOCON2bits.OVRENL = 1; break;
#else
        case 0:
            PG1IOCONbits.OVRDAT = 0b00;
            PG1IOCONbits.OVRENH = 1; PG1IOCONbits.OVRENL = 1; break;
        case 1:
            PG2IOCONbits.OVRDAT = 0b00;
            PG2IOCONbits.OVRENH = 1; PG2IOCONbits.OVRENL = 1; break;
        case 2:
            PG3IOCONbits.OVRDAT = 0b00;
            PG3IOCONbits.OVRENH = 1; PG3IOCONbits.OVRENL = 1; break;
#endif
    }
}
#endif /* FEATURE_SINE_STARTUP || FEATURE_FOC || FEATURE_FOC_V2 */

#if FEATURE_FOC || FEATURE_FOC_V2 || FEATURE_FOC_V3 || FEATURE_FOC_AN1078 || FEATURE_IF_STARTUP
/**
 * @brief Set 3-phase PWM duty cycles from float [0.0, 1.0] values.
 * Converts normalised float duties to PWM register counts, clamps,
 * and writes in slave-before-master order.
 */
void HAL_PWM_SetDutyFloat3Phase(float da, float db, float dc)
{
    /* NaN/Inf safety: clamp inputs to [0,1] before float→uint32 conversion.
     * NaN bypasses normal comparisons (NaN < x and NaN > x both false),
     * so (uint32_t)(NaN * range) is undefined behavior — can cause a
     * CPU trap on dsPIC33AK FPU. Force to 0.5 (safe center duty). */
    if (!(da >= 0.0f && da <= 1.0f)) da = 0.5f;
    if (!(db >= 0.0f && db <= 1.0f)) db = 0.5f;
    if (!(dc >= 0.0f && dc <= 1.0f)) dc = 0.5f;

    /* Map [0,1] → [0, LOOPTIME_TCY] (full PWM range).
     * SVM already outputs centered duties — no MIN_DUTY offset needed.
     * Clamp to [MIN_DUTY, MAX_DUTY] for dead-time safety only. */
    uint32_t a = (uint32_t)(da * (float)LOOPTIME_TCY);
    uint32_t b = (uint32_t)(db * (float)LOOPTIME_TCY);
    uint32_t c = (uint32_t)(dc * (float)LOOPTIME_TCY);
    if (a < MIN_DUTY) a = MIN_DUTY;  if (a > MAX_DUTY) a = MAX_DUTY;
    if (b < MIN_DUTY) b = MIN_DUTY;  if (b > MAX_DUTY) b = MAX_DUTY;
    if (c < MIN_DUTY) c = MIN_DUTY;  if (c > MAX_DUTY) c = MAX_DUTY;
    PG3DC = c;  /* Slave C first */
    PG2DC = b;  /* Slave B */
    PG1DC = a;  /* Master A last (triggers broadcast) */
}
#endif /* FEATURE_FOC || FEATURE_FOC_V2 */
