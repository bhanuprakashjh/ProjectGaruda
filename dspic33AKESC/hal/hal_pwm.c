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

    /* Set tickle charge duty */
    PWM_PHASE3 = TICKLE_CHARGE_DUTY;
    PWM_PHASE2 = TICKLE_CHARGE_DUTY;
    PWM_PHASE1 = TICKLE_CHARGE_DUTY;
    PWM_PDC3 = TICKLE_CHARGE_DUTY;
    PWM_PDC2 = TICKLE_CHARGE_DUTY;
    PWM_PDC1 = TICKLE_CHARGE_DUTY;

    /* Release L-side override to allow charging */
    PG1IOCONbits.OVRENL = 0;
    PG2IOCONbits.OVRENL = 0;
    PG3IOCONbits.OVRENL = 0;

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
    PG3IOCONbits.OVRENH = 0;
    PG2IOCONbits.OVRENH = 0;
    PG1IOCONbits.OVRENH = 0;
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
    PG1EVTbits.FLTIEN = 1;         /* Fault interrupt enabled */
    PG1EVTbits.CLIEN = 0;
    PG1EVTbits.FFIEN = 0;
    PG1EVTbits.SIEN = 0;
    PG1EVTbits.IEVTSEL = 3;        /* Time base interrupts disabled */
    PG1EVTbits.ADTR2EN3 = 0;
    PG1EVTbits.ADTR2EN2 = 0;
    PG1EVTbits.ADTR2EN1 = 0;
    PG1EVTbits.ADTR1OFS = 0;

    /* Fault PCI — external overcurrent via PCI8 pin (RB11/RP28) */
#ifdef ENABLE_PWM_FAULT_PCI
    PG1FPCI     = 0x0000;
    PG1FPCIbits.TSYNCDIS = 0;
    PG1FPCIbits.TERM = 1;
    PG1FPCIbits.AQPS = 0;
    PG1FPCIbits.AQSS = 0;
    PG1FPCIbits.PSYNC = 0;
    PG1FPCIbits.PPS = 1;           /* Inverted polarity */
    PG1FPCIbits.PSS = 0b01000;     /* RPn input (PCI8R) */
    PG1FPCIbits.BPEN = 0;
    PG1FPCIbits.BPSEL = 0;
    PG1FPCIbits.TERMPS = 0;
    PG1FPCIbits.ACP = 3;           /* Latched */
    PG1FPCIbits.LATMOD = 0;
    PG1FPCIbits.TQPS = 0;
    PG1FPCIbits.TQSS = 0;
#else
    PG1FPCI     = 0x0000;
#endif

    PG1CLPCI    = 0x0000;
    PG1FFPCI    = 0x0000;
    PG1SPCI     = 0x0000;
    PG1LEB      = 0x0000;

    PG1PHASEbits.PHASE = MIN_DUTY;
    PG1DCbits.DC = MIN_DUTY;
    PG1DCA      = 0x0000;
    PG1PER      = 0x0000;
    PG1DTbits.DTH = DEADTIME_COUNTS;
    PG1DTbits.DTL = DEADTIME_COUNTS;

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

#ifdef ENABLE_PWM_FAULT_PCI
    PG2FPCI     = 0x0000;
    PG2FPCIbits.TSYNCDIS = 0;
    PG2FPCIbits.TERM = 1;
    PG2FPCIbits.AQPS = 0;
    PG2FPCIbits.AQSS = 0;
    PG2FPCIbits.PSYNC = 0;
    PG2FPCIbits.PPS = 1;
    PG2FPCIbits.PSS = 0b01000;     /* RPn input (PCI8R) */
    PG2FPCIbits.BPEN = 0;
    PG2FPCIbits.BPSEL = 0;
    PG2FPCIbits.TERMPS = 0;
    PG2FPCIbits.ACP = 3;
    PG2FPCIbits.LATMOD = 0;
    PG2FPCIbits.TQPS = 0;
    PG2FPCIbits.TQSS = 0;
#else
    PG2FPCI     = 0x0000;
#endif

    PG2CLPCI    = 0x0000;
    PG2FFPCI    = 0x0000;
    PG2SPCI     = 0x0000;
    PG2LEB      = 0x0000;

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

#ifdef ENABLE_PWM_FAULT_PCI
    PG3FPCI     = 0x0000;
    PG3FPCIbits.TSYNCDIS = 0;
    PG3FPCIbits.TERM = 1;
    PG3FPCIbits.AQPS = 0;
    PG3FPCIbits.AQSS = 0;
    PG3FPCIbits.PSYNC = 0;
    PG3FPCIbits.PPS = 1;
    PG3FPCIbits.PSS = 0b01000;     /* RPn input (PCI8R) */
    PG3FPCIbits.BPEN = 0;
    PG3FPCIbits.BPSEL = 0;
    PG3FPCIbits.TERMPS = 0;
    PG3FPCIbits.ACP = 3;
    PG3FPCIbits.LATMOD = 0;
    PG3FPCIbits.TQPS = 0;
    PG3FPCIbits.TQSS = 0;
#else
    PG3FPCI     = 0x0000;
#endif

    PG3CLPCI    = 0x0000;
    PG3FFPCI    = 0x0000;
    PG3SPCI     = 0x0000;
    PG3LEB      = 0x0000;

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

/**
 * @brief Apply a 6-step commutation pattern to PWM override registers.
 *
 * For each phase:
 *   PHASE_PWM_ACTIVE: H-side from PWM generator (OVRENH=0), L-side complementary (OVRENL=0)
 *   PHASE_LOW:        H-side overridden LOW (OVRENH=1,OVRDAT<1>=0),
 *                     L-side overridden HIGH (OVRENL=1,OVRDAT<0>=1) to sink current
 *   PHASE_FLOAT:      Both overridden LOW (OVRENH=1,OVRENL=1,OVRDAT=0) — high-Z
 *
 * @param step Commutation step index 0-5
 */
void HAL_PWM_SetCommutationStep(uint8_t step)
{
    const COMMUTATION_STEP_T *s = &commutationTable[step];

    /* Phase A — PG1 */
    switch (s->phaseA)
    {
        case PHASE_PWM_ACTIVE:
            PG1IOCONbits.OVRENH = 0;
            PG1IOCONbits.OVRENL = 0;
            break;
        case PHASE_LOW:
            PG1IOCONbits.OVRDAT = 0b01;    /* H=LOW, L=HIGH (sink) */
            PG1IOCONbits.OVRENH = 1;
            PG1IOCONbits.OVRENL = 1;
            break;
        case PHASE_FLOAT:
            PG1IOCONbits.OVRDAT = 0b00;
            PG1IOCONbits.OVRENH = 1;
            PG1IOCONbits.OVRENL = 1;
            break;
    }

    /* Phase B — PG2 */
    switch (s->phaseB)
    {
        case PHASE_PWM_ACTIVE:
            PG2IOCONbits.OVRENH = 0;
            PG2IOCONbits.OVRENL = 0;
            break;
        case PHASE_LOW:
            PG2IOCONbits.OVRDAT = 0b01;
            PG2IOCONbits.OVRENH = 1;
            PG2IOCONbits.OVRENL = 1;
            break;
        case PHASE_FLOAT:
            PG2IOCONbits.OVRDAT = 0b00;
            PG2IOCONbits.OVRENH = 1;
            PG2IOCONbits.OVRENL = 1;
            break;
    }

    /* Phase C — PG3 */
    switch (s->phaseC)
    {
        case PHASE_PWM_ACTIVE:
            PG3IOCONbits.OVRENH = 0;
            PG3IOCONbits.OVRENL = 0;
            break;
        case PHASE_LOW:
            PG3IOCONbits.OVRDAT = 0b01;
            PG3IOCONbits.OVRENH = 1;
            PG3IOCONbits.OVRENL = 1;
            break;
        case PHASE_FLOAT:
            PG3IOCONbits.OVRDAT = 0b00;
            PG3IOCONbits.OVRENH = 1;
            PG3IOCONbits.OVRENL = 1;
            break;
    }
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
