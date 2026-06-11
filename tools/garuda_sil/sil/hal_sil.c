/**
 * sil/hal_sil.c — host reimplementation of the HAL function surface used by
 * the compiled firmware set. Mirrors hal/hal_pwm.c, hal/hal_adc.c,
 * hal/hal_timer.c, hal/hal_comparator.c and hal/board_service.c semantics,
 * but writes into g_silHw instead of real SFRs.
 *
 * Includes the FIRMWARE's own headers so every signature stays in lockstep
 * with what garuda_service.c / motor/*.c expect.
 */
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "garuda_types.h"
#include "garuda_config.h"
#include "garuda_calc_params.h"
#include "hal/hal_pwm.h"
#include "hal/hal_adc.h"
#include "hal/hal_timer.h"
#include "hal/hal_comparator.h"
#include "hal/board_service.h"
#include "motor/commutation.h"

#include "virtual_hw.h"

/* ── PWM ─────────────────────────────────────────────────────────────── */

static uint8_t legModeFromPhase(uint8_t phaseRole)
{
    switch (phaseRole) {
        case PHASE_PWM_ACTIVE: return SIL_LEG_PWM;
        case PHASE_LOW:        return SIL_LEG_LOW;
        case PHASE_FLOAT:      return SIL_LEG_HIZ;
        default:               return SIL_LEG_HIZ;
    }
}

void HAL_PWM_SetCommutationStep(uint8_t step)
{
    if (step >= 6) step %= 6;
    const COMMUTATION_STEP_T *s = &commutationTable[step];
    g_silHw.leg_mode[0] = legModeFromPhase(s->phaseA);
    g_silHw.leg_mode[1] = legModeFromPhase(s->phaseB);
    g_silHw.leg_mode[2] = legModeFromPhase(s->phaseC);
    g_silHw.last_step = step;
    g_silHw.writes_set_step++;
}

void HAL_PWM_SetDutyCycle(uint32_t duty)
{
    if (duty < MIN_DUTY) duty = MIN_DUTY;
    if (duty > MAX_DUTY) duty = MAX_DUTY;
    g_silHw.pdc[0] = duty;
    g_silHw.pdc[1] = duty;
    g_silHw.pdc[2] = duty;
    g_silHw.writes_duty++;
}

void HAL_PWM_SetDutyCycle3Phase(uint32_t dutyA, uint32_t dutyB, uint32_t dutyC)
{
    if (dutyA < MIN_DUTY) dutyA = MIN_DUTY;
    if (dutyA > MAX_DUTY) dutyA = MAX_DUTY;
    if (dutyB < MIN_DUTY) dutyB = MIN_DUTY;
    if (dutyB > MAX_DUTY) dutyB = MAX_DUTY;
    if (dutyC < MIN_DUTY) dutyC = MIN_DUTY;
    if (dutyC > MAX_DUTY) dutyC = MAX_DUTY;
    g_silHw.pdc[0] = dutyA;
    g_silHw.pdc[1] = dutyB;
    g_silHw.pdc[2] = dutyC;
    g_silHw.writes_duty++;
}

void HAL_PWM_ReleaseAllOverrides(void)
{
    g_silHw.leg_mode[0] = SIL_LEG_PWM;
    g_silHw.leg_mode[1] = SIL_LEG_PWM;
    g_silHw.leg_mode[2] = SIL_LEG_PWM;
}

void HAL_PWM_ReleaseFloatPhase(uint8_t step)
{
    uint8_t fp = commutationTable[step].floatingPhase;
    if (fp < 3) g_silHw.leg_mode[fp] = SIL_LEG_PWM;
}

void HAL_PWM_FloatPhaseToHiZ(uint8_t step)
{
    uint8_t fp = commutationTable[step].floatingPhase;
    if (fp < 3) g_silHw.leg_mode[fp] = SIL_LEG_HIZ;
}

/* ── board service surface (subset used by the compiled set) ────────── */

void HAL_MC1PWMDisableOutputs(void)
{
    /* real impl: OVRDAT=0 + OVREN on all legs = both FETs off = Hi-Z */
    g_silHw.leg_mode[0] = SIL_LEG_HIZ;
    g_silHw.leg_mode[1] = SIL_LEG_HIZ;
    g_silHw.leg_mode[2] = SIL_LEG_HIZ;
    g_silHw.pdc[0] = g_silHw.pdc[1] = g_silHw.pdc[2] = MIN_DUTY;
}

void HAL_MC1PWMEnableOutputs(void)
{
    g_silHw.leg_mode[0] = SIL_LEG_PWM;
    g_silHw.leg_mode[1] = SIL_LEG_PWM;
    g_silHw.leg_mode[2] = SIL_LEG_PWM;
}

void HAL_MC1ClearPWMPCIFault(void)
{
    PG1STATbits.FLTACT = 0;
    PG2STATbits.FLTACT = 0;
    PG3STATbits.FLTACT = 0;
}

void BoardServiceStepIsr(void) { /* button debounce — unused in SIL */ }
void BoardServiceInit(void)    { }
void BoardService(void)        { }
bool IsPressed_Button1(void)   { return false; }
bool IsPressed_Button2(void)   { return false; }
void HAL_InitPeripherals(void) { }
void HAL_ResetPeripherals(void){ }
void HAL_TrapHandler(void)     { }

/* ── ADC ─────────────────────────────────────────────────────────────── */

bool HAL_ADC_SelectBEMFChannel(uint8_t floatingPhase)
{
    /* mirrors hal/hal_adc.c: phase B is on AD1 (no mux change); A/C share
     * AD2CH0 via PINSEL — return true only when the mux actually moved. */
    switch (floatingPhase) {
        case FLOATING_PHASE_A:
            if (g_silHw.bemf_mux != 10) { g_silHw.bemf_mux = 10; return true; }
            return false;
        case FLOATING_PHASE_B:
            return false;
        case FLOATING_PHASE_C:
            if (g_silHw.bemf_mux != 7) { g_silHw.bemf_mux = 7; return true; }
            return false;
        default:
            return false;
    }
}

void InitializeADCs(void) { }

#if FEATURE_ADC_CMP_ZC
void HAL_ADC_InitHighSpeedBEMF(void)
{
    _AD1CMP5IE = 0; _AD1CMP5IF = 0;
    _AD2CMP1IE = 0; _AD2CMP1IF = 0;
}

void HAL_ADC_ConfigComparator(uint8_t adcCore, uint16_t threshold, bool risingZc)
{
    if (adcCore > 2) return;
    g_silHw.cmp_thresh[adcCore] = threshold;
    g_silHw.cmp_rising[adcCore] = risingZc ? 1u : 0u;
    if (adcCore == 1) AD1CH5CMPLO = threshold;
    else              AD2CH1CMPLO = threshold;
}

void HAL_ADC_UpdateComparatorThreshold(uint8_t adcCore, uint16_t threshold)
{
    if (adcCore > 2) return;
    g_silHw.cmp_thresh[adcCore] = threshold;
    if (adcCore == 1) AD1CH5CMPLO = threshold;
    else              AD2CH1CMPLO = threshold;
}

void HAL_ADC_EnableComparatorIE(uint8_t adcCore)
{
    if (adcCore == 1) _AD1CMP5IE = 1;
    else if (adcCore == 2) _AD2CMP1IE = 1;
}

void HAL_ADC_DisableComparatorIE(uint8_t adcCore)
{
    if (adcCore == 1) _AD1CMP5IE = 0;
    else if (adcCore == 2) _AD2CMP1IE = 0;
}

void HAL_ADC_ClearComparatorFlag(uint8_t adcCore)
{
    if (adcCore == 1) { AD1CMPSTATbits.CH5CMP = 0; _AD1CMP5IF = 0; }
    else if (adcCore == 2) { AD2CMPSTATbits.CH1CMP = 0; _AD2CMP1IF = 0; }
}

void HAL_ADC_SetHighSpeedPinsel(uint8_t pinsel)
{
    g_silHw.hs_pinsel = pinsel;
}
#endif /* FEATURE_ADC_CMP_ZC */

/* ── comparator 3 (bus OC DAC) ───────────────────────────────────────── */
#if FEATURE_HW_OVERCURRENT
void HAL_CMP3_EnableOvercurrent(void) { }
void HAL_CMP3_SetThreshold(uint16_t dacVal) { g_silHw.cmp3_dac = dacVal; }
#endif
void InitializeCMPs(void) { }
void HAL_CMP_EnableFloatingPhase(uint8_t phase) { (void)phase; }
void HAL_CMP_SetReference(uint16_t vbusHalf)    { (void)vbusHalf; }
uint8_t HAL_CMP_ReadStatus(uint8_t phase)       { (void)phase; return 0; }

/* ── SCCP timers ─────────────────────────────────────────────────────── */
#if FEATURE_ADC_CMP_ZC
void HAL_SCCP1_Init(void) { g_silHw.sccp1_armed = 0; }

void HAL_SCCP1_StartOneShot(uint32_t ticks)
{
    g_silHw.sccp1_deadline = g_silHw.sccp2_now + (uint64_t)ticks;
    g_silHw.sccp1_armed = 1;
}

void HAL_SCCP1_Stop(void) { g_silHw.sccp1_armed = 0; }

void HAL_SCCP2_Init(void) { }

uint32_t HAL_SCCP2_ReadTimestamp(void)
{
    return (uint32_t)g_silHw.sccp2_now;   /* natural 32-bit wrap, like HW */
}

void HAL_SCCP3_InitPeriodic(uint32_t periodTicks)
{
    g_silHw.sccp3_period = periodTicks;
    g_silHw.sccp3_running = 1;
}

void HAL_SCCP3_Stop(void) { g_silHw.sccp3_running = 0; }
#endif /* FEATURE_ADC_CMP_ZC */
