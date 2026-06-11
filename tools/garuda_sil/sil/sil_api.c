/**
 * sil/sil_api.c — flat C API for the Python harness.
 *
 * Calls the REAL firmware ISR bodies (compiled unmodified from
 * dspic33AKESC/) and replicates main.c's arming/stop sequences.
 */
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "garuda_types.h"
#include "garuda_config.h"
#include "garuda_calc_params.h"
#include "garuda_service.h"
#include "hal/hal_adc.h"
#include "hal/board_service.h"
#if FEATURE_ADC_CMP_ZC
#include "motor/hwzc.h"
#endif
#if FEATURE_GSP
#include "gsp/gsp_params.h"
#endif

#include "virtual_hw.h"
#include "sil_api.h"

/* the real firmware ISR symbols (garuda_service.c) */
extern void _AD1CH0Interrupt(void);     /* GARUDA_ADC_INTERRUPT */
extern void _T1Interrupt(void);
extern void _CCT1Interrupt(void);
extern void _AD1CMP5Interrupt(void);
extern void _AD2CMP1Interrupt(void);

/* ── lifecycle ───────────────────────────────────────────────────────── */

void sil_init(void)
{
    SilHw_Reset();
#if FEATURE_GSP
    GSP_ParamsInitDefaults();   /* compile-time defaults (no EEPROM overlay) */
    GSP_RecomputeDerived();
#endif
    GARUDA_ServiceInit();
}

/* ── ISR dispatch ────────────────────────────────────────────────────── */

void sil_adc_isr(void)
{
    _AD1CH0Interrupt();
}

void sil_timer1_isr(void)
{
    _T1Interrupt();
}

void sil_cct1_isr(void)
{
    if (!_CCT1IE) return;
    /* hardware one-shot: timer stops at the event, ISR may re-arm */
    g_silHw.sccp1_armed = 0;
    _CCT1IF = 1;
    _CCT1Interrupt();
}

void sil_cmp_isr(int core)
{
    if (core == 1) {
        if (!_AD1CMP5IE) return;
        AD1CMPSTATbits.CH5CMP = 1;
        _AD1CMP5IF = 1;
        _AD1CMP5Interrupt();
    } else if (core == 2) {
        if (!_AD2CMP1IE) return;
        AD2CMPSTATbits.CH1CMP = 1;
        _AD2CMP1IF = 1;
        _AD2CMP1Interrupt();
    }
}

/* ── bridge ──────────────────────────────────────────────────────────── */

SilHw *sil_hw(void) { return &g_silHw; }

void sil_set_time(uint64_t sccp2_ticks) { g_silHw.sccp2_now = sccp2_ticks; }

void sil_set_adc(uint16_t phaseB, uint16_t phaseAC, uint16_t pot,
                 uint16_t vbus, uint16_t ibus, uint16_t ia, uint16_t ib)
{
    AD1CH0DATA = phaseB;
    AD2CH0DATA = phaseAC;
    AD1CH1DATA = pot;
    AD1CH4DATA = vbus;
    AD1CH2DATA = ibus;
    AD1CH3DATA = ia;
    AD2CH2DATA = ib;
}

void sil_set_cmp_adc(uint16_t ad1ch5, uint16_t ad2ch1)
{
    AD1CH5DATA = ad1ch5;
    AD2CH1DATA = ad2ch1;
}

void sil_set_pwm_on(int on)
{
    /* HWZC PWM-state gate reads the H-side GPIO of the PWMing phase:
     * RD2 = PWM1H (A), RD0 = PWM2H (B), RC3 = PWM3H (C). Set all three
     * coherently — the gate only samples the active phase's pin. */
    if (on) {
        PORTD |= (1u << 2) | (1u << 0);
        PORTC |= (1u << 3);
    } else {
        PORTD &= ~((1u << 2) | (1u << 0));
        PORTC &= ~(1u << 3);
    }
}

/* ── firmware → plant getters ────────────────────────────────────────── */

int      sil_leg_mode(int leg)   { return (leg >= 0 && leg < 3) ? g_silHw.leg_mode[leg] : SIL_LEG_HIZ; }
uint32_t sil_pdc(int leg)        { return (leg >= 0 && leg < 3) ? g_silHw.pdc[leg] : 0; }
int      sil_bemf_mux(void)      { return g_silHw.bemf_mux; }
int      sil_hs_pinsel(void)     { return g_silHw.hs_pinsel; }
int      sil_cmp_ie(int core)    { return (core == 1) ? _AD1CMP5IE : (core == 2) ? _AD2CMP1IE : 0; }
int      sil_cmp_rising(int core){ return (core >= 1 && core <= 2) ? g_silHw.cmp_rising[core] : 0; }
uint16_t sil_cmp_thresh(int core){ return (core >= 1 && core <= 2) ? g_silHw.cmp_thresh[core] : 0; }
int      sil_sccp1_armed(void)   { return g_silHw.sccp1_armed; }
uint64_t sil_sccp1_deadline(void){ return g_silHw.sccp1_deadline; }
int      sil_cct1_ie(void)       { return _CCT1IE; }

/* ── garudaData getters ──────────────────────────────────────────────── */

int      sil_state(void)        { return (int)garudaData.state; }
int      sil_fault(void)        { return (int)garudaData.faultCode; }
uint32_t sil_duty(void)         { return garudaData.duty; }
int      sil_step(void)         { return garudaData.currentStep; }
int      sil_direction(void)    { return garudaData.direction; }
int      sil_zc_synced(void)    { return garudaData.timing.zcSynced ? 1 : 0; }
uint16_t sil_step_period(void)  { return garudaData.timing.stepPeriod; }
uint32_t sil_systick(void)      { return garudaData.systemTick; }
uint16_t sil_throttle(void)     { return garudaData.throttle; }
uint16_t sil_zc_threshold(void) { return garudaData.bemf.zcThreshold; }
uint16_t sil_bemf_raw(void)     { return garudaData.bemf.bemfRaw; }
uint16_t sil_sw_good_zc(void)   { return garudaData.timing.goodZcCount; }
int      sil_run_command(void)  { return garudaData.runCommandActive ? 1 : 0; }
uint16_t sil_arm_counter(void)  { return (uint16_t)garudaData.armCounter; }
uint16_t sil_ramp_step_period(void) { return garudaData.rampStepPeriod; }
uint32_t sil_min_duty(void)     { return MIN_DUTY; }
uint32_t sil_looptime_tcy(void) { return LOOPTIME_TCY; }

#if FEATURE_ADC_CMP_ZC
int      sil_hwzc_enabled(void) { return garudaData.hwzc.enabled ? 1 : 0; }
int      sil_hwzc_phase(void)   { return (int)garudaData.hwzc.phase; }
uint32_t sil_hwzc_period(void)  { return garudaData.hwzc.stepPeriodHR; }
uint16_t sil_good_zc(void)      { return garudaData.hwzc.goodZcCount; }
uint32_t sil_hwzc_total_zc(void){ return garudaData.hwzc.totalZcCount; }
#else
int      sil_hwzc_enabled(void) { return 0; }
int      sil_hwzc_phase(void)   { return 0; }
uint32_t sil_hwzc_period(void)  { return 0; }
uint16_t sil_good_zc(void)      { return 0; }
uint32_t sil_hwzc_total_zc(void){ return 0; }
#endif

#if FEATURE_SINE_STARTUP
int      sil_morph_subphase(void) { return (int)garudaData.morph.subPhase; }
uint16_t sil_morph_alpha(void)    { return garudaData.morph.alpha; }
uint16_t sil_sine_amplitude(void) { return garudaData.sine.amplitude; }
uint32_t sil_sine_erpm(void)      { return garudaData.sine.erpmFrac >> 16; }
#else
int      sil_morph_subphase(void) { return 0; }
uint16_t sil_morph_alpha(void)    { return 0; }
uint16_t sil_sine_amplitude(void) { return 0; }
uint32_t sil_sine_erpm(void)      { return 0; }
#endif

uint32_t sil_erpm(void)
{
#if FEATURE_ADC_CMP_ZC
    if (garudaData.hwzc.enabled && garudaData.hwzc.stepPeriodHR > 0)
        return HWZC_TICKS_TO_ERPM(garudaData.hwzc.stepPeriodHR);
#endif
    if (garudaData.timing.stepPeriod > 0
        && garudaData.state == ESC_CLOSED_LOOP)
        return ERPM_FROM_ADC_STEP_NUM / garudaData.timing.stepPeriod;
    return 0;
}

/* ── fast-path pointers ──────────────────────────────────────────────── */

volatile uint8_t *sil_cmp_ie_ptr(int core)
{
    return (core == 1) ? &_AD1CMP5IE : &_AD2CMP1IE;
}

volatile uint8_t *sil_cct1_ie_ptr(void) { return &_CCT1IE; }

volatile uint32_t *sil_cmp_data_ptr(int core)
{
    return (core == 1) ? &AD1CH5DATA : &AD2CH1DATA;
}

/* ── commands (replicating main.c) ───────────────────────────────────── */

void sil_cmd_start(void)
{
    if (garudaData.state == ESC_IDLE) {
        garudaData.runCommandActive = true;
        garudaData.desyncRestartAttempts = 0;
        garudaData.armCounter = 0;
        garudaData.state = ESC_ARMED;
    }
}

void sil_cmd_stop(void)
{
    if (garudaData.state != ESC_IDLE && garudaData.state != ESC_FAULT) {
        garudaData.state = ESC_IDLE;
        garudaData.runCommandActive = false;
        garudaData.desyncRestartAttempts = 0;
#if FEATURE_ADC_CMP_ZC
        if (garudaData.hwzc.enabled)
            HWZC_Disable(&garudaData);
        garudaData.hwzc.fallbackPending = false;
#endif
        HAL_MC1PWMDisableOutputs();
        LATCbits.LATC9 = 0;
    }
}

void sil_cmd_fault_clear(void)
{
    if (garudaData.state == ESC_FAULT) {
        garudaData.state = ESC_IDLE;
        garudaData.runCommandActive = false;
        garudaData.desyncRestartAttempts = 0;
        garudaData.faultCode = FAULT_NONE;
#if FEATURE_ADC_CMP_ZC
        if (garudaData.hwzc.enabled)
            HWZC_Disable(&garudaData);
        garudaData.hwzc.fallbackPending = false;
#endif
        HAL_MC1ClearPWMPCIFault();
        HAL_MC1PWMDisableOutputs();
        LATCbits.LATC9 = 0;
    }
}
