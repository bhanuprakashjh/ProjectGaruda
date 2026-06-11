/**
 * sil/sil_api.h — flat C API exported by libgaruda_sil.so for the Python
 * harness (cffi ABI mode / ctypes).
 */
#ifndef SIL_API_H
#define SIL_API_H

#include <stdint.h>
#include "virtual_hw.h"

#ifdef __cplusplus
extern "C" {
#endif

/* lifecycle */
void sil_init(void);

/* ISR entry points (call the REAL firmware ISRs) */
void sil_adc_isr(void);
void sil_timer1_isr(void);
void sil_cct1_isr(void);          /* honors _CCT1IE; disarms SCCP1 first    */
void sil_cmp_isr(int core);       /* honors _ADxCMPyIE; sets CMPSTAT first  */

/* bridge */
SilHw *sil_hw(void);

/* plant → firmware inputs */
void sil_set_time(uint64_t sccp2_ticks);
void sil_set_adc(uint16_t phaseB, uint16_t phaseAC, uint16_t pot,
                 uint16_t vbus, uint16_t ibus, uint16_t ia, uint16_t ib);
void sil_set_cmp_adc(uint16_t ad1ch5, uint16_t ad2ch1);
void sil_set_pwm_on(int on);

/* firmware → plant getters */
int      sil_leg_mode(int leg);
uint32_t sil_pdc(int leg);
int      sil_bemf_mux(void);
int      sil_hs_pinsel(void);
int      sil_cmp_ie(int core);
int      sil_cmp_rising(int core);
uint16_t sil_cmp_thresh(int core);
int      sil_sccp1_armed(void);
uint64_t sil_sccp1_deadline(void);
int      sil_cct1_ie(void);

/* garudaData getters */
int      sil_state(void);
int      sil_fault(void);
uint32_t sil_duty(void);
int      sil_step(void);
int      sil_direction(void);
int      sil_zc_synced(void);
uint16_t sil_step_period(void);
uint32_t sil_hwzc_period(void);
int      sil_hwzc_enabled(void);
int      sil_hwzc_phase(void);
uint16_t sil_good_zc(void);
uint32_t sil_hwzc_total_zc(void);
uint32_t sil_hwzc_timer_period(void);
uint32_t sil_hwzc_cap_frac_pm(void);
int      sil_pll_active(void);
int      sil_pll_good(void);
uint32_t sil_erpm(void);
uint32_t sil_systick(void);
uint16_t sil_throttle(void);
uint16_t sil_zc_threshold(void);
uint16_t sil_bemf_raw(void);
int      sil_morph_subphase(void);
uint16_t sil_morph_alpha(void);
uint16_t sil_sw_good_zc(void);
uint16_t sil_sine_amplitude(void);
uint32_t sil_sine_erpm(void);
int      sil_run_command(void);
uint16_t sil_arm_counter(void);
uint16_t sil_ramp_step_period(void);
uint32_t sil_min_duty(void);
uint32_t sil_looptime_tcy(void);

/* fast-path pointers for the Python inner loop (avoid FFI call overhead) */
volatile uint8_t  *sil_cmp_ie_ptr(int core);    /* _AD1CMP5IE / _AD2CMP1IE */
volatile uint8_t  *sil_cct1_ie_ptr(void);       /* _CCT1IE */
volatile uint32_t *sil_cmp_data_ptr(int core);  /* AD1CH5DATA / AD2CH1DATA */

/* commands — replicate main.c's SW1/GSP intent handling */
void sil_cmd_start(void);
void sil_cmd_stop(void);
void sil_cmd_fault_clear(void);

#ifdef __cplusplus
}
#endif

#endif /* SIL_API_H */
