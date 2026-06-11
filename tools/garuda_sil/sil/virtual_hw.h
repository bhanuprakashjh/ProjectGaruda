/**
 * sil/virtual_hw.h — the bridge register file between the REAL firmware
 * (via sil/hal_sil.c) and the Python plant.
 *
 * OUT  = firmware → plant (PWM duties, leg modes, comparator config, SCCP1)
 * IN   = plant → firmware (ADC values are written straight into the mock
 *        AD*DATA globals by sil_api setters; comparator/timer events are
 *        injected by calling the ISR entry points).
 */
#ifndef SIL_VIRTUAL_HW_H
#define SIL_VIRTUAL_HW_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* leg drive mode (firmware → plant) */
enum {
    SIL_LEG_PWM  = 0,   /* complementary PWM from pdc[] */
    SIL_LEG_LOW  = 1,   /* override LOW: H off, L on (phase tied to GND) */
    SIL_LEG_HIZ  = 2    /* override float: both FETs off */
};

typedef struct {
    /* ── firmware → plant ─────────────────────────────────────────── */
    uint32_t pdc[3];          /* PGx duty registers (PWM counts, vs LOOPTIME_TCY) */
    uint8_t  leg_mode[3];     /* SIL_LEG_* per leg A/B/C */

    uint8_t  bemf_mux;        /* AD2CH0 PINSEL: 10 = phase A, 7 = phase C */
    uint8_t  hs_pinsel;       /* AD2CH1 PINSEL: 10 = phase A, 7 = phase C */

    /* high-speed digital comparator config, per ADC core (index 1 and 2) */
    uint16_t cmp_thresh[3];   /* threshold written by ConfigComparator/Update */
    uint8_t  cmp_rising[3];   /* 1 = fire on rising crossing */

    uint16_t cmp3_dac;        /* bus-OC CMP3 DAC threshold */

    /* SCCP1 one-shot timer */
    uint8_t  sccp1_armed;
    uint64_t sccp1_deadline;  /* absolute SCCP2 (100 MHz logical) tick */

    /* SCCP2 free-running timestamp — Python advances this */
    uint64_t sccp2_now;

    /* SCCP3 periodic trigger (1 MHz ADC pacing) — recorded, not simulated */
    uint32_t sccp3_period;
    uint8_t  sccp3_running;

    /* diagnostics */
    uint32_t writes_set_step;     /* count of HAL_PWM_SetCommutationStep calls */
    uint32_t writes_duty;         /* count of HAL_PWM_SetDutyCycle* calls */
    uint8_t  last_step;           /* last commutation step applied */
} SilHw;

extern SilHw g_silHw;

void SilHw_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* SIL_VIRTUAL_HW_H */
