/**
 * sil/virtual_hw.c — instances of the mock SFR globals (mock/xc.h) and the
 * SilHw bridge register file.
 */
#include <xc.h>
#include <string.h>
#include "virtual_hw.h"

/* ── mock SFR instances ──────────────────────────────────────────────── */
volatile uint32_t AD1CH0DATA, AD2CH0DATA, AD1CH1DATA, AD1CH2DATA;
volatile uint32_t AD1CH3DATA, AD2CH2DATA, AD1CH4DATA;
volatile uint32_t AD1CH5DATA, AD2CH1DATA;
volatile uint32_t AD1CH5CMPLO, AD2CH1CMPLO, AD1CH5CMPHI, AD2CH1CMPHI;

volatile MOCK_ADCHCON_BITS AD1CH0CONbits, AD1CH5CONbits, AD2CH0CONbits, AD2CH1CONbits;
volatile MOCK_ADCMPSTAT_BITS AD1CMPSTATbits, AD2CMPSTATbits;

volatile uint8_t _AD1CH0IE,  _AD1CH0IF,  _AD1CH0IP;
volatile uint8_t _AD1CMP5IE, _AD1CMP5IF, _AD1CMP5IP;
volatile uint8_t _AD2CMP1IE, _AD2CMP1IF, _AD2CMP1IP;
volatile uint8_t _CCT1IE,    _CCT1IF,    _CCT1IP;
volatile uint8_t _T1IE,      _T1IF,      _T1IP;
volatile uint8_t _PWM1IE,    _PWM1IF,    _PWM1IP;

volatile MOCK_T1CON_BITS T1CONbits;
volatile uint32_t PR1, TMR1;

volatile MOCK_PGSTAT_BITS PG1STATbits, PG2STATbits, PG3STATbits;
volatile uint32_t PG1STAT, PG2STAT, PG3STAT;

volatile uint32_t PORTC, PORTD;
volatile MOCK_PORTD_BITS PORTDbits;
volatile MOCK_LATD_BITS  LATDbits;
volatile MOCK_LATC_BITS  LATCbits;

/* ── SilHw bridge ────────────────────────────────────────────────────── */
SilHw g_silHw;

void SilHw_Reset(void)
{
    memset(&g_silHw, 0, sizeof(g_silHw));
    g_silHw.leg_mode[0] = SIL_LEG_HIZ;
    g_silHw.leg_mode[1] = SIL_LEG_HIZ;
    g_silHw.leg_mode[2] = SIL_LEG_HIZ;
    g_silHw.bemf_mux = 10;
    g_silHw.hs_pinsel = 10;

    /* clear the mock SFRs the firmware reads */
    AD1CH0DATA = AD2CH0DATA = AD1CH1DATA = AD1CH2DATA = 0;
    AD1CH3DATA = AD2CH2DATA = AD1CH4DATA = 0;
    AD1CH5DATA = AD2CH1DATA = 0;
    AD1CH5CMPLO = AD2CH1CMPLO = AD1CH5CMPHI = AD2CH1CMPHI = 0;
    _AD1CH0IE = _AD1CH0IF = 0;
    _AD1CMP5IE = _AD1CMP5IF = 0;
    _AD2CMP1IE = _AD2CMP1IF = 0;
    _CCT1IE = _CCT1IF = 0;
    _T1IE = _T1IF = 0;
    _PWM1IE = _PWM1IF = 0;
    PORTC = PORTD = 0;
}
