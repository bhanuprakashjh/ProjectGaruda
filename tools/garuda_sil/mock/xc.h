/**
 * mock/xc.h — fake dsPIC33AK SFR surface for the garuda_sil x86 build.
 *
 * Included instead of the real <xc.h> because -I mock precedes the XC32
 * include dirs. Declares ONLY what the compiled firmware set references,
 * as plain volatile globals + bitfield structs with the exact field names
 * the firmware uses. Instances live in sil/virtual_hw.c.
 *
 * NOTE: register layout/bit positions are NOT modeled — only names. The
 * SIL plant interacts through sil/virtual_hw.h and these globals.
 */
#ifndef MOCK_XC_H
#define MOCK_XC_H

#include <stdint.h>

/* Neutralize the dsPIC ISR attributes. On x86-64 gcc, `interrupt` is a REAL
 * attribute with hard signature/SSE constraints, so it must not survive.
 * The firmware spells them __attribute__((__interrupt__, no_auto_psv)). */
#define __interrupt__ __unused__
#define no_auto_psv   __unused__

#ifdef __cplusplus
extern "C" {
#endif

/* ── ADC channel data / comparator registers ─────────────────────────── */
extern volatile uint32_t AD1CH0DATA;   /* phase B voltage (6-step) */
extern volatile uint32_t AD2CH0DATA;   /* phase A/C voltage (muxed) */
extern volatile uint32_t AD1CH1DATA;   /* pot */
extern volatile uint32_t AD1CH2DATA;   /* ibus */
extern volatile uint32_t AD1CH3DATA;   /* Ia monitor */
extern volatile uint32_t AD2CH2DATA;   /* Ib monitor */
extern volatile uint32_t AD1CH4DATA;   /* vbus */
extern volatile uint32_t AD1CH5DATA;   /* high-speed BEMF cmp channel (phase B) */
extern volatile uint32_t AD2CH1DATA;   /* high-speed BEMF cmp channel (phase A/C) */
extern volatile uint32_t AD1CH5CMPLO;
extern volatile uint32_t AD2CH1CMPLO;
extern volatile uint32_t AD1CH5CMPHI;
extern volatile uint32_t AD2CH1CMPHI;

/* generic ADC channel CON bitfield — field names only */
typedef struct {
    unsigned PINSEL  : 6;
    unsigned CMPMOD  : 3;
    unsigned TRG1SRC : 5;
    unsigned TRG2SRC : 5;
    unsigned SAMC    : 10;
    unsigned MODE    : 2;
    unsigned ACCNUM  : 2;
    unsigned CMPIE   : 1;
} MOCK_ADCHCON_BITS;
extern volatile MOCK_ADCHCON_BITS AD1CH0CONbits;
extern volatile MOCK_ADCHCON_BITS AD1CH5CONbits;
extern volatile MOCK_ADCHCON_BITS AD2CH0CONbits;
extern volatile MOCK_ADCHCON_BITS AD2CH1CONbits;

typedef struct {
    unsigned CH0CMP : 1;
    unsigned CH1CMP : 1;
    unsigned CH2CMP : 1;
    unsigned CH3CMP : 1;
    unsigned CH4CMP : 1;
    unsigned CH5CMP : 1;
} MOCK_ADCMPSTAT_BITS;
extern volatile MOCK_ADCMPSTAT_BITS AD1CMPSTATbits;
extern volatile MOCK_ADCMPSTAT_BITS AD2CMPSTATbits;

/* ── Interrupt enable/flag/priority pseudo-bits ──────────────────────── */
extern volatile uint8_t _AD1CH0IE,  _AD1CH0IF,  _AD1CH0IP;
extern volatile uint8_t _AD1CMP5IE, _AD1CMP5IF, _AD1CMP5IP;
extern volatile uint8_t _AD2CMP1IE, _AD2CMP1IF, _AD2CMP1IP;
extern volatile uint8_t _CCT1IE,    _CCT1IF,    _CCT1IP;
extern volatile uint8_t _T1IE,      _T1IF,      _T1IP;
extern volatile uint8_t _PWM1IE,    _PWM1IF,    _PWM1IP;

/* ── Timer1 ──────────────────────────────────────────────────────────── */
typedef struct {
    unsigned ON    : 1;
    unsigned TSYNC : 1;
    unsigned TCS   : 1;
    unsigned TCKPS : 2;
} MOCK_T1CON_BITS;
extern volatile MOCK_T1CON_BITS T1CONbits;
extern volatile uint32_t PR1, TMR1;

/* ── PWM generator status (FLTACT/CLEVT/FLTEVT polling) ──────────────── */
typedef struct {
    unsigned FLTACT : 1;
    unsigned CLACT  : 1;
    unsigned FFACT  : 1;
    unsigned SACT   : 1;
    unsigned CLEVT  : 1;
    unsigned FLTEVT : 1;
} MOCK_PGSTAT_BITS;
extern volatile MOCK_PGSTAT_BITS PG1STATbits, PG2STATbits, PG3STATbits;
/* full-word W1C writes (PGnSTAT = mask) — separate words, SIL ignores */
extern volatile uint32_t PG1STAT, PG2STAT, PG3STAT;

/* ── GPIO ports / latches ────────────────────────────────────────────── */
extern volatile uint32_t PORTC, PORTD;

typedef struct {
    unsigned RD0 : 1; unsigned RD1 : 1; unsigned RD2 : 1; unsigned RD3 : 1;
    unsigned RD4 : 1; unsigned RD5 : 1; unsigned RD6 : 1; unsigned RD7 : 1;
    unsigned RD8 : 1; unsigned RD9 : 1; unsigned RD10 : 1;
} MOCK_PORTD_BITS;
extern volatile MOCK_PORTD_BITS PORTDbits;

typedef struct {
    unsigned LATD0 : 1; unsigned LATD1 : 1; unsigned LATD2 : 1;
    unsigned LATD3 : 1; unsigned LATD4 : 1; unsigned LATD5 : 1;
} MOCK_LATD_BITS;
extern volatile MOCK_LATD_BITS LATDbits;

typedef struct {
    unsigned LATC0 : 1; unsigned LATC1 : 1; unsigned LATC2 : 1;
    unsigned LATC3 : 1; unsigned LATC4 : 1; unsigned LATC5 : 1;
    unsigned LATC6 : 1; unsigned LATC7 : 1; unsigned LATC8 : 1;
    unsigned LATC9 : 1;
} MOCK_LATC_BITS;
extern volatile MOCK_LATC_BITS LATCbits;

#ifdef __cplusplus
}
#endif

#endif /* MOCK_XC_H */
