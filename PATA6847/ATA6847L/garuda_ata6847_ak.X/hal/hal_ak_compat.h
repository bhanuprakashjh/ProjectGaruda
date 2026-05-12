/**
 * @file hal_ak_compat.h
 * @brief CK→AK SFR compatibility shim.
 *
 * dsPIC33CK split many 32-bit peripherals into `XxxL` (low 16) and
 * `XxxH` (high 16) sibling SFRs. dsPIC33AK exposes the same registers
 * as single 32-bit SFRs (no L/H split). The bit-field structs are
 * named `XxxLbits` on CK; on AK it's `Xxxbits`.
 *
 * To let the verbatim CK code (hal_pwm.c, hal_capture.c, etc.) compile
 * on AK without rewriting every line, this header maps the L/H half-
 * word names back onto the AK 32-bit register via pointer casts. Each
 * `XxxL` becomes the low 16 bits of `Xxx`, and `XxxH` becomes the high
 * 16 bits. Bit-field structs map similarly.
 *
 * Include this header from any source that touches PG/CCP/PCLKCON
 * SFRs and originated as a CK file.  No effect on AK-native code that
 * already uses the 32-bit names.
 */
#ifndef HAL_AK_COMPAT_H
#define HAL_AK_COMPAT_H

#include <xc.h>
#include <stdint.h>

/* [AK PORT RISK] These helpers pointer-cast a 32-bit SFR to a 16-bit
 * pointer to access half-words. Assumptions:
 *   1. Little-endian byte order — dsPIC33A is LE, so _AK_LO16 maps to
 *      bits [15:0] and _AK_HI16 maps to bits [31:16].  Confirmed by
 *      the SFR layout tables in DS70005539 (MPER, PG1CON etc.) which
 *      describe registers as `MPER[19:16] || [15:8] || [7:4] || rsvd`
 *      written left-to-right in low-address-first byte order.
 *   2. Hardware accepts independent 16-bit writes to the halves —
 *      true for memory-mapped SFRs on dsPIC. Read-modify-write of
 *      one half does NOT race with hardware updates of the other.
 *   3. Strict-aliasing — XC-DSC currently doesn't enforce TBAA over
 *      `volatile *` casts, but if a future toolchain version turns
 *      on `-fstrict-aliasing` aggressively, these casts could be
 *      optimised away. Defence in depth: keep the `volatile`. */
#define _AK_LO16(reg)   (*(volatile uint16_t*)&(reg))
#define _AK_HI16(reg)   (*((volatile uint16_t*)&(reg) + 1))

/* PWM Generator 1/2/3 control + I/O + event + fault + LEB registers. */
#define PG1CONL     _AK_LO16(PG1CON)
#define PG1CONH     _AK_HI16(PG1CON)
#define PG2CONL     _AK_LO16(PG2CON)
#define PG2CONH     _AK_HI16(PG2CON)
#define PG3CONL     _AK_LO16(PG3CON)
#define PG3CONH     _AK_HI16(PG3CON)

#define PG1IOCONL   _AK_LO16(PG1IOCON)
#define PG1IOCONH   _AK_HI16(PG1IOCON)
#define PG2IOCONL   _AK_LO16(PG2IOCON)
#define PG2IOCONH   _AK_HI16(PG2IOCON)
#define PG3IOCONL   _AK_LO16(PG3IOCON)
#define PG3IOCONH   _AK_HI16(PG3IOCON)

#define PG1EVTL     _AK_LO16(PG1EVT)
#define PG1EVTH     _AK_HI16(PG1EVT)
#define PG2EVTL     _AK_LO16(PG2EVT)
#define PG2EVTH     _AK_HI16(PG2EVT)
#define PG3EVTL     _AK_LO16(PG3EVT)
#define PG3EVTH     _AK_HI16(PG3EVT)

#define PG1FPCIL    _AK_LO16(PG1FPCI)
#define PG1FPCIH    _AK_HI16(PG1FPCI)
#define PG2FPCIL    _AK_LO16(PG2FPCI)
#define PG2FPCIH    _AK_HI16(PG2FPCI)
#define PG3FPCIL    _AK_LO16(PG3FPCI)
#define PG3FPCIH    _AK_HI16(PG3FPCI)

#define PG1LEBL     _AK_LO16(PG1LEB)
#define PG1LEBH     _AK_HI16(PG1LEB)
#define PG2LEBL     _AK_LO16(PG2LEB)
#define PG2LEBH     _AK_HI16(PG2LEB)
#define PG3LEBL     _AK_LO16(PG3LEB)
#define PG3LEBH     _AK_HI16(PG3LEB)

/* CCP1/2/3/4 register halves (CCP5 doesn't exist on AK — shimmed
 * separately as a dummy variable in motor/sector_pi.c). */
#define CCP1CON1L   _AK_LO16(CCP1CON1)
#define CCP1CON1H   _AK_HI16(CCP1CON1)
#define CCP2CON1L   _AK_LO16(CCP2CON1)
#define CCP2CON1H   _AK_HI16(CCP2CON1)
#define CCP3CON1L   _AK_LO16(CCP3CON1)
#define CCP3CON1H   _AK_HI16(CCP3CON1)
#define CCP4CON1L   _AK_LO16(CCP4CON1)
#define CCP4CON1H   _AK_HI16(CCP4CON1)

#define CCP1TMRL    _AK_LO16(CCP1TMR)
#define CCP2TMRL    _AK_LO16(CCP2TMR)
#define CCP3TMRL    _AK_LO16(CCP3TMR)
#define CCP4TMRL    _AK_LO16(CCP4TMR)

/* PCI low-level fault / capture-light inputs — same single-32-bit
 * collapse as the main PG registers. */
#define PG1CLPCIL   _AK_LO16(PG1CLPCI)
#define PG1CLPCIH   _AK_HI16(PG1CLPCI)
#define PG2CLPCIL   _AK_LO16(PG2CLPCI)
#define PG2CLPCIH   _AK_HI16(PG2CLPCI)
#define PG3CLPCIL   _AK_LO16(PG3CLPCI)
#define PG3CLPCIH   _AK_HI16(PG3CLPCI)

/* Feed-forward fault PCI registers. */
#define PG1FFPCIL   _AK_LO16(PG1FFPCI)
#define PG1FFPCIH   _AK_HI16(PG1FFPCI)
#define PG2FFPCIL   _AK_LO16(PG2FFPCI)
#define PG2FFPCIH   _AK_HI16(PG2FFPCI)
#define PG3FFPCIL   _AK_LO16(PG3FFPCI)
#define PG3FFPCIH   _AK_HI16(PG3FFPCI)

/* Sync-source PCI registers. */
#define PG1SPCIL    _AK_LO16(PG1SPCI)
#define PG1SPCIH    _AK_HI16(PG1SPCI)
#define PG2SPCIL    _AK_LO16(PG2SPCI)
#define PG2SPCIH    _AK_HI16(PG2SPCI)
#define PG3SPCIL    _AK_LO16(PG3SPCI)
#define PG3SPCIH    _AK_HI16(PG3SPCI)

/* Dead-time registers — note: CK uses PG1DTL (low/rising) + PG1DTH
 * (high/falling) as two SEPARATE 16-bit registers. AK uses PG1DT as
 * one 32-bit register where bits [15:0] = DTL and bits [31:16] = DTH.
 * Same byte layout, so the half-word aliases work as drop-in. */
#define PG1DTL      _AK_LO16(PG1DT)
#define PG1DTH      _AK_HI16(PG1DT)
#define PG2DTL      _AK_LO16(PG2DT)
#define PG2DTH      _AK_HI16(PG2DT)
#define PG3DTL      _AK_LO16(PG3DT)
#define PG3DTH      _AK_HI16(PG3DT)

#define CCP1PRL     _AK_LO16(CCP1PR)
#define CCP2PRL     _AK_LO16(CCP2PR)
#define CCP3PRL     _AK_LO16(CCP3PR)
#define CCP4PRL     _AK_LO16(CCP4PR)

#define CCP1BUFL    _AK_LO16(CCP1BUF)
#define CCP2BUFL    _AK_LO16(CCP2BUF)
#define CCP3BUFL    _AK_LO16(CCP3BUF)
#define CCP4BUFL    _AK_LO16(CCP4BUF)

/* L-suffixed bit-field accessors — XxxLbits → Xxxbits. */
#define PG1CONLbits     PG1CONbits
#define PG2CONLbits     PG2CONbits
#define PG3CONLbits     PG3CONbits
#define PG1IOCONLbits   PG1IOCONbits
#define PG2IOCONLbits   PG2IOCONbits
#define PG3IOCONLbits   PG3IOCONbits
#define PG1EVTLbits     PG1EVTbits
#define PG2EVTLbits     PG2EVTbits
#define PG3EVTLbits     PG3EVTbits
#define CCP1CON1Lbits   CCP1CON1bits
#define CCP2CON1Lbits   CCP2CON1bits
#define CCP3CON1Lbits   CCP3CON1bits
#define CCP4CON1Lbits   CCP4CON1bits
#define CCP1STATLbits   CCP1STATbits
#define CCP2STATLbits   CCP2STATbits
#define CCP3STATLbits   CCP3STATbits
#define CCP4STATLbits   CCP4STATbits

#endif /* HAL_AK_COMPAT_H */
