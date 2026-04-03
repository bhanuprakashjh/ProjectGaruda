/**
 * @file hal_ata6847.c
 * @brief ATA6847 gate driver SPI interface and initialization.
 *
 * SPI protocol: 16-bit word
 *   [15:9] = 7-bit register address
 *   [8]    = W/R (0=write, 1=read)
 *   [7:0]  = register data
 *
 * Init sequence:
 *   1. Disable watchdog
 *   2. Configure all protection/control registers
 *   3. Write all registers to chip
 *   4. On motor start: enable CSA → GDU Standby → GDU Normal → poll DSR1.GDUS
 */

#include <xc.h>
#include "../garuda_config.h"  /* FCY must be defined before libpic30.h */
#include <libpic30.h>
#include "hal_ata6847.h"
#include "hal_spi.h"
#include "hal_uart.h"

/* SPI word builder */
static inline uint16_t MakeSpiWord(uint8_t addr, uint8_t wr, uint8_t data)
{
    /* [15:9]=addr, [8]=wr, [7:0]=data */
    return ((uint16_t)(addr & 0x7F) << 9) | ((uint16_t)(wr & 1) << 8) | data;
}

void HAL_ATA6847_WriteReg(uint8_t addr, uint8_t data)
{
    HAL_SPI_Exchange16(MakeSpiWord(addr, 0, data));
}

uint8_t HAL_ATA6847_ReadReg(uint8_t addr)
{
    uint16_t rx = HAL_SPI_Exchange16(MakeSpiWord(addr, 1, 0x00));
    return (uint8_t)(rx & 0xFF);
}

void HAL_ATA6847_Init(void)
{
    uint8_t rb;

    /* Disable watchdog first (match reference exactly) */
    HAL_ATA6847_WriteReg(ATA_WDTRIG, 0x55);
    HAL_ATA6847_WriteReg(ATA_WDCR1, (0x01 << 5));  /* WDC=WGD_OFF in bits[7:5] */
    __delay_us(1);

    /* Read and clear power-up faults */
    HAL_ATA6847_ReadReg(ATA_SIR1);
    __delay_us(1);

    /* Write protection off + unused regs */
    HAL_ATA6847_WriteReg(ATA_RWPCR, 0x00);
    HAL_ATA6847_WriteReg(ATA_MLDCR, 0x00);

    /* LIN normal mode */
    HAL_ATA6847_WriteReg(ATA_LOPMCR, 0x02);

    /* Wake-up control */
    HAL_ATA6847_WriteReg(ATA_WUCR, 0x03);

    /* Current limitation — DISABLED.
     * Cycle-by-cycle chopping limits prop performance: at 24V with
     * 8" props, phase current transients during commutation reach
     * 15-25A even at mid-speed, triggering ILIM and capping eRPM.
     * VDS short-circuit protection (SCPCR) still active for hardware
     * safety. Software RECOVER mode handles desync current. */
    HAL_ATA6847_WriteReg(ATA_ILIMCR, (0 << 7) | (6 << 3) | (0 << 2));
    HAL_ATA6847_WriteReg(ATA_ILIMTH, ILIM_DAC);

    /* Short circuit protection */
    HAL_ATA6847_WriteReg(ATA_SCPCR, (1 << 7) | (7 << 3) | 7);
    /* SCTHSEL=7 (2000mV). VDS spikes scale with Vbus — at 24V the
     * switching transients exceed 1500mV at moderate current, tripping
     * SCTHSEL=5. 2000mV gives headroom for 24V operation.
     * SCTHLSEL=7 for low-side too. Still protects real shorts. */

    /* Current sense: disabled, gain=16, offset=VRef/2 */
    HAL_ATA6847_WriteReg(ATA_CSCR, (0x00 << 5) | (0x03 << 2) | 0x01);

    /* GDU OFF */
    HAL_ATA6847_WriteReg(ATA_GOPMCR, GDU_OFF);

    /* GDUCR1-4 (match reference byte values) */
    HAL_ATA6847_WriteReg(ATA_GDUCR1, (7 << 2) | (1 << 1) | 1);   /* 0x1F: BEMFEN=1 for 6-step ZC */
    /* GDUCR2: Edge blanking time + standby behavior.
     * EGBLT[3:0] = N → blanking = N × 250ns (0=off, 1=250ns, ..., 15=3750ns).
     * Was 5 (1250ns). Increased to 12 (3000ns = 3µs) — at high speed the
     * BEMF comparator sees switching transients during demag that pass
     * the software deglitch filter. HW blanking suppresses the comparator
     * output itself, which software blanking cannot do.
     * HSOFF=1, LSOFF=1: HS/LS off in standby. TSWTO=0b00: 250ns adaptive. */
    HAL_ATA6847_WriteReg(ATA_GDUCR2, (1 << 7) | (1 << 6) | 15);  /* 0xCF: EGBLT=15 (3.75µs MAX).
                                                                     * Helps at 18V. At 24V the Vbus
                                                                     * swings dominate, not EGBLT.
                                                                     * 3µs covers the ringing. */
    /* GDUCR3: Slew rate + adaptive dead-time.
     * HSSRC=0b11 (12.5%), LSSRC=0b00 (full speed), no adaptive dead time.
     * Asymmetric by design — slow HS reduces ringing. Tested alternatives:
     *   Both 50%: desync at 28k eRPM (slower LS = more noise)
     *   Both full: desync at 45k eRPM (fast HS ringing)
     *   Adaptive dead time: unreliable CL entry (3 attempts needed) */
    HAL_ATA6847_WriteReg(ATA_GDUCR3, (0x03 << 2));                /* 0x0C */
    HAL_ATA6847_WriteReg(ATA_GDUCR4, (1 << 6) | (1 << 5) | (1 << 4) | 2); /* 0x72 */

    /* Interrupt masks.
     * SIECER1 bit 5 (ILIMM) = 0: mask ILIM from nIRQ.
     * ILIM chopping is cycle-by-cycle and non-latching — it should
     * chop silently, not trigger a fault that kills the motor.
     * Was 0xE7 (ILIM unmasked), now 0xC7 (ILIM masked). */
    HAL_ATA6847_WriteReg(ATA_SIECER1, 0xC7);
    HAL_ATA6847_WriteReg(ATA_SIECER2, 0x1F);

    /* Device operation mode: Normal + RSTLVL
     * Write with delay and verify — this is critical for GDU startup */
    HAL_ATA6847_WriteReg(ATA_DOPMCR, (1 << 7) | 0x07);
    __delay_ms(2);

    /* Verify DOPMCR — retry if needed */
    rb = HAL_ATA6847_ReadReg(ATA_DOPMCR);
    HAL_UART_WriteString(" DOM=");
    HAL_UART_WriteHex8(rb);
    if ((rb & 0x07) != 0x07)
    {
        /* Device didn't accept Normal mode — try stepping through modes */
        HAL_UART_WriteString("!retry ");
        /* Standby first, then Normal */
        HAL_ATA6847_WriteReg(ATA_DOPMCR, (1 << 7) | 0x04);
        __delay_ms(5);
        HAL_ATA6847_WriteReg(ATA_DOPMCR, (1 << 7) | 0x07);
        __delay_ms(5);
        rb = HAL_ATA6847_ReadReg(ATA_DOPMCR);
        HAL_UART_WriteHex8(rb);

        if ((rb & 0x07) != 0x07)
        {
            /* Still not Normal — try without RSTLVL */
            HAL_ATA6847_WriteReg(ATA_DOPMCR, 0x07);
            __delay_ms(5);
            rb = HAL_ATA6847_ReadReg(ATA_DOPMCR);
            HAL_UART_WriteString("/");
            HAL_UART_WriteHex8(rb);
        }
    }
    HAL_UART_WriteByte(' ');

    /* Read back all critical registers for diagnostics */
    HAL_UART_WriteString("V:");
    HAL_UART_WriteHex8(HAL_ATA6847_ReadReg(ATA_GDUCR1));
    HAL_UART_WriteByte('.');
    HAL_UART_WriteHex8(HAL_ATA6847_ReadReg(ATA_CSCR));
    HAL_UART_WriteByte('.');
    HAL_UART_WriteHex8(HAL_ATA6847_ReadReg(ATA_GOPMCR));
}

/**
 * @brief Power up GDU to Normal mode.
 * Sequence: enable CSA → GDU Standby → GDU Normal → poll DSR1.GDUS.
 * @return true on success, false on timeout
 */
bool HAL_ATA6847_EnterGduNormal(void)
{
    uint16_t attempts;

    /* Debug: print DSR1/DSR2 before anything */
    HAL_UART_WriteString("pre:");
    HAL_UART_WriteHex8(HAL_ATA6847_ReadReg(ATA_DSR1));
    HAL_UART_WriteByte('/');
    HAL_UART_WriteHex8(HAL_ATA6847_ReadReg(ATA_DSR2));
    HAL_UART_WriteByte(' ');

    /* Enable all current sense amplifiers (match reference sequence) */
    uint8_t cscr = HAL_ATA6847_ReadReg(ATA_CSCR);
    cscr |= (0x07 << 5);  /* Set CSA1EN, CSA2EN, CSA3EN */
    HAL_ATA6847_WriteReg(ATA_CSCR, cscr);

    /* Verify key registers were written correctly during Init */
    HAL_UART_WriteString("GDU1=");
    HAL_UART_WriteHex8(HAL_ATA6847_ReadReg(ATA_GDUCR1));
    HAL_UART_WriteString(" DOM=");
    HAL_UART_WriteHex8(HAL_ATA6847_ReadReg(ATA_DOPMCR));
    HAL_UART_WriteString(" CSC=");
    HAL_UART_WriteHex8(HAL_ATA6847_ReadReg(ATA_CSCR));
    HAL_UART_WriteByte(' ');

    /* Standby → Normal (match reference sequence exactly) */
    HAL_ATA6847_WriteReg(ATA_GOPMCR, GDU_STANDBY);

    /* Debug: check after Standby */
    HAL_UART_WriteString("stby:");
    HAL_UART_WriteHex8(HAL_ATA6847_ReadReg(ATA_DSR1));
    HAL_UART_WriteByte(' ');

    HAL_ATA6847_WriteReg(ATA_GOPMCR, GDU_NORMAL);

    /* Debug: check immediately after Normal */
    HAL_UART_WriteString("norm:");
    HAL_UART_WriteHex8(HAL_ATA6847_ReadReg(ATA_DSR1));
    HAL_UART_WriteByte(' ');

    /* Poll DSR1.GDUS (bit 2) until GDU is ready */
    for (attempts = 0; attempts < 50000u; attempts++)
    {
        uint8_t dsr1 = HAL_ATA6847_ReadReg(ATA_DSR1);
        if (dsr1 & 0x04)   /* GDUS bit */
        {
            HAL_UART_WriteString("OK@");
            HAL_UART_WriteU16(attempts);
            HAL_UART_WriteByte(' ');
            return true;
        }
        /* Print first few poll results */
        if (attempts < 5 || attempts == 100 || attempts == 1000 ||
            attempts == 10000 || attempts == 49999)
        {
            HAL_UART_WriteString("p");
            HAL_UART_WriteU16(attempts);
            HAL_UART_WriteString("=");
            HAL_UART_WriteHex8(dsr1);
            HAL_UART_WriteByte(' ');
        }
    }

    return false;  /* Timeout — GDU failed to enter Normal */
}

/**
 * @brief Read GDU fault/status registers for diagnostics.
 * Stores results in provided array: [DSR1, DSR2, SIR1, SIR2, SIR3, GOPMCR]
 */
void HAL_ATA6847_ReadDiag(uint8_t diag[8])
{
    diag[0] = HAL_ATA6847_ReadReg(ATA_DSR1);
    diag[1] = HAL_ATA6847_ReadReg(ATA_DSR2);
    diag[2] = HAL_ATA6847_ReadReg(ATA_SIR1);
    diag[3] = HAL_ATA6847_ReadReg(ATA_SIR2);
    diag[4] = HAL_ATA6847_ReadReg(ATA_SIR3);
    diag[5] = HAL_ATA6847_ReadReg(ATA_SIR4);
    diag[6] = HAL_ATA6847_ReadReg(ATA_SIR5);
    diag[7] = HAL_ATA6847_ReadReg(ATA_GOPMCR);
}

void HAL_ATA6847_EnterGduStandby(void)
{
    HAL_ATA6847_WriteReg(ATA_GOPMCR, GDU_OFF);

    /* Disable CSA */
    uint8_t cscr = HAL_ATA6847_ReadReg(ATA_CSCR);
    cscr &= ~(0x07 << 5);
    HAL_ATA6847_WriteReg(ATA_CSCR, cscr);
}

void HAL_ATA6847_ClearFaults(void)
{
    uint8_t sir;

    HAL_ATA6847_ReadReg(ATA_DSR1);
    HAL_ATA6847_ReadReg(ATA_DSR2);

    sir = HAL_ATA6847_ReadReg(ATA_SIR1);
    HAL_ATA6847_WriteReg(ATA_SIR1, sir);
    sir = HAL_ATA6847_ReadReg(ATA_SIR2);
    HAL_ATA6847_WriteReg(ATA_SIR2, sir);
    sir = HAL_ATA6847_ReadReg(ATA_SIR3);
    HAL_ATA6847_WriteReg(ATA_SIR3, sir);
    sir = HAL_ATA6847_ReadReg(ATA_SIR4);
    HAL_ATA6847_WriteReg(ATA_SIR4, sir);
    sir = HAL_ATA6847_ReadReg(ATA_SIR5);
    HAL_ATA6847_WriteReg(ATA_SIR5, sir);
}
