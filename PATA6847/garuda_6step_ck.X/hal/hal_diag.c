/**
 * @file hal_diag.c
 * @brief Interactive diagnostic commands via UART.
 *
 * Single-character commands (type in terminal):
 *   h  — Help (list commands)
 *   a  — ATA6847 register dump (all readable registers)
 *   b  — BEMF comparator pin states (RC6/RC7/RD10)
 *   d  — ADC readings (pot, Vbus, phase currents)
 *   p  — PWM register dump (IOCONL, DC, MPER, DT)
 *   c  — Clock/PLL status registers
 *   s  — Full state dump (ESC state, timing, BEMF, counters)
 *   g  — GDU power-up test (enter Normal, read DSR1)
 *   f  — Read and clear ATA6847 faults (SIR1-5)
 *   t  — Toggle LED_RUN (visual heartbeat test)
 *   1  — Manual commutation step (advance +1)
 *   0  — Force all phases float (safe state)
 *   r  — Read single ADC snapshot (enable ADC, read, disable)
 *   i  — GPIO pin state dump (PORTA/B/C/D)
 *   m  — SPI loopback: write/read ATA6847 WUCR register
 *   v  — Verbose continuous mode toggle (100ms status)
 *   x  — IC ZC stats (FEATURE_IC_ZC only)
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "hal_diag.h"
#include "hal_uart.h"
#include "hal_ata6847.h"
#include "hal_pwm.h"
#include "hal_adc.h"
#include "hal_opa.h"
#include "port_config.h"
#include "../garuda_config.h"
#include "../garuda_service.h"
#include "../motor/commutation.h"

/* Verbose mode flag — toggled by 'v' command */
static bool verboseMode = false;
bool DIAG_IsVerbose(void) { return verboseMode; }

/* Helper: print register name and 16-bit hex value */
static void PrintReg16(const char *name, uint16_t val)
{
    HAL_UART_WriteString(name);
    HAL_UART_WriteString("=0x");
    HAL_UART_WriteHex16(val);
    HAL_UART_WriteByte(' ');
}

/* Helper: print register name and 8-bit hex value */
static void PrintReg8(const char *name, uint8_t val)
{
    HAL_UART_WriteString(name);
    HAL_UART_WriteString("=0x");
    HAL_UART_WriteHex8(val);
    HAL_UART_WriteByte(' ');
}

#if FEATURE_IC_ZC
static void CmdIcStats(void);
#endif

/* ── 'h' — Help ───────────────────────────────────────────────────── */
static void CmdHelp(void)
{
    HAL_UART_WriteString("--- Diag Commands ---");
    HAL_UART_NewLine();
    HAL_UART_WriteString("h=Help a=ATA regs b=BEMF pins d=ADC");
    HAL_UART_NewLine();
    HAL_UART_WriteString("p=PWM regs c=Clock s=State dump");
    HAL_UART_NewLine();
    HAL_UART_WriteString("g=GDU test f=Faults t=LED toggle");
    HAL_UART_NewLine();
    HAL_UART_WriteString("1=Step+1 0=AllFloat r=ADC snap");
    HAL_UART_NewLine();
    HAL_UART_WriteString("i=GPIO pins m=SPI test v=Verbose");
    HAL_UART_NewLine();
#if FEATURE_IC_ZC
    HAL_UART_WriteString("x=IC ZC stats");
    HAL_UART_NewLine();
#endif
}

/* ── 'a' — ATA6847 register dump ──────────────────────────────────── */
static void CmdAtaRegs(void)
{
    HAL_UART_WriteString("ATA6847 Regs:");
    HAL_UART_NewLine();

    PrintReg8("DOPMCR",  HAL_ATA6847_ReadReg(ATA_DOPMCR));
    PrintReg8("LOPMCR",  HAL_ATA6847_ReadReg(ATA_LOPMCR));
    PrintReg8("GOPMCR",  HAL_ATA6847_ReadReg(ATA_GOPMCR));
    HAL_UART_NewLine();

    PrintReg8("GDUCR1",  HAL_ATA6847_ReadReg(ATA_GDUCR1));
    PrintReg8("GDUCR2",  HAL_ATA6847_ReadReg(ATA_GDUCR2));
    PrintReg8("GDUCR3",  HAL_ATA6847_ReadReg(ATA_GDUCR3));
    PrintReg8("GDUCR4",  HAL_ATA6847_ReadReg(ATA_GDUCR4));
    HAL_UART_NewLine();

    PrintReg8("ILIMCR",  HAL_ATA6847_ReadReg(ATA_ILIMCR));
    PrintReg8("ILIMTH",  HAL_ATA6847_ReadReg(ATA_ILIMTH));
    PrintReg8("SCPCR",   HAL_ATA6847_ReadReg(ATA_SCPCR));
    PrintReg8("CSCR",    HAL_ATA6847_ReadReg(ATA_CSCR));
    HAL_UART_NewLine();

    PrintReg8("DSR1",    HAL_ATA6847_ReadReg(ATA_DSR1));
    PrintReg8("DSR2",    HAL_ATA6847_ReadReg(ATA_DSR2));
    HAL_UART_NewLine();

    PrintReg8("WDCR1",   HAL_ATA6847_ReadReg(ATA_WDCR1));
    PrintReg8("WUCR",    HAL_ATA6847_ReadReg(ATA_WUCR));
    PrintReg8("MLDCR",   HAL_ATA6847_ReadReg(ATA_MLDCR));
    PrintReg8("RWPCR",   HAL_ATA6847_ReadReg(ATA_RWPCR));
    HAL_UART_NewLine();

    PrintReg8("SIECER1", HAL_ATA6847_ReadReg(ATA_SIECER1));
    PrintReg8("SIECER2", HAL_ATA6847_ReadReg(ATA_SIECER2));
    HAL_UART_NewLine();

    /* Decode key fields */
    uint8_t gopmcr = HAL_ATA6847_ReadReg(ATA_GOPMCR);
    HAL_UART_WriteString("GDU mode: ");
    if (gopmcr == GDU_NORMAL) HAL_UART_WriteString("NORMAL");
    else if (gopmcr == GDU_STANDBY) HAL_UART_WriteString("STANDBY");
    else if (gopmcr == GDU_OFF) HAL_UART_WriteString("OFF");
    else { HAL_UART_WriteString("?0x"); HAL_UART_WriteHex8(gopmcr); }
    HAL_UART_NewLine();

    uint8_t dsr1 = HAL_ATA6847_ReadReg(ATA_DSR1);
    HAL_UART_WriteString("DSR1 bits: GDUS=");
    HAL_UART_WriteByte((dsr1 & 0x04) ? '1' : '0');
    HAL_UART_WriteString(" CPOK=");
    HAL_UART_WriteByte((dsr1 & 0x02) ? '1' : '0');
    HAL_UART_WriteString(" VSUVS=");
    HAL_UART_WriteByte((dsr1 & 0x01) ? '1' : '0');
    HAL_UART_NewLine();

    uint8_t gducr1 = HAL_ATA6847_ReadReg(ATA_GDUCR1);
    HAL_UART_WriteString("BEMFEN=");
    HAL_UART_WriteByte((gducr1 & 0x01) ? '1' : '0');
    HAL_UART_WriteString(" CCEN=");
    HAL_UART_WriteByte((gducr1 & 0x02) ? '1' : '0');
    HAL_UART_WriteString(" CCPT=");
    HAL_UART_WriteU16((gducr1 >> 2) & 0x3F);
    HAL_UART_NewLine();
}

/* ── 'b' — BEMF pin states ────────────────────────────────────────── */
static void CmdBemfPins(void)
{
    HAL_UART_WriteString("BEMF: A(RC6)=");
    HAL_UART_WriteByte(BEMF_A_GetValue() ? '1' : '0');
    HAL_UART_WriteString(" B(RC7)=");
    HAL_UART_WriteByte(BEMF_B_GetValue() ? '1' : '0');
    HAL_UART_WriteString(" C(RD10)=");
    HAL_UART_WriteByte(BEMF_C_GetValue() ? '1' : '0');

    HAL_UART_WriteString("  nIRQ(RD1)=");
    HAL_UART_WriteByte(nIRQ_GetValue() ? '1' : '0');
    HAL_UART_NewLine();
}

/* ── 'd' — ADC readings ───────────────────────────────────────────── */
static void CmdAdc(void)
{
    HAL_UART_WriteString("ADC: Pot=");
    HAL_UART_WriteU16(gData.potRaw);
    HAL_UART_WriteString(" Vbus=");
    HAL_UART_WriteU16(gData.vbusRaw);

    /* Compute Vbus in millivolts: Vbus = ADC * 3300 / 4096 * KVbus */
    /* KVbus=16, so Vbus_mV = ADC * 3300 * 16 / 4096 = ADC * 12.89 */
    uint32_t vbusMv = ((uint32_t)gData.vbusRaw * 3300UL * 16UL) / 4096UL;
    HAL_UART_WriteString(" (");
    HAL_UART_WriteU32(vbusMv / 1000);
    HAL_UART_WriteByte('.');
    HAL_UART_WriteU16((uint16_t)(vbusMv % 1000));
    HAL_UART_WriteString("V)");

    HAL_UART_WriteString(" ADON=");
    HAL_UART_WriteByte(ADCON1Lbits.ADON ? '1' : '0');
    HAL_UART_NewLine();
}

/* ── 'p' — PWM register dump ──────────────────────────────────────── */
static void CmdPwm(void)
{
    HAL_UART_WriteString("PWM Regs:");
    HAL_UART_NewLine();

    PrintReg16("MPER", MPER);
    PrintReg16("PCLKCON", PCLKCON);
    HAL_UART_NewLine();

    HAL_UART_WriteString("PG1: ");
    PrintReg16("DC", PG1DC);
    PrintReg16("IOCONL", PG1IOCONL);
    PrintReg16("IOCONH", PG1IOCONH);
    PrintReg16("CONL", PG1CONL);
    HAL_UART_NewLine();

    HAL_UART_WriteString("PG2: ");
    PrintReg16("DC", PG2DC);
    PrintReg16("IOCONL", PG2IOCONL);
    HAL_UART_NewLine();

    HAL_UART_WriteString("PG3: ");
    PrintReg16("DC", PG3DC);
    PrintReg16("IOCONL", PG3IOCONL);
    HAL_UART_NewLine();

    /* Decode override state per phase */
    HAL_UART_WriteString("Overrides: ");
    uint16_t pg1 = PG1IOCONL, pg2 = PG2IOCONL, pg3 = PG3IOCONL;

    const char *phaseLabel[] = {"A:", "B:", "C:"};
    uint16_t regs[] = {pg1, pg2, pg3};
    uint8_t i;
    for (i = 0; i < 3; i++)
    {
        HAL_UART_WriteString(phaseLabel[i]);
        bool ovH = (regs[i] & 0x2000) != 0;
        bool ovL = (regs[i] & 0x1000) != 0;
        uint8_t dat = (uint8_t)((regs[i] >> 8) & 0x03);

        if (!ovH && !ovL)
            HAL_UART_WriteString("PWM ");
        else if (ovH && ovL && dat == 0x01)
            HAL_UART_WriteString("LOW ");
        else if (ovH && ovL && dat == 0x00)
            HAL_UART_WriteString("FLT ");
        else
        {
            HAL_UART_WriteString("?");
            HAL_UART_WriteHex8((uint8_t)(regs[i] >> 8));
            HAL_UART_WriteByte(' ');
        }
    }
    HAL_UART_NewLine();

    PrintReg16("DTL", PG1DTL);
    PrintReg16("DTH", PG1DTH);
    HAL_UART_WriteString("ON=");
    HAL_UART_WriteByte(PG1CONLbits.ON ? '1' : '0');
    HAL_UART_NewLine();
}

/* ── 'c' — Clock status ───────────────────────────────────────────── */
static void CmdClock(void)
{
    HAL_UART_WriteString("Clock:");
    HAL_UART_NewLine();
    PrintReg16("OSCCON", OSCCON);
    PrintReg16("CLKDIV", CLKDIV);
    PrintReg16("PLLFBD", PLLFBD);
    PrintReg16("PLLDIV", PLLDIV);
    HAL_UART_NewLine();

    HAL_UART_WriteString("LOCK=");
    HAL_UART_WriteByte(OSCCONbits.LOCK ? '1' : '0');
    HAL_UART_WriteString(" COSC=");
    HAL_UART_WriteU16(OSCCONbits.COSC);
    HAL_UART_NewLine();
}

/* ── 's' — Full state dump ─────────────────────────────────────────── */
static void CmdState(void)
{
    static const char *stateNames[] = {
        "IDLE", "ARMED", "ALIGN", "OL_RAMP", "CL", "RECOV", "FAULT"
    };
    static const char *faultNames[] = {
        "NONE", "OV", "UV", "STALL", "DESYNC", "START_TO", "ATA"
    };

    HAL_UART_WriteString("=== ESC State ===");
    HAL_UART_NewLine();

    HAL_UART_WriteString("State: ");
    if (gData.state <= ESC_FAULT)
        HAL_UART_WriteString(stateNames[gData.state]);
    HAL_UART_WriteString("  Fault: ");
    if (gData.faultCode <= FAULT_ATA6847)
        HAL_UART_WriteString(faultNames[gData.faultCode]);
    HAL_UART_NewLine();

    HAL_UART_WriteString("Step=");
    HAL_UART_WriteU16(gData.currentStep);
    HAL_UART_WriteString(" Dir=");
    HAL_UART_WriteU16(gData.direction);
    HAL_UART_WriteString(" Duty=");
    HAL_UART_WriteU32(gData.duty);
    HAL_UART_NewLine();

    HAL_UART_WriteString("Pot=");
    HAL_UART_WriteU16(gData.potRaw);
    HAL_UART_WriteString(" Vbus=");
    HAL_UART_WriteU16(gData.vbusRaw);
    HAL_UART_WriteString(" Thr=");
    HAL_UART_WriteU16(gData.throttle);
    HAL_UART_NewLine();

    HAL_UART_WriteString("SysTick=");
    HAL_UART_WriteU32(gData.systemTick);
    HAL_UART_WriteString(" T1Tick=");
    HAL_UART_WriteU16(gData.timer1Tick);
    HAL_UART_NewLine();

    HAL_UART_WriteString("AlignCnt=");
    HAL_UART_WriteU32(gData.alignCounter);
    HAL_UART_WriteString(" RampPer=");
    HAL_UART_WriteU32(gData.rampStepPeriod);
    HAL_UART_WriteString(" RampCnt=");
    HAL_UART_WriteU32(gData.rampCounter);
    HAL_UART_NewLine();

    HAL_UART_WriteString("RunCmd=");
    HAL_UART_WriteByte(gData.runCommandActive ? '1' : '0');
    HAL_UART_WriteString(" DesyncRetry=");
    HAL_UART_WriteU16(gData.desyncRestartAttempts);
    HAL_UART_WriteString(" RecovCnt=");
    HAL_UART_WriteU32(gData.recoveryCounter);
    HAL_UART_NewLine();

    HAL_UART_WriteString("--- BEMF ---");
    HAL_UART_NewLine();
    HAL_UART_WriteString("ZcDet=");
    HAL_UART_WriteByte(gData.bemf.zeroCrossDetected ? '1' : '0');
    HAL_UART_WriteString(" CmpPrev=");
    HAL_UART_WriteHex8(gData.bemf.cmpPrev);
    HAL_UART_WriteString(" CmpExp=");
    HAL_UART_WriteU16(gData.bemf.cmpExpected);
    HAL_UART_WriteString(" FiltCnt=");
    HAL_UART_WriteU16(gData.bemf.filterCount);
    HAL_UART_NewLine();

    HAL_UART_WriteString("--- Timing ---");
    HAL_UART_NewLine();
    HAL_UART_WriteString("StepPer=");
    HAL_UART_WriteU16(gData.timing.stepPeriod);
    HAL_UART_WriteString(" LastComm=");
    HAL_UART_WriteU16(gData.timing.lastCommTick);
    HAL_UART_WriteString(" LastZc=");
    HAL_UART_WriteU16(gData.timing.lastZcTick);
    HAL_UART_NewLine();

    HAL_UART_WriteString("ZcInt=");
    HAL_UART_WriteU16(gData.timing.zcInterval);
    HAL_UART_WriteString(" CommDL=");
    HAL_UART_WriteU16(gData.timing.commDeadline);
    HAL_UART_WriteString(" ForcCD=");
    HAL_UART_WriteU16(gData.timing.forcedCountdown);
    HAL_UART_NewLine();

    HAL_UART_WriteString("GoodZC=");
    HAL_UART_WriteU16(gData.timing.goodZcCount);
    HAL_UART_WriteString(" MissSteps=");
    HAL_UART_WriteU16(gData.timing.consecutiveMissedSteps);
    HAL_UART_WriteString(" SinceZC=");
    HAL_UART_WriteU16(gData.timing.stepsSinceLastZc);
    HAL_UART_NewLine();

    HAL_UART_WriteString("Synced=");
    HAL_UART_WriteByte(gData.timing.zcSynced ? '1' : '0');
    HAL_UART_WriteString(" DLactive=");
    HAL_UART_WriteByte(gData.timing.deadlineActive ? '1' : '0');
    HAL_UART_WriteString(" HasPrevZc=");
    HAL_UART_WriteByte(gData.timing.hasPrevZc ? '1' : '0');
    HAL_UART_NewLine();

    /* Also show BEMF pins and nIRQ */
    CmdBemfPins();

#if FEATURE_IC_ZC
    CmdIcStats();
#endif
}

/* ── 'g' — GDU power-up test ──────────────────────────────────────── */
static void CmdGduTest(void)
{
    HAL_UART_WriteString("GDU power-up test...");
    HAL_UART_NewLine();

    HAL_ATA6847_ClearFaults();

    /* Read DSR1 before */
    PrintReg8("DSR1(pre)", HAL_ATA6847_ReadReg(ATA_DSR1));
    HAL_UART_NewLine();

    bool ok = HAL_ATA6847_EnterGduNormal();
    HAL_UART_WriteString("EnterGduNormal: ");
    HAL_UART_WriteString(ok ? "OK" : "FAIL (timeout)");
    HAL_UART_NewLine();

    /* Read DSR1 after */
    uint8_t dsr1 = HAL_ATA6847_ReadReg(ATA_DSR1);
    PrintReg8("DSR1(post)", dsr1);
    HAL_UART_WriteString(" GDUS=");
    HAL_UART_WriteByte((dsr1 & 0x04) ? '1' : '0');
    HAL_UART_NewLine();

    /* Read GOPMCR */
    PrintReg8("GOPMCR", HAL_ATA6847_ReadReg(ATA_GOPMCR));
    HAL_UART_NewLine();

    /* Leave GDU on for further testing — user can press '0' to float */
}

/* ── 'f' — Read and clear ATA faults ──────────────────────────────── */
static void CmdFaults(void)
{
    HAL_UART_WriteString("ATA Fault/Status Regs:");
    HAL_UART_NewLine();

    PrintReg8("DSR1", HAL_ATA6847_ReadReg(ATA_DSR1));
    PrintReg8("DSR2", HAL_ATA6847_ReadReg(ATA_DSR2));
    HAL_UART_NewLine();

    uint8_t sir;
    sir = HAL_ATA6847_ReadReg(ATA_SIR1);
    PrintReg8("SIR1", sir);
    if (sir) { HAL_ATA6847_WriteReg(ATA_SIR1, sir); HAL_UART_WriteString("(clr) "); }

    sir = HAL_ATA6847_ReadReg(ATA_SIR2);
    PrintReg8("SIR2", sir);
    if (sir) { HAL_ATA6847_WriteReg(ATA_SIR2, sir); HAL_UART_WriteString("(clr) "); }

    sir = HAL_ATA6847_ReadReg(ATA_SIR3);
    PrintReg8("SIR3", sir);
    if (sir) { HAL_ATA6847_WriteReg(ATA_SIR3, sir); HAL_UART_WriteString("(clr) "); }

    sir = HAL_ATA6847_ReadReg(ATA_SIR4);
    PrintReg8("SIR4", sir);
    if (sir) { HAL_ATA6847_WriteReg(ATA_SIR4, sir); HAL_UART_WriteString("(clr) "); }

    sir = HAL_ATA6847_ReadReg(ATA_SIR5);
    PrintReg8("SIR5", sir);
    if (sir) { HAL_ATA6847_WriteReg(ATA_SIR5, sir); HAL_UART_WriteString("(clr) "); }
    HAL_UART_NewLine();

    /* Decode SIR1 bits */
    sir = HAL_ATA6847_ReadReg(ATA_SIR1);
    if (sir == 0)
    {
        HAL_UART_WriteString("No active faults in SIR1");
    }
    else
    {
        if (sir & 0x80) HAL_UART_WriteString("GSC ");
        if (sir & 0x40) HAL_UART_WriteString("VDHOV ");
        if (sir & 0x20) HAL_UART_WriteString("ILIM ");
        if (sir & 0x04) HAL_UART_WriteString("VSUV ");
        if (sir & 0x02) HAL_UART_WriteString("SPIF ");
        if (sir & 0x01) HAL_UART_WriteString("OTPW ");
    }
    HAL_UART_NewLine();

    HAL_UART_WriteString("nIRQ=");
    HAL_UART_WriteByte(nIRQ_GetValue() ? '1' : '0');
    HAL_UART_NewLine();
}

/* ── 't' — LED toggle test ─────────────────────────────────────────── */
static void CmdLedToggle(void)
{
    LED_RUN ^= 1;
    HAL_UART_WriteString("LED_RUN=");
    HAL_UART_WriteByte(LED_RUN ? '1' : '0');
    HAL_UART_NewLine();
}

/* ── '1' — Manual step advance ─────────────────────────────────────── */
static void CmdStepAdvance(void)
{
    if (gData.state != ESC_IDLE)
    {
        HAL_UART_WriteString("ERR: not IDLE");
        HAL_UART_NewLine();
        return;
    }

    gData.currentStep++;
    if (gData.currentStep >= 6) gData.currentStep = 0;
    HAL_PWM_SetCommutationStep(gData.currentStep);

    HAL_UART_WriteString("Step=");
    HAL_UART_WriteU16(gData.currentStep);

    /* Show what each phase is doing */
    const COMMUTATION_STEP_T *s = &commutationTable[gData.currentStep];
    const char *stateStr[] = {"PWM", "LOW", "FLT"};
    HAL_UART_WriteString(" A=");
    HAL_UART_WriteString(stateStr[s->phaseA]);
    HAL_UART_WriteString(" B=");
    HAL_UART_WriteString(stateStr[s->phaseB]);
    HAL_UART_WriteString(" C=");
    HAL_UART_WriteString(stateStr[s->phaseC]);
    HAL_UART_WriteString(" Float=");
    HAL_UART_WriteByte('A' + s->floatingPhase);
    HAL_UART_WriteString(" ZCpol=");
    HAL_UART_WriteByte(s->zcPolarity > 0 ? '+' : '-');
    HAL_UART_NewLine();

    /* Show BEMF pin for the floating phase */
    CmdBemfPins();
}

/* ── '0' — Force all float ─────────────────────────────────────────── */
static void CmdForceFloat(void)
{
    HAL_PWM_ForceAllFloat();
    HAL_UART_WriteString("All phases FLOAT");
    HAL_UART_NewLine();
}

/* ── 'r' — Read single ADC snapshot ────────────────────────────────── */
static void CmdAdcSnap(void)
{
    /* Temporarily enable ADC, do a software trigger, read results */
    HAL_OPA_Enable();
    ADCON1Lbits.ADON = 1;

    /* Software trigger shared core channels */
    ADCON3Lbits.SWCTRG = 1;

    /* Brief wait for conversion (~1µs at 100MHz) */
    volatile uint16_t delay;
    for (delay = 0; delay < 200; delay++);

    HAL_UART_WriteString("ADC Snap: ");
    HAL_UART_WriteString("AN0(Ic)=");
    HAL_UART_WriteU16(ADCBUF0);
    HAL_UART_WriteString(" AN1(Ia)=");
    HAL_UART_WriteU16(ADCBUF1);
    HAL_UART_WriteString(" AN6(Pot)=");
    HAL_UART_WriteU16(ADCBUF6);
    HAL_UART_WriteString(" AN9(Vbus)=");
    HAL_UART_WriteU16(ADCBUF9);
    HAL_UART_NewLine();

    /* Don't disable ADC if motor is running */
    if (gData.state == ESC_IDLE)
    {
        ADCON1Lbits.ADON = 0;
        HAL_OPA_Disable();
    }
}

/* ── 'i' — GPIO pin state dump ─────────────────────────────────────── */
static void CmdGpio(void)
{
    HAL_UART_WriteString("GPIO:");
    HAL_UART_NewLine();
    PrintReg16("PORTA", PORTA);
    PrintReg16("TRISA", TRISA);
    PrintReg16("ANSELA", ANSELA);
    HAL_UART_NewLine();
    PrintReg16("PORTB", PORTB);
    PrintReg16("TRISB", TRISB);
    PrintReg16("ANSELB", ANSELB);
    HAL_UART_NewLine();
    PrintReg16("PORTC", PORTC);
    PrintReg16("TRISC", TRISC);
    PrintReg16("ANSELC", ANSELC);
    HAL_UART_NewLine();
    PrintReg16("PORTD", PORTD);
    PrintReg16("TRISD", TRISD);
    PrintReg16("ANSELD", ANSELD);
    HAL_UART_NewLine();

    /* Decode specific pins */
    HAL_UART_WriteString("BTN1(RC0)=");
    HAL_UART_WriteByte(BTN1_GetValue() ? '1' : '0');
    HAL_UART_WriteString(" BTN2(RD13)=");
    HAL_UART_WriteByte(BTN2_GetValue() ? '1' : '0');
    HAL_UART_WriteString(" nCS(RC11)=");
    HAL_UART_WriteByte(nCS ? '1' : '0');
    HAL_UART_NewLine();
}

/* ── 'm' — SPI loopback test ──────────────────────────────────────── */
static void CmdSpiTest(void)
{
    HAL_UART_WriteString("SPI test: write WUCR=0xAA, readback...");
    HAL_UART_NewLine();

    /* Save original */
    uint8_t orig = HAL_ATA6847_ReadReg(ATA_WUCR);
    PrintReg8("WUCR(orig)", orig);

    /* Write test pattern */
    HAL_ATA6847_WriteReg(ATA_WUCR, 0x03);
    uint8_t rb = HAL_ATA6847_ReadReg(ATA_WUCR);
    PrintReg8("WUCR(read)", rb);

    if (rb == 0x03)
        HAL_UART_WriteString(" PASS");
    else
        HAL_UART_WriteString(" FAIL");
    HAL_UART_NewLine();

    /* Restore */
    HAL_ATA6847_WriteReg(ATA_WUCR, orig);
}

#if FEATURE_IC_ZC
/* ── 'x' — IC ZC diagnostics ─────────────────────────────────────── */
static void CmdIcStats(void)
{
    static const char *phaseNames[] = {
        "BLANK", "ARMED", "PEND", "CONF", "DONE"
    };

    HAL_UART_WriteString("=== IC ZC Stats ===");
    HAL_UART_NewLine();

    HAL_UART_WriteString("Phase: ");
    if (gData.icZc.phase <= IC_ZC_DONE)
        HAL_UART_WriteString(phaseNames[gData.icZc.phase]);
    HAL_UART_WriteString("  Chan=");
    if (gData.icZc.activeChannel < 3)
        HAL_UART_WriteByte('A' + gData.icZc.activeChannel);
    else
        HAL_UART_WriteByte('-');
    HAL_UART_NewLine();

    HAL_UART_WriteString("FP_Acc=");
    HAL_UART_WriteU16(gData.icZc.diagAccepted);
    HAL_UART_WriteString(" Backup_Acc=");
    HAL_UART_WriteU16(gData.icZc.diagLcoutAccepted);
    HAL_UART_WriteString(" FalseZC=");
    HAL_UART_WriteU16(gData.icZc.diagFalseZc);
    HAL_UART_WriteString(" PollCyc=");
    HAL_UART_WriteU16(gData.icZc.diagPollCycles);
    HAL_UART_NewLine();

    HAL_UART_WriteString("BlankEndHR=");
    HAL_UART_WriteU16(gData.icZc.blankingEndHR);
    HAL_UART_WriteString(" FilterLvl=");
    HAL_UART_WriteU16(gData.icZc.filterLevel);
    HAL_UART_WriteString(" PollFilt=");
    HAL_UART_WriteU16(gData.icZc.pollFilter);
    HAL_UART_NewLine();

    /* Acceptance ratio: fast poll vs ADC backup */
    {
        uint16_t totalAcc = gData.icZc.diagAccepted + gData.icZc.diagLcoutAccepted;
        if (totalAcc > 0)
        {
            uint16_t fpPct = (uint16_t)((uint32_t)gData.icZc.diagAccepted * 100 / totalAcc);
            HAL_UART_WriteString("FP%=");
            HAL_UART_WriteU16(fpPct);
            HAL_UART_WriteString(" Backup%=");
            HAL_UART_WriteU16(100 - fpPct);
        }
    }
    HAL_UART_NewLine();
}
#endif /* FEATURE_IC_ZC */

/* ── 'v' — Verbose toggle ─────────────────────────────────────────── */
static void CmdVerbose(void)
{
    verboseMode = !verboseMode;
    HAL_UART_WriteString("Verbose: ");
    HAL_UART_WriteString(verboseMode ? "ON (100ms)" : "OFF (500ms)");
    HAL_UART_NewLine();
}

/* ── Command Dispatcher ───────────────────────────────────────────── */

void DIAG_ProcessCommand(uint8_t cmd)
{
    switch (cmd)
    {
        case 'h': case 'H': case '?': CmdHelp(); break;
        case 'a': case 'A': CmdAtaRegs(); break;
        case 'b': case 'B': CmdBemfPins(); break;
        case 'd': case 'D': CmdAdc(); break;
        case 'p': case 'P': CmdPwm(); break;
        case 'c': case 'C': CmdClock(); break;
        case 's': case 'S': CmdState(); break;
        case 'g': case 'G': CmdGduTest(); break;
        case 'f': case 'F': CmdFaults(); break;
        case 't': case 'T': CmdLedToggle(); break;
        case '1':           CmdStepAdvance(); break;
        case '0':           CmdForceFloat(); break;
        case 'r': case 'R': CmdAdcSnap(); break;
        case 'i': case 'I': CmdGpio(); break;
        case 'm': case 'M': CmdSpiTest(); break;
        case 'v': case 'V': CmdVerbose(); break;
#if FEATURE_IC_ZC
        case 'x': case 'X': CmdIcStats(); break;
#endif
        case '\r': case '\n': break;  /* Ignore newlines */
        default:
            HAL_UART_WriteString("? '");
            HAL_UART_WriteByte(cmd);
            HAL_UART_WriteString("' (h=help)");
            HAL_UART_NewLine();
            break;
    }
}

void DIAG_DumpAll(void)
{
    CmdClock();
    CmdGpio();
    CmdAtaRegs();
    CmdPwm();
    CmdBemfPins();
    CmdAdcSnap();
    CmdState();
}
