// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file pwm.c
 *
 * @brief This module configures and enables the PWM Module 
 * 
 * Definitions in this file are for dsPIC33AK512MC510
 *
 * Component: PWM
 *
 */
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Disclaimer ">

/*
© [2025] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/


// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <xc.h>
#include <stdint.h>

#include "pwm.h"

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

/**
* <B> Function: InitPWMGenerators()    </B>
*
* @brief Function initializes the registers common for all PWM modules and 
* configures individual PWM module. The function enables the PWM module 
* and completes the boot strap charging sequence
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitPWMGenerators();     </CODE>
*
*/
void InitPWMGenerators(void)
{

    PCLKCON      = 0x0000;
    /* PWM Clock Divider Selection bits
       0b11 = 1:16 ; 0b10 = 1:8 ;0b01 = 1:4 ; 0b00 = 1:2*/
    PCLKCONbits.DIVSEL = 0;
    /* PWM Master Clock Selection bits
       0b11 = AFPLLO ; 0b10 = FPLLO ; 0b01 = AFVCO/2 ; 0b00 = FOSC */
    PCLKCONbits.MCLKSEL = 1;
    /* Lock bit: 0 = Write-protected registers and bits are unlocked   */
    PCLKCONbits.LOCK = 0;

    /* Initialize Master Phase Register */
    MPHASE       = 0x0000;
    /* Initialize Master Duty Cycle */
    MDC          = 0x0000;
    /* Initialize Master Period Register */
    MPER         = LOOPTIME_TCY;
    
    /* Initialize FREQUENCY SCALE REGISTER*/
    FSCL          = 0x0000;
    /* Initialize FREQUENCY SCALING MINIMUM PERIOD REGISTER */
    FSMINPER     = 0x0000;
    /* Initialize Linear Feedback Shift Register */
    LFSR         = 0x0000;
    /* Initialize Combinational Trigger Register */
    CMBTRIG     = 0x0000;
    /* Initialize LOGIC CONTROL REGISTER 1 */
    LOGCONA     = 0x0000;
    /* Initialize LOGIC CONTROL REGISTER 1 */
    LOGCONB     = 0x0000;
    /* Initialize LOGIC CONTROL REGISTER 2 */
    LOGCONC     = 0x0000;
    /* Initialize LOGIC CONTROL REGISTER 2 */
    LOGCOND     = 0x0000;
    /* Initialize LOGIC CONTROL REGISTER 3 */
    LOGCONE     = 0x0000;
    /* Initialize LOGIC CONTROL REGISTER 3 */
    LOGCONF     = 0x0000;
    /* PWM EVENT OUTPUT CONTROL REGISTER A */
    PWMEVTA     = 0x0000;     
    /* PWM EVENT OUTPUT CONTROL REGISTER B */
    PWMEVTB     = 0x0000;
    /* PWM EVENT OUTPUT CONTROL REGISTER C */
    PWMEVTC     = 0x0000;
    /* PWM EVENT OUTPUT CONTROL REGISTER D */
    PWMEVTD     = 0x0000;
    /* PWM EVENT OUTPUT CONTROL REGISTER E */
    PWMEVTE     = 0x0000;
    /* PWM EVENT OUTPUT CONTROL REGISTER F */
    PWMEVTF     = 0x0000;
    
    /* PWM Event Output Enable bit  
       1 = Event output signal is output on PWMEy pin
       0 = Event output signal is internal only */
    PWMEVTAbits.EVTAOEN  = 0;
    /* EVTyPOL: PWM Event Output Polarity bit
       1 = Event output signal is active-low
       0 = Event output signal is active-high */
    PWMEVTAbits.EVTAPOL  = 0;
    /* EVTySTRD: PWM Event Output Stretch Disable bit
       1 = Event output signal pulse width is not stretched
       0 = Event output signal is stretched to eight PWM clock cycles minimum */
    PWMEVTAbits.EVTASTRD = 1; 
    /* EVTySYNC: PWM Event Output Sync bit
       1 = Event output signal is synchronized to the system clock
       0 = Event output is not synchronized to the system clock
       Event output signal pulse will be two system clocks when this bit is set and EVTySTRD = 1 */
    PWMEVTAbits.EVTASYNC = 1;
    /* EVTySEL[3:0]: PWM Event Selection bits
       1111-1010 = Reserved
       1010 = ADC Trigger 2 signal
       1001 = ADC Trigger 1 signal
       0111 = STEER signal (available in Push-Pull Output modes only)(4)
       0110 = CAHALF signal (available in Center-Aligned modes only)(4)
       0101 = PCI Fault active output signal
       0100 = PCI Current limit active output signal
       0011 = PCI Feed-forward active output signal
       0010 = PCI Sync active output signal
       0001 = PWM Generator output signal(3)
       0000 = Source is selected by the PGTRGSEL[2:0] bits */
    PWMEVTAbits.EVTASEL  = 0b01001;
    /* EVTyPGS[2:0]: PWM Event Source Selection bits
       111-100 = Reserved
       011 = PWM Generator 4
       ...
       000 = PWM Generator 1 */
    PWMEVTAbits.EVTAPGS  = 0;//0b011;
    PWMEVTAbits.EVTAOEN  = 1;
    
    IFS1bits.PEVTAIF = 0;
    IPC4bits.PEVTAIP = 6;
    IEC1bits.PEVTAIE = 0;
    
    /* PWM Event Output Enable bit  
   1 = Event output signal is output on PWMEB pin
   0 = Event output signal is internal only */
    PWMEVTBbits.EVTBOEN  = 0;
    /* EVTyPOL: PWM Event Output Polarity bit
       1 = Event output signal is active-low
       0 = Event output signal is active-high */
    PWMEVTBbits.EVTBPOL  = 0;
    /* EVTySTRD: PWM Event Output Stretch Disable bit
       1 = Event output signal pulse width is not stretched
       0 = Event output signal is stretched to eight PWM clock cycles minimum */
    PWMEVTBbits.EVTBSTRD = 1;
    /* EVTySYNC: PWM Event Output Sync bit
       1 = Event output signal is synchronized to the system clock
       0 = Event output is not synchronized to the system clock */
    PWMEVTBbits.EVTBSYNC = 1;
    /* EVTySEL[3:0]: PWM Event Selection bits */
    PWMEVTBbits.EVTBSEL  = 0b1010;
    /* EVTyPGS[2:0]: PWM Event Source Selection bits */
    PWMEVTBbits.EVTBPGS  = 1;
    PWMEVTBbits.EVTBOEN  = 1;
    /* Interrupt setup */
//    IFS1bits.PEVTBIF = 0;
//    IPC4bits.PEVTBIP = 5;
//    IEC1bits.PEVTBIE = 1;

    /* PWM Event Output Enable bit  
       1 = Event output signal is output on PWMEC pin
       0 = Event output signal is internal only */
    PWMEVTCbits.EVTCOEN  = 0;
    /* EVTyPOL: PWM Event Output Polarity bit */
    PWMEVTCbits.EVTCPOL  = 0;
    /* EVTySTRD: PWM Event Output Stretch Disable bit */
    PWMEVTCbits.EVTCSTRD = 1;
    /* EVTySYNC: PWM Event Output Sync bit */
    PWMEVTCbits.EVTCSYNC = 1;
    /* EVTySEL[3:0]: PWM Event Selection bits */
    PWMEVTCbits.EVTCSEL  = 0b01001;
    /* EVTyPGS[2:0]: PWM Event Source Selection bits */
    PWMEVTCbits.EVTCPGS  = 1;
    PWMEVTCbits.EVTCOEN  = 1;
    /* Interrupt setup */
    IFS1bits.PEVTCIF = 0;
    IPC5bits.PEVTCIP = 7;
    IEC1bits.PEVTCIE = 0;

    /* Function call to Initialize individual PWM modules*/
    InitPWMGenerator1 ();
    InitPWMGenerator2 ();
    InitPWMGenerator3 (); 
    
    InitDutyPWM123Generators();

    /* Clearing and disabling PWM Interrupt */
    _PWM1IF = 0;
    _PWM1IE = 0;
    _PWM1IP = 7;
    
    /* Enable the PWM module after initializing generators*/
	PG2CONbits.ON = 1;      
    PG3CONbits.ON = 1;      
    PG1CONbits.ON = 1;      

}

/**
* <B> Function: InitDutyPWM123Generators()    </B>
*
* @brief Function initialize the duty to zero by overriding and 
* reset the duty registers for Boot strap charging
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitDutyPWM123Generators();     </CODE>
*
*/
void InitDutyPWM123Generators(void)
{
    
    /** Set Override Data on all PWM outputs */
    /* 0b00 = State for PWM3H,L, if Override is Enabled */
    PG3IOCON2bits.OVRDAT = 0;
    /* 0b00 = State for PWM2H,L, if Override is Enabled */
    PG2IOCON2bits.OVRDAT = 0; 
    /* 0b00 = State for PWM1H,L, if Override is Enabled */
    PG1IOCON2bits.OVRDAT = 0; 

    /* 1 = OVRDAT<1> provides data for output on PWM3H */
    PG3IOCON2bits.OVRENH = 1; 
    /* 1 = OVRDAT<0> provides data for output on PWM3L */
    PG3IOCON2bits.OVRENL = 1; 
    
    /* 1 = OVRDAT<1> provides data for output on PWM2H */
    PG2IOCON2bits.OVRENH = 1;
    /* 1 = OVRDAT<0> provides data for output on PWM2L */
    PG2IOCON2bits.OVRENL = 1; 
    
    /* 1 = OVRDAT<1> provides data for output on PWM1H */
    PG1IOCON2bits.OVRENH = 1;  
    /* 1 = OVRDAT<0> provides data for output on PWM1L */
    PG1IOCON2bits.OVRENL = 1;  

    /* Set all PWM Duty to zero */
    PG3DC = 0;
    PG2DC = 0;      
    PG1DC = 0;

}

/**
* <B> Function: ChargeBootstrapCapacitors()    </B>
*
* @brief Function to implement Boot Strap Capacitor charging sequence
*        
* @param none.
* @return none.
* 
* @example
* <CODE> ChargeBootstrapCapacitors();     </CODE>
*
*/
void ChargeBootstrapCapacitors(void)
{
    uint32_t i = BOOTSTRAP_CHARGING_COUNTS;
    uint8_t prevStatusCAHALF = 0,currStatusCAHALF = 0;

    /* Enable PWMs only on PWMxL ,to charge bootstrap capacitors at the beginning
     * Hence PWMxH is over-ridden to "LOW" */
    /* 0b00 = State for PWM3H-L,PWM2H-L and PWM1H-L if Override is Enabled*/
    PG3IOCON2bits.OVRDAT = 0;  
    PG2IOCON2bits.OVRDAT = 0;  
    PG1IOCON2bits.OVRDAT = 0; 
    
    /* 1 = OVRDAT<1> provides data for output on PWM3H,PWM2H and PWM1H*/
    PG3IOCON2bits.OVRENH = 1;  
    PG2IOCON2bits.OVRENH = 1;  
    PG1IOCON2bits.OVRENH = 1;  

    /* 1 = OVRDAT<1> provides data for output on PWM3L,PWM2L and PWM1L*/
    PG3IOCON2bits.OVRENL = 1;  
    PG2IOCON2bits.OVRENL = 1;  
    PG1IOCON2bits.OVRENL = 1;  

    /* PDCx: PWMx GENERATOR DUTY CYCLE REGISTER
     * Initialize the PWM duty cycle for charging */
    PWM_PHASE3 = TICKLE_CHARGE_DUTY;
    PWM_PHASE2 = TICKLE_CHARGE_DUTY;
    PWM_PHASE1 = TICKLE_CHARGE_DUTY;
    PWM_PDC3 = TICKLE_CHARGE_DUTY;
    PWM_PDC2 = TICKLE_CHARGE_DUTY;
    PWM_PDC1 = TICKLE_CHARGE_DUTY;
    
    /* 0 = PWM generator provides data for PWM1L/2L/3L pin */
    PG1IOCON2bits.OVRENL = 0;
    PG2IOCON2bits.OVRENL = 0;  
    PG3IOCON2bits.OVRENL = 0; 

    /*While loop for the Bootstrap charging time in number of PWM Half Cycles*/
    while(i)
    {
        prevStatusCAHALF = currStatusCAHALF;
        currStatusCAHALF = PG1STATbits.CAHALF;
        if (prevStatusCAHALF != currStatusCAHALF)
        {
            /*Monitoring the Bootstrap charging time in number of PWM Half Cycles*/
            if (currStatusCAHALF == 0)
            {
                i--; 
            }
        }
    }
    /* PDCx: PWMx GENERATOR DUTY CYCLE REGISTER
     * Reset the PWM duty cycle to zero after charging */
    PWM_PHASE3 = 0;
    PWM_PHASE2 = 0;
    PWM_PHASE1 = 0;
    PWM_PDC3 = 0;
    PWM_PDC2 = 0;
    PWM_PDC1 = 0;

    /* 0 = PWM generator provides data for output on PWM3H,PWM2H and PWM1H*/
    PG3IOCON2bits.OVRENH = 0;  
    PG2IOCON2bits.OVRENH = 0;  
    PG1IOCON2bits.OVRENH = 0;   
     
}

/**
* <B> Function: InitPWMGenerator1()    </B>
*
* @brief Function to configure PWM Module # 1
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitPWMGenerator1();     </CODE>
*
*/
void InitPWMGenerator1 (void)
{

    /* Initialize PWM GENERATOR 1 CONTROL REGISTER */
    PG1CON      = 0x0000;
    /* PWM Generator 1 Enable bit : 1 = Is enabled, 0 = Is not enabled */
    /* Ensuring PWM Generator is disabled prior to configuring module */
    PG1CONbits.ON = 0;
    /* Clock Selection bits
       0b01 = Macro uses Master clock selected by the PCLKCON.MCLKSEL bits*/
    PG1CONbits.CLKSEL = 1;
    /* PWM Mode Selection bits
       100 = Center-Aligned PWM mode(interrupt/register update once per cycle)
       000 = Independent Edge PWM mode */
    PG1CONbits.MODSEL = 0;
    /* Trigger Count Select bits
       000 = PWM Generator produces 1 PWM cycle after triggered */
    PG1CONbits.TRGCNT = 0;
    
    /* Initialize PWM GENERATOR 1 CONTROL REGISTER */
    /* Master Duty Cycle Register Select bit
       1 = Macro uses the MDC register instead of PG1DC
       0 = Macro uses the PG1DC register*/
    PG1CONbits.MDCSEL = 0;
    /* Master Period Register Select bit
       1 = Macro uses the MPER register instead of PG1PER
       0 = Macro uses the PG1PER register */
    PG1CONbits.MPERSEL = 1;
    /* MPHSEL: Master Phase Register Select bit
       1 = Macro uses the MPHASE register instead of PG1PHASE
       0 = Macro uses the PG1PHASE register */
    PG1CONbits.MPHSEL = 0;
    /* Master Update Enable bit
       1 = PWM Generator broadcasts software set/clear of UPDATE status bit and 
           EOC signal to other PWM Generators
       0 = PWM Generator does not broadcast UPDATE status bit or EOC signal */
    PG1CONbits.MSTEN = 1;
    /* PWM Buffer Update Mode Selection bits 
       Update Data registers at start of next PWM cycle if UPDATE = 1. */
    PG1CONbits.UPDMOD = 0;
    /* PWM Generator Trigger Mode Selection bits
       0b00 = PWM Generator operates in Single Trigger mode */
    PG1CONbits.TRGMOD = 0;
    /* Start of Cycle Selection bits
       0000 = Local EOC*/
    PG1CONbits.SOCS = 0;
    
    /* Clear PWM GENERATOR 1 STATUS REGISTER*/
    PG1STAT      = 0x0000;
    /* Initialize PWM GENERATOR 1 I/O CONTROL REGISTER */
    PG1IOCON2    = 0x0000;

    /* Current Limit Mode Select bit
       0 = If PCI current limit is active, then the CLDAT<1:0> bits define 
       the PWM output levels */
    PG1IOCON2bits.CLMOD = 0;
    /* Swap PWM Signals to PWM1H and PWM1L Device Pins bit 
       0 = PWM1H/L signals are mapped to their respective pins */
    PG1IOCON1bits.SWAP = 0;
    /* User Override Enable for PWM1H Pin bit
       0 = PWM Generator provides data for the PWM1H pin*/
    PG1IOCON2bits.OVRENH = 0;
    /* User Override Enable for PWM1L Pin bit
       0 = PWM Generator provides data for the PWM1L pin*/
    PG1IOCON2bits.OVRENL = 0;
    /* Data for PWM1H/PWM1L Pins if Override is Enabled bits
       If OVERENH = 1, then OVRDAT<1> provides data for PWM1H.
       If OVERENL = 1, then OVRDAT<0> provides data for PWM1L */
    PG1IOCON2bits.OVRDAT = 0;
    /* User Output Override Synchronization Control bits
       00 = User output overrides via the OVRENL/H and OVRDAT<1:0> bits are 
       synchronized to the local PWM time base (next start of cycle)*/
    PG1IOCON2bits.OSYNC = 0;
    /* Data for PWM1H/PWM1L Pins if FLT Event is Active bits
       If Fault is active, then FLTDAT<1> provides data for PWM1H.
       If Fault is active, then FLTDAT<0> provides data for PWM1L.*/
    PG1IOCON2bits.FLT1DAT = 0;
    /* Data for PWM1H/PWM1L Pins if CLMT Event is Active bits
       If current limit is active, then CLDAT<1> provides data for PWM1H.
       If current limit is active, then CLDAT<0> provides data for PWM1L.*/
    PG1IOCON2bits.CLDAT = 0;
    /* Data for PWM1H/PWM1L Pins if Feed-Forward Event is Active bits
       If feed-forward is active, then FFDAT<1> provides data for PWM1H.
       If feed-forward is active, then FFDAT<0> provides data for PWM1L.*/
    PG1IOCON2bits.FFDAT = 0;
    /* Data for PWM1H/PWM1L Pins if Debug Mode is Active and PTFRZ = 1 bits
       If Debug mode is active and PTFRZ=1,then DBDAT<1> provides PWM1H data.
       If Debug mode is active and PTFRZ=1,then DBDAT<0> provides PWM1L data. */
    PG1IOCON2bits.DBDAT = 0;
    
    /* Initialize PWM GENERATOR 1 I/O CONTROL REGISTER */    

    /* Time Base Capture Source Selection bits
       000 = No hardware source selected for time base capture ? software only*/
    PG1IOCON1bits.CAPSRC = 0;
    /* Dead-Time Compensation Select bit 
       0 = Dead-time compensation is controlled by PCI Sync logic */
    PG1IOCON1bits.DTCMPSEL = 0;
    /* PWM Generator Output Mode Selection bits
       00 = PWM Generator outputs operate in Complementary mode
       01 = PWM Generator outputs operate in Independent mode */
    PG1IOCON1bits.PMOD = 1;
    /* PWM1H Output Port Enable bit
       1 = PWM Generator controls the PWM1H output pin
       0 = PWM Generator does not control the PWM1H output pin */
    PG1IOCON1bits.PENH = 1;
    /* PWM1L Output Port Enable bit
       1 = PWM Generator controls the PWM1L output pin
       0 = PWM Generator does not control the PWM1L output pin */
    PG1IOCON1bits.PENL = 1;
    /* PWM1H Output Polarity bit
       1 = Output pin is active-low
       0 = Output pin is active-high*/
    PG1IOCON1bits.POLH = 0;
    /* PWM1L Output Polarity bit
       1 = Output pin is active-low
       0 = Output pin is active-high*/
    PG1IOCON1bits.POLL = 0;
    
    /* Initialize PWM GENERATOR 1 EVENT REGISTER */
    PG1EVT1      = 0x0000;
    /* ADC Trigger 1 Post-scaler Selection bits
       00000 = 1:1 */
    PG1EVT1bits.ADTR1PS = 0;
    /* ADC Trigger 1 Source is PG1TRIGC Compare Event Enable bit
       0 = PG1TRIGC register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    PG1EVT1bits.ADTR1EN3  = 0;
    /* ADC Trigger 1 Source is PG1TRIGB Compare Event Enable bit
       0 = PG1TRIGB register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    PG1EVT1bits.ADTR1EN2 = 0;    
    /* ADC Trigger 1 Source is PG1TRIGA Compare Event Enable bit
       1 = PG1TRIGA register compare event is enabled as trigger source for 
           ADC Trigger 1 */
    PG1EVT1bits.ADTR1EN1 = 1;
    /* Update Trigger Select bits
       01 = A write of the PG1DC register automatically sets the UPDATE bit*/
    PG1EVT1bits.UPDTRG = 1;
    /* PWM Generator Trigger Output Selection bits
       000 = EOC event is the PWM Generator trigger*/
    PG1EVT1bits.PGTRGSEL = 0;
    
    /* Initialize PWM GENERATOR 1 EVENT REGISTER */
    /* FLTIEN: PCI Fault Interrupt Enable bit
       1 = Fault interrupt is enabled */
    PG1EVT1bits.FLT1IEN = 1;
    /* PCI Current Limit Interrupt Enable bit
       0 = Current limit interrupt is disabled */
    PG1EVT1bits.CLIEN = 0;
    /* PCI Feed-Forward Interrupt Enable bit
       0 = Feed-forward interrupt is disabled */
    PG1EVT1bits.FFIEN = 0;
    /* PCI Sync Interrupt Enable bit
       0 = Sync interrupt is disabled */
    PG1EVT1bits.SIEN = 0;
    /* Interrupt Event Selection bits
       00 = Interrupts CPU at EOC
       01 = Interrupts CPU at TRIGA compare event
       10 = Interrupts CPU at ADC Trigger 1 event
       11 = Time base interrupts are disabled */
    PG1EVT1bits.IEVTSEL = 3;
        /* ADC Trigger 2 Source is PG1TRIGC Compare Event Enable bit
       0 = PG1TRIGC register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG1EVT2bits.ADTR2EN3 = 0;
    /* ADC Trigger 2 Source is PG1TRIGB Compare Event Enable bit
       1 = PG1TRIGB register compare event is enabled as trigger source for 
            ADC Trigger 2 */
    PG1EVT2bits.ADTR2EN2 = 1;
    /* ADC Trigger 2 Source is PG1TRIGA Compare Event Enable bit
       0 = PG1TRIGA register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG1EVT2bits.ADTR2EN1 = 0;
    /* ADC Trigger 1 Offset Selection bits
       00000 = No offset */
    PG1EVT1bits.ADTR1OFS = 0;
    
#ifdef ENABLE_PWM_FAULT
    /* PWM GENERATOR 1 Fault PCI REGISTER */
    PG1F1PCI1     = 0x0000;
    /* Termination Synchronization Disable bit
       1 = Termination of latched PCI occurs immediately
       0 = Termination of latched PCI occurs at PWM EOC */
    PG1F1PCI1bits.TSYNCDIS = 0;
    /* Termination Event Selection bits
       001 = Auto-Terminate: Terminate when PCI source transitions from 
             active to inactive */
    PG1F1PCI1bits.TERM = 1;
    /* Acceptance Qualifier Polarity Select bit: 0 = Not inverted 1 = Inverted*/
    PG1F1PCI1bits.AQPS = 0;
    /* Acceptance Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to 0)
       110 = Selects PCI Source #9
       101 = Selects PCI Source #8
       100 = Selects PCI Source #1 (PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base PWM Generator signal)        
       000 = No acceptance qualifier is used (qualifier forced to 1) */
    PG1F1PCI1bits.AQSS = 0;
    /* PCI Synchronization Control bit
       1 = PCI source is synchronized to PWM EOC
       0 = PCI source is not synchronized to PWM EOC*/
    PG1F1PCI1bits.PSYNC = 0;
    /* PCI Polarity Select bit 0 = Not inverted 1 = Inverted */
    PG1F1PCI1bits.PPS = 1;
    /* PCI Source Selection bits
       31 : CLC1 output
       30 : Comparator 3 output(0x40000000)
       ? ?
       8 : PPS PCI8R in RPINR20 register input(0x0100)
       ? ?
       1 : Internally connected to the output of PWMPCI[2:0] bits in PGxEVT1 registers
       0 : Tied to '0' */
    PG1F1PCI2 = 0x0100;
    /* PCI Bypass Enable bit
       0 = PCI function is not bypassed */
    PG1F1PCI1bits.BPEN   = 0;
    /* PCI Bypass Source Selection bits(1)
       000 = PCI control is sourced from PG1 PCI logic when BPEN = 1 */
    PG1F1PCI1bits.BPSEL   = 0;
    /* PCI Termination Polarity Select bits 
       0 = Inverter, 1 = Non Inverted */
    PG1F1PCI1bits.TERMPS = 0;
    /* PCI Acceptance Criteria Selection bits
       101 = Latched any edge(2)
       100 = Latched rising edge
       011 = Latched
       010 = Any edge
       001 = Rising edge
       000 = Level-sensitive*/
    PG1F1PCI1bits.ACP   = 3;
    /* PCI SR Latch Mode bit
       1 = SR latch is Reset-dominant in Latched Acceptance modes
       0 = SR latch is Set-dominant in Latched Acceptance modes*/
    PG1F1PCI1bits.LATMOD  = 0;
    /* Termination Qualifier Polarity Select bit 1 = Inverted 0 = Not inverted*/
    PG1F1PCI1bits.TQPS   = 0;
    /* Termination Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to '0')(3)
       110 = Selects PCI Source #9 (pwm_pci[9] input port)
       101 = Selects PCI Source #8 (pwm_pci[8] input port)
       100 = Selects PCI Source #1 (PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base PWM Generator signal)
       000 = No termination qualifier used (qualifier forced to '1')(3)*/
    PG1F1PCI1bits.TQSS  = 0;
#else
    /* PWM GENERATOR 1 Fault PCI REGISTER */
    PG1F1PCI1     = 0x0000;
#endif
#ifdef CYCLE_BY_CYCLE_CURRENT_LIMIT
/*PCI logic configuration for current limit cycle by cycle mode, comparator 3 
 output as PCI source*/

    /* PWM GENERATOR 1 Current Limit PCI REGISTER */
    PG1CLPCI1     = 0x0000;
    /* Termination Synchronization Disable bit
       1 = Termination of latched PCI occurs immediately
       0 = Termination of latched PCI occurs at PWM EOC */
    PG1CLPCI1bits.TSYNCDIS = 0;
    /* Termination Event Selection bits
       001 = Auto-Terminate: Terminate when PCI source transitions from 
             active to inactive */
    PG1CLPCI1bits.TERM = 0b001;
    /* Acceptance Qualifier Polarity Select bit: 0 = Not inverted 1 = Inverted*/
    PG1CLPCI1bits.AQPS = 0;
    /* Acceptance Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to 0)
       110 = Selects PCI Source #9
       101 = Selects PCI Source #8
       100 = Selects PCI Source #1 (PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base PWM Generator signal)        
       000 = No acceptance qualifier is used (qualifier forced to 1) */
    PG1CLPCI1bits.AQSS = 0;
    /* PCI Synchronization Control bit
       1 = PCI source is synchronized to PWM EOC
       0 = PCI source is not synchronized to PWM EOC*/
    PG1CLPCI1bits.PSYNC = 0;
    /* PCI Polarity Select bit 0 = Not inverted 1 = Inverted */
    PG1CLPCI1bits.PPS = 0;
    /* PCI Source Selection bits
       31 : CLC1 output
       30 : Comparator 3 output(0x40000000)
       ? ?
       8 : PPS PCI8R in RPINR20 register input(0x0100)
       ? ?
       1 : Internally connected to the output of PWMPCI[2:0] bits in PGxEVT1 registers
       0 : Tied to '0' */
    PG1CLPCI2 = 0x40000000;
    /* PCI Bypass Enable bit
       0 = PCI function is not bypassed */
    PG1CLPCI1bits.BPEN   = 0;
    /* PCI Bypass Source Selection bits(1)
       000 = PCI control is sourced from PG1 PCI logic when BPEN = 1 */
    PG1CLPCI1bits.BPSEL   = 0;
    /* PCI Termination Polarity Select bits 
       0 = Inverter, 1 = Non Inverted */
    PG1F1PCI1bits.TERMPS = 0;
    /* PCI Acceptance Criteria Selection bits
       101 = Latched any edge(2)
       100 = Latched rising edge
       011 = Latched
       010 = Any edge
       001 = Rising edge
       000 = Level-sensitive*/
    PG1CLPCI1bits.ACP   = 3;
    /* PCI SR Latch Mode bit
       1 = SR latch is Reset-dominant in Latched Acceptance modes
       0 = SR latch is Set-dominant in Latched Acceptance modes*/
    PG1CLPCI1bits.LATMOD  = 0;
    /* Termination Qualifier Polarity Select bit 1 = Inverted 0 = Not inverted*/
    PG1CLPCI1bits.TQPS   = 0;
    /* Termination Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to ?1?b0?)(3)
       110 = Selects PCI Source #9 (pwm_pci[9] input port)
       101 = Selects PCI Source #8 (pwm_pci[8] input port)
       100 = Selects PCI Source #1 (PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base PWM Generator signal)
       000 = No termination qualifier used (qualifier forced to ?1?b1?)(3)*/
    PG1CLPCI1bits.TQSS  = 0;
#else
    /* PWM GENERATOR 1 Current Limit PCI REGISTER */
    PG1CLPCI1    = 0x0000;
#endif  

    /* PWM GENERATOR 1 Fault 2 PCI REGISTER */
    PG1F2PCI1    = 0x0000;
    /* PWM GENERATOR 1 Feed Forward PCI REGISTER  */
    PG1FFPCI1    = 0x0000;
    /* PWM GENERATOR 1 Sync PCI REGISTER */
    PG1SPCI1    = 0x0000;
    
    /* Initialize PWM GENERATOR 1 LEADING-EDGE BLANKING REGISTER */
    PG1LEB      = 0x0000;
    
    /* Initialize PWM GENERATOR 1 PHASE REGISTER */
    PG1PHASEbits.PHASE     = MIN_DUTY;
    /* Initialize PWM GENERATOR 1 DUTY CYCLE REGISTER */
    PG1DCbits.DC           = MIN_DUTY;
    /* Initialize PWM GENERATOR 1 DUTY CYCLE ADJUSTMENT REGISTER */
    PG1DCA       = 0x0000;
    /* Initialize PWM GENERATOR 1 PERIOD REGISTER */
    PG1PER       = 0x0000;
    /* Initialize PWM GENERATOR 1 DEAD-TIME REGISTER */
    PG1DTbits.DTH = DEADTIME;
    /* Initialize PWM GENERATOR 1 DEAD-TIME REGISTER */
    PG1DTbits.DTL = DEADTIME;

    /* Initialize PWM GENERATOR 1 TRIGGER A REGISTER */
    PG1TRIGAbits.CAHALF = 0;
    PG1TRIGAbits.TRIGA  = 0;
    /* Initialize PWM GENERATOR 1 TRIGGER B REGISTER */
    PG1TRIGBbits.CAHALF = 0;
    PG1TRIGBbits.TRIGB     = ADC_SAMPLING_POINT;
    /* Initialize PWM GENERATOR 1 TRIGGER C REGISTER */
    PG1TRIGCbits.CAHALF = 0;
    PG1TRIGCbits.TRIGC     = (LOOPTIME_TCY/2);
    
} 

/**
* <B> Function: InitPWMGenerator2()    </B>
*
* @brief Function to configure PWM Module # 2
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitPWMGenerator2();     </CODE>
*
*/
void InitPWMGenerator2 (void)
{

    /* Initialize PWM GENERATOR 2 CONTROL REGISTER */
    PG2CON      = 0x0000;
    /* PWM Generator 2 Enable bit : 1 = Is enabled, 0 = Is not enabled */
    /* PWM Generator is disabled prior to configuring module */
    PG2CONbits.ON = 0;
    /* Clock Selection bits
       0b01 = Macro uses Master clock selected by the PCLKCON.MCLKSEL bits*/
    PG2CONbits.CLKSEL = 1;
    /* PWM Mode Selection bits
       100 = Center-Aligned PWM mode(interrupt/register update once per cycle)
       000 = Independent Edge PWM mode */
    PG2CONbits.MODSEL = 0;
    /* Trigger Count Select bits
       000 = PWM Generator produces 1 PWM cycle after triggered */
    PG2CONbits.TRGCNT = 0;
    
    /* Initialize PWM GENERATOR 2 CONTROL REGISTER */
    /* Master Duty Cycle Register Select bit
       1 = Macro uses the MDC register instead of PG2DC
       0 = Macro uses the PG2DC register*/
    PG2CONbits.MDCSEL = 0;
    /* Master Period Register Select bit
       1 = Macro uses the MPER register instead of PG2PER
       0 = Macro uses the PG2PER register */
    PG2CONbits.MPERSEL = 1;
    /* MPHSEL: Master Phase Register Select bit
       1 = Macro uses the MPHASE register instead of PG2PHASE
       0 = Macro uses the PG2PHASE register */
    PG2CONbits.MPHSEL = 0;
    /* Master Update Enable bit
       1 = PWM Generator broadcasts software set/clear of UPDATE status bit and 
           EOC signal to other PWM Generators
       0 = PWM Generator does not broadcast UPDATE status bit or EOC signal */
    PG2CONbits.MSTEN = 0;
     /* PWM Buffer Update Mode Selection bits 
       0b010 = Slaved SOC Update Data registers at start of next cycle if a 
       master update request is received. A master update request will be 
       transmitted if MSTEN = 1 and UPDATE = 1 for the requesting PWM
       Generator.. */
	PG2CONbits.UPDMOD = 0b010;
    /* PWM Generator Trigger Mode Selection bits
       0b00 = PWM Generator operates in Single Trigger mode */
    PG2CONbits.TRGMOD = 0;
    /* Start of Cycle Selection bits
       0001 = PWM1 trigger o/p selected by PG1 PGTRGSEL<2:0> bits(PGxEVT<2:0>)*/
    PG2CONbits.SOCS = 1;
    
    /* Clear PWM GENERATOR 2 STATUS REGISTER*/
    PG2STAT      = 0x0000;
    /* Initialize PWM GENERATOR 2 I/O CONTROL REGISTER */
    PG2IOCON2    = 0x0000;

    /* Current Limit Mode Select bit
       0 = If PCI current limit is active, then the CLDAT<1:0> bits define 
       the PWM output levels */
    PG2IOCON2bits.CLMOD = 0;
    /* Swap PWM Signals to PWM2H and PWM2L Device Pins bit 
       0 = PWM2H/L signals are mapped to their respective pins */
    PG2IOCON1bits.SWAP = 0;
    /* User Override Enable for PWM2H Pin bit
       0 = PWM Generator provides data for the PWM2H pin*/
    PG2IOCON2bits.OVRENH = 0;
    /* User Override Enable for PWM2L Pin bit
       0 = PWM Generator provides data for the PWM2L pin*/
    PG2IOCON2bits.OVRENL = 0;
    /* Data for PWM2H/PWM2L Pins if Override is Enabled bits
       If OVERENH = 1, then OVRDAT<1> provides data for PWM2H.
       If OVERENL = 1, then OVRDAT<0> provides data for PWM2L */
    PG2IOCON2bits.OVRDAT = 0;
    /* User Output Override Synchronization Control bits
       00 = User output overrides via the OVRENL/H and OVRDAT<1:0> bits are 
       synchronized to the local PWM time base (next start of cycle)*/
    PG2IOCON2bits.OSYNC = 0;
    /* Data for PWM2H/PWM2L Pins if FLT Event is Active bits
       If Fault is active, then FLTDAT<1> provides data for PWM2H.
       If Fault is active, then FLTDAT<0> provides data for PWM2L.*/
    PG2IOCON2bits.FLT1DAT = 0;
    /* Data for PWM2H/PWM2L Pins if CLMT Event is Active bits
       If current limit is active, then CLDAT<1> provides data for PWM2H.
       If current limit is active, then CLDAT<0> provides data for PWM2L.*/
    PG2IOCON2bits.CLDAT = 0;
    /* Data for PWM2H/PWM2L Pins if Feed-Forward Event is Active bits
       If feed-forward is active, then FFDAT<1> provides data for PWM2H.
       If feed-forward is active, then FFDAT<0> provides data for PWM2L.*/
    PG2IOCON2bits.FFDAT = 0;
    /* Data for PWM2H/PWM2L Pins if Debug Mode is Active and PTFRZ = 1 bits
       If Debug mode is active and PTFRZ=1,then DBDAT<1> provides PWM2H data.
       If Debug mode is active and PTFRZ=1,then DBDAT<0> provides PWM2L data. */
    PG2IOCON2bits.DBDAT = 0;
    
    /* Initialize PWM GENERATOR 2 I/O CONTROL REGISTER */    
    /* Time Base Capture Source Selection bits
       000 = No hardware source selected for time base capture ? software only*/
    PG2IOCON1bits.CAPSRC = 0; 
    /* Dead-Time Compensation Select bit 
       0 = Dead-time compensation is controlled by PCI Sync logic */
    PG2IOCON1bits.DTCMPSEL = 0;
    /* PWM Generator Output Mode Selection bits
       00 = PWM Generator outputs operate in Complementary mode
       01 = PWM Generator outputs operate in Independent mode */
    PG2IOCON1bits.PMOD = 1;
    /* PWM2H Output Port Enable bit
       1 = PWM Generator controls the PWM2H output pin
       0 = PWM Generator does not control the PWM2H output pin */
    PG2IOCON1bits.PENH = 1;
    /* PWM2L Output Port Enable bit
       1 = PWM Generator controls the PWM2L output pin
       0 = PWM Generator does not control the PWM2L output pin */
    PG2IOCON1bits.PENL = 1;
    /* PWM2H Output Polarity bit
       1 = Output pin is active-low
       0 = Output pin is active-high*/
    PG2IOCON1bits.POLH = 0;
    /* PWM2L Output Polarity bit
       1 = Output pin is active-low
       0 = Output pin is active-high*/
    PG2IOCON1bits.POLL = 0;
    
    /* Initialize PWM GENERATOR 2 EVENT REGISTER */
    PG2EVT1      = 0x0000;
    /* ADC Trigger 1 Post-scaler Selection bits
       00000 = 1:1 */
    PG2EVT1bits.ADTR1PS = 0;
    /* ADC Trigger 1 Source is PG2TRIGC Compare Event Enable bit
       0 = PG2TRIGC register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    PG2EVT1bits.ADTR1EN3  = 0;
    /* ADC Trigger 1 Source is PG2TRIGB Compare Event Enable bit
       0 = PG2TRIGB register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    PG2EVT1bits.ADTR1EN2 = 0;
    /* ADC Trigger 1 Source is PG2TRIGA Compare Event Enable bit
       0 = PG2TRIGA register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    PG2EVT1bits.ADTR1EN1 = 1;
    /* Update Trigger Select bits
       01 = A write of the PG2DC register automatically sets the UPDATE bit
       00 = User must set the UPDATE bit manually*/
    PG2EVT1bits.UPDTRG = 0;
    /* PWM Generator Trigger Output Selection bits
       000 = EOC event is the PWM Generator trigger*/
    PG2EVT1bits.PGTRGSEL = 0;
    
    /* Initialize PWM GENERATOR 2 EVENT REGISTER */
    /* FLTIEN: PCI Fault Interrupt Enable bit
       0 = Fault interrupt is disabled */
    PG2EVT1bits.FLT1IEN = 0;
    /* PCI Current Limit Interrupt Enable bit
       0 = Current limit interrupt is disabled */
    PG2EVT1bits.CLIEN = 0;
    /* PCI Feed-Forward Interrupt Enable bit
       0 = Feed-forward interrupt is disabled */
    PG2EVT1bits.FFIEN = 0;
    /* PCI Sync Interrupt Enable bit
       0 = Sync interrupt is disabled */
    PG2EVT1bits.SIEN = 0;
    /* Interrupt Event Selection bits
       00 = Interrupts CPU at EOC
       01 = Interrupts CPU at TRIGA compare event
       10 = Interrupts CPU at ADC Trigger 1 event
       11 = Time base interrupts are disabled */
    PG2EVT1bits.IEVTSEL = 3;
    /* ADC Trigger 2 Source is PG2TRIGC Compare Event Enable bit
       0 = PG2TRIGC register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG2EVT2bits.ADTR2EN3 = 0;
    /* ADC Trigger 2 Source is PG2TRIGB Compare Event Enable bit
       0 = PG2TRIGB register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG2EVT2bits.ADTR2EN2 = 1;
    /* ADC Trigger 2 Source is PG2TRIGA Compare Event Enable bit
       0 = PG2TRIGA register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG2EVT2bits.ADTR2EN1 = 0;
    /* ADC Trigger 1 Offset Selection bits
       00000 = No offset */
    PG2EVT1bits.ADTR1OFS = 0;
    
#ifdef ENABLE_PWM_FAULT
    /* PWM GENERATOR 2 Fault PCI REGISTER */
    PG2F1PCI1     = 0x0000;
    /* Termination Synchronization Disable bit
       1 = Termination of latched PCI occurs immediately
       0 = Termination of latched PCI occurs at PWM EOC */
    PG2F1PCI1bits.TSYNCDIS = 0;
    /* Termination Event Selection bits
       001 = Auto-Terminate: Terminate when PCI source transitions from 
             active to inactive */
    PG2F1PCI1bits.TERM = 1;
    /* Acceptance Qualifier Polarity Select bit: 0 = Not inverted 1 = Inverted*/
    PG2F1PCI1bits.AQPS = 0;
    /* Acceptance Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to 0)
       110 = Selects PCI Source #9
       101 = Selects PCI Source #8
       100 = Selects PCI Source #1 (PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base PWM Generator signal)        
       000 = No acceptance qualifier is used (qualifier forced to 1) */
    PG2F1PCI1bits.AQSS = 0;
    /* PCI Synchronization Control bit
       1 = PCI source is synchronized to PWM EOC
       0 = PCI source is not synchronized to PWM EOC*/
    PG2F1PCI1bits.PSYNC = 0;
    /* PCI Polarity Select bit 0 = Not inverted 1 = Inverted */
    PG2F1PCI1bits.PPS = 1;
    /* PCI Source Selection bits
       31 : CLC1 output
       30 : Comparator 3 output(0x40000000)
       ? ?
       8 : PPS PCI8R in RPINR20 register input(0x0100)
       ? ?
       1 : Internally connected to the output of PWMPCI[2:0] bits in PGxEVT1 registers
       0 : Tied to '0' */
    PG2F1PCI2 = 0x0100;
    /* PCI Bypass Enable bit
       0 = PCI function is not bypassed */
    PG2F1PCI1bits.BPEN   = 0;
    /* PCI Bypass Source Selection bits(1)
       000 = PCI control is sourced from PG1 PCI logic when BPEN = 1 */
    PG2F1PCI1bits.BPSEL   = 0;
    /* PCI Termination Polarity Select bits 
       0 = Inverter, 1 = Non Inverted */
    PG2F1PCI1bits.TERMPS = 0;
    /* PCI Acceptance Criteria Selection bits
       101 = Latched any edge(2)
       100 = Latched rising edge
       011 = Latched
       010 = Any edge
       001 = Rising edge
       000 = Level-sensitive*/
    PG2F1PCI1bits.ACP   = 3;
    /* PCI SR Latch Mode bit
       1 = SR latch is Reset-dominant in Latched Acceptance modes
       0 = SR latch is Set-dominant in Latched Acceptance modes*/
    PG2F1PCI1bits.LATMOD  = 0;
    /* Termination Qualifier Polarity Select bit 1 = Inverted 0 = Not inverted*/
    PG2F1PCI1bits.TQPS   = 0;
    /* Termination Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to '0')(3)
       110 = Selects PCI Source #9 (pwm_pci[9] input port)
       101 = Selects PCI Source #8 (pwm_pci[8] input port)
       100 = Selects PCI Source #1 (PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base PWM Generator signal)
       000 = No termination qualifier used (qualifier forced to '1')(3)*/
    PG2F1PCI1bits.TQSS  = 0;
#else
    /* PWM GENERATOR 2 Fault PCI REGISTER */
    PG2F1PCI1     = 0x0000;
#endif
#ifdef CYCLE_BY_CYCLE_CURRENT_LIMIT
/*PCI logic configuration for current limit cycle by cycle mode, comparator 3 
 output as PCI source*/

    /* PWM GENERATOR 2 Current Limit PCI REGISTER */
    PG2CLPCI1     = 0x0000;
    /* Termination Synchronization Disable bit
       1 = Termination of latched PCI occurs immediately
       0 = Termination of latched PCI occurs at PWM EOC */
    PG2CLPCI1bits.TSYNCDIS = 0;
    /* Termination Event Selection bits
       001 = Auto-Terminate: Terminate when PCI source transitions from 
             active to inactive */
    PG2CLPCI1bits.TERM = 0b001;
    /* Acceptance Qualifier Polarity Select bit: 0 = Not inverted 1 = Inverted*/
    PG2CLPCI1bits.AQPS = 0;
    /* Acceptance Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to 0)
       110 = Selects PCI Source #9
       101 = Selects PCI Source #8
       100 = Selects PCI Source #1 (PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base PWM Generator signal)        
       000 = No acceptance qualifier is used (qualifier forced to 1) */
    PG2CLPCI1bits.AQSS = 0;
    /* PCI Synchronization Control bit
       1 = PCI source is synchronized to PWM EOC
       0 = PCI source is not synchronized to PWM EOC*/
    PG2CLPCI1bits.PSYNC = 0;
    /* PCI Polarity Select bit 0 = Not inverted 1 = Inverted */
    PG2CLPCI1bits.PPS = 0;
    /* PCI Source Selection bits
       31 : CLC1 output
       30 : Comparator 3 output(0x40000000)
       ? ?
       8 : PPS PCI8R in RPINR20 register input(0x0100)
       ? ?
       1 : Internally connected to the output of PWMPCI[2:0] bits in PGxEVT1 registers
       0 : Tied to '0' */
    PG2CLPCI2 = 0x40000000;
    /* PCI Bypass Enable bit
       0 = PCI function is not bypassed */
    PG2CLPCI1bits.BPEN   = 0;
    /* PCI Bypass Source Selection bits(1)
       000 = PCI control is sourced from PG1 PCI logic when BPEN = 1 */
    PG2CLPCI1bits.BPSEL   = 0;
    /* PCI Termination Polarity Select bits 
       0 = Inverter, 1 = Non Inverted */
    PG2CLPCI1bits.TERMPS = 0;
    /* PCI Acceptance Criteria Selection bits
       101 = Latched any edge(2)
       100 = Latched rising edge
       011 = Latched
       010 = Any edge
       001 = Rising edge
       000 = Level-sensitive*/
    PG2CLPCI1bits.ACP   = 3;
    /* PCI SR Latch Mode bit
       1 = SR latch is Reset-dominant in Latched Acceptance modes
       0 = SR latch is Set-dominant in Latched Acceptance modes*/
    PG2CLPCI1bits.LATMOD  = 0;
    /* Termination Qualifier Polarity Select bit 1 = Inverted 0 = Not inverted*/
    PG2CLPCI1bits.TQPS   = 0;
    /* Termination Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to ?1?b0?)(3)
       110 = Selects PCI Source #9 (pwm_pci[9] input port)
       101 = Selects PCI Source #8 (pwm_pci[8] input port)
       100 = Selects PCI Source #1 (PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base PWM Generator signal)
       000 = No termination qualifier used (qualifier forced to ?1?b1?)(3)*/
    PG2CLPCI1bits.TQSS  = 0;
#else
    /* PWM GENERATOR 2 Current Limit PCI REGISTER */
    PG2CLPCI1    = 0x0000;
#endif  
    
    /* PWM GENERATOR 2 Fault 2 PCI REGISTER */
    PG2F2PCI1    = 0x0000;
    /* PWM GENERATOR 2 Feed Forward PCI REGISTER */
    PG2FFPCI1    = 0x0000;
    /* PWM GENERATOR 2 Sync PCI REGISTER */
    PG2SPCI1     = 0x0000;
    
    /* Initialize PWM GENERATOR 2 LEADING-EDGE BLANKING REGISTER */
    PG2LEB      = 0x0000;
    
    /* Initialize PWM GENERATOR 2 PHASE REGISTER */
    PG2PHASEbits.PHASE     = MIN_DUTY;
    /* Initialize PWM GENERATOR 2 DUTY CYCLE REGISTER */
    PG2DCbits.DC           = MIN_DUTY;
    /* Initialize PWM GENERATOR 2 DUTY CYCLE ADJUSTMENT REGISTER */
    PG2DCA       = 0x0000;
    /* Initialize PWM GENERATOR 2 PERIOD REGISTER */
    PG2PER       = 0x0000;
    /* Initialize PWM GENERATOR 2 DEAD-TIME REGISTER */
    PG2DTbits.DTH = DEADTIME;
    /* Initialize PWM GENERATOR 2 DEAD-TIME REGISTER */
    PG2DTbits.DTL = DEADTIME;

    /* Initialize PWM GENERATOR 2 TRIGGER A REGISTER */
    PG2TRIGA     = 0x0000;
    /* Initialize PWM GENERATOR 2 TRIGGER B REGISTER */
    PG2TRIGB     = 0x0000;
    /* Initialize PWM GENERATOR 2 TRIGGER C REGISTER */
    PG2TRIGC     = 0x0000;
    
}

/**
* <B> Function: InitPWMGenerator3()    </B>
*
* @brief Function to configure PWM Module # 3
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitPWMGenerator3();     </CODE>
*
*/
void InitPWMGenerator3 (void)
{

    /* Initialize PWM GENERATOR 3 CONTROL REGISTER */
    PG3CON      = 0x0000;
    /* PWM Generator 3 Enable bit : 1 = Is enabled, 0 = Is not enabled */
    /* PWM Generator is disabled prior to configuring module */
    PG3CONbits.ON = 0;
    /* Clock Selection bits
       0b01 = Macro uses Master clock selected by the PCLKCON.MCLKSEL bits*/
    PG3CONbits.CLKSEL = 1;
    /* PWM Mode Selection bits
       100 = Center-Aligned PWM mode(interrupt/register update once per cycle)
       000 = Independent Edge PWM mode*/
    PG3CONbits.MODSEL = 0;  
    /* Trigger Count Select bits
       000 = PWM Generator produces 1 PWM cycle after triggered */
    PG3CONbits.TRGCNT = 0;
    
    /* Initialize PWM GENERATOR 3 CONTROL REGISTER */
    /* Master Duty Cycle Register Select bit
       1 = Macro uses the MDC register instead of PG3DC
       0 = Macro uses the PG3DC register*/
    PG3CONbits.MDCSEL = 0;
    /* Master Period Register Select bit
       1 = Macro uses the MPER register instead of PG3PER
       0 = Macro uses the PG3PER register */
    PG3CONbits.MPERSEL = 1;
    /* MPHSEL: Master Phase Register Select bit
       1 = Macro uses the MPHASE register instead of PG3PHASE
       0 = Macro uses the PG3PHASE register */
    PG3CONbits.MPHSEL = 0;
    /* Master Update Enable bit
       1 = PWM Generator broadcasts software set/clear of UPDATE status bit and 
           EOC signal to other PWM Generators
       0 = PWM Generator does not broadcast UPDATE status bit or EOC signal */
    PG3CONbits.MSTEN = 0;
    /* PWM Buffer Update Mode Selection bits 
       0b010 = Slaved SOC Update Data registers at start of next cycle if a 
       master update request is received. A master update request will be 
       transmitted if MSTEN = 1 and UPDATE = 1 for the requesting PWM
       Generator.. */
	PG3CONbits.UPDMOD = 0b010;
    /* PWM Generator Trigger Mode Selection bits
       0b00 = PWM Generator operates in Single Trigger mode */
    PG3CONbits.TRGMOD = 0;
    /* Start of Cycle Selection bits
       0001 = PWM1 trigger o/p selected by PG1 PGTRGSEL<2:0> bits(PGxEVT<2:0>)*/
    PG3CONbits.SOCS = 1;
    
    /* Clear PWM GENERATOR 3 STATUS REGISTER*/
    PG3STAT      = 0x0000;
    /* Initialize PWM GENERATOR 3 I/O CONTROL REGISTER */
    PG3IOCON2    = 0x0000;

    /* Current Limit Mode Select bit
       0 = If PCI current limit is active, then the CLDAT<1:0> bits define 
       the PWM output levels */
    PG3IOCON2bits.CLMOD = 0;
    /* Swap PWM Signals to PWM3H and PWM3L Device Pins bit 
       0 = PWM3H/L signals are mapped to their respective pins */
    PG3IOCON1bits.SWAP = 0;
    /* User Override Enable for PWM3H Pin bit
       0 = PWM Generator provides data for the PWM3H pin*/
    PG3IOCON2bits.OVRENH = 0;
    /* User Override Enable for PWM3L Pin bit
       0 = PWM Generator provides data for the PWM3L pin*/
    PG3IOCON2bits.OVRENL = 0;
    /* Data for PWM3H/PWM3L Pins if Override is Enabled bits
       If OVERENH = 1, then OVRDAT<1> provides data for PWM3H.
       If OVERENL = 1, then OVRDAT<0> provides data for PWM3L */
    PG3IOCON2bits.OVRDAT = 0;
    /* User Output Override Synchronization Control bits
       00 = User output overrides via the OVRENL/H and OVRDAT<1:0> bits are 
       synchronized to the local PWM time base (next start of cycle)*/
    PG3IOCON2bits.OSYNC = 0;
    /* Data for PWM3H/PWM3L Pins if FLT Event is Active bits
       If Fault is active, then FLTDAT<1> provides data for PWM3H.
       If Fault is active, then FLTDAT<0> provides data for PWM3L.*/
    PG3IOCON2bits.FLT1DAT = 0;
    /* Data for PWM3H/PWM3L Pins if CLMT Event is Active bits
       If current limit is active, then CLDAT<1> provides data for PWM3H.
       If current limit is active, then CLDAT<0> provides data for PWM3L.*/
    PG3IOCON2bits.CLDAT = 0;
    /* Data for PWM3H/PWM3L Pins if Feed-Forward Event is Active bits
       If feed-forward is active, then FFDAT<1> provides data for PWM3H.
       If feed-forward is active, then FFDAT<0> provides data for PWM3L.*/
    PG3IOCON2bits.FFDAT = 0;
    /* Data for PWM3H/PWM3L Pins if Debug Mode is Active and PTFRZ = 1 bits
       If Debug mode is active and PTFRZ=1,then DBDAT<1> provides PWM3H data.
       If Debug mode is active and PTFRZ=1,then DBDAT<0> provides PWM3L data. */
    PG3IOCON2bits.DBDAT = 0;
    
    /* Initialize PWM GENERATOR 3 I/O CONTROL REGISTER */    
    /* Time Base Capture Source Selection bits
       000 = No hardware source selected for time base capture ? software only*/
    PG3IOCON1bits.CAPSRC = 0;
    /* Dead-Time Compensation Select bit 
       0 = Dead-time compensation is controlled by PCI Sync logic */
    PG3IOCON1bits.DTCMPSEL = 0;
    /* PWM Generator Output Mode Selection bits
       00 = PWM Generator outputs operate in Complementary mode
       01 = PWM Generator outputs operate in Independent mode */
    PG3IOCON1bits.PMOD = 1;
    /* PWM3H Output Port Enable bit
       1 = PWM Generator controls the PWM3H output pin
       0 = PWM Generator does not control the PWM3H output pin */
    PG3IOCON1bits.PENH = 1;
    /* PWM3L Output Port Enable bit
       1 = PWM Generator controls the PWM3L output pin
       0 = PWM Generator does not control the PWM3L output pin */
    PG3IOCON1bits.PENL = 1;
    /* PWM3H Output Polarity bit
       1 = Output pin is active-low
       0 = Output pin is active-high*/
    PG3IOCON1bits.POLH = 0;
    /* PWM3L Output Polarity bit
       1 = Output pin is active-low
       0 = Output pin is active-high*/
    PG3IOCON1bits.POLL = 0;
    
    /* Initialize PWM GENERATOR 3 EVENT REGISTER */
    PG3EVT1      = 0x0000;
    /* ADC Trigger 1 Post-scaler Selection bits
       00000 = 1:1 */
    PG3EVT1bits.ADTR1PS = 0;
    /* ADC Trigger 1 Source is PG3TRIGC Compare Event Enable bit
       0 = PG3TRIGC register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    PG3EVT1bits.ADTR1EN3  = 0;
    /* ADC Trigger 1 Source is PG3TRIGB Compare Event Enable bit
       0 = PG3TRIGB register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    PG3EVT1bits.ADTR1EN2 = 0;
    /* ADC Trigger 1 Source is PG3TRIGA Compare Event Enable bit
       0 = PG3TRIGA register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    PG3EVT1bits.ADTR1EN1 = 0;
    /* Update Trigger Select bits
       01 = A write of the PG3DC register automatically sets the UPDATE bit
       00 = User must set the UPDATE bit manually*/
    PG3EVT1bits.UPDTRG = 0;
    /* PWM Generator Trigger Output Selection bits
       000 = EOC event is the PWM Generator trigger*/
    PG3EVT1bits.PGTRGSEL = 0;
    
    /* Initialize PWM GENERATOR 3 EVENT REGISTER */
    /* FLTIEN: PCI Fault Interrupt Enable bit
       0 = Fault interrupt is disabled */
    PG3EVT1bits.FLT1IEN = 0;
    /* PCI Current Limit Interrupt Enable bit
       0 = Current limit interrupt is disabled */
    PG3EVT1bits.CLIEN = 0;
    /* PCI Feed-Forward Interrupt Enable bit
       0 = Feed-forward interrupt is disabled */
    PG3EVT1bits.FFIEN = 0;
    /* PCI Sync Interrupt Enable bit
       0 = Sync interrupt is disabled */
    PG3EVT1bits.SIEN = 0;
    /* Interrupt Event Selection bits
       00 = Interrupts CPU at EOC
       01 = Interrupts CPU at TRIGA compare event
       10 = Interrupts CPU at ADC Trigger 1 event
       11 = Time base interrupts are disabled */
    PG3EVT1bits.IEVTSEL = 3;
    /* ADC Trigger 3 Source is PG3TRIGC Compare Event Enable bit
       0 = PG3TRIGC register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG3EVT2bits.ADTR2EN3 = 0;
    /* ADC Trigger 2 Source is PG3TRIGB Compare Event Enable bit
       0 = PG3TRIGB register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG3EVT2bits.ADTR2EN2 = 0;
    /* ADC Trigger 2 Source is PG3TRIGA Compare Event Enable bit
       0 = PG3TRIGA register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG3EVT2bits.ADTR2EN1 = 0;
    /* ADC Trigger 1 Offset Selection bits
       00000 = No offset */
    PG3EVT1bits.ADTR1OFS = 0;
    
#ifdef ENABLE_PWM_FAULT
    /* PWM GENERATOR 3 Fault PCI REGISTER */
    PG3F1PCI1     = 0x0000;
    /* Termination Synchronization Disable bit
       1 = Termination of latched PCI occurs immediately
       0 = Termination of latched PCI occurs at PWM EOC */
    PG3F1PCI1bits.TSYNCDIS = 0;
    /* Termination Event Selection bits
       001 = Auto-Terminate: Terminate when PCI source transitions from 
             active to inactive */
    PG3F1PCI1bits.TERM = 1;
    /* Acceptance Qualifier Polarity Select bit: 0 = Not inverted 1 = Inverted*/
    PG3F1PCI1bits.AQPS = 0;
    /* Acceptance Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to 0)
       110 = Selects PCI Source #9
       101 = Selects PCI Source #8
       100 = Selects PCI Source #1 (PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base PWM Generator signal)        
       000 = No acceptance qualifier is used (qualifier forced to 1) */
    PG3F1PCI1bits.AQSS = 0;
    /* PCI Synchronization Control bit
       1 = PCI source is synchronized to PWM EOC
       0 = PCI source is not synchronized to PWM EOC*/
    PG3F1PCI1bits.PSYNC = 0;
    /* PCI Polarity Select bit 0 = Not inverted 1 = Inverted */
    PG3F1PCI1bits.PPS = 1;
    /* PCI Source Selection bits
       31 : CLC1 output
       30 : Comparator 3 output(0x40000000)
       ? ?
       8 : PPS PCI8R in RPINR20 register input(0x0100)
       ? ?
       1 : Internally connected to the output of PWMPCI[2:0] bits in PGxEVT1 registers
       0 : Tied to '0' */
    PG3F1PCI2 = 0x0100;
    /* PCI Bypass Enable bit
       0 = PCI function is not bypassed */
    PG3F1PCI1bits.BPEN   = 0;
    /* PCI Bypass Source Selection bits(1)
       000 = PCI control is sourced from PG1 PCI logic when BPEN = 1 */
    PG3F1PCI1bits.BPSEL   = 0;
    /* PCI Termination Polarity Select bits 
       0 = Inverter, 1 = Non Inverted */
    PG3F1PCI1bits.TERMPS = 0;
    /* PCI Acceptance Criteria Selection bits
       101 = Latched any edge(2)
       100 = Latched rising edge
       011 = Latched
       010 = Any edge
       001 = Rising edge
       000 = Level-sensitive*/
    PG3F1PCI1bits.ACP   = 3;
    /* PCI SR Latch Mode bit
       1 = SR latch is Reset-dominant in Latched Acceptance modes
       0 = SR latch is Set-dominant in Latched Acceptance modes*/
    PG3F1PCI1bits.LATMOD  = 0;
    /* Termination Qualifier Polarity Select bit 1 = Inverted 0 = Not inverted*/
    PG3F1PCI1bits.TQPS   = 0;
    /* Termination Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to '0')(3)
       110 = Selects PCI Source #9 (pwm_pci[9] input port)
       101 = Selects PCI Source #8 (pwm_pci[8] input port)
       100 = Selects PCI Source #1 (PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base PWM Generator signal)
       000 = No termination qualifier used (qualifier forced to '1')(3)*/
    PG3F1PCI1bits.TQSS  = 0;
#else
    /* PWM GENERATOR 3 Fault PCI REGISTER */
    PG3F1PCI1     = 0x0000;
#endif
#ifdef CYCLE_BY_CYCLE_CURRENT_LIMIT
/*PCI logic configuration for current limit cycle by cycle mode, comparator 3 
 output as PCI source*/

    /* PWM GENERATOR 3 Current Limit PCI REGISTER */
    PG3CLPCI1     = 0x0000;
    /* Termination Synchronization Disable bit
       1 = Termination of latched PCI occurs immediately
       0 = Termination of latched PCI occurs at PWM EOC */
    PG3CLPCI1bits.TSYNCDIS = 0;
    /* Termination Event Selection bits
       001 = Auto-Terminate: Terminate when PCI source transitions from 
             active to inactive */
    PG3CLPCI1bits.TERM = 0b001;
    /* Acceptance Qualifier Polarity Select bit: 0 = Not inverted 1 = Inverted*/
    PG3CLPCI1bits.AQPS = 0;
    /* Acceptance Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to 0)
       110 = Selects PCI Source #9
       101 = Selects PCI Source #8
       100 = Selects PCI Source #1 (PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base PWM Generator signal)        
       000 = No acceptance qualifier is used (qualifier forced to 1) */
    PG3CLPCI1bits.AQSS = 0;
    /* PCI Synchronization Control bit
       1 = PCI source is synchronized to PWM EOC
       0 = PCI source is not synchronized to PWM EOC*/
    PG3CLPCI1bits.PSYNC = 0;
    /* PCI Polarity Select bit 0 = Not inverted 1 = Inverted */
    PG3CLPCI1bits.PPS = 0;
    /* PCI Source Selection bits
       31 : CLC1 output
       30 : Comparator 3 output(0x40000000)
       ? ?
       8 : PPS PCI8R in RPINR20 register input(0x0100)
       ? ?
       1 : Internally connected to the output of PWMPCI[2:0] bits in PGxEVT1 registers
       0 : Tied to '0' */
    PG3CLPCI2 = 0x40000000;
    /* PCI Bypass Enable bit
       0 = PCI function is not bypassed */
    PG3CLPCI1bits.BPEN   = 0;
    /* PCI Bypass Source Selection bits(1)
       000 = PCI control is sourced from PG1 PCI logic when BPEN = 1 */
    PG3CLPCI1bits.BPSEL   = 0;
    /* PCI Termination Polarity Select bits 
       0 = Inverter, 1 = Non Inverted */
    PG3CLPCI1bits.TERMPS = 0;
    /* PCI Acceptance Criteria Selection bits
       101 = Latched any edge(2)
       100 = Latched rising edge
       011 = Latched
       010 = Any edge
       001 = Rising edge
       000 = Level-sensitive*/
    PG3CLPCI1bits.ACP   = 3;
    /* PCI SR Latch Mode bit
       1 = SR latch is Reset-dominant in Latched Acceptance modes
       0 = SR latch is Set-dominant in Latched Acceptance modes*/
    PG3CLPCI1bits.LATMOD  = 0;
    /* Termination Qualifier Polarity Select bit 1 = Inverted 0 = Not inverted*/
    PG3CLPCI1bits.TQPS   = 0;
    /* Termination Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to ?1?b0?)(3)
       110 = Selects PCI Source #9 (pwm_pci[9] input port)
       101 = Selects PCI Source #8 (pwm_pci[8] input port)
       100 = Selects PCI Source #1 (PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base PWM Generator signal)
       000 = No termination qualifier used (qualifier forced to ?1?b1?)(3)*/
    PG3CLPCI1bits.TQSS  = 0;
#else
    /* PWM GENERATOR 3 Current Limit PCI REGISTER */
    PG3CLPCI1    = 0x0000;
#endif   
    
    /* PWM GENERATOR 3 Fault 2 PCI REGISTER */
    PG3F2PCI1    = 0x0000;
    /* PWM GENERATOR 3 Feed Forward PCI REGISTER */
    PG3FFPCI1    = 0x0000;
    /* PWM GENERATOR 3 Sync PCI REGISTER */
    PG3SPCI1     = 0x0000;
    
    /* Initialize PWM GENERATOR 3 LEADING-EDGE BLANKING REGISTER */
    PG3LEB      = 0x0000;
    
    /* Initialize PWM GENERATOR 3 PHASE REGISTER */
    PG3PHASEbits.PHASE     = MIN_DUTY;
    /* Initialize PWM GENERATOR 3 DUTY CYCLE REGISTER */
    PG3DCbits.DC           = MIN_DUTY;
    /* Initialize PWM GENERATOR 3 DUTY CYCLE ADJUSTMENT REGISTER */
    PG3DCA       = 0x0000;
    /* Initialize PWM GENERATOR 3 PERIOD REGISTER */
    PG3PER       = 0x0000;
    /* Initialize PWM GENERATOR 3 DEAD-TIME REGISTER */
    PG3DTbits.DTH = DEADTIME;
    /* Initialize PWM GENERATOR 3 DEAD-TIME REGISTER */
    PG3DTbits.DTL = DEADTIME;

    /* Initialize PWM GENERATOR 3 TRIGGER A REGISTER */
    PG3TRIGA     = 0x0000;
    /* Initialize PWM GENERATOR 3 TRIGGER B REGISTER */
    PG3TRIGB     = 0x0000;
    /* Initialize PWM GENERATOR 3 TRIGGER C REGISTER */
    PG3TRIGC     = 0x0000;
    
}

// </editor-fold>