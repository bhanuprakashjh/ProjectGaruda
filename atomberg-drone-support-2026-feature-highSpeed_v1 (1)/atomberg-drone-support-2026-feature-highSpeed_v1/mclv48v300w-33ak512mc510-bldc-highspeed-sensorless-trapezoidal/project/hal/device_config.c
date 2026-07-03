// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file device_config.c
 *
 * @brief This module implements the device configuration
 * 
 * Definitions in this file are for dsPIC33AK512MC510
 *
 * Component: DEVICE CONFIGURATION
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

// <editor-fold defaultstate="expanded" desc="CONFIGURATION BITS ">

// DSPIC33AK512MC510 Configuration Bit Settings

// 'C' source line config statements

// FCP
#pragma config FCP_CP = OFF             // Memory protection enable bit. (Memory protection is disabled.)
#pragma config FCP_CRC = OFF            // CRC enable bit. (CRC is disabled.)
#pragma config FCP_WPUCA = OFF          // User configuration areas write protection enable bits. (Protection is disabled.)

// FICD
#pragma config FICD_JTAGEN = OFF        // JTAG enable bit. (JTAG is disabled.)
#pragma config FICD_NOBTSWP = OFF       // BOOTSWP instruction disable bit (BOOTSWP instruction is disabled)

// FDEVOPT
#pragma config FDEVOPT_ALTI2C1 = OFF    // Alternate I2C1 pins selection bit. (Primary I2C1 pins are used.)
#pragma config FDEVOPT_ALTI2C2 = OFF    // Alternate I2C2 pins selection bit. (Primary I2C2 pins are used.)
#pragma config FDEVOPT_ALTI2C3 = OFF    // Alternate I2C3 pins selection bit. (Primary I2C3 pins are used.)
#pragma config FDEVOPT_BISTDIS = OFF    // Hardware RAM test disable bit. (RAM test is disabled)
#pragma config FDEVOPT_SPI2PIN = OFF    // SPI2 peripheral pin selection disable bit. (SPI2 pins are selected by peripheral pin selection feature.)

// FWDT
#pragma config FWDT_WINDIS = OFF        // Watchdog Timer Window Disable bit (Watchdog Timer operates in Window mode)
#pragma config FWDT_SWDTMPS = PS2147483648// Sleep Mode Watchdog Timer Post Scaler select bits (1:2147483648)
#pragma config FWDT_RCLKSEL = BFRC256   // Watchdog Timer Clock Select bits (WDT Run Mode uses BFRC:256)
#pragma config FWDT_RWDTPS = PS2147483648// Run Mode Watchdog Timer Post Scaler select bits (1:2147483648)
#pragma config FWDT_WDTWIN = WIN25      // Watchdog Timer Window Select bits (WDT Window is 25% of WDT period)
#pragma config FWDT_WDTEN = SW          // Watchdog Timer Enable bit (WDT is controlled by software, use WDTCON.ON bit)
#pragma config FWDT_WDTRSTEN = ON       // Watchdog Timer Reset Enable bit (WDT Event Generates a Reset)
#pragma config FWDT_WDTNVMSTL = ON      // WDT Stall During NVM Operation (Stall WDT During NVM Operation)
// </editor-fold>