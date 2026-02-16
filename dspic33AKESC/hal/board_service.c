/**
 * @file board_service.c
 *
 * @brief Board service routines — button scanning, peripheral initialization,
 * PWM output enable/disable, heartbeat, trap handler.
 * Adapted from AN1292 reference for Project Garuda ESC.
 *   - Removes: Op-amp init, FOC-specific duty cycle functions,
 *              motor input read (MCAPP_MEASURE_T)
 *   - Keeps: Button debounce, Timer1 setup, PWM enable/disable
 *   - Changes: Init sequence calls our new HAL functions
 *
 * Component: BOARD SERVICE
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "board_service.h"

BUTTON_T buttonStartStop;
BUTTON_T buttonDirectionChange;

uint16_t boardServiceISRCounter = 0;

static void ButtonGroupInitialize(void);
static void ButtonScan(BUTTON_T *, bool);

/**
 * @brief Check if start/stop button was pressed.
 */
bool IsPressed_Button1(void)
{
    if (buttonStartStop.status)
    {
        buttonStartStop.status = false;
        return true;
    }
    return false;
}

/**
 * @brief Check if direction change button was pressed.
 */
bool IsPressed_Button2(void)
{
    if (buttonDirectionChange.status)
    {
        buttonDirectionChange.status = false;
        return true;
    }
    return false;
}

/**
 * @brief Increment board service counter (called from Timer1 ISR).
 */
void BoardServiceStepIsr(void)
{
    if (boardServiceISRCounter < BOARD_SERVICE_TICK_COUNT)
    {
        boardServiceISRCounter += 1;
    }
}

/**
 * @brief Board service tick — scans buttons at 1ms rate.
 */
void BoardService(void)
{
    if (boardServiceISRCounter == BOARD_SERVICE_TICK_COUNT)
    {
        ButtonScan(&buttonStartStop, BUTTON_START_STOP);
        ButtonScan(&buttonDirectionChange, BUTTON_DIRECTION_CHANGE);
        boardServiceISRCounter = 0;
    }
}

/**
 * @brief Initialize board service state.
 */
void BoardServiceInit(void)
{
    ButtonGroupInitialize();
    boardServiceISRCounter = BOARD_SERVICE_TICK_COUNT;
}

/**
 * @brief Scan a button with debounce logic.
 */
static void ButtonScan(BUTTON_T *pButton, bool button)
{
    if (button == true)
    {
        if (pButton->debounceCount < BUTTON_DEBOUNCE_COUNT)
        {
            pButton->debounceCount++;
            pButton->state = BUTTON_DEBOUNCE;
        }
    }
    else
    {
        if (pButton->debounceCount < BUTTON_DEBOUNCE_COUNT)
        {
            pButton->state = BUTTON_NOT_PRESSED;
        }
        else
        {
            pButton->state = BUTTON_PRESSED;
            pButton->status = true;
        }
        pButton->debounceCount = 0;
    }
}

/**
 * @brief Initialize button group state.
 */
static void ButtonGroupInitialize(void)
{
    buttonStartStop.state = BUTTON_NOT_PRESSED;
    buttonStartStop.debounceCount = 0;
    buttonStartStop.status = false;

    buttonDirectionChange.state = BUTTON_NOT_PRESSED;
    buttonDirectionChange.debounceCount = 0;
    buttonDirectionChange.status = false;
}

/**
 * @brief Initialize all peripherals (ADC, PWM, Timer1).
 * No op-amp or CMP init — ZC uses ADC threshold, not hardware comparators.
 */
void HAL_InitPeripherals(void)
{
    InitializeADCs();

    /* TODO: Re-add InitializeCMPs() + HAL_CMP_SetReference() here if PCI fault
     * is switched from RPn source (PSS=0b01000) to CMP3 source (PSS=0b11101) */

    /* Make sure ADC does not generate interrupt while initializing */
    GARUDA_DisableADCInterrupt();

    InitPWMGenerators();

    /* Enable PWM fault interrupt */
    ClearPWMIF();
    EnablePWMIF();

    /* Timer1 initialization — 100us tick */
    TIMER1_Initialize();
    TIMER1_InputClockSet();
    TIMER1_PeriodSet(TIMER1_PERIOD_COUNT);
    TIMER1_InterruptPrioritySet(5);
    TIMER1_InterruptFlagClear();
    TIMER1_InterruptEnable();
    TIMER1_ModuleStart();
}

/**
 * @brief Reset peripherals — clear ADC interrupt, disable PWM outputs.
 */
void HAL_ResetPeripherals(void)
{
    GARUDA_ClearADCIF();
    GARUDA_EnableADCInterrupt();
    HAL_MC1PWMDisableOutputs();
}

/**
 * @brief Enable PWM outputs — remove overrides, PWM generators drive pins.
 */
void HAL_MC1PWMEnableOutputs(void)
{
    PWM_PDC3 = 0;
    PWM_PDC2 = 0;
    PWM_PDC1 = 0;

    PG3IOCONbits.OVRENH = 0;
    PG3IOCONbits.OVRENL = 0;
    PG2IOCONbits.OVRENH = 0;
    PG2IOCONbits.OVRENL = 0;
    PG1IOCONbits.OVRENH = 0;
    PG1IOCONbits.OVRENL = 0;
}

/**
 * @brief Disable PWM outputs — override all to LOW.
 */
void HAL_MC1PWMDisableOutputs(void)
{
    PWM_PDC3 = 0;
    PWM_PDC2 = 0;
    PWM_PDC1 = 0;

    PG3IOCONbits.OVRDAT = 0;
    PG2IOCONbits.OVRDAT = 0;
    PG1IOCONbits.OVRDAT = 0;

    PG3IOCONbits.OVRENH = 1;
    PG3IOCONbits.OVRENL = 1;
    PG2IOCONbits.OVRENH = 1;
    PG2IOCONbits.OVRENL = 1;
    PG1IOCONbits.OVRENH = 1;
    PG1IOCONbits.OVRENL = 1;
}

/**
 * @brief Clear PWM PCI fault via software termination.
 */
void HAL_MC1ClearPWMPCIFault(void)
{
    PG1FPCIbits.SWTERM = 1;
    PG2FPCIbits.SWTERM = 1;
    PG3FPCIbits.SWTERM = 1;
}

/**
 * @brief Trap handler — disables all outputs and halts.
 */
void HAL_TrapHandler(void)
{
    HAL_MC1PWMDisableOutputs();
    while (1)
    {
        Nop();
        Nop();
        Nop();
    }
}
