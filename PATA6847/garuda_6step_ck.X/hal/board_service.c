/**
 * @file board_service.c
 * @brief Button debounce with edge detection.
 *
 * Call BoardService() from the main loop (runs at ~1ms via systemTick).
 * Buttons are active-low on EV43F54A board.
 */

#include "board_service.h"
#include "port_config.h"

#define DEBOUNCE_COUNT  20  /* ~20 ms at 1ms poll rate */

typedef struct {
    uint8_t count;
    bool    pressed;
    bool    edge;
    bool    prevRaw;
} BUTTON_STATE_T;

static BUTTON_STATE_T btn1, btn2;

void BoardServiceInit(void)
{
    btn1.count = 0; btn1.pressed = false; btn1.edge = false; btn1.prevRaw = true;
    btn2.count = 0; btn2.pressed = false; btn2.edge = false; btn2.prevRaw = true;
}

static void DebounceButton(bool rawLevel, BUTTON_STATE_T *b)
{
    b->edge = false;

    if (rawLevel == b->prevRaw)
    {
        if (b->count < DEBOUNCE_COUNT)
            b->count++;
        else
        {
            bool newState = !rawLevel;  /* Active-low */
            if (newState && !b->pressed)
                b->edge = true;
            b->pressed = newState;
        }
    }
    else
    {
        b->count = 0;
    }

    b->prevRaw = rawLevel;
}

void BoardService(void)
{
    DebounceButton(BTN1_GetValue(), &btn1);
    DebounceButton(BTN2_GetValue(), &btn2);
}

bool IsPressed_Button1(void) { bool e = btn1.edge; btn1.edge = false; return e; }
bool IsPressed_Button2(void) { bool e = btn2.edge; btn2.edge = false; return e; }
