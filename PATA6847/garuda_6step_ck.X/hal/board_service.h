/**
 * @file board_service.h
 * @brief Button debounce and board-level services.
 */
#ifndef HAL_BOARD_SERVICE_H
#define HAL_BOARD_SERVICE_H

#include <stdint.h>
#include <stdbool.h>

void BoardServiceInit(void);
void BoardService(void);
bool IsPressed_Button1(void);
bool IsPressed_Button2(void);

#endif
