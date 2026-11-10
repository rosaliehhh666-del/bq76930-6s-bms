#include "stm32f4xx.h"
#include "bms_board.h"
#include "bms_app.h"
#include "bms_display.h"

int main(void)
{
    /* Startup already called SystemInit; delay.c must not take over SysTick. */
    static const uint8_t message[] = "BMS 6S F407 SPL ready; check status and README_BMS before operation.\r\n";
    board_init();
    bms_display_init();
    bms_app_init();
    board_uart_send(0, message, sizeof(message) - 1);
    while (1) {
        bms_app_task();
        bms_display_task();
    }
}
