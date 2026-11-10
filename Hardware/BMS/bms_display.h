#ifndef BMS_DISPLAY_H
#define BMS_DISPLAY_H
#include <stdint.h>
#include <stdbool.h>
void bms_display_init(void);
void bms_display_task(void);
/* Platform port; the view model can also run against a host test renderer. */
bool bms_lcd_init(void);
bool bms_lcd_page_key(void);
void bms_lcd_backlight(bool on);
void bms_lcd_line(unsigned row,const char *text,uint16_t color);
void bms_touch_init(void);
void bms_touch_calibration_start(void);
bool bms_touch_calibration_task(void);
bool bms_touch_read(uint16_t *x,uint16_t *y);
bool bms_touch_pressed(void);
#endif
