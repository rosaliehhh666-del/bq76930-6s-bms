#ifndef BMS_APP_H
#define BMS_APP_H
#include "bms_core.h"
#include "ina226.h"
typedef struct {bms_core_t core;bms_sample_t sample;ina226_sample_t usb;uint32_t uptime_ms;uint32_t comm_errors;bool afe_ready;} bms_app_t;
extern bms_app_t bms;
void bms_app_init(void);
void bms_app_task(void);
/* 0 success, 1 illegal value, 2 busy/unsafe, 3 hardware/I2C/storage failure */
uint8_t bms_app_command(uint16_t command);
uint8_t bms_app_config(uint16_t address,uint16_t value);
uint16_t bms_app_register(uint16_t address);
void bms_comms_task(void);
bool bms_app_idle_allowed(void);
bool bms_comms_standby(void);
#endif
