#ifndef BMS_INA226_H
#define BMS_INA226_H
#include <stdint.h>
#include <stdbool.h>
typedef struct {uint32_t bus_mv,time_ms;int32_t current_ma;bool valid;} ina226_sample_t;
bool ina226_init(void);
bool ina226_read(ina226_sample_t *s,uint32_t now);
#endif
