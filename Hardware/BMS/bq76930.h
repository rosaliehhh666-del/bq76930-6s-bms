#ifndef BQ76930_H
#define BQ76930_H
#include "bms_core.h"
typedef struct {uint16_t gain_uv;int8_t offset_mv;bool crc,ready,has_current;uint32_t current_ms;int32_t current_ma;uint8_t outputs,balance,protection[6];uint32_t shunt_uohm;} bq76930_t;
bool bq_init(bq76930_t *d,const bms_config_t *cfg);
bool bq_sample(bq76930_t *d,bms_sample_t *s,uint32_t now);
bool bq_outputs(bq76930_t *d,bool chg,bool dis);
bool bq_balance(bq76930_t *d,uint8_t mask);
bool bq_check(bq76930_t *d);
bool bq_status(bq76930_t *d,uint8_t *status);
bool bq_clear_faults(bq76930_t *d,uint8_t allowed);
bool bq_ship(bq76930_t *d);
#endif
