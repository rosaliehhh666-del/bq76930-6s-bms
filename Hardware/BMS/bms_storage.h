#ifndef BMS_STORAGE_H
#define BMS_STORAGE_H
#include "bms_config.h"
#include "bms_core.h"
bool bms_storage_load(bms_config_t *c);
bool bms_storage_save(const bms_config_t *c);
bool bms_fault_log_load(bms_fault_snapshot_t *snapshot);
bool bms_fault_log_append(const bms_fault_snapshot_t *snapshot);
bool bms_fault_log_erase(void);
#endif
