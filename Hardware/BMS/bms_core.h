#ifndef BMS_CORE_H
#define BMS_CORE_H
#include "bms_config.h"
enum {BMS_F_COMM=1u,BMS_F_SENSOR=2u,BMS_F_OV=4u,BMS_F_UV=8u,
 BMS_F_CHG_TEMP=16u,BMS_F_DIS_TEMP=32u,BMS_F_CHG_OC=64u,BMS_F_DIS_OC=128u,
 BMS_F_HW=256u,BMS_F_PRECHARGE=512u,BMS_F_OUTPUT=1024u,BMS_F_WIRING=2048u,
 BMS_F_HW_OV=4096u,BMS_F_HW_UV=8192u,BMS_F_HW_OCD=16384u,BMS_F_HW_SCD=32768u,
 BMS_F_SPREAD_WARN=65536u,BMS_F_SPREAD_TRIP=131072u,
 BMS_F_T1=262144u,BMS_F_T2=524288u,BMS_F_BOARD_SENSOR=1048576u,BMS_F_BOARD_OT=2097152u};
#define BMS_FATAL_MASK (BMS_F_COMM|BMS_F_SENSOR|BMS_F_HW|BMS_F_DIS_OC|BMS_F_PRECHARGE|BMS_F_OUTPUT|BMS_F_SPREAD_TRIP)
#define BMS_LATCH_MASK (BMS_FATAL_MASK|BMS_F_CHG_OC|BMS_F_HW_OCD|BMS_F_HW_SCD|BMS_F_T1|BMS_F_T2|BMS_F_BOARD_SENSOR)

typedef enum {BMS_INIT,BMS_OFF,BMS_PRECHARGE,BMS_NORMAL,BMS_CHARGE_ONLY,
 BMS_DISCHARGE_ONLY,BMS_FAULT,BMS_SHUTDOWN} bms_state_t;
typedef struct {
    uint16_t cell_mv[BMS_CELLS];
    uint32_t pack_mv,output_mv,current_time_ms;
    int32_t current_ma;
    int16_t temp_dc[BMS_TS_COUNT],board_temp_dc;
    uint8_t hw_status;
    bool valid,temps_valid,current_valid,output_valid;
} bms_sample_t;

typedef struct {
    bool valid,has_sample;
    uint32_t time_ms,sample_ms,faults;
    bms_state_t state;
    uint8_t outputs;
    bms_sample_t sample;
} bms_fault_snapshot_t;
typedef struct {
    bms_state_t state;
    bms_config_t cfg;
    uint32_t faults,latched,last_sample_ms,precharge_start,idle_since,decision_ms;
    uint32_t oc_since_chg,oc_since_dis;
    bool request_chg,request_dis,hardware_verified;
    bool chg,dis,precharge,clear_request,shutdown_request;
    bool initialized,oc_chg_active,oc_dis_active,pre_active;
    float soc_pct;
    uint32_t soc_last_ms;
    bool soc_initialized;
    uint8_t balance_mask; /* six logical cells; converted to AFE banks by driver */
    uint16_t spread_mv;
    uint32_t spread_warn_since,spread_trip_since,spread_recover_since;
    bool spread_warn_active,spread_trip_active,spread_recover_active;
    bms_sample_t latest;
    bool has_latest,recovery_active;
    uint32_t recovery_since;
    bms_fault_snapshot_t snapshot;
} bms_core_t;
void bms_core_init(bms_core_t *b,const bms_config_t *c,bool verified,uint32_t now);
void bms_core_update(bms_core_t *b,const bms_sample_t *s,uint32_t now);
void bms_core_step(bms_core_t *b,const bms_sample_t *s,uint32_t now);
void bms_core_hw_fault(bms_core_t *b,uint8_t status,uint32_t now);
void bms_core_fail(bms_core_t *b,uint32_t fault);
float bms_soc_from_ocv(uint16_t mv);
bool bms_core_recovery_ready(const bms_core_t *b,const bms_sample_t *s,uint32_t now);
#endif
