#include "bms_app.h"
#include "bms_board.h"
#include "bq76930.h"
#include "bms_storage.h"
#include "bms_util.h"
#include <string.h>
bms_app_t bms;
static bq76930_t afe;
static uint32_t sample_at,control_at,retry_at,ina_retry_at,ready_at,key_at,key_long_at;
static bool ina_ready,key_previous,key_stable,key_long,sampling_settle,shutdown_done,pending_clear;
static uint32_t settle_at,alert_at;
static bool off_confirmed,event_overflow,log_write_failed;
static uint32_t task_last;static uint16_t task_gap_max;
static bms_fault_snapshot_t events[4];
static uint8_t event_head,event_count;
static uint32_t observed_faults;
static bool snapshot_saved,previous_valid;
static uint32_t log_retry_at;
static bms_fault_snapshot_t previous_fault;
/* Attempt both writes even if the first one fails. Readback is not MOS proof. */
static bool stop_outputs(void)
{bool a=bq_outputs(&afe,false,false),b=bq_balance(&afe,0);return a&&b;}
static bool logs_pending(void)
{return (bms.core.snapshot.valid&&!snapshot_saved)||event_count!=0;}
static void fail(uint32_t fault)
{board_precharge(false);off_confirmed=false;bms.core.decision_ms=board_ms();bms_core_fail(&bms.core,fault);
 if(afe.ready)off_confirmed=stop_outputs();
 if(!off_confirmed)bms_core_fail(&bms.core,BMS_F_OUTPUT);bms.comm_errors++;}
static bool analog(void)
{
    uint16_t p,t;int16_t dc;
    bms.sample.output_valid=false;bms.sample.board_temp_dc=(int16_t)0x8000;
    if(!board_adc(&p,&t)||!bms_ntc_dc((uint32_t)t*3300u/4095u,3300,100000,100000,BMS_BOARD_NTC_BETA,&dc)){bms.sample.temps_valid=false;return false;}
    bms.sample.output_mv=(uint32_t)p*BMS_ADC_VREF_MV*BMS_PACK_DIV_NUM/(4095u*BMS_PACK_DIV_DEN);
    bms.sample.output_valid=p<4090;bms.sample.board_temp_dc=dc;return true;
}
void bms_app_init(void)
{
    bms_config_t cfg;memset(&bms,0,sizeof(bms));bms_config_defaults(&cfg);(void)bms_storage_load(&cfg);
    bms_core_init(&bms.core,&cfg,BMS_HARDWARE_VERIFIED!=0,board_ms());
    /* Every reset requires explicit arming, even when no history was saved. */
    bms.core.request_chg=bms.core.request_dis=false;
    sampling_settle=shutdown_done=pending_clear=off_confirmed=false;
    task_last=board_ms();task_gap_max=0;log_write_failed=false;
    event_head=event_count=0;observed_faults=0;event_overflow=false;
    key_previous=key_stable=key_long=false;alert_at=board_ms()-10;

    previous_valid=bms_fault_log_load(&previous_fault);snapshot_saved=false;log_retry_at=board_ms()-1000;
    if(previous_valid){
        bms.core.latched=previous_fault.faults&BMS_LATCH_MASK;
        if(bms.core.latched){bms.core.snapshot=previous_fault;snapshot_saved=true;observed_faults=bms.core.latched;}
    }
    bms.afe_ready=bq_init(&afe,&cfg);ready_at=board_ms();retry_at=ready_at;sample_at=ready_at;control_at=ready_at;
    ina_ready=ina226_init();ina_retry_at=board_ms();board_watchdog_start();
}
static bool apply(void)
{
    bms_core_t *c=&bms.core;uint8_t expected=(uint8_t)((c->chg?1:0)|(c->dis?2:0));
    if(!c->precharge)board_precharge(false);
    /* CHG write and readback must succeed before the discrete precharge can turn on. */
    if(afe.outputs!=expected&&!bq_outputs(&afe,c->chg,c->dis)){fail(BMS_F_OUTPUT);return false;}
    if(expected||c->precharge)off_confirmed=false;
    if(c->precharge)board_precharge(true);
    if(!sampling_settle&&afe.balance!=c->balance_mask&&!bq_balance(&afe,c->balance_mask)){fail(BMS_F_COMM);return false;}return true;
}
void bms_app_task(void)
{
    uint32_t now=board_ms(),gap=now-task_last;bool k=board_key();bms.uptime_ms=now;
    task_last=now;if(gap>65535)gap=65535;if(gap>task_gap_max)task_gap_max=(uint16_t)gap;
    if(shutdown_done){board_precharge(false);bms_comms_task();board_watchdog_feed();return;}
    if(k!=key_previous){key_previous=k;key_at=now;}
    if(now-key_at>=30&&key_stable!=k){key_stable=k;if(k){key_long_at=now;key_long=false;}else if(!key_long&&bms.core.initialized)(void)bms_app_command((bms.core.request_chg||bms.core.request_dis)?0:1);}
    if(key_stable&&!key_long&&now-key_long_at>=2000){key_long=true;(void)bms_app_command(3);}
    if(!bms.afe_ready){
        board_precharge(false);
        if(now-retry_at>=1000){retry_at=now;bms.afe_ready=bq_init(&afe,&bms.core.cfg);if(bms.afe_ready)ready_at=now;else bms.comm_errors++;}
    }else{
        if(board_afe_alert()&&now-alert_at>=10){
            uint8_t status;alert_at=now;
            if(!bq_status(&afe,&status)){fail(BMS_F_COMM);bms.afe_ready=false;afe.ready=false;}
            else if(status&0x3f){
                bms.sample.hw_status=status&0x3f;
                bms_core_hw_fault(&bms.core,status,now);
                if(!apply())bms.afe_ready=false;
            }
            if(!bms.afe_ready)goto fault_tail;
        }
        if(now-sample_at>=BMS_SAMPLE_MS&&!sampling_settle){
            if(afe.balance){if(!bq_balance(&afe,0))fail(BMS_F_COMM);sampling_settle=true;settle_at=now;}
            else {sampling_settle=true;settle_at=now-100;}
        }
        if(sampling_settle&&now-settle_at>=100){
            bms_sample_t s;sampling_settle=false;sample_at=now;
            if(!bq_sample(&afe,&s,now)||!bq_check(&afe)){bms.sample.valid=false;fail(BMS_F_COMM);bms.afe_ready=false;afe.ready=false;}
            else {
                bms.sample=s;(void)analog();
                /* TS conversions can take seconds after BOOT. Keep outputs off while settling. */
                if(bms.core.initialized||(s.current_valid&&bms.sample.temps_valid)||now-ready_at>=3000){
                    bms_core_update(&bms.core,&bms.sample,now);
                    if(pending_clear){bms.core.clear_request=true;pending_clear=false;}
                }
            }
            if(ina_ready){if(!ina226_read(&bms.usb,now)){ina_ready=false;ina_retry_at=now;}}else if(now-ina_retry_at>=1000){ina_retry_at=now;ina_ready=ina226_init();}
        }
        if(now-control_at>=10){
            control_at=now;
            /* A previous OFF request may have disconnected the load since
               the last 250 ms snapshot. Never arm DSG using that old ADC. */
            if(bms.core.precharge||(!bms.core.dis&&bms.core.request_dis)){
                if(!analog())fail(BMS_F_SENSOR|BMS_F_BOARD_SENSOR);
                else if(!bms.sample.output_valid)fail(BMS_F_OUTPUT);
            }
            if(board_precharge_expired())fail(BMS_F_PRECHARGE);
            if(bms.sample.current_valid&&now-bms.sample.current_time_ms>BMS_STALE_MS)bms.sample.current_valid=false;
            if(bms.core.initialized&&!bms.sample.current_valid&&!(bms.core.latched&BMS_F_SENSOR))fail(BMS_F_SENSOR);
            bms_core_step(&bms.core,&bms.sample,now);
            if(!apply())bms.afe_ready=false;
        }
    }
fault_tail:
    {uint32_t all=bms.core.faults|bms.core.latched;
        uint32_t added=(all&~observed_faults)&~(BMS_F_WIRING|BMS_F_SPREAD_WARN);
        observed_faults=all;
        if(added&&bms.core.snapshot.valid&&(snapshot_saved||(added&~bms.core.snapshot.faults))){
            if(event_count<4){bms_fault_snapshot_t *e=&events[(event_head+event_count)%4];
                *e=bms.core.snapshot;e->time_ms=now;e->faults=added|bms.core.latched;
                e->has_sample=bms.core.has_latest;e->sample=bms.sample;e->sample_ms=bms.core.last_sample_ms;e->state=bms.core.state;
                e->outputs=(uint8_t)((bms.core.chg?1:0)|(bms.core.dis?2:0)|(bms.core.precharge?4:0));
                event_count++;
            }else event_overflow=true;
        }
    }
    if(!bms.afe_ready&&now-ready_at>=3000&&!(bms.core.latched&BMS_F_COMM))fail(BMS_F_COMM);
    /* Persist only after locally stopping outputs, with explicit AFE readback.
       Directional faults are recorded in RAM immediately, Flash is deferred
       while the opposite direction remains active. Never erase on a fault. */
    if(bms.core.snapshot.valid&&(!snapshot_saved||event_count)&&now-log_retry_at>=1000&&
       afe.ready&&!afe.outputs&&!bms.core.chg&&!bms.core.dis&&!bms.core.precharge){
        log_retry_at=now;
        const bms_fault_snapshot_t *entry=snapshot_saved?&events[event_head]:&bms.core.snapshot;
        log_write_failed=true;
        if(stop_outputs()&&bms_fault_log_append(entry)){
            log_write_failed=false;
            off_confirmed=true;previous_fault=*entry;previous_valid=true;
            if(snapshot_saved){event_head=(event_head+1)%4;event_count--;}else snapshot_saved=true;
        }
    }
    if(bms.core.shutdown_request&&!shutdown_done){
        board_precharge(false);if(!logs_pending()&&afe.ready&&stop_outputs()&&bq_ship(&afe)){shutdown_done=true;bms.afe_ready=false;bms.sample.valid=false;bms.usb.valid=false;bms.core.state=BMS_SHUTDOWN;}
        /* REGOUT->MP9486_EN removes MCU supply. If it does not, remain safe. */
    }
    if(shutdown_done){board_precharge(false);bms.afe_ready=false;}
    board_buzzer((bms.core.latched!=0)&&((now%2000)<100));bms_comms_task();board_watchdog_feed();
}
uint8_t bms_app_command(uint16_t command)
{
    switch(command){
    case 0:
        bms.core.request_chg=bms.core.request_dis=false;
        bms.core.chg=bms.core.dis=bms.core.precharge=bms.core.pre_active=false;
        bms.core.balance_mask=0;board_precharge(false);
        if(!afe.ready||!stop_outputs()){fail(BMS_F_OUTPUT);return 3;}
        if(!bms.core.shutdown_request)bms.core.state=BMS_OFF;
        return 0;
    case 1:if(!BMS_PORT_ARMING_VERIFIED||!BMS_HARDWARE_VERIFIED||!bms.afe_ready||!bms.core.initialized||
        !bms.sample.valid||board_ms()-bms.core.last_sample_ms>BMS_STALE_MS||
        bms.core.shutdown_request||bms.core.latched)return 2;
        bms.core.request_chg=true;bms.core.request_dis=true;return 0;
    case 2:
        if(afe.outputs||bms.core.precharge||!bms.sample.valid||bms.sample.current_ma>200||bms.sample.current_ma< -200)return 2;
        if(board_precharge_expired())return 2;/* hard timeout requires reset/inspection */
        if(!bms_core_recovery_ready(&bms.core,&bms.sample,board_ms()))return 2;
        off_confirmed=stop_outputs();if(off_confirmed)off_confirmed=bq_check(&afe);
        if(!off_confirmed){fail(BMS_F_OUTPUT);return 3;}
        if(!bq_clear_faults(&afe,0x3f))return 3;bms.core.request_chg=bms.core.request_dis=false;pending_clear=true;return 0;
    case 3:bms.core.shutdown_request=true;bms.core.request_chg=bms.core.request_dis=false;board_precharge(false);return 0;
    case 4:
        if(!bms.afe_ready||afe.outputs||bms.core.request_chg||bms.core.request_dis||bms.core.precharge)return 2;
        /* Flash erase stalls execution: first verify balancing is disabled. */
        if(!stop_outputs()){fail(BMS_F_COMM);return 3;}
        bms.core.balance_mask=0;
        return bms_storage_save(&bms.core.cfg)?0:3;
    case 6:/* Explicitly re-arm the FIRST-fault recorder, not a fault reset. */
        if(afe.outputs||bms.core.request_chg||bms.core.request_dis||bms.core.precharge||
           bms.core.latched||(bms.core.faults&~BMS_F_WIRING))return 2;
        if(event_count||(bms.core.snapshot.valid&&!snapshot_saved))return 2;/* do not discard an unflushed event */
        memset(&bms.core.snapshot,0,sizeof(bms.core.snapshot));snapshot_saved=false;event_overflow=false;return 0;
    case 7:/* Maintenance only: explicit log-sector erase, never fault recovery. */
        if(logs_pending()||!afe.ready||afe.outputs||bms.core.request_chg||bms.core.request_dis||bms.core.precharge)return 2;
        if(!stop_outputs()||!bms_fault_log_erase())return 3;
        previous_valid=false;snapshot_saved=false;return 0;
    default:return 1;
    }
}
uint8_t bms_app_config(uint16_t address,uint16_t value)
{
    bms_config_t c=bms.core.cfg;
    if(afe.outputs||bms.core.precharge||bms.core.request_chg||bms.core.request_dis)return 2;
    switch(address){case 0x110:c.capacity_mah=value;break;case 0x111:c.charge_ma=value;break;case 0x112:c.discharge_ma=value;break;case 0x113:c.balance_enable=(uint8_t)value;if(value>1)return 1;break;default:return 1;}
    if(!bms_config_valid(&c))return 1;bms.core.cfg=c;return 0;
}
bool bms_app_idle_allowed(void)
{
    uint32_t now=board_ms();
    return bms.afe_ready&&bms.core.initialized&&bms.sample.valid&&
        bms.sample.temps_valid&&bms.sample.current_valid&&
        now-bms.core.last_sample_ms<=BMS_STALE_MS&&
        now-bms.sample.current_time_ms<=BMS_STALE_MS&&
        !(bms.core.faults|bms.core.latched)&&!pending_clear&&
        !logs_pending()&&!log_write_failed&&!bms.core.shutdown_request&&!bms.core.request_chg&&!bms.core.request_dis&&
        !bms.core.chg&&!bms.core.dis&&!bms.core.precharge&&!afe.outputs&&
        !afe.balance&&!sampling_settle&&
        bms.sample.current_ma>=-50&&bms.sample.current_ma<=50&&
        bms.usb.valid&&now-bms.usb.time_ms<=1000u&&bms.usb.bus_mv<4000;
}
static uint16_t data_flags(void)
{
    uint32_t now=board_ms();
    bool fresh=bms.sample.valid&&bms.core.initialized&&bms.sample.current_valid&&
        now-bms.core.last_sample_ms<=BMS_STALE_MS&&
        now-bms.sample.current_time_ms<=BMS_STALE_MS;
    bool usb=bms.usb.valid&&now-bms.usb.time_ms<=1000u;
    return (uint16_t)((fresh?1:0)|(usb?2:0)|(BMS_HARDWARE_VERIFIED?4:0));
}
uint16_t bms_app_register(uint16_t address)
{
    if(address==0x16)return (uint16_t)((bms.core.faults|bms.core.latched)>>16);
    if(address==0x17)return bms.core.spread_mv;
    if(address==0x18)return (uint16_t)((snapshot_saved?1:0)|(previous_valid?2:0)|(off_confirmed?4:0)|(bms.core.recovery_active?8:0)|(event_count?16:0)|(event_overflow?32:0)|(log_write_failed?64:0)|((bms.core.shutdown_request&&logs_pending())?128:0));
    if(address==0x19)return event_count+(bms.core.snapshot.valid&&!snapshot_saved?1:0);
    if(address==0x1a)return task_gap_max;
    if(address==0x1b)return (uint16_t)((BMS_HARDWARE_VERIFIED?1:0)|(BMS_PORT_ARMING_VERIFIED?2:0));
    if((address>=0x30&&address<0x50)||(address>=0x60&&address<0x80)){
        const bms_fault_snapshot_t *f=address>=0x60?&previous_fault:&bms.core.snapshot;const bms_sample_t *s=&f->sample;
        unsigned a=address-(address>=0x60?0x60:0x30);
        if(address>=0x60&&!previous_valid)return 0;
        if(a>=12&&a<18)return s->cell_mv[a-12];
        switch(a){case 0:return (uint16_t)((f->valid?1:0)|(f->has_sample?2:0)|(s->valid?4:0)|(s->temps_valid?8:0)|(s->current_valid?16:0));
        case 1:return (uint16_t)f->time_ms;case 2:return (uint16_t)(f->time_ms>>16);
        case 3:return (uint16_t)f->sample_ms;case 4:return (uint16_t)(f->sample_ms>>16);
        case 5:return (uint16_t)f->faults;case 6:return (uint16_t)(f->faults>>16);
        case 7:return (uint16_t)f->state;case 8:return f->outputs;case 9:return s->hw_status;
        case 10:return (uint16_t)s->current_time_ms;case 11:return (uint16_t)(s->current_time_ms>>16);
        case 18:return (uint16_t)s->current_ma;case 19:return (uint16_t)((uint32_t)s->current_ma>>16);
        case 20:return (uint16_t)s->pack_mv;case 21:return (uint16_t)(s->pack_mv>>16);
        case 22:return (uint16_t)s->output_mv;case 23:return (uint16_t)(s->output_mv>>16);
        case 24:return (uint16_t)s->temp_dc[0];case 25:return (uint16_t)s->temp_dc[1];case 26:return (uint16_t)s->board_temp_dc;
        default:return 0;}
    }
    if(address>=0x10&&address<0x16)return bms.sample.cell_mv[address-0x10];
    switch(address){case 0:return 0x0100;case 1:return (uint16_t)bms.core.state;case 2:return (uint16_t)(bms.core.faults|bms.core.latched);case 3:return (uint16_t)(bms.core.soc_pct*10);case 4:return (uint16_t)bms.sample.pack_mv;case 5:return (uint16_t)bms.sample.current_ma;case 6:return (uint16_t)bms.sample.output_mv;case 7:return (uint16_t)bms.sample.temp_dc[0];case 8:return (uint16_t)bms.sample.temp_dc[1];case 9:return (uint16_t)bms.sample.board_temp_dc;case 10:return (uint16_t)((bms.core.chg?1:0)|(bms.core.dis?2:0)|(bms.core.precharge?4:0));case 11:return bms.core.balance_mask;case 12:return (uint16_t)bms.usb.bus_mv;case 13:return (uint16_t)bms.usb.current_ma;case 14:return data_flags();case 15:return (uint16_t)(bms.uptime_ms/1000);case 0x110:return (uint16_t)bms.core.cfg.capacity_mah;case 0x111:return bms.core.cfg.charge_ma;case 0x112:return bms.core.cfg.discharge_ma;case 0x113:return bms.core.cfg.balance_enable;default:return 0xffff;}
}
