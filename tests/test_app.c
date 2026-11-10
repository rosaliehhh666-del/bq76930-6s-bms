/* Runs the real application scheduler/core with mocked peripherals. */
#include "bms_app.h"
#include "bq76930.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static unsigned checks,inits,ships,reads,saves,balance_writes;static uint32_t now,time_shift;static bool alert,history_valid,append_ok=true;static uint8_t raw_status;static bms_fault_snapshot_t history;static bool pre,read_ok=true,adc_ok=true,write_ok=true,balance_ok=true,raw_trip;static uint16_t pack_adc=1230;
#define CHECK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL app line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
bool board_afe_alert(void){return alert;}
bool bq_status(bq76930_t *d,uint8_t *status){(void)d;*status=raw_status|(raw_trip?2:0);return read_ok;}
uint32_t board_ms(void){return now;}bool board_key(void){return false;}
void board_precharge(bool on){pre=on;}bool board_precharge_expired(void){return false;}
void board_buzzer(bool on){(void)on;}void board_watchdog_start(void){}void board_watchdog_feed(void){}void bms_comms_task(void){}
bool board_adc(uint16_t *p,uint16_t *t){*p=pack_adc;*t=2048;return adc_ok;}
bool bms_fault_log_load(bms_fault_snapshot_t *s){if(history_valid)*s=history;return history_valid;}
bool bms_fault_log_append(const bms_fault_snapshot_t *s){if(!append_ok)return false;history=*s;history_valid=true;return true;}
bool bms_fault_log_erase(void){return true;}
bool bms_storage_load(bms_config_t *c){(void)c;return false;}bool bms_storage_save(const bms_config_t *c){(void)c;saves++;return true;}
bool ina226_init(void){return false;}bool ina226_read(ina226_sample_t *s,uint32_t t){(void)s;(void)t;return false;}
bool bq_init(bq76930_t *d,const bms_config_t *c){(void)c;inits++;memset(d,0,sizeof(*d));d->ready=true;return true;}
bool bq_outputs(bq76930_t *d,bool c,bool x){if(!write_ok)return false;d->outputs=(uint8_t)((c?1:0)|(x?2:0));return true;}
bool bq_balance(bq76930_t *d,uint8_t m){balance_writes++;if(!balance_ok)return false;d->balance=m;return true;}
bool bq_check(bq76930_t *d){(void)d;return true;}
bool bq_clear_faults(bq76930_t *d,uint8_t mask){(void)d;(void)mask;raw_trip=false;return true;}
bool bq_ship(bq76930_t *d){ships++;d->ready=false;d->outputs=0;return true;}
bool bq_sample(bq76930_t *d,bms_sample_t *s,uint32_t t)
{unsigned i;(void)d;reads++;if(!read_ok)return false;memset(s,0,sizeof(*s));for(i=0;i<6;i++)s->cell_mv[i]=3800;s->pack_mv=22800;s->temp_dc[0]=s->temp_dc[1]=250;s->valid=s->temps_valid=s->current_valid=true;s->current_time_ms=t;s->hw_status=raw_status|(raw_trip?2:0);return true;}
static void task(uint32_t t){now=t+time_shift;bms_app_task();}
int main(void)
{
#if !BMS_PORT_ARMING_VERIFIED
    bms_app_init();task(250);CHECK(bms.core.initialized);
    CHECK(bms_app_command(1)==2);task(260);CHECK(!pre&&!bms.core.dis&&!bms.core.chg);
    printf("PASS: %u checks (unverified P+ topology blocks arming)\n",checks);return 0;
#endif
    bms_app_init();CHECK(bms_app_command(1)==2);task(250);CHECK(bms.core.initialized&&!bms.core.dis&&!bms.core.chg);
    CHECK(bms_app_command(1)==0);task(260);CHECK(bms.core.dis&&bms.core.chg);
    CHECK(bms_app_config(0x110,3500)==2);CHECK(bms_app_command(0)==0);task(260);CHECK(!bms.core.chg&&!bms.core.dis&&!pre);
    CHECK(bms_app_config(0x110,3500)==0&&bms.core.cfg.capacity_mah==3500);CHECK(bms_app_config(0x110,1)==1&&bms.core.cfg.capacity_mah==3500);CHECK(bms_app_command(4)==0);
    CHECK(!bms_app_idle_allowed());/* no USB measurement: cannot assert no load */
    bms.usb.valid=true;bms.usb.time_ms=260;CHECK(bms_app_idle_allowed());
    bms.usb.bus_mv=5000;CHECK(!bms_app_idle_allowed());bms.usb.bus_mv=0;
    bms.sample.current_ma=100;CHECK(!bms_app_idle_allowed());bms.sample.current_ma=0;bms.usb.valid=false;
    pack_adc=0;CHECK(bms_app_command(1)==0);task(500);CHECK(bms.core.precharge&&pre&&!bms.core.dis);pack_adc=1230;task(510);CHECK(bms.core.dis&&!pre);
    read_ok=false;task(750);CHECK((bms.core.latched&BMS_F_COMM)&&!bms.core.dis&&!pre&&!bms.afe_ready);
    CHECK(bms.core.snapshot.valid&&bms.core.snapshot.time_ms==750);
    CHECK(bms_app_register(0x31)==750&&bms_app_register(0x35)==BMS_F_COMM&&bms_app_register(0x3c)==3800);
    read_ok=true;task(1000);task(1250);CHECK(bms.afe_ready&&!bms.core.dis);CHECK(bms_app_command(2)==2);
    {unsigned i;for(i=1;i<=20;i++)task(1250+i*250);}time_shift+=5000;CHECK(bms_app_command(2)==0);task(1260);CHECK(bms.core.latched!=0);task(1500);CHECK(bms.core.latched==0&&!bms.core.dis&&!bms.core.request_dis);
    CHECK(bms_app_register(0x31)==750&&bms_app_register(0x61)==750);
    CHECK(bms_app_command(6)==0&&!bms.core.snapshot.valid);
    CHECK(bms_app_command(1)==0);task(1510);CHECK(bms.core.dis);
    adc_ok=false;task(1750);CHECK((bms.core.latched&BMS_F_SENSOR)&&!bms.core.dis&&!pre);adc_ok=true;CHECK(bms_app_command(2)==2);task(2000);CHECK(bms_app_command(2)==2);
    {unsigned i;for(i=1;i<=20;i++)task(2000+i*250);}time_shift+=5000;CHECK(bms_app_command(2)==0);task(2250);CHECK(!bms.core.latched&&!bms.core.dis);
    CHECK(bms_app_register(14)&1);bms.usb.valid=true;bms.usb.time_ms=0;
    now=3250+time_shift;CHECK((bms_app_register(14)&3)==0);now=2250+time_shift;bms.usb.valid=false;
    /* OFF->ON between control ticks must clear old DSG and use a new ADC. */
    CHECK(bms_app_command(1)==0);task(2260);CHECK(bms.core.dis);
    CHECK(bms_app_command(0)==0);CHECK(!bms.core.dis&&!bms.core.chg&&!bms.core.pre_active);
    pack_adc=0;CHECK(bms_app_command(1)==0);task(2270);CHECK(pre&&!bms.core.dis);
    CHECK(bms_app_command(0)==0);pack_adc=1230;
    {unsigned saved=saves;balance_ok=false;CHECK(bms_app_command(4)==3);CHECK(saves==saved);balance_ok=true;}
    {unsigned saved=saves,bw=balance_writes;write_ok=false;CHECK(bms_app_command(4)==3);CHECK(saves==saved);CHECK(balance_writes>bw);write_ok=true;}
    CHECK(bms_app_command(3)==0);task(2280);CHECK(ships==1&&bms.core.state==BMS_SHUTDOWN&&!bms.sample.valid);
    {unsigned n=inits;task(6000);task(10000);CHECK(inits==n&&ships==1&&!pre);}
    history_valid=false;bms_app_init();task(10250);CHECK(bms_app_command(1)==0);task(10260);
    {unsigned before=reads;alert=raw_trip=true;task(10270);CHECK(reads==before);CHECK((bms.core.latched&BMS_F_HW_SCD)&&!bms.core.dis&&!pre);}
    raw_trip=false;raw_status=4;task(10280);raw_status=8;task(10290);
    CHECK(bms_app_register(0x18)&16);CHECK(bms_app_command(6)==2);
    raw_status=0;adc_ok=false;task(10520);CHECK(bms.core.latched&BMS_F_BOARD_SENSOR);
    adc_ok=true;write_ok=false;CHECK(bms_app_command(0)==3);task(10530);
    bms.core.faults|=BMS_F_CHG_TEMP;task(10531); /* another distinct event fills queue */
    read_ok=false;task(10770);CHECK(bms.core.latched&BMS_F_COMM);
    CHECK((bms.core.latched&BMS_F_OUTPUT)&&!(bms_app_register(0x18)&4));
    CHECK(bms_app_register(0x18)&32); /* bounded queue never silently overwrites */
    write_ok=read_ok=true;alert=false;
    {unsigned i;uint32_t first=bms.core.snapshot.time_ms;for(i=0;i<=24;i++)task(12000+i*250);
        CHECK(!(bms_app_register(0x18)&16));CHECK(bms_app_register(0x18)&32);
        CHECK(bms.core.snapshot.time_ms==first);CHECK(history.faults&BMS_F_HW_SCD);}
    bms_app_init();CHECK(!bms.core.request_dis&&!bms.core.request_chg);
    CHECK(bms.core.latched&BMS_F_HW_SCD);CHECK(bms_app_command(1)==2);
    /* Lose the ADC during a live precharge, between full BQ samples. */
    history_valid=false;bms_app_init();task(19000);CHECK(bms_app_command(1)==0);
    pack_adc=0;task(19010);CHECK(pre&&!bms.core.dis);
    adc_ok=false;task(19020);CHECK(!pre&&!bms.core.chg&&!bms.core.dis);
    CHECK(bms.core.latched&BMS_F_BOARD_SENSOR);adc_ok=true;pack_adc=1230;
    /* Pending log must survive maintenance/SHIP requests until append succeeds. */
    history_valid=false;bms_app_init();
    {unsigned sh=ships;write_ok=false;CHECK(bms_app_command(0)==3);write_ok=true;
     CHECK(bms_app_command(7)==2);CHECK(bms_app_command(3)==0);
     append_ok=false;bms_app_task();CHECK(ships==sh);CHECK((bms_app_register(0x18)&192)==192);CHECK(bms_app_register(0x19)>0);append_ok=true;now+=1000;bms_app_task();CHECK(ships==sh+1);}
    printf("PASS: %u checks (application scheduling, precharge ordering, I2C/ADC faults, explicit recovery, shutdown)\n",checks);return 0;
}
