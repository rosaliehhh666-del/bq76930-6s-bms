#include "bms_core.h"
#include "bms_util.h"
#include "bq76930.h"
#include "ina226.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
static unsigned checks;
#define CHECK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#x);exit(1);}}while(0)
static uint8_t regs[256];static bool nack,bad_crc,readback_bad;static unsigned writes,reads;
static int16_t raw_cc=100;
/* Bus mock exercises real driver code, CRC packing and readback failure paths. */
bool board_i2c(unsigned bus,uint8_t address,const uint8_t *tx,unsigned nt,uint8_t *rx,unsigned nr)
{
    unsigned i;uint8_t crc[3];if(nack)return false;
    if(bus!=0)return false;
    if(address==0x40){uint16_t v=0;if(address!=0x40)return false;if(nt==3){regs[200]=tx[1];regs[201]=tx[2];return true;}if(nr!=2)return false;switch(tx[0]){case 0xfe:v=0x5449;break;case 0xff:v=0x2260;break;case 0:v=(uint16_t)((regs[200]<<8)|regs[201]);break;case 1:v=1000;break;case 2:v=4000;break;default:break;}rx[0]=(uint8_t)(v>>8);rx[1]=(uint8_t)v;return true;}
    if(address!=8)return false;
    if(nt==3&&!nr){crc[0]=0x10;crc[1]=tx[0];crc[2]=tx[1];CHECK(tx[2]==bms_crc8(crc,3));writes++;if(tx[0]==0)regs[0]&=(uint8_t)~tx[1];else regs[tx[0]]=tx[1];return true;}
    if(nt!=1||nr%2)return false;
    reads++;
    for(i=0;i<nr/2;i++){uint8_t v=regs[(uint8_t)(tx[0]+i)];if(readback_bad&&tx[0]==5)v^=1;rx[i*2]=v;crc[0]=0x11;crc[1]=v;rx[i*2+1]=i?bms_crc8(crc+1,1):bms_crc8(crc,2);if(bad_crc)rx[i*2+1]^=1;}
    return true;
}
static void fixture(bms_core_t *b,bms_sample_t *s,bool verified,uint32_t now)
{
    bms_config_t c;unsigned i;bms_config_defaults(&c);bms_core_init(b,&c,verified,now);memset(s,0,sizeof(*s));for(i=0;i<6;i++)s->cell_mv[i]=3800;s->pack_mv=s->output_mv=22800;s->valid=s->temps_valid=s->current_valid=s->output_valid=true;s->temp_dc[0]=s->temp_dc[1]=s->board_temp_dc=250;s->current_time_ms=now;
}
static void step(bms_core_t *b,bms_sample_t *s,uint32_t now){bms_core_update(b,s,now);bms_core_step(b,s,now);}
static void test_util(void)
{
    int16_t t;uint8_t p[4];bms_config_t c;
    CHECK(bms_crc8((const uint8_t*)"123456789",9)==0xf4);CHECK(bms_crc16((const uint8_t*)"123456789",9)==0x4b37);CHECK(bms_crc32("123456789",9)==0xcbf43926);
    CHECK(bms_ntc_dc(1650,3300,10000,10000,3435,&t)&&t==250);CHECK(!bms_ntc_dc(0,3300,10000,10000,3435,&t));CHECK(!bms_ntc_dc(3300,3300,10000,10000,3435,&t));
    bms_put32(p,0x1234abcd);CHECK(bms_le16(p)==0xabcd&&bms_le16(p+2)==0x1234);
    bms_config_defaults(&c);CHECK(bms_config_valid(&c));c.precharge_ms=100;CHECK(!bms_config_valid(&c));
}
static void test_core(void)
{
    bms_core_t b;bms_sample_t s;float soc;unsigned i;
    fixture(&b,&s,false,0);step(&b,&s,250);CHECK(!b.chg&&!b.dis&&!b.precharge&&(b.faults&BMS_F_WIRING));
    fixture(&b,&s,true,0);bms_core_step(&b,&s,10);CHECK(b.state==BMS_INIT&&!b.chg&&!b.dis);
    step(&b,&s,250);CHECK(b.state==BMS_NORMAL&&b.chg&&b.dis);
    bms_core_step(&b,&s,1001);CHECK(b.state==BMS_FAULT&&!b.dis&&(b.latched&BMS_F_COMM));
    fixture(&b,&s,true,0);s.cell_mv[0]=2999;step(&b,&s,250);CHECK(b.state==BMS_CHARGE_ONLY&&b.chg&&!b.dis);
    s.cell_mv[0]=3200;step(&b,&s,500);CHECK(!b.dis);s.cell_mv[0]=3300;step(&b,&s,750);CHECK(b.dis);
    fixture(&b,&s,true,0);s.cell_mv[0]=4240;step(&b,&s,250);CHECK(!b.chg&&b.dis&&b.state==BMS_DISCHARGE_ONLY);
    fixture(&b,&s,true,0);s.temp_dc[0]=-10;step(&b,&s,250);CHECK(!b.chg&&b.dis);
    fixture(&b,&s,true,0);s.temp_dc[1]=650;step(&b,&s,250);CHECK(!b.chg&&!b.dis);
    fixture(&b,&s,true,0);s.board_temp_dc=810;step(&b,&s,250);CHECK(!b.chg&&!b.dis);
    fixture(&b,&s,true,0);s.temps_valid=false;step(&b,&s,250);CHECK((b.latched&BMS_F_SENSOR)&&!b.chg);
    fixture(&b,&s,true,0);s.output_valid=false;step(&b,&s,250);CHECK((b.latched&BMS_F_OUTPUT)&&!b.dis);
    fixture(&b,&s,true,0);s.output_mv=0;step(&b,&s,250);CHECK(b.precharge&&b.chg&&!b.dis);bms_core_step(&b,&s,329);CHECK(b.precharge);bms_core_step(&b,&s,330);CHECK(!b.precharge&&(b.latched&BMS_F_PRECHARGE));
    s.output_mv=s.pack_mv;step(&b,&s,500);CHECK(!b.dis);/* no automatic retry */
    fixture(&b,&s,true,0);s.output_mv=0;step(&b,&s,250);s.output_mv=22500;bms_core_step(&b,&s,280);CHECK(b.dis&&!b.precharge&&b.state==BMS_NORMAL);
    fixture(&b,&s,true,0);s.output_mv=26000;step(&b,&s,250);CHECK((b.latched&BMS_F_PRECHARGE)&&!b.dis);
    fixture(&b,&s,true,0);s.current_ma=-11000;step(&b,&s,250);step(&b,&s,450);CHECK((b.latched&BMS_F_DIS_OC)&&!b.dis);
    fixture(&b,&s,true,0);s.current_ma=2000;step(&b,&s,250);step(&b,&s,750);CHECK((b.latched&BMS_F_CHG_OC)&&!b.chg);
    fixture(&b,&s,true,0);s.hw_status=2;step(&b,&s,250);CHECK((b.latched&BMS_F_HW)&&!b.dis);
    fixture(&b,&s,true,0);step(&b,&s,250);b.shutdown_request=true;bms_core_step(&b,&s,260);CHECK(b.state==BMS_SHUTDOWN&&!b.chg&&!b.dis);
    fixture(&b,&s,true,0);step(&b,&s,0);soc=b.soc_pct;s.current_ma=-1000;for(i=1;i<=14400;i++)bms_core_update(&b,&s,i*250);CHECK(fabsf((soc-b.soc_pct)-33.3333f)<0.1f);
    fixture(&b,&s,true,0xfffffff0u);step(&b,&s,0xfffffff0u);bms_core_step(&b,&s,0x10);CHECK(b.dis);/* uint32 time wrap */
    fixture(&b,&s,true,0);for(i=0;i<6;i++)s.cell_mv[i]=(uint16_t)(3900+i*10);step(&b,&s,31000);CHECK(b.balance_mask!=0);CHECK((b.balance_mask&(b.balance_mask<<1))==0);
    b.request_chg=b.request_dis=false;bms_core_step(&b,&s,31001);CHECK(!b.chg&&!b.dis&&!b.balance_mask);
}
static void init_regs(void)
{
    unsigned i;static const uint8_t c[6]={0,1,4,5,6,9};uint16_t raw=3800*1000u/365u;
    memset(regs,0,sizeof(regs));nack=bad_crc=readback_bad=false;writes=0;
    for(i=0;i<6;i++){regs[0x0c+c[i]*2]=(uint8_t)(raw>>8);regs[0x0d+c[i]*2]=(uint8_t)raw;}
    raw=1650*1000u/382u;regs[0x2c]=regs[0x2e]=(uint8_t)(raw>>8);regs[0x2d]=regs[0x2f]=(uint8_t)raw;
    regs[0x32]=(uint8_t)(raw_cc>>8);regs[0x33]=(uint8_t)raw_cc;regs[0]=0x80;
}
static void test_drivers(void)
{
    bq76930_t d;bms_config_t cfg;bms_sample_t s;ina226_sample_t usb;
    bms_config_defaults(&cfg);init_regs();CHECK(bq_init(&d,&cfg));CHECK(d.crc&&d.gain_uv==365);CHECK(regs[6]==0x8a&&regs[7]==0x45&&regs[11]==0x19);CHECK(regs[5]==0x40);
    CHECK(bq_sample(&d,&s,250));CHECK(s.pack_mv>22780&&s.pack_mv<22810);CHECK(s.current_valid&&s.current_ma==211);CHECK(s.temps_valid&&s.temp_dc[0]>=249&&s.temp_dc[0]<=251);CHECK((regs[0]&0x80)==0);
    CHECK(bq_sample(&d,&s,1001)&&!s.current_valid);CHECK(bq_outputs(&d,true,true));CHECK(d.outputs==3);reads=0;CHECK(bq_check(&d));CHECK(reads==2);
    bad_crc=true;CHECK(!bq_check(&d)&&d.outputs==3);bad_crc=false;
    CHECK(!bq_balance(&d,3));CHECK(bq_balance(&d,0x24));CHECK(regs[1]==0x10&&regs[2]==0x10);CHECK(bq_check(&d));
    {unsigned j;for(j=6;j<=11;j++){uint8_t old=regs[j];regs[j]^=1;CHECK(!bq_check(&d));regs[j]=old;}CHECK(bq_check(&d));}
    readback_bad=true;CHECK(!bq_outputs(&d,true,false));CHECK(regs[5]==0x40);readback_bad=false;
    bad_crc=true;CHECK(!bq_sample(&d,&s,2000));writes=0;CHECK(!bq_init(&d,&cfg)&&writes==0);bad_crc=false;
    init_regs();CHECK(bq_init(&d,&cfg));CHECK(bq_outputs(&d,false,false));CHECK(bq_ship(&d));CHECK(regs[4]==2&&!d.ready);
    init_regs();regs[1]=1;regs[2]=2;readback_bad=true;
    CHECK(!bq_init(&d,&cfg));CHECK(regs[1]==0&&regs[2]==0&&!d.ready);
    regs[1]=1;regs[2]=2;CHECK(!bq_ship(&d));CHECK(regs[1]==0&&regs[2]==0);
    readback_bad=false;
    nack=true;CHECK(!bq_init(&d,&cfg));nack=false;CHECK(ina226_init());
    {uint8_t reg=0xfe,r[2];CHECK(!board_i2c(1,0x40,&reg,1,r,2));}CHECK(ina226_read(&usb,500)&&usb.bus_mv==5000&&usb.current_ma==500);nack=true;CHECK(!ina226_read(&usb,750)&&!usb.valid);
}
static void test_protection(void)
{
    bms_core_t b;bms_sample_t s;unsigned i;uint32_t first;
    fixture(&b,&s,true,0);s.hw_status=4;step(&b,&s,250);
    CHECK(!b.chg&&b.dis&&!b.latched&&(b.faults&BMS_F_HW_OV));
    CHECK(b.snapshot.valid&&b.snapshot.sample.hw_status==4&&b.snapshot.time_ms==250);
    s.cell_mv[0]=4200;s.current_time_ms=250;CHECK(!bms_core_recovery_ready(&b,&s,250));
    s.cell_mv[0]=3800;CHECK(bms_core_recovery_ready(&b,&s,250));
    first=b.snapshot.faults;s.hw_status=2;step(&b,&s,500);
    CHECK(!b.chg&&!b.dis&&(b.latched&BMS_F_HW_SCD));CHECK(b.snapshot.faults==first&&b.snapshot.time_ms==250);
    fixture(&b,&s,true,0);s.hw_status=8;step(&b,&s,250);CHECK(b.chg&&!b.dis&&!b.latched&&(b.faults&BMS_F_HW_UV));
    s.cell_mv[0]=3200;s.current_time_ms=250;CHECK(!bms_core_recovery_ready(&b,&s,250));
    fixture(&b,&s,true,0);s.hw_status=1;step(&b,&s,250);CHECK(!b.dis&&!b.chg&&(b.latched&BMS_F_HW_OCD));
    fixture(&b,&s,true,0);s.current_ma=2000;step(&b,&s,0);step(&b,&s,500);CHECK(!b.chg&&b.dis&&(b.latched&BMS_F_CHG_OC));
    s.current_ma=0;step(&b,&s,750);CHECK(!b.chg&&b.dis);/* no auto retry */
    fixture(&b,&s,true,0);s.cell_mv[0]=3550;
    for(i=0;i<=20;i++){s.current_time_ms=i*250;step(&b,&s,i*250);}
    CHECK((b.faults&BMS_F_SPREAD_WARN)&&b.chg&&b.dis&&!b.snapshot.valid);
    s.cell_mv[0]=3650;step(&b,&s,5250);CHECK(!(b.faults&BMS_F_SPREAD_WARN));
    fixture(&b,&s,true,0);s.cell_mv[0]=3500;s.cell_mv[1]=4100;
    for(i=0;i<=8;i++){s.current_time_ms=i*250;step(&b,&s,i*250);}
    CHECK((b.latched&BMS_F_SPREAD_TRIP)&&!b.chg&&!b.dis);CHECK(b.snapshot.valid&&b.snapshot.time_ms==2000&&b.snapshot.sample.cell_mv[0]==3500);
    s.cell_mv[0]=s.cell_mv[1]=3800;
    for(i=9;i<=28;i++){s.current_time_ms=i*250;step(&b,&s,i*250);}
    CHECK(!bms_core_recovery_ready(&b,&s,7000));s.current_time_ms=7250;step(&b,&s,7250);
    CHECK(bms_core_recovery_ready(&b,&s,7250));b.request_chg=b.request_dis=false;b.clear_request=true;bms_core_step(&b,&s,7250);CHECK(!b.latched&&!b.dis);
    CHECK(b.snapshot.time_ms==2000);/* recovery does not overwrite the evidence */
    fixture(&b,&s,true,0);s.cell_mv[0]=3500;s.cell_mv[1]=4100;step(&b,&s,0);step(&b,&s,3000);
    CHECK(!(b.latched&BMS_F_SPREAD_TRIP));/* missing samples do not satisfy hold time */
    fixture(&b,&s,true,0);s.current_ma=2000;step(&b,&s,0);step(&b,&s,1000);
    CHECK(!(b.latched&BMS_F_CHG_OC));step(&b,&s,1500);CHECK(b.latched&BMS_F_CHG_OC);
    fixture(&b,&s,true,0);s.current_ma=-11000;step(&b,&s,0);step(&b,&s,1000);
    CHECK(!(b.latched&BMS_F_DIS_OC));step(&b,&s,1250);CHECK(b.latched&BMS_F_DIS_OC);
    fixture(&b,&s,true,0);s.current_ma=2000;step(&b,&s,0);s.current_valid=false;step(&b,&s,250);
    CHECK(!b.oc_chg_active&&!(b.faults&BMS_F_CHG_OC));s.current_valid=true;step(&b,&s,500);
    CHECK(!(b.faults&BMS_F_CHG_OC));
    fixture(&b,&s,true,0);step(&b,&s,0);bms_core_hw_fault(&b,2,10);
    CHECK((b.latched&BMS_F_HW_SCD)&&!b.chg&&!b.dis);
    for(i=1;i<=20;i++){s.current_time_ms=i*250;step(&b,&s,i*250);}
    CHECK(!bms_core_recovery_ready(&b,&s,5000));s.current_time_ms=5250;step(&b,&s,5250);CHECK(bms_core_recovery_ready(&b,&s,5250));
    s.temp_dc[0]=440;s.current_time_ms=5500;step(&b,&s,5500);CHECK(!bms_core_recovery_ready(&b,&s,5500));
    s.temp_dc[0]=250;s.current_time_ms=5750;step(&b,&s,5750);CHECK(!bms_core_recovery_ready(&b,&s,5750));
    s.current_time_ms=12000;step(&b,&s,12000);CHECK(!bms_core_recovery_ready(&b,&s,12000));
    fixture(&b,&s,true,0);step(&b,&s,0);bms_core_hw_fault(&b,4,10);CHECK(!b.chg&&b.dis&&!b.latched);
    fixture(&b,&s,true,0);s.temp_dc[0]=(int16_t)0x8000;s.temps_valid=false;step(&b,&s,250);
    CHECK((b.faults&BMS_F_T1)&&!b.chg&&!b.dis);
    fixture(&b,&s,true,0);s.temp_dc[1]=(int16_t)0x8000;s.temps_valid=false;step(&b,&s,250);CHECK(b.faults&BMS_F_T2);
    fixture(&b,&s,true,0);s.board_temp_dc=810;step(&b,&s,250);CHECK((b.faults&BMS_F_BOARD_OT)&&!b.chg&&!b.dis);
    s.board_temp_dc=790;step(&b,&s,500);CHECK((b.faults&BMS_F_BOARD_OT)&&!b.chg&&!b.dis);
    s.board_temp_dc=750;step(&b,&s,750);CHECK(!(b.faults&BMS_F_BOARD_OT)&&b.chg&&b.dis);
    {bq76930_t d;bms_config_t cfg;bms_config_defaults(&cfg);init_regs();CHECK(bq_init(&d,&cfg));CHECK(bq_outputs(&d,true,true));regs[0]=4;regs[5]=0x42;CHECK(bq_check(&d)&&d.outputs==2);regs[5]=0x43;CHECK(!bq_check(&d));}
}
static void test_adc_loss_running(void)
{
    bms_core_t b;bms_sample_t s;
    fixture(&b,&s,true,0);step(&b,&s,0);CHECK(b.chg&&b.dis);
    s.output_valid=false;step(&b,&s,250);
    CHECK((b.latched&BMS_F_OUTPUT)&&!b.chg&&!b.dis&&!b.precharge);
    s.output_valid=true;step(&b,&s,500);CHECK(!b.dis); /* no automatic retry */
}
int main(void){test_adc_loss_running();test_util();test_core();test_drivers();test_protection();printf("PASS: %u checks (core, protection, snapshots, CRC, SOC, mocked BQ/INA buses)\n",checks);return 0;}
