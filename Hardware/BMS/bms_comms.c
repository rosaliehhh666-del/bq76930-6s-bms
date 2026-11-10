#include "bms_app.h"
#include "bms_board.h"
#include "bms_util.h"
#include <string.h>
/* All protocols use explicit byte serialization, never compiler struct layout. */
static uint8_t rtu[128],esp[128];
static unsigned rn,en;
static uint32_t rtime,etime,can_at,telemetry_at,heartbeat_at,reset_at,cooldown_at,extended_at;
static bool heartbeat_seen,resetting,rtu_drop;
static uint8_t can_phase;
static uint16_t seq;
typedef enum {PM_ACTIVE,PM_WAIT_READY,PM_WAIT_DRAIN,PM_ASLEEP,PM_WAKING} pm_state_t;
static pm_state_t pm;
static bool sleep_capable,peer_ready,alarm_pending;
static uint16_t sleep_seq;
static uint32_t idle_at,pm_at,last_faults,alarm_faults;
static bool alarm_inflight;
static uint16_t alarm_seq;
static uint32_t alarm_sent_bits,alarm_at;
bool bms_comms_standby(void){return pm==PM_ASLEEP;}
static void activity(void)
{
    idle_at=board_ms();
    if(pm==PM_ASLEEP||pm==PM_WAIT_DRAIN){
        board_links_standby(false);board_esp_reset(true);pm=PM_WAKING;
        pm_at=board_ms();heartbeat_seen=false;peer_ready=false;
    }else if(pm==PM_WAIT_READY){pm=PM_ACTIVE;/* Late READY will be ignored. */}
}
static bool register_valid(uint16_t a){return a<0x1c||(a>=0x30&&a<0x50)||(a>=0x60&&a<0x80)||(a>=0x110&&a<=0x113);}
static void rtu_process(void)
{
    uint8_t out[128],status=0;unsigned n=0,i;uint16_t address,count,crc;
    if(rn<4||rtu[0]!=BMS_MODBUS_ADDRESS||bms_crc16(rtu,rn-2)!=bms_le16(rtu+rn-2))return;
    activity();
    out[0]=rtu[0];out[1]=rtu[1];
    if((rtu[1]==3||rtu[1]==4)&&rn==8){
        address=(uint16_t)((rtu[2]<<8)|rtu[3]);count=(uint16_t)((rtu[4]<<8)|rtu[5]);
        if(!count||count>48||((uint32_t)address+count)>65536)status=3;
        for(i=0;!status&&i<count;i++)if(!register_valid((uint16_t)(address+i)))status=2;
        if(!status){out[2]=(uint8_t)(count*2);for(i=0;i<count;i++){uint16_t v=bms_app_register((uint16_t)(address+i));out[3+i*2]=(uint8_t)(v>>8);out[4+i*2]=(uint8_t)v;}n=3+count*2;}
    }else if(rtu[1]==6&&rn==8){
        uint8_t result;address=(uint16_t)((rtu[2]<<8)|rtu[3]);count=(uint16_t)((rtu[4]<<8)|rtu[5]);
        if(address==0x100)result=bms_app_command(count);
        else if(address>=0x110&&address<=0x113)result=bms_app_config(address,count);
        else result=1;
        if(result)status=result==1?3:result==2?6:4;else{memcpy(out,rtu,6);n=6;}
    }else status=1;
    if(status){out[1]|=0x80;out[2]=status;n=3;}crc=bms_crc16(out,n);bms_put16(out+n,crc);(void)board_uart_send(1,out,n+2);
}
static void modbus_task(uint32_t now)
{
    uint8_t v;uint32_t t;unsigned budget=256;
    if(board_uart_overflow(1)){
        /* Loss position is unknown: discard the queued bytes, not just the
           partial frame, then wait for a fresh idle interval. */
        while(budget--&&board_uart_get(1,&v,&t)){}rn=0;rtu_drop=true;rtime=now;return;
    }
    while(budget--&&board_uart_get(1,&v,&t)){
        /* 115200 baud: Modbus recommends fixed 1.75 ms above 19200; use 2 ms. */
        if((rn||rtu_drop)&&t-rtime>=2){if(!rtu_drop)rtu_process();rn=0;rtu_drop=false;}
        rtime=t;if(!rtu_drop){if(rn<sizeof(rtu))rtu[rn++]=v;else{rn=0;rtu_drop=true;}}
    }
    if((rn||rtu_drop)&&now-rtime>=2){if(!rtu_drop)rtu_process();rn=0;rtu_drop=false;}
}
static unsigned status_payload(uint8_t *p,unsigned count)
{
    unsigned i;/* 22 registers, 44 bytes; LE signed current/temperature. */
    for(i=0;i<count;i++)bms_put16(p+2*i,bms_app_register((uint16_t)i));return 2*count;
}
static bool esp_send(uint8_t type,uint16_t sequence,const uint8_t *payload,unsigned n)
{
    uint8_t out[110];if(n>96)return false;out[0]=0xa5;out[1]=0x5a;out[2]=1;out[3]=type;bms_put16(out+4,sequence);bms_put16(out+6,(uint16_t)n);if(n)memcpy(out+8,payload,n);bms_put16(out+8+n,bms_crc16(out+2,n+6));return board_uart_send(2,out,n+10);
}
static void esp_process(uint32_t now)
{
    uint8_t reply[96],type=esp[3];unsigned n=bms_le16(esp+6),len=1;uint16_t sequence=bms_le16(esp+4);
    if(esp[2]!=1||bms_crc16(esp+2,n+6)!=bms_le16(esp+8+n))return;
    if(type==0xc0){
        if(alarm_inflight&&sequence==alarm_seq&&n==1&&esp[8]==0){
            alarm_faults&=~alarm_sent_bits;alarm_pending=alarm_faults!=0;alarm_inflight=false;
        }
        return;
    }
    if(type==0xb0){
        if(pm==PM_WAIT_READY&&sequence==sleep_seq&&n==1&&esp[8]==0&&
           bms_app_idle_allowed()&&esp_send(0x31,sleep_seq,0,0)){
            pm=PM_WAIT_DRAIN;pm_at=now;
        }
        return;/* READY is a response, do not send a response to it. */
    }
    reply[0]=0;
    if((type==1&&n==0)||(type==0x20&&n==1)){
        /* A timer-woken ESP is already running: do not reset it mid-HELLO. */
        if(pm==PM_ASLEEP){board_links_standby(false);pm=PM_ACTIVE;idle_at=now;}
        if(type==0x20){activity();sleep_capable=esp[8]==1;}
        heartbeat_seen=true;heartbeat_at=now;peer_ready=pm!=PM_WAKING;
    }
    else if(type==2&&n==0)len=status_payload(reply,22);
    else if(type==3&&n==0)len=status_payload(reply,28);
    else if(type==0x10&&n==2){activity();reply[0]=bms_app_command(bms_le16(esp+8));}
    else if(type==0x11&&n==4){activity();reply[0]=bms_app_config(bms_le16(esp+8),bms_le16(esp+10));}
    else if(type==0x12&&n==4){
        unsigned i,count=bms_le16(esp+10);uint16_t address=bms_le16(esp+8);activity();
        if(!count||count>32||(uint32_t)address+count>65536)reply[0]=1;
        for(i=0;!reply[0]&&i<count;i++)if(!register_valid((uint16_t)(address+i)))reply[0]=1;
        if(!reply[0]){for(i=0;i<count;i++)bms_put16(reply+1+i*2,bms_app_register((uint16_t)(address+i)));len=1+2*count;}
    }
    else reply[0]=1;
    esp_send((uint8_t)(type|0x80),sequence,reply,len);
}
static void esp_task(uint32_t now)
{
    uint8_t v;uint32_t t;unsigned budget=256;
    if(board_uart_overflow(2)){while(budget--&&board_uart_get(2,&v,&t)){}en=0;return;}
    while(budget--&&board_uart_get(2,&v,&t)){
        if(en&&t-etime>100)en=0;etime=t;
        if(!en){if(v==0xa5)esp[en++]=v;continue;}
        if(en==1){if(v==0x5a)esp[en++]=v;else en=v==0xa5?1:0;continue;}
        esp[en++]=v;
        if(en==8&&bms_le16(esp+6)>96){en=0;continue;}
        if(en>=10&&en==(unsigned)bms_le16(esp+6)+10){esp_process(now);en=0;}
    }
    if(en&&now-etime>100)en=0;
    if(pm==PM_ACTIVE&&now-telemetry_at>=1000){uint8_t p[44];telemetry_at=now;status_payload(p,22);esp_send(0x82,seq++,p,44);}
    else if(pm==PM_ACTIVE&&now-telemetry_at>=500&&now-extended_at>=1000){
        uint8_t p[56];if(esp_send(0x83,seq,p,status_payload(p,28))){seq++;extended_at=now;}
    }
    if(pm==PM_ACTIVE&&!resetting&&heartbeat_seen&&now-heartbeat_at>5000&&now-cooldown_at>30000){board_esp_reset(true);resetting=true;reset_at=now;cooldown_at=now;heartbeat_seen=false;peer_ready=false;}
    if(resetting&&now-reset_at>=20){board_esp_reset(false);resetting=false;}
}
static void can_receive_task(void)
{
    uint8_t p[8];unsigned budget=4;
    while(budget--&&board_can_receive(p)){
        uint8_t op=p[1],status=0;uint16_t a=bms_le16(p+4),v=0;
        if(p[0]!=1||p[3]||bms_crc16(p,6)!=bms_le16(p+6))continue;
        activity();
        if(op==1){if(register_valid(a))v=bms_app_register(a);else status=1;}
        else if(op==2){
            if(a>3)status=1; /* no remote erase or config save through CAN */
            else status=BMS_CAN_COMMANDS?bms_app_command(a):2;
        }else status=1;
        p[1]=(uint8_t)(op|0x80);p[3]=status;bms_put16(p+4,v);bms_put16(p+6,bms_crc16(p,6));
        (void)board_can_send(0x322,p);
    }
}
static void can_task(uint32_t now)
{
    uint8_t p[8];unsigned i;uint16_t id;
    if(pm!=PM_ACTIVE||now-can_at<100)return;can_at=now;memset(p,0,sizeof(p));
    if(can_phase==0){id=0x301;bms_put16(p,(uint16_t)bms.sample.pack_mv);bms_put16(p+2,(uint16_t)bms.sample.current_ma);bms_put16(p+4,(uint16_t)(bms.core.soc_pct*10));p[6]=(uint8_t)bms.core.state;p[7]=(uint8_t)bms_app_register(10);}
    else if(can_phase<3){id=(uint16_t)(0x301+can_phase);for(i=0;i<3;i++)bms_put16(p+2*i,bms.sample.cell_mv[(can_phase-1)*3+i]);bms_put16(p+6,(uint16_t)bms.sample.temp_dc[can_phase-1]);}
    else {id=0x304;bms_put32(p,bms.core.faults|bms.core.latched);bms_put16(p+4,(uint16_t)bms.sample.output_mv);p[6]=(uint8_t)bms_app_register(14);p[7]=bms.core.balance_mask;}
    if(board_can_send(id,p))can_phase=(uint8_t)((can_phase+1)%4);
}
static void power_task(uint32_t now)
{
    uint32_t faults=bms.core.faults|bms.core.latched;bool idle=bms_app_idle_allowed();
    if(faults!=last_faults){last_faults=faults;if(faults){alarm_faults|=faults;alarm_pending=true;activity();}}
    if(!idle){idle_at=now;if(pm==PM_WAIT_READY||pm==PM_WAIT_DRAIN||pm==PM_ASLEEP)activity();}
    if(pm==PM_WAKING&&now-pm_at>=20){board_esp_reset(false);pm=PM_ACTIVE;idle_at=now;}
    if(pm==PM_ACTIVE&&idle&&sleep_capable&&peer_ready&&!alarm_pending&&!resetting&&
       now-idle_at>=30000&&rn==0&&en==0&&board_links_idle()){
        sleep_seq=seq++;
        if(esp_send(0x30,sleep_seq,0,0)){pm=PM_WAIT_READY;pm_at=now;}
    }
    if(pm==PM_WAIT_READY&&now-pm_at>=2000){pm=PM_ACTIVE;idle_at=now;}
    if(pm==PM_WAIT_DRAIN){
        if(now-pm_at>=2000)activity();
        else if(idle&&rn==0&&en==0&&board_links_idle()){
            board_links_standby(true);pm=PM_ASLEEP;peer_ready=false;heartbeat_seen=false;
        }
    }
    if(alarm_pending&&peer_ready&&pm==PM_ACTIVE&&(!alarm_inflight||now-alarm_at>=2000)){
        uint8_t p[4];uint16_t sequence=alarm_inflight?alarm_seq:seq++;
        uint32_t bits=alarm_inflight?alarm_sent_bits:alarm_faults;bms_put32(p,bits);
        if(esp_send(0x40,sequence,p,4)){alarm_inflight=true;alarm_seq=sequence;alarm_sent_bits=bits;alarm_at=now;}
    }
}
void bms_comms_task(void){uint32_t now=board_ms();power_task(now);can_receive_task();modbus_task(now);esp_task(now);power_task(now);can_task(now);}
