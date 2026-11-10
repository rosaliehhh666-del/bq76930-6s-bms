#include "bms_app.h"
#include "bms_util.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static uint8_t can_input[8],can_reply[8];static bool can_pending;
static unsigned checks,commands;static uint16_t last_command;static uint32_t now;
static bool idle_allowed,links_idle=true,links_sleep;
bool bms_app_idle_allowed(void){return idle_allowed;}
bool board_links_idle(void){return links_idle;}
void board_links_standby(bool on){links_sleep=on;}
#define CHECK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL comms line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
bms_app_t bms;
static uint8_t input[3][512],output[3][256];static unsigned length[3],position[3],outlen[3];static uint32_t stamps[3][512];static bool overflow[3],esp_reset;static uint16_t can_id;
uint32_t board_ms(void){return now;}
bool board_uart_get(unsigned p,uint8_t *v,uint32_t *t){if(position[p]>=length[p])return false;*v=input[p][position[p]];*t=stamps[p][position[p]++];return true;}
bool board_uart_overflow(unsigned p){bool x=overflow[p];overflow[p]=false;return x;}
bool board_uart_send(unsigned p,const uint8_t *v,unsigned n){memcpy(output[p],v,n);outlen[p]=n;return true;}
void board_esp_reset(bool reset){esp_reset=reset;}
bool board_can_send(uint16_t id,const uint8_t data[8]){if(id==0x322)memcpy(can_reply,data,8);can_id=id;return true;}

bool board_can_receive(uint8_t p[8]){if(!can_pending)return false;can_pending=false;memcpy(p,can_input,8);return true;}
uint8_t bms_app_command(uint16_t command){commands++;last_command=command;return command>4?1:0;}
uint8_t bms_app_config(uint16_t a,uint16_t v){return a==0x110&&v>=500?0:1;}
uint16_t bms_app_register(uint16_t a){return (uint16_t)(a+100);}
static void feed(unsigned port,const uint8_t *p,unsigned n,uint32_t time)
{unsigned i;length[port]=n;position[port]=0;memcpy(input[port],p,n);for(i=0;i<n;i++)stamps[port][i]=time;now=time;bms_comms_task();}
static void modbus(void)
{
    uint8_t p[9]={1,3,0,0,0,2,0,0,0};
    bms_put16(p+6,bms_crc16(p,6));feed(1,p,8,10);CHECK(!outlen[1]);now=12;bms_comms_task();CHECK(outlen[1]==9&&output[1][1]==3&&output[1][2]==4&&output[1][4]==100&&output[1][6]==101);CHECK(bms_crc16(output[1],7)==bms_le16(output[1]+7));
    p[1]=6;p[2]=1;p[3]=0;p[4]=0;p[5]=1;bms_put16(p+6,bms_crc16(p,6));feed(1,p,8,20);now=22;bms_comms_task();CHECK(commands==1&&last_command==1&&outlen[1]==8);
    p[7]^=1;feed(1,p,8,30);now=32;bms_comms_task();CHECK(commands==1);
    p[0]=0;bms_put16(p+6,bms_crc16(p,6));feed(1,p,8,40);now=42;bms_comms_task();CHECK(commands==1);/* broadcast control deliberately disabled */
    p[0]=1;p[1]=3;p[2]=0xff;p[3]=0xff;p[4]=0;p[5]=2;bms_put16(p+6,bms_crc16(p,6));feed(1,p,8,50);now=52;bms_comms_task();CHECK(outlen[1]==5&&(output[1][1]&0x80));
    p[1]=6;p[2]=1;p[3]=0;p[4]=0;p[5]=0;bms_put16(p+6,bms_crc16(p,6));overflow[1]=true;feed(1,p,8,60);now=62;bms_comms_task();CHECK(commands==1);
}
static void esp_protocol(void)
{
    uint8_t p[16]={0xa5,0x5a,1,2,0x34,0x12,0,0,0,0};
    bms_put16(p+8,bms_crc16(p+2,6));feed(2,p,10,100);CHECK(outlen[2]==54&&output[2][3]==0x82&&bms_le16(output[2]+4)==0x1234);CHECK(bms_crc16(output[2]+2,50)==bms_le16(output[2]+52));
    p[3]=0x10;p[6]=2;p[8]=3;p[9]=0;bms_put16(p+10,bms_crc16(p+2,8));feed(2,p,12,200);CHECK(commands==2&&last_command==3&&output[2][8]==0);
    p[10]^=0x20;feed(2,p,12,210);CHECK(commands==2);
    p[6]=97;feed(2,p,8,220);CHECK(commands==2);
    p[3]=1;p[6]=0;bms_put16(p+8,bms_crc16(p+2,6));feed(2,p,10,230);now=31001;bms_comms_task();CHECK(esp_reset);now=31021;bms_comms_task();CHECK(!esp_reset);
    now=31121;bms_comms_task();CHECK(can_id>=0x301&&can_id<=0x304);
    p[3]=0x12;p[6]=4;p[8]=0x30;p[9]=0;p[10]=32;p[11]=0;bms_put16(p+12,bms_crc16(p+2,10));
    feed(2,p,14,31200);CHECK(outlen[2]==75&&output[2][3]==0x92&&output[2][8]==0&&bms_le16(output[2]+9)==148);
    CHECK(bms_crc16(output[2]+2,71)==bms_le16(output[2]+73));
    p[8]=0x4f;p[10]=2;bms_put16(p+12,bms_crc16(p+2,10));feed(2,p,14,31201);CHECK(outlen[2]==11&&output[2][8]==1);
}
static void pm_frame(uint8_t type,uint16_t sequence,uint8_t value,uint32_t t)
{uint8_t p[11]={0xa5,0x5a,1,0,0,0,1,0,0,0,0};p[3]=type;bms_put16(p+4,sequence);p[8]=value;bms_put16(p+9,bms_crc16(p+2,7));feed(2,p,11,t);}
static void standby(void)
{
    uint16_t sequence;idle_allowed=true;pm_frame(0x20,1,1,32000);
    now=62001;bms_comms_task();CHECK(output[2][3]==0x30);sequence=bms_le16(output[2]+4);
    pm_frame(0xb0,(uint16_t)(sequence+1),0,62002);CHECK(!bms_comms_standby());
    links_idle=false;pm_frame(0xb0,sequence,0,62003);CHECK(output[2][3]==0x31&&!bms_comms_standby());
    links_idle=true;now=62004;bms_comms_task();CHECK(bms_comms_standby()&&links_sleep);
    now=69000;bms_comms_task();CHECK(bms_comms_standby()&&!esp_reset);
    idle_allowed=false;bms.core.latched=BMS_F_HW;now=69001;bms_comms_task();CHECK(esp_reset&&!links_sleep&&!bms_comms_standby());
    now=69021;bms_comms_task();CHECK(!esp_reset);
    pm_frame(0x20,2,1,69022);CHECK(output[2][3]==0x40&&bms_le16(output[2]+8)==BMS_F_HW);
    sequence=bms_le16(output[2]+4);pm_frame(0xc0,sequence,0,69023);
    bms.core.latched=0;idle_allowed=true;now=99023;bms_comms_task();CHECK(output[2][3]==0x30);
    now=101024;bms_comms_task();CHECK(!bms_comms_standby());/* missing READY */
}
static void extensions(void)
{
    uint8_t p[10]={0xa5,0x5a,1,3,0x42,0,0,0,0,0};unsigned before=commands;
    bms_put16(p+8,bms_crc16(p+2,6));feed(2,p,10,101025);
    CHECK(outlen[2]==66&&output[2][3]==0x83&&bms_le16(output[2]+8+0x16*2)==122);
    CHECK(bms_crc16(output[2]+2,62)==bms_le16(output[2]+64));
    memset(can_input,0,8);can_input[0]=1;can_input[1]=1;can_input[2]=9;can_input[4]=0x18;
    bms_put16(can_input+6,bms_crc16(can_input,6));can_pending=true;now++;bms_comms_task();
    CHECK(can_reply[1]==0x81&&can_reply[2]==9&&!can_reply[3]&&bms_le16(can_reply+4)==124);
    can_input[1]=2;can_input[4]=0; bms_put16(can_input+6,bms_crc16(can_input,6));can_pending=true;now++;bms_comms_task();
    CHECK(commands==before+1&&last_command==0);
    can_input[4]=7;bms_put16(can_input+6,bms_crc16(can_input,6));can_pending=true;now++;bms_comms_task();CHECK(commands==before+1&&can_reply[3]==1);
    can_input[4]=1;can_pending=true;now++;bms_comms_task();CHECK(commands==before+1); /* bad CRC */
    can_input[1]=1;can_input[4]=0xff;can_input[5]=0xff;bms_put16(can_input+6,bms_crc16(can_input,6));
    can_pending=true;now++;bms_comms_task();CHECK(can_reply[3]==1&&commands==before+1);
    now=102025;bms_comms_task();CHECK(output[2][3]==0x82&&outlen[2]==54);
    now=102525;bms_comms_task();CHECK(output[2][3]==0x83&&outlen[2]==66);

}
int main(void){modbus();esp_protocol();standby();extensions();printf("PASS: %u checks (Modbus/ESP framing, standby handshake, timeout, fault wake, CAN)\n",checks);return 0;}
