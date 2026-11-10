#include "bms_display.h"
#include "bms_app.h"
#include "bms_board.h"
#include <stdio.h>
#include <string.h>
#define WHITE 0x18e3u
#define RED 0xc900u
#define YELLOW 0xa3c0u
#define CYAN 0x1b96u
static bool available,key_raw,key_down,key_held,force_frame,seen_fresh;
static unsigned page,row;
static bool touch_down,calibrating,calibration_started;
static uint32_t touch_at;
static uint32_t key_at,key_started,paint_at,frame_at,seen_faults,interaction_at;
static char lines[15][41];
static uint16_t colors[15];
static char painted[15][41];
static uint16_t painted_colors[15];
static const char *const states[]={"INIT","OFF","PRECHARGE","NORMAL","CHARGE ONLY","DISCHARGE ONLY","FAULT","SHUTDOWN"};
static bool fresh(uint32_t now)
{return bms.core.initialized&&bms.sample.valid&&bms.sample.current_valid&&now-bms.core.last_sample_ms<=BMS_STALE_MS&&now-bms.sample.current_time_ms<=BMS_STALE_MS;}
static void fixed(char *p,unsigned n,int32_t value,unsigned scale,unsigned decimals)
{uint32_t v=value<0?(uint32_t)(-(int64_t)value):(uint32_t)value;snprintf(p,n,"%s%lu.%0*lu",value<0?"-":"",(unsigned long)(v/scale),(int)decimals,(unsigned long)(v%scale));}
static const char *fault_name(uint32_t f)
{
    if(f&BMS_F_HW_SCD)return "SHORT CIRCUIT";if(f&BMS_F_HW_OCD)return "HARDWARE DISCHARGE OC";
    if(f&BMS_F_SPREAD_TRIP)return "SEVERE CELL IMBALANCE";
    if(f&BMS_F_T1)return "BATTERY T1 SENSOR";if(f&BMS_F_T2)return "BATTERY T2 SENSOR";
    if(f&BMS_F_BOARD_SENSOR)return "BOARD SENSOR / ADC";if(f&BMS_F_BOARD_OT)return "BOARD OVERTEMPERATURE";
    if(f&BMS_F_SENSOR)return "SENSOR / DATA INVALID";
    if(f&BMS_F_COMM)return "AFE COMMUNICATION";if(f&BMS_F_PRECHARGE)return "PRECHARGE FAILED";
    if(f&BMS_F_OUTPUT)return "OUTPUT MEASUREMENT / CONTROL";if(f&BMS_F_HW)return "AFE HARDWARE FAULT";
    if(f&BMS_F_OV)return "CELL OVERVOLTAGE";if(f&BMS_F_UV)return "CELL UNDERVOLTAGE";
    if(f&BMS_F_CHG_OC)return "CHARGE OVERCURRENT";if(f&BMS_F_DIS_OC)return "DISCHARGE OVERCURRENT";
    if(f&BMS_F_CHG_TEMP)return "CHARGE TEMPERATURE";if(f&BMS_F_DIS_TEMP)return "DISCHARGE TEMPERATURE";
    if(f&BMS_F_WIRING)return "HARDWARE NOT VERIFIED";if(f&BMS_F_SPREAD_WARN)return "CELL SPREAD WARNING";return "NONE";
}
static void build_frame(uint32_t now)
{
    char a[24],c[24],d[24];unsigned i;uint32_t f=bms.core.faults|bms.core.latched;bool valid=fresh(now);
    memset(lines,0,sizeof(lines));for(i=0;i<15;i++)colors[i]=WHITE;
    snprintf(lines[0],41," BMS / %s                     %u/3",page==0?"OVERVIEW":page==1?"CELLS":"FAULTS",page+1);colors[0]=CYAN;
    snprintf(lines[1],41,"State: %s",bms.core.state<=BMS_SHUTDOWN?states[bms.core.state]:"INVALID");
    if(page==1){
        uint16_t lo=65535,hi=0;
        for(i=0;i<6;i++){
            if(valid){
                if(bms.sample.cell_mv[i]<lo)lo=bms.sample.cell_mv[i];if(bms.sample.cell_mv[i]>hi)hi=bms.sample.cell_mv[i];}
            else snprintf(lines[2+i],41,"Cell %u: --  DATA INVALID",i+1);
        }
        if(valid){snprintf(lines[9],41,"Min:%u mV   Max:%u mV",lo,hi);snprintf(lines[10],41,"Spread: %u mV",(unsigned)(hi-lo));}
        else {snprintf(lines[9],41,"Min / Max / Spread: --");colors[9]=RED;}
    }else if(page==2){
        const bms_fault_snapshot_t *s=&bms.core.snapshot;
        snprintf(lines[2],41,"%s",fault_name(f));colors[2]=f?RED:WHITE;
        snprintf(lines[3],41,"Fault mask: %08lX",(unsigned long)f);
        snprintf(lines[4],41,"CHG:%s DSG:%s PRE:%s",bms.core.chg?"ON":"OFF",bms.core.dis?"ON":"OFF",bms.core.precharge?"ON":"OFF");
        if(s->valid){snprintf(lines[6],41,"FIRST FAULT SNAPSHOT (not live)");colors[6]=YELLOW;
            snprintf(lines[7],41,"Time:%lu ms Mask:%08lX",(unsigned long)s->time_ms,(unsigned long)s->faults);
            if(s->has_sample){snprintf(lines[8],41,"C1:%u C2:%u C3:%u mV",s->sample.cell_mv[0],s->sample.cell_mv[1],s->sample.cell_mv[2]);
                snprintf(lines[9],41,"C4:%u C5:%u C6:%u mV",s->sample.cell_mv[3],s->sample.cell_mv[4],s->sample.cell_mv[5]);
                snprintf(lines[10],41,"I:%ld mA T1:%d T2:%d (0.1C)",(long)s->sample.current_ma,s->sample.temp_dc[0],s->sample.temp_dc[1]);
                snprintf(lines[11],41,"Sample:%lu ms AFE:%02X",(unsigned long)s->sample_ms,s->sample.hw_status);
            }else snprintf(lines[8],41,"No valid pre-fault sample");
            snprintf(lines[12],41,"Flash: %s",(bms_app_register(0x18)&1)?"SAVED":"PENDING / NOT SAVED");
        }else snprintf(lines[6],41,"No fault snapshot recorded");
    }
    /* Compose each overview value once. */
    if(page==0){
        if(valid){
            if(bms.core.soc_initialized){unsigned soc=(unsigned)(bms.core.soc_pct*10);if(soc>1000)soc=1000;
                snprintf(lines[2],41,"SOC %u.%u%%",soc/10,soc%10);snprintf(lines[4],41,"LEVEL %u",soc/10);
            }else {snprintf(lines[2],41,"SOC --");snprintf(lines[4],41,"LEVEL --");}
            fixed(a,sizeof(a),(int32_t)bms.sample.pack_mv,1000,3);fixed(c,sizeof(c),bms.sample.current_ma,1000,3);
            snprintf(lines[6],41,"%-20s%18s",a,c);
            fixed(a,sizeof(a),(int32_t)((int64_t)bms.sample.pack_mv*bms.sample.current_ma/100000),10,1);snprintf(lines[7],41,"POWER %s W     %s",a,bms.sample.current_ma>50?"CHARGING":bms.sample.current_ma< -50?"DISCHARGING":"IDLE");
        }else {snprintf(lines[2],41,"SOC --");snprintf(lines[4],41,"LEVEL --");snprintf(lines[6],41,"--                                  --");snprintf(lines[7],41,"DATA INVALID / STALE");colors[7]=RED;}
        strcpy(lines[3],lines[2]);colors[2]=colors[3]=CYAN;
        snprintf(lines[5],41,"PACK VOLTAGE (V)            CURRENT (A)");colors[5]=CYAN;
        if(valid&&bms.sample.temps_valid){fixed(a,sizeof(a),bms.sample.temp_dc[0],10,1);fixed(c,sizeof(c),bms.sample.temp_dc[1],10,1);fixed(d,sizeof(d),bms.sample.board_temp_dc,10,1);
            snprintf(lines[8],41,"T1:%s  T2:%s C",a,c);snprintf(lines[9],41,"Board: %s C",d);
        }else snprintf(lines[8],41,"Temperature: -- (invalid)");
        snprintf(lines[10],41,"CHG:%s DSG:%s PRE:%s",bms.core.chg?"ON":"OFF",bms.core.dis?"ON":"OFF",bms.core.precharge?"ON":"OFF");
        snprintf(lines[12],41,"%s",f?fault_name(f):"NO ACTIVE ALARMS");colors[12]=f?RED:WHITE;
    }else if(page==1&&valid){
        uint16_t lo=65535,hi=0;
        for(i=0;i<6;i++){if(bms.sample.cell_mv[i]<lo)lo=bms.sample.cell_mv[i];if(bms.sample.cell_mv[i]>hi)hi=bms.sample.cell_mv[i];}
        for(i=0;i<6;i++){
            uint16_t v=bms.sample.cell_mv[i];snprintf(lines[2+i],41," C%u   %4u mV    %-3s   %s",i+1,v,v==lo&&lo!=hi?"MIN":v==hi&&lo!=hi?"MAX":"",(bms.core.balance_mask&(1u<<i))?"BAL ON":"");
            colors[2+i]=v>=bms.core.cfg.ov_mv||v<=bms.core.cfg.uv_mv?RED:v==hi&&lo!=hi?YELLOW:v==lo&&lo!=hi?CYAN:WHITE;
        }
        snprintf(lines[12],41,"%s",(f&(BMS_F_SPREAD_WARN|BMS_F_SPREAD_TRIP))?"CELL SPREAD ALARM":"CELL VOLTAGES LIVE");colors[12]=(f&(BMS_F_SPREAD_WARN|BMS_F_SPREAD_TRIP))?RED:WHITE;
    }else if(page==2&&bms.core.snapshot.valid){
        const bms_fault_snapshot_t *s=&bms.core.snapshot;
        if(s->has_sample&&s->sample.temps_valid&&s->sample.current_valid){
            fixed(a,sizeof(a),s->sample.temp_dc[0],10,1);fixed(c,sizeof(c),s->sample.temp_dc[1],10,1);
            snprintf(lines[10],41,"I:%ld mA   T:%s / %s C",(long)s->sample.current_ma,a,c);
        }else {snprintf(lines[10],41,"SNAPSHOT SENSOR DATA NOT VALID");colors[10]=RED;}
        snprintf(lines[13],41,"Read-only: fix cause before recovery");colors[13]=YELLOW;
    }
    if(page==0)snprintf(lines[11],41,"%s | ESP %s",BMS_HARDWARE_VERIFIED?"CONTROL ENABLED":"OUTPUTS LOCKED",bms_comms_standby()?"SLEEP":"ACTIVE");
    snprintf(lines[13],41,"    HOME          CELLS         ALERTS");lines[14][0]=0;colors[13]=colors[14]=CYAN;
}
void bms_display_init(void)
{available=bms_lcd_init();if(available)bms_touch_init();touch_down=calibrating=calibration_started=false;touch_at=board_ms();memset(painted,0xff,sizeof(painted));page=row=0;force_frame=true;seen_fresh=false;seen_faults=0;key_raw=key_down=key_held=false;interaction_at=board_ms();paint_at=board_ms()-20;frame_at=board_ms()-500;
    if(available){calibrating=true;bms_touch_calibration_start();}
}
void bms_display_task(void)
{
    uint32_t now=board_ms(),f=bms.core.faults|bms.core.latched;bool k;
    if(!available)return;
    k=bms_lcd_page_key();
    if(k!=key_raw){key_raw=k;key_at=now;interaction_at=now;}
    if(now-key_at>=30&&key_down!=k){key_down=k;if(k){key_started=now;key_held=false;calibration_started=false;}
        else if(!key_held){page=(page+1)%3;force_frame=true;}}
    if(key_down&&!key_held&&now-key_started>=1000){key_held=true;page=0;force_frame=true;}
    if(key_down&&!calibration_started&&now-key_started>=3000&&!bms.core.precharge&&!bms.core.pre_active){
        calibration_started=true;calibrating=true;bms_touch_calibration_start();
    }
    if(calibrating){
        bms_lcd_backlight(true);
        if(!bms_touch_calibration_task()){
            calibrating=false;touch_down=true;memset(painted,0xff,sizeof(painted));force_frame=true;
        }
        return; /* Only UI yields: the caller continues protection and communications. */
    }
    if(f&~seen_faults){page=2;force_frame=true;touch_down=true;}seen_faults=f;
    if(fresh(now)!=seen_fresh){seen_fresh=fresh(now);force_frame=true;}
    bms_lcd_backlight(!bms_comms_standby()||f||key_down||now-interaction_at<30000u);
    if(bms.core.precharge||bms.core.pre_active)return;
    if(now-touch_at>=30){uint16_t tx,ty;bool down;touch_at=now;down=bms_touch_read(&tx,&ty);
        if(down&&!touch_down&&tx<320&&ty<240){interaction_at=now;
            /* Content is read-only. Only the visible bottom navigation is
               actionable; never use a content tap as a next-page command. */
            if(ty>=208){unsigned target=tx<107?0:tx<214?1:2;
                if(page!=target){page=target;force_frame=true;}}
        }
        /* A failed ADC sample while the pen is still held is not a release. */
        if(down)touch_down=true;else if(!bms_touch_pressed())touch_down=false;}
    if(force_frame||(row==15&&now-frame_at>=500)){build_frame(now);
        if(memcmp(painted[0],lines[0],41)){memset(painted[13],0xff,41);memset(painted[14],0xff,41);}
        row=0;frame_at=now;force_frame=false;}
    if(row<15&&now-paint_at>=20){
        if(memcmp(painted[row],lines[row],41)||painted_colors[row]!=colors[row]){
            bms_lcd_line(row,lines[row],colors[row]);memcpy(painted[row],lines[row],41);painted_colors[row]=colors[row];
        }row++;paint_at=now;
    }
}
