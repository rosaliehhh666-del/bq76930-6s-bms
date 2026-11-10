#include "bms_display.h"
#include "bms_app.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static unsigned checks,draws;static uint32_t now;static bool key,standby,light,panel=true,touched;static uint16_t touch_x,touch_y;
static uint16_t ink[15];
static char screen[15][41];bms_app_t bms;
#define CHECK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL display line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
void bms_touch_init(void){}
static unsigned calibration_calls;
void bms_touch_calibration_start(void){calibration_calls++;}
bool bms_touch_calibration_task(void){return false;}
bool bms_touch_read(uint16_t *x,uint16_t *y){*x=touch_x;*y=touch_y;return touched;}
bool bms_touch_pressed(void){return touched;}
uint32_t board_ms(void){return now;}bool bms_comms_standby(void){return standby;}
uint16_t bms_app_register(uint16_t address){(void)address;return 1;}
bool bms_lcd_init(void){return panel;}bool bms_lcd_page_key(void){return key;}
void bms_lcd_backlight(bool on){light=on;}
void bms_lcd_line(unsigned row,const char *s,uint16_t color)
{ink[row]=color;CHECK(row<15&&strlen(s)<=40);strcpy(screen[row],s);draws++;}
static void frame(void)
{unsigned i;for(i=0;i<30;i++){now+=20;bms.core.last_sample_ms=bms.sample.current_time_ms=now;bms_display_task();}}
static void save(const char *name)
{FILE *f=fopen(name,"w");unsigned i;CHECK(f!=0);for(i=0;i<15;i++)fprintf(f,"%04X %s\n",ink[i],screen[i]);fclose(f);}
static void press(void)
{key=true;now+=1;bms_display_task();now+=31;bms_display_task();key=false;now++;bms_display_task();now+=31;bms_display_task();frame();}
int main(void)
{
    unsigned i;memset(&bms,0,sizeof(bms));bms.core.initialized=bms.sample.valid=bms.sample.current_valid=bms.sample.temps_valid=true;
    bms.core.cfg.ov_mv=4230;bms.core.cfg.uv_mv=3000;bms.core.soc_initialized=true;bms.core.soc_pct=75.5f;bms.core.state=BMS_NORMAL;bms.core.chg=bms.core.dis=true;
    bms.sample.pack_mv=22800;bms.sample.current_ma=-1000;bms.sample.temp_dc[0]=250;bms.sample.temp_dc[1]=260;bms.sample.board_temp_dc=300;
    for(i=0;i<6;i++)bms.sample.cell_mv[i]=(uint16_t)(3800+i*2);
    bms_display_init();CHECK(calibration_calls==1);frame();CHECK(strstr(screen[0],"OVERVIEW")!=0);CHECK(strstr(screen[2],"75.5")!=0);
    CHECK(strstr(screen[6],"22.800")!=0);CHECK(strstr(screen[6],"-1.000")!=0);CHECK(strstr(screen[7],"-22.8")!=0);
    {unsigned d=draws;frame();CHECK(draws==d);}/* unchanged rows do not flash or rewrite GRAM */
    save("build/tests/lcd_overview.txt");
    press();CHECK(strstr(screen[0],"CELLS")!=0);CHECK(strstr(screen[2],"3800")!=0);CHECK(strstr(screen[10],"10 mV")!=0);
    save("build/tests/lcd_cells.txt");
    touched=true;touch_x=20;touch_y=220;frame();CHECK(strstr(screen[0],"OVERVIEW")!=0);
    frame();CHECK(strstr(screen[0],"OVERVIEW")!=0);/* held touch does not cycle */
    touched=false;frame();touched=true;touch_x=160;frame();CHECK(strstr(screen[0],"CELLS")!=0);touched=false;frame();
    touched=true;touch_x=160;touch_y=120;frame();CHECK(strstr(screen[0],"CELLS")!=0);
    frame();CHECK(strstr(screen[0],"CELLS")!=0);touched=false;frame();
    touched=true;touch_x=280;touch_y=220;frame();CHECK(strstr(screen[0],"FAULTS")!=0);
    frame();CHECK(strstr(screen[0],"FAULTS")!=0);touched=false;frame();
    touched=true;touch_x=25;touch_y=220;frame();CHECK(strstr(screen[0],"OVERVIEW")!=0);touched=false;frame();
    touched=true;touch_x=319;touch_y=207;frame();CHECK(strstr(screen[0],"OVERVIEW")!=0);touched=false;frame();
    bms.core.latched=BMS_F_HW_SCD;bms.core.state=BMS_FAULT;bms.core.chg=bms.core.dis=false;frame();CHECK(strstr(screen[0],"FAULTS")!=0);CHECK(strstr(screen[2],"SHORT CIRCUIT")!=0);
    bms.core.snapshot.valid=bms.core.snapshot.has_sample=true;bms.core.snapshot.time_ms=100;bms.core.snapshot.sample=bms.sample;bms.core.snapshot.faults=BMS_F_HW_SCD;bms.core.snapshot.sample.cell_mv[0]=4100;frame();CHECK(strstr(screen[6],"not live")!=0);CHECK(strstr(screen[8],"4100")!=0);
    frame();
    save("build/tests/lcd_faults.txt");
    key=true;now++;bms_display_task();now+=31;bms_display_task();now+=1001;bms_display_task();key=false;now++;bms_display_task();now+=31;bms_display_task();frame();CHECK(strstr(screen[0],"OVERVIEW")!=0);
    bms.sample.valid=false;frame();CHECK(strstr(screen[7],"INVALID")!=0);CHECK(strstr(screen[2],"--")!=0);
    {unsigned d=draws;bms.core.precharge=true;frame();CHECK(draws==d);bms.core.precharge=false;}
    bms.core.latched=0;standby=true;now+=31000;bms_display_task();CHECK(!light);
    key=true;now++;bms_display_task();CHECK(light);key=false;
    key=false;now+=40;bms_display_task();key=true;now++;bms_display_task();now+=31;bms_display_task();now+=3001;bms_display_task();CHECK(calibration_calls==2);
    now+=100;bms_display_task();CHECK(calibration_calls==2);key=false;now++;bms_display_task();now+=31;bms_display_task();frame();CHECK(strstr(screen[0],"OVERVIEW")!=0);
    panel=false;bms_display_init();{unsigned d=draws;frame();CHECK(draws==d);}
    printf("PASS: %u checks (LCD pages, signs, stale data, fault priority, precharge, backlight, absent panel)\n",checks);return 0;
}
