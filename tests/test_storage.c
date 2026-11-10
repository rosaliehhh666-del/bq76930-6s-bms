#include "bms_storage.h"
#include "stm32f4xx.h"
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static unsigned checks,erases,writes;static int remaining=-1;static bool unlocked;
static uint16_t flash_kb=1024;
uint16_t board_flash_kb(void){return flash_kb;}
#define CHECK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL storage line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
void board_watchdog_feed(void){}
void FLASH_Unlock(void){unlocked=true;}void FLASH_Lock(void){unlocked=false;}
void FLASH_ClearFlag(uint32_t f){(void)f;}
FLASH_Status FLASH_EraseSector(uint32_t s,uint8_t v)
{(void)v;if(!unlocked||s<9||s>11)return FLASH_ERROR;memset((void*)(uintptr_t)(0x080a0000u+(s-9)*0x20000),0xff,0x20000);erases++;return FLASH_COMPLETE;}
FLASH_Status FLASH_ProgramWord(uint32_t a,uint32_t value)
{uint32_t *p=(uint32_t*)(uintptr_t)a;if(!unlocked||a<0x080a0000u||a>=0x08100000u||remaining==0)return FLASH_ERROR;if(remaining>0)remaining--;if((*p&value)!=value)return FLASH_ERROR;*p=value;writes++;return FLASH_COMPLETE;}
int main(void)
{
    bms_fault_snapshot_t s,out;bms_config_t c,loaded;unsigned i,before;
    void *flash=VirtualAlloc((void*)(uintptr_t)0x080a0000u,0x60000,MEM_RESERVE|MEM_COMMIT,PAGE_READWRITE);
    CHECK(flash==(void*)(uintptr_t)0x080a0000u);memset(flash,0xff,0x60000);memset(&s,0,sizeof(s));
    bms_config_defaults(&c);flash_kb=512;CHECK(!bms_storage_load(&c)&&!bms_storage_save(&c));CHECK(!bms_fault_log_load(&out)&&!bms_fault_log_append(&s)&&!bms_fault_log_erase());CHECK(erases==0&&writes==0);flash_kb=1024;
    CHECK(!bms_fault_log_load(&out));s.valid=s.has_sample=true;s.time_ms=100;s.faults=BMS_F_HW_SCD;s.sample.cell_mv[0]=3800;
    CHECK(bms_fault_log_append(&s));CHECK(erases==0);CHECK(bms_fault_log_load(&out)&&out.time_ms==100&&out.sample.cell_mv[0]==3800);
    /* Power loss before every programmed word/commit must leave old evidence readable. */
    for(i=0;i<21;i++){remaining=(int)i;s.time_ms=200+i;CHECK(!bms_fault_log_append(&s));CHECK(bms_fault_log_load(&out)&&out.time_ms==100);}
    remaining=-1;s.time_ms=300;CHECK(bms_fault_log_append(&s));CHECK(bms_fault_log_load(&out)&&out.time_ms==300);CHECK(erases==0&&!unlocked);
    {uint32_t *last=0;for(i=0;i<0x20000;i+=128){uint32_t *p=(uint32_t*)((uint8_t*)flash+i);if(p[0]==0x464c5431u&&p[2]==2)last=p;}
     CHECK(last!=0);last[3]^=0x10000u;CHECK(bms_fault_log_load(&out)&&out.time_ms==100);last[3]^=0x10000u;CHECK(bms_fault_log_load(&out)&&out.time_ms==300);}
    bms_config_defaults(&c);CHECK(bms_storage_save(&c));CHECK(bms_storage_load(&loaded)&&loaded.capacity_mah==3000);
    c.capacity_mah=3500;remaining=2;CHECK(!bms_storage_save(&c));remaining=-1;CHECK(bms_storage_load(&loaded)&&loaded.capacity_mah==3000);
    CHECK(bms_fault_log_load(&out)&&out.time_ms==300);/* config erases cannot touch logs */
    before=erases;CHECK(bms_fault_log_erase());CHECK(erases==before+1&&!bms_fault_log_load(&out));
    memset(flash,0,0x20000);before=writes;CHECK(!bms_fault_log_append(&s));CHECK(writes==before);/* full: never auto erase */
    CHECK(bms_storage_load(&loaded)&&loaded.capacity_mah==3000);
    VirtualFree(flash,0,MEM_RELEASE);printf("PASS: %u checks (fault Flash journal, interrupted writes, full log, config isolation)\n",checks);return 0;
}
