#include "bms_storage.h"
#include "bms_util.h"
#include "bms_board.h"
#include "stm32f4xx.h"
#include <string.h>
#include <stddef.h>
/* Only STM32F407ZG 1 MiB: linker reserves sector 9 for faults and 10/11 for configuration. */
#define MAGIC 0x424d5331u
#define COMMIT 0x43464731u
typedef struct {uint32_t magic,version,sequence; bms_config_t cfg;uint32_t crc,commit;} record_t;
static const uint32_t address[2]={0x080c0000u,0x080e0000u};
static const uint32_t sector[2]={FLASH_Sector_10,FLASH_Sector_11};
static bool valid(const record_t *r)
{return r->magic==MAGIC&&r->version==BMS_CONFIG_VERSION&&r->commit==COMMIT&&r->crc==bms_crc32(r,offsetof(record_t,crc))&&bms_config_valid(&r->cfg);}
static int latest(void)
{const record_t *a=(const record_t*)(uintptr_t)address[0],*b=(const record_t*)(uintptr_t)address[1];if(!valid(a))return valid(b)?1:-1;if(!valid(b))return 0;return (int32_t)(b->sequence-a->sequence)>0?1:0;}
bool bms_storage_load(bms_config_t *c)
{int i;if(board_flash_kb()<1024)return false;i=latest();if(i<0)return false;*c=((const record_t*)(uintptr_t)address[i])->cfg;return true;}
bool bms_storage_save(const bms_config_t *c)
{
    int old,next;record_t r;unsigned offset;uint32_t v;bool ok=true;
    if(board_flash_kb()<1024)return false;old=latest();next=old==0?1:0;
    if(!bms_config_valid(c))return false;
    memset(&r,0,sizeof(r));r.magic=MAGIC;r.version=BMS_CONFIG_VERSION;r.sequence=old<0?1:((const record_t*)(uintptr_t)address[old])->sequence+1;r.cfg=*c;r.crc=bms_crc32(&r,offsetof(record_t,crc));r.commit=COMMIT;
    board_watchdog_feed();FLASH_Unlock();FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_OPERR|FLASH_FLAG_WRPERR|FLASH_FLAG_PGAERR|FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
    if(FLASH_EraseSector(sector[next],VoltageRange_3)!=FLASH_COMPLETE)ok=false;
    for(offset=0;ok&&offset<sizeof(r);offset+=4){memcpy(&v,(const uint8_t*)&r+offset,4);if(FLASH_ProgramWord(address[next]+offset,v)!=FLASH_COMPLETE)ok=false;}
    FLASH_Lock();board_watchdog_feed();return ok&&valid((const record_t*)(uintptr_t)address[next]);
}

/* Sector 9 is append-only during faults; NEVER erase automatically. */
#define LOG_BASE 0x080a0000u
#define LOG_BYTES 0x20000u
#define LOG_SLOT 128u
#define LOG_MAGIC 0x464c5431u
typedef struct {uint32_t magic,version,sequence;bms_fault_snapshot_t snapshot;uint32_t crc,commit;} fault_record_t;
typedef char log_record_must_fit[(sizeof(fault_record_t)<=LOG_SLOT&&sizeof(fault_record_t)%4==0)?1:-1];
static bool log_valid(const fault_record_t *r)
{return r->magic==LOG_MAGIC&&r->version==1&&r->commit==COMMIT&&r->snapshot.valid&&r->crc==bms_crc32(r,offsetof(fault_record_t,crc));}
static const fault_record_t *log_latest(void)
{
    uint32_t offset;const fault_record_t *best=0,*r;
    for(offset=0;offset<LOG_BYTES;offset+=LOG_SLOT){r=(const fault_record_t*)(uintptr_t)(LOG_BASE+offset);
        if(log_valid(r)&&(!best||(int32_t)(r->sequence-best->sequence)>0))best=r;}
    return best;
}
bool bms_fault_log_load(bms_fault_snapshot_t *snapshot)
{const fault_record_t *r;if(board_flash_kb()<1024)return false;r=log_latest();if(!r)return false;*snapshot=r->snapshot;return true;}
bool bms_fault_log_append(const bms_fault_snapshot_t *snapshot)
{
    const fault_record_t *old;fault_record_t r;uint32_t offset,i,slot=0,v;bool erased,ok=true;
    if(board_flash_kb()<1024||!snapshot->valid)return false;
    for(offset=0;offset<LOG_BYTES;offset+=LOG_SLOT){
        erased=true;for(i=0;i<LOG_SLOT;i+=4)if(*(const uint32_t*)(uintptr_t)(LOG_BASE+offset+i)!=0xffffffffu){erased=false;break;}
        if(erased){slot=LOG_BASE+offset;break;}
    }
    if(!slot)return false;old=log_latest();memset(&r,0,sizeof(r));
    r.magic=LOG_MAGIC;r.version=1;r.sequence=old?old->sequence+1:1;r.snapshot=*snapshot;
    r.crc=bms_crc32(&r,offsetof(fault_record_t,crc));r.commit=COMMIT;
    board_watchdog_feed();FLASH_Unlock();FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_OPERR|FLASH_FLAG_WRPERR|FLASH_FLAG_PGAERR|FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
    for(i=0;ok&&i<sizeof(r);i+=4){memcpy(&v,(const uint8_t*)&r+i,4);if(FLASH_ProgramWord(slot+i,v)!=FLASH_COMPLETE)ok=false;}
    FLASH_Lock();board_watchdog_feed();return ok&&log_valid((const fault_record_t*)(uintptr_t)slot);
}
bool bms_fault_log_erase(void)
{
    if(board_flash_kb()<1024)return false;
    FLASH_Status status;board_watchdog_feed();FLASH_Unlock();FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_OPERR|FLASH_FLAG_WRPERR|FLASH_FLAG_PGAERR|FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
    status=FLASH_EraseSector(FLASH_Sector_9,VoltageRange_3);FLASH_Lock();board_watchdog_feed();return status==FLASH_COMPLETE;
}
