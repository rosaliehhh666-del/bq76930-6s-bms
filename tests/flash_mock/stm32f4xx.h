#ifndef TEST_FLASH_SPL_H
#define TEST_FLASH_SPL_H
#include <stdint.h>
typedef enum {FLASH_COMPLETE,FLASH_ERROR} FLASH_Status;
#define FLASH_Sector_9 9u
#define FLASH_Sector_10 10u
#define FLASH_Sector_11 11u
#define VoltageRange_3 3u
#define FLASH_FLAG_EOP 1u
#define FLASH_FLAG_OPERR 2u
#define FLASH_FLAG_WRPERR 4u
#define FLASH_FLAG_PGAERR 8u
#define FLASH_FLAG_PGPERR 16u
#define FLASH_FLAG_PGSERR 32u
void FLASH_Unlock(void);
void FLASH_Lock(void);
void FLASH_ClearFlag(uint32_t flags);
FLASH_Status FLASH_EraseSector(uint32_t sector,uint8_t voltage);
FLASH_Status FLASH_ProgramWord(uint32_t address,uint32_t value);
#endif
