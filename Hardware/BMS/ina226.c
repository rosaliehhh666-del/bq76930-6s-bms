#include "ina226.h"
#include "bms_board.h"
#include "bms_config.h"
/* BQ76930 (0x08) and INA226 (0x40) share PB6/PB7 on the actual board. */
static bool rd(uint8_t reg,uint16_t *v){uint8_t p[2];if(!board_i2c(0,0x40,&reg,1,p,2))return false;*v=(uint16_t)((p[0]<<8)|p[1]);return true;}
bool ina226_init(void)
{uint16_t m,d,c;uint8_t tx[3]={0,0x45,0x27};return rd(0xfe,&m)&&rd(0xff,&d)&&m==0x5449&&(d&0xfff0)==0x2260&&board_i2c(0,0x40,tx,3,0,0)&&rd(0,&c)&&c==0x4527;}
bool ina226_read(ina226_sample_t *s,uint32_t now)
{uint16_t v,i;s->valid=false;if(!rd(2,&v)||!rd(1,&i))return false;s->bus_mv=(uint32_t)v*5/4;s->current_ma=(int32_t)((int64_t)(int16_t)i*2500/BMS_INA_SHUNT_UOHM);s->time_ms=now;s->valid=true;return true;}
