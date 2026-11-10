#include "bms_util.h"
#include <math.h>
uint8_t bms_crc8(const uint8_t *p,size_t n)
{
    uint8_t c=0;unsigned i;
    while(n--){c^=*p++;for(i=0;i<8;i++)c=(uint8_t)((c&0x80)?(c<<1)^0x07:c<<1);}
    return c;
}
uint16_t bms_crc16(const uint8_t *p,size_t n)
{
    uint16_t c=0xffff;unsigned i;
    while(n--){c^=*p++;for(i=0;i<8;i++)c=(uint16_t)((c&1)?(c>>1)^0xa001:c>>1);}
    return c;
}
uint32_t bms_crc32(const void *p,size_t n)
{
    const uint8_t *b=(const uint8_t*)p;uint32_t c=0xffffffffu;unsigned i;
    while(n--){c^=*b++;for(i=0;i<8;i++)c=(c>>1)^((c&1)?0xedb88320u:0);}
    return ~c;
}
bool bms_ntc_dc(uint32_t mv,uint32_t supply,float rpull,float r25,float beta,int16_t *out)
{
    float r,k,t;
    if(!out || supply<1000 || mv<50 || mv+50>=supply || beta<=0)return false;
    r=rpull*(float)mv/(float)(supply-mv);
    k=1.0f/(1.0f/298.15f+logf(r/r25)/beta);t=(k-273.15f)*10.0f;
    if(!isfinite(t)||t< -400||t>1250)return false;
    *out=(int16_t)(t>=0?t+0.5f:t-0.5f);return true;
}
uint16_t bms_le16(const uint8_t *p){return (uint16_t)(p[0]|((uint16_t)p[1]<<8));}
void bms_put16(uint8_t *p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
void bms_put32(uint8_t *p,uint32_t v){bms_put16(p,(uint16_t)v);bms_put16(p+2,(uint16_t)(v>>16));}
