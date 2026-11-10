#include "bq76930.h"
#include "bms_board.h"
#include "bms_util.h"
#include <string.h>
#define ADDRESS 0x08u
static const uint8_t channel[6]={0,1,4,5,6,9};
static bool read_reg(bq76930_t *d,uint8_t reg,uint8_t *out,unsigned count)
{
    uint8_t buf[40],check[2];unsigned i;if(count>20)return false;
    if(!board_i2c(0,ADDRESS,&reg,1,buf,d->crc?2*count:count))return false;
    if(!d->crc){memcpy(out,buf,count);return true;}
    for(i=0;i<count;i++){
        check[0]=(uint8_t)((ADDRESS<<1)|1);check[1]=buf[2*i];
        if(buf[2*i+1]!=(i?bms_crc8(check+1,1):bms_crc8(check,2)))return false;
    }
    for(i=0;i<count;i++)out[i]=buf[2*i];return true;
}
static bool write_reg(bq76930_t *d,uint8_t reg,uint8_t value)
{uint8_t tx[3]={reg,value,0},p[3]={(uint8_t)(ADDRESS<<1),reg,value};tx[2]=bms_crc8(p,3);return board_i2c(0,ADDRESS,tx,d->crc?3:2,0,0);}
static bool checked(bq76930_t *d,uint8_t reg,uint8_t v,uint8_t mask)
{uint8_t r;return write_reg(d,reg,v)&&read_reg(d,reg,&r,1)&&((r&mask)==(v&mask));}
bool bq_outputs(bq76930_t *d,bool chg,bool dis)
{
    uint8_t v=(uint8_t)(0x40|(chg?1:0)|(dis?2:0));
    if(!checked(d,5,v,0x7f)){(void)write_reg(d,5,0x40);d->outputs=0;return false;}d->outputs=v&3;return true;
}
bool bq_balance(bq76930_t *d,uint8_t mask)
{
    unsigned i,j;uint8_t bank[2]={0,0};if(mask&0xc0)return false;
    for(i=0;i<6;i++)if(mask&(1u<<i)){
        for(j=0;j<i;j++)if((mask&(1u<<j))&&((i==j+1)||(channel[i]/5==channel[j]/5&&channel[i]==channel[j]+1)))return false;
        bank[channel[i]/5]|=(uint8_t)(1u<<(channel[i]%5));
    }
    if(!checked(d,1,bank[0],0x1f)||!checked(d,2,bank[1],0x1f)){(void)write_reg(d,1,0);(void)write_reg(d,2,0);d->balance=0;return false;}d->balance=mask;return true;
}
bool bq_init(bq76930_t *d,const bms_config_t *cfg)
{
    uint8_t a,b,o,r;uint32_t raw;uint8_t ov,uv;
    memset(d,0,sizeof(*d));d->shunt_uohm=cfg->shunt_uohm;
    /* BQ7693003DBT is CRC-enabled. Never downgrade after a bad CRC. */
    d->crc=true;
    if(!read_reg(d,0x50,&a,1)||!read_reg(d,0x51,&o,1)||!read_reg(d,0x59,&b,1))return false;
    d->gain_uv=(uint16_t)(365+((a&0x0c)<<1)+((b&0xe0)>>5));d->offset_mv=(int8_t)o;
    {bool a=bq_outputs(d,false,false),b=bq_balance(d,0);if(!a||!b)return false;}
    /* Board profile: 4 mOhm, OCD=11 A/160 ms, SCD=22.25 A/100 us.
       A different shunt requires a reviewed threshold table, never silently reuse. */
    if(cfg->shunt_uohm!=4000)return false;
    raw=(uint32_t)(4250-d->offset_mv)*1000u/d->gain_uv;
    if(raw<0x2008||raw>0x2ff8)return false;
    ov=(uint8_t)((raw-8)>>4); /* round down, including fixed OV low nibble 8 */
    raw=(uint32_t)(2900-d->offset_mv)*1000u/d->gain_uv;
    if(raw<0x1000||raw>0x1ff0)return false;
    uv=(uint8_t)((raw+15)>>4); /* round up for conservative UV */
    if(!checked(d,6,0x8a,0x9f)||!checked(d,7,0x45,0x7f)||!checked(d,8,0,0xff)||
       !checked(d,9,ov,0xff)||!checked(d,10,uv,0xff)||!checked(d,11,0x19,0x3f)||
       !checked(d,4,0x18,0x1b)||!read_reg(d,0,&r,1))return false;
    /* Do not silently clear stored hardware trips on reset. Host recovery is explicit. */
    d->protection[0]=0x8a;d->protection[1]=0x45;d->protection[2]=0;
    d->protection[3]=ov;d->protection[4]=uv;d->protection[5]=0x19;
    d->ready=true;return true;
}
bool bq_sample(bq76930_t *d,bms_sample_t *out,uint32_t now)
{
    bms_sample_t s;uint8_t cells[20],ts[4],cc[2],stat;unsigned i;uint32_t raw,mv;bool valid=true;
    memset(&s,0,sizeof(s));
    if(!d->ready||!read_reg(d,0,&stat,1)||!read_reg(d,0x0c,cells,20)||!read_reg(d,0x2c,ts,4))return false;
    for(i=0;i<6;i++){raw=((uint32_t)(cells[channel[i]*2]&0x3f)<<8)|cells[channel[i]*2+1];mv=raw*d->gain_uv/1000u;s.cell_mv[i]=(uint16_t)((int32_t)mv+d->offset_mv);s.pack_mv+=s.cell_mv[i];}
    for(i=0;i<2;i++){s.temp_dc[i]=(int16_t)0x8000;raw=((uint32_t)(ts[2*i]&0x3f)<<8)|ts[2*i+1];mv=raw*382u/1000u;if(!bms_ntc_dc(mv,3300,10000,10000,BMS_AFE_NTC_BETA,&s.temp_dc[i]))valid=false;}
    if(stat&0x80){
        if(!read_reg(d,0x32,cc,2)||!write_reg(d,0,0x80))return false;
        d->current_ma=(int32_t)((int64_t)(int16_t)(((uint16_t)cc[0]<<8)|cc[1])*8440*BMS_CURRENT_SIGN/(int32_t)d->shunt_uohm);
        d->current_ms=now;d->has_current=true;
    }
    s.current_ma=d->current_ma;s.current_time_ms=d->current_ms;s.current_valid=d->has_current&&now-d->current_ms<=BMS_STALE_MS;
    s.hw_status=stat&0x3f;s.temps_valid=valid;s.valid=true;*out=s;return true;
}
bool bq_check(bq76930_t *d)
{
    uint8_t stat,c1,c2,b1,b2,status[3],control[8],disabled=0,expected1=0,expected2=0;unsigned i;
    for(i=0;i<6;i++)if(d->balance&(1u<<i)){if(channel[i]<5)expected1|=(uint8_t)(1u<<channel[i]);else expected2|=(uint8_t)(1u<<(channel[i]-5));}
    /* Two CRC-checked bursts; skip reserved register 0x03. */
    if(!read_reg(d,0,status,3)||!read_reg(d,4,control,8))return false;
    {static const uint8_t masks[6]={0x9f,0x7f,0xff,0xff,0xff,0x3f};
        for(i=0;i<6;i++)if((control[i+2]&masks[i])!=(d->protection[i]&masks[i]))return false;}
    stat=status[0];b1=status[1];b2=status[2];c1=control[0];c2=control[1];
    if(stat&4)disabled|=1;/* OV clears CHG_ON. */
    if(stat&0x0b)disabled|=2;/* UV/OCD/SCD clear DSG_ON. */
    if(stat&0x30)disabled=3;
    if((c1&0x1b)!=0x18||(c2&0x7c)!=0x40)return false;
    /* Accept only hardware-caused falling bits, never unexpected enabling. */
    if((c2&3)!=d->outputs&&(c2&3)!=(d->outputs&~disabled))return false;
    if(stat&0x20){if(b1||b2)return false;d->balance=0;}
    else if((b1&0x1f)!=expected1||(b2&0x1f)!=expected2)return false;
    d->outputs=c2&3;return true;
}
bool bq_status(bq76930_t *d,uint8_t *status)
{return d->ready&&read_reg(d,0,status,1);}
bool bq_clear_faults(bq76930_t *d,uint8_t allowed)
{uint8_t r;if(d->outputs||!read_reg(d,0,&r,1))return false;return write_reg(d,0,(uint8_t)(r&allowed&0x3f));}
bool bq_ship(bq76930_t *d)
{bool a=bq_outputs(d,false,false),b=bq_balance(d,0);if(!a||!b)return false;if(!write_reg(d,4,0)||!write_reg(d,4,1)||!write_reg(d,4,2))return false;d->ready=false;return true;}
