/* State/SOC separation inspired by LibreSolar/bms-firmware (Apache-2.0).
 * OCV data derived from its generic NMC table; NOT a measured Samsung 30Q curve.
 * See THIRD_PARTY_NOTICES.md. */
#include "bms_core.h"
#include <string.h>
#include <math.h>
#include <limits.h>
static const uint16_t ocv[21]={4198,4135,4089,4056,4026,3993,3962,3924,3883,3858,3838,
 3819,3803,3787,3764,3745,3726,3702,3684,3588,2800};
static int32_t absi(int32_t x){return x<0?-x:x;}
float bms_soc_from_ocv(uint16_t mv)
{
    unsigned i;if(mv>=ocv[0])return 100;if(mv<=ocv[20])return 0;
    for(i=0;i<20;i++)if(mv>=ocv[i+1])return 100.0f-5.0f*i-
      5.0f*(ocv[i]-mv)/(float)(ocv[i]-ocv[i+1]);
    return 0;
}
void bms_core_init(bms_core_t *b,const bms_config_t *c,bool verified,uint32_t now)
{
    memset(b,0,sizeof(*b));b->cfg=*c;b->state=BMS_INIT;b->hardware_verified=verified;
    b->request_chg=true;b->request_dis=true;b->last_sample_ms=now;b->idle_since=now;
}
static void capture(bms_core_t *b,uint32_t reason,uint32_t now)
{
    reason&=~(BMS_F_WIRING|BMS_F_SPREAD_WARN);
    if(!reason||b->snapshot.valid)return;
    b->snapshot.valid=true;b->snapshot.has_sample=b->has_latest;
    b->snapshot.time_ms=now;b->snapshot.sample_ms=b->last_sample_ms;
    b->snapshot.faults=reason;b->snapshot.state=b->state;
    b->snapshot.outputs=(uint8_t)((b->chg?1:0)|(b->dis?2:0)|(b->precharge?4:0));
    b->snapshot.sample=b->latest;
}
void bms_core_fail(bms_core_t *b,uint32_t fault)
{
    if(fault&~b->latched)b->recovery_active=false;
    capture(b,fault,b->decision_ms);/* decision_ms tracks the latest scheduler time. */
    b->latched|=fault;b->chg=false;b->dis=false;b->precharge=false;b->pre_active=false;
    b->balance_mask=0;b->state=BMS_FAULT;
}
static uint32_t hardware_faults(uint8_t status)
{
    uint32_t f=0;
    if(status&4)f|=BMS_F_HW_OV|BMS_F_OV;
    if(status&8)f|=BMS_F_HW_UV|BMS_F_UV;
    if(status&1)f|=BMS_F_HW_OCD|BMS_F_HW;
    if(status&2)f|=BMS_F_HW_SCD|BMS_F_HW;
    if(status&0x30)f|=BMS_F_HW;
    return f;
}
void bms_core_hw_fault(bms_core_t *b,uint8_t status,uint32_t now)
{
    uint32_t f=hardware_faults(status);b->decision_ms=now;
    if(!f)return;
    if(f&~(b->faults|b->latched))b->recovery_active=false;
    b->faults|=f;b->latest.hw_status=status&0x3f;
    capture(b,f,now);
    if(f&BMS_FATAL_MASK)bms_core_fail(b,f);
    else {if(f&BMS_F_OV)b->chg=false;if(f&BMS_F_UV)b->dis=false;
        b->precharge=b->pre_active=false;b->balance_mask=0;}
}
static bool recovery_sample(const bms_core_t *b,const bms_sample_t *s,uint32_t now)
{
    unsigned i;
    if(!s->valid||!s->temps_valid||!s->current_valid||!s->output_valid||
       now-s->current_time_ms>BMS_STALE_MS||absi(s->current_ma)>=200||
       s->board_temp_dc>750||(s->hw_status&0x30))return false;
    for(i=0;i<BMS_CELLS;i++)if(s->cell_mv[i]<b->cfg.uv_recover_mv||s->cell_mv[i]>b->cfg.ov_recover_mv)return false;
    for(i=0;i<BMS_TS_COUNT;i++)if(s->temp_dc[i]<b->cfg.charge_min_dc+b->cfg.hysteresis_dc||
       s->temp_dc[i]>b->cfg.charge_max_dc-b->cfg.hysteresis_dc||
       s->temp_dc[i]<b->cfg.discharge_min_dc+b->cfg.hysteresis_dc||
       s->temp_dc[i]>b->cfg.discharge_max_dc-b->cfg.hysteresis_dc)return false;
    return true;
}
static uint32_t measure_faults(bms_core_t *b,const bms_sample_t *s,uint32_t now)
{
    uint32_t f=0;unsigned i;uint16_t lo=65535,hi=0;int16_t tl=INT16_MAX,th=INT16_MIN;
    for(i=0;i<BMS_CELLS;i++){if(s->cell_mv[i]<lo)lo=s->cell_mv[i];if(s->cell_mv[i]>hi)hi=s->cell_mv[i];}
    b->spread_mv=(uint16_t)(hi-lo);
    /* Do not count unobserved gaps as continuously abnormal/recovered. */
    if(b->has_latest&&now-b->last_sample_ms>BMS_STALE_MS){
        b->spread_warn_active=b->spread_trip_active=b->spread_recover_active=false;
        b->oc_chg_active=b->oc_dis_active=false;
    }
    /* Invalid current cannot confirm a continuous overcurrent interval. */
    if(!s->current_valid)b->oc_chg_active=b->oc_dis_active=false;
    if(b->spread_mv>=BMS_SPREAD_WARN_MV){
        if(!b->spread_warn_active){b->spread_warn_active=true;b->spread_warn_since=now;}
        if(now-b->spread_warn_since>=BMS_SPREAD_WARN_MS)f|=BMS_F_SPREAD_WARN;
    }else b->spread_warn_active=false;
    if((b->faults&BMS_F_SPREAD_WARN)&&b->spread_mv>BMS_SPREAD_WARN_RELEASE_MV)f|=BMS_F_SPREAD_WARN;
    if(b->spread_mv>=BMS_SPREAD_TRIP_MV){
        if(!b->spread_trip_active){b->spread_trip_active=true;b->spread_trip_since=now;}
        if(now-b->spread_trip_since>=BMS_SPREAD_TRIP_MS)f|=BMS_F_SPREAD_TRIP;
    }else b->spread_trip_active=false;
    if(b->spread_mv<=BMS_SPREAD_RECOVER_MV){
        if(!b->spread_recover_active){b->spread_recover_active=true;b->spread_recover_since=now;}
    }else b->spread_recover_active=false;
    if(!s->output_valid)f|=BMS_F_OUTPUT;
    if(lo<1800||hi>4500||s->pack_mv<10800||s->pack_mv>27000)f|=BMS_F_SENSOR;
    if(s->temp_dc[0]==INT16_MIN)f|=BMS_F_T1;
    if(s->temp_dc[1]==INT16_MIN)f|=BMS_F_T2;
    if(s->board_temp_dc==INT16_MIN)f|=BMS_F_BOARD_SENSOR;
    if(!s->temps_valid||!s->current_valid||(f&(BMS_F_T1|BMS_F_T2|BMS_F_BOARD_SENSOR)))f|=BMS_F_SENSOR;
    for(i=0;i<BMS_TS_COUNT;i++){if(s->temp_dc[i]<tl)tl=s->temp_dc[i];if(s->temp_dc[i]>th)th=s->temp_dc[i];}
    if(hi>=b->cfg.ov_mv || ((b->faults&BMS_F_OV)&&hi>b->cfg.ov_recover_mv))f|=BMS_F_OV;
    if(lo<=b->cfg.uv_mv || ((b->faults&BMS_F_UV)&&lo<b->cfg.uv_recover_mv))f|=BMS_F_UV;
    if(tl<b->cfg.charge_min_dc || th>b->cfg.charge_max_dc ||
      ((b->faults&BMS_F_CHG_TEMP)&&(tl<b->cfg.charge_min_dc+b->cfg.hysteresis_dc||th>b->cfg.charge_max_dc-b->cfg.hysteresis_dc)))f|=BMS_F_CHG_TEMP;
    if(tl<b->cfg.discharge_min_dc || th>b->cfg.discharge_max_dc ||
      ((b->faults&BMS_F_DIS_TEMP)&&(tl<b->cfg.discharge_min_dc+b->cfg.hysteresis_dc||th>b->cfg.discharge_max_dc-b->cfg.hysteresis_dc)))f|=BMS_F_DIS_TEMP;
    if(s->board_temp_dc>800||((b->faults&BMS_F_BOARD_OT)&&s->board_temp_dc>750))
        f|=BMS_F_BOARD_OT|BMS_F_CHG_TEMP|BMS_F_DIS_TEMP;
    if(s->current_valid&&s->current_ma>(int32_t)b->cfg.charge_ma){if(!b->oc_chg_active){b->oc_chg_active=true;b->oc_since_chg=now;}if(now-b->oc_since_chg>=500)f|=BMS_F_CHG_OC;}
    else b->oc_chg_active=false;
    if(s->current_valid&&s->current_ma<-(int32_t)b->cfg.discharge_ma){if(!b->oc_dis_active){b->oc_dis_active=true;b->oc_since_dis=now;}if(now-b->oc_since_dis>=200)f|=BMS_F_DIS_OC;}
    else b->oc_dis_active=false;
    f|=hardware_faults(s->hw_status);
    return f;
}
void bms_core_update(bms_core_t *b,const bms_sample_t *s,uint32_t now)
{
    unsigned i;uint32_t sum=0,dt;float delta;
    b->decision_ms=now;
    if(!s->valid){bms_core_fail(b,BMS_F_COMM);return;}
    if(!recovery_sample(b,s,now)){b->recovery_active=false;}
    else if(!b->recovery_active||now-b->last_sample_ms>BMS_STALE_MS){b->recovery_active=true;b->recovery_since=now;}
    b->faults=measure_faults(b,s,now);
    b->last_sample_ms=now;b->initialized=true;b->latest=*s;b->has_latest=true;
    capture(b,b->faults,now);
    b->latched|=b->faults&BMS_LATCH_MASK;
    for(i=0;i<BMS_CELLS;i++)sum+=s->cell_mv[i];
    if(!b->soc_initialized){b->soc_pct=bms_soc_from_ocv((uint16_t)(sum/BMS_CELLS));b->soc_initialized=true;}
    else {
        dt=now-b->soc_last_ms;
        /* Never integrate a missing/stale sampling gap. */
        if(dt<=BMS_STALE_MS && s->current_valid){
            delta=(float)s->current_ma*(float)dt/(36000.0f*(float)b->cfg.capacity_mah);
            b->soc_pct+=delta;if(b->soc_pct>100)b->soc_pct=100;if(b->soc_pct<0)b->soc_pct=0;
        }
    }
    b->soc_last_ms=now;
    if(absi(s->current_ma)>b->cfg.balance_idle_ma)b->idle_since=now;
}
static uint8_t select_balance(bms_core_t *b,const bms_sample_t *s)
{
    uint16_t lo=65535,hi;unsigned i,j,count=0;uint8_t mask=0;int best;
    static const uint8_t channel[BMS_CELLS]={0,1,4,5,6,9};
    for(i=0;i<BMS_CELLS;i++)if(s->cell_mv[i]<lo)lo=s->cell_mv[i];
    for(j=0;j<2;j++){
        best=-1;hi=0;
        for(i=0;i<BMS_CELLS;i++){
            bool adjacent=false;unsigned k;
            if((mask&(1u<<i))||s->cell_mv[i]<b->cfg.balance_min_mv||s->cell_mv[i]<lo+b->cfg.balance_delta_mv)continue;
            for(k=0;k<BMS_CELLS;k++)if(mask&(1u<<k)){
                if((i+1==k)||(k+1==i))adjacent=true;
                if(channel[i]/5==channel[k]/5 && (channel[i]+1==channel[k]||channel[k]+1==channel[i]))adjacent=true;
            }
            if(!adjacent&&s->cell_mv[i]>hi){hi=s->cell_mv[i];best=(int)i;}
        }
        if(best<0)break;mask|=(uint8_t)(1u<<best);count++;
    }
    (void)count;return mask;
}
void bms_core_step(bms_core_t *b,const bms_sample_t *s,uint32_t now)
{
    bool ca,da;uint32_t hard;
    b->decision_ms=now;
    b->balance_mask=0;
    if(b->shutdown_request){b->chg=b->dis=b->precharge=false;b->pre_active=false;b->state=BMS_SHUTDOWN;return;}
    if(!b->hardware_verified){b->faults|=BMS_F_WIRING;b->chg=b->dis=b->precharge=false;b->state=BMS_OFF;return;}
    if(!b->initialized&&now-b->last_sample_ms<=3000u){b->chg=b->dis=b->precharge=false;b->state=BMS_INIT;return;}
    if(!b->initialized||!s->valid||now-b->last_sample_ms>BMS_STALE_MS){bms_core_fail(b,BMS_F_COMM);return;}
    hard=b->faults&BMS_FATAL_MASK;
    if(b->clear_request){
        b->clear_request=false;
        if(!hard && bms_core_recovery_ready(b,s,now)){b->latched=0;b->state=BMS_OFF;}
    }
    if((b->latched&BMS_FATAL_MASK)||hard){bms_core_fail(b,b->latched|hard);return;}
    ca=b->request_chg && !((b->faults|b->latched)&(BMS_F_OV|BMS_F_CHG_TEMP|BMS_F_CHG_OC));
    da=b->request_dis && !(b->faults&(BMS_F_UV|BMS_F_DIS_TEMP));
    if(!ca&&!da){b->chg=b->dis=b->precharge=false;b->pre_active=false;b->state=BMS_OFF;return;}
    if(!da){b->chg=ca;b->dis=false;b->precharge=false;b->pre_active=false;b->state=BMS_CHARGE_ONLY;return;}
    if(!b->dis){
        if(!s->output_valid){bms_core_fail(b,BMS_F_OUTPUT);return;}
        if(s->output_mv+1000>=s->pack_mv && s->output_mv<=s->pack_mv+2000){
            b->dis=true;b->precharge=false;b->pre_active=false;
        } else {
            /* No precharge through an unsafe charge path; reverse-feed is not a precharge. */
            if(!ca||s->output_mv>s->pack_mv+2000){bms_core_fail(b,BMS_F_PRECHARGE);return;}
            if(!b->pre_active){b->pre_active=true;b->precharge_start=now;}
            if(now-b->precharge_start>=b->cfg.precharge_ms){bms_core_fail(b,BMS_F_PRECHARGE);return;}
            b->chg=true;b->precharge=true;b->state=BMS_PRECHARGE;return;
        }
    }
    b->chg=ca;b->state=ca?BMS_NORMAL:BMS_DISCHARGE_ONLY;
    if(b->cfg.balance_enable && !(b->faults&~BMS_F_SPREAD_WARN) && !b->latched && s->temps_valid &&
       s->board_temp_dc<500 && now-b->idle_since>=(uint32_t)b->cfg.balance_idle_s*1000u &&
       absi(s->current_ma)<=b->cfg.balance_idle_ma)b->balance_mask=select_balance(b,s);
}
bool bms_core_recovery_ready(const bms_core_t *b,const bms_sample_t *s,uint32_t now)
{
    unsigned i;uint32_t f=b->faults|b->latched;
    if(!recovery_sample(b,s,now))return false;
    if(b->latched&&(!b->recovery_active||now-b->recovery_since<5000u))return false;
    if(!b->initialized||!s->valid||!s->temps_valid||!s->current_valid||
       now-b->last_sample_ms>BMS_STALE_MS||now-s->current_time_ms>BMS_STALE_MS||
       absi(s->current_ma)>=200||s->board_temp_dc>800)return false;
    for(i=0;i<BMS_CELLS;i++){
        if(s->cell_mv[i]<1800||s->cell_mv[i]>4500)return false;
        if((f&(BMS_F_OV|BMS_F_HW_OV))&&s->cell_mv[i]>b->cfg.ov_recover_mv)return false;
        if((f&(BMS_F_UV|BMS_F_HW_UV))&&s->cell_mv[i]<b->cfg.uv_recover_mv)return false;
    }
    if((f&BMS_F_SPREAD_TRIP)&&(!b->spread_recover_active||
       b->spread_mv>BMS_SPREAD_RECOVER_MV||now-b->spread_recover_since<BMS_SPREAD_RECOVER_MS))return false;
    return true;
}
