#include "bms_config.h"
#include <string.h>
void bms_config_defaults(bms_config_t *c)
{
    memset(c,0,sizeof(*c));
    c->capacity_mah=3000; c->shunt_uohm=4000;
    c->ov_mv=4230; c->uv_mv=3000; c->ov_recover_mv=4100; c->uv_recover_mv=3300;
    c->discharge_ma=10000; c->charge_ma=1500;
    /* R25 energy/load capacitance is unverified: never wait indefinitely. */
    c->precharge_ms=80; c->balance_delta_mv=20; c->balance_min_mv=3800;
    c->balance_idle_ma=200; c->balance_idle_s=30;
    c->charge_min_dc=0; c->charge_max_dc=450;
    c->discharge_min_dc=-200; c->discharge_max_dc=600;
    c->hysteresis_dc=30; c->balance_enable=1;
}
bool bms_config_valid(const bms_config_t *c)
{
    return c && c->capacity_mah>=500 && c->capacity_mah<=20000 &&
        c->shunt_uohm>=1000 && c->shunt_uohm<=10000 &&
        c->ov_mv>=4100 && c->ov_mv<=4250 && c->uv_mv>=2800 && c->uv_mv<=3300 &&
        c->ov_recover_mv>=3900 && c->ov_recover_mv<c->ov_mv &&
        c->uv_recover_mv>c->uv_mv && c->uv_recover_mv<=3600 &&
        c->charge_ma>=100 && c->charge_ma<=3000 &&
        c->discharge_ma>=1000 && c->discharge_ma<=12000 &&
        c->precharge_ms>=10 && c->precharge_ms<BMS_PRECHARGE_HARD_MS &&
        c->balance_delta_mv>=10 && c->balance_delta_mv<=100 &&
        c->balance_min_mv>=3600 && c->balance_min_mv<=4150 &&
        c->balance_idle_ma<=500 && c->balance_idle_s>=10 && c->balance_idle_s<=3600 &&
        c->charge_min_dc>=0 && c->charge_min_dc<=100 &&
        c->charge_max_dc>=350 && c->charge_max_dc<=450 &&
        c->discharge_min_dc>=-200 && c->discharge_min_dc<=100 &&
        c->discharge_max_dc>=350 && c->discharge_max_dc<=600 &&
        c->hysteresis_dc>=10 && c->hysteresis_dc<=100 && c->balance_enable<=1;
}
