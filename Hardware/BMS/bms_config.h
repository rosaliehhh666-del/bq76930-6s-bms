#ifndef BMS_CONFIG_H
#define BMS_CONFIG_H
#include <stdint.h>
#include <stdbool.h>

/* Commissioning interlock, not a declaration that every protection feature is absent.
 * Live schematic checked 2026-11-05: R70 is GND-to-P-, NOT shorted to GND.
 * Verify populated shunt/divider/NTC parts, MCU wiring, current sign and actual
 * turn-off behavior before enabling. Value text alone is not BOM verification.
 * No independent MCU-to-CHG_EN/DSG_EN kill branch was found in that schematic.
 */
#ifndef BMS_HARDWARE_VERIFIED
#define BMS_HARDWARE_VERIFIED 0
#endif
/* P+ is also the charger bus. Voltage matching alone cannot prove a load
 * capacitor is charged. Keep arming blocked until both charger-present and
 * charger-absent transitions are verified on the actual port topology.
 * Independent from the general hardware commissioning switch above. */
#ifndef BMS_PORT_ARMING_VERIFIED
#define BMS_PORT_ARMING_VERIFIED 0
#endif
#define BMS_CELLS 6
#define BMS_TS_COUNT 2
#define BMS_SAMPLE_MS 250u
#define BMS_STALE_MS 750u
#define BMS_PRECHARGE_HARD_MS 100u
#define BMS_ADC_VREF_MV 3300u
#define BMS_PACK_DIV_NUM 23u /* R12=220k, R13=10k; use ACTUAL populated values */
#define BMS_PACK_DIV_DEN 1u
#define BMS_AFE_NTC_BETA 3435.0f /* 10k 103AT-compatible probes, not 100k */
#define BMS_BOARD_NTC_BETA 4250.0f
#define BMS_CURRENT_SIGN 1 /* positive=charge; confirm by a known charge current */
#define BMS_INA_SHUNT_UOHM 5000u /* R60 only; INA226 measures USB-side current */
#define BMS_MODBUS_ADDRESS 1u
#define BMS_CAN_COMMANDS 1 /* commands still pass all local safety interlocks */
#define BMS_CONFIG_VERSION 1u
/* Commissioning defaults, not validated Samsung cell limits. */
#define BMS_SPREAD_WARN_MV 200u
#define BMS_SPREAD_WARN_RELEASE_MV 150u
#define BMS_SPREAD_TRIP_MV 500u
#define BMS_SPREAD_RECOVER_MV 100u
#define BMS_SPREAD_WARN_MS 5000u
#define BMS_SPREAD_TRIP_MS 2000u
#define BMS_SPREAD_RECOVER_MS 5000u

typedef struct {
    uint32_t capacity_mah;
    uint32_t shunt_uohm;
    uint16_t ov_mv, uv_mv, ov_recover_mv, uv_recover_mv;
    uint16_t discharge_ma, charge_ma;
    uint16_t precharge_ms, balance_delta_mv, balance_min_mv;
    uint16_t balance_idle_ma, balance_idle_s;
    int16_t charge_min_dc, charge_max_dc, discharge_min_dc, discharge_max_dc;
    uint16_t hysteresis_dc;
    uint8_t balance_enable;
    uint8_t reserved;
} bms_config_t;
void bms_config_defaults(bms_config_t *c);
bool bms_config_valid(const bms_config_t *c);
#endif
