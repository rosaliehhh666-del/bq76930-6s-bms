#ifndef BMS_UTIL_H
#define BMS_UTIL_H
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
uint8_t bms_crc8(const uint8_t *p,size_t n);
uint16_t bms_crc16(const uint8_t *p,size_t n);
uint32_t bms_crc32(const void *p,size_t n);
bool bms_ntc_dc(uint32_t mv,uint32_t supply,float rpull,float r25,float beta,int16_t *out);
uint16_t bms_le16(const uint8_t *p);
void bms_put16(uint8_t *p,uint16_t v);
void bms_put32(uint8_t *p,uint32_t v);
#endif
