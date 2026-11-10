#ifndef BMS_BOARD_H
#define BMS_BOARD_H
#include <stdint.h>
#include <stdbool.h>
void board_init(void);
uint32_t board_ms(void);
void board_us(uint32_t us);
bool board_i2c(unsigned bus,uint8_t address,const uint8_t *tx,unsigned nt,uint8_t *rx,unsigned nr);
bool board_adc(uint16_t *pack_raw,uint16_t *ntc_raw);
void board_precharge(bool on);
bool board_precharge_expired(void);
void board_buzzer(bool on);
void board_esp_reset(bool reset);
bool board_key(void);
void board_wake(bool on);
bool board_afe_alert(void);
bool board_uart_send(unsigned port,const uint8_t *p,unsigned n);
bool board_uart_get(unsigned port,uint8_t *v,uint32_t *stamp);
bool board_uart_overflow(unsigned port);
bool board_can_receive(uint8_t data[8]);
bool board_can_send(uint16_t id,const uint8_t data[8]);
void board_watchdog_start(void);
void board_watchdog_feed(void);
void board_emergency_off(void);
uint16_t board_flash_kb(void);
bool board_links_idle(void);
void board_links_standby(bool standby);
#endif
