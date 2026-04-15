#ifndef APP_H_
#define APP_H_

#include <stdint.h>

void app_init(void);
void app(void);

void spi1_init(void);
uint8_t spi1_transfer_byte(uint8_t addr, uint8_t data);
void spi1_transfer(uint8_t addr, uint8_t * src, uint8_t * dest, uint8_t len);

void cs_low();
void cs_high();
void ce_low();
void ce_high();

#endif