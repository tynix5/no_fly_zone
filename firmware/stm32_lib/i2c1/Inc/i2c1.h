
#ifndef INC_I2C1_H_
#define INC_I2C1_H_

void i2c1_config();
void i2c1_sw_rst();
void i2c1_writebyte(uint8_t slave_addr, uint8_t reg_addr, uint8_t byte);
uint8_t i2c1_readbyte(uint8_t slave_addr, uint8_t reg_addr);
void i2c1_readburst(uint8_t slave_addr, uint8_t reg_addr, uint8_t len, uint8_t * bytes);
void i2c1_start();
void i2c1_request_stop();
void i2c1_ack();
void i2c1_nack();
void i2c1_release();


#endif
