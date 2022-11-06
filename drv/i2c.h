#ifndef __I2C_H
#define __I2C_H

#include <stdint.h>

#define I2C_ADDR_NACK   1
#define I2C_OK          0

void i2c_init(void);
uint8_t i2c_check_address(uint8_t i2c_addr);
void i2c_write_register(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data);
void i2c_read_register(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *data);
void i2c_write_registers(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buffer, uint8_t size);
void i2c_read_registers(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buffer, uint8_t size);

/* https://github.com/alepnm/bandymas_hal_ll/blob/master/BSP/hardware/iic.c */

#endif /* _I2C_H */
