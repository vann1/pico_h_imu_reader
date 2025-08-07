#ifndef I2C_HELPERS_H
#include <stdio.h>         
#include <pico/stdlib.h>   
#include <hardware/i2c.h>
#include <pico/error.h>
#define I2C_HELPERS_H
bool reserved_addr(uint8_t addr);
int setup_I2C_pins();
void i2c_scan(i2c_inst_t *i2c_port);
#endif
