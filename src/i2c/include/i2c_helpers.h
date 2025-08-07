#ifndef I2C_HELPERS_H
#include <stdio.h>         
#include <pico/stdlib.h>   
#include <hardware/i2c.h>
#include <pico/error.h>
#define I2C_HELPERS_H
void i2c_scan(i2c_inst_t *i2c_port);
#endif
