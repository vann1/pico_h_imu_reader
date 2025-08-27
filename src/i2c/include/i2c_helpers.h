#ifndef I2C_HELPERS_H
#include <stdio.h>         
#include <pico/stdlib.h>   
#include <hardware/i2c.h>
#include <pico/error.h>
#define I2C_HELPERS_H
bool reserved_addr(uint8_t addr);
int setup_I2C_pins();
void i2c_scan(i2c_inst_t *i2c_port);
// I2C(0) configuration
#define I2C_PORT_0 i2c0
#define I2C_SDA_0 0
#define I2C_SCL_0 1

// I2C(1) configuration
#define I2C_PORT_1 i2c1
#define I2C_SDA_1 2
#define I2C_SCL_1 3

#define SAMPLE_RATE (120.0f)
#define SENSOR_COUNT 2
#define CHANNEL_COUNT 1

#endif
