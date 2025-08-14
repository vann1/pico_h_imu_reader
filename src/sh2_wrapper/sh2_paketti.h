#ifndef SH2_PAKETTI_H
#define SH2_PAKETTI_H
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include "sh2_hal.h"

static void clear_i2c_flags();
static int i2c_open(sh2_Hal_t* pInstance);
static void i2c_close(sh2_Hal_t* pInstance);
static int i2c_read(sh2_Hal_t* pInstance, uint8_t *pData, unsigned len, uint32_t *pTimestamp_us);
static int i2c_write(sh2_Hal_t* pInstance, uint8_t *pData, unsigned len);
static uint32_t get_time_us(sh2_Hal_t* pInstance);
static void sensor_handler(void *cookie, sh2_SensorEvent_t *event);
static void async_handler(void *cookie, sh2_AsyncEvent_t *event);
static void sh2_open_or_halt();
static void sh2_setSensorCallback_or_halt();
static void sh2_devReset_or_halt();
static void sh2_setSensorConfig_or_halt();
static void initialize_HALL();
static void wait_for_reset_or_halt();
void setup_sh2_service();
void read_super_sensor();
#endif // SH2_PAKETTI_H