#include "sh2_paketti.h"

// I2C configuration
#define I2C_PORT i2c0
#define I2C_SDA 0 // GPIO4 (Pico pin 6)
#define I2C_SCL 1  // GPIO5 (Pico pin 7) 
#define I2C_BAUD 400000  // 400 kHz
#define BNO08X_ADDR 0x4A  // BNO08x I2C address

static sh2_Hal_t hal;
static bool reset_received = false;
static int rc;

// This is needed for some reason to avoid some peculiar bugs
static void clear_i2c_flags() {
    i2c_hw_t *i2c = i2c_get_hw(I2C_PORT);
    i2c->enable = 0;  // Disable first for safety
    (void)i2c->clr_tx_abrt;
    (void)i2c->clr_rd_req;
    (void)i2c->clr_tx_over;
    (void)i2c->clr_rx_done;
    (void)i2c->clr_rx_over;
    (void)i2c->clr_rx_under;
    (void)i2c->clr_activity;  // If ACTIVITY_BITS include master/slave activity, clear them
    i2c->enable = 1;  // Re-enable
}


// HAL: Initialize I2C0
static int i2c_open(sh2_Hal_t* pInstance) {
    i2c_init(I2C_PORT, I2C_BAUD);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    return SH2_OK;
}

// HAL: Close I2C
static void i2c_close(sh2_Hal_t* pInstance) {
    i2c_deinit(I2C_PORT);
}

// HAL: Read I2C data
static int i2c_read(sh2_Hal_t* pInstance, uint8_t *pData, unsigned len, uint32_t *pTimestamp_us) {
    // Add delay and bus check
    sleep_ms(5);  // Give some time between operations

    // Wait for bus to be completely idle
    while (i2c_get_hw(I2C_PORT)->status & I2C_IC_STATUS_ACTIVITY_BITS) {
        tight_loop_contents();
    }

    clear_i2c_flags();

    int rc = i2c_read_blocking(I2C_PORT, BNO08X_ADDR, pData, len, false);
    if (rc != len) return SH2_ERR;
    *pTimestamp_us = to_us_since_boot(get_absolute_time());
    return rc;
}


// HAL: Write I2C data
static int i2c_write(sh2_Hal_t* pInstance, uint8_t *pData, unsigned len) {
    // Add delay and bus check
    sleep_ms(5);  // Give some time between operations

    // Wait for bus to be completely idle
    while (i2c_get_hw(I2C_PORT)->status & I2C_IC_STATUS_ACTIVITY_BITS) {
        tight_loop_contents();
    }

    clear_i2c_flags();

    int result = i2c_write_blocking(I2C_PORT, BNO08X_ADDR, pData, len, false);
    return (result == len) ? result : SH2_ERR;
}

// HAL: Get microsecond timestamp
static uint32_t get_time_us(sh2_Hal_t* pInstance) {
    return to_us_since_boot(get_absolute_time());
}

// Sensor event callback
static void sensor_handler(void *cookie, sh2_SensorEvent_t *event) {
    sh2_SensorValue_t value;
    if (sh2_decodeSensorEvent(&value, event) == SH2_OK) {
        if (value.sensorId == SH2_ROTATION_VECTOR) {
            printf("Rotation: w=%.6f, x=%.6f, y=%.6f, z=%.6f\n",
                   value.un.rotationVector.real,
                   value.un.rotationVector.i,
                   value.un.rotationVector.j,
                   value.un.rotationVector.k);
        }
    } else {
        printf("wrong event?\n");
    }
}

// Async event callback
static void async_handler(void *cookie, sh2_AsyncEvent_t *event) {
    if (event->eventId == SH2_RESET) {
        printf("BNO08x reset\n");
        reset_received = true;
    }
}

static void sh2_open_or_halt() {
    // Open SH-2 interface
    rc = sh2_open(&hal, async_handler, NULL);
    if (rc != SH2_OK) {
        printf("SH-2 open failed: %d\n", rc);
        while (1);
    }
}

static void sh2_setSensorCallback_or_halt() {
    rc = sh2_setSensorCallback(sensor_handler, NULL);
    if (rc != SH2_OK) {
        printf("Callback setup failed: %d\n", rc);
        sh2_close();
        while (1);
    }
}

static void sh2_devReset_or_halt() {
    rc = sh2_devReset();
    if (rc != SH2_OK) {
        printf("Reset failed: %d\n", rc);
        sh2_close();
        while (1);
    }
    reset_received = false;
    sleep_ms(300);// Wait for stabilization
}

static void sh2_setSensorConfig_or_halt() {
    // Enable rotation vector (100 Hz)
    sh2_SensorConfig_t config;
    memset(&config, 0, sizeof(config));
    config.reportInterval_us = 10000; // 10 ms = 100 Hz
    rc = sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config);
    if (rc != SH2_OK) {
        printf("Config failed: %d\n", rc);
        sh2_close();
        while (1);
    }
}

static void initialize_HALL() {
    hal.open = i2c_open;
    hal.close = i2c_close;
    hal.read = i2c_read;
    hal.write = i2c_write;
    hal.getTimeUs = get_time_us;
}

static void wait_for_reset_or_halt() {
    absolute_time_t timeout = make_timeout_time_ms(2000);
    while (!reset_received) {
        sh2_service();
        if (time_reached(timeout)) {
            printf("Timeout waiting for reset event!\n");
            sh2_close();
            while(1);
        }
        sleep_ms(10);
    }
}

void setup_sh2_service() {
    initialize_HALL();
    sh2_open_or_halt();
    sh2_setSensorCallback_or_halt();
    // most likely unneeded because sh2 open already does software reset
    sh2_devReset_or_halt(); 
    wait_for_reset_or_halt();
    sh2_setSensorConfig_or_halt();
}

void read_super_sensor() {
    sh2_service();
}
