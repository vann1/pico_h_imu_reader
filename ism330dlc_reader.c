#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "tusb.h"
#include "ism330dlc_registers.h"
#include "ism330dlc_config.h"


// I2C(0) configuration
#define I2C_PORT_0 i2c0
#define I2C_SDA_0 0
#define I2C_SCL_0 1

// I2C(1) configuration
#define I2C_PORT_1 i2c1
#define I2C_SDA_1 2
#define I2C_SCL_1 3

// Function to write to ISM330DHCX register
bool ism330dhcx_write_reg(i2c_inst_t *i2c_port, uint8_t device_addr, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    int result = i2c_write_blocking(i2c_port, device_addr, buf, 2, false);
    return result == 2;
}

// Function to read from ISM330DHCX register
bool ism330dhcx_read_reg(i2c_inst_t *i2c_port, uint8_t device_addr, uint8_t reg, uint8_t *value) {
    int result = i2c_write_blocking(i2c_port, device_addr, &reg, 1, true);
    if (result != 1) return false;
    result = i2c_read_blocking(i2c_port, device_addr, value, 1, false);
    return result == 1;
}

bool ism330dhcx_read_gyro(i2c_inst_t* i2c_port, uint8_t device_addr, uint8_t* value) {
	
	
	return 1;
}

bool ism330dhcx_read_accelerometer(i2c_inst_t* i2c_port, uint8_t device_addr, uint8_t reg, uint8_t* value) {
	return 1;
}

bool ism330dhcx_read(i2c_inst_t* i2c_port, uint8_t device_addr, uint8_t reg, uint8_t* value) {
	// ism330dhcx_read_accelerometer();
	// ism330dhcx_read_gyro();
	return 0;
}



// Function to read multiple bytes
bool ism330dhcx_read_bytes(i2c_inst_t *i2c_port, uint8_t device_addr, uint8_t reg, uint8_t *buffer, uint8_t len) {
    // i2c_write_blocking is like waking call for reading the actual data, when nostop parameter is True
    int result = i2c_write_blocking(i2c_port, device_addr, &reg, 1, true);
    if (result != 1) return false;
    result = i2c_read_blocking(i2c_port, device_addr, buffer, len, false);
    return result == len;
}

bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

void i2c_scan(i2c_inst_t *i2c_port) {
    printf("\nI2C Bus Scan\n");
    printf("---0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }

        // Perform a 1-byte dummy read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.

        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        if (reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            ret = i2c_read_blocking(i2c_port, addr, &rxdata, 1, false);

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
}

// Initialize ISM330DHCX
bool ism330dhcx_init(i2c_inst_t *i2c_port, uint8_t device_addr) {
    uint8_t id = 0;

    // Check WHO_AM_I register
    // if (!ism330dhcx_read_reg(i2c_port, device_addr, WHO_AM_I, &id)) {
        // printf("Failed to read WHO_AM_I\n");
        // return false;
    // }

    // if (id != ISM330DHCX_ID) {
        // printf("Wrong device ID: 0x%02X (expected 0x%02X)\n", id, ISM330DHCX_ID);
        // return false;
    // }

    // Configure accelerometer 
    // ODR = 6.66 Hz, ±2g
    uint8_t xl_cntrl1_val = XL_ODR | XL_G_RANGE;
    if (!ism330dhcx_write_reg(i2c_port, device_addr, CTRL1_XL, xl_cntrl_val)) {
        printf("Failed to configure accelerometer\n");
        return false;
    }
    
    // Configure gyroscope
    // ODR = 6.66 kHz, ±250 dps
    uint8_t g_cntrl_val = G_ODR | G_DPS_RANGE_MASK;
    printf("Gyro control value: %d\n", g_cntrl_val);
    if (!ism330dhcx_write_reg(i2c_port, device_addr, CTRL2_G, G_ODR | G_DPS_RANGE_MASK)) {
        printf("Failed to configure gyroscope\n");
        return false;
    }

    return true;
}


int main() {
    stdio_init_all();
    while (!tud_cdc_connected()) {
        sleep_ms(100);
    }
    //Scan i2c devices

	printf("ism register CTRL3_C: %d\n", CTRL3_C);
    // Initialize i2c0 bus and gpio pins
    i2c_init(I2C_PORT_0, 100*1000);
    gpio_set_function(I2C_SDA_0, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_0, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_0);
    gpio_pull_up(I2C_SCL_0);
    // Initialize i2c1 bus and gpio pins
    i2c_init(I2C_PORT_1, 100*1000);
    gpio_set_function(I2C_SDA_1, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_1, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_1);
    gpio_pull_up(I2C_SCL_1);
    printf("ISM330DHCX USB CDC Reader\n");
    printf("Initializing sensor...\n");
    
    
    // Initialize sensors
    // if (!ism330dhcx_init(I2C_PORT_0, ISM330DHCX_ADDR_DO_LOW)) {
        // printf("Failed to initialize ISM330DHCX with i2c_port: %s and i2c_address: 0x%02x!\n", I2C_PORT_0, ISM330DHCX_ADDR_DO_LOW);
        // while (1) {
            // sleep_ms(1000);
        // }
    // }
    
    if (!ism330dhcx_init(I2C_PORT_1, ISM330DHCX_ADDR_DO_LOW)) {
        printf("Failed to initialize ISM330DHCX with i2c_port: %s and i2c_address: 0x%02x!\n", I2C_PORT_1, ISM330DHCX_ADDR_DO_LOW);
        while (1) {
            sleep_ms(1000);
        }
    }
    
    printf("Sensor initialized successfully!\n");
    printf("Starting data stream...\n");
    // i2c_scan(I2C_PORT_0);
    // i2c_scan(I2C_PORT_1);


    // Main loop
    while (1) {
        continue;
    }
    
    return 0;
}
