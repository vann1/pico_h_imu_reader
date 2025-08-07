#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "tusb.h"
#include "ism330dlc_registers.h"
#include "ism330dlc_config.h"
#include "bit_ops.h"
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
bool ism330dhcx_read_reg(i2c_inst_t *i2c_port, uint8_t device_addr, uint8_t reg, uint8_t* value, uint8_t read_count) {
    int result = i2c_write_blocking(i2c_port, device_addr, &reg, 1, true);
    if (result != 1) return false;
    result = i2c_read_blocking(i2c_port, device_addr, value, read_count, false);
    return result == 1;
}

bool ism330dhcx_read_gyro(i2c_inst_t* i2c_port, uint8_t device_addr) {
    uint8_t raw_gyro_values[6];
	ism330dhcx_read_reg(i2c_port, device_addr, OUTX_L_G ,raw_gyro_values ,6);
    int16_t gyro_x = combine_8_bits(raw_gyro_values[0], raw_gyro_values[1]);
    int16_t gyro_y = combine_8_bits(raw_gyro_values[2], raw_gyro_values[3]);
    int16_t gyro_z = combine_8_bits(raw_gyro_values[4], raw_gyro_values[5]);

	printf("gyro_x: %d, gyro_y: %d, gyro_z: %d \n",gyro_x, gyro_y, gyro_z);
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



// // Function to read multiple bytes
// bool ism330dhcx_read_bytes(i2c_inst_t *i2c_port, uint8_t device_addr, uint8_t reg, uint8_t *buffer, uint8_t len) {
//     // i2c_write_blocking is like waking call for reading the actual data, when nostop parameter is True
//     int result = i2c_write_blocking(i2c_port, device_addr, &reg, 1, true);
//     if (result != 1) return false;
//     result = i2c_read_blocking(i2c_port, device_addr, buffer, len, false);
//     return result == len;
// }

bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

// Initialize ISM330DHCX
bool ism330dhcx_init(i2c_inst_t *i2c_port, uint8_t device_addr) {
    uint8_t id = 0;

    // Configure accelerometer 
    // ODR = 6.66 Hz, ±2g
    uint8_t xl_cntrl1_val = XL_ODR | XL_G_RANGE_MASK;
    if (!ism330dhcx_write_reg(i2c_port, device_addr, CTRL1_XL, xl_cntrl1_val)) {
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

int setup_I2C_pins() {
    // Initialize i2c0 bus and gpio pins
    int BAUD_RATE = 100*1000;
    uint result = i2c_init(I2C_PORT_0, BAUD_RATE);
    if (result != BAUD_RATE) return 0;
    gpio_set_function(I2C_SDA_0, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_0, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_0);
    gpio_pull_up(I2C_SCL_0);
    // Initialize i2c1 bus and gpio pins
    uint result = i2c_init(I2C_PORT_1, BAUD_RATE);
    if (result != BAUD_RATE) return 0;
    gpio_set_function(I2C_SDA_1, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_1, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_1);
    gpio_pull_up(I2C_SCL_1);
    
    return 1;
}

int initialize_sensors(void) {
	printf("Intializing sensors\n");
	    // Initialize sensors
    // if (!ism330dhcx_init(I2C_PORT_0, ISM330DHCX_ADDR_DO_LOW)) {
        // printf("Failed to initialize ISM330DHCX with i2c_port: %s and i2c_address: 0x%02x!\n", I2C_PORT_0, ISM330DHCX_ADDR_DO_LOW);
    // }

   // if (!ism330dhcx_init(I2C_PORT_0, ISM330DHCX_ADDR_DO_HIGH)) {
            // printf("Failed to initialize ISM330DHCX with i2c_port: %s and i2c_address: 0x%02x!\n", I2C_PORT_0, ISM330DHCX_ADDR_DO_HIGH);
      // }
    if (!ism330dhcx_init(I2C_PORT_1, ISM330DHCX_ADDR_DO_LOW)) {
        printf("Failed to initialize ISM330DHCX with i2c_port: %s and i2c_address: 0x%02x!\n", I2C_PORT_1, ISM330DHCX_ADDR_DO_LOW);
    }
    // if (!ism330dhcx_init(I2C_PORT_1, ISM330DHCX_ADDR_DO_HIGH)) {
           // printf("Failed to initialize ISM330DHCX with i2c_port: %s and i2c_address: 0x%02x!\n", I2C_PORT_1, ISM330DHCX_ADDR_DO_HIGH);
       // }
       printf("Sensor initialized successfully!\n");
}

int main() {
    stdio_init_all();
    while (!tud_cdc_connected()) {
        sleep_ms(100);
    }

    int result = setup_I2C_pins();
    if (result != 1) {
		printf("I2C pin setup failed");
		return 1;
    }
    //Scan i2c devices
	initialize_sensors();    
    
    printf("Starting data stream...\n");
    // i2c_scan(I2C_PORT_0);
    // i2c_scan(I2C_PORT_1);


    // Main loop

    while (1) {
        sleep_ms(500);
        ism330dhcx_read_gyro(I2C_PORT_0,ISM330DHCX_ADDR_DO_LOW);
    }
    
    return 0;
}
