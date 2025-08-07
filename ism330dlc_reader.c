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
#include "i2c_helpers.h"

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
void print_list(uint8_t list[], int size){
    printf("Raw gyro values: ");
    for (int i = 0; i < size; i++){
        printf("%d ", list[i]);
    }
    printf("\n");
}

bool ism330dhcx_read_gyro(i2c_inst_t* i2c_port, uint8_t device_addr) {
    uint8_t raw_gyro_values[6];
	ism330dhcx_read_reg(i2c_port, device_addr, OUTX_L_G ,raw_gyro_values ,6);
    int16_t raw_gyro_x = combine_8_bits(raw_gyro_values[0], raw_gyro_values[1]);
    int16_t raw_gyro_y = combine_8_bits(raw_gyro_values[2], raw_gyro_values[3]);
    int16_t raw_gyro_z = combine_8_bits(raw_gyro_values[4], raw_gyro_values[5]);

    float gyro_x = ((float)raw_gyro_x/32768.0f)*(float)G_DPS_RANGE;
    float gyro_y = ((float)raw_gyro_y/32768.0f)*(float)G_DPS_RANGE;
    float gyro_z = ((float)raw_gyro_z/32768.0f)*(float)G_DPS_RANGE;

	printf("gyro_x: %f, gyro_y: %f, gyro_z: %f \n",gyro_x, gyro_y, gyro_z);
	return 1;
}

bool ism330dhcx_read_accelerometer(i2c_inst_t* i2c_port, uint8_t device_addr) {
    uint8_t raw_acc_values[6];
	ism330dhcx_read_reg(i2c_port, device_addr, OUTX_L_XL ,raw_acc_values ,6);
    int16_t raw_acc_x = combine_8_bits(raw_acc_values[0], raw_acc_values[1]);
    int16_t raw_acc_y = combine_8_bits(raw_acc_values[2], raw_acc_values[3]);
    int16_t raw_acc_z = combine_8_bits(raw_acc_values[4], raw_acc_values[5]);

    float acc_x = ((float)raw_acc_x/32768.0f)*(float)XL_G_RANGE;
    float acc_y = ((float)raw_acc_y/32768.0f)*(float)XL_G_RANGE;
    float acc_z = ((float)raw_acc_z/32768.0f)*(float)XL_G_RANGE;

	printf("acc_x: %f, acc_y: %f, acc_z: %f \n",acc_x, acc_y, acc_z);
	return 1;
}

bool ism330dhcx_read(i2c_inst_t* i2c_port, uint8_t device_addr) {
	ism330dhcx_read_accelerometer(i2c_port,device_addr);
	ism330dhcx_read_gyro(i2c_port,device_addr);
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
	initialize_sensors();    
    
    printf("Starting data stream...\n");
    i2c_scan(I2C_PORT_1);


    // Main loop

    while (1) {
        ism330dhcx_read(I2C_PORT_1,ISM330DHCX_ADDR_DO_LOW);
        sleep_ms(1000);
    }
    
    return 0;
}
