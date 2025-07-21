#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "tusb.h"

// ISM330DHCX I2C address (SDO/SA0 pin low)
#define ISM330DHCX_ADDR_DO_LOW 0x6A
// ISM330DHCX I2C address (SDO/SA0 pin high)
#define ISM330DHCX_ADDR_DO_HIGH 0x6B

// Use 0x6B if SDO/SA0 pin is high

// ISM330DHCX Register addresses
#define WHO_AM_I 0x0F
#define CTRL3_C 0x12 // THIS IS ONLY FOR TESTING, REMOVE AFTER
#define CTRL1_XL 0x10  // Accelerometer control
#define CTRL2_G 0x11   // Gyroscope control
#define STATUS_REG 0x1E
#define OUTX_L_G 0x22  // Gyroscope output registers
#define OUTX_L_XL 0x28 // Accelerometer output registers

// Expected WHO_AM_I value for ISM330DHCX
#define ISM330DHCX_ID 0x6B

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

// Function to read multiple bytes
bool ism330dhcx_read_bytes(i2c_inst_t *i2c_port, uint8_t device_addr, uint8_t reg, uint8_t *buffer, uint8_t len) {
    // i2c_write_blocking is like waking call for reading the actual data, when nostop parameter is True
    int result = i2c_write_blocking(i2c_port, device_addr, &reg, 1, true);
    if (result != 1) return false;
    result = i2c_read_blocking(i2c_port, device_addr, buffer, len, false);
    return result == len;
}


// Initialize ISM330DHCX
bool ism330dhcx_init(i2c_inst_t *i2c_port, uint8_t device_addr) {
    uint8_t id = 0;

    // Check WHO_AM_I register
    if (!ism330dhcx_read_reg(i2c_port, device_addr, WHO_AM_I, &id)) {
        printf("Failed to read WHO_AM_I\n");
        return false;
    }

    if (id != ISM330DHCX_ID) {
        printf("Wrong device ID: 0x%02X (expected 0x%02X)\n", id, ISM330DHCX_ID);
        return false;
    }

    // Configure accelerometer 
    // ODR = 104 Hz, ±2g, Anti-aliasing filter bandwidth = 100 Hz
    if (!ism330dhcx_write_reg(i2c_port, device_addr, CTRL1_XL, 0x40)) {
        printf("Failed to configure accelerometer\n");
        return false;
    }
    
    // Configure gyroscope
    // ODR = 104 Hz, ±250 dps
    if (!ism330dhcx_write_reg(i2c_port, device_addr, CTRL2_G, 0x40)) {
        printf("Failed to configure gyroscope\n");
        return false;
    }

    return true;
}

// Read accelerometer data
bool read_accelerometer(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t buffer[6];
    
    if (!ism330dhcx_read_bytes(I2C_PORT_0, ISM330DHCX_ADDR_DO_LOW,OUTX_L_XL, buffer, 6)) {
        return false;
    }
    
    *x = (int16_t)(buffer[1] << 8 | buffer[0]);
    *y = (int16_t)(buffer[3] << 8 | buffer[2]);
    *z = (int16_t)(buffer[5] << 8 | buffer[4]);
    
    return true;
}

// Read gyroscope data
bool read_gyroscope(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t buffer[6];
    
    if (!ism330dhcx_read_bytes(I2C_PORT_0, ISM330DHCX_ADDR_DO_LOW, OUTX_L_G, buffer, 6)) {
        return false;
    }
    
    *x = (int16_t)(buffer[1] << 8 | buffer[0]);
    *y = (int16_t)(buffer[3] << 8 | buffer[2]);
    *z = (int16_t)(buffer[5] << 8 | buffer[4]);
    
    return true;
}

// Convert raw accelerometer data to mg
float accel_to_mg(int16_t raw) {
    // For ±2g range: 0.061 mg/LSB
    return raw * 0.061f;
}

// Convert raw gyroscope data to mdps (milli-degrees per second)
float gyro_to_mdps(int16_t raw) {
    // For ±250 dps range: 8.75 mdps/LSB
    return raw * 8.75f;
}

// Calculate pitch angle from accelerometer (in degrees)
float calculate_pitch(float ax_mg, float ay_mg, float az_mg) {
    // Pitch: rotation around Y-axis
    // atan2 returns radians, convert to degrees
    return atan2f(ax_mg, sqrtf(ay_mg * ay_mg + az_mg * az_mg)) * 180.0f / M_PI;
}

// Calculate roll angle from accelerometer (in degrees)
float calculate_roll(float ax_mg, float ay_mg, float az_mg) {
    // Roll: rotation around X-axis
    // atan2 returns radians, convert to degrees
    return atan2f(ay_mg, sqrtf(ax_mg * ax_mg + az_mg * az_mg)) * 180.0f / M_PI;
}

int main() {
    stdio_init_all();
    while (!tud_cdc_connected()) {
        sleep_ms(100);
    }
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
    if (!ism330dhcx_init(I2C_PORT_0, ISM330DHCX_ADDR_DO_LOW)) {
        printf("Failed to initialize ISM330DHCX with i2c_port: %s and i2c_address: 0x%02x!\n", I2C_PORT_0, ISM330DHCX_ADDR_DO_LOW);
        while (1) {
            sleep_ms(1000);
        }
    }
    if (!ism330dhcx_init(I2C_PORT_1, ISM330DHCX_ADDR_DO_LOW)) {
        printf("Failed to initialize ISM330DHCX with i2c_port: %s and i2c_address: 0x%02x!\n", I2C_PORT_1, ISM330DHCX_ADDR_DO_LOW);
        while (1) {
            sleep_ms(1000);
        }
    }
    
    printf("Sensor initialized successfully!\n");
    printf("Starting data stream...\n");
    
    // Main loop
    while (1) {
        int16_t ax, ay, az;
        int16_t gx, gy, gz;
        // float values[];
        // Read sensor data
        if (read_accelerometer(&ax, &ay, &az) && read_gyroscope(&gx, &gy, &gz)) {
            // Convert to physical units
            float ax_mg = accel_to_mg(ax);
            float ay_mg = accel_to_mg(ay);
            float az_mg = accel_to_mg(az);
            
            float gx_mdps = gyro_to_mdps(gx);
            float gy_mdps = gyro_to_mdps(gy);
            float gz_mdps = gyro_to_mdps(gz);
            
            // Calculates pitch and roll
            float pitch = calculate_pitch(ax_mg,ay_mg,az_mg);
            float roll = calculate_roll(ax_mg,ay_mg,az_mg);
        } else {
            printf("{\"error\":\"Failed to read sensor\"}\n");
        }
        // Create JSON string
        // char json_buffer[256];
        // snprintf(json_buffer, sizeof(json_buffer),
        //         "{\"accel\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},"
        //         "\"gyro\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},"
        //         "\"pitch\":{\"pitch\":%.5f},"
        //         "\"roll\":{\"roll\":%.5f}}\n",
        //         ax_mg, ay_mg, az_mg, gx_mdps, gy_mdps, gz_mdps, pitch, roll);
        
        // Send over USB CDC
        // printf("%s", json_buffer);

        // Delay between readings (adjust as needed)
        sleep_ms((60/2)*16.6666666666); // 2Hz update rate
    }
    
    return 0;
}