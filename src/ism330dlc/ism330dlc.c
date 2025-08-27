#include "ism330dlc.h"
#include "i2c_helpers.h"
#include "FusionMath.h"
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

bool ism330dhcx_read_gyro(i2c_inst_t* i2c_port, uint8_t device_addr, FusionVector* fusion_vector) {
    uint8_t raw_gyro_values[6];
	ism330dhcx_read_reg(i2c_port, device_addr, OUTX_L_G ,raw_gyro_values ,6);
    int16_t raw_gyro_x = combine_8_bits(raw_gyro_values[0], raw_gyro_values[1]);
    int16_t raw_gyro_y = combine_8_bits(raw_gyro_values[2], raw_gyro_values[3]);
    int16_t raw_gyro_z = combine_8_bits(raw_gyro_values[4], raw_gyro_values[5]);

    fusion_vector->axis.x = ((float)raw_gyro_x/32768.0f)*(float)G_DPS_RANGE;
    fusion_vector->axis.y = ((float)raw_gyro_y/32768.0f)*(float)G_DPS_RANGE;
    fusion_vector->axis.z = ((float)raw_gyro_z/32768.0f)*(float)G_DPS_RANGE;
	return 1;
}

bool ism330dhcx_read(i2c_inst_t* i2c_port, uint8_t device_addr, uint8_t reg, uint8_t* value) {
	// ism330dhcx_read_accelerometer();
	// ism330dhcx_read_gyro();
	return 0;
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

int initialize_sensors(void) {
	printf("Intializing sensors\n");
    // Initialize sensors
    if (!ism330dhcx_init(I2C_PORT_0, ISM330DHCX_ADDR_DO_LOW)) {
        printf("Failed to initialize ISM330DHCX with i2c_port: %s and i2c_address: 0x%02x!\n", I2C_PORT_0, ISM330DHCX_ADDR_DO_LOW);
    }

   if (!ism330dhcx_init(I2C_PORT_0, ISM330DHCX_ADDR_DO_HIGH)) {
            printf("Failed to initialize ISM330DHCX with i2c_port: %s and i2c_address: 0x%02x!\n", I2C_PORT_0, ISM330DHCX_ADDR_DO_HIGH);
      }
    // if (!ism330dhcx_init(I2C_PORT_1, ISM330DHCX_ADDR_DO_LOW)) {
    //     printf("Failed to initialize ISM330DHCX with i2c_port: %s and i2c_address: 0x%02x!\n", I2C_PORT_1, ISM330DHCX_ADDR_DO_LOW);
    // }
    // if (!ism330dhcx_init(I2C_PORT_1, ISM330DHCX_ADDR_DO_HIGH)) {
           // printf("Failed to initialize ISM330DHCX with i2c_port: %s and i2c_address: 0x%02x!\n", I2C_PORT_1, ISM330DHCX_ADDR_DO_HIGH);
       // }
       printf("Sensor initialized successfully!\n");
}

bool ism330dhcx_read_accelerometer(i2c_inst_t* i2c_port, uint8_t device_addr, FusionVector* fusion_vector) {
    uint8_t raw_acc_values[6];
	ism330dhcx_read_reg(i2c_port, device_addr, OUTX_L_XL ,raw_acc_values ,6);
    int16_t raw_acc_x = combine_8_bits(raw_acc_values[0], raw_acc_values[1]);
    int16_t raw_acc_y = combine_8_bits(raw_acc_values[2], raw_acc_values[3]);
    int16_t raw_acc_z = combine_8_bits(raw_acc_values[4], raw_acc_values[5]);

    fusion_vector->axis.x = ((float)raw_acc_x/32768.0f)*(float)XL_G_RANGE;
    fusion_vector->axis.y = ((float)raw_acc_y/32768.0f)*(float)XL_G_RANGE;
    fusion_vector->axis.z = ((float)raw_acc_z/32768.0f)*(float)XL_G_RANGE;
	return 1;
}
