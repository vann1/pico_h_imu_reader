[1mdiff --git a/ism330dlc_reader.c b/ism330dlc_reader.c[m
[1mindex c6fa1a5..c3563e4 100644[m
[1m--- a/ism330dlc_reader.c[m
[1m+++ b/ism330dlc_reader.c[m
[36m@@ -47,13 +47,24 @@[m [mbool ism330dhcx_read_gyro(i2c_inst_t* i2c_port, uint8_t device_addr) {[m
 	return 1;[m
 }[m
 [m
[31m-bool ism330dhcx_read_accelerometer(i2c_inst_t* i2c_port, uint8_t device_addr, uint8_t reg, uint8_t* value) {[m
[32m+[m[32mbool ism330dhcx_read_accelerometer(i2c_inst_t* i2c_port, uint8_t device_addr) {[m
[32m+[m[32m    uint8_t raw_acc_values[6];[m
[32m+[m	[32mism330dhcx_read_reg(i2c_port, device_addr, OUTX_L_XL ,raw_acc_values ,6);[m
[32m+[m[32m    int16_t raw_acc_x = combine_8_bits(raw_acc_values[0], raw_acc_values[1]);[m
[32m+[m[32m    int16_t raw_acc_y = combine_8_bits(raw_acc_values[2], raw_acc_values[3]);[m
[32m+[m[32m    int16_t raw_acc_z = combine_8_bits(raw_acc_values[4], raw_acc_values[5]);[m
[32m+[m
[32m+[m[32m    float acc_x = ((float)raw_acc_x/32768.0f)*(float)XL_G_RANGE;[m
[32m+[m[32m    float acc_y = ((float)raw_acc_y/32768.0f)*(float)XL_G_RANGE;[m
[32m+[m[32m    float acc_z = ((float)raw_acc_z/32768.0f)*(float)XL_G_RANGE;[m
[32m+[m
[32m+[m	[32mprintf("acc_x: %f, acc_y: %f, acc_z: %f \n",acc_x, acc_y, acc_z);[m
 	return 1;[m
 }[m
 [m
[31m-bool ism330dhcx_read(i2c_inst_t* i2c_port, uint8_t device_addr, uint8_t reg, uint8_t* value) {[m
[31m-	// ism330dhcx_read_accelerometer();[m
[31m-	// ism330dhcx_read_gyro();[m
[32m+[m[32mbool ism330dhcx_read(i2c_inst_t* i2c_port, uint8_t device_addr) {[m
[32m+[m	[32mism330dhcx_read_accelerometer(i2c_port,device_addr);[m
[32m+[m	[32mism330dhcx_read_gyro(i2c_port,device_addr);[m
 	return 0;[m
 }[m
 [m
[36m@@ -135,9 +146,8 @@[m [mint main() {[m
     // Main loop[m
 [m
     while (1) {[m
[31m-[m
[32m+[m[32m        ism330dhcx_read(I2C_PORT_1,ISM330DHCX_ADDR_DO_LOW);[m
         sleep_ms(1000);[m
[31m-        ism330dhcx_read_gyro(I2C_PORT_1,ISM330DHCX_ADDR_DO_LOW);[m
     }[m
     [m
     return 0;[m
