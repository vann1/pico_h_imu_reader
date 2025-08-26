#include "read.h"
#include "ism330dlc.h"
void read_all_sensors(Sensor* sensors) {
    int index = 0;
   for (int i = 0; i < CHANNEL_COUNT; i++) {
        if(i == 0) {
            ism330dhcx_read_accelerometer(I2C_PORT_0,ISM330DHCX_ADDR_DO_HIGH, &sensors[index].accelerometer);
            ism330dhcx_read_gyro(I2C_PORT_0,ISM330DHCX_ADDR_DO_HIGH, &sensors[index].gyroscope);
            sensors[index].timestamp = time_us_64();
            index++;
            ism330dhcx_read_accelerometer(I2C_PORT_0,ISM330DHCX_ADDR_DO_LOW, &sensors[index].accelerometer);
            ism330dhcx_read_gyro(I2C_PORT_0,ISM330DHCX_ADDR_DO_LOW, &sensors[index].gyroscope);
            sensors[index].timestamp = time_us_64();
            index++;
        } else if (i == 1) {
            ism330dhcx_read_accelerometer(I2C_PORT_1,ISM330DHCX_ADDR_DO_LOW, &sensors[index].accelerometer);
            ism330dhcx_read_gyro(I2C_PORT_1,ISM330DHCX_ADDR_DO_LOW, &sensors[index].gyroscope);
            sensors[index].timestamp = time_us_64();
            index++;
        }
   }
}
 