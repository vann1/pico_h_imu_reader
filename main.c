#include "ism330dlc.h"
#include "i2c_helpers.h"
#include "tusb.h"
#include "Fusion.h"
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <sh2_paketti.h>
#include "read.h"

#define SLEEP_DURATION(hz) (float)(1.0f/hz * 1000.0f)

int main() {
    stdio_init_all();
    while (!tud_cdc_connected()) {
        sleep_ms(100);
    }
    setup_sh2_service();
    // printf("sh2 service has been setup\n");
    // for (int i = 0;i<10;i++) {
    //     read_super_sensor();
    // }
    // while(1);

    int result = setup_I2C_pins();

    if (result != 1) {
		printf("I2C pin setup failed");
		return 1;
    }

	initialize_sensors();

    Sensor sensors[SENSOR_COUNT];
    initialize_calibrations(sensors); 
    printf("calibrated\n");
    initialize_algos(sensors);   
    printf("algoed!\n");
    
    // printf("Starting data stream...\n");
    i2c_scan(I2C_PORT_0);

    while (true) {
        read_all_sensors(sensors);
        for (int i=0; i<SENSOR_COUNT;i++) {
            sensors[i].gyroscope = FusionCalibrationInertial(sensors[i].gyroscope, sensors[i].calibration.gyroscopeMisalignment, sensors[i].calibration.gyroscopeSensitivity, sensors[i].calibration.gyroscopeOffset);
            sensors[i].accelerometer = FusionCalibrationInertial(sensors[i].accelerometer, sensors[i].calibration.accelerometerMisalignment, sensors[i].calibration.accelerometerSensitivity, sensors[i].calibration.accelerometerOffset);
            sensors[i].gyroscope = FusionOffsetUpdate(&sensors[i].offset, sensors[i].gyroscope);

            const float deltaTime = (float) (sensors[i].timestamp - sensors[i].previousTimestamp) / (float) CLOCKS_PER_SEC;
            sensors[i].previousTimestamp = sensors[i].timestamp;

            FusionAhrsUpdateNoMagnetometer(&sensors[i].ahrs, sensors[i].gyroscope, sensors[i].accelerometer, deltaTime);
            const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&sensors[i].ahrs));
                    printf("y%0.1fyp%0.1fpr%0.1fr\n",
               euler.angle.yaw, euler.angle.pitch, euler.angle.roll);
            read_super_sensor();
        }
        printf("---\n");

        sleep_ms(SLEEP_DURATION((float)SAMPLE_RATE));
    }
    return 0;
}
