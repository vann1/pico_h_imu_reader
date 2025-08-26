#include "ism330dlc.h"
#include "i2c_helpers.h"
#include "tusb.h"
#include "Fusion.h"
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <sh2_paketti.h>
#include "read.h"
#include "InitFusion.h"
extern sh2_vector_list_t sh2_vector_list;
#define TIME_SLEEP (1000.0f/(float)SAMPLE_RATE)
#define SLEEP_DURATION(hz) (float)(1.0f/hz * 1000.0f)

float sensors_data[SENSOR_COUNT][4];
#define LPF_ALPHA 0.1f


void print_output_data (void) {
    for (int i = 0; i < SENSOR_COUNT; i++) {
        printf("w%d: %.4f, x%d: %.4f, y%d: %.4f, z%d: %.4f | ",i, sensors_data[i][0],i, sensors_data[i][1],i, sensors_data[i][2],i, sensors_data[i][3] );
    }
    printf("\n");
}

float apply_lpf(float new_val, float old_value) {
    return old_value*(1.0f-LPF_ALPHA) + LPF_ALPHA*new_val;
}


int main() {
    stdio_init_all();
    while (!tud_cdc_connected()) {
        sleep_ms(100);
    }
    // setup_sh2_service();
    int result = setup_I2C_pins();

    if (result != 1) {
		printf("I2C pin setup failed");
		return 1;
    } 
    initialize_sensors();

    Sensor sensors[SENSOR_COUNT];
    initialize_sensors_values(sensors);
    initialize_calibrations(sensors); 
    initialize_algos(sensors);   
    int counter = 0;
    while (true) {
        // uint64_t start_time = time_us_64();
        
        read_all_sensors(sensors);
        for (int i=0; i<SENSOR_COUNT;i++) {
            sensors[i].gyroscope_old.axis.x = apply_lpf(sensors[i].gyroscope.axis.x, sensors[i].gyroscope_old.axis.x);
            sensors[i].gyroscope_old.axis.y = apply_lpf(sensors[i].gyroscope.axis.y, sensors[i].gyroscope_old.axis.y);
            sensors[i].gyroscope_old.axis.z = apply_lpf(sensors[i].gyroscope.axis.z, sensors[i].gyroscope_old.axis.z);

            sensors[i].accelerometer_old.axis.x = apply_lpf(sensors[i].accelerometer.axis.x, sensors[i].accelerometer_old.axis.x);
            sensors[i].accelerometer_old.axis.y = apply_lpf(sensors[i].accelerometer.axis.y, sensors[i].accelerometer_old.axis.y);
            sensors[i].accelerometer_old.axis.z = apply_lpf(sensors[i].accelerometer.axis.z, sensors[i].accelerometer_old.axis.z);

            // printf("gyro_x: %.4f, gyro_y: %.4f, gyro_z: %.4f, acc_x: %.4f, acc_y: %.4f, acc_z: %.4f\n", sensors[i].gyroscope.axis.x,sensors[i].gyroscope.axis.y,sensors[i].gyroscope.axis.z,sensors[i].accelerometer.axis.x,sensors[i].accelerometer.axis.y,sensors[i].accelerometer.axis.z);
            sensors[i].gyroscope = FusionCalibrationInertial(sensors[i].gyroscope_old, sensors[i].calibration.gyroscopeMisalignment, sensors[i].calibration.gyroscopeSensitivity, sensors[i].calibration.gyroscopeOffset);
            sensors[i].accelerometer = FusionCalibrationInertial(sensors[i].accelerometer_old, sensors[i].calibration.accelerometerMisalignment, sensors[i].calibration.accelerometerSensitivity, sensors[i].calibration.accelerometerOffset);
            sensors[i].gyroscope = FusionOffsetUpdate(&sensors[i].offset, sensors[i].gyroscope);

            const float deltaTime = (float) (sensors[i].timestamp - sensors[i].previousTimestamp) / 1e6f;
            sensors[i].previousTimestamp = sensors[i].timestamp;
            FusionAhrsUpdateNoMagnetometer(&sensors[i].ahrs, sensors[i].gyroscope, sensors[i].accelerometer, deltaTime);
            // FusionAhrsUpdateExternalHeading(&sensors[i].ahrs, sensors[i].gyroscope, sensors[i].accelerometer, 0.0f, deltaTime);
            const FusionQuaternion quat = FusionAhrsGetQuaternion(&sensors[i].ahrs);
            sensors_data[i][0] = quat.element.w;
            sensors_data[i][1] = quat.element.x;
            sensors_data[i][2] = quat.element.y;
            sensors_data[i][3] = quat.element.z;           
        }
        print_output_data();
        // uint64_t loop_end = time_us_64();
        // uint64_t iteration_time = loop_end - start_time;
        // printf("Iteration time: %llu\n", iteration_time);
        sleep_ms(TIME_SLEEP-6.252f); 
    }
    return 0;
}
