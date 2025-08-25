#include "ism330dlc.h"
#include "i2c_helpers.h"
#include "tusb.h"
#include "Fusion.h"
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <sh2_paketti.h>
#include "read.h"
extern sh2_vector_list_t sh2_vector_list;

#define SLEEP_DURATION(hz) (float)(1.0f/hz * 1000.0f)
#define ARRAY_SIZE (1024)
#define RESULT_COUNT (64)

float sensors_data[SENSOR_COUNT][4];


typedef struct data_fluctuation_t {
    float gyro[3][ARRAY_SIZE];
    float gyro[3][ARRAY_SIZE];
    float results[2][RESULT_COUNT];
    int cursor;
} data_fluctuation_t;

data_fluctuation_t benchmark = {.cursor = 0};


void print_output_data (void) {
    for (int i = 0; i < SENSOR_COUNT; i++) {
        printf("w%d: %.4f, x%d: %.4f, y%d: %.4f, z%d: %.4f | ",i, sensors_data[i][0],i, sensors_data[i][1],i, sensors_data[i][2],i, sensors_data[i][3] );
    }
    printf("\n");
}

void print_raw_sensor_data(Sensor* sensors) {
    for (int i=0; i<SENSOR_COUNT; i++) {
        printf("ax%d: %.4f, ay%d: %.4f, az%d: %.4f| gx%d: %.4f, gy%d: %.4f, gz%d: %.4f \n", i, sensors[i].accelerometer.axis.x, i, sensors[i].accelerometer.axis.y, i, sensors[i].accelerometer.axis.z, i, sensors[i].gyroscope.axis.x, i, sensors[i].gyroscope.axis.y, i, sensors[i].gyroscope.axis.z);
    }
    printf("--------------------------------------------------------------------------------------------------------------------------------------\n");
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
    initialize_calibrations(sensors); 
    initialize_algos(sensors);   
    
    int counter = 0;
    uint64_t start_time = time_us_64();
    while (true) {
        read_all_sensors(sensors);
        // for (int i=0; i<SENSOR_COUNT;i++) {
        //     sensors[i].gyroscope = FusionCalibrationInertial(sensors[i].gyroscope, sensors[i].calibration.gyroscopeMisalignment, sensors[i].calibration.gyroscopeSensitivity, sensors[i].calibration.gyroscopeOffset);
        //     sensors[i].accelerometer = FusionCalibrationInertial(sensors[i].accelerometer, sensors[i].calibration.accelerometerMisalignment, sensors[i].calibration.accelerometerSensitivity, sensors[i].calibration.accelerometerOffset);
        //     sensors[i].gyroscope = FusionOffsetUpdate(&sensors[i].offset, sensors[i].gyroscope);

        //     const float deltaTime = (float) (sensors[i].timestamp - sensors[i].previousTimestamp) / 1e6f;
        //     sensors[i].previousTimestamp = sensors[i].timestamp;
        //     // FusionAhrsUpdateNoMagnetometer(&sensors[i].ahrs, sensors[i].gyroscope, sensors[i].accelerometer, deltaTime);

        //     FusionAhrsUpdateExternalHeading(&sensors[i].ahrs, sensors[i].gyroscope, sensors[i].accelerometer, 0.0f, deltaTime);
        //     const FusionQuaternion quat = FusionAhrsGetQuaternion(&sensors[i].ahrs);
        //     sensors_data[i][0] = quat.element.w;
        //     sensors_data[i][1] = quat.element.x;
        //     sensors_data[i][2] = quat.element.y;
        //     sensors_data[i][3] = quat.element.z;           
        // }
        // print_output_data();
        print_raw_sensor_data(sensors);
        sleep_ms(100); // 120hz | 2
    }
    return 0;
}
