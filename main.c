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

float sensors_data[SENSOR_COUNT][4];


void print_output_data (void) {
    for (int i = 0; i < 1; i++) {
        printf("w%d: %.2f, x%d: %.2f, y%d: %.2f, z%d: %.2f | ",i, sensors_data[i][0],i, sensors_data[i][1],i, sensors_data[i][2],i, sensors_data[i][3] );
    }
    printf("\n");
}


int main() {
    stdio_init_all();
    while (!tud_cdc_connected()) {
        sleep_ms(100);
    }
    // int result = setup_I2C_pins();

    // if (result != 1) {
	// 	printf("I2C pin setup failed");
	// 	return 1;
    // }

	// initialize_sensors();
    setup_sh2_service();

    Sensor sensors[SENSOR_COUNT];
    // initialize_calibrations(sensors); 
    // initialize_algos(sensors);   
    
    int counter = 0;
    clock_t start_time = clock();
    clock_t elapsed_time = start_time;
    while (true) {
        // // uint64_t start = time_us_64();
        // read_all_sensors(sensors);
        //     // uint64_t end = time_us_64();
        // // printf("READ ALL SENSORS ------ %llu\n", end-start);
        
        // // uint64_t start = time_us_64();
        // for (int i=0; i<SENSOR_COUNT;i++) {
        //     sensors[i].gyroscope = FusionCalibrationInertial(sensors[i].gyroscope, sensors[i].calibration.gyroscopeMisalignment, sensors[i].calibration.gyroscopeSensitivity, sensors[i].calibration.gyroscopeOffset);
        //     sensors[i].accelerometer = FusionCalibrationInertial(sensors[i].accelerometer, sensors[i].calibration.accelerometerMisalignment, sensors[i].calibration.accelerometerSensitivity, sensors[i].calibration.accelerometerOffset);
        //     sensors[i].gyroscope = FusionOffsetUpdate(&sensors[i].offset, sensors[i].gyroscope);
            
        //     const float deltaTime = (float) (sensors[i].timestamp - sensors[i].previousTimestamp) / (float) CLOCKS_PER_SEC;
        //     sensors[i].previousTimestamp = sensors[i].timestamp;

        //     FusionAhrsUpdateNoMagnetometer(&sensors[i].ahrs, sensors[i].gyroscope, sensors[i].accelerometer, deltaTime);
        //     const FusionQuaternion quat = FusionAhrsGetQuaternion(&sensors[i].ahrs);
        //     sensors_data[i][0] = quat.element.w;
        //     sensors_data[i][1] = quat.element.x;
        //     sensors_data[i][2] = quat.element.y;
        //     sensors_data[i][3] = quat.element.z;           
        // }
        // uint64_t end = time_us_64();
        // printf("FUSION AHRS UPDATE ------ %llu\n", end-start);
        // uint64_t start = time_us_64();
        // print_output_data();
        // uint64_t end = time_us_64();
        // printf("WHOLE THINGY ------ %llu\n", end-start);
        read_super_sensor();
        print_output_data();
        counter++;
        elapsed_time = (clock() - start_time) / (float)CLOCKS_PER_SEC;
        if (elapsed_time >= 1) {
            printf("\n\n\nCounter ------------- %d \n\n\n", counter);
            start_time = clock();
            counter = 0;
        }

        // sleep_ms(SLEEP_DURATION((float)SAMPLE_RATE));
        sleep_ms(2.1); // Hardcoded sleep to achieve 120 hz sample rate for this setup
    }
    return 0;
}
