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

float sensors_data[SENSOR_COUNT+1][4];


void print_output_data (void) {
    for (int i = 0; i < SENSOR_COUNT+1; i++) {
        printf("w%d: %.2f, x%d: %.2f, y%d: %.2f, z%d: %.2f | ",i, sensors_data[i][0],i, sensors_data[i][1],i, sensors_data[i][2],i, sensors_data[i][3] );
    }
    printf("\n");
}


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
    int counter = 0;
    clock_t start_time = clock();

    while (true) {
        read_all_sensors(sensors);
        for (int i=0; i<SENSOR_COUNT;i++) {
            sensors[i].gyroscope = FusionCalibrationInertial(sensors[i].gyroscope, sensors[i].calibration.gyroscopeMisalignment, sensors[i].calibration.gyroscopeSensitivity, sensors[i].calibration.gyroscopeOffset);
            sensors[i].accelerometer = FusionCalibrationInertial(sensors[i].accelerometer, sensors[i].calibration.accelerometerMisalignment, sensors[i].calibration.accelerometerSensitivity, sensors[i].calibration.accelerometerOffset);
            sensors[i].gyroscope = FusionOffsetUpdate(&sensors[i].offset, sensors[i].gyroscope);

            const float deltaTime = (float) (sensors[i].timestamp - sensors[i].previousTimestamp) / (float) CLOCKS_PER_SEC;
            sensors[i].previousTimestamp = sensors[i].timestamp;

            FusionAhrsUpdateNoMagnetometer(&sensors[i].ahrs, sensors[i].gyroscope, sensors[i].accelerometer, deltaTime);
            const FusionQuaternion quat = FusionAhrsGetQuaternion(&sensors[i].ahrs);
            sensors_data[i][0] = quat.element.w;
            sensors_data[i][1] = quat.element.x;
            sensors_data[i][2] = quat.element.y;
            sensors_data[i][3] = quat.element.z;           
        }
        read_super_sensor();
        if (sh2_vector_list.data_ready == false) {
            sleep_ms(1000);
            sh2_vector_list.data_ready == true;
        } 
        else {
            sensors_data[SENSOR_COUNT][0] = sh2_vector_list.rolling_list[sh2_vector_list.cursor][0];
            sensors_data[SENSOR_COUNT][1] = sh2_vector_list.rolling_list[sh2_vector_list.cursor][1];
            sensors_data[SENSOR_COUNT][2] = sh2_vector_list.rolling_list[sh2_vector_list.cursor][2];
            sensors_data[SENSOR_COUNT][3] =sh2_vector_list.rolling_list[sh2_vector_list.cursor][3];
            // print_output_data();
            // printf("---\n");
            sleep_ms(SLEEP_DURATION((float)SAMPLE_RATE));
        }   
        float elapsed_time = clock() - start_time;
        counter++;
        if ((elapsed_time/ (float) CLOCKS_PER_SEC) >= 1) {
            printf("adsada: %d\n", (elapsed_time/ (float) CLOCKS_PER_SEC));
            printf("cauntteri: %d", counter);
            counter = 0;
            start_time = clock();
        }
    }
    return 0;
}
