#include "ism330dlc.h"
#include "i2c_helpers.h"
#include "tusb.h"
#include "Fusion.h"
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <sh2_paketti.h>
#include <math.h>
#include "read.h"
extern sh2_vector_list_t sh2_vector_list;

#define SLEEP_DURATION(hz) (float)(1.0f/hz * 1000.0f)
#define SAMPLE_COUNT (128)
#define RESULT_COUNT (10)

float sensors_data[SENSOR_COUNT][4];

typedef struct data_fluctuation_t {
    float gyro[SAMPLE_COUNT][3];
    float accel[SAMPLE_COUNT][3];
    float results[RESULT_COUNT][2][3];
} data_fluctuation_t;

data_fluctuation_t benchmark;

void calculate_results_avg(float results[][2][3], float answers[][3]) {
    float avgs[2][3] = {{0}};
    for (int i = 0; i < RESULT_COUNT; i++) {
        // gyro
        avgs[0][0] += results[i][0][0];
        avgs[0][1] += results[i][0][1];
        avgs[0][2] += results[i][0][2];
        //Accel
        avgs[1][0] += results[i][1][0];
        avgs[1][1] += results[i][1][1];
        avgs[1][2] += results[i][1][2];
    }
    answers[0][0] = avgs[0][0] / (float)RESULT_COUNT;
    answers[0][1] = avgs[0][1] / (float)RESULT_COUNT;
    answers[0][2] = avgs[0][2] / (float)RESULT_COUNT;

    answers[1][0] = avgs[1][0] / (float)RESULT_COUNT;
    answers[1][1] = avgs[1][1] / (float)RESULT_COUNT;
    answers[1][2] = avgs[1][2] / (float)RESULT_COUNT;
}

void print_avg_fluctuations(float results[][3], int sensorId) {
    printf("Avg fluctuations for sensor Id of %d a gyroscope and accelerometer from a pool of %d results and each containing %d sample counts\n",sensorId, RESULT_COUNT, SAMPLE_COUNT);
    printf("Gyroscope: x: %.4f; y: %.4f; z: %.4f\n", results[0][0], results[0][1], results[0][2]);
    printf("Accelerometer: x: %.4f; y: %.4f; z: %.4f\n", results[1][0], results[1][1], results[1][2]);
}

void calculate_avg_fluctuation(float gyro[][3], float* result) {
    float avg_diff_sum[3]  = {0};
    for (int i = 0; i < SAMPLE_COUNT-1; i++) {
        avg_diff_sum[0] += fabs(gyro[i][0] - gyro[i+1][0]);
        avg_diff_sum[1] += fabs(gyro[i][1] - gyro[i+1][1]);
        avg_diff_sum[2] += fabs(gyro[i][2] - gyro[i+1][2]);
    }
    result[0] = avg_diff_sum[0] / (float)(SAMPLE_COUNT-1);
    result[1] = avg_diff_sum[1] / (float)(SAMPLE_COUNT-1);
    result[2] = avg_diff_sum[2] / (float)(SAMPLE_COUNT-1);
}

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

float apply_lpf(float new_val, float old_value, bool first) {
    float alpha = 0.1f;
    if (first) {
        return new_val;
    } else {
        return old_value*(1-alpha) + alpha*new_val;
    }
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
    // initialize_calibrations(sensors); 
    // initialize_algos(sensors);   
    
    int counter = 0;
    uint64_t start_time = time_us_64();
    while (true) {
        read_all_sensors(sensors);
        for (int i=0; i<SENSOR_COUNT;i++) {
            sensors[i].gyroscope = FusionCalibrationInertial(sensors[i].gyroscope, sensors[i].calibration.gyroscopeMisalignment, sensors[i].calibration.gyroscopeSensitivity, sensors[i].calibration.gyroscopeOffset);
            sensors[i].accelerometer = FusionCalibrationInertial(sensors[i].accelerometer, sensors[i].calibration.accelerometerMisalignment, sensors[i].calibration.accelerometerSensitivity, sensors[i].calibration.accelerometerOffset);
            sensors[i].gyroscope = FusionOffsetUpdate(&sensors[i].offset, sensors[i].gyroscope);

            const float deltaTime = (float) (sensors[i].timestamp - sensors[i].previousTimestamp) / 1e6f;
            sensors[i].previousTimestamp = sensors[i].timestamp;
            // FusionAhrsUpdateNoMagnetometer(&sensors[i].ahrs, sensors[i].gyroscope, sensors[i].accelerometer, deltaTime);

            FusionAhrsUpdateExternalHeading(&sensors[i].ahrs, sensors[i].gyroscope, sensors[i].accelerometer, 0.0f, deltaTime);
            const FusionQuaternion quat = FusionAhrsGetQuaternion(&sensors[i].ahrs);
            sensors_data[i][0] = quat.element.w;
            sensors_data[i][1] = quat.element.x;
            sensors_data[i][2] = quat.element.y;
            sensors_data[i][3] = quat.element.z;           
        }
        print_output_data();
        uint64_t loop_end = time_us_64();
        sleep_ms(2); // 120hz
    }
    return 0;
}
