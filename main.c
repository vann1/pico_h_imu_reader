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
#include <stdlib.h>
#include <string.h>

extern sh2_vector_list_t sh2_vector_list;

#define SLEEP_DURATION(hz) (float)(1.0f/hz * 1000.0f)
#define SAMPLE_COUNT (128)
#define RESULT_COUNT (10)
#define SETTINGS_BUF_LEN (256)

float sensors_data[SENSOR_COUNT][4];

enum settings_enum_e {
    S_SENSOR_COUNT,
    S_LPF_ENABLED,
    S_LPF_ALPHA,
    S_SAMPLE_RATE
};
typedef enum settings_enum_e settings_enum;
settings_enum settings_option;

typedef struct data_fluctuation_t {
    float gyro[SAMPLE_COUNT][3];
    float accel[SAMPLE_COUNT][3];
    float results[RESULT_COUNT][2][3];
} data_fluctuation_t;

typedef struct imu_reader_settings_t {
    int sensorCount;
    int lpfEnabled;
    float lpf_alpha;
    int sampleRate;
} imu_reader_settings_t;
static imu_reader_settings_t imu_reader_settings = {0};
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
    float alpha = 0.05f;
    if (first) {
        return new_val;
    } else {
        return old_value*(1-alpha) + alpha*new_val;
    }
}

void extract_part(const char* part, const char* buf, settings_enum setting) {
        char *pos = strstr(buf, part);

        if (pos == NULL) {
            printf("Error while extracting part %s not found in the buffer\n", part);
            while(1);
        } else {
            printf("Found part %s!\n", part);
            char part_buffer[64];
            memset(part_buffer, 0, sizeof(part_buffer));
            int cursor = 0;
            int part_index = (pos - buf) + strlen(part);
            for (int j=part_index;j<SETTINGS_BUF_LEN-part_index; j++) {
                if (buf[j] != '|') {
                    part_buffer[cursor++] = buf[j];
                    continue;
                }
                part_buffer[cursor++] = '\0';
                break;
            }
            printf("Part value extracted: %s\n", part_buffer);
            switch (setting) {
                case S_SENSOR_COUNT:
                    imu_reader_settings.sensorCount = atoi(part_buffer);
                    break;
                case S_LPF_ENABLED:
                    imu_reader_settings.lpfEnabled = atoi(part_buffer);
                    break;
                case S_LPF_ALPHA:
                    imu_reader_settings.lpf_alpha = atof(part_buffer);
                    break;
                case S_SAMPLE_RATE:
                    imu_reader_settings.sampleRate = atoi(part_buffer);
                    break;
                default:
                    printf("Error: Wrong setting %d\n", setting);
                    while(1);
            }
            printf("Part %s found and saved: %s\n", part, part_buffer);
        }
}

void excract_settings(const char* buf) {
    settings_option = S_SENSOR_COUNT;
    extract_part("SC=", buf, settings_option);

    settings_option = S_LPF_ENABLED;
    extract_part("LPF_ENABLED=", buf, settings_option);

    settings_option = S_LPF_ALPHA;
    extract_part("LPF_ALPHA=", buf, settings_option);

    settings_option = S_SAMPLE_RATE;
    printf("Starting to look for SR= part\n");
    extract_part("SR=", buf, settings_option);

    printf("Settings: Sensor Count: %d; LPF Enabled %d; LPF_ALPHA %f; Sample Rate: %d\n",imu_reader_settings.sensorCount, imu_reader_settings.lpfEnabled, imu_reader_settings.lpf_alpha, imu_reader_settings.sampleRate);
}

void set_settings() {
    #define SENSOR_COUNT imu_reader_settings.sensorCount
    #define LPF_ENABLED imu_reader_settings.lpfEnabled
    #define LPF_ALPHA imu_reader_settings.lpf_alpha
    #define SAMPLE_RATE imu_reader_settings.sampleRate
}

// Waits for the user to send in some settings through the serial port
void wait_for_settings() {
    char buf[SETTINGS_BUF_LEN];
    while (1) {
        if (tud_cdc_available()) {
            uint32_t count = tud_cdc_read(buf, sizeof(buf)-1);
            if (count > 0) {
                buf[count] = '\0';
                // Process input (e.g., echo back)

                printf("Received: %s\n", buf);
                excract_settings(buf);
                set_settings();
                break;
            }
        }
        sleep_ms(1000); // Avoid tight loop
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
    wait_for_settings();

	initialize_sensors();

    Sensor sensors[SENSOR_COUNT];
    // initialize_calibrations(sensors); 
    // initialize_algos(sensors);   
    
    int counter = 0;
    printf("Starting to benchmark...\n");
    for (int i = 0; i<RESULT_COUNT;i++)
    {
        printf("Starting benchmark iteration: %d\n", i);

        float previous_values[2][3] = {{0}};
        bool first = true;

        for (int j = 0; j<SAMPLE_COUNT;j++) {
            read_all_sensors(sensors);
            benchmark.gyro[j][0] = apply_lpf(sensors[0].gyroscope.axis.x, previous_values[0][0], first);
            benchmark.gyro[j][1] = apply_lpf(sensors[0].gyroscope.axis.y, previous_values[0][1], first);
            benchmark.gyro[j][2] = apply_lpf(sensors[0].gyroscope.axis.z, previous_values[0][2], first);
            
            benchmark.accel[j][0] = apply_lpf(sensors[0].accelerometer.axis.x, previous_values[1][0], first);
            benchmark.accel[j][1] = apply_lpf(sensors[0].accelerometer.axis.y, previous_values[1][1], first);
            benchmark.accel[j][2] = apply_lpf(sensors[0].accelerometer.axis.z, previous_values[1][2], first);

            // Gyro
            previous_values[0][0] = benchmark.gyro[j][0];
            previous_values[0][1] = benchmark.gyro[j][1];
            previous_values[0][2] = benchmark.gyro[j][2];

            //Accel 
            previous_values[1][0] = benchmark.accel[j][0];
            previous_values[1][1] = benchmark.accel[j][1];
            previous_values[1][2] = benchmark.accel[j][2];

            first = false;
            sleep_ms(10);
        }
        printf("Benchmark iteration %d is done\n", i);
        float gyro_results[3]= {0};
        float accel_results[3] = {0};
        calculate_avg_fluctuation(benchmark.gyro, gyro_results);
        calculate_avg_fluctuation(benchmark.accel, accel_results);

        //Gyro
        benchmark.results[i][0][0] = gyro_results[0];
        benchmark.results[i][0][1] = gyro_results[1];
        benchmark.results[i][0][2] = gyro_results[2];

        //accel
        benchmark.results[i][1][0] = accel_results[0];
        benchmark.results[i][1][1] = accel_results[1];
        benchmark.results[i][1][2] = accel_results[2];
        }
    
    float final_answers[2][3] = {{0}};
    calculate_results_avg(benchmark.results, final_answers);
    print_avg_fluctuations(final_answers, 0);

    while(1);
    return 0;
}
