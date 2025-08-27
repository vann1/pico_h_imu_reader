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
#include <stdlib.h>
extern sh2_vector_list_t sh2_vector_list;


#define TIME_SLEEP (1000.0f/(float)SAMPLE_RATE)

float sensors_data[SENSOR_COUNT][4];

#define LPF_ALPHA 0.1f

#define ARRAY_SIZE (1024)
#define RESULT_COUNT (64)
#define SETTINGS_BUF_LEN (256)

enum settings_enum_e {
    S_SENSOR_COUNT,
    S_LPF_ENABLED,
    S_LPF_ALPHA,
    S_SAMPLE_RATE
};
typedef enum settings_enum_e settings_enum;
settings_enum settings_option;

typedef struct imu_reader_settings_t {
    int sensorCount;
    int lpfEnabled;
    float lpf_alpha;
    int sampleRate;
} imu_reader_settings_t;
static imu_reader_settings_t imu_reader_settings = {0};

typedef struct data_fluctuation_t {
    float gyro[3][ARRAY_SIZE];
    float results[2][RESULT_COUNT];
    int cursor;
} data_fluctuation_t;

data_fluctuation_t benchmark = {.cursor = 0};

float apply_lpf(float new_val, float old_value) {
    return old_value*(1.0f-LPF_ALPHA) + LPF_ALPHA*new_val;
}

void extract_part(const char* part, const char* buf, settings_enum setting) {
        char *pos = strstr(buf, part);
        
        if (pos == NULL) {
            printf("Error while extracting part %s not found in the buffer\n", part);
            while(1);
        } else {
            char part_buffer[64];//SR=jotain|
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
            switch (setting) {
                case S_SENSOR_COUNT:
                    imu_reader_settings.sensorCount = atoi(part_buffer);
                    break;
                case S_LPF_ENABLED:
                    imu_reader_settings.lpfEnabled = atoi(part_buffer);
                    break;
                case S_LPF_ALPHA:
                    sscanf(part_buffer, "%f", &imu_reader_settings.lpf_alpha);
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
    extract_part("SR=", buf, settings_option);

    printf("Settings: Sensor Count: %d; LPF Enabled %d; LPF_ALPHA %f; Sample Rate: %d\n",imu_reader_settings.sensorCount, imu_reader_settings.lpfEnabled, imu_reader_settings.lpf_alpha, imu_reader_settings.sampleRate);
}

void set_settings() {
    #undef SENSOR_COUNT
    #define SENSOR_COUNT (imu_reader_settings.sensorCount)
    #undef LPF_ENABLED
    #define LPF_ENABLED (imu_reader_settings.lpfEnabled)
    #undef LPF_ALPHA
    #define LPF_ALPHA (imu_reader_settings.lpf_alpha)
    #undef SAMPLE_RATE
    #define SAMPLE_RATE (imu_reader_settings.sampleRate)
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

                printf("Received settings: %s\n", buf);
                excract_settings(buf);
                set_settings();
                break;
            }
        }
        sleep_ms(300); // Avoid tight loop
    }
}

void print_output_data (void) {
    for (int i = 0; i < SENSOR_COUNT; i++) {
        printf("w%d: %.4f, x%d: %.4f, y%d: %.4f, z%d: %.4f | ",i, sensors_data[i][0],i, sensors_data[i][1],i, sensors_data[i][2],i, sensors_data[i][3] );
    }
    printf("\n");
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
    uint64_t start_time = time_us_64();
    while (true) {
        read_all_sensors(sensors);
        for (int i=0; i<SENSOR_COUNT;i++) {
            sensors[i].gyroscope_old.axis.x = apply_lpf(sensors[i].gyroscope.axis.x, sensors[i].gyroscope_old.axis.x);
            sensors[i].gyroscope_old.axis.y = apply_lpf(sensors[i].gyroscope.axis.y, sensors[i].gyroscope_old.axis.y);
            sensors[i].gyroscope_old.axis.z = apply_lpf(sensors[i].gyroscope.axis.z, sensors[i].gyroscope_old.axis.z);

            sensors[i].accelerometer_old.axis.x = apply_lpf(sensors[i].accelerometer.axis.x, sensors[i].accelerometer_old.axis.x);
            sensors[i].accelerometer_old.axis.y = apply_lpf(sensors[i].accelerometer.axis.y, sensors[i].accelerometer_old.axis.y);
            sensors[i].accelerometer_old.axis.z = apply_lpf(sensors[i].accelerometer.axis.z, sensors[i].accelerometer_old.axis.z);

            sensors[i].gyroscope = FusionCalibrationInertial(sensors[i].gyroscope_old, sensors[i].calibration.gyroscopeMisalignment, sensors[i].calibration.gyroscopeSensitivity, sensors[i].calibration.gyroscopeOffset);
            sensors[i].accelerometer = FusionCalibrationInertial(sensors[i].accelerometer_old, sensors[i].calibration.accelerometerMisalignment, sensors[i].calibration.accelerometerSensitivity, sensors[i].calibration.accelerometerOffset);
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
        sleep_ms(TIME_SLEEP-6.252f); // 120hz
    }
    return 0;
}