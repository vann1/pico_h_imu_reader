#include "ism330dlc.h"
#include "i2c_helpers.h"
#include "tusb.h"
#include "Fusion.h"
#include <stdbool.h>
#include <stdio.h>
#include <time.h>

#define SAMPLE_RATE (20) // replace this with actual sample rate
#define SLEEP_DURATION(hz) (float)(1.0f/hz * 1000.0f)
#define SENSOR_COUNT 4
#define CHANNEL_COUNT 2

typedef struct FusionCalibration {
    FusionMatrix gyroscopeMisalignment;
    FusionVector gyroscopeSensitivity;
    FusionVector gyroscopeOffset;
    FusionMatrix accelerometerMisalignment;
    FusionVector accelerometerSensitivity;
    FusionVector accelerometerOffset;
} FusionCalibration;

typedef struct Sensor {
    FusionCalibration calibration;
    FusionAhrs ahrs;
    FusionOffset offset;
    FusionAhrsSettings settings;
    FusionVector accelerometer;
    FusionVector gyroscope;
    clock_t previousTimestamp;
    clock_t timestamp;
} Sensor;

void initialize_calibrations(Sensor* sensors) {
    for (int i = 0; i<SENSOR_COUNT;i++) {
        sensors[i].calibration.gyroscopeMisalignment = (FusionMatrix){ 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
        sensors[i].calibration.gyroscopeSensitivity = (FusionVector){1.0f, 1.0f, 1.0f};
        sensors[i].calibration.gyroscopeOffset = (FusionVector) {0.0f, 0.0f, 0.0f};
        sensors[i].calibration.accelerometerMisalignment = (FusionMatrix) {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
        sensors[i].calibration.accelerometerSensitivity = (FusionVector) {1.0f, 1.0f, 1.0f};
        sensors[i].calibration.accelerometerOffset = (FusionVector) {0.0f, 0.0f, 0.0f};
    }
}

void initialize_algos(Sensor* sensors) {
    for (int i = 0; i<SENSOR_COUNT;i++) {
        FusionOffsetInitialise(&sensors[i].offset, SAMPLE_RATE);
        FusionAhrsInitialise(&sensors[i].ahrs);
            // Set AHRS algorithm settings
        sensors[i].settings = (FusionAhrsSettings){
                .convention = FusionConventionNwu,
                .gain = 0.5f,
                .gyroscopeRange = 250.0f, /* replace this with actual gyroscope range in degrees/s */
                .accelerationRejection = 10.0f,
                .magneticRejection = 10.0f,
                .recoveryTriggerPeriod = 5 * SAMPLE_RATE, /* 5 seconds */
        };
        FusionAhrsSetSettings(&sensors[i].ahrs, &sensors[i].settings);
    }
}

void read_all_sensors(Sensor* sensors) {
    int index = 0;
   for (int i = 0; i < CHANNEL_COUNT; i++) {
        if(i == 0) {
            ism330dhcx_read_accelerometer(I2C_PORT_0,0x4A, &sensors[index].accelerometer);
            ism330dhcx_read_gyro(I2C_PORT_0,0x4A, &sensors[index].gyroscope);
            sensors[index].timestamp = clock();
            index++;
            ism330dhcx_read_accelerometer(I2C_PORT_0,ISM330DHCX_ADDR_DO_HIGH, &sensors[index].accelerometer);
            ism330dhcx_read_gyro(I2C_PORT_0,ISM330DHCX_ADDR_DO_HIGH, &sensors[index].gyroscope);
            sensors[index].timestamp = clock();
            index++;
        } else if (i == 1) {
            ism330dhcx_read_accelerometer(I2C_PORT_1,ISM330DHCX_ADDR_DO_LOW, &sensors[index].accelerometer);
            ism330dhcx_read_gyro(I2C_PORT_1,ISM330DHCX_ADDR_DO_LOW, &sensors[index].gyroscope);
            sensors[index].timestamp = clock();
            index++;
            ism330dhcx_read_accelerometer(I2C_PORT_1,ISM330DHCX_ADDR_DO_HIGH, &sensors[index].accelerometer);
            ism330dhcx_read_gyro(I2C_PORT_1,ISM330DHCX_ADDR_DO_HIGH, &sensors[index].gyroscope);
            sensors[index].timestamp = clock();
        }
   }
}


int main() {
    stdio_init_all();
    while (!tud_cdc_connected()) {
        sleep_ms(100);
    }

    int result = setup_I2C_pins();
    if (result != 1) {
		printf("I2C pin setup failed");
		return 1;
    }

	initialize_sensors();

    Sensor sensors[SENSOR_COUNT];
    initialize_calibrations(sensors); 
    initialize_algos(sensors);   
    
    printf("Starting data stream...\n");
    //i2c_scan(I2C_PORT_1);

    // This loop should repeat each time new gyroscope data is available
    while (true) {
        read_all_sensors(sensors);
        // Apply calibration
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

        }
        printf("---\n");
        // const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        // const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);
        // const FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs);
        // printf("y%0.1fyp%0.1fpr%0.1fr\n",
        //        euler.angle.yaw, euler.angle.pitch, euler.angle.roll);

        sleep_ms(SLEEP_DURATION((float)SAMPLE_RATE));
    }
    return 0;
}
