#include "InitFusion.h"

void initialize_sensors_values(Sensor* sensors) {
    for (int i = 0; i<SENSOR_COUNT;i++) {
        sensors[i].gyroscope.axis.x = 0.0f;
        sensors[i].gyroscope.axis.y = 0.0f;
        sensors[i].gyroscope.axis.z = 0.0f;
        sensors[i].gyroscope_old.axis.x = 0.0f;
        sensors[i].gyroscope_old.axis.y = 0.0f;
        sensors[i].gyroscope_old.axis.z = 0.0f;

        sensors[i].accelerometer.axis.x = 0.0f;
        sensors[i].accelerometer.axis.y = 0.0f;
        sensors[i].accelerometer.axis.z = 0.0f;
        sensors[i].accelerometer_old.axis.x = 0.0f;
        sensors[i].accelerometer_old.axis.y = 0.0f;
        sensors[i].accelerometer_old.axis.z = 0.0f;
    }
}

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