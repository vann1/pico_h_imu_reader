#ifndef INITFUSION_H
#define INITFUSION_H

#include "Fusion.h"
#include "i2c_helpers.h"
#include <time.h>
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
    FusionVector magnetometer;
} Sensor;

void initialize_calibrations(Sensor* sensors);
void initialize_algos(Sensor* sensors);
#endif //INITFUSION_H


