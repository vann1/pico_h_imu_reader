#ifndef FUSIONSTRUCTS_H
#define FUSIONSTRUCTS_H

#include "Fusion.h"
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

#endif // FUSIONSTRUCTS_H