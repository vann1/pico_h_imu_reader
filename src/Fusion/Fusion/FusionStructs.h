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
<<<<<<< HEAD
    FusionVector accelerometer_old;
    FusionVector gyroscope;
    FusionVector gyroscope_old;
=======
<<<<<<< HEAD
    FusionVector accelerometer_old;
    FusionVector gyroscope;
    FusionVector gyroscope_old;
=======
    FusionVector gyroscope;
>>>>>>> fc3668f (Fresh start)
>>>>>>> d205c2c (?)
    uint64_t previousTimestamp;
    uint64_t timestamp;
    FusionVector magnetometer;
} Sensor;

#endif // FUSIONSTRUCTS_H