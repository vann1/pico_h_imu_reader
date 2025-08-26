#ifndef ISM330DLC_CONFIG_H
#define ISM330DLC_CONFIG_H

// Accelerometer
#define XL_ODR 0xA0 // 6.66hz
#define XL_G_RANGE_MASK 0x00 // 2g
#define XL_G_RANGE 2

// Gyroscope
#define G_DPS_RANGE_MASK 0x2 // 125dps
#define G_DPS_RANGE 125
#define G_ODR 0x50 // 208 hz
#endif //ISM330DLC_CONFIG_H
