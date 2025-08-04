
#ifndef ISM330DHCL_REGISTERS_H
#define ISM330DHCL_REGISTERS_H
// ISM330DHCX Register addresses
#define WHO_AM_I 0x0F
#define CTRL3_C 0x12 // THIS IS ONLY FOR TESTING, REMOVE AFTER
#define CTRL1_XL 0x10  // Accelerometer control
#define CTRL2_G 0x11   // Gyroscope control
#define STATUS_REG 0x1E
#define OUTX_L_G 0x22  // Gyroscope output registers
#define OUTX_L_XL 0x28 // Accelerometer output registers
#endif
