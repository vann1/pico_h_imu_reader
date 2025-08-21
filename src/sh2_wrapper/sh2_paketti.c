#include "sh2_paketti.h"
#include <time.h>
#include "sh2_SensorValue.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

// SPI configuration
#define SPI_INST spi1
#define SPI_MISO 8 // GPIO8 (RX)
#define SPI_MOSI 11 // GPIO11 (TX)
#define SPI_SCK 10  // GPIO10 (SCK)
#define SPI_CS 9  // GPIO9 (CS)
#define SPI_BAUD 3000000 // 3 MHz
#define SPI_RESET 13 //GPIO13 (RST)
static void bno_int_handler(uint gpio, uint32_t events);
// TODO - täytä oikeialla
#define BNO_INT 12
#define BNO_P0 14  // GPIO14 (BNO_P0)



sh2_vector_list_t sh2_vector_list = {
    .cursor = 0,
    .data_ready = false
};

static sh2_Hal_t hal;
static bool reset_received = false;
static bno_ready = false;
static int rc;

// HAL: Initialize SPI
static int spi_open(sh2_Hal_t* pInstance) {
    gpio_init(BNO_P0);
    gpio_set_dir(BNO_P0, GPIO_OUT);
    gpio_put(BNO_P0, 1); 

    spi_init(SPI_INST, SPI_BAUD);

    gpio_set_function(SPI_MISO, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCK, GPIO_FUNC_SPI);

    gpio_init(BNO_INT);
    gpio_set_dir(BNO_INT, GPIO_IN);
    gpio_pull_up(BNO_INT);
    gpio_set_irq_enabled_with_callback(BNO_INT, GPIO_IRQ_EDGE_FALL, true, &bno_int_handler);    

    gpio_init(SPI_RESET);
    gpio_set_dir(SPI_RESET, GPIO_OUT);

    // reseting the sensor
    gpio_put(SPI_RESET, 0);
    sleep_ms(1);
    gpio_put(SPI_RESET, 1);

    gpio_init(SPI_CS);
    gpio_set_dir(SPI_CS, GPIO_OUT);
    gpio_put(SPI_CS, 1);

    spi_set_format(SPI_INST, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    return SH2_OK;
}

static void bno_int_handler(uint gpio, uint32_t events) {
    if (gpio == BNO_INT) {
        // printf("bno_int_handler\n");
        bno_ready = true;
    }
}

// HAL: Close SPI
static void spi_close(sh2_Hal_t* pInstance) {
    spi_deinit(SPI_INST);
}
// HAL: Read SPI data
static int spi_read(sh2_Hal_t* pInstance, uint8_t *pData, unsigned len, uint32_t *pTimestamp_us) {
    if (!bno_ready) {
        return 0;
    } else {
        // printf("mhm\n");
        gpio_put(SPI_CS, 0);
        int rc = spi_read_blocking(SPI_INST, 0xFF, pData, len);
        gpio_put(SPI_CS, 1);
        bno_ready = 0;
        if (rc != len) return SH2_ERR;
        pTimestamp_us = to_us_since_boot(get_absolute_time());
        return rc;
    }
}

// HAL: Write SPI data
static int spi_write(sh2_Hal_t pInstance, uint8_t pData, unsigned len) {
        if(!bno_ready) {
            gpio_put(BNO_P0, 0); 
            return 0;
        }
        // printf("mhm2\n");
        gpio_put(SPI_CS, 0);
        gpio_put(BNO_P0, 1); 
        printf("len: %d\n", len);

        int result = spi_write_blocking(SPI_INST, pData, len);
        printf("result: %d\n", result);
        
        gpio_put(SPI_CS, 1);
        bno_ready = false;
        return (result == len) ? result : SH2_ERR;
}
// HAL: Get microsecond timestamp
static uint32_t _get_time_us(sh2_Hal_t pInstance) {
    return to_us_since_boot(get_absolute_time());
}
// Sensor event callback
static void sensor_handler(void *cookie, sh2_SensorEvent_t *event) {
    uint16_t local_cursor = sh2_vector_list.cursor;
    #define l sh2_vector_list.rolling_list
    sh2_SensorValue_t value;
    if (sh2_decodeSensorEvent(&value, event) == SH2_OK) {
        if (value.sensorId == SH2_ROTATION_VECTOR) {
            if (l[sh2_vector_list.cursor] >= SH_2_VECOTR_LIST_ROW_MAX || !sh2_vector_list.data_ready) {
                local_cursor = 0;
            }
            else {
                 local_cursor++;
            }
                l[local_cursor][0] = value.un.rotationVector.real;
                l[local_cursor][1] = value.un.rotationVector.i;
                l[local_cursor][2] = value.un.rotationVector.j;
                l[local_cursor][3] = value.un.rotationVector.k;
                sh2_vector_list.cursor = local_cursor;
                if(sh2_vector_list.data_ready == false)
            {
                sh2_vector_list.data_ready = true;
            }
        }
    } else {
         printf("wrong event?\n");
    }
    #undef l
}
// Async event callback
static void async_handler(void *cookie, sh2_AsyncEvent_t *event) {
    if (event->eventId == SH2_RESET) {
        printf("BNO08x reset\n");
        reset_received = true;
}
}
static void sh2_open_or_halt() {
    // Open SH-2 interface
    rc = sh2_open(&hal, async_handler, NULL);
    if (rc != SH2_OK) {
        printf("SH-2 open failed: %d\n", rc);
        while (1);
}
}

void printEventWrapper(void * cookie, sh2_SensorEvent_t* event) {
    printEvent(event);
}

static void sh2_setSensorCallback_or_halt() {
    rc = sh2_setSensorCallback(sensor_handler, NULL);
    if (rc != SH2_OK) {
        printf("Callback setup failed: %d\n", rc);
        sh2_close();
        while (1);
    }
}
static void sh2_devReset_or_halt() {
    rc = sh2_devReset();
    if (rc != SH2_OK) {
    printf("Reset failed: %d\n", rc);
    sh2_close();
    while (1);
    }
    reset_received = false;
    sleep_ms(300);// Wait for stabilization
}
// Print a sensor event to the console
void printEvent(const sh2_SensorEvent_t * event)
{
    int rc;
    sh2_SensorValue_t value;
    float scaleRadToDeg = 180.0 / 3.14159265358;
    float r, i, j, k, acc_deg, x, y, z;
    float t;
    static int skip = 0;

    if(sh2_vector_list.data_ready == false) 
    {
        sh2_vector_list.data_ready = true; 
    }

    rc = sh2_decodeSensorEvent(&value, event);
    if (rc != SH2_OK) {
        printf("Error decoding sensor event: %d\n", rc);
        return;
    }

    t = value.timestamp / 1000000.0;  // time in seconds.
    switch (value.sensorId) {
        case SH2_RAW_ACCELEROMETER:
            printf("%8.4f Raw acc: %d %d %d time_us:%d\n",
                   (double)t,
                   value.un.rawAccelerometer.x,
                   value.un.rawAccelerometer.y,
                   value.un.rawAccelerometer.z,
                   value.un.rawAccelerometer.timestamp);
            break;

        case SH2_ACCELEROMETER:
            printf("%8.4f Acc: %f %f %f\n",
                   (double)t,
                   (double)value.un.accelerometer.x,
                   (double)value.un.accelerometer.y,
                   (double)value.un.accelerometer.z);
            break;
            
        case SH2_RAW_GYROSCOPE:
            printf("%8.4f Raw gyro: x:%d y:%d z:%d temp:%d time_us:%d\n",
                   (double)t,
                   value.un.rawGyroscope.x,
                   value.un.rawGyroscope.y,
                   value.un.rawGyroscope.z,
                   value.un.rawGyroscope.temperature,
                   value.un.rawGyroscope.timestamp);
            break;
            
        case SH2_ROTATION_VECTOR:
            r = value.un.rotationVector.real;
            i = value.un.rotationVector.i;
            j = value.un.rotationVector.j;
            k = value.un.rotationVector.k;
            acc_deg = scaleRadToDeg * 
                value.un.rotationVector.accuracy;
            printf("%8.4f Rotation Vector: "
                   "r:%0.6f i:%0.6f j:%0.6f k:%0.6f (acc: %0.6f deg)\n",
                   (double)t,
                   (double)r, (double)i, (double)j, (double)k, (double)acc_deg);
            break;
        case SH2_GAME_ROTATION_VECTOR:
            r = value.un.gameRotationVector.real;
            i = value.un.gameRotationVector.i;
            j = value.un.gameRotationVector.j;
            k = value.un.gameRotationVector.k;
            printf("%8.4f GRV: "
                   "r:%0.6f i:%0.6f j:%0.6f k:%0.6f\n",
                   (double)t,
                   (double)r, (double)i, (double)j, (double)k);
            break;
        case SH2_GYROSCOPE_CALIBRATED:
            x = value.un.gyroscope.x;
            y = value.un.gyroscope.y;
            z = value.un.gyroscope.z;
            printf("%8.4f GYRO: "
                   "x:%0.6f y:%0.6f z:%0.6f\n",
                   (double)t,
                   (double)x, (double)y, (double)z);
            break;
        case SH2_GYROSCOPE_UNCALIBRATED:
            x = value.un.gyroscopeUncal.x;
            y = value.un.gyroscopeUncal.y;
            z = value.un.gyroscopeUncal.z;
            printf("%8.4f GYRO_UNCAL: "
                   "x:%0.6f y:%0.6f z:%0.6f\n",
                   (double)t,
                   (double)x, (double)y, (double)z);
            break;
        case SH2_GYRO_INTEGRATED_RV:
            // These come at 1kHz, too fast to print all of them.
            // So only print every 10th one
            skip++;
            if (skip == 10) {
                skip = 0;
                r = value.un.gyroIntegratedRV.real;
                i = value.un.gyroIntegratedRV.i;
                j = value.un.gyroIntegratedRV.j;
                k = value.un.gyroIntegratedRV.k;
                x = value.un.gyroIntegratedRV.angVelX;
                y = value.un.gyroIntegratedRV.angVelY;
                z = value.un.gyroIntegratedRV.angVelZ;
                printf("%8.4f Gyro Integrated RV: "
                       "r:%0.6f i:%0.6f j:%0.6f k:%0.6f x:%0.6f y:%0.6f z:%0.6f\n",
                       (double)t,
                       (double)r, (double)i, (double)j, (double)k,
                       (double)x, (double)y, (double)z);
            }
            break;
        case SH2_IZRO_MOTION_REQUEST:
            printf("IZRO Request: intent:%d, request:%d\n",
                   value.un.izroRequest.intent,
                   value.un.izroRequest.request);
            break;
        case SH2_SHAKE_DETECTOR:
            printf("Shake Axis: %c%c%c\n", 
                   (value.un.shakeDetector.shake & SHAKE_X) ? 'X' : '.',
                   (value.un.shakeDetector.shake & SHAKE_Y) ? 'Y' : '.',
                   (value.un.shakeDetector.shake & SHAKE_Z) ? 'Z' : '.');

            break;
        case SH2_STABILITY_CLASSIFIER:
            printf("Stability Classification: %d\n",
                   value.un.stabilityClassifier.classification);
            break;
        case SH2_STABILITY_DETECTOR:
            printf("Stability Detector: %d\n",
                   value.un.stabilityDetector.stability);
            break;
        default:
            printf("Unknown sensor: %d\n", value.sensorId);
            break;
    }
}

static void sh2_setSensorConfig_or_halt() {
    // Enable rotation vector (100 Hz)
    sh2_SensorConfig_t config;
    memset(&config, 0, sizeof(config));
    float test = (1.0f/(float)SAMPLE_RATE) * 1000000.0f;
    config.reportInterval_us = test; // 10 ms = 100 Hz
    printf("Setsensorfocnfg\n");
    rc = sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config);
    printf("Rc: %d\n", rc);
    if (rc != SH2_OK) {
        printf("Config failed: %d\n", rc);
        sh2_close();
        while (1);
}
}
static void initialize_HALL() {
    hal.open = spi_open;
    hal.close = spi_close;
    hal.read = spi_read;
    hal.write = spi_write;
    hal.getTimeUs = _get_time_us;
}
static void wait_for_reset_or_halt() {
    absolute_time_t timeout = make_timeout_time_ms(2000);
    while (!reset_received) {
    sh2_service();
    if (time_reached(timeout)) {
        printf("Timeout waiting for reset event!\n");
        sh2_close();
        while(1);
    }
    sleep_ms(10);
    }
}
void setup_sh2_service() {
    printf("pääskö tänne \n");

    initialize_HALL();
    printf("pääskö tänne 1\n");

    sh2_open_or_halt();
    printf("pääskö tänne 2\n");

    sh2_setSensorCallback_or_halt();
    printf("pääskö tänne 3\n");
 
    // // most likely unneeded because sh2 open already does software reset
    // sh2_devReset_or_halt();
    // printf("pääskö tänne 4\n");

    // wait_for_reset_or_halt();
    // printf("pääskö tänne 5\n");

    sh2_setSensorConfig_or_halt();
    printf("pääskö tänne 4\n");
}
void read_super_sensor() {
    sh2_service();
}