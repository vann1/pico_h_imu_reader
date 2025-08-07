#include "ism330dlc.h"
#include "i2c_helpers.h"
#include "tusb.h"

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
    
    printf("Starting data stream...\n");
    i2c_scan(I2C_PORT_1);

    while (1) {

        sleep_ms(1000);
        ism330dhcx_read_gyro(I2C_PORT_1,ISM330DHCX_ADDR_DO_LOW);
    }
    
    return 0;
}
