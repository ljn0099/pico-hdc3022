#include "pico/stdlib.h"
#include <stdio.h>

#include "hardware/i2c.h"
#include "sensor/hdc3022.h"

#define I2C_BUS i2c0
#define I2C_SDA 16
#define I2C_SCL 17

int main() {
    stdio_init_all();

    i2c_init(I2C_BUS, 100000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    hdc3022_t hdc3022;
    if (!hdc3022_init_struct(&hdc3022, I2C_BUS, HDC3022_DEFAULT_ADDR, HDC3022_AUTO_MEAS_NONE,
                             true)) { // Auto detect default mode
        printf("Error to init the struct\n");
        return -1;
    }

    float temp, humidity;

    if (!hdc3022_readout_manual_meas(&hdc3022, HDC3022_MANUAL_MEAS_LPM_0, HDC3022_TEMP_CELSIUS,
                                     &temp, &humidity)) { // Manual readout
        printf("Error doing the manual readout\n");
        return -1;
    }
    printf("Temperature %f ºC, Humidity %f\n", temp, humidity);

    if (!hdc3022_set_auto_meas_mode(&hdc3022,
                                    HDC3022_AUTO_MEAS_1X_1S_LPM_0)) { // Set auto meas mode
        printf("Error setting the auto meas mode\n");
        return -1;
    }
    sleep_ms(1200); // Wait for the sensor to prepare the reading

    for (int i = 0; i < 6; i++) {
        if (!hdc3022_readout_auto_meas(&hdc3022, HDC3022_AUTO_MEAS_READOUT, HDC3022_TEMP_CELSIUS,
                                       &temp, &humidity)) { // Do a readout
            printf("Error doing the readout\n");
            return -1;
        }
        printf("Temp: %f, RH: %f\n", temp, humidity);
        sleep_ms(1200); // Wait for the sensor to prepare the reading
    }

    if (!hdc3022_soft_reset(&hdc3022)) {
        printf("Error reseting\n");
        return -1;
    }

    // Set again the default mode
    if (!hdc3022_init_struct(&hdc3022, I2C_BUS, HDC3022_DEFAULT_ADDR, HDC3022_AUTO_MEAS_NONE,
                             true)) { // Auto detect default mode
        printf("Error to init the struct\n");
        return -1;
    }

    // Read the nist ID
    uint8_t nist_id[6];

    if (hdc3022_read_nist_id(&hdc3022, nist_id)) {
        printf("NIST ID: ");
        for (int i = 0; i < 6; i++) {
            printf("%02X", nist_id[i]);
            if (i < 5)
                printf(":");
        }
        printf("\n");
    }
    else {
        printf("Error leyendo el NIST ID\n");
    }

    // Read the manufacturer ig
    uint16_t manufacturerId;

    manufacturerId = hdc3022_read_manufacturer_id(&hdc3022);

    printf("Manufacturer id: %x\n", manufacturerId);

    hdc3022_read_alert(&hdc3022, HDC3022_ALERT_THRESHOLD_SET_HIGH_READ, HDC3022_TEMP_CELSIUS, &temp,
                       &humidity);
    printf("Read alert temp: %f, rh: %f\n", temp, humidity);

    sleep_ms(500);

    // Write -5, 2 offsets to the EEPROM
    if (!hdc3022_write_offsets(&hdc3022, HDC3022_TEMP_CELSIUS, -5, 2))
        printf("Erorr writting offsets\n");

    // Read the offsets from the EEPROM
    if (!hdc3022_read_offsets(&hdc3022, HDC3022_TEMP_CELSIUS, &temp, &humidity)) {
        printf("Erorr reading offsets\n");
        return -1;
    }

    printf("Offsets: temp %f, humidity %f\n", temp, humidity);

    // Reset offsets to the default
    if (!hdc3022_write_offsets(&hdc3022, HDC3022_TEMP_CELSIUS, 0, 0)) {
        printf("Erorr writting offsets\n");
        return -1;
    }

    // Read the default mode

    hdc3022_DefaultMeasMode_t defaultModeRaw;

    if (!hdc3022_read_default_meas_state(&hdc3022, &defaultModeRaw))
        printf("Error reading default mode\n");

    printf("Readed value (raw): 0x%04X\n", defaultModeRaw);

    return 0;
}
