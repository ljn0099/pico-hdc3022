#include "hardware/i2c.h"
#include "hdc3022_private.h"
#include "pico/stdlib.h"
#include "sensor/hdc3022.h"
#include <math.h>

bool hdc3022_init_struct(hdc3022_t *hdc3022, i2c_inst_t *i2c, uint8_t i2cAddress,
                         hdc3022_AutoMeasMode_t autoMeasConfig, bool checkDefaultMode) {
    hdc3022->i2c = i2c;
    hdc3022->i2cAddress = i2cAddress;

    if (!checkDefaultMode) {
        if (autoMeasConfig != HDC3022_AUTO_MEAS_NONE) {
            hdc3022->autoMeasConfig = autoMeasConfig;
            hdc3022->autoModeEnabled = true;
        }
        else {
            hdc3022->autoModeEnabled = false;
        }
    }
    else {
        hdc3022_DefaultMeasMode_t defaultMode;
        if (!hdc3022_read_default_meas_state(hdc3022, &defaultMode))
            return false;

        if (defaultMode == HDC3022_MANUAL_CONFIG) {
            hdc3022->autoModeEnabled = false;
            return true;
        }

        hdc3022_AutoMeasMode_t defaultAutoMeasConfig;
        defaultAutoMeasConfig = hdc3022_configMode_to_measMode(defaultMode);

        if (defaultAutoMeasConfig == HDC3022_AUTO_MEAS_NONE)
            return false;

        hdc3022->autoModeEnabled = true;
        hdc3022->autoMeasConfig = defaultAutoMeasConfig;
    }

    return true;
}

// Auto mode

bool hdc3022_set_auto_meas_mode(hdc3022_t *hdc3022, hdc3022_AutoMeasMode_t autoMeasConfig) {
    if (autoMeasConfig == HDC3022_AUTO_MEAS_NONE)
        return hdc3022_exit_auto_meas_mode(hdc3022);

    if (!hdc3022_write_command(hdc3022, autoMeasConfig, false))
        return false;
    hdc3022->autoMeasConfig = autoMeasConfig;
    hdc3022->autoModeEnabled = true;
    return true;
}

bool hdc3022_exit_auto_meas_mode(hdc3022_t *hdc3022) {
    if (!hdc3022->autoModeEnabled)
        return false;

    if (!hdc3022_write_command(hdc3022, HDC3022_AUTO_MEAS_EXIT, false))
        return false;

    hdc3022->autoModeEnabled = false;

    return true;
}

bool hdc3022_readout_auto_meas(hdc3022_t *hdc3022, hdc3022_AutoMeasReadoutCommand_t readoutCommand,
                               hdc3022_TempUnit_t tempUnit, float *temp, float *humidity) {

    if (!hdc3022_write_command(hdc3022, readoutCommand, true))
        return false;

    if (readoutCommand == HDC3022_AUTO_MEAS_READOUT) {
        uint16_t rawTemp, rawHumidity;
        if (!hdc3022_read_multi_data(hdc3022, &rawTemp, &rawHumidity, false))
            return false;

        *temp = hdc3022_calculate_temp(rawTemp, tempUnit);

        *humidity = hdc3022_calculate_humidity(rawHumidity);
    }
    else if (readoutCommand == HDC3022_AUTO_MEAS_READOUT_MIN_TEMP ||
             readoutCommand == HDC3022_AUTO_MEAS_READOUT_MAX_TEMP) {
        uint16_t rawTemp;
        if (!hdc3022_read_single_data(hdc3022, &rawTemp, false))
            return false;

        *temp = hdc3022_calculate_temp(rawTemp, tempUnit);
    }
    else {
        uint16_t rawHumidity;
        if (!hdc3022_read_single_data(hdc3022, &rawHumidity, false))
            return false;

        *humidity = hdc3022_calculate_humidity(rawHumidity);
    }

    return true;
}

// Manual mode

bool hdc3022_readout_manual_meas(hdc3022_t *hdc3022, hdc3022_ManualMeasMode_t readCommand,
                                 hdc3022_TempUnit_t tempUnit, float *temp, float *humidity) {
    if (hdc3022->autoModeEnabled)
        return false;

    if (!hdc3022_write_command(hdc3022, readCommand, false))
        return false;

    switch (readCommand) {
        case HDC3022_MANUAL_MEAS_LPM_0:
            sleep_ms(HDC3022_DELAY_LPM_0_MS);
            break;
        case HDC3022_MANUAL_MEAS_LPM_1:
            sleep_ms(HDC3022_DELAY_LPM_1_MS);
            break;
        case HDC3022_MANUAL_MEAS_LPM_2:
            sleep_ms(HDC3022_DELAY_LPM_2_MS);
            break;
        case HDC3022_MANUAL_MEAS_LPM_3:
            sleep_ms(HDC3022_DELAY_LPM_3_MS);
            break;
    }

    uint16_t rawTemp, rawHumidity;
    if (!hdc3022_read_multi_data(hdc3022, &rawTemp, &rawHumidity, false))
        return false;

    *temp = hdc3022_calculate_temp(rawTemp, tempUnit);

    *humidity = hdc3022_calculate_humidity(rawHumidity);

    return true;
}

// Heater

bool hdc3022_set_heater_enabled(hdc3022_t *hdc3022, bool enabled) {
    if (enabled)
        return hdc3022_write_command(hdc3022, HDC3022_HEATER_ENABLE, false);
    else
        return hdc3022_write_command(hdc3022, HDC3022_HEATER_DISABLE, false);
}

bool hdc3022_set_heater_power(hdc3022_t *hdc3022, hdc3022_HeaterPower_t heaterPower) {
    return hdc3022_write_command_data(hdc3022, HDC3022_HEATER_CONFIG, heaterPower, false);
}

// Soft reset

bool hdc3022_soft_reset(hdc3022_t *hdc3022) {
    return hdc3022_write_command(hdc3022, HDC3022_SOFT_RESET, false);
}

// Status

bool hdc3022_clear_status(hdc3022_t *hdc3022) {
    return hdc3022_write_command(hdc3022, HDC3022_STATUS_REGISTER_CLEAR, false);
}

bool hdc3022_read_status_all(hdc3022_t *hdc3022, bool statusArray[16]) {
    uint16_t status;
    if (!hdc3022_write_command(hdc3022, HDC3022_STATUS_REGISTER_READ, true))
        return false;

    if (!hdc3022_read_single_data(hdc3022, &status, false))
        return false;

    for (int i = 0; i < 16; i++)
        statusArray[i] = (status >> i) & 1;

    return true;
}

bool hdc3022_read_status(hdc3022_t *hdc3022, hdc3022_AlertStatus_t statusPosition,
                         bool *statusValue) {
    uint16_t status;
    if (!hdc3022_write_command(hdc3022, HDC3022_STATUS_REGISTER_READ, true))
        return false;

    if (!hdc3022_read_single_data(hdc3022, &status, false))
        return false;

    *statusValue = (status & (1UL << statusPosition));

    return true;
}

// Identifiers

uint16_t hdc3022_read_manufacturer_id(hdc3022_t *hdc3022) {
    uint16_t manufacturerId = 0;
    if (!hdc3022_write_command(hdc3022, HDC3022_MANUFACTER_ID_READ, true))
        return 0;
    if (!hdc3022_read_single_data(hdc3022, &manufacturerId, false))
        return 0;

    return manufacturerId;
}

bool hdc3022_read_nist_id(hdc3022_t *hdc3022, uint8_t nist[6]) {
    uint16_t part1, part2, part3;

    if (!hdc3022_write_command(hdc3022, HDC3022_NIST_ID_BYTES_5_4_READ, true))
        return false;
    if (!hdc3022_read_single_data(hdc3022, &part1, false))
        return false;

    if (!hdc3022_write_command(hdc3022, HDC3022_NIST_ID_BYTES_3_2_READ, true))
        return false;
    if (!hdc3022_read_single_data(hdc3022, &part2, false))
        return false;

    if (!hdc3022_write_command(hdc3022, HDC3022_NIST_ID_BYTES_1_0_READ, true))
        return false;
    if (!hdc3022_read_single_data(hdc3022, &part3, false))
        return false;

    nist[0] = (uint8_t)(part1 >> 8);
    nist[1] = (uint8_t)(part1 & 0xFF);
    nist[2] = (uint8_t)(part2 >> 8);
    nist[3] = (uint8_t)(part2 & 0xFF);
    nist[4] = (uint8_t)(part3 >> 8);
    nist[5] = (uint8_t)(part3 & 0xFF);

    return true;
}

// Alerts

bool hdc3022_config_alert(hdc3022_t *hdc3022,
                          hdc3022_ConfigAlertThresholdsCommand_t alertSetCommand,
                          hdc3022_TempUnit_t tempUnit, float temp, float humidity) {
    uint16_t threshold;

    threshold = hdc3022_calculate_alert_threshold(tempUnit, temp, humidity);

    if (!hdc3022_write_command_data(hdc3022, alertSetCommand, threshold, false))
        return false;

    return true;
}

bool hdc3022_read_alert(hdc3022_t *hdc3022, hdc3022_ReadAlertThresholdsCommand_t alertReadCommand,
                        hdc3022_TempUnit_t tempUnit, float *temp, float *humidity) {

    if (!hdc3022_write_command(hdc3022, alertReadCommand, true))
        return false;

    uint16_t rawTemp, rawHumidity;
    uint16_t threshold;

    if (!hdc3022_read_single_data(hdc3022, &threshold, false))
        return false;

    hdc3022_decode_alert_threshold(threshold, tempUnit, temp, humidity);

    return true;
}

bool hdc3022_transfer_alert_thresholds_eeprom(hdc3022_t *hdc3022) {
    if (hdc3022->autoModeEnabled)
        return false;

    if (!hdc3022_write_command(hdc3022, HDC3022_TRANSFER_ALERT_THRESHOLDS_EEPROM, false))
        return false;

    sleep_ms(HDC3022_DELAY_EEPROM_MS);

    return true;
}

// EEPROM

bool hdc3022_set_default_meas_state(hdc3022_t *hdc3022, hdc3022_DefaultMeasMode_t measMode) {
    if (hdc3022->autoModeEnabled)
        return false;

    if (!hdc3022_write_command_data(hdc3022, HDC3022_DEFAULT_DEVICE_MEAS_STATE_ACCESS, measMode,
                                    false))
        return false;

    sleep_ms(HDC3022_DELAY_EEPROM_MS);

    return true;
}

bool hdc3022_read_default_meas_state(hdc3022_t *hdc3022, hdc3022_DefaultMeasMode_t *defaultMode) {
    if (!hdc3022_write_command(hdc3022, HDC3022_DEFAULT_DEVICE_MEAS_STATE_ACCESS, true))
        return false;

    uint16_t defaultModeRaw;
    if (!hdc3022_read_single_data(hdc3022, &defaultModeRaw, false))
        return false;

    *defaultMode = (hdc3022_DefaultMeasMode_t)defaultModeRaw;
    return true;
}

// OFFSETS

bool hdc3022_write_offsets(hdc3022_t *hdc3022, hdc3022_TempUnit_t tempUnit, float temp,
                           float humidity) {
    if (hdc3022->autoModeEnabled)
        return false;

    uint16_t offset = hdc3022_calculate_offset(tempUnit, temp, humidity);

    if (!hdc3022_write_command_data(hdc3022, HDC3022_OFFSET_VALUE_ACCESS, offset, false))
        return false;

    sleep_ms(HDC3022_DELAY_EEPROM_MS);

    return true;
}

bool hdc3022_read_offsets(hdc3022_t *hdc3022, hdc3022_TempUnit_t tempUnit, float *temp,
                          float *humidity) {
    uint16_t rawOffset;

    if (!hdc3022_write_command(hdc3022, HDC3022_OFFSET_VALUE_ACCESS, true))
        return false;

    if (!hdc3022_read_single_data(hdc3022, &rawOffset, false))
        return false;

    hdc3022_decode_offset(rawOffset, tempUnit, temp, humidity);

    return true;
}

// Low level functions
bool hdc3022_write_command(hdc3022_t *hdc3022, uint16_t command, bool nostop) {
    uint8_t buffer[2];
    buffer[0] = (uint8_t)(command >> 8);   // MSB Command
    buffer[1] = (uint8_t)(command & 0xFF); // LSB Command

    int written = i2c_write_blocking(hdc3022->i2c, hdc3022->i2cAddress, buffer, 2,
                                     nostop); // false == stop comunication
    if (written == 2)
        return true;
    else
        return false;
}

bool hdc3022_write_command_data(hdc3022_t *hdc3022, uint16_t command, uint16_t data, bool nostop) {
    uint8_t buffer[5];
    buffer[0] = (uint8_t)(command >> 8);              // MSB Command
    buffer[1] = (uint8_t)(command & 0xFF);            // LSB Command
    buffer[2] = (uint8_t)(data >> 8);                 // MSB Data
    buffer[3] = (uint8_t)(data & 0xFF);               // LSB Data
    buffer[4] = hdc3022_calculateCRC8(buffer + 2, 2); // Calculate CRC for the data

    int written = i2c_write_blocking(hdc3022->i2c, hdc3022->i2cAddress, buffer, 5,
                                     nostop); // false == stop comunication
    if (written == 5)
        return true;
    else
        return false;
}

bool hdc3022_read_single_data(hdc3022_t *hdc3022, uint16_t *data, bool nostop) {
    uint8_t buffer[3];

    // Read data + CRC
    int read = i2c_read_blocking(hdc3022->i2c, hdc3022->i2cAddress, buffer, 3, nostop);

    if (read != 3)
        return false;

    // Validate CRC
    if (hdc3022_calculateCRC8(buffer, 2) != buffer[2])
        return false;

    *data = (uint16_t)(buffer[0] << 8 | buffer[1]);

    return true;
}

bool hdc3022_read_multi_data(hdc3022_t *hdc3022, uint16_t *data1, uint16_t *data2, bool nostop) {
    uint8_t buffer[6];
    i2c_read_blocking(hdc3022->i2c, hdc3022->i2cAddress, buffer, 6, nostop);

    // Validate CRC for data1
    if (hdc3022_calculateCRC8(buffer, 2) != buffer[2])
        return false;

    // Validate CRC for data2
    if (hdc3022_calculateCRC8(buffer + 3, 2) != buffer[5])
        return false;

    *data1 = (buffer[0] << 8) | buffer[1];
    *data2 = (buffer[3] << 8) | buffer[4];

    return true;
}

float hdc3022_calculate_temp(uint16_t rawTemp, hdc3022_TempUnit_t tempUnit) {
    switch (tempUnit) {
        case HDC3022_TEMP_CELSIUS:
            return ((rawTemp / 65535.0f) * 175.0f) - 45.0f;
        case HDC3022_TEMP_FAHRENHEIT:
            return ((rawTemp / 65535.0f) * 315.0f) - 49.0f;
        default:
            return 130.0f;
    }
}

float hdc3022_calculate_humidity(uint16_t rawHumidity) {
    return (rawHumidity / 65535.0f) * 100.0f;
}

uint16_t hdc3022_calculate_rawTemp(hdc3022_TempUnit_t tempUnit, float temp) {
    float rawTemp;
    switch (tempUnit) {
        case HDC3022_TEMP_CELSIUS:
            rawTemp = ((temp + 45.0f) / 175.0f) * 65535.0f;
            break;
        case HDC3022_TEMP_FAHRENHEIT:
            rawTemp = ((temp + 49.0f) / 315.0f) * 65535.0f;
            break;
        default:
            rawTemp = 0.0f;
            break;
    }
    if (rawTemp < 0.0f)
        rawTemp = 0.0f;
    else if (rawTemp > 65535.0f)
        rawTemp = 65535.0f;

    return (uint16_t)(rawTemp + 0.5f);
}

uint16_t hdc3022_calculate_rawHumidity(float humidity) {
    float rawHumidity;
    if (humidity < 0.0f)
        humidity = 0.0f;
    else if (humidity > 100.0f)
        humidity = 100.0f;

    rawHumidity = (humidity / 100.0f) * 65535.0f;
    return (uint16_t)(rawHumidity + 0.5f);
}

uint16_t hdc3022_calculate_alert_threshold(hdc3022_TempUnit_t tempUnit, float temp,
                                           float humidity) {
    uint16_t rawTemp = hdc3022_calculate_rawTemp(tempUnit, temp);
    uint16_t rawHumidity = hdc3022_calculate_rawHumidity(humidity);
    // Retain the 7 MSBs for RH and the 9 MSBs for T
    uint16_t msbHumidity = (rawHumidity >> 9) & 0x7F;
    uint16_t msbTemp = (rawTemp >> 7) & 0x1FF;

    // Concatenate the 7 MSBs for RH with the 9 MSBs for T
    return (msbHumidity << 9) | msbTemp;
}

void hdc3022_decode_alert_threshold(uint16_t threshold, hdc3022_TempUnit_t tempUnit, float *temp,
                                    float *humidity) {
    uint16_t msbHumidity = (threshold >> 9) & 0x7F;
    uint16_t msbTemp = threshold & 0x1FF;

    uint16_t rawHumidity = msbHumidity << 9;
    uint16_t rawTemp = msbTemp << 7;

    *temp = hdc3022_calculate_temp(rawTemp, tempUnit);
    *humidity = hdc3022_calculate_humidity(rawHumidity);
}

uint8_t hdc3022_calculate_temp_offset(hdc3022_TempUnit_t tempUnit, float temp) {
    if (tempUnit == HDC3022_TEMP_FAHRENHEIT)
        temp = (temp - 32.0f) * 5.0f / 9.0f;

    uint8_t tempSign;
    if (temp < 0)
        tempSign = HDC3022_NEGATIVE_SIGN_OFFSET;
    else
        tempSign = HDC3022_POSITIVE_SIGN_OFFSET;

    float tempAbs = fabsf(temp);

    uint8_t tempOffset = (uint8_t)(roundf(tempAbs / HDC3022_TEMP_LSB_OFFSET));
    return tempSign | tempOffset;
}

uint8_t hdc3022_calculate_humidity_offset(float humidity) {
    uint8_t humiditySign = HDC3022_POSITIVE_SIGN_OFFSET;
    float humidityAbs = fabsf(humidity);
    uint8_t humidityOffset = (uint8_t)(roundf(humidityAbs / HDC3022_HUMIDITY_LSB_OFFSET));
    return humiditySign | humidityOffset;
}

uint16_t hdc3022_calculate_offset(hdc3022_TempUnit_t tempUnit, float temp, float humidity) {
    uint8_t tempOffset = hdc3022_calculate_temp_offset(tempUnit, temp);
    uint8_t humidityOffset = hdc3022_calculate_humidity_offset(humidity);

    return ((uint16_t)humidityOffset << 8) | tempOffset;
}

float hdc3022_decode_temp_offset(hdc3022_TempUnit_t tempUnit, uint8_t tempOffset) {
    // Extract the sign bit
    bool isNegative;

    if ((tempOffset & HDC3022_POSITIVE_SIGN_OFFSET) == 0) {
        isNegative = true;
    }
    else {
        isNegative = false;
    }

    // Remove the sign bit from the offset
    uint8_t absOffset = tempOffset & 0x7F;

    // Calculate the floating point value
    float temp = absOffset * HDC3022_TEMP_LSB_OFFSET;

    // Apply the sign bit
    if (isNegative)
        temp = -temp;

    if (tempUnit == HDC3022_TEMP_FAHRENHEIT)
        temp = (temp - 32.0f) * 5.0f / 9.0f;

    return temp;
}

float hdc3022_decode_humidity_offset(uint8_t humidityOffset) {
    // Remove the sign bit from the offset
    uint8_t absOffset = humidityOffset & 0x7F;

    // Calculate the floating point value
    float humidity = absOffset * HDC3022_HUMIDITY_LSB_OFFSET;

    return humidity;
}

void hdc3022_decode_offset(uint16_t offset, hdc3022_TempUnit_t tempUnit, float *temp,
                           float *humidity) {
    uint8_t humidityOffset = (offset >> 8) & 0xFF;
    uint8_t tempOffset = offset & 0xFF;

    *temp = hdc3022_decode_temp_offset(tempUnit, tempOffset);
    *humidity = hdc3022_decode_humidity_offset(humidityOffset);
}

// From github.com/adafruit/Adafruit_HDC302x
uint8_t hdc3022_calculateCRC8(const uint8_t *data, int len) {
    uint8_t crc = 0xFF; // Typical initial value
    for (int i = 0; i < len; i++) {
        crc ^= data[i];                  // XOR byte into least sig. byte of crc
        for (int j = 8; j > 0; j--) {    // Loop over each bit
            if (crc & 0x80) {            // If the uppermost bit is 1...
                crc = (crc << 1) ^ 0x31; // Polynomial used by HDC3022
            }
            else {
                crc = (crc << 1);
            }
        }
    }
    return crc; // Final XOR value can also be applied if specified by device
}

hdc3022_AutoMeasMode_t hdc3022_configMode_to_measMode(hdc3022_DefaultMeasMode_t defaultMode) {
    switch (defaultMode) {
        case HDC3022_AUTO_CONFIG_1X_2S_LPM_0:
            return HDC3022_AUTO_CONFIG_1X_2S_LPM_0;
        case HDC3022_AUTO_CONFIG_1X_2S_LPM_1:
            return HDC3022_AUTO_MEAS_1X_2S_LPM_1;
        case HDC3022_AUTO_CONFIG_1X_2S_LPM_2:
            return HDC3022_AUTO_MEAS_1X_2S_LPM_2;
        case HDC3022_AUTO_CONFIG_1X_2S_LPM_3:
            return HDC3022_AUTO_MEAS_1X_2S_LPM_3;

        case HDC3022_AUTO_CONFIG_1X_1S_LPM_0:
            return HDC3022_AUTO_MEAS_1X_1S_LPM_0;
        case HDC3022_AUTO_CONFIG_1X_1S_LPM_1:
            return HDC3022_AUTO_MEAS_1X_1S_LPM_1;
        case HDC3022_AUTO_CONFIG_1X_1S_LPM_2:
            return HDC3022_AUTO_MEAS_1X_1S_LPM_0;
        case HDC3022_AUTO_CONFIG_1X_1S_LPM_3:
            return HDC3022_AUTO_MEAS_1X_1S_LPM_3;

        case HDC3022_AUTO_CONFIG_2X_1S_LPM_0:
            return HDC3022_AUTO_MEAS_1X_2S_LPM_0;
        case HDC3022_AUTO_CONFIG_2X_1S_LPM_1:
            return HDC3022_AUTO_MEAS_1X_2S_LPM_1;
        case HDC3022_AUTO_CONFIG_2X_1S_LPM_2:
            return HDC3022_AUTO_MEAS_1X_2S_LPM_2;
        case HDC3022_AUTO_CONFIG_2X_1S_LPM_3:
            return HDC3022_AUTO_MEAS_1X_2S_LPM_3;

        case HDC3022_AUTO_CONFIG_4X_1S_LPM_0:
            return HDC3022_AUTO_MEAS_4X_1S_LPM_0;
        case HDC3022_AUTO_CONFIG_4X_1S_LPM_1:
            return HDC3022_AUTO_MEAS_4X_1S_LPM_1;
        case HDC3022_AUTO_CONFIG_4X_1S_LPM_2:
            return HDC3022_AUTO_MEAS_4X_1S_LPM_2;
        case HDC3022_AUTO_CONFIG_4X_1S_LPM_3:
            return HDC3022_AUTO_MEAS_4X_1S_LPM_3;

        case HDC3022_AUTO_CONFIG_10X_1S_LPM_0:
            return HDC3022_AUTO_MEAS_10X_1S_LPM_0;
        case HDC3022_AUTO_CONFIG_10X_1S_LPM_1:
            return HDC3022_AUTO_MEAS_10X_1S_LPM_1;
        case HDC3022_AUTO_CONFIG_10X_1S_LPM_2:
            return HDC3022_AUTO_MEAS_10X_1S_LPM_2;
        case HDC3022_AUTO_CONFIG_10X_1S_LPM_3:
            return HDC3022_AUTO_MEAS_10X_1S_LPM_3;

        default:
            return HDC3022_AUTO_MEAS_NONE;
    }
}
