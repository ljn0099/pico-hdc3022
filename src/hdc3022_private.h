/**
 * @file hdc3022_private.h
 * @brief Internal functions and private definitions for the HDC3022 sensor driver.
 *
 * This file contains internal implementations and macros intended for
 * use within the HDC3022 driver only. It is not meant for direct use by
 * application code.
 *
 * Users should not include this file outside of the driver context.
 *
 * @author ljn0099
 *
 * @license MIT License
 * Copyright (c) 2025 ljn0099
 *
 * See LICENSE file for details.
 */
#ifndef HDC3022_PRIVATE_H
#define HDC3022_PRIVATE_H

#define HDC3022_SOFT_RESET 0x3041

#define HDC3022_DELAY_LPM_0_MS 15
#define HDC3022_DELAY_LPM_1_MS 9
#define HDC3022_DELAY_LPM_2_MS 6
#define HDC3022_DELAY_LPM_3_MS 5

#define HDC3022_DELAY_RESET_MS 3
#define HDC3022_DELAY_POWER_ON_MS 5
#define HDC3022_DELAY_EEPROM_MS 80

#define HDC3022_AUTO_MEAS_EXIT 0x3093
#define HDC3022_TRANSFER_ALERT_THRESHOLDS_EEPROM 0x6155
#define HDC3022_OFFSET_VALUE_ACCESS 0xA004
#define HDC3022_DEFAULT_DEVICE_MEAS_STATE_ACCESS 0x61BB

#define HDC3022_TEMP_LSB_OFFSET 0.1708984375f
#define HDC3022_HUMIDITY_LSB_OFFSET 0.1953125f
#define HDC3022_POSITIVE_SIGN_OFFSET 0x80
#define HDC3022_NEGATIVE_SIGN_OFFSET 0x00

#define HDC3022_HEATER_ENABLE 0x306D
#define HDC3022_HEATER_DISABLE 0x3066
#define HDC3022_HEATER_CONFIG 0x306E

#define HDC3022_STATUS_REGISTER_READ 0xF32D
#define HDC3022_STATUS_REGISTER_CLEAR 0x3041

#define HDC3022_NIST_ID_BYTES_5_4_READ 0x3683
#define HDC3022_NIST_ID_BYTES_3_2_READ 0x3684
#define HDC3022_NIST_ID_BYTES_1_0_READ 0x3685
#define HDC3022_MANUFACTER_ID_READ 0x3781

/**
 * @brief Sends a 16-bit command to the HDC3022 sensor via I2C.
 *
 * This function writes a 16-bit command to the HDC3022 sensor without any accompanying data.
 * The `nostop` parameter controls whether a stop condition is generated after the transmission.
 *
 * @param[in] hdc3022 Pointer to the HDC3022 device structure.
 * @param[in] command 16-bit command to send to the sensor.
 * @param[in] nostop If true, no stop condition is sent on the I2C bus after writing.
 *
 * @return true if the write operation was successful (2 bytes written), false otherwise.
 */
bool hdc3022_write_command(hdc3022_t *hdc3022, uint16_t command, bool nostop);

/**
 * @brief Sends a command and data to the HDC3022 sensor via I2C.
 *
 * This function writes a 16-bit command followed by a 16-bit data value to the HDC3022 sensor.
 * A CRC byte is calculated and appended to ensure data integrity.
 * The `nostop` parameter controls whether a stop condition is generated after the transmission.
 *
 * @param[in] hdc3022 Pointer to the HDC3022 device structure.
 * @param[in] command 16-bit command to send to the sensor.
 * @param[in] data 16-bit data associated with the command.
 * @param[in] nostop If true, no stop condition is sent on the I2C bus after writing.
 *
 * @return true if the write operation was successful (5 bytes written), false otherwise.
 */
bool hdc3022_write_command_data(hdc3022_t *hdc3022, uint16_t command, uint16_t data, bool nostop);

/**
 * @brief Reads a single 16-bit data value from the HDC3022 sensor via I2C.
 *
 * This function reads 2 bytes of data followed by a CRC byte from the sensor,
 * verifies the CRC to ensure data integrity, and stores the received data.
 * The `nostop` parameter controls whether a stop condition is generated after reading.
 *
 * @param[in] hdc3022 Pointer to the HDC3022 device structure.
 * @param[out] data Pointer to a variable where the read 16-bit data will be stored.
 * @param[in] nostop If true, no stop condition is sent on the I2C bus after reading.
 *
 * @return true if the read and CRC validation were successful, false otherwise.
 */
bool hdc3022_read_single_data(hdc3022_t *hdc3022, uint16_t *data, bool nostop);

/**
 * @brief Reads two 16-bit data values from the HDC3022 sensor via I2C.
 *
 * This function reads 6 bytes from the sensor: two sets of 2-byte data values each followed by a
 * CRC byte. It validates the CRC for both data values to ensure data integrity before storing them.
 * The `nostop` parameter controls whether a stop condition is generated after reading.
 *
 * @param[in] hdc3022 Pointer to the HDC3022 device structure.
 * @param[out] data1 Pointer to a variable where the first 16-bit data will be stored.
 * @param[out] data2 Pointer to a variable where the second 16-bit data will be stored.
 * @param[in] nostop If true, no stop condition is sent on the I2C bus after reading.
 *
 * @return true if both data values are successfully read and CRC validated, false otherwise.
 */
bool hdc3022_read_multi_data(hdc3022_t *hdc3022, uint16_t *data1, uint16_t *data2, bool nostop);

/**
 * @brief Converts raw temperature data from the HDC3022 sensor to a specified unit.
 *
 * This function takes a 16-bit raw temperature value read from the sensor and converts it
 * to a floating-point temperature value based on the unit specified in the @ref hdc3022_TempUnit_t
 * enum.
 *
 * @param[in] rawTemp Raw 16-bit temperature value from the sensor.
 * @param[in] tempUnit Temperature unit for conversion. See @ref hdc3022_TempUnit_t.
 *
 * @return The converted temperature value in the specified unit. If the unit is invalid,
 *         the function returns 130.0f as an error indicator.
 */
float hdc3022_calculate_temp(uint16_t rawTemp, hdc3022_TempUnit_t tempUnit);

/**
 * @brief Converts raw humidity data from the HDC3022 sensor to relative humidity (%RH).
 *
 * This function converts a 16-bit raw humidity value from the sensor into a percentage
 * representing relative humidity.
 *
 * @param[in] rawHumidity Raw 16-bit humidity value from the sensor.
 *
 * @return Relative humidity as a floating-point value in the range 0.0 to 100.0 (%RH).
 */
float hdc3022_calculate_humidity(uint16_t rawHumidity);

/**
 * @brief Converts a temperature value to a raw 16-bit representation for the HDC3022 sensor.
 *
 * This function takes a temperature value in a specified unit and converts it into the raw 16-bit
 * format expected by the HDC3022 sensor. The conversion is based on the unit provided through
 * @ref hdc3022_TempUnit_t.
 *
 * If the resulting raw value falls outside the valid 16-bit range (0–65535), it is saturated to the
 * nearest boundary value.
 *
 * @param[in] tempUnit Temperature unit used for the input value. See @ref hdc3022_TempUnit_t.
 * @param[in] temp Temperature value to convert.
 *
 * @return The corresponding raw 16-bit temperature value for the HDC3022 sensor.
 */
uint16_t hdc3022_calculate_rawTemp(hdc3022_TempUnit_t tempUnit, float temp);

/**
 * @brief Converts a relative humidity value (%RH) to a raw 16-bit format for the HDC3022 sensor.
 *
 * This function converts a relative humidity value expressed in percent (%RH) to the corresponding
 * 16-bit raw format required by the HDC3022 sensor.
 *
 * If the input value is outside the valid range [0.0, 100.0], it is saturated to the nearest
 * boundary (0.0 for values below range, 100.0 for values above).
 *
 * @param[in] humidity Relative humidity value in percent (%RH).
 *
 * @return The corresponding raw 16-bit humidity value for the HDC3022 sensor.
 */
uint16_t hdc3022_calculate_rawHumidity(float humidity);

/**
 * @brief Calculates the 16-bit alert threshold value for the HDC3022 sensor based on temperature
 * and humidity.
 *
 * This function computes a threshold value used for configuring the alert limits in the HDC3022
 * sensor.
 *
 * @param[in] tempUnit Temperature unit used for the input temperature. See @ref hdc3022_TempUnit_t.
 * @param[in] temp Temperature value to include in the threshold.
 * @param[in] humidity Relative humidity value (%RH) to include in the threshold.
 *
 * @return A 16-bit threshold value formatted for the HDC3022 alert configuration register.
 */
uint16_t hdc3022_calculate_alert_threshold(hdc3022_TempUnit_t tempUnit, float temp, float humidity);

/**
 * @brief Decodes a 16-bit alert threshold value into temperature and humidity values.
 *
 * This function takes a 16-bit threshold value—typically obtained from the HDC3022 sensor or
 * configuration— and converts it into temperature and relative humidity values based on the
 * specified temperature unit.
 *
 * @param[in] threshold 16-bit encoded threshold value to decode.
 * @param[in] tempUnit Temperature unit to use for the decoded temperature. See @ref
 * hdc3022_TempUnit_t.
 * @param[out] temp Pointer to a variable where the decoded temperature will be stored.
 * @param[out] humidity Pointer to a variable where the decoded relative humidity (%RH) will be
 * stored.
 */
void hdc3022_decode_alert_threshold(uint16_t threshold, hdc3022_TempUnit_t tempUnit, float *temp,
                                    float *humidity);

/**
 * @brief Calculates the temperature offset value for the HDC3022 sensor.
 *
 * Converts a temperature value in the specified unit into the sensor-specific offset format.
 *
 * @param[in] tempUnit Temperature unit of the input value. See @ref hdc3022_TempUnit_t.
 * @param[in] temp Temperature value to convert.
 *
 * @return Encoded temperature offset as an 8-bit value combining sign and magnitude.
 */
uint8_t hdc3022_calculate_temp_offset(hdc3022_TempUnit_t tempUnit, float temp);

/**
 * @brief Calculates the humidity offset value for the HDC3022 sensor.
 *
 * Converts a relative humidity value into the sensor-specific offset format.
 * The offset value is always treated as positive.
 *
 * @param[in] humidity Relative humidity value to convert.
 *
 * @return Encoded humidity offset as an 8-bit value.
 */
uint8_t hdc3022_calculate_humidity_offset(float humidity);

/**
 * @brief Calculates the combined temperature and humidity offset for the HDC3022 sensor.
 *
 * This function encodes temperature and humidity offset values into a single 16-bit value
 * formatted for the HDC3022 sensor. Temperature and humidity are converted individually
 * using their respective offset calculation functions.
 *
 * @param[in] tempUnit Temperature unit of the input temperature. See \ref hdc3022_TempUnit_t.
 * @param[in] temp Temperature value to convert.
 * @param[in] humidity Relative humidity value to convert.
 *
 * @return A 16-bit encoded offset value combining temperature and humidity offsets.
 */
uint16_t hdc3022_calculate_offset(hdc3022_TempUnit_t tempUnit, float temp, float humidity);

/**
 * @brief Decodes a temperature offset value from the HDC3022 sensor format.
 *
 * This function converts an 8-bit encoded temperature offset into a floating-point temperature
 * value, taking into account the sign and temperature unit.
 *
 * @param[in] tempUnit Temperature unit to use for the decoded value. See \ref hdc3022_TempUnit_t.
 * @param[in] tempOffset Encoded 8-bit temperature offset value.
 *
 * @return Decoded temperature as a floating-point value.
 */
float hdc3022_decode_temp_offset(hdc3022_TempUnit_t tempUnit, uint8_t tempOffset);

/**
 * @brief Decodes a humidity offset value from the HDC3022 sensor format.
 *
 * This function converts an 8-bit encoded humidity offset into a floating-point relative humidity
 * value.
 *
 * @param[in] humidityOffset Encoded 8-bit humidity offset value.
 *
 * @return Decoded relative humidity as a floating-point value (%RH).
 */
float hdc3022_decode_humidity_offset(uint8_t humidityOffset);

/**
 * @brief Decodes a combined 16-bit offset value into temperature and humidity.
 *
 * This function splits a 16-bit encoded offset value into separate temperature and humidity
 * offsets, then decodes each into floating-point values.
 *
 * @param[in] offset Combined 16-bit encoded offset value.
 * @param[in] tempUnit Temperature unit to use for decoding the temperature offset. See \ref
 * hdc3022_TempUnit_t.
 * @param[out] temp Pointer to store the decoded temperature value.
 * @param[out] humidity Pointer to store the decoded relative humidity value (%RH).
 */
void hdc3022_decode_offset(uint16_t offset, hdc3022_TempUnit_t tempUnit, float *temp,
                           float *humidity);

/**
 * @brief Calculates the CRC-8 checksum for HDC3022 data.
 *
 * This function computes an 8-bit cyclic redundancy check (CRC) using the polynomial
 * 0x31, as specified for the HDC3022 sensor communication protocol.
 *
 * Implementation adapted from the Adafruit HDC302x library:
 * https://github.com/adafruit/Adafruit_HDC302x
 *
 * @param[in] data Pointer to the data buffer over which to calculate the CRC.
 * @param[in] len Length of the data buffer in bytes.
 *
 * @return Calculated 8-bit CRC checksum.
 */
uint8_t hdc3022_calculateCRC8(const uint8_t *data, int len);

/**
 * @brief Converts a default configuration measurement mode to an automatic measurement mode.
 *
 * This function maps a value from @ref hdc3022_DefaultMeasMode_t to the corresponding
 * @ref hdc3022_AutoMeasMode_t value used for automatic measurement configuration.
 *
 * @param[in] defaultMode The default measurement mode to convert.
 *
 * @return The corresponding automatic measurement mode. Returns @ref HDC3022_AUTO_MEAS_NONE
 *         if the input does not match any known configuration.
 */
hdc3022_AutoMeasMode_t hdc3022_configMode_to_measMode(hdc3022_DefaultMeasMode_t defaultMode);
#endif
