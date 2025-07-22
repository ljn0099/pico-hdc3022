/**
 * @file hdc3022.h
 * @brief HDC3022 I2C Library for the Raspberry Pi pico-sdk
 *
 * @author ljn0099
 *
 * @license MIT License
 * Copyright (c) 2025 ljn0099
 *
 * See LICENSE file for details.
 */
#ifndef HDC3022_H
#define HDC3022_H

#include "hardware/i2c.h"

/** Default I2C address of the HDC3022 sensor */
#define HDC3022_DEFAULT_ADDR 0x44

/**
 * @enum hdc3022_ManualMeasMode_t
 * @brief Manual measurement modes for the HDC3022 sensor.
 *
 * This enumeration defines the manual measurement commands that can be sent to the HDC3022 sensor.
 * Each value corresponds to a specific command that triggers a temperature and humidity measurement
 * in an specific low power mode (LPM).
 *
 * The hexadecimal values represent the command codes that must be sent to the sensor to start
 * manual measurement in the respective mode.
 *
 * These commands are mainly used with the function @ref hdc3022_readout_manual_meas to select
 * the desired manual measurement mode.
 */
typedef enum {
    HDC3022_MANUAL_MEAS_LPM_0 = 0x2400, /**< Manual measurement command for LPM mode 0 */
    HDC3022_MANUAL_MEAS_LPM_1 = 0x240B, /**< Manual measurement command for LPM mode 1 */
    HDC3022_MANUAL_MEAS_LPM_2 = 0x2416, /**< Manual measurement command for LPM mode 2 */
    HDC3022_MANUAL_MEAS_LPM_3 = 0x24FF, /**< Manual measurement command for LPM mode 3 */
} hdc3022_ManualMeasMode_t;

/**
 * @enum hdc3022_AutoMeasMode_t
 * @brief Automatic measurement modes for the HDC3022 sensor.
 *
 * This enumeration defines the automatic measurement modes that can be configured on the HDC3022
 * sensor. Each value corresponds to a specific automatic measurement configuration including
 * measurement frequency, sample averaging, and low power mode (LPM) settings.
 *
 * The hexadecimal values represent the command codes to configure the sensor to perform
 * measurements automatically at different sampling rates and power modes.
 *
 * The available modes include:
 * - Number of measurements per interval (e.g., 1x, 2x, 4x, 10x)
 * - Measurement interval duration (e.g., 1 second, 2 seconds)
 * - Low power mode level (LPM_0 to LPM_3)
 *
 * The mode `HDC3022_AUTO_MEAS_NONE` disables automatic measurement mode.
 *
 * These modes are typically used with the function @ref hdc3022_set_auto_meas_mode to control
 * the sensor’s automatic measurement behavior.
 */
typedef enum {
    HDC3022_AUTO_MEAS_1X_2S_LPM_0 = 0x2032, /**< 1 measurement every 2 seconds, LPM 0 */
    HDC3022_AUTO_MEAS_1X_2S_LPM_1 = 0x2024, /**< 1 measurement every 2 seconds, LPM 1 */
    HDC3022_AUTO_MEAS_1X_2S_LPM_2 = 0x202F, /**< 1 measurement every 2 seconds, LPM 2 */
    HDC3022_AUTO_MEAS_1X_2S_LPM_3 = 0x20FF, /**< 1 measurement every 2 seconds, LPM 3 */

    HDC3022_AUTO_MEAS_1X_1S_LPM_0 = 0x2130, /**< 1 measurement every 1 second, LPM 0 */
    HDC3022_AUTO_MEAS_1X_1S_LPM_1 = 0x2126, /**< 1 measurement every 1 second, LPM 1 */
    HDC3022_AUTO_MEAS_1X_1S_LPM_2 = 0x212D, /**< 1 measurement every 1 second, LPM 2 */
    HDC3022_AUTO_MEAS_1X_1S_LPM_3 = 0x21FF, /**< 1 measurement every 1 second, LPM 3 */

    HDC3022_AUTO_MEAS_2X_1S_LPM_0 = 0x2236, /**< 2 measurements every 1 second, LPM 0 */
    HDC3022_AUTO_MEAS_2X_1S_LPM_1 = 0x2220, /**< 2 measurements every 1 second, LPM 1 */
    HDC3022_AUTO_MEAS_2X_1S_LPM_2 = 0x222B, /**< 2 measurements every 1 second, LPM 2 */
    HDC3022_AUTO_MEAS_2X_1S_LPM_3 = 0x22FF, /**< 2 measurements every 1 second, LPM 3 */

    HDC3022_AUTO_MEAS_4X_1S_LPM_0 = 0x2334, /**< 4 measurements every 1 second, LPM 0 */
    HDC3022_AUTO_MEAS_4X_1S_LPM_1 = 0x2322, /**< 4 measurements every 1 second, LPM 1 */
    HDC3022_AUTO_MEAS_4X_1S_LPM_2 = 0x2329, /**< 4 measurements every 1 second, LPM 2 */
    HDC3022_AUTO_MEAS_4X_1S_LPM_3 = 0x23FF, /**< 4 measurements every 1 second, LPM 3 */

    HDC3022_AUTO_MEAS_10X_1S_LPM_0 = 0x2737, /**< 10 measurements every 1 second, LPM 0 */
    HDC3022_AUTO_MEAS_10X_1S_LPM_1 = 0x2721, /**< 10 measurements every 1 second, LPM 1 */
    HDC3022_AUTO_MEAS_10X_1S_LPM_2 = 0x272A, /**< 10 measurements every 1 second, LPM 2 */
    HDC3022_AUTO_MEAS_10X_1S_LPM_3 = 0x27FF, /**< 10 measurements every 1 second, LPM 3 */

    HDC3022_AUTO_MEAS_NONE = 0 /**< Disable automatic measurement mode */
} hdc3022_AutoMeasMode_t;

/**
 * @enum hdc3022_DefaultMeasMode_t
 * @brief Default measurement modes for the HDC3022 sensor.
 *
 * This enumeration defines the default measurement modes that can be programmed into the HDC3022
 * sensor's EEPROM. These modes determine the sensor's behavior after power-on or reset, overriding
 * the default sleep mode.
 *
 * Each value corresponds to a specific measurement configuration, including measurement frequency,
 * sample averaging, and low power mode (LPM) level.
 *
 * Modes available:
 * - Number of measurements per interval (e.g., 1x, 2x, 4x, 10x)
 * - Measurement interval duration (e.g., 1 second, 2 seconds)
 * - Low power mode level (LPM_0 to LPM_3)
 *
 * The mode `HDC3022_MANUAL_CONFIG` sets the sensor to manual configuration mode (sleep mode).
 *
 * These modes are used with @ref hdc3022_set_default_meas_state and @ref
 * hdc3022_read_default_meas_state to write and read the sensor’s default power-on measurement
 * state.
 */
typedef enum {
    HDC3022_AUTO_CONFIG_1X_2S_LPM_0 = 0x0003, /**< 1 measurement every 2 seconds, LPM 0 */
    HDC3022_AUTO_CONFIG_1X_2S_LPM_1 = 0x0013, /**< 1 measurement every 2 seconds, LPM 1 */
    HDC3022_AUTO_CONFIG_1X_2S_LPM_2 = 0x0023, /**< 1 measurement every 2 seconds, LPM 2 */
    HDC3022_AUTO_CONFIG_1X_2S_LPM_3 = 0x0033, /**< 1 measurement every 2 seconds, LPM 3 */

    HDC3022_AUTO_CONFIG_1X_1S_LPM_0 = 0x0005, /**< 1 measurement every 1 second, LPM 0 */
    HDC3022_AUTO_CONFIG_1X_1S_LPM_1 = 0x0015, /**< 1 measurement every 1 second, LPM 1 */
    HDC3022_AUTO_CONFIG_1X_1S_LPM_2 = 0x0025, /**< 1 measurement every 1 second, LPM 2 */
    HDC3022_AUTO_CONFIG_1X_1S_LPM_3 = 0x0035, /**< 1 measurement every 1 second, LPM 3 */

    HDC3022_AUTO_CONFIG_2X_1S_LPM_0 = 0x0007, /**< 2 measurements every 1 second, LPM 0 */
    HDC3022_AUTO_CONFIG_2X_1S_LPM_1 = 0x0017, /**< 2 measurements every 1 second, LPM 1 */
    HDC3022_AUTO_CONFIG_2X_1S_LPM_2 = 0x0027, /**< 2 measurements every 1 second, LPM 2 */
    HDC3022_AUTO_CONFIG_2X_1S_LPM_3 = 0x0037, /**< 2 measurements every 1 second, LPM 3 */

    HDC3022_AUTO_CONFIG_4X_1S_LPM_0 = 0x0009, /**< 4 measurements every 1 second, LPM 0 */
    HDC3022_AUTO_CONFIG_4X_1S_LPM_1 = 0x0019, /**< 4 measurements every 1 second, LPM 1 */
    HDC3022_AUTO_CONFIG_4X_1S_LPM_2 = 0x0029, /**< 4 measurements every 1 second, LPM 2 */
    HDC3022_AUTO_CONFIG_4X_1S_LPM_3 = 0x0039, /**< 4 measurements every 1 second, LPM 3 */

    HDC3022_AUTO_CONFIG_10X_1S_LPM_0 = 0x000B, /**< 10 measurements every 1 second, LPM 0 */
    HDC3022_AUTO_CONFIG_10X_1S_LPM_1 = 0x001B, /**< 10 measurements every 1 second, LPM 1 */
    HDC3022_AUTO_CONFIG_10X_1S_LPM_2 = 0x002B, /**< 10 measurements every 1 second, LPM 2 */
    HDC3022_AUTO_CONFIG_10X_1S_LPM_3 = 0x003B, /**< 10 measurements every 1 second, LPM 3 */

    HDC3022_MANUAL_CONFIG = 0x0000 /**< Manual measurement mode (sleep mode)*/
} hdc3022_DefaultMeasMode_t;

/**
 * @enum hdc3022_AutoMeasReadoutCommand_t
 * @brief Commands for reading data from the HDC3022 sensor in automatic measurement mode.
 *
 * This enumeration defines the different readout commands that can be sent to the HDC3022 sensor
 * when it is operating in automatic measurement mode.
 * Each command corresponds to a specific type of data that can be retrieved:
 * - Current temperature and humidity values
 * - Only humidity value
 * - Minimum and maximum recorded temperature
 * - Minimum and maximum recorded humidity
 *
 * These commands are used with the function @ref hdc3022_readout_auto_meas to specify which data
 * should be read from the sensor.
 */
typedef enum {
    HDC3022_AUTO_MEAS_READOUT = 0xE000,          /**< Read current temperature and humidity */
    HDC3022_AUTO_MEAS_READOUT_RH = 0xE001,       /**< Read current humidity only */
    HDC3022_AUTO_MEAS_READOUT_MIN_TEMP = 0xE002, /**< Read minimum recorded temperature */
    HDC3022_AUTO_MEAS_READOUT_MAX_TEMP = 0xE003, /**< Read maximum recorded temperature */
    HDC3022_AUTO_MEAS_READOUT_MIN_RH = 0xE004,   /**< Read minimum recorded humidity */
    HDC3022_AUTO_MEAS_READOUT_MAX_RH = 0xE005    /**< Read maximum recorded humidity */
} hdc3022_AutoMeasReadoutCommand_t;

/**
 * @enum hdc3022_ConfigAlertThresholdsCommand_t
 * @brief Commands to configure alert threshold registers on the HDC3022 sensor.
 *
 * This enumeration defines the commands used to set or clear the alert thresholds for temperature
 * and humidity on the HDC3022 sensor. Each command corresponds to a specific threshold register:
 * - Low threshold set
 * - High threshold set
 * - Low threshold clear
 * - High threshold clear
 *
 * These commands are used with the function @ref hdc3022_config_alert to program alert thresholds.
 */
typedef enum {
    HDC3022_ALERT_THRESHOLD_SET_LOW_CONFIG = 0x6100,   /**< Set low alert threshold */
    HDC3022_ALERT_THRESHOLD_SET_HIGH_CONFIG = 0x611D,  /**< Set high alert threshold */
    HDC3022_ALERT_THRESHOLD_CLEAR_LOW_CONFIG = 0x610B, /**< Clear low alert threshold */
    HDC3022_ALERT_THRESHOLD_CLEAR_HIGH_CONFIG = 0x6116 /**< Clear high alert threshold */
} hdc3022_ConfigAlertThresholdsCommand_t;

/**
 * @enum hdc3022_ReadAlertThresholdsCommand_t
 * @brief Commands to read alert threshold registers from the HDC3022 sensor.
 *
 * This enumeration defines the commands used to read the configured alert threshold registers for
 * temperature and humidity on the HDC3022 sensor. Each command corresponds to a specific threshold
 * register:
 * - Low alert threshold set
 * - High alert threshold set
 * - Low alert threshold clear
 * - High alert threshold clear
 *
 * These commands are used with the function @ref hdc3022_read_alert to retrieve the threshold
 * values.
 */
typedef enum {
    HDC3022_ALERT_THRESHOLD_SET_LOW_READ = 0xE102,   /**< Read low alert threshold set register */
    HDC3022_ALERT_THRESHOLD_SET_HIGH_READ = 0xE11F,  /**< Read high alert threshold set register */
    HDC3022_ALERT_THRESHOLD_CLEAR_LOW_READ = 0xE109, /**< Read low alert threshold clear register */
    HDC3022_ALERT_THRESHOLD_CLEAR_HIGH_READ =
        0xE114 /**< Read high alert threshold clear register */
} hdc3022_ReadAlertThresholdsCommand_t;

/**
 * @enum hdc3022_HeaterPower_t
 * @brief Heater power levels for the HDC3022 sensor.
 *
 * This enumeration defines the available power levels for the HDC3022 sensor’s internal heater.
 * These levels control the amount of power supplied to the heater but do **not** turn the heater on
 * or off. Use @ref hdc3022_set_heater_enabled() to enable or disable the heater.
 */
typedef enum {
    HDC3022_HEATER_OFF_POWER = 0x0000,     /**< Heater off (no power) */
    HDC3022_HEATER_QUARTER_POWER = 0x009F, /**< Heater at quarter power */
    HDC3022_HEATER_HALF_POWER = 0x03FF,    /**< Heater at half power */
    HDC3022_HEATER_FULL_POWER = 0x3FFF     /**< Heater at full power */
} hdc3022_HeaterPower_t;

/**
 * @enum hdc3022_TempUnit_t
 * @brief Temperature unit options for HDC3022 sensor readings.
 *
 * This enumeration defines the units in which temperature data can be returned
 * from the sensor measurement functions.
 */
typedef enum {
    HDC3022_TEMP_CELSIUS,   /**< Temperature in degrees Celsius */
    HDC3022_TEMP_FAHRENHEIT /**< Temperature in degrees Fahrenheit */
} hdc3022_TempUnit_t;

/**
 * @enum hdc3022_AlertStatus_t
 * @brief Bit positions for alert and status flags in the 16-bit status register of the HDC3022
 * sensor.
 *
 * The HDC3022 status register provides various alert and status indicators, where each bit
 * corresponds to a specific condition or flag. This enum defines the bit positions for those flags.
 *
 * Detailed bit descriptions:
 * - Bit 15 (HDC3022_ALERT_STATUS): Overall Alert Status
 *     - 0 = No active alerts
 *     - 1 = At least one tracking or reset alert active
 * - Bit 13 (HDC3022_HEATER_STATUS): Heater Status
 *     - 0 = Heater Disabled
 *     - 1 = Heater Enabled
 * - Bit 11 (HDC3022_RH_ALERT_STATUS): Relative Humidity (RH) Tracking Alert (Mirrored on Alert pin)
 *     - 0 = No RH alert
 *     - 1 = RH alert active
 * - Bit 10 (HDC3022_TEMP_ALERT_STATUS): Temperature (T) Tracking Alert (Mirrored on Alert pin)
 *     - 0 = No temperature alert
 *     - 1 = Temperature alert active
 * - Bit 9 (HDC3022_RH_HIGH_ALERT_STATUS): RH High Tracking Alert
 *     - 0 = No RH High alert
 *     - 1 = RH High alert active
 * - Bit 8 (HDC3022_RH_LOW_ALERT_STATUS): RH Low Tracking Alert
 *     - 0 = No RH Low alert
 *     - 1 = RH Low alert active
 * - Bit 7 (HDC3022_TEMP_HIGH_ALERT_STATUS): Temperature High Tracking Alert
 *     - 0 = No T High alert
 *     - 1 = T High alert active
 * - Bit 6 (HDC3022_TEMP_LOW_ALERT_STATUS): Temperature Low Tracking Alert
 *     - 0 = No T Low alert
 *     - 1 = T Low alert active
 * - Bit 4 (HDC3022_DEVICE_RESET_STATUS): Device Reset Detected (Mirrored on Alert pin)
 *     - 0 = No reset detected since last clearing of Status Register
 *     - 1 = Device reset detected (via hard reset, soft reset command, or power supply cycling)
 * - Bit 0 (HDC3022_CHECKSUM_VERIFY_STATUS): Checksum Verification of Last Data Write
 *     - 0 = Pass (correct checksum received)
 *     - 1 = Fail (incorrect checksum received)
 *     - Can be cleared only by a successful data write or a reset event
 *
 * Bits 14, 12, 5, 3, 2, and 1 are reserved and should be ignored.
 */
typedef enum {
    HDC3022_ALERT_STATUS = 15,          /**< Overall alert status */
    HDC3022_HEATER_STATUS = 13,         /**< Heater enabled/disabled status */
    HDC3022_RH_ALERT_STATUS = 11,       /**< Relative humidity tracking alert */
    HDC3022_TEMP_ALERT_STATUS = 10,     /**< Temperature tracking alert */
    HDC3022_RH_HIGH_ALERT_STATUS = 9,   /**< Relative humidity high alert */
    HDC3022_RH_LOW_ALERT_STATUS = 8,    /**< Relative humidity low alert */
    HDC3022_TEMP_HIGH_ALERT_STATUS = 7, /**< Temperature high alert */
    HDC3022_TEMP_LOW_ALERT_STATUS = 6,  /**< Temperature low alert */
    HDC3022_DEVICE_RESET_STATUS = 4,    /**< Device reset detected since last clear */
    HDC3022_CHECKSUM_VERIFY_STATUS = 0  /**< Checksum verification status of last data write */
} hdc3022_AlertStatus_t;

/**
 * @struct hdc3022_t
 * @brief Structure representing an instance of the HDC3022 sensor.
 *
 * This structure holds all necessary information to interact with a specific
 * HDC3022 sensor device over I2C, including the bus instance, device address,
 * and current automatic measurement mode configuration.
 *
 * @param i2c Pointer to the I2C bus instance used for communication.
 * @param i2cAddress 7-bit I2C address of the sensor.
 * @param autoModeEnabled Flag indicating if automatic measurement mode is enabled.
 * @param autoMeasConfig Current automatic measurement mode setting (see @ref
 * hdc3022_AutoMeasMode_t).
 */
typedef struct {
    i2c_inst_t *i2c;                       /**< Pointer to the I2C bus instance */
    uint8_t i2cAddress;                    /**< I2C address of the sensor */
    bool autoModeEnabled;                  /**< Automatic measurement mode enabled flag */
    hdc3022_AutoMeasMode_t autoMeasConfig; /**< Current automatic measurement mode */
} hdc3022_t;

// High level functions
/**
 * @brief Initializes the HDC3022 sensor structure and configures its measurement mode state.
 *
 * This function initializes the given HDC3022 structure by setting the I2C instance,
 * I2C address, and measurement mode state. It **does not write** the `autoMeasConfig` value to the
 * sensor; instead, `autoMeasConfig` should reflect the expected current measurement mode of the
 * sensor.
 *
 * If @p checkDefaultMode is true, the function reads the sensor's actual default measurement mode
 * and updates the structure accordingly, ignoring the `autoMeasConfig` parameter.
 *
 * If @p checkDefaultMode is false, the function uses the provided `autoMeasConfig` value to
 * initialize the structure, assuming the sensor is already configured in that mode.
 *
 * @param[in,out] hdc3022 Pointer to the HDC3022 sensor structure to initialize.
 * @param[in] i2c Pointer to the I2C bus instance used for communication.
 * @param[in] i2cAddress I2C address of the HDC3022 sensor.
 * @param[in] autoMeasConfig Expected automatic measurement mode state of the sensor; this
 *        value is **not written** to the sensor, only used to initialize the structure when
 *        @p checkDefaultMode is false (see @ref hdc3022_AutoMeasMode_t).
 * @param[in] checkDefaultMode If true, the function reads the sensor's actual mode and
 *        updates the structure accordingly, ignoring `autoMeasConfig`.
 *
 * @return true if initialization was successful; false if reading the sensor mode failed.
 */
bool hdc3022_init_struct(hdc3022_t *hdc3022, i2c_inst_t *i2c, uint8_t i2cAddress,
                         hdc3022_AutoMeasMode_t autoMeasConfig, bool checkDefaultMode);

/**
 * @brief Sets the automatic measurement mode of the HDC3022 sensor.
 *
 * This function configures the sensor to operate in the specified automatic measurement mode.
 * If the mode is set to `HDC3022_AUTO_MEAS_NONE`, the function exits the automatic measurement
 * mode.
 *
 * @param[in,out] hdc3022 Pointer to the initialized HDC3022 sensor structure.
 * @param[in] autoMeasConfig The desired automatic measurement mode to set (see @ref
 * hdc3022_AutoMeasMode_t).
 *
 * @return true if the mode was set successfully; false if the command to the sensor failed.
 */
bool hdc3022_set_auto_meas_mode(hdc3022_t *hdc3022, hdc3022_AutoMeasMode_t autoMeasConfig);

/**
 * @brief Disables the automatic measurement mode of the HDC3022 sensor.
 *
 * This function exits the sensor's automatic measurement mode.
 * If the sensor is not currently in automatic mode or the i2c write failed, the function returns
 * false.
 *
 * @param[in,out] hdc3022 Pointer to the initialized HDC3022 sensor structure.
 *
 * @return true if the automatic mode was successfully exited; false otherwise.
 */
bool hdc3022_exit_auto_meas_mode(hdc3022_t *hdc3022);

/**
 * @brief Reads data from the HDC3022 sensor in automatic measurement mode.
 *
 * Sends a readout command to the sensor to retrieve temperature and/or humidity data
 * depending on the specified `readoutCommand` (see @ref hdc3022_AutoMeasReadoutCommand_t for
 * details).
 *
 *
 * @param[in,out] hdc3022 Pointer to the initialized HDC3022 sensor structure.
 * @param[in] readoutCommand Command specifying which data to read.
 * @param[in] tempUnit Unit for temperature conversion (see @ref hdc3022_TempUnit_t).
 * @param[out] temp Pointer to store the temperature value (if applicable).
 * @param[out] humidity Pointer to store the humidity value (if applicable).
 *
 * @return true if the data was successfully read and converted; false otherwise.
 *
 * @note This function returns false immediately if the sensor is currently in automatic measurement
 * mode.
 */
bool hdc3022_readout_auto_meas(hdc3022_t *hdc3022, hdc3022_AutoMeasReadoutCommand_t readoutCommand,
                               hdc3022_TempUnit_t tempUnit, float *temp, float *humidity);

/**
 * @brief Reads temperature and humidity data from the HDC3022 sensor in manual measurement mode.
 *
 * This function sends a manual measurement command to the sensor, waits for the required
 * measurement time depending on the command, and then reads the temperature and humidity data.
 *
 * @param[in] hdc3022 Pointer to the initialized HDC3022 sensor structure.
 * @param[in] readCommand Manual measurement command to send (see @ref hdc3022_ManualMeasMode_t).
 * @param[in] tempUnit Unit for temperature conversion (see @ref hdc3022_TempUnit_t).
 * @param[out] temp Pointer to a float to store the measured temperature.
 * @param[out] humidity Pointer to a float to store the measured humidity.
 *
 * @return true if the measurement and data reading were successful; false otherwise.
 *
 * @note This function returns false immediately if the sensor is currently in automatic measurement
 * mode.
 */
bool hdc3022_readout_manual_meas(hdc3022_t *hdc3022, hdc3022_ManualMeasMode_t readCommand,
                                 hdc3022_TempUnit_t tempUnit, float *temp, float *humidity);

/**
 * @brief Enables or disables the heater in the HDC3022 sensor.
 *
 * This function turns the heater on or off by sending the appropriate command to the sensor.
 *
 * @param[in] hdc3022 Pointer to the initialized HDC3022 sensor structure.
 * @param[in] enabled If true, the heater is enabled; if false, the heater is disabled.
 *
 * @return true if the command was successfully sent to the sensor; false otherwise.
 */
bool hdc3022_set_heater_enabled(hdc3022_t *hdc3022, bool enabled);

/**
 * @brief Sets the heater power level of the HDC3022 sensor.
 *
 * This function configures the heater power level but does **not** enable or disable the heater.
 * To turn the heater on or off, use the function @ref hdc3022_set_heater_enabled().
 *
 * @param[in] hdc3022 Pointer to the initialized HDC3022 sensor structure.
 * @param[in] heaterPower Desired heater power level (see @ref hdc3022_HeaterPower_t).
 *
 * @return true if the command was successfully sent; false otherwise.
 */
bool hdc3022_set_heater_power(hdc3022_t *hdc3022, hdc3022_HeaterPower_t heaterPower);

/**
 * @brief Performs a software reset of the HDC3022 sensor.
 *
 * This function sends a soft reset command that forces the device to return to its default state
 * without removing power. It is equivalent to a hardware reset via power cycling or toggling the
 * RESET pin.
 *
 * When executed, the sensor:
 * - Resets the Status Register
 * - Reloads calibration data and programmed humidity/temperature offset errors from memory
 * - Clears previously stored measurement results
 * - Resets Interrupt Threshold limits to default conditions
 *
 * @param[in] hdc3022 Pointer to the initialized HDC3022 sensor structure.
 *
 * @return true if the reset command was successfully sent; false otherwise.
 *
 * @note Rember to call @ref hdc3022_init_struct() after reset to set the default measurement mode
 * again.
 */
bool hdc3022_soft_reset(hdc3022_t *hdc3022);

/**
 * @brief Clears specific bits in the status register of the HDC3022 sensor.
 *
 * This function sends a command to clear only the reset and tracking bits in the sensor's status
 * register. Other status bits remain unchanged.
 *
 * @param[in] hdc3022 Pointer to the initialized HDC3022 sensor structure.
 *
 * @return true if the clear command was successfully sent; false otherwise.
 */
bool hdc3022_clear_status(hdc3022_t *hdc3022);

/**
 * @brief Reads the full 16-bit status register of the HDC3022 sensor.
 *
 * This function reads the 16-bit status register and stores each bit as a boolean value
 * in the provided array, where index 0 corresponds to bit 0 (least significant bit)
 * and index 15 to bit 15 (most significant bit).
 *
 * The status bits correspond to various alert and status flags defined in @ref
 * hdc3022_AlertStatus_t.
 *
 * @param[in] hdc3022 Pointer to the initialized HDC3022 sensor structure.
 * @param[out] statusArray Boolean array of size 16 where each element represents a bit in the
 * status register.
 *
 * @return true if the status was successfully read; false otherwise.
 */
bool hdc3022_read_status_all(hdc3022_t *hdc3022, bool statusArray[16]);

/**
 * @brief Reads a specific bit from the HDC3022 status register.
 *
 * This function reads the entire 16-bit status register and extracts the value of the bit
 * specified by @p statusPosition.
 *
 * @param[in] hdc3022 Pointer to the initialized HDC3022 sensor structure.
 * @param[in] statusPosition The specific status bit to read (see @ref hdc3022_AlertStatus_t).
 * @param[out] statusValue Pointer to a boolean variable where the bit value will be stored.
 *
 * @return true if the status register was successfully read; false otherwise.
 */
bool hdc3022_read_status(hdc3022_t *hdc3022, hdc3022_AlertStatus_t statusPosition,
                         bool *statusValue);

/**
 * @brief Reads the Manufacturer ID from the HDC3022 sensor.
 *
 * This function sends a command to read the 16-bit Manufacturer ID register.
 * According to the datasheet, the Manufacturer ID for HDC3022 is always 0x3000,
 * which corresponds to Texas Instruments.
 *
 * @param[in] hdc3022 Pointer to the initialized HDC3022 sensor structure.
 *
 * @return The Manufacturer ID read from the sensor, or 0 if the read operation failed.
 */
uint16_t hdc3022_read_manufacturer_id(hdc3022_t *hdc3022);

/**
 * @brief Reads the 6-byte NIST ID from the HDC3022 sensor.
 *
 * This function reads the unique 6-byte NIST identification number stored in the sensor,
 * by sequentially reading three 16-bit parts from specific sensor registers.
 *
 * @param[in] hdc3022 Pointer to the initialized HDC3022 sensor structure.
 * @param[out] nist Array of 6 bytes where the NIST ID will be stored.
 *
 * @return true if the NIST ID was read successfully; false if any read command failed.
 */
bool hdc3022_read_nist_id(hdc3022_t *hdc3022, uint8_t nist[6]);

/**
 * @brief Configures alert threshold registers on the HDC3022 sensor.
 *
 * This function calculates the threshold value from the given temperature and humidity,
 * then writes this threshold.
 *
 * @param[in] hdc3022 Pointer to the initialized HDC3022 sensor structure.
 * @param[in] alertSetCommand Command specifying which alert threshold register to set.
 *            This should be one of the commands in @ref hdc3022_ReadAlertThresholdsCommand_t
 * @param[in] tempUnit Unit for temperature conversion (see @ref hdc3022_TempUnit_t).
 * @param[in] temp Temperature value to set for the alert threshold.
 * @param[in] humidity Humidity value to set for the alert threshold.
 *
 * @return true if the threshold was successfully written; false if the write command failed.
 */
bool hdc3022_config_alert(hdc3022_t *hdc3022,
                          hdc3022_ConfigAlertThresholdsCommand_t alertSetCommand,
                          hdc3022_TempUnit_t tempUnit, float temp, float humidity);

/**
 * @brief Reads alert threshold values from the HDC3022 sensor.
 *
 * This function reads one of the configured alert threshold registers from the sensor
 * and decodes it into temperature and/or humidity values, depending on the type of threshold.
 *
 * @param[in] hdc3022 Pointer to the initialized HDC3022 sensor structure.
 * @param[in] alertReadCommand Command specifying which alert threshold register to read.
 *            This should be one of the commands in @ref hdc3022_ReadAlertThresholdsCommand_t
 * @param[in] tempUnit Unit for temperature conversion (see @ref hdc3022_TempUnit_t).
 * @param[out] temp Pointer to store the decoded temperature threshold value (if applicable).
 * @param[out] humidity Pointer to store the decoded humidity threshold value (if applicable).
 *
 * @return true if the threshold was successfully read and decoded; false if the command failed.
 */

bool hdc3022_read_alert(hdc3022_t *hdc3022, hdc3022_ReadAlertThresholdsCommand_t alertReadCommand,
                        hdc3022_TempUnit_t tempUnit, float *temp, float *humidity);

/**
 * @brief Transfers the currently configured alert threshold registers into non-volatile memory.
 *
 * This function stores the currently configured alert threshold registers
 * into non-volatile memory. The operation is only allowed when the sensor
 * is not in automatic measurement mode.
 *
 * The sensor **must be in sleep mode** before issuing this command. No communication is possible
 * while EEPROM programming is in progress, so the function waits internally for the EEPROM write
 * delay.
 *
 * @param[in] hdc3022 Pointer to the initialized HDC3022 sensor structure.
 *
 * @return true if the thresholds were successfully transferred; false otherwise.
 *
 * @note This function returns false immediately if the sensor is currently in automatic measurement
 * mode.
 * @note This function waits internally for the required EEPROM write delay after sending the
 * transfer command.
 */
bool hdc3022_transfer_alert_thresholds_eeprom(hdc3022_t *hdc3022);

/**
 * @brief Sets the default measurement mode state of the HDC3022 sensor.
 *
 * This function programs the sensor's EEPROM to override the default measurement mode the device
 * enters after power-on or reset. By default, the device enters sleep mode after power-on or reset.
 * Using this function, you can configure the device to start in the desired measurement mode
 * automatically.
 *
 * The sensor **must be in sleep mode** before issuing this command. No communication is possible
 * while EEPROM programming is in progress, so the function waits internally for the EEPROM write
 * delay.
 *
 * @param[in] hdc3022 Pointer to the initialized HDC3022 sensor structure.
 * @param[in] measMode Desired default measurement mode to set (see @ref hdc3022_DefaultMeasMode_t).
 *
 * @return true if the mode was successfully written; false otherwise.
 *
 * @note This function returns false immediately if the sensor is in automatic measurement mode.
 * @note This function waits internally for the required EEPROM write delay after sending the write
 * command.
 */
bool hdc3022_set_default_meas_state(hdc3022_t *hdc3022, hdc3022_DefaultMeasMode_t measMode);

/**
 * @brief Reads the default measurement mode state from the HDC3022 sensor.
 *
 * This function reads the sensor’s stored default measurement mode from
 * non-volatile memory. The default mode defines the sensor’s behavior after
 * power-on or reset.
 *
 * @param[in] hdc3022 Pointer to the initialized HDC3022 sensor structure.
 * @param[out] defaultMode Pointer to a variable where the default measurement
 *                         mode will be stored (see @ref hdc3022_DefaultMeasMode_t).
 *
 * @return true if the default measurement mode was successfully read; false otherwise.
 *
 */
bool hdc3022_read_default_meas_state(hdc3022_t *hdc3022, hdc3022_DefaultMeasMode_t *defaultMode);

/**
 * @brief Writes temperature and humidity offset calibration values to the HDC3022 sensor.
 *
 * This function calculates the offset value based on the provided temperature and humidity,
 * then writes it to the sensor's non-volatile memory to adjust measurements accordingly.
 * It only works if the sensor is **not** in automatic measurement mode.
 *
 * The sensor **must be in sleep mode** before issuing this command. No communication is possible
 * while EEPROM programming is in progress, so the function waits internally for the EEPROM write
 * delay.
 *
 * @param[in] hdc3022 Pointer to the initialized HDC3022 sensor structure.
 * @param[in] tempUnit Unit for temperature conversion (see @ref hdc3022_TempUnit_t).
 * @param[in] temp Temperature offset value to set.
 * @param[in] humidity Humidity offset value to set.
 *
 * @return true if the offset was successfully written; false otherwise.
 *
 * @note This function returns false immediately if the sensor is in automatic measurement mode.
 * @note This function waits internally for the EEPROM write delay after sending the command.
 */
bool hdc3022_write_offsets(hdc3022_t *hdc3022, hdc3022_TempUnit_t tempUnit, float temp,
                           float humidity);

/**
 * @brief Reads temperature and humidity offset calibration values from the HDC3022 sensor.
 *
 * This function reads the offset values stored in the sensor's non-volatile memory and
 * decodes them into temperature and humidity offsets.
 *
 * @param[in] hdc3022 Pointer to the initialized HDC3022 sensor structure.
 * @param[in] tempUnit Unit for temperature conversion (see @ref hdc3022_TempUnit_t).
 * @param[out] temp Pointer to a float to store the read temperature offset.
 * @param[out] humidity Pointer to a float to store the read humidity offset.
 *
 * @return true if the offsets were successfully read; false otherwise.
 */
bool hdc3022_read_offsets(hdc3022_t *hdc3022, hdc3022_TempUnit_t tempUnit, float *temp,
                          float *humidity);
#endif
