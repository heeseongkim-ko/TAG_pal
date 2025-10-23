/**
 * @file Aply_tag_configuration.h
 * @brief Tag configuration module for TEIA platform
 * @author Platform Tag Team
 * @date 2025
 * @version 1.0
 */

#ifndef APLY_TAG_CONFIGURATION_H
#define APLY_TAG_CONFIGURATION_H

#include <stdint.h>
#include <stdbool.h>
#include "def_config.h"
#include "def_packet.h"
#include "Aply_nfc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Tag configuration structure
 * @details Contains all tag information that goes into info packet
 */
typedef struct {
	// System information
	uint8_t platform;					/**< Type of tag */
	uint8_t hw_ver[2];					/**< Hardware version and revision */
	uint8_t fw_ver[3];					/**< Firmware version, subversion and revision */
	
	// UWB configuration
	uint8_t channel;					/**< Used UWB radio channel */
	uint8_t data_rate;					/**< UWB radio data rate */
	uint8_t preamble;					/**< UWB radio - preamble code length */
	uint8_t prf;						/**< UWB radio - pulse repetition frequency */
	uint8_t preamCode;					/**< UWB radio preamble code */
	uint8_t nSfd;						/**< Start frame delimiter */
	uint8_t pac_size;					/**< PAC Size (0=8, 1=16) */
	uint16_t sfd_timeout;				/**< SFD Timeout (129 or 257) */
	uint8_t TXpower[4];					/**< UWB radio transmitter power */
	uint8_t refresh_interval[4];		/**< TDOA Blink refresh rate */
	uint8_t sm_refresh_interval[4];		/**< TDOA Blink refresh rate - no motion sleep mode */
	
	// Motion and power management
	uint8_t mcr;						/**< No motion sleep mode */
	uint8_t motion_sleep_time[4];		/**< Motion sleep entry time in milliseconds */
	uint8_t random_dev;					/**< Random deviation of TDOA blink refreshrate */
	uint8_t mcr_threshold[2];			/**< No motion - acceleration threshold */
	
	// Sensor configuration
	uint8_t mounted_sensors;			/**< Each bit represents one sensor, if set: concrete sensor is mounted on tag */
	uint8_t active_sensors;				/**< Each bit represents one sensor, if set: concrete sensor is active */
	uint8_t IMU_FS_range;				/**< Inertial measurement unit (accelerometer and gyroscope full scale ranges */
	uint8_t BARO_setting;				/**< Barometer precision setting */
	uint8_t AHRS_representation;		/**< Spatial rotation representation. 0 - disabled, 1 - Tait-brian angles, 2 - Quaternion */
	
	// Backchannel configuration
	uint8_t bc_version;					/**< Current version of back-channel protocol */
	uint8_t bc_period_ri;				/**< Back-channel period during normal operation [every Xth refresh interval] */
	uint8_t bc_period_rism;				/**< Back-channel period during sleep mode [every Xth refresh interval in sleep mode] */
} tag_configuration_t;

/**
 * @brief Configuration field enumeration for unified access
 */
typedef enum {
	CONFIG_FIELD_PLATFORM = 0,
	CONFIG_FIELD_CHANNEL,
	CONFIG_FIELD_DATA_RATE,
	CONFIG_FIELD_PREAMBLE,
	CONFIG_FIELD_PRF,
	CONFIG_FIELD_PREAMCODE,
	CONFIG_FIELD_NSFD,
	CONFIG_FIELD_PAC_SIZE,
	CONFIG_FIELD_SFD_TIMEOUT,
	CONFIG_FIELD_MCR,
	CONFIG_FIELD_RANDOM_DEV,
	CONFIG_FIELD_MOUNTED_SENSORS,
	CONFIG_FIELD_ACTIVE_SENSORS,
	CONFIG_FIELD_IMU_FS_RANGE,
	CONFIG_FIELD_BARO_SETTING,
	CONFIG_FIELD_AHRS_REPRESENTATION,
	CONFIG_FIELD_BC_VERSION,
	CONFIG_FIELD_BC_PERIOD_RI,
	CONFIG_FIELD_BC_PERIOD_RISM,
	// Array fields for get_array_field function
	CONFIG_FIELD_HW_VER,
	CONFIG_FIELD_FW_VER,
	CONFIG_FIELD_TX_POWER,
	CONFIG_FIELD_REFRESH_INTERVAL,
	CONFIG_FIELD_SM_REFRESH_INTERVAL,
	CONFIG_FIELD_MOTION_SLEEP_TIME,
	CONFIG_FIELD_MCR_THRESHOLD,
	CONFIG_FIELD_COUNT
} config_field_e;

/**
 * @brief Initialize tag configuration with default values
 * @details Initializes all configuration fields with default values from def_config.h definitions.
 *          Reads current UWB settings from API functions to ensure accurate initial state.
 *          Sets up platform type, hardware/firmware versions, and all subsystem configurations.
 * @return None
 * @note This function must be called before using any other configuration functions.
 *       All UWB API functions must be properly initialized before calling this function.
 */
void Aply_tag_configuration_init_default(void);


/**
 * @brief Update configuration with current UWB settings
 * @details Synchronizes internal configuration with current UWB hardware settings.
 *          Reads channel, data rate, preamble, PRF, preamble code, SFD, and TX power
 *          from UWB API functions and updates the internal configuration structure.
 *          Should be called whenever UWB settings are changed during runtime.
 * @return true if successful, false if not initialized
 * @note This function requires UWB system to be properly initialized and operational.
 */
bool Aply_tag_configuration_sync_uwb_settings(void);

/**
 * @brief Update configuration with current sensor settings
 * @details Synchronizes internal configuration with current sensor status information.
 *          Updates mounted and active sensor bit fields in the configuration structure.
 *          Each bit represents a specific sensor (e.g., accelerometer, gyroscope, barometer).
 *          Should be called when sensor configuration changes during runtime.
 * @param[in] mounted_sensors Bit field indicating which sensors are physically mounted
 * @param[in] active_sensors Bit field indicating which sensors are currently active
 * @return true if successful, false if not initialized
 * @note Sensor bit definitions are found in def_config.h file.
 */
bool Aply_tag_configuration_sync_sensor_settings(uint8_t mounted_sensors, uint8_t active_sensors);

/**
 * @brief Get configuration field value using unified enum-based access
 * @details Provides unified access to any single-byte configuration field using enum identifiers.
 *          Uses switch statement for efficient field selection and value retrieval.
 *          Supports all platform, UWB, sensor, and backchannel configuration fields.
 *          For multi-byte fields, use get_array_field function instead.
 * @param[in] field Configuration field identifier from config_field_e enumeration
 * @return Field value as uint8_t, returns 0 if error or configuration not initialized
 * @note This function is optimized for single-byte fields only.
 *       For array fields (hw_ver, fw_ver, TXpower, etc.), use get_array_field function.
 */
uint8_t Aply_tag_configuration_get_field(config_field_e field);

/**
 * @brief Get SFD timeout value (16-bit)
 * @return SFD timeout value, returns 0 if configuration not initialized
 */
uint16_t Aply_tag_configuration_get_sfd_timeout(void);

/**
 * @brief Set configuration field value using unified enum-based access
 * @details Provides unified access to modify any single-byte configuration field using enum identifiers.
 *          Uses switch statement for efficient field selection and value assignment.
 *          Supports all platform, UWB, sensor, and backchannel configuration fields.
 *          For multi-byte fields, use set_array_field function instead.
 * @param[in] field Configuration field identifier from config_field_e enumeration
 * @param[in] value New field value to be assigned
 * @return true if successful, false if invalid field or configuration not initialized
 * @note This function is optimized for single-byte fields only.
 *       For array fields (hw_ver, fw_ver, TXpower, etc.), use set_array_field function.
 */
bool Aply_tag_configuration_set_field(config_field_e field, uint8_t value);

/**
 * @brief Get array field value for multi-byte configuration fields
 * @details Provides access to multi-byte configuration arrays using enum identifiers and index.
 *          Supports hardware version (2 bytes), firmware version (3 bytes), TX power (4 bytes),
 *          refresh intervals (4 bytes each), and MCR threshold (2 bytes).
 *          Includes bounds checking to prevent array overflow access.
 * @param[in] field Configuration field identifier from config_field_e enumeration
 * @param[in] index Array index (0-based, must be within field array bounds)
 * @return Field value at specified index, returns 0 if error or invalid index
 * @note Array bounds: hw_ver[2], fw_ver[3], TXpower[4], refresh_interval[4], 
 *       sm_refresh_interval[4], mcr_threshold[2]. Index must be within these limits.
 */
uint8_t Aply_tag_configuration_get_array_field(config_field_e field, uint8_t index);

/**
 * @brief Set array field value for multi-byte configuration fields
 * @details Provides access to modify multi-byte configuration arrays using enum identifiers and index.
 *          Supports hardware version (2 bytes), firmware version (3 bytes), TX power (4 bytes),
 *          refresh intervals (4 bytes each), and MCR threshold (2 bytes).
 *          Includes bounds checking to prevent array overflow access and memory corruption.
 * @param[in] field Configuration field identifier from config_field_e enumeration
 * @param[in] index Array index (0-based, must be within field array bounds)
 * @param[in] value New field value to be assigned at specified index
 * @return true if successful, false if invalid field, index out of bounds, or not initialized
 * @note Array bounds: hw_ver[2], fw_ver[3], TXpower[4], refresh_interval[4],
 *       sm_refresh_interval[4], mcr_threshold[2]. Index must be within these limits.
 */
bool Aply_tag_configuration_set_array_field(config_field_e field, uint8_t index, uint8_t value);

/**
 * @brief Load tag configuration from NFC EEPROM data
 * @details Converts NFC EEPROM data structure to internal tag configuration structure.
 *          Handles endianness conversion for multi-byte fields.
 * @param[in] nfc_data Pointer to nfc_eeprom_tag_config_data_t structure from NFC
 * @return None
 */
void Aply_tag_configuration_load_from_nfc(const nfc_eeprom_tag_config_data_t* nfc_data);


#ifdef __cplusplus
}
#endif

#endif // APLY_TAG_CONFIGURATION_H
