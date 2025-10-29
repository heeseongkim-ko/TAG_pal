/**
 * @file Aply_tag_configuration.c
 * @brief Tag configuration module implementation for TEIA platform
 * @author Platform Tag Team
 * @date 2025
 * @version 1.0
 */

#include "Aply_tag_configuration.h"
#include "Aply_nfc.h"
#include "Func_UART_LOG.h"
#include "Api_uwb.h"
#include "Api_uwb_param.h"
#include "Api_motion.h"
#include "def_config.h"
#include "def_packet.h"
#include <string.h>
#include <stdint.h>

/**
 * @brief Static tag configuration storage
 */
static tag_configuration_t s_tag_config;

/**
 * @brief Configuration initialization flag
 */
static bool s_config_initialized = false;

/**
 * @brief Global MAC header structure for UWB communication
 */
mac_header_bb_t g_mac_header;

/**
 * @brief Sensor configuration structure
 */
sensor_config_t s_sensor_config;

/**
 * @brief Forward declaration for calculate_motion_threshold
 */
static uint8_t calculate_motion_threshold(uint8_t sensitivity_level, uint8_t full_scale);
/**
 * @brief Initialize tag configuration with default values or NFC data
 * @details First checks NFC data for valid configuration. If MAC address matches
 *          and status is 0x01 (Default), uses NFC values. Otherwise initializes
 *          with default values from def_config.h definitions.
 *          Reads current UWB settings from API functions to ensure accurate initial state.
 * @return None
 * @note This function must be called before using any other configuration functions.
 *       All UWB API functions must be properly initialized before calling this function.
 */
void Aply_tag_configuration_init_default(void)
{
	// Initialize with default values from def_config.h
	memset(&s_tag_config, 0, sizeof(tag_configuration_t));
	
	// System information (default values)
	s_tag_config.platform = TEIA_CAR;			// From def_config.h
	s_tag_config.hw_ver[0] = HW_VERSION_MAJOR;		// From def_config.h
	s_tag_config.hw_ver[1] = HW_VERSION_MINOR;		// From def_config.h
	s_tag_config.fw_ver[0] = SW_VERSION_MAJOR;		// From def_config.h
	s_tag_config.fw_ver[1] = SW_VERSION_MINOR;		// From def_config.h
	s_tag_config.fw_ver[2] = SW_VERSION_PATCH;		// From def_config.h
	
	// UWB configuration (get from actual UWB settings via API)
	s_tag_config.channel = Api_uwb_get_channel();			// Get actual UWB channel
	s_tag_config.data_rate = Api_uwb_get_data_rate();		// Get actual data rate
	s_tag_config.preamble = Api_uwb_get_preamble_length();	// Get actual preamble
	s_tag_config.prf = Api_uwb_get_prf();					// Get actual PRF
	s_tag_config.preamCode = Api_uwb_get_preamble_code();	// Get actual preamble code
	s_tag_config.nSfd = Api_uwb_get_sfd();					// Get actual SFD
	s_tag_config.pac_size = Api_uwb_get_pac_size();		// Get actual PAC size
	s_tag_config.sfd_timeout = Api_uwb_get_sfd_timeout();	// Get actual SFD timeout
	
	// TX power (get from actual UWB settings)
	Api_uwb_get_tx_power(s_tag_config.TXpower);
	
	// Refresh intervals (default values from def_config.h) - using direct values
	s_tag_config.refresh_interval[0] = (uint8_t)(TX_INTERVAL_MS_DEFAULT & 0xFF);
	s_tag_config.refresh_interval[1] = (uint8_t)((TX_INTERVAL_MS_DEFAULT >> 8) & 0xFF);
	s_tag_config.refresh_interval[2] = (uint8_t)((TX_INTERVAL_MS_DEFAULT >> 16) & 0xFF);
	s_tag_config.refresh_interval[3] = (uint8_t)((TX_INTERVAL_MS_DEFAULT >> 24) & 0xFF);
	
	s_tag_config.sm_refresh_interval[0] = (uint8_t)(TX_INTERVAL_MOTION_SLEEP_MS_DEFAULT & 0xFF);
	s_tag_config.sm_refresh_interval[1] = (uint8_t)((TX_INTERVAL_MOTION_SLEEP_MS_DEFAULT >> 8) & 0xFF);
	s_tag_config.sm_refresh_interval[2] = (uint8_t)((TX_INTERVAL_MOTION_SLEEP_MS_DEFAULT >> 16) & 0xFF);
	s_tag_config.sm_refresh_interval[3] = (uint8_t)((TX_INTERVAL_MOTION_SLEEP_MS_DEFAULT >> 24) & 0xFF);
	
	// Motion and power management
	s_tag_config.mcr = MOTION_SLEEP_ENABLED_DEFAULT;	// From def_config.h
	s_tag_config.motion_sleep_time[0] = (uint8_t)(ACC_TIME_TO_WAIT_INMOTION_DEFAULT & 0xFF);
	s_tag_config.motion_sleep_time[1] = (uint8_t)((ACC_TIME_TO_WAIT_INMOTION_DEFAULT >> 8) & 0xFF);
	s_tag_config.motion_sleep_time[2] = (uint8_t)((ACC_TIME_TO_WAIT_INMOTION_DEFAULT >> 16) & 0xFF);
	s_tag_config.motion_sleep_time[3] = (uint8_t)((ACC_TIME_TO_WAIT_INMOTION_DEFAULT >> 24) & 0xFF);
	s_tag_config.random_dev = RANDOM_TX_ENABLE_DEFAULT;	// From def_config.h
	s_tag_config.mcr_threshold[0] = MCR_THRESHOLD_LOW_BYTE_DEFAULT;		// From def_config.h
	s_tag_config.mcr_threshold[1] = MCR_THRESHOLD_HIGH_BYTE_DEFAULT;	// From def_config.h
	s_tag_config.motion_threshold = MOTION_DETECTION_THRESHOLD;	// From def_config.h
	
	// Sensor configuration (default values)
	s_tag_config.mounted_sensors = SENSOR_DEFAULT_MOTION_MOUNTED;	// From def_config.h
	s_tag_config.active_sensors = SENSOR_DEFAULT_MOTION_ACTIVE;		// From def_config.h
	s_tag_config.IMU_FS_range = SENSOR_DEFAULT_IMU_FS_RANGE;		// From def_config.h
	s_tag_config.BARO_setting = SENSOR_DEFAULT_BARO_SETTING;		// From def_config.h
	s_tag_config.AHRS_representation = SENSOR_DEFAULT_AHRS_REPRESENTATION;	// From def_config.h
	
	// Backchannel configuration
	s_tag_config.bc_version = BACKCHANNEL_VERSION_DEFAULT;		// From def_config.h
	s_tag_config.bc_period_ri = BACKCHANNEL_PACKET_INTERVAL_COUNT_DEFAULT;		// From def_packet.h
	s_tag_config.bc_period_rism = BACKCHANNEL_PACKET_INTERVAL_COUNT_DEFAULT;	// From def_packet.h
	
	s_config_initialized = true;
	
	LOG_API_UWB("Tag Configuration: Initialized with default values\r\n");
}


/**
 * @brief Update configuration with current UWB settings
 * @details Synchronizes internal configuration with current UWB hardware settings.
 *          Reads channel, data rate, preamble, PRF, preamble code, SFD, and TX power
 *          from UWB API functions and updates the internal configuration structure.
 *          Should be called whenever UWB settings are changed during runtime.
 * @return true if successful, false if not initialized
 * @note This function requires UWB system to be properly initialized and operational.
 */
bool Aply_tag_configuration_sync_uwb_settings(void)
{
	if (!s_config_initialized) {
		return false;
	}
	
	// Update UWB settings from current API values
	s_tag_config.channel = Api_uwb_get_channel();
	s_tag_config.data_rate = Api_uwb_get_data_rate();
	s_tag_config.preamble = Api_uwb_get_preamble_length();
	s_tag_config.prf = Api_uwb_get_prf();
	s_tag_config.preamCode = Api_uwb_get_preamble_code();
	s_tag_config.nSfd = Api_uwb_get_sfd();
	Api_uwb_get_tx_power(s_tag_config.TXpower);
	
	LOG_API_UWB("Tag Configuration: UWB settings synchronized\r\n");
	return true;
}

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
bool Aply_tag_configuration_sync_sensor_settings(uint8_t mounted_sensors, uint8_t active_sensors)
{
	if (!s_config_initialized) {
		return false;
	}
	
	// Update sensor settings
	s_tag_config.mounted_sensors = mounted_sensors;
	s_tag_config.active_sensors = active_sensors;
	
	LOG_API_UWB("Tag Configuration: Sensor settings synchronized\r\n");
	return true;
}

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
uint8_t Aply_tag_configuration_get_field(config_field_e field)
{
	if (!s_config_initialized || field >= CONFIG_FIELD_COUNT) {
		return 0;
	}
	
	switch (field) {
		case CONFIG_FIELD_PLATFORM:			return s_tag_config.platform;
		case CONFIG_FIELD_CHANNEL:				return s_tag_config.channel;
		case CONFIG_FIELD_DATA_RATE:			return s_tag_config.data_rate;
		case CONFIG_FIELD_PREAMBLE:			return s_tag_config.preamble;
		case CONFIG_FIELD_PRF:					return s_tag_config.prf;
		case CONFIG_FIELD_PREAMCODE:			return s_tag_config.preamCode;
		case CONFIG_FIELD_NSFD:					return s_tag_config.nSfd;
		case CONFIG_FIELD_PAC_SIZE:				return s_tag_config.pac_size;
		case CONFIG_FIELD_MCR:					return s_tag_config.mcr;
		case CONFIG_FIELD_RANDOM_DEV:			return s_tag_config.random_dev;
		case CONFIG_FIELD_MOUNTED_SENSORS:		return s_tag_config.mounted_sensors;
		case CONFIG_FIELD_ACTIVE_SENSORS:		return s_tag_config.active_sensors;
		case CONFIG_FIELD_IMU_FS_RANGE:		return s_tag_config.IMU_FS_range;
		case CONFIG_FIELD_BARO_SETTING:		return s_tag_config.BARO_setting;
		case CONFIG_FIELD_AHRS_REPRESENTATION:	return s_tag_config.AHRS_representation;
		case CONFIG_FIELD_BC_VERSION:			return s_tag_config.bc_version;
		case CONFIG_FIELD_BC_PERIOD_RI:		return s_tag_config.bc_period_ri;
		case CONFIG_FIELD_BC_PERIOD_RISM:		return s_tag_config.bc_period_rism;
		case CONFIG_FIELD_MOTION_THRESHOLD:	return s_tag_config.motion_threshold;
		default:								return 0;
	}
}

/**
 * @brief Get SFD timeout value (16-bit)
 * @return SFD timeout value, returns 0 if configuration not initialized
 */
uint16_t Aply_tag_configuration_get_sfd_timeout(void)
{
	if (!s_config_initialized) {
		return 0;
	}
	return s_tag_config.sfd_timeout;
}

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
bool Aply_tag_configuration_set_field(config_field_e field, uint8_t value)
{
	if (!s_config_initialized || field >= CONFIG_FIELD_COUNT) {
		return false;
	}
	
	switch (field) {
		case CONFIG_FIELD_PLATFORM:			s_tag_config.platform = value; break;
		case CONFIG_FIELD_CHANNEL:				s_tag_config.channel = value; break;
		case CONFIG_FIELD_DATA_RATE:			s_tag_config.data_rate = value; break;
		case CONFIG_FIELD_PREAMBLE:			s_tag_config.preamble = value; break;
		case CONFIG_FIELD_PRF:					s_tag_config.prf = value; break;
		case CONFIG_FIELD_PREAMCODE:			s_tag_config.preamCode = value; break;
		case CONFIG_FIELD_NSFD:					s_tag_config.nSfd = value; break;
		case CONFIG_FIELD_MCR:					s_tag_config.mcr = value; break;
		case CONFIG_FIELD_RANDOM_DEV:			s_tag_config.random_dev = value; break;
		case CONFIG_FIELD_MOUNTED_SENSORS:		s_tag_config.mounted_sensors = value; break;
		case CONFIG_FIELD_ACTIVE_SENSORS:		s_tag_config.active_sensors = value; break;
		case CONFIG_FIELD_IMU_FS_RANGE:		s_tag_config.IMU_FS_range = value; break;
		case CONFIG_FIELD_BARO_SETTING:		s_tag_config.BARO_setting = value; break;
		case CONFIG_FIELD_AHRS_REPRESENTATION:	s_tag_config.AHRS_representation = value; break;
		case CONFIG_FIELD_BC_VERSION:			s_tag_config.bc_version = value; break;
		case CONFIG_FIELD_BC_PERIOD_RI:		s_tag_config.bc_period_ri = value; break;
		case CONFIG_FIELD_BC_PERIOD_RISM:		s_tag_config.bc_period_rism = value; break;
		case CONFIG_FIELD_MOTION_THRESHOLD:	s_tag_config.motion_threshold = value; break;
		default:								return false;
	}
	
	return true;
}

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
uint8_t Aply_tag_configuration_get_array_field(config_field_e field, uint8_t index)
{
	if (!s_config_initialized) {
		return 0;
	}
	
	switch (field) {
		case CONFIG_FIELD_HW_VER:  // hw_ver[2]
			if (index < 2) return s_tag_config.hw_ver[index];
			break;
			
		case CONFIG_FIELD_FW_VER:   // fw_ver[3] 
			if (index < 3) return s_tag_config.fw_ver[index];
			break;
			
		case CONFIG_FIELD_TX_POWER: // TXpower[4]
			if (index < 4) return s_tag_config.TXpower[index];
			break;
			
		case CONFIG_FIELD_REFRESH_INTERVAL:  // refresh_interval[4]
			if (index < 4) return s_tag_config.refresh_interval[index];
			break;
			
		case CONFIG_FIELD_SM_REFRESH_INTERVAL:       // sm_refresh_interval[4]
			if (index < 4) return s_tag_config.sm_refresh_interval[index];
			break;
			
		case CONFIG_FIELD_MOTION_SLEEP_TIME:        // motion_sleep_time[4]
			if (index < 4) return s_tag_config.motion_sleep_time[index];
			break;
			
		case CONFIG_FIELD_MCR_THRESHOLD: // mcr_threshold[2]
			if (index < 2) return s_tag_config.mcr_threshold[index];
			break;
			
		default:
			break;
	}
	
	return 0;
}

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
bool Aply_tag_configuration_set_array_field(config_field_e field, uint8_t index, uint8_t value)
{
	if (!s_config_initialized) {
		return false;
	}
	
	switch (field) {
		case CONFIG_FIELD_HW_VER:  // hw_ver[2]
			if (index < 2) {
				s_tag_config.hw_ver[index] = value;
				return true;
			}
			break;
			
		case CONFIG_FIELD_FW_VER:   // fw_ver[3]
			if (index < 3) {
				s_tag_config.fw_ver[index] = value;
				return true;
			}
			break;
			
		case CONFIG_FIELD_TX_POWER: // TXpower[4]
			if (index < 4) {
				s_tag_config.TXpower[index] = value;
				return true;
			}
			break;
			
		case CONFIG_FIELD_REFRESH_INTERVAL:  // refresh_interval[4]
			if (index < 4) {
				s_tag_config.refresh_interval[index] = value;
				return true;
			}
			break;
			
		case CONFIG_FIELD_SM_REFRESH_INTERVAL:       // sm_refresh_interval[4]
			if (index < 4) {
				s_tag_config.sm_refresh_interval[index] = value;
				return true;
			}
			break;
			
		case CONFIG_FIELD_MOTION_SLEEP_TIME:        // motion_sleep_time[4]
			if (index < 4) {
				s_tag_config.motion_sleep_time[index] = value;
				return true;
			}
			break;
			
		case CONFIG_FIELD_MCR_THRESHOLD: // mcr_threshold[2]
			if (index < 2) {
				s_tag_config.mcr_threshold[index] = value;
				return true;
			}
			break;
			
		default:
			break;
	}
	
	return false;
}

/**
 * @brief Load tag configuration from NFC EEPROM data
 * @details Converts NFC EEPROM data structure to internal tag configuration structure.
 *          Handles endianness conversion for multi-byte fields.
 * @param nfc_data Pointer to nfc_eeprom_tag_config_data_t structure from NFC
 * @return None
 */
void Aply_tag_configuration_load_from_nfc(const nfc_eeprom_tag_config_data_t* nfc_data)
{
	if (nfc_data == NULL) {
		return;
	}
	
	// Initialize with zeros first
	memset(&s_tag_config, 0, sizeof(tag_configuration_t));
	
	// System information (default values) - same as init_default
	s_tag_config.platform = TEIA_CAR;			// From def_config.h
	s_tag_config.hw_ver[0] = HW_VERSION_MAJOR;		// From def_config.h
	s_tag_config.hw_ver[1] = HW_VERSION_MINOR;		// From def_config.h
	s_tag_config.fw_ver[0] = SW_VERSION_MAJOR;		// From def_config.h
	s_tag_config.fw_ver[1] = SW_VERSION_MINOR;		// From def_config.h
	s_tag_config.fw_ver[2] = SW_VERSION_PATCH;		// From def_config.h
	
	// UWB configuration - use NFC RF profile settings
	switch (nfc_data->rf_profile) {
		case 0x04: // RF4
			s_tag_config.data_rate = 1;        // 850k
			s_tag_config.preamble = 5;         // 256 bits
			s_tag_config.nSfd = 1;             // Non-standard
			s_tag_config.pac_size = 1;         // 16 symbols
			s_tag_config.sfd_timeout = 257;    // Longer timeout
			break;
		case 0x05: // RF5
			s_tag_config.data_rate = 2;        // 6M8
			s_tag_config.preamble = 6;         // 128 bits
			s_tag_config.nSfd = 0;             // Standard
			s_tag_config.pac_size = 0;         // 8 symbols
			s_tag_config.sfd_timeout = 129;    // Shorter timeout
			break;
		default:
			// Fallback to current UWB settings
			s_tag_config.data_rate = Api_uwb_get_data_rate();
			s_tag_config.preamble = Api_uwb_get_preamble_length();
			s_tag_config.nSfd = Api_uwb_get_sfd();
			s_tag_config.pac_size = Api_uwb_get_pac_size();
			s_tag_config.sfd_timeout = Api_uwb_get_sfd_timeout();
			break;
	}
	
	// Other UWB settings from NFC
	s_tag_config.channel = nfc_data->channel;              // From NFC
	s_tag_config.prf = Api_uwb_get_prf();                 // Keep current PRF
	s_tag_config.preamCode = Api_uwb_get_preamble_code();  // Keep current preamble code
	
	// TX power from NFC (Big Endian to Little Endian conversion)
	s_tag_config.TXpower[0] = nfc_data->tx_power[3];      // LSB
	s_tag_config.TXpower[1] = nfc_data->tx_power[2];
	s_tag_config.TXpower[2] = nfc_data->tx_power[1];
	s_tag_config.TXpower[3] = nfc_data->tx_power[0];      // MSB
	
	// Refresh intervals (from NFC - Little Endian to Big Endian conversion)
	s_tag_config.refresh_interval[0] = nfc_data->refresh_interval[3]; // MSB
	s_tag_config.refresh_interval[1] = nfc_data->refresh_interval[2];
	s_tag_config.refresh_interval[2] = nfc_data->refresh_interval[1];
	s_tag_config.refresh_interval[3] = nfc_data->refresh_interval[0]; // LSB
	
	s_tag_config.sm_refresh_interval[0] = nfc_data->refresh_sleep_interval[3]; // MSB
	s_tag_config.sm_refresh_interval[1] = nfc_data->refresh_sleep_interval[2];
	s_tag_config.sm_refresh_interval[2] = nfc_data->refresh_sleep_interval[1];
	s_tag_config.sm_refresh_interval[3] = nfc_data->refresh_sleep_interval[0]; // LSB
	
	// Motion and power management (from NFC)
	s_tag_config.mcr = nfc_data->sleep_mode;
	s_tag_config.motion_sleep_time[0] = nfc_data->sleep_motion_interval[3]; // MSB
	s_tag_config.motion_sleep_time[1] = nfc_data->sleep_motion_interval[2];
	s_tag_config.motion_sleep_time[2] = nfc_data->sleep_motion_interval[1];
	s_tag_config.motion_sleep_time[3] = nfc_data->sleep_motion_interval[0]; // LSB
	s_tag_config.random_dev = nfc_data->sleep_random_dev;
	
	// Motion threshold from sleep_threshold
	s_tag_config.motion_threshold = nfc_data->sleep_threshold;
	
	// Get threshold from sleep_custom_threshold when sleep_threshold is 4
	if (nfc_data->sleep_threshold == 4) {
		s_tag_config.mcr_threshold[0] = nfc_data->sleep_custom_threshold[1];
		s_tag_config.mcr_threshold[1] = nfc_data->sleep_custom_threshold[0];
	}
	// When sleep_threshold < 4, use sleep_threshold - 1
	else if (nfc_data->sleep_threshold < 4) {
		s_tag_config.mcr_threshold[0] = nfc_data->sleep_threshold;
		s_tag_config.mcr_threshold[1] = nfc_data->sleep_custom_threshold[1];
	}
	
	// Sensor configuration (from NFC)
	//s_tag_config.mounted_sensors = SENSOR_DEFAULT_MOTION_MOUNTED;	// Keep default
	s_tag_config.active_sensors = SENSOR_DEFAULT_MOTION_ACTIVE;		// Keep default
	s_tag_config.IMU_FS_range = nfc_data->accel_range;
	s_tag_config.BARO_setting = SENSOR_DEFAULT_BARO_SETTING;		// Keep default
	s_tag_config.AHRS_representation = SENSOR_DEFAULT_AHRS_REPRESENTATION;	// Keep default
	
	// Backchannel configuration (from NFC)
	s_tag_config.bc_version = BACKCHANNEL_VERSION_DEFAULT;		// Keep default
	s_tag_config.bc_period_ri = nfc_data->back_channel_period;
	s_tag_config.bc_period_rism = nfc_data->back_channel_period_sleep;
	
	s_config_initialized = true;
	
	LOG_API_UWB("Tag Configuration: Loaded from NFC EEPROM data\r\n");
}

/**
 * @brief Generate and store UWB MAC address
 * @details Generates a unique MAC address for UWB communication and stores it
 *          in the global MAC header structure. Sets default frame code and
 *          initializes sequence number.
 * @return None
 * @note This function must be called after UWB system initialization.
 *       The generated MAC address is stored in g_mac_header.MAC_addr.
 */
void Aply_get_uwb_mac_address(void)
{
	if (Api_uwb_compose_MAC_address(g_mac_header.MAC_addr)) {
		g_mac_header.fcode = UWB_FCODE_BLINK;  // Set default frame code
		g_mac_header.seqNum = 0;               // Initialize sequence number
		
		//printf_uart("MAC address generated and stored: ");
		//for (int i = 0; i < MAC_ADDR_BYTE_SIZE; i++) {
		//	printf_uart("%02X", g_mac_header.MAC_addr[i]);
		//	if (i < MAC_ADDR_BYTE_SIZE - 1) printf_uart(":");
		//printf_uart("\r\n");
		//}
	} else {
		//printf_uart("Failed to generate MAC address\r\n");
	}
}

/**
 * @brief Apply motion sensor configuration from tag configuration
 * @details Reads full scale and motion threshold from tag configuration,
 *          calculates threshold value, and applies to motion sensor hardware
 * @return None
 */
void Aply_tag_configuration_set_motion_config(void)
{
	uint8_t l_full_scale = Aply_tag_configuration_get_field(CONFIG_FIELD_IMU_FS_RANGE);
	uint8_t l_motion_threshold;
	
	// Calculate motion threshold based on sensitivity level and full scale
	l_motion_threshold = calculate_motion_threshold(
		Aply_tag_configuration_get_field(CONFIG_FIELD_MOTION_THRESHOLD),
		l_full_scale
	);
	
	// Apply full scale and threshold to motion sensor hardware
	Api_motion_set_full_scale(l_full_scale);
	Api_motion_set_threshold(l_motion_threshold);
}

/**
 * @brief Calculate motion threshold value based on sensitivity level and full scale range
 * @param[in] sensitivity_level 0=48mg, 1=256mg, 2=1024mg, 3=custom value
 * @param[in] full_scale Full scale range (0=2g, 1=4g, 2=8g, 3=16g)
 * @return Calculated threshold value
 */
static uint8_t calculate_motion_threshold(uint8_t sensitivity_level, uint8_t full_scale)
{
	uint16_t l_mg_value = 0;
	uint16_t l_threshold = 0;
	
	// Convert sensitivity level to mg value
	switch (sensitivity_level) {
		case 1:
			l_mg_value = 48;   // 48mg
			break;
		case 2:
			l_mg_value = 256;  // 256mg
			break;
		case 3:
			l_mg_value = 1024; // 1024mg
			break;
		case 4:
			// Custom value - use mcr_threshold as uint16_t and convert to mg
			// Little endian: mcr_threshold[1] is high byte, mcr_threshold[0] is low byte
			l_mg_value = (s_tag_config.mcr_threshold[1] << 8) | s_tag_config.mcr_threshold[0];
			break;
		default:
			l_mg_value = 48;   // Default: 48mg
			break;
	}
	
	// Calculate threshold based on full scale range
	// LIS2DH12: +/-2g (1 LSB = 16mg), +/-4g (1 LSB = 32mg), +/-8g (1 LSB = 64mg), +/-16g (1 LSB = 192mg)
	switch (full_scale) {
		case 0: // +/-2g, 1 LSB = 16mg
			l_threshold = (uint16_t)(l_mg_value / 16);
			break;
		case 1: // +/-4g, 1 LSB = 32mg
			l_threshold = (uint16_t)(l_mg_value / 32);
			break;
		case 2: // +/-8g, 1 LSB = 64mg
			l_threshold = (uint16_t)(l_mg_value / 64);
			break;
		case 3: // +/-16g, 1 LSB = 192mg
			l_threshold = (uint16_t)(l_mg_value / 192);
			break;
		default:
			// Default to +/-2g for invalid full_scale
			l_threshold = (uint16_t)(l_mg_value / 16);
			break;
	}
	
	// Ensure threshold is within valid range (1-127)
	if (l_threshold == 0) {
		l_threshold = 1;
	}
	if (l_threshold > 127) {
		l_threshold = 127;
	}
	
	return (uint8_t)l_threshold;
}

/**
 * @brief Initialize LIS2DH12 for motion detection
 * @details Configures LIS2DH12 accelerometer for low-power motion detection
 * @return true if initialization successful, false otherwise
 */
bool Aply_tag_configuration_init_motion_detection(void)
{
    motion_error_t result;
    motion_config_t config;
    
    // Configure motion sensor settings
    config.i2c_address = MOTION_DEFAULT_I2C_ADDR;  // Default I2C address
    config.output_data_rate = MOTION_ODR_25HZ;  // 25Hz ODR
    config.full_scale = Aply_tag_configuration_get_field(CONFIG_FIELD_IMU_FS_RANGE);   // Get from configuration
    // Calculate motion threshold based on sensitivity level and full scale
    config.motion_threshold = calculate_motion_threshold(
		Aply_tag_configuration_get_field(CONFIG_FIELD_MOTION_THRESHOLD),
		config.full_scale
	);
    config.motion_duration = MOTION_DETECTION_DURATION;    // 40ms duration
    config.enable_xyz_axes = true;  // Enable X, Y, Z axes
    
    // Initialize motion sensor with our configuration
    result = Api_motion_init(&config);
	
    if (result != MOTION_SUCCESS) {
        printf_uart("Motion sensor init failed: %d\r\n", result);
        return false;
    }
    else {
        s_sensor_config.motion_sensor_mounted |= LIS2DH12;
        s_tag_config.mounted_sensors = s_sensor_config.motion_sensor_mounted;  // Update tag config
    }
    
    // Disable motion detection interrupt (polling mode)
	result = Api_motion_disable_interrupt();
    if (result != MOTION_SUCCESS) {
        printf_uart("Interrupt disable failed: %d\r\n", result);
        return false;
    }
    
    
    printf_uart("Motion detection initialized successfully (polling mode)\r\n");
    return true;
}

