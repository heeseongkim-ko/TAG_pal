/**
 * @file Aply_nfc.c
 * @brief NFC application layer implementation
 * @author Platform Tag Team
 * @date 2025
 */

#include "Aply_nfc.h"
#include "Api_nfc.h"
#include "Api_uwb.h"
#include "Api_uwb_param.h"
#include "Aply_tag_configuration.h"
#include "Func_UART_LOG.h"
#include "def_packet.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Write default tag configuration to NFC EEPROM
 * @details Reads current tag configuration and writes it to NFC EEPROM.
 *          Uses structure-based approach for better maintainability.
 * @return true if successful, false if failed
 */
bool Aply_nfc_write_default_config(void)
{
	nfc_eeprom_tag_config_data_t config;
	memset(&config, 0, sizeof(config));
	
	printf_uart("=== Writing Default Configuration to NFC ===\r\n");
	
	// MAC Address (6 bytes) - Get from global MAC header
	extern mac_header_bb_t g_mac_header;
	memcpy(config.mac_addr, g_mac_header.MAC_addr, 6);
	
	// Platform (1 byte)
	config.platform = Aply_tag_configuration_get_field(CONFIG_FIELD_PLATFORM);
	
	// Tag Type (1 byte) - Set default value from def_config.h
	config.tag_type = TAG_TYPE_NORMAL; // Normal Tag
	
	// Hardware Version (2 bytes) - Big Endian
	config.hw_version[0] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_HW_VER, 0);
	config.hw_version[1] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_HW_VER, 1);
	
	// Software Version (3 bytes)
	config.sw_version[0] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_FW_VER, 0);
	config.sw_version[1] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_FW_VER, 1);
	config.sw_version[2] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_FW_VER, 2);
	
	// Reserved bytes (3 bytes) - Page 7 remaining
	config.reserved[0] = 0x00;
	config.reserved[1] = 0x00;
	config.reserved[2] = 0x00;
	
	// Refresh Interval (4 bytes) - Little Endian
	config.refresh_interval[0] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 3); // LSB
	config.refresh_interval[1] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 2);
	config.refresh_interval[2] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 1);
	config.refresh_interval[3] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 0); // MSB
	
	// Channel (1 byte)
	config.channel = Aply_tag_configuration_get_field(CONFIG_FIELD_CHANNEL);
	
	// RF Profile (1 byte) - Convert from individual UWB settings
	uint8_t data_rate = Aply_tag_configuration_get_field(CONFIG_FIELD_DATA_RATE);
	uint8_t preamble_length = Aply_tag_configuration_get_field(CONFIG_FIELD_PREAMBLE);
	uint8_t nsfd = Aply_tag_configuration_get_field(CONFIG_FIELD_NSFD);
	uint8_t pac_size = Aply_tag_configuration_get_field(CONFIG_FIELD_PAC_SIZE);
	uint16_t sfd_timeout = Aply_tag_configuration_get_sfd_timeout();
	
	// Convert individual settings to RF profile
	if (data_rate == 1 && preamble_length == 5 && nsfd == 1 && pac_size == 1 && sfd_timeout == 257) {
		config.rf_profile = 0x04; // RF4
	} else if (data_rate == 2 && preamble_length == 6 && nsfd == 0 && pac_size == 0 && sfd_timeout == 129) {
		config.rf_profile = 0x05; // RF5
	} else {
		config.rf_profile = 0x05; // Default to RF5
	}
	
	// TX Power (4 bytes) - Big Endian
	config.tx_power[0] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_TX_POWER, 0);
	config.tx_power[1] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_TX_POWER, 1);
	config.tx_power[2] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_TX_POWER, 2);
	config.tx_power[3] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_TX_POWER, 3);
	
	// Sleep Random Dev. (1 byte)
	config.sleep_random_dev = Aply_tag_configuration_get_field(CONFIG_FIELD_RANDOM_DEV);
	
	// Sleep Mode (1 byte) - Map from mcr
	config.sleep_mode = Aply_tag_configuration_get_field(CONFIG_FIELD_MCR);
	
	// Sleep Threshold (1 byte) - Get motion threshold from configuration
	config.sleep_threshold = Aply_tag_configuration_get_field(CONFIG_FIELD_MOTION_THRESHOLD);
	
	// Sleep Custom Threshold (2 bytes) - Big Endian
	config.sleep_custom_threshold[0] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_MCR_THRESHOLD, 0);
	config.sleep_custom_threshold[1] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_MCR_THRESHOLD, 1);
	
	// Refresh Interval in Sleep (4 bytes) - Little Endian
	config.refresh_sleep_interval[0] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_SM_REFRESH_INTERVAL, 3); // LSB
	config.refresh_sleep_interval[1] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_SM_REFRESH_INTERVAL, 2);
	config.refresh_sleep_interval[2] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_SM_REFRESH_INTERVAL, 1);
	config.refresh_sleep_interval[3] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_SM_REFRESH_INTERVAL, 0); // MSB
	
	// Sleep Mode(Motion) Change Interval (4 bytes) - Little Endian
	config.sleep_motion_interval[0] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_MOTION_SLEEP_TIME, 3); // LSB
	config.sleep_motion_interval[1] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_MOTION_SLEEP_TIME, 2);
	config.sleep_motion_interval[2] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_MOTION_SLEEP_TIME, 1);
	config.sleep_motion_interval[3] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_MOTION_SLEEP_TIME, 0); // MSB
	
	
	// Accelerometer Dynamic Range (1 byte) - Map from IMU_FS_range
	config.accel_range = Aply_tag_configuration_get_field(CONFIG_FIELD_IMU_FS_RANGE);
	
	// Back Channel Period (1 byte)
	config.back_channel_period = Aply_tag_configuration_get_field(CONFIG_FIELD_BC_PERIOD_RI);
	
	// Back Channel Period in sleep (1 byte)
	config.back_channel_period_sleep = Aply_tag_configuration_get_field(CONFIG_FIELD_BC_PERIOD_RISM);
	
	// Configuration Status (last byte)
	config.config_status = NFC_CONFIG_STATUS_DEFAULT;
	
	// Write all configuration data to NFC using structure
	nfc_result_t result = Api_nfc_write_data(NFC_CONFIG_START_PAGE * NFC_CONFIG_PAGE_SIZE, (uint8_t*)&config, sizeof(config));
	if (result != NFC_SUCCESS) {
		printf_uart("NFC write failed with error: %d\r\n", result);
		return false;
	}
	
	printf_uart("=== Default Configuration Written to NFC Successfully ===\r\n");
	return true;
}

/**
 * @brief Read all NFC configuration data
 * @details Reads 44 bytes of configuration data from NFC EEPROM starting from page 4.
 *          Uses structure-based approach for better maintainability.
 * @param config Pointer to nfc_config_data_t structure to store read data
 * @return true if successful, false if failed
 */
bool Aply_nfc_read_all_config(nfc_eeprom_tag_config_data_t* config)
{
	if (config == NULL) {
		return false;
	}
	
	//printf_uart("=== Reading NFC Configuration Data ===\r\n");
	
	// Read all configuration data from NFC (44 bytes)
	nfc_result_t result = Api_nfc_read_data(NFC_CONFIG_START_PAGE * NFC_CONFIG_PAGE_SIZE, (uint8_t*)config, sizeof(nfc_eeprom_tag_config_data_t));
	if (result != NFC_SUCCESS) {
		//printf_uart("NFC read failed with error: %d\r\n", result);
		return false;
	}
	
	//printf_uart("=== NFC Configuration Data Read Successfully ===\r\n");
	return true;
}

/**
 * @brief Parse NFC configuration data for logging
 * @details Parses the nfc_config_data_t structure and logs all configuration fields
 *          with proper formatting and Big Endian conversion.
 * @param config Pointer to nfc_config_data_t structure containing configuration data
 * @return None
 */
void Aply_nfc_parse_config_log(const nfc_eeprom_tag_config_data_t* config)
{
	if (config == NULL) {
		printf_uart("Invalid configuration data\r\n");
		return;
	}
	
	printf_uart("=== NFC Configuration Data ===\r\n");
	
	// MAC Address (6 bytes)
	printf_uart("MAC Address: ");
	for(int i = 0; i < 6; ++i) {
		printf_uart("%02X", config->mac_addr[i]);
		if(i < 5) printf_uart(":");
	}
	printf_uart("\r\n");
	
	// Platform (1 byte)
	printf_uart("Platform: %d\r\n", config->platform);
	
	// Tag Type (1 byte)
	printf_uart("Tag Type: %d\r\n", config->tag_type);
	
	// Hardware Version (2 bytes) - Big Endian
	printf_uart("Hardware Version: %d.%d\r\n", config->hw_version[0], config->hw_version[1]);
	
	// Software Version (3 bytes)
	printf_uart("Software Version: %d.%d.%d\r\n", config->sw_version[0], config->sw_version[1], config->sw_version[2]);
	
	// Refresh Interval (4 bytes) - Big Endian
	uint32_t refresh_interval = (config->refresh_interval[0] << 24) | 
	                           (config->refresh_interval[1] << 16) | 
	                           (config->refresh_interval[2] << 8) | 
	                           config->refresh_interval[3];
	printf_uart("Refresh Interval: %lu ms\r\n", refresh_interval);
	
	// Channel (1 byte)
	printf_uart("Channel: %d\r\n", config->channel);
	
	// RF Profile (1 byte)
	printf_uart("RF Profile: %d\r\n", config->rf_profile);
	
	// TX Power (4 bytes) - Big Endian
	uint32_t tx_power = (config->tx_power[0] << 24) | 
	                   (config->tx_power[1] << 16) | 
	                   (config->tx_power[2] << 8) | 
	                   config->tx_power[3];
	printf_uart("TX Power: 0x%08lX\r\n", tx_power);
	
	// Sleep Random Dev. (1 byte)
	printf_uart("Sleep Random Dev.: %d\r\n", config->sleep_random_dev);
	
	// Sleep Mode (1 byte)
	printf_uart("Sleep Mode: %d\r\n", config->sleep_mode);
	
	// Sleep Threshold (1 byte)
	printf_uart("Sleep Threshold: %d\r\n", config->sleep_threshold);
	
	// Sleep Custom Threshold (2 bytes) - Big Endian
	uint16_t sleep_custom_threshold = (config->sleep_custom_threshold[0] << 8) | config->sleep_custom_threshold[1];
	printf_uart("Sleep Custom Threshold: %d\r\n", sleep_custom_threshold);
	
	// Refresh Interval in Sleep (4 bytes) - Big Endian
	uint32_t refresh_sleep_interval = (config->refresh_sleep_interval[0] << 24) | 
	                                 (config->refresh_sleep_interval[1] << 16) | 
	                                 (config->refresh_sleep_interval[2] << 8) | 
	                                 config->refresh_sleep_interval[3];
	printf_uart("Refresh Interval in Sleep: %lu ms\r\n", refresh_sleep_interval);
	
	// Sleep Mode(Motion) Change Interval (4 bytes) - Big Endian
	uint32_t sleep_motion_interval = (config->sleep_motion_interval[0] << 24) | 
	                               (config->sleep_motion_interval[1] << 16) | 
	                               (config->sleep_motion_interval[2] << 8) | 
	                               config->sleep_motion_interval[3];
	printf_uart("Sleep Mode(Motion) Change Interval: %lu ms\r\n", sleep_motion_interval);
	
	
	// Accelerometer Dynamic Range (1 byte)
	printf_uart("Accelerometer Dynamic Range: %d\r\n", config->accel_range);
	
	// Back Channel Period (1 byte)
	printf_uart("Back Channel Period: %d\r\n", config->back_channel_period);
	
	// Back Channel Period in sleep (1 byte)
	printf_uart("Back Channel Period in sleep: %d\r\n", config->back_channel_period_sleep);
	
	// Configuration Status (1 byte)
	printf_uart("Configuration Status: 0x%02X\r\n", config->config_status);
}

// Static flag to indicate UWB settings need reconfiguration
static bool s_uwb_settings_changed = false;

/**
 * @brief Apply UWB settings from tag configuration to hardware
 * @details This function compares RF profile settings and TX power with s_tag_config
 *          and applies changes only if different. Sets flag for reconfiguration.
 * @return true if settings were changed, false if no changes needed
 */
bool Aply_nfc_apply_uwb_settings(void)
{
	bool settings_changed = false;  // Initialize local flag
	
	// Compare RF profile settings (data rate, preamble, nsfd)
	if (Api_uwb_get_data_rate() != Aply_tag_configuration_get_field(CONFIG_FIELD_DATA_RATE)) {
		settings_changed = true;
	}
	
	if (Api_uwb_get_preamble_length() != Aply_tag_configuration_get_field(CONFIG_FIELD_PREAMBLE)) {
		settings_changed = true;
	}
	
	if (Api_uwb_get_sfd() != Aply_tag_configuration_get_field(CONFIG_FIELD_NSFD)) {
		settings_changed = true;
	}
	
	// Compare PAC size
	if (Api_uwb_get_pac_size() != Aply_tag_configuration_get_field(CONFIG_FIELD_PAC_SIZE)) {
		settings_changed = true;
	}
	
	// Compare SFD timeout
	if (Api_uwb_get_sfd_timeout() != Aply_tag_configuration_get_sfd_timeout()) {
		settings_changed = true;
	}
	
	// Compare TX power (4 bytes from s_tag_config)
	uint8_t current_tx_power[4];
	uint8_t config_tx_power[4];
	Api_uwb_get_tx_power(current_tx_power);
	
	// Get TX power from s_tag_config using get_array_field
	config_tx_power[0] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_TX_POWER, 0);
	config_tx_power[1] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_TX_POWER, 1);
	config_tx_power[2] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_TX_POWER, 2);
	config_tx_power[3] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_TX_POWER, 3);
	
	if (memcmp(current_tx_power, config_tx_power, 4) != 0) {
		settings_changed = true;
	}
	
	// Set flag if settings changed
	if (settings_changed) {
		s_uwb_settings_changed = true;
	}
	
	return settings_changed;
}

/**
 * @brief Get UWB settings change flag status
 * @return true if UWB settings need reconfiguration, false otherwise
 */
bool Aply_nfc_get_uwb_settings_changed(void)
{
	return s_uwb_settings_changed;
}

/**
 * @brief Check if UWB settings need reconfiguration and apply if needed
 * @details This function checks the settings change flag and applies
 *          UWB reconfiguration if needed. Called during UWB wakeup.
 */
void Aply_nfc_check_and_apply_uwb_settings(void)
{
	if (s_uwb_settings_changed) {
		// Apply RF profile settings (RF4/RF5)
		Api_uwb_set_data_rate(Aply_tag_configuration_get_field(CONFIG_FIELD_DATA_RATE));
		Api_uwb_set_preamble_length(Aply_tag_configuration_get_field(CONFIG_FIELD_PREAMBLE));
		Api_uwb_set_sfd(Aply_tag_configuration_get_field(CONFIG_FIELD_NSFD));
		
		// Apply PAC size and SFD timeout
		Api_uwb_set_pac_size(Aply_tag_configuration_get_field(CONFIG_FIELD_PAC_SIZE));
		Api_uwb_set_sfd_timeout(Aply_tag_configuration_get_sfd_timeout());
		
		// Apply TX power
		uint8_t tx_power[4];
		tx_power[0] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_TX_POWER, 0);
		tx_power[1] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_TX_POWER, 1);
		tx_power[2] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_TX_POWER, 2);
		tx_power[3] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_TX_POWER, 3);
		Api_uwb_set_tx_power(tx_power);
		
		// Reconfigure UWB with new settings
		//Api_uwb_start_init();
		
		// Clear flag only after successful processing
		s_uwb_settings_changed = false;
	}
}

/**
 * @brief Check NFC data and initialize tag configuration
 * @details Checks NFC data first. If MAC address matches current tag and status is 0x01 (Default),
 *          loads configuration from NFC. Otherwise initializes with default values.
 * @return None
 */
void Aply_nfc_check_and_init_tag_configuration(void)
{
	// Step 1: Read NFC data first
	nfc_eeprom_tag_config_data_t nfc_tag_config;
	bool nfc_read_success = Aply_nfc_read_all_config(&nfc_tag_config);
	
	if (!nfc_read_success) {
		//printf_uart("Tag Configuration: NFC read failed, using defaults\r\n");
		Aply_tag_configuration_init_default();
		return;
	}
	
	// Step 2: Get current MAC address for comparison
	extern mac_header_bb_t g_mac_header;
	
	// Step 3: Compare MAC addresses
	bool mac_match = true;
	for (int i = 0; i < 6; i++) {
		if (nfc_tag_config.mac_addr[i] != g_mac_header.MAC_addr[i]) {
			mac_match = false;
			break;
		}
	}
	
	// Step 4: Check MAC match, status, and TX interval
	// Check if TX interval is 0 (invalid)
	uint32_t tx_interval = (nfc_tag_config.refresh_interval[3] << 24) | 
	                      (nfc_tag_config.refresh_interval[2] << 16) | 
	                      (nfc_tag_config.refresh_interval[1] << 8) | 
	                      nfc_tag_config.refresh_interval[0];
	bool tx_interval_valid = (tx_interval != 0);
	
	if (mac_match && tx_interval_valid) {
		//printf_uart("Tag Configuration: Using NFC data (MAC match)\r\n");
		Aply_tag_configuration_load_from_nfc(&nfc_tag_config);
	} else {
		//printf_uart("Tag Configuration: NFC data invalid, using defaults\r\n");
		Aply_tag_configuration_init_default();
		// Write default configuration to NFC after initialization
		Aply_nfc_write_default_config();
	}
}
