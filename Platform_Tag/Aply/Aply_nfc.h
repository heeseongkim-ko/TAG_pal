/**
 * @file Aply_nfc.h
 * @brief Application layer NFC management header
 * @details Header file for NFC application layer functions
 */

#ifndef APLY_NFC_H
#define APLY_NFC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief NFC configuration status codes
 */
typedef enum {
	NFC_CONFIG_STATUS_UNWRITTEN = 0x00,	///< No configuration written yet
	NFC_CONFIG_STATUS_DEFAULT = 0x01,		///< Default configuration only
	NFC_CONFIG_STATUS_EXTERNAL = 0x11		///< External configuration written
} nfc_config_status_t;

/**
 * @brief NFC EEPROM tag configuration data structure
 * @details Contains all tag configuration fields stored in NFC EEPROM
 */
typedef struct {
	uint8_t mac_addr[6];           ///< MAC Address (6 bytes)
	uint8_t platform;               ///< Platform (1 byte)
	uint8_t tag_type;               ///< Tag Type (1 byte)
	uint8_t hw_version[2];          ///< Hardware Version (2 bytes) - Little Endian
	uint8_t sw_version[3];          ///< Software Version (3 bytes)
	uint8_t reserved[3];            ///< Reserved bytes (3 bytes) - Page 7 remaining
	uint8_t refresh_interval[4];    ///< Refresh Interval (4 bytes) - Little Endian
	uint8_t channel;                ///< Channel (1 byte)
	uint8_t rf_profile;            ///< RF Profile (1 byte)
	uint8_t tx_power[4];            ///< TX Power (4 bytes) - Little Endian
	uint8_t sleep_random_dev;       ///< Sleep Random Dev. (1 byte)
	uint8_t sleep_mode;             ///< Sleep Mode (1 byte)
	uint8_t sleep_threshold;        ///< Sleep Threshold (1 byte)
	uint8_t sleep_custom_threshold[2]; ///< Sleep Custom Threshold (2 bytes) - Little Endian
	uint8_t refresh_sleep_interval[4]; ///< Refresh Interval in Sleep (4 bytes) - Little Endian
	uint8_t sleep_motion_interval[4]; ///< Sleep Mode(Motion) Change Interval (4 bytes) - Little Endian
	uint8_t accelerometer;          ///< Accelerometer (1 byte)
	uint8_t accel_range;            ///< Accelerometer Dynamic Range (1 byte)
	uint8_t back_channel_period;    ///< Back Channel Period (1 byte)
	uint8_t back_channel_period_sleep; ///< Back Channel Period in sleep (1 byte)
	uint8_t config_status;          ///< Configuration Status (1 byte)
} __attribute__((packed)) nfc_eeprom_tag_config_data_t;

/**
 * @brief NFC configuration data - starts from page 4
 */
#define NFC_CONFIG_START_PAGE			4		///< Configuration starts from page 4
#define NFC_CONFIG_PAGE_SIZE			4		///< Each page is 4 bytes

/**
 * @brief Read EEPROM page (4 bytes per page)
 * @param page_number Page number to read (starting from page 4)
 * @param data Buffer to store the read data (4 bytes)
 * @return true if read successful, false otherwise
 */
bool Aply_nfc_read_page(uint8_t page_number, uint8_t* data);

/**
 * @brief Write EEPROM page (4 bytes per page)
 * @param page_number Page number to write (starting from page 0)
 * @param data Data to write (4 bytes)
 * @return true if write successful, false otherwise
 */
bool Aply_nfc_write_page(uint8_t page_number, const uint8_t* data);

/**
 * @brief Read all NFC configuration data
 * @param config Pointer to nfc_eeprom_tag_config_data_t structure to store read data
 * @return true if read successful, false otherwise
 */
bool Aply_nfc_read_all_config(nfc_eeprom_tag_config_data_t* config);

/**
 * @brief Parse NFC configuration data for logging
 * @param config Pointer to nfc_eeprom_tag_config_data_t structure containing configuration data
 * @return None
 */
void Aply_nfc_parse_config_log(const nfc_eeprom_tag_config_data_t* config);

/**
 * @brief Write all 11 pages of configuration data
 * @param data Data to write (44 bytes = 11 pages × 4 bytes)
 * @return true if write successful, false otherwise
 */
bool Aply_nfc_write_all_config(const uint8_t* data);

/**
 * @brief Get NFC configuration status
 * @return Current configuration status
 */
nfc_config_status_t Aply_nfc_get_config_status(void);

/**
 * @brief Apply UWB settings from tag configuration to hardware
 * @details This function compares current UWB settings with s_tag_config
 *          and applies changes only if different. Sets flag for reconfiguration.
 * @return true if settings were changed, false if no changes needed
 */
bool Aply_nfc_apply_uwb_settings(void);

/**
 * @brief Get UWB settings change flag status
 * @return true if UWB settings need reconfiguration, false otherwise
 */
bool Aply_nfc_get_uwb_settings_changed(void);

/**
 * @brief Check if UWB settings need reconfiguration and apply if needed
 * @details This function checks the settings change flag and applies
 *          UWB reconfiguration if needed. Called during UWB wakeup.
 */
void Aply_nfc_check_and_apply_uwb_settings(void);

/**
 * @brief Check NFC data and initialize tag configuration
 * @details Checks NFC data first. If MAC address matches current tag and status is 0x01 (Default),
 *          loads configuration from NFC. Otherwise initializes with default values.
 * @return None
 */
void Aply_nfc_check_and_init_tag_configuration(void);

/**
 * @brief Write default tag configuration to NFC EEPROM
 * @details Reads current tag configuration and writes it to NFC EEPROM.
 *          Handles both Read Only (Page 4-7) and R/W (Page 8-14) configuration data.
 *          Sets configuration status to default after successful writing.
 * @return true if successful, false if failed
 */
bool Aply_nfc_write_default_config(void);

#ifdef __cplusplus
}
#endif

#endif // APLY_NFC_H
