/**
 * @file Aply_uwb_tx.c
 * @brief UWB TX module implementation for TEIA platform
 * @author Platform Tag Team
 * @date 2025
 * @version 1.0
 */

#include "Aply_uwb_tx.h"
#include "Aply_uwb_rx.h"
#include "Func_UART_LOG.h"
#include "Aply_tag_configuration.h"
#include "def_packet.h"
#include "Api_uwb.h"
#include "Api_battery.h"
#include <stdbool.h>
#include <string.h>

/**
 * @brief UWB TX components for packet assembly
 */
static mac_header_bb_t s_mac_header;            /**< MAC header with MAC address */
static uint8_t s_info_payload[100];             /**< Info payload buffer */
static uint8_t s_battery_payload[10];           /**< Battery payload buffer */
static uint8_t s_tx_buffer[128];                /**< Final TX buffer */
static uint8_t s_bc_buffer[7];                  /**< BC header extension buffer */

static uint16_t s_info_payload_length = 0;     /**< Length of info payload */
static uint16_t s_battery_payload_length = 0;  /**< Length of battery payload */
static uint8_t s_sequence_number = 0;          /**< Sequence number for packet transmission */
static bool s_header_initialized = false;      /**< Flag indicating if MAC header is initialized */
static packet_flags_t s_packet_flags;          /**< Packet transmission flags */
static bool s_last_packet_was_bc = false;      /**< Flag indicating if last prepared packet was BC */

/**
 * @brief Update pre-built packets with current configuration
 * @details Rebuilds all packet templates using current tag configuration values.
 *          Should be called when configuration changes or during initialization.
 *          Uses s_tag_config to populate packet fields.
 * @return true if successful, false if configuration not available
 * @note This function is called only when configuration changes, not on every transmission.
 */
bool Aply_uwb_tx_update_packets(void)
{
	// Initialize MAC header if not done yet
	if (!s_header_initialized) {
		// Get actual MAC address from DW3000 chip
		if (!Api_uwb_compose_MAC_address(s_mac_header.MAC_addr)) {
			// Fallback to default MAC address if composition fails
			s_mac_header.MAC_addr[0] = 0x00;
			s_mac_header.MAC_addr[1] = 0x01;
			s_mac_header.MAC_addr[2] = 0x02;
			s_mac_header.MAC_addr[3] = 0x03;
			s_mac_header.MAC_addr[4] = 0x04;
			s_mac_header.MAC_addr[5] = 0x05;
		}
		s_header_initialized = true;
	}
	
	// Update payload buffers
	s_info_payload_length = Aply_uwb_tx_build_info_payload();
	s_battery_payload_length = Aply_uwb_tx_build_battery_payload();
	
	return true;
}

/**
 * @brief Prepare UWB packet for transmission based on flags
 * @details Follows original structure: check flags > update header > prepare payload > compose > API
 * @return Total packet length in bytes, 0 if error
 * @note This function checks packet flags and prepares appropriate packet type
 */
uint16_t Aply_uwb_tx_prepare_packet(void)
{
	// Initialize components if not done yet
	if (!s_header_initialized) {
		if (!Aply_uwb_tx_update_packets()) {
			return 0;
		}
	}
	
	// 1. Header decision (BC or BB) - check backchannel flag first
	if (s_packet_flags.bc_packet_flag) {
		s_mac_header.fcode = UWB_FCODE_BC_POLL;
		s_packet_flags.bc_packet_flag = false;  // Clear BC packet flag
		s_last_packet_was_bc = true;  // Mark as BC packet
		
		// Configure BC header extension buffer
		s_bc_buffer[0] = 0x01;   // BC_version
		s_bc_buffer[1] = 0x00;   // BC_ack_num[0]
		s_bc_buffer[2] = 0x00;   // BC_ack_num[1]
		s_bc_buffer[3] = 0x00;   // BC_ack_num[2]
		s_bc_buffer[4] = 0x00;   // BC_ack_num[3]
		s_bc_buffer[5] = 0x00;   // BC_ack_num[4]
		s_bc_buffer[6] = 100;    // BC_T_toRX
	} else {
		s_mac_header.fcode = UWB_FCODE_BLINK;
		s_last_packet_was_bc = false;  // Mark as regular packet
	}
	s_mac_header.seqNum = s_sequence_number++;
	
	// 2. Payload decision (Battery or Info) - check payload flags
	const uint8_t* l_payload_data = NULL;
	uint8_t l_payload_length = 0;
	
	// Battery checked first (more frequent transmission)
	if (s_packet_flags.battery_packet_flag) {
		l_payload_data = s_battery_payload;
		l_payload_length = s_battery_payload_length;
		// Update battery voltage before transmission
		s_battery_payload[4] = Api_battery_getRawData();  // Update real-time voltage
		s_packet_flags.battery_packet_flag = false;
	} else if (s_packet_flags.info_packet_flag) {
		l_payload_data = s_info_payload;
		l_payload_length = s_info_payload_length;
		s_packet_flags.info_packet_flag = false;
	} else {
		// No payload (default blink)
		l_payload_data = NULL;
		l_payload_length = 0;
	}
	
	// 3. Move to TX buffer (compose header + payload)
	uint16_t l_total_length = 0;
	uint16_t l_header_length = sizeof(mac_header_bb_t);
	
	// Extend header length for BC packets
	if (s_last_packet_was_bc) {
		l_header_length += 7; // Add BC extension header
	}
	
	uint16_t l_required_size = l_header_length + l_payload_length;
	
	if (l_required_size <= sizeof(s_tx_buffer)) {
		// Copy MAC header
		memcpy(s_tx_buffer, &s_mac_header, sizeof(mac_header_bb_t));
		l_total_length = sizeof(mac_header_bb_t);
		
		// Add BC extension data between MAC header and payload
		if (s_last_packet_was_bc) {
			memcpy(s_tx_buffer + l_total_length, s_bc_buffer, 7);
			l_total_length += 7;
		}
		
		// Copy payload if provided
		if (l_payload_data != NULL && l_payload_length > 0) {
			memcpy(s_tx_buffer + l_total_length, l_payload_data, l_payload_length);
			l_total_length += l_payload_length;
		}
		
		// 4. Set UWB transmission buffer
		Api_uwb_set_tx_message(s_tx_buffer, l_total_length);
	}
	
	return l_total_length;
}

/**
 * @brief Build tag_info_msg_t structure from configuration
 * @details Fills tag_info_msg_t structure with current system configuration values.
 *          Uses centralized s_tag_config data.
 * @param[out] info_data Pointer to tag_info_msg_t structure to fill
 * @return None
 */
static void Aply_uwb_tx_build_tag_info_data(tag_info_msg_t* info_data)
{
	if (info_data == NULL) {
		return;
	}
	
	// Battery voltage (dynamic)
	info_data->battVoltage = Api_battery_getRawData();
	
	// Platform and versions
	info_data->platform = Aply_tag_configuration_get_field(CONFIG_FIELD_PLATFORM);
	info_data->hw_ver[0] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_HW_VER, 0);
	info_data->hw_ver[1] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_HW_VER, 1);
	info_data->fw_ver[0] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_FW_VER, 0);
	info_data->fw_ver[1] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_FW_VER, 1);
	info_data->fw_ver[2] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_FW_VER, 2);
	
	// UWB settings
	info_data->channel = Aply_tag_configuration_get_field(CONFIG_FIELD_CHANNEL);
	info_data->data_rate = Aply_tag_configuration_get_field(CONFIG_FIELD_DATA_RATE);
	info_data->preamble = Aply_tag_configuration_get_field(CONFIG_FIELD_PREAMBLE);
	info_data->prf = Aply_tag_configuration_get_field(CONFIG_FIELD_PRF);
	info_data->preamCode = Aply_tag_configuration_get_field(CONFIG_FIELD_PREAMCODE);
	info_data->nSfd = Aply_tag_configuration_get_field(CONFIG_FIELD_NSFD);
	
	// System settings
	info_data->mcr = Aply_tag_configuration_get_field(CONFIG_FIELD_MCR);
	info_data->random_dev = Aply_tag_configuration_get_field(CONFIG_FIELD_RANDOM_DEV);
	
	// Refresh intervals (4 bytes each)
	info_data->refresh_interval[0] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 0);
	info_data->refresh_interval[1] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 1);
	info_data->refresh_interval[2] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 2);
	info_data->refresh_interval[3] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 3);
	info_data->sm_refresh_interval[0] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_SM_REFRESH_INTERVAL, 0);
	info_data->sm_refresh_interval[1] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_SM_REFRESH_INTERVAL, 1);
	info_data->sm_refresh_interval[2] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_SM_REFRESH_INTERVAL, 2);
	info_data->sm_refresh_interval[3] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_SM_REFRESH_INTERVAL, 3);
	
	// TX power (4 bytes)
	info_data->TXpower[0] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_TX_POWER, 0);
	info_data->TXpower[1] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_TX_POWER, 1);
	info_data->TXpower[2] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_TX_POWER, 2);
	info_data->TXpower[3] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_TX_POWER, 3);
	
	// Sensor settings
	info_data->mounted_sensors = Aply_tag_configuration_get_field(CONFIG_FIELD_MOUNTED_SENSORS);
	info_data->active_sensors = Aply_tag_configuration_get_field(CONFIG_FIELD_ACTIVE_SENSORS);
	
	// MCR threshold (2 bytes)
	uint8_t l_motion_threshold = Aply_tag_configuration_get_field(CONFIG_FIELD_MOTION_THRESHOLD);
	if (l_motion_threshold == 4) {
		// Custom value: no offset
		info_data->mcr_threshold[0] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_MCR_THRESHOLD, 0);
	} else {
		// Normal value: subtract 1
		info_data->mcr_threshold[0] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_MCR_THRESHOLD, 0) - 1;
	}
	info_data->mcr_threshold[1] = Aply_tag_configuration_get_array_field(CONFIG_FIELD_MCR_THRESHOLD, 1);
	
	// Sensor ranges and settings
	info_data->IMU_FS_range = Aply_tag_configuration_get_field(CONFIG_FIELD_IMU_FS_RANGE);
	info_data->BARO_setting = Aply_tag_configuration_get_field(CONFIG_FIELD_BARO_SETTING);
	info_data->AHRS_representation = Aply_tag_configuration_get_field(CONFIG_FIELD_AHRS_REPRESENTATION);
	
	// Backchannel settings
	info_data->bc_version = Aply_tag_configuration_get_field(CONFIG_FIELD_BC_VERSION);
	info_data->bc_period_ri = Aply_tag_configuration_get_field(CONFIG_FIELD_BC_PERIOD_RI);
	info_data->bc_period_rism = Aply_tag_configuration_get_field(CONFIG_FIELD_BC_PERIOD_RISM);
}

/**
 * @brief Build info payload using current configuration
 * @details Creates info payload data using s_tag_config values.
 *          Builds complete tag_info_msg_t structure with backchannel wrapper.
 * @return Length of built payload in bytes
 */
uint16_t Aply_uwb_tx_build_info_payload(void)
{
	// Fixed structure - info payload header (first 4 bytes)
	s_info_payload[0] = USER_APP_MESSAGE_TYPE;     // Message type (0x64)
	s_info_payload[1] = MSGTYPE_INFO_HIGH_BYTE;    // App ID high byte
	s_info_payload[2] = MSGTYPE_INFO_LOW_BYTE;     // App ID low byte
	s_info_payload[3] = sizeof(tag_info_msg_t);    // Length field
	
	// Build tag_info_msg_t structure using s_tag_config
	tag_info_msg_t l_info_data;
	Aply_uwb_tx_build_tag_info_data(&l_info_data);
	
	// Copy complete structure to payload buffer
	memcpy(&s_info_payload[INFO_PAYLOAD_HEADER_LENGTH], &l_info_data, sizeof(tag_info_msg_t));
	s_info_payload_length = INFO_PAYLOAD_HEADER_LENGTH + sizeof(tag_info_msg_t);
	
	return s_info_payload_length;
}

/**
 * @brief Build battery payload 
 * @details Creates battery payload data with backchannel wrapper.
 *          Includes message type, app ID, length, and battery voltage.
 * @return Length of built payload in bytes
 */
uint16_t Aply_uwb_tx_build_battery_payload(void)
{
	// Fixed structure - battery payload is always 5 bytes (header + data)
	s_battery_payload[0] = USER_APP_MESSAGE_TYPE;        // Message type (0x64)
	s_battery_payload[1] = MSGTYPE_BATT_HIGH_BYTE;       // App ID high byte
	s_battery_payload[2] = MSGTYPE_BATT_LOW_BYTE;        // App ID low byte  
	s_battery_payload[3] = BATTERY_PAYLOAD_LENGTH_DEFAULT; // Length (0x01)
	s_battery_payload[4] = Api_battery_getRawData();                // Battery voltage placeholder
	
	s_battery_payload_length = BATTERY_PAYLOAD_TOTAL_LENGTH;  // Fixed length (5 bytes)
	
	return s_battery_payload_length;
}

/**
 * @brief Set packet transmission flags
 * @details Sets the internal packet flags for controlling packet transmission types.
 * @param info_flag Flag for info packet transmission
 * @param battery_flag Flag for battery packet transmission  
 * @param bc_flag Flag for backchannel packet transmission
 * @return None
 */
void Aply_uwb_tx_set_packet_flags(bool info_flag, bool battery_flag, bool bc_flag)
{
	s_packet_flags.info_packet_flag = info_flag;
	s_packet_flags.battery_packet_flag = battery_flag;
	s_packet_flags.bc_packet_flag = bc_flag;
}

/**
 * @brief Get current packet transmission flags
 * @details Retrieves the current packet flags by copying them to the provided structure.
 * @param flags Pointer to packet_flags_t structure to receive the current flags
 * @return None
 */
void Aply_uwb_tx_get_packet_flags(packet_flags_t* flags)
{
	if (flags != NULL) {
		flags->info_packet_flag = s_packet_flags.info_packet_flag;
		flags->battery_packet_flag = s_packet_flags.battery_packet_flag;
		flags->bc_packet_flag = s_packet_flags.bc_packet_flag;
	}
}

/**
 * @brief Reset all packet transmission flags
 * @details Resets all packet flags to false, disabling all packet types.
 * @return None
 */
void Aply_uwb_tx_reset_packet_flags(void)
{
	s_packet_flags.info_packet_flag = false;
	s_packet_flags.battery_packet_flag = false;
	s_packet_flags.bc_packet_flag = false;
}

/**
 * @brief Check if last prepared packet was BC packet
 * @details Returns whether the last packet prepared by Aply_uwb_tx_prepare_packet() was a BC packet.
 * @return true if last packet was BC, false if regular packet
 */
bool Aply_uwb_tx_was_last_packet_bc(void)
{
	return s_last_packet_was_bc;
}

/**
 * @brief Send ACK packet in response to BD packet
 * @return true if ACK packet was sent successfully, false otherwise
 */
bool Aply_uwb_tx_send_ack_packet(void)
{
	// Get ACK number from received BD packet
	uint8_t ack_number[6];
	if (!Aply_uwb_rx_get_ack_number(ack_number))
	{
		return false; // No ACK number available
	}
	
	// Create BA packet: BA + MAC + SEQ + ACK_NUMBER
	uint8_t ba_packet[14];
	
	ba_packet[0] = 0xBA; // BA packet type
	memcpy(&ba_packet[1], s_mac_header.MAC_addr, 6); // MAC address from existing header
	ba_packet[7] = s_sequence_number++; // Sequence number
	memcpy(&ba_packet[8], ack_number, 6); // ACK number from BD packet
	
	// Set TX message and prepare for transmission (don't start TX here)
	Api_uwb_set_tx_message(ba_packet, 14);
	
	// Reset BC flag after ACK packet preparation
	s_last_packet_was_bc = false;
	
	return true;
}

/**
 * @brief Clear BC packet flag
 * @details Resets the BC packet flag to false
 */
void Aply_uwb_tx_clear_bc_flag(void)
{
	s_last_packet_was_bc = false;
}

