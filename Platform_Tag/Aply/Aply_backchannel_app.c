/**
 * @file Aply_backchannel_app.c
 * @brief Backchannel application module implementation for TEIA platform
 * @author Platform Tag Team
 * @date 2025
 * @version 1.0
 */

#include "Aply_backchannel_app.h"
#include "Aply_uwb_rx.h"
#include "Aply_nfc.h"
#include "Aply_tag_configuration.h"
#include "Aply_tag_scheduler.h"
#include "Aply_uwb_tx.h"
#include "Api_Led.h"
#include "Func_UART_LOG.h"
#include <string.h>

// Static variables
static backchannel_parsed_data_t s_parsed_data;
static bool s_uwb_settings_changed = false;

// Backchannel LED control flag
static bool s_bc_led_blink_requested = false;
static set_led_blinky_params_t s_bc_led_params;

/**
 * @brief Parse BD packet data
 * @details Parses BD packet into structured data
 * @return true if parsing successful, false otherwise
 */
bool Aply_backchannel_parse_bd_packet(void)
{
    // Initialize parsed data
    memset(&s_parsed_data, 0, sizeof(s_parsed_data));
    s_parsed_data.valid = false;
    
    // Get buffer and length
    const uint8_t* buffer = Aply_uwb_rx_get_buffer();
    uint16_t length = Aply_uwb_rx_get_buffer_length();
    
    // Validate buffer
    if (buffer == NULL || length == 0 || length < 14) {
        return false;
    }
    
                // Parse payload if available
                if (length > 14) {
                    const uint8_t* payload = &buffer[14];
                    uint16_t payload_length = length - 14;
                    
                    if (payload_length >= 5) {
                        s_parsed_data.ack_response_count = payload[0]; // ACK response count
                        s_parsed_data.msg_type = payload[1];            // Message type (0x64)
                        s_parsed_data.app_id = payload[2] | (payload[3] << 8); // App ID (little endian)
                        s_parsed_data.data_length = payload[4];        // Data length
            
            // Check if APP_ID is supported
            if (s_parsed_data.app_id == BACKCHANNEL_APP_ID_LED_CTRL ||
                s_parsed_data.app_id == BACKCHANNEL_APP_ID_UWB_SET) {
                
                // Validate data length based on APP_ID
                bool l_valid_length = false;
                if (s_parsed_data.app_id == BACKCHANNEL_APP_ID_LED_CTRL) {
                    // LED control: 6 bytes (action_id + timeout + period + pw)
                    l_valid_length = (s_parsed_data.data_length == 6);
                } else if (s_parsed_data.app_id == BACKCHANNEL_APP_ID_UWB_SET) {
                    // UWB_SET: 23 bytes
                    l_valid_length = (s_parsed_data.data_length == 23);
                }
                
                            if (l_valid_length && s_parsed_data.data_length <= (payload_length - 5)) {
                                s_parsed_data.raw_data = &payload[5];
                    
                    // Set action flag based on APP_ID
                    if (s_parsed_data.app_id == BACKCHANNEL_APP_ID_LED_CTRL) {
                        s_parsed_data.action_flag = BACKCHANNEL_ACTION_LED_CTRL;
                    } else if (s_parsed_data.app_id == BACKCHANNEL_APP_ID_UWB_SET) {
                        s_parsed_data.action_flag = BACKCHANNEL_ACTION_UWB_SET;
                    }
                } else {
                    s_parsed_data.raw_data = NULL;
                    return false; // Invalid data length
                }
            } else {
                // Unsupported APP_ID
                s_parsed_data.raw_data = NULL;
                return false;
            }
        }
    }
    
    s_parsed_data.valid = true;
    return true;
}

/**
 * @brief Process LED control command
 * @details Processes LED control data from backchannel and applies LED blink pattern
 */
void Aply_backchannel_process_led_ctrl(void)
{
    if (!s_parsed_data.valid || s_parsed_data.raw_data == NULL) {
        return;
    }
    
  
    Aply_tag_scheduler_set_led_counter_backchannel();
    // Set flag for tag manager
    s_bc_led_blink_requested = true;
}

/**
 * @brief Process UWB_SET command
 * @details Processes UWB configuration data from backchannel and updates NFC
 */
void Aply_backchannel_process_uwb_set(void)
{
    if (!s_parsed_data.valid || s_parsed_data.raw_data == NULL) {
        return;
    }
    
    // Parse UWB_SET data from raw_data
    const uint8_t* data = s_parsed_data.raw_data;
    
    // Parse individual fields from raw_data
    uint8_t sleep_mode = data[0];
    uint8_t random = data[1];
    uint8_t acc_level = data[2];
    
    
    // Update tag configuration with parsed UWB_SET data
    Aply_tag_configuration_set_field(CONFIG_FIELD_MCR, sleep_mode);
    Aply_tag_configuration_set_field(CONFIG_FIELD_RANDOM_DEV, random);
    // acc_level is threshold, store in motion_threshold field
    Aply_tag_configuration_set_field(CONFIG_FIELD_MOTION_THRESHOLD, acc_level);
    
    // TX Power (4 bytes) - Little Endian
    Aply_tag_configuration_set_array_field(CONFIG_FIELD_TX_POWER, 0, data[6]); // MSB
    Aply_tag_configuration_set_array_field(CONFIG_FIELD_TX_POWER, 1, data[5]);
    Aply_tag_configuration_set_array_field(CONFIG_FIELD_TX_POWER, 2, data[4]);
    Aply_tag_configuration_set_array_field(CONFIG_FIELD_TX_POWER, 3, data[3]); // LSB
    
    // TX Period (4 bytes) - Little Endian
    Aply_tag_configuration_set_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 0, data[10]); // MSB
    Aply_tag_configuration_set_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 1, data[9]);
    Aply_tag_configuration_set_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 2, data[8]);
    Aply_tag_configuration_set_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 3, data[7]); // LSB
    
    // Sleep IN Period (4 bytes) - Little Endian
    Aply_tag_configuration_set_array_field(CONFIG_FIELD_SM_REFRESH_INTERVAL, 0, data[14]); // MSB
    Aply_tag_configuration_set_array_field(CONFIG_FIELD_SM_REFRESH_INTERVAL, 1, data[13]);
    Aply_tag_configuration_set_array_field(CONFIG_FIELD_SM_REFRESH_INTERVAL, 2, data[12]);
    Aply_tag_configuration_set_array_field(CONFIG_FIELD_SM_REFRESH_INTERVAL, 3, data[11]); // LSB
    
    // Sleep TX Period (4 bytes) - Little Endian
    Aply_tag_configuration_set_array_field(CONFIG_FIELD_MOTION_SLEEP_TIME, 0, data[18]); // MSB
    Aply_tag_configuration_set_array_field(CONFIG_FIELD_MOTION_SLEEP_TIME, 1, data[17]);
    Aply_tag_configuration_set_array_field(CONFIG_FIELD_MOTION_SLEEP_TIME, 2, data[16]);
    Aply_tag_configuration_set_array_field(CONFIG_FIELD_MOTION_SLEEP_TIME, 3, data[15]); // LSB
    
    // Write updated configuration to NFC
    Aply_nfc_write_default_config();
    
    // Apply UWB settings from updated configuration
    Aply_nfc_apply_uwb_settings();
    
    // Reinitialize packet scheduler with new configuration
    Aply_tag_scheduler_init();
    
    // Update TX packets with new configuration
    Aply_uwb_tx_update_packets();
    
    // Set info packet flag to send updated configuration
    Aply_uwb_tx_set_packet_flags(true, false, false); // info=true, battery=false, bc=false
    
    // Set UWB settings changed flag
    s_uwb_settings_changed = true;
}

/**
 * @brief Process parsed backchannel data
 * @details Routes to appropriate handler based on action_flag
 */
void Aply_backchannel_process_data(void)
{
    if (!s_parsed_data.valid) {
        return;
    }
    
    switch (s_parsed_data.action_flag) {
        case BACKCHANNEL_ACTION_LED_CTRL:
            Aply_backchannel_process_led_ctrl();
            break;
        case BACKCHANNEL_ACTION_UWB_SET:
            Aply_backchannel_process_uwb_set();
            break;
        default:
            // Unknown action flag
            break;
    }
    
    // Clear parsed data after processing
    s_parsed_data.valid = false;
    s_parsed_data.action_flag = 0;
    s_parsed_data.raw_data = NULL;
}

/**
 * @brief Get UWB settings changed flag status
 * @return true if UWB settings need reconfiguration, false otherwise
 */
bool Aply_backchannel_get_uwb_settings_changed(void)
{
    return s_uwb_settings_changed;
}

/**
 * @brief Clear UWB settings changed flag
 * @details Clears the flag after UWB settings have been applied
 */
void Aply_backchannel_clear_uwb_settings_changed(void)
{
	s_uwb_settings_changed = false;
}

bool Aply_backchannel_is_led_blink_requested(void)
{
	return s_bc_led_blink_requested;
}

set_led_blinky_params_t* Aply_backchannel_get_led_params(void)
{
	return &s_bc_led_params;
}

void Aply_backchannel_clear_led_blink_requested(void)
{
	s_bc_led_blink_requested = false;
}

