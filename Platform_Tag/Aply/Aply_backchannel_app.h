/**
 * @file Aply_backchannel_app.h
 * @brief Backchannel application module for TEIA platform
 * @author Platform Tag Team
 * @date 2025
 * @version 1.0
 */

#ifndef APLY_BACKCHANNEL_APP_H
#define APLY_BACKCHANNEL_APP_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Backchannel parsed data structure
typedef struct {
    uint8_t packet_type;       // BD (0xBD)
    uint8_t mac_addr[6];      // MAC address
    uint8_t sequence;          // Sequence number
    uint8_t ack_number[6];    // ACK number
    uint8_t ack_response_count; // ACK 응답 개수 (0-15)
    uint8_t msg_type;          // Message type (0x64)
    uint16_t app_id;           // Application ID
    uint8_t data_length;       // Command data length
    const uint8_t* raw_data;   // Pointer to raw command data
    uint8_t action_flag;       // Action flag for processing
    bool valid;                // Parsing validity
} backchannel_parsed_data_t;

// LED control command structure
typedef struct {
    uint8_t action_id;    // 02 = SET_LED_BLINKY_ID
    uint16_t timeout;     // 70 17 = 0x1770 = 6000
    uint16_t period;      // 0F 00 = 0x000F = 15
    uint8_t pw;           // 10 = 16
} set_led_blinky_params_t;

// UWB_SET data structure (23 bytes)
typedef struct {
    uint8_t sleep_mode;       // 1 Byte - Sleep mode
    uint8_t random;           // 1 Byte - Random value
    uint8_t acc_level;        // 1 Byte - Acceleration level
    uint32_t tx_power;        // 4 Bytes - TX power
    uint32_t tx_period;       // 4 Bytes - TX period
    uint32_t sleep_in_period; // 4 Bytes - Sleep in period
    uint32_t sleep_tx_period; // 4 Bytes - Sleep TX period
    uint32_t crc32;           // 4 Bytes - CRC32 checksum
} uwb_set_data_t;

// Backchannel Application IDs
#define BACKCHANNEL_APP_ID_LED_CTRL        0x0400
#define BACKCHANNEL_APP_ID_UWB_SET         0x0800

// Backchannel Action Flags
#define BACKCHANNEL_ACTION_LED_CTRL        0x01
#define BACKCHANNEL_ACTION_UWB_SET         0x02

// Function declarations
bool Aply_backchannel_parse_bd_packet(void);
void Aply_backchannel_process_led_ctrl(void);
void Aply_backchannel_process_uwb_set(void);
void Aply_backchannel_process_data(void);
bool Aply_backchannel_get_uwb_settings_changed(void);
void Aply_backchannel_clear_uwb_settings_changed(void);
bool Aply_backchannel_is_led_blink_requested(void);
set_led_blinky_params_t* Aply_backchannel_get_led_params(void);
void Aply_backchannel_clear_led_blink_requested(void);

#ifdef __cplusplus
}
#endif

#endif // APLY_BACKCHANNEL_APP_H
