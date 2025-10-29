/**
 * @file Aply_uwb_rx.c
 * @brief UWB RX management for Aply module
 * @author Platform Tag Team
 * @date 2025-01-20
 */

#include "Aply_uwb_rx.h"
#include "Api_uwb.h"
#include "Aply_tag_configuration.h"
#include "Func_UART_LOG.h"
#include <string.h>

// Static variables for RX data
static uint8_t s_rx_buffer[100];
static uint16_t s_rx_data_length = 0;
static bool s_bc_data_received = false;
static uint8_t s_bc_ack_number[6]; // ACK number extracted from BD packet

// Static variables for RX processing
static uint8_t s_rx_temp_buffer[100]; // Temporary RX buffer
static uint16_t s_rx_temp_length = 0;

// Static variables for backchannel RX timeout
static uint32_t s_bc_rx_timeout_counter = 0;  // 1ms = 1 tick (default), 100ms = 100 ticks (when own MAC received)
static bool s_bc_rx_timeout_enabled = false;

/**
 * @brief Get tag MAC address (shared with Aply_uwb_tx)
 * @param[out] mac_addr 6-byte MAC address array
 * @return true if successful, false if failed
 */
static bool get_tag_mac_address(uint8_t* mac_addr)
{
	// Use the MAC address already generated in main.c
	extern mac_header_bb_t g_mac_header;
	memcpy(mac_addr, g_mac_header.MAC_addr, 6);
	return true;
}

/**
 * @brief Reset backchannel RX timeout to 100ms
 * @details Called when own MAC address is received
 */
static void reset_bc_rx_timeout(void)
{
	s_bc_rx_timeout_counter = 100;  // Reset to 3ms when own MAC received
	s_bc_rx_timeout_enabled = true;
}

/**
 * @brief Process received RX data
 * @return true if valid BC data was processed, false otherwise
 */
bool Aply_uwb_rx_process_data(void)
{
	// Step 1: Check if data is available
	if (!Api_uwb_is_rx_data_available()) {
		return false; // No data available
	}
	
	// Step 2: Get data via API and copy to local buffer
	uwb_rx_message_t rx_msg;
	if (!Api_uwb_get_rx_message(&rx_msg)) {
		return false; // Failed to get RX data
	}
	
	// Copy data to local buffer
	s_rx_temp_length = rx_msg.length;
	if (s_rx_temp_length > sizeof(s_rx_temp_buffer)) {
		s_rx_temp_length = sizeof(s_rx_temp_buffer); // Limit to buffer size
	}
	memcpy(s_rx_temp_buffer, rx_msg.data, s_rx_temp_length);
	
	// Step 2.5: Check FCODE (only process BC_DATA)
	if (s_rx_temp_buffer[0] != 0xBD) { // UWB_FCODE_BC_DATA
		return false; // Not BC_DATA
	}
	
	// Step 3: Check MAC address
	// Get tag MAC address (cached after first call)
	uint8_t my_mac_addr_bytes[6];
	if (!get_tag_mac_address(my_mac_addr_bytes)) {
		return false; // MAC address initialization failed
	}
	
	// Convert MAC addresses to uint64_t for comparison
	uint64_t dest_addr = 0;
	uint64_t my_mac_addr = 0;
	memcpy(&dest_addr, &s_rx_temp_buffer[1], 6);
	memcpy(&my_mac_addr, my_mac_addr_bytes, 6);
	dest_addr &= 0xFFFFFFFFFFFFULL; // 48-bit mask
	my_mac_addr &= 0xFFFFFFFFFFFFULL; // 48-bit mask
	
	// Check if packet is addressed to this tag (MAC address match) or broadcast
	bool is_broadcast = (dest_addr == 0xFFFFFFFFFFFFULL);
	bool is_my_mac = (dest_addr == my_mac_addr);
	
	if (!is_broadcast && !is_my_mac) {
		// Not for this tag - clear buffer to allow other data reception
		Api_uwb_clear_rx_message();
		return false; // Not for this tag
	}
	
	// Step 4: Parse BD packet and extract ACK number
	if (s_rx_temp_length >= 14) { // BD packet minimum length: BD(1) + MAC(6) + SEQ(1) + ACK(6) = 14
		// Extract ACK number from BD packet (bytes 8-13)
		memcpy(s_bc_ack_number, &s_rx_temp_buffer[8], 6);
		
		memcpy(s_rx_buffer, s_rx_temp_buffer, s_rx_temp_length);
		s_rx_data_length = s_rx_temp_length;
		s_bc_data_received = true;
		
		printf_uart("[RX : %d] : ", s_rx_temp_length);
		for (uint16_t i = 0; i < s_rx_temp_length; i++) {
			printf_uart("%02X ", s_rx_temp_buffer[i]);
		}
		printf_uart("\r\n");
		
		//reset_bc_rx_timeout();
		
		return true;
	}
	
	return false; // Buffer size exceeded
}

/**
 * @brief Check and handle backchannel RX timeout
 * @details Called from main timer tick (1ms)
 */
void Aply_uwb_rx_timer_tick(void)
{
	if (s_bc_rx_timeout_enabled && s_bc_rx_timeout_counter > 0) {
		s_bc_rx_timeout_counter--;
		if (s_bc_rx_timeout_counter == 0) {
			// Timeout occurred - stop and disable RX
			//Api_uwb_stop_rx();
			
			s_bc_rx_timeout_enabled = false;
			s_bc_data_received = false;
		}
	}
}

/**
 * @brief Start backchannel RX with 1ms timeout (extends to 100ms when own MAC received)
 * @return true if started successfully, false otherwise
 */
bool Aply_uwb_rx_start_backchannel(void)
{
	// if (!Api_uwb_start_rx()) {
	// 	return false;
	// }
	
	//s_bc_rx_timeout_counter = 100;  // 3ms default timeout
	//s_bc_rx_timeout_enabled = true;
	//s_bc_data_received = false;
	
	return true;
}

/**
 * @brief Get ACK number from last received BD packet
 * @param[out] ack_number Buffer to store ACK number (6 bytes)
 * @return true if ACK number is available, false otherwise
 */
bool Aply_uwb_rx_get_ack_number(uint8_t* ack_number)
{
	if (ack_number == NULL || !s_bc_data_received) {
		return false;
	}
	
	memcpy(ack_number, s_bc_ack_number, 6);
	return true;
}

bool Aply_uwb_rx_is_bc_data_received(void)
{
	return s_bc_data_received;
}

void Aply_uwb_rx_clear_bc_data_received(void)
{
	s_bc_data_received = false;
}

const uint8_t* Aply_uwb_rx_get_buffer(void)
{
	if (!s_bc_data_received) {
		return NULL;
	}
	return s_rx_buffer;
}

uint16_t Aply_uwb_rx_get_buffer_length(void)
{
	if (!s_bc_data_received) {
		return 0;
	}
	return s_rx_data_length;
}

void Aply_uwb_rx_clear_buffer(void)
{
	s_bc_data_received = false;
	s_rx_data_length = 0;
	memset(s_rx_buffer, 0, sizeof(s_rx_buffer));
}