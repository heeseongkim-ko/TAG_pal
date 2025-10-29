/**
 * @file Aply_uwb_rx.h
 * @brief UWB RX management for Aply module
 * @author Platform Tag Team
 * @date 2025-01-20
 */

#ifndef APLY_UWB_RX_H_INCLUDED
#define APLY_UWB_RX_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Process received RX data
 * @return true if valid BC data was processed, false otherwise
 */
bool Aply_uwb_rx_process_data(void);

/**
 * @brief Check and handle backchannel RX timeout
 * @details Called from main timer tick (1ms)
 */
void Aply_uwb_rx_timer_tick(void);

/**
 * @brief Get ACK number from last received BD packet
 * @param[out] ack_number Buffer to store ACK number (6 bytes)
 * @return true if ACK number is available, false otherwise
 */
bool Aply_uwb_rx_get_ack_number(uint8_t* ack_number);

/**
 * @brief Get BC data received flag
 * @details Returns whether BC data has been received
 * @return true if BC data received, false otherwise
 */
bool Aply_uwb_rx_is_bc_data_received(void);

/**
 * @brief Clear BC data received flag
 * @details Resets the BC data received flag
 * @return None
 */
void Aply_uwb_rx_clear_bc_data_received(void);

/**
 * @brief Get RX buffer data
 * @details Returns pointer to RX buffer containing BD packet data
 * @return Pointer to RX buffer, NULL if no data
 */
const uint8_t* Aply_uwb_rx_get_buffer(void);

/**
 * @brief Get RX buffer data length
 * @details Returns length of data in RX buffer
 * @return Data length in bytes
 */
uint16_t Aply_uwb_rx_get_buffer_length(void);

/**
 * @brief Clear RX buffer
 * @details Clears the RX buffer and resets length
 * @return None
 */
void Aply_uwb_rx_clear_buffer(void);

#endif /* APLY_UWB_RX_H_INCLUDED */