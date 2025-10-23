/**
 * @file Api_uwb_param.h
 * @brief UWB API Parameter Functions Header
 * @details This module provides getter and setter functions for UWB parameters
 * @author Tag Platform Development Team
 * @date 2025
 */

#ifndef API_UWB_PARAM_H
#define API_UWB_PARAM_H

#include <stdint.h>
#include <stdbool.h>

// ========================================================================================
// UWB Configuration Getter Functions
// ========================================================================================

/**
 * @brief Read LOT_ID and PART_ID from DW3000 device
 * @param[out] lotid - LOT_ID from DW3000
 * @param[out] partid - PART_ID from DW3000
 * @return true if successful, false if failed
 */
bool Api_uwb_read_IDs(uint32_t* lotid, uint32_t* partid);

/**
 * @brief Compose MAC address from device IDs
 * @param[out] MAC_addr 6-byte array to store MAC address
 * @return true if successful, false if failed
 */
bool Api_uwb_compose_MAC_address(uint8_t* MAC_addr);

/**
 * @brief Get current UWB channel setting
 * @return Current UWB channel
 */
uint8_t Api_uwb_get_channel(void);

/**
 * @brief Get current UWB data rate setting
 * @return Current UWB data rate (1=850k, 2=6M8)
 */
uint8_t Api_uwb_get_data_rate(void);

/**
 * @brief Get current UWB preamble length setting
 * @return Current UWB preamble length (5=256, 6=128)
 */
uint8_t Api_uwb_get_preamble_length(void);

/**
 * @brief Get current UWB SFD setting
 * @return Current UWB SFD (0=Standard, 1=Non-standard)
 */
uint8_t Api_uwb_get_sfd(void);

/**
 * @brief Get current UWB preamble code setting
 * @return Current UWB preamble code
 */
uint8_t Api_uwb_get_preamble_code(void);

/**
 * @brief Get current UWB PRF setting
 * @return Current UWB PRF (0=16MHz, 1=64MHz)
 */
uint8_t Api_uwb_get_prf(void);

/**
 * @brief Get current UWB PAC (Preamble Acquisition Chunk) size
 * @return Current UWB PAC size (0=8, 1=16)
 */
uint8_t Api_uwb_get_pac_size(void);

/**
 * @brief Get current UWB SFD timeout
 * @return Current UWB SFD timeout value
 */
uint16_t Api_uwb_get_sfd_timeout(void);

/**
 * @brief Get current TX power setting
 * @param[out] tx_power 4-byte array to store TX power values
 * @return None
 */
void Api_uwb_get_tx_power(uint8_t* tx_power);

// ========================================================================================
// UWB Configuration Set Functions
// ========================================================================================

/**
 * @brief Set UWB data rate
 * @param[in] data_rate Data rate (1=850k, 2=6M8)
 * @return true if successful, false if invalid parameter
 */
bool Api_uwb_set_data_rate(uint8_t data_rate);

/**
 * @brief Set UWB preamble length
 * @param[in] preamble_length Preamble length (5=256, 6=128)
 * @return true if successful, false if invalid parameter
 */
bool Api_uwb_set_preamble_length(uint8_t preamble_length);

/**
 * @brief Set UWB SFD (Start Frame Delimiter)
 * @param[in] sfd SFD type (0=Standard, 1=Non-standard)
 * @return true if successful, false if invalid parameter
 */
bool Api_uwb_set_sfd(uint8_t sfd);

/**
 * @brief Set UWB TX power
 * @param[in] tx_power 4-byte array containing TX power values
 * @return true if successful, false if invalid parameter
 */
bool Api_uwb_set_tx_power(const uint8_t* tx_power);

/**
 * @brief Set UWB PAC (Preamble Acquisition Chunk) size
 * @param[in] pac_size PAC size (0=8, 1=16)
 * @return true if successful, false if invalid parameter
 */
bool Api_uwb_set_pac_size(uint8_t pac_size);

/**
 * @brief Set UWB SFD timeout
 * @param[in] sfd_timeout SFD timeout value
 * @return true if successful, false if invalid parameter
 */
bool Api_uwb_set_sfd_timeout(uint16_t sfd_timeout);

#endif /* API_UWB_PARAM_H */

