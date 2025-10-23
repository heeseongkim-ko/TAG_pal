/*! ----------------------------------------------------------------------------
 *  @file    Api_uwb_param.c
 *  @brief   UWB API Implementation - Public Interface Layer
 *
 *  This module provides the public API interface for UWB device operations.
 *  It acts as a wrapper around the driver layer functions and provides
 *  application-friendly interface for UWB functionality.
 *
 *  Key Features:
 *  - Public API functions for application use
 *  - Configuration management and validation
 *  - Status monitoring and control
 *  - Hardware interface initialization
 *
 * @author Modified for API/Driver separation
 * @version 1.0
 * @date 2025
 *
 */

#include <string.h>

#include "sdk_config.h"
#include "boards.h"
#include "deca_probe_interface.h"
#include "nrf_delay.h"
#include "Port.h"
#include "Api_uwb.h"
#include "Driver/Drv_uwb.h"
#include <deca_device_api.h>
#include "Func_UART_LOG.h"

// ========================================================================================
// MAC Address Management Functions
// ========================================================================================

/**
 * @brief Read LOT_ID and PART_ID from DW3000 device
 * @param[out] lotid - LOT_ID from DW3000
 * @param[out] partid - PART_ID from DW3000
 * @return true if successful, false if failed
 * 
 * @note This function reads the device identification registers from DW3000.
 *       LOT_ID and PART_ID are used to generate unique MAC addresses.
 */
bool Api_uwb_read_IDs(uint32_t* lotid, uint32_t* partid)
{
	return Drv_uwb_config_read_device_ids(lotid, partid);
}

/**
 * @brief Compose the MAC address of the tag using LOT_ID and PART_ID of the DW3000
 * @param[out] MAC_addr - composed MAC address
 * @return true if successful, false if failed
 * 
 * @note This function reads the device IDs from DW3000 and compresses the PART_ID
 *       from 32 bits to 28 bits by removing 4 zero bits, then combines it with
 *       the lowest 2.5 bytes of LOT_ID to create a unique 48-bit MAC address.
 */
bool Api_uwb_compose_MAC_address(uint8_t* MAC_addr)
{
	return Drv_uwb_config_compose_mac_address(MAC_addr);
}

/**
 * @brief Get current UWB channel setting
 * @return Current UWB channel number
 */
uint8_t Api_uwb_get_channel(void)
{
	dwt_config_t l_config;
	Drv_uwb_get_config(&l_config);
	return l_config.chan;
}

/**
 * @brief Get current UWB data rate setting
 * @return Current UWB data rate (0=850k, 1=6.8M)
 */
uint8_t Api_uwb_get_data_rate(void)
{
	dwt_config_t l_config;
	Drv_uwb_get_config(&l_config);
	return (l_config.dataRate == DWT_BR_850K) ? 1 : 2;
}

/**
 * @brief Get current UWB preamble length setting
 * @return Current UWB preamble length (0=64, 1=128, 2=512, 3=1024)
 */
uint8_t Api_uwb_get_preamble_length(void)
{
	dwt_config_t l_config;
	Drv_uwb_get_config(&l_config);
	
	switch (l_config.txPreambLength) {
		case DWT_PLEN_64:   return 6;
		case DWT_PLEN_128:  return 6;
		case DWT_PLEN_256:  return 5;  // RF4 needs 256 bits
		case DWT_PLEN_512:  return 1;
		case DWT_PLEN_1024: return 2;
		default:            return 6; // Default to 128
	}
}

/**
 * @brief Get current UWB PRF setting
 * @return Current UWB PRF (0=16MHz, 1=64MHz)
 */
uint8_t Api_uwb_get_prf(void)
{
	dwt_config_t l_config;
	Drv_uwb_get_config(&l_config);
	
	// PRF is determined by preamble code range
	// Codes 1-8: 16MHz PRF, Codes 9-12: 64MHz PRF
	return (l_config.txCode >= 9) ? 1 : 0;
}

/**
 * @brief Get current UWB preamble code
 * @return Current UWB preamble code
 */
uint8_t Api_uwb_get_preamble_code(void)
{
	dwt_config_t l_config;
	Drv_uwb_get_config(&l_config);
	return l_config.txCode;
}

/**
 * @brief Get current UWB SFD setting
 * @return Current UWB SFD setting (0=standard, 1=non-standard)
 */
uint8_t Api_uwb_get_sfd(void)
{
	dwt_config_t l_config;
	Drv_uwb_get_config(&l_config);
	return l_config.sfdType;
}

/**
 * @brief Get current UWB PAC (Preamble Acquisition Chunk) size
 * @return Current UWB PAC size (0=8, 1=16)
 */
uint8_t Api_uwb_get_pac_size(void)
{
	dwt_config_t l_config;
	Drv_uwb_get_config(&l_config);
	return (l_config.rxPAC == DWT_PAC8) ? 0 : 1;
}

/**
 * @brief Get current UWB SFD timeout
 * @return Current UWB SFD timeout value
 */
uint16_t Api_uwb_get_sfd_timeout(void)
{
	dwt_config_t l_config;
	Drv_uwb_get_config(&l_config);
	return l_config.sfdTO;
}

/**
 * @brief Get current TX power setting
 * @param[out] tx_power 4-byte array to store TX power values
 * @return None
 */
void Api_uwb_get_tx_power(uint8_t* tx_power)
{
	if (tx_power != NULL) {
		dwt_txconfig_t l_txconfig;
		Drv_uwb_get_txconfig(&l_txconfig);
		
		// Extract TX power from txconfig_options
		uint32_t power_value = l_txconfig.power;
		tx_power[0] = (uint8_t)(power_value & 0xFF);
		tx_power[1] = (uint8_t)((power_value >> 8) & 0xFF);
		tx_power[2] = (uint8_t)((power_value >> 16) & 0xFF);
		tx_power[3] = (uint8_t)((power_value >> 24) & 0xFF);
	}
}

// ========================================================================================
// UWB Configuration Set Functions
// ========================================================================================

/**
 * @brief Set UWB data rate
 * @param[in] data_rate Data rate (1=850k, 2=6M8)
 * @return true if successful, false if invalid parameter
 */
bool Api_uwb_set_data_rate(uint8_t data_rate)
{
	if (data_rate != 1 && data_rate != 2) {
		return false;
	}
	
	dwt_config_t l_config;
	Drv_uwb_get_config(&l_config);
	
	l_config.dataRate = (data_rate == 1) ? DWT_BR_850K : DWT_BR_6M8;
	
	return Drv_uwb_set_config(&l_config);
}

/**
 * @brief Set UWB preamble length
 * @param[in] preamble_length Preamble length (5=256, 6=128)
 * @return true if successful, false if invalid parameter
 */
bool Api_uwb_set_preamble_length(uint8_t preamble_length)
{
	if (preamble_length != 5 && preamble_length != 6) {
		return false;
	}
	
	dwt_config_t l_config;
	Drv_uwb_get_config(&l_config);
	
	l_config.txPreambLength = (preamble_length == 5) ? DWT_PLEN_256 : DWT_PLEN_128;
	
	return Drv_uwb_set_config(&l_config);
}

/**
 * @brief Set UWB SFD (Start Frame Delimiter)
 * @param[in] sfd SFD type (0=Standard, 1=Non-standard)
 * @return true if successful, false if invalid parameter
 */
bool Api_uwb_set_sfd(uint8_t sfd)
{
	if (sfd != 0 && sfd != 1) {
		return false;
	}
	
	dwt_config_t l_config;
	Drv_uwb_get_config(&l_config);
	
	l_config.sfdType = (sfd == 0) ? DWT_SFD_IEEE_4A : DWT_SFD_DW_16;
	
	return Drv_uwb_set_config(&l_config);
}

/**
 * @brief Set UWB TX power
 * @param[in] tx_power 4-byte array containing TX power values
 * @return true if successful, false if invalid parameter
 */
bool Api_uwb_set_tx_power(const uint8_t* tx_power)
{
	if (tx_power == NULL) {
		return false;
	}
	
	dwt_txconfig_t l_txconfig;
	Drv_uwb_get_txconfig(&l_txconfig);
	
	// Convert 4-byte array to 32-bit power value
	uint32_t power_value = (tx_power[3] << 24) | (tx_power[2] << 16) | 
	                       (tx_power[1] << 8) | tx_power[0];
	
	l_txconfig.power = power_value;
	
	return Drv_uwb_set_txconfig(&l_txconfig);
}

/**
 * @brief Set UWB PAC (Preamble Acquisition Chunk) size
 * @param[in] pac_size PAC size (0=8, 1=16)
 * @return true if successful, false if invalid parameter
 */
bool Api_uwb_set_pac_size(uint8_t pac_size)
{
	if (pac_size != 0 && pac_size != 1) {
		return false;
	}
	
	dwt_config_t l_config;
	Drv_uwb_get_config(&l_config);
	
	l_config.rxPAC = (pac_size == 0) ? DWT_PAC8 : DWT_PAC16;
	
	return Drv_uwb_set_config(&l_config);
}

/**
 * @brief Set UWB SFD timeout
 * @param[in] sfd_timeout SFD timeout value
 * @return true if successful, false if invalid parameter
 */
bool Api_uwb_set_sfd_timeout(uint16_t sfd_timeout)
{
	if (sfd_timeout == 0) {
		return false;
	}
	
	dwt_config_t l_config;
	Drv_uwb_get_config(&l_config);
	
	l_config.sfdTO = sfd_timeout;
	
	return Drv_uwb_set_config(&l_config);
}


