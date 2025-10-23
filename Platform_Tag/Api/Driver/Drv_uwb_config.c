/**
 * @file    Drv_uwb_config.c
 * @brief   UWB Driver Configuration Management Implementation
 * 
 * This module contains configuration management functions including
 * getter/setter functions for UWB parameters, message management,
 * device information reading, and MAC address composition.
 * 
 * Key Features:
 * - UWB configuration parameter management
 * - TX/RX message buffer management
 * - Device ID reading and validation
 * - MAC address composition from device IDs
 * - Configuration validation and error checking
 * 
 * @author  Tag Platform Development Team
 * @version 1.0
 * @date    2025
 */

#include "Drv_uwb_internal.h"

// ========================================================================================
// Static Variables (Configuration)
// ========================================================================================

/**
 * @brief Default UWB communication configuration
 * 
 * Uses standard DW3000 settings for channel 5 operation
 * with 6.8M data rate and 128 symbol preamble
 */
static dwt_config_t drv_uwb_config_g = {
	5,                /* Channel number (6.5 GHz center frequency) */
	DWT_PLEN_128,     /* Preamble length (128 symbols for better range) */
	DWT_PAC8,         /* Preamble acquisition chunk size (8 symbols) */
	12,               /* TX preamble code (channel 5, 64MHz PRF) */
	12,               /* RX preamble code (same as TX for compatibility) */
	0,                /* Standard Frame Delimiter (IEEE 802.15.4 standard) */
	DWT_BR_6M8,       /* Data rate (6.8 Mbps for good range/speed balance) */
	DWT_PHRMODE_STD,  /* PHY header mode (standard IEEE 802.15.4) */
	DWT_PHRRATE_STD,  /* PHY header rate (standard rate) */
	129,              /* SFD timeout (preamble + SFD detection timeout) */
	DWT_STS_MODE_OFF, /* STS mode off (no secure timestamping) */
	DWT_STS_LEN_64,   /* STS length (not used when STS is off) */
	DWT_PDOA_M0       /* PDOA mode off (no phase difference of arrival) */
};

/**
 * @brief External TX RF configuration
 */
static dwt_txconfig_t drv_uwb_txconfig_g = {
	0x34,       /* PG delay. */
	0xfdfdfdfd, /* TX power. */
	0x0         /*PG count*/
};

// ========================================================================================
// Configuration Management Functions
// ========================================================================================

/**
 * @brief Get UWB configuration
 * 
 * @details Copies the current UWB configuration structure to the provided buffer.
 * This includes channel, data rate, preamble settings, etc.
 * 
 * @param[out] config Pointer to configuration structure
 * @return true if successful, false if invalid parameters
 */
bool Drv_uwb_get_config(dwt_config_t *config)
{
	if (config == NULL) 
	{
		return false;
	}
	
	memcpy(config, &drv_uwb_config_g, sizeof(dwt_config_t));
	
	return true;
}

/**
 * @brief Set UWB configuration
 * 
 * @details Updates the current UWB configuration structure with new values.
 * Configuration will take effect on next device initialization.
 * 
 * @param[in] config Pointer to configuration structure
 * @return true if successful, false if invalid parameters
 */
bool Drv_uwb_set_config(const dwt_config_t *config)
{
	if (config == NULL) 
	{
		return false;
	}
	
	memcpy(&drv_uwb_config_g, config, sizeof(dwt_config_t));
	
	return true;
}

/**
 * @brief Get TX RF configuration
 * 
 * @details Copies the current TX RF configuration structure to the provided buffer.
 * This includes power settings and calibration values.
 * 
 * @param[out] txconfig Pointer to TX configuration structure
 * @return true if successful, false if invalid parameters
 */
bool Drv_uwb_get_txconfig(dwt_txconfig_t *txconfig)
{
	if (txconfig == NULL) 
	{
		return false;
	}
	
	memcpy(txconfig, &drv_uwb_txconfig_g, sizeof(dwt_txconfig_t));
	
	return true;
}

/**
 * @brief Set TX RF configuration
 * 
 * @details Updates the current TX RF configuration structure with new values.
 * Configuration will take effect immediately if device is ready.
 * 
 * @param[in] txconfig Pointer to TX configuration structure
 * @return true if successful, false if invalid parameters
 */
bool Drv_uwb_set_txconfig(const dwt_txconfig_t *txconfig)
{
	if (txconfig == NULL) 
	{
		return false;
	}
	
	memcpy(&drv_uwb_txconfig_g, txconfig, sizeof(dwt_txconfig_t));
	
	dwt_configuretxrf(&drv_uwb_txconfig_g);
	
	return true;
}



// ========================================================================================
// Device Information Functions
// ========================================================================================

/**
 * @brief Read LOT_ID and PART_ID from DW3000 device
 * 
 * @details Reads the unique device identifiers from the DW3000 OTP memory.
 * These IDs are used for device identification and MAC address composition.
 * 
 * @param[out] lotid LOT_ID from DW3000
 * @param[out] partid PART_ID from DW3000
 * @return true if successful, false if failed
 */
bool Drv_uwb_config_read_device_ids(uint32_t* lotid, uint32_t* partid)
{
	// Read LOT_ID and PART_ID from DW3000
	*lotid = dwt_getlotid();
	*partid = dwt_getpartid();
	
	if (*lotid == 0x00000000)
	{
		*lotid = 0xffffffff;
	}

	LOG_API_UWB("Device IDs read: LOT_ID=0x%08lX, PART_ID=0x%08lX\r\n", *lotid, *partid);
	return true;
}

/**
 * @brief Compose MAC address using device IDs
 * 
 * @details Creates a unique 6-byte MAC address by combining and compressing
 * the device LOT_ID and PART_ID values according to the composition algorithm.
 * 
 * @param[out] MAC_addr 6-byte MAC address array
 * @return true if successful, false if failed
 */
bool Drv_uwb_config_compose_mac_address(uint8_t* MAC_addr)
{
	uint32_t lotid, partid;
	
	// Read LOT_ID and PART_ID from DW3000
	if (!Drv_uwb_config_read_device_ids(&lotid, &partid)) 
	{
		LOG_API_UWB("MAC address composition failed: could not read device IDs\r\n");
		return false;
	}
	
	/* 4 bits in partid are always zero, it can be compressed from 32 to 28 bits */
	uint32_t compressed_partid = 0;
	compressed_partid |= partid & PARTID_SECONDS_MASK;
	compressed_partid |= (partid & PARTID_MINUTES_MASK) >> 1;
	compressed_partid |= (partid & PARTID_HOURS_MASK) >> 2;
	compressed_partid |= (partid & PARTID_DAYS_MASK) >> 3;
	compressed_partid |= (partid & PARTID_LOADBOARD_MASK) >> 4;
	
	/*composed MAC address consists of compressed partid and lowest 2.5 bytes of lotid*/
	uint64_t composed_mac = ((uint64_t) lotid << 32) | (uint64_t) compressed_partid;    //compose MAC address
	composed_mac &= MAC_ADDR_44b_MASK;                                         //mask out highest nibble of 48bit MAC address
	composed_mac |= MAC_ADDR_COMPOSITION_MARK;                                 //add MAC address composition version to the highest nibble  
	
	/* Convert 64-bit MAC address to 6-byte array (Little-endian) */
	for (int i = 0; i < MAC_ADDR_BYTE_SIZE; i++) 
	{
		MAC_addr[i] = (uint8_t)((composed_mac >> (i * 8)) & 0xFF);
	}
	
	// Print MAC address in parts to avoid uint64_t formatting issues
	LOG_API_UWB("MAC address composed: 0x%08lX%08lX\r\n", 
		(uint32_t)((composed_mac >> 32) & 0xFFFFFFFF), 
		(uint32_t)(composed_mac & 0xFFFFFFFF));
	
	return true;
}

// ========================================================================================
// System Register Functions
// ========================================================================================

/**
 * @brief Read and display SYS_CFG register contents
 * 
 * @details Reads the system configuration register and displays detailed
 * bit field information for debugging purposes.
 * 
 * @return Current SYS_CFG register value
 */
uint32_t Drv_uwb_config_read_sys_cfg(void)
{
	if (!Drv_uwb_get_device_wakeup()) 
	{
		LOG_API_UWB("Device not ready\r\n");
		return 0;
	}
	
	uint32_t l_sys_cfg = dwt_read_reg(0x10);
	
	LOG_API_UWB("=== SYS_CFG Register (0x00:10) ===\r\n");
	LOG_API_UWB("Raw value: 0x%08lX\r\n", l_sys_cfg);
	
	// Check specific bits according to the image
	LOG_API_UWB("FFEN      (bit0) : %s\r\n", (l_sys_cfg & (1UL << 0)) ? "ON" : "OFF");
	LOG_API_UWB("SPI_CRCEN (bit1) : %s\r\n", (l_sys_cfg & (1UL << 1)) ? "ON" : "OFF");
	LOG_API_UWB("DIS_FCS_TX(bit2) : %s\r\n", (l_sys_cfg & (1UL << 2)) ? "ON" : "OFF");
	LOG_API_UWB("DIS_DRXB  (bit3) : %s\r\n", (l_sys_cfg & (1UL << 3)) ? "ON" : "OFF");
	LOG_API_UWB("PHR_6M8   (bit4) : %s\r\n", (l_sys_cfg & (1UL << 4)) ? "ON" : "OFF");
	LOG_API_UWB("PHR_MODE  (bit5) : %s\r\n", (l_sys_cfg & (1UL << 5)) ? "ON" : "OFF");
	LOG_API_UWB("SPI_CRCEN (bit6) : %s\r\n", (l_sys_cfg & (1UL << 6)) ? "ON" : "OFF");
	LOG_API_UWB("CIA_IPATOV(bit7) : %s\r\n", (l_sys_cfg & (1UL << 7)) ? "ON" : "OFF");
	LOG_API_UWB("CIA_STS   (bit8) : %s\r\n", (l_sys_cfg & (1UL << 8)) ? "ON" : "OFF");
	
	return l_sys_cfg;
}

/**
 * @brief Check if SPI CRC mode is enabled
 * 
 * @details Checks the SPI_CRCEN bit in the SYS_CFG register to determine
 * if SPI CRC error checking is enabled.
 * 
 * @return true if SPI CRC mode is enabled, false if disabled
 */
bool Drv_uwb_config_is_spi_crc_enabled(void)
{
	if (!Drv_uwb_get_device_wakeup()) 
	{
		return false;
	}
	
	uint32_t l_sys_cfg = dwt_read_reg(0x10);
	bool l_spi_crc_enabled = (l_sys_cfg & (1UL << 1)) != 0;  // bit 1 = SPI_CRCEN
	
	LOG_API_UWB("SPI_CRCEN: %s (bit1 = %d)\r\n", 
		l_spi_crc_enabled ? "ENABLED" : "DISABLED",
		l_spi_crc_enabled ? 1 : 0);
	
	return l_spi_crc_enabled;
}

