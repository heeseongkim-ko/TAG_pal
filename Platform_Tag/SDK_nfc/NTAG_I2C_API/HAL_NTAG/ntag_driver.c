/*
 ****************************************************************************
 * Copyright(c) 2014 NXP Semiconductors                                     *
 * All rights are reserved.                                                 *
 *                                                                          *
 * Software that is described herein is for illustrative purposes only.     *
 * This software is supplied "AS IS" without any warranties of any kind,    *
 * and NXP Semiconductors disclaims any and all warranties, express or      *
 * implied, including all implied warranties of merchantability,            *
 * fitness for a particular purpose and non-infringement of intellectual    *
 * property rights.  NXP Semiconductors assumes no responsibility           *
 * or liability for the use of the software, conveys no license or          *
 * rights under any patent, copyright, mask work right, or any other        *
 * intellectual property rights in or to any products. NXP Semiconductors   *
 * reserves the right to make changes in the software without notification. *
 * NXP Semiconductors also makes no representation or warranty that such    *
 * application will be suitable for the specified use without further       *
 * testing or modification.                                                 *
 *                                                                          *
 * Permission to use, copy, modify, and distribute this software and its    *
 * documentation is hereby granted, under NXP Semiconductors' relevant      *
 * copyrights in the software, without fee, provided that it is used in     *
 * conjunction with NXP Semiconductor products(UCODE I2C, NTAG I2C).        *
 * This  copyright, permission, and disclaimer notice must appear in all    *
 * copies of this code.                                                     *
 ****************************************************************************
 */
/***********************************************************************/
/* INCLUDES                                                            */
/***********************************************************************/
//#include "HAL_timer_driver.h" // delete by TEIA
#include "ntag_driver_intern.h"

// nRF52840 specific includes
#ifdef  __TEIA__
#include <string.h>  // for memset
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_twi.h"
#include "app_error.h"
#endif

/***********************************************************************/
/* DEFINES                                                             */
/***********************************************************************/
#undef NTAG_DEVICE_LIST_BEGIN
#undef NTAG_DEVICE_ENTRY
#undef NTAG_DEVICE_LIST_END

#define NTAG_DEVICE_LIST_BEGIN                                          struct NTAG_DEVICE ntag_device_list[NTAG_ID_MAX_DEVICES] = \
                                                                        {
#ifdef HAVE_NTAG_INTERRUPT
#define NTAG_DEVICE_ENTRY(label, i2c_address, isr, fd_port, fd_pin)        { NTAG_CLOSED, HAL_I2C_INVALID_HANDLE, i2c_address, isr, fd_port, fd_pin, {0}, {0} }
#else
#define NTAG_DEVICE_ENTRY(label, i2c_address, isr, fd_port, fd_pin)        { NTAG_CLOSED, HAL_I2C_INVALID_HANDLE, i2c_address, {0}, {0} }
#endif

#define NTAG_DEVICE_LIST_END                                            };

/***********************************************************************/
/* GLOBAL VARIABLES                                                    */
/***********************************************************************/
#ifdef  __TEIA__
extern void Api_nfc_timer_delay(uint32_t ticks);
#endif
/* second include of device list for generation of ntag_device_list array */
NTAG_DEVICE_LIST_BEGIN
#include "ntag_device_list.h"
NTAG_DEVICE_LIST_END/***********************************************************************/
/* GLOBAL PUBLIC FUNCTIONS                                             */
/***********************************************************************/

#ifdef __TEIA__
/* GPIO interrupt handler for nRF52840 */
static void ntag_fd_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    ntag_semaphore = TRUE;
}
#endif

BOOL NTAG_WaitForFDPinState(NTAG_HANDLE_T ntag, NTAG_FD_STATE_T state,
		uint32_t timeout_ms) {
	uint32_t time_elapsed_ms = 0;

	switch (state) {
	case NTAG_FD_PIN_STATE_LO:
		#ifdef  __TEIA__
        {
			// Configure for falling edge detection
			nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
			config.pull = NRF_GPIO_PIN_PULLUP;

			ret_code_t err_code = nrf_drv_gpiote_in_init(ntag->fd_pin, &config, ntag_fd_pin_handler);
			if (err_code == NRF_SUCCESS)
			{
			      nrf_drv_gpiote_in_event_enable(ntag->fd_pin, true);
			}
        }
		#else
		PORT_SetPinInterruptConfig(ntag->fd_port, ntag->fd_pin, kPORT_InterruptFallingEdge);
		#endif
		break;

	case NTAG_FD_PIN_STATE_HI:
		#ifdef  __TEIA__
		// Configure for rising edge detection
		{
			nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
			config.pull = NRF_GPIO_PIN_PULLDOWN;

			ret_code_t err_code = nrf_drv_gpiote_in_init(ntag->fd_pin, &config, ntag_fd_pin_handler);
			if (err_code == NRF_SUCCESS)
			{
				nrf_drv_gpiote_in_event_enable(ntag->fd_pin, true);
			}
		}
		#else
		PORT_SetPinInterruptConfig(ntag->fd_port, ntag->fd_pin, kPORT_InterruptRisingEdge);
		#endif
		break;
	default:
		return NTAG_ERROR_INVALID_PARAM;
	}

	do
	{
		/* Start waiting for interrupt */
#ifdef  __TEIA__
		// Interrupt is already enabled via GPIOTE
#else
		NVIC_EnableIRQ(ntag->isr);
#endif

#ifdef  __TEIA__
		Api_nfc_timer_delay(1);
#else
		HAL_Timer_delay_ms(1);
#endif
		time_elapsed_ms++;

		/* Disable interrupt during check */
		#ifdef  __TEIA__
			// Check is done via semaphore, no need to disable
		#else
		NVIC_DisableIRQ(ntag->isr);
		#endif
	}while(ntag_semaphore == false && time_elapsed_ms < timeout_ms);

#ifdef __TEIA__
	// Cleanup - disable the interrupt
	nrf_drv_gpiote_in_event_disable(ntag->fd_pin);
	nrf_drv_gpiote_in_uninit(ntag->fd_pin);
#endif

	if(ntag_semaphore == true)
	{
		ntag_semaphore = false;
		return FALSE;
	}

	return TRUE;
}

NTAG_HANDLE_T NTAG_InitDevice(NTAG_ID_T ntag_id, HAL_I2C_HANDLE_T i2cbus)
{
	if( ntag_id < NTAG_ID_MAX_DEVICES )
	{
		if( ntag_device_list[ntag_id].status == NTAG_CLOSED )
		{
			ntag_device_list[ntag_id].i2cbus = i2cbus;
			ntag_device_list[ntag_id].status = NTAG_OK;
			
			#ifdef __TEIA__
			// Initialize GPIOTE if not already initialized
			if (!nrf_drv_gpiote_is_init())
			{
				ret_code_t err_code = nrf_drv_gpiote_init();
				APP_ERROR_CHECK(err_code);
			}
			#endif
			
			return &ntag_device_list[ntag_id];
		}
	}
	return NTAG_INVALID_HANDLE;
}

void NTAG_CloseDevice(NTAG_HANDLE_T ntag) {
	if (ntag) {
		ntag->i2cbus = HAL_I2C_INVALID_HANDLE;
		ntag->status = NTAG_CLOSED;
	}
}

BOOL NTAG_ReadBytes(NTAG_HANDLE_T ntag, uint16_t address, uint8_t *bytes,
		uint16_t len) {
	uint16_t bytes_read = 0;

	if (ntag->status == NTAG_CLOSED)
		return TRUE;

	ntag->status = NTAG_OK;

	while (bytes_read < len) {
		uint8_t current_block = (address + bytes_read) / NTAG_I2C_BLOCK_SIZE;
		uint8_t begin = (address + bytes_read) % NTAG_I2C_BLOCK_SIZE;
		uint8_t current_len = MIN(len - bytes_read,
				NTAG_I2C_BLOCK_SIZE - begin);

		if (current_len < NTAG_I2C_BLOCK_SIZE) {
			size_t i = 0;

			/* read block into ntag->rx_buffer only */
			if (NTAG_ReadBlock(ntag, current_block, NULL, 0))
				break;

			/* modify rx_buffer */
			for (i = 0; i < current_len; i++)
				bytes[bytes_read + i] = ntag->rx_buffer[RX_START + begin + i];
		} else {
			/* full block read */
			if (NTAG_ReadBlock(ntag, current_block, bytes + bytes_read,
			NTAG_I2C_BLOCK_SIZE))
				break;
		}

		bytes_read += current_len;
	}
	return ntag->status;
}

BOOL NTAG_WriteBytes(NTAG_HANDLE_T ntag, uint16_t address, const uint8_t *bytes,
		uint16_t len) {
	uint16_t bytes_written = 0;

	if (ntag->status == NTAG_CLOSED)
		return TRUE;

	ntag->status = NTAG_OK;

	while (bytes_written < len) {
		uint8_t current_block = (address + bytes_written) / NTAG_I2C_BLOCK_SIZE;
		uint8_t begin = (address + bytes_written) % NTAG_I2C_BLOCK_SIZE;
		uint8_t current_len = MIN(len - bytes_written,
				NTAG_I2C_BLOCK_SIZE - begin);

		if (current_len < NTAG_I2C_BLOCK_SIZE) {
			size_t i = 0;

			/* read block into ntag->rx_buffer only */
			if (NTAG_ReadBlock(ntag, current_block, NULL, 0))
				break;

			/* check if it is the first Block(0x00) and not the I2C Addr */
			/* be careful with writing of first byte in management block */
			/* the byte contains part of the serial number on read but   */
			/* on write the I2C address of the device can be modified    */
			if (0x00 == current_block && NTAG_MEM_ADRR_I2C_ADDRESS < begin)
				ntag->rx_buffer[RX_START + 0] = ntag->address;

			/* modify rx_buffer */
			for (i = 0; i < current_len; i++)
				ntag->rx_buffer[RX_START + begin + i] =
						bytes[bytes_written + i];

			/* writeback modified buffer */
			if (NTAG_WriteBlock(ntag, current_block, ntag->rx_buffer + RX_START,
			NTAG_I2C_BLOCK_SIZE))
				break;
		} else {
			/* full block write */
			if (NTAG_WriteBlock(ntag, current_block, bytes + bytes_written,
			NTAG_I2C_BLOCK_SIZE))
				break;
		}

		bytes_written += current_len;
	}

	return ntag->status;
}

BOOL NTAG_ReadRegister(NTAG_HANDLE_T ntag, uint8_t reg, uint8_t *val) {

	#ifdef  __TEIA__
	ret_code_t err_code;
	
	ntag->tx_buffer[TX_START + 0] = NTAG_MEM_BLOCK_SESSION_REGS;
	ntag->tx_buffer[TX_START + 1] = reg;
	
	// Send block number and register offset
	err_code = nrf_drv_twi_tx((nrf_drv_twi_t*)ntag->i2cbus, ntag->address, 
	                          ntag->tx_buffer, 2, true);  // No stop for repeated start
	if (err_code != NRF_SUCCESS)
	{
		ntag->status = NTAG_ERROR_TX_FAILED;
		return TRUE;
	}
	
	// Read register value
	err_code = nrf_drv_twi_rx((nrf_drv_twi_t*)ntag->i2cbus, ntag->address,
	                          ntag->rx_buffer, 1);
	if (err_code != NRF_SUCCESS)
	{
		ntag->status = NTAG_ERROR_RX_FAILED;
		return TRUE;
	}
	
	*val = ntag->rx_buffer[RX_START + 0];
	#else
	ntag->tx_buffer[TX_START + 0] = NTAG_MEM_BLOCK_SESSION_REGS;
	ntag->tx_buffer[TX_START + 1] = reg;

	i2c_master_transfer_t masterXfer;

	memset(&masterXfer, 0, sizeof(masterXfer));

	masterXfer.slaveAddress = ntag->address;
	masterXfer.subaddress = (uint32_t)NULL;
	masterXfer.subaddressSize = 0;

	/* send block number */
	masterXfer.direction = kI2C_Write;
	masterXfer.data = ntag->tx_buffer;
	masterXfer.dataSize = 2;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	if(kStatus_Success != I2C_MasterTransferBlocking(ntag->i2cbus, &masterXfer))
	{
		ntag->status = NTAG_ERROR_TX_FAILED;
		return TRUE;
	}

	/* receive bytes */
	masterXfer.direction = kI2C_Read;
	masterXfer.data = ntag->rx_buffer;
	masterXfer.dataSize = 1;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	if(kStatus_Success != I2C_MasterTransferBlocking(ntag->i2cbus, &masterXfer))
	{
		ntag->status = NTAG_ERROR_RX_FAILED;
		return TRUE;
	}

	*val = ntag->rx_buffer[RX_START + 0];
	#endif
	return FALSE;
}

BOOL NTAG_WriteRegister(NTAG_HANDLE_T ntag, uint8_t reg, uint8_t mask,
		uint8_t val) {

#ifdef  __TEIA__
	ret_code_t err_code;
	
	ntag->tx_buffer[TX_START + 0] = NTAG_MEM_BLOCK_SESSION_REGS;
	ntag->tx_buffer[TX_START + 1] = reg;
	ntag->tx_buffer[TX_START + 2] = mask;
	ntag->tx_buffer[TX_START + 3] = val;
	
	// Send register write command
	err_code = nrf_drv_twi_tx((nrf_drv_twi_t*)ntag->i2cbus, ntag->address,
	                          ntag->tx_buffer, 4, false);
	if (err_code != NRF_SUCCESS)
	{
		ntag->status = NTAG_ERROR_TX_FAILED;
		return TRUE;
	}
#else
	ntag->tx_buffer[TX_START + 0] = NTAG_MEM_BLOCK_SESSION_REGS;
	ntag->tx_buffer[TX_START + 1] = reg;
	ntag->tx_buffer[TX_START + 2] = mask;
	ntag->tx_buffer[TX_START + 3] = val;

	i2c_master_transfer_t masterXfer;

	memset(&masterXfer, 0, sizeof(masterXfer));

	masterXfer.slaveAddress = ntag->address;
	masterXfer.subaddress = (uint32_t)NULL;
	masterXfer.subaddressSize = 0;

	masterXfer.direction = kI2C_Write;
	masterXfer.data = ntag->tx_buffer;
	masterXfer.dataSize = 4;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	if(kStatus_Success != I2C_MasterTransferBlocking(ntag->i2cbus, &masterXfer))
	{
		ntag->status = NTAG_ERROR_TX_FAILED;
		return TRUE;
	}
#endif
	return FALSE;
}

BOOL NTAG_ReadConfiguration(NTAG_HANDLE_T ntag, uint8_t reg, uint8_t *val) {
#ifdef NTAG_2k
	uint8_t config = NTAG_MEM_BLOCK_CONFIGURATION_2k;
#elif NTAG_1k
	uint8_t config = NTAG_MEM_BLOCK_CONFIGURATION_1k;
#endif

	uint8_t I2C_Buf[NTAG_I2C_BLOCK_SIZE];
	if (NTAG_ReadBlock(ntag, config, I2C_Buf, NTAG_I2C_BLOCK_SIZE))
		return NTAG_ERR_COMMUNICATION;

	*val = I2C_Buf[reg];
	return NTAG_ERR_OK;
}

BOOL NTAG_WriteConfiguration(NTAG_HANDLE_T ntag, uint8_t reg, uint8_t mask,
		uint8_t val) {
#ifdef NTAG_2k
	uint8_t config = NTAG_MEM_BLOCK_CONFIGURATION_2k;
#elif NTAG_1k
	uint8_t config = NTAG_MEM_BLOCK_CONFIGURATION_1k;
#endif

	uint8_t I2C_Buf[NTAG_I2C_BLOCK_SIZE];

	if (NTAG_ReadBlock(ntag, config, I2C_Buf, NTAG_I2C_BLOCK_SIZE))
		return NTAG_ERR_COMMUNICATION;

	// Clear all other bits of the val
	val = val & mask;

	// Clear specific bit in the Buffer
	I2C_Buf[reg] = I2C_Buf[reg] & ~mask;

	// write bits in the Buffer
	I2C_Buf[reg] = I2C_Buf[reg] | val;

	if (NTAG_WriteBlock(ntag, config, I2C_Buf, NTAG_I2C_BLOCK_SIZE))
		return NTAG_ERR_COMMUNICATION;

	return NTAG_ERR_OK;
}

NTAG_STATUS_T NTAG_GetLastError(NTAG_HANDLE_T ntag) {
	return ntag->status;
}

/***********************************************************************/
/* GLOBAL PRIVATE FUNCTIONS                                            */
/***********************************************************************/
BOOL NTAG_ReadBlock(NTAG_HANDLE_T ntag, uint8_t block, uint8_t *bytes,
		uint8_t len) {

#ifdef  __TEIA__
	size_t i = 0;
	ret_code_t err_code;
	
	ntag->tx_buffer[TX_START] = block;
	
	// Send block number
	err_code = nrf_drv_twi_tx((nrf_drv_twi_t*)ntag->i2cbus, ntag->address,
	                          ntag->tx_buffer, 1, true);  // No stop for repeated start
	if (err_code != NRF_SUCCESS)
	{
		ntag->status = NTAG_ERROR_TX_FAILED;
		return TRUE;
	}
	
	// Read block data
	err_code = nrf_drv_twi_rx((nrf_drv_twi_t*)ntag->i2cbus, ntag->address,
	                          ntag->rx_buffer, NTAG_I2C_BLOCK_SIZE);
	if (err_code != NRF_SUCCESS)
	{
		ntag->status = NTAG_ERROR_RX_FAILED;
		return TRUE;
	}
	
	len = MIN(len, NTAG_I2C_BLOCK_SIZE);
	
	// Write to bytes buffer if provided
	if (bytes != NULL)
	{
		for (i = 0; i < len; i++)
			bytes[i] = ntag->rx_buffer[RX_START + i];
	}
#else
	size_t i = 0;

	i2c_master_transfer_t masterXfer;

	memset(&masterXfer, 0, sizeof(masterXfer));

	masterXfer.slaveAddress = ntag->address;
	masterXfer.subaddress = (uint32_t)NULL;
	masterXfer.subaddressSize = 0;

	ntag->tx_buffer[TX_START] = block;

	/* send block number */
	masterXfer.direction = kI2C_Write;
	masterXfer.data = ntag->tx_buffer;
	masterXfer.dataSize = 1;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	if(kStatus_Success != I2C_MasterTransferBlocking(ntag->i2cbus, &masterXfer))
	{
		ntag->status = NTAG_ERROR_TX_FAILED;
		return TRUE;
	}

	/* receive bytes */

	masterXfer.direction = kI2C_Read;
	masterXfer.data = ntag->rx_buffer;
	masterXfer.dataSize = NTAG_I2C_BLOCK_SIZE;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	if(kStatus_Success != I2C_MasterTransferBlocking(ntag->i2cbus, &masterXfer))
	{
		ntag->status = NTAG_ERROR_RX_FAILED;
		return TRUE;
	}

	len = MIN(len, NTAG_I2C_BLOCK_SIZE);

	/* write to bytes buffer */
	for (i = 0; i < len; i++)
		bytes[i] = ntag->rx_buffer[RX_START + i];
#endif
	return FALSE;
}

BOOL NTAG_WriteBlock(NTAG_HANDLE_T ntag, uint8_t block, const uint8_t *bytes,
		uint8_t len) {

#ifdef  __TEIA__
	uint8_t ns_reg = 0;
	uint32_t timeout = NTAG_MAX_WRITE_DELAY_MS / 5 + 1;
	size_t i = 0;
	ret_code_t err_code;
	
	ntag->tx_buffer[TX_START] = block;
	
	len = MIN(len, NTAG_I2C_BLOCK_SIZE);
	
	/* copy len bytes */
	for (i = 0; i < len; i++)
		ntag->tx_buffer[TX_START + i + 1] = bytes[i];
	
	/* zero rest of the buffer */
	for (i = len; i < NTAG_I2C_BLOCK_SIZE; i++)
		ntag->tx_buffer[TX_START + i + 1] = 0;
	
	/* send block number and data */
	err_code = nrf_drv_twi_tx((nrf_drv_twi_t*)ntag->i2cbus, ntag->address,
	                          ntag->tx_buffer, NTAG_I2C_BLOCK_SIZE + 1, false);
	if (err_code != NRF_SUCCESS)
	{
		ntag->status = NTAG_ERROR_TX_FAILED;
		return TRUE;
	}
	
	/* do not wait for completion when writing SRAM */
	if (block >= NTAG_MEM_BLOCK_START_SRAM
			&& block < NTAG_MEM_BLOCK_START_SRAM + NTAG_MEM_SRAM_BLOCKS)
		return ntag->status;
	
	/* wait for completion */
	do {
#ifdef  __TEIA__
		Api_nfc_timer_delay(5);
#else
		HAL_Timer_delay_ms(5);
#endif
		if (NTAG_ReadRegister(ntag, NTAG_MEM_OFFSET_NS_REG, &ns_reg))
			break;
		timeout--;
	} while (timeout && (ns_reg & NTAG_NS_REG_MASK_EEPROM_WR_BUSY));
	
	if (0 == timeout)
		ntag->status = NTAG_ERROR_WRITE_TIMEOUT;
#else
	uint8_t ns_reg = 0;
	uint32_t timeout = NTAG_MAX_WRITE_DELAY_MS / 5 + 1;
	size_t i = 0;

	i2c_master_transfer_t masterXfer;

	memset(&masterXfer, 0, sizeof(masterXfer));

	masterXfer.slaveAddress = ntag->address;
	masterXfer.subaddress = (uint32_t)NULL;
	masterXfer.subaddressSize = 0;

	ntag->tx_buffer[TX_START] = block;

	len = MIN(len, NTAG_I2C_BLOCK_SIZE);

	/* copy len bytes */
	for (i = 0; i < len; i++)
		ntag->tx_buffer[TX_START + i + 1] = bytes[i];

	/* zero rest of the buffer */
	for (i = len; i < NTAG_I2C_BLOCK_SIZE; i++)
		ntag->tx_buffer[TX_START + i + 1] = 0;

	/* send block number */
	masterXfer.direction = kI2C_Write;
	masterXfer.data = ntag->tx_buffer;
	masterXfer.dataSize = NTAG_I2C_BLOCK_SIZE+1;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	if(kStatus_Success != I2C_MasterTransferBlocking(ntag->i2cbus, &masterXfer))
	{
		ntag->status = NTAG_ERROR_TX_FAILED;
		return TRUE;
	}

	/* do not wait for completion when writing SRAM */
	if (block >= NTAG_MEM_BLOCK_START_SRAM
			&& block < NTAG_MEM_BLOCK_START_SRAM + NTAG_MEM_SRAM_BLOCKS)
		return ntag->status;

	/* wait for completion */
	do {
		HAL_Timer_delay_ms(5);
		if (NTAG_ReadRegister(ntag, NTAG_MEM_OFFSET_NS_REG, &ns_reg))
			break;
		timeout--;
	} while (timeout && ns_reg & NTAG_NS_REG_MASK_EEPROM_WR_BUSY);

	if (0 == timeout)
		ntag->status = NTAG_ERROR_WRITE_TIMEOUT;
#endif
	return ntag->status;
}

