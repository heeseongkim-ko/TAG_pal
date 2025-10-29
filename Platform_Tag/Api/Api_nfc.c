/**
 * @file Api_nfc.c
 * @brief NFC API implementation for Unified SDK
 * 
 * Implements high-level NFC interface using NTAG I2C Plus driver
 * for nRF52840 platform integration
 */

#include "sdk_config.h"
#include "ntag_driver.h"
#include "ntag_bridge.h"
#include "nrfx_gpiote.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_twi.h"
#include "nrf_gpio.h"
#include "ntag_driver_intern.h"
#include "Func_UART_LOG.h"
#include "app_error.h"

#include "Api_port.h"
#include "Api_nfc.h"
#include "Api_sleep.h"

#include <string.h>
#include <stdio.h>

static uint32_t nfc_i2c_scl_pin_ui32 = P_NFC_SCL;
static uint32_t nfc_i2c_sda_pin_ui32 = P_NFC_SDA;
static uint32_t nfc_timer_ui32 = 0;
static const nrf_drv_twi_t nfc_twi_p = NRF_DRV_TWI_INSTANCE(0);

static NTAG_HANDLE_T nfc_ntag_handle_p = NULL;

// Field detection variables
static bool nfc_field_detection_initialized_b = false;
static nfc_reader_state_t nfc_last_reader_state_g = NFC_READER_NOT_PRESENT;

#if NFC_USE_INTERRUPT_DETECTION
static void field_detection_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
#endif

static nfc_result_t memory_read_data(uint16_t address, uint8_t *data, uint16_t length);
static nfc_result_t memory_write_data(uint16_t address, const uint8_t *data, uint16_t length);

/**
 * @brief Timer tick function for NFC timing operations
 */
void Api_nfc_timer_tick(void)
{
	++nfc_timer_ui32;
}

/**
 * @brief Delay function using internal timer
 * 
 * @param ticks	Number of timer ticks to delay
 */
void Api_nfc_timer_delay(uint32_t ticks)
{
	nfc_timer_ui32 = 0;
	while(1)
	{
		if (ticks < nfc_timer_ui32)
		{
			break;
		}
	}
}

bool Api_nfc_power_on(void)
{	
	nrf_gpio_cfg_output(P_NFC_PW);

	// Turn off all LEDs initially (active low)
	nrf_gpio_pin_set(P_NFC_PW);
}

/**
 * @brief Test communication with NFC device at specified I2C address
 * 
 * Attempts to perform I2C communication to verify device presence.
 * 
 * @param addr7	7-bit I2C address to probe
 * @return true if ACK received, false otherwise
 */
static bool address_probe(uint8_t addr7)
{
	ret_code_t l_err;
	uint8_t l_tx = 0x00; /* Block number 0 */
	uint8_t l_rx = 0x00;

	/* Try Repeated-Start sequence (no STOP) */
	l_err = nrf_drv_twi_tx(&nfc_twi_p, addr7, &l_tx, 1, true);
	if (l_err == NRF_SUCCESS)
	{
		l_err = nrf_drv_twi_rx(&nfc_twi_p, addr7, &l_rx, 1);
		if (l_err == NRF_SUCCESS)
		{
			return true;
		}
	}

	/* Fallback: send STOP then separate READ */
	l_err = nrf_drv_twi_tx(&nfc_twi_p, addr7, &l_tx, 1, false);
	if (l_err == NRF_SUCCESS)
	{
		l_err = nrf_drv_twi_rx(&nfc_twi_p, addr7, &l_rx, 1);
		if (l_err == NRF_SUCCESS)
		{
			return true;
		}
	}

	/* Last resort: plain read without setting pointer */
	l_err = nrf_drv_twi_rx(&nfc_twi_p, addr7, &l_rx, 1);
	return (l_err == NRF_SUCCESS);
}

/**
 * @brief Automatically detect NTAG I2C device address (internal implementation)
 */
static nfc_result_t address_auto_detect(uint8_t *address_found)
{
	if (address_found == NULL)
	{
		return NFC_ERROR_INVALID_PARAM;
	}
	
	if (address_probe(NTAG_I2C_PLUS_ADDRESS))
	{
		*address_found = NTAG_I2C_PLUS_ADDRESS;
		// printf("NTAG detected at 0x%02X\n", NTAG_I2C_PLUS_ADDRESS);
		LOG_API_NFC("NTAG detected at 0x%02X\r\n", NTAG_I2C_PLUS_ADDRESS);
		return NFC_SUCCESS;
	}
		
	// printf("NTAG Scan I2C Address\n");
	LOG_API_NFC("NTAG Scan I2C Address\r\n");
	/* Extended scan if not found in common range */
	for (uint8_t l_a = 0x00; l_a <= 0x99; l_a++)
	{
		if (address_probe(l_a))
		{
			*address_found = l_a;
			// printf("NTAG detected at 0x%02X (extended scan)\n", l_a);
			LOG_API_NFC("NTAG detected at 0x%02X (extended scan)\r\n", l_a);
			return NFC_SUCCESS;
		}
	}
	return NFC_ERROR_DEVICE_NOT_FOUND;
}

/**
 * @brief Restore NTAG I2C address to default (0x55)
 * 
 * @return NFC_SUCCESS on success, error code otherwise
 */
static nfc_result_t address_restore_default(void)
{
	uint8_t l_block_data[16];
	nfc_result_t l_result;
		
	// printf("Restoring I2C address to default (0x55)...\n");
	LOG_API_NFC("Restoring I2C address to default (0x55)...\r\n");
	
	/* Read block 0 with current address */
	l_result = memory_read_data(0x00, l_block_data, 16);
	if (l_result != NFC_SUCCESS)
	{
		// printf("Failed to read block 0 with current address\n");
		LOG_API_NFC("Failed to read block 0 with current address\r\n");
		return l_result;
	}
	
	/* Change I2C address byte to default value */
	l_block_data[0] = 0xAA;  /* 0x55 << 1 = 0xAA (default I2C address) */
	
	/* Write modified data to current address */
	l_result = memory_write_data(0x00, l_block_data, 16);
	if (l_result != NFC_SUCCESS)
	{
		// printf("Failed to write block 0\n");
		LOG_API_NFC("Failed to write block 0\r\n");
		return l_result;
	}
	
	/* Wait for EEPROM programming time */
	Api_nfc_timer_delay(10);  /* 10 ticks = ~10ms */
	
	/* Test with new address */
	uint8_t l_test_addr = 0;
	if (address_auto_detect(&l_test_addr) == NFC_SUCCESS)
	{
		if (l_test_addr == 0x55)
		{
			/* Update handle with new address */
			((struct NTAG_DEVICE*)nfc_ntag_handle_p)->address = 0x55;
			// printf("I2C address successfully restored to 0x55\n");
			LOG_API_NFC("I2C address successfully restored to 0x55\r\n");
			return NFC_SUCCESS;
		}
		else
		{
			// printf("Address restored but detected at 0x%02X\n", l_test_addr);
			LOG_API_NFC("Address restored but detected at 0x%02X\r\n", l_test_addr);
			((struct NTAG_DEVICE*)nfc_ntag_handle_p)->address = l_test_addr;
			return NFC_SUCCESS;
		}
	}
	
	// printf("Failed to detect device after address restore\n");
	LOG_API_NFC("Failed to detect device after address restore\r\n");
	return NFC_ERROR_COMMUNICATION;
}

/**
 * @brief Test and restore I2C address if needed
 */
static void address_restore(void)
{
	uint8_t l_detected_addr = 0;
	
	// printf("\n=== I2C Address Restore Test ===\n");
	LOG_API_NFC("\r\n=== I2C Address Restore Test ===\r\n");
	
	if (address_auto_detect(&l_detected_addr) == NFC_SUCCESS)
	{
		// printf("Current I2C address: 0x%02X\n", l_detected_addr);
		LOG_API_NFC("Current I2C address: 0x%02X\r\n", l_detected_addr);
		
		if (l_detected_addr != 0x55)
		{
			// printf("Attempting to restore to default address...\n");
			LOG_API_NFC("Attempting to restore to default address...\r\n");
			
			if (address_restore_default() == NFC_SUCCESS)
			{
				// printf("Address restore completed successfully!\n");
				LOG_API_NFC("Address restore completed successfully!\r\n");
			}
			else
			{
				// printf("Address restore failed!\n");
				LOG_API_NFC("Address restore failed!\r\n");
			}
		}
		else
		{
			// printf("Address is already at default value\n");
			LOG_API_NFC("Address is already at default value\r\n");
		}
	}
	else
	{
		// printf("Failed to detect current I2C address\n");
		LOG_API_NFC("Failed to detect current I2C address\r\n");
	}
	
	// printf("=== Test Completed ===\n\n");
	LOG_API_NFC("=== Test Completed ===\r\n\r\n");
}

/**
 * @brief Initialize TWI (I2C) interface for NFC communication
 *
 * This function configures and initializes the TWI peripheral to communicate
 * with the NFC chip (NT3H2211W0FHK). The initialization sequence includes:
 * 1. Configure TWI parameters (SCL/SDA pins, frequency, interrupt priority)
 * 2. Initialize TWI peripheral (pins are automatically configured as open-drain with pull-ups)
 * 3. Enable TWI peripheral for operation
 * 4. Log initialization result
 *
 * TWI Configuration:
 * - Bus frequency: 100kHz (compatible with NT3H2211 specifications)
 * - Interrupt priority: Low (APP_IRQ_PRIORITY_LOW)
 * - Bus clearing: Disabled on init
 * - Bus hold: Disabled on uninit
 *
 * @note The SCL and SDA pins are configured from nfc_i2c_scl_pin_ui32 and 
 *       nfc_i2c_sda_pin_ui32 global variables.
 * @note Pin configuration (open-drain with pull-ups) is handled automatically
 *       by the nrf_drv_twi_init() function.
 *
 * @return nfc_result_t  NFC_SUCCESS on successful initialization
 *                       NFC_ERROR_INIT_FAILED if TWI initialization fails
 */
nfc_result_t Api_nfc_twi_init(void)
{
	// TWI configuration structure
	nrf_drv_twi_config_t l_twi_cfg = {
		.scl				= (uint32_t)nfc_i2c_scl_pin_ui32,
		.sda				= (uint32_t)nfc_i2c_sda_pin_ui32,
		.frequency			= NRF_TWI_FREQ_100K,		// 100kHz for NFC chip compatibility
		.interrupt_priority	= APP_IRQ_PRIORITY_LOW,
		.clear_bus_init		= false,				// Skip bus clearing on init
		.hold_bus_uninit	= false					// Don't hold bus on uninit
	};
	
	// Initialize TWI peripheral (automatically configures pins as open-drain with pull-ups)
	ret_code_t l_err = nrf_drv_twi_init(&nfc_twi_p, &l_twi_cfg, NULL, NULL);
	if (l_err != NRF_SUCCESS)
	{
		// Log initialization error
		LOG_API_NFC("TWI init failed: %d\r\n", l_err);
		return NFC_ERROR_INIT_FAILED;
	}
	
	// Enable TWI peripheral
	nrf_drv_twi_enable(&nfc_twi_p);
	
	// Log successful initialization
	LOG_API_NFC("NFC TWI initialized successfully (SCL:%d, SDA:%d)\r\n", 
				nfc_i2c_scl_pin_ui32, nfc_i2c_sda_pin_ui32);
	
	return NFC_SUCCESS;
}

nfc_result_t Api_nfc_twi_unInit(void)
{
	nrf_drv_twi_disable(&nfc_twi_p);
	nrf_drv_twi_uninit(&nfc_twi_p);
	
	LOG_API_NFC("NFC uninitialized for sleep mode\r\n");
	
	return NFC_SUCCESS;
}

void callback_nfc(nfc_reader_state_t state)
{
	if  (state == NFC_READER_PRESENT)
	{
	}
	else
	{
	}
}

nfc_result_t Api_nfc_init(void)
{
	NTAG_STATUS_T l_ntag_status;
	uint8_t l_ns_reg;

	Api_nfc_power_on();
	Api_nfc_twi_init();
		
	nfc_ntag_handle_p = NTAG_InitDevice(NTAG0, (HAL_I2C_HANDLE_T)&nfc_twi_p);
	
	//address_restore();
	
	l_ntag_status = NTAG_ReadRegister(nfc_ntag_handle_p, NTAG_MEM_OFFSET_NS_REG, &l_ns_reg);

	if  (l_ntag_status == NTAG_OK)
	{
		LOG_API_NFC("%s:OK\r\n", __func__);
	}
	else
	{
		LOG_API_NFC("%s:error:%d\r\n", __func__, l_ntag_status);
		return NFC_ERROR_INIT_FAILED;
	}
	
	Api_nfc_field_detection_init();
		
	return NFC_SUCCESS;
}

#if NFC_USE_INTERRUPT_DETECTION
static void field_detection_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	nfc_reader_state_t l_current_state;
	
	// NT3H2211W0FHK FD pin is open-drain, pulled LOW when NFC field present
	// Read actual pin state to determine reader presence
	uint32_t l_pin_state = nrf_gpio_pin_read(P_NFC_FD);
	l_current_state = (l_pin_state == 0) ? NFC_READER_PRESENT : NFC_READER_NOT_PRESENT;
	
	if (l_current_state != nfc_last_reader_state_g)
	{
		nfc_last_reader_state_g = l_current_state;

		if  (l_current_state == NFC_READER_PRESENT)
		{
			Api_sleep_setWakeupReason(API_SLEEP_WAKEUP_REASON_MASK_NFC);
		}

		// callback_nfc(nfc_last_reader_state_g);
	}
}
#endif

nfc_result_t Api_nfc_field_detection_init(void)
{
	if (nfc_field_detection_initialized_b)
	{
		return NFC_SUCCESS;
	}
		
#if NFC_USE_INTERRUPT_DETECTION
	ret_code_t l_err;
	
	if (!nrf_drv_gpiote_is_init())
	{
		l_err = nrf_drv_gpiote_init();
		if (l_err != NRF_SUCCESS)
		{
			LOG_API_NFC("GPIOTE init failed: %d\r\n", l_err);
			return NFC_ERROR_INIT_FAILED;
		}
	}
	
	// Configure for both HIGH-to-LOW and LOW-to-HIGH transitions
	// NT3H2211W0FHK FD pin: LOW=field present, HIGH=field absent
	nrf_drv_gpiote_in_config_t l_in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
	l_in_config.pull = NRF_GPIO_PIN_PULLUP;
	
	l_err = nrf_drv_gpiote_in_init(P_NFC_FD, &l_in_config, field_detection_handler);
	if (l_err != NRF_SUCCESS)
	{
		LOG_API_NFC("Field detection pin init failed: %d\r\n", l_err);
		return NFC_ERROR_INIT_FAILED;
	}
	
	nrf_drv_gpiote_in_event_enable(P_NFC_FD, true);
	
	LOG_API_NFC("NFC Field Detection: Interrupt mode enabled (TOGGLE)\r\n");
#else
	nrf_gpio_cfg_input(P_NFC_FD, NRF_GPIO_PIN_PULLUP);
	LOG_API_NFC("NFC Field Detection: Polling mode enabled\r\n");
#endif
	
	nfc_field_detection_initialized_b = true;
	nfc_last_reader_state_g = Api_nfc_poll_field_detection();
	
	return NFC_SUCCESS;
}

nfc_reader_state_t Api_nfc_poll_field_detection(void)
{
	if (!nfc_field_detection_initialized_b)
	{
		return NFC_READER_NOT_PRESENT;
	}	
	
#if NFC_USE_INTERRUPT_DETECTION
	return(nfc_last_reader_state_g);
#else
	uint32_t l_pin_state = nrf_gpio_pin_read(P_NFC_FD);
	nfc_reader_state_t l_current_state = (l_pin_state == 0) ? NFC_READER_PRESENT : NFC_READER_NOT_PRESENT;
	
	if (l_current_state != nfc_last_reader_state_g)
	{		
		nrf_gpio_cfg_output(P_NFC_PW);
		nrf_gpio_pin_set(P_NFC_PW);
		
		nfc_last_reader_state_g = l_current_state;

		callback_nfc(nfc_last_reader_state_g);
		
		LOG_API_NFC("NFC Field %s\r\n", 
			(l_current_state == NFC_READER_PRESENT) ? "Detected" : "Removed");
	}
	return l_current_state;
#endif
}

// EEPROM change monitoring variables
static bool nfc_monitoring_initialized_b = false;
static uint8_t nfc_last_ns_reg_ui8 = 0;

nfc_result_t Api_nfc_get_session_status(uint8_t *ns_reg)
{
	if (ns_reg == NULL)
	{
		return NFC_ERROR_INVALID_PARAM;
	}
	
	if (nfc_ntag_handle_p == NULL)
	{
		return NFC_ERROR_NOT_INITIALIZED;
	}
	
	NTAG_STATUS_T l_status = NTAG_ReadRegister(nfc_ntag_handle_p, NTAG_MEM_OFFSET_NS_REG, ns_reg);
	if (l_status != NTAG_OK)
	{
		LOG_API_NFC("Failed to read NS_REG: %d\r\n", l_status);
		return NFC_ERROR_COMMUNICATION;
	}
	
	return NFC_SUCCESS;
}

bool Api_nfc_was_ndef_read(void)
{
	uint8_t l_ns_reg;
	nfc_result_t l_result = Api_nfc_get_session_status(&l_ns_reg);
	
	if (l_result != NFC_SUCCESS)
	{
		return false;
	}
	
	// Check NDEF_DATA_READ bit (bit 7)
	bool l_ndef_read = (l_ns_reg & 0x80) != 0;
	
	if (l_ndef_read)
	{
		LOG_API_NFC("NDEF data was read by NFC reader\r\n");
		// Note: NDEF_DATA_READ bit is automatically cleared when read
		return true;
	}
	
	return false;
}

nfc_result_t Api_nfc_clear_error_flags(void)
{
	if (nfc_ntag_handle_p == NULL)
	{
		return NFC_ERROR_NOT_INITIALIZED;
	}
	
	uint8_t l_ns_reg;
	nfc_result_t l_result = Api_nfc_get_session_status(&l_ns_reg);
	if (l_result != NFC_SUCCESS)
	{
		return l_result;
	}
	
	// Clear EEPROM_WR_ERR bit (bit 2) if set
	if (l_ns_reg & 0x04)
	{
		LOG_API_NFC("Clearing EEPROM write error flag\r\n");
		
		// Write 0 to bit 2 to clear the error flag
		// Using register write operation: MASK=0x04, DATA=0x00
		NTAG_STATUS_T l_status = NTAG_WriteRegister(nfc_ntag_handle_p, 
			NTAG_MEM_OFFSET_NS_REG, 0x04, 0x00);
		if (l_status != NTAG_OK)
		{
			LOG_API_NFC("Failed to clear error flag: %d\r\n", l_status);
			return NFC_ERROR_COMMUNICATION;
		}
	}
	
	return NFC_SUCCESS;
}

nfc_eeprom_status_t Api_nfc_monitor_eeprom_status(void)
{
	uint8_t l_current_ns_reg;
	bool l_changes_detected = false;
	nfc_eeprom_status_t l_status = NFC_EEPROM_NONE;
	
	if (nfc_ntag_handle_p == NULL)
	{
		return l_status;
	}
	
	// Initialize monitoring on first call
	if (!nfc_monitoring_initialized_b)
	{
		Api_nfc_get_session_status(&nfc_last_ns_reg_ui8);
		nfc_monitoring_initialized_b = true;
		LOG_API_NFC("NS_REG monitoring initialized (NS_REG: 0x%02X)\r\n", nfc_last_ns_reg_ui8);
		return l_status;
	}
	
	// Read current session register
	nfc_result_t l_result = Api_nfc_get_session_status(&l_current_ns_reg);
	if (l_result != NFC_SUCCESS)
	{
		return l_status;
	}
	
	// Check for significant state changes
	uint8_t l_changed_bits = l_current_ns_reg ^ nfc_last_ns_reg_ui8;
	
	if (l_changed_bits != 0)
	{
		LOG_API_NFC("NS_REG changed: 0x%02X -> 0x%02X (changed: 0x%02X)\r\n", 
			nfc_last_ns_reg_ui8, l_current_ns_reg, l_changed_bits);
		
		// Check for NDEF read completion
		if ((l_current_ns_reg & 0x80) && !(nfc_last_ns_reg_ui8 & 0x80))
		{
			LOG_API_NFC("NDEF message read detected\r\n");
			l_changes_detected = true;
			l_status = NFC_EEPROM_READ_DETECTED;
		}
		
		// Check for NFC interface activity (RF_LOCKED changes)
		if ((l_changed_bits & 0x20) != 0)
		{
			if (l_current_ns_reg & 0x20)
			{
				LOG_API_NFC("NFC interface locked memory\r\n");
				l_status = NFC_EEPROM_IF_LOCKED;
			}
			else
			{
				LOG_API_NFC("NFC interface released memory\r\n");
				// Memory released - potential EEPROM changes
				l_changes_detected = true;
				l_status = NFC_EEPROM_IF_RELEASED;
			}
		}
		
		// Check for EEPROM write activity completion
		if ((l_changed_bits & 0x02) != 0)
		{
			if (!(l_current_ns_reg & 0x02) && (nfc_last_ns_reg_ui8 & 0x02))
			{
				LOG_API_NFC("EEPROM write completed\r\n");
				l_changes_detected = true;
				l_status = NFC_EEPROM_WRITE_COMPLETED;
			}
		}
		
		// Check for EEPROM write error
		if ((l_current_ns_reg & 0x04) && !(nfc_last_ns_reg_ui8 & 0x04))
		{
			LOG_API_NFC("EEPROM write error detected\r\n");
			l_status = NFC_EEPROM_WRITE_ERROR;
			// Clear error flag
			Api_nfc_clear_error_flags();
		}
		
		// Check for SRAM activity
		if (l_changed_bits & 0x18)  // SRAM_I2C_READY or SRAM_RF_READY
		{
			if (l_current_ns_reg & 0x08) LOG_API_NFC("SRAM ready for RF read\r\n");
			if (l_current_ns_reg & 0x10) LOG_API_NFC("SRAM ready for I2C read\r\n");
			l_changes_detected = true;
			l_status = NFC_EEPROM_NONE;
		}
		
		nfc_last_ns_reg_ui8 = l_current_ns_reg;
	}
	
	return l_status;
}

bool Api_nfc_is_reader_present(void)
{
	return (Api_nfc_poll_field_detection() == NFC_READER_PRESENT);
}

nfc_reader_state_t Api_nfc_get_reader_state(void)
{
	return Api_nfc_poll_field_detection();
}

/**
 * @brief Read data from NFC tag memory (internal implementation)
 */
 
static nfc_result_t memory_read_data(uint16_t address, uint8_t *data, uint16_t length)
{
	NTAG_STATUS_T l_status;
	
	if (nfc_ntag_handle_p == NULL)
	{
		return NFC_ERROR_NOT_INITIALIZED;
	}
	
	if (data == NULL || length == 0)
	{
		return NFC_ERROR_INVALID_PARAM;
	}
	
	if (address + length > NFC_EEPROM_SIZE)
	{
		return NFC_ERROR_INVALID_PARAM;
	}
	
	// printf("Reading %d bytes from address 0x%04X\n", length, address);
	LOG_API_NFC("Reading %d bytes from address 0x%04X\r\n", length, address);
	
	l_status = NTAG_ReadBytes(nfc_ntag_handle_p, address, data, length);
	
	if (l_status != NTAG_OK)
	{
		// printf("Read failed with status: %d\n", l_status);
		LOG_API_NFC("Read failed with status: %d\r\n", l_status);
		return NFC_ERROR_INVALID_PARAM;
	}
	
	return NFC_SUCCESS;
}

nfc_result_t Api_nfc_read_data(uint16_t address, uint8_t *data, uint16_t length)
{
		
	return memory_read_data(address, data, length);
}

/**
 * @brief Write data to NFC tag memory (internal implementation)
 */
static nfc_result_t memory_write_data(uint16_t address, const uint8_t *data, uint16_t length)
{
	NTAG_STATUS_T l_status;
	
	if (nfc_ntag_handle_p == NULL)
	{
		return NFC_ERROR_NOT_INITIALIZED;
	}
	
	if (data == NULL || length == 0)
	{
		return NFC_ERROR_INVALID_PARAM;
	}
	
	if (address + length > NFC_EEPROM_SIZE)
	{
		return NFC_ERROR_INVALID_PARAM;
	}
	
	// printf("Writing %d bytes to address 0x%04X\n", length, address);
	LOG_API_NFC("Writing %d bytes to address 0x%04X\r\n", length, address);
	
	l_status = NTAG_WriteBytes(nfc_ntag_handle_p, address, data, length);
	
	if (l_status != NTAG_OK)
	{
		// printf("Write failed with status: %d\n", l_status);
		LOG_API_NFC("Write failed with status: %d\r\n", l_status);
		return NFC_ERROR_INVALID_PARAM;
	}
	
	return NFC_SUCCESS;
}

nfc_result_t Api_nfc_write_data(uint16_t address, const uint8_t *data, uint16_t length)
{
	return memory_write_data(address, data, length);
}

/**
 * @brief Read data from SRAM buffer (internal implementation)
 */
nfc_result_t Api_nfc_read_sram(uint8_t *data, uint16_t length)
{
	if (nfc_ntag_handle_p == NULL)
	{
		return NFC_ERROR_NOT_INITIALIZED;
	}
	
	if (data == NULL || length == 0)
	{
		return NFC_ERROR_INVALID_PARAM;
	}
	
	if (length > NFC_SRAM_SIZE)
	{
		length = NFC_SRAM_SIZE;
	}
	
	// printf("Reading %d bytes from SRAM\n", length);
	LOG_API_NFC("Reading %d bytes from SRAM\r\n", length);
	
	return memory_read_data(NFC_SRAM_START_ADDRESS, data, length);
}

/**
 * @brief Write data to SRAM buffer (internal implementation)
 */
nfc_result_t Api_nfc_write_sram(const uint8_t *data, uint16_t length)
{
	if (nfc_ntag_handle_p == NULL)
	{
		return NFC_ERROR_NOT_INITIALIZED;
	}
	
	if (data == NULL || length == 0)
	{
		return NFC_ERROR_INVALID_PARAM;
	}
	
	if (length > NFC_SRAM_SIZE)
	{
		length = NFC_SRAM_SIZE;
	}
	
	// printf("Writing %d bytes to SRAM\n", length);
	LOG_API_NFC("Writing %d bytes to SRAM\r\n", length);
	
	return memory_write_data(NFC_SRAM_START_ADDRESS, data, length);
}


/**
 * @brief Enable pass-through mode for bidirectional data exchange (internal implementation)
 */
static nfc_result_t passthrough_enable(nfc_transfer_direction_t direction)
{
	if (nfc_ntag_handle_p == NULL)
	{
		return NFC_ERROR_NOT_INITIALIZED;
	}

	NTAG_TRANSFER_DIR_T l_dir = (direction == NFC_TRANSFER_I2C_TO_NFC) ? I2C_TO_RF : RF_TO_I2C;
	if (NTAG_SetTransferDir(nfc_ntag_handle_p, l_dir) != NTAG_ERR_OK)
	{
		return NFC_ERROR_COMMUNICATION;
	}
	if (NTAG_SetPthruOnOff(nfc_ntag_handle_p, true) != NTAG_ERR_OK)
	{
		return NFC_ERROR_COMMUNICATION;
	}
	return NFC_SUCCESS;
}

/**
 * @brief Disable pass-through mode (internal implementation)
 */
static nfc_result_t passthrough_disable(void)
{
	NTAG_STATUS_T l_status;
	
	if (nfc_ntag_handle_p == NULL)
	{
		return NFC_ERROR_NOT_INITIALIZED;
	}
	
	// printf("Disabling pass-through mode\n");
	LOG_API_NFC("Disabling pass-through mode\r\n");
	
	l_status = NTAG_WriteRegister(nfc_ntag_handle_p,
								  NTAG_MEM_OFFSET_NC_REG,
								  NTAG_NC_REG_MASK_PTHRU_ON_OFF,
								  0x00);
	
	if (l_status != NTAG_OK)
	{
		// printf("Failed to disable pass-through mode\n");
		LOG_API_NFC("Failed to disable pass-through mode\r\n");
		return NFC_ERROR_COMMUNICATION;
	}
	
	return NFC_SUCCESS;
}


