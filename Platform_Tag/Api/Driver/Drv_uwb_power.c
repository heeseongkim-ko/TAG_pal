/**
 * @file    Drv_uwb_power.c
 * @brief   UWB Driver Power Management State Machine Implementation
 * 
 * This module contains the power management state machines that handle
 * sleep and wake-up sequences for the DW3000 UWB transceiver. It provides
 * non-blocking state machines for entering and exiting low-power modes.
 * 
 * Key Features:
 * - Non-blocking sleep state machine
 * - Non-blocking wake-up state machine  
 * - Proper SPI communication during power transitions
 * - Context saving and restoration
 * - Device ID verification after wake-up
 * - Comprehensive error handling and timeouts
 * 
 * @author  Tag Platform Development Team
 * @version 1.0
 * @date    2025
 */

#include "Drv_uwb_internal.h"
#include "Drv_uwb_spi.h"
#include "Api_failsafe.h"

// ========================================================================================
// Static Variables (Power Management)
// ========================================================================================

/**
 * @brief Power management state machine variables
 */
static uwb_wakeup_state_e drv_uwb_wakeup_state_g = UWB_WAKEUP_STATE_IDLE;
static uwb_sleep_state_e drv_uwb_sleep_state_g = UWB_SLEEP_STATE_IDLE;
static bool drv_uwb_configUpdate = false;

// ========================================================================================
// Private Function Declarations - Wake-up State Machine
// ========================================================================================

static void uwb_wakeup_state_start(void);
static void uwb_wakeup_state_spi_slow(void);
static void uwb_wakeup_state_cs_low(void);
static void uwb_wakeup_state_cs_hold(void);
static void uwb_wakeup_state_cs_high(void);
static void uwb_wakeup_state_osc_wait(void);
static void uwb_wakeup_state_spirdy_wait(void);
static void uwb_wakeup_state_check_devid(void);
static void uwb_wakeup_state_check_idle(void);
static void uwb_wakeup_state_restore_config(void);
static void uwb_wakeup_state_spi_fast(void);
static void uwb_wakeup_state_complete(void);
static void uwb_wakeup_state_error(void);

// ========================================================================================
// Private Function Declarations - Sleep State Machine
// ========================================================================================

static void uwb_sleep_state_start(void);
static void uwb_sleep_state_force_off(void);
static void uwb_sleep_state_clear_status(void);
static void uwb_sleep_state_save_context(void);
static void uwb_sleep_state_enter_sleep(void);
static void uwb_sleep_state_spi_slow(void);
static void uwb_sleep_state_complete(void);
static void uwb_sleep_state_error(void);

// ========================================================================================
// Private Function Declarations - Context Management
// ========================================================================================

static void restore_uwb_context(void);
static void save_uwb_context(void);

// ========================================================================================
// Private Function Implementations - Wake-up State Machine
// ========================================================================================

/**
 * @brief Wake-up state machine - Start state handler
 * 
 * @details Initiates the device wake-up sequence by transitioning
 * to the SPI slow rate state and logging wake-up start.
 */
static void uwb_wakeup_state_start(void)
{
	LOG_API_UWB("%s\r\n", __func__);
	drv_uwb_wakeup_state_g = UWB_WAKEUP_STATE_SPI_SLOW;
}

/**
 * @brief Wake-up state machine - Set SPI slow rate state handler
 * 
 * @details Sets SPI to slow rate for reliable wake-up communication.
 * Slow SPI rate ensures stable communication during device wake-up.
 */
static void uwb_wakeup_state_spi_slow(void)
{
	LOG_API_UWB("%s\r\n", __func__);
	Drv_uwb_spi_slowrate();
	drv_uwb_wakeup_state_g = UWB_WAKEUP_STATE_CS_LOW;
}

/**
 * @brief Wake-up state machine - CS pin low state handler
 * 
 * @details Configures CS pin as output and pulls it low to initiate
 * the wake-up sequence. Sets delay counter for CS hold time.
 */
static void uwb_wakeup_state_cs_low(void)
{
	LOG_API_UWB("%s\r\n", __func__);
	nrf_gpio_cfg_output(DW3000_CS_Pin);
	nrf_gpio_pin_clear(DW3000_CS_Pin);
	drv_uwb_wakeup_state_g = UWB_WAKEUP_STATE_CS_HOLD;
	Drv_uwb_set_sequence_timer(1);
}

/**
 * @brief Wake-up state machine - CS pin hold state handler
 * 
 * @details Waits for CS hold delay to complete before releasing CS pin.
 * The delay ensures proper wake-up signal timing.
 */
static void uwb_wakeup_state_cs_hold(void)
{
	LOG_API_UWB("%s\r\n", __func__);
	drv_uwb_wakeup_state_g = UWB_WAKEUP_STATE_CS_HIGH;
}

/**
 * @brief Wake-up state machine - CS pin high state handler
 * 
 * @details Releases CS pin high and initiates oscillator wait period.
 * Sets delay counter for oscillator stabilization time.
 */
static void uwb_wakeup_state_cs_high(void)
{
	LOG_API_UWB("%s\r\n", __func__);
	nrf_gpio_pin_set(DW3000_CS_Pin);
	drv_uwb_wakeup_state_g = UWB_WAKEUP_STATE_OSC_WAIT;
	Drv_uwb_set_sequence_timer(1);
	Drv_uwb_set_sequence_timeout_timer(UWB_WAKEUP_SPIRDY_TIMEOUT_COUNT);
}

/**
 * @brief Wake-up state machine - Oscillator wait state handler
 * 
 * @details Waits for low power oscillator to stabilize before proceeding
 * to SPIRDY wait. Sets timeout counter for SPIRDY detection.
 */
static void uwb_wakeup_state_osc_wait(void)
{
	if (Drv_uwb_get_sequence_timer() == 0) 
	{
		LOG_API_UWB("%s\r\n", __func__);
		drv_uwb_wakeup_state_g = UWB_WAKEUP_STATE_SPIRDY_WAIT;
		Drv_uwb_set_sequence_timeout_timer(UWB_WAKEUP_SPIRDY_TIMEOUT_COUNT);
		
		Drv_uwb_set_sequence_timer(1);
	}
	else
	{
		if (Drv_uwb_get_sequence_timeout_timer() == 0u) 
		{
			uwb_wakeup_state_error();
			Api_failsafe_set_uwb_fail(UWB_FAIL_WAKEUP_05);
		}
		else
		{
			//Drv_uwb_set_sequence_timer(1u);
		}
		
	}
}

/**
 * @brief Wake-up state machine - SPIRDY wait state handler
 * 
 * @details Waits for SPI Ready signal from DW3000 indicating the device
 * is ready for SPI communication after wake-up. Includes timeout handling.
 */
static void uwb_wakeup_state_spirdy_wait(void)
{
	uint32_t l_status = dwt_readsysstatuslo();
	
	if (l_status & DWT_INT_SPIRDY_BIT_MASK) 
	{
		drv_uwb_wakeup_state_g = UWB_WAKEUP_STATE_CHECK_DEVID;
		LOG_API_UWB("%s\r\n", __func__);
	} 
	else 
	{
		if (Drv_uwb_get_sequence_timeout_timer() == 0u) 
		{
			uwb_wakeup_state_error();
			Api_failsafe_set_uwb_fail(UWB_FAIL_WAKEUP_03);
			LOG_API_UWB("%s:%X\r\n", __func__, l_status);
		}
		else
		{
			//Drv_uwb_set_sequence_timer(1u);
		}
	}
}

/**
 * @brief Wake-up state machine - Device ID check state handler
 * 
 * @details Verifies the device ID to confirm successful wake-up and
 * SPI communication. Expected device ID is 0xDECA0302 for DW3000.
 */
static void uwb_wakeup_state_check_devid(void)
{
	uint32_t l_dev_id = dwt_readdevid();
	
	if (l_dev_id == 0xDECA0302) 
	{
		drv_uwb_wakeup_state_g = UWB_WAKEUP_STATE_CHECK_IDLE;
		Drv_uwb_set_sequence_timeout_timer(UWB_WAKEUP_IDLERC_TIMEOUT_COUNT);
		LOG_API_UWB("%s\r\n", __func__);
	} 
	else 
	{
		uwb_wakeup_state_error();
		Api_failsafe_set_uwb_fail(UWB_FAIL_WAKEUP_02);
		LOG_API_UWB("%s:error\r\n", __func__);
	}
}

/**
 * @brief Wake-up state machine - IDLE_RC check state handler
 * 
 * @details Verifies that the device is in IDLE_RC state and ready for
 * configuration restoration. Includes timeout for safety.
 */
static void uwb_wakeup_state_check_idle(void)
{
	if (dwt_checkidlerc()) 
	{
		drv_uwb_wakeup_state_g = UWB_WAKEUP_STATE_RESTORE_CONFIG;
	} 
	else 
	{
		if (Drv_uwb_get_sequence_timeout_timer() == 0u) 
		{
			uwb_wakeup_state_error();
			Api_failsafe_set_uwb_fail(UWB_FAIL_WAKEUP_02);
			LOG_API_UWB("%s\r\n", __func__);
		}
	}
}

/**
 * @brief Wake-up state machine - Restore configuration state handler
 * 
 * @details Restores the device configuration that was saved before sleep.
 * Uses dwt_restoreconfig() to restore all AON memory settings.
 */
static void uwb_wakeup_state_restore_config(void)
{
	LOG_API_UWB("%s\r\n", __func__);
	dwt_restoreconfig(1);
	drv_uwb_wakeup_state_g = UWB_WAKEUP_STATE_SPI_FAST;
}

/**
 * @brief Wake-up state machine - Set SPI fast rate state handler
 * 
 * @details Sets SPI to fast rate for normal operation and configures
 * TX RF parameters for optimal performance after wake-up.
 */
static void uwb_wakeup_state_spi_fast(void)
{
	dwt_txconfig_t l_txconfig;
	
	LOG_API_UWB("%s\r\n", __func__);
	Drv_uwb_spi_fastrate();
	Drv_uwb_get_txconfig(&l_txconfig);
	dwt_configuretxrf(&l_txconfig);
	drv_uwb_wakeup_state_g = UWB_WAKEUP_STATE_COMPLETE;
	Drv_uwb_set_sequence_timer(1u);
}

/**
 * @brief Wake-up state machine - Complete state handler
 * 
 * @details Wake-up sequence completed successfully. Device is ready
 * for normal operation and state machine returns to idle.
 */
static void uwb_wakeup_state_complete(void)
{
	LOG_API_UWB("%s\r\n", __func__);
	Drv_uwb_set_device_wakeup(true);
	drv_uwb_wakeup_state_g = UWB_WAKEUP_STATE_IDLE;
	
	Api_failsafe_set_uwb_fail(UWB_FAIL_NONE);
}

/**
 * @brief Wake-up state machine - Error state handler
 * 
 * @details Wake-up sequence failed. Device is not ready and
 * state machine returns to idle for potential retry.
 */
static void uwb_wakeup_state_error(void)
{
	LOG_API_UWB("%s\r\n", __func__);
	Drv_uwb_set_device_wakeup(false);
	drv_uwb_wakeup_state_g = UWB_WAKEUP_STATE_IDLE;
}

// ========================================================================================
// Private Function Implementations - Sleep State Machine
// ========================================================================================

/**
 * @brief Sleep state machine - Start state handler
 * 
 * @details Initiates the device sleep sequence by transitioning
 * to the force off state and logging sleep start.
 */
static void uwb_sleep_state_start(void)
{
	LOG_API_UWB("%s - Starting sleep sequence\r\n", __func__);
	drv_uwb_sleep_state_g = UWB_SLEEP_STATE_FORCE_OFF;
}

/**
 * @brief Sleep state machine - Force off state handler
 * 
 * @details Forces the transceiver off to ensure clean transition
 * to sleep mode. Stops any ongoing TX/RX operations.
 */
static void uwb_sleep_state_force_off(void)
{
	dwt_forcetrxoff();
	drv_uwb_sleep_state_g = UWB_SLEEP_STATE_CLEAR_STATUS;
}

/**
 * @brief Sleep state machine - Clear status state handler
 * 
 * @details Clears all status register flags to ensure clean state
 * before entering sleep mode.
 */
static void uwb_sleep_state_clear_status(void)
{
	dwt_writesysstatuslo(0xFFFFFFFF);
	drv_uwb_sleep_state_g = UWB_SLEEP_STATE_SAVE_CONTEXT;
}

/**
 * @brief Sleep state machine - Save context state handler
 * 
 * @details Saves device context before entering sleep mode.
 * Currently placeholder for future context saving implementation.
 */
static void uwb_sleep_state_save_context(void)
{
	// Save context if needed (currently logging only)
	save_uwb_context();
	drv_uwb_sleep_state_g = UWB_SLEEP_STATE_ENTER_SLEEP;
}

/**
 * @brief Sleep state machine - Enter sleep state handler
 * 
 * @details Commands the device to enter sleep mode with IDLE_RC state.
 * The device will maintain AON memory and can be woken up via CS or WAKE pin.
 */
static void uwb_sleep_state_enter_sleep(void)
{
	dwt_entersleep(DWT_DW_IDLE_RC);
	drv_uwb_sleep_state_g = UWB_SLEEP_STATE_SPI_SLOW;
}

/**
 * @brief Sleep state machine - Set SPI slow rate state handler
 * 
 * @details Sets SPI to slow rate for low power mode communication.
 */
static void uwb_sleep_state_spi_slow(void)
{
	Drv_uwb_spi_slowrate();
	drv_uwb_sleep_state_g = UWB_SLEEP_STATE_COMPLETE;
}

/**
 * @brief Sleep state machine - Complete state handler
 * 
 * @details Sleep sequence completed successfully. Device is in sleep mode
 * and state machine returns to idle.
 */
static void uwb_sleep_state_complete(void)
{
	LOG_API_UWB("UWB sleep entered successfully\r\n");
	Drv_uwb_set_device_wakeup(false);
	drv_uwb_sleep_state_g = UWB_SLEEP_STATE_IDLE;
}

/**
 * @brief Sleep state machine - Error state handler
 * 
 * @details Sleep sequence failed. Logs error and returns state machine
 * to idle for potential retry or error recovery.
 */
static void uwb_sleep_state_error(void)
{
	LOG_API_UWB("UWB sleep entry failed\r\n");
	drv_uwb_sleep_state_g = UWB_SLEEP_STATE_IDLE;
}

// ========================================================================================
// Private Function Implementations - Context Management
// ========================================================================================

/**
 * @brief Context restoration function for UWB device after wake-up
 * 
 * @details This function restores any UWB device context that may have been
 * lost during sleep mode. Currently implements basic context restoration.
 * 
 * @note This is a placeholder for future context restoration implementation.
 *       Additional device-specific context may need to be restored here.
 */
static void restore_uwb_context(void)
{
	// printf_uart("UWB context restored after wake-up\r\n");
}

/**
 * @brief Context saving function for UWB device before entering sleep
 * 
 * @details This function saves any necessary UWB device context before
 * entering sleep mode. Currently implements basic context saving.
 * 
 * @note This is a placeholder for future context saving implementation.
 *       Additional device-specific context may need to be saved here.
 */
static void save_uwb_context(void)
{
	// Save context if needed (currently logging only)
	// printf_uart("UWB context saved before sleep\r\n");
}

// ========================================================================================
// Getter/Setter Functions
// ========================================================================================

/**
 * @brief Get wake-up state
 */
uwb_wakeup_state_e Drv_uwb_power_get_wakeup_state(void)
{
	return drv_uwb_wakeup_state_g;
}

/**
 * @brief Set wake-up state
 */
void Drv_uwb_power_set_wakeup_state(uwb_wakeup_state_e state)
{
	drv_uwb_wakeup_state_g = state;
}

/**
 * @brief Get sleep state
 */
uwb_sleep_state_e Drv_uwb_power_get_sleep_state(void)
{
	return drv_uwb_sleep_state_g;
}

/**
 * @brief Set sleep state
 */
void Drv_uwb_power_set_sleep_state(uwb_sleep_state_e state)
{
	drv_uwb_sleep_state_g = state;
}

void Drv_uwb_set_configUpdate(bool configUpdate)
{
	drv_uwb_configUpdate = configUpdate;
}

// ========================================================================================
// Public Function Implementations
// ========================================================================================
/**
 * @brief Blocking UWB wake-up sequence (optimized for power consumption)
 * 
 * @details This function performs a complete blocking wake-up sequence.
 * Non-blocking approach was partially abandoned to reduce power consumption.
 * The function will block until wake-up completes or fails.
 * 
 * Sequence:
 * 1. Set SPI to slow rate
 * 2. Toggle CS pin (wake-up signal)
 * 3. Wait for SPIRDY signal (with timeout)
 * 4. Verify device ID
 * 5. Check IDLE_RC state (with timeout)
 * 6. Restore configuration
 * 7. Set SPI to fast rate
 * 8. Configure TX RF parameters
 * 
 * @return API_UWB_STATUS_e
 *   - API_UWB_STATUS_SUCCESS: Wake-up completed successfully
 *   - API_UWB_STATUS_FAIL: Wake-up failed at any step
 * 
 * @warning This function blocks CPU execution during wake-up sequence.
 *          Ensure this is called from appropriate context.
 */
#if  1
API_UWB_STATUS_e Drv_uwb_wakeup_machine(void)
{
	API_UWB_STATUS_e status = API_UWB_STATUS_IDLE;
	uint32_t l_status_ui32 = 0u;
	uint32_t l_dev_id_ui32 = 0u;
	uint16_t i = 0;
	bool spi_fast_b = false;
	
	if (true == Drv_uwb_get_device_wakeup()) 
	{
		return API_UWB_STATUS_IDLE;
	}
	
	
	if  (drv_uwb_wakeup_state_g != UWB_WAKEUP_STATE_START)
	{
		return API_UWB_STATUS_IDLE;
	}
	
	LOG_API_UWB("%s\r\n", __func__);	
	LOG_API_UWB("Start blocking wakeup\r\n");
	
	// Step 1: Set SPI to slow rate for wake-up sequence
	//status = Drv_uwb_spi_slowrate();
	if (status == API_UWB_STATUS_FAIL)
	{
		uwb_wakeup_state_error();
		Api_failsafe_set_uwb_fail(UWB_FAIL_WAKEUP_01);
		LOG_API_UWB("SPI slowrate failed\r\n");
		return API_UWB_STATUS_FAIL;
	}
	
	// Step 2: CS toggle sequence to wake up device
	nrf_gpio_cfg_output(DW3000_CS_Pin);
	nrf_gpio_pin_clear(DW3000_CS_Pin);
	nrf_delay_us(600);
	nrf_gpio_pin_set(DW3000_CS_Pin);
	
	// Step 3: Wait for SPIRDY signal with timeout
	Drv_uwb_set_sequence_timeout_timer(3000u);
    //nrf_delay_us(800);
	
	nrf_gpio_cfg(
		DW3000_IRQn_Pin,
		NRF_GPIO_PIN_DIR_INPUT,
		NRF_GPIO_PIN_INPUT_CONNECT,
		NRF_GPIO_PIN_NOPULL,
		NRF_GPIO_PIN_S0S1,
		NRF_GPIO_PIN_NOSENSE
	);

	while(1)
	{
		if  (nrf_gpio_pin_read(DW3000_IRQn_Pin))
		{
			if  (spi_fast_b == false)
			{
				spi_fast_b = true;
				Drv_uwb_spi_fastrate();
			}

			l_status_ui32 = dwt_readsysstatuslo();

			if  ((l_status_ui32 & (DWT_INT_SPIRDY_BIT_MASK | DWT_INT_RCINIT_BIT_MASK))
				== (DWT_INT_SPIRDY_BIT_MASK | DWT_INT_RCINIT_BIT_MASK))
			{
				break;
			}
		}
		
		if (Drv_uwb_get_sequence_timeout_timer() == 0u) 
		{
			uwb_wakeup_state_error();
			Api_failsafe_set_uwb_fail(UWB_FAIL_WAKEUP_04);
			LOG_API_UWB("IDLE_RC timeout\r\n");
			return API_UWB_STATUS_FAIL;
		}
	}
	
    dwt_writesysstatuslo(DWT_INT_RCINIT_BIT_MASK | DWT_INT_SPIRDY_BIT_MASK);
	
	// Step 6: Restore device configuration from AON memory
	dwt_restoreconfig(1);
	
	// Step 8: Configure TX RF parameters
	if  (drv_uwb_configUpdate)
	{
		dwt_txconfig_t l_txconfig;
		dwt_config_t l_config;

		// Apply UWB configuration
		Drv_uwb_get_config(&l_config);
		dwt_configure(&l_config);

		Drv_uwb_get_txconfig(&l_txconfig);
		dwt_configuretxrf(&l_txconfig);

		drv_uwb_configUpdate = false;
	}
	
	// Mark device as awake and ready
	Drv_uwb_set_device_wakeup(true);
	drv_uwb_wakeup_state_g = UWB_WAKEUP_STATE_IDLE;
	Api_failsafe_set_uwb_fail(UWB_FAIL_NONE);	
	
	LOG_API_UWB("Wakeup complete\r\n");
	
	return API_UWB_STATUS_SUCCESS;
}
#else
API_UWB_STATUS_e Drv_uwb_wakeup_machine(void)
{
	API_UWB_STATUS_e status = API_UWB_STATUS_IDLE;
	uint32_t l_status_ui32 = 0u;
	uint32_t l_dev_id_ui32 = 0u;
	uint16_t i = 0;
	
	if (true == Drv_uwb_get_device_wakeup()) 
	{
		return API_UWB_STATUS_IDLE;
	}
	
	
	if  (drv_uwb_wakeup_state_g != UWB_WAKEUP_STATE_START)
	{
		return API_UWB_STATUS_IDLE;
	}
	
	LOG_API_UWB("%s\r\n", __func__);	
	LOG_API_UWB("Start blocking wakeup\r\n");
	
	// Step 1: Set SPI to slow rate for wake-up sequence
	status = Drv_uwb_spi_slowrate();
	if (status == API_UWB_STATUS_FAIL)
	{
		uwb_wakeup_state_error();
		Api_failsafe_set_uwb_fail(UWB_FAIL_WAKEUP_01);
		LOG_API_UWB("SPI slowrate failed\r\n");
		return API_UWB_STATUS_FAIL;
	}
	
	// Step 2: CS toggle sequence to wake up device
	nrf_gpio_cfg_output(DW3000_CS_Pin);
	nrf_gpio_pin_clear(DW3000_CS_Pin);
	nrf_delay_us(600);
	nrf_gpio_pin_set(DW3000_CS_Pin);
	
	// Step 3: Wait for SPIRDY signal with timeout
	Drv_uwb_set_sequence_timeout_timer(3000u);
    //nrf_delay_us(800);
	
	i = 0;
	while (1)
	{
        nrf_delay_us(100);

		l_status_ui32 = dwt_readsysstatuslo();
		
		if (l_status_ui32 & DWT_INT_SPIRDY_BIT_MASK) 
		{
			LOG_API_UWB("SPIRDY detected:%d\r\n", i);
			break;
		} 
		
		if (Drv_uwb_get_sequence_timeout_timer() == 0u) 
		{
			uwb_wakeup_state_error();
			Api_failsafe_set_uwb_fail(UWB_FAIL_WAKEUP_03);
			LOG_API_UWB("SPIRDY timeout (status:0x%X)\r\n", l_status_ui32);
			return API_UWB_STATUS_FAIL;
		}
		++i;
	}
	
	// Step 7: Set SPI to fast rate for normal operation
	Drv_uwb_spi_fastrate();
	
	// Step 4: Verify device ID
	#if  0
	l_dev_id_ui32 = dwt_readdevid();
	if (l_dev_id_ui32 != 0xDECA0302u) 
	{
		uwb_wakeup_state_error();
		Api_failsafe_set_uwb_fail(UWB_FAIL_WAKEUP_02);
		LOG_API_UWB("Invalid device ID: 0x%X\r\n", l_dev_id_ui32);
		return API_UWB_STATUS_FAIL;
	}
	LOG_API_UWB("Device ID verified\r\n");
	#endif
	
	// Step 5: Check IDLE_RC state with timeout
	Drv_uwb_set_sequence_timeout_timer(3000u);
	i = 0;
	while (1)
	{
		if (dwt_checkidlerc()) 
		{
			LOG_API_UWB("IDLE_RC confirmed:%d\r\n", i);
			break;
		}
		
		if (Drv_uwb_get_sequence_timeout_timer() == 0u) 
		{
			uwb_wakeup_state_error();
			Api_failsafe_set_uwb_fail(UWB_FAIL_WAKEUP_04);
			LOG_API_UWB("IDLE_RC timeout\r\n");
			return API_UWB_STATUS_FAIL;
		}		
		++i;
	}

	// Step 6: Restore device configuration from AON memory
	dwt_restoreconfig(1);
	
	// Step 8: Configure TX RF parameters
	if  (drv_uwb_configUpdate)
	{
		dwt_txconfig_t l_txconfig;
		dwt_config_t l_config;

		// Apply UWB configuration
		Drv_uwb_get_config(&l_config);
		dwt_configure(&l_config);

		Drv_uwb_get_txconfig(&l_txconfig);
		dwt_configuretxrf(&l_txconfig);

		drv_uwb_configUpdate = false;
	}
	
	// Mark device as awake and ready
	Drv_uwb_set_device_wakeup(true);
	drv_uwb_wakeup_state_g = UWB_WAKEUP_STATE_IDLE;
	Api_failsafe_set_uwb_fail(UWB_FAIL_NONE);
	
	LOG_API_UWB("Wakeup complete\r\n");
	
	return API_UWB_STATUS_SUCCESS;
}
#endif

/**
 * @brief Execute one step of wake-up state machine
 * 
 * @details Non-blocking state machine that processes one state per call.
 * Must be called periodically to advance through wake-up sequence.
 */
void Drv_uwb_wakeup_process_state_machine(void)
{
	// Handle delay counter (1ms tick resolution)
	if (Drv_uwb_get_sequence_timer() > 0) 
	{
		return;
	}
	
	switch (drv_uwb_wakeup_state_g) 
	{
		case UWB_WAKEUP_STATE_IDLE:
			break;
			
		case UWB_WAKEUP_STATE_START:
			uwb_wakeup_state_start();
			uwb_wakeup_state_spi_slow();
			uwb_wakeup_state_cs_low();
//			break;
			
//		case UWB_WAKEUP_STATE_CS_HOLD:
			uwb_wakeup_state_cs_hold();
			break;
			
		case UWB_WAKEUP_STATE_CS_HIGH:
			uwb_wakeup_state_cs_high();
			break;
			
		case UWB_WAKEUP_STATE_OSC_WAIT:
			uwb_wakeup_state_osc_wait();
			break;
			
		case UWB_WAKEUP_STATE_SPIRDY_WAIT:
			uwb_wakeup_state_spirdy_wait();
			break;
			
		case UWB_WAKEUP_STATE_CHECK_DEVID:
			uwb_wakeup_state_check_devid();
//			break;
			
//		case UWB_WAKEUP_STATE_CHECK_IDLE:
			uwb_wakeup_state_check_idle();
//			break;
			
//		case UWB_WAKEUP_STATE_RESTORE_CONFIG:
			uwb_wakeup_state_restore_config();
			uwb_wakeup_state_spi_fast();
			break;
			
		case UWB_WAKEUP_STATE_COMPLETE:
			uwb_wakeup_state_complete();
			break;
			
//		case UWB_WAKEUP_STATE_ERROR:
//			uwb_wakeup_state_error();
//			break;
			
		default:
			drv_uwb_wakeup_state_g = UWB_WAKEUP_STATE_IDLE;
			break;
	}
}

/**
 * @brief Blocking UWB sleep sequence (optimized for power consumption)
 * 
 * @details This function performs a complete blocking sleep entry sequence.
 * Non-blocking approach was partially abandoned to reduce power consumption.
 * The function will block until sleep entry completes.
 * 
 * Sequence:
 * 1. Force transceiver off (stop TX/RX)
 * 2. Clear all status registers
 * 3. Save device context (placeholder)
 * 4. Enter sleep mode (IDLE_RC with AON retention)
 * 5. Set SPI to slow rate for power saving
 * 6. Mark device as not awake
 * 
 * @return API_UWB_STATUS_e
 *   - API_UWB_STATUS_SUCCESS: Sleep entry completed successfully
 *   - API_UWB_STATUS_FAIL: Sleep entry failed
 * 
 * @warning This function blocks CPU execution during sleep entry.
 *          Typical sleep entry time is ~100-200
 */
API_UWB_STATUS_e Drv_uwb_sleep_machine(void)
{
	API_UWB_STATUS_e status = API_UWB_STATUS_SUCCESS;

	if  (drv_uwb_sleep_state_g != UWB_SLEEP_STATE_START)
	{
		return API_UWB_STATUS_IDLE;
	}
	
	LOG_API_UWB("%s\r\n", __func__);
	LOG_API_UWB("Start blocking sleep\r\n");
	
	// Step 1: Force transceiver off to ensure clean transition
	dwt_forcetrxoff();
	LOG_API_UWB("Transceiver forced off\r\n");
	
	// Step 2: Clear all status register flags
	dwt_writesysstatuslo(0xFFFFFFFFu);
	LOG_API_UWB("Status cleared\r\n");
	
	// Step 3: Save device context (currently placeholder)
	save_uwb_context();
	
	// Step 4: Enter sleep mode with IDLE_RC (AON memory retention)
	// Device can be woken up via CS pin or WAKE pin
	dwt_entersleep(DWT_DW_IDLE_RC);
	LOG_API_UWB("Sleep mode entered\r\n");
	
	// Step 5: Set SPI to slow rate for low power communication
	//status = Drv_uwb_spi_slowrate();
	if (status == API_UWB_STATUS_FAIL)
	{
		LOG_API_UWB("SPI slowrate failed\r\n");
		// Continue anyway - not critical error
	}
	
	// Step 6: Mark device as not awake
	Drv_uwb_set_device_wakeup(false);
	drv_uwb_sleep_state_g = UWB_SLEEP_STATE_IDLE;
	
	LOG_API_UWB("Sleep complete\r\n");
	
	return API_UWB_STATUS_SUCCESS;
}

/**
 * @brief Execute one step of sleep state machine
 * 
 * @details Non-blocking state machine that processes one state per call.
 * Must be called periodically to advance through sleep sequence.
 */
void Drv_uwb_sleep_process_state_machine(void)
{
	switch (drv_uwb_sleep_state_g) 
	{
		case UWB_SLEEP_STATE_IDLE:
			break;
			
		case UWB_SLEEP_STATE_START:
			uwb_sleep_state_start();
			uwb_sleep_state_force_off();
			uwb_sleep_state_clear_status();
			break;
			
		case UWB_SLEEP_STATE_SAVE_CONTEXT:
			uwb_sleep_state_save_context();
			uwb_sleep_state_enter_sleep();
			uwb_sleep_state_spi_slow();
			uwb_sleep_state_complete();
			break;
			
		case UWB_SLEEP_STATE_COMPLETE:
			uwb_sleep_state_complete();
			break;
			
		case UWB_SLEEP_STATE_ERROR:
			uwb_sleep_state_error();
			break;
			
		default:
			drv_uwb_sleep_state_g = UWB_SLEEP_STATE_IDLE;
			break;
	}
}

