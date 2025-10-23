/**
 * @file    Drv_uwb_init.c
 * @brief   UWB Driver Initialization State Machine Implementation
 * 
 * This module contains the device initialization state machine that handles
 * hardware reset sequence, device probing, and initial configuration of
 * the DW3000 UWB transceiver.
 * 
 * Key Features:
 * - Non-blocking initialization state machine
 * - Hardware reset sequence management
 * - Device ID verification and validation
 * - Basic device configuration setup
 * - Sleep mode configuration
 * - TX RF parameter configuration
 * 
 * @author  Tag Platform Development Team
 * @version 1.0
 * @date    2025
 */

#include "Drv_uwb_internal.h"
#include "Api_failsafe.h"

device_init_state_e drv_uwb_init_state_g = DEVICE_INIT_STATE_IDLE;

// ========================================================================================
// Private Function Implementations
// ========================================================================================

/**
 * @brief Initialize state machine - start state handler
 * 
 * @details Initiates the device initialization sequence by transitioning
 * to the reset pin low state and clearing initialization variables.
 */
static void uwb_init_state_start(void)
{
	LOG_API_UWB("%s - Starting initialization\r\n", __func__);
	Drv_uwb_init_set_state(DEVICE_INIT_STATE_RESET_PIN_LOW);
	Drv_uwb_set_sequence_timer(0);
	Drv_uwb_set_device_ready(false);
	Drv_uwb_set_device_wakeup(false);
}

/**
 * @brief Initialize state machine - Reset pin LOW state handler
 * 
 * @details Configures reset pin as output and pulls it low to initiate
 * hardware reset sequence of the UWB device.
 */
static void uwb_init_state_reset_pin_low(void)
{
	nrf_gpio_cfg_output(DW3000_RESET_Pin);
	nrf_gpio_pin_clear(DW3000_RESET_Pin);
	Drv_uwb_init_set_state(DEVICE_INIT_STATE_RESET_DELAY);
	Drv_uwb_set_sequence_timer(UWB_RESET_DELAY_MS);
}

/**
 * @brief Initialize state machine - Reset delay state handler
 * 
 * @details Waits for the reset pulse duration to complete before
 * proceeding to the next initialization step.
 */
static void uwb_init_state_reset_delay(void)
{
	if (Drv_uwb_get_sequence_timer() == 0) 
	{
		Drv_uwb_init_set_state(DEVICE_INIT_STATE_RESET_PIN_HIGH);
	}
}

/**
 * @brief Initialize state machine - Reset pin HIGH state handler
 * 
 * @details Releases the reset pin by configuring it as input with no pull,
 * allowing the device to come out of reset state.
 */
static void uwb_init_state_reset_pin_high(void)
{
	nrf_gpio_cfg_input(DW3000_RESET_Pin, NRF_GPIO_PIN_NOPULL);
	Drv_uwb_init_set_state(DEVICE_INIT_STATE_RESET_WAIT);
	Drv_uwb_set_sequence_timer(UWB_RESET_WAIT_MS);
}

/**
 * @brief Initialize state machine - Reset wait state handler
 * 
 * @details Waits for the device to stabilize after reset before
 * attempting to probe the device.
 */
static void uwb_init_state_reset_wait(void)
{
	if (Drv_uwb_get_sequence_timer() == 0) 
	{
		Drv_uwb_init_set_state(DEVICE_INIT_STATE_PROBE_DEVICE);
	}
}

/**
 * @brief Initialize state machine - Probe device state handler
 * 
 * @details Probes the DW3000 device and verifies device ID to ensure
 * proper SPI communication and device presence.
 */
static void uwb_init_state_probe_device(void)
{
    dwt_error_e ret = DWT_ERROR;
	
	ret = dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);

	if  (ret == DWT_ERROR)
	{
		LOG_API_UWB("Device probe failed\r\n");
		Drv_uwb_init_set_state(DEVICE_INIT_STATE_IDLE);
		Api_failsafe_set_uwb_fail(UWB_FAIL_INIT_01);
		return;
	}
	
	LOG_API_UWB("Device probe ok\r\n");
	Drv_uwb_init_set_state(DEVICE_INIT_STATE_READ_DEVICEID);
}
	
static void uwb_init_state_read_deviceID(void)
{
	uint32_t l_dev_id;
	
	l_dev_id = dwt_readdevid();
	
	if (l_dev_id == 0 || l_dev_id == 0xFFFFFFFF) 
	{
		LOG_API_UWB("Device ID failed\r\n");
		Drv_uwb_init_set_state(DEVICE_INIT_STATE_IDLE);
		Api_failsafe_set_uwb_fail(UWB_FAIL_INIT_01);
	} 
	else 
	{
		LOG_API_UWB("DEV ID:0x%X\r\n", l_dev_id);
		Drv_uwb_init_set_state(DEVICE_INIT_STATE_CHECK_IDLE);
	}
}

/**
 * @brief Initialize state machine - Check idle state handler
 * 
 * @details Verifies that the device is in IDLE_RC state and ready
 * for initialization commands.
 */
static void uwb_init_state_check_idle(void)
{
	if (dwt_checkidlerc()) 
	{
		Drv_uwb_init_set_state(DEVICE_INIT_STATE_INITIALIZE);
	}
}

/**
 * @brief Initialize state machine - Initialize state handler
 * 
 * @details Performs the DW3000 device initialization including OTP reading
 * and basic device setup.
 */
static void uwb_init_state_initialize(void)
{
	if (dwt_initialise(DWT_DW_INIT | DWT_READ_OTP_PID | DWT_READ_OTP_LID) == DWT_ERROR) 
	{
		LOG_API_UWB("INIT FAILED\r\n");
		Drv_uwb_init_set_state(DEVICE_INIT_STATE_IDLE);
		Api_failsafe_set_uwb_fail(UWB_FAIL_INIT_02);
	} 
	else 
	{
		Drv_uwb_init_set_state(DEVICE_INIT_STATE_CONFIG_SLEEP);
	}
}

/**
 * @brief Initialize state machine - Configure sleep state handler
 * 
 * @details Configures UWB device sleep and wake-up behavior including
 * sleep counter calibration and wake-up pin configuration.
 */
static void uwb_init_state_sleep_configure(void)
{
	dwt_calibratesleepcnt();
	// Configure sleep/wake behavior: preserve config to AON, wake by CSN and WAKE pin, enable sleep
	//dwt_configuresleep(DWT_CONFIG | DWT_PGFCAL, DWT_PRES_SLEEP | DWT_WAKE_CSN | DWT_WAKE_WUP | DWT_SLP_EN);
	dwt_configuresleep(DWT_CONFIG, DWT_PRES_SLEEP | DWT_WAKE_CSN | DWT_SLP_EN);
	LOG_API_UWB("UWB Sleep configured\r\n");
	Drv_uwb_init_set_state(DEVICE_INIT_STATE_CONFIGURE);
}

/**
 * @brief Initialize state machine - Configure state handler
 * 
 * @details Applies the UWB configuration (channel, data rate, etc.)
 * to the device.
 */
static void uwb_init_state_configure(void)
{
	dwt_config_t l_config;

	Drv_uwb_get_config(&l_config);
	
	if (dwt_configure(&l_config)) 
	{
		LOG_API_UWB("CONFIG FAILED\r\n");
		Drv_uwb_init_set_state(DEVICE_INIT_STATE_IDLE);		
		Api_failsafe_set_uwb_fail(UWB_FAIL_INIT_03);
	} 
	else 
	{
		Drv_uwb_init_set_state(DEVICE_INIT_STATE_CONFIG_TX_RF);
	}
}

/**
 * @brief Initialize state machine - Configure TX RF state handler
 * 
 * @details Configures the TX RF parameters for optimal transmission
 * performance.
 */
static void uwb_init_state_config_tx_rf(void)
{
	dwt_txconfig_t l_txconfig;
	Drv_uwb_get_txconfig(&l_txconfig);
	dwt_configuretxrf(&l_txconfig);
	Drv_uwb_init_set_state(DEVICE_INIT_STATE_COMPLETE);
}

/**
 * @brief Initialize state machine - Complete state handler
 * 
 * @details Initialization completed successfully. Device is ready
 * for normal operation.
 */
static void uwb_init_state_complete(void)
{
	LOG_API_UWB("UWB Device initialization completed\r\n");
	Drv_uwb_set_device_ready(true);
	Drv_uwb_set_device_wakeup(true);
	Drv_uwb_init_set_state(DEVICE_INIT_STATE_IDLE);
	Drv_uwb_tx_set_state(UWB_TX_STATE_IDLE);
	Drv_uwb_set_sequence_timer(0);
	
	Drv_uwb_rx_forceStop();
	
	Api_failsafe_set_uwb_fail(UWB_FAIL_NONE);

	dwt_writesysstatuslo(DWT_INT_RCINIT_BIT_MASK | DWT_INT_SPIRDY_BIT_MASK);
		
//	dwt_setinterrupt(DWT_INT_SPIRDY_BIT_MASK | DWT_INT_RCINIT_BIT_MASK, 
//	                 0,  // bitmask (don't care for enable)
//	                 DWT_ENABLE_INT);
}

// ========================================================================================
// Public Function Implementations
// ========================================================================================

/**
 * @brief Execute one step of device initialization state machine
 * 
 * @details Non-blocking state machine that processes one state per call.
 * Must be called periodically to advance through initialization sequence.
 */
void Drv_uwb_init_process_state_machine(void)
{
	if (true == Drv_uwb_get_device_wakeup()) 
	{
		return;
	}
	
	if (0 != Drv_uwb_get_sequence_timer()) 
	{
		return;
	}
	
	switch (Drv_uwb_init_get_state()) 
	{
		case DEVICE_INIT_STATE_IDLE:
			break;
			
		case DEVICE_INIT_STATE_START:
			uwb_init_state_start();
			break;
			
		case DEVICE_INIT_STATE_RESET_PIN_LOW:
			uwb_init_state_reset_pin_low();
			break;
			
		case DEVICE_INIT_STATE_RESET_DELAY:
			uwb_init_state_reset_delay();
			break;
			
		case DEVICE_INIT_STATE_RESET_PIN_HIGH:
			uwb_init_state_reset_pin_high();
			break;
			
		case DEVICE_INIT_STATE_RESET_WAIT:
			uwb_init_state_reset_wait();
			break;
			
		case DEVICE_INIT_STATE_PROBE_DEVICE:
			uwb_init_state_probe_device();
			break;
		case DEVICE_INIT_STATE_READ_DEVICEID:
			uwb_init_state_read_deviceID();
			break;
			
		case DEVICE_INIT_STATE_CHECK_IDLE:
			uwb_init_state_check_idle();
			break;
			
		case DEVICE_INIT_STATE_INITIALIZE:
			uwb_init_state_initialize();
			break;
			
		case DEVICE_INIT_STATE_CONFIG_SLEEP:
			uwb_init_state_sleep_configure();
			break;
			
		case DEVICE_INIT_STATE_CONFIGURE:
			uwb_init_state_configure();
			break;
			
		case DEVICE_INIT_STATE_CONFIG_TX_RF:
			uwb_init_state_config_tx_rf();
			uwb_init_state_complete();
			break;
			
		case DEVICE_INIT_STATE_COMPLETE:
			uwb_init_state_complete();
			break;
			
	//	case DEVICE_INIT_STATE_ERROR:
	//		uwb_init_state_error();
	//		break;
			
		default:
			Drv_uwb_init_set_state(DEVICE_INIT_STATE_IDLE);
			break;
	}
}


/**
 * @brief Set device initialization state
 * @param[in] state New initialization state
 */
void Drv_uwb_init_set_state(device_init_state_e state)
{
	drv_uwb_init_state_g = state;
}

/**
 * @brief Get current device initialization state
 * @return Current initialization state
 */
device_init_state_e Drv_uwb_init_get_state(void)
{
	return drv_uwb_init_state_g;
}


