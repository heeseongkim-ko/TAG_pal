/**
 * @file Api_battery.c
 * @brief Battery voltage measurement API implementation
 * @details Simple battery voltage measurement with non-blocking operation.
 *          
 *          Implementation features:
 *          - State machine based non-blocking measurement
 *          - SAADC channel 1 configuration with optimal settings
 *          - Automatic GPIO pin configuration and restoration
 *          - Voltage calculation with hardware compensation
 *          - Error handling and timeout protection
 *          
 * @author Platform Tag Team
 * @date 2024
 * @version 1.0
 * 
 * @addtogroup BATTERY_API
 * @{
 */

#include "Api_port.h"
#include "Api_battery.h"
#include "Func_UART_LOG.h"

#include "nrf_drv_saadc.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_common.h"


/**
 * @defgroup BATTERY_Internal Internal Variables
 * @{
 */

/** ADC channel number for battery measurement on SAADC */
#define API_BATTERY_CHANNEL		1

/** Module initialization status flag */
static bool Api_battery_initialized_b = false;

/** Current state of battery measurement state machine */
static Api_battery_state_g Api_battery_current_state_g = API_BATTERY_STATE_IDLE;

/** Last measured battery voltage in millivolts */
static uint16_t Api_battery_voltage_mv_ui16 = 0;

static uint16_t Api_battery_RawData_ui16 = 0;

/** Timeout counter for settling time and error detection (decremented every 1ms) */
static uint32_t Api_battery_timeout_counter_ui32 = 0;

/** Flag indicating ADC conversion completion (used in interrupt handler) */
static bool Api_battery_conversion_done_b = false;

/** ADC timeout in milliseconds for error detection and recovery */
#define API_BATTERY_TIMEOUT_MS		100

/** @} */

/** @} */

/**
 * @defgroup BATTERY_Internal_Functions Internal Functions
 * @{
 */

/**
 * @brief SAADC event handler
 * @param[in] p_event SAADC event
 */
static void Api_battery_saadc_handler(nrfx_saadc_evt_t const * p_event);

/**
 * @brief Convert raw ADC to voltage
 * @param[in] raw_adc Raw ADC value
 * @return Voltage in mV
 */
static uint16_t Api_battery_convert_voltage(uint16_t raw_adc);

/** @} */

/**
 * @defgroup BATTERY_Public_Functions Public Functions
 * @{
 */

/**
 * @brief Initialize battery measurement system
 * @details Configures SAADC with Internal 0.6V reference and 1/4 gain for battery voltage measurement.
 *          Sets up channel 1 for AIN0 input. This is a blocking function that must complete
 *          before other battery functions can be used.
 * @return true on successful initialization, false on failure
 * @note Must be called once during system initialization before using other battery functions
 */
bool Api_battery_init(void)
{
	// Configure SAADC with 12-bit resolution and low power mode
	nrf_drv_saadc_config_t adc_conf = NRF_DRV_SAADC_DEFAULT_CONFIG;
	adc_conf.resolution = NRF_SAADC_RESOLUTION_8BIT;
	adc_conf.oversample = NRF_SAADC_OVERSAMPLE_DISABLED;
	adc_conf.interrupt_priority = SAADC_CONFIG_IRQ_PRIORITY;
	adc_conf.low_power_mode = SAADC_CONFIG_LP_MODE;

	/*
	* Channel 0 configuration (commented out - available for VDD measurement)
	* nrf_saadc_channel_config_t adc_ch0_conf = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(SAADC_CH_PSELP_PSELP_VDD);
	* adc_ch0_conf.gain = NRF_SAADC_GAIN1_6;
	* adc_ch0_conf.acq_time = NRF_SAADC_ACQTIME_40US;
	* nrf_drv_saadc_init(&adc_conf, _ADC_IRQ_handler);
	* nrf_drv_saadc_channel_init(ADC_VDD_channel, &adc_ch0_conf);
	*/

	// Configure channel 1 for battery voltage measurement on AIN0
	nrf_saadc_channel_config_t adc_ch1_conf = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
	adc_ch1_conf.gain = NRF_SAADC_GAIN1_4;  // 1/4 gain: max input = 2.4V
	adc_ch1_conf.acq_time = NRF_SAADC_ACQTIME_40US;
	adc_ch1_conf.reference = NRF_SAADC_REFERENCE_INTERNAL;  // 0.6V internal reference

	// Initialize SAADC driver with event handler
	nrf_drv_saadc_init(&adc_conf, Api_battery_saadc_handler);
	nrf_drv_saadc_channel_init(API_BATTERY_CHANNEL, &adc_ch1_conf);

	// Configure MEAS pin as input
	nrf_gpio_cfg_input(P_ADC_MEAS, NRF_GPIO_PIN_NOPULL);
	
	nrf_gpio_cfg_output(P_ADC_RDIV);
	nrf_gpio_pin_set(P_ADC_RDIV);
	// Set initialization complete and log success
	Api_battery_initialized_b = true;
	LOG_API_BAT("Battery initialized with Internal 0.6V ref, 1/4 gain\r\n");
	
	return true;
}

/**
 * @brief Uninitialize battery measurement system
 *
 * This function uninitializes the SAADC peripheral used for battery voltage measurement.
 * It is designed to be called before entering sleep mode to minimize power consumption.
 * The function performs a complete shutdown of the battery measurement system by
 * uninitializing the SAADC driver and resetting all module state variables.
 *
 * @details Execution sequence:
 *          1. Check if module is initialized (early exit if not)
 *          2. Restore GPIO pins to safe state (ADC_MEAS_TEIA = pullup, ADC_RDIV_TEIA = high)
 *          3. Uninitialize SAADC driver to release all ADC resources
 *          4. Reset all module state variables:
 *             - Clear initialization flag
 *             - Reset state machine to IDLE
 *             - Clear voltage measurement data
 *             - Reset timeout counter
 *             - Clear conversion done flag
 *          5. Log uninitialization result
 *
 * @note This function should be called before entering system sleep mode to ensure
 *       minimal power consumption. The battery system must be reinitialized with
 *       Api_battery_init() before resuming voltage measurements after wake-up.
 *
 * @note GPIO pins are configured for minimum power consumption:
 *       - ADC_MEAS_TEIA: Disconnected (high-Z) with no pull resistors
 *       - ADC_RDIV_TEIA: Set high (disables voltage divider to save power)
 *
 * @warning Do not call this function while a measurement is in progress (BUSY state).
 *          Ensure Api_battery_current_state_g is either IDLE, READY, or ERROR before calling.
 *
 * @return true   Always returns true on successful uninitialization
 * @retval false  Reserved for future error handling
 *
 * @see Api_battery_init() to reinitialize battery measurement after waking from sleep
 * @see Api_battery_state() to check current state before calling this function
 */
bool Api_battery_unInit(void)
{
	// Exit early if module is not initialized
	if (!Api_battery_initialized_b)
	{
		return true;  // Already uninitialized, consider this a success
	}
	
	// Restore GPIO pins to safe state for sleep mode
	// ADC_MEAS_TEIA: Configure as disconnected (high-Z) to minimize power consumption
	// No pullup/pulldown needed since ADC is disabled and voltage divider is off
	nrf_gpio_cfg(
		P_ADC_MEAS,
		NRF_GPIO_PIN_DIR_INPUT,
		NRF_GPIO_PIN_INPUT_DISCONNECT,
		NRF_GPIO_PIN_NOPULL,
		NRF_GPIO_PIN_S0S1,
		NRF_GPIO_PIN_NOSENSE
	);
	// ADC_RDIV_TEIA: Set high to disable voltage divider and reduce power consumption
	nrf_gpio_pin_set(P_ADC_RDIV);
	
	// Uninitialize SAADC driver - releases all ADC resources
	nrf_drv_saadc_uninit();
	
	// Reset all module state variables
	Api_battery_initialized_b = false;
	Api_battery_current_state_g = API_BATTERY_STATE_IDLE;
	Api_battery_voltage_mv_ui16 = 0;
	Api_battery_timeout_counter_ui32 = 0;
	Api_battery_conversion_done_b = false;
	
	// Log successful uninitialization
	LOG_API_BAT("Battery uninitialized for sleep mode\r\n");
	
	return true;
}

/**
 * @brief Helper function for SAADC sample conversion
 * @details Performs blocking SAADC conversion on specified channel.
 *          Used internally for battery voltage measurement.
 *          
 *          Function behavior:
 *          - Triggers single ADC conversion on specified channel
 *          - Waits for completion (blocking, ~100us)
 *          - Returns raw 12-bit ADC result (0-4095 range)
 *          
 * @param[in] channel SAADC channel number to convert (typically API_BATTERY_CHANNEL)
 * @return Raw ADC conversion result (12-bit value, 0-4095)
 * @note This is a blocking function that completes in approximately 100 microseconds
 * @warning Only call this function when SAADC is properly initialized
 */
uint16_t battery_saadc_sample_convert(uint8_t channel)
{
    nrf_saadc_value_t meas_value = 0;
    
    // Perform blocking SAADC conversion
    nrf_drv_saadc_sample_convert(channel, &meas_value);
    
    // Return as unsigned 16-bit value
    return (uint16_t)meas_value;
}

/**
 * @brief Main processing function for battery measurement
 * @details Processes battery measurement state machine. Must be called regularly from main loop.
 *          Handles timeout checking, ADC measurement execution, and state transitions.
 *          Uses switch statement for clear state management according to project guidelines.
 * @return None
 * @note This is a non-blocking function that should be called from main application loop
 */
void Api_battery_main(void)
{
// Exit early if module not initialized
	if (!Api_battery_initialized_b)
	{
		return;
	}

	// Process state machine using switch statement (required by project guidelines)
	switch (Api_battery_current_state_g)
	{
		case API_BATTERY_STATE_IDLE:
			// Idle state - waiting for measurement request
			// Nothing to do until Api_battery_startRead() is called
		break;

		case API_BATTERY_STATE_BUSY:
			// Busy state - settling time in progress or timeout occurred
			if (Api_battery_timeout_counter_ui32 == 0)
			{
				// Settling time complete (1ms), perform ADC measurement
				uint16_t raw_voltage = battery_saadc_sample_convert(API_BATTERY_CHANNEL);

				// Convert raw ADC value to actual battery voltage
				Api_battery_voltage_mv_ui16 = Api_battery_convert_voltage(raw_voltage);

				// Restore pin states for protection and energy saving
			//	nrf_gpio_cfg_input(P_ADC_MEAS, NRF_GPIO_PIN_PULLUP);

				nrf_gpio_pin_set(P_ADC_RDIV);

				// Measurement complete, transition to ready state
				Api_battery_current_state_g = API_BATTERY_STATE_READY;

				// Log measurement result for debugging
				LOG_API_BAT("Battery: %d mV (ADC: %d)\r\n", Api_battery_voltage_mv_ui16, raw_voltage);
			}
		break;

		case API_BATTERY_STATE_READY:
			// Ready state - measurement result available
			// Result can be read using Api_battery_getLevel()
			// Nothing to do until next measurement request
		break;

		case API_BATTERY_STATE_ERROR:
			// Error state - something went wrong during measurement
			// Waiting for next measurement request to retry
		break;

		default:
			// Invalid state detected - reset to idle for safety
			Api_battery_current_state_g = API_BATTERY_STATE_IDLE;
		break;
	}
}

/**
 * @brief Timer tick function for battery measurement (1ms interval)
 * @details Decrements timeout counter used for settling time and error detection.
 *          Must be called every 1ms from timer interrupt according to project guidelines.
 *          Only performs counter operations - no logic processing per project rules.
 * @return None
 * @note This function should be called exactly every 1ms from main timer interrupt
 */
void Api_battery_timer_tick(void)
{
	// Decrement timeout counter if greater than zero (countdown timer approach)
	if (Api_battery_timeout_counter_ui32 > 0)
	{
		Api_battery_timeout_counter_ui32--;
	}
}

/**
 * @brief Get last measured battery voltage level
 * @details Returns the battery voltage from the last completed measurement.
 *          Value is only valid when measurement state is READY.
 * @return Battery voltage in millivolts (mV), or API_BATTERY_INVALID_VOLTAGE if not ready
 * @note Check Api_battery_state() first to ensure measurement is complete
 */
uint16_t Api_battery_getLevel(void)
{
	// Return invalid voltage if not initialized or no valid measurement available
	if (!Api_battery_initialized_b || Api_battery_current_state_g != API_BATTERY_STATE_READY)
	{
		return API_BATTERY_INVALID_VOLTAGE;
	}
	
	// Return cached voltage measurement result
	return Api_battery_voltage_mv_ui16;
}

/**
 * @brief Start non-blocking battery voltage measurement
 * @details Configures GPIO pins for measurement and starts 1ms settling timer.
 *          This is a non-blocking function that returns immediately.
 *          Use Api_battery_main() to process the measurement and Api_battery_getLevel() to get result.
 * @return true if measurement started successfully, false if busy or not initialized
 * @note Call Api_battery_main() regularly to process the measurement
 */
bool Api_battery_startRead(void)
{
	// Check if module is initialized
	if (!Api_battery_initialized_b)
	{
		LOG_API_BAT("Battery not initialized\r\n");
		return false;
	}
	
	// Check if already busy with measurement
	if (Api_battery_current_state_g != API_BATTERY_STATE_IDLE && Api_battery_current_state_g != API_BATTERY_STATE_READY)
	{
		LOG_API_BAT("Battery busy\r\n");
		return false;
	}
	
	// Configure GPIO pins for battery voltage measurement
	nrf_gpio_pin_clear(P_ADC_RDIV);      // Set RDIV pin low for voltage divider
	
	// Start 1ms settling timer (non-blocking approach)
	Api_battery_current_state_g = API_BATTERY_STATE_BUSY;
	Api_battery_timeout_counter_ui32 = 1;  // 1ms settling time
	
	LOG_API_BAT("Battery read started, settling...\r\n");
	return true;
}

/**
 * @brief Get current battery measurement state
 * @details Returns the current state of the battery measurement state machine.
 *          Can be used by application to check measurement progress.
 * @return Current state (IDLE, BUSY, READY, ERROR)
 * @note Use this to check if measurement is complete before calling Api_battery_getLevel()
 */
Api_battery_state_g Api_battery_state(void)
{
	// Return current state machine state
	return Api_battery_current_state_g;
}

/** @} */

/**
 * @defgroup BATTERY_Internal_Functions Internal Function Implementation
 * @{
 */

/**
 * @brief SAADC event handler for battery measurement
 * @details Handles SAADC completion and error events. Called by SAADC driver
 *          when ADC conversion is complete or when an error occurs.
 *          Sets flags for processing in main function.
 * @param[in] p_event Pointer to SAADC event structure containing event type and data
 * @return None
 * @note This function is called from interrupt context
 */
static void Api_battery_saadc_handler(nrfx_saadc_evt_t const * p_event)
{
	// Check event type
	if (p_event->type == NRFX_SAADC_EVT_DONE)
	{
		// ADC conversion completed successfully
		Api_battery_conversion_done_b = true;
		LOG_API_BAT("ADC done\r\n");
	}
	else
	{
		// ADC error occurred
		LOG_API_BAT("ADC error: %d\r\n", p_event->type);
		Api_battery_current_state_g = API_BATTERY_STATE_ERROR;
	}
}

/**
 * @brief Convert raw ADC value to actual battery voltage
 * @details Converts 12-bit ADC value to battery voltage in millivolts.
 *          Takes into account SAADC reference voltage, gain setting, and external voltage divider.
 *          
 *          Calculation steps:
 *          1. ADC input voltage = (raw_adc * 2400) / 4096 mV
 *             - 2400mV = 0.6V reference * 4 (1/4 gain)
 *          2. Actual battery voltage = ADC input voltage * 2
 *             - Factor of 2 compensates for 1/2 voltage divider (0.5 ratio)
 *          
 * @param[in] raw_adc Raw 12-bit ADC conversion result (0-4095)
 * @return Actual battery voltage in millivolts (0-5000mV), clamped to reasonable range
 * @note Assumes 1/2 voltage divider circuit on battery input
 */
static uint16_t Api_battery_convert_voltage(uint16_t raw_adc)
{
	// Step 1: Convert raw ADC to voltage at ADC input pin
	// SAADC configuration: Internal 0.6V reference + 1/4 gain = max input 2.4V
	// Formula: ADC_voltage = (raw_adc * max_input_voltage) / max_adc_value
	uint32_t adc_voltage_mv = ((uint32_t)raw_adc * API_BATTERY_SAADC_FULL_SCALE_MV) / API_BATTERY_SAADC_MAX_VALUE;
	
	// Step 2: Convert ADC input voltage to actual battery voltage
	// Hardware has 1/2 voltage divider: Vbatt * 0.5 = Vadc_input
	// Therefore: Vbatt = Vadc_input / 0.5 = Vadc_input * 2
	uint32_t battery_voltage_mv = adc_voltage_mv * API_BATTERY_VOLTAGE_DIVIDER_RATIO;
	
	// Step 3: Clamp result to reasonable battery voltage range (0-5V)
	if (battery_voltage_mv > API_BATTERY_MAX_VOLTAGE_MV)
	{
		battery_voltage_mv = API_BATTERY_MAX_VOLTAGE_MV;
	}

	Api_battery_RawData_ui16 = raw_adc;
	// Return as 16-bit value
	return (uint16_t)battery_voltage_mv;
}

uint16_t Api_battery_getRawData(void)
{
	return Api_battery_RawData_ui16;
}

/** @} */

