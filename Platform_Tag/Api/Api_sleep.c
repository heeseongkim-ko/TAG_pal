/**
 * @file Api_sleep.c
 * @brief Sleep management API implementation for Tag Platform
 * @details This module implements sleep functionality and wakeup reason tracking
 *          for power management in the Tag Platform system
 * @author Tag Platform Development Team
 * @date 2025
 */

#include "sdk_config.h"
#include "app_error.h"
#include "drv_rtc.h"
#include "nrf_rtc.h"
#include "nrf52840.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"

#include "nrf_sdh.h"

#include "Func_UART_LOG.h"

#include <string.h>
#include <stdio.h>

#include "Api_port.h"
#include "Api_sleep.h"
#include "Api_motion.h"
#include "Api_sleep_peripheral.h"
#include "Api_failsafe.h"
#include "Api_ble.h"
#include "Api_Led.h"
#include "Api_battery.h"

/*==============================================================================
 * Private Variables
 *============================================================================*/

/**
 * @brief Global variable storing the current wakeup reason
 * @details This variable tracks the reason for the last wakeup event and is
 *          updated when the device wakes from sleep mode
 */
uint8_t api_sleep_wakeupReason_ui8;

/**
 * @brief Sleep request flag
 * @details Boolean flag indicating whether a sleep request has been made.
 *          Set to true when Api_sleep_setSleep() is called, processed by
 *          Api_sleep_main() function
 */
bool api_sleep_request_b;

/**
 * @brief Sleep time duration in milliseconds
 * @details Global variable storing the configured sleep duration time.
 *          This value determines how long the device should remain in sleep mode
 *          before automatically waking up (if no other wakeup source triggers first)
 * @note Time unit is in milliseconds
 * @note Value of 0 may indicate indefinite sleep until external wakeup
 */
uint32_t api_sleep_time_ui32;

/*==============================================================================
 * Private Variables
 *============================================================================*/

/**
 * @brief RTC2 instance for sleep timer functionality
 * @details Static RTC instance used exclusively for sleep timer operations.
 *          Uses RTC2 peripheral to provide accurate timing for sleep duration
 *          control with low power consumption.
 * @note RTC2 is dedicated for sleep timing and should not be used by other modules
 * @note Configured to use 32.768 kHz low frequency clock for power efficiency
 */
static drv_rtc_t sleep_rtc_instance = DRV_RTC_INSTANCE(2);

/*==============================================================================
 * Private Functions
 *============================================================================*/

/**
 * @brief RTC event handler for sleep timer
 * @details Callback function invoked when RTC2 compare event occurs.
 *          Handles the timer expiration event and clears the interrupt flag
 *          to prevent infinite interrupts.
 * @param[in] p_instance Pointer to the RTC instance that triggered the event
 * @note This function runs in interrupt context, keep processing minimal
 * @note Automatically clears EVENTS_COMPARE[0] to prevent interrupt retriggering
 * @note Should set wakeup reason to TIMER when timer expires
 */
static void sleep_rtc_event_handler(drv_rtc_t const * const p_instance)
{
	(void)p_instance;  // Unused parameter
	
	// Prevent infinite interrupt by clearing the event
	NRF_RTC2->EVENTS_COMPARE[0] = 0;
	
	// Set wakeup reason to indicate timer-based wakeup
	Api_sleep_setWakeupReason(API_SLEEP_WAKEUP_REASON_MASK_TIMER);
}

/*==============================================================================
 * Private Functions - Sleep Timer Management
 *============================================================================*/

/**
 * @brief Initialize the sleep timer system
 * @details Initializes RTC2 peripheral for sleep timer functionality.
 *          Configures the RTC with default settings, registers event handler,
 *          and enables necessary interrupts for timer operation.
 * @return bool Initialization status
 * @retval true Initialization successful
 * @retval false Initialization failed
 * @note Must be called before using sleep_timer_start() or sleep_timer_stop()
 * @note Uses RTC2 peripheral exclusively - do not use RTC2 elsewhere
 * @note Enables NVIC interrupt for RTC2 to handle timer events
 * 
 * @code
 * // Initialize sleep timer system
 * if (!sleep_timer_init()) {
 *     // Handle initialization error
 * }
 * @endcode
 */
bool sleep_timer_init(void)
{
	// RTC2 configuration with default settings
	drv_rtc_config_t config = DRV_RTC_DEFAULT_CONFIG;
	
	// Initialize RTC2 with configuration and event handler
	ret_code_t err_code = drv_rtc_init(&sleep_rtc_instance, &config, sleep_rtc_event_handler);
	if (err_code != NRF_SUCCESS) {
		return false;
	}
	
	// Enable RTC2 interrupt for compare channel 0
	nrf_rtc_int_enable(sleep_rtc_instance.p_reg, NRF_RTC_INT_COMPARE0_MASK);
	NVIC_EnableIRQ(RTC2_IRQn);
	
	return true;
}

/**
 * @brief Stop the sleep timer
 * @details Stops the RTC2 counter to halt sleep timer operation.
 *          Can be called to cancel an active sleep timer before it expires.
 * @note Safe to call even if timer is not running
 * @note Does not disable interrupts or deinitialize the RTC
 * @note Timer can be restarted with sleep_timer_start() after stopping
 * 
 * @code
 * // Stop active sleep timer
 * sleep_timer_stop();
 * @endcode
 */
void sleep_timer_stop(void)
{
	drv_rtc_stop(&sleep_rtc_instance);
}

/**
 * @brief Start the sleep timer with specified interval
 * @details Starts RTC2 timer to expire after the specified interval.
 *          Converts milliseconds to RTC ticks and sets up compare event
 *          to trigger when the interval elapses.
 * @param[in] interval_ms Sleep duration in milliseconds (1ms minimum)
 * @return bool Timer start status
 * @retval true Timer started successfully
 * @retval false Timer start failed (should not occur with valid parameters)
 * @note Minimum interval is 1ms, 0ms intervals are converted to 1ms
 * @note Uses 32.768 kHz clock: 1ms ? 32.768 ticks
 * @note Maximum interval limited by 24-bit RTC counter (?512 seconds)
 * @note Automatically handles counter overflow with 24-bit masking
 * 
 * @code
 * // Start 5 second sleep timer
 * if (sleep_timer_start(5000)) {
 *     // Timer started successfully
 * }
 * 
 * // Start 100ms timer
 * sleep_timer_start(100);
 * @endcode
 */
bool sleep_timer_start(uint32_t interval_ms)
{
	// Convert milliseconds to RTC ticks (32.768 kHz clock)
	uint32_t ticks = (interval_ms * 32768u) / 1000u;
	if (ticks == 0u) ticks = 1u;  // Prevent 0ms intervals, ensure minimum 1 tick
	
	// Calculate compare value with current counter (handle 24-bit overflow)
	uint32_t now = nrf_rtc_counter_get(sleep_rtc_instance.p_reg);
	uint32_t cc  = (now + ticks) & RTC_COUNTER_COUNTER_Msk;  // 24-bit mask for overflow
	
	// Clear any pending compare events and setup new compare value
	nrf_rtc_event_clear(sleep_rtc_instance.p_reg, NRF_RTC_EVENT_COMPARE_0);
	drv_rtc_compare_set(&sleep_rtc_instance, 0, cc, true);
	
	// Start RTC2 counter to begin timing
	drv_rtc_start(&sleep_rtc_instance);
	
	return true;
}


#define NRF52_ONRAM1_OFFRAM1  	POWER_RAM_POWER_S0POWER_On << POWER_RAM_POWER_S0POWER_Pos | POWER_RAM_POWER_S1POWER_On  << POWER_RAM_POWER_S1POWER_Pos      \
                                    | POWER_RAM_POWER_S0RETENTION_On  << POWER_RAM_POWER_S0RETENTION_Pos | POWER_RAM_POWER_S1RETENTION_On  << POWER_RAM_POWER_S1RETENTION_Pos;

#define NRF52_ONRAM1_OFFRAM0    POWER_RAM_POWER_S0POWER_On << POWER_RAM_POWER_S0POWER_Pos | POWER_RAM_POWER_S1POWER_On << POWER_RAM_POWER_S1POWER_Pos      \
                                    | POWER_RAM_POWER_S0RETENTION_Off << POWER_RAM_POWER_S0RETENTION_Pos | POWER_RAM_POWER_S1RETENTION_Off << POWER_RAM_POWER_S1RETENTION_Pos;

#define NRF52_ONRAM0_OFFRAM0    POWER_RAM_POWER_S0POWER_Off<< POWER_RAM_POWER_S0POWER_Pos | POWER_RAM_POWER_S1POWER_Off<< POWER_RAM_POWER_S1POWER_Pos;

/**
 * @brief Safe RAM power configuration for typical BLE application
 */
void sleep_ram_power_optimize(void)
{
	// Conservative approach: Keep RAM[0-2] and RAM[7] ON
	NRF_POWER->RAM[0].POWER = NRF52_ONRAM1_OFFRAM0;  // ON
	NRF_POWER->RAM[1].POWER = NRF52_ONRAM1_OFFRAM0;  // ON
	NRF_POWER->RAM[2].POWER = NRF52_ONRAM1_OFFRAM0;  // ON
	NRF_POWER->RAM[3].POWER = 0;           // OFF
	NRF_POWER->RAM[4].POWER = 0;           // OFF
	NRF_POWER->RAM[5].POWER = 0;           // OFF
	NRF_POWER->RAM[6].POWER = 0;           // OFF
	NRF_POWER->RAM[7].POWER = NRF52_ONRAM1_OFFRAM0;  // ON (Stack)
}

/*==============================================================================
 * Public Functions
 *============================================================================*/

/**
 * @brief Request the device to enter sleep mode
 * @details Sets the sleep request flag and resets the wakeup reason to NULL.
 *          The actual sleep entry is handled by the main sleep handler.
 * @note This function only sets the request flag, actual sleep implementation
 *       depends on system state and other module requirements
 */
void Api_sleep_start_sleep(void)
{
	api_sleep_request_b = true;
	LOG_API_SLEEP("StartSleep\r\n");
}

/**
 * @brief Set the wakeup reason for the current/next wakeup event
 * @details Sets the global wakeup reason variable to track what caused
 *          or will cause the device to wake up from sleep mode
 * @param[in] reason The wakeup reason to be set
 * @note This function should be called by interrupt handlers or other
 *       modules that detect wakeup events to properly track the source
 */
void Api_sleep_setWakeupReason(uint8_t reason)
{
	api_sleep_wakeupReason_ui8 = api_sleep_wakeupReason_ui8 | reason;
}

void Api_sleep_claerWakeupReasonMask(uint8_t reason)
{
	api_sleep_wakeupReason_ui8 = api_sleep_wakeupReason_ui8 & (~reason);
}

/**
 * @brief Get the reason for the last wakeup event
 * @details Returns the enumerated value indicating what caused the device
 *          to wake up from its last sleep period
 * @return api_sleep_wakeupReason_t The wakeup reason enumeration value
 * @retval API_SLEEP_WAKEUPREASON_NULL No wakeup reason set or default state
 * @retval API_SLEEP_WAKEUPREASON_TIMER Wakeup caused by timer
 * @retval API_SLEEP_WAKEUPREASON_MOTION Wakeup caused by motion sensor
 * @retval API_SLEEP_WAKEUPREASON_NFC Wakeup caused by NFC activity
 * @retval API_SLEEP_WAKEUPREASON_OTHER Wakeup caused by other sources
 */
uint8_t Api_sleep_getWakeupReason(void)
{
	return api_sleep_wakeupReason_ui8;
}

/**
 * @brief Set the sleep duration time
 * @details Configures the amount of time the device should remain in sleep mode
 *          before automatically waking up. This sets the sleep timer duration.
 * @param[in] times Sleep duration in milliseconds
 * @note Setting times to 0 may result in indefinite sleep until external wakeup
 * @note The actual sleep duration may be affected by system timer resolution
 * @note This function only sets the duration, actual timer start occurs during sleep entry
 */
void Api_sleep_setSleepTime(uint32_t times)
{
	api_sleep_time_ui32 = times;
}

extern void Aply_timer_stop(void);
extern void Aply_timer_start(void);

void Api_sleep_core(void)
{
#if	0//def SOFTDEVICE_PRESENT
    if (nrf_sdh_is_enabled())
    {
        ret_code_t ret_code = sd_app_evt_wait();
        ASSERT((ret_code == NRF_SUCCESS) || (ret_code == NRF_ERROR_SOFTDEVICE_NOT_ENABLED));
        UNUSED_VARIABLE(ret_code);
    }
    else
#endif // SOFTDEVICE_PRESENT
    {
        // Wait for an event.
        __WFE();
        // Clear the internal event register.
        __SEV();
        __WFE();
    }
}

void check_clock_status(void)
{
    LOG_API_SLEEP("\r\n=== Clock Status ===\r\n");
    
    LOG_API_SLEEP("HFCLKSTAT: 0x%08X\r\n", NRF_CLOCK->HFCLKSTAT);
    if (NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_STATE_Msk)
    {
        LOG_API_SLEEP("HFCLK: RUNNING! ");
        if (NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_SRC_Msk)
            LOG_API_SLEEP("(XTAL)\r\n");
        else
            LOG_API_SLEEP("(RC)\r\n");
    }
    else
    {
        LOG_API_SLEEP("HFCLK: OFF\r\n");
    }
    
    LOG_API_SLEEP("LFCLKSTAT: 0x%08X\r\n", NRF_CLOCK->LFCLKSTAT);
    
    LOG_API_SLEEP("HFCLKRUN: %u\r\n", NRF_CLOCK->HFCLKRUN);
    
    LOG_API_SLEEP("===================\r\n\r\n");
}

//nrf_gpio_cfg_output(DW3000_RESET_Pin);
//nrf_gpio_pin_clear(DW3000_RESET_Pin);

bool sleep_is_used_pin(uint32_t pin)
{
	if  (pin == DW3000_RESET_Pin)
	{
		return true;
	}

	return false;
}

void Api_sleep_configure_unused_pins(void)
{
    // P0.0 ~ P0.31
    for (uint32_t pin = 0; pin < 48; pin++)
    {
    	if  (sleep_is_used_pin(pin) == false)
    	{
	    	nrf_gpio_cfg(
				pin,
				NRF_GPIO_PIN_DIR_INPUT,
				NRF_GPIO_PIN_INPUT_DISCONNECT,
				NRF_GPIO_PIN_NOPULL,
				NRF_GPIO_PIN_S0S1,
				NRF_GPIO_PIN_NOSENSE
			);
    	}
    }
}

/**
 * @brief Main sleep management routine
 * @details Handles the sleep state machine and processes sleep requests.
 *          This function should be called periodically from the main loop
 *          to check for sleep requests and manage power states.
 * @note This function returns immediately if no sleep request is pending
 */
void Api_sleep_main(void)
{	
	if  ((Api_failsafe_isRecovery()) || (Api_failsafe_isMajor()))
	{
		return;
	}
	
	/* Early return if no sleep request is pending */
	if (api_sleep_request_b == false)
	{
		return;
	}

	if  (Api_ble_dfu_enabled() == true)
	{
		return;
	}	

	if  (Api_Led_IsActive() == true)
	{
	#if  1
		while(1)
		{
			if  (Api_sleep_getWakeupReason() & API_SLEEP_WAKEUP_REASON_MASK_TICK_TIMER)
			{
				break;
			}
			Api_sleep_core();
		}
	#endif
	
		Api_sleep_claerWakeupReasonMask(API_SLEEP_WAKEUP_REASON_MASK_TICK_TIMER);

		return;
	}
	
	api_sleep_request_b = false;

	Aply_timer_stop();

	if  (Api_motion_is_detected())
	{
		Api_motion_clear_interrupt();
	}

	LOG_API_SLEEP("Sleep:%ld\r\n", api_sleep_time_ui32);
	
	api_sleep_wakeupReason_ui8 = API_SLEEP_WAKEUP_REASON_MASK_NULL;

	Api_sleep_peripheral_disable_before_sleep();

	Api_sleep_configure_unused_pins();

	sleep_timer_start(api_sleep_time_ui32);

    nrf_drv_clock_hfclk_release();
	
	while(1)
	{		
		if  (api_sleep_wakeupReason_ui8 != API_SLEEP_WAKEUP_REASON_MASK_NULL)
		{
			break;
		}
		Api_sleep_core();
	}
		
	nrf_drv_clock_hfclk_request(NULL);
		
	if (Api_failsafe_clock_check_hfclk_start())
	{
		Api_failsafe_set_success(FAILSAFE_CODE_16);
	}
	else
	{
		Api_failsafe_set_fail(FAILSAFE_CODE_16);
	}

	Api_sleep_peripheral_enable_after_wakeup();

	sleep_timer_stop();

	Api_failsafe_battery_time_update(api_sleep_time_ui32);

	if  (Api_sleep_getWakeupReason() & API_SLEEP_WAKEUP_REASON_MASK_TIMER)
	{
		LOG_API_SLEEP("Wakeup : TIMER\r\n");
	}
	if  (Api_sleep_getWakeupReason() & API_SLEEP_WAKEUP_REASON_MASK_TICK_TIMER)
	{
		LOG_API_SLEEP("Wakeup : TICK TIMER\r\n");
	}
	if  (Api_sleep_getWakeupReason() & API_SLEEP_WAKEUP_REASON_MASK_MOTION)
	{
		LOG_API_SLEEP("Wakeup : MOTION\r\n");
	}
	if  (Api_sleep_getWakeupReason() & API_SLEEP_WAKEUP_REASON_MASK_NFC)
	{
		LOG_API_SLEEP("Wakeup : NFC\r\n");
	}
	if  (Api_sleep_getWakeupReason() == API_SLEEP_WAKEUP_REASON_MASK_ALL)
	{
		LOG_API_SLEEP("Wakeup : ALL\r\n");
	}

	Api_battery_startRead();
	Aply_timer_start();
}

/**
 * @brief Initialize the Sleep API module
 * @details Initializes the sleep management system and underlying power management.
 *          This function must be called once during system startup before using
 *          any other sleep API functions. Internally initializes the nRF power
 *          management module which handles low-power modes and system idle states.
 * @note This function should be called only once during system initialization
 * @note Must be called after basic system initialization but before main loop
 * @warning Do not call this function multiple times as it may cause system instability
 */
void Api_sleep_init(void)
{
	/* Initialize nRF power management module */
	/* This sets up the low-power infrastructure for the entire system */
	
	//sleep_ram_power_optimize();
	
	// nrf_pwr_mgmt_init();
	
	/* Initialize local sleep management variables */
	api_sleep_wakeupReason_ui8 = API_SLEEP_WAKEUP_REASON_MASK_NULL;
	api_sleep_request_b = false;
	api_sleep_time_ui32 = 0;
	
	sleep_timer_init();
}

