/**
 * @file Api_Led.c
 * @brief Non-blocking LED control implementation for TEIA platform
 * @details Simple LED control using 1ms timer interrupt for counting
 * @author Platform Tag Team
 * @date 2024
 * @version 1.0
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "Api_port.h"
#include "Api_Led.h"
#include "nrf_gpio.h"
#include "Func_UART_LOG.h"

/**
 * @defgroup LED_Internal_Variables Internal State Variables
 * @{
 */

/** LED state timer for timing control (decremented every 1ms) */
static uint32_t Api_Led_state_timer_ui32 = 0;

/** Current LED operation mode */
static Api_Led_mode_g Api_Led_current_mode_g = API_LED_OFF;

/** Current state machine state */
static Api_Led_state_g Api_Led_current_state_g = API_LED_STATE_IDLE;

/** LED on time setting in milliseconds */
static uint32_t Api_Led_on_time_setting_ui32 = API_LED_DEFAULT_ON_TIME_MS;

/** LED off time setting in milliseconds */
static uint32_t Api_Led_off_time_setting_ui32 = API_LED_DEFAULT_OFF_TIME_MS;

/** Total repeat count for blinking pattern */
static uint32_t Api_Led_repeat_count_ui32 = 0;

/** Current repeat counter (incremented each blink cycle) */
static uint32_t Api_Led_current_repeat_ui32 = 0;

/** Module initialization flag - true when LED module is initialized */
static bool Api_Led_is_initialized_b = false;

/** @} */

/**
 * @defgroup LED_Internal_Functions Internal Functions
 * @{
 */

/**
 * @brief Control green LED hardware pin
 * @details Controls the green LED using active-low logic.
 *          Pin is cleared (LOW) to turn LED on, set (HIGH) to turn LED off.
 * @param[in] on true to turn LED on, false to turn LED off
 * @return None
 */
static void Api_Led_set_green_led(bool on)
{
	if (on)
	{
		nrf_gpio_pin_clear(P_LED_GREEN);  // Active low - clear pin to turn ON
	}
	else
	{
		nrf_gpio_pin_set(P_LED_GREEN);    // Active low - set pin to turn OFF
	}
}

/**
 * @brief Control red LED hardware pin
 * @details Controls the red LED using active-low logic.
 *          Pin is cleared (LOW) to turn LED on, set (HIGH) to turn LED off.
 * @param[in] on true to turn LED on, false to turn LED off
 * @return None
 */
static void Api_Led_set_red_led(bool on)
{
	if (on)
	{
		nrf_gpio_pin_clear(P_LED_RED);    // Active low - clear pin to turn ON
	}
	else
	{
		nrf_gpio_pin_set(P_LED_RED);      // Active low - set pin to turn OFF
	}
}

/**
 * @brief Set LEDs according to current mode
 * @details Controls both LEDs to display the specified color mode.
 *          Supports OFF, GREEN, RED, and ORANGE (both LEDs) modes.
 * @param[in] mode LED color mode to display
 * @return None
 * @note ORANGE mode turns on both green and red LEDs simultaneously
 */
static void Api_Led_set_led_mode(Api_Led_mode_g mode)
{
	switch (mode)
	{
		case API_LED_OFF:
			Api_Led_set_green_led(false);  // Turn off green LED
			Api_Led_set_red_led(false);    // Turn off red LED
			LOG_API_LED("[LED] Api_Led_set_led_mode:OFF\r\n");
			break;

		case API_LED_GREEN:
			Api_Led_set_green_led(true);   // Turn on green LED
			Api_Led_set_red_led(false);    // Turn off red LED
			LOG_API_LED("[LED] Api_Led_set_led_mode:GREEAN\r\n");
			break;

		case API_LED_RED:
			Api_Led_set_green_led(false);  // Turn off green LED
			Api_Led_set_red_led(true);     // Turn on red LED
			LOG_API_LED("[LED] Api_Led_set_led_mode:RED\r\n");
			break;

		case API_LED_ORANGE:
			Api_Led_set_green_led(true);   // Turn on green LED
			Api_Led_set_red_led(true);     // Turn on red LED (creates orange)
			LOG_API_LED("[LED] Api_Led_set_led_mode:ORANGE\r\n");
			break;

		default:
			// Invalid mode - default to OFF for safety
			Api_Led_set_green_led(false);
			Api_Led_set_red_led(false);
			LOG_API_LED("[LED] Api_Led_set_led_mode:??\r\n");
			break;
	}
}

/**
 * @brief Reset LED state machine to idle state
 * @details Clears all state variables and turns off all LEDs.
 *          Resets repeat counters, timers, and sets state to IDLE.
 * @return None
 * @note This function is called during initialization and when stopping LED operation
 */
static void Api_Led_reset_state_machine(void)
{
	Api_Led_current_state_g = API_LED_STATE_IDLE;
	Api_Led_current_repeat_ui32 = 0;
	Api_Led_state_timer_ui32 = 0;
	Api_Led_set_led_mode(API_LED_OFF);
}

/** @} */

/**
 * @defgroup LED_Public_Functions Public Function Implementations
 * @{
 */

/**
 * @brief Initialize LED control module
 * @details Configures GPIO pins for LED control and initializes internal state.
 *          Sets up green and red LED pins as outputs with active-low configuration.
 *          All LEDs are turned off initially and state machine is reset to idle.
 * @return true on successful initialization, false on failure
 * @note This function must be called before using any other LED functions
 * @note LEDs use active-low logic (pin LOW = LED ON, pin HIGH = LED OFF)
 */
bool Api_Led_Init(void)
{
	// Configure GPIO pins for LED control
	nrf_gpio_cfg_output(P_LED_GREEN);
	nrf_gpio_cfg_output(P_LED_RED);

	// Turn off all LEDs initially (active low)
	nrf_gpio_pin_set(P_LED_GREEN);
	nrf_gpio_pin_set(P_LED_RED);

	// Initialize state machine
	Api_Led_reset_state_machine();

	// Mark as initialized
	Api_Led_is_initialized_b = true;

	LOG_API_LED("[LED] Init completed successfully\r\n");
	return true;
}

/**
 * @brief Set LED to solid color (continuously on)
 * @details Sets LED to specified color and keeps it on without blinking.
 *          Immediately stops any ongoing blink pattern and switches to solid mode.
 * @param[in] mode LED color mode (API_LED_GREEN, API_LED_RED, API_LED_ORANGE, API_LED_OFF)
 * @return true on success, false if module not initialized or invalid mode
 * @note API_LED_OFF mode will turn off LEDs and set state to IDLE
 * @note This function overrides any currently running LED pattern
 */
bool Api_Led_SetSolid(Api_Led_mode_g mode)
{
	// Check initialization
	if (!Api_Led_is_initialized_b)
	{
		LOG_API_LED("[LED] SetSolid failed - not initialized\r\n");
		return false;
	}

	// Validate parameters
	if (mode >= API_LED_MODE_COUNT)
	{
		LOG_API_LED("[LED] SetSolid failed - invalid mode: %d\r\n", mode);
		return false;
	}

	// Set solid mode
	Api_Led_current_mode_g = mode;
	Api_Led_current_state_g = (mode == API_LED_OFF) ? API_LED_STATE_IDLE : API_LED_STATE_SOLID;
	Api_Led_current_repeat_ui32 = 0;
	Api_Led_state_timer_ui32 = 0;
	
	// Set LED hardware
	Api_Led_set_led_mode(mode);

	LOG_API_LED("[LED] SetSolid mode: %d, state: %d\r\n", mode, Api_Led_current_state_g);
	return true;
}

/**
 * @brief Start LED blinking pattern with custom timing
 * @details Starts LED blinking with specified color, timing, and repeat count.
 *          Uses 1ms timer interrupt for precise timing control.
 *          Immediately stops any ongoing LED operation and starts new pattern.
 * @param[in] mode LED color mode (API_LED_GREEN, API_LED_RED, API_LED_ORANGE)
 * @param[in] on_time_ms Time LED stays on during each blink cycle (milliseconds)
 * @param[in] off_time_ms Time LED stays off during each blink cycle (milliseconds)
 * @param[in] repeat_count Number of blink cycles (use API_LED_REPEAT_INFINITE for infinite)
 * @return true on success, false if module not initialized, invalid mode, or API_LED_OFF
 * @note API_LED_OFF mode is not valid for blinking - use Api_Led_TurnOff() instead
 * @note Uses default timing if on_time_ms or off_time_ms is 0
 * @note Pattern starts immediately with LED ON state
 */
bool Api_Led_StartBlink(Api_Led_mode_g mode, uint32_t on_time_ms, uint32_t off_time_ms, uint32_t repeat_count)
{
	// Check initialization
	if (!Api_Led_is_initialized_b)
	{
		LOG_API_LED("[LED] StartBlink failed - not initialized\r\n");
		return false;
	}

	// Validate parameters
	if (mode >= API_LED_MODE_COUNT || mode == API_LED_OFF)
	{
		LOG_API_LED("[LED] StartBlink failed - invalid mode: %d\r\n", mode);
		return false;
	}

	// Store parameters
	Api_Led_current_mode_g = mode;
	Api_Led_on_time_setting_ui32 = (on_time_ms > 0) ? on_time_ms : API_LED_DEFAULT_ON_TIME_MS;
	Api_Led_off_time_setting_ui32 = (off_time_ms > 0) ? off_time_ms : API_LED_DEFAULT_OFF_TIME_MS;
	Api_Led_repeat_count_ui32 = repeat_count;
	Api_Led_current_repeat_ui32 = 0;

	// Start blinking - begin with LED on
	Api_Led_current_state_g = API_LED_STATE_BLINK_ON;
	Api_Led_state_timer_ui32 = Api_Led_on_time_setting_ui32;
	Api_Led_set_led_mode(mode);

	LOG_API_LED("[LED] StartBlink mode: %d, on: %dms, off: %dms, repeat: %d\r\n", 
			mode, Api_Led_on_time_setting_ui32, Api_Led_off_time_setting_ui32, repeat_count);
	return true;
}

/**
 * @brief Start LED blinking with default timing
 * @details Convenience function that starts blinking with default timing values.
 *          Uses API_LED_DEFAULT_ON_TIME_MS and API_LED_DEFAULT_OFF_TIME_MS (500ms each).
 * @param[in] mode LED color mode (API_LED_GREEN, API_LED_RED, API_LED_ORANGE)
 * @param[in] repeat_count Number of blink cycles (use API_LED_REPEAT_INFINITE for infinite)
 * @return true on success, false if module not initialized or invalid mode
 * @note This is a wrapper function for Api_Led_StartBlink() with default timing
 */
bool Api_Led_StartBlinkDefault(Api_Led_mode_g mode, uint32_t repeat_count)
{
	return Api_Led_StartBlink(mode, API_LED_DEFAULT_ON_TIME_MS, API_LED_DEFAULT_OFF_TIME_MS, repeat_count);
}

/**
 * @brief Turn off all LEDs and stop any ongoing operation
 * @details Immediately stops current LED operation and turns off all LEDs.
 *          Resets state machine to IDLE and clears all timers and counters.
 * @return None
 * @note This function can be called at any time to stop LED operation
 * @note Does nothing if module is not initialized
 */
void Api_Led_TurnOff(void)
{
	if (!Api_Led_is_initialized_b)
	{
		return;
	}

	LOG_API_LED("[LED] TurnOff called\r\n");
	Api_Led_reset_state_machine();
}

/**
 * @brief Timer tick function for LED state machine processing
 * @details This function must be called every 1ms from timer interrupt.
 *          Processes the LED state machine for blinking patterns by decrementing
 *          the state timer and handling state transitions.
 *          
 *          State machine flow:
 *          - BLINK_ON: Decrements timer, switches to BLINK_OFF when expired
 *          - BLINK_OFF: Decrements timer, switches to BLINK_ON or finishes pattern
 *          - SOLID/IDLE: No processing needed
 *          
 * @return None
 * @note This function should be called exactly every 1ms for accurate timing
 * @note Function returns early if module is not initialized or in IDLE/SOLID state
 */
void Api_Led_Timer_tick(void)
{
	if (Api_Led_state_timer_ui32 > 0)
	{
		Api_Led_state_timer_ui32--;
	}
}

void Api_Led_Main(void)
{
	if (!Api_Led_is_initialized_b)
	{
		return;
	}

	// Skip processing if idle or solid
	if (Api_Led_current_state_g == API_LED_STATE_IDLE || Api_Led_current_state_g == API_LED_STATE_SOLID)
	{
		return;
	}

	// Process state machine for blinking - called every 1ms
	switch (Api_Led_current_state_g)
	{
		case API_LED_STATE_BLINK_ON:			
			// Check if on time has elapsed
			if (Api_Led_state_timer_ui32 == 0)
			{
				// Turn off LED and switch to off state
				Api_Led_set_led_mode(API_LED_OFF);
				Api_Led_current_state_g = API_LED_STATE_BLINK_OFF;
				Api_Led_state_timer_ui32 = Api_Led_off_time_setting_ui32;
				LOG_API_LED("[LED] Blink ON->OFF, timer: %dms\r\n", Api_Led_off_time_setting_ui32);
			}
			break;

		case API_LED_STATE_BLINK_OFF:			
			// Check if off time has elapsed
			if (Api_Led_state_timer_ui32 == 0)
			{
				// Increment repeat counter
				Api_Led_current_repeat_ui32++;

				// Check if we've completed all repeats
				if (Api_Led_repeat_count_ui32 != API_LED_REPEAT_INFINITE && 
					Api_Led_current_repeat_ui32 >= Api_Led_repeat_count_ui32)
				{
					// Pattern finished
					Api_Led_current_state_g = API_LED_STATE_FINISHED;
					Api_Led_set_led_mode(API_LED_OFF);
					LOG_API_LED("[LED] Blink pattern finished, count: %d\r\n", Api_Led_current_repeat_ui32);
				}
				else
				{
					// Turn on LED and switch to on state
					Api_Led_set_led_mode(Api_Led_current_mode_g);
					Api_Led_current_state_g = API_LED_STATE_BLINK_ON;
					Api_Led_state_timer_ui32 = Api_Led_on_time_setting_ui32;
					LOG_API_LED("[LED] Blink OFF->ON, count: %d, timer: %dms\r\n", Api_Led_current_repeat_ui32, Api_Led_on_time_setting_ui32);
				}
			}
			break;

		case API_LED_STATE_FINISHED:
			// Pattern complete, reset to idle
			LOG_API_LED("[LED] Pattern finished, reset to idle\r\n");
			Api_Led_reset_state_machine();
			break;

		default:
			// Invalid state, reset
			LOG_API_LED("[LED] Invalid state: %d, reset to idle\r\n", Api_Led_current_state_g);
			Api_Led_reset_state_machine();
			break;
	}
}

/**
 * @brief Check if LED is currently active (displaying any pattern)
 * @details Returns true if LED is currently displaying any pattern or solid color.
 *          Returns false only when LED is in IDLE state (completely off).
 * @return true if LED is active (SOLID, BLINK_ON, BLINK_OFF states), false if idle
 * @note Returns false if module is not initialized
 */
bool Api_Led_IsActive(void)
{
	return (Api_Led_is_initialized_b && Api_Led_current_state_g != API_LED_STATE_IDLE);
}

/**
 * @brief Get current LED state machine state
 * @details Returns the current state of the LED state machine.
 *          Useful for debugging and monitoring LED operation.
 * @return Current LED state (IDLE, SOLID, BLINK_ON, BLINK_OFF, FINISHED)
 * @note State values defined in Api_Led_state_g enumeration
 */
Api_Led_state_g Api_Led_GetState(void)
{
	return Api_Led_current_state_g;
}

/**
 * @brief Get current LED color mode
 * @details Returns the current LED color mode being displayed.
 *          Shows which color is currently active or was last set.
 * @return Current LED mode (OFF, GREEN, RED, ORANGE)
 * @note Mode values defined in Api_Led_mode_g enumeration
 */
Api_Led_mode_g Api_Led_GetCurrentMode(void)
{
	return Api_Led_current_mode_g;
}

/**
 * @brief Get remaining blink cycles count
 * @details Returns how many blink cycles are remaining in current pattern.
 *          Useful for monitoring progress of finite blink patterns.
 * @return Remaining blink count, or API_LED_REPEAT_INFINITE for infinite patterns
 * @note Returns 0 if pattern is complete or not in blink mode
 * @note Count decrements after each complete ON-OFF cycle
 */
uint32_t Api_Led_GetRemainingCount(void)
{
	if (Api_Led_repeat_count_ui32 == API_LED_REPEAT_INFINITE)
	{
		return API_LED_REPEAT_INFINITE;
	}
	
	if (Api_Led_current_repeat_ui32 >= Api_Led_repeat_count_ui32)
	{
		return 0;
	}
	
	return (Api_Led_repeat_count_ui32 - Api_Led_current_repeat_ui32);
}

/**
 * @brief Get actual hardware state of LED pins
 * @details Reads the actual GPIO pin states to determine LED hardware status.
 *          Accounts for active-low logic (pin LOW = LED ON).
 *          Useful for debugging hardware issues or verifying LED state.
 * @param[out] green_on Pointer to store green LED state (true=ON, false=OFF), can be NULL
 * @param[out] red_on Pointer to store red LED state (true=ON, false=OFF), can be NULL  
 * @return None
 * @note Uses active-low logic: reads pin state and inverts it
 * @note Safe to pass NULL pointers for unused outputs
 */
void Api_Led_GetHardwareState(bool *green_on, bool *red_on)
{
	if (green_on != NULL)
	{
		// Active low logic - pin low means LED on
		*green_on = !nrf_gpio_pin_read(P_LED_GREEN);
	}
	
	if (red_on != NULL)
	{
		// Active low logic - pin low means LED on
		*red_on = !nrf_gpio_pin_read(P_LED_RED);
	}
}

/**
 * @brief Get comprehensive LED operation information
 * @details Returns all LED status information in a single function call.
 *          Convenient way to get complete LED state for monitoring or debugging.
 * @param[out] mode Pointer to store current LED mode, can be NULL
 * @param[out] state Pointer to store current state, can be NULL
 * @param[out] remaining_count Pointer to store remaining blink count, can be NULL
 * @param[out] is_blinking Pointer to store blinking status, can be NULL
 * @return true if LED is active, false if idle
 * @note All output parameters are optional (can be NULL)
 * @note is_blinking is true when in BLINK_ON or BLINK_OFF states
 */
bool Api_Led_GetInfo(Api_Led_mode_g *mode, Api_Led_state_g *state, 
                     uint32_t *remaining_count, bool *is_blinking)
{
	if (mode != NULL)
	{
		*mode = Api_Led_current_mode_g;
	}
	
	if (state != NULL)
	{
		*state = Api_Led_current_state_g;
	}
	
	if (remaining_count != NULL)
	{
		*remaining_count = Api_Led_GetRemainingCount();
	}
	
	if (is_blinking != NULL)
	{
		*is_blinking = (Api_Led_current_state_g == API_LED_STATE_BLINK_ON || 
		                Api_Led_current_state_g == API_LED_STATE_BLINK_OFF);
	}
	
	return Api_Led_IsActive();
}

/**
 * @brief Start fast blinking pattern
 * @details Convenience function for fast blinking with predefined timing.
 *          Uses API_LED_FAST_BLINK_ON_MS and API_LED_FAST_BLINK_OFF_MS (100ms each).
 * @param[in] mode LED color mode (API_LED_GREEN, API_LED_RED, API_LED_ORANGE)
 * @param[in] repeat_count Number of blink cycles (use API_LED_REPEAT_INFINITE for infinite)
 * @return true on success, false if module not initialized or invalid mode
 * @note This is a wrapper function for Api_Led_StartBlink() with fast timing
 */
bool Api_Led_StartFastBlink(Api_Led_mode_g mode, uint32_t repeat_count)
{
	return Api_Led_StartBlink(mode, API_LED_FAST_BLINK_ON_MS, API_LED_FAST_BLINK_OFF_MS, repeat_count);
}

/**
 * @brief Start slow blinking pattern
 * @details Convenience function for slow blinking with predefined timing.
 *          Uses API_LED_SLOW_BLINK_ON_MS and API_LED_SLOW_BLINK_OFF_MS (1000ms each).
 * @param[in] mode LED color mode (API_LED_GREEN, API_LED_RED, API_LED_ORANGE)
 * @param[in] repeat_count Number of blink cycles (use API_LED_REPEAT_INFINITE for infinite)
 * @return true on success, false if module not initialized or invalid mode
 * @note This is a wrapper function for Api_Led_StartBlink() with slow timing
 */
bool Api_Led_StartSlowBlink(Api_Led_mode_g mode, uint32_t repeat_count)
{
	return Api_Led_StartBlink(mode, API_LED_SLOW_BLINK_ON_MS, API_LED_SLOW_BLINK_OFF_MS, repeat_count);
}

/**
 * @brief Force direct hardware control of LEDs
 * @details Directly controls LED hardware bypassing the state machine.
 *          Use with caution - for debugging or emergency signaling only.
 *          Does not update internal state variables.
 * @param[in] green_on true to turn on green LED, false to turn off
 * @param[in] red_on true to turn on red LED, false to turn off
 * @return None
 * @warning This function bypasses the state machine and may cause inconsistency
 * @note Recommended for debugging and emergency use only
 * @note Does not check if module is initialized
 */
void Api_Led_ForceHardware(bool green_on, bool red_on)
{
	Api_Led_set_green_led(green_on);
	Api_Led_set_red_led(red_on);
}

/** @} */
