
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "Api_port.h"
#include "Api_Led.h"
#include "nrf_gpio.h"
#include "Func_UART_LOG.h"

static uint32_t Api_Led_state_timer_ui32 = 0;

static Api_Led_mode_g Api_Led_current_mode_g = API_LED_OFF;

static Api_Led_state_g Api_Led_current_state_g = API_LED_STATE_IDLE;

static uint32_t Api_Led_on_time_setting_ui32 = API_LED_DEFAULT_ON_TIME_MS;

static uint32_t Api_Led_off_time_setting_ui32 = API_LED_DEFAULT_OFF_TIME_MS;

static uint32_t Api_Led_repeat_count_ui32 = 0;

static uint32_t Api_Led_current_repeat_ui32 = 0;

static bool Api_Led_is_initialized_b = false;

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

static void Api_Led_reset_state_machine(void)
{
	Api_Led_current_state_g = API_LED_STATE_IDLE;
	Api_Led_current_repeat_ui32 = 0;
	Api_Led_state_timer_ui32 = 0;
	Api_Led_set_led_mode(API_LED_OFF);
}

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

bool Api_Led_StartBlinkDefault(Api_Led_mode_g mode, uint32_t repeat_count)
{
	return Api_Led_StartBlink(mode, API_LED_DEFAULT_ON_TIME_MS, API_LED_DEFAULT_OFF_TIME_MS, repeat_count);
}

void Api_Led_TurnOff(void)
{
	if (!Api_Led_is_initialized_b)
	{
		return;
	}

	LOG_API_LED("[LED] TurnOff called\r\n");
	Api_Led_reset_state_machine();
}

void Api_Led_Timer_tick(void)
{
	if (Api_Led_state_timer_ui32 > 0)
	{
		Api_Led_state_timer_ui32--;
	}
}

bool Api_Led_IsActive(void)
{
	return (Api_Led_is_initialized_b && Api_Led_current_state_g != API_LED_STATE_IDLE);
}

Api_Led_state_g Api_Led_GetState(void)
{
	return Api_Led_current_state_g;
}

Api_Led_mode_g Api_Led_GetCurrentMode(void)
{
	return Api_Led_current_mode_g;
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

