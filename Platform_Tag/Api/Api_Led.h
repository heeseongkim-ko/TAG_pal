/**
 * @file Api_Led.h
 * @brief Non-blocking LED control functionality for TEIA platform
 * @details This module provides simple LED control including solid colors and
 *          blinking patterns for battery level indication according to TEIA specifications.
 *          All operations are non-blocking and use timer-based state machine.
 * @author Platform Tag Team
 * @date 2024
 * @version 1.0
 */

#ifndef API_LED_H
#define API_LED_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup LED_Types LED Type Definitions
 * @{
 */

/**
 * @brief LED color modes according to TEIA specification
 */
typedef enum
{
	API_LED_OFF = 0,		/**< All LEDs off (black) */
	API_LED_GREEN,			/**< Green LED only (battery >50% or power supplied) */
	API_LED_RED,			/**< Red LED only (battery 10-30% or charging) */
	API_LED_ORANGE,			/**< Orange/Yellow (both LEDs) for battery 30-50% */
	API_LED_MODE_COUNT		/**< Total number of LED modes */
} Api_Led_mode_g;

/**
 * @brief LED state machine states
 */
typedef enum
{
	API_LED_STATE_IDLE = 0,		/**< LED controller idle */
	API_LED_STATE_SOLID,		/**< LED is solid on */
	API_LED_STATE_BLINK_ON,		/**< LED is currently on (during blink cycle) */
	API_LED_STATE_BLINK_OFF,	/**< LED is currently off (during blink cycle) */
	API_LED_STATE_FINISHED		/**< Blink pattern finished */
} Api_Led_state_g;

/** @} */

/**
 * @defgroup LED_Constants LED Timing Constants
 * @{
 */

/** Default timing values in milliseconds */
#define API_LED_DEFAULT_ON_TIME_MS			500		/**< Default LED on time (ms) */
#define API_LED_DEFAULT_OFF_TIME_MS			500		/**< Default LED off time (ms) */
#define API_LED_BATTERY_BLINK_ON_MS			200		/**< Battery blink on time (ms) */
#define API_LED_BATTERY_BLINK_OFF_MS		800		/**< Battery blink off time (ms) */
#define API_LED_FAST_BLINK_ON_MS			100		/**< Fast blink on time (ms) */
#define API_LED_FAST_BLINK_OFF_MS			100		/**< Fast blink off time (ms) */
#define API_LED_SLOW_BLINK_ON_MS			1000	/**< Slow blink on time (ms) */
#define API_LED_SLOW_BLINK_OFF_MS			1000	/**< Slow blink off time (ms) */

/** Infinite repeat count */
#define API_LED_REPEAT_INFINITE				0xFFFFFFFF

/** @} */

/**
 * @defgroup LED_Functions LED Function Declarations
 * @{
 */

/**
 * @brief Initialize LED control module
 * @details Configures GPIO pins for LED control and initializes internal state.
 *          Must be called before using any LED functions.
 * @return true on success, false on failure
 */
bool Api_Led_Init(void);

/**
 * @brief Set LED to solid color (always on)
 * @details Sets LED to specified color and keeps it on continuously.
 * @param[in] mode LED color mode (green, red, orange, off)
 * @return true on success, false on failure
 */
bool Api_Led_SetSolid(Api_Led_mode_g mode);

/**
 * @brief Start LED blinking with custom timing
 * @details Starts LED blinking with specified timing and repeat count.
 *          This is a non-blocking function that uses timer-based state machine.
 * @param[in] mode LED color mode (green, red, orange)
 * @param[in] on_time_ms Time LED stays on
 * @param[in] off_time_ms Time LED stays off
 * @param[in] repeat_count Number of blink cycles (use API_LED_REPEAT_INFINITE for infinite)
 * @return true on success, false on failure
 */
bool Api_Led_StartBlink(Api_Led_mode_g mode, uint32_t on_time_ms, uint32_t off_time_ms, uint32_t repeat_count);

/**
 * @brief Start LED blinking with default timing
 * @details Convenience function that uses default timing values (500ms on/off).
 * @param[in] mode LED color mode
 * @param[in] repeat_count Number of blink cycles
 * @return true on success, false on failure
 */
bool Api_Led_StartBlinkDefault(Api_Led_mode_g mode, uint32_t repeat_count);

/**
 * @brief Turn off all LEDs
 * @details Immediately stops current LED operation and turns off all LEDs.
 * @return None
 */
void Api_Led_TurnOff(void);

/**
 * @brief Timer tick function for LED processing
 * @details Must be called every 1ms from timer interrupt to process LED state machine.
 *          This function handles timing counters and LED state transitions.
 * @return None
 * @note This function should be called every 1ms from main timer interrupt
 */
void Api_Led_Main(void);

/**
 * @brief Check if LED display is active
 * @details Returns true if LED is currently displaying any pattern.
 * @return true if LED is active, false if idle
 */
bool Api_Led_IsActive(void);

/**
 * @brief Get current LED state
 * @details Returns current state of LED state machine.
 * @return Current LED state
 */
Api_Led_state_g Api_Led_GetState(void);

/**
 * @brief Get current LED mode (color)
 * @details Returns the current LED color mode being displayed.
 * @return Current LED mode
 */
Api_Led_mode_g Api_Led_GetCurrentMode(void);

/**
 * @brief Get remaining blink count
 * @details Returns how many blink cycles are remaining.
 * @return Remaining blink count (0xFFFFFFFF for infinite)
 */
uint32_t Api_Led_GetRemainingCount(void);

/**
 * @brief Get current hardware LED state
 * @details Returns the actual current state of LED hardware.
 * @param[out] green_on True if green LED is currently on
 * @param[out] red_on True if red LED is currently on
 * @return None
 */
void Api_Led_GetHardwareState(bool *green_on, bool *red_on);

/**
 * @brief Get LED operation info
 * @details Returns comprehensive information about current LED operation.
 * @param[out] mode Current LED mode
 * @param[out] state Current state
 * @param[out] remaining_count Remaining blink cycles
 * @param[out] is_blinking True if currently in blink mode
 * @return true if LED is active, false if idle
 */
bool Api_Led_GetInfo(Api_Led_mode_g *mode, Api_Led_state_g *state, 
                     uint32_t *remaining_count, bool *is_blinking);

/**
 * @brief Start fast blinking
 * @details Convenience function for fast blinking pattern (100ms on/off).
 * @param[in] mode LED color mode
 * @param[in] repeat_count Number of blink cycles
 * @return true on success, false on failure
 */
bool Api_Led_StartFastBlink(Api_Led_mode_g mode, uint32_t repeat_count);

/**
 * @brief Start slow blinking
 * @details Convenience function for slow blinking pattern (1000ms on/off).
 * @param[in] mode LED color mode
 * @param[in] repeat_count Number of blink cycles
 * @return true on success, false on failure
 */
bool Api_Led_StartSlowBlink(Api_Led_mode_g mode, uint32_t repeat_count);

/**
 * @brief Force LED hardware state
 * @details Directly controls LED hardware without affecting state machine.
 *          Use with caution - for debugging or emergency signaling only.
 * @param[in] green_on True to turn on green LED
 * @param[in] red_on True to turn on red LED
 * @return None
 */
void Api_Led_ForceHardware(bool green_on, bool red_on);

void Api_Led_Timer_tick(void);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* API_LED_H */

