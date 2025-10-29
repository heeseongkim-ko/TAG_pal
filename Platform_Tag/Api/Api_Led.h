
#ifndef API_LED_H
#define API_LED_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
	API_LED_OFF = 0,		/**< All LEDs off (black) */
	API_LED_GREEN,			/**< Green LED only (battery >50% or power supplied) */
	API_LED_RED,			/**< Red LED only (battery 10-30% or charging) */
	API_LED_ORANGE,			/**< Orange/Yellow (both LEDs) for battery 30-50% */
	API_LED_MODE_COUNT		/**< Total number of LED modes */
} Api_Led_mode_g;

typedef enum
{
	API_LED_STATE_IDLE = 0,		/**< LED controller idle */
	API_LED_STATE_SOLID,		/**< LED is solid on */
	API_LED_STATE_BLINK_ON,		/**< LED is currently on (during blink cycle) */
	API_LED_STATE_BLINK_OFF,	/**< LED is currently off (during blink cycle) */
	API_LED_STATE_FINISHED		/**< Blink pattern finished */
} Api_Led_state_g;

#define API_LED_DEFAULT_ON_TIME_MS			500		/**< Default LED on time (ms) */
#define API_LED_DEFAULT_OFF_TIME_MS			500		/**< Default LED off time (ms) */
#define API_LED_BATTERY_BLINK_ON_MS			200		/**< Battery blink on time (ms) */
#define API_LED_BATTERY_BLINK_OFF_MS		800		/**< Battery blink off time (ms) */
#define API_LED_FAST_BLINK_ON_MS			100		/**< Fast blink on time (ms) */
#define API_LED_FAST_BLINK_OFF_MS			100		/**< Fast blink off time (ms) */
#define API_LED_SLOW_BLINK_ON_MS			1000	/**< Slow blink on time (ms) */
#define API_LED_SLOW_BLINK_OFF_MS			1000	/**< Slow blink off time (ms) */

#define API_LED_REPEAT_INFINITE				0xFFFFFFFF

bool Api_Led_Init(void);

bool Api_Led_StartBlink(Api_Led_mode_g mode, uint32_t on_time_ms, uint32_t off_time_ms, uint32_t repeat_count);

bool Api_Led_StartBlinkDefault(Api_Led_mode_g mode, uint32_t repeat_count);

void Api_Led_TurnOff(void);

void Api_Led_Main(void);

bool Api_Led_IsActive(void);

Api_Led_state_g Api_Led_GetState(void);

Api_Led_mode_g Api_Led_GetCurrentMode(void);

void Api_Led_Timer_tick(void);

#ifdef __cplusplus
}
#endif

#endif /* API_LED_H */

