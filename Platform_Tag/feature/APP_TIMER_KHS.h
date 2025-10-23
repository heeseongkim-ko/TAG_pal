#ifndef APP_TIMER_KHS_H
#define APP_TIMER_KHS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile bool s_sleep_state;

// ========================================================================================
// RTC2-based simple timer functions
// ========================================================================================

/**
 * @brief Initialize RTC2 timer
 * @return true: success, false: failure
 */
bool APP_TIMER_Init(void);

/**
 * @brief Start timer
 * @param interval_ms timer interval (milliseconds)
 * @return true: success, false: failure
 */
bool APP_TIMER_Start(uint32_t interval_ms);

/**
 * @brief Stop timer
 */
void APP_TIMER_Stop(void);

/**
 * @brief Get timer counter value
 * @return current timer counter value
 */
uint32_t APP_TIMER_GetCount(void);

/**
 * @brief Delay function (milliseconds)
 * @param ms delay time (milliseconds)
 */
void APP_TIMER_Delay(uint32_t ms);

/**
 * @brief Delay function (microseconds)
 * @param us delay time (microseconds)
 */
void APP_TIMER_DelayUs(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif // APP_TIMER_KHS_H
