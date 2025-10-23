/**
 * @file Func_TIMER.h
 * @brief RTC2-based timer functionality for the nRF52840 platform
 * @details This module provides simple timer functions based on RTC2 peripheral,
 *          including initialization, start/stop control, and delay functions.
 * @author Platform Tag Team
 * @date 2024
 * @version 1.0
 */

#ifndef FUNC_TIMER_KHS_H
#define FUNC_TIMER_KHS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup TIMER_External_Variables External Variables
 * @{
 */
extern volatile bool s_sleep_state;  /**< Global sleep state flag for power management */
/** @} */

/**
 * @defgroup TIMER_Overview Module Overview
 * @{
 * This module provides RTC2-based simple timer functions for the nRF52840 platform.
 * It includes:
 * - Timer initialization and configuration
 * - Start/stop control with millisecond precision
 * - Simple delay functions for both milliseconds and microseconds
 * - Sleep state management for power efficiency
 * @}
 */

/**
 * @defgroup TIMER_Functions Timer Function Declarations
 * @{
 */

/**
 * @brief Initialize RTC2 timer
 * @details Configures RTC2 peripheral with default settings and enables interrupts.
 *          Sets up compare channel 0 for timer functionality.
 * @return true on success, false on failure
 * @note This function must be called before using any timer functions
 */
bool Func_TIMER_Init(void);

/**
 * @brief Start timer with specified interval
 * @param[in] interval_ms Timer interval in milliseconds
 * @return true on success, false on failure
 * @note Timer will automatically stop when interval expires
 */
bool Func_TIMER_Start(uint32_t interval_ms);

/**
 * @brief Stop RTC2 timer
 * @details Disables compare channel and stops timer operation.
 * @return None
 * @note Timer can be restarted with Func_TIMER_Start()
 */
void Func_TIMER_Stop(void);

/**
 * @brief Get current timer counter value
 * @details Returns the current value of the RTC2 counter.
 * @return Current timer counter value (24-bit)
 */
uint32_t Func_TIMER_GetCount(void);

/**
 * @brief Simple delay function in milliseconds
 * @details Implements a busy-wait delay using a simple loop.
 *          Not precise but useful for short delays.
 * @param[in] ms Delay time in milliseconds
 * @return None
 * @note This is a blocking function - CPU will be busy during delay
 */
void Func_TIMER_Delay(uint32_t ms);

/**
 * @brief Simple delay function in microseconds
 * @details Implements a busy-wait delay using a simple loop.
 *          Not precise but useful for very short delays.
 * @param[in] us Delay time in microseconds
 * @return None
 * @note This is a blocking function - CPU will be busy during delay
 */
void Func_TIMER_DelayUs(uint32_t us);

/** @} */

#ifdef __cplusplus
}
#endif

#endif // FUNC_TIMER_KHS_H

