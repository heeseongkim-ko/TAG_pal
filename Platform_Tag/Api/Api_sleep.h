/**
 * @file Api_sleep.h
 * @brief Sleep management API for Tag Platform
 * @details This module provides sleep functionality and wakeup reason tracking for power management
 * @author Tag Platform Development Team
 * @date 2025
 */

#ifndef API_SLEEP_H
#define API_SLEEP_H

#include <stdbool.h>
#include <stdint.h>
/**
 * @brief Wake-up reason bit masks for sleep mode management
 * 
 * These bit masks are used to identify and configure different wake-up sources
 * for the device sleep functionality. Multiple sources can be combined using
 * bitwise OR operations.
 */

/** @brief No wake-up reason specified */
#define API_SLEEP_WAKEUP_REASON_MASK_NULL       0x00

/** @brief Wake-up triggered by general timer */
#define API_SLEEP_WAKEUP_REASON_MASK_TIMER      0x01

/** @brief Wake-up triggered by system tick timer */
#define API_SLEEP_WAKEUP_REASON_MASK_TICK_TIMER 0x02

/** @brief Wake-up triggered by motion detection (accelerometer) */
#define API_SLEEP_WAKEUP_REASON_MASK_MOTION     0x04

/** @brief Wake-up triggered by NFC field detection */
#define API_SLEEP_WAKEUP_REASON_MASK_NFC        0x08

/** @brief Wake-up triggered by other sources (external interrupts, etc.) */
#define API_SLEEP_WAKEUP_REASON_MASK_OTHER      0x10

/** @brief All wake-up sources enabled (combination of all above masks) */
#define API_SLEEP_WAKEUP_REASON_MASK_ALL        0x1F

/**
 * @brief Request the device to enter sleep mode
 * @details Sets the sleep request flag and resets the wakeup reason to NULL.
 *          The actual sleep entry is handled by the main sleep handler.
 * @note This function only sets the request flag, actual sleep implementation
 *       depends on system state and other module requirements
 */
void Api_sleep_start_sleep(void);

/**
 * @brief Set the wakeup reason for the current/next wakeup event
 * @details Sets the global wakeup reason variable to track what caused
 *          or will cause the device to wake up from sleep mode
 * @param[in] reason The wakeup reason to be set
 * @note This function should be called by interrupt handlers or other
 *       modules that detect wakeup events to properly track the source
 */
void Api_sleep_setWakeupReason(uint8_t reason);

void Api_sleep_claerWakeupReasonMask(uint8_t reason);

/**
 * @brief Set the sleep duration time
 * @details Sets the duration for how long the device should remain in sleep mode.
 *          This time is typically used by timer-based wakeup mechanisms.
 * @param[in] times Sleep duration in system time units (typically milliseconds)
 * @note The actual sleep implementation and time unit depends on the underlying
 *       power management system and timer configuration
 */
void Api_sleep_setSleepTime(uint32_t times);

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
uint8_t Api_sleep_getWakeupReason(void);

void Api_sleep_core(void);

/**
 * @brief Main sleep management routine
 * @details Handles the sleep state machine and processes sleep requests.
 *          This function should be called periodically from the main loop
 *          to check for sleep requests and manage power states.
 * @note This function returns immediately if no sleep request is pending
 */
void Api_sleep_main(void);

/**
 * @brief Initialize the Sleep API module
 * @details Initializes the sleep management system and underlying power management.
 *          This function must be called once during system startup before using
 *          any other sleep API functions.
 * @note This function should be called only once during system initialization
 * @note Must be called after basic system initialization but before main loop
 * @warning Do not call this function multiple times as it may cause system instability
 * 
 * @code
 * // Typical usage in main()
 * int main(void)
 * {
 *     // Basic system init
 *     log_init();
 *     timers_init();
 *     
 *     // Initialize sleep API (call once)
 *     Api_sleep_init();
 *     
 *     // Other module inits...
 *     Api_ble_init();
 *     
 *     // Main loop
 *     for(;;) {
 *         Api_sleep_main();
 *     }
 * }
 * @endcode
 */
void Api_sleep_init(void);

#endif /* API_SLEEP_H */

