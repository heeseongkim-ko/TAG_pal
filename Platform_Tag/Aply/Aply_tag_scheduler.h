/**
 * @file Aply_tag_scheduler.h
 * @brief Tag scheduler module for TEIA platform
 * @author Platform Tag Team
 * @date 2025
 * @version 1.0
 */

#ifndef APLY_TAG_SCHEDULER_H
#define APLY_TAG_SCHEDULER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Process cycle completion and update packet scheduling
 * @details Handles packet counter decrements and flag setting after each UWB cycle
 */
void Aply_tag_scheduler_normal_process_cycle_complete(void);

/**
 * @brief Initialize the tag scheduler
 * @details Sets up initial counter values and scheduling state
 */
void Aply_tag_scheduler_init(void);

/**
 * @brief Get LED flag status
 * @details Returns current LED flag status without modifying it
 * @return true if LED action is needed, false otherwise
 */
bool Aply_tag_scheduler_get_led_flag(void);

/**
 * @brief Clear LED flag
 * @details Resets the LED flag to false
 * @return None
 */
void Aply_tag_scheduler_clear_led_flag(void);

/**
 * @brief Get battery check flag status
 * @details Returns current battery check flag status without modifying it
 * @return true if battery check is needed, false otherwise
 */
bool Aply_tag_scheduler_get_battery_flag(void);

/**
 * @brief Clear battery check flag
 * @details Resets the battery check flag to false
 * @return None
 */
void Aply_tag_scheduler_clear_battery_flag(void);

/**
 * @brief Get motion sleep flag status
 * @details Returns current motion sleep flag status without modifying it
 * @return true if motion sleep should be activated, false otherwise
 */
bool Aply_tag_scheduler_get_motion_sleep_flag(void);

/**
 * @brief Clear motion sleep flag
 * @details Resets the motion sleep flag to false
 * @return None
 */
void Aply_tag_scheduler_clear_motion_sleep_flag(void);

/**
 * @brief Reset motion sleep counter
 * @details Resets the motion sleep counter to initial value for motion detection restart
 * @return None
 */
void Aply_tag_scheduler_reset_motion_sleep_counter(void);

/**
 * @brief Set LED counter to 1 for backchannel LED control
 * @details Sets the LED counter to 1 to trigger immediate LED blink on next cycle
 * @return None
 */
void Aply_tag_scheduler_set_led_counter_backchannel(void);

#ifdef __cplusplus
}
#endif

#endif // APLY_TAG_SCHEDULER_H
