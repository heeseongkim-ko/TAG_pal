/**
 * @file Aply_tag_manager.h
 * @brief Simple tag manager module for TEIA platform
 * @author Platform Tag Team
 * @date 2025
 * @version 1.0
 */

#ifndef APLY_TAG_MANAGER_H
#define APLY_TAG_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Tag manager states
 */
typedef enum {
	TAG_MANAGER_STATE_WAIT_UWB_INIT = 0,
	TAG_MANAGER_STATE_NORMAL,
	TAG_MANAGER_STATE_MOTION_SLEEP,
	TAG_MANAGER_STATE_NFC
} tag_manager_state_e;

/**
 * @brief Initialize tag manager
 */
void Aply_tag_manager_init(void);

/**
 * @brief Process tag manager state machine
 */
void Aply_tag_manager_process(void);

/**
 * @brief Get current state
 */
tag_manager_state_e Aply_tag_manager_get_state(void);

#ifdef __cplusplus
}
#endif

#endif // APLY_TAG_MANAGER_H