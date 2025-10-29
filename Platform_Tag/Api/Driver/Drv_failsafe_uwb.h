/**
 * @file Drv_failsafe_uwb.h
 * @brief UWB-specific FailSafe driver header
 */

#ifndef DRV_FAILSAFE_UWB_H
#define DRV_FAILSAFE_UWB_H

#include <stdbool.h>
#include <stdint.h>
#include "Api_failsafe.h"

/**
 * @brief Initialize UWB FailSafe driver
 */
void Drv_failsafe_uwb_init(void);

/**
 * @brief Timer tick handler for UWB FailSafe driver
 */
void Drv_failsafe_uwb_timer_tick(void);

/**
 * @brief Main processing function for UWB FailSafe driver
 */
void Drv_failsafe_uwb_main(void);

/**
 * @brief Attempt recovery for UWB-related failure
 * @param tag Fail tag
 * @param retry_count Current retry count
 * @return Always returns false (success is determined by Api_failsafe_setSuccess())
 */
bool Drv_failsafe_uwb_recover(FAILSAFE_CODE tag, uint8_t retry_count);

#endif // DRV_FAILSAFE_UWB_H

