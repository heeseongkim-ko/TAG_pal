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
 * @brief Attempt recovery for UWB-related failure
 * @param tag Fail tag
 * @param retry_count Current retry count
 * @return Always returns false (success is determined by Api_failsafe_setSuccess())
 */
bool Drv_failsafe_uwb_recover(FAILSAFE_CODE tag, failsafe_information_t *info);

#endif // DRV_FAILSAFE_UWB_H

