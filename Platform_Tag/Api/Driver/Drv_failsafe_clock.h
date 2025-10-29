#ifndef DRV_FAILSAFE_CLOCK_H
#define DRV_FAILSAFE_CLOCK_H

#include <stdint.h>
#include <stdbool.h>
#include "Api_failsafe.h"

/**
 * @brief Check HFCLK startup with timeout
 * @return true if HFCLK started successfully, false if timeout
 */
bool Drv_failsafe_clock_check_hfclk_start(void);

/**
 * @brief Check LFCLK startup with timeout
 * @return true if LFCLK started successfully, false if timeout
 */
bool Drv_failsafe_clock_check_lfclk_start(void);

/**
 * @brief Recover system clocks (HFCLK + LFCLK)
 * @param code Failsafe code (FAILSAFE_CODE_16)
 * @param retry_count Current retry attempt count
 */
void Drv_failsafe_clock_recover(FAILSAFE_CODE code, uint8_t retry_count);

#endif // DRV_FAILSAFE_CLOCK_H
