/**
 * @file Drv_failsafe_uwb.c
 * @brief UWB-specific FailSafe driver implementation
 * 
 * @details
 * This module implements recovery strategies for UWB failures.
 * Driver simply triggers recovery operations without checking results.
 * Success/failure is determined by Api_uwb calling Api_failsafe_setSuccess().
 */

#include "Drv_failsafe_uwb.h"
#include "Api_uwb.h"
#include "Func_UART_LOG.h"

// ========================================================================================
// Internal Functions
// ========================================================================================

/**
 * @brief Get tag name string
 */
static const char* get_fail_name(FAILSAFE_CODE code)
{
	switch (code)
	{
		case FAILSAFE_CODE_02: return "UWB_INIT";
		case FAILSAFE_CODE_03: return "UWB_TX";
		case FAILSAFE_CODE_04: return "UWB_WAKEUP";
		default: return "UNKNOWN";
	}
}

/**
 * @brief Start UWB Init recovery
 */
static void recovery_start_init(void)
{
	LOG_API_FAIL("[FAIL_DRV] Starting UWB init...\r\n");
	
	Api_uwb_reset_init();
	Api_uwb_start_init();
}

/**
 * @brief Start UWB TX recovery
 */
static void recovery_start_tx(void)
{
	LOG_API_FAIL("[FAIL_DRV] Starting TX...\r\n");
	Api_uwb_start_tx(false, false);
}

/**
 * @brief Start UWB Wakeup recovery
 */
static void recovery_start_wakeup(void)
{
	LOG_API_FAIL("[FAIL_DRV] Starting wakeup...\r\n");
	Api_uwb_start_wakeup(false);
}

// ========================================================================================
// Public Functions
// ========================================================================================

void Drv_failsafe_uwb_init(void)
{
	LOG_API_FAIL("[FAIL_DRV] UWB FailSafe driver ready\r\n");
}

void Drv_failsafe_uwb_timer_tick(void)
{
	// No timer needed
}

void Drv_failsafe_uwb_main(void)
{
	// No processing needed
}

bool Drv_failsafe_uwb_recover(FAILSAFE_CODE code, uint8_t retry_count)
{
	LOG_API_FAIL("[FAIL_DRV] %s: Recovery attempt %d/%d\r\n", 
				get_fail_name(code), retry_count, FAILSAFE_MAX_RETRY_COUNT);
	
	switch (code)
	{
		case FAILSAFE_CODE_02:
			recovery_start_init();
			break;
		case FAILSAFE_CODE_03:
			recovery_start_tx();
			break;
		case FAILSAFE_CODE_04:
			recovery_start_wakeup();
			break;
		default:
			LOG_API_FAIL("[FAIL_DRV] Unknown tag %d\r\n", code);
			return false;
	}
	
	return false;
}

