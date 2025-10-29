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
	LOG_API_FAIL("%s\r\n", __func__);
	
	Api_uwb_reset_init();
	Api_uwb_start_init();
}

/**
 * @brief Start UWB TX recovery
 */
static void recovery_start_tx(void)
{
	LOG_API_FAIL("%s\r\n", __func__);
	Api_uwb_start_tx(false, false);
}

/**
 * @brief Start UWB Wakeup recovery
 */
static void recovery_start_wakeup(void)
{
	LOG_API_FAIL("%s\r\n", __func__);
	Api_uwb_start_wakeup(false);
}

// ========================================================================================
// Public Functions
// ========================================================================================

bool Drv_failsafe_uwb_recover(FAILSAFE_CODE code, failsafe_information_t *info)
{	
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
			return false;
	}
	
	return false;
}

