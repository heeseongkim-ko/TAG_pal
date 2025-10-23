/**
 * @file    Api_failsafe.c
 * @brief   Centralized Failsafe Management System Implementation
 */

#include <string.h>
#include <stdio.h>

#include "sdk_config.h"
#include "app_error.h"

#include "Func_UART_LOG.h"

#include "Api_failsafe.h"
#include "Api_uwb.h"
#include "Api_nfc.h"
#include "Api_sleep.h"

// ========================================================================================
// Configuration
// ========================================================================================

#define RECOVERY_DELAY_MS		500u
#define STABILIZATION_DELAY_MS	1000u

// ========================================================================================
// Module Context
// ========================================================================================

typedef struct
{
	api_failsafe_state_e state;
	uint8_t fail_code;
	uint8_t fail_count;
	uint16_t recovery_timer;
	bool enabled;
} failsafe_module_context_t;

typedef enum
{
	MODULE_UWB = 0,
	MODULE_NFC,
	MODULE_MAX
} module_id_e;

// ========================================================================================
// Static Variables
// ========================================================================================

static failsafe_module_context_t failsafe_modules_g[MODULE_MAX];
static uint8_t failsafe_results_ui8[TAG_SAFE_MAX];

// ========================================================================================
// Helper Functions - String Conversion
// ========================================================================================

static const char* failsafe_get_uwb_fail_name(uint8_t fail_code)
{
	switch (fail_code)
	{
		case UWB_FAIL_NONE: return "OK";
		case UWB_FAIL_INIT_01: return "INIT_Probe";
		case UWB_FAIL_INIT_02: return "INIT_Func";
		case UWB_FAIL_INIT_03: return "INIT_Config";
		case UWB_FAIL_TX_01: return "TX_Timeout";
		case UWB_FAIL_WAKEUP_01: return "WKUP_01";
		case UWB_FAIL_WAKEUP_02: return "WKUP_DevID";
		case UWB_FAIL_WAKEUP_03: return "WKUP_SPIRDY";
		case UWB_FAIL_WAKEUP_04: return "WKUP_04";
		case UWB_FAIL_SPI_01: return "SPI_Timeout";
		case UWB_FAIL_SPI_02: return "SPI_CRC";
		case UWB_FAIL_SPI_03: return "SPI_NoResp";
		default: return "Unknown";
	}
}

// ========================================================================================
// Recovery Functions - UWB
// ========================================================================================

static bool failsafe_uwb_recovery(uint8_t fail_code)
{
	bool success = false;
	
	switch (fail_code)
	{
		case UWB_FAIL_INIT_01:
		case UWB_FAIL_INIT_02:
		case UWB_FAIL_INIT_03:
		case UWB_FAIL_WAKEUP_01:
		case UWB_FAIL_WAKEUP_02:
		case UWB_FAIL_WAKEUP_03:
		case UWB_FAIL_WAKEUP_04:
			success = Api_uwb_start_init();
			break;
			
		case UWB_FAIL_TX_01:
			success = Api_uwb_start_tx(false, false);
			break;
			
		case UWB_FAIL_SPI_01:
		case UWB_FAIL_SPI_02:
		case UWB_FAIL_SPI_03:
			Api_uwb_spi_init(false);
			success = true;
			break;
			
		default:
			break;
	}
	
	return success;
}

static api_failsafe_code_e failsafe_uwb_convert(uint8_t fail_code)
{
	switch (fail_code)
	{
		case UWB_FAIL_INIT_01:
		case UWB_FAIL_INIT_02:
		case UWB_FAIL_INIT_03:
			return TAG_SAFE_02;
			
		case UWB_FAIL_WAKEUP_01:
		case UWB_FAIL_WAKEUP_02:
		case UWB_FAIL_WAKEUP_03:
		case UWB_FAIL_WAKEUP_04:
			return TAG_SAFE_03;
			
		case UWB_FAIL_TX_01:
			return TAG_SAFE_04;
			
		case UWB_FAIL_SPI_01:
		case UWB_FAIL_SPI_02:
		case UWB_FAIL_SPI_03:
			return TAG_SAFE_05;
			
		default:
			return TAG_SAFE_01;
	}
}

static api_failsafe_state_e failsafe_uwb_get_terminal_state(uint8_t fail_code)
{
	switch (fail_code)
	{
		case UWB_FAIL_INIT_01:
		case UWB_FAIL_INIT_02:
		case UWB_FAIL_INIT_03:
		case UWB_FAIL_WAKEUP_01:
		case UWB_FAIL_WAKEUP_02:
		case UWB_FAIL_WAKEUP_03:
		case UWB_FAIL_WAKEUP_04:
		case UWB_FAIL_SPI_01:
		case UWB_FAIL_SPI_02:
		case UWB_FAIL_SPI_03:
			return FAILSAFE_STATE_TERMINATED_SYSTEM;
			
		case UWB_FAIL_TX_01:
		default:
			return FAILSAFE_STATE_IDLE;
	}
}

// ========================================================================================
// Recovery Functions - NFC
// ========================================================================================

static bool failsafe_nfc_recovery(uint8_t fail_code)
{
	return false;
}

static api_failsafe_code_e failsafe_nfc_convert(uint8_t fail_code)
{
	return TAG_SAFE_06;
}

static api_failsafe_state_e failsafe_nfc_get_terminal_state(uint8_t fail_code)
{
	return FAILSAFE_STATE_IDLE;
}

// ========================================================================================
// Helper Functions
// ========================================================================================

static failsafe_module_context_t* failsafe_get_module(module_id_e module_id)
{
	if (module_id >= MODULE_MAX)
	{
		return NULL;
	}
	
	return &failsafe_modules_g[module_id];
}

static bool failsafe_write_result(api_failsafe_code_e code, api_failsafe_result_e result)
{
	if (code >= TAG_SAFE_MAX)
	{
		return false;
	}
	
	failsafe_results_ui8[code] = (uint8_t)result;
	return Api_nfc_write_data(FAILSAFE_ADDRESS, failsafe_results_ui8, TAG_SAFE_MAX);
}

// ========================================================================================
// State Machine
// ========================================================================================

static void failsafe_process_module(module_id_e module_id)
{
	failsafe_module_context_t* ctx = failsafe_get_module(module_id);
	if (ctx == NULL || !ctx->enabled)
	{
		return;
	}
	
	switch (ctx->state)
	{
		case FAILSAFE_STATE_IDLE:
			if (ctx->fail_code != 0u)
			{
				LOG_API_FAIL("FailSafe Fail: %s\r\n", failsafe_get_uwb_fail_name(ctx->fail_code));
				ctx->state = FAILSAFE_STATE_TRIGGERED;
			}
			break;
			
		case FAILSAFE_STATE_TRIGGERED:
			if (ctx->recovery_timer > 0u)
			{
				return;
			}
			
			LOG_API_FAIL("FailSafe Try %d/%d\r\n", ctx->fail_count + 1, FAILSAFE_LIMIT_COUNT);
			ctx->state = FAILSAFE_STATE_RECOVERY;
			break;
			
		case FAILSAFE_STATE_RECOVERY:
		{
			bool recovery_started = false;
			
			if (module_id == MODULE_UWB)
			{
				recovery_started = failsafe_uwb_recovery(ctx->fail_code);
			}
			else if (module_id == MODULE_NFC)
			{
				recovery_started = failsafe_nfc_recovery(ctx->fail_code);
			}
			
			if (recovery_started)
			{
				ctx->recovery_timer = STABILIZATION_DELAY_MS;
				ctx->state = FAILSAFE_STATE_WAIT_RESULT;
			}
			else
			{
				ctx->fail_count++;
				ctx->recovery_timer = RECOVERY_DELAY_MS;
				ctx->state = FAILSAFE_STATE_TRIGGERED;
			}
			break;
		}
		
		case FAILSAFE_STATE_WAIT_RESULT:
			if (ctx->recovery_timer > 0u)
			{
				return;
			}
			
			ctx->state = FAILSAFE_STATE_CHECK;
			break;
			
		case FAILSAFE_STATE_CHECK:
			if (ctx->fail_code == 0u)
			{
				LOG_API_FAIL("FailSafe OK (cnt:%d)\r\n", ctx->fail_count + 1);
				ctx->fail_count = 0u;
				ctx->state = FAILSAFE_STATE_IDLE;
			}
			else
			{
				ctx->fail_count++;
				
				if (ctx->fail_count >= FAILSAFE_LIMIT_COUNT)
				{
					LOG_API_FAIL("FailSafe LIMIT!\r\n");
					ctx->state = FAILSAFE_STATE_PERMANENT_FAIL;
				}
				else
				{
					ctx->recovery_timer = RECOVERY_DELAY_MS;
					ctx->state = FAILSAFE_STATE_TRIGGERED;
				}
			}
			break;
			
		case FAILSAFE_STATE_PERMANENT_FAIL:
		{
			api_failsafe_code_e safe_code = TAG_SAFE_01;
			api_failsafe_state_e next_state = FAILSAFE_STATE_IDLE;
			
			if (module_id == MODULE_UWB)
			{
				safe_code = failsafe_uwb_convert(ctx->fail_code);
				next_state = failsafe_uwb_get_terminal_state(ctx->fail_code);
			}
			else if (module_id == MODULE_NFC)
			{
				safe_code = failsafe_nfc_convert(ctx->fail_code);
				next_state = failsafe_nfc_get_terminal_state(ctx->fail_code);
			}
			
			failsafe_write_result(safe_code, TAG_FAIL_FAIL);
			LOG_API_FAIL("FailSafe TAG_SAFE_%02d=FAIL\r\n", safe_code);
			
			ctx->state = next_state;
			
			if (ctx->state == FAILSAFE_STATE_IDLE)
			{
				ctx->enabled = false;
				LOG_API_FAIL("FailSafe Module disabled\r\n");
			}
			
			ctx->fail_count = 0u;
			break;
		}
		
		case FAILSAFE_STATE_TERMINATED_SYSTEM:
			Api_sleep_setSleepTime(1000 * 60 * 60);	// 1 hour
			Api_sleep_start_sleep();
			break;
			
		default:
			ctx->state = FAILSAFE_STATE_IDLE;
			break;
	}
}

// ========================================================================================
// Public Functions
// ========================================================================================

void Api_failsafe_init(void)
{
	memset(failsafe_modules_g, 0, sizeof(failsafe_modules_g));
	
	for (uint8_t i = 0u; i < MODULE_MAX; i++)
	{
		failsafe_modules_g[i].state = FAILSAFE_STATE_IDLE;
		failsafe_modules_g[i].enabled = true;
	}
	
	memset(failsafe_results_ui8, TAG_FAIL_PASS, TAG_SAFE_MAX);
	Api_nfc_write_data(FAILSAFE_ADDRESS, failsafe_results_ui8, TAG_SAFE_MAX);
	
	LOG_API_FAIL("FailSafe init (limit:%d)\r\n", FAILSAFE_LIMIT_COUNT);
}

void Api_failsafe_timer_tick(void)
{
	for (uint8_t i = 0u; i < MODULE_MAX; i++)
	{
		if (failsafe_modules_g[i].recovery_timer > 0u)
		{
			failsafe_modules_g[i].recovery_timer--;
		}
	}
}

void Api_failsafe_main(void)
{
	for (uint8_t i = 0u; i < MODULE_MAX; i++)
	{
		failsafe_process_module((module_id_e)i);
	}
}

// ========================================================================================
// UWB Interface
// ========================================================================================

void Api_failsafe_set_uwb_fail(uwb_fail_code_e code)
{
	failsafe_module_context_t* ctx = failsafe_get_module(MODULE_UWB);
	
	if (ctx == NULL)
	{
		return;
	}
	
	if (ctx->state == FAILSAFE_STATE_IDLE)
	{
		if (code != UWB_FAIL_NONE)
		{
			ctx->fail_code = (uint8_t)code;
			ctx->fail_count = 0u;
			ctx->recovery_timer = RECOVERY_DELAY_MS;
		}
	}
	else
	{
		ctx->fail_code = (uint8_t)code;
		LOG_API_FAIL("FailSafe Code:%d\r\n", code);
	}
}

uwb_fail_code_e Api_failsafe_get_uwb_fail(void)
{
	failsafe_module_context_t* ctx = failsafe_get_module(MODULE_UWB);
	if (ctx == NULL)
	{
		return UWB_FAIL_NONE;
	}
	
	return (uwb_fail_code_e)ctx->fail_code;
}

bool Api_failsafe_get_uwb_status(void)
{
	failsafe_module_context_t* ctx = failsafe_get_module(MODULE_UWB);
	if (ctx == NULL)
	{
		return false;
	}
	
	return (ctx->state != FAILSAFE_STATE_IDLE);
}

api_failsafe_state_e Api_failsafe_get_uwb_state(void)
{
	failsafe_module_context_t* ctx = failsafe_get_module(MODULE_UWB);
	if (ctx == NULL)
	{
		return FAILSAFE_STATE_IDLE;
	}
	
	return ctx->state;
}

// ========================================================================================
// NFC Interface
// ========================================================================================

void Api_failsafe_set_nfc_fail(nfc_fail_code_e code)
{
	failsafe_module_context_t* ctx = failsafe_get_module(MODULE_NFC);
	if (ctx == NULL)
	{
		return;
	}
	
	if (ctx->state == FAILSAFE_STATE_IDLE && code != NFC_FAIL_NONE)
	{
		ctx->fail_code = (uint8_t)code;
		ctx->fail_count = 0u;
		ctx->recovery_timer = RECOVERY_DELAY_MS;
	}
	else
	{
		ctx->fail_code = (uint8_t)code;
	}
}

nfc_fail_code_e Api_failsafe_get_nfc_fail(void)
{
	failsafe_module_context_t* ctx = failsafe_get_module(MODULE_NFC);
	return (ctx != NULL) ? (nfc_fail_code_e)ctx->fail_code : NFC_FAIL_NONE;
}

bool Api_failsafe_get_nfc_status(void)
{
	failsafe_module_context_t* ctx = failsafe_get_module(MODULE_NFC);
	return (ctx != NULL) ? (ctx->state != FAILSAFE_STATE_IDLE) : false;
}

bool Api_failsafe_get_activated(void)
{
	bool ret = false;

	if  ((Api_failsafe_get_nfc_fail())
		|| (Api_failsafe_get_uwb_fail()))
	{
		ret = true;
	}
	
	return ret;
}

// ========================================================================================
// Global
// ========================================================================================

bool Api_failsafe_has_permanent_failure(void)
{
	for (uint8_t i = 0u; i < TAG_SAFE_MAX; i++)
	{
		if (failsafe_results_ui8[i] == TAG_FAIL_FAIL)
		{
			return true;
		}
	}
	
	return false;
}

