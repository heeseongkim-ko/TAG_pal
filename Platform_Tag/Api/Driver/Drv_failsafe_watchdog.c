
#include "nrf.h"
#include "nrf_drv_wdt.h"
#include "nrf_drv_clock.h"

#include "Api_failsafe.h"
#include "Func_UART_LOG.h"

nrf_drv_wdt_channel_id wdt_channel_id;
bool failsafe_watchdog_init_b = false;

void wdt_event_handler(void)
{
	// This should not be called if WDT is properly fed
	// If called, system will reset
}

void failsafe_watchdog_init(void)
{
	uint32_t err_code;
	nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
	
	// Configure timeout (example: 10 seconds)
	config.reload_value = 10000; // 10000ms = 10 seconds
	
	err_code = nrf_drv_wdt_init(&config, wdt_event_handler);
	APP_ERROR_CHECK(err_code);
	
	err_code = nrf_drv_wdt_channel_alloc(&wdt_channel_id);
	APP_ERROR_CHECK(err_code);
	
	nrf_drv_wdt_enable();

	failsafe_watchdog_init_b = true;
	
	LOG_API_FAIL("[MAIN] Watchdog initialized (timeout: 10s)\r\n");
}

bool failsafe_watchdog_check_reset(failsafe_information_t *rec)
{
	LOG_API_FAIL("%s\r\n", __func__);
	
	if (NRF_POWER->RESETREAS & POWER_RESETREAS_DOG_Msk)
	{
		LOG_API_FAIL("Watchdog reset detected\r\n");
		
		// Clear reset reason
		NRF_POWER->RESETREAS = POWER_RESETREAS_DOG_Msk;
		
		// Check current result from NFC
		if (rec->result == FAILSAFE_RESULT_SUCCESS)
		{
			return true;
		}
		else if (rec->result == FAILSAFE_RESULT_FAIL)
		{
		}		
	}

	return false;
}

void failsafe_watchdog_channel_feed(void)
{	
	if  (failsafe_watchdog_init_b == false)
	{
		return;
	}
	
	nrf_drv_wdt_channel_feed(wdt_channel_id);
}

