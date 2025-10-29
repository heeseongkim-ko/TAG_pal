
#include "nrf_delay.h"
#include "Api_failsafe.h"
#include "Func_UART_LOG.h"

static uint16_t failsafe_battery_level_ui16;
static uint32_t failsafe_battery_time_ui32;
static bool failsafe_battery_init_b = false;

void Drv_failsafe_battery_init(void)
{
	failsafe_battery_time_ui32 = 0u;
	failsafe_battery_level_ui16 = 0u;
	failsafe_battery_init_b = false;
}

bool Drv_failsafe_battery_update_level(uint16_t level, failsafe_information_t *rec)
{
	int16_t voltage_diff_i16;
	uint16_t abs_voltage_diff_ui16;
	uint32_t elapsed_minutes_ui32;
	uint16_t voltage_change_per_minute_ui16;
	bool ret = false;
	
	// Initialize on first call
	if  (failsafe_battery_init_b == false)
	{
		failsafe_battery_init_b = true;
		failsafe_battery_level_ui16 = level;
		failsafe_battery_time_ui32 = 0u;
		LOG_API_FAIL("[FAIL] BATTERY: Initial level = %d mV\r\n", level);
		return ret;
	}

	// Check only if at least 1 minute has elapsed
	if  (failsafe_battery_time_ui32 >= (1 * 60 * 1000))		// >= 1 min
	{
		// Calculate elapsed time in minutes
		elapsed_minutes_ui32 = failsafe_battery_time_ui32 / (60 * 1000);
		
		// Calculate voltage difference
		voltage_diff_i16 = (int16_t)level - (int16_t)failsafe_battery_level_ui16;
		
		// Get absolute value
		if  (voltage_diff_i16 < 0)
		{
			abs_voltage_diff_ui16 = (uint16_t)(-voltage_diff_i16);
		}
		else
		{
			abs_voltage_diff_ui16 = (uint16_t)voltage_diff_i16;
		}
		
		// Calculate voltage change per minute
		voltage_change_per_minute_ui16 = abs_voltage_diff_ui16 / elapsed_minutes_ui32;
		
		// Check if voltage change rate exceeds 0.1V (100mV) per minute
		if  (voltage_change_per_minute_ui16 > FAILSAFE_VOLTAGE_0_1V)
		{
			rec->retry_count++;
			LOG_API_FAIL("[FAIL] BATTERY: Abnormal change! %d mV -> %d mV (%+d mV in %u min, %u mV/min) [%d/%d]\r\n",
						failsafe_battery_level_ui16, level, voltage_diff_i16, 
						elapsed_minutes_ui32, voltage_change_per_minute_ui16,
						rec->retry_count, FAILSAFE_MAX_RETRY_COUNT);
			
			// Check if max retry count exceeded
			if  (rec->retry_count > FAILSAFE_MAX_RETRY_COUNT)
			{
				LOG_API_FAIL("[FAIL] BATTERY: Max retry exceeded, declaring FAIL\r\n");
				
				Api_failsafe_set_fail(FAILSAFE_CODE_08);

				rec->fail_flag = true;
				rec->result = FAILSAFE_RESULT_FAIL;
				rec->status = FAILSAFE_STATUS_TERMINATED;
				rec->retry_count = 0;

				ret = true;
			}
		}
		else
		{
			// Voltage change rate within normal range - reset retry count
			if  (rec->retry_count > 0)
			{
				LOG_API_FAIL("[FAIL] BATTERY: Voltage change normal (%u mV/min), reset retry count\r\n",
							voltage_change_per_minute_ui16);
			}
			rec->retry_count = 0;
		}
		
		// Update baseline and reset timer
		failsafe_battery_level_ui16 = level;
		failsafe_battery_time_ui32 = 0u;
	}
	// If less than 1 minute, just continue accumulating time

	return ret;
}

void Drv_failsafe_battery_time_update(uint32_t time)
{
	failsafe_battery_time_ui32 = failsafe_battery_time_ui32 + time;
}


