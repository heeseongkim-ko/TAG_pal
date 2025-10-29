
#include "Api_failsafe.h"
#include "Api_nfc.h"
#include "Api_Led.h"
#include "Drv_failsafe_uwb.h"
#include "Drv_failsafe_watchdog.h"
#include "Drv_failsafe_clock.h"
#include "Drv_failsafe_battery.h"
#include "Drv_failsafe_memory.h"
#include "Func_UART_LOG.h"

/******************************************************************************
 * Memory Check Definitions
 ******************************************************************************/

/**
 * @brief Magic value stored in Flash
 * @details volatile const ensures:
 * - const: Stored in Flash (read-only)
 * - volatile: Prevents compiler optimization
 * This value is written at compile time and read at boot time for Flash verification
 */

static failsafe_information_t failsafe_records_g[FAILSAFE_CODE_MAX];
static bool failsafe_initialized_b = false;
static failsafe_information_t failsafe_info_g[FAILSAFE_CODE_MAX];
static bool failsafe_system_terminated_b = false;

static const char* failsafe_get_fail_name(FAILSAFE_CODE code)
{
	switch (code)
	{
		case FAILSAFE_CODE_01: return "WATCHDOG";
		case FAILSAFE_CODE_02: return "UWB_INIT";
		case FAILSAFE_CODE_03: return "UWB_TX";
		case FAILSAFE_CODE_04: return "UWB_WAKEUP";
		case FAILSAFE_CODE_05: return "NFC_INIT";
		case FAILSAFE_CODE_06: return "NFC_EEPROM";
		case FAILSAFE_CODE_07: return "BLE_INIT";
		case FAILSAFE_CODE_08: return "BATTERY";
		case FAILSAFE_CODE_09: return "BLE_OTA";
		case FAILSAFE_CODE_10: return "STACK_OVERFLOW";
		case FAILSAFE_CODE_11: return "INTERNAL_MEM";
		case FAILSAFE_CODE_12: return "SPI_UWB";
		case FAILSAFE_CODE_13: return "I2C_NFC";
		case FAILSAFE_CODE_14: return "I2C_IMU";
		case FAILSAFE_CODE_15: return "FW_INTEGRITY";
		case FAILSAFE_CODE_16: return "SYS_CLOCK";
		case FAILSAFE_DIAG_01: return "UWB_POWER";
		case FAILSAFE_DIAG_02: return "RX_RATE";
		default: return "UNKNOWN";
	}
}

static bool failsafe_is_major_code(FAILSAFE_CODE code)
{
	if ((code == FAILSAFE_CODE_02) || (code == FAILSAFE_CODE_03) || (code == FAILSAFE_CODE_04) ||
		(code == FAILSAFE_CODE_08) || (code == FAILSAFE_CODE_11) || (code == FAILSAFE_CODE_15) ||
		(code == FAILSAFE_CODE_16))
	{
		return true;
	}
	
	return false;
}

static void failsafe_load_from_nfc(void)
{
	uint8_t nfc_data_ui8[FAILSAFE_CODE_MAX];
	nfc_result_t result;
	bool need_init = false;
		
	result = Api_nfc_read_data(FAILSAFE_NFC_ADDRESS, nfc_data_ui8, FAILSAFE_CODE_MAX);
	
	if (result != NFC_SUCCESS)
	{
		need_init = true;
	}
	else
	{
		for (uint8_t i = 0; i < FAILSAFE_CODE_MAX; i++)
		{
			if ((nfc_data_ui8[i] != FAILSAFE_RESULT_SUCCESS) && (nfc_data_ui8[i] != FAILSAFE_RESULT_FAIL))
			{
				need_init = true;
				break;
			}
		}
	}
	
	if (need_init)
	{
		for (uint8_t i = 0; i < FAILSAFE_CODE_MAX; i++)
		{
			nfc_data_ui8[i] = FAILSAFE_RESULT_SUCCESS;
		}
		
		Api_nfc_write_data(FAILSAFE_NFC_ADDRESS, nfc_data_ui8, FAILSAFE_CODE_MAX);
	}
	
	for (uint8_t i = 0; i < FAILSAFE_CODE_MAX; i++)
	{
		failsafe_info_g[i].result = nfc_data_ui8[i];
		failsafe_info_g[i].status = FAILSAFE_STATUS_TRIGGER;
		failsafe_info_g[i].retry_count = 0;
		failsafe_info_g[i].fail_flag = false;
		failsafe_info_g[i].success_flag = false;

		if  ((failsafe_is_major_code(i)) && (failsafe_info_g[i].result == FAILSAFE_RESULT_FAIL))
		{
			failsafe_system_terminated_b = true;
		}
	}
}

static void failsafe_save_to_nfc(FAILSAFE_CODE code, uint8_t result)
{
	uint8_t nfc_data_ui8[FAILSAFE_CODE_MAX];
	
	LOG_API_FAIL("%s:Saving %s result to NFC: %s\r\n", __func__,
				failsafe_get_fail_name(code),
				(result == FAILSAFE_RESULT_SUCCESS) ? "SUCCESS" : "FAIL");
	
	Api_nfc_read_data(FAILSAFE_NFC_ADDRESS, nfc_data_ui8, FAILSAFE_CODE_MAX);
	nfc_data_ui8[code] = result;
	
	nfc_result_t nfc_result = Api_nfc_write_data(FAILSAFE_NFC_ADDRESS, nfc_data_ui8, FAILSAFE_CODE_MAX);
	
	if (nfc_result == NFC_SUCCESS)
	{
		LOG_API_FAIL("NFC write successful\r\n");
	}
	else
	{
		LOG_API_FAIL("NFC write failed (error=%d)\r\n", nfc_result);
	}
}

void Api_failsafe_timer_tick(void)
{
	failsafe_information_t *info;
	int8_t i;

	for(i = 0; i < FAILSAFE_CODE_MAX; ++i)
	{
		info = &failsafe_info_g[i];

		if  (info->timer > 0)
		{
			info->timer--;
		}
	}
}

bool Api_failsafe_clock_check_hfclk_start(void)
{
	return Drv_failsafe_clock_check_hfclk_start();
}

bool Api_failsafe_clock_check_lfclk_start(void)
{
	return Drv_failsafe_clock_check_lfclk_start();
}

void Api_failsafe_set_fail(FAILSAFE_CODE code)
{
	if (!failsafe_initialized_b || (code >= FAILSAFE_CODE_MAX))
	{
		return;
	}

	if  (failsafe_info_g[code].result == FAILSAFE_RESULT_SUCCESS)
	{
		LOG_API_FAIL("%s:%s\r\n", __func__, failsafe_get_fail_name(code));
	
		failsafe_info_g[code].success_flag = false;
		failsafe_info_g[code].fail_flag = true;
	}
	
	if  (failsafe_info_g[code].status == FAILSAFE_STATUS_RECOVERING)
	{
		failsafe_info_g[code].status = FAILSAFE_STATUS_CHECK_RECOVER;
	}
}

void Api_failsafe_set_success(FAILSAFE_CODE code)
{
	if (!failsafe_initialized_b || (code >= FAILSAFE_CODE_MAX))
	{
		return;
	}
	
	LOG_API_FAIL("%s:%s\r\n", __func__, failsafe_get_fail_name(code));
	
	failsafe_info_g[code].success_flag = true;
	failsafe_info_g[code].fail_flag = false;

	if  (failsafe_info_g[code].status == FAILSAFE_STATUS_RECOVERING)
	{
		failsafe_info_g[code].status = FAILSAFE_STATUS_CHECK_RECOVER;
	}
}

bool Api_failsafe_isMajor(void)
{	
	for (uint8_t i = 0; i < FAILSAFE_CODE_MAX; i++)
	{
		if (failsafe_is_major_code(i))
		{
			return true;
		}
	}
	
	return false;
}

bool Api_failsafe_blocking_system(void)
{
	uint8_t i;

	if  (failsafe_system_terminated_b == true)
	{
		return true;
	}
	
	for(i = 0; i < FAILSAFE_CODE_MAX; i++)
	{
		if  (failsafe_info_g[i].status > FAILSAFE_STATUS_TERMINATED)
		{
			return true;
		}
	}

	return false;
}

/**
 * @brief Update battery level and check for abnormal voltage changes
 * 
 * Monitors battery voltage and detects failures when voltage changes exceed 100mV per minute.
 * Handles irregular update intervals by normalizing voltage change to per-minute rate.
 * 
 * Detection logic:
 * 1. Calculate elapsed time since last check
 * 2. Calculate voltage change per minute: |voltage_change| / (elapsed_time / 60sec)
 * 3. If voltage_change_per_minute > 100mV: increment retry_count
 * 4. If voltage_change_per_minute <= 100mV: reset retry_count to 0
 * 5. If retry_count > 5: declare FAIL
 * 
 * @param level Current battery voltage in mV
 */
void Api_failsafe_battery_update_level(uint16_t level)
{
	if  (Drv_failsafe_battery_update_level(level, &failsafe_info_g[FAILSAFE_CODE_08]))
	{				
		failsafe_info_g[FAILSAFE_CODE_08].retry_count = 0;
		failsafe_info_g[FAILSAFE_CODE_08].fail_flag = true;
		failsafe_info_g[FAILSAFE_CODE_08].success_flag = false;
	}
}

void Api_failsafe_battery_time_update(uint32_t time)
{
	Drv_failsafe_battery_time_update(time);
}

/**
 * @brief Check internal memory (Flash/RAM) integrity at boot
 * @return true if memory is healthy, false otherwise
 * 
 * @details
 * Memory check procedure:
 * 1. Check Flash integrity by reading magic value
 * 2. Check RAM integrity by R/W test
 * 3. If any check fails:
 *    - Set FAILSAFE_CODE_11
 *    - Save to NFC
 *    - Set status to TERMINATED
 *    - Return false
 * 
 * @note This function should be called at the very beginning of Api_failsafe_init()
 * @warning System should not continue if this function returns false
 */
bool failsafe_check_memory(void)
{
	bool result = true;

	LOG_API_FAIL("%s\r\n", __func__);

	// Check Flash integrity
	if (!Drv_failsafe_memory_check_flash())
	{
		result = false;
	}

	// Check RAM integrity
	if (!Drv_failsafe_memory_check_ram())
	{
		result = false;
	}

	// If any check failed, set failsafe code
	if (!result)
	{
		LOG_API_FAIL("Memory check FAILED, system will terminate\r\n");
		
		// Set failsafe record to terminated state
		failsafe_info_g[FAILSAFE_CODE_11].retry_count = 0;
		failsafe_info_g[FAILSAFE_CODE_11].fail_flag = true;
		failsafe_info_g[FAILSAFE_CODE_11].success_flag = false;
		
		return false;
	}

	LOG_API_FAIL("Memory check passed\r\n");
	return true;
}

void Api_failsafe_init(void)
{
	LOG_API_FAIL("%s\r\n", __func__);

	failsafe_system_terminated_b = false;
	failsafe_initialized_b = true;
	
	failsafe_load_from_nfc();

	if  (failsafe_watchdog_check_reset(&failsafe_info_g[FAILSAFE_CODE_01]))
	{
		if  (failsafe_info_g[FAILSAFE_CODE_01].result == FAILSAFE_RESULT_SUCCESS)
		{
			failsafe_save_to_nfc(FAILSAFE_CODE_01, FAILSAFE_RESULT_FAIL);
		}
	}

	if (!failsafe_check_memory())
	{
		if  (failsafe_info_g[FAILSAFE_CODE_11].result == FAILSAFE_RESULT_SUCCESS)
		{
			failsafe_save_to_nfc(FAILSAFE_CODE_11, FAILSAFE_RESULT_FAIL);
		}
		
		failsafe_system_terminated_b = true;
		return;
	}

	Drv_failsafe_battery_init();

}

void Api_failsafe_watchdog_init(void)
{
	LOG_API_FAIL("%s\r\n", __func__);
	
	failsafe_watchdog_init();
}

void failsafe_set_recover(FAILSAFE_CODE code, failsafe_information_t *info)
{
	Drv_failsafe_uwb_recover(code, info);

	info->retry_count++;	
	info->status = FAILSAFE_STATUS_RECOVERING;
	info->recovery_result = false;
}

void failsafe_check_recover(FAILSAFE_CODE code, failsafe_information_t *info)
{
	LOG_API_FAIL("%s:%s\r\n", __func__, failsafe_get_fail_name(code));
	
	if  (info->success_flag)
	{
	}
	else
	{
		info->retry_count++;
		if  (info->retry_count > FAILSAFE_MAX_RETRY_COUNT)
		{
			if  (info->result == FAILSAFE_RESULT_SUCCESS)
			{
				failsafe_save_to_nfc(code, FAILSAFE_RESULT_FAIL);
				info->result = FAILSAFE_RESULT_FAIL;
			}
			
			if  (Api_failsafe_isMajor())
			{
				failsafe_system_terminated_b = true;
			}
		}
		else
		{
			failsafe_set_recover(code, info);
		}
	}

	info->status = FAILSAFE_STATUS_TRIGGER;
	info->success_flag = false;
	info->fail_flag = false;
}

void failsafe_fail_trigger(FAILSAFE_CODE code, failsafe_information_t *info)
{
	if  (info->fail_flag == false)
	{
		return;		
	}

	info->retry_count = 0;
	info->fail_flag = false;
	info->success_flag = false;
	info->recovery_result = false;
	failsafe_set_recover(code, info);
}

void failsafe_state_machine(FAILSAFE_CODE code)
{
	failsafe_information_t *info;

	info = &failsafe_info_g[code];
	
	switch  (info->status)
	{
		case FAILSAFE_STATUS_TRIGGER:
			failsafe_fail_trigger(code, info);
			break;
		case FAILSAFE_STATUS_RECOVERING:
			break;
		case FAILSAFE_STATUS_CHECK_RECOVER:
			failsafe_check_recover(code, info);
			break;
	}
}

void failsafe_state_terminated(void)
{
	if  (Api_Led_IsActive() == false)
	{
		Api_Led_StartBlinkDefault(API_LED_ORANGE, API_LED_REPEAT_INFINITE);
	}
}

void Api_failsafe_main(void)
{	
	if (!failsafe_initialized_b)
	{
		return;
	}

	failsafe_watchdog_channel_feed();

	// Terminated add please
	if  (failsafe_system_terminated_b == true)
	{
		return;
	}
	
	failsafe_state_machine(FAILSAFE_CODE_01);
	failsafe_state_machine(FAILSAFE_CODE_02);
	failsafe_state_machine(FAILSAFE_CODE_03);
	failsafe_state_machine(FAILSAFE_CODE_04);
	failsafe_state_machine(FAILSAFE_CODE_05);
	failsafe_state_machine(FAILSAFE_CODE_06);
	failsafe_state_machine(FAILSAFE_CODE_07);
	failsafe_state_machine(FAILSAFE_CODE_08);
	failsafe_state_machine(FAILSAFE_CODE_09);
	failsafe_state_machine(FAILSAFE_CODE_10);
	failsafe_state_machine(FAILSAFE_CODE_11);
	failsafe_state_machine(FAILSAFE_CODE_12);
	failsafe_state_machine(FAILSAFE_CODE_13);
	failsafe_state_machine(FAILSAFE_CODE_14);
	failsafe_state_machine(FAILSAFE_CODE_15);
	failsafe_state_machine(FAILSAFE_CODE_16);
	failsafe_state_machine(FAILSAFE_DIAG_01);
	failsafe_state_machine(FAILSAFE_DIAG_02);
}

