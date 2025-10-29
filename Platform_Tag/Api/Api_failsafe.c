
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

static failsafe_record_t failsafe_records_g[FAILSAFE_CODE_MAX];
static bool failsafe_initialized_b = false;

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

static void failsafe_load_from_nfc(void)
{
	uint8_t nfc_data_ui8[FAILSAFE_CODE_MAX];
	nfc_result_t result;
	bool need_init = false;
	
	LOG_API_FAIL("[FAIL] Loading FailSafe data from NFC (addr=0x%02X)\r\n", FAILSAFE_NFC_ADDRESS);
	
	result = Api_nfc_read_data(FAILSAFE_NFC_ADDRESS, nfc_data_ui8, FAILSAFE_CODE_MAX);
	
	if (result != NFC_SUCCESS)
	{
		LOG_API_FAIL("[FAIL] NFC read failed, initializing to SUCCESS\r\n");
		need_init = true;
	}
	else
	{
		for (uint8_t i = 0; i < FAILSAFE_CODE_MAX; i++)
		{
			if ((nfc_data_ui8[i] != FAIL_RESULT_SUCCESS) && 
				(nfc_data_ui8[i] != FAIL_RESULT_FAIL))
			{
				LOG_API_FAIL("[FAIL] Invalid data at index %d (0x%02X), need init\r\n", i, nfc_data_ui8[i]);
				need_init = true;
				break;
			}
		}
	}
	
	if (need_init)
	{
		for (uint8_t i = 0; i < FAILSAFE_CODE_MAX; i++)
		{
			nfc_data_ui8[i] = FAIL_RESULT_SUCCESS;
		}
		Api_nfc_write_data(FAILSAFE_NFC_ADDRESS, nfc_data_ui8, FAILSAFE_CODE_MAX);
		LOG_API_FAIL("[FAIL] NFC initialized with SUCCESS values\r\n");
	}
	else
	{
		LOG_API_FAIL("[FAIL] NFC data loaded successfully\r\n");
	}
	
	for (uint8_t i = 0; i < FAILSAFE_CODE_MAX; i++)
	{
		failsafe_records_g[i].result = nfc_data_ui8[i];
		failsafe_records_g[i].status = FAIL_STATUS_NORMAL;
		failsafe_records_g[i].retry_count = 0;
		failsafe_records_g[i].fail_flag = false;
		failsafe_records_g[i].success_flag = false;
		failsafe_records_g[i].parent_code = FAILSAFE_CODE_MAX;
		
		if (nfc_data_ui8[i] == FAIL_RESULT_FAIL)
		{
			LOG_API_FAIL("[FAIL] %s: Previous FAIL detected\r\n", failsafe_get_fail_name(i));
		}
	}
}

static void failsafe_save_to_nfc(FAILSAFE_CODE code, uint8_t result)
{
	uint8_t nfc_data_ui8[FAILSAFE_CODE_MAX];
	
	LOG_API_FAIL("[FAIL] Saving %s result to NFC: %s\r\n", 
				failsafe_get_fail_name(code),
				(result == FAIL_RESULT_SUCCESS) ? "SUCCESS" : "FAIL");
	
	Api_nfc_read_data(FAILSAFE_NFC_ADDRESS, nfc_data_ui8, FAILSAFE_CODE_MAX);
	nfc_data_ui8[code] = result;
	
	nfc_result_t nfc_result = Api_nfc_write_data(FAILSAFE_NFC_ADDRESS, nfc_data_ui8, FAILSAFE_CODE_MAX);
	
	if (nfc_result == NFC_SUCCESS)
	{
		LOG_API_FAIL("[FAIL] NFC write successful\r\n");
	}
	else
	{
		LOG_API_FAIL("[FAIL] NFC write failed (error=%d)\r\n", nfc_result);
	}
}

static bool failsafe_is_major_code(FAILSAFE_CODE code)
{
	if ((code == FAILSAFE_CODE_02) || (code == FAILSAFE_CODE_03) || (code == FAILSAFE_CODE_04) ||
		(code == FAILSAFE_CODE_11) || (code == FAILSAFE_CODE_16))
	{
		return true;
	}
	
	return false;
}

static bool failsafe_handle_success(FAILSAFE_CODE code, failsafe_record_t *rec)
{
	if (!rec->success_flag)
	{
		return false;
	}
	
	LOG_API_FAIL("[FAIL] %s: Recovery SUCCESS\r\n", failsafe_get_fail_name(code));
	
	rec->success_flag = false;
	rec->fail_flag = false;
	rec->status = FAIL_STATUS_NORMAL;
	rec->retry_count = 0;
	rec->result = FAIL_RESULT_SUCCESS;
	
	if (rec->parent_code != FAILSAFE_CODE_MAX)
	{
		FAILSAFE_CODE parent = rec->parent_code;
		rec->parent_code = FAILSAFE_CODE_MAX;
		
		LOG_API_FAIL("[FAIL] %s: Parent %s also SUCCESS\r\n", 
					failsafe_get_fail_name(code), failsafe_get_fail_name(parent));
		
		failsafe_records_g[parent].status = FAIL_STATUS_NORMAL;
		failsafe_records_g[parent].retry_count = 0;
		failsafe_records_g[parent].result = FAIL_RESULT_SUCCESS;
		failsafe_records_g[parent].fail_flag = false;
		
		failsafe_save_to_nfc(parent, FAIL_RESULT_SUCCESS);
	}
	
	failsafe_save_to_nfc(code, FAIL_RESULT_SUCCESS);
	return true;
}

static void failsafe_handle_init_max_retry(FAILSAFE_CODE code, failsafe_record_t *rec)
{
	if (rec->parent_code != FAILSAFE_CODE_02)
	{
		FAILSAFE_CODE parent = rec->parent_code;
		rec->parent_code = FAILSAFE_CODE_02;
		
		LOG_API_FAIL("[FAIL] %s: INIT failed, parent %s also FAIL\r\n", 
					failsafe_get_fail_name(code), failsafe_get_fail_name(parent));
		
		rec->status = FAIL_STATUS_TERMINATED;
		rec->result = FAIL_RESULT_FAIL;
		failsafe_save_to_nfc(code, FAIL_RESULT_FAIL);
		
		failsafe_records_g[parent].status = FAIL_STATUS_TERMINATED;
		failsafe_records_g[parent].result = FAIL_RESULT_FAIL;
		failsafe_records_g[parent].retry_count = 0;
	}
	else
	{
		LOG_API_FAIL("[FAIL] %s: INIT failed, TERMINATED\r\n", failsafe_get_fail_name(code));
		
		rec->status = FAIL_STATUS_TERMINATED;
		rec->result = FAIL_RESULT_FAIL;
		failsafe_save_to_nfc(code, FAIL_RESULT_FAIL);
	}
}

static void failsafe_handle_txwakeup_max_retry(FAILSAFE_CODE code, failsafe_record_t *rec)
{
	LOG_API_FAIL("[FAIL] %s: Failed %d times, switching to INIT recovery\r\n", 
				failsafe_get_fail_name(code), FAILSAFE_MAX_RETRY_COUNT);
	
	rec->result = FAIL_RESULT_FAIL;
	rec->retry_count = 0;
	rec->status = FAIL_STATUS_NORMAL;
	failsafe_save_to_nfc(code, FAIL_RESULT_FAIL);
	
	failsafe_record_t *init_rec = &failsafe_records_g[FAILSAFE_CODE_02];
	init_rec->parent_code = code;
	init_rec->fail_flag = true;
	init_rec->status = FAIL_STATUS_NORMAL;
	init_rec->retry_count = 0;
}

static void failsafe_handle_other_max_retry(FAILSAFE_CODE code, failsafe_record_t *rec)
{
	LOG_API_FAIL("[FAIL] %s: Max retry reached, FAIL\r\n", failsafe_get_fail_name(code));
	
	rec->status = FAIL_STATUS_TERMINATED;
	rec->result = FAIL_RESULT_FAIL;
	failsafe_save_to_nfc(code, FAIL_RESULT_FAIL);
}

static void failsafe_handle_max_retry(FAILSAFE_CODE code, failsafe_record_t *rec)
{
	LOG_API_FAIL("[FAIL] %s: Max retry reached (%d/%d)\r\n", 
				failsafe_get_fail_name(code), rec->retry_count, FAILSAFE_MAX_RETRY_COUNT);
	
	if (code == FAILSAFE_CODE_02)
	{
		failsafe_handle_init_max_retry(code, rec);
	}
	else if ((code == FAILSAFE_CODE_03) || (code == FAILSAFE_CODE_04))
	{
		failsafe_handle_txwakeup_max_retry(code, rec);
	}
	else
	{
		failsafe_handle_other_max_retry(code, rec);
	}
}

static void failsafe_start_recovery(FAILSAFE_CODE code, failsafe_record_t *rec)
{
	LOG_API_FAIL("[FAIL] %s: Starting recovery (retry %d/%d)\r\n", 
				failsafe_get_fail_name(code), rec->retry_count, FAILSAFE_MAX_RETRY_COUNT);
	
	rec->status = FAIL_STATUS_RECOVERING;
	
	switch (code)
	{
		case FAILSAFE_CODE_02:
		case FAILSAFE_CODE_03:
		case FAILSAFE_CODE_04:
			Drv_failsafe_uwb_recover(code, rec->retry_count);
			break;
			
		case FAILSAFE_CODE_16:
			Drv_failsafe_clock_recover(code, rec->retry_count);
			break;
			
		default:
			break;
	}
}

static void failsafe_process_recovery(FAILSAFE_CODE code)
{
	failsafe_record_t *rec = &failsafe_records_g[code];
	
	// Check success first
	if (failsafe_handle_success(code, rec))
	{
		return;
	}
	
	// Check fail flag
	if (!rec->fail_flag)
	{
		return;
	}
	
	// Already failed
	if (rec->result == FAIL_RESULT_FAIL)
	{
		LOG_API_FAIL("[FAIL] %s: Already failed, TERMINATED\r\n", failsafe_get_fail_name(code));
		rec->fail_flag = false;
		rec->status = FAIL_STATUS_TERMINATED;
		return;
	}
	
	// Already recovering
	if (rec->status == FAIL_STATUS_RECOVERING)
	{
		return;
	}
	
	// Clear flag and increment retry
	rec->fail_flag = false;
	rec->retry_count++;
	
	// Check max retry
	if (rec->retry_count > FAILSAFE_MAX_RETRY_COUNT)
	{
		failsafe_handle_max_retry(code, rec);
		return;
	}
	
	// Start recovery
	failsafe_start_recovery(code, rec);
}

void failsafe_process_terminated(void)
{
	if  (Api_Led_IsActive() == false)
	{
		Api_Led_StartBlinkDefault(API_LED_ORANGE, API_LED_REPEAT_INFINITE);
	}
}

void Api_failsafe_timer_tick(void)
{
	if (!failsafe_initialized_b)
	{
		return;
	}
	
	Drv_failsafe_uwb_timer_tick();
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
	
	LOG_API_FAIL("[FAIL] %s: Fail detected\r\n", failsafe_get_fail_name(code));
	
	failsafe_records_g[code].fail_flag = true;
}

void Api_failsafe_set_success(FAILSAFE_CODE code)
{
	if (!failsafe_initialized_b || (code >= FAILSAFE_CODE_MAX))
	{
		return;
	}
	
	LOG_API_FAIL("[FAIL] %s: Success detected\r\n", failsafe_get_fail_name(code));
	
	failsafe_records_g[code].success_flag = true;
}

bool Api_failsafe_isRecovery(void)
{
	if (!failsafe_initialized_b)
	{
		return false;
	}
	
	for (uint8_t i = 0; i < FAILSAFE_CODE_MAX; i++)
	{
		if (failsafe_records_g[i].status == FAIL_STATUS_RECOVERING)
		{
			return true;
		}
	}
	
	return false;
}

bool Api_failsafe_isMajor(void)
{
	if (!failsafe_initialized_b)
	{
		return false;
	}
	
	for (uint8_t i = 0; i < FAILSAFE_CODE_MAX; i++)
	{
		if (failsafe_is_major_code(i))
		{
			if ((failsafe_records_g[i].result == FAIL_RESULT_FAIL) || 
				(failsafe_records_g[i].status == FAIL_STATUS_TERMINATED))
			{
				return true;
			}
		}
	}
	
	return false;
}

uint8_t Api_failsafe_getResult(FAILSAFE_CODE code)
{
	if (!failsafe_initialized_b || (code >= FAILSAFE_CODE_MAX))
	{
		return FAIL_RESULT_NONE;
	}
	
	return failsafe_records_g[code].result;
}

uint8_t Api_failsafe_getStatus(FAILSAFE_CODE code)
{
	if (!failsafe_initialized_b || (code >= FAILSAFE_CODE_MAX))
	{
		return FAIL_STATUS_NORMAL;
	}
	
	return failsafe_records_g[code].status;
}

uint8_t Api_failsafe_getRetryCount(FAILSAFE_CODE code)
{
	if (!failsafe_initialized_b || (code >= FAILSAFE_CODE_MAX))
	{
		return 0;
	}
	
	return failsafe_records_g[code].retry_count;
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
	if  (Drv_failsafe_battery_update_level(level, &failsafe_records_g[FAILSAFE_CODE_08]))
	{		
		failsafe_save_to_nfc(FAILSAFE_CODE_08, FAIL_RESULT_FAIL);
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
bool Api_failsafe_check_memory(void)
{
	bool result = true;

	LOG_API_FAIL("[FAIL] INTERNAL_MEM: Starting memory check...\r\n");

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
		LOG_API_FAIL("[FAIL] INTERNAL_MEM: Memory check FAILED, system will terminate\r\n");
		
		// Set failsafe record to terminated state
		failsafe_records_g[FAILSAFE_CODE_11].result = FAIL_RESULT_FAIL;
		failsafe_records_g[FAILSAFE_CODE_11].status = FAIL_STATUS_TERMINATED;
		failsafe_records_g[FAILSAFE_CODE_11].retry_count = 0;
		failsafe_records_g[FAILSAFE_CODE_11].fail_flag = true;
		failsafe_records_g[FAILSAFE_CODE_11].success_flag = false;
		failsafe_records_g[FAILSAFE_CODE_11].parent_code = FAILSAFE_CODE_MAX;
		
		return false;
	}

	LOG_API_FAIL("[FAIL] INTERNAL_MEM: Memory check passed\r\n");
	return true;
}

void Api_failsafe_init(void)
{
	LOG_API_FAIL("[FAIL] FailSafe System Initialization\r\n");
	
	// Memory check MUST be done BEFORE loading from NFC
	// If memory is corrupted, we cannot trust NFC data either
	if (!Api_failsafe_check_memory())
	{
		// Memory check failed - system will be terminated
		// Save fail result to NFC (if NFC is accessible)
		failsafe_save_to_nfc(FAILSAFE_CODE_11, FAIL_RESULT_FAIL);
		
		// Initialize other modules to minimal state
		failsafe_initialized_b = true;
		
		// System will be terminated in Api_failsafe_main()
		return;
	}

	failsafe_load_from_nfc();

	Drv_failsafe_uwb_init();
	LOG_API_FAIL("[FAIL] UWB recovery driver initialized\r\n");

	Drv_failsafe_clock_init();
	LOG_API_FAIL("[FAIL] Clock recovery driver initialized\r\n");

	if  (failsafe_watchdog_check_reset(&failsafe_records_g[FAILSAFE_CODE_01]))
	{		
		failsafe_save_to_nfc(FAILSAFE_CODE_01, FAIL_RESULT_FAIL);
	}

	Drv_failsafe_battery_init();

	failsafe_initialized_b = true;

	LOG_API_FAIL("[FAIL] FailSafe system ready\r\n");
}

void Api_failsafe_watchdog_init(void)
{
	LOG_API_FAIL("[FAIL] %s\r\n", __func__);
	
	failsafe_watchdog_init();
}

bool Api_failsafe_main(void)
{
	uint8_t i;
	
	if (!failsafe_initialized_b)
	{
		return false;
	}

	failsafe_watchdog_channel_feed();
	
	for (i = 0; i < FAILSAFE_CODE_MAX; ++i)
	{
		if  (failsafe_records_g[i].result == FAIL_STATUS_TERMINATED)
		{
			failsafe_process_terminated();
			return false;
		}
	}
	
	failsafe_process_recovery(FAILSAFE_CODE_02);
	failsafe_process_recovery(FAILSAFE_CODE_03);
	failsafe_process_recovery(FAILSAFE_CODE_04);
	failsafe_process_recovery(FAILSAFE_CODE_16);
		
	return Api_failsafe_isMajor();
}

