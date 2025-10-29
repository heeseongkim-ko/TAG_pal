
#ifndef API_FAILSAFE_H
#define API_FAILSAFE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
	FAILSAFE_CODE_01 = 0,  ///< Watchdog Reset : Major -> OK
	FAILSAFE_CODE_02,      ///< UWB Init failure : Major -> OK
	FAILSAFE_CODE_03,      ///< UWB TX failure : Major -> OK
	FAILSAFE_CODE_04,      ///< UWB Wakeup failure : Major -> OK
	FAILSAFE_CODE_05,      ///< NFC Init : Minor 
	FAILSAFE_CODE_06,      ///< NFC EEPROM : Minor 
	FAILSAFE_CODE_07,      ///< BLE Init : Minor
	FAILSAFE_CODE_08,      ///< Battery : Major -> OK
	FAILSAFE_CODE_09,      ///< BLE OTA Update : Minor
	FAILSAFE_CODE_10,      ///< Stack Overflow : Major -> RTOS Only
	FAILSAFE_CODE_11,      ///< Internal Memory : Major -> OK
	FAILSAFE_CODE_12,      ///< SPI UWB : Major -> OK
	FAILSAFE_CODE_13,      ///< I2C( NFC) : Minor
	FAILSAFE_CODE_14,      ///< I2C IMU : Minor
	FAILSAFE_CODE_15,      ///< Firmware Integrity Verification : Major
	FAILSAFE_CODE_16,      ///< System Clock : Major
	FAILSAFE_DIAG_01,      ///< UWB Power : None
	FAILSAFE_DIAG_02,      ///< RX Rate : None
	FAILSAFE_CODE_MAX
} FAILSAFE_CODE;

#define FAILSAFE_RESULT_NONE     0x00  ///< No failure detected
#define FAILSAFE_RESULT_SUCCESS  0x01  ///< Operating normally
#define FAILSAFE_RESULT_FAIL     0x02  ///< Final fail (5+ retries)

#define	FAILSAFE_STATUS_TRIGGER 		0x00
#define	FAILSAFE_STATUS_RECOVERING 		0x01
#define FAILSAFE_STATUS_CHECK_RECOVER 	0x02
#define	FAILSAFE_STATUS_TERMINATED 		0x03

#define FAILSAFE_NFC_ADDRESS  0xE0

#define FAILSAFE_MAX_RETRY_COUNT  5

#define	FAILSAFE_VOLTAGE_0_1V	100

typedef struct
{
	uint8_t result;       ///< Fail result (NFC storage): SUCCESS/FAIL
	uint8_t status;       ///< Fail status (RAM only): NORMAL/RECOVERING/TERMINATED
	uint8_t retry_count;  ///< Current retry count (0~5)
	uint16_t timer;
	bool fail_flag;       ///< Fail detection flag
	bool success_flag;    ///< Success detection flag
	bool recovery_result;
} failsafe_information_t;

bool Api_failsafe_clock_check_hfclk_start(void);

bool Api_failsafe_clock_check_lfclk_start(void);

void Api_failsafe_init(void);

void Api_failsafe_watchdog_init(void);

void Api_failsafe_timer_tick(void);

void Api_failsafe_set_fail(FAILSAFE_CODE code);

void Api_failsafe_set_success(FAILSAFE_CODE code);

bool Api_failsafe_isMajor(void);

void Api_failsafe_main(void);

void Api_failsafe_battery_update_level(uint16_t level);

void Api_failsafe_battery_time_update(uint32_t time);

bool Api_failsafe_blocking_system(void);

#endif // API_FAILSAFE_H
