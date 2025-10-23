/**
 * @file    Api_failsafe.h
 * @brief   Centralized Failsafe Management System
 */

#ifndef API_FAILSAFE_H
#define API_FAILSAFE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ========================================================================================
// Configuration
// ========================================================================================

#define FAILSAFE_LIMIT_COUNT	10u
#define FAILSAFE_ADDRESS		104u

// ========================================================================================
// UWB Fail Codes
// ========================================================================================

typedef enum
{
	UWB_FAIL_NONE = 0,
	UWB_FAIL_INIT_01,
	UWB_FAIL_INIT_02,
	UWB_FAIL_INIT_03,
	UWB_FAIL_TX_01,		///< TX timeout
	UWB_FAIL_TX_02,		///< Device not awake
	UWB_FAIL_WAKEUP_01,
	UWB_FAIL_WAKEUP_02,
	UWB_FAIL_WAKEUP_03,
	UWB_FAIL_WAKEUP_04,
	UWB_FAIL_WAKEUP_05,
	UWB_FAIL_SPI_01,
	UWB_FAIL_SPI_02,
	UWB_FAIL_SPI_03,
	UWB_FAIL_MAX
} uwb_fail_code_e;

// ========================================================================================
// NFC Fail Codes
// ========================================================================================

typedef enum
{
	NFC_FAIL_NONE = 0,
	NFC_FAIL_I2C_TIMEOUT,
	NFC_FAIL_I2C_NACK,
	NFC_FAIL_WRITE_ERROR,
	NFC_FAIL_READ_ERROR,
	NFC_FAIL_MAX
} nfc_fail_code_e;

// ========================================================================================
// TAG_SAFE Codes
// ========================================================================================

typedef enum
{
	TAG_SAFE_01 = 0,
	TAG_SAFE_02,		///< UWB Init failure
	TAG_SAFE_03,		///< UWB Wakeup failure
	TAG_SAFE_04,		///< UWB TX failure
	TAG_SAFE_05,		///< UWB SPI failure
	TAG_SAFE_06,		///< NFC failure
	TAG_SAFE_07,
	TAG_SAFE_08,
	TAG_SAFE_09,
	TAG_SAFE_10,
	TAG_SAFE_11,
	TAG_SAFE_12,
	TAG_SAFE_13,
	TAG_SAFE_MAX
} api_failsafe_code_e;

typedef enum
{
	TAG_FAIL_NONE = 0,
	TAG_FAIL_PASS = 1,
	TAG_FAIL_FAIL = 2
} api_failsafe_result_e;

// ========================================================================================
// State
// ========================================================================================

typedef enum
{
	FAILSAFE_STATE_IDLE = 0,
	FAILSAFE_STATE_TRIGGERED,
	FAILSAFE_STATE_RECOVERY,
	FAILSAFE_STATE_WAIT_RESULT,
	FAILSAFE_STATE_CHECK,
	FAILSAFE_STATE_PERMANENT_FAIL,
	FAILSAFE_STATE_TERMINATED_SYSTEM
} api_failsafe_state_e;

// ========================================================================================
// Public Functions
// ========================================================================================

void Api_failsafe_init(void);
void Api_failsafe_timer_tick(void);
void Api_failsafe_main(void);

// UWB
void Api_failsafe_set_uwb_fail(uwb_fail_code_e code);
uwb_fail_code_e Api_failsafe_get_uwb_fail(void);
bool Api_failsafe_get_uwb_status(void);
api_failsafe_state_e Api_failsafe_get_uwb_state(void);

// NFC
void Api_failsafe_set_nfc_fail(nfc_fail_code_e code);
nfc_fail_code_e Api_failsafe_get_nfc_fail(void);
bool Api_failsafe_get_nfc_status(void);

// Global

bool Api_failsafe_get_activated(void);

bool Api_failsafe_has_permanent_failure(void);

#ifdef __cplusplus
}
#endif

#endif /* API_FAILSAFE_H */
