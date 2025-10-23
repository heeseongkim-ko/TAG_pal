/**
 * @file Api_battery.h
 * @brief Battery voltage measurement API for TEIA platform
 * @details Simple battery voltage measurement using SAADC on P0.31 (BAT1).
 *          Voltage divider ratio: R9/(R8+R9) = 499k/(100k+499k) = 0.833
 *          
 *          Features:
 *          - Non-blocking measurement with state machine
 *          - 12-bit ADC resolution with internal 0.6V reference
 *          - 1/4 gain setting for optimal battery voltage range (0-2.4V input)
 *          - Automatic GPIO configuration and restoration
 *          - 1ms settling time for accurate readings
 *          
 * @author Platform Tag Team
 * @date 2024
 * @version 1.0
 * 
 * @addtogroup BATTERY_API
 * @{
 */

#ifndef API_BATTERY_H
#define API_BATTERY_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup BATTERY_Types Battery Type Definitions
 * @{
 */

/**
 * @brief Battery measurement states
 */
typedef enum
{
	API_BATTERY_STATE_IDLE = 0,		/**< Ready for new measurement */
	API_BATTERY_STATE_BUSY,			/**< ADC conversion in progress */
	API_BATTERY_STATE_READY,		/**< Result ready */
	API_BATTERY_STATE_ERROR			/**< Error occurred */
} Api_battery_state_g;

/** @} */

/**
 * @defgroup BATTERY_Constants Battery Constants
 * @{
 */
#define API_BATTERY_INVALID_VOLTAGE	0xFFFF	/**< Invalid voltage value */
/** @} */

/**
 * @brief SAADC gain value for battery voltage measurement (mV)
 * @note Used to convert SAADC raw values to actual battery voltage
 *       Typically calculated as: (Reference voltage * Gain factor * 1000)
 *       For nRF52: 0.6V reference * 4x gain * 1000 = 2400mV
 */
#define API_BATTERY_SAADC_FULL_SCALE_MV		2400
/** @} */

/**
 * @brief Maximum ADC resolution value (12-bit ADC)
 * @note nRF52840 SAADC has 12-bit resolution: 2^12 = 4096
 */
#define API_BATTERY_SAADC_MAX_VALUE			256

/**
 * @brief Hardware voltage divider ratio for battery measurement
 * @note Battery voltage is divided by 2 through hardware resistor divider
 *       Actual battery voltage = measured voltage * divider ratio
 */
#define API_BATTERY_VOLTAGE_DIVIDER_RATIO	2

/**
 * @brief Maximum allowable battery voltage in millivolts
 * @note Used for voltage clamping to prevent invalid readings
 *       Typical Li-ion/Li-po maximum voltage is around 4.2V
 */
#define API_BATTERY_MAX_VOLTAGE_MV			5000

/**
 * @brief Battery voltage clamping value in millivolts
 * @note When measured voltage exceeds maximum, clamp to this value
 */
#define API_BATTERY_CLAMP_VOLTAGE_MV		5000

/**
 * @defgroup BATTERY_Functions Battery Function Declarations
 * @{
 */

/**
 * @brief Initialize battery measurement system
 * @details Configures SAADC with following settings:
 *          - 12-bit resolution
 *          - Internal 0.6V reference
 *          - 1/4 gain (max input 2.4V)
 *          - Channel 1 on AIN0 pin
 *          - 40us acquisition time
 *          This is a blocking function that must complete successfully
 *          before other battery functions can be used.
 * @return true on successful initialization, false on failure
 * @note Must be called once during system initialization
 * @warning Do not call other battery functions before successful initialization
 */
bool Api_battery_init(void);

/**
 * @brief Uninitialize battery measurement system (for sleep mode)
 * @return true on successful uninitialization, false on failure
 */
bool Api_battery_unInit(void);

/**
 * @brief Main processing function for battery measurement state machine
 * @details Processes battery measurement state machine in non-blocking manner.
 *          Handles state transitions, timeout checking, ADC measurement execution,
 *          and GPIO pin restoration. Must be called regularly from main loop.
 *          
 *          State machine:
 *          - IDLE: Waiting for measurement request
 *          - BUSY: Settling time in progress
 *          - READY: Measurement complete, result available
 *          - ERROR: Error occurred during measurement
 *          
 * @return None
 * @note Call this function regularly from main application loop
 * @see Api_battery_startRead(), Api_battery_getLevel()
 */
void Api_battery_main(void);

/**
 * @brief Timer tick function for battery measurement timing (1ms interval)
 * @details Decrements timeout counter used for settling time and error detection.
 *          Must be called exactly every 1ms from timer interrupt handler.
 *          Only performs counter operations - no logic processing per project rules.
 * @return None
 * @note Call this function from main timer interrupt every 1ms
 * @warning Missing timer ticks will cause measurement timeouts
 */
void Api_battery_timer_tick(void);

/**
 * @brief Get last measured battery voltage level
 * @details Returns the battery voltage from the last completed measurement.
 *          Value is only valid when measurement state is READY.
 *          Voltage calculation accounts for:
 *          - SAADC reference voltage (0.6V internal)
 *          - Gain setting (1/4 gain)
 *          - External voltage divider (1/2 ratio)
 * @return Battery voltage in millivolts (mV), range 0-5000mV
 * @retval API_BATTERY_INVALID_VOLTAGE if measurement not ready or not initialized
 * @note Check Api_battery_state() first to ensure measurement is complete
 * @see Api_battery_state(), Api_battery_startRead()
 */
uint16_t Api_battery_getLevel(void);

/**
 * @brief Start non-blocking battery voltage measurement
 * @details Configures GPIO pins for measurement and starts settling timer.
 *          This function returns immediately and does not block.
 *          
 *          GPIO configuration:
 *          - ADC_RDIV_TEIA: Set low to enable voltage divider
 *          - ADC_MEAS_TEIA: Configure as input for ADC measurement
 *          
 *          Process flow:
 *          1. Configure GPIO pins
 *          2. Start 1ms settling timer
 *          3. State transitions to BUSY
 *          4. Api_battery_main() completes measurement
 *          
 * @return true if measurement started successfully
 * @retval false if already busy, not initialized, or error occurred
 * @note Call Api_battery_main() regularly to process the measurement
 * @note Use Api_battery_getLevel() to retrieve result after completion
 * @see Api_battery_main(), Api_battery_getLevel(), Api_battery_state()
 */
bool Api_battery_startRead(void);

/**
 * @brief Get current battery measurement state
 * @details Returns the current state of the battery measurement state machine.
 *          Can be used by application to check measurement progress and
 *          determine when result is ready.
 *          
 *          States:
 *          - API_BATTERY_STATE_IDLE: Ready for new measurement
 *          - API_BATTERY_STATE_BUSY: Measurement in progress (settling)
 *          - API_BATTERY_STATE_READY: Measurement complete, result available
 *          - API_BATTERY_STATE_ERROR: Error occurred during measurement
 *          
 * @return Current state from Api_battery_state_g enumeration
 * @note Use this to check if measurement is complete before calling Api_battery_getLevel()
 * @see Api_battery_getLevel(), Api_battery_startRead()
 */
Api_battery_state_g Api_battery_state(void);

/** @} */

uint16_t Api_battery_getRawData(void);

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* API_BATTERY_H */

