/**
 * @file Func_ADC.h
 * @brief Analog-to-Digital Converter (ADC) functionality for battery voltage measurement
 * @details This module provides SAADC initialization and battery voltage measurement functions
 *          using the nRF52840's built-in SAADC peripheral.
 * @author Platform Tag Team
 * @date 2024
 * @version 1.0
 */

#ifndef FUNC_ADC_H
#define FUNC_ADC_H

#include <stdint.h>
#include <stdbool.h>
#include "def_pin.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup ADC_Channels ADC Channel Definitions
 * @{
 */
#define ADC_BATT_channel    1   /**< Battery voltage measurement channel */
#define ADC_VDD_channel     0   /**< VDD measurement channel (unused) */
/** @} */

/**
 * @defgroup ADC_Constants ADC Constants
 * @{
 */
#define BATTERY_MEASURE_AFTER_RR     10  /**< Measure battery after every 10 operations */
/** @} */

/**
 * @defgroup ADC_External_Functions External Function Declarations
 * @{
 */
extern void printf_uart(const char* format, ...);  /**< UART printf function */
/** @} */

/**
 * @defgroup ADC_Global_Variables Global Battery Variables
 * @{
 */
/** @} */

/**
 * @defgroup ADC_Functions ADC Function Declarations
 * @{
 */

/**
 * @brief Initialize Analog to Digital Converter
 * @details Configures SAADC with 8-bit resolution, no oversampling, and low power mode.
 *          Sets up channel 1 for battery voltage measurement via resistive divider.
 * @return None
 * @note This function must be called before using any ADC functions
 */
void Func_ADC_Init(void);

/**
 * @brief Set ADC_RDIV pin as output - low state
 * @details Used for battery voltage measurement via resistive divider.
 *          Second resistor must be tied to ground during measurement.
 * @return None
 */
void Func_ADC_RDIV_Pull_Down(void);

/**
 * @brief Set ADC_MEAS pin as High Z input
 * @details Should be used during measurement to avoid loading the voltage divider.
 * @return None
 */
void Func_ADC_MEAS_Pin_Bypass(void);

/**
 * @brief Set ADC_MEAS pin as Pull-upped input
 * @details Used for protection against high voltage when not measuring.
 * @return None
 */
void Func_ADC_MEAS_Pin_Pull_Up(void);

/**
 * @brief Set ADC_RDIV pin as Pull-upped input
 * @details Used for energy saving in high Z state when not measuring.
 * @return None
 */
void Func_ADC_RDIV_Pull_Up(void);

/**
 * @brief Perform one-shot measurement on specified ADC channel
 * @param[in] channel ADC channel to measure (0 or 1)
 * @return Measured value in RAW form (16-bit)
 * @note Channel 1 is configured for battery voltage measurement
 */
uint16_t Func_ADC_One_Shot_Measure(uint8_t channel);

/**
 * @brief Measure battery voltage in raw format
 * @details Configures pins, performs measurement, and restores pin states.
 *          Includes voltage correction factor (0.98) for accuracy.
 * @return Raw battery voltage value (8-bit, 0-255)
 * @note This function handles all pin configuration automatically
 */
uint8_t Func_ADC_Measure_Battery_Voltage_Raw(void);

/**
 * @brief Check battery status and perform measurement if needed
 * @param[in] first_measure Force measurement if true, otherwise use cached value
 * @return Current battery voltage in raw format (8-bit)
 * @note Measurements are cached and only updated every 10 calls unless forced
 */
uint8_t Func_ADC_Check_Battery_Status(bool first_measure);

/**
 * @brief Get current battery voltage from cache
 * @return Cached battery voltage value (8-bit)
 * @note This function returns the last measured value without performing new measurement
 */
uint8_t Func_ADC_Get_Current_Battery_Voltage(void);

/** @} */

#ifdef __cplusplus
}
#endif

#endif // FUNC_ADC_H



