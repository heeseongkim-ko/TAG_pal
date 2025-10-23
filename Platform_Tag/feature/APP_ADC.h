#ifndef APP_ADC_H
#define APP_ADC_H

#include <stdint.h>
#include <stdbool.h>
#include "def_pin.h"

#ifdef __cplusplus
extern "C" {
#endif

// ADC channel definitions
#define ADC_BATT_channel    1   // Battery voltage measurement channel (restored to original)
#define ADC_VDD_channel     0   // VDD measurement channel (restored to original)

// Battery measurement constants
#define BATTERY_MEASURE_AFTER_RR     10  // Measure battery after every 10 operations

// Function declarations
extern void printf_uart(const char* format, ...);

/**
 * @brief Initialize Analog to Digital Converter
 * @details Configures SAADC with 8-bit resolution, no oversampling, and low power mode
 */
void APP_ADC_Init(void);

/**
 * @brief Set ADC_RDIV pin as output - low state
 * @details Used for battery voltage measurement via resistive divider
 */
void APP_ADC_RDIV_Pull_Down(void);

/**
 * @brief Set ADC_MEAS pin as High Z input
 * @details Should be used during measurement
 */
void APP_ADC_MEAS_Pin_Bypass(void);

/**
 * @brief Set ADC_MEAS pin as Pull-upped input
 * @details Used for protection against high voltage
 */
void APP_ADC_MEAS_Pin_Pull_Up(void);

/**
 * @brief Set ADC_RDIV pin as Pull-upped input
 * @details Used for energy saving in high Z state
 */
void APP_ADC_RDIV_Pull_Up(void);

/**
 * @brief Perform one-shot measurement on specified ADC channel
 * @param channel ADC channel to measure
 * @return Measured value in RAW form (8-bit)
 */
uint16_t APP_ADC_One_Shot_Measure(uint8_t channel);

/**
 * @brief Measure battery voltage in raw format
 * @details Configures pins, performs measurement, and restores pin states
 * @return Raw battery voltage value (8-bit)
 */
uint8_t APP_ADC_Measure_Battery_Voltage_Raw(void);

/**
 * @brief Check battery status and perform measurement if needed
 * @param first_measure Force measurement if true
 * @return Current battery voltage in raw format (8-bit)
 */
uint8_t APP_ADC_Check_Battery_Status(bool first_measure);

/**
 * @brief Get current battery voltage
 * @return Current battery voltage in raw format (8-bit)
 */
uint8_t APP_ADC_Get_Current_Battery_Voltage(void);

#ifdef __cplusplus
}
#endif

#endif // APP_ADC_H



