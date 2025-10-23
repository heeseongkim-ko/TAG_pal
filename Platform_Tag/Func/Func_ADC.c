/**
 * @file Func_ADC.c
 * @brief Analog-to-Digital Converter (ADC) implementation for battery voltage measurement
 * @details This module implements SAADC functionality for measuring battery voltage
 *          using a resistive divider circuit on the nRF52840 platform.
 * @author Platform Tag Team
 * @date 2024
 * @version 1.0
 */

#include "Func_ADC.h"
#include "nrf_drv_saadc.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_common.h"

/**
 * @defgroup ADC_Static_Variables Static Variables
 * @{
 */
static uint8_t s_battery_voltage_raw = 0;  /**< Cached battery voltage measurement */
/** @} */

/**
 * @defgroup ADC_Global_Variables Global Variables
 * @{
 */
/** @} */

/**
 * @defgroup ADC_IRQ_Handlers Interrupt Request Handlers
 * @{
 */

/**
 * @brief Handler for interrupt request from ADC
 * @details The ADC is used in polling regime, so this is a dummy function.
 *          No actual interrupt handling is required.
 * @param[in] p_event Pointer to SAADC event structure
 * @return None
 */
void _ADC_IRQ_handler(nrfx_saadc_evt_t const * p_event)
{
    // Dummy function - ADC is used in polling mode
    (void)p_event;
}

/** @} */

/**
 * @defgroup ADC_Initialization ADC Initialization Functions
 * @{
 */

/**
 * @brief Initialize Analog to Digital Converter
 * @details Configures SAADC with 8-bit resolution, no oversampling, and low power mode.
 *          Sets up channel 1 for battery voltage measurement via resistive divider.
 *          Channel 0 is commented out but available for VDD measurement if needed.
 * @return None
 * @note This function must be called before using any ADC functions
 */
void Func_ADC_Init(void)
{
    nrf_drv_saadc_config_t adc_conf = NRF_DRV_SAADC_DEFAULT_CONFIG;
    adc_conf.resolution = NRF_SAADC_RESOLUTION_8BIT;
    adc_conf.oversample = NRF_SAADC_OVERSAMPLE_DISABLED;
    adc_conf.interrupt_priority = SAADC_CONFIG_IRQ_PRIORITY;
    adc_conf.low_power_mode = SAADC_CONFIG_LP_MODE;

    /*
     * Channel 0 configuration (commented out - available for VDD measurement)
     * nrf_saadc_channel_config_t adc_ch0_conf = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(SAADC_CH_PSELP_PSELP_VDD);
     * adc_ch0_conf.gain = NRF_SAADC_GAIN1_6;
     * adc_ch0_conf.acq_time = NRF_SAADC_ACQTIME_40US;
     * nrf_drv_saadc_init(&adc_conf, _ADC_IRQ_handler);
     * nrf_drv_saadc_channel_init(ADC_VDD_channel, &adc_ch0_conf);
     */

    // Configure channel 1 for battery voltage measurement
    nrf_saadc_channel_config_t adc_ch1_conf = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
    adc_ch1_conf.gain = NRF_SAADC_GAIN1_4;  // Optimized gain for battery voltage range
    adc_ch1_conf.acq_time = NRF_SAADC_ACQTIME_40US;
    adc_ch1_conf.reference = SAADC_CH_CONFIG_REFSEL_Internal;
    
    nrf_drv_saadc_init(&adc_conf, _ADC_IRQ_handler);
    nrf_drv_saadc_channel_init(ADC_BATT_channel, &adc_ch1_conf);

    s_battery_voltage_raw = Func_ADC_Measure_Battery_Voltage_Raw();
}

/** @} */

/**
 * @defgroup ADC_Pin_Control ADC Pin Control Functions
 * @{
 */

/**
 * @brief Set the ADC_RDIV pin as output - low state
 * @details Used for battery voltage measurement via resistive divider.
 *          Second resistor must be tied to ground during measurement.
 * @return None
 */
void Func_ADC_RDIV_Pull_Down(void)
{
    nrf_gpio_pin_clear(ADC_RDIV_TEIA);
}

/**
 * @brief Set the ADC_MEAS pin as High Z input
 * @details Should be used during measurement to avoid loading the voltage divider.
 * @return None
 */
void Func_ADC_MEAS_Pin_Bypass(void)
{
    nrf_gpio_cfg_input(ADC_MEAS_TEIA, NRF_GPIO_PIN_NOPULL);
}

/**
 * @brief Set the ADC_MEAS pin as Pull-upped input
 * @details Used for protection against high voltage when not measuring.
 * @return None
 */
void Func_ADC_MEAS_Pin_Pull_Up(void)
{
    nrf_gpio_cfg_input(ADC_MEAS_TEIA, NRF_GPIO_PIN_PULLUP);
}

/**
 * @brief Set the ADC_RDIV pin as Pull-upped input
 * @details Used for energy saving in high Z state when not measuring.
 * @return None
 */
void Func_ADC_RDIV_Pull_Up(void)
{
    nrf_gpio_pin_set(ADC_RDIV_TEIA);
}

/** @} */

/**
 * @defgroup ADC_Measurement ADC Measurement Functions
 * @{
 */

/**
 * @brief Perform measurement on defined ADC channel
 * @param[in] channel ADC channel whose input should be measured (0 or 1)
 * @return Measured value in RAW form (16-bit)
 * @note Channel 1 is configured for battery voltage measurement
 */
uint16_t Func_ADC_One_Shot_Measure(uint8_t channel)
{
    nrf_saadc_value_t meas_value = 0;
    nrf_drv_saadc_sample_convert(channel, &meas_value);
    return (uint16_t)meas_value;
}

/**
 * @brief Measure battery voltage in raw format
 * @details Configures pins, performs measurement, and restores pin states.
 *          Includes voltage correction factor (0.98) for accuracy.
 *          Handles the complete measurement cycle automatically.
 * @return Raw battery voltage value (8-bit, 0-255)
 * @note This function handles all pin configuration automatically
 */
uint8_t Func_ADC_Measure_Battery_Voltage_Raw(void)
{
    uint16_t raw_voltage = 0;

    // Configure pins for measurement
    Func_ADC_RDIV_Pull_Down();      // Battery is connected via resistive divider - second resistor must be tied to ground
    Func_ADC_MEAS_Pin_Bypass();     // Set measurement pin to High-Z input
    
    // Allow settling time
    nrf_delay_ms(1);
    
    // Perform measurement
    raw_voltage = Func_ADC_One_Shot_Measure(ADC_BATT_channel);
    
    // Apply correction factor for accuracy
    raw_voltage = (uint8_t)((float)(raw_voltage - 1) * 0.98f);
    
    // Restore pin states for protection and energy saving
    Func_ADC_MEAS_Pin_Pull_Up();    // Measurement pin must be pulled up to protect against high voltage
    Func_ADC_RDIV_Pull_Up();        // Battery is connected via resistive divider - second resistor in high Z state for energy saving
    
    return raw_voltage;
}

/** @} */

/**
 * @defgroup ADC_Battery_Management ADC Battery Management Functions
 * @{
 */

/**
 * @brief Check battery status and perform measurement if needed
 * @details Implements a counter-based measurement strategy to avoid excessive measurements.
 *          Only measures every 10 calls unless forced by first_measure parameter.
 * @param[in] first_measure Force measurement if true, otherwise use cached value
 * @return Current battery voltage in raw format (8-bit)
 * @note Measurements are cached and only updated every 10 calls unless forced
 */
uint8_t Func_ADC_Check_Battery_Status(bool first_measure)
{
    static uint8_t s_measure_counter = 0;
    
    if ((s_measure_counter >= BATTERY_MEASURE_AFTER_RR) || (first_measure))
    {
        s_battery_voltage_raw = Func_ADC_Measure_Battery_Voltage_Raw();
        s_measure_counter = 0;
    }

    s_measure_counter++;
    
    return s_battery_voltage_raw;
}

/**
 * @brief Get current battery voltage from cache
 * @details Returns the last measured battery voltage without performing new measurement.
 *          Use this function for frequent voltage checks to avoid excessive measurements.
 * @return Cached battery voltage value (8-bit)
 * @note This function returns the last measured value without performing new measurement
 */
uint8_t Func_ADC_Get_Current_Battery_Voltage(void)
{
    return s_battery_voltage_raw;
}

/** @} */


