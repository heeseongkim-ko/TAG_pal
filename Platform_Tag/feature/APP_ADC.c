#include "APP_ADC.h"
#include "nrf_drv_saadc.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_common.h"

// Static variables for battery measurement
static uint8_t s_battery_voltage_raw = 0;

/************************************************************************ADC
/**
 * @brief Handler for interrupt request from ADC.
 * @detailed The ADC is used in polling regime, so this is dummy function.
 */
void _ADC_IRQ_handler(nrfx_saadc_evt_t const * p_event)
{

}

/**
 * @brief Function that initialize Analog to digital converter.
 * @detailed The parameters are hardcoded in this function with respect to the application requirements.
 */
void APP_ADC_Init(void)
{
    nrf_drv_saadc_config_t adc_conf= NRF_DRV_SAADC_DEFAULT_CONFIG;
    adc_conf.resolution = NRF_SAADC_RESOLUTION_8BIT;
    adc_conf.oversample = NRF_SAADC_OVERSAMPLE_DISABLED;
    adc_conf.interrupt_priority = SAADC_CONFIG_IRQ_PRIORITY;
    adc_conf.low_power_mode = SAADC_CONFIG_LP_MODE;
/*
    nrf_saadc_channel_config_t adc_ch0_conf = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(SAADC_CH_PSELP_PSELP_VDD);
    adc_ch0_conf.gain = NRF_SAADC_GAIN1_6;
    adc_ch0_conf.acq_time = NRF_SAADC_ACQTIME_40US;
    nrf_drv_saadc_init(&adc_conf,_ADC_IRQ_handler);

    nrf_drv_saadc_channel_init(ADC_VDD_channel,&adc_ch0_conf);
*/
    nrf_saadc_channel_config_t adc_ch1_conf = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
    adc_ch1_conf.gain = NRF_SAADC_GAIN1_4;//NRF_SAADC_GAIN1_6;//NRF_SAADC_GAIN1_4;
    adc_ch1_conf.acq_time = NRF_SAADC_ACQTIME_40US;
    adc_ch1_conf.reference = SAADC_CH_CONFIG_REFSEL_Internal;
    nrf_drv_saadc_init(&adc_conf,_ADC_IRQ_handler);
    nrf_drv_saadc_channel_init(ADC_BATT_channel,&adc_ch1_conf);
}

/**
 * @brief Set the ADC_RDIV (second side of resistive divider) pin as output - low state.
 */
void APP_ADC_RDIV_Pull_Down(void)
{
    nrf_gpio_pin_clear(ADC_RDIV_TEIA);
}

/**
 * @brief Set the ADC_MEAS pin as High Z input - it should be used during measure.
 */
void APP_ADC_MEAS_Pin_Bypass(void)
{
    //NRF_SAADC->CH[1].CONFIG &= ~(11UL);
    nrf_gpio_cfg_input(ADC_MEAS_TEIA,NRF_GPIO_PIN_NOPULL);
}

/**
 * @brief Set the ADC_MEAS pin as Pull-upped input.
 */
void APP_ADC_MEAS_Pin_Pull_Up(void)
{
   // NRF_SAADC->CH[1].CONFIG |= 2;
    nrf_gpio_cfg_input(ADC_MEAS_TEIA,NRF_GPIO_PIN_PULLUP);
}

/**
 * @brief Set the ADC_RDIV (second side of resistive divider) pin as Pull-upped input.
 */
void APP_ADC_RDIV_Pull_Up(void)
{
    nrf_gpio_pin_set(ADC_RDIV_TEIA);
}

/**
 * @brief Function that perform measurement on defined ADC channel.
 * @param channel - ADC channel whose input should be measured.
 * @return Measured value in RAW form.
 */
uint16_t APP_ADC_One_Shot_Measure(uint8_t channel)
{
    nrf_saadc_value_t meas_value = 0;
    nrf_drv_saadc_sample_convert(channel,&meas_value);
    return (uint16_t)meas_value;
}

uint8_t APP_ADC_Measure_Battery_Voltage_Raw(void)
{
    uint16_t raw_voltage = 0;//uint8_t raw_voltage = 0;

    APP_ADC_RDIV_Pull_Down();                                           //battery is connected via resistive divider - second resistor must be tied to ground
    APP_ADC_MEAS_Pin_Bypass();                                          //
    nrf_delay_ms(1);
    raw_voltage = APP_ADC_One_Shot_Measure(ADC_BATT_channel);
    //printf_uart("raw_voltag____ = %d \n\r", raw_voltage);//debug
    raw_voltage = (uint8_t)((float)(raw_voltage-1)*0.98f);
    //printf("raw_voltag____ = %d \n\r", raw_voltage);//debug
    APP_ADC_MEAS_Pin_Pull_Up();                                        //meas pin must be pulled up to protect against high voltage
    APP_ADC_RDIV_Pull_Up();                                            //battery is connected via resistive divider - second resistor be in high Z state to energy saving
    
    return raw_voltage;
}

uint8_t APP_ADC_Check_Battery_Status(bool first_measure)
{
    static uint8_t s_measure_counter = 0;
    
    if ((s_measure_counter >= BATTERY_MEASURE_AFTER_RR) || (first_measure))
    {
      s_battery_voltage_raw = APP_ADC_Measure_Battery_Voltage_Raw();
      s_measure_counter = 0;
    }

    s_measure_counter++;
    
    return s_battery_voltage_raw;
}

uint8_t APP_ADC_Get_Current_Battery_Voltage(void)
{
    return s_battery_voltage_raw;
}


