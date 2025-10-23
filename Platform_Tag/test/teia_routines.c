
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h> 
#include <stdlib.h>
#include "nrf_gpio.h"
#include "nrf_delay.h"
//#include "nrfx_spi.h"
//#include "nrfx_spim.h"
#include "nrf_uart.h"
#include "app_uart.h"
//************************************************************************
#include "def_pin.h"
#include "teia_routines.h"
#include "UWB_routines.h"
//************************************************************************khs20250514_add
#include "sdk_config.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"
#include "def_var.h"
#include "nrf_drv_gpiote.h"
#include "dw3000_device_api.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_twi.h"
#include "Teia_sensor.h"
#include "non_volatile_memory.h"
//************************************************************************


const app_uart_comm_params_t comm_params_log = 
      {
          UART_RX_PIN_NUMBER,
          UART_TX_PIN_NUMBER,
          0,//0xFFFFFFFF,//NULL,//RTS_PIN_NUMBER,
          0,//0xFFFFFFFF,//NULL,//CTS_PIN_NUMBER,
          UART_HWFC,
          false,
          NRF_UART_BAUDRATE_115200//NRF_UART_BAUDRATE_57600//NRF_UART_BAUDRATE_38400//NRF_UART_BAUDRATE_115200
      };

void uart_rx_handle(app_uart_evt_t * p_event)
{
}

void UART_LOG_INIT(void)
{
  uint32_t                     err_code;
  APP_UART_FIFO_INIT(&comm_params_log,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_rx_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);

/*
  app_uart_init(&comm_params_log,
                 NULL,
                 uart_rx_handle,//APP_IRQ_PRIORITY_LOWEST,
                 APP_IRQ_PRIORITY_LOW);

  //APP_ERROR_CHECK(err_code);
  */
}
void UART_LOG_UNINIT(void)
{
  app_uart_close();
}
//************************************************************************spi
void SPI_pins_enable(void)
{
    nrf_gpio_pin_clear(DW3000_MOSI_Pin);
    nrf_gpio_cfg_output(DW3000_MOSI_Pin);

    nrf_gpio_pin_clear(DW3000_CLK_Pin);
    nrf_gpio_cfg_output(DW3000_CLK_Pin);

    nrf_gpio_pin_set(DW3000_CS_Pin);
    nrf_gpio_cfg_output(DW3000_CS_Pin);

    nrf_gpio_cfg_input(DW3000_MISO_Pin,NRF_GPIO_PIN_NOPULL);
}

/**
 * @brief Set SPI pins to lowest power mode.
*/
void SPI_pins_disable(void)
{
    nrf_gpio_cfg_input(DW3000_MOSI_Pin,NRF_GPIO_PIN_NOPULL);
   // nrf_gpio_cfg_input(SPI_MOSI,NRF_GPIO_PIN_PULLDOWN);
   nrf_gpio_cfg_input(DW3000_MOSI_Pin,NRF_GPIO_PIN_NOPULL);
  //  nrf_gpio_cfg_input(SPI_MOSI,NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(DW3000_CS_Pin,NRF_GPIO_PIN_PULLUP);
  //nrf_gpio_cfg_input(SPI_CS,NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(DW3000_MISO_Pin,NRF_GPIO_PIN_NOPULL);
}
//************************************************************************peripheral
void LED_ON(uint8_t color)
{
  switch(color)
  {
    case RED : 
      nrf_gpio_pin_clear(LED_RED);
      break;
    case GREEN : 
      nrf_gpio_pin_clear(LED_GREEN);
      break;
    case ORANGE : 
      nrf_gpio_pin_clear(LED_RED);
      nrf_gpio_pin_clear(LED_GREEN);
      break;
    default :
      break;
  }
}

void LED_OFF(uint8_t color)
{
  switch(color)
  {
    case RED : 
      nrf_gpio_pin_set(LED_RED);
      break;
    case GREEN : 
      nrf_gpio_pin_set(LED_GREEN);
      break;
    case ORANGE : 
      nrf_gpio_pin_set(LED_RED);
      nrf_gpio_pin_set(LED_GREEN);
      break;
    default :
      break;
  }
}
//************************************************************************I2C
static const nrf_drv_twi_t i2c = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);     /**< I2C instance definition. */
volatile static uint8_t i2c_transfer_complete = false;                               /**< I2C transfer status flag. */
/**
 * @brief I2C interrupt requests handler.
 * @param *p_event - type of I2C event those require interrupt.
 */
void I2C_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            i2c_transfer_complete = true;
            break;
        default:
            i2c_transfer_complete = true;
            break;
    }
}
/**
 * @brief Initialize I2C periphery and set communication frequency according to @p freq_Hz.
 * @param freq_kHz - SCL frequency in kiloHertz. Usable values are @100, @250 and @400.
 */
void I2C_config(uint16_t freq_kHz)
{
    nrf_gpio_cfg_output(I2C_SCL);
    nrf_gpio_cfg_input(I2C_SDA,NRF_GPIO_PIN_NOPULL);

    for(uint8_t i = 0; i<9; i++)
    {
        nrf_gpio_pin_clear(I2C_SCL);
        nrf_delay_us(5);
        nrf_gpio_pin_set(I2C_SCL);
        nrf_delay_us(5);

    }
    nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
    i2c_config.scl = I2C_SCL;
    i2c_config.sda = I2C_SDA;

    if(freq_kHz == 100) i2c_config.frequency = I2C_FREQ_100k;
    else if(freq_kHz == 250) i2c_config.frequency = I2C_FREQ_250k;
    else if(freq_kHz == 400) i2c_config.frequency = I2C_FREQ_400k;


    nrf_drv_twi_init(&i2c,&i2c_config,I2C_handler,NULL);

    nrf_drv_twi_enable(&i2c);
}

/**
 * @brief Burst read from the multiple registers
 * @param device_addr - Address of I2C device to be read from
 * @param reg_addr - Address of register to be read from
 * @param length - number of bytes to be read
 * @param *data - pointer to read data
 * @return 0x00 if OK, 0xFF if error
 */
uint8_t I2CDRV_burst_readFromRegister(uint8_t device_addr, uint8_t reg_addr,uint8_t length, uint8_t *data )
{

    uint8_t ret = 0;

    i2c_transfer_complete = false;
    ret += nrf_drv_twi_tx(&i2c,device_addr,&reg_addr,1,true);
 
    while(!i2c_transfer_complete);

    i2c_transfer_complete = false;
    ret += nrf_drv_twi_rx(&i2c,device_addr,data,length);
 
    while(!i2c_transfer_complete);
  
    return ret;
}

/**
 * @brief Read from register
 * @param reg Register to be read from
 * @return Readed data
 */
uint8_t I2CDRV_readFromRegister(uint8_t device_addr, uint8_t reg_addr)
{
    uint8_t reg_data;

    i2c_transfer_complete = false;
    nrf_drv_twi_tx(&i2c,device_addr,&reg_addr,1,true);

    while(!i2c_transfer_complete);

    i2c_transfer_complete = false;
    nrf_drv_twi_rx(&i2c,device_addr,&reg_data,1);
    while(!i2c_transfer_complete);
    return reg_data;
}
/**************************************************************************//**
 * @brief Write byte to I2C device

 * @param[in] data
 * 		byte to be written to address
 * @param[in] reg_addr
 * 		address of register to be written to
 *
 * @param[in] data
 * 		byte to be written to address
 *****************************************************************************/
uint8_t I2CDRV_writeToRegister(uint8_t device_addr, uint8_t reg_addr, uint8_t data)
{
    uint8_t tx_buff[2];
    uint8_t ret = 0;
    tx_buff[0] = reg_addr;
    tx_buff[1] = data;

    i2c_transfer_complete = false;
    ret = nrf_drv_twi_tx(&i2c,device_addr,tx_buff,2,false);
    while(!i2c_transfer_complete);

    return ret;
}

/**
 * @brief Burst write to the multiple registers.
 * @param device_addr - Address of I2C device to be write to.
 * @param reg_addr - Address of register to be write to.
 * @param length - number of bytes to be write
 * @param *data - pointer to data that should be written.
 * @return 0x00 if OK, 0xFF if error
 */
uint8_t I2CDRV_burst_writeToRegister(uint8_t device_addr, uint8_t reg_addr,uint8_t length, uint8_t *data )
{

    uint8_t ret = 0;

    uint8_t i2c_data[17];
    i2c_data[0] = reg_addr;
    memcpy(&i2c_data[1],data, length);

    i2c_transfer_complete = false;
    ret = nrf_drv_twi_tx(&i2c,device_addr,i2c_data,length+1,false);
    while(!i2c_transfer_complete);

/*    i2c_transfer_complete = false;
    nrf_drv_twi_tx(&i2c,*data,(data+1),length-1,false);
    while(!i2c_transfer_complete);
*/

    return ret;
}
/**
 * @brief Power ON NFC chip (nt3h2111) and change the I2C frequency to 1OOkHz because of the NFC chip do not work properly at 400kHz.
 */
void NFC_PWR_ON(void)
{
    //nrf_gpio_pin_set(NFC_PWR);
    I2C_config(I2C_FREQ_STD);       /**nt3h2111 work properly at @100kHz*/
    nrf_delay_ms(10);         //wait for power up of NFC/EEPROM chip
}

/**
 * @brief Power OFF NFC chip (nt3h2111). Power saving.
 */
void NFC_PWR_OFF(void)
{
    //nrf_gpio_pin_clear(NFC_PWR);
    I2C_config(I2C_FREQ_FAST);       /**all other chips can work @400kHz*/
}

/**
 * @brief Get state of Field detection pin of NFC chip.
 * @return Return true if pin state is 1 else return false.
 */
 /*
bool get_FD_state(void)
{
    return nrf_gpio_pin_read(NFC_FD);
}
*/
//************************************************************************ADC
/**
 * @brief Handler for interrupt request from ADC.
 * @detailed The ADC is used in polling regime, so this is dummy function.
 */
void _ADC_IRQ_handler(nrf_drv_saadc_evt_t const * p_event)
{

}

/**
 * @brief Function that initialize Analog to digital converter.
 * @detailed The parameters are hardcoded in this function with respect to the application requirements.
 */
void ADC_init(void)
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
void ADC_RDIV_PULL_DOWN(void)
{
    nrf_gpio_pin_clear(ADC_RDIV);
}
/**
 * @brief Set the ADC_MEAS pin as High Z input - it should be used during measure.
 */
void ADC_MEAS_PIN_BYPASS(void)
{
    //NRF_SAADC->CH[1].CONFIG &= ~(11UL);
    nrf_gpio_cfg_input(ADC_MEAS,NRF_GPIO_PIN_NOPULL);
}
/**
 * @brief Set the ADC_MEAS pin as Pull-upped input.
 */
void ADC_MEAS_PIN_PULL_UP(void)
{
   // NRF_SAADC->CH[1].CONFIG |= 2;
    nrf_gpio_cfg_input(ADC_MEAS,NRF_GPIO_PIN_PULLUP);
}
/**
 * @brief Set the ADC_RDIV (second side of resistive divider) pin as Pull-upped input.
 */
void ADC_RDIV_PULL_UP(void)
{
    nrf_gpio_pin_set(ADC_RDIV);
}
/**
 * @brief Function that perform measurement on defined ADC channel.
 * @param channel - ADC channel whose input should be measured.
 * @return Measured value in RAW form.
 */
uint16_t ADC_one_shot_measure(uint8_t channel)
{
    nrf_saadc_value_t meas_value = 0;
    nrf_drv_saadc_sample_convert(channel,&meas_value);
    return (uint16_t)meas_value;
}
uint8_t meas_battery_voltage_raw(void)
{
    uint16_t raw_voltage = 0;//uint8_t raw_voltage = 0;

    ADC_RDIV_PULL_DOWN();                                           //battery is connected via resistive divider - second resistor must be tied to ground
    ADC_MEAS_PIN_BYPASS();                                          //
    nrf_delay_ms(1);
    raw_voltage = ADC_one_shot_measure(ADC_BATT_channel);
    raw_voltage = (uint8_t)((float)(raw_voltage-1)*0.98f);
    //printf("raw_voltag____ = %d \n\r", raw_voltage);//debug
    ADC_MEAS_PIN_PULL_UP();                                        //meas pin must be pulled up to protect against high voltage
    ADC_RDIV_PULL_UP();                                            //battery is connected via resistive divider - second resistor be in high Z state to energy saving
    
    return raw_voltage;
}
void check_batteryStatus(bool first_measure)
{
    if ((systemValues.need_to_measure_battery == MEAS_BATT_AFTER_RR) || (first_measure))		//battery voltage is not measured after every blink
    {
      systemValues.actual_batt_voltage_raw = meas_battery_voltage_raw();
      //printf("raw_voltag____ = %d \n\r", meas_battery_voltage_raw());//debug
      systemValues.need_to_measure_battery = 0;
    }

    systemValues.need_to_measure_battery++;
}
//************************************************************************init
#define NRF52_ONRAM1_OFFRAM1  	POWER_RAM_POWER_S0POWER_On << POWER_RAM_POWER_S0POWER_Pos | POWER_RAM_POWER_S1POWER_On  << POWER_RAM_POWER_S1POWER_Pos      \
                                    | POWER_RAM_POWER_S0RETENTION_On  << POWER_RAM_POWER_S0RETENTION_Pos | POWER_RAM_POWER_S1RETENTION_On  << POWER_RAM_POWER_S1RETENTION_Pos;

#define NRF52_ONRAM1_OFFRAM0    POWER_RAM_POWER_S0POWER_On << POWER_RAM_POWER_S0POWER_Pos | POWER_RAM_POWER_S1POWER_On << POWER_RAM_POWER_S1POWER_Pos      \
                                    | POWER_RAM_POWER_S0RETENTION_Off << POWER_RAM_POWER_S0RETENTION_Pos | POWER_RAM_POWER_S1RETENTION_Off << POWER_RAM_POWER_S1RETENTION_Pos;

#define NRF52_ONRAM0_OFFRAM0    POWER_RAM_POWER_S0POWER_Off<< POWER_RAM_POWER_S0POWER_Pos | POWER_RAM_POWER_S1POWER_Off<< POWER_RAM_POWER_S1POWER_Pos;

void configure_ram_retention(void)
{
		// Configure nRF52 RAM retention parameters. Set for System On 64kB RAM retention
		NRF_POWER->RAM[0].POWER = NRF52_ONRAM1_OFFRAM0;
		NRF_POWER->RAM[1].POWER = NRF52_ONRAM1_OFFRAM0;
		NRF_POWER->RAM[2].POWER = NRF52_ONRAM1_OFFRAM0;
		NRF_POWER->RAM[3].POWER = NRF52_ONRAM1_OFFRAM0;
		NRF_POWER->RAM[4].POWER = NRF52_ONRAM1_OFFRAM0;
		NRF_POWER->RAM[5].POWER = NRF52_ONRAM1_OFFRAM0;
		NRF_POWER->RAM[6].POWER = NRF52_ONRAM1_OFFRAM0;
		NRF_POWER->RAM[7].POWER = NRF52_ONRAM1_OFFRAM0;
};

/**
 * @brief Initialize states of all used GPIOs.
 */
void GPIOs_init(void)
{
  nrf_gpio_cfg_output(LED_RED);
  nrf_gpio_cfg_output(LED_GREEN);
  nrf_gpio_pin_set(LED_RED);
  nrf_gpio_pin_set(LED_GREEN);

}

void MCU_init(void)
{
  configure_ram_retention();
  GPIOs_init();
  SPI_pins_enable();
};
//************************************************************************time_20250514
#define RTC_FREQUENCY       RTC_DEFAULT_CONFIG_FREQUENCY

/**
 * convertTime2Ticks
 *
 * This function converts the time based values to the ticks (RTC values)
 * @param time_us - insert time in microseconds.
 * @return number of RTC  ticks that correspond to inserted time.
 */
uint32_t convertTime2Ticks(uint32_t time_us)
{
	uint32_t ticks = 0;
	ticks = (uint32_t) ((((float) RTC_FREQUENCY * (float) time_us) / 1000000.0)+0.5f);
	return ticks;
}



//************************************************************************init_20250514
volatile uint8_t RTCCNT_MSByte;
volatile bool rtcDelayComplete[3][4]; //

//const nrf_drv_rtc_t rtc0 = NRF_DRV_RTC_INSTANCE(0); /**<Definition of RTC0 instance.*/
const nrf_drv_rtc_t rtc1 = NRF_DRV_RTC_INSTANCE(1); /**<Definition of RTC1 instance.*/
const nrf_drv_rtc_t rtc2 = NRF_DRV_RTC_INSTANCE(2); /**<Definition of RTC2 instance.*/

void (*_app_event_handler)() = (void*)NULL;

void set_app_event_handler(void* handler)
{
    _app_event_handler = handler;
}
/**
 * @brief Handle for all types of RTC2 interrupts.
 * @param int_type - Source of interrupt request.
 */
void _RTC2_evt_handler(nrf_drv_rtc_int_type_t int_type)
{
    switch(int_type)
    {
        case(NRF_DRV_RTC_INT_COMPARE0):
            rtcDelayComplete[2][0]= true;
            NRF_RTC2->EVENTS_COMPARE[0] = 0;
            (*_app_event_handler)();
            break;

        case (NRF_DRV_RTC_INT_COMPARE1):
            rtcDelayComplete[2][1]= true;
            NRF_RTC2->EVENTS_COMPARE[1] = 0;
            break;

        case (NRF_DRV_RTC_INT_COMPARE2):
            rtcDelayComplete[2][2]= true;
            NRF_RTC2->EVENTS_COMPARE[2] = 0;
            break;
/*
        case (NRF_DRV_RTC_INT_COMPARE3):
            rtcDelayComplete[2][3]= true;
            NRF_RTC2->EVENTS_COMPARE[3] = 0;
            PB_handler(PB_TIMEOUT);
            int_flag.wake_up = true;
            break;

*/
        case (NRF_DRV_RTC_INT_OVERFLOW):
            RTCCNT_MSByte++;
            NRF_RTC2->EVENTS_OVRFLW = 0;
            break;

        default:
            break;
    }
    // Clear all other events (also unexpected ones)
    NRF_RTC2->EVENTS_TICK = 0;
}

/**
 * @brief Configuration of RTC2. RTC2 is the main source of timing for the entire application. All parameters are hard coded in Function with respect to application requirements.
 */
void rtc_configuration(void)
{
    uint32_t err_code;

    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    err_code = nrf_drv_rtc_init(&rtc2, &config,_RTC2_evt_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_rtc_enable(&rtc2);                                      //enable rtc2 counter


    nrf_drv_rtc_counter_clear(&rtc2);                               //clear RTC counter value
    nrf_drv_rtc_overflow_enable(&rtc2,true);
    RTCCNT_MSByte = 0UL;                                            //clear RTC MSByte
}

void clock_configuration_test(void)
{
  uint32_t err_code;
  err_code = nrf_drv_clock_init();
  APP_ERROR_CHECK(err_code);
  if(!nrf_drv_clock_lfclk_is_running())
  {
      nrf_drv_clock_lfclk_request(NULL);
      //printf("clock_init__err = %d\n\r",nrf_drv_clock_lfclk_is_running());
  }
}

bool WDOG_running = false;  /**<State of WDOG timer (true if WDOG is enabled)*/
volatile bool NFC_should_be_check;
uint32_t timeout_tcks = 0;  /**<Timeout of WDOG represented as number of RTC ticks.*/
/**
 * @brief Initialize pseudo watchdog (WDOG is emulated using one of RTCs)
 * @param rr_us - Halftime of watchdog timer overflow.
 */
/**
 * @brief Real time counter 1 - Interrupt handler. This RTC instance is used to WDOG emulation and NFC timing.
 * @param int_type - Interrupt source.
 */
void _RTC1_evt_handler(nrf_drv_rtc_int_type_t int_type)
{
    switch(int_type)
    {
        case(NRF_DRV_RTC_INT_COMPARE0):                 //wdt stage
            NRF_RTC1->EVENTS_COMPARE[0] = 0;
            NVIC_SystemReset();
            break;
        case(NRF_DRV_RTC_INT_COMPARE1):                 //nfc field detection stage
            NRF_RTC1->EVENTS_COMPARE[1] = 0;
            NFC_should_be_check = true;
            int_flag.wake_up = true;
            break;
        /*
        case(NRF_DRV_RTC_INT_COMPARE2):                 //low battery signalization stage
            NRF_RTC1->EVENTS_COMPARE[2] = 0;
            Low_battery_signalization = true;
            int_flag.wake_up = true;
            break;
            */
        case(NRF_DRV_RTC_INT_COMPARE3):                 //dw_temperature measurement timing stage
            NRF_RTC1->EVENTS_COMPARE[3] = 0;
            dw_temperature_measure = true;
            break;
            
        default:
            break;
    }

}

void shedule_time_of_next_measurement(void)
{
    RTC_CC_set(TEMP_MEAS_RTC_instance,TEMP_MEAS_RTC_chan, get_RTC_counter(TEMP_MEAS_RTC_instance)+DW_TEMPERATURE_MEAS_PERIOD_TCKS, true);
}

/**
 * @brief Get state of Real time counter.
 * @param RTC_instance - number Real time counter instance whole counter state is required.
 * @return Current state of selecter RTC counter register.
 */
uint32_t get_RTC_counter(uint8_t RTC_instance)
{
    uint32_t ret = ~0UL;
    if(RTC_instance == 0)
    {
        //ret = nrf_drv_rtc_counter_get(&rtc0);
    }
    else if(RTC_instance == 1)
    {
        ret = nrf_drv_rtc_counter_get(&rtc1);
    }
    else if(RTC_instance == 2)
    {
        ret = nrf_drv_rtc_counter_get(&rtc2);
    }
    return ret;
}

/**
 * @brief Set new value to RTC Counter Compare register.
 * @param RTC_instance - number of RTC instance for which it should be applied.
 * @param channel - Compare channel - there is more then one compare register for each counter.
 * @param val - New value for selected compare register. Max value is 0xFFFFFF, higher value will be masked in lower level function.
 * @param IRQ_en - Interrupt request enable, true if IRQ is required on compare.
 */
void RTC_CC_set(uint8_t RTC_instance,uint32_t channel, uint32_t val, bool IRQ_en)
{
    if(RTC_instance == 0)
    {
        //NRF_RTC0->EVENTS_COMPARE[channel] = 0;
        //nrf_drv_rtc_cc_set(&rtc0,channel,val,IRQ_en);
    }
    else if(RTC_instance == 1)
    {
        NRF_RTC1->EVENTS_COMPARE[channel] = 0;
        nrf_drv_rtc_cc_set(&rtc1,channel,val,IRQ_en);
    }
    else if(RTC_instance == 2)
    {
        NRF_RTC2->EVENTS_COMPARE[channel] = 0;
        nrf_drv_rtc_cc_set(&rtc2,channel,val,IRQ_en);
    }
}

/**
 * convertTime2Ticks
 *
 * @brief This function return actual state of RTC.
 * @param r_32bit - false if 24-bit timestamp is required, true if 32-bit timestamp is required.
 * @return RTC_timestamp.
 */
uint32_t get_RTC_timestamp(bool r_32bit)
{
    uint32_t RTC_ticks;
    RTC_ticks = get_RTC_counter(RTC_delay_instance);
    if(r_32bit)
    RTC_ticks |= (RTCCNT_MSByte << 24);

    return(RTC_ticks);
}

/**
 * @brief get_RTCelapsedCount - get the time which elapsed since timestamp
 * @param earlyCNT - the input parameter which holds the old RTC state value (timestamp)
 * @return - the elapsed RTC counts from input @p earlyCNT to actual RTC count when this function was called
 */
uint32_t get_RTCelapsedCount(uint32_t earlyCNT)
{
    uint32_t cnt = get_RTC_timestamp(true);

    return (cnt - earlyCNT);
}

/**
 * @brief set_sleep_time - set the RTC compare register to value @p earlyCNT + @p tcks.
 * @param earlyCNT - the input parameter which holds the old RTC state value (timestamp)
 */
uint8_t set_sleep_time(uint32_t tcks, uint32_t early)
{
    uint8_t ret;
    if(tcks <= 1) return 0;
    else
    {
        if(tcks == 2) ret = 1;
        else ret = 2;
        rtcDelayComplete[RTC_delay_instance][deep_sleep_CC_reg]=false;
        RTC_CC_set(RTC_delay_instance,deep_sleep_CC_reg,get_RTC_counter(RTC_delay_instance)+tcks,true);
    }
    return ret;

}

/**
 * @brief Disable interrupt request from RTC that is dedicated for deep sleep mode.
 */
 void deep_sleep_RTC_IRQ_disable(void)
{
        RTC_CC_set(RTC_delay_instance,deep_sleep_CC_reg,0,false);
}

//---------------------------------------------------------------------------------------
#define  WDT_RTC_instance       1
#define  WDT_RTC_chan           0
#define RTC_TOP             0x00FFFFFFUL  //maximum value of RTC counter

void watchdog_init(uint32_t rr_us)
{
    uint32_t err_code;
    nrf_drv_rtc_uninit(&rtc1);
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.interrupt_priority = 2;                                  //highest priority
    err_code = nrf_drv_rtc_init(&rtc1, &config,_RTC1_evt_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_rtc_enable(&rtc1);                                      //enable rtc1 counter


    nrf_drv_rtc_counter_clear(&rtc1);                               //clear RTC counter value
    timeout_tcks = convertTime2Ticks(rr_us*2UL + WD_GUARD_INTERVAL_US);
    uint32_t rtccnt_val = get_RTC_counter(WDT_RTC_instance);
    RTC_CC_set(WDT_RTC_instance,WDT_RTC_chan, (rtccnt_val + timeout_tcks) & RTC_TOP, true);
    WDOG_running = true;
}

/**
 * @brief Feed the pseudo watch dog.
 */
void WDOG_Feed(void)
{
    if(WDOG_running)
    {
        uint32_t rtccnt_val = get_RTC_counter(WDT_RTC_instance);
        RTC_CC_set(WDT_RTC_instance,WDT_RTC_chan, (rtccnt_val + timeout_tcks) & RTC_TOP, true);
    }

}

/**
 * @brief Get watchdog state (en/dis).
 * @return True if WDOG is enabled, false if WDOG is disabled.
 */
bool wdog_get_state(void)
{
    return WDOG_running;
}

/**
 * @brief Change timeout of pseudo watchdog (WDOG is emulated using one of RTCs)
 * @param rr_us - Halftime of watchdog timer overflow.
 */
void watchdog_set_timeout(uint32_t rr_us)
{
    timeout_tcks = convertTime2Ticks(rr_us*2UL + WD_GUARD_INTERVAL_US);
    uint32_t rtccnt_val = get_RTC_counter(WDT_RTC_instance);
    RTC_CC_set(WDT_RTC_instance,WDT_RTC_chan, (rtccnt_val + timeout_tcks) & RTC_TOP, true);
    WDOG_running = true;
}

/**
 * @brief Enable/disable wdog timer (pseudo wdog timer based on RTC).
 * @param en - True for enable WDOG, false for disable WDOG.
 */
void WDOG_enable(bool en)
{
    if(en)
    {
        uint32_t rtccnt_val = get_RTC_counter(WDT_RTC_instance);
        RTC_CC_set(WDT_RTC_instance,WDT_RTC_chan, (rtccnt_val + timeout_tcks) & RTC_TOP, true);
        WDOG_running = true;

    }
    else
    {
        uint32_t rtccnt_val = get_RTC_counter(WDT_RTC_instance);
       RTC_CC_set(WDT_RTC_instance,WDT_RTC_chan, (rtccnt_val + timeout_tcks) & RTC_TOP, false);
       //nrf_drv_rtc_cc_disable(&rtc1,WDT_RTC_chan);
       WDOG_running = false;
    }

}
//------------------------------------------------------------------------------
/**
 * @brief Initialize states of all used GPIOs.
 */
void GPIOs_init_test(void)
{
    nrf_gpio_cfg_output(LED_RED);
    nrf_gpio_cfg_output(LED_GREEN);
    nrf_gpio_pin_set(LED_RED);
    nrf_gpio_pin_set(LED_GREEN);
  //configure SPI pins
    nrf_gpio_cfg_output(DW3000_MOSI_Pin);
    nrf_gpio_cfg_output(DW3000_CLK_Pin);
    nrf_gpio_cfg_output(DW3000_CS_Pin);
    nrf_gpio_cfg_input(DW3000_MISO_Pin,NRF_GPIO_PIN_NOPULL);

    nrf_gpio_cfg_input(DW3000_IRQ_Pin,NRF_GPIO_PIN_PULLDOWN);
    nrf_gpio_input_disconnect(DW3000_RST_Pin);
    //sensor
    //configure ACC irq pin as input
    nrf_gpio_cfg_input(ACC_IRQ,NRF_GPIO_PIN_NOPULL);

    //configure FD (field detect) pin as input
    nrf_gpio_cfg_input(NFC_FD,NRF_GPIO_PIN_PULLUP);
    //configure ADC_RDIV
    nrf_gpio_cfg_output(ADC_RDIV);
    nrf_gpio_pin_set(ADC_RDIV);

    nrf_gpio_cfg_input(ADC_MEAS,NRF_GPIO_PIN_PULLUP);
}

void _GPIO_IRQ_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  if(ext_irq_en.dw_irq)
    {
        while(nrf_gpio_pin_read(DW3000_IRQ_Pin))
        {
            dwt_isr();
        }
    }

  if(ext_irq_en.acc_irq)
    {
        if(nrf_gpio_pin_read(ACC_IRQ))
        {
            int_flag.motion_action_detected = true;
        }
    }


    if(ext_irq_en.dw_rst)
    {
        if(nrf_gpio_pin_read(DW3000_RST_Pin))
        {
            int_flag.DW_XTAL_stable =true;
        }
    }
/*
    if(ext_irq_en.nfc_fd)
    {
        if(!nrf_gpio_pin_read(NFC_FD))
        {
            //uint32_t timestamp = get_RTC_counter(WDT_RTC_instance);
            //RTC_CC_set(WDT_RTC_instance,WDT_RTC_NFC_chan,timestamp + DELAY_BEFORE_NEW_CONFIG_CHECK_TCKS,true);
            //printf("NFC_DEBUGE\n\r");
        }
    }
    */
}

bool ext_irq_init(void)
{
    uint32_t err_code = 0UL;
    if(!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
    }
    else return(false);


    // init interupt pin DW_IRQ//
    if(!err_code)
    {
        nrf_drv_gpiote_in_config_t gpiote_in_config;
        gpiote_in_config.hi_accuracy = false;
        gpiote_in_config.is_watcher = false;
        gpiote_in_config.sense = NRF_GPIOTE_POLARITY_LOTOHI;
        gpiote_in_config.pull = NRF_GPIO_PIN_PULLDOWN;
        err_code = nrf_drv_gpiote_in_init(DW3000_IRQ_Pin, &gpiote_in_config, _GPIO_IRQ_handler);
        nrf_drv_gpiote_in_event_enable(DW3000_IRQ_Pin, true);
        ext_irq_en.dw_irq = true; 
    }
    else return(false);

    // init interupt pin NFC_FIELD//
    /*
    if(!err_code)
    {
        nrf_drv_gpiote_in_config_t gpiote_in_config;
        gpiote_in_config.hi_accuracy = false;                                                    //low power mode
        gpiote_in_config.is_watcher = false;
        gpiote_in_config.sense = NRF_GPIOTE_POLARITY_TOGGLE;                                    //Nordic workaround for low power mode IRQ
        gpiote_in_config.pull = NRF_GPIO_PIN_PULLUP;                                            //second terminal of pushbutton is grounded
        err_code = nrf_drv_gpiote_in_init(NFC_FD, &gpiote_in_config, _GPIO_IRQ_handler);    //init CHG interrupt request
        //nrf_drv_gpiote_in_event_enable(NFC_FD,true);                                       //CHG interrupt must be disabled after reset
        ext_irq_en.nfc_fd = false;//true;
        NFC_should_be_check = false;                                                                  //clear PB pressed variables after reset - just in case

    }
    else return(false);
*/
     // init interupt pin ACCELEROMETER//
    if(!err_code)
    {
        nrf_drv_gpiote_in_config_t gpiote_in_config;
        gpiote_in_config.hi_accuracy = false;                                                   //low power mode
        gpiote_in_config.is_watcher = false;
        gpiote_in_config.sense = NRF_GPIOTE_POLARITY_TOGGLE;                                    //Nordic workaround for low power mode IRQ
        gpiote_in_config.pull = NRF_GPIO_PIN_NOPULL;                                          //ACC interrupt pin is active high
        err_code = nrf_drv_gpiote_in_init(ACC_IRQ, &gpiote_in_config, _GPIO_IRQ_handler);       //init pushbutton interrupt request
        nrf_drv_gpiote_in_event_enable(ACC_IRQ,false);                                          //accelerometer interrupt must be disabled after reset
        ext_irq_en.acc_irq = false;
    }
    else return(false);


       // init interupt pin PUSHBUTTON//
/*
    if(!err_code)
    {
        nrf_drv_gpiote_in_config_t gpiote_in_config;
        gpiote_in_config.hi_accuracy = false;                                                    //low power mode
        gpiote_in_config.is_watcher = false;
        gpiote_in_config.sense = NRF_GPIOTE_POLARITY_TOGGLE;                                    //Nordic workaround for low power mode IRQ
        gpiote_in_config.pull = NRF_GPIO_PIN_PULLUP;                                            //second terminal of pushbutton is grounded
        err_code = nrf_drv_gpiote_in_init(PUSHBUTTON, &gpiote_in_config, _GPIO_IRQ_handler);    //init PB interrupt request
        nrf_drv_gpiote_in_event_enable(PUSHBUTTON,false);                                       //PB interrupt must be disabled after reset
        ext_irq_en.pushbutton = false;

        PB_long_pressed = false;
        PB_short_pressed_3t = false;                                                            //clear PB pressed variables after reset - just in case
    }
    else return(false);
*/
    return (true);

}

//////////////////////////** SPI peripheral function **/////////////////////////////////////////////////////////
#define SPI_INSTANCE                        0 /**< SPI instance index. */
#define SPI_DEFAULT_CONFIG_IRQ_PRIORITY     6 /**<Lowest priority. */
volatile uint16_t baudrate;


static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */

 /**
 * @brief Initialize SPI.
 * @param freq_kHz - Frequency that should be used for SPI communication in kiloHertz. Inserted value will be rounded to nearest lower value.
 */
uint16_t SPI_init(uint16_t freq_kHz)
{
    nrf_drv_spi_uninit(&spi);
    uint16_t ret = 0;
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.miso_pin = DW3000_MISO_Pin;
    spi_config.mosi_pin =  DW3000_MOSI_Pin;
    spi_config.sck_pin = DW3000_CLK_Pin;
    spi_config.ss_pin = DW3000_CS_Pin;
  //  spi_config.ss_pin = NRF_DRV_SPI_PIN_NOT_USED;
    if      (freq_kHz>= 8000)  {spi_config.frequency = NRF_DRV_SPI_FREQ_8M;  ret = 8000;}
    else if (freq_kHz>= 4000)  {spi_config.frequency = NRF_DRV_SPI_FREQ_4M;  ret = 4000;}
    else if (freq_kHz>= 2000)  {spi_config.frequency = NRF_DRV_SPI_FREQ_2M;  ret = 2000;}
    else if (freq_kHz>= 1000)  {spi_config.frequency = NRF_DRV_SPI_FREQ_1M;  ret = 1000;}
    else if (freq_kHz>= 500)   {spi_config.frequency = NRF_DRV_SPI_FREQ_500K;ret = 500;}
    else if (freq_kHz>= 250)   {spi_config.frequency = NRF_DRV_SPI_FREQ_250K;ret = 250;}
    else                       {spi_config.frequency = NRF_DRV_SPI_FREQ_125K;ret = 125;}
    //nrf_drv_spi_init(&spi,&spi_config,spi_event_handler);
    nrf_drv_spi_init(&spi,&spi_config,NULL,NULL);
    return ret;
}

//************************************************************************

#define MAX_SPI_XFER_SIZE 512  // 원하는 최대 크기 설정
static uint8_t spi_tx_buf[MAX_SPI_XFER_SIZE];
static uint8_t spi_rx_buf[MAX_SPI_XFER_SIZE];

int writetospi(uint16_t headerLength, uint8_t *headerBuffer,
               uint16_t bodyLength, uint8_t *bodyBuffer)
{
    uint32_t total_len = headerLength + bodyLength;
    if (total_len > MAX_SPI_XFER_SIZE) return DWT_ERROR;

    memcpy(spi_tx_buf, headerBuffer, headerLength);
    memcpy(spi_tx_buf + headerLength, bodyBuffer, bodyLength);

    nrf_gpio_pin_clear(DW3000_CS_Pin);
/*
    const nrfx_spim_xfer_desc_t xfer_desc = {
        .p_tx_buffer = spi_tx_buf,
        .tx_length   = total_len,
        .p_rx_buffer = NULL,
        .rx_length   = 0
    };

    ret_code_t err_code = nrfx_spim_xfer(&spim, &xfer_desc, 0);
*/
    ret_code_t err_code = nrf_drv_spi_transfer(
        &spi,
        spi_tx_buf, total_len,
        NULL, 0
    );

    nrf_gpio_pin_set(DW3000_CS_Pin);

    return (err_code == NRF_SUCCESS) ? DWT_SUCCESS : DWT_ERROR;
}


int readfromspi(uint16_t headerLength, uint8_t *headerBuffer,
                uint16_t readLength, uint8_t *readBuffer)
{
    uint32_t total_len = headerLength + readLength;
    if (total_len > MAX_SPI_XFER_SIZE) return DWT_ERROR;

    memset(spi_tx_buf, 0, total_len);
    memcpy(spi_tx_buf, headerBuffer, headerLength);

    memset(spi_rx_buf, 0, total_len);

    nrf_gpio_pin_clear(DW3000_CS_Pin);
/*
    const nrfx_spim_xfer_desc_t xfer_desc = {
        .p_tx_buffer = spi_tx_buf,
        .tx_length   = total_len,
        .p_rx_buffer = spi_rx_buf,
        .rx_length   = total_len
    };

    ret_code_t err_code = nrfx_spim_xfer(&spim, &xfer_desc, 0);
*/

    ret_code_t err_code = nrf_drv_spi_transfer(
        &spi,
        spi_tx_buf, total_len,
        spi_rx_buf, total_len
    );

    nrf_gpio_pin_set(DW3000_CS_Pin);

    if (err_code != NRF_SUCCESS) return DWT_ERROR;

    memcpy(readBuffer, spi_rx_buf + headerLength, readLength);

    return DWT_SUCCESS;
}


// ---------------------------------------------------------------------------
//
// NB: The purpose of this file is to provide for microprocessor interrupt enable/disable, this is used for
//     controlling mutual exclusion from critical sections in the code where interrupts and background
//     processing may interact.  The code using this is kept to a minimum and the disabling time is also
//     kept to a minimum, so blanket interrupt disable may be the easiest way to provide this.  But at a
//     minimum those interrupts coming from the decawave device should be disabled/re-enabled by this activity.
//
//     In porting this to a particular microprocessor, the implementer may choose to use #defines in the
//     deca_irq.h include file to map these calls transparently to the target system.  Alternatively the
//     appropriate code may be embedded in the functions provided below.
//
//     This mutex dependent on HW port.
//     If HW port uses EXT_IRQ line to receive ready/busy status from DW3000 then mutex should use this signal
//     If HW port not use EXT_IRQ line (i.e. SW polling) then no necessary for decamutex(on/off)
//
// ---------------------------------------------------------------------------

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: decamutexon()
 *
 * Description: This function should disable interrupts. This is called at the start of a critical section
 * It returns the irq state before disable, this value is used to re-enable in decamutexoff call
 *
 * Note: The body of this function is defined in deca_mutex.c and is platform specific
 *
 * input parameters:
 *
 * output parameters:
 *
 * returns the state of the DW3000 interrupt
 */
decaIrqStatus_t decamutexon(void)
{
/* NRF chip has only 1 IRQ for all GPIO pins.
 * Disablin of the NVIC would not be of the best ideas.
 */
    decaIrqStatus_t s = nrf_drv_gpiote_in_is_set(DW3000_IRQ_Pin);//current_irq_pin
    if(s)
    {
        nrf_drv_gpiote_in_event_disable(DW3000_IRQ_Pin);
    }
    return s;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: decamutexoff()
 *
 * Description: This function should re-enable interrupts, or at least restore their state as returned(&saved) by decamutexon
 * This is called at the end of a critical section
 *
 * Note: The body of this function is defined in deca_mutex.c and is platform specific
 *
 * input parameters:
 * @param s - the state of the DW3000 interrupt as returned by decamutexon
 *
 * output parameters:
 *
 * returns the state of the DW3000 interrupt
 */
void decamutexoff(decaIrqStatus_t s) // put a function here that re-enables the interrupt at the end of the critical section
{
    if (s)
    {
        nrf_drv_gpiote_in_event_enable(DW3000_IRQ_Pin, true);//current_irq_pin
    }
}

void MCU_init_test(void)
{
  clock_configuration_test(); ///initialize clock module
  rtc_configuration();  ///initialize Real time counters
  //watchdog_init(WDOG_INTERVAL_DURING_FW_INIT);       ///init WDOG timer, set time to 6 sec before tdoa is inited (input parameter is doubled inside function)
  configure_ram_retention();
  GPIOs_init_test();
  //ext_irq_init();                 ///init interrupts from external peripherals
  SPI_pins_enable();              ///enable SPI pins - it is in High Z by defoult - becouse of power saving
  baudrate = SPI_init(SPI_BAUDRATE_LOW);                 ///init SPI with clk frequency 1 MHz
  ext_irq_init();                 ///init interrupts from external peripherals
  I2C_config(100);                ///init I2C with clk frequency 100 kHz
  nt3h2111_test1();
  ADC_init();                     ///init Analog to digital converter for battery measurement
  //init_app_timer_handler();
  nrf_delay_ms(10);                   ///wait until any init procedures is done - just in case
  printf("---MCU_INIT---\n\r");     //khs_debug
}
/*
void restore_NT3H2211_block0(void)
{
    uint8_t block0_init[16] = {
        0x04, 0x8A, 0x6E, 0x02,
        0xB3, 0x1E, 0x90, 0x00,
        0x44, 0x00, 0x03, 0x00,
        0x00, 0x00, 0x00, 0x00
    };
    uint8_t block0_readback[16] = {0};

    I2CDRV_burst_writeToRegister(0x55, 0x03, 16, block0_init);//(NT3H2211_ADDR, 0x00, 16, block0_init);
     // Read-back
     nrf_delay_ms(10);
    I2CDRV_burst_readFromRegister(0x55, 0x03, 16, block0_readback);

    // 결과 확인용 출력
    printf("Read back block 0: ");
    for (int i = 0; i < 16; i++) {
        printf("%02X ", block0_readback[i]);
    }
    printf("\n\r");
}
*/
bool nt3h2111_test1(void)
{
	uint8_t page[16]={0xff};
	I2CDRV_burst_readFromRegister(0x55,0,16,page);
        printf("nt3h2211_test_print = %d \n\r", page[0]);
        nrf_delay_us(100); 
	//if (page[0]== NT3H_WHO_AM_I_VAL) return true;
	//else return false;
}

void HW_init_test(void)
{
  SPI_pins_enable();                  ///DWM module is connected via SPI bus which is disabled by default
  DWM1000_initAndDoHwResetProcedure();
  DWM1000_init();

  sens_check_and_standby();
  //read_sensors_data();
  //NFC_PWR_ON();
 // restore_NT3H2211_block0();
  //nrf_delay_ms(50);
  //nt3h2111_test1();
}
/**
 * @brief Disable I2C peripheral, and set both I2C pins as High Z input.
 */
void I2C_disable(void)
{
    nrf_drv_twi_disable(&i2c);
    nrf_gpio_cfg_input(I2C_SCL,NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(I2C_SDA,NRF_GPIO_PIN_NOPULL);

}
//************************************************************************init_20250515_sleep
#define RTC_delay_instance 2
#define delay_CC_reg 1
/**
 * @brief Blocking delay - timing is based on RTC.
 * @param t - Delay time.
 * @param is_ms - If true - the specified time will be considered as milliseconds else it will be considered as microseconds.
 * @param use_deep_sleep - If true, the MCU will be in deep_sleep (HF clock will be halted) during delay, else HF clock still running.
 */
void delay_and_sleep(uint32_t t, bool is_ms,bool use_deep_sleep)
{
    rtcDelayComplete[RTC_delay_instance][delay_CC_reg] = false;

    uint32_t time_us = (is_ms) ? (t * 1000UL) : (t);
    uint32_t ticks = convertTime2Ticks(time_us);

    if(ticks<= 1)
    {
        nrf_delay_us(time_us);
        return;
    }

    uint32_t rtccnt = (get_RTC_counter(RTC_delay_instance)) & RTC_TOP;
    RTC_CC_set(RTC_delay_instance,delay_CC_reg,(rtccnt+ticks)& RTC_TOP,true);

    if(use_deep_sleep)
    {
        while(rtcDelayComplete[RTC_delay_instance][delay_CC_reg] == false)
        {
            __WFI();
        }
    }
    else
    {
        while(!rtcDelayComplete[RTC_delay_instance][delay_CC_reg])__NOP();
    }
    return;

}

//************************************************************************data_20250515
/**
 * @brief This function load predefined tag parameters.
 */
void loadTagParameters(void)
{
/*
  tag_info_t loadedData;
	if(load_TagInfo(&loadedData)== RET_OK)  		//load saved setting from flash
    {
        systemValues.fw_version = loadedData.fw_version;
        systemValues.fw_subversion = loadedData.fw_subversion;
        systemValues.fw_revision = loadedData.fw_revision;

        systemValues.hw_version = loadedData.hw_version;
        systemValues.hw_revision = loadedData.hw_revision;

        systemValues.platform = loadedData.platform;

        memcpy((void*)&systemValues.MAC_addr,(void*)&loadedData.MAC_addr,sizeof(systemValues.MAC_addr));

    }
    else do_SWResetMCU(0);
*/
  systemValues.fw_version = FW_VERSION;
  systemValues.fw_subversion = FW_SUBVER;
  systemValues.fw_revision = FW_REVISION;

  systemValues.hw_version = HW_VERSION;
  systemValues.hw_revision = HW_REVISION;

  systemValues.platform = PLATFORM;
}

/**
 * @brief This function initialize the capabilities/limitations of the running platform.
 */
void platform_init(void)
{
    uint8_t platform = systemValues.platform;

    if((platform == TEIA_TOOL)||(platform == TEIA_CAR))
    {
        systemValues.min_batt_voltage_raw = MIN_BATT_V_LIPO_BATT_RAW;
        systemValues.max_batt_voltage_raw = MAX_BATT_V_LIPO_BATT_RAW;
        systemValues.low_batt_voltage_raw = LOW_BATT_V_LIPO_BATT_RAW;
        systemValues.min_batt_voltage_for_DFU_raw = MIN_BATT_V_FOR_DFU_LIPO_BATT_RAW;
        systemValues.periodical_rx_possibility = true;
        systemValues.min_rr_ms = MIN_RR_MS_LIPO_BATT;
        systemValues.max_rr_ms = MAX_RR_MS;
        systemValues.min_rrsm_ms = MIN_RRSM_MS_LIPO_BATT;
        systemValues.max_rrsm_ms = MAX_RRSM_MS;
    }
}

static volatile optional_functionality_t optional_func;


void get_optional_func(optional_functionality_t* values)
{
    memcpy((void*)values,(void*)&optional_func,sizeof(optional_functionality_t));
}


void set_optional_func(optional_functionality_t* values)
{
    memcpy((void*)&optional_func,(void*)values, sizeof(optional_functionality_t));
}

void init_default_optional_func(void)
{
       optional_functionality_t default_opt_func;

       default_opt_func.LED_Low_batt_sig = LOW_BAT_SIGNAL_DEFAULT;
       default_opt_func.PB_enable = PB_ENABLE_DEFAULT;
       default_opt_func.PB_enter_DFU = PB_ENTER_DFU_DEFAULT;
       default_opt_func.PB_enter_UWB_config = PB_ENTER_UWB_CONFIG_DEFAULT;
       default_opt_func.PB_factory_reset = PB_FACTORY_RESET_DEFAULT;
       default_opt_func.PB_special_blink_on_press = PB_SPECIAL_BLINK_ON_PRESS_DEFAULT;
       default_opt_func.PB_tag_switch_on_off = PB_ON_OFF_DEFAULT;
       set_optional_func(&default_opt_func);
}

void use_default_config(void)
{
    tdoaParameters.channel = CHANNEL_DEFAULT;
    tdoaParameters.RF_profile = RF_PROFILE_DEFAULT;
    tdoaParameters.data_rate = DR_MODE_DEFAULT;
    tdoaParameters.preamble = PREAMBLE_DEFAULT;
    tdoaParameters.prf = PRF_DEFAULT;
    tdoaParameters.preamCode = PREAM_CODE_DEFAULT;
    tdoaParameters.nSfd = NSFD_DEFAULT;
    tdoaParameters.use_random_deviation = RANDOM_DEVIATION_DEFAULT;
    tdoaParameters.motion_control_mode = SLEEP_MODE_DEFAULT;
    sensors_params.motion_control_mode = SLEEP_MODE_DEFAULT;
    sensors_params.wakeup_threshold = ACC_SENS_DEFAULT;
    tdoaParameters.refresh_rate_ms = REFRESH_RATE_DEFAULT_LIPO_BATT;
    tdoaParameters.no_motion_refresh_rate = NO_MOT_REFRESH_RATE_DEFAULT;
    tdoaParameters.tx_pwr_level_conf = TX_LEVEL_DEFAULT;
    sensors_params.EBlink_cont = EBLINK_DEFAULT;
    sensors_params.sens_corr_data = EBLINK_DEFAULT;
    sensors_params.AHRS_enable = AHRS_DEFAULT;
    sensors_params.acc_FS = ACC_FS_DEFAULT;
    sensors_params.gyro_FS = GYRO_FS_DEFAULT;
    sensors_params.BARO_setting = BARO_DEFAULT;
    sensors_params.mag_calib_mode = MAG_CALIB_MODE_DEFAULT;
    sensors_data.geo_mag_flux = 0;

    uint16_t dummy[3] = {0,0,0};
    memcpy(&sensors_data.mag_offsets,dummy, sizeof(sensors_data.mag_offsets));
    memcpy(&sensors_data.acc_offsets,dummy, sizeof(sensors_data.acc_offsets));
    memcpy(&sensors_data.gyro_offsets,dummy, sizeof(sensors_data.gyro_offsets));

    init_default_optional_func();

    tdoaParameters.tx_pwr_correction = TX_POWER_CORRECTION_DEFAULT;

    tdoaParameters.bc_period_ri = BC_PERIOD_RI_DEFAULT;
    tdoaParameters.bc_period_rism = BC_PERIOD_RISM_DEFAULT;
    tdoaParameters.bc_tx2rx_time = BC_TX2RX_TIME_DEFAULT;
    tdoaParameters.uwb_protocol =  UWB_PROTOCOL_DEFAULT;

    return;
}
/*
uint8_t calc_RF_profile(tdoaParameters_t* params)
{

  
    bool STD = true;
    if(params->prf == PRF_16)
    {
        if     ((params->channel == 1) && (params->preamCode == 1)) STD = true;
        else if((params->channel == 2) && (params->preamCode == 3)) STD = true;
        else if((params->channel == 3) && (params->preamCode == 5)) STD = true;
        else if((params->channel == 4) && (params->preamCode == 7)) STD = true;
        else if((params->channel == 5) && (params->preamCode == 3)) STD = true;
        else if((params->channel == 7) && (params->preamCode == 7)) STD = true;
        else STD = false;

        if(STD)
        {
            if      ((params->data_rate == DR_110k) && (params->preamble == PREAM_L_1024) && (params->nSfd == nSFD_nSTD)) params->RF_profile = 0;
            else if ((params->data_rate == DR_850k) && (params->preamble == PREAM_L_256) && (params->nSfd == nSFD_nSTD)) params->RF_profile = 1;
            else if ((params->data_rate == DR_6M8) && (params->preamble == PREAM_L_128) && (params->nSfd == nSFD_STD)) params->RF_profile = 2;
            else if ((params->data_rate == DR_110k) && (params->preamble == PREAM_L_2048) && (params->nSfd == nSFD_nSTD)) params->RF_profile = 6;
            else STD = false;
        }
    }
    else if(params->prf == PRF_64)
    {
        if     ((params->channel == 1) && (params->preamCode == 9)) STD = true;
        else if((params->channel == 2) && (params->preamCode == 10)) STD = true;
        else if((params->channel == 3) && (params->preamCode == 11)) STD = true;
        else if((params->channel == 4) && (params->preamCode == 20)) STD = true;
        else if((params->channel == 5) && (params->preamCode == 12)) STD = true;
        else if((params->channel == 7) && (params->preamCode == 17)) STD = true;
        else STD = false;

        if(STD)
        {
            if      ((params->data_rate == DR_110k) && (params->preamble == PREAM_L_1024) && (params->nSfd == nSFD_nSTD)) params->RF_profile = 3;
            else if ((params->data_rate == DR_850k) && (params->preamble == PREAM_L_256) && (params->nSfd == nSFD_nSTD)) params->RF_profile = 4;
            else if ((params->data_rate == DR_6M8) && (params->preamble == PREAM_L_128) && (params->nSfd == nSFD_STD)) params->RF_profile = 5;
            else if ((params->data_rate == DR_110k) && (params->preamble == PREAM_L_2048) && (params->nSfd == nSFD_nSTD)) params->RF_profile = 7;
            else STD = false;
        }
    }

    if(!STD) params->RF_profile = 9;

    return params->RF_profile;

}
*/

uint8_t calc_RF_profile(tdoaParameters_t* params)
{

  
    bool STD = true;
    if(params->prf == PRF_16)
    {
    
        if((params->channel == 5) && (params->preamCode == 3)) STD = true;
        else STD = false;

        if(STD)
        {
            if      ((params->data_rate == DR_110k) && (params->preamble == PREAM_L_1024) && (params->nSfd == nSFD_nSTD)) params->RF_profile = 0;
            else if ((params->data_rate == DR_850k) && (params->preamble == PREAM_L_256) && (params->nSfd == nSFD_nSTD)) params->RF_profile = 1;
            else if ((params->data_rate == DR_6M8) && (params->preamble == PREAM_L_128) && (params->nSfd == nSFD_STD)) params->RF_profile = 2;
            else if ((params->data_rate == DR_110k) && (params->preamble == PREAM_L_2048) && (params->nSfd == nSFD_nSTD)) params->RF_profile = 6;
            else STD = false;
        }
    }
    else if(params->prf == PRF_64)
    {
        if((params->channel == 5) && (params->preamCode == 12)) STD = true;
        else STD = false;

        if(STD)
        {
            if      ((params->data_rate == DR_110k) && (params->preamble == PREAM_L_1024) && (params->nSfd == nSFD_nSTD)) params->RF_profile = 3;
            else if ((params->data_rate == DR_850k) && (params->preamble == PREAM_L_256) && (params->nSfd == nSFD_nSTD)) params->RF_profile = 4;
            else if ((params->data_rate == DR_6M8) && (params->preamble == PREAM_L_128) && (params->nSfd == nSFD_STD)) params->RF_profile = 5;
            else if ((params->data_rate == DR_110k) && (params->preamble == PREAM_L_2048) && (params->nSfd == nSFD_nSTD)) params->RF_profile = 7;
            else STD = false;
        }
    }

    if(!STD) params->RF_profile = 9;

    return params->RF_profile;

}

void loadSetting(void)
{
  use_default_config();
  calc_RF_profile(&tdoaParameters);
}

/**
 *
 * @brief Calculate refresh rate.
 */
uint32_t calc_rtls_refreshrate(void)
{
    tdoaTiming.RefreshRate_ms = tdoaParameters.refresh_rate_ms;
    tdoaTiming.RefreshRate_no_mot_ms = tdoaParameters.no_motion_refresh_rate;


    //wrong IMU data from MPU9250 workaround - compensation of delay after MPU wakeup
    /*
    if(sensors_params.ACC_mounted & MPU9250)
    {
        if(tdoaTiming.RefreshRate_no_mot_ms > MPU9250_WAKEUP_DELAY_MS)
        {
            tdoaTiming.RefreshRate_no_mot_ms -= MPU9250_WAKEUP_DELAY_MS;
        }
    }
*/
    if (tdoaParameters.use_random_deviation) // if random deviation is enabled by user, than calculate the deviation value
    {
            uint32_t auto_random_rr;
            auto_random_rr = tdoaParameters.refresh_rate_ms	- 10;


            if (auto_random_rr > 1) // if random RR is applicable
            {
                    tdoaTiming.RefreshRate_us = tdoaParameters.refresh_rate_ms * 1000; //convert to microseconds
                    tdoaTiming.RefreshRate_tck = convertTime2Ticks(tdoaTiming.RefreshRate_us);
                    tdoaTiming.RRwaitTime_tck = tdoaTiming.RefreshRate_tck;
                    tdoaTiming.random_rr_deviation_tck = convertTime2Ticks(auto_random_rr * 1000);
                    tdoaTiming.random_rr_deviation_half_tck = tdoaTiming.random_rr_deviation_tck / 2;
                    tdoaTiming.RRwaitTime_tck -= tdoaTiming.random_rr_deviation_half_tck; //minus RANDOM_RR_DEVIATION/2 ms because random RR value is enabled, which generates 0 - RANDOM_RR_DEVIATION ms


                    tdoaTiming.RefreshRate_no_mot_tck  = convertTime2Ticks(tdoaTiming.RefreshRate_no_mot_ms * 1000);

            }
            else
            {
                    tdoaParameters.use_random_deviation = 0; //set it to 0, because the user settings of RR does not allow any random deviation

                    tdoaTiming.RefreshRate_us = tdoaParameters.refresh_rate_ms * 1000; //convert to microseconds
                    tdoaTiming.RefreshRate_tck = convertTime2Ticks(tdoaTiming.RefreshRate_us);

                    tdoaTiming.RefreshRate_no_mot_tck =  convertTime2Ticks(tdoaTiming.RefreshRate_no_mot_ms * 1000);
                    tdoaTiming.RRwaitTime_tck = tdoaTiming.RefreshRate_tck;
            }

    }
    else
    {
            tdoaTiming.RefreshRate_us = tdoaParameters.refresh_rate_ms * 1000; //convert to microseconds
            tdoaTiming.RefreshRate_tck = convertTime2Ticks(tdoaTiming.RefreshRate_us);
            tdoaTiming.RRwaitTime_tck = tdoaTiming.RefreshRate_tck;
            tdoaTiming.RefreshRate_no_mot_tck = convertTime2Ticks(tdoaTiming.RefreshRate_no_mot_ms * 1000);
    }
    tdoaTiming.RR_calculated = tdoaTiming.RRwaitTime_tck; // save the calculated RR
    srand(tdoaParameters.this_tag_MACaddress);			  //random seed for random deviation
    tdoaTiming.RRwaitTime_tck_actual = ONE_SEC_RTC_TCKS;  //at start 3 infoblinks are sending with 1 sec period

    return 0;
}

uint64_t MAC_address;



#define PARTID_SECONDS_MASK     (0x3FULL)
#define PARTID_MINUTES_MASK     (0x1F80ULL)
#define PARTID_HOURS_MASK       (0x7C000ULL)
#define PARTID_DAYS_MASK        (0x1FF00000ULL)
#define PARTID_LOADBOARD_MASK   (0xC0000000ULL)

/**
* @brief Compose the MAC address of the tag using LOT_ID and PART_ID of the DW1000.
* @param[out] MAC_addr - composed MAC address
*/
void compose_MAC_address(uint64_t* MAC_addr)
{
    uint32_t lotid, partid;
    //wake_up_radio();
    DMW1000_wake_up_test();
    dw_read_IDs(&lotid, &partid);                                 //read LOT_ID and PART_ID from DW1000

    /*4 bits in partid are always zero, it can be compressed from 32 to 28 bits*/
    uint32_t compressed_partid = 0;
    compressed_partid |= partid & PARTID_SECONDS_MASK;
    compressed_partid |= (partid & PARTID_MINUTES_MASK) >>1;
    compressed_partid |= (partid & PARTID_HOURS_MASK) >> 2;
    compressed_partid |= (partid & PARTID_DAYS_MASK) >> 3;
    compressed_partid |= (partid & PARTID_LOADBOARD_MASK) >> 4;

    /*composed MAC address consists of compressed partid and lowest 2.5 bytes of lotid*/
    *MAC_addr = ((uint64_t) lotid << 32) | (uint64_t) compressed_partid;    //compose MAC address
    *MAC_addr &= MAC_ADDR_44b_MASK;                                         //mask out highest nibble of 48bit MAC address
    *MAC_addr |= MAC_ADDR_COMPOSITION_MARK;                                 //add MAC address composition version to the highest nibble
}

/**
 * @brief init_MAC_address- function that initialize MAC address of the tag.
 */
static void init_MAC_address(uint64_t* MAC_address)
{
    //if(!load_predef_MAC_addr(MAC_address)) return;  //try to load predefined MAC address (genarated by Sewio) from memory
    //else compose_MAC_address(MAC_address);          //else (if it is not saved in memory) compose it from DW1000 ID
    compose_MAC_address(MAC_address);
}


/**
 * @brief Initialize TDOA parameters like timing, content of blinks etc.
 */
void init_system_params(void)
{
    WDOG_Feed();
    calc_rtls_refreshrate();
    init_MAC_address(&tdoaParameters.this_tag_MACaddress);

    //init_TX_gain();//20250605

    initialize_mac_headers(&tdoaParameters.this_tag_MACaddress);
    init_tag_info_payload();
    //app_initializer();

    tdoaTiming.no_motion_delay = ACC_TIME_TO_WAIT_INMOTION / tdoaTiming.RefreshRate_ms;	  //calculate number cycles

    tdoaTiming.random_dev_en = tdoaParameters.use_random_deviation;
    tdoaValues.RR_counter = 0;
    tdoaValues.on_start_info_msg = NUM_OF_IBLINK_SENDIG_AT_THE_BEGIN;
    tdoaTiming.last_Blink_timestamp = get_RTC_timestamp(true);
    dw_temperature_measure = true;
}


/**
 * @brief Blocking delay - timing is based on RTC.
 * @param ticks - Delay time as number of RTC ticks.
 * @param early - RTC timestamp from which the end of the delay is calculated.
 * @param use_deep_sleep - If true, the MCU will be in deep_sleep (HF clock will be halted) during delay, else HF clock still runnign.
 */
uint32_t delay_and_sleep_tck(uint32_t ticks, uint32_t early, bool use_deep_sleep)
{
    rtcDelayComplete[RTC_delay_instance][delay_CC_reg] = false;
    uint32_t rtccnt = get_RTC_timestamp(true);
    uint32_t elapsedTicks = (rtccnt - early);
    uint32_t ticks_in_sleep = 0;
    if(elapsedTicks >= ticks)
    {
        return ticks_in_sleep;
    }
    else
    {
        uint32_t remainingTicks = ticks - elapsedTicks;

        if(remainingTicks<= 2UL)
        {
            nrf_delay_us(30*ticks);
            return 1UL;
        }
        else if(remainingTicks >= RTC_TOP)
        {
            RTC_CC_set(RTC_delay_instance,delay_CC_reg,(rtccnt-1) & RTC_TOP,true);
            remainingTicks -= RTC_TOP;
            ticks_in_sleep += RTC_TOP;
        }
        else
        {
            RTC_CC_set(RTC_delay_instance,delay_CC_reg,(rtccnt + remainingTicks) & RTC_TOP,true);
            ticks_in_sleep += remainingTicks;
            remainingTicks = 0;
        }

        while((!rtcDelayComplete[RTC_delay_instance][delay_CC_reg]) && (int_flag.wake_up != true))
        {
            if(use_deep_sleep) __WFI();
            else               __NOP();
        }

        if(remainingTicks) delay_and_sleep_tck(remainingTicks,get_RTC_timestamp(true),use_deep_sleep);
        else return ticks_in_sleep;
    }
    return ticks_in_sleep;

}

/**
 * @brief Generate pseudo-random number with semi-uniform distribution in interval O - @p max_num.
 * @param max_num - Maximal number that should be generated
 */
uint32_t uniform_rand(uint32_t max_num)
{
    if (max_num == 0) return 0;
    if (max_num == RAND_MAX) return rand();
    if (max_num > RAND_MAX) return ~0UL;
    uint32_t width = RAND_MAX / max_num;
    uint32_t rand_max = width * max_num;
    uint32_t random = rand()% rand_max;
    random = random / width;
    return random;
}

/**
 * @brief Calculate random deviation of Refresh rate.
 * @param deviation_max - maximal deviation of refresh rate.
 * @return Random deviation in range <-deviation_max; deviation_max>.
 */
int32_t calculate_dev(uint32_t deviation_max)
{
    if(tdoaParameters.use_random_deviation)
    {
        int32_t deviation = uniform_rand(deviation_max << 1);
        deviation -= deviation_max;
        return deviation;
    }
    else return 0UL;

}

/**
 * @brief This function set MCU to sleep state for time defined by refresh rate
 */
void TAG_sleep_STD(void)    //standard sleep of MCU - called if AHRS is disable
{
	WDOG_Feed();

	if(CHG_active) return;
        uint32_t rtc_before = get_RTC_counter(true);
	if (tdoaTiming.random_dev_en)
	{
        //generate new actual random deviation value
        int32_t random = calculate_dev(tdoaTiming.random_rr_deviation_tck)+ (int32_t)tdoaTiming.random_rr_deviation_half_tck;

        //calculate actual wait time
        tdoaTiming.RRwaitTime_tck_actual = (uint32_t)(tdoaTiming.RRwaitTime_tck + random);

	}
	else //if (!random_dev_enabled)
	{
		tdoaTiming.RRwaitTime_tck_actual = tdoaTiming.RRwaitTime_tck;
	}
    
    delay_and_sleep_tck(tdoaTiming.RRwaitTime_tck_actual,tdoaTiming.last_Blink_timestamp, true);   //enter MCU into sleep mode for defined time
   
    uint32_t rtc_after = get_RTC_counter(true);
    printf("Slept ticks: %lu (expected ~%lu)\n\r",
           (rtc_after - rtc_before) & 0x00FFFFFF,
           tdoaTiming.RRwaitTime_tck_actual);
    
    int_flag.wake_up = false;
    tdoaTiming.last_Blink_timestamp =get_RTC_timestamp(true);											//get timestamp of wake up - that is used for next sleep

return;
}

void LED_Blink(uint8_t  color, uint32_t ontime )
{
	switch( color ){
		case RED:
			//RED_ON(), nrf_delay_ms(ontime), RED_OFF();
			break;
		case GREEN:
			//GREEN_ON(), nrf_delay_ms(ontime), GREEN_OFF();
			break ;
		case ORANGE:
			//ORANGE_ON(), nrf_delay_ms(ontime), ORANGE_OFF();
			break;
		case BLACK:
			//ORANGE_OFF() ;
			break;
	}
}

void charging_led_state(uint8_t battery_value, bool flag_led_toggle)
{
    if(flag_led_toggle == true)
    {
        if(GREEN_LIMIT_POWER < battery_value)
        {
            //LED_Blink(GREEN,10);
            LED_ON(GREEN);
            LED_OFF(RED);
        }

        else if ((ORANGE_LIMIT_POWER < battery_value))
        {
            //LED_Blink(ORANGE,10);
            LED_ON(ORANGE);
        }

        else LED_OFF(GREEN),
            LED_ON(RED);//LED_Blink(RED,10);
        //LED_ON_STATE = false;
    }
}
