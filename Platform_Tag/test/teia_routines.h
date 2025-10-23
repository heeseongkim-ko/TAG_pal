#ifndef TEIA_ROUTINES_H_INCLUDED
#define TEIA_ROUTINES_H_INCLUDED

#include "dw3000_device_api.h"
#include "nrf_drv_twi.h"

//extern volatile bool dw_temperature_measure;
//volatile bool blink_sent;
//************************************************************************uart
void UART_LOG_INIT(void);
void UART_LOG_UNINIT(void);
//************************************************************************spi
void SPI_pins_enable(void);
void SPI_pins_disable(void);
//************************************************************************ADC
void ADC_init(void);
void ADC_RDIV_PULL_DOWN(void);
void ADC_MEAS_PIN_BYPASS(void);
void ADC_MEAS_PIN_PULL_UP(void);
void ADC_RDIV_PULL_UP(void);
uint16_t ADC_one_shot_measure(uint8_t channel);
uint8_t meas_battery_voltage_raw(void);
void check_batteryStatus(bool first_measure);
//************************************************************************I2C
void I2C_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);
void I2C_config(uint16_t freq_kHz);
void I2C_disable(void);
uint8_t I2CDRV_burst_readFromRegister(uint8_t device_addr, uint8_t reg_addr,uint8_t length, uint8_t *data );
uint8_t I2CDRV_readFromRegister(uint8_t device_addr, uint8_t reg_addr);
uint8_t I2CDRV_writeToRegister(uint8_t device_addr, uint8_t reg_addr, uint8_t data);
uint8_t I2CDRV_burst_writeToRegister(uint8_t device_addr, uint8_t reg_addr,uint8_t length, uint8_t *data );
void NFC_PWR_ON(void);
void NFC_PWR_OFF(void);
bool get_FD_state(void);
//************************************************************************peripheral
void LED_ON(uint8_t color);
void LED_OFF(uint8_t color);
void LED_Blink(uint8_t  color, uint32_t ontime );
void charging_led_state(uint8_t battery_value, bool flag_led_toggle);
//************************************************************************khs20250514_add
uint32_t get_RTC_timestamp(bool r_32bit);
uint32_t get_RTCelapsedCount(uint32_t earlyCNT);
void RTC_CC_set(uint8_t RTC_instance,uint32_t channel, uint32_t val, bool IRQ_en);
uint32_t get_RTC_counter(uint8_t RTC_instance);
void shedule_time_of_next_measurement(void);
void deep_sleep_RTC_IRQ_disable(void);
uint32_t delay_and_sleep_tck(uint32_t ticks, uint32_t early, bool use_deep_sleep);
//************************************************************************khs20250514_add
void configure_ram_retention(void);
void clock_configuration_test(void);
void MCU_init_test(void);
void HW_init_test(void);
void delay_and_sleep(uint32_t t, bool is_ms,bool use_deep_sleep);
uint8_t set_sleep_time(uint32_t tcks, uint32_t early);
void WDOG_Feed(void);
bool wdog_get_state(void);
void watchdog_set_timeout(uint32_t rr_us);
void WDOG_enable(bool en);
uint16_t SPI_init(uint16_t freq_kHz);
int writetospi(uint16_t headerLength, uint8_t *headerBuffer,
               uint16_t bodyLength, uint8_t *bodyBuffer);
int readfromspi(uint16_t headerLength, uint8_t *headerBuffer,
                uint16_t readLength, uint8_t *readBuffer);
decaIrqStatus_t decamutexon(void);
void decamutexoff(decaIrqStatus_t s);
//************************************************************************
void loadTagParameters(void);
void platform_init(void);
void use_default_config(void);
void loadSetting(void);
void loadSetting(void);
void loadTagParameters(void);
void platform_init(void);
uint32_t convertTime2Ticks(uint32_t time_us);
//void get_optional_func(optional_functionality_t* values);
//void set_optional_func(optional_functionality_t* values);
void init_default_optional_func(void);
void use_default_config(void);
//uint8_t calc_RF_profile(tdoaParameters_t* params);
uint32_t calc_rtls_refreshrate(void);
void compose_MAC_address(uint64_t* MAC_addr);
static void init_MAC_address(uint64_t* MAC_address);
void init_system_params(void);
void TAG_sleep_STD(void);
//************************************************************************
bool nt3h2111_test1(void);
#endif  /* TEIA_ROUTINES_H_INCLUDED */

