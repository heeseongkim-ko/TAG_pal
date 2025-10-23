/**
 * @file Api_sleep_peripheral.h
 * @brief Peripheral disable API for low power sleep mode
 * @details Provides functions to disable all MCU peripherals before entering sleep mode
 *          to minimize power consumption. This includes UART, SPI, I2C, TIMER, PWM, ADC,
 *          and other peripherals.
 *          
 *          Features:
 *          - Disable all communication peripherals (UART/SPI/I2C)
 *          - Disable all timing peripherals (TIMER/PWM)
 *          - Disable analog peripherals (SAADC)
 *          - Disable other peripherals (GPIOTE/PPI/QDEC/COMP)
 *          - Optional high frequency clock release
 *          
 * @author Platform Tag Team
 * @date 2024
 * @version 1.0
 * 
 * @addtogroup SLEEP_PERIPHERAL_API
 * @{
 */

#ifndef API_SLEEP_PERIPHERAL_H
#define API_SLEEP_PERIPHERAL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup SLEEP_PERIPHERAL_Functions Sleep Peripheral Function Declarations
 * @{
 */

/**
 * @brief Disable all UART/UARTE instances before sleep
 * @details Stops and disables UART0, UARTE0, and UARTE1 (if available).
 *          Ensures all pending transmissions are stopped before disabling.
 *          
 *          Disabled instances:
 *          - UART0 (legacy UART)
 *          - UARTE0 (UART with EasyDMA)
 *          - UARTE1 (UART with EasyDMA, nRF52840 only)
 *          
 * @return None
 * @note Safe to call even if UARTs are already disabled
 * @warning UART communication will not work after this call until re-initialized
 */
void Api_sleep_peripheral_uart_disable_all(void);

/**
 * @brief Disable all SPI/SPIM/SPIS instances before sleep
 * @details Stops and disables all SPI master/slave instances including EasyDMA versions.
 *          Ensures ongoing transfers are stopped before disabling.
 *          
 *          Disabled instances:
 *          - SPI0, SPI1, SPI2 (legacy SPI)
 *          - SPIM0, SPIM1, SPIM2, SPIM3 (SPI master with EasyDMA)
 *          - SPIS0, SPIS1, SPIS2 (SPI slave with EasyDMA)
 *          
 * @return None
 * @note Safe to call even if SPI peripherals are already disabled
 * @warning SPI communication will not work after this call until re-initialized
 */
void Api_sleep_peripheral_spi_disable_all(void);

/**
 * @brief Disable all TWI/TWIM/TWIS (I2C) instances before sleep
 * @details Stops and disables all I2C master/slave instances including EasyDMA versions.
 *          Sends stop condition on active buses before disabling.
 *          
 *          Disabled instances:
 *          - TWI0, TWI1 (legacy TWI/I2C)
 *          - TWIM0, TWIM1 (TWI master with EasyDMA)
 *          - TWIS0, TWIS1 (TWI slave with EasyDMA)
 *          
 * @return None
 * @note Safe to call even if I2C peripherals are already disabled
 * @warning I2C communication will not work after this call until re-initialized
 */
void Api_sleep_peripheral_twi_disable_all(void);

/**
 * @brief Disable all TIMER instances before sleep
 * @details Stops and clears all hardware timers.
 *          
 *          Disabled instances:
 *          - TIMER0, TIMER1, TIMER2, TIMER3, TIMER4
 *          
 * @return None
 * @note RTC timers are not affected by this function
 * @warning APP_TIMER and other timer-dependent functions will not work after this call
 */
void Api_sleep_peripheral_timer_disable_all(void);

/**
 * @brief Disable all PWM instances before sleep
 * @details Stops and disables all PWM modules.
 *          
 *          Disabled instances:
 *          - PWM0, PWM1, PWM2, PWM3
 *          
 * @return None
 * @note Safe to call even if PWM modules are already disabled
 * @warning PWM outputs will stop after this call
 */
void Api_sleep_peripheral_pwm_disable_all(void);

/**
 * @brief Disable SAADC before sleep
 * @details Stops and disables the Successive Approximation ADC.
 *          Ensures ongoing conversions are stopped before disabling.
 * @return None
 * @note Safe to call even if SAADC is already disabled
 * @warning ADC measurements will not work after this call until re-initialized
 */
void Api_sleep_peripheral_saadc_disable(void);

/**
 * @brief Disable other peripherals before sleep
 * @details Disables QDEC, COMP, LPCOMP, GPIOTE, and PPI.
 *          Clears all channel configurations to ensure no spurious events.
 *          
 *          Disabled peripherals:
 *          - QDEC (Quadrature decoder)
 *          - COMP (Comparator)
 *          - LPCOMP (Low power comparator)
 *          - GPIOTE (GPIO tasks and events)
 *          - PPI (Programmable peripheral interconnect)
 *          
 * @return None
 * @note Safe to call even if peripherals are already disabled
 * @warning GPIO interrupts and PPI connections will be disabled
 */
void Api_sleep_peripheral_others_disable(void);

/**
 * @brief Release high frequency clock if not needed
 * @details Releases HFCLK request to allow automatic clock management.
 *          Only releases when SoftDevice is not present.
 * @return None
 * @note Does not affect operation when SoftDevice is enabled
 * @warning Do not call if HFCLK is required for external peripherals
 */
void Api_sleep_peripheral_hfclk_release(void);

/**
 * @brief Main function to disable all peripherals before entering sleep mode
 * @details Disables all MCU peripherals in the correct sequence to prepare
 *          for low power sleep mode. This includes:
 *          1. Communication peripherals (UART/SPI/I2C)
 *          2. Timing peripherals (TIMER/PWM)
 *          3. Analog peripherals (SAADC)
 *          4. Other peripherals (GPIOTE/PPI/QDEC/COMP)
 *          5. High frequency clock release
 *          
 *          A small delay is added after disabling to ensure all operations complete.
 *          
 * @return None
 * @note Call this function immediately before entering deep sleep mode
 * @warning This will disable ALL peripherals except RTC and wake-up sources
 * @warning Re-initialize needed peripherals after wake-up
 * @see Api_sleep_peripheral_uart_disable_all(), Api_sleep_peripheral_spi_disable_all()
 * 
 * @example
 * @code
 * // Before entering sleep
 * Api_sleep_peripheral_disable_all();
 * 
 * // Enter sleep mode
 * sd_app_evt_wait();
 * 
 * // After wake-up, re-initialize needed peripherals
 * // ... your initialization code ...
 * @endcode
 */
void Api_sleep_peripheral_disable_all(void);

/**
 * @brief Re-enable peripherals after waking up from sleep mode
 *
 * @return void
 */
void Api_sleep_peripheral_enable_after_wakeup(void);

void Api_sleep_peripheral_disable_before_sleep(void);

/** @} */

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* API_SLEEP_PERIPHERAL_H */

