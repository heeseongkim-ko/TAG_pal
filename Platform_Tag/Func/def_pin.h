/**
 * @file def_pin.h
 * @brief Pin definitions and configurations for TEIA platform
 * @details This file contains all GPIO pin definitions for the TEIA platform,
 *          including LED, UART, SPI, I2C, ADC, and IRQ pin assignments.
 * @author Platform Tag Team
 * @date 2024
 * @version 1.0
 */

// Copied and organized from test/def_pin.h

#ifndef DEF_PIN_H_INCLUDED
#define DEF_PIN_H_INCLUDED

#include "nrf_gpio.h"

/**
 * @defgroup PIN_Platform_Selection Platform Selection
 * @{
 */

/**
 * @brief DWM3001 reference platform pin definitions
 * @details These definitions are for reference only and are not used in the current build.
 *          Kept for documentation and potential future use.
 */
#if 0  // dwm3001 (reference)
/**
 * @defgroup PIN_DWM3001_LED DWM3001 LED Pins
 * @{
 */
#define LED_RED   NRF_GPIO_PIN_MAP(0,14)  /**< Red LED pin for DWM3001 */
#define LED_GREEN NRF_GPIO_PIN_MAP(0,22)  /**< Green LED pin for DWM3001 */
#define RED 1                              /**< Red LED identifier */
#define GREEN 2                             /**< Green LED identifier */
#define ORANGE 3                            /**< Orange LED identifier (both LEDs) */
/** @} */

/**
 * @defgroup PIN_DWM3001_UART DWM3001 UART Pins
 * @{
 */
#define UART_TX_PIN_NUMBER NRF_GPIO_PIN_MAP(0,19)  /**< UART TX pin for DWM3001 */
#define UART_RX_PIN_NUMBER NRF_GPIO_PIN_MAP(0,15)  /**< UART RX pin for DWM3001 */
/** @} */

/**
 * @defgroup PIN_DWM3001_SPI DWM3001 SPI Pins
 * @{
 */
#define DW3000_CLK_Pin   NRF_GPIO_PIN_MAP(0,  3)  /**< DW3000 SPI CLK pin for DWM3001 */
#define DW3000_MISO_Pin  NRF_GPIO_PIN_MAP(0, 29)  /**< DW3000 SPI MISO pin for DWM3001 */
#define DW3000_MOSI_Pin  NRF_GPIO_PIN_MAP(0,  8)  /**< DW3000 SPI MOSI pin for DWM3001 */
#define DW3000_CS_Pin    NRF_GPIO_PIN_MAP(1,  6)  /**< DW3000 SPI CS pin for DWM3001 */
#define DW3000_WKUP_Pin  NRF_GPIO_PIN_MAP(1, 19)  /**< DW3000 Wake-up pin for DWM3001 */
#define DW3000_IRQ_Pin   NRF_GPIO_PIN_MAP(1,  2)  /**< DW3000 IRQ pin for DWM3001 */
#define DW3000_RST_Pin   NRF_GPIO_PIN_MAP(0, 25)  /**< DW3000 Reset pin for DWM3001 */
/** @} */
#endif

/**
 * @brief TEIA platform pin definitions (active configuration)
 * @details These are the currently active pin definitions used by the TEIA platform.
 */
#if 1
/**
 * @defgroup PIN_TEIA_LED TEIA LED Pins
 * @{
 */
#define LED_RED_TEIA   NRF_GPIO_PIN_MAP(0,7)   /**< Red LED pin for TEIA platform */
#define LED_GREEN_TEIA NRF_GPIO_PIN_MAP(0,23)  /**< Green LED pin for TEIA platform */
#define RED_TEIA 1                              /**< Red LED identifier for TEIA */
#define GREEN_TEIA 2                             /**< Green LED identifier for TEIA */
#define ORANGE_TEIA 3                            /**< Orange LED identifier for TEIA (both LEDs) */
#define BLACK_TEIA 4                             /**< Black LED identifier for TEIA (all LEDs off) */
/** @} */

/**
 * @defgroup PIN_TEIA_UART TEIA UART Pins
 * @{
 */
#define UART_TX_PIN_NUMBER_TEIA NRF_GPIO_PIN_MAP(0,26)  /**< UART TX pin for TEIA platform */
#define UART_RX_PIN_NUMBER_TEIA NRF_GPIO_PIN_MAP(0,19)  /**< UART RX pin for TEIA platform */
#define UART_HWFC_TEIA false                             /**< Hardware flow control disabled for TEIA */
#define UART_RX_BUF_SIZE_TEIA 256                        /**< UART RX buffer size for TEIA */
#define UART_TX_BUF_SIZE_TEIA 256                        /**< UART TX buffer size for TEIA */
/** @} */

/**
 * @defgroup PIN_TEIA_SPI TEIA SPI Pins for DW3000
 * @{
 */
#define DW3000_CLK_Pin_TEIA   NRF_GPIO_PIN_MAP(0,  3)  /**< DW3000 SPI CLK pin for TEIA */
#define DW3000_MISO_Pin_TEIA  NRF_GPIO_PIN_MAP(0, 29)  /**< DW3000 SPI MISO pin for TEIA */
#define DW3000_MOSI_Pin_TEIA  NRF_GPIO_PIN_MAP(0,  8)  /**< DW3000 SPI MOSI pin for TEIA */
#define DW3000_CS_Pin_TEIA    NRF_GPIO_PIN_MAP(1,  6)  /**< DW3000 SPI CS pin for TEIA */
#define DW3000_WKUP_Pin_TEIA  NRF_GPIO_PIN_MAP(1, 3)   /**< DW3000 Wake-up pin for TEIA */
#define DW3000_IRQ_Pin_TEIA   NRF_GPIO_PIN_MAP(1,  2)  /**< DW3000 IRQ pin for TEIA */
#define DW3000_RST_Pin_TEIA   NRF_GPIO_PIN_MAP(0, 25)  /**< DW3000 Reset pin for TEIA */
/** @} */

/**
 * @defgroup PIN_TEIA_ADC TEIA ADC Pins
 * @{
 */
#define ADC_RDIV_TEIA  NRF_GPIO_PIN_MAP(0,30)  /**< ADC resistive divider pin for TEIA */
#define ADC_MEAS_TEIA  NRF_GPIO_PIN_MAP(0,2)   /**< ADC measurement pin for TEIA */
/** @} */

/**
 * @defgroup PIN_TEIA_I2C TEIA I2C Pins
 * @{
 */
#define I2C_SCL_TEIA   NRF_GPIO_PIN_MAP(0,27)  /**< I2C SCL pin for TEIA platform */
#define I2C_SDA_TEIA   NRF_GPIO_PIN_MAP(0,31)  /**< I2C SDA pin for TEIA platform */
/** @} */

/**
 * @defgroup PIN_TEIA_IRQ TEIA IRQ Pins
 * @{
 */
#define ACC_IRQ_TEIA   NRF_GPIO_PIN_MAP(0,16)  /**< Accelerometer IRQ pin for TEIA */
#define NFC_FD_TEIA    NRF_GPIO_PIN_MAP(0,17)  /**< NFC Field Detect pin for TEIA */
/** @} */
#endif

/** @} */

#endif  /* DEF_PIN_H_INCLUDED */


