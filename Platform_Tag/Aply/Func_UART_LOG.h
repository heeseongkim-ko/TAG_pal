/**
 * @file Func_UART_LOG.h
 * @brief Lightweight UART logging functionality for TEIA platform
 * @details This module provides UART initialization and logging functions using nrfx_uarte driver.
 *          Includes printf-style formatting and direct string transmission capabilities.
 * @author Platform Tag Team
 * @date 2024
 * @version 1.0
 */

#ifndef FUNC_UART_LOG_H
#define FUNC_UART_LOG_H

/**
 * @defgroup UART_Configuration UART Configuration Constants
 * @{
 */
#define UART_TX_BUF_SIZE_TEIA 256  /**< UART transmit buffer size in bytes */
#define UART_RX_BUF_SIZE_TEIA 256  /**< UART receive buffer size in bytes */
#define UART_HWFC_TEIA false        /**< Hardware flow control disabled */
/** @} */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup UART_Overview Module Overview
 * @{
 * The UART logging module provides lightweight UART functionality for the TEIA platform.
 * It includes:
 * - UART initialization and configuration
 * - printf-style formatted output
 * - Direct string transmission
 * - Configurable buffer sizes and flow control
 * @}
 */

/**
 * @defgroup UART_Functions UART Function Declarations
 * @{
 */

/**
 * @brief Initialize UART logging port
 * @details Initializes UARTE driver with configured pins and settings.
 *          Uses UART_TX_PIN_NUMBER_TEIA and UART_RX_PIN_NUMBER_TEIA from def_pin.h.
 * @return None
 * @note This function must be called before using any UART functions
 */
void UART_LOG_INIT(void);

/**
 * @brief Uninitialize/close UART logging port
 * @details Closes UARTE driver and frees associated resources.
 * @return None
 */
void UART_LOG_UNINIT(void);

/**
 * @brief UART printf function with variable arguments
 * @details Formats and transmits string using printf-style formatting.
 *          Supports standard printf format specifiers.
 * @param[in] format Format string with optional format specifiers
 * @param[in] ... Variable arguments for formatting
 * @return None
 * @note Buffer size is limited to 256 bytes
 */
void printf_uart(const char* format, ...);

/**
 * @brief Directly send string to UART
 * @details Transmits string directly without formatting.
 *          More efficient than printf_uart for simple strings.
 * @param[in] str Null-terminated string to transmit
 * @return None
 * @note String must be null-terminated
 */
void uart_send_string(const char* str);

#define LOG_API_BLE	//printf_uart2
#define LOG_API_NFC	//printf_uart//printf_uart2
#define LOG_API_UWB	//printf_uart//printf_uart2
#define	LOG_API_IMU	//printf_uart//printf_uart2
#define	LOG_API_LED	//printf_uart//printf_uart2
#define LOG_API_BAT     //printf_uart//printf_uart2
#define LOG_API_SLEEP   //printf_uart//printf_uart2
#define LOG_API_FAIL    printf_uart//printf_uart2
/** @} */

#ifdef __cplusplus
}
#endif

#endif // FUNC_UART_LOG_H


