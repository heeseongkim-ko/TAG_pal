/**
 * @file Func_UART_LOG.c
 * @brief Lightweight UART logging implementation for TEIA platform
 * @details This module implements UART functionality using nrfx_uarte driver,
 *          providing printf-style formatting and direct string transmission.
 * @author Platform Tag Team
 * @date 2024
 * @version 1.0
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#include "Func_UART_LOG.h"  // includes def_pin.h and UART buffer/flow settings
#include "nrfx_uarte.h"
#include "nrf_gpio.h"

#define UART_HWFC_TEIA false                             /**< Hardware flow control disabled for TEIA */

#define UART_TX_PIN_NUMBER_TEIA NRF_GPIO_PIN_MAP(0,26)  /**< UART TX pin for TEIA platform */
#define UART_RX_PIN_NUMBER_TEIA NRF_GPIO_PIN_MAP(0,19)  /**< UART RX pin for TEIA platform */

bool uart_log_init_b = false;

/**
 * @defgroup UART_Static_Variables Static Variables
 * @{
 */
static nrfx_uarte_t uarte_instance = NRFX_UARTE_INSTANCE(0);  /**< UARTE instance for logging */

/**
 * @brief UART configuration for logging port
 * @details Configures UARTE with TEIA-specific pin assignments and settings.
 *          Uses 115200 baud rate with no parity and no flow control.
 */
static nrfx_uarte_config_t uarte_config = {
    .pseltxd = UART_TX_PIN_NUMBER_TEIA,    /**< TX pin from def_pin.h */
    .pselrxd = UART_RX_PIN_NUMBER_TEIA,    /**< RX pin from def_pin.h */
    .pselrts = 0xFFFFFFFF,                 /**< RTS not used */
    .pselcts = 0xFFFFFFFF,                 /**< CTS not used */
    .hwfc = UART_HWFC_TEIA,                /**< Hardware flow control disabled */
    .parity = NRF_UARTE_PARITY_EXCLUDED,   /**< No parity */
    .baudrate = NRF_UARTE_BAUDRATE_921600, /**< 115200 baud rate */
    .interrupt_priority = NRFX_UARTE_DEFAULT_CONFIG_IRQ_PRIORITY,  /**< Default interrupt priority */
    .p_context = NULL                       /**< No context pointer */
};
/** @} */

/**
 * @defgroup UART_Initialization UART Initialization Functions
 * @{
 */

/**
 * @brief Initialize UART logging port
 * @details Initializes UARTE driver with configured pins and settings.
 *          Uses UART_TX_PIN_NUMBER_TEIA and UART_RX_PIN_NUMBER_TEIA from def_pin.h.
 * @return None
 * @note This function must be called before using any UART functions
 */
void UART_LOG_INIT(void)
{
    uint32_t err_code;
    
    // Manually configure UART pins to ensure they are set correctly
    nrf_gpio_cfg_output(UART_TX_PIN_NUMBER_TEIA);  // TX as output
    nrf_gpio_cfg_input(UART_RX_PIN_NUMBER_TEIA, NRF_GPIO_PIN_PULLUP);  // RX as input with pullup
    // Initialize UARTE driver
    err_code = nrfx_uarte_init(&uarte_instance, &uarte_config, NULL);
    if (err_code != NRFX_SUCCESS) {
        // Handle error if needed
        return;
    }
	uart_log_init_b = true;
}

/**
 * @brief Uninitialize/close UART logging port
 * @details Closes UARTE driver and frees associated resources.
 * @return None
 */
void UART_LOG_UNINIT(void)
{
	uart_log_init_b = false;

    nrfx_uarte_uninit(&uarte_instance);
}

/** @} */

uint32_t log_timer;
void printf_uart_timer_tick(void)
{
	++log_timer;
}

/**
 * @defgroup UART_Transmission UART Transmission Functions
 * @{
 */

/**
 * @brief UART printf function with variable arguments
 * @details Formats and transmits string using printf-style formatting.
 *          Supports standard printf format specifiers.
 *          Buffer size is limited to 256 bytes for safety.
 * @param[in] format Format string with optional format specifiers
 * @param[in] ... Variable arguments for formatting
 * @return None
 * @note Buffer size is limited to 256 bytes
 */
#if  0
/**
 * @brief Print formatted string to UART with timestamp
 * @param format Format string (printf style)
 * @param ... Variable arguments for format string
 */
void printf_uart(const char* format, ...)
{
	char l_buffer[256];
	char l_final_buffer[300];
	va_list args;
	
	va_start(args, format);
	vsnprintf(l_buffer, sizeof(l_buffer), format, args);
	va_end(args);
	
	// Add timestamp prefix (use %u for 32-bit or split 64-bit)
	uint32_t l_timestamp_ms = (uint32_t)(log_timer & 0xFFFFFFFF);
	snprintf(l_final_buffer, sizeof(l_final_buffer), "[%u] %s", l_timestamp_ms, l_buffer);
	
	// Send string using nrfx_uarte
	uint32_t l_len = strlen(l_final_buffer);
	if (l_len > 0) {
		nrfx_uarte_tx(&uarte_instance, (uint8_t*)l_final_buffer, l_len);
	}
}

#else
/**
 * @brief Print formatted string to UART with timestamp
 * @param format Format string (printf style)
 * @param ... Variable arguments
 * @note log_timer is uint32_t
 */
void printf_uart2(const char* format, ...)
{
	if  (uart_log_init_b == false)
	{
		return;
	}
#if 0
	static char log_buffer[280];  // Static to prevent stack issues with async UART
	char buffer[256];
	va_list args;
	uint32_t offset = 0;
	
	// Option 1: Simple millisecond timestamp [12345ms]
	offset = snprintf(log_buffer, sizeof(log_buffer), "[%lums] ", log_timer);
	
	// Option 2: Seconds.milliseconds format [12.345s]
	// uint32_t seconds = log_timer / 1000;
	// uint32_t millis = log_timer % 1000;
	// offset = snprintf(log_buffer, sizeof(log_buffer), "[%lu.%03lus] ", seconds, millis);
	
	// Option 3: Minutes:Seconds.milliseconds [00:12.345]
	// uint32_t minutes = log_timer / 60000;
	// uint32_t seconds = (log_timer / 1000) % 60;
	// uint32_t millis = log_timer % 1000;
	// offset = snprintf(log_buffer, sizeof(log_buffer), "[%02lu:%02lu.%03lu] ", 
	//                   minutes, seconds, millis);
	
	// Format the actual message
	va_start(args, format);
	vsnprintf(buffer, sizeof(buffer), format, args);
	va_end(args);
	
	// Append message to timestamp
	snprintf(log_buffer + offset, sizeof(log_buffer) - offset, "%s", buffer);
	
	// Send string using nrfx_uarte
	uint32_t len = strlen(log_buffer);
	if (len > 0) 
	{
		nrfx_uarte_tx(&uarte_instance, (uint8_t*)log_buffer, len);
	}
#endif
}



void printf_uart(const char* format, ...)
{
#if  0
    char buffer[256];
    va_list args;
    
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    // Send string using nrfx_uarte
    uint32_t len = strlen(buffer);
    if (len > 0) {
        nrfx_uarte_tx(&uarte_instance, (uint8_t*)buffer, len);
    }
  #endif
}
#endif

/**
 * @brief Directly send string to UART
 * @details Transmits string directly without formatting.
 *          More efficient than printf_uart for simple strings.
 *          Includes null pointer check for safety.
 * @param[in] str Null-terminated string to transmit
 * @return None
 * @note String must be null-terminated
 */
void uart_send_string(const char* str)
{
    if (str != NULL) {
        uint32_t len = strlen(str);
        if (len > 0) {
            nrfx_uarte_tx(&uarte_instance, (uint8_t*)str, len);
        }
    }
}

/** @} */


