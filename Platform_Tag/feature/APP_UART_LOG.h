#ifndef APP_UART_LOG_H
#define APP_UART_LOG_H

#include <stdbool.h>
#include <stdint.h>

// Pin selections
#include "def_pin.h"

// UART buffer and flow control settings
#define UART_TX_BUF_SIZE_TEIA 256
#define UART_RX_BUF_SIZE_TEIA 256
#define UART_HWFC_TEIA false

#ifdef __cplusplus
extern "C" {
#endif

// Initialize UART logging port (FIFO mode via app_uart)
void UART_LOG_INIT(void);

// Uninitialize/close UART logging port
void UART_LOG_UNINIT(void);

// UART로 직접 데이터를 보내는 함수들
void printf_uart(const char* format, ...);
void uart_send_string(const char* str);

#ifdef __cplusplus
}
#endif

#endif // APP_UART_LOG_H


