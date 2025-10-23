// Lightweight UART logging init/uninit helpers using nrfx_uarte

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#include "APP_UART_LOG.h"  // includes def_pin.h and UART buffer/flow settings
#include "nrfx_uarte.h"
#include "nrf_gpio.h"

// UART instance
static nrfx_uarte_t uarte_instance = NRFX_UARTE_INSTANCE(0);

// UART configuration for logging port
static nrfx_uarte_config_t uarte_config = {
    .pseltxd = UART_TX_PIN_NUMBER_TEIA,
    .pselrxd = UART_RX_PIN_NUMBER_TEIA,
    .pselrts = 0xFFFFFFFF,
    .pselcts = 0xFFFFFFFF,
    .hwfc = UART_HWFC_TEIA,
    .parity = NRF_UARTE_PARITY_EXCLUDED,
    .baudrate = NRF_UARTE_BAUDRATE_115200,
    .interrupt_priority = NRFX_UARTE_DEFAULT_CONFIG_IRQ_PRIORITY,
    .p_context = NULL
};

void UART_LOG_INIT(void)
{
    uint32_t err_code;
    
    // Initialize UARTE driver
    err_code = nrfx_uarte_init(&uarte_instance, &uarte_config, NULL);
    if (err_code != NRFX_SUCCESS) {
        // Handle error if needed
        return;
    }
}

void UART_LOG_UNINIT(void)
{
    nrfx_uarte_uninit(&uarte_instance);
}

// UART printf function using nrfx_uarte
void printf_uart(const char* format, ...)
{
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
}

// Function to directly send string to UART
void uart_send_string(const char* str)
{
    if (str != NULL) {
        uint32_t len = strlen(str);
        if (len > 0) {
            nrfx_uarte_tx(&uarte_instance, (uint8_t*)str, len);
        }
    }
}


