// Provides simple timer functions based on RTC2
#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "drv_rtc.h"
#include "nrf_rtc.h"
#include "sdk_errors.h"

// RTC2 instance
static drv_rtc_t rtc2_instance = DRV_RTC_INSTANCE(2);

// Timer state variables
static volatile bool s_timer_running = false;
static volatile uint32_t s_timer_count = 0;

// RTC2 event handler
static void rtc2_event_handler(drv_rtc_t const * const p_instance)
{
    printf_uart("RTC2 Event: 5 seconds elapsed!\r\n");
    
    // 다음 5초 타이머 설정 (반복)
    uint32_t current_count = drv_rtc_counter_get(&rtc2_instance);
    uint32_t next_target = current_count + (5000 * 32768) / 1000;
    drv_rtc_compare_set(&rtc2_instance, 0, next_target, true);
}

/**
 * @brief Initialize RTC2 timer
 * @return true if successful, false otherwise
 */
bool APP_TIMER_Init(void)
{
    // RTC2 configuration
    drv_rtc_config_t config = DRV_RTC_DEFAULT_CONFIG;
    
    // RTC2 initialization
    ret_code_t err_code = drv_rtc_init(&rtc2_instance, &config, rtc2_event_handler);
    if (err_code != NRF_SUCCESS) {
        return false;
    }
    
    // Start RTC2
    drv_rtc_start(&rtc2_instance);
    
    s_timer_running = false;
    s_timer_count = 0;
    
    return true;
}

/**
 * @brief Start RTC2 timer with specified interval
 * @param interval_ms Timer interval in milliseconds
 * @return true if successful, false otherwise
 */
bool APP_TIMER_Start(uint32_t interval_ms)
{
    if (!s_timer_running) {
        // Set compare value (convert ms to ticks)
        uint32_t ticks = (interval_ms * 32768) / 1000; // 1ms = 32.768 ticks at 32.768kHz
        drv_rtc_compare_set(&rtc2_instance, 0, ticks, true);
        
        s_timer_running = true;
        return true;
    }
    return false;
}

/**
 * @brief Stop RTC2 timer
 */
void APP_TIMER_Stop(void)
{
    if (s_timer_running) {
        // Disable compare channel
        drv_rtc_compare_disable(&rtc2_instance, 0);
        
        s_timer_running = false;
    }
}

/**
 * @brief Get current timer count
 * @return Current timer count
 */
uint32_t APP_TIMER_GetCount(void)
{
    return s_timer_count;
}

/**
 * @brief Simple delay function in milliseconds
 * @param ms Delay time in milliseconds
 */
void APP_TIMER_Delay(uint32_t ms)
{
    for (volatile uint32_t i = 0; i < ms * 8000; i++) {
        // Simple delay loop
    }
}

/**
 * @brief Simple delay function in microseconds
 * @param us Delay time in microseconds
 */
void APP_TIMER_DelayUs(uint32_t us)
{
    for (volatile uint32_t i = 0; i < us * 8; i++) {
        // Simple delay loop
    }
}
