/**
 * @file Func_TIMER.c
 * @brief RTC2-based timer implementation for the nRF52840 platform
 * @details This module implements simple timer functions using the RTC2 peripheral,
 *          providing millisecond precision timing and sleep state management.
 * @author Platform Tag Team
 * @date 2024
 * @version 1.0
 */

#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "drv_rtc.h"
#include "nrf_rtc.h"
#include "nrf52840.h"
#include "sdk_errors.h"
#include "Func_UART_LOG.h"
#include "Func_TIMER.h"

/**
 * @defgroup TIMER_Static_Variables Static Variables
 * @{
 */
static drv_rtc_t rtc2_instance = DRV_RTC_INSTANCE(2);  /**< RTC2 instance for timer functionality */
static volatile bool s_timer_running = false;           /**< Timer running state flag */
static volatile uint32_t s_timer_count = 0;             /**< Timer counter value */
volatile bool s_sleep_state = false;                    /**< Global sleep state for power management */
/** @} */

/**
 * @defgroup TIMER_Private_Functions Private Function Declarations
 * @{
 */
// No private function declarations needed
/** @} */

/**
 * @defgroup TIMER_Event_Handlers Event Handlers
 * @{
 */

/**
 * @brief RTC2 event handler for timer interrupts
 * @details Handles RTC2 compare match events and manages sleep state.
 *          Automatically stops timer to prevent infinite interrupts.
 * @param[in] p_instance Pointer to RTC instance (unused)
 * @return None
 */
static void rtc2_event_handler(drv_rtc_t const * const p_instance)
{
    (void)p_instance;  // Unused parameter
    
    printf_uart("RTC2 Event: Timer elapsed!\r\n");
    s_sleep_state = false;
    
    // Prevent infinite interrupt
    NRF_RTC2->EVENTS_COMPARE[0] = 0;
    
    Func_TIMER_Stop();
}

/** @} */

/**
 * @defgroup TIMER_Initialization Timer Initialization Functions
 * @{
 */

/**
 * @brief Initialize RTC2 timer
 * @details Configures RTC2 peripheral with default settings, enables interrupts,
 *          and starts the RTC2 counter for timer functionality.
 * @return true on success, false on failure
 * @note This function must be called before using any timer functions
 */
bool Func_TIMER_Init(void)
{
    // RTC2 configuration
    drv_rtc_config_t config = DRV_RTC_DEFAULT_CONFIG;
    
    // RTC2 initialization
    ret_code_t err_code = drv_rtc_init(&rtc2_instance, &config, rtc2_event_handler);
    if (err_code != NRF_SUCCESS) {
        return false;
    }
    
    // Enable RTC2 interrupt for compare channel 0
    nrf_rtc_int_enable(rtc2_instance.p_reg, NRF_RTC_INT_COMPARE0_MASK);
    NVIC_EnableIRQ(RTC2_IRQn);
    
    // Start RTC2 counter
    drv_rtc_start(&rtc2_instance);
    
    // Initialize state variables
    s_timer_running = false;
    s_timer_count = 0;
    
    return true;
}

/** @} */

/**
 * @defgroup TIMER_Control Timer Control Functions
 * @{
 */

/**
 * @brief Stop RTC2 timer
 * @details Disables compare channel and stops timer operation.
 *          Resets the running state flag.
 * @return None
 * @note Timer can be restarted with Func_TIMER_Start()
 */
void Func_TIMER_Stop(void)
{
    if (s_timer_running) {
        // Disable compare channel
        drv_rtc_compare_disable(&rtc2_instance, 0);
        
        s_timer_running = false;
    }
}

/**
 * @brief Start RTC2 timer with specified interval
 * @details Configures compare channel 0 with the specified interval.
 *          Automatically stops any running timer before starting new one.
 * @param[in] interval_ms Timer interval in milliseconds
 * @return true on success, false on failure
 * @note Timer will automatically stop when interval expires
 */
bool Func_TIMER_Start(uint32_t interval_ms)
{
    if (s_timer_running) {
        Func_TIMER_Stop();
    }

    // Convert milliseconds to RTC ticks (32.768 kHz clock)
    uint32_t ticks = (interval_ms * 32768u) / 1000u;
    if (ticks == 0u) ticks = 1u;  // Prevent 0ms intervals

    // Set compare value with current counter value (24-bit mask)
    uint32_t now = nrf_rtc_counter_get(rtc2_instance.p_reg);
    uint32_t cc  = (now + ticks) & RTC_COUNTER_COUNTER_Msk;

    // Event compare setup + IRQ enable
    nrf_rtc_event_clear(rtc2_instance.p_reg, NRF_RTC_EVENT_COMPARE_0);
    drv_rtc_compare_set(&rtc2_instance, 0, cc, true);

    s_timer_running = true;
    return true;
}

/** @} */

/**
 * @defgroup TIMER_Utility Timer Utility Functions
 * @{
 */

/**
 * @brief Get current timer count
 * @details Returns the current value of the RTC2 counter.
 * @return Current timer counter value (24-bit)
 */
uint32_t Func_TIMER_GetCount(void)
{
    return s_timer_count;
}

/**
 * @brief Simple delay function in milliseconds
 * @details Implements a busy-wait delay using a simple loop.
 *          Not precise but useful for short delays.
 * @param[in] ms Delay time in milliseconds
 * @return None
 * @note This is a blocking function - CPU will be busy during delay
 */
void Func_TIMER_Delay(uint32_t ms)
{
    for (volatile uint32_t i = 0; i < ms * 8000; i++) {
        // Simple delay loop - approximately 1ms per 8000 iterations
    }
}

/**
 * @brief Simple delay function in microseconds
 * @details Implements a busy-wait delay using a simple loop.
 *          Not precise but useful for very short delays.
 * @param[in] us Delay time in microseconds
 * @return None
 * @note This is a blocking function - CPU will be busy during delay
 */
void Func_TIMER_DelayUs(uint32_t us)
{
    for (volatile uint32_t i = 0; i < us * 8; i++) {
        // Simple delay loop - approximately 1us per 8 iterations
    }
}

/** @} */
