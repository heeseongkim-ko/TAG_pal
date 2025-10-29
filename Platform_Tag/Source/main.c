/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

File    : main.c
Purpose : Generic application start

*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <deca_spi.h>
#include <port.h>

#include "nrf_drv_timer.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "nrf_clock.h"
#include "boards.h"
#include "Deca_device_api.h"

#include "Api_uwb.h"
#include "Api_ble.h"
#include "Api_nfc.h"
#include "Api_motion.h"
#include "Api_Led.h"
#include "Api_battery.h"
#include "Api_sleep.h"
#include "Api_sleep_peripheral.h"
#include "Api_failsafe.h"

#include "Aply_tag_manager.h"
#include "Aply_tag_configuration.h"
#include "Aply_tag_scheduler.h"
#include "Aply_uwb_tx.h"
#include "Aply_uwb_rx.h"
#include "Aply_nfc.h"

#include "Func_UART_LOG.h"
#include "drv_rtc.h"
#include "def_config.h"
#include "def_packet.h"
#include "sdk_errors.h"
#include "nrf_error.h"

#define TEIA_API_EXAMPLE
extern void printf_uart_timer_tick(void);

APP_TIMER_DEF(m_timer_base_id);
#define	APP_TIME_BASE	1u	// 5msec

uint32_t BaseTimer_10msec_ui32;
uint32_t BaseTimer_100msec_ui32;
uint32_t BaseTimer_1000msec_ui32;

volatile bool ntag_semaphore = false;

void Aply_timer_stop(void)
{	
	app_timer_stop(m_timer_base_id);
}

void Aply_timer_start(void)
{	
	app_timer_start(m_timer_base_id, APP_TIMER_TICKS(APP_TIME_BASE), NULL);
}

void Init_gpiote(void)
{
	ret_code_t err_code;
	err_code = nrfx_gpiote_init();
}

void Timer_Counter_10msec(void)
{
}

void Timer_Counter_100msec(void)
{
}

void Timer_Counter(void)
{
	Api_uwb_timer_tick();
	Api_nfc_timer_tick();
	Api_Led_Timer_tick();
	Api_battery_timer_tick();
	Api_failsafe_timer_tick();
	//Aply_uwb_rx_timer_tick(); 

	printf_uart_timer_tick();
	
	if  (++BaseTimer_10msec_ui32 > (10u / APP_TIME_BASE))
	{
		BaseTimer_10msec_ui32 = 0u;
		Timer_Counter_10msec();
		if  (++BaseTimer_100msec_ui32 > (10u / APP_TIME_BASE))
		{
			BaseTimer_100msec_ui32 = 0;
			Timer_Counter_100msec();
		}
	}
    
	Api_sleep_setWakeupReason(API_SLEEP_WAKEUP_REASON_MASK_TICK_TIMER);
}

/**
 * @brief Handler for timer events.
 */
void timer_base_event_handler(void* p_context)
{
	Timer_Counter();
}

static void clock_config(void)
{
	ret_code_t err_code;
	bool lfclk_ok_b;
	bool hfclk_ok_b;
	
	err_code = nrf_drv_clock_init();
	if (err_code != NRF_ERROR_MODULE_ALREADY_INITIALIZED)
	{
		APP_ERROR_CHECK(err_code);
	}

	/* Request Low Frequency Clock */
	nrf_drv_clock_lfclk_request(NULL);

	/* Check LFCLK startup with timeout */
	lfclk_ok_b = Api_failsafe_clock_check_lfclk_start();
	
	if (!lfclk_ok_b)
	{
		Api_failsafe_set_fail(FAILSAFE_CODE_16);
	}
	else
	{
	}
	
#if 1
	/* Request High Frequency Clock (External 32MHz Crystal) */
	nrf_drv_clock_hfclk_request(NULL);
	nrf_delay_ms(100);
	
	/* Check HFCLK startup with timeout */
	hfclk_ok_b = Api_failsafe_clock_check_hfclk_start();

	if (!hfclk_ok_b)
	{
		Api_failsafe_set_fail(FAILSAFE_CODE_16);
	}
	else
	{
	}
#endif
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
	// Initialize timer module.
	uint32_t err_code = app_timer_init();
	APP_ERROR_CHECK(err_code);

	err_code = app_timer_create(&m_timer_base_id, APP_TIMER_MODE_REPEATED, timer_base_event_handler);
	APP_ERROR_CHECK(err_code);
	app_timer_start(m_timer_base_id, APP_TIMER_TICKS(APP_TIME_BASE), NULL);	// start when wake up from sleep
}

int main(void)
{
	Api_failsafe_init();
	
	/* Ensure LFCLK is started before using app_timer. */
	clock_config();
	
	// Api_failsafe_watchdog_init();
	
#ifdef  BLE_ENABLE_FOR_OTA
	Api_ble_Init();
#endif
	
	Api_sleep_init();
	/* Initialize UART for logging */
	//debug
	UART_LOG_INIT();

	// ===== BASIC INITIALIZATION =====
	Init_gpiote();
		
	//Api_ble_advertising_start(false);
	//Func_TEIA_Reset_All_Variables();  // Commented out - not used since example_application() is disabled
	/* INITIALIZE LED start state as GREEN */
	//Func_TEIA_LED_Set(GREEN_TEIA, true);  // Commented out - using Api_Led_Init() instead

	/* Initialize ADC */
	Api_battery_init();

	/* Configuring interrupt*/
	//Api_uwb_irq_init();
	//printf_uart("UWB IRQ initialized\r\n");

	/* Small pause before startup */
	nrf_delay_ms(2);
	timers_init();

	/* Initialize RTC2 */
	//Func_TIMER_Init();  // Commented out - RTC2 is initialized by Api_sleep_init() instead

	Api_nfc_init();

	// Initialize motion detection
	Aply_tag_configuration_init_default();

	//Func_TEIA_Init_Tag_Info();  // Commented out - not used since example_application() is disabled
	Aply_tag_configuration_init_motion_detection();

	Api_Led_Init();
	
	// ===== APLY MODULE INITIALIZATION =====
	/* Check NFC data and initialize tag configuration */
	//Aply_nfc_check_and_init_tag_configuration();
	
	/* Initialize Aply tag manager */
	Aply_tag_manager_init();
	
	/* Initialize Aply tag scheduler */
	//Aply_tag_scheduler_init();
		
#ifdef  BLE_ENABLE_FOR_OTA
	Api_ble_advertising_start(false);
#endif

#ifndef  DEBUG_SLEEP
	Api_uwb_start_init();
#endif

	LOG_API_VER("Ver B\r\n");
	

#ifdef DEBUG_SLEEP	// Debugging sleep
	Api_sleep_setSleepTime(1000);
#endif

	while(1)
	{
	#if  0
		while(1)
		{
			if  (Api_sleep_getWakeupReason() & API_SLEEP_WAKEUP_REASON_MASK_TICK_TIMER)
			{
				break;
			}
			Api_sleep_core();
		}
	#endif
		
#ifdef DEBUG_SLEEP	// Debugging sleep	
        Api_sleep_start_sleep();
#endif
		
		Api_sleep_claerWakeupReasonMask(API_SLEEP_WAKEUP_REASON_MASK_TICK_TIMER);
		Api_uwb_main();
		Api_Led_Main();
		Api_battery_main();
		Api_sleep_main();
#ifndef DEBUG_SLEEP
		//Api_failsafe_main();
#endif
		
	#ifndef DEBUG_SLEEP
		Aply_tag_manager_process();
	#endif
	}
}

