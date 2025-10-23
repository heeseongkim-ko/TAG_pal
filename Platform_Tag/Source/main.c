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
#include "Func_TEIA_ROUTINES.h"
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

extern void NFC_Timer_Tick(void);

//#define DEBUG_SLEEP

/**
 * @brief Check and display current clock sources status
 */
void clock_status_check(void)
{
	//printf_uart2("\r\n===== Clock Status Check =====\r\n");
	
	// LFCLK Status
	if (nrf_clock_lf_is_running())
	{
		uint32_t lfclk_src_ui32 = NRF_CLOCK->LFCLKSRC & CLOCK_LFCLKSRC_SRC_Msk;
		
		//printf_uart2("LFCLK: Running\r\n");
		
		switch (lfclk_src_ui32)
		{
			case CLOCK_LFCLKSRC_SRC_RC:
				//printf_uart2("  Source: Internal RC Oscillator\r\n");
				break;
				
			case CLOCK_LFCLKSRC_SRC_Xtal:
				//printf_uart2("  Source: External 32.768kHz Crystal\r\n");
				break;
				
			case CLOCK_LFCLKSRC_SRC_Synth:
				//printf_uart2("  Source: Synthesized from HFCLK\r\n");
				break;
				
			default:
				//printf_uart2("  Source: Unknown (0x%X)\r\n", lfclk_src_ui32);
				break;
		}
		
		if (NRF_CLOCK->LFCLKSTAT & CLOCK_LFCLKSTAT_STATE_Msk)
		{
			//printf_uart2("  State: Active\r\n");
		}
	}
	else
	{
		//printf_uart2("LFCLK: Not Running\r\n");
	}
	
	// HFCLK Status
	//printf_uart2("\r\n");
	
	if (NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_STATE_Msk)
	{
		//printf_uart2("HFCLK: Running\r\n");
		
		if (NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_SRC_Msk)
		{
			//printf_uart2("  Source: External 32MHz Crystal\r\n");
			//printf_uart2("  Accuracy: High (+-20ppm)\r\n");
		}
		else
		{
			//printf_uart2("  Source: Internal 64MHz RC Oscillator\r\n");
			//printf_uart2("  Accuracy: Low (+-4%%)\r\n");
		}
	}
	else
	{
		//printf_uart2("HFCLK: Not Running\r\n");
	}
	
	//printf_uart2("==============================\r\n\r\n");
}

uint16_t example_state = 0u;
uint16_t example_state_timer = 0u;
uint16_t example_adc_timer = 0u;

void example_battery(void)
{
	uint16_t level;

	if  (example_adc_timer > 0)
	{
		return;
	}

	if  (Api_battery_state() == API_BATTERY_STATE_BUSY)
	{
	}
	else
	{
		level = Api_battery_getLevel();
		LOG_API_BAT("BAT:%d\r\n", level);
		Api_battery_startRead();
		example_adc_timer = 3000;
	}
	
}

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
	APP_ERROR_CHECK(err_code);
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
	Aply_uwb_rx_timer_tick(); 

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

	if  (example_state_timer > 0u)
	{
		--example_state_timer;
	}

	if  (example_adc_timer > 0u)
	{
		--example_adc_timer;
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
	ret_code_t err_code = nrf_drv_clock_init();
	if (err_code != NRF_ERROR_MODULE_ALREADY_INITIALIZED)
	{
		APP_ERROR_CHECK(err_code);
	}

	/* Request Low Frequency Clock. */
	nrf_drv_clock_lfclk_request(NULL);

	/* Busy-wait until LFCLK is running to ensure app_timer has a clock source. */
	while (!nrf_clock_lf_is_running())
	{
	}
	
#if  1
	/* Request High Frequency Clock (External 32MHz Crystal). */
	nrf_drv_clock_hfclk_request(NULL);

	/* Busy-wait until HFCLK is running (External Crystal stabilized). */
	while (!nrf_drv_clock_hfclk_is_running())
	{
	}

	printf_uart("HFCLK: External 32MHz Crystal activated\r\n");
#endif
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
	/* Ensure LFCLK is started before using app_timer. */
	clock_config();

	// Initialize timer module.
	uint32_t err_code = app_timer_init();
	APP_ERROR_CHECK(err_code);

	err_code = app_timer_create(&m_timer_base_id, APP_TIMER_MODE_REPEATED, timer_base_event_handler);
	APP_ERROR_CHECK(err_code);
	app_timer_start(m_timer_base_id, APP_TIMER_TICKS(APP_TIME_BASE), NULL);	// start when wake up from sleep
}

static bool nfc_falg = false;

void nfc_read_example(void)
{
#if  1
	if  (Api_nfc_is_reader_present() == true)
	{
		if  (nfc_falg == false)
		{
			uint8_t	buf[512];
			uint16_t i;
			printf_uart("NFC Detected\r\n");
			nfc_falg = true;
			Api_nfc_read_data(NFC_USER_DATA_START_ADDR, &buf[0], sizeof(buf));
			for(i = 0; i < sizeof(buf); ++i)
			{
				if  ((i % 4) == 0)
				{					 	
					printf_uart("\r\n");
				}
				printf_uart("%X ", buf[i]);
			}
		}

		if  (Api_nfc_monitor_eeprom_status() == NFC_EEPROM_WRITE_COMPLETED)
		{
			uint8_t	buf[512];
			uint16_t i;
			Api_nfc_read_data(NFC_USER_DATA_START_ADDR, &buf[0], sizeof(buf));
			#if  0
			for(i = 0; i < sizeof(buf); ++i)
			{
				if  ((i % 4) == 0)
				{					 	
					printf_uart("\r\n");
				}
				printf_uart("%X ", buf[i]);
			}
			printf_uart("\r\n");
			#endif
		}
	}
	else
	{
		if  (nfc_falg == true)
		{
			printf_uart("NFC Removed\r\n");
			nfc_falg = false;
		}
	}
#endif
}

#define NFC_EXAMPLE_WRITE_NUM	100

void nfc_write_example(void)
{
	uint8_t data[NFC_EXAMPLE_WRITE_NUM];
	uint8_t i;

	for(i = 0; i < NFC_EXAMPLE_WRITE_NUM; ++i)
	{
		data[i] = 0x00;
	}
	Api_nfc_write_data(NFC_USER_DATA_START_ADDR, &data[0], NFC_EXAMPLE_WRITE_NUM);
	
//	Api_nfc_write_data(0x40 * 4, &data[0], sizeof(data));
}

void configure_unused_pins(void)
{
    // P0.0 ~ P0.31
    for (uint32_t pin = 0; pin < 32; pin++)
    {
    	nrf_gpio_cfg(
			pin,
			NRF_GPIO_PIN_DIR_INPUT,
			NRF_GPIO_PIN_INPUT_DISCONNECT,
			NRF_GPIO_PIN_NOPULL,
			NRF_GPIO_PIN_S0S1,
			NRF_GPIO_PIN_NOSENSE
		);
    }
    
    // P1.0 ~ P1.15 (nRF52840)
    for (uint32_t pin = 32; pin < 48; pin++)
    {
    	nrf_gpio_cfg(
			pin,
			NRF_GPIO_PIN_DIR_INPUT,
			NRF_GPIO_PIN_INPUT_DISCONNECT,
			NRF_GPIO_PIN_NOPULL,
			NRF_GPIO_PIN_S0S1,
			NRF_GPIO_PIN_NOSENSE
		);
    }
}

#if  0
int main(void)
{
	configure_unused_pins();
	
	nrf_gpio_cfg_output(DW3000_RESET_Pin);
	nrf_gpio_pin_clear(DW3000_RESET_Pin);
	
	NRF_POWER->SYSTEMOFF = 1;
}
#else
int main(void)
{
#ifdef  BLE_ENABLE_FOR_OTA
	Api_ble_Init();
#endif
	
	Api_sleep_init();
	/* Initialize UART for logging */
	//debug
	UART_LOG_INIT();
	printf_uart("=== TagPlatform Starting ===\r\n");
	printf_uart("UART initialized successfully\r\n");

	// ===== BASIC INITIALIZATION =====
	Init_gpiote();
	
	configure_unused_pins();
	
	//Api_ble_advertising_start(false);
	//Func_TEIA_Reset_All_Variables();  // Commented out - not used since example_application() is disabled
	/* INITIALIZE LED start state as GREEN */
	//Func_TEIA_LED_Set(GREEN_TEIA, true);  // Commented out - using Api_Led_Init() instead

	/* Initialize ADC */
	Api_battery_init();
	printf_uart("ADC initialized successfully\r\n");

	/* Configuring interrupt*/
	//Api_uwb_irq_init();
	//printf_uart("UWB IRQ initialized\r\n");

	/* Small pause before startup */
	nrf_delay_ms(2);
	timers_init();
	printf_uart("Timers initialized\r\n");

	/* Initialize RTC2 */
	//Func_TIMER_Init();  // Commented out - RTC2 is initialized by Api_sleep_init() instead

	Api_nfc_init();

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// UWB Api Init Start
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* Initialise the SPI for nRF52840-DK */
	Api_uwb_spi_init(false);
	printf_uart("UWB SPI initialized\r\n");

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Initialize motion detection
	if (Func_Motion_Detection_Init()) {
		printf_uart("Motion detection initialized successfully\r\n");
	} else {
		printf_uart("Motion detection initialization failed\r\n");
	}

	/* INITIALIZE LED end state as GREEN */
	nrf_delay_ms(2000);

	//Func_TEIA_Init_Tag_Info();  // Commented out - not used since example_application() is disabled

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

	Api_failsafe_init();

#ifndef  DEBUG_SLEEP
	Api_uwb_start_init();
#endif

	//LOG_API_VER("Ver B\r\n");
	
	// Check clock status after all initialization
	clock_status_check();

        nfc_eeprom_tag_config_data_t config;
        if (Aply_nfc_read_all_config(&config)) {
            printf_uart("=== NFC Raw Data (Page 4-14) ===\r\n");
            
            // Print raw data page by page (4 bytes per page)
            uint8_t* data = (uint8_t*)&config;
            for(int page = 0; page < 11; page++) {
                printf_uart("Page %d: ", page + 4);
                for(int i = 0; i < 4; i++) {
                    printf_uart("%02X ", data[page * 4 + i]);
                }
                printf_uart("\r\n");
            }
			
            printf_uart("===============================\r\n");
        } else {
            printf_uart("Failed to read NFC configuration\r\n");
        }
        UART_LOG_UNINIT();

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
		Api_failsafe_main();
#endif
		
	#ifndef DEBUG_SLEEP
		Aply_tag_manager_process();
	#endif
	}
}
#endif

