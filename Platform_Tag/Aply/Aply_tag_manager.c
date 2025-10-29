/**
 * @file Aply_tag_manager.c
 * @brief Simple tag manager module implementation for TEIA platform
 * @author Platform Tag Team
 * @date 2025
 * @version 1.0
 */

#include "Aply_tag_manager.h"
#include "Aply_uwb_tx.h"
#include "Aply_uwb_rx.h"
#include "Aply_backchannel_app.h"
#include "Aply_tag_scheduler.h"
#include "Aply_tag_configuration.h"
#include "Aply_nfc.h"
#include "Api_uwb.h"
#include "Api_Led.h"
#include "Api_battery.h"
#include "Api_motion.h"
#include "Api_sleep.h"
#include "Api_failsafe.h"
#include "Api_nfc.h"
#include "Func_UART_LOG.h"
#include "nrf_delay.h"
#include <stdlib.h>

// External function declarations

// Internal function declarations
static void management_check_motion_wakeup(void);
static void management_enter_sleep_mode(void);
static bool management_is_ready_for_sleep_state_based(void);
static void management_led_process(void);
static void management_battery_check_process(void);
static void management_uwb_transmission_process(void);
static void management_nfc_process(void);

extern void Aply_get_uwb_mac_address(void);
// LED state machine defines
#define LED_STATE_IDLE     0u
#define LED_STATE_BLINK     1u

// Battery check state machine defines
#define BATTERY_STATE_IDLE     0u
#define BATTERY_STATE_BUSY      1u

// UWB state machine defines
#define UWB_STATE_IDLE              0u
#define UWB_STATE_WAKEUP_START      1u
#define UWB_STATE_WAKEUP_WAIT       2u
#define UWB_STATE_TX_WAIT           3u
#define UWB_STATE_RX_START          4u
#define UWB_STATE_RX_WAIT           5u
#define UWB_STATE_ACK               6u
#define UWB_STATE_SLEEP_START       7u
#define UWB_STATE_SLEEP_WAIT        8u

static tag_manager_state_e s_tag_manager_state = TAG_MANAGER_STATE_NORMAL;
static uint32_t s_uwb_state = UWB_STATE_IDLE;           // UWB state machine state
static uint32_t s_led_state = LED_STATE_IDLE;                     // LED state machine state
static uint32_t s_battery_check_state = BATTERY_STATE_IDLE;       // Battery check state machine state
static uint8_t back_channel_led_state = 0;
// Static variables for NFC processing
static bool s_nfc_field_detected = false;  // NFC field detection flag
static bool s_nfc_get_read_b = false;  // NFC field detection flag
static bool s_first_init_flag = true;
// Process completion flags
static bool s_led_complete_flag = true;                            // LED process completion flag (default ready)
static bool s_battery_complete_flag = true;                        // Battery check process completion flag (default ready)
static bool s_uwb_sleep_flag = false;                              // UWB sleep flag - prevents immediate restart

//----------test
static int s_ack_retry_count = 0;
//----------
/**
 * @brief LED process function (internal use only)
 * @details State machine for LED control based on scheduler flags
 */
static void management_led_process(void)
{
	switch (s_led_state)
	{
		case LED_STATE_IDLE: // LED_IDLE - Start LED based on flag
		if (Aply_tag_scheduler_get_led_flag())
		{
			if(Aply_backchannel_is_led_blink_requested() == true){
				back_channel_led_state++;
				if(back_channel_led_state > 10){
					Aply_backchannel_clear_led_blink_requested();
					Aply_tag_scheduler_clear_led_flag();
					back_channel_led_state = 0;
				}
				else Aply_tag_scheduler_set_led_counter_backchannel();
			}
			Aply_tag_scheduler_clear_led_flag();
			
			// Get current battery voltage raw data via API (Raw ADC value)
			uint16_t l_battery_voltage = Api_battery_getRawData();
			
			// Configure LED based on battery level (using existing TEIA reference values)
			if (l_battery_voltage > 185)
			{
				// Battery >185 (3.65V) - Green LED, 1 time (10ms on, 1ms off)
				Api_Led_StartBlink(API_LED_GREEN, 10, 1, 0);
			}
			else if (l_battery_voltage >= 178)
			{
				// Battery 178-185 (3.5V-3.65V) - Orange LED, 1 time (10ms on, 1ms off)
				Api_Led_StartBlink(API_LED_ORANGE, 10, 1, 0);
			}
			else
			{
				// Battery <178 (3.5V) - Red LED, 1 time (10ms on, 1ms off)
				Api_Led_StartBlink(API_LED_RED, 10, 1, 0);
			}
			
			s_led_state = LED_STATE_BLINK; // Go to BLINK check
		}
			break;
			
		case LED_STATE_BLINK: // LED_BLINK - Check if blinking (BLINK_ON/BLINK_OFF)
			{
				Api_Led_state_g led_state = Api_Led_GetState();
				if (led_state != API_LED_STATE_BLINK_ON && led_state != API_LED_STATE_BLINK_OFF)
				{
					// Not blinking anymore - go back to IDLE
					s_led_state = LED_STATE_IDLE;
				}
				// If still blinking, stay in current state
			}
			break;
			
		default:
			s_led_state = LED_STATE_IDLE;
			break;
	}
}

/**
 * @brief Battery check process function (internal use only)
 * @details State machine for periodic battery voltage measurement
 */
static void management_battery_check_process(void)
{
	switch (s_battery_check_state)
	{
		case BATTERY_STATE_IDLE: // BATTERY_CHECK_IDLE - Start battery measurement
			if (Aply_tag_scheduler_get_battery_flag())
			{
				Aply_tag_scheduler_clear_battery_flag();
			//	if (Api_battery_startRead())
			//	{
			//		s_battery_check_state = BATTERY_STATE_BUSY; // Go to BUSY check
			//	}
				// If start failed, stay in IDLE state
			}
			break;
			
		case BATTERY_STATE_BUSY: // BATTERY_CHECK_BUSY - Check if busy
			if (Api_battery_state() != API_BATTERY_STATE_BUSY)
			{
				// Not busy anymore - go back to IDLE
				s_battery_check_state = BATTERY_STATE_IDLE;
			}
			// If still busy, stay in current state
			break;
			
		default:
			s_battery_check_state = BATTERY_STATE_IDLE;
			break;
	}
}

/**
 * @brief NFC process function (internal use only)
 * @details Handles NFC field detection and sets flag for later processing
 */
static void management_nfc_process(void)
{
	// Check NFC field detection status
	nfc_reader_state_t nfc_state = Api_nfc_poll_field_detection();
	
	if (nfc_state == NFC_READER_PRESENT) {
		s_nfc_field_detected = true;
		s_nfc_get_read_b = false;
	}
	else
	{
		if  (s_nfc_field_detected)
		{
			s_nfc_get_read_b = true;
		}
	}
}

/**
 * @brief UWB transmission process function (internal use only)
 * @details Implements UWB transmission and sleep state machine
 */
/**
 * @brief Check motion wakeup and handle motion sleep flag clearing
 * @details Checks if device woke up due to motion detection and clears motion sleep flag
 * @return None
 */
static void management_check_motion_wakeup(void)
{
	uint8_t l_wakeup_reason = Api_sleep_getWakeupReason();
        
	if (l_wakeup_reason & API_SLEEP_WAKEUP_REASON_MASK_MOTION) {
		// Motion detected wakeup - clear motion sleep flag
		Aply_tag_scheduler_clear_motion_sleep_flag();
		Api_motion_disable_interrupt();  // Disable interrupt for normal mode return
		Aply_tag_scheduler_reset_motion_sleep_counter();
	}
}

/**
 * @brief Check if all tasks are ready for sleep entry (State-based approach)
 * @details Uses state machine states to determine readiness instead of complex flag combinations
 * @return true if all tasks are ready for sleep, false otherwise
 */
static bool management_is_ready_for_sleep_state_based(void)
{
	// LED Ready: IDLE state
	bool led_ready = (s_led_state == LED_STATE_IDLE);
	
	// Battery Ready: IDLE state
	bool battery_ready = (s_battery_check_state == BATTERY_STATE_IDLE);
	
	// UWB Ready: IDLE state
	bool uwb_ready = (s_uwb_state == UWB_STATE_IDLE);
	
	return (led_ready && battery_ready && uwb_ready);
	//return (battery_ready && uwb_ready);
}

static void management_uwb_transmission_process(void)
{
	switch (s_uwb_state)
	{
		case UWB_STATE_IDLE: // IDLE - Waiting for sleep flag to be cleared
			if (!s_uwb_sleep_flag)
			{
				if (Api_uwb_is_device_wakeup())
				{
					if (Api_uwb_start_sleep())
					{
						s_uwb_state = UWB_STATE_SLEEP_WAIT;
					}
					else
					{
						s_uwb_state = UWB_STATE_IDLE;
					}
				}
				else
				{
					Aply_tag_scheduler_normal_process_cycle_complete();

					Aply_uwb_tx_prepare_packet(); // Use Aply instead of Func_TEIA_Prepare_Packet
					
					if (Aply_nfc_get_uwb_settings_changed() || Aply_backchannel_get_uwb_settings_changed()) {					
						Aply_nfc_check_and_apply_uwb_settings();
						Aply_backchannel_clear_uwb_settings_changed();
						if (Api_uwb_start_wakeup(true))
						{
							s_uwb_state = UWB_STATE_WAKEUP_WAIT;						
						}
						else
						{							
							s_uwb_state = UWB_STATE_IDLE;
						}
					}
					else{
						if (Api_uwb_start_wakeup(false))
						{
							s_uwb_state = UWB_STATE_WAKEUP_WAIT;							
						}
						else
						{							
							s_uwb_state = UWB_STATE_IDLE;
						}
					}
				}
			}
			break;
			
		case UWB_STATE_WAKEUP_START: // WAKEUP_START - Start UWB device wake-up process
			if (Api_uwb_start_wakeup(false))
			{
				s_uwb_state = UWB_STATE_WAKEUP_WAIT;
			}
			else
			{
				s_uwb_state = UWB_STATE_IDLE;
			}
			break;
			
		case UWB_STATE_WAKEUP_WAIT: // WAKEUP_WAIT - Wait for wake-up process to complete
			if (Api_uwb_is_wakeup_complete())
			{
				if (Api_uwb_is_device_wakeup())
				{
					if (Api_uwb_is_device_ready())
					{
						// Check if last packet was BC packet
						if (Aply_uwb_tx_was_last_packet_bc()) {
							Api_uwb_set_rx_timeout(3);
							Api_uwb_start_tx(false, true);  // BC packet with RX
						} else {
							Api_uwb_start_tx(true, false); // Regular packet without RX
						}
						s_uwb_state = UWB_STATE_TX_WAIT;
					}
					else
					{
						s_uwb_state = UWB_STATE_IDLE;
					}
				}
				else
				{
					s_uwb_state = UWB_STATE_IDLE;
				}
			}
			else if (!Api_uwb_wakeup_is_busy())
			{
				// Wake-up process failed or timed out
				s_uwb_state = UWB_STATE_IDLE;
			}
			break;
			
		case UWB_STATE_TX_WAIT: // TX_WAIT - Wait for transmission to complete
			if (!Api_uwb_tx_is_busy())
			{
				// Check if this was a BC packet - if so, open RX for response
				if (Aply_uwb_tx_was_last_packet_bc()) {
					//s_uwb_state = UWB_STATE_RX_START;
					s_uwb_state = UWB_STATE_RX_WAIT;
				} else {
					s_uwb_state = UWB_STATE_SLEEP_START;
				}
			}
			break;
			
		case UWB_STATE_RX_WAIT: // RX_WAIT - Wait for RX completion or timeout
			// Process any received data regardless of completion status
			if (Aply_uwb_rx_process_data())
			{
				// Valid BC data received - clear BC flag and proceed to ACK
				Aply_uwb_tx_clear_bc_flag();
				s_ack_retry_count = 0;
				s_uwb_state = UWB_STATE_ACK;
			}
			else if (!Api_uwb_rx_is_busy())
			{
				// RX completed but no valid data - clear BC flag and proceed to sleep
				Aply_uwb_tx_clear_bc_flag();
				s_uwb_state = UWB_STATE_SLEEP_START;
			}
			break;
			
		case UWB_STATE_ACK: // ACK - Send ACK response
#if 1
			// Wait for RX to be inactive before sending ACK
			if (!Api_uwb_rx_is_busy())
			{
				if (Aply_uwb_tx_send_ack_packet())
				{
					// ACK packet prepared - start TX
					if (Api_uwb_start_tx(true, false)) {
						s_uwb_state = UWB_STATE_TX_WAIT;
					} else {
						// TX start failed - go to sleep
						s_uwb_state = UWB_STATE_SLEEP_START;
					}
				}
				else
				{
					// No ACK number available - go to sleep
					s_uwb_state = UWB_STATE_SLEEP_START;
				}
			}
			// If RX is still busy, stay in ACK state and wait
#else
			// New ACK logic: Send ACK 3 times for reliability
			//static int s_ack_retry_count = 0;
			
			// Wait for RX to be inactive before sending ACK
			if (!Api_uwb_rx_is_busy())
			{
				if (s_ack_retry_count < 5)
				{
					if (!Api_uwb_tx_is_busy())
					{
						if (Aply_uwb_tx_send_ack_packet())
						{
								// ACK packet prepared - start TX
							if(s_ack_retry_count != 5){
								Api_uwb_start_tx(false, false); // No auto sleep, no RX
								s_ack_retry_count++;
								s_uwb_state = UWB_STATE_ACK;
								}
							else{
								s_ack_retry_count = 0;
								Api_uwb_start_tx(true, false);
								s_uwb_state = UWB_STATE_TX_WAIT;
							}
						}
						else
						{
							// No ACK number available - go to sleep
							s_ack_retry_count = 0;
							s_uwb_state = UWB_STATE_SLEEP_START;
						}
					}
					// If TX is busy, stay in ACK state and wait
				}
				else
				{
					// 3 ACK transmissions completed - go to sleep
					s_ack_retry_count = 0;
					s_uwb_state = UWB_STATE_SLEEP_START;
				}
			}
			// If RX is still busy, stay in ACK state and wait
#endif
			break;
			
		case UWB_STATE_SLEEP_START: // SLEEP_START - Start UWB device sleep process
			if (Api_uwb_start_sleep())
			{
				s_uwb_state = UWB_STATE_SLEEP_WAIT;
			}
			else
			{
				s_uwb_state = UWB_STATE_IDLE;
			}
			break;
			
		case UWB_STATE_SLEEP_WAIT: // SLEEP_WAIT - Wait for sleep process to complete
			if (Api_uwb_is_sleep_complete())
			{
				// Process packet scheduling for next cycle
				//Aply_tag_scheduler_normal_process_cycle_complete();
				
				// Set sleep flag to prevent immediate restart
				s_uwb_sleep_flag = true;
				
				// Reset to IDLE for next cycle
				s_uwb_state = UWB_STATE_IDLE;
			}
			else if (!Api_uwb_sleep_is_busy())
			{
				// Sleep process failed or timed out - set sleep flag
				s_uwb_sleep_flag = true;
				
				// Reset to IDLE for next cycle
				s_uwb_state = UWB_STATE_IDLE;
			}
			break;
			
		default:
			// Invalid state, reset to initial state
			s_uwb_state = UWB_STATE_IDLE;
			break;
	}
}

void Aply_tag_manager_init(void)
{
	s_tag_manager_state = TAG_MANAGER_STATE_WAIT_UWB_INIT;
	//LOG_API_UWB("Tag Manager: Initialized\r\n");
}

void Aply_tag_manager_process(void)
{	
	if  (Api_failsafe_blocking_system())
	{
		return;
	}
	
	switch (s_tag_manager_state)
	{
		case TAG_MANAGER_STATE_WAIT_UWB_INIT:
			if  (Api_uwb_is_device_ready() == true)
			{
				Aply_get_uwb_mac_address();
				Aply_nfc_check_and_init_tag_configuration();
				Aply_tag_scheduler_init();
				Aply_tag_configuration_set_motion_config();
				/* Initialize UWB TX packets */
				Aply_uwb_tx_update_packets();
				s_tag_manager_state = TAG_MANAGER_STATE_NORMAL;
			}
			break;
			
		case TAG_MANAGER_STATE_NORMAL:
			// Call NFC process function
			management_nfc_process();
			
			// Call UWB transmission process function
			management_uwb_transmission_process();
			
			// Call LED process function
			management_led_process();
			
			// Call Battery check process function
			management_battery_check_process();
			
			// Check if all tasks are complete for sleep entry (State-based approach)
			if (management_is_ready_for_sleep_state_based())
			{
				// Check motion wakeup before entering sleep
				management_check_motion_wakeup();
				
				// Check NFC field detection before entering sleep
				if (s_nfc_get_read_b || s_first_init_flag) {
					// NFC field detected - check and apply NFC configuration
					Aply_nfc_check_and_init_tag_configuration();
					
					// Apply UWB settings from updated configuration
					Aply_nfc_apply_uwb_settings();
					
					// Reinitialize packet scheduler with new configuration
					Aply_tag_scheduler_init();

					// Initialize motion detection
					Aply_tag_configuration_set_motion_config();

					// Update TX packets with new configuration
					Aply_uwb_tx_update_packets();
					
					// Set info packet flag to send updated configuration
					Aply_uwb_tx_set_packet_flags(true, false, false); // info=true, battery=false, bc=false
					
					s_nfc_get_read_b = false; // Reset flag after processing
					s_first_init_flag = false;
				}
				
				// Check for backchannel data
				if (Aply_uwb_rx_is_bc_data_received()) {
					if (Aply_backchannel_parse_bd_packet()) {
						// Backchannel data parsed successfully - process it
						Aply_backchannel_process_data();
					}
					// Clear backchannel data after processing
					Aply_uwb_rx_clear_bc_data_received();
				}
				
				// All tasks complete - ready for sleep
				
				// Enter sleep mode based on current state
				management_enter_sleep_mode();
				
				// Reset sleep flag for next cycle
				s_uwb_sleep_flag = false;
			}
			break;
			
		default:
			s_tag_manager_state = TAG_MANAGER_STATE_NORMAL;
			break;
	}
}

tag_manager_state_e Aply_tag_manager_get_state(void)
{
	return s_tag_manager_state;
}

/**
 * @brief Enter sleep mode based on current state
 * @details Enters sleep mode with appropriate duration based on motion sleep flag.
 *          Motion sleep mode uses motion sleep TX interval.
 *          Normal mode uses random TX interval with random offset.
 */
static void management_enter_sleep_mode(void)
{
	if (Aply_tag_scheduler_get_motion_sleep_flag()) {
		// Motion sleep mode: get motion sleep TX interval from configuration
		uint32_t l_motion_sleep_interval = 0;
		l_motion_sleep_interval |= Aply_tag_configuration_get_array_field(CONFIG_FIELD_SM_REFRESH_INTERVAL, 0);
		l_motion_sleep_interval |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_SM_REFRESH_INTERVAL, 1) << 8;
		l_motion_sleep_interval |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_SM_REFRESH_INTERVAL, 2) << 16;
		l_motion_sleep_interval |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_SM_REFRESH_INTERVAL, 3) << 24;
		
		Api_sleep_setSleepTime(l_motion_sleep_interval);
		Api_sleep_start_sleep();
	} else {
		// Normal mode: get normal TX interval with random offset
		uint32_t l_normal_interval = 0;
		l_normal_interval |= Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 0);
		l_normal_interval |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 1) << 8;
		l_normal_interval |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 2) << 16;
		l_normal_interval |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 3) << 24;
		
		// Apply random offset if enabled (±90% deviation)
		if (Aply_tag_configuration_get_field(CONFIG_FIELD_RANDOM_DEV)) {
			uint32_t l_random_range = (l_normal_interval * 90) / 100; // 90% of base interval
			uint32_t l_random_offset = (rand() % (l_random_range * 2 + 1)) - l_random_range;
			uint32_t l_result = (int32_t)l_normal_interval + l_random_offset;
			l_normal_interval = (l_result < 100) ? 100 : (uint32_t)l_result; // Minimum 100ms
		}
		
		Api_sleep_setSleepTime(l_normal_interval);
		Api_sleep_start_sleep();
	}
}

