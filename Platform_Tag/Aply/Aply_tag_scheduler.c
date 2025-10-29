/**
 * @file Aply_tag_scheduler.c
 * @brief Tag scheduler module implementation for TEIA platform
 * @author Platform Tag Team
 * @date 2025
 * @version 1.0
 */

#include "Aply_tag_scheduler.h"
#include "Aply_uwb_tx.h"
#include "Aply_tag_configuration.h"
#include "def_packet.h"
#include "def_config.h"
#include "Api_motion.h"

// Static variables for packet scheduling counters
static uint32_t s_normal_bc_packet_counter;
static uint32_t s_motion_sleep_bc_packet_counter;
static uint32_t s_bat_packet_counter;
static uint32_t s_info_packet_counter;
static uint32_t s_start_info_packet_counter;
static uint32_t s_led_counter;          // LED counter
static uint32_t s_battery_check_counter; // Battery check counter
static bool s_normal_bc_enabled = false;  // Normal BC enabled flag
static bool s_motion_sleep_bc_enabled = false;  // Motion sleep BC enabled flag

// Static flags for LED and battery functionality
static bool s_led_flag = false;         // LED action flag
static bool s_battery_check_flag = false; // Battery check action flag

// Motion sleep functionality
static uint32_t s_motion_sleep_counter = 0;  // Motion sleep counter
static bool s_motion_sleep_flag = false;     // Motion sleep flag
static bool s_motion_sleep_enabled = false;  // Motion sleep enabled flag

void Aply_tag_scheduler_init(void)
{
	// Get TX interval for time-based counter calculation (4-byte array)
	uint32_t l_tx_interval_ms = 0;
	l_tx_interval_ms |= Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 0);
	l_tx_interval_ms |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 1) << 8;
	l_tx_interval_ms |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 2) << 16;
	l_tx_interval_ms |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 3) << 24;
	
	// Initialize BC packet scheduling counters (using system configuration)
	s_normal_bc_packet_counter = Aply_tag_configuration_get_field(CONFIG_FIELD_BC_PERIOD_RI);  // Normal state BC period
	s_normal_bc_enabled = (s_normal_bc_packet_counter > 0);  // Enable normal BC only if period > 0
	
	s_motion_sleep_bc_packet_counter = Aply_tag_configuration_get_field(CONFIG_FIELD_BC_PERIOD_RISM);  // Motion sleep BC period
	s_motion_sleep_bc_enabled = (s_motion_sleep_bc_packet_counter > 0);  // Enable motion sleep BC only if period > 0
	
	// Initialize common packet scheduling counters
	s_bat_packet_counter = BATTERY_PACKET_INTERVAL_COUNT;             // 15 cycles
	s_info_packet_counter = INFO_PACKET_INTERVAL_COUNT;               // 15 cycles  
	s_start_info_packet_counter = START_INFO_PACKET_INTERVAL_COUNT_DEFAULT; // 3 times
	
	// Initialize time-based counters (divide time by TX interval)
	s_led_counter = (l_tx_interval_ms > 0) ? (LED_BLINK_INTERVAL_MS_DEFAULT / l_tx_interval_ms) : 10;
	s_battery_check_counter = (l_tx_interval_ms > 0) ? (ADC_MEASUREMENT_INTERVAL_MS_DEFAULT / l_tx_interval_ms) : 5;
	
	// Initialize motion sleep functionality
	s_motion_sleep_enabled = Aply_tag_configuration_get_field(CONFIG_FIELD_MCR);
	if (s_motion_sleep_enabled) {
		// Get motion sleep time from configuration (4-byte array)
		uint32_t l_motion_sleep_time_ms = 0;
		l_motion_sleep_time_ms |= Aply_tag_configuration_get_array_field(CONFIG_FIELD_MOTION_SLEEP_TIME, 0);
		l_motion_sleep_time_ms |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_MOTION_SLEEP_TIME, 1) << 8;
		l_motion_sleep_time_ms |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_MOTION_SLEEP_TIME, 2) << 16;
		l_motion_sleep_time_ms |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_MOTION_SLEEP_TIME, 3) << 24;
		
		s_motion_sleep_counter = (l_tx_interval_ms > 0) ? (l_motion_sleep_time_ms / l_tx_interval_ms) : 30;
	}
	
	// Scheduler initialized
}

void Aply_tag_scheduler_normal_process_cycle_complete(void)
{
	// Decrement packet counters
	if (s_motion_sleep_flag) {
		// Motion sleep mode: use motion sleep BC counter
		if (s_motion_sleep_bc_enabled) s_motion_sleep_bc_packet_counter--;
	} else {
		// Normal mode: use normal BC counter
		if (s_normal_bc_enabled) s_normal_bc_packet_counter--;
	}

    if  (s_bat_packet_counter > 0)
    {
      s_bat_packet_counter--;
    }
    if  (s_led_counter > 0)
    {
      s_led_counter--;
    }
    if  (s_battery_check_counter > 0)
    {
      s_battery_check_counter--;
    }
	
	bool info_flag = false;
	bool battery_flag = false;
	bool bc_flag = false;
	
	// Process BC packet functionality (only if enabled)
	if (s_motion_sleep_flag) {
		// Motion sleep mode: use motion sleep BC counter
		if (s_motion_sleep_bc_enabled && s_motion_sleep_bc_packet_counter == 0) {
			bc_flag = true;
			s_motion_sleep_bc_packet_counter = Aply_tag_configuration_get_field(CONFIG_FIELD_BC_PERIOD_RISM);
		}
	} else {
		// Normal mode: use normal BC counter
		if (s_normal_bc_enabled && s_normal_bc_packet_counter == 0) {
			bc_flag = true;
			s_normal_bc_packet_counter = Aply_tag_configuration_get_field(CONFIG_FIELD_BC_PERIOD_RI);
		}
	}
	
	// Process info packet functionality (send 3 times at startup)
	if (s_start_info_packet_counter > 0)
	{
		s_start_info_packet_counter--;
		info_flag = true;
	}

	// Process battery packet functionality (every 15 cycles)
	if (s_bat_packet_counter == 0) 
	{
		battery_flag = true;
		s_bat_packet_counter = BATTERY_PACKET_INTERVAL_COUNT;
		s_info_packet_counter--;
		
		if (s_info_packet_counter == 0) 
		{
			// Override battery with info packet (every 15 cycles)
			battery_flag = false;
			info_flag = true;
			s_info_packet_counter = INFO_PACKET_INTERVAL_COUNT;
		}
	}
	
	// Process LED functionality (time-based)
	if (s_led_counter == 0) 
	{
		s_led_flag = true;
		// Get TX interval (4-byte array)
		uint32_t l_tx_interval_ms = 0;
		l_tx_interval_ms |= Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 0);
		l_tx_interval_ms |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 1) << 8;
		l_tx_interval_ms |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 2) << 16;
		l_tx_interval_ms |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 3) << 24;
		s_led_counter = (l_tx_interval_ms > 0) ? (LED_BLINK_INTERVAL_MS_DEFAULT / l_tx_interval_ms) : 10;
	}
	
	// Process battery check functionality (time-based)
	if (s_battery_check_counter == 0) 
	{
		s_battery_check_flag = true;
		// Get TX interval (4-byte array)
		uint32_t l_tx_interval_ms = 0;
		l_tx_interval_ms |= Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 0);
		l_tx_interval_ms |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 1) << 8;
		l_tx_interval_ms |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 2) << 16;
		l_tx_interval_ms |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 3) << 24;
		s_battery_check_counter = (l_tx_interval_ms > 0) ? (ADC_MEASUREMENT_INTERVAL_MS_DEFAULT / l_tx_interval_ms) : 5;
	}
	
	// Process motion sleep functionality (only if enabled)
	if (s_motion_sleep_enabled) {
		// Only process if not already in motion sleep mode
		if (!s_motion_sleep_flag) {
			if (Api_motion_check_motion_by_polling()) {
				// Motion detected - reset counter and clear flag
				// Get TX interval (4-byte array)
				uint32_t l_tx_interval_ms = 0;
				l_tx_interval_ms |= Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 0);
				l_tx_interval_ms |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 1) << 8;
				l_tx_interval_ms |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 2) << 16;
				l_tx_interval_ms |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 3) << 24;
				
				// Get motion sleep time from configuration (4-byte array)
				uint32_t l_motion_sleep_time_ms = 0;
				l_motion_sleep_time_ms |= Aply_tag_configuration_get_array_field(CONFIG_FIELD_MOTION_SLEEP_TIME, 0);
				l_motion_sleep_time_ms |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_MOTION_SLEEP_TIME, 1) << 8;
				l_motion_sleep_time_ms |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_MOTION_SLEEP_TIME, 2) << 16;
				l_motion_sleep_time_ms |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_MOTION_SLEEP_TIME, 3) << 24;
				
				s_motion_sleep_counter = (l_tx_interval_ms > 0) ? (l_motion_sleep_time_ms / l_tx_interval_ms) : 30;
			} else {
				// No motion - decrement counter
				if (s_motion_sleep_counter > 0) {
					s_motion_sleep_counter--;
				}
				
				// Set motion sleep flag when counter reaches 0
				if (s_motion_sleep_counter == 0) {
					s_motion_sleep_flag = true;
					Api_motion_enable_interrupt();  // Enable interrupt for motion sleep entry
				}
			}
		}
	}
	
	// Set packet flags for next transmission
	Aply_uwb_tx_set_packet_flags(info_flag, battery_flag, bc_flag);
}

/**
 * @brief Get LED flag status
 * @details Returns current LED flag status without modifying it
 * @return true if LED action is needed, false otherwise
 */
bool Aply_tag_scheduler_get_led_flag(void)
{
	return s_led_flag;
}

/**
 * @brief Clear LED flag
 * @details Resets the LED flag to false
 * @return None
 */
void Aply_tag_scheduler_clear_led_flag(void)
{
	s_led_flag = false;
}

/**
 * @brief Get battery check flag status
 * @details Returns current battery check flag status without modifying it
 * @return true if battery check is needed, false otherwise
 */
bool Aply_tag_scheduler_get_battery_flag(void)
{
	return s_battery_check_flag;
}

/**
 * @brief Clear battery check flag
 * @details Resets the battery check flag to false
 * @return None
 */
void Aply_tag_scheduler_clear_battery_flag(void)
{
	s_battery_check_flag = false;
}

/**
 * @brief Get motion sleep flag status
 * @details Returns current motion sleep flag status without modifying it
 * @return true if motion sleep should be activated, false otherwise
 */
bool Aply_tag_scheduler_get_motion_sleep_flag(void)
{
	return s_motion_sleep_flag;
}

/**
 * @brief Clear motion sleep flag
 * @details Resets the motion sleep flag to false
 * @return None
 */
void Aply_tag_scheduler_clear_motion_sleep_flag(void)
{
	s_motion_sleep_flag = false;
}

/**
 * @brief Reset motion sleep counter
 * @details Resets the motion sleep counter to initial value for motion detection restart
 * @return None
 */
void Aply_tag_scheduler_reset_motion_sleep_counter(void)
{
	// Get TX interval (4-byte array)
	uint32_t l_tx_interval_ms = 0;
	l_tx_interval_ms |= Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 0);
	l_tx_interval_ms |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 1) << 8;
	l_tx_interval_ms |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 2) << 16;
	l_tx_interval_ms |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_REFRESH_INTERVAL, 3) << 24;
	
	// Get motion sleep time from configuration (4-byte array)
	uint32_t l_motion_sleep_time_ms = 0;
	l_motion_sleep_time_ms |= Aply_tag_configuration_get_array_field(CONFIG_FIELD_MOTION_SLEEP_TIME, 0);
	l_motion_sleep_time_ms |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_MOTION_SLEEP_TIME, 1) << 8;
	l_motion_sleep_time_ms |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_MOTION_SLEEP_TIME, 2) << 16;
	l_motion_sleep_time_ms |= (uint32_t)Aply_tag_configuration_get_array_field(CONFIG_FIELD_MOTION_SLEEP_TIME, 3) << 24;
	
	s_motion_sleep_counter = (l_tx_interval_ms > 0) ? (l_motion_sleep_time_ms / l_tx_interval_ms) : 30;
}

/**
 * @brief Set LED counter to 1 for backchannel LED control
 * @details Sets the LED counter to 1 to trigger immediate LED blink on next cycle
 * @return None
 */
void Aply_tag_scheduler_set_led_counter_backchannel(void)
{
	s_led_counter = 1;
}

