/**
 * @file    Drv_uwb.c
 * @brief   UWB Driver Layer Main Implementation
 * 
 * This module contains the main driver implementation including global variables,
 * common functions, and coordinating functions for the UWB driver subsystem.
 * It serves as the central hub that coordinates all state machines and provides
 * the main driver interface functions.
 * 
 * Key Features:
 * - Global variable definitions and management
 * - Driver system initialization and coordination
 * - Common utility functions
 * - State machine coordination
 * - Timer management
 * - System reset and recovery functions
 * 
 * @author  Tag Platform Development Team
 * @version 1.0
 * @date    2025
 */

#include "Drv_uwb_internal.h"
#include "Drv_uwb.h"

// ========================================================================================
// Static Variables (Common/Timer)
// ========================================================================================

/**
 * @brief Common timer variables shared across all state machines
 */
static uint32_t drv_uwb_sequence_timer_ms_ui32 = 0;
static uint32_t drv_uwb_sequence_timeout_timer_ms_ui32 = 0;

/**
 * @brief Device ready status
 */
static bool drv_uwb_device_ready_b = false;
static bool drv_uwb_device_wakeup_b = false;

// ========================================================================================
// Main Driver Functions (called from Api_uwb.c)
// ========================================================================================

/**
 * @brief Initialize driver system - must be called at startup
 * 
 * @details Initializes all driver variables and prepares the system for operation.
 * This function must be called before any other driver functions.
 */
void Drv_uwb_init_system(void)
{
	// Initialize all driver variables to default values
	Drv_uwb_init_variables();
	
	LOG_API_UWB("UWB Driver System Initialized\r\n");
}

/**
 * @brief Initialize all driver variables to default values
 * 
 * @details Resets all state machine variables, timers, and status flags
 * to their default values. Used during system startup and reset operations.
 */
void Drv_uwb_init_variables(void)
{
	Drv_uwb_init_set_state(DEVICE_INIT_STATE_IDLE);
	Drv_uwb_tx_set_state(UWB_TX_STATE_IDLE);
	Drv_uwb_tx_set_enabled(false);
	Drv_uwb_rx_set_state(UWB_RX_STATE_IDLE);
	Drv_uwb_rx_set_enabled(false);
	Drv_uwb_rx_set_data_available(false);
	Drv_uwb_rx_set_timeout(UWB_RX_TIMEOUT_DEFAULT_MS);
	Drv_uwb_rx_clear_message();
	Drv_uwb_power_set_wakeup_state(UWB_WAKEUP_STATE_IDLE);
	Drv_uwb_power_set_sleep_state(UWB_SLEEP_STATE_IDLE);
	Drv_uwb_set_sequence_timer(0);
	Drv_uwb_set_sequence_timeout_timer(0);
	Drv_uwb_set_device_ready(false);
	Drv_uwb_set_device_wakeup(false);
}

/**
 * @brief Decrement all driver timers (called every 1ms)
 * 
 * @details Decrements all active timer variables by 1ms. This function
 * is critical for state machine timing and must be called every 1ms.
 */
void Drv_uwb_timer_tick(void)
{
	if (drv_uwb_sequence_timer_ms_ui32 > 0) 
	{
		drv_uwb_sequence_timer_ms_ui32--;
	}
			
	if (drv_uwb_sequence_timeout_timer_ms_ui32 > 0u)
	{
		drv_uwb_sequence_timeout_timer_ms_ui32--;
	}		
}

/**
 * @brief Reset all state machines and variables
 * 
 * @details Resets all state machines to idle and clears all variables.
 * Used for error recovery and reinitialization scenarios.
 */
void Drv_uwb_reset_all_states(void)
{
	Drv_uwb_init_set_state(DEVICE_INIT_STATE_IDLE);
	Drv_uwb_set_device_ready(false);
	Drv_uwb_set_device_wakeup(false);
	Drv_uwb_tx_set_state(UWB_TX_STATE_IDLE);
	Drv_uwb_tx_set_enabled(false);
	Drv_uwb_rx_set_state(UWB_RX_STATE_IDLE);
	Drv_uwb_rx_set_enabled(false);
	Drv_uwb_rx_set_data_available(false);
	Drv_uwb_rx_set_timeout(UWB_RX_TIMEOUT_DEFAULT_MS);
	Drv_uwb_rx_clear_message();
	Drv_uwb_power_set_wakeup_state(UWB_WAKEUP_STATE_IDLE);
	Drv_uwb_power_set_sleep_state(UWB_SLEEP_STATE_IDLE);
	Drv_uwb_set_sequence_timer(0);
	Drv_uwb_set_sequence_timeout_timer(0);
}

// ========================================================================================
// State Machine Getter Functions (Wrapper functions for backward compatibility)
// ========================================================================================

/**
 * @brief Get device ready status
 * @return true if device is ready, false otherwise
 */
bool Drv_uwb_get_device_ready(void)
{
	return drv_uwb_device_ready_b;
}

bool Drv_uwb_get_device_wakeup(void)
{
	return drv_uwb_device_wakeup_b;
}

/**
 * @brief Set device ready status
 * @param[in] ready New device ready status
 */
void Drv_uwb_set_device_ready(bool ready)
{
	drv_uwb_device_ready_b = ready;
}

void Drv_uwb_set_device_wakeup(bool ready)
{
	drv_uwb_device_wakeup_b = ready;
}

/**
 * @brief Get sequence timer value
 * @return Current sequence timer value in milliseconds
 */
uint32_t Drv_uwb_get_sequence_timer(void)
{
	return drv_uwb_sequence_timer_ms_ui32;
}

/**
 * @brief Get sequence timeout timer value
 * @return Current sequence timeout timer value in milliseconds
 */
uint32_t Drv_uwb_get_sequence_timeout_timer(void)
{
	return drv_uwb_sequence_timeout_timer_ms_ui32;
}

// ========================================================================================
// State Machine Setter Functions (Wrapper functions for backward compatibility)
// ========================================================================================

/**
 * @brief Set sequence timer value
 * @param[in] timer_ms New sequence timer value in milliseconds
 */
void Drv_uwb_set_sequence_timer(uint32_t timer_ms)
{
	drv_uwb_sequence_timer_ms_ui32 = timer_ms;
}

/**
 * @brief Set sequence timeout timer value
 * @param[in] timer_ms New sequence timeout timer value in milliseconds
 */
void Drv_uwb_set_sequence_timeout_timer(uint32_t timer_ms)
{
	drv_uwb_sequence_timeout_timer_ms_ui32 = timer_ms;
}

