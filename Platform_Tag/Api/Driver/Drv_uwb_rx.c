/**
 * @file    Drv_uwb_rx.c
 * @brief   UWB Driver RX State Machine Implementation
 * 
 * This module contains the receive state machine that handles all aspects
 * of UWB message reception including receiver enabling, message monitoring,
 * data extraction, validation, and timeout handling.
 * 
 * Key Features:
 * - Non-blocking RX state machine
 * - Receiver enabling and configuration
 * - Message reception monitoring with timeout
 * - Received data extraction and validation
 * - Sequence number extraction
 * - Status flag management
 * - Continuous reception support
 * 
 * @author  Tag Platform Development Team
 * @version 1.0
 * @date    2025
 */

#include "Drv_uwb_internal.h"

// ========================================================================================
// Static Variables (RX Related)
// ========================================================================================

/**
 * @brief RX state machine variables
 */
static uwb_rx_state_e drv_uwb_rx_state_g = UWB_RX_STATE_IDLE;
static bool drv_uwb_rx_enabled_b = false;

/**
 * @brief RX message buffer and status
 */
static uwb_rx_message_t drv_uwb_rx_message_g;
static bool drv_uwb_rx_data_available_b = false;
static uint32_t drv_uwb_rx_timeout_ms_ui32 = UWB_RX_TIMEOUT_DEFAULT_MS;

// ========================================================================================
// Private Function Declarations
// ========================================================================================

static void uwb_rx_state_idle(void);
static void uwb_rx_state_enable_rx(void);
static void uwb_rx_state_wait_and_process(void);
static void uwb_rx_state_complete(void);

// ========================================================================================
// Private Function Implementations
// ========================================================================================

/**
 * @brief RX state machine - IDLE state handler
 * 
 * @details RX idle state - waits for enable command to start reception.
 */
static void uwb_rx_state_idle(void)
{
	// RX is idle, waiting for enable command
	// No action needed in this state
}

/**
 * @brief RX state machine - Enable RX state handler
 * 
 * @details Enables the receiver and starts listening for incoming messages.
 * Sets up timeout monitoring for reception completion.
 */
static void uwb_rx_state_enable_rx(void)
{
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
	drv_uwb_rx_state_g = UWB_RX_STATE_WAIT_AND_PROCESS;
	LOG_API_UWB("UWB RX enabled\r\n");
}

/**
 * @brief RX state machine - Wait for reception and process received data
 * 
 * @details Combined state handler that waits for RX completion, reads data,
 * processes it, and clears status flags all in one state transition.
 * Handles both successful reception and timeout scenarios.
 */
static void uwb_rx_state_wait_and_process(void)
{
	uint32_t l_status = dwt_readsysstatuslo();
	
	Drv_uwb_set_sequence_timer(1);
	
	// Check for RX completion or timeout
	if (l_status & DWT_INT_RXFCG_BIT_MASK) 
	{
		// RX completed successfully - read and process data
		uint16_t l_frame_len;
		uint8_t l_rng_bit;
		
		// Read frame length using the proper API function
		l_frame_len = dwt_getframelength(&l_rng_bit);
		
		if (l_frame_len <= UWB_RX_MSG_BUFFER_SIZE) 
		{
			// Read received data
			dwt_readrxdata(drv_uwb_rx_message_g.data, l_frame_len - UWB_FCS_LENGTH, 0);
			drv_uwb_rx_message_g.length = l_frame_len - UWB_FCS_LENGTH;
			
			// Extract sequence number if available
			if (drv_uwb_rx_message_g.length >= 2) 
			{
				drv_uwb_rx_message_g.sequence_number = drv_uwb_rx_message_g.data[1];
			} 
			else 
			{
				drv_uwb_rx_message_g.sequence_number = 0xFF;
			}
			
			// Set timestamp (using a simple counter for now)
			drv_uwb_rx_message_g.timestamp = 0; // TODO: Implement proper timestamp
			
			// Validate data (basic checks)
			drv_uwb_rx_message_g.valid = (drv_uwb_rx_message_g.length > 0 && 
										  drv_uwb_rx_message_g.length <= UWB_RX_MSG_BUFFER_SIZE);
			
			if (drv_uwb_rx_message_g.valid) 
			{
				LOG_API_UWB("UWB_RX_COMPLETE: seq=%d, len=%d\r\n", 
				drv_uwb_rx_message_g.sequence_number, 
				drv_uwb_rx_message_g.length);
				drv_uwb_rx_data_available_b = true;
			}
		} 
		else 
		{
			LOG_API_UWB("UWB RX frame too long: %d\r\n", l_frame_len);
		}
		
		// Clear status flags and move to complete
		dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK | DWT_INT_RXFTO_BIT_MASK);
		drv_uwb_rx_state_g = UWB_RX_STATE_COMPLETE;
		
	} 
	else if (l_status & DWT_INT_RXFTO_BIT_MASK) 
	{
		// RX timeout occurred
		LOG_API_UWB("UWB RX UWB timeout\r\n");
		dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK | DWT_INT_RXFTO_BIT_MASK);
		drv_uwb_rx_state_g = UWB_RX_STATE_ENABLE_RX;		
	} 
	else 
	{
		// Still waiting for RX completion
		if (Drv_uwb_get_sequence_timeout_timer() == 0u) 
		{
			LOG_API_UWB("UWB RX timeout\r\n");
			dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK | DWT_INT_RXFTO_BIT_MASK);
			drv_uwb_rx_state_g = UWB_RX_STATE_IDLE;
			drv_uwb_rx_enabled_b = false;
		}
	}
}

/**
 * @brief RX state machine - Complete state handler
 * 
 * @details Handles RX completion and decides whether to continue
 * reception or return to idle state.
 */
static void uwb_rx_state_complete(void)
{
	if (drv_uwb_rx_enabled_b) 
	{
		drv_uwb_rx_state_g = UWB_RX_STATE_ENABLE_RX; // Continue RX if enabled
	} 
	else 
	{
		drv_uwb_rx_state_g = UWB_RX_STATE_IDLE;
	}
}

// ========================================================================================
// Getter/Setter Functions
// ========================================================================================

/**
 * @brief Get RX state
 */
uwb_rx_state_e Drv_uwb_rx_get_state(void)
{
	return drv_uwb_rx_state_g;
}

/**
 * @brief Set RX state
 */
void Drv_uwb_rx_set_state(uwb_rx_state_e state)
{
	drv_uwb_rx_state_g = state;
}

/**
 * @brief Get RX enabled status
 */
bool Drv_uwb_rx_get_enabled(void)
{
	return drv_uwb_rx_enabled_b;
}

/**
 * @brief Set RX enabled status
 */
void Drv_uwb_rx_set_enabled(bool enabled)
{
	drv_uwb_rx_enabled_b = enabled;
}

void Drv_uwb_rx_forceStop(void)
{
	dwt_forcetrxoff();
}

/**
 * @brief Get RX message
 */
bool Drv_uwb_rx_get_message(uwb_rx_message_t *rx_msg)
{
	if (rx_msg == NULL)
	{
		return false;
	}
	memcpy(rx_msg, &drv_uwb_rx_message_g, sizeof(uwb_rx_message_t));
	return true;
}

/**
 * @brief Clear RX message
 */
void Drv_uwb_rx_clear_message(void)
{
	memset(&drv_uwb_rx_message_g, 0, sizeof(uwb_rx_message_t));
}

/**
 * @brief Get RX data available status
 */
bool Drv_uwb_rx_get_data_available(void)
{
	return drv_uwb_rx_data_available_b;
}

/**
 * @brief Set RX data available status
 */
void Drv_uwb_rx_set_data_available(bool available)
{
	drv_uwb_rx_data_available_b = available;
}

/**
 * @brief Get RX timeout
 */
uint32_t Drv_uwb_rx_get_timeout(void)
{
	return drv_uwb_rx_timeout_ms_ui32;
}

/**
 * @brief Set RX timeout
 */
void Drv_uwb_rx_set_timeout(uint32_t timeout_ms)
{
	if (timeout_ms > 0)
	{
		drv_uwb_rx_timeout_ms_ui32 = timeout_ms;
	}
}

// ========================================================================================
// Public Function Implementations
// ========================================================================================

/**
 * @brief Execute one step of RX state machine
 * 
 * @details Non-blocking state machine that processes one state per call.
 * Must be called periodically to advance through reception sequence.
 */
void Drv_uwb_rx_process_state_machine(void)
{
	if (!Drv_uwb_get_device_wakeup()) 
	{
		return;
	}

	if (0 != Drv_uwb_get_sequence_timer()) 
	{
		return;
	}

	if (!drv_uwb_rx_enabled_b) 
	{
		return;
	}		

	switch (drv_uwb_rx_state_g) 
	{
		case UWB_RX_STATE_IDLE:
			uwb_rx_state_idle();
			break;
			
		case UWB_RX_STATE_ENABLE_RX:
			uwb_rx_state_enable_rx();
			break;
			
		case UWB_RX_STATE_WAIT_AND_PROCESS:
			uwb_rx_state_wait_and_process();
			break;
			
		case UWB_RX_STATE_COMPLETE:
			uwb_rx_state_complete();
			break;
			
		default:
			drv_uwb_rx_state_g = UWB_RX_STATE_IDLE;
			break;
	}
}

