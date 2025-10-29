/**
 * @file    Drv_uwb_tx.c
 * @brief   UWB Driver TX State Machine Implementation
 * 
 * This module contains the transmit state machine that handles all aspects
 * of UWB message transmission including data preparation, frame control
 * setup, transmission initiation, and completion handling.
 * 
 * Key Features:
 * - Non-blocking TX state machine
 * - Message data preparation and validation
 * - Frame control register configuration
 * - TX completion monitoring with timeout
 * - Status flag management
 * - Error handling and recovery
 * 
 * @author  Tag Platform Development Team
 * @version 1.0
 * @date    2025
 */

#include "Drv_uwb_internal.h"
#include "Api_failsafe.h"

// ========================================================================================
// Static Variables (TX Related)
// ========================================================================================

/**
 * @brief TX message configuration - application configurable
 */
static uint8_t drv_uwb_tx_msg_g[UWB_TX_MSG_BUFFER_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static uint8_t drv_uwb_tx_msg_length_ui8 = 100;

/**
 * @brief TX state machine variables
 */
static uwb_tx_state_e drv_uwb_tx_state_g = UWB_TX_STATE_IDLE;
static bool drv_uwb_tx_enabled_b = false;

static bool drv_uwb_tx_afterSleep_b = false;
static bool drv_uwb_tx_afterRx_b = false;

static bool drv_uwb_tx_try_b = false;

// ========================================================================================
// Private Function Declarations
// ========================================================================================

static void uwb_tx_state_start(void);
static void uwb_tx_state_prepare_data(void);
static void uwb_tx_state_write_frame_ctrl(void);
static void uwb_tx_state_start_tx(void);
static void uwb_tx_state_wait_tx_complete(void);
static void uwb_tx_state_clear_status(void);
static void uwb_tx_state_complete(void);

// ========================================================================================
// Private Function Implementations
// ========================================================================================

/**
 * @brief TX state machine - start state handler
 * 
 * @details Checks if TX is enabled and initiates transmission
 * sequence if conditions are met.
 */
static void uwb_tx_state_start(void)
{
	if (drv_uwb_tx_enabled_b)
	{
		drv_uwb_tx_state_g = UWB_TX_STATE_PREPARE_DATA;
		Drv_uwb_set_sequence_timeout_timer(0);
		LOG_API_UWB("Starting UWB TX\r\n");
	}
}

/**
 * @brief TX state machine - Prepare data state handler
 * 
 * @details Writes the TX message data to the device's transmit buffer
 * in preparation for transmission.
 */
static void uwb_tx_state_prepare_data(void)
{
	dwt_writetxdata(drv_uwb_tx_msg_length_ui8, drv_uwb_tx_msg_g, 0);
	drv_uwb_tx_state_g = UWB_TX_STATE_WRITE_FRAME_CTRL;
}

/**
 * @brief TX state machine - Write frame control state handler
 * 
 * @details Configures the frame control register with message length
 * and other transmission parameters.
 */
static void uwb_tx_state_write_frame_ctrl(void)
{
	dwt_writetxfctrl(FRAME_LENGTH(drv_uwb_tx_msg_length_ui8), 0, 0);
}

/**
 * @brief TX state machine - Start TX state handler
 * 
 * @details Initiates the actual transmission and sets up timeout
 * monitoring for completion.
 */
static void uwb_tx_state_start_tx(void)
{
	dwt_starttx(DWT_START_TX_IMMEDIATE);
	drv_uwb_tx_state_g = UWB_TX_STATE_WAIT_TX_COMPLETE;
	Drv_uwb_set_sequence_timeout_timer(UWB_TX_TIMEOUT_COUNT);
	Drv_uwb_set_sequence_timer(1u);
}

/**
 * @brief TX state machine - Wait TX complete state handler
 * 
 * @details Monitors for TX completion by checking status register.
 * Handles both successful completion and timeout scenarios.
 */
static void uwb_tx_state_wait_tx_complete(void)
{
	uint32_t l_status = dwt_readsysstatuslo();

	if (l_status & DWT_INT_TXFRS_BIT_MASK) 
	{
		drv_uwb_tx_state_g = UWB_TX_STATE_CLEAR_STATUS;
	} 
	else 
	{
		Drv_uwb_set_sequence_timer(1u);
		if (Drv_uwb_get_sequence_timeout_timer() == 0u) 
		{
			LOG_API_UWB("UWB TX timeout error:0x%X\r\n", l_status);
			drv_uwb_tx_state_g = UWB_TX_STATE_IDLE;
			drv_uwb_tx_enabled_b = false;
		}
	}
}

/**
 * @brief TX state machine - Clear status state handler
 * 
 * @details Clears the TX completion status flags to prepare
 * for the next transmission.
 */
static void uwb_tx_state_clear_status(void)
{
	dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
	drv_uwb_tx_state_g = UWB_TX_STATE_COMPLETE;
}

/**
 * @brief TX state machine - Complete state handler
 * 
 * @details Handles successful transmission completion including
 * status updates and logging.
 */
static void uwb_tx_state_complete(void)
{
	LOG_API_UWB("UWB_TX_COMPLETE: seq=%d\r\n", drv_uwb_tx_msg_g[7]);
	drv_uwb_tx_state_g = UWB_TX_STATE_IDLE;

	drv_uwb_tx_enabled_b = false;
}

// ========================================================================================
// Getter/Setter Functions
// ========================================================================================

/**
 * @brief Get TX state
 */
uwb_tx_state_e Drv_uwb_tx_get_state(void)
{
	return drv_uwb_tx_state_g;
}

/**
 * @brief Set TX state
 */
void Drv_uwb_tx_set_state(uwb_tx_state_e state)
{
	drv_uwb_tx_state_g = state;
}

void Drv_uwb_tx_set_afterSleep(bool afterSleep)
{
	drv_uwb_tx_afterSleep_b = afterSleep;
}

void Drv_uwb_tx_set_afterRx(bool afterRx)
{
	drv_uwb_tx_afterRx_b = afterRx;
}

bool Drv_uwb_tx_get_afterSleep(void)
{
	return drv_uwb_tx_afterSleep_b;
}

bool Drv_uwb_tx_get_afterRx(void)
{
	return drv_uwb_tx_afterRx_b;
}

/**
 * @brief Get TX enabled status
 */
bool Drv_uwb_tx_get_enabled(void)
{
	return drv_uwb_tx_enabled_b;
}

/**
 * @brief Set TX enabled status
 */
void Drv_uwb_tx_set_enabled(bool enabled)
{
	drv_uwb_tx_enabled_b = enabled;
}

void Drv_uwb_tx_set_try(bool enabled)
{
	drv_uwb_tx_try_b = enabled;
}

bool Drv_uwb_tx_get_try(void)
{
	return drv_uwb_tx_try_b;
}

/**
 * @brief Get TX message
 */
bool Drv_uwb_tx_get_message(uint8_t *msg, uint8_t *length)
{
	if (msg == NULL || length == NULL)
	{
		return false;
	}
	memcpy(msg, drv_uwb_tx_msg_g, drv_uwb_tx_msg_length_ui8);
	*length = drv_uwb_tx_msg_length_ui8;
	return true;
}

/**
 * @brief Set TX message
 */
bool Drv_uwb_tx_set_message(const uint8_t *msg, uint8_t length)
{
	if (msg == NULL || length == 0 || length > UWB_TX_MSG_BUFFER_SIZE)
	{
		return false;
	}
	memcpy(drv_uwb_tx_msg_g, msg, length);
	drv_uwb_tx_msg_length_ui8 = length + 2u;
	return true;
}

// ========================================================================================
// Public Function Implementations
// ========================================================================================

/**
 * @brief Blocking UWB TX sequence (optimized for power consumption)
 * 
 * @details This function performs a complete blocking transmission sequence.
 * Non-blocking approach was partially abandoned to reduce power consumption.
 * The function will block until TX completes or fails.
 * 
 * Sequence:
 * 1. Check if device is awake
 * 2. Prepare TX data (write to device buffer)
 * 3. Write frame control register
 * 4. Start transmission
 * 5. Wait for TX completion (with timeout)
 * 6. Clear status flags
 * 
 * @return API_UWB_STATUS_e
 *   - API_UWB_STATUS_SUCCESS: TX completed successfully
 *   - API_UWB_STATUS_FAIL: TX failed (device not ready or timeout)
 * 
 * @warning This function blocks CPU execution during transmission.
 *          Typical TX time is ~500us - 1ms depending on message length.
 */
API_UWB_STATUS_e Drv_uwb_tx_machine(void)
{
	uint8_t tx_option = DWT_START_TX_IMMEDIATE;
	uint32_t l_status;
	int tx_result;
	
	// Early return if device not ready or not in START state
	if (!Drv_uwb_get_device_wakeup()) 
	{
		return API_UWB_STATUS_IDLE;
	}
	
	if (drv_uwb_tx_state_g != UWB_TX_STATE_START)
	{
		return API_UWB_STATUS_IDLE;
	}
	
	LOG_API_UWB("%s - Start TX\r\n", __func__);
	
	// Check previous TX result
	l_status = dwt_readsysstatuslo();
		
	// Clear all status except error flag before sleep
	dwt_writesysstatuslo(SYS_STATUS_HPDWARN_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);
	if (l_status & SYS_STATUS_HPDWARN_BIT_MASK)
	{
		// Previous TX error
		Api_failsafe_set_fail(FAILSAFE_CODE_03);
		LOG_API_UWB("Previous TX error (HPDWARN)\r\n");
		return API_UWB_STATUS_FAIL;
	}
	else if (l_status & SYS_STATUS_TXFRS_BIT_MASK)
	{
		// Previous TX success
		Api_failsafe_set_success(FAILSAFE_CODE_03);
		LOG_API_UWB("Previous TX success\r\n");
	}
	else
	{
        #if  0
		if  (drv_uwb_tx_try_b == true)
		{
			drv_uwb_tx_try_b = false;
			Api_failsafe_set_fail(FAILSAFE_CODE_03);
			LOG_API_UWB("Previous TX error (NONE)\r\n");
			return API_UWB_STATUS_FAIL;
		}
        #endif
	}
	
	// Configure post-TX behavior
	if (Drv_uwb_tx_get_afterRx())
	{
		dwt_entersleepaftertx(0u);
		tx_option |= DWT_RESPONSE_EXPECTED;
		dwt_setrxaftertxdelay(60);
		dwt_setrxtimeout(Drv_uwb_rx_get_timeout()*1000);
		Drv_uwb_set_sequence_timeout_timer(Drv_uwb_rx_get_timeout());
		
		Drv_uwb_rx_set_enabled(true);
		Drv_uwb_rx_set_state(UWB_RX_STATE_WAIT_AND_PROCESS);
	}
	else
	{
		if (Drv_uwb_tx_get_afterSleep())
		{
			dwt_entersleepaftertx(1);
			Drv_uwb_set_device_wakeup(false);
		}
		else
		{
			dwt_entersleepaftertx(0u);
		}
	}
		
	// Prepare and transmit
	dwt_writetxdata(drv_uwb_tx_msg_length_ui8, drv_uwb_tx_msg_g, 0);
	dwt_writetxfctrl(drv_uwb_tx_msg_length_ui8, 0, 0);
	
	tx_result = dwt_starttx(tx_option);
	
	if (tx_result != DWT_SUCCESS)
	{
		// TX start failed (HW level error)
		Api_failsafe_set_fail(FAILSAFE_CODE_03);
		LOG_API_UWB("TX start failed\r\n");
		drv_uwb_tx_state_g = UWB_TX_STATE_IDLE;
		drv_uwb_tx_enabled_b = false;
		return API_UWB_STATUS_FAIL;
	}
	
	LOG_API_UWB("TX started (len=%d, seq=%d)\r\n", 
	            drv_uwb_tx_msg_length_ui8, 
	            drv_uwb_tx_msg_g[7]);	
	
	// Update state
	drv_uwb_tx_state_g = UWB_TX_STATE_IDLE;
	drv_uwb_tx_enabled_b = false;
	drv_uwb_tx_try_b = true;
	
	return API_UWB_STATUS_SUCCESS;
}

/**
 * @brief Execute one step of TX state machine
 * 
 * @details Non-blocking state machine that processes one state per call.
 * Must be called periodically to advance through transmission sequence.
 */
void Drv_uwb_tx_process_state_machine(void)
{
	if (!Drv_uwb_get_device_wakeup()) 
	{
		return;
	}

	if (Drv_uwb_get_sequence_timer() != 0)
	{
		return;
	}

	switch (drv_uwb_tx_state_g) 
	{
		case UWB_TX_STATE_IDLE:
			break;
			
		case UWB_TX_STATE_START:
			uwb_tx_state_start();
			break;
			
		case UWB_TX_STATE_PREPARE_DATA:
			uwb_tx_state_prepare_data();
			uwb_tx_state_write_frame_ctrl();
			uwb_tx_state_start_tx();
			break;
			
		case UWB_TX_STATE_WAIT_TX_COMPLETE:
			uwb_tx_state_wait_tx_complete();
			break;
			
		case UWB_TX_STATE_CLEAR_STATUS:
			uwb_tx_state_clear_status();
			uwb_tx_state_complete();
			break;
			
		case UWB_TX_STATE_COMPLETE:
			uwb_tx_state_complete();
			break;
			
		default:
			drv_uwb_tx_state_g = UWB_TX_STATE_IDLE;
			break;
	}
}

