/*! ----------------------------------------------------------------------------
 *  @file    Api_uwb.c
 *  @brief   UWB API Implementation - Public Interface Layer
 *
 *  This module provides the public API interface for UWB device operations.
 *  It acts as a wrapper around the driver layer functions and provides
 *  application-friendly interface for UWB functionality.
 *
 *  Key Features:
 *  - Public API functions for application use
 *  - Configuration management and validation
 *  - Status monitoring and control
 *  - Hardware interface initialization
 *
 * @author Modified for API/Driver separation
 * @version 1.0
 * @date 2025
 *
 */

#include <string.h>

#include "sdk_config.h"
#include "boards.h"
#include "deca_probe_interface.h"
#include "nrf_delay.h"
#include "Port.h"
#include "Api_uwb.h"
#include "Api_failsafe.h"
#include "Drv_uwb.h"
#include "Drv_uwb_spi.h"
#include "Drv_uwb_internal.h"
#include <deca_device_api.h>
#include "Func_UART_LOG.h"

// ========================================================================================
// Public API Functions - Core State Machines
// ========================================================================================

/**
 * @brief UWB Device Initialize - Blocking version
 * 
 * Blocking version of UWB device initialization. This function performs the complete
 * initialization sequence and returns only when initialization is complete (success
 * or failure). Useful for system startup where blocking initialization is acceptable.
 * 
 * @param[in] timeout_ms Maximum time to wait for initialization (0 = no timeout)
 * 
 * @return true if initialization completed successfully, false if failed or timeout
 * 
 * @warning This function will block the calling thread until initialization
 *          completes or timeout occurs. Do not call from interrupt context.
 */
bool Api_uwb_init_device_blocking(uint32_t timeout_ms)
{
	uint32_t l_timeout_counter = 0;
	uint32_t l_max_timeout = (timeout_ms > 0) ? timeout_ms : 0xFFFFFFFF;
	
	// Reset initialization state if already completed or in error
	if (Drv_uwb_init_get_state() == DEVICE_INIT_STATE_COMPLETE || Drv_uwb_init_get_state() == DEVICE_INIT_STATE_ERROR) {
		Api_uwb_reset_init();
	}

	Drv_uwb_init_set_state(DEVICE_INIT_STATE_START);
	// Perform initialization until complete or timeout
	while (!Drv_uwb_get_device_ready() && l_timeout_counter < l_max_timeout) {
		// Call the non-blocking initialization function
		Drv_uwb_init_process_state_machine();
		
		// Handle timing delays internally
		if (Drv_uwb_get_sequence_timer() > 0) {
			// Simulate timer tick for delay handling
			Api_uwb_timer_tick();
			nrf_delay_ms(1);  // 1ms delay
		}
		
		l_timeout_counter++;		
	}
	
	// Return success if device is ready, false otherwise
	return Drv_uwb_get_device_ready();
}

/**
 * @brief Timer Tick Handler
 * 
 * Decrements all UWB-related timers. This function must be called every 1ms from
 * the main loop or timer interrupt to maintain accurate timing for TX interval
 * timing and initialization delay timing.
 * 
 * @note This function is critical for proper operation of the state machines.
 *       Failure to call this regularly will cause timing issues.
 * 
 * @warning Must be called exactly every 1ms for accurate timing.
 */
void Api_uwb_timer_tick(void)
{
	Drv_uwb_timer_tick();
}

// ========================================================================================
// Public API Functions - Status and Control
// ========================================================================================

/**
 * @brief Check Device Ready Status
 * 
 * @return true if device is initialized and ready for operation, false otherwise
 * 
 * @note Device is considered ready when initialization state machine has completed
 *       successfully and all configuration is applied.
 */
bool Api_uwb_is_device_ready(void)
{
	return Drv_uwb_get_device_ready();
}

bool Api_uwb_is_device_wakeup(void)
{
	return Drv_uwb_get_device_wakeup();
}

/**
 * @brief Check TX Busy Status
 * 
 * @return true if TX state machine is actively processing, false if idle
 * 
 * @note TX is considered busy when not in UWB_TX_STATE_IDLE state.
 *       Use this to avoid configuration changes during transmission.
 */
bool Api_uwb_tx_is_busy(void)
{
	return (Drv_uwb_tx_get_enabled());
}

/**
 * @brief Check RX Busy Status
 * 
 * @return true if RX state machine is actively processing, false if idle
 * 
 * @note RX is considered busy when not in UWB_RX_STATE_IDLE state.
 *       Use this to avoid configuration changes during reception.
 */
bool Api_uwb_rx_is_busy(void)
{
	return (Drv_uwb_rx_get_enabled());
}

/**
 * @brief Check if UWB wake-up process is currently busy
 * 
 * @return true if wake-up state machine is actively processing, false if idle
 * 
 * @note Wake-up is considered busy when not in UWB_WAKEUP_STATE_IDLE state.
 */
bool Api_uwb_wakeup_is_busy(void)
{
	return (Drv_uwb_power_get_wakeup_state() != UWB_WAKEUP_STATE_IDLE);
}

/**
 * @brief Check if UWB sleep process is currently busy
 * 
 * @return true if sleep state machine is actively processing, false if idle
 * 
 * @note Sleep is considered busy when not in UWB_SLEEP_STATE_IDLE state.
 */
bool Api_uwb_sleep_is_busy(void)
{
	return (Drv_uwb_power_get_sleep_state() != UWB_SLEEP_STATE_IDLE);
}

/**
 * @brief Check if UWB wake-up process is complete
 * 
 * @return true if wake-up completed successfully and device is ready
 * 
 * @note Wake-up is complete when state is idle and device is ready.
 */
bool Api_uwb_is_wakeup_complete(void)
{
	return (Drv_uwb_power_get_wakeup_state() == UWB_WAKEUP_STATE_IDLE) && Drv_uwb_get_device_wakeup();
}

/**
 * @brief Check if UWB sleep process is complete
 * 
 * @return true if sleep completed successfully and device is in sleep mode
 * 
 * @note Sleep is complete when state is idle and device is not ready.
 */
bool Api_uwb_is_sleep_complete(void)
{
	return (Drv_uwb_power_get_sleep_state() == UWB_SLEEP_STATE_IDLE) && !Drv_uwb_get_device_wakeup();
}

/**
 * @brief Get Current Device Initialization State
 * 
 * @return Current initialization state from device_init_state_e enumeration
 * 
 * @note Useful for debugging initialization issues or displaying status.
 *       Check against DEVICE_INIT_STATE_COMPLETE for successful initialization.
 */
device_init_state_e Api_uwb_get_init_state(void)
{
	return Drv_uwb_init_get_state();
}

/**
 * @brief Get Current TX State
 * 
 * @return Current TX state from uwb_tx_state_e enumeration
 * 
 * @note Useful for debugging TX issues or displaying detailed status.
 */
uwb_tx_state_e Api_uwb_get_tx_state(void)
{
	return Drv_uwb_tx_get_state();
}

/**
 * @brief Get Current RX State
 * 
 * @return Current RX state from uwb_rx_state_e enumeration
 * 
 * @note Useful for debugging RX issues or displaying detailed status.
 */
uwb_rx_state_e Api_uwb_get_rx_state(void)
{
	return Drv_uwb_rx_get_state();
}

/**
 * @brief Get current UWB wake-up state
 * 
 * @return Current wake-up state from uwb_wakeup_state_e enumeration
 * 
 * @note Useful for debugging wake-up issues or monitoring wake-up progress.
 */
uwb_wakeup_state_e Api_uwb_get_wakeup_state(void)
{
	return Drv_uwb_power_get_wakeup_state();
}

/**
 * @brief Get current UWB sleep state
 * 
 * @return Current sleep state from uwb_sleep_state_e enumeration
 * 
 * @note Useful for debugging sleep issues or monitoring sleep progress.
 */
uwb_sleep_state_e Api_uwb_get_sleep_state(void)
{
	return Drv_uwb_power_get_sleep_state();
}

/**
 * @brief Reset Device Initialization
 * 
 * Resets all state machines and variables to initial state. This forces
 * a complete re-initialization sequence on the next Api_uwb_init_device() call.
 * 
 * @note Use this for error recovery or when hardware reset is required.
 *       All configuration will be lost and must be re-applied.
 */
void Api_uwb_reset_init(void)
{
	Drv_uwb_reset_all_states();
}

// ========================================================================================
// Public API Functions - Configuration Management
// ========================================================================================

/**
 * @brief Set TX Message Content
 * 
 * Updates the message content that will be transmitted in subsequent TX operations.
 * The message is copied to internal buffer and validated for length constraints.
 * 
 * @param[in] msg     Pointer to message data buffer
 * @param[in] length  Message length in bytes (excluding FCS, max 14 bytes)
 * 
 * @return true if set successfully, false if invalid parameters or TX busy
 * 
 * @note Message cannot be changed while TX is in progress.
 *       Frame Check Sequence (FCS) is automatically added by hardware.
 * 
 * @warning Message buffer must remain valid until this function returns.
 */
bool Api_uwb_set_tx_message(const uint8_t *msg, uint8_t length)
{
	if (msg == NULL || length == 0 || length > UWB_TX_MSG_MAX_LENGTH) {
		return false;
	}
	
	if (Api_uwb_tx_is_busy()) {
		return false;
	}
	
	return Drv_uwb_tx_set_message(msg, length);
}

/**
 * @brief Get Current TX Message Content
 * 
 * Copies the current TX message content to provided buffer.
 * 
 * @param[out] msg    Pointer to buffer to copy message (minimum 16 bytes)
 * @param[out] length Pointer to store message length
 * 
 * @return true if copied successfully, false if invalid parameters
 * 
 * @note Caller must provide sufficient buffer space (minimum 16 bytes).
 */
bool Api_uwb_get_tx_message(uint8_t *msg, uint8_t *length)
{
	return Drv_uwb_tx_get_message(msg, length);
}

/**
 * @brief Update Sequence Number in TX Message
 * 
 * Updates the sequence number field in the TX message (byte at index 1).
 * This is commonly used for packet tracking and duplicate detection.
 * 
 * @param[in] seq_num New sequence number (0-255)
 * 
 * @return true if updated successfully, false if message too short
 * 
 * @note Message must be at least 2 bytes long to contain sequence number.
 *       Sequence number is stored at index 1 (second byte) of message.
 */
bool Api_uwb_set_sequence_number(uint8_t seq_num)
{
	uint8_t l_msg[UWB_TX_MSG_BUFFER_SIZE];
	uint8_t l_length;
	
	if (!Drv_uwb_tx_get_message(l_msg, &l_length) || l_length < 2) {
		return false;
	}
	
	l_msg[1] = seq_num;
	return Drv_uwb_tx_set_message(l_msg, l_length);
}

/**
 * @brief Get Current Sequence Number from TX Message
 * 
 * @return Current sequence number from message byte 1, or 0xFF if message too short
 * 
 * @note Returns 0xFF if message is less than 2 bytes (invalid condition).
 */
uint8_t Api_uwb_get_sequence_number(void)
{
	uint8_t l_msg[UWB_TX_MSG_BUFFER_SIZE];
	uint8_t l_length;
	
	if (!Drv_uwb_tx_get_message(l_msg, &l_length) || l_length < 2) {
		return 0xFF;
	}
	
	return l_msg[1];
}

/**
 * @brief Get Received UWB Message
 * 
 * Retrieves the most recently received UWB message and its metadata.
 * 
 * @param[out] rx_msg Pointer to uwb_rx_message_t structure to receive data
 * 
 * @return true if valid data is available, false if no data or invalid parameters
 * 
 * @note This function copies data from internal buffer. Multiple calls will return
 *       the same data until new reception occurs.
 */
bool Api_uwb_get_rx_message(uwb_rx_message_t *rx_msg)
{
	if (rx_msg == NULL || !Drv_uwb_rx_get_data_available()) {
		return false;
	}
	
	bool l_result = Drv_uwb_rx_get_message(rx_msg);
	if (l_result) {
		Drv_uwb_rx_set_data_available(false);
	}
	
	return l_result;
}

void Api_uwb_clear_rx_message(void)
{
	Drv_uwb_rx_clear_message();
}

/**
 * @brief Check if New RX Data is Available
 * 
 * @return true if new received data is available and has not been read yet
 * 
 * @note This function provides a quick way to check if new data has been received
 *       without calling the more expensive Api_uwb_get_rx_message() function.
 */
bool Api_uwb_is_rx_data_available(void)
{
	return Drv_uwb_rx_get_data_available();
}

/**
 * @brief Set RX Timeout Value
 * 
 * Configures the timeout value for RX operations.
 * 
 * @param[in] timeout_ms RX timeout in milliseconds (must be > 0)
 * 
 * @return true if set successfully, false if invalid parameter
 * 
 * @note The timeout is used for each individual RX operation.
 */
bool Api_uwb_set_rx_timeout(uint32_t timeout_ms)
{
	if (timeout_ms == 0) {
		return false;
	}
	
	Drv_uwb_rx_set_timeout(timeout_ms);
	return true;
}

/**
 * @brief Get Current RX Timeout Value
 * 
 * @return Current RX timeout in milliseconds
 */
uint32_t Api_uwb_get_rx_timeout(void)
{
	return Drv_uwb_rx_get_timeout();
}

// ========================================================================================
// Hardware Interface Functions
// ========================================================================================

/**
 * @brief Initialize SPI Interface for UWB Communication
 * 
 * Initializes SPI peripheral for DW3000 UWB transceiver communication.
 * 
 * @param fastRate true for fast rate (32MHz), false for slow rate (4MHz)
 */
void Api_uwb_spi_init(bool fastRate)
{	
	uint32_t ret;
	
	ret = Drv_uwb_spi_init(fastRate);

	if  (ret == NRF_SUCCESS)
	{
		Api_failsafe_set_success(FAILSAFE_CODE_12);
	}
	else
	{
		Api_failsafe_set_fail(FAILSAFE_CODE_12);
	}
}

/**
 * @brief Uninitialize SPI Interface for UWB Communication
 * 
 * Deinitializes SPI peripheral and releases resources.
 */
void Api_uwb_spi_uninit(void)
{
	Drv_uwb_spi_uninit();
}

/**
 * @brief Initialize IRQ Interface for UWB Communication
 * 
 * Initializes the interrupt request (IRQ) interface for asynchronous communication
 * with the DW3000 UWB transceiver. This enables hardware interrupt-driven event
 * notification, reduced polling overhead for status monitoring, and faster
 * response to UWB events.
 * 
 * @note Must be called before enabling UWB interrupts.
 *       IRQ pin configuration is platform-specific.
 */
void Api_uwb_irq_init(void)
{
	dw_irq_init();
}

/**
 * @brief Enable UWB module interrupt
 * 
 * This function enables the GPIO interrupt for the DW3000 UWB module IRQ pin.
 * When enabled, the interrupt handler will be triggered on pin state changes
 * from the UWB module.
 */
void Api_uwb_irq_enable(void)
{
    nrf_drv_gpiote_in_event_enable(DW3000_IRQn_Pin, true);
}

/**
 * @brief Disable UWB module interrupt
 * 
 * This function disables the GPIO interrupt for the DW3000 UWB module IRQ pin.
 * When disabled, no interrupt events will be generated from the UWB module pin.
 */
void Api_uwb_irq_disable(void)
{
    nrf_drv_gpiote_in_event_disable(DW3000_IRQn_Pin);
}

// ========================================================================================
// New Public API Functions - Start/Stop Control Functions
// ========================================================================================

/**
 * @brief Start UWB device initialization process
 * 
 * @details Initiates the UWB device initialization state machine if not already
 * running. Sets the initialization state to begin the initialization sequence.
 * 
 * @return true if initialization started successfully, false if already running
 * 
 * @note The device must not be in a running initialization state for this to succeed.
 *       Use Api_uwb_init_device() repeatedly to progress through initialization.
 */
bool Api_uwb_start_init(void)
{
	if (Drv_uwb_init_get_state() != DEVICE_INIT_STATE_IDLE 
		&& Drv_uwb_init_get_state() != DEVICE_INIT_STATE_COMPLETE 
		&& Drv_uwb_init_get_state() != DEVICE_INIT_STATE_ERROR) {
		return false; // Already running
	}
	
	Drv_uwb_init_set_state(DEVICE_INIT_STATE_START);
	Drv_uwb_set_device_ready(false);
	Drv_uwb_set_device_wakeup(false);

	return true;
}

/**
 * @brief Start UWB TX operations
 * 
 * @details Enables automatic UWB transmission operations based on the configured
 * TX interval. The TX state machine will begin transmitting messages at regular intervals.
 * 
 * @return true if TX started successfully, false if device not ready
 * 
 * @note Device must be initialized and ready before starting TX operations.
 *       TX will begin immediately (timer set to 0) when started.
 */
bool Api_uwb_start_tx(bool afterSleep, bool afterRx)
{
	if (!Drv_uwb_get_device_wakeup()) {
		return false;
	}
	
	Drv_uwb_tx_set_enabled(true);
	
	Drv_uwb_tx_set_state(UWB_TX_STATE_START);
	Drv_uwb_tx_set_afterSleep(afterSleep);
	Drv_uwb_tx_set_afterRx(afterRx);
	
	return true;
}

/**
 * @brief Stop UWB TX operations
 * 
 * @details Stops automatic UWB transmission operations. The TX state machine
 * will be reset to idle state and no further transmissions will occur.
 * 
 * @note This function can be called at any time to stop TX operations.
 *       Any ongoing transmission will be allowed to complete.
 */
void Api_uwb_stop_tx(void)
{
	Drv_uwb_tx_set_enabled(false);
	Drv_uwb_tx_set_state(UWB_TX_STATE_IDLE);
	Drv_uwb_set_sequence_timer(0u);
}

/**
 * @brief Start UWB RX operations
 * 
 * @details Enables UWB reception mode. The device will start listening for
 * incoming UWB messages using the RX state machine.
 * 
 * @return true if RX started successfully, false if device not ready
 * 
 * @note Device must be initialized and ready before starting RX operations.
 *       RX will begin immediately when the state machine is processed.
 */
bool Api_uwb_start_rx(void)
{
	if (!Drv_uwb_get_device_wakeup()) {
		return false;
	}
	
	Drv_uwb_rx_set_enabled(true);
	if (Drv_uwb_rx_get_state() == UWB_RX_STATE_IDLE) {
		Drv_uwb_rx_set_state(UWB_RX_STATE_ENABLE_RX);
		Drv_uwb_set_sequence_timeout_timer(Drv_uwb_rx_get_timeout());
	}
	return true;
}

/**
 * @brief Stop UWB RX operations
 * 
 * @details Stops UWB reception mode. The RX state machine will be reset
 * to idle state and no further reception attempts will be made.
 * 
 * @note This function can be called at any time to stop RX operations.
 *       Any received data in the buffer remains available for reading.
 */
void Api_uwb_stop_rx(void)
{
	Drv_uwb_set_sequence_timeout_timer(0);
	Drv_uwb_rx_set_enabled(false);
	Drv_uwb_rx_set_state(UWB_RX_STATE_IDLE);
	Drv_uwb_rx_forceStop();
}

/**
 * @brief Start UWB wake-up process
 * 
 * @details Initiates the UWB device wake-up state machine if not already running.
 * This begins the non-blocking wake-up sequence from sleep mode.
 * 
 * @return true if wake-up started successfully, false if already running
 * 
 * @note The wake-up state machine must be in idle state for this to succeed.
 *       Use Api_uwb_wakeup() repeatedly to progress through wake-up sequence.
 */
bool Api_uwb_start_wakeup(bool configUpdate)
{
	if (Drv_uwb_power_get_wakeup_state() != UWB_WAKEUP_STATE_IDLE) {
		return false; // Already running
	}
	
	Drv_uwb_power_set_wakeup_state(UWB_WAKEUP_STATE_START); // Will be processed in next call
	Drv_uwb_set_configUpdate(configUpdate);
	
	return true;
}

/**
 * @brief Start UWB sleep process
 * 
 * @details Initiates the UWB device sleep state machine if conditions are met.
 * This begins the non-blocking sleep entry sequence.
 * 
 * @return true if sleep started successfully, false if conditions not met
 * 
 * @note Device must be ready and TX must not be busy for sleep to start.
 *       Sleep state machine must be in idle state.
 */
bool Api_uwb_start_sleep(void)
{
	if (Drv_uwb_power_get_sleep_state() != UWB_SLEEP_STATE_IDLE) {
		return false; // Already running
	}
	
	if  (Api_uwb_tx_is_busy())
	{
		return false;
	}

	if  (Drv_uwb_get_device_wakeup() == false)
	{
		Drv_uwb_power_set_sleep_state(UWB_SLEEP_STATE_IDLE);		
	}
	else
	{
		Drv_uwb_power_set_sleep_state(UWB_SLEEP_STATE_START); // Will be processed in next call
	}

	return true;
}

/**
 * @brief Set TX Configuration Parameters
 * 
 * @brief Configures the TX RF parameters for UWB transmission.
 * 
 * @param[in] config Pointer to TX configuration structure
 * 
 * @note The uwb_tx_config_t structure is cast to dwt_txconfig_t internally.
 *       These structures must have compatible memory layouts.
 */
void Api_uwb_set_txconfig(uwb_tx_config_t *config)
{
	Drv_uwb_set_txconfig((const dwt_txconfig_t *)config);
}

/**
 * @brief Read and display SYS_CFG register contents
 * 
 * @details Reads the SYS_CFG register (0x00:10) and displays
 * the current system configuration settings
 * 
 * @return Current SYS_CFG register value
 */
uint32_t Api_uwb_read_sys_cfg(void)
{
	return Drv_uwb_config_read_sys_cfg();
}

/**
 * @brief Check if SPI CRC mode is enabled
 * 
 * @return true if SPI CRC mode is enabled, false if disabled
 */
bool Api_uwb_is_spi_crc_enabled(void)
{
	return Drv_uwb_config_is_spi_crc_enabled();
}

/**
 * @brief Initialize UWB System
 * 
 * @details This function must be called at system startup before any other
 * UWB API functions. It initializes the driver layer and prepares all
 * variables for operation.
 * 
 * @note Call this function once during system initialization.
 */
void Api_uwb_init_system(void)
{
	Drv_uwb_init_system();
}

// ========================================================================================
// UWB API Functions - Main Control Function
// ========================================================================================

/**
 * @brief UWB Main Process Function
 * 
 * @details This function processes all active UWB state machines in the correct order.
 * It should be called from the main application loop to handle all UWB operations.
 * Each state machine is processed independently and non-blocking.
 * 
 * Processing Order:
 * 1. Device initialization (if not complete)
 * 2. Wake-up operations (if active)
 * 3. Sleep operations (if active)
 * 4. RX operations (if enabled and device ready)
 * 5. TX operations (if enabled and device ready)
 * 
 * @note This function is non-blocking and returns immediately after processing
 *       all active state machines. It handles the coordination between different
 *       UWB operations automatically. Call this function regularly from main loop.
 * 
 * @see Api_uwb_timer_tick() - Must be called every 1ms for timing control
 */
void Api_uwb_main(void)
{	
	// 1. Device initialization (if not complete)
	Drv_uwb_init_process_state_machine();
	
	// 2. Wake-up operations (if active)
	#ifdef  API_UWB_BLOCKING
	Drv_uwb_wakeup_machine();
	#else
	Drv_uwb_wakeup_process_state_machine();
	#endif
	
	// 3. Sleep operations (if active)
	#ifdef  API_UWB_BLOCKING
	Drv_uwb_sleep_machine();
	#else
	Drv_uwb_sleep_process_state_machine();
	#endif
	
	// 4. RX operations (if enabled and device ready)
	Drv_uwb_rx_process_state_machine();
	
	// 5. TX operations (if enabled and device ready)
	#ifdef  API_UWB_BLOCKING
	Drv_uwb_tx_machine();
	#else
	Drv_uwb_tx_process_state_machine();
	#endif
}

