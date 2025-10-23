/**
 * @file    Api_uwb.h
 * @brief   Unified SDK API Header for Tag Platform Integration
 * 
 * This header provides a unified API interface for the Tag Platform project,
 * integrating MCU (nRF52840), UWB (DW3000), and NFC (NT3H2211W0FHK) functionalities
 * into a single, cohesive SDK. The API is designed to be non-blocking and
 * state-machine driven for optimal real-time performance.
 * 
 * Key Features:
 * - Non-blocking UWB operations with state machine architecture
 * - BLE integration for OTA functionality
 * - NFC data exchange capabilities
 * - Runtime configurable parameters
 * - Comprehensive error handling and status reporting
 * 
 * Target Hardware:
 * - MCU: nRF52840 (ARM Cortex-M4F with BLE)
 * - UWB: DW3000 series (DW3210 for ranging and positioning)
 * - NFC: NT3H2211W0FHK (I2C interface with NDEF support)
 * 
 * @author  Tag Platform Development Team
 * @version 1.0
 * @date    2025
 * 
 * @note    All comments in code are written in English as per coding standards.
 *          Variables follow naming convention: g_ (global), s_ (static), l_ (local).
 */

#ifndef API_UWB_H
#define API_UWB_H

#include <stdint.h>
#include <stdbool.h>

#define API_UWB_BLOCKING

// ========================================================================================
// Constants and Definitions
// ========================================================================================

/** @brief MAC address size in bytes for UWB device identification */
#define MAC_ADDR_BYTE_SIZE    6

// ========================================================================================
// Type Definitions and Enumerations
// ========================================================================================

/**
 * @brief UWB API Status enumeration for high-level status reporting
 * 
 * This enumeration provides simplified status information for application-level
 * monitoring and control. For detailed state information, use the specific
 * state getter functions.
 */
typedef enum
{
	API_UWB_STATUS_IDLE,
	API_UWB_STATUS_ONGOING,
	API_UWB_STATUS_SUCCESS,
	API_UWB_STATUS_FAIL
} API_UWB_STATUS_e;

/**
 * @brief Device initialization state machine states
 * 
 * Detailed state enumeration for UWB device initialization process.
 * Used for debugging, monitoring, and precise control of initialization sequence.
 * Each state represents a specific step in the hardware initialization process.
 */
typedef enum {
	DEVICE_INIT_STATE_IDLE = 0,         ///< Initial state, ready to start initialization
	DEVICE_INIT_STATE_START,            ///< Initial state, start
	DEVICE_INIT_STATE_RESET_PIN_LOW,    ///< Pulling reset pin low to reset device
	DEVICE_INIT_STATE_RESET_DELAY,      ///< Waiting for reset pulse duration (2ms)
	DEVICE_INIT_STATE_RESET_PIN_HIGH,   ///< Releasing reset pin (high-Z)
	DEVICE_INIT_STATE_RESET_WAIT,       ///< Waiting for device to come out of reset (5ms)
	DEVICE_INIT_STATE_PROBE_DEVICE,     ///< Probing device and reading device ID
	DEVICE_INIT_STATE_READ_DEVICEID,
	DEVICE_INIT_STATE_CHECK_IDLE,       ///< Checking if device is in IDLE_RC state
	DEVICE_INIT_STATE_INITIALIZE,       ///< Calling dwt_initialise() function
	DEVICE_INIT_STATE_CONFIG_SLEEP,     ///< Configuring sleep wake up setting //khs
	DEVICE_INIT_STATE_CONFIGURE,        ///< Configuring device with communication parameters
	DEVICE_INIT_STATE_CONFIG_TX_RF,     ///< Configuring TX RF parameters (power, spectrum)
	DEVICE_INIT_STATE_COMPLETE,         ///< Initialization completed successfully
	DEVICE_INIT_STATE_ERROR             ///< Error occurred during initialization
} device_init_state_e;

/**
 * @brief UWB TX State Machine states for transmission operations
 * 
 * Detailed state enumeration for UWB transmission process. Each state represents
 * a specific step in the transmission sequence, allowing for precise monitoring
 * and control of the TX operation.
 */
typedef enum {
	UWB_TX_STATE_IDLE = 0,              ///< TX idle, waiting for timer or trigger
	UWB_TX_STATE_START,                 ///< TX start,
	UWB_TX_STATE_PREPARE_DATA,          ///< Writing TX data to device buffer
	UWB_TX_STATE_WRITE_FRAME_CTRL,      ///< Writing frame control information
	UWB_TX_STATE_WAIT_TX_COMPLETE,      ///< Waiting for TX completion (non-blocking)
	UWB_TX_STATE_CLEAR_STATUS,          ///< Clearing TX completion status flags
	UWB_TX_STATE_COMPLETE               ///< TX completed, updating sequence and timers
} uwb_tx_state_e;

/**
 * @brief UWB RX State Machine states for reception operations
 */
typedef enum {
	UWB_RX_STATE_IDLE = 0,              ///< RX idle, waiting for enable command
	UWB_RX_STATE_ENABLE_RX,             ///< Enabling RX mode on device
	UWB_RX_STATE_WAIT_AND_PROCESS,      ///< Wait for RX, read data, process and clear status
	UWB_RX_STATE_COMPLETE               ///< RX completed, data available for application
} uwb_rx_state_e;

/**
 * @brief UWB Wake-up State Machine states for non-blocking wake-up operations
 */
typedef enum {
	UWB_WAKEUP_STATE_IDLE = 0,          ///< Wake-up idle, process none
	UWB_WAKEUP_STATE_START,				/// < Wake-up start, ready to start wake-up
	UWB_WAKEUP_STATE_SPI_SLOW,          ///< Setting SPI to slow rate
	UWB_WAKEUP_STATE_CS_LOW,            ///< Pulling CS pin low
	UWB_WAKEUP_STATE_CS_HOLD,           ///< Holding CS low for required time
	UWB_WAKEUP_STATE_CS_HIGH,           ///< Releasing CS pin high
	UWB_WAKEUP_STATE_OSC_WAIT,          ///< Waiting for LP OSC to start
	UWB_WAKEUP_STATE_SPIRDY_WAIT,       ///< Waiting for SPIRDY signal
	UWB_WAKEUP_STATE_CHECK_DEVID,       ///< Checking device ID
	UWB_WAKEUP_STATE_CHECK_IDLE,        ///< Checking IDLE_RC state
	UWB_WAKEUP_STATE_RESTORE_CONFIG,    ///< Restoring device configuration
	UWB_WAKEUP_STATE_SPI_FAST,          ///< Setting SPI to fast rate
	UWB_WAKEUP_STATE_COMPLETE,          ///< Wake-up completed successfully
	UWB_WAKEUP_STATE_ERROR              ///< Error occurred during wake-up
} uwb_wakeup_state_e;

/**
 * @brief UWB Sleep State Machine states for non-blocking sleep operations
 */
typedef enum {
	UWB_SLEEP_STATE_IDLE = 0,           ///< Sleep idle, ready to start sleep
	UWB_SLEEP_STATE_START,	            ///< Sleep start, start to sleep
	UWB_SLEEP_STATE_FORCE_OFF,          ///< Forcing radio off
	UWB_SLEEP_STATE_CLEAR_STATUS,       ///< Clearing all status registers
	UWB_SLEEP_STATE_SAVE_CONTEXT,       ///< Saving device context
	UWB_SLEEP_STATE_ENTER_SLEEP,        ///< Entering sleep mode
	UWB_SLEEP_STATE_SPI_SLOW,           ///< Setting SPI to slow rate for power saving
	UWB_SLEEP_STATE_COMPLETE,           ///< Sleep entered successfully
	UWB_SLEEP_STATE_ERROR               ///< Error occurred during sleep entry
} uwb_sleep_state_e;

/**
 * @brief RX message structure for received data
 * 
 * Contains received message data along with metadata for processing
 */
typedef struct {
	uint8_t data[128];                   ///< Received message data buffer
	uint8_t length;                     ///< Actual received data length
	uint8_t sequence_number;            ///< Sequence number from received message
	uint32_t timestamp;                 ///< Reception timestamp (ms)
	bool valid;                         ///< Data validity flag
} uwb_rx_message_t;

typedef struct
{
	uint8_t PGdly;
		// TX POWER
		// 31:24     TX_CP_PWR
		// 23:16     TX_SHR_PWR
		// 15:8      TX_PHR_PWR
		// 7:0       TX_DATA_PWR
	uint32_t power;
	uint16_t PGcount;
} uwb_tx_config_t;

// ========================================================================================
// UWB API Functions - Core State Machine Operations
// ========================================================================================

/**
 * @brief UWB Device Initialize - Blocking version
 * 
 * This function implements a blocking version of UWB device initialization.
 * It performs the complete initialization sequence and returns only when
 * initialization is complete (success or failure). This is useful for system
 * startup where blocking initialization is acceptable and ensures the device
 * is ready before proceeding.
 * 
 * Initialization Sequence:
 * 1. Hardware reset sequence (pin control with precise timing)
 * 2. Device probe and ID verification
 * 3. Driver initialization with error checking
 * 4. Communication parameter configuration
 * 5. TX RF parameter setup
 * 
 * @note This function blocks until initialization is complete. Use this for
 *       system startup or when you need to ensure the device is ready before
 *       proceeding with other operations. The function handles all timing
 *       internally and calls Api_uwb_timer_tick() as needed.
 * 
 * @param[in] timeout_ms Maximum time to wait for initialization (0 = no timeout)
 * 
 * @return true if initialization completed successfully, false if failed or timeout
 * 
 * @warning This function will block the calling thread until initialization
 *          completes or timeout occurs. Do not call from interrupt context.
 *          Use Api_uwb_init_device() for non-blocking operation.
 * 
 * @see Api_uwb_init_device() - Non-blocking version
 * @see Api_uwb_is_device_ready() - Check device readiness after blocking init
 * @see Api_uwb_get_init_state() - Get initialization result details
 */
bool Api_uwb_init_device_blocking(uint32_t timeout_ms);

/**
 * @brief Timer Tick Handler - Decrement UWB timers
 * 
 * This function manages all UWB-related timing by decrementing internal timers.
 * It must be called exactly every 1ms to maintain accurate timing for both
 * initialization delays and TX interval control.
 * 
 * Managed Timers:
 * - TX interval timer (controls automatic transmission timing)
 * - Initialization delay timer (controls reset and startup delays)
 * 
 * @note This function is critical for proper operation of the UWB state machines.
 *       Failure to call this regularly will cause timing issues and malfunction.
 * 
 * @warning Must be called exactly every 1ms for accurate timing. Irregular calling
 *          intervals will affect transmission timing and initialization delays.
 * 
 * Integration Examples:
 * - Call from 1ms timer interrupt
 * - Call from main loop with 1ms tick verification
 * - Call from RTOS task with 1ms period
 */
void Api_uwb_timer_tick(void);

// ========================================================================================
// UWB API Functions - Status and State Monitoring
// ========================================================================================

/**
 * @brief Check if UWB device is ready for operation
 * 
 * @return true if device is initialized and ready for TX operations, false otherwise
 * 
 * @note Device is considered ready when the initialization state machine has completed
 *       successfully (DEVICE_INIT_STATE_COMPLETE) and all configuration has been applied.
 *       TX operations should only be attempted when this function returns true.
 * 
 * @see Api_uwb_init_device() - Function that performs initialization
 * @see Api_uwb_get_init_state() - Get detailed initialization state
 */
bool Api_uwb_is_device_ready(void);

bool Api_uwb_is_device_wakeup(void);

/**
 * @brief Check if UWB TX is currently busy
 * 
 * @return true if TX state machine is actively processing, false if idle
 * 
 * @note TX is considered busy when not in UWB_TX_STATE_IDLE state.
 *       Use this to avoid configuration changes during transmission.
 */
bool Api_uwb_tx_is_busy(void);
/**
 * @brief Check if UWB RX is currently busy
 * 
 * @return true if RX state machine is actively processing, false if idle
 * 
 * @note RX is considered busy when not in UWB_RX_STATE_IDLE state.
 *       Use this to avoid configuration changes during reception.
 */
bool Api_uwb_rx_is_busy(void);
/**
 * @brief Check if UWB wake-up process is currently busy
 * 
 * @return true if wake-up state machine is actively processing, false if idle
 * 
 * @note Wake-up is considered busy when not in UWB_WAKEUP_STATE_IDLE state.
 */
bool Api_uwb_wakeup_is_busy(void);
/**
 * @brief Check if UWB sleep process is currently busy
 * 
 * @return true if sleep state machine is actively processing, false if idle
 * 
 * @note Sleep is considered busy when not in UWB_SLEEP_STATE_IDLE state.
 */
bool Api_uwb_sleep_is_busy(void);
/**
 * @brief Check if UWB wake-up process is complete
 * 
 * @return true if wake-up completed successfully and device is ready
 * 
 * @note Wake-up is complete when state is idle and device is ready.
 */
bool Api_uwb_is_wakeup_complete(void);
/**
 * @brief Check if UWB sleep process is complete
 * 
 * @return true if sleep completed successfully and device is in sleep mode
 * 
 * @note Sleep is complete when state is idle and device is not ready.
 */
bool Api_uwb_is_sleep_complete(void);

/**
 * @brief Get current device initialization state
 * 
 * @return Current initialization state from device_init_state_e enumeration
 * 
 * @note Useful for debugging initialization issues or displaying detailed status
 *       in user interfaces. Check against DEVICE_INIT_STATE_COMPLETE for
 *       successful initialization, or DEVICE_INIT_STATE_ERROR for failures.
 * 
 * @see device_init_state_e - Complete list of initialization states
 * @see Api_uwb_is_device_ready() - Simplified ready status check
 */
device_init_state_e Api_uwb_get_init_state(void);
/**
 * @brief Get current UWB TX state
 * 
 * @return Current TX state from uwb_tx_state_e enumeration
 * 
 * @note Useful for debugging TX issues or displaying detailed status.
 */
uwb_tx_state_e Api_uwb_get_tx_state(void);

/**
 * @brief Get current UWB RX state
 * 
 * @return Current RX state from uwb_rx_state_e enumeration
 * 
 * @note Useful for debugging RX issues or displaying detailed status.
 */
uwb_rx_state_e Api_uwb_get_rx_state(void);
/**
 * @brief Get current UWB wake-up state
 * 
 * @return Current wake-up state from uwb_wakeup_state_e enumeration
 * 
 * @note Useful for debugging wake-up issues or monitoring wake-up progress.
 */
uwb_wakeup_state_e Api_uwb_get_wakeup_state(void);
/**
 * @brief Get current UWB sleep state
 * 
 * @return Current sleep state from uwb_sleep_state_e enumeration
 * 
 * @note Useful for debugging sleep issues or monitoring sleep progress.
 */
uwb_sleep_state_e Api_uwb_get_sleep_state(void);

/**
 * @brief Reset UWB device initialization state
 * 
 * Resets all state machines and variables to their initial state, forcing a
 * complete re-initialization sequence on the next Api_uwb_init_device() call.
 * This function is used for error recovery or when hardware reset is required.
 * 
 * Actions Performed:
 * - Reset initialization state to IDLE
 * - Clear device ready flag
 * - Reset TX state to IDLE
 * - Reset RX state to IDLE
 * - Clear all timers and counters
 * - Restore default TX interval
 * - Clear RX message buffer
 * 
 * @note All configuration (TX message, interval, etc.) will be lost and must be
 *       re-applied after re-initialization. Use this for error recovery scenarios
 *       or when hardware issues are suspected.
 * 
 * @warning This does not perform hardware reset. Hardware reset requires calling
 *          Api_uwb_init_device() after this function.
 */
void Api_uwb_reset_init(void);

// ========================================================================================
// UWB Configuration Functions - Runtime Configurable Parameters
// ========================================================================================

/**
 * @brief Set TX message content
 * 
 * Updates the message content that will be transmitted in subsequent TX operations.
 * The message is copied to an internal buffer and validated for length constraints
 * according to UWB frame limitations.
 * 
 * @param[in] msg     Pointer to message data buffer (must not be NULL)
 * @param[in] length  Message length in bytes (excluding FCS, range: 1-14 bytes)
 * 
 * @return true if set successfully, false if invalid parameters or TX is busy
 * 
 * Message Format:
 * - Byte 0: Application-specific header or message type
 * - Byte 1: Sequence number (if length >= 2, auto-incremented by TX state machine)
 * - Bytes 2-13: Application data payload
 * - FCS: Frame Check Sequence (automatically added by hardware)
 * 
 * @note Message cannot be changed while TX is in progress (Api_uwb_tx_is_busy() == true).
 *       The Frame Check Sequence (FCS) is automatically calculated and added by the
 *       DW3000 hardware, so it should not be included in the message length.
 * 
 * @warning Message buffer must remain valid until this function returns. The function
 *          copies the data, so the original buffer can be modified after return.
 * 
 * @see Api_uwb_get_tx_message() - Retrieve current message content
 * @see Api_uwb_set_sequence_number() - Update sequence number only
 * @see Api_uwb_tx_is_busy() - Check if message can be changed
 */
bool Api_uwb_set_tx_message(const uint8_t *msg, uint8_t length);

/**
 * @brief Get current TX message content
 * 
 * Copies the current TX message content to the provided buffer for inspection
 * or backup purposes.
 * 
 * @param[out] msg    Pointer to buffer to receive message copy (minimum 16 bytes)
 * @param[out] length Pointer to variable to receive message length
 * 
 * @return true if copied successfully, false if invalid parameters (NULL pointers)
 * 
 * @note Caller must provide sufficient buffer space (minimum 16 bytes) to avoid
 *       buffer overflow. The actual message may be shorter than 16 bytes.
 * 
 * @warning Both msg and length parameters must be valid pointers. No bounds
 *          checking is performed on the output buffer size.
 * 
 * @see Api_uwb_set_tx_message() - Set new message content
 */
bool Api_uwb_get_tx_message(uint8_t *msg, uint8_t *length);

/**
 * @brief Update sequence number in TX message
 * 
 * Updates only the sequence number field in the TX message (byte at index 1).
 * This is commonly used for packet tracking, duplicate detection, and protocol
 * implementation without changing the entire message content.
 * 
 * @param[in] seq_num New sequence number value (0-255)
 * 
 * @return true if updated successfully, false if message is too short (< 2 bytes)
 * 
 * @note Message must be at least 2 bytes long to contain a sequence number field.
 *       The sequence number is stored at index 1 (second byte) of the message.
 *       The TX state machine automatically increments this field after each
 *       successful transmission when TEIA_API_EXAMPLE is defined.
 * 
 * @see Api_uwb_get_sequence_number() - Read current sequence number
 * @see Api_uwb_set_tx_message() - Set complete message including sequence number
 */
bool Api_uwb_set_sequence_number(uint8_t seq_num);

/**
 * @brief Get current sequence number from TX message
 * 
 * @return Current sequence number from message byte 1, or 0xFF if message too short
 * 
 * @note Returns 0xFF if the message is less than 2 bytes long, indicating an
 *       invalid condition where no sequence number field exists.
 * 
 * @see Api_uwb_set_sequence_number() - Update sequence number
 */
uint8_t Api_uwb_get_sequence_number(void);

/**
 * @brief Get received UWB message
 * 
 * Retrieves the most recently received UWB message and its metadata. This function
 * copies the received data to the provided buffer structure.
 * 
 * @param[out] rx_msg Pointer to uwb_rx_message_t structure to receive data
 * 
 * @return true if valid data is available, false if no data or invalid parameters
 * 
 * @note This function copies data from internal buffer. Multiple calls will return
 *       the same data until new reception occurs. Check the valid flag in rx_msg
 *       to confirm data integrity.
 * 
 * @warning The rx_msg parameter must be a valid pointer. No bounds checking is
 *          performed on the structure size.
 * 
 * @see uwb_rx_message_t - Structure definition for received data
 * @see Api_uwb_enable_rx() - Start reception to receive data
 * @see Api_uwb_rx_is_busy() - Check if new data might be available
 */
bool Api_uwb_get_rx_message(uwb_rx_message_t *rx_msg);

void Api_uwb_clear_rx_message(void);

/**
 * @brief Check if new RX data is available
 * 
 * @return true if new received data is available and has not been read yet
 * 
 * @note This function provides a quick way to check if new data has been received
 *       without calling the more expensive Api_uwb_get_rx_message() function.
 *       The flag is cleared when data is read via Api_uwb_get_rx_message().
 * 
 * @see Api_uwb_get_rx_message() - Retrieve the available data
 * @see Api_uwb_enable_rx() - Start reception to receive new data
 */
bool Api_uwb_is_rx_data_available(void);

/**
 * @brief Set RX timeout value
 * 
 * Configures the timeout value for RX operations. This determines how long the
 * RX state machine will wait for incoming data before timing out.
 * 
 * @param[in] timeout_ms RX timeout in milliseconds (must be > 0)
 * 
 * @return true if set successfully, false if invalid parameter (timeout_ms == 0)
 * 
 * @note The timeout is used for each individual RX operation. After timeout,
 *       the RX state machine returns to idle state and can be restarted.
 *       Longer timeouts increase the chance of receiving data but also
 *       increase power consumption and response time.
 * 
 * @warning Setting very short timeouts may cause RX to timeout before
 *          receiving valid data, especially in noisy environments.
 * 
 * @see Api_uwb_get_rx_timeout() - Get current timeout setting
 */
bool Api_uwb_set_rx_timeout(uint32_t timeout_ms);

/**
 * @brief Get current RX timeout value
 * 
 * @return Current RX timeout in milliseconds
 * 
 * @note Returns the configured timeout value, not the remaining time for
 *       current RX operation.
 * 
 * @see Api_uwb_set_rx_timeout() - Set new timeout value
 */
uint32_t Api_uwb_get_rx_timeout(void);

// ========================================================================================
// Hardware Interface Initialization Functions
// ========================================================================================

/**
 * @brief Initialize SPI interface for UWB communication
 * 
 * Initializes the SPI peripheral and configures it for high-speed communication
 * with the DW3000 UWB transceiver. This function sets up the necessary hardware
 * interfaces and optimizes SPI parameters for reliable UWB communication.
 * 
 * Configuration includes:
 * - nRF52840 SPI controller initialization
 * - Fast SPI rate configuration for optimal performance
 * - Pin configuration for SPI signals (MOSI, MISO, SCK, CS)
 * - SPI mode and timing parameters
 * 
 * @note Must be called before any UWB operations or Api_uwb_init_device().
 *       SPI speed is automatically optimized for DW3000 communication requirements.
 *       The function configures platform-specific SPI settings.
 * 
 * @warning Calling this function multiple times is safe but unnecessary.
 *          SPI configuration persists until system reset.
 * 
 * @see Api_uwb_init_device() - Requires SPI to be initialized first
 */
void Api_uwb_spi_init(bool fastRate);

/**
 * @brief Uninitialize SPI Interface for UWB Communication
 * 
 * Deinitializes SPI peripheral and releases resources.
 */
void Api_uwb_spi_uninit(void);

/**
 * @brief Initialize IRQ interface for UWB communication
 * 
 * Initializes the interrupt request (IRQ) interface for asynchronous communication
 * with the DW3000 UWB transceiver. This enables hardware interrupt-driven event
 * notification, reducing polling overhead and improving system responsiveness.
 * 
 * Configuration includes:
 * - IRQ pin configuration (input with appropriate pull settings)
 * - Interrupt controller setup
 * - Event detection configuration (edge/level triggering)
 * - Interrupt priority settings
 * 
 * Features enabled:
 * - Hardware interrupt-driven event notification
 * - Reduced CPU usage compared to polling
 * - Faster response to UWB events (TX completion, RX events, errors)
 * - Asynchronous status monitoring
 * 
 * @note Must be called before enabling UWB interrupts in the DW3000 configuration.
 *       IRQ pin configuration is platform-specific and depends on hardware design.
 *       The current implementation may use polling instead of interrupts.
 * 
 * @warning Calling this function multiple times is safe but unnecessary.
 *          IRQ configuration persists until system reset.
 * 
 * @see Api_uwb_init_device() - May require IRQ to be initialized first
 */
void Api_uwb_irq_init(void);

/**
 * @brief Enable UWB module interrupt
 * 
 * This function enables the GPIO interrupt for the DW3000 UWB module IRQ pin.
 * When enabled, the interrupt handler will be triggered on pin state changes
 * from the UWB module.
 * 
 * @note Must be called after Api_uwb_irq_init() to enable interrupt events.
 *       The IRQ pin will generate interrupts based on UWB module events.
 */
void Api_uwb_irq_enable(void);

/**
 * @brief Disable UWB module interrupt
 * 
 * This function disables the GPIO interrupt for the DW3000 UWB module IRQ pin.
 * When disabled, no interrupt events will be generated from the UWB module pin.
 * 
 * @note Use this to temporarily disable interrupts without reinitializing.
 */
void Api_uwb_irq_disable(void);

// ========================================================================================
// UWB API Functions - Start/Stop Control Functions
// ========================================================================================

/**
 * @brief Start UWB Device Initialization Process
 * 
 * Initiates the UWB device initialization state machine. This function
 * sets the initialization state to begin the initialization sequence.
 * 
 * @return true if initialization started successfully, false if already running
 */
bool Api_uwb_start_init(void);

/**
 * @brief Start UWB TX Operations
 * 
 * Enables automatic UWB transmission operations based on the configured
 * TX interval. The TX state machine will begin transmitting messages
 * at regular intervals.
 * 
 * @return true if TX started successfully, false if device not ready
 */
bool Api_uwb_start_tx(bool afterSleep, bool afterRx);

/**
 * @brief Stop UWB TX Operations
 * 
 * Stops automatic UWB transmission operations. The TX state machine
 * will be reset to idle state.
 */
void Api_uwb_stop_tx(void);

/**
 * @brief Start UWB RX Operations
 * 
 * Enables UWB reception mode. The device will start listening for
 * incoming UWB messages.
 * 
 * @return true if RX started successfully, false if device not ready
 */
bool Api_uwb_start_rx(void);

/**
 * @brief Stop UWB RX Operations
 * 
 * Disables UWB reception mode. The RX state machine will be reset
 * to idle state.
 */
void Api_uwb_stop_rx(void);

/**
 * @brief Start UWB Wake-up Process
 * 
 * Initiates the UWB device wake-up state machine. This function
 * begins the non-blocking wake-up sequence.
 * 
 * @return true if wake-up started successfully, false if already running
 */
bool Api_uwb_start_wakeup(bool configUpdate);

/**
 * @brief Start UWB Sleep Process
 * 
 * Initiates the UWB device sleep state machine. This function
 * begins the non-blocking sleep entry sequence.
 * 
 * @return true if sleep started successfully, false if conditions not met
 */
bool Api_uwb_start_sleep(void);

// ========================================================================================
// UWB API Functions - Main Control Function
// ========================================================================================

/**
 * @brief UWB Main Process Function
 * 
 * This function processes all active UWB state machines in the correct order.
 * It should be called from the main application loop to handle all UWB operations.
 * 
 * Processing Order:
 * 1. Device initialization (if not complete)
 * 2. Wake-up operations (if active)
 * 3. Sleep operations (if active)  
 * 4. RX operations (if enabled)
 * 5. TX operations (if enabled)
 * 
 * @note This function is non-blocking and returns immediately after processing
 *       all active state machines. It handles the coordination between different
 *       UWB operations automatically.
 */
void Api_uwb_main(void);
// ========================================================================================
// MAC Address Management Functions
// ========================================================================================

/**
 * @brief Read LOT_ID and PART_ID from DW3000 device
 * @param[out] lotid - LOT_ID from DW3000
 * @param[out] partid - PART_ID from DW3000
 * @return true if successful, false if failed
 * 
 * @note This function reads the device identification registers from DW3000.
 *       LOT_ID and PART_ID are used to generate unique MAC addresses.
 */
bool Api_uwb_read_IDs(uint32_t* lotid, uint32_t* partid);

/**
 * @brief Compose the MAC address of the tag using LOT_ID and PART_ID of the DW3000
 * @param[out] MAC_addr - 6-byte MAC address array
 * @return true if successful, false if failed
 * 
 * @note This function generates a unique 6-byte MAC address by combining
 *       LOT_ID and compressed PART_ID from the DW3000 device.
 *       The MAC address is stored directly in the provided 6-byte array.
 */
bool Api_uwb_compose_MAC_address(uint8_t* MAC_addr);

// UWB Configuration Getter Functions
uint8_t Api_uwb_get_channel(void);
uint8_t Api_uwb_get_data_rate(void);
uint8_t Api_uwb_get_preamble_length(void);
uint8_t Api_uwb_get_prf(void);
uint8_t Api_uwb_get_preamble_code(void);
uint8_t Api_uwb_get_sfd(void);
void Api_uwb_get_tx_power(uint8_t* tx_power);

void Api_uwb_set_txconfig(uwb_tx_config_t *config);


// ========================================================================================
// System Initialization Functions
// ========================================================================================

/**
 * @brief Initialize UWB System
 * 
 * @details This function must be called at system startup before any other
 * UWB API functions. It initializes the driver layer and prepares all
 * variables for operation.
 * 
 * @note Call this function once during system initialization, before calling
 *       any other UWB API functions.
 */
void Api_uwb_init_system(void);

/**
 * @brief Read and display SYS_CFG register contents
 * @return Current SYS_CFG register value
 * 
 * @note This function reads the SYS_CFG register (0x00:10) and displays
 *       the current system configuration settings.
 */
uint32_t Api_uwb_read_sys_cfg(void);

/**
 * @brief Check if SPI CRC mode is enabled
 * @return true if SPI CRC mode is enabled, false if disabled
 */
bool Api_uwb_is_spi_crc_enabled(void);

#endif // API_UWB_H
