/**
 * @file Api_nfc.h
 * @brief High-level NFC API for Unified SDK
 * 
 * Provides unified interface for NFC operations using NTAG I2C Plus
 * 
 * @example Usage Example:
 * @code
 * // Field detection callback function
 * void nfc_field_event_handler(nfc_reader_state_t state)
 * {
 *     if (state == NFC_READER_PRESENT)
 *     {
 *         printf("NFC Reader detected!\n");
 *         // Update NFC data for reader
 *     }
 *     else
 *     {
 *         printf("NFC Reader removed!\n");
 *     }
 * }
 * 
 * int main()
 * {
 *     // Initialize NFC subsystem
 *     if (Api_nfc_init() != NFC_SUCCESS)
 *     {
 *         printf("NFC init failed\n");
 *         return -1;
 *     }
 * 
 *     // Initialize field detection with callback (for interrupt mode)
 *     if (Api_nfc_field_detection_init(nfc_field_event_handler) != NFC_SUCCESS)
 *     {
 *         printf("Field detection init failed\n");
 *         return -1;
 *     }
 * 
 *     while (1)
 *     {
 *         // For polling mode, call periodically
 *         #if !NFC_USE_INTERRUPT_DETECTION
 *         nfc_reader_state_t state = Api_nfc_poll_field_detection();
 *         // Process based on state...
 *         #endif
 * 
 *         // Other application code...
 *         nrf_delay_ms(100);
 *     }
 * 
 *     return 0;
 * }
 * @endcode
 */

#ifndef API_NFC_H
#define API_NFC_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NTAG_I2C_PLUS_ADDRESS	0x55

/**
 * @brief NFC operation result codes
 */
typedef enum {
	NFC_SUCCESS = 0,					///< Operation completed successfully
	NFC_ERROR_NOT_INITIALIZED,			///< NFC subsystem not initialized
	NFC_ERROR_INIT_FAILED,				///< Initialization failed
	NFC_ERROR_INVALID_PARAM,			///< Invalid parameter
	NFC_ERROR_COMMUNICATION,			///< I2C communication error
	NFC_ERROR_TIMEOUT,					///< Operation timeout
	NFC_ERROR_DEVICE_NOT_FOUND,			///< NFC device not found
	NFC_ERROR_MEMORY_ACCESS,			///< Memory access error
	NFC_ERROR_UNKNOWN					///< Unknown error
} nfc_result_t;

/**
 * @brief NFC operation result codes
 */
typedef enum {
	NFC_EEPROM_NONE = 0,
	NFC_EEPROM_READ_DETECTED,
	NFC_EEPROM_IF_LOCKED,
	NFC_EEPROM_IF_RELEASED,
	NFC_EEPROM_WRITE_COMPLETED,
	NFC_EEPROM_WRITE_ERROR
} nfc_eeprom_status_t;

/**
 * @brief NFC reader detection states
 */
typedef enum {
	NFC_READER_NOT_PRESENT = 0,			///< No NFC reader detected
	NFC_READER_PRESENT = 1				///< NFC reader detected
} nfc_reader_state_t;

/**
 * @brief NFC transfer direction for pass-through mode
 */
typedef enum {
	NFC_TRANSFER_I2C_TO_NFC = 0,		///< Data flows from I2C to NFC interface
	NFC_TRANSFER_NFC_TO_I2C = 1			///< Data flows from NFC to I2C interface
} nfc_transfer_direction_t;

/**
 * @brief NFC memory map constants
 */
#define NFC_EEPROM_SIZE					2048	///< Total EEPROM size in bytes
#define NFC_SRAM_SIZE					64		///< SRAM buffer size in bytes
#define NFC_SRAM_START_ADDRESS			0xF80	///< SRAM start address
#define NFC_CONFIG_START_ADDRESS		0x7A0	///< Configuration start address

/**
 * @brief User data area constants (based on TagInfo scan analysis)
 */
#define NFC_USER_DATA_START_ADDR		0x04 * 4 // 0x04 * 10u		///< User data starts from page 004 (after UID/CC)
#define NFC_USER_DATA_END_ADDR			0xE0 * 4		///< User data ends at page 1E7 (before system config)
//#define NFC_USER_DATA_SIZE				((NFC_USER_DATA_END_ADDR - NFC_USER_DATA_START_ADDR + 1) * 4)  ///< 1936 bytes available

/**
 * @brief System reserved areas (DO NOT MODIFY)
 */
#define NFC_UID_START_ADDR				0x00		///< UID area (pages 000-001)
#define NFC_CONFIG_LOCK_ADDR			0x02		///< Config/Lock area (page 002)
#define NFC_CAPABILITY_CONTAINER_ADDR	0x03		///< Capability Container (page 003)
#define NFC_SYSTEM_CONFIG_START			0x1E8		///< System config area (pages 1E8-1FF)

/**
 * @brief NDEF message constants
 */
#define NFC_NDEF_MAX_SIZE				1924	///< Maximum NDEF message size
#define NFC_NDEF_START_ADDRESS			0x04	///< NDEF message start address (same as user data)

/**
 * @brief NFC field detection event callback function type
 */
typedef void (*nfc_field_event_callback_t)(nfc_reader_state_t state);

// NFC Field Detection Configuration
#define NFC_USE_INTERRUPT_DETECTION	1		///< 0: Polling mode, 1: Interrupt mode

/**
 * @brief Initialize TWI (I2C) interface for NFC communication
 *
 * @return nfc_result_t  NFC_SUCCESS on successful initialization
 *                       NFC_ERROR_INIT_FAILED if TWI initialization fails
 */
nfc_result_t Api_nfc_twi_init(void);

nfc_result_t Api_nfc_twi_unInit(void);

/**
 * @brief Initialize NFC subsystem
 * @return NFC_SUCCESS on success, error code otherwise
 */
nfc_result_t Api_nfc_init(void);

/**
 * @brief Initialize NFC field detection
 * @param[in] callback Optional callback function for field events (can be NULL for polling mode)
 * @return NFC_SUCCESS on success, error code otherwise
 */
nfc_result_t Api_nfc_field_detection_init(void);

/**
 * @brief Check if NFC reader is present
 * @return true if reader is present, false otherwise
 */
bool Api_nfc_is_reader_present(void);

/**
 * @brief Get current NFC reader state
 * @return Current reader state (NFC_READER_PRESENT or NFC_READER_NOT_PRESENT)
 */
nfc_reader_state_t Api_nfc_get_reader_state(void);

/**
 * @brief Poll for NFC field detection (for polling mode)
 * @return Current reader state
 */
nfc_reader_state_t Api_nfc_poll_field_detection(void);

/**
 * @brief Read data from NFC tag memory (internal implementation)
 */
nfc_result_t Api_nfc_read_data(uint16_t address, uint8_t *data, uint16_t length);

/**
 * @brief Write data to NFC tag memory (internal implementation)
 */
nfc_result_t Api_nfc_write_data(uint16_t address, const uint8_t *data, uint16_t length);
/**
 * @brief Read data from SRAM buffer (internal implementation)
 */
nfc_result_t Api_nfc_read_sram(uint8_t *data, uint16_t length);

/**
 * @brief Write data to SRAM buffer (internal implementation)
 */
nfc_result_t Api_nfc_write_sram(const uint8_t *data, uint16_t length);

/**
 * @brief Monitor NFC activity and detect EEPROM changes
 * Should be called periodically or after field detection events
 * @return true if changes detected, false otherwise
 */
nfc_eeprom_status_t Api_nfc_monitor_eeprom_status(void);

/**
 * @brief Check if NFC reader has read NDEF data
 * @return true if NDEF was read since last check
 */
bool Api_nfc_was_ndef_read(void);

/**
 * @brief Get current NFC session register status
 * @param[out] ns_reg Pointer to store NS_REG value
 * @return NFC_SUCCESS on success, error code otherwise
 */
nfc_result_t Api_nfc_get_session_status(uint8_t *ns_reg);

/**
 * @brief Clear NFC error flags (e.g., EEPROM write errors)
 * @return NFC_SUCCESS on success, error code otherwise
 */
nfc_result_t Api_nfc_clear_error_flags(void);

/**
 * @brief Debug function to check NFC field detection status
 * Prints current pin state, internal state, and configuration info
 */
void Api_nfc_debug_field_detection(void);

void Api_nfc_timer_tick(void);

bool Api_nfc_power_on(void);


#ifdef __cplusplus
}
#endif

#endif // API_NFC_H
