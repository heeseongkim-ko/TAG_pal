/**
 * @file Api_ble.h
 * @brief Header file for BLE API with state management
 * 
 * @version 1.0
 * @date 2025-08-06
 * @note Simple state management for unified SDK integration
 */

#ifndef API_BLE_H
#define API_BLE_H

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"

/*==================================================================================================
 *                                       TYPE DEFINITIONS
 *==================================================================================================*/

/**
 * @brief BLE state enumeration for unified SDK integration
 */
typedef enum
{
	API_BLE_STATE_UNINITIALIZED = 0,    /**< BLE stack not initialized */
	API_BLE_STATE_INITIALIZING,         /**< BLE stack initialization in progress */
	API_BLE_STATE_IDLE,                 /**< BLE stack ready, not advertising or connected */
	API_BLE_STATE_ADVERTISING,          /**< Device is advertising */
	API_BLE_STATE_CONNECTED,            /**< Device is connected to a peer */
	API_BLE_STATE_DISCONNECTING,        /**< Disconnection in progress */
	API_BLE_STATE_DFU_MODE,             /**< Device entering DFU mode */
	API_BLE_STATE_ERROR                 /**< Error state requiring recovery */
} api_ble_state_t;

/**
 * @brief BLE data transmission result
 */
typedef enum
{
	API_BLE_DATA_SUCCESS = 0,           /**< Data transmission successful */
	API_BLE_DATA_ERROR_NOT_CONNECTED,   /**< Not connected to any peer */
	API_BLE_DATA_ERROR_INVALID_PARAM,   /**< Invalid parameters provided */
	API_BLE_DATA_ERROR_TRANSMISSION,    /**< Transmission failed */
	API_BLE_DATA_ERROR_NOT_SUPPORTED    /**< Operation not supported */
} api_ble_data_result_t;

/**
 * @brief BLE data reception callback function type
 */
typedef void (*api_ble_data_received_cb_t)(uint16_t conn_handle, uint8_t *p_data, uint16_t length);

/*==================================================================================================
 *                                    FUNCTION DECLARATIONS
 *==================================================================================================*/

/**
 * @brief Initialize BLE module
 * 
 * @details This function initializes all BLE components in the correct order
 */
void Api_ble_Init(void);

/**
 * @brief Function for starting advertising
 *
 * @details This function initiates BLE advertising. If `erase_bonds` is true,
 * it first deletes all stored bonding information before starting advertising.
 * Otherwise, it directly starts fast advertising.
 *
 * @param[in] erase_bonds If true, existing bonding information will be erased before advertising starts.
 */
void Api_ble_advertising_start(bool erase_bonds);

void Api_bonding_delete(void);

/**
 * @brief Get current BLE state
 * 
 * @return Current BLE state
 */
api_ble_state_t Api_ble_get_state(void);

/**
 * @brief Check if BLE is connected
 * 
 * @return true if connected, false otherwise
 */
bool Api_ble_is_connected(void);

/**
 * @brief Check if BLE is advertising
 * 
 * @return true if advertising, false otherwise
 */
bool Api_ble_is_advertising(void);

/**
 * @brief Perform BLE health check
 * 
 * @details Checks the current state and health of the BLE module
 * 
 * @return true if BLE is healthy, false if issues detected
 */
bool Api_ble_health_check(void);

bool Api_ble_dfu_enabled(void);
/**
 * @brief Initializes the DFU buttonless asynchronous SVCI
 *
 * @details This function initializes the necessary SVCI components for the buttonless DFU
 * feature, enabling asynchronous operations related to DFU through the SoftDevice.
 *
 * @return NRF_SUCCESS on successful initialization, otherwise an error code.
 */
uint32_t Api_ble_dfu_buttonless_async_svci_init(void);

/*==================================================================================================
 *                                    DATA COMMUNICATION FUNCTIONS
 *==================================================================================================*/

/**
 * @brief Register callback function for data reception
 *
 * @details This function registers a callback that will be called when data is received
 * from a connected BLE client.
 *
 * @param[in] callback Function pointer to the data reception callback
 * @return API_BLE_DATA_SUCCESS on success, error code otherwise
 */
api_ble_data_result_t Api_ble_register_data_callback(api_ble_data_received_cb_t callback);

/**
 * @brief Send data to connected BLE client
 *
 * @details This function sends data to the currently connected BLE client.
 * The data will be sent as a notification if the client has enabled notifications.
 *
 * @param[in] p_data Pointer to the data to send
 * @param[in] length Length of the data in bytes
 * @return API_BLE_DATA_SUCCESS on success, error code otherwise
 */
api_ble_data_result_t Api_ble_send_data(uint8_t *p_data, uint16_t length);

/**
 * @brief Send data to specific BLE client
 *
 * @details This function sends data to a specific BLE client identified by connection handle.
 *
 * @param[in] conn_handle Connection handle of the target client
 * @param[in] p_data Pointer to the data to send
 * @param[in] length Length of the data in bytes
 * @return API_BLE_DATA_SUCCESS on success, error code otherwise
 */
api_ble_data_result_t Api_ble_send_data_to_client(uint16_t conn_handle, uint8_t *p_data, uint16_t length);

/**
 * @brief Send notification to connected BLE client
 *
 * @details This function sends a notification to the currently connected BLE client.
 * Notifications are unreliable but fast.
 *
 * @param[in] p_data Pointer to the data to send
 * @param[in] length Length of the data in bytes
 * @return API_BLE_DATA_SUCCESS on success, error code otherwise
 */
api_ble_data_result_t Api_ble_send_notification(uint8_t *p_data, uint16_t length);

/**
 * @brief Send indication to connected BLE client
 *
 * @details This function sends an indication to the currently connected BLE client.
 * Indications are reliable but slower than notifications.
 *
 * @param[in] p_data Pointer to the data to send
 * @param[in] length Length of the data in bytes
 * @return API_BLE_DATA_SUCCESS on success, error code otherwise
 */
api_ble_data_result_t Api_ble_send_indication(uint8_t *p_data, uint16_t length);

/**
 * @brief Get maximum data length that can be sent
 *
 * @details This function returns the maximum data length that can be sent in a single
 * transmission based on the current connection parameters.
 *
 * @return Maximum data length in bytes
 */
uint16_t Api_ble_get_max_data_length(void);

/**
 * @brief Disconnect from current BLE client
 *
 * @details This function disconnects from the currently connected BLE client.
 *
 * @return API_BLE_DATA_SUCCESS on success, error code otherwise
 */
api_ble_data_result_t Api_ble_disconnect(void);

#endif // API_BLE_H