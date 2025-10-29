
#ifndef API_BLE_H
#define API_BLE_H

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"

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

typedef enum
{
	API_BLE_DATA_SUCCESS = 0,           /**< Data transmission successful */
	API_BLE_DATA_ERROR_NOT_CONNECTED,   /**< Not connected to any peer */
	API_BLE_DATA_ERROR_INVALID_PARAM,   /**< Invalid parameters provided */
	API_BLE_DATA_ERROR_TRANSMISSION,    /**< Transmission failed */
	API_BLE_DATA_ERROR_NOT_SUPPORTED    /**< Operation not supported */
} api_ble_data_result_t;

typedef void (*api_ble_data_received_cb_t)(uint16_t conn_handle, uint8_t *p_data, uint16_t length);

void Api_ble_Init(void);

void Api_ble_advertising_start(bool erase_bonds);

void Api_bonding_delete(void);

api_ble_state_t Api_ble_get_state(void);

bool Api_ble_is_connected(void);

bool Api_ble_is_advertising(void);

bool Api_ble_health_check(void);

bool Api_ble_dfu_enabled(void);

uint32_t Api_ble_dfu_buttonless_async_svci_init(void);

api_ble_data_result_t Api_ble_register_data_callback(api_ble_data_received_cb_t callback);

api_ble_data_result_t Api_ble_send_data(uint8_t *p_data, uint16_t length);

api_ble_data_result_t Api_ble_send_data_to_client(uint16_t conn_handle, uint8_t *p_data, uint16_t length);

api_ble_data_result_t Api_ble_send_notification(uint8_t *p_data, uint16_t length);

api_ble_data_result_t Api_ble_send_indication(uint8_t *p_data, uint16_t length);

uint16_t Api_ble_get_max_data_length(void);

api_ble_data_result_t Api_ble_disconnect(void);

#endif // API_BLE_H