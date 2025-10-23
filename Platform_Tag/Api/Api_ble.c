/**
 * @file API_ble.c
 * @brief This file contains the implementation of BLE (Bluetooth Low Energy) functionalities
 * for the application. It includes initialization of the BLE stack, advertising,
 * peer management, GATT, and connection parameters, along with handlers for BLE and
 * Peer Manager events.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"

#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_dfu.h"
#include "ble_gattc.h" 

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "fds.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"
#include "nrf_power.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "Func_UART_LOG.h"
#include "nrf_bootloader_info.h"

// DFU-related #includes
#include "nrf_power.h"
#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"
#include "ble_dfu.h"
#include "nrf_bootloader_info.h"

#include "Api_ble.h"

/*==================================================================================================
 *                                        CONSTANTS
 *==================================================================================================*/

#define DEVICE_NAME                     "TEIA Tag"                         /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "TEIA"                                  /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_DURATION                18000                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

/*==================================================================================================
 *                                    MODULE INSTANCES
 *==================================================================================================*/

NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */

/*==================================================================================================
 *                                    STATE MANAGEMENT VARIABLES
 *==================================================================================================*/

static api_ble_state_t ble_state_ui32 = API_BLE_STATE_UNINITIALIZED;              /**< Current BLE state */

/**
 * @brief Static variable to track current advertising state
 */
static bool ble_is_advertising_b = false;                                          /**< Advertising state flag */

/**
 * @brief Connection handle for current BLE connection
 */
static uint16_t ble_conn_handle_ui16 = BLE_CONN_HANDLE_INVALID;                       /**< Handle of the current connection. */

/**
 * @brief UUIDs to include in advertising data
 */
static ble_uuid_t ble_adv_uuids_p[] = {{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}};

/*==================================================================================================
 *                                    DATA COMMUNICATION VARIABLES
 *==================================================================================================*/

/**
 * @brief Callback function pointer for data reception events
 */
static api_ble_data_received_cb_t ble_data_received_callback_p = NULL;              /**< Callback for data reception */

/**
 * @brief GATT characteristic handles for BLE communication
 */
static uint16_t ble_notification_handle_ui16 = BLE_GATT_HANDLE_INVALID;               /**< Handle for notification characteristic */
static uint16_t ble_indication_handle_ui16 = BLE_GATT_HANDLE_INVALID;                 /**< Handle for indication characteristic */
static uint16_t ble_write_handle_ui16 = BLE_GATT_HANDLE_INVALID;                      /**< Handle for write characteristic */

/*==================================================================================================
 *                                    PRIVATE FUNCTION DECLARATIONS
 *==================================================================================================*/

/**
 * @brief BLE event handler function
 *
 * @details This function handles various BLE events from the SoftDevice,
 * including connection/disconnection events, GAP events, GATT events,
 * and updates the application state accordingly.
 *
 * @param[in] p_ble_evt Pointer to the BLE event structure
 * @param[in] p_context Context parameter (unused)
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);

/**
 * @brief Peer Manager event handler function
 *
 * @details This function handles events from the Peer Manager module,
 * which manages bonding, pairing, and security procedures.
 *
 * @param[in] p_evt Pointer to the Peer Manager event
 */
static void peer_manager_evt_handler(pm_evt_t const * p_evt);

static void sleep_mode_enter(void);

/**
 * @brief Advertising event handler function
 *
 * @details This function handles advertising events and updates the
 * application state based on advertising status changes.
 *
 * @param[in] ble_adv_evt The advertising event type
 */
static void advertising_evt_handler(ble_adv_evt_t ble_adv_evt);

/**
 * @brief Error handler for Queued Write module
 *
 * @details This function handles errors from the Queued Write module
 * by calling the application error handler.
 *
 * @param[in] nrf_error Error code from the Queued Write module
 */
static void qwr_error_handler(uint32_t nrf_error);

static void ble_disconnect(uint16_t conn_handle, void * p_context);

static void advertising_config_get(ble_adv_modes_config_t * p_config);

static void dfu_evt_handler(ble_dfu_buttonless_evt_type_t event);

/**
 * @brief Connection parameters event handler
 *
 * @details This function handles events from the Connection Parameters module,
 * including negotiation failures and parameter update results.
 *
 * @param[in] p_evt Pointer to the connection parameters event
 */
static void conn_params_evt_handler(ble_conn_params_evt_t * p_evt);

/**
 * @brief Error handler for Connection Parameters module
 *
 * @details This function handles errors from the Connection Parameters module
 * by calling the application error handler.
 *
 * @param[in] nrf_error Error code from the Connection Parameters module
 */
static void conn_params_error_handler(uint32_t nrf_error);

/**
 * @brief Delete all stored bonding information
 */
static void bonding_delete(void);

/**
 * @brief Initialize the BLE stack and SoftDevice
 */
static void ble_stack_init(void);

/**
 * @brief Initialize the Peer Manager for bonding and security
 */
static void peer_manager_init(void);

/**
 * @brief Initialize GAP parameters
 */
static void gap_params_init(void);

/**
 * @brief Initialize GATT module
 */
static void gatt_init(void);

/**
 * @brief Initialize advertising functionality
 */
static void advertising_init(void);

/**
 * @brief Initialize BLE services
 */
static void services_init(void);

/**
 * @brief Initialize connection parameters module
 */
static void conn_params_init(void);

static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event);
static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void * p_context);

/*==================================================================================================
 *                                    PRIVATE FUNCTION IMPLEMENTATIONS
 *==================================================================================================*/

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
	uint32_t l_err_code = NRF_SUCCESS;

	switch (p_ble_evt->header.evt_id)
	{
		case BLE_GAP_EVT_DISCONNECTED:
			ble_conn_handle_ui16 = BLE_CONN_HANDLE_INVALID;
			ble_is_advertising_b = false;
			ble_state_ui32 = API_BLE_STATE_IDLE;
			break;

		case BLE_GAP_EVT_CONNECTED:
			l_err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
			APP_ERROR_CHECK(l_err_code);
			ble_conn_handle_ui16 = p_ble_evt->evt.gap_evt.conn_handle;
			ble_is_advertising_b = false;
			ble_state_ui32 = API_BLE_STATE_CONNECTED;
			l_err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, ble_conn_handle_ui16);
			APP_ERROR_CHECK(l_err_code);
			break;

		case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
		{
			LOG_API_BLE("PHY update request.\r\n");
			ble_gap_phys_t const l_phys =
			{
				.rx_phys = BLE_GAP_PHY_AUTO,
				.tx_phys = BLE_GAP_PHY_AUTO,
			};
			l_err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &l_phys);
			APP_ERROR_CHECK(l_err_code);
			break;
		}

		case BLE_GATTC_EVT_TIMEOUT:
			LOG_API_BLE("GATT Client Timeout.\r\n");
			ble_state_ui32 = API_BLE_STATE_DISCONNECTING;
			l_err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
										   BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			APP_ERROR_CHECK(l_err_code);
			break;

		case BLE_GATTS_EVT_TIMEOUT:
			LOG_API_BLE("GATT Server Timeout.\r\n");
			ble_state_ui32 = API_BLE_STATE_DISCONNECTING;
			l_err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
										   BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			APP_ERROR_CHECK(l_err_code);
			break;

		case BLE_GATTS_EVT_WRITE:
		{
			ble_gatts_evt_write_t const * l_p_write = &p_ble_evt->evt.gatts_evt.params.write;
			
			if (l_p_write->handle == ble_write_handle_ui16 && ble_data_received_callback_p != NULL)
			{
				ble_data_received_callback_p(p_ble_evt->evt.gatts_evt.conn_handle, 
								(uint8_t*)l_p_write->data, l_p_write->len);
			}
			break;
		}

		case BLE_GATTS_EVT_HVC:
		{
			ble_gatts_evt_hvc_t const * l_p_hvc = &p_ble_evt->evt.gatts_evt.params.hvc;
			
			if (l_p_hvc->handle == ble_indication_handle_ui16)
			{
				LOG_API_BLE("Indication confirmation received\r\n");
			}
			break;
		}

		default:
			break;
	}
}

static void peer_manager_evt_handler(pm_evt_t const * p_evt)
{
	pm_handler_on_pm_evt(p_evt);
	pm_handler_disconnect_on_sec_failure(p_evt);
	pm_handler_flash_clean(p_evt);
}

/**
 * @brief Function for putting the chip into sleep mode.
 *
 * @details This function prepares the device for low-power sleep mode, including setting
 * BSP indication, preparing wakeup buttons, and disabling the SoftDevice to allow
 * writing to the GPREGRET2 register for CRC check skipping on next boot.
 *
 * @note This function will not return as the chip enters sleep.
 */
static void sleep_mode_enter(void)
{
	uint32_t l_err_code = bsp_indication_set(BSP_INDICATE_IDLE);

	APP_ERROR_CHECK(l_err_code);

	l_err_code = bsp_btn_ble_sleep_mode_prepare();
	APP_ERROR_CHECK(l_err_code);

	l_err_code = nrf_sdh_disable_request();
	APP_ERROR_CHECK(l_err_code);
}

static void advertising_evt_handler(ble_adv_evt_t ble_adv_evt)
{
	switch (ble_adv_evt)
	{
		case BLE_ADV_EVT_FAST:
			ble_is_advertising_b = true;
			ble_state_ui32 = API_BLE_STATE_ADVERTISING;
			break;

		case BLE_ADV_EVT_IDLE:
			ble_is_advertising_b = false;
			ble_state_ui32 = API_BLE_STATE_IDLE;
			break;

		default:
			break;
	}
}

static void qwr_error_handler(uint32_t nrf_error)
{
	APP_ERROR_HANDLER(nrf_error);
}

/**
 * @brief Disconnects a given BLE connection.
 *
 * @details This function attempts to disconnect a BLE connection identified by its handle.
 * It logs a warning if the disconnection fails.
 *
 * @param[in] conn_handle The connection handle to disconnect.
 * @param[in] p_context   Unused context pointer.
 */
static void ble_disconnect(uint16_t conn_handle, void * p_context)
{
	UNUSED_PARAMETER(p_context);

	ret_code_t l_err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
	if (l_err_code != NRF_SUCCESS)
	{
		LOG_API_BLE("Failed to disconnect connection. Connection handle: %d Error: %d\r\n", conn_handle, l_err_code);
	}
	else
	{
		LOG_API_BLE("Disconnected connection handle %d\r\n", conn_handle);
	}
}

/**
 * @brief Configures the advertising modes.
 *
 * @details This function sets up the advertising configuration, specifically enabling
 * fast advertising and defining its interval and duration.
 *
 * @param[out] p_config Pointer to the advertising modes configuration structure to populate.
 */
static void advertising_config_get(ble_adv_modes_config_t * p_config)
{
	memset(p_config, 0, sizeof(ble_adv_modes_config_t));

	p_config->ble_adv_fast_enabled  = true;
	p_config->ble_adv_fast_interval = APP_ADV_INTERVAL;
	p_config->ble_adv_fast_timeout  = APP_ADV_DURATION;
}

/**
 * @brief Function for handling DFU (Device Firmware Update) buttonless events.
 *
 * @details This function is a callback for events related to buttonless DFU procedures.
 * It handles preparation for entering bootloader mode, actual bootloader entry,
 * and error conditions during these processes.
 *
 * @param[in] event The type of DFU buttonless event.
 */

bool ble_dfu_enabled = false;

bool Api_ble_dfu_enabled(void)
{
	return ble_dfu_enabled;
}

static void dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
#if 0
	switch (event)
	{
		case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
		{
			LOG_API_BLE("Device is preparing to enter bootloader mode.\r\n");
			ble_state_ui32 = API_BLE_STATE_DFU_MODE;

			ble_adv_modes_config_t l_config;
			advertising_config_get(&l_config);
			l_config.ble_adv_on_disconnect_disabled = true;
			ble_advertising_modes_config_set(&m_advertising, &l_config);

			uint32_t l_conn_count = ble_conn_state_for_each_connected(ble_disconnect, NULL);
			LOG_API_BLE("Disconnected %d links.\r\n", l_conn_count);
			break;
		}

		case BLE_DFU_EVT_BOOTLOADER_ENTER:
			LOG_API_BLE("Device will enter bootloader mode.\r\n");
			break;

		case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
			LOG_API_BLE("Request to enter bootloader mode failed asynchroneously.\r\n");
			ble_state_ui32 = API_BLE_STATE_ERROR;
			break;

		case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
			LOG_API_BLE("Request to send a response to client failed.\r\n");
			ble_state_ui32 = API_BLE_STATE_ERROR;
			APP_ERROR_CHECK(false);
			break;

		default:
			LOG_API_BLE("Unknown event from ble_dfu_buttonless.\r\n");
			break;
	}
#else
    ret_code_t    err_code;
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
            NRF_LOG_INFO("Device is preparing to enter bootloader mode\r\n");
			ble_dfu_enabled = true;
            break;
 
        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            NRF_LOG_INFO("Device will enter bootloader mode\r\n");
			ble_dfu_enabled = true;
            break;
 
        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            NRF_LOG_ERROR("Device failed to enter bootloader mode\r\n");
            break;
        default:
            NRF_LOG_INFO("Unknown event from ble_dfu.\r\n");
            break;
    }
 #endif
}

static void conn_params_evt_handler(ble_conn_params_evt_t * p_evt)
{
	if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
	{
		ble_state_ui32 = API_BLE_STATE_DISCONNECTING;
		uint32_t l_err_code = sd_ble_gap_disconnect(ble_conn_handle_ui16, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
		APP_ERROR_CHECK(l_err_code);
	}
}

static void conn_params_error_handler(uint32_t nrf_error)
{
	APP_ERROR_HANDLER(nrf_error);
}

/**
 * @brief Clears bonding information from persistent storage.
 *
 * @details This function initiates the process of deleting all peer bonding information
 * stored in flash memory via the Peer Manager.
 */
static void bonding_delete(void)
{
	ret_code_t l_err_code;

	LOG_API_BLE("Erase bonds!\r\n");

	l_err_code = pm_peers_delete();
	APP_ERROR_CHECK(l_err_code);
}

/**
 * @brief Initializes the BLE stack.
 *
 * @details This function enables the SoftDevice and configures the BLE stack with default
 * settings, including determining the application's RAM start address.
 */
static void ble_stack_init(void)
{
	ret_code_t l_err_code;

	ble_state_ui32 = API_BLE_STATE_INITIALIZING;

	l_err_code = nrf_sdh_enable_request();
	APP_ERROR_CHECK(l_err_code);

	uint32_t l_ram_start = 0;
	l_err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &l_ram_start);
	APP_ERROR_CHECK(l_err_code);

	l_err_code = nrf_sdh_ble_enable(&l_ram_start);
	APP_ERROR_CHECK(l_err_code);

	NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**
 * @brief Initializes the Peer Manager module.
 *
 * @details This function initializes the Peer Manager, which handles bonding,
 * pairing, and peer data storage. It sets up the security parameters
 * for BLE connections.
 */
static void peer_manager_init(void)
{
	ble_gap_sec_params_t l_sec_param;
	ret_code_t           l_err_code;

	l_err_code = pm_init();
	APP_ERROR_CHECK(l_err_code);

	memset(&l_sec_param, 0, sizeof(ble_gap_sec_params_t));

	l_sec_param.bond           = SEC_PARAM_BOND;
	l_sec_param.mitm           = SEC_PARAM_MITM;
	l_sec_param.lesc           = SEC_PARAM_LESC;
	l_sec_param.keypress       = SEC_PARAM_KEYPRESS;
	l_sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
	l_sec_param.oob            = SEC_PARAM_OOB;
	l_sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
	l_sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
	l_sec_param.kdist_own.enc  = 1;
	l_sec_param.kdist_own.id   = 1;
	l_sec_param.kdist_peer.enc = 1;
	l_sec_param.kdist_peer.id  = 1;

	l_err_code = pm_sec_params_set(&l_sec_param);
	APP_ERROR_CHECK(l_err_code);

	l_err_code = pm_register(peer_manager_evt_handler);
	APP_ERROR_CHECK(l_err_code);
}

/**
 * @brief Initializes GAP (Generic Access Profile) parameters.
 *
 * @details This function sets up the device name, appearance (if applicable),
 * and preferred connection parameters for BLE GAP operations.
 */
static void gap_params_init(void)
{
	uint32_t                l_err_code;
	ble_gap_conn_params_t   l_gap_conn_params;
	ble_gap_conn_sec_mode_t l_sec_mode;

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&l_sec_mode);

	l_err_code = sd_ble_gap_device_name_set(&l_sec_mode,
										  (const uint8_t *)DEVICE_NAME,
										  strlen(DEVICE_NAME));
	APP_ERROR_CHECK(l_err_code);

	memset(&l_gap_conn_params, 0, sizeof(l_gap_conn_params));

	l_gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
	l_gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
	l_gap_conn_params.slave_latency     = SLAVE_LATENCY;
	l_gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

	l_err_code = sd_ble_gap_ppcp_set(&l_gap_conn_params);
	APP_ERROR_CHECK(l_err_code);
}

/**
 * @brief Function for initializing the GATT (Generic Attribute Profile) module.
 *
 * @details The GATT module handles ATT_MTU (Attribute Protocol Maximum Transmission Unit)
 * and Data Length update procedures automatically. This function initializes it.
 */
static void gatt_init(void)
{
	ret_code_t l_err_code = nrf_ble_gatt_init(&m_gatt, NULL);
	APP_ERROR_CHECK(l_err_code);
}

/**
 * @brief Function for initializing the Advertising functionality.
 *
 * @details This function configures and initializes the BLE advertising module,
 * including advertising data, appearance, flags, and event handler.
 */
static void advertising_init(void)
{
	uint32_t               l_err_code;
	ble_advertising_init_t l_init;

	memset(&l_init, 0, sizeof(l_init));

	l_init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
	l_init.advdata.include_appearance      = true;
	l_init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
	l_init.advdata.uuids_complete.uuid_cnt = sizeof(ble_adv_uuids_p) / sizeof(ble_adv_uuids_p[0]);
	l_init.advdata.uuids_complete.p_uuids  = ble_adv_uuids_p;

	advertising_config_get(&l_init.config);

	l_init.evt_handler = advertising_evt_handler;

	l_err_code = ble_advertising_init(&m_advertising, &l_init);
	APP_ERROR_CHECK(l_err_code);

	ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**
 * @brief Function for initializing services that will be used by the application.
 *
 * @details This function initializes the Queued Write Module (QWR) and the
 * buttonless DFU service. Placeholder for additional service
 * initializations is also provided.
 */
static void services_init(void)
{
	uint32_t                  l_err_code;
	nrf_ble_qwr_init_t        l_qwr_init  = {0};
	ble_dfu_buttonless_init_t l_dfus_init = {0};

	l_qwr_init.error_handler = qwr_error_handler;

	l_err_code = nrf_ble_qwr_init(&m_qwr, &l_qwr_init);
	APP_ERROR_CHECK(l_err_code);

	l_dfus_init.evt_handler = dfu_evt_handler;

	l_err_code = ble_dfu_buttonless_init(&l_dfus_init);
	APP_ERROR_CHECK(l_err_code);
}

/**
 * @brief Initializes the Connection Parameters module.
 *
 * @details This function sets up the connection parameters negotiation process,
 * defining delays and counts for updates, and registers a handler for
 * connection parameter events and errors.
 */
static void conn_params_init(void)
{
	uint32_t               l_err_code;
	ble_conn_params_init_t l_cp_init;

	memset(&l_cp_init, 0, sizeof(l_cp_init));

	l_cp_init.p_conn_params                  = NULL;
	l_cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
	l_cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
	l_cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
	l_cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
	l_cp_init.disconnect_on_fail             = false;
	l_cp_init.evt_handler                    = conn_params_evt_handler;
	l_cp_init.error_handler                  = conn_params_error_handler;

	l_err_code = ble_conn_params_init(&l_cp_init);
	APP_ERROR_CHECK(l_err_code);
}

/*==================================================================================================
 *                                    PUBLIC API FUNCTIONS
 *==================================================================================================*/

void Api_ble_Init(void)
{
	ret_code_t err_code;

	// Initialize the async SVCI interface to bootloader before any interrupts are enabled.
	err_code = ble_dfu_buttonless_async_svci_init();
	APP_ERROR_CHECK(err_code);
		
	ble_state_ui32 = API_BLE_STATE_INITIALIZING;
	
	ble_stack_init();
	gap_params_init();
	gatt_init();
	advertising_init();
	services_init();
	peer_manager_init();
	conn_params_init();
	
	ble_state_ui32 = API_BLE_STATE_IDLE;
	
	LOG_API_BLE("BLE initialization completed successfully\r\n");
}

void Api_ble_advertising_start(bool erase_bonds)
{
	if (ble_state_ui32 == API_BLE_STATE_UNINITIALIZED)
	{
		LOG_API_BLE("BLE not initialized, cannot start advertising\r\n");
		return;
	}
	
	if (erase_bonds == true)
	{
		bonding_delete();
	}
	else
	{
		uint32_t l_err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
		APP_ERROR_CHECK(l_err_code);

		LOG_API_BLE("advertising is started\r\n");
	}
}

void Api_bonding_delete(void)
{
	bonding_delete();
}

api_ble_state_t Api_ble_get_state(void)
{
	return ble_state_ui32;
}

bool Api_ble_is_connected(void)
{
	return (ble_state_ui32 == API_BLE_STATE_CONNECTED);
}

bool Api_ble_is_advertising(void)
{
	return (ble_state_ui32 == API_BLE_STATE_ADVERTISING);
}

bool Api_ble_health_check(void)
{
	if (ble_state_ui32 == API_BLE_STATE_ERROR)
	{
		LOG_API_BLE("BLE health check failed: Error state detected\r\n");
		return false;
	}
	
	if (ble_state_ui32 == API_BLE_STATE_CONNECTED)
	{
		if (ble_conn_handle_ui16 == BLE_CONN_HANDLE_INVALID)
		{
			LOG_API_BLE("BLE health check failed: Connected state but invalid handle\r\n");
			return false;
		}
	}
	else
	{
		if (ble_conn_handle_ui16 != BLE_CONN_HANDLE_INVALID)
		{
			LOG_API_BLE("BLE health check failed: Not connected but valid handle exists\r\n");
			return false;
		}
	}
	
	if (ble_state_ui32 == API_BLE_STATE_ADVERTISING && !ble_is_advertising_b)
	{
		LOG_API_BLE("BLE health check failed: Advertising state inconsistency\r\n");
		return false;
	}
	
	return true;
}

uint32_t Api_ble_dfu_buttonless_async_svci_init(void)
{
	return(ble_dfu_buttonless_async_svci_init());
}

/*==================================================================================================
 *                                    DATA COMMUNICATION FUNCTIONS
 *==================================================================================================*/

api_ble_data_result_t Api_ble_register_data_callback(api_ble_data_received_cb_t callback)
{
	if (callback == NULL)
	{
		return API_BLE_DATA_ERROR_INVALID_PARAM;
	}
	
	ble_data_received_callback_p = callback;
	return API_BLE_DATA_SUCCESS;
}

api_ble_data_result_t Api_ble_send_data(uint8_t *p_data, uint16_t length)
{
	if (p_data == NULL || length == 0)
	{
		return API_BLE_DATA_ERROR_INVALID_PARAM;
	}
	
	if (ble_state_ui32 != API_BLE_STATE_CONNECTED || ble_conn_handle_ui16 == BLE_CONN_HANDLE_INVALID)
	{
		return API_BLE_DATA_ERROR_NOT_CONNECTED;
	}
	
	return Api_ble_send_notification(p_data, length);
}

api_ble_data_result_t Api_ble_send_data_to_client(uint16_t conn_handle, uint8_t *p_data, uint16_t length)
{
	if (p_data == NULL || length == 0)
	{
		return API_BLE_DATA_ERROR_INVALID_PARAM;
	}
	
	if (ble_state_ui32 != API_BLE_STATE_CONNECTED)
	{
		return API_BLE_DATA_ERROR_NOT_CONNECTED;
	}
	
	if (ble_notification_handle_ui16 == BLE_GATT_HANDLE_INVALID)
	{
		return API_BLE_DATA_ERROR_NOT_SUPPORTED;
	}
	
	ble_gatts_hvx_params_t l_hvx_params;
	memset(&l_hvx_params, 0, sizeof(l_hvx_params));
	
	l_hvx_params.handle = ble_notification_handle_ui16;
	l_hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
	l_hvx_params.offset = 0;
	l_hvx_params.p_len  = &length;
	l_hvx_params.p_data = p_data;
	
	uint32_t l_err_code = sd_ble_gatts_hvx(conn_handle, &l_hvx_params);
	if (l_err_code != NRF_SUCCESS)
	{
		LOG_API_BLE("Failed to send notification: %d\r\n", l_err_code);
		return API_BLE_DATA_ERROR_TRANSMISSION;
	}
	
	return API_BLE_DATA_SUCCESS;
}

api_ble_data_result_t Api_ble_send_notification(uint8_t *p_data, uint16_t length)
{
	if (p_data == NULL || length == 0)
	{
		return API_BLE_DATA_ERROR_INVALID_PARAM;
	}
	
	if (ble_state_ui32 != API_BLE_STATE_CONNECTED || ble_conn_handle_ui16 == BLE_CONN_HANDLE_INVALID)
	{
		return API_BLE_DATA_ERROR_NOT_CONNECTED;
	}
	
	if (ble_notification_handle_ui16 == BLE_GATT_HANDLE_INVALID)
	{
		return API_BLE_DATA_ERROR_NOT_SUPPORTED;
	}
	
	ble_gatts_hvx_params_t l_hvx_params;
	memset(&l_hvx_params, 0, sizeof(l_hvx_params));
	
	l_hvx_params.handle = ble_notification_handle_ui16;
	l_hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
	l_hvx_params.offset = 0;
	l_hvx_params.p_len  = &length;
	l_hvx_params.p_data = p_data;
	
	uint32_t l_err_code = sd_ble_gatts_hvx(ble_conn_handle_ui16, &l_hvx_params);
	if (l_err_code != NRF_SUCCESS)
	{
		LOG_API_BLE("Failed to send notification: %d\r\n", l_err_code);
		return API_BLE_DATA_ERROR_TRANSMISSION;
	}
	
	return API_BLE_DATA_SUCCESS;
}

api_ble_data_result_t Api_ble_send_indication(uint8_t *p_data, uint16_t length)
{
	if (p_data == NULL || length == 0)
	{
		return API_BLE_DATA_ERROR_INVALID_PARAM;
	}
	
	if (ble_state_ui32 != API_BLE_STATE_CONNECTED || ble_conn_handle_ui16 == BLE_CONN_HANDLE_INVALID)
	{
		return API_BLE_DATA_ERROR_NOT_CONNECTED;
	}
	
	if (ble_indication_handle_ui16 == BLE_GATT_HANDLE_INVALID)
	{
		return API_BLE_DATA_ERROR_NOT_SUPPORTED;
	}
	
	ble_gatts_hvx_params_t l_hvx_params;
	memset(&l_hvx_params, 0, sizeof(l_hvx_params));
	
	l_hvx_params.handle = ble_indication_handle_ui16;
	l_hvx_params.type   = BLE_GATT_HVX_INDICATION;
	l_hvx_params.offset = 0;
	l_hvx_params.p_len  = &length;
	l_hvx_params.p_data = p_data;
	
	uint32_t l_err_code = sd_ble_gatts_hvx(ble_conn_handle_ui16, &l_hvx_params);
	if (l_err_code != NRF_SUCCESS)
	{
		LOG_API_BLE("Failed to send indication: %d\r\n", l_err_code);
		return API_BLE_DATA_ERROR_TRANSMISSION;
	}
	
	return API_BLE_DATA_SUCCESS;
}

uint16_t Api_ble_get_max_data_length(void)
{
	if (ble_state_ui32 != API_BLE_STATE_CONNECTED || ble_conn_handle_ui16 == BLE_CONN_HANDLE_INVALID)
	{
		return 0;
	}
	
	uint16_t l_mtu_size = nrf_ble_gatt_eff_mtu_get(&m_gatt, ble_conn_handle_ui16);
	if (l_mtu_size == 0)
	{
		l_mtu_size = BLE_GATT_ATT_MTU_DEFAULT;
	}
	
	return (l_mtu_size > 3) ? (l_mtu_size - 3) : 0;
}

api_ble_data_result_t Api_ble_disconnect(void)
{
	if (ble_state_ui32 != API_BLE_STATE_CONNECTED || ble_conn_handle_ui16 == BLE_CONN_HANDLE_INVALID)
	{
		return API_BLE_DATA_ERROR_NOT_CONNECTED;
	}
	
	uint32_t l_err_code = sd_ble_gap_disconnect(ble_conn_handle_ui16, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
	if (l_err_code != NRF_SUCCESS)
	{
		LOG_API_BLE("Failed to disconnect: %d\r\n", l_err_code);
		return API_BLE_DATA_ERROR_TRANSMISSION;
	}
	
	ble_state_ui32 = API_BLE_STATE_DISCONNECTING;
	return API_BLE_DATA_SUCCESS;
}

/**@brief Handler for shutdown preparation.
 *
 * @details During shutdown procedures, this function will be called at a 1 second interval
 *          untill the function returns true. When the function returns true, it means that the
 *          app is ready to reset to DFU mode.
 *
 * @param[in]   event   Power manager event.
 *
 * @retval  True if shutdown is allowed by this power manager handler, otherwise false.
 */
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            NRF_LOG_INFO("Power management wants to reset to DFU mode.");
            // YOUR_JOB: Get ready to reset into DFU mode
            //
            // If you aren't finished with any ongoing tasks, return "false" to
            // signal to the system that reset is impossible at this stage.
            //
            // Here is an example using a variable to delay resetting the device.
            //
            // if (!m_ready_for_reset)
            // {
            //      return false;
            // }
            // else
            //{
            //
            //    // Device ready to enter
            //    uint32_t err_code;
            //    err_code = sd_softdevice_disable();
            //    APP_ERROR_CHECK(err_code);
            //    err_code = app_timer_stop_all();
            //    APP_ERROR_CHECK(err_code);
            //}
            break;
        default:
            // YOUR_JOB: Implement any of the other events available from the power management module:
            //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
            //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
            //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
            return true;
    }
    NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
    return true;
}

/**@brief Register application shutdown handler with priority 0.
 */
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);
static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void * p_context)
{
    if (state == NRF_SDH_EVT_STATE_DISABLED)
    {
        // Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.
        nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);
        //Go to system off.
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
}
/* nrf_sdh state observer. */
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) =
{
    .handler = buttonless_dfu_sdh_state_observer,
};
