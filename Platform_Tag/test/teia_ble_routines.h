#ifndef TEIA_BLE_ROUTINES_H_INCLUDED
#define TEIA_BLE_ROUTINES_H_INCLUDED
#include "ble.h"
#include "nrf_ble_gatt.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "app_timer.h"
//************************************************************************ble
#define APP_BLE_OBSERVER_PRIO           3                                       
/**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1         
/**< A tag identifying the SoftDevice BLE configuration. */
#define BLE_UUID_NUS_SERVICE 0x0001 //khs_add_20250512

#define APP_ADV_INTERVAL                     40                                /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_DURATION                     18000                             /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define DEVICE_NAME "TEIA_BLE_TEST" // name of device
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             
/**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. = 20 / 1.2 5*/
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)
/**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */ 
#define SLAVE_LATENCY                   0                                           
/**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             
/**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

//************************************************************************ble
void ble_stack_init(void);
void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);
void gap_params_init(void);
void gatt_init(void);
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt);
void advertising_init(void);
void on_adv_evt(ble_adv_evt_t ble_adv_evt);
void services_init(void);
void conn_params_init(void);
void on_conn_params_evt(ble_conn_params_evt_t * p_evt);
void conn_params_error_handler(uint32_t nrf_error);
void advertising_start(void);
//************************************************************************uwb

#endif  /* TEIA_BLE_ROUTINES_H_INCLUDED */

