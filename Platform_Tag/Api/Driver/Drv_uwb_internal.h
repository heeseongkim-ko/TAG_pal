/**
 * @file    Drv_uwb_internal.h
 * @brief   UWB Driver Internal Shared Header
 * 
 * This header contains shared declarations, types, and variables used
 * internally across multiple UWB driver source files. It should only
 * be included by driver implementation files (.c files in Driver folder).
 * 
 * Key Features:
 * - Internal state machine getter/setter functions
 * - Shared timer access functions
 * - Message buffer management (internal)
 * - Configuration access (internal)
 * - Device status management (internal)
 * 
 * @author  Tag Platform Development Team
 * @version 1.0
 * @date    2025
 * 
 * @warning This header is for INTERNAL driver use only.
 *          Application code and API layer should NOT include this header.
 *          Only Drv_uwb*.c files should include this header.
 */

#ifndef DRV_UWB_INTERNAL_H
#define DRV_UWB_INTERNAL_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "sdk_config.h"
#include "boards.h"
#include "deca_probe_interface.h"
#include "nrf_delay.h"
#include "Port.h"
#include "Drv_uwb.h"
#include <deca_device_api.h>
#include "Func_UART_LOG.h"
#include "def_config.h"
#include "def_packet.h"

// ========================================================================================
// Shared Constants
// ========================================================================================

#define MAC_ADDR_BYTE_SIZE                          6      ///< MAC address size in bytes

// MAC address composition masks
#define PARTID_SECONDS_MASK                         (0x3FULL)
#define PARTID_MINUTES_MASK                         (0x1F80ULL)
#define PARTID_HOURS_MASK                           (0x7C000ULL)
#define PARTID_DAYS_MASK                            (0x1FF00000ULL)
#define PARTID_LOADBOARD_MASK                       (0xC0000000ULL)
#define MAC_ADDR_44b_MASK                           (0xFFFFFFFFFFULL)
#define MAC_ADDR_COMPOSITION_MARK                   (0x300000000000ULL)

// ========================================================================================
// Initialization State Machine Functions (Drv_uwb_init.c)
// ========================================================================================

void Drv_uwb_init_set_state(device_init_state_e state);
device_init_state_e Drv_uwb_init_get_state(void);

// ========================================================================================
// TX State Machine Functions (Drv_uwb_tx.c)
// ========================================================================================

uwb_tx_state_e Drv_uwb_tx_get_state(void);
void Drv_uwb_tx_set_state(uwb_tx_state_e state);
bool Drv_uwb_tx_get_enabled(void);
void Drv_uwb_tx_set_enabled(bool enabled);

void Drv_uwb_tx_set_afterSleep(bool afterSleep);
void Drv_uwb_tx_set_afterRx(bool afterRx);
bool Drv_uwb_tx_get_afterSleep(void);
bool Drv_uwb_tx_get_afterRx(void);

// ========================================================================================
// RX State Machine Functions (Drv_uwb_rx.c)
// ========================================================================================

uwb_rx_state_e Drv_uwb_rx_get_state(void);
void Drv_uwb_rx_set_state(uwb_rx_state_e state);
bool Drv_uwb_rx_get_enabled(void);
void Drv_uwb_rx_set_enabled(bool enabled);
void Drv_uwb_rx_forceStop(void);
bool Drv_uwb_rx_get_data_available(void);
void Drv_uwb_rx_set_data_available(bool available);
uint32_t Drv_uwb_rx_get_timeout(void);
void Drv_uwb_rx_set_timeout(uint32_t timeout_ms);

// ========================================================================================
// Power Management Functions (Drv_uwb_power.c)
// ========================================================================================

uwb_wakeup_state_e Drv_uwb_power_get_wakeup_state(void);
void Drv_uwb_power_set_wakeup_state(uwb_wakeup_state_e state);
uwb_sleep_state_e Drv_uwb_power_get_sleep_state(void);
void Drv_uwb_power_set_sleep_state(uwb_sleep_state_e state);
void Drv_uwb_set_configUpdate(bool configUpdate);

#endif // DRV_UWB_INTERNAL_H

