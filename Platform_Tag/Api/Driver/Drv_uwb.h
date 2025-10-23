/**
 * @file    Drv_uwb.h
 * @brief   UWB Driver Public API Header
 * 
 * This header provides the public API interface for UWB driver operations.
 * Only functions declared in this header may be called from the API layer
 * (Api_uwb.c). Internal driver implementation details are hidden in
 * Drv_uwb_internal.h and should not be accessed by API layer code.
 * 
 * Key Features:
 * - State machine control functions (init, tx, rx, power)
 * - Configuration management
 * - Status and state query functions
 * - Device information functions
 * - Timer management
 * 
 * @author  Tag Platform Development Team
 * @version 1.0
 * @date    2025
 * 
 * @note    This is the ONLY driver header that Api_uwb.c should include.
 *          Application code should NOT include this header - use Api_uwb.h instead.
 */

#ifndef DRV_UWB_H
#define DRV_UWB_H

#include <stdint.h>
#include <stdbool.h>
#include "Api_uwb.h"
#include <deca_device_api.h>

// ========================================================================================
// Driver Constants and Configuration
// ========================================================================================

#define UWB_RESET_DELAY_MS               2      ///< Reset pin low duration (ms)
#define UWB_RESET_WAIT_MS                5      ///< Reset pin high wait time (ms)
#define UWB_TX_TIMEOUT_COUNT             10000  ///< TX timeout counter limit (iterations)
#define UWB_TX_WAIT_TIME                 1      ///< TX wait limit 
#define UWB_TX_MSG_MAX_LENGTH            100    ///< Maximum TX message length (excluding FCS)
#define UWB_TX_MSG_BUFFER_SIZE           100    ///< TX message buffer size
#define UWB_FCS_LENGTH                   2      ///< Frame Check Sequence length

#define UWB_RX_TIMEOUT_COUNT             50000  ///< RX timeout counter limit (iterations)
#define UWB_RX_MSG_BUFFER_SIZE           128    ///< RX message buffer size
#define UWB_RX_TIMEOUT_DEFAULT_MS        1000   ///< Default RX timeout (ms)

#define UWB_WAKEUP_CS_HOLD_TIME_MS       2      ///< CS hold time for wake-up (ms)
#define UWB_WAKEUP_OSC_WAIT_TIME_MS      1      ///< LP OSC start wait time (ms)
#define UWB_WAKEUP_SPIRDY_WAIT_TIME_MS   1      ///< SPIRDY wait time (ms)
#define UWB_WAKEUP_SPIRDY_TIMEOUT_COUNT  5000   ///< SPIRDY timeout counter limit (iterations)
#define UWB_WAKEUP_IDLERC_TIMEOUT_COUNT  2000   ///< IDLE_RC timeout counter limit (iterations)

#define UWB_READ_DELAY_MS                1

/**
 * @brief Calculate total frame length including FCS (Frame Check Sequence)
 */
#define FRAME_LENGTH(msg_len) ((msg_len) + UWB_FCS_LENGTH)

// ========================================================================================
// System Initialization and Management Functions
// ========================================================================================

void Drv_uwb_init_system(void);
void Drv_uwb_init_variables(void);
void Drv_uwb_timer_tick(void);
void Drv_uwb_reset_all_states(void);

// ========================================================================================
// State Machine Process Functions
// ========================================================================================

void Drv_uwb_init_process_state_machine(void);
void Drv_uwb_tx_process_state_machine(void);
void Drv_uwb_rx_process_state_machine(void);
API_UWB_STATUS_e Drv_uwb_wakeup_machine(void);
void Drv_uwb_wakeup_process_state_machine(void);
void Drv_uwb_sleep_process_state_machine(void);

// ========================================================================================
// Blocking Functions (Power Optimized)
// ========================================================================================

API_UWB_STATUS_e Drv_uwb_wakeup_machine(void);
API_UWB_STATUS_e Drv_uwb_sleep_machine(void);
API_UWB_STATUS_e Drv_uwb_tx_machine(void);

// ========================================================================================
// Configuration and Information Functions
// ========================================================================================

uint32_t Drv_uwb_config_read_sys_cfg(void);
bool Drv_uwb_config_is_spi_crc_enabled(void);
bool Drv_uwb_config_read_device_ids(uint32_t* lotid, uint32_t* partid);
bool Drv_uwb_config_compose_mac_address(uint8_t* MAC_addr);

// ========================================================================================
// State Getter Functions
// ========================================================================================

bool Drv_uwb_get_device_ready(void);
bool Drv_uwb_get_device_wakeup(void);
uint32_t Drv_uwb_get_sequence_timer(void);
uint32_t Drv_uwb_get_sequence_timeout_timer(void);

// ========================================================================================
// State Setter Functions
// ========================================================================================

void Drv_uwb_set_device_ready(bool ready);
void Drv_uwb_set_device_wakeup(bool ready);
void Drv_uwb_set_sequence_timer(uint32_t timer_ms);
void Drv_uwb_set_sequence_timeout_timer(uint32_t timer_ms);

// ========================================================================================
// Message Management Functions
// ========================================================================================

bool Drv_uwb_tx_get_message(uint8_t *msg, uint8_t *length);
bool Drv_uwb_tx_set_message(const uint8_t *msg, uint8_t length);
bool Drv_uwb_rx_get_message(uwb_rx_message_t *rx_msg);
void Drv_uwb_rx_clear_message(void);

// ========================================================================================
// Configuration Management Functions
// ========================================================================================

bool Drv_uwb_get_config(dwt_config_t *config);
bool Drv_uwb_set_config(const dwt_config_t *config);
bool Drv_uwb_get_txconfig(dwt_txconfig_t *txconfig);
bool Drv_uwb_set_txconfig(const dwt_txconfig_t *txconfig);

#endif // DRV_UWB_H

