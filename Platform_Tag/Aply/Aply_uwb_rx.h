/**
 * @file Aply_uwb_rx.h
 * @brief UWB RX management for Aply module
 * @author Platform Tag Team
 * @date 2025-01-20
 */

#ifndef APLY_UWB_RX_H_INCLUDED
#define APLY_UWB_RX_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Process received RX data
 * @return true if valid BC data was processed, false otherwise
 */
bool Aply_uwb_rx_process_data(void);

/**
 * @brief Check and handle backchannel RX timeout
 * @details Called from main timer tick (1ms)
 */
void Aply_uwb_rx_timer_tick(void);

/**
 * @brief Start backchannel RX with 100ms timeout
 * @return true if started successfully, false otherwise
 */
bool Aply_uwb_rx_start_backchannel(void);

#endif /* APLY_UWB_RX_H_INCLUDED */