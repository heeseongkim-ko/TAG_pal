/**
 * @file Aply_uwb_tx.h
 * @brief UWB TX module for TEIA platform
 * @author Platform Tag Team
 * @date 2025
 * @version 1.0
 */

#ifndef APLY_UWB_TX_H
#define APLY_UWB_TX_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Update pre-built packets with current configuration
 * @details Rebuilds all packet templates using current tag configuration values.
 *          Should be called when configuration changes or during initialization.
 *          Uses s_tag_config to populate packet fields.
 * @return true if successful, false if configuration not available
 * @note This function is called only when configuration changes, not on every transmission.
 */
bool Aply_uwb_tx_update_packets(void);

/**
 * @brief Prepare UWB packet for transmission based on flags
 * @details Follows original structure: check flags update header prepare payload compose API
 * @return Total packet length in bytes, 0 if error
 * @note This function checks packet flags and prepares appropriate packet type
 */
uint16_t Aply_uwb_tx_prepare_packet(void);

/**
 * @brief Build info payload using current configuration
 * @details Creates info payload data using s_tag_config values.
 *          Builds complete tag_info_msg_t structure with backchannel wrapper.
 * @return Length of built payload in bytes
 */
uint16_t Aply_uwb_tx_build_info_payload(void);

/**
 * @brief Build battery payload 
 * @details Creates battery payload data with backchannel wrapper.
 *          Includes message type, app ID, length, and battery voltage.
 * @return Length of built payload in bytes
 */
uint16_t Aply_uwb_tx_build_battery_payload(void);

/**
 * @brief Set packet flags for transmission control
 * @details Sets flags to control which packet types should be transmitted
 * @param[in] info_flag Set true to request info packet transmission
 * @param[in] battery_flag Set true to request battery packet transmission
 * @param[in] bc_flag Set true to request backchannel packet transmission
 * @return None
 */
void Aply_uwb_tx_set_packet_flags(bool info_flag, bool battery_flag, bool bc_flag);

/**
 * @brief Get current packet flags status
 * @details Returns current state of packet transmission flags
 * @param[out] flags Pointer to packet_flags_t structure to fill
 * @return None
 */
//void Aply_uwb_tx_get_packet_flags(packet_flags_t* flags);

/**
 * @brief Reset all packet flags to false
 * @details Clears all packet transmission flags
 * @return None
 */
void Aply_uwb_tx_reset_packet_flags(void);

/**
 * @brief Check if last prepared packet was BC packet
 * @details Returns whether the last packet prepared by Aply_uwb_tx_prepare_packet() was a BC packet.
 * @return true if last packet was BC, false if regular packet
 */
bool Aply_uwb_tx_was_last_packet_bc(void);

#ifdef __cplusplus
}
#endif

#endif // APLY_UWB_TX_H
