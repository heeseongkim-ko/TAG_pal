/**
 * @file    def_packet.h
 * @brief   Packet structure definitions for Tag Platform
 * 
 * This header contains packet types, structures, and related definitions
 * for UWB communication.
 * 
 * @author  Tag Platform Development Team
 * @version 1.0
 * @date    2025
 */

#ifndef DEF_PACKET_H
#define DEF_PACKET_H

#include <stdint.h>
#include <stdbool.h>

// ========================================================================================
// Constants
// ========================================================================================

#define MAC_ADDR_BYTE_SIZE               6       // MAC address size in bytes

// ========================================================================================
// UWB Fcode Definitions
// ========================================================================================

#define UWB_FCODE_BLINK                 0xBB    // Standard UWB blink
#define UWB_FCODE_CONF                  0xCC    // UWB configuration message
#define UWB_FCODE_BC_POLL               0xBC    // Backchannel poll
#define UWB_FCODE_BC_DATA               0xBD    // Backchannel data
#define UWB_FCODE_BC_ACK                0xBA    // Backchannel acknowledgment
#define UWB_FCODE_BC_REPORT             0xBE    // Backchannel report
#define UWB_FCODE_USER_APP              0x64    // User application message type (backchannel)

// ========================================================================================
// Packet Prepare State Definitions
// ========================================================================================

#define PACKET_PREPARE_STATE_HEADER     0       // 1. Header decision (BC or BB)
#define PACKET_PREPARE_STATE_PAYLOAD    1       // 2. Payload decision (Battery or Info)
#define PACKET_PREPARE_STATE_TX_BUFFER  2       // 3. Move to TX buffer
#define PACKET_PREPARE_STATE_COMPLETE   3       // Complete

// ========================================================================================
// Packet Transmission Intervals
// ========================================================================================

#define BATTERY_PACKET_INTERVAL_COUNT    15      // Battery packet transmission interval (every 15th TX)
#define INFO_PACKET_INTERVAL_COUNT       15      // Info packet transmission interval (every 15th TX)
#define BACKCHANNEL_PACKET_INTERVAL_COUNT_DEFAULT 4      // Backchannel packet transmission interval (every 4th TX)
#define START_INFO_PACKET_INTERVAL_COUNT_DEFAULT 3      // Start info packet transmission interval (send 3 times at startup)


// ========================================================================================
// Packet Type Definitions
// ========================================================================================

/**
 * @brief Packet type enumeration
 */
typedef enum {
    PACKET_TYPE_DEFAULT = 0,    /**< Default packet type */
    PACKET_TYPE_BATTERY,        /**< Battery information packet */
    PACKET_TYPE_SETTING,        /**< Setting/configuration packet */
    PACKET_TYPE_TAG_INFO        /**< Tag information packet */
} packet_type_e;

/**
 * @brief Packet transmission flags structure
 * @details Contains flags indicating which packet types should be transmitted
 *          based on counter thresholds.
 */
typedef struct {
    bool bc_packet_flag;        /**< Backchannel packet transmission flag */
    bool battery_packet_flag;   /**< Battery packet transmission flag */
    bool info_packet_flag;      /**< Info packet transmission flag */
} packet_flags_t;

/**
 * @brief ADC measurement flag structure
 * @details Contains flags for ADC measurement control
 */
typedef struct {
    bool adc_measurement_flag;  /**< ADC measurement flag */
    bool led_processing_flag;   /**< LED processing flag */
} adc_flags_t;

/**
 * @brief Motion control flag structure
 * @details Contains flags for motion detection and sleep control
 */
typedef struct {
    bool motion_sleep_enabled;  /**< Motion sleep feature enabled flag */
    bool motion_sleep_flag;     /**< Motion sleep flag */
} motion_flags_t;

// ========================================================================================
// Message Type Definitions
// ========================================================================================

#define MSGTYPE_BATT               (1)   /**< Message with information about battery state */
#define MSGTYPE_TAG_INFO           (14)  /**< Message type with informations about tag HW,FW,user settings etc */

// ========================================================================================
// Message Type Byte Definitions
// ========================================================================================

#define MSGTYPE_BATT_HIGH_BYTE             0x01   // Battery message type high byte (MSGTYPE_BATT)
#define MSGTYPE_BATT_LOW_BYTE              0x00   // Battery message type low byte
#define MSGTYPE_INFO_HIGH_BYTE             0x0E   // Info message type high byte (MSGTYPE_TAG_INFO = 14)
#define MSGTYPE_INFO_LOW_BYTE              0x00   // Info message type low byte

#define BATTERY_PAYLOAD_LENGTH_DEFAULT     0x01   // Battery payload length (single battery voltage byte)
#define BATTERY_PAYLOAD_TOTAL_LENGTH       5     // Total battery payload length
#define INFO_PAYLOAD_HEADER_LENGTH         4     // Info payload header length

#define USER_APP_MESSAGE_TYPE              0x64   // User application message type

// ========================================================================================
// Packet Size Definitions
// ========================================================================================

/**
 * @brief Standard UWB blink MAC header (0xBB)
 * @details The standard UWB blink MAC header without any backchannel content.
 *          Uses packed alignment for tight memory layout.
 */
typedef struct __attribute__((packed, aligned(1))) {
    uint8_t fcode;                       /**< MAC header type identifier - 0xBB - standard blink */
    uint8_t MAC_addr[MAC_ADDR_BYTE_SIZE]; /**< MAC address of the tag */
    uint8_t seqNum;                      /**< The sequence number of the blink */
} mac_header_bb_t;

/**
 * @brief Packet payload structure (2-part - payload section)
 * @details Contains payload data for different packet types
 */
typedef struct __attribute__((packed, aligned(1))) {
    uint8_t payload_type;                /**< Payload type identifier */
    uint8_t payload_data[50];            /**< Payload data buffer */
    uint8_t payload_length;              /**< Length of payload data */
} packet_payload_t;

/**
 * @brief TDOA UWB configuration message structure
 * @details Contains UWB radio configuration parameters including channel, data rate,
 *          preamble settings, power management, and sensor configurations.
 */
typedef struct {
    uint8_t fcode;                       /**< Type of message - 0xCC for wireless config message */
    uint8_t destAddr[MAC_ADDR_BYTE_SIZE]; /**< Address of tag that should be set */
    uint8_t seqNum;                      /**< Sequence number */
    uint8_t channel;                     /**< UWB radio channel */
    uint8_t datarate;                    /**< Data rate - UWB radio */
    uint8_t preamble;                    /**< Preamble length - UWB radio */
    uint8_t prf;                         /**< Pulse repetition frequency */
    uint8_t preamCode;                   /**< UWB radio preamble code */
    uint8_t nSfd;                        /**< Start frame delimiter - standard/non-standard */
    uint8_t randomDev;                   /**< Random deviation of TDOA refresh rate */
    uint8_t acc_mode;                    /**< Mode of sleep controlled by acceleration of tag - no motion detection */
    uint8_t EblinkCont;                  /**< Setting of sensors whose data should be put into extended blink */
    uint8_t AHRS_enable;                 /**< Rotation determination setting */
    uint8_t RR[4];                       /**< TDOA refresh rate - period of UWB blink */
    uint8_t no_mot_RR[4];               /**< TDOA refresh rate during no motion */
    uint8_t tx_power[4];                /**< Transmitter power - UWB radio */
    uint8_t MCR_sens[2];                /**< Threshold of acceleration for wakeup tag from deep sleep - motion controlled ranging */
    uint8_t IMU_FS_range;               /**< Full scale range of gyroscope and accelerometer */
    uint8_t RX_period[4];               /**< Period of RX state - UWB radio */
    uint8_t RX_duration[2];             /**< Duration of RX state - UWB radio */
    uint8_t BARO_setting;               /**< Barometer precision */
    uint8_t CALIB_START;                /**< Sensors calibration start */
    uint8_t crc[2];                     /**< Placeholder for CRC - calculated in UWB radio */
} tdoa_uwb_conf_msg;

/**
 * @brief Battery blinks. It contains battery level represented as ADC output.
 */
typedef struct {
    uint8_t battVoltage;                 /**< Battery level in RAW form */
} battery_msg_t;

/**
 * @brief Tag information message structure
 * @details Contains comprehensive tag information including battery, platform, 
 *          hardware/firmware versions, UWB settings, and sensor configurations.
 */
typedef struct {
    uint8_t battVoltage;                 /**< Battery level in RAW form */
    uint8_t platform;                    /**< Type of tag */
    uint8_t hw_ver[2];                   /**< Hardware version and revision */
    uint8_t fw_ver[3];                   /**< Firmware version, subversion and revision */
    uint8_t channel;                     /**< Used UWB radio channel */
    uint8_t data_rate;                   /**< UWB radio data rate */
    uint8_t preamble;                    /**< UWB radio - preamble code length */
    uint8_t prf;                         /**< UWB radio - pulse repetition frequency */
    uint8_t preamCode;                   /**< UWB radio preamble code */
    uint8_t nSfd;                        /**< Start frame delimiter */
    uint8_t mcr;                         /**< No motion sleep mode */
    uint8_t random_dev;                  /**< Random deviation of TDOA blink refreshrate */
    uint8_t refresh_interval[4];         /**< TDOA Blink refresh rate */
    uint8_t sm_refresh_interval[4];      /**< TDOA Blink refresh rate - no motion sleep mode */
    uint8_t TXpower[4];                  /**< UWB radio transmitter power */
    uint8_t mounted_sensors;             /**< Each bit represents one sensor, if set: concrete sensor is mounted on tag */
    uint8_t active_sensors;              /**< Each bit represents one sensor, if set: concrete sensor is active */
    uint8_t mcr_threshold[2];            /**< No motion - acceleration threshold */
    uint8_t IMU_FS_range;                /**< Inertial measurement unit (accelerometer and gyroscope full scale ranges */
    uint8_t BARO_setting;                /**< Barometer precision setting */
    uint8_t AHRS_representation;         /**< Spatial rotation representation. 0 - disabled, 1 - Tait-brian angles, 2 - Quaternion */
    uint8_t bc_version;                  /**< Current version of back-channel protocol */
    uint8_t bc_period_ri;                /**< Back-channel period during normal operation [every Xth refresh interval] */
    uint8_t bc_period_rism;              /**< Back-channel period during sleep mode [every Xth refresh interval in sleep mode] */
} tag_info_msg_t;

/**
 * @brief Backchannel application data structure
 * @details Structure for receiving data from server on tag.
 *          Based on Sewio UWB Backchannel specification.
 */
typedef struct {
    uint16_t app_id;                     /**< ID of the application */
    uint8_t len;                         /**< Length of data in bytes */
    uint8_t data[100];                   /**< Application data buffer (max 100B) */
} app_data_t;

/**
 * @brief Backchannel message header structure
 * @details Standard backchannel message header with msgType 0x64 for user applications.
 *          Based on Sewio documentation: https://docs.sewio.net/docs/uwb-data-exchange-43715025.html
 */
typedef struct {
    uint8_t msgType;                     /**< Message type - 0x64 for user applications */
    uint16_t app_id;                     /**< Application ID (range: 4096-16565) */
    uint8_t length;                      /**< Length of data in bytes (max 100B) */
    uint8_t data[100];                   /**< Application data payload */
} backchannel_msg_t;

/**
 * @brief TLV (Type-Length-Value) structure for payload encapsulation
 * @details Compact binary TLV encoding for over-the-air payload.
 *          All multi-byte data types are Little-endian.
 */
typedef struct {
    uint16_t type;                       /**< Type identifier (APP_ID) */
    uint8_t length;                      /**< Length of data in bytes */
    uint8_t data[];                      /**< Variable length data (flexible array member) */
} tlv_structure_t;

#endif // DEF_PACKET_H
