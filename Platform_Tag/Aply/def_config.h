/**
 * @file    def_config.h
 * @brief   Configuration definitions for Tag Platform
 * 
 * This header contains all configuration values including timing, intervals,
 * random ranges, and LED control settings.
 * 
 * @author  Tag Platform Development Team
 * @version 1.0
 * @date    2025
 */

#ifndef DEF_CONFIG_H
#define DEF_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

// Forward declarations for random TX control functions
void Func_TEIA_Set_Random_TX_Settings(bool enable, int32_t min_offset, int32_t max_offset);
void Func_TEIA_Get_Random_TX_Settings(bool* enable, int32_t* min_offset, int32_t* max_offset);

// ========================================================================================
// Timing Configuration
// ========================================================================================

// TX interval (base timing unit)
#define TX_INTERVAL_MS_DEFAULT                   (1000UL)    // TX interval: 1000 ms
#define TX_INTERVAL_MOTION_SLEEP_MS_DEFAULT      (30000UL)   // TX interval during motion sleep: 30 seconds

// Measurement and packet intervals
#define ADC_MEASUREMENT_INTERVAL_MS_DEFAULT      (5000UL)    // ADC measurement interval: 5 seconds
#define LED_BLINK_INTERVAL_MS_DEFAULT            (10000UL)    // LED blink interval: 10 second
#define ACC_TIME_TO_WAIT_INMOTION_DEFAULT        (30000UL) // 30 seconds

// ========================================================================================
// Motion Sleep Configuration
// ========================================================================================

// Motion sleep feature can be enabled/disabled at runtime
// Default: enabled (1), can be changed via configuration
#define MOTION_SLEEP_ENABLED_DEFAULT      1       // Motion sleep enabled by default

// ========================================================================================
// Platform Information
// ========================================================================================

#define TEIA_CAR    (9)  // custom vehicle
#define TEIA_TOOL   (8)  // custom personal

// Tag Type Information
// ========================================================================================

#define TAG_TYPE_UNKNOWN    (0x00)  // Unknown
#define TAG_TYPE_SMALL      (0x01)  // Small Tag
#define TAG_TYPE_NORMAL     (0x02)  // Normal Tag

// ========================================================================================
// Version Information
// ========================================================================================

#define SW_VERSION_MAJOR                1       // Software major version
#define SW_VERSION_MINOR                2       // Software minor version  
#define SW_VERSION_PATCH                3       // Software patch version

#define HW_VERSION_MAJOR                1       // Hardware major version
#define HW_VERSION_MINOR                2       // Hardware minor version

// ========================================================================================
// Random TX Interval Configuration
// ========================================================================================

// Default values (can be changed at runtime)
#define RANDOM_TX_ENABLE_DEFAULT         1       // Default random TX interval setting
#define RANDOM_TX_MIN_OFFSET_MS_DEFAULT  -495    // Default minimum random offset (-495ms)
#define RANDOM_TX_MAX_OFFSET_MS_DEFAULT  485     // Default maximum random offset (+485ms)

// ========================================================================================
// Sensor Bit Definitions
// ========================================================================================

#define LIS2DH12                    (1<<0) // LIS2DH12 sensor bit position

// ========================================================================================
// Motion Detection Configuration
// ========================================================================================

// LIS2DH12 Motion Detection Settings
#define MOTION_DETECTION_ODR_25HZ       25      // 25Hz ODR for motion detection
#define MOTION_DETECTION_THRESHOLD      0x01    // 48mg threshold (0x01)
#define MOTION_DETECTION_DURATION       0x01    // 40ms duration (0x01)
#define MOTION_DETECTION_CTRL_REG1      0x3F    // 25Hz ODR + Low-power + All axes enabled


// ========================================================================================
// Sensor Configuration
// ========================================================================================

/**
 * @brief Sensor configuration structure
 * @details Contains configuration settings for various sensors including
 *          motion, IMU, barometer, and AHRS settings.
 */
typedef struct {
    // Motion sensor settings
    uint8_t motion_sensor_mounted;        /**< Motion sensor mounted status (bit field) */
    uint8_t motion_sensor_active;         /**< Motion sensor active status (bit field) */
    uint16_t mcr_threshold;               /**< Motion control range threshold */
    
    // IMU settings
    uint8_t imu_fs_range;                 /**< IMU full scale range setting */
    
    // Barometer settings
    uint8_t baro_setting;                 /**< Barometer precision setting */
    
    // AHRS settings
    uint8_t ahrs_representation;          /**< AHRS representation setting */
    
    // Extended blink content settings
    uint8_t eblink_cont;                  /**< Extended blink content control */
    bool ahrs_enable;                     /**< AHRS enable/disable flag */
} sensor_config_t;

// Default sensor configuration values
#define SENSOR_DEFAULT_MOTION_MOUNTED     0x00    // Motion sensor not mounted by default
#define SENSOR_DEFAULT_MOTION_ACTIVE      0x00    // Motion sensor not active by default
#define SENSOR_DEFAULT_MCR_THRESHOLD      0x0000  // No motion control by default
#define SENSOR_DEFAULT_IMU_FS_RANGE       0x00    // IMU not configured by default
#define SENSOR_DEFAULT_BARO_SETTING       0x00    // Barometer not configured by default
#define SENSOR_DEFAULT_AHRS_REPRESENTATION 0x00   // AHRS disabled by default
#define SENSOR_DEFAULT_EBLINK_CONT        0x00    // No extended blink content by default
#define SENSOR_DEFAULT_AHRS_ENABLE        false   // AHRS disabled by default

// ========================================================================================
// Motion Control Range (MCR) Configuration
// ========================================================================================

#define MCR_THRESHOLD_LOW_BYTE_DEFAULT     0x00   // Default MCR threshold low byte
#define MCR_THRESHOLD_HIGH_BYTE_DEFAULT    0x00   // Default MCR threshold high byte

// ========================================================================================
// Backchannel Configuration
// ========================================================================================

#define BACKCHANNEL_VERSION_DEFAULT       1       // Default backchannel protocol version

// ========================================================================================
// Counter Thresholds (calculated at compile time)
// ========================================================================================

// Counter thresholds calculated from intervals (default values)
#define ADC_COUNTER_THRESHOLD_DEFAULT            10
#define LED_BLINK_COUNTER_THRESHOLD_DEFAULT      5
#define NO_MOTION_DELAY_COUNT_THRESHOLD_DEFAULT  50 // No motion delay count threshold

#endif // DEF_CONFIG_H
