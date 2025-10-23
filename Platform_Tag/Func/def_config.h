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
#define TX_INTERVAL_MS_DEFAULT                   (5000UL)    // TX interval: 5000 ms (5 seconds)
#define TX_INTERVAL_MOTION_SLEEP_MS_DEFAULT      (30000UL)   // TX interval during motion sleep: 30 seconds

// Measurement and packet intervals
#define ADC_MEASUREMENT_INTERVAL_MS_DEFAULT      (5000UL)    // ADC measurement interval: 5 seconds
#define LED_BLINK_INTERVAL_MS_DEFAULT            (1000UL)//(10000UL)    // LED blink interval: 10 second
#define ACC_TIME_TO_WAIT_INMOTION_DEFAULT        (30000UL) // 30 seconds

// ========================================================================================
// Motion Sleep Configuration
// ========================================================================================

// Motion sleep feature can be enabled/disabled at runtime
// Default: enabled (1), can be changed via configuration
#define MOTION_SLEEP_ENABLED_DEFAULT      1       // Motion sleep enabled by default

// ========================================================================================
// Version Information
// ========================================================================================

#define SW_VERSION_MAJOR                1       // Software major version
#define SW_VERSION_MINOR                1       // Software minor version  
#define SW_VERSION_PATCH                1       // Software patch version

#define HW_VERSION_MAJOR                1       // Hardware major version
#define HW_VERSION_MINOR                1      // Hardware minor version

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
#define MOTION_DETECTION_THRESHOLD      0x03    // 48mg threshold (0x03)
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
// Counter Thresholds (calculated at compile time)
// ========================================================================================

// Counter thresholds calculated from intervals (default values)
#define ADC_COUNTER_THRESHOLD_DEFAULT            ((ADC_MEASUREMENT_INTERVAL_MS_DEFAULT + TX_INTERVAL_MS_DEFAULT/2) / TX_INTERVAL_MS_DEFAULT)
#define LED_BLINK_COUNTER_THRESHOLD_DEFAULT      ((LED_BLINK_INTERVAL_MS_DEFAULT + TX_INTERVAL_MS_DEFAULT/2) / TX_INTERVAL_MS_DEFAULT)
#define NO_MOTION_DELAY_COUNT_THRESHOLD_DEFAULT  ((ACC_TIME_TO_WAIT_INMOTION_DEFAULT + TX_INTERVAL_MS_DEFAULT/2) / TX_INTERVAL_MS_DEFAULT) // No motion delay in TX intervals

#endif // DEF_CONFIG_H
