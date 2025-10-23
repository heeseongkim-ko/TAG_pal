/**
 * @file Func_TEIA_ROUTINES.h
 * @brief GPIO initialization and control functions for TEIA platform
 * @details This module provides comprehensive GPIO management for the TEIA platform,
 *          including LED control, UART/SPI/I2C pin configuration, and power management.
 * @author Platform Tag Team
 * @date 2024
 * @version 1.0
 */

#ifndef FUNC_TEIA_ROUTINES_H
#define FUNC_TEIA_ROUTINES_H

#include <stdbool.h>
#include <stdint.h>
#include "def_packet.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup TEIA_System_Values System Value Structures
 * @{
 */

/**
 * @brief TEIA system configuration parameters structure
 * @details Contains all configurable system parameters that can be modified at runtime
 */
typedef struct {
	uint32_t tx_interval;					///< TX transmission interval in milliseconds (default: TX_INTERVAL_MS_DEFAULT)
	bool random_apply_enable;				///< Random TX interval application enable/disable (default: RANDOM_TX_ENABLE_DEFAULT)
	bool motion_sleep_apply_enable;		///< Motion sleep mode application enable/disable (default: MOTION_SLEEP_ENABLED_DEFAULT)
	uint32_t motion_sleep_check_time;		///< Motion sleep check time interval in milliseconds (default: ACC_TIME_TO_WAIT_INMOTION_DEFAULT)
	uint32_t motion_sleep_tx_interval;		///< TX interval during motion sleep mode in milliseconds (default: TX_INTERVAL_MOTION_SLEEP_MS_DEFAULT)
} teia_system_config_t;

/**
 * @brief TEIA time-based counter values structure
 * @details Contains time intervals converted to TX counter values
 */
typedef struct {
	uint32_t adc_counter_threshold;			///< ADC measurement interval in TX counts (default: ADC_COUNTER_THRESHOLD_DEFAULT)
	uint32_t led_blink_counter_threshold;	///< LED blink interval in TX counts (default: LED_BLINK_COUNTER_THRESHOLD_DEFAULT)
	uint32_t no_motion_delay_count_threshold;	///< No motion delay interval in TX counts (default: NO_MOTION_DELAY_COUNT_THRESHOLD_DEFAULT)
} teia_time_counters_t;

/**
 * @brief TEIA packet counters structure
 * @details Contains pure packet-based counters and system state
 */
typedef struct {
	// Pure packet counters (fixed TX intervals)
	uint16_t bc_packet_counter;				///< Backchannel packet transmission counter (every 4th TX)
	uint16_t bat_packet_counter;			///< Battery packet transmission counter (every 15th TX)
	uint16_t info_packet_counter;			///< Info packet transmission counter (every 15th TX)
	uint16_t start_info_packet_counter;		///< Startup info packet counter (send 3 times)
} teia_counters_t;

/** @} */

/**
 * @defgroup TEIA_Overview Module Overview
 * @{
 * The TEIA routines module provides comprehensive GPIO management for the TEIA platform.
 * It includes:
 * - Complete GPIO initialization for all peripherals
 * - LED control with multiple color options
 * - Pin configuration for UART, SPI, I2C, and ADC
 * - Power management and RAM retention configuration
 * @}
 */

/**
 * @defgroup TEIA_GPIO_Init GPIO Initialization Functions
 * @{
 */

/**
 * @brief Initialize all GPIO pins for TEIA platform
 * @details Calls all individual pin initialization functions to set up
 *          the complete GPIO configuration for the platform.
 * @return None
 * @note This function must be called first before using any GPIO functions
 */
void Func_TEIA_GPIO_Init(void);

/**
 * @brief Initialize LED GPIO pins
 * @details Configures RED and GREEN LED pins as outputs with initial OFF state.
 * @return None
 */
void Func_TEIA_LED_Init(void);

/**
 * @brief Initialize UART GPIO pins
 * @details Configures TX pin as output and RX pin as input with pull-up.
 * @return None
 */
void Func_TEIA_UART_Pins_Init(void);

/**
 * @brief Initialize SPI GPIO pins for DW3000
 * @details Configures all SPI pins including CLK, MOSI, MISO, CS, Wake-up, IRQ, and Reset.
 * @return None
 */
void Func_TEIA_SPI_Pins_Init(void);

/**
 * @brief Initialize ADC GPIO pins
 * @details Configures ADC measurement and resistive divider pins for battery voltage measurement.
 * @return None
 */
void Func_TEIA_ADC_Pins_Init(void);

/**
 * @brief Initialize I2C GPIO pins
 * @details Configures SCL and SDA pins as inputs with pull-up resistors.
 * @return None
 */
void Func_TEIA_I2C_Pins_Init(void);

/**
 * @brief Initialize IRQ GPIO pins
 * @details Configures accelerometer IRQ and NFC FD pins as inputs with pull-up resistors.
 * @return None
 */
void Func_TEIA_IRQ_Pins_Init(void);

/** @} */

/**
 * @defgroup TEIA_LED_Control LED Control Functions
 * @{
 */

/**
 * @brief Set LED state (ON/OFF)
 * @details Controls individual LED states with support for RED, GREEN, and ORANGE colors.
 *          ORANGE is implemented as both RED and GREEN LEDs simultaneously.
 * @param[in] led_id LED identifier (RED_TEIA, GREEN_TEIA, ORANGE_TEIA)
 * @param[in] state true for ON, false for OFF
 * @return None
 * @note LEDs are active LOW (LOW = ON, HIGH = OFF)
 */
void Func_TEIA_LED_Set(uint8_t led_id, bool state);

/**
 * @brief Toggle LED state
 * @details Toggles the current state of specified LED(s).
 *          ORANGE toggles both RED and GREEN LEDs simultaneously.
 * @param[in] led_id LED identifier (RED_TEIA, GREEN_TEIA, ORANGE_TEIA)
 * @return None
 */
void Func_TEIA_LED_Toggle(uint8_t led_id);

/** @} */

/**
 * @defgroup TEIA_Power_Management Power Management Functions
 * @{
 */

/**
 * @brief Configure RAM retention for power management
 * @details Enables power for all RAM banks (RAM0-RAM7) for 64kB total retention.
 *          Uses official Nordic SDK register access for safe configuration.
 * @return None
 * @note Retention settings are handled automatically by the SDK
 */
void Func_TEIA_Configure_RAM_Retention(void);

/** @} */

/**
 * @defgroup TEIA_Integrated_Processing Integrated Processing Functions
 * @{
 */

/**
 * @brief Process counter-based integrated functions
 * @details Manages ADC measurement, packet preparation, and LED control
 *          based on counter thresholds and intervals.
 * @return None
 * @note This function should be called after each UWB TX completion
 */
void Func_TEIA_Process_counter(void);

/**
 * @brief Get current packet type
 * @details Returns the current packet type for external use.
 * @return Current packet type (packet_type_e)
 */
uint8_t Func_TEIA_Get_Current_Packet_Type(void);

/**
 * @brief Reset all counters
 * @details Resets all internal counters to zero.
 * @return None
 */
void Func_TEIA_Reset_Counters(void);

/**
 * @brief Reset packet flags
 * @details Resets all packet flags to false.
 * @return None
 */
void Func_TEIA_Reset_Packet_Flags(void);

/**
 * @brief Reset all TEIA variables
 * @details Resets all counters, packet flags, and other internal variables.
 * @return None
 */
void Func_TEIA_Reset_All_Variables(void);

/**
 * @brief Update packet header with new fcode and auto-increment sequence
 * @details Updates the packet header fcode and automatically increments sequence number.
 * @param[in] fcode UWB frame code (BB, BC, etc.)
 * @return None
 */
void Func_TEIA_Update_Packet_Header(uint8_t fcode);

/**
 * @brief Compose complete TX packet with header and payload
 * @details Creates complete UWB packet by combining header and payload data.
 * @param[in] payload_data Pointer to payload data
 * @param[in] payload_length Length of payload data in bytes
 * @return Total packet length in bytes
 */
uint16_t Func_TEIA_Compose_TX_Packet(uint8_t* payload_data, uint16_t payload_length);

/**
 * @brief Prepare packet based on current flags
 * @details Checks packet flags and prepares the appropriate packet type for transmission
 * @return Total packet length in bytes
 */
uint16_t Func_TEIA_Prepare_Packet(void);

/**
 * @brief Get current TX buffer pointer
 * @details Returns pointer to the composed TX buffer for UWB transmission.
 * @return Pointer to TX buffer
 */
uint8_t* Func_TEIA_Get_TX_Buffer(void);

/**
 * @brief Get current TX buffer length
 * @details Returns the current length of data in TX buffer.
 * @return TX buffer length in bytes
 */
uint16_t Func_TEIA_Get_TX_Buffer_Length(void);

/**
 * @brief Reset sequence number to zero
 * @details Resets the sequence number counter to zero.
 * @return None
 */
void Func_TEIA_Reset_Sequence(void);

/**
 * @brief Reset packet prepare state to header state
 * @details Resets the packet prepare state to PACKET_PREPARE_STATE_HEADER.
 *          This allows the packet preparation process to start over from the beginning.
 * @return None
 */
void Func_TEIA_Reset_Packet_Prepare_State(void);

/**
 * @brief Perform ADC measurement when flag is set
 * @details Checks ADC measurement flag and performs battery voltage measurement.
 *          Updates global battery voltage variable.
 * @return None
 */
void Func_TEIA_Process_ADC_Measurement(void);

/**
 * @brief Initialize tag info message structure
 * @details Fills the static s_info_payload structure with current system values
 * @return None
 */
void Func_TEIA_Init_Tag_Info(void);

/**
 * @brief Process battery LED based on voltage level
 * @details Controls LED display based on battery voltage:
 *          - Green: > 185 (3.65V) - Solid ON
 *          - Orange: 178-185 (3.5V-3.65V) - Solid ON  
 *          - Red: < 178 (3.5V) - Blinking (10ms on/off)
 * @return None
 */
void Func_TEIA_Process_Battery_LED(void);

/**
 * @brief Set random TX interval settings
 * @details Configures random TX interval parameters at runtime
 * @param[in] enable Enable/disable random TX interval
 * @param[in] min_offset Minimum random offset in ms
 * @param[in] max_offset Maximum random offset in ms
 * @return None
 */
void Func_TEIA_Set_Random_TX_Settings(bool enable, int32_t min_offset, int32_t max_offset);

/**
 * @brief Get current random TX settings
 * @details Returns current random TX interval settings
 * @param[out] enable Pointer to store enable flag
 * @param[out] min_offset Pointer to store minimum offset
 * @param[out] max_offset Pointer to store maximum offset
 * @return None
 */
void Func_TEIA_Get_Random_TX_Settings(bool* enable, int32_t* min_offset, int32_t* max_offset);

/**
 * @brief Initialize random TX settings with system config
 * @details Initializes random TX settings from system configuration
 * @return None
 */
void Func_TEIA_Init_Random_TX_Settings(void);

/**
 * @brief Initialize all counters from structures
 * @details Initializes both time-based and packet counters from their respective structures
 * @return None
 */
void Func_TEIA_Init_All_Counters(void);

/**
 * @brief Reset sensor configuration to default values
 * @details Initializes sensor configuration settings to default configuration
 * @return None
 */
void Func_TEIA_Reset_Sensor_Config(void);

/**
 * @brief Initialize LIS2DH12 for motion detection
 * @details Configures LIS2DH12 accelerometer for low-power motion detection
 * @return true if initialization successful, false otherwise
 */
bool Func_Motion_Detection_Init(void);

/**
 * @brief Check for motion detection
 * @details Polls the motion sensor to detect movement
 */
void Func_Motion_Detection_Check(void);

/**
 * @brief Check motion wakeup condition during sleep
 * @details Checks if motion is detected and motion sleep flag is set, then clears flags
 * @return true if motion wakeup should occur, false otherwise
 */
bool Func_TEIA_Check_Motion_Wakeup(void);

/**
 * @brief Get TX interval based on motion sleep flag
 * @details Returns appropriate TX interval based on motion sleep state
 * @return TX interval in milliseconds
 */
uint32_t Func_TEIA_Get_TX_Interval(void);

/**
 * @brief Initialize system configuration with default values
 * @details Initializes teia_system_config_t with default values from def_config.h
 * @return None
 */
void Func_TEIA_Init_System_Config_Default(void);

/**
 * @brief Initialize time-based counter reset values
 * @details Calculates and initializes teia_time_counters_t from system config
 * @return None
 */
void Func_TEIA_Init_Time_Counters_Default(void);

/**
 * @brief Initialize packet counters with default values
 * @details Initializes teia_counters_t with default counter values
 * @return None
 */
void Func_TEIA_Init_Packet_Counters_Default(void);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif // FUNC_TEIA_ROUTINES_H
