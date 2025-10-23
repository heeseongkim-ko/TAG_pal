/**
 * @file Func_TEIA_ROUTINES.c
 * @brief GPIO initialization and control functions implementation for TEIA platform
 * @details This module implements comprehensive GPIO management for the TEIA platform,
 *          including LED control, peripheral pin configuration, and power management.
 * @author Platform Tag Team
 * @date 2024
 * @version 1.0
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "Func_TEIA_ROUTINES.h"
#include "def_pin.h"
#include "def_packet.h"
#include "def_config.h"
#include "Func_ADC.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_power.h"
#include "Api_uwb.h"
#include "Api_motion.h"
#include "Api_Led.h"
#include "Func_UART_LOG.h"

// ========================================================================================
// Static Variables
// ========================================================================================

// Counter variables for different functionalities
static uint16_t s_adc_counter = ADC_COUNTER_THRESHOLD_DEFAULT;           // 
static uint16_t s_bc_packet_counter = BACKCHANNEL_PACKET_INTERVAL_COUNT_DEFAULT;  // 
static uint16_t s_bat_packet_counter = BATTERY_PACKET_INTERVAL_COUNT;     // 
static uint16_t s_info_packet_counter = INFO_PACKET_INTERVAL_COUNT;       //
static uint16_t s_led_counter = LED_BLINK_COUNTER_THRESHOLD_DEFAULT;      // 
static uint16_t s_start_info_packet_counter = START_INFO_PACKET_INTERVAL_COUNT_DEFAULT;       // Send info packet 3 times at startup
static uint32_t s_motion_counter = NO_MOTION_DELAY_COUNT_THRESHOLD_DEFAULT;  // Motion detection counter 

static bool s_led_state = false;

// Static variables for packet header management
static uint8_t s_prepare_state = PACKET_PREPARE_STATE_HEADER;
static uint8_t s_sequence_number = 0;
static uint8_t s_tx_buffer[100];  // TX buffer (max 100 bytes)
static uint16_t s_tx_buffer_length = 0;

extern mac_header_bb_t g_mac_header;

// Static variables for packet flags
static packet_flags_t s_packet_flags;  // Packet flags structure
static motion_flags_t s_motion_flags;  // Motion flags structure

// Static variables for ADC measurement
static adc_flags_t s_adc_flags;  // ADC flags structure

// TEIA system structures
static teia_system_config_t s_teia_system_config;
static teia_time_counters_t s_teia_time_counters;
static teia_counters_t s_teia_counters;

// Static tag info structure
static tag_info_msg_t s_info_payload;  // Static tag info payload structure

// Battery LED control definitions
#define GREEN_LIMIT_POWER   	185 // 3.65V
#define ORANGE_LIMIT_POWER 	178 // 3.5V
#define RED_LIMIT_POWER   		170 // 3.35V

// Battery LED control variables
static bool s_battery_led_state = false;  // Current LED state for blinking
static uint16_t s_battery_led_blink_counter = 0;  // Blink counter for RED LED
static uint8_t s_current_led_mode = 0;  // Current LED mode: 0=none, 1=green, 2=orange, 3=red_blink

// Random TX interval control variables (runtime configurable)
static bool s_random_tx_enable = RANDOM_TX_ENABLE_DEFAULT;  // Random TX enable flag
static int32_t s_random_tx_min_offset = RANDOM_TX_MIN_OFFSET_MS_DEFAULT;  // Min random offset
static int32_t s_random_tx_max_offset = RANDOM_TX_MAX_OFFSET_MS_DEFAULT;  // Max random offset

// Sensor configuration (runtime configurable)
sensor_config_t s_sensor_config;


 
// ========================================================================================
// GPIO Initialization Functions
// ========================================================================================

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
void Func_TEIA_GPIO_Init(void)
{
    // Initialize LED pins
    Func_TEIA_LED_Init();
    
    // Initialize UART pins
    Func_TEIA_UART_Pins_Init();
    
    // Initialize SPI pins
    Func_TEIA_SPI_Pins_Init();
    
    // Initialize ADC pins
    Func_TEIA_ADC_Pins_Init();
    
    // Initialize I2C pins
    Func_TEIA_I2C_Pins_Init();
    
    // Initialize IRQ pins
    Func_TEIA_IRQ_Pins_Init();
}

/**
 * @brief Initialize LED GPIO pins
 * @details Configures RED and GREEN LED pins as outputs with initial OFF state.
 *          LEDs are active LOW (LOW = ON, HIGH = OFF).
 * @return None
 */
void Func_TEIA_LED_Init(void)
{
    // Configure LED pins as outputs
    nrf_gpio_cfg_output(LED_RED_TEIA);
    nrf_gpio_cfg_output(LED_GREEN_TEIA);
    
    // Initial state is OFF (LEDs are active HIGH)
    nrf_gpio_pin_set(LED_RED_TEIA);    // LED OFF (HIGH = OFF)
    nrf_gpio_pin_set(LED_GREEN_TEIA);  // LED OFF (HIGH = OFF)
}

/**
 * @brief Initialize UART GPIO pins
 * @details Configures TX pin as output and RX pin as input with pull-up.
 *          Sets initial TX state to high (idle state).
 * @return None
 */
void Func_TEIA_UART_Pins_Init(void)
{
    // Manually configure UART pins to ensure they are set correctly
    nrf_gpio_cfg_output(UART_TX_PIN_NUMBER_TEIA);  // TX as output
    nrf_gpio_cfg_input(UART_RX_PIN_NUMBER_TEIA, NRF_GPIO_PIN_PULLUP);  // RX as input with pullup
    
    // Set initial TX state to high (idle state)
    nrf_gpio_pin_set(UART_TX_PIN_NUMBER_TEIA);
}

/**
 * @brief Initialize SPI GPIO pins for DW3000
 * @details Configures all SPI pins including CLK, MOSI, MISO, CS, Wake-up, IRQ, and Reset.
 *          Sets appropriate initial states for each pin.
 * @return None
 */
void Func_TEIA_SPI_Pins_Init(void)
{
    // Configure SPI CLK pin as output
    nrf_gpio_cfg_output(DW3000_CLK_Pin_TEIA);
    
    // Configure SPI MOSI pin as output
    nrf_gpio_cfg_output(DW3000_MOSI_Pin_TEIA);
    
    // Configure SPI MISO pin as input (no pull-up)
    nrf_gpio_cfg_input(DW3000_MISO_Pin_TEIA, NRF_GPIO_PIN_NOPULL);
    
    // Configure SPI CS pin as output (initial state is HIGH)
    nrf_gpio_cfg_output(DW3000_CS_Pin_TEIA);
    nrf_gpio_pin_set(DW3000_CS_Pin_TEIA);
    
    // Configure DW3000 Wake-up pin as output (initial state is LOW)
    nrf_gpio_cfg_output(DW3000_WKUP_Pin_TEIA);
    nrf_gpio_pin_clear(DW3000_WKUP_Pin_TEIA);
    
    // Configure DW3000 IRQ pin as input (pull-up enabled)
    nrf_gpio_cfg_input(DW3000_IRQ_Pin_TEIA, NRF_GPIO_PIN_PULLUP);
    
    // Configure DW3000 Reset pin as output (initial state is HIGH)
    nrf_gpio_cfg_output(DW3000_RST_Pin_TEIA);
    nrf_gpio_pin_set(DW3000_RST_Pin_TEIA);
}

/**
 * @brief Initialize ADC GPIO pins
 * @details Configures ADC measurement and resistive divider pins for battery voltage measurement.
 *          Sets initial states for safe operation.
 * @return None
 */
void Func_TEIA_ADC_Pins_Init(void)
{
    nrf_gpio_cfg_output(ADC_RDIV_TEIA);
    nrf_gpio_pin_set(ADC_RDIV_TEIA);

    nrf_gpio_cfg_input(ADC_MEAS_TEIA, NRF_GPIO_PIN_PULLUP);
}

/**
 * @brief Initialize I2C GPIO pins
 * @details Configures SCL and SDA pins as inputs with pull-up resistors.
 *          Standard I2C configuration for open-drain operation.
 * @return None
 */
void Func_TEIA_I2C_Pins_Init(void)
{
    // Configure I2C SCL pin as input (pull-up enabled)
    nrf_gpio_cfg_input(I2C_SCL_TEIA, NRF_GPIO_PIN_PULLUP);
    
    // Configure I2C SDA pin as input (pull-up enabled)
    nrf_gpio_cfg_input(I2C_SDA_TEIA, NRF_GPIO_PIN_PULLUP);
}

/**
 * @brief Initialize IRQ GPIO pins
 * @details Configures accelerometer IRQ and NFC FD pins as inputs with pull-up resistors.
 *          Enables interrupt detection for these peripherals.
 * @return None
 */
void Func_TEIA_IRQ_Pins_Init(void)
{
    // Configure ACC IRQ pin as input (pull-up enabled)
    nrf_gpio_cfg_input(ACC_IRQ_TEIA, NRF_GPIO_PIN_PULLUP);
    
    // Configure NFC FD pin as input (pull-up enabled)
    nrf_gpio_cfg_input(NFC_FD_TEIA, NRF_GPIO_PIN_PULLUP);
}

/** @} */

// ========================================================================================
// LED Control Functions
// ========================================================================================

/**
 * @defgroup TEIA_LED_Control LED Control Functions
 * @{
 */

/**
 * @brief Set LED state (ON/OFF)
 * @details Controls individual LED states with support for RED, GREEN, and ORANGE colors.
 *          ORANGE is implemented as both RED and GREEN LEDs simultaneously.
 *          LEDs are active LOW (LOW = ON, HIGH = OFF).
 * @param[in] led_id LED identifier (RED_TEIA, GREEN_TEIA, ORANGE_TEIA)
 * @param[in] state true for ON, false for OFF
 * @return None
 */
void Func_TEIA_LED_Set(uint8_t led_id, bool state)
{
    switch (led_id)
    {
        case RED_TEIA:
            if (state)
                nrf_gpio_pin_clear(LED_RED_TEIA);    // LED ON (LOW = ON)
            else
                nrf_gpio_pin_set(LED_RED_TEIA);      // LED OFF (HIGH = OFF)
            break;
            
        case GREEN_TEIA:
            if (state)
                nrf_gpio_pin_clear(LED_GREEN_TEIA);  // LED ON (LOW = ON)
            else
                nrf_gpio_pin_set(LED_GREEN_TEIA);    // LED OFF (HIGH = OFF)
            break;
            
        case ORANGE_TEIA:
            // ORANGE is both RED and GREEN turned on simultaneously
            if (state)
            {
                nrf_gpio_pin_clear(LED_RED_TEIA);    // LED ON (LOW = ON)
                nrf_gpio_pin_clear(LED_GREEN_TEIA);  // LED ON (LOW = ON)
            }
            else
            {
                nrf_gpio_pin_set(LED_RED_TEIA);      // LED OFF (HIGH = OFF)
                nrf_gpio_pin_set(LED_GREEN_TEIA);    // LED OFF (HIGH = OFF)
            }
            break;
            
        default:
            break;
    }
}

/**
 * @brief Toggle LED state
 * @details Toggles the current state of specified LED(s).
 *          ORANGE toggles both RED and GREEN LEDs simultaneously.
 * @param[in] led_id LED identifier (RED_TEIA, GREEN_TEIA, ORANGE_TEIA)
 * @return None
 */
void Func_TEIA_LED_Toggle(uint8_t led_id)
{
    switch (led_id)
    {
        case RED_TEIA:
            nrf_gpio_pin_toggle(LED_RED_TEIA);
            break;
            
        case GREEN_TEIA:
            nrf_gpio_pin_toggle(LED_GREEN_TEIA);
            break;
            
        case ORANGE_TEIA:
            // ORANGE is both RED and GREEN toggled simultaneously
            nrf_gpio_pin_toggle(LED_RED_TEIA);
            nrf_gpio_pin_toggle(LED_GREEN_TEIA);
            break;
            
        default:
            break;
    }
}

/**
 * @brief Set random TX interval settings
 * @details Configures random TX interval parameters at runtime
 * @param[in] enable Enable/disable random TX interval
 * @param[in] min_offset Minimum random offset in ms
 * @param[in] max_offset Maximum random offset in ms
 * @return None
 */
void Func_TEIA_Set_Random_TX_Settings(bool enable, int32_t min_offset, int32_t max_offset)
{
    s_random_tx_enable = enable;
    s_random_tx_min_offset = min_offset;
    s_random_tx_max_offset = max_offset;
    
    printf_uart("Random TX settings updated: enable=%d, min=%ld, max=%ld\r\n", 
                enable, min_offset, max_offset);
}

/**
 * @brief Get current random TX settings
 * @details Returns current random TX interval settings
 * @param[out] enable Pointer to store enable flag
 * @param[out] min_offset Pointer to store minimum offset
 * @param[out] max_offset Pointer to store maximum offset
 * @return None
 */
void Func_TEIA_Get_Random_TX_Settings(bool* enable, int32_t* min_offset, int32_t* max_offset)
{
    if (enable != NULL) *enable = s_random_tx_enable;
    if (min_offset != NULL) *min_offset = s_random_tx_min_offset;
    if (max_offset != NULL) *max_offset = s_random_tx_max_offset;
}

/**
 * @brief Initialize random TX settings with system config
 * @details Initializes random TX settings from system configuration
 * @return None
 */
void Func_TEIA_Init_Random_TX_Settings(void)
{
    s_random_tx_enable = s_teia_system_config.random_apply_enable;
    s_random_tx_min_offset = RANDOM_TX_MIN_OFFSET_MS_DEFAULT;
    s_random_tx_max_offset = RANDOM_TX_MAX_OFFSET_MS_DEFAULT;
}

/**
 * @brief Initialize all counters from structures
 * @details Initializes both time-based and packet counters from their respective structures
 * @return None
 */
void Func_TEIA_Init_All_Counters(void)
{
    // Initialize time-based counters from thresholds
    s_adc_counter = s_teia_time_counters.adc_counter_threshold;
    s_led_counter = s_teia_time_counters.led_blink_counter_threshold;
    s_motion_counter = s_teia_time_counters.no_motion_delay_count_threshold;
    
    // Initialize packet counters from structure
    s_bc_packet_counter = s_teia_counters.bc_packet_counter;
    s_bat_packet_counter = s_teia_counters.bat_packet_counter;
    s_info_packet_counter = s_teia_counters.info_packet_counter;
    s_start_info_packet_counter = s_teia_counters.start_info_packet_counter;
}

/**
 * @brief Reset sensor configuration to default values
 * @details Initializes sensor configuration settings to default configuration
 * @return None
 */
void Func_TEIA_Reset_Sensor_Config(void)
{
    s_sensor_config.motion_sensor_mounted = SENSOR_DEFAULT_MOTION_MOUNTED;
    s_sensor_config.motion_sensor_active = SENSOR_DEFAULT_MOTION_ACTIVE;
    s_sensor_config.mcr_threshold = SENSOR_DEFAULT_MCR_THRESHOLD;
    s_sensor_config.imu_fs_range = SENSOR_DEFAULT_IMU_FS_RANGE;
    s_sensor_config.baro_setting = SENSOR_DEFAULT_BARO_SETTING;
    s_sensor_config.ahrs_representation = SENSOR_DEFAULT_AHRS_REPRESENTATION;
    s_sensor_config.eblink_cont = SENSOR_DEFAULT_EBLINK_CONT;
    s_sensor_config.ahrs_enable = SENSOR_DEFAULT_AHRS_ENABLE;
}



/** @} */

// ========================================================================================
// Power Management Functions
// ========================================================================================

/**
 * @defgroup TEIA_Power_Management Power Management Functions
 * @{
 */

/**
 * @brief Configure RAM retention for power management
 * @details Enables power for all RAM banks (RAM0-RAM7) for 64kB total retention.
 *          Uses official Nordic SDK register access for safe configuration.
 *          Retention settings are handled automatically by the SDK.
 * @return None
 * @note This function should be called during system initialization
 */
void Func_TEIA_Configure_RAM_Retention(void)
{
    // Configure nRF52 RAM retention parameters for System On 64kB RAM retention
    // Using official Nordic SDK register access (safe way)
    
    // Enable RAM0-7 power (64kB total)
    // Note: This is the safe way to configure RAM power in nRF52
    NRF_POWER->RAM[0].POWER = 1;  // RAM0 ON
    NRF_POWER->RAM[1].POWER = 1;  // RAM1 ON
    NRF_POWER->RAM[2].POWER = 1;  // RAM2 ON
    NRF_POWER->RAM[3].POWER = 1;  // RAM3 ON
    NRF_POWER->RAM[4].POWER = 1;  // RAM4 ON
    NRF_POWER->RAM[5].POWER = 1;  // RAM5 ON
    NRF_POWER->RAM[6].POWER = 1;  // RAM6 ON
    NRF_POWER->RAM[7].POWER = 1;  // RAM7 ON
    
    // Note: Retention settings are handled automatically by the SDK
    // when entering/exiting sleep modes
}

/** @} */

// ========================================================================================
// Integrated Processing Functions
// ========================================================================================

/**
 * @defgroup TEIA_Integrated_Processing Integrated Processing Functions
 * @{
 */

/**
 * @brief Process all integrated functions based on counters
 * @details Manages ADC measurement, packet preparation, and LED control
 *          based on configured intervals and thresholds.
 * @return None
 * @note This function should be called after each UWB TX completion
 */
void Func_TEIA_Process_counter(void)
{
    // Decrement counters
    s_adc_counter--;      // 
    s_bc_packet_counter--; // 
    s_bat_packet_counter--; // 
    s_led_counter--;      //  
    //s_motion_counter--;   // Motion detection counter
    
    // Process ADC functionality
    if (s_adc_counter <= 0) 
    {
        // Set ADC measurement flag
        s_adc_flags.adc_measurement_flag = true;
        s_adc_counter = s_teia_time_counters.adc_counter_threshold;
    }
    
    // Process BC functionality 
    if(s_bc_packet_counter == 0) 
    {
        //bc packet flag
        s_packet_flags.bc_packet_flag = true;
        s_bc_packet_counter = BACKCHANNEL_PACKET_INTERVAL_COUNT_DEFAULT;
    }
    
    // Process info packet functionality (send 3 times at startup)
    if(s_start_info_packet_counter > 0)
    {
        s_start_info_packet_counter--;
        s_packet_flags.info_packet_flag = true;
    }

    // Process battery packet functionality
    if(s_bat_packet_counter == 0) {
        //bat packet flag
        s_packet_flags.battery_packet_flag = true;
        s_bat_packet_counter = BATTERY_PACKET_INTERVAL_COUNT;
        s_info_packet_counter--;  //
        
        if(s_info_packet_counter == 0) {
            //info packet flag 
            s_packet_flags.battery_packet_flag = false;
            s_packet_flags.info_packet_flag = true;
            s_info_packet_counter = INFO_PACKET_INTERVAL_COUNT;
        }
    }

    // Process LED functionality
    if (s_led_counter <= 0) 
    {
        // Set LED flag for battery LED processing
        s_adc_flags.led_processing_flag = true;
        s_led_counter = s_teia_time_counters.led_blink_counter_threshold;
    }
    
    // Process motion sleep functionality (only if enabled)
    if (s_motion_flags.motion_sleep_enabled && s_motion_counter == 0) 
    {
        // Set motion sleep flag when no motion detected for threshold time
        s_motion_flags.motion_sleep_flag = true;
        printf_uart("Motion sleep flag set - no motion detected\r\n");
    }
}

/**
 * @brief Reset all counters
 * @details Resets all internal counters to zero.
 * @return None
 */
void Func_TEIA_Reset_Counters(void)
{
    // Initialize time-based counters from thresholds
    s_adc_counter = s_teia_time_counters.adc_counter_threshold;
    s_led_counter = s_teia_time_counters.led_blink_counter_threshold;
    s_motion_counter = s_teia_time_counters.no_motion_delay_count_threshold;
    
    // Initialize packet counters from structure
    s_bc_packet_counter = s_teia_counters.bc_packet_counter;
    s_bat_packet_counter = s_teia_counters.bat_packet_counter;
    s_info_packet_counter = s_teia_counters.info_packet_counter;
    s_start_info_packet_counter = s_teia_counters.start_info_packet_counter;
}

/**
 * @brief Reset packet flags
 * @details Resets all packet flags to false.
 * @return None
 */
void Func_TEIA_Reset_Packet_Flags(void)
{
    s_packet_flags.bc_packet_flag = false;
    s_packet_flags.battery_packet_flag = false;
    s_packet_flags.info_packet_flag = true;
}

/**
 * @brief Initialize system configuration with default values
 * @details Initializes teia_system_config_t with default values from def_config.h
 * @return None
 */
void Func_TEIA_Init_System_Config_Default(void)
{
    s_teia_system_config.tx_interval = TX_INTERVAL_MS_DEFAULT;
    s_teia_system_config.random_apply_enable = RANDOM_TX_ENABLE_DEFAULT;
    s_teia_system_config.motion_sleep_apply_enable = MOTION_SLEEP_ENABLED_DEFAULT;
    s_teia_system_config.motion_sleep_check_time = ACC_TIME_TO_WAIT_INMOTION_DEFAULT;
    s_teia_system_config.motion_sleep_tx_interval = TX_INTERVAL_MOTION_SLEEP_MS_DEFAULT;
}

/**
 * @brief Initialize time-based counter reset values
 * @details Calculates and initializes teia_time_counters_t from system config
 * @return None
 */
void Func_TEIA_Init_Time_Counters_Default(void)
{
    s_teia_time_counters.adc_counter_threshold = ADC_COUNTER_THRESHOLD_DEFAULT;
    s_teia_time_counters.led_blink_counter_threshold = LED_BLINK_COUNTER_THRESHOLD_DEFAULT;
    s_teia_time_counters.no_motion_delay_count_threshold = NO_MOTION_DELAY_COUNT_THRESHOLD_DEFAULT;
}

/**
 * @brief Initialize packet counters with default values
 * @details Initializes teia_counters_t with default counter values
 * @return None
 */
void Func_TEIA_Init_Packet_Counters_Default(void)
{
    // Pure packet counters (fixed TX intervals)
    s_teia_counters.bc_packet_counter = BACKCHANNEL_PACKET_INTERVAL_COUNT_DEFAULT;
    s_teia_counters.bat_packet_counter = BATTERY_PACKET_INTERVAL_COUNT;
    s_teia_counters.info_packet_counter = INFO_PACKET_INTERVAL_COUNT;
    s_teia_counters.start_info_packet_counter = START_INFO_PACKET_INTERVAL_COUNT_DEFAULT;
}

/**
 * @brief Reset all TEIA variables
 * @details Resets all counters, packet flags, and other internal variables.
 *          Initializes new system structures with default values.
 * @return None
 */
void Func_TEIA_Reset_All_Variables(void)
{
    // Initialize new system structures with default values
    Func_TEIA_Init_System_Config_Default();
    Func_TEIA_Init_Time_Counters_Default();
    Func_TEIA_Init_Packet_Counters_Default();
    
    // Initialize all counters from structures
    Func_TEIA_Init_All_Counters();
    
    // Initialize random TX settings
    Func_TEIA_Init_Random_TX_Settings();
    
    // Initialize motion sleep from system config
    s_motion_flags.motion_sleep_enabled = s_teia_system_config.motion_sleep_apply_enable;
    s_motion_flags.motion_sleep_flag = false;
    
    // Reset other functions
    Func_TEIA_Reset_Packet_Flags();
    Func_TEIA_Reset_Sequence();
    Func_TEIA_Reset_Sensor_Config();
    
    // Reset ADC flags
    s_adc_flags.adc_measurement_flag = false;
    
    // Initial battery voltage measurement (no longer needed as we use function calls)
    
    s_tx_buffer_length = 0;
}

/**
 * @brief Update packet header with new fcode and auto-increment sequence
 * @details Updates the packet header fcode and automatically increments sequence number.
 * @param[in] fcode UWB frame code (BB, BC, etc.)
 * @return None
 */
void Func_TEIA_Update_Packet_Header(uint8_t fcode)
{
    // g_mac_header
    g_mac_header.fcode = fcode;
    g_mac_header.seqNum = s_sequence_number++;
}

/**
 * @brief Compose complete TX packet with header and payload
 * @details Creates complete UWB packet by combining header and payload data.
 * @param[in] payload_data Pointer to payload data
 * @param[in] payload_length Length of payload data in bytes
 * @return Total packet length in bytes
 */
uint16_t Func_TEIA_Compose_TX_Packet(uint8_t* payload_data, uint16_t payload_length)
{
    uint16_t total_length = 0;
    
    // 1. (g_mac_header )
    memcpy(s_tx_buffer, &g_mac_header, sizeof(mac_header_bb_t));
    total_length = sizeof(mac_header_bb_t);
    
    // 2. 
    if (payload_data != NULL && payload_length > 0) {
        memcpy(s_tx_buffer + total_length, payload_data, payload_length);
        total_length += payload_length;
    }
    
    // 3. 
    s_tx_buffer_length = total_length;
    
    //printf_uart("TX Packet Composed: Header=%d bytes, Payload=%d bytes, Total=%d bytes\r\n", 
    //            sizeof(mac_header_bb_t), payload_length, total_length);
    
    return total_length;
}

/**
 * @brief Prepare packet based on current flags
 * @details Checks packet flags and prepares the appropriate packet type for transmission
 * @return Total packet length in bytes
 */
static battery_msg_t battery_payload;
static uint8_t* payload_data = NULL;
static uint16_t payload_length = 0;

uint16_t Func_TEIA_Prepare_Packet(void)
{
    uint16_t total_length = 0;
    
    // 1. Header decision (BC or BB)
    if (s_packet_flags.bc_packet_flag) {
        Func_TEIA_Update_Packet_Header(UWB_FCODE_BC_POLL);
        printf_uart("Header: BC packet (0x%02X)\r\n", UWB_FCODE_BC_POLL);
        s_packet_flags.bc_packet_flag = false;  // Reset flag after processing
    } else {
        Func_TEIA_Update_Packet_Header(UWB_FCODE_BLINK);
    }
    
    // 2. Payload decision (Battery or Info)
    if (s_packet_flags.info_packet_flag) {
        // Info payload data preparation with message type wrapper
        static uint8_t info_packet_buffer[sizeof(tag_info_msg_t) + 4];  // Buffer for complete info packet
        uint8_t packet_index = 0;
        
        // Update battery voltage in global info payload
        s_info_payload.battVoltage = Func_ADC_Get_Current_Battery_Voltage();  // Get current battery voltage
        // Message type (0x64 for info message)
        info_packet_buffer[packet_index++] = 0x64;  // Message type
        
        // App ID (MSGTYPE_TAG_INFO = 14 for tag info message)
        info_packet_buffer[packet_index++] = MSGTYPE_TAG_INFO;  // App ID high byte
        info_packet_buffer[packet_index++] = 0x00;  // App ID low byte
        
        // Length (size of tag_info_msg_t)
        info_packet_buffer[packet_index++] = sizeof(tag_info_msg_t);  // Length
        
        // Copy tag info data
        memcpy(&info_packet_buffer[packet_index], &s_info_payload, sizeof(tag_info_msg_t));
        packet_index += sizeof(tag_info_msg_t);
        
        // Set payload data and length
        payload_data = info_packet_buffer;
        payload_length = packet_index;
        
        printf_uart("Payload: Info packet (type: 0x%02X, size: %d bytes)\r\n", 
                   MSGTYPE_TAG_INFO, payload_length);
        s_packet_flags.info_packet_flag = false;
    } else if (s_packet_flags.battery_packet_flag) {
        // Battery payload data preparation with message type
        static uint8_t battery_packet_buffer[10];  // Buffer for complete battery packet
        uint8_t packet_index = 0;
        
        // Message type (0x64 for battery message)
        battery_packet_buffer[packet_index++] = 0x64;  // Message type
        
        // App ID (MSGTYPE_BATT = 1 for battery message)
        battery_packet_buffer[packet_index++] = MSGTYPE_BATT;  // App ID high byte
        battery_packet_buffer[packet_index++] = 0x00;  // App ID low byte
        
        // Length (0x01 for single battery value)
        battery_packet_buffer[packet_index++] = 0x01;  // Length
        
        // App Data (battery voltage)
        battery_packet_buffer[packet_index] = Func_ADC_Get_Current_Battery_Voltage();  // Battery value
        
        // Set payload data and length
        payload_data = battery_packet_buffer;
        payload_length = packet_index + 1;  // +1 because we just added the last byte
        
        printf_uart("Payload: Battery packet (type: 0x%02X, voltage: %d, length: %d)\r\n", 
                   MSGTYPE_BATT, Func_ADC_Get_Current_Battery_Voltage(), payload_length);
        s_packet_flags.battery_packet_flag = false;
    } else {
        // No payload (default)
        payload_data = NULL;
        payload_length = 0;
    }
    
    // 3. Move to TX buffer
    total_length = Func_TEIA_Compose_TX_Packet(payload_data, payload_length);
    Api_uwb_set_tx_message(s_tx_buffer, total_length);
    
    // Debug: Print buffer contents
    printf_uart("Buffer [%d]: ", total_length);
    for (uint16_t i = 0; i < total_length; i++) {
        printf_uart("0x%02X", s_tx_buffer[i]);
        if (i < total_length - 1) {
            printf_uart(", ");
        }
    }
    printf_uart("\r\n");
    
    return total_length;
}

/**
 * @brief Get current TX buffer pointer
 * @details Returns pointer to the composed TX buffer for UWB transmission.
 * @return Pointer to TX buffer
 */
uint8_t* Func_TEIA_Get_TX_Buffer(void)
{
    return s_tx_buffer;
}

/**
 * @brief Get current TX buffer length
 * @details Returns the current length of data in TX buffer.
 * @return TX buffer length in bytes
 */
uint16_t Func_TEIA_Get_TX_Buffer_Length(void)
{
    return s_tx_buffer_length;
}

/**
 * @brief Reset sequence number to zero
 * @details Resets the sequence number counter to zero.
 * @return None
 */
void Func_TEIA_Reset_Sequence(void)
{
    s_sequence_number = 0;
    printf_uart("Sequence number reset to 0\r\n");
}

/**
 * @brief Reset packet prepare state to header state
 * @details Resets the packet prepare state to PACKET_PREPARE_STATE_HEADER.
 *          This allows the packet prepare process to start over from the beginning.
 * @return None
 */
void Func_TEIA_Reset_Packet_Prepare_State(void)
{
    s_prepare_state = PACKET_PREPARE_STATE_HEADER;
    printf_uart("Packet prepare state reset to HEADER (0)\r\n");
}

/**
 * @brief Perform ADC measurement when flag is set
 * @details Checks ADC measurement flag and performs battery voltage measurement.
 *          Updates global battery voltage variable.
 * @return None
 */
void Func_TEIA_Process_ADC_Measurement(void)
{
    if (s_adc_flags.adc_measurement_flag) {
        // Perform battery voltage measurement
        uint8_t l_battery_voltage = Func_ADC_Get_Current_Battery_Voltage();
        printf_uart("ADC Measurement: Battery voltage = %d\r\n", l_battery_voltage);
        
        // Reset flag after measurement
        s_adc_flags.adc_measurement_flag = false;
    }
}

/**
 * @brief Initialize tag info message structure
 * @details Fills the tag_info_msg_t structure with current system values including
 *          hardware/firmware versions, UWB settings, and sensor configurations.
 * @param[out] tag_info Pointer to tag_info_msg_t structure to initialize
 * @return None
 */
void Func_TEIA_Init_Tag_Info(void)
{
    // Battery voltage (get fresh measurement)
    s_info_payload.battVoltage = Func_ADC_Get_Current_Battery_Voltage();  // Get current battery voltage
    
    // Platform type (from def_config.h)
    s_info_payload.platform = 9;  // EIA_CAR platformT
    
    // Hardware version (from def_config.h)
    s_info_payload.hw_ver[0] = HW_VERSION_MAJOR;  // 0
    s_info_payload.hw_ver[1] = HW_VERSION_MINOR;  // 1
    
    // Firmware version (from def_config.h)
    s_info_payload.fw_ver[0] = SW_VERSION_MAJOR;  // 0
    s_info_payload.fw_ver[1] = SW_VERSION_MINOR;  // 0
    s_info_payload.fw_ver[2] = SW_VERSION_PATCH;  // 1
    
    // UWB radio settings (get actual values from UWB API)
    s_info_payload.channel = Api_uwb_get_channel();                    // Actual channel
    s_info_payload.data_rate = Api_uwb_get_data_rate();                // Actual data rate
    s_info_payload.preamble = Api_uwb_get_preamble_length();           // Actual preamble length
    s_info_payload.prf = Api_uwb_get_prf();                           // Actual PRF
    s_info_payload.preamCode = Api_uwb_get_preamble_code();            // Actual preamble code
    s_info_payload.nSfd = Api_uwb_get_sfd();                          // Actual SFD setting
    
    // Motion control and refresh settings
    s_info_payload.mcr = s_teia_system_config.motion_sleep_apply_enable;                 // From system config
    s_info_payload.random_dev = s_teia_system_config.random_apply_enable;  // From system config
    
    // Refresh intervals (in milliseconds, little-endian)
    uint32_t refresh_rate = s_teia_system_config.tx_interval;
    s_info_payload.refresh_interval[0] = (uint8_t)(refresh_rate & 0xFF);
    s_info_payload.refresh_interval[1] = (uint8_t)((refresh_rate >> 8) & 0xFF);
    s_info_payload.refresh_interval[2] = (uint8_t)((refresh_rate >> 16) & 0xFF);
    s_info_payload.refresh_interval[3] = (uint8_t)((refresh_rate >> 24) & 0xFF);
    
    // Sleep mode refresh interval (same as normal)
    memcpy(s_info_payload.sm_refresh_interval, s_info_payload.refresh_interval, 4);
    
    // TX power settings (get actual values from UWB API)
    Api_uwb_get_tx_power(s_info_payload.TXpower);
    
    // Sensor settings (from sensor configuration structure)
    s_info_payload.mounted_sensors = s_sensor_config.motion_sensor_mounted;  // Motion sensor mounted status
    s_info_payload.active_sensors = s_sensor_config.motion_sensor_active;    // Motion sensor active status
    
    // Motion control threshold (from sensor configuration)
    s_info_payload.mcr_threshold[0] = (uint8_t)(s_sensor_config.mcr_threshold & 0xFF);        // Low byte
    s_info_payload.mcr_threshold[1] = (uint8_t)((s_sensor_config.mcr_threshold >> 8) & 0xFF); // High byte
    
    // IMU settings (from sensor configuration)
    s_info_payload.IMU_FS_range = s_sensor_config.imu_fs_range;
    
    // Barometer settings (from sensor configuration)
    s_info_payload.BARO_setting = s_sensor_config.baro_setting;
    
    // AHRS settings (from sensor configuration)
    s_info_payload.AHRS_representation = s_sensor_config.ahrs_representation;
    
    // Backchannel settings
    s_info_payload.bc_version = 0x01;              // Backchannel version 1
    s_info_payload.bc_period_ri = s_teia_counters.bc_packet_counter;               // From structure
    s_info_payload.bc_period_rism = 0;             // Sleep mode disabled
    printf_uart("Tag info initialized: Platform=%d, HW=%d.%d, FW=%d.%d.%d\r\n", 
                s_info_payload.platform, 
                s_info_payload.hw_ver[0], s_info_payload.hw_ver[1],
                s_info_payload.fw_ver[0], s_info_payload.fw_ver[1], s_info_payload.fw_ver[2]);
    printf_uart("UWB settings: Ch=%d, Rate=%d, Preamble=%d, PRF=%d, Code=%d, SFD=%d\r\n",
                s_info_payload.channel, s_info_payload.data_rate, s_info_payload.preamble,
                s_info_payload.prf, s_info_payload.preamCode, s_info_payload.nSfd);
    printf_uart("TX Power: 0x%02X%02X%02X%02X\r\n",
                s_info_payload.TXpower[0], s_info_payload.TXpower[1], 
                s_info_payload.TXpower[2], s_info_payload.TXpower[3]);
}

/**
 * @brief Process battery LED based on voltage level
 * @details Controls LED display based on battery voltage:
 *          - Green: > 185 (3.65V) - Solid ON
 *          - Orange: 178-185 (3.5V-3.65V) - Solid ON  
 *          - Red: < 178 (3.5V) - Blinking (10ms on/off)
 * @return None
 */
void Func_TEIA_Process_Battery_LED(void)
{
    uint8_t new_led_mode = 0;
    uint8_t l_battery_voltage;

	#if  1
	if  (Api_Led_IsActive())
	{
		return;
	}	
	if (s_adc_flags.led_processing_flag) 
	{
        // Reset flag after processing
        s_adc_flags.led_processing_flag = false;
		return;
	}
    // Determine new LED mode based on voltage
    l_battery_voltage = Func_ADC_Get_Current_Battery_Voltage();
	
    if (l_battery_voltage >= GREEN_LIMIT_POWER) {
        new_led_mode = 1;  // Green

		Api_Led_StartBlink(API_LED_GREEN, 10, 0, 0);
    }
    else if (l_battery_voltage >= ORANGE_LIMIT_POWER) {
        new_led_mode = 2;  // Orange
		Api_Led_StartBlink(API_LED_ORANGE, 10, 0, 0);
    }
    else {
        new_led_mode = 3;  // Red blink
		Api_Led_StartBlink(API_LED_RED, 10, 0, 0);
    }

    #else
    l_battery_voltage = Func_ADC_Get_Current_Battery_Voltage();
	
    if (l_battery_voltage >= GREEN_LIMIT_POWER) {
        new_led_mode = 1;  // Green
    }
    else if (l_battery_voltage >= ORANGE_LIMIT_POWER) {
        new_led_mode = 2;  // Orange
    }
    else {
        new_led_mode = 3;  // Red blink
    }
    // Check if LED processing flag is set (10s interval)
    if (s_adc_flags.led_processing_flag) {
        // Reset flag after processing
        s_adc_flags.led_processing_flag = false;
        
        // Only change LED mode if it's different from current mode
        if (s_current_led_mode != new_led_mode) {
            s_current_led_mode = new_led_mode;
            
            // Turn off all LEDs first
            Func_TEIA_LED_Set(RED_TEIA, false);
            Func_TEIA_LED_Set(GREEN_TEIA, false);
            
            if (s_current_led_mode == 1) {
                // Green LED - Battery good
                Func_TEIA_LED_Set(GREEN_TEIA, true);
                printf_uart("Battery LED: GREEN (Voltage: %d)\r\n", l_battery_voltage);
            }
            else if (s_current_led_mode == 2) {
                // Orange LED - Battery warning
                Func_TEIA_LED_Set(RED_TEIA, true);
                Func_TEIA_LED_Set(GREEN_TEIA, true);
                printf_uart("Battery LED: ORANGE (Voltage: %d)\r\n", l_battery_voltage);
            }
            else if (s_current_led_mode == 3) {
                // Red LED - Battery low (turn on for 10ms)
                Func_TEIA_LED_Set(RED_TEIA, true);  //
                s_battery_led_blink_counter = 0;  //
                printf_uart("Battery LED: RED ON (Voltage: %d)\r\n", l_battery_voltage);
            }
        }
    }
    
    // Handle LED timing (1ms interval) 
    if (s_current_led_mode == 1 || s_current_led_mode == 2 || s_current_led_mode == 3) {
        s_battery_led_blink_counter++;
        
        // Keep LED ON for 10ms, then turn off
        if (s_battery_led_blink_counter >= 10) {
            Func_TEIA_LED_Set(RED_TEIA, false);
            Func_TEIA_LED_Set(GREEN_TEIA, false);
            s_current_led_mode = 0;  
            s_battery_led_blink_counter = 0;
        }
    }
	#endif
}

/**
 * @brief Initialize LIS2DH12 for motion detection
 * @details Configures LIS2DH12 accelerometer for low-power motion detection
 * @return true if initialization successful, false otherwise
 */
bool Func_Motion_Detection_Init(void)
{
    motion_error_t result;
    motion_config_t config;
    
    // Configure motion sensor settings
    config.i2c_address = MOTION_DEFAULT_I2C_ADDR;  // Default I2C address
    config.output_data_rate = MOTION_ODR_25HZ;  // 25Hz ODR
    config.full_scale = MOTION_FS_2G;   // ±2g full scale
    config.motion_threshold = MOTION_DETECTION_THRESHOLD;  // 48mg threshold
    config.motion_duration = MOTION_DETECTION_DURATION;    // 40ms duration
    config.enable_xyz_axes = true;  // Enable X, Y, Z axes
    
    // Initialize motion sensor with our configuration
    result = Api_motion_init(&config);
	
    if (result != MOTION_SUCCESS) {
        printf_uart("Motion sensor init failed: %d\r\n", result);
        return false;
    }
    else s_sensor_config.motion_sensor_mounted |= LIS2DH12;
    
    printf_uart("Motion detection initialized successfully (polling mode)\r\n");
    return true;
}

/**
 * @brief Check for motion detection
 * @details Polls the motion sensor to detect movement
 */
void Func_Motion_Detection_Check(void)
{
    // Use polling for motion detection in normal mode
    if (Api_motion_is_detected()) {
        printf_uart("Motion detected by polling\r\n");

        Api_motion_clear_interrupt();
        // Reset motion counter on detection
        s_motion_flags.motion_sleep_flag = false;
        s_motion_counter = s_teia_time_counters.no_motion_delay_count_threshold;
     }
}

/**
 * @brief Check motion wakeup condition during sleep
 * @details Checks if motion is detected and motion sleep flag is set, then clears flags
 * @return true if motion wakeup should occur, false otherwise
 */
bool Func_TEIA_Check_Motion_Wakeup(void)
{
    // Only check motion if currently in motion sleep mode
    if (s_motion_flags.motion_sleep_flag && Api_motion_is_detected()) {
        Api_motion_clear_interrupt();
        s_motion_flags.motion_sleep_flag = false;
        s_motion_counter = s_teia_time_counters.no_motion_delay_count_threshold;
        return true;
    }
    return false;
}

/**
 * @brief Get TX interval with motion sleep and random offset
 * @details Returns TX interval considering motion sleep flag and random offset
 * @return TX interval in milliseconds
 */
uint32_t Func_TEIA_Get_TX_Interval(void)
{
    uint32_t l_base_interval;
    
    // Get base interval based on motion sleep flag (only if motion sleep is enabled)
    if (s_motion_flags.motion_sleep_enabled && s_motion_flags.motion_sleep_flag) {
        // Motion sleep mode: use fixed 30s interval (no random)
        l_base_interval = TX_INTERVAL_MOTION_SLEEP_MS_DEFAULT;
    }
    else {
        // Normal mode: use configured base interval
        l_base_interval = s_teia_system_config.tx_interval;
        
        // Add random offset if enabled (only in normal mode)
        bool l_random_enable;
        uint32_t l_random_min, l_random_max;
        Func_TEIA_Get_Random_TX_Settings(&l_random_enable, &l_random_min, &l_random_max);
        
        if (l_random_enable) {
            uint32_t l_random_offset = (rand() % (l_random_max - l_random_min + 1)) + l_random_min;
            l_base_interval += l_random_offset;
        }
    }
    
    return l_base_interval;
}


/** @} */
