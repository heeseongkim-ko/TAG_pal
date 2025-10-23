/**
 * @file Api_motion.c
 * @brief LIS2DH12 motion sensor API implementation
 * @details This file contains the implementation of API functions for interfacing
 *          with the LIS2DH12 3-axis MEMS accelerometer. It provides motion detection,
 *          acceleration data reading, and interrupt handling capabilities.
 * @author Tag Platform Team
 * @version 1.0
 * @date 2025-01-01
 */

#include "Api_port.h"
#include "Api_motion.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_error.h"
#include "nrf_log.h"
#include "Func_UART_LOG.h"
#include "def_config.h"
#include "Api_sleep.h"

#include <string.h>

//=============================================================================
// DEFINES
//=============================================================================

#define MOTION_I2C_TIMEOUT_MS		100

//=============================================================================
// STATIC VARIABLES
//=============================================================================

static const nrf_drv_twi_t motion_twi_instance_p = NRF_DRV_TWI_INSTANCE(1);
static motion_config_t motion_config_g;
static bool motion_initialized_b = false;
static bool motion_enabled_b = false;
static bool motion_interrupt_enabled_b = false;

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================

// External sensor configuration (defined in Func_TEIA_ROUTINES.c)
extern sensor_config_t s_sensor_config;

volatile bool motion_interrupt_flag_b = false;

//=============================================================================
// STATIC FUNCTION PROTOTYPES
//=============================================================================

/**
 * @brief Write a single register value to LIS2DH12
 * @param[in] reg_addr Register address to write
 * @param[in] value Value to write to register
 * @return Motion sensor error code
 */
static motion_error_t motion_write_register(uint8_t reg_addr, uint8_t value);

/**
 * @brief Read a single register value from LIS2DH12
 * @param[in] reg_addr Register address to read
 * @param[out] p_value Pointer to store read value
 * @return Motion sensor error code
 */
static motion_error_t motion_read_register(uint8_t reg_addr, uint8_t *p_value);

/**
 * @brief Read multiple consecutive registers from LIS2DH12
 * @param[in] reg_addr Starting register address
 * @param[out] p_data Pointer to buffer for read data
 * @param[in] length Number of bytes to read
 * @return Motion sensor error code
 */
static motion_error_t motion_read_multiple_registers(uint8_t reg_addr, uint8_t *p_data, uint8_t length);

/**
 * @brief Configure motion detection interrupts
 * @return Motion sensor error code
 */
static motion_error_t motion_configure_interrupts(void);

//=============================================================================
// STATIC FUNCTION IMPLEMENTATIONS
//=============================================================================

static motion_error_t motion_write_register(uint8_t reg_addr, uint8_t value)
{
	ret_code_t motion_err_code_ui32;
	uint8_t motion_tx_data_aui8[2];

	motion_tx_data_aui8[0] = reg_addr;
	motion_tx_data_aui8[1] = value;

	motion_err_code_ui32 = nrf_drv_twi_tx(&motion_twi_instance_p, motion_config_g.i2c_address, motion_tx_data_aui8, 2, false);
	if (motion_err_code_ui32 != NRF_SUCCESS) {
		LOG_API_IMU("I2C write failed, reg: 0x%02X, error: %d\r\n", reg_addr, motion_err_code_ui32);
		return MOTION_ERROR_I2C;
	}

	return MOTION_SUCCESS;
}

static motion_error_t motion_read_register(uint8_t reg_addr, uint8_t *p_value)
{
	ret_code_t motion_err_code_ui32;

	if (p_value == NULL) {
		return MOTION_ERROR_INVALID_PARAM;
	}

	motion_err_code_ui32 = nrf_drv_twi_tx(&motion_twi_instance_p, motion_config_g.i2c_address, &reg_addr, 1, true);
	if (motion_err_code_ui32 != NRF_SUCCESS) {
		LOG_API_IMU("I2C write address failed, reg: 0x%02X, error: %d\r\n", reg_addr, motion_err_code_ui32);
		return MOTION_ERROR_I2C;
	}

	motion_err_code_ui32 = nrf_drv_twi_rx(&motion_twi_instance_p, motion_config_g.i2c_address, p_value, 1);
	if (motion_err_code_ui32 != NRF_SUCCESS) {
		LOG_API_IMU("I2C read failed, reg: 0x%02X, error: %d\r\n", reg_addr, motion_err_code_ui32);
		return MOTION_ERROR_I2C;
	}

	return MOTION_SUCCESS;
}

static motion_error_t motion_read_multiple_registers(uint8_t reg_addr, uint8_t *p_data, uint8_t length)
{
	ret_code_t motion_err_code_ui32;
	uint8_t motion_reg_addr_ui8;

	if (p_data == NULL || length == 0) {
		return MOTION_ERROR_INVALID_PARAM;
	}

	motion_reg_addr_ui8 = reg_addr | 0x80;

	motion_err_code_ui32 = nrf_drv_twi_tx(&motion_twi_instance_p, motion_config_g.i2c_address, &motion_reg_addr_ui8, 1, true);
	if (motion_err_code_ui32 != NRF_SUCCESS) {
		LOG_API_IMU("I2C multi-read address failed, reg: 0x%02X, error: %d\r\n", reg_addr, motion_err_code_ui32);
		return MOTION_ERROR_I2C;
	}

	motion_err_code_ui32 = nrf_drv_twi_rx(&motion_twi_instance_p, motion_config_g.i2c_address, p_data, length);
	if (motion_err_code_ui32 != NRF_SUCCESS) {
		LOG_API_IMU("I2C multi-read failed, reg: 0x%02X, length: %d, error: %d\r\n", reg_addr, length, motion_err_code_ui32);
		return MOTION_ERROR_I2C;
	}

	return MOTION_SUCCESS;
}

static motion_error_t motion_configure_interrupts(void)
{
	motion_error_t motion_result_ui32;
	uint8_t motion_reg_value_ui8;

	motion_result_ui32 = motion_write_register(LIS2DH12_CTRL_REG3, LIS2DH12_I1_AOI1);
	if (motion_result_ui32 != MOTION_SUCCESS) {
		return motion_result_ui32;
	}

	motion_result_ui32 = motion_write_register(LIS2DH12_CTRL_REG5, LIS2DH12_LIR_INT1);
	if (motion_result_ui32 != MOTION_SUCCESS) {
		return motion_result_ui32;
	}

	motion_reg_value_ui8 = LIS2DH12_XHIE | LIS2DH12_YHIE | LIS2DH12_ZHIE;
	motion_result_ui32 = motion_write_register(LIS2DH12_INT1_CFG, motion_reg_value_ui8);
	if (motion_result_ui32 != MOTION_SUCCESS) {
		return motion_result_ui32;
	}

	motion_result_ui32 = motion_write_register(LIS2DH12_INT1_THS, motion_config_g.motion_threshold);
	if (motion_result_ui32 != MOTION_SUCCESS) {
		return motion_result_ui32;
	}

	motion_result_ui32 = motion_write_register(LIS2DH12_INT1_DURATION, motion_config_g.motion_duration);
	if (motion_result_ui32 != MOTION_SUCCESS) {
		return motion_result_ui32;
	}

	return MOTION_SUCCESS;
}

static void motion_interrupt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	if (pin == P_MOTION_INT) {
		motion_interrupt_flag_b = true;
		Api_sleep_setWakeupReason(API_SLEEP_WAKEUP_REASON_MASK_MOTION);
	}
}

//=============================================================================
// PUBLIC FUNCTION IMPLEMENTATIONS
//=============================================================================
/**
 * @brief Initialize TWI (I2C) interface for motion sensor
 *
 * This function initializes the TWI peripheral to communicate with the 
 * motion sensor (LIS2DH12) via I2C protocol. It configures the SCL/SDA pins,
 * sets the bus frequency to 100kHz, and enables the TWI instance.
 *
 * @note If TWI is already initialized (NRF_ERROR_INVALID_STATE), the function
 *       treats it as a successful operation and proceeds to enable the TWI.
 *
 * @return ret_code_t  NRF_SUCCESS on successful initialization
 *                     MOTION_ERROR_I2C if TWI initialization fails
 */
motion_error_t Api_motion_twi_init(void)
{
	motion_error_t motion_err_code_ui32;
	const nrf_drv_twi_config_t motion_twi_config_g = {
		.scl = P_MOTION_I2C_SCL,
		.sda = P_MOTION_I2C_SDA,
		.frequency = NRF_DRV_TWI_FREQ_100K,
		.interrupt_priority = APP_IRQ_PRIORITY_HIGH,
		.clear_bus_init = false
	};
	motion_err_code_ui32 = nrf_drv_twi_init(&motion_twi_instance_p, &motion_twi_config_g, NULL, NULL);
	if (motion_err_code_ui32 != NRF_SUCCESS && motion_err_code_ui32 != NRF_ERROR_INVALID_STATE) {
		LOG_API_IMU("TWI initialization failed: %d\r\n", motion_err_code_ui32);
		return MOTION_ERROR_I2C;
	}
	nrf_drv_twi_enable(&motion_twi_instance_p);
	return motion_err_code_ui32;
}

/**
 * @brief Uninitialize TWI (I2C) interface for motion sensor
 *
 * This function uninitializes the TWI peripheral used for motion sensor communication.
 * It is designed to be called before entering sleep mode to reduce power consumption.
 * The function first disables the TWI peripheral, then completely uninitializes it
 * to release all associated resources and minimize current draw.
 *
 * @details Execution sequence:
 *          1. Disable TWI peripheral to stop all communication
 *          2. Uninitialize TWI driver to release resources and reduce power
 *          3. Log the uninitialization result
 *
 * @note This function should be called before entering system sleep mode to ensure
 *       minimal power consumption. The TWI must be reinitialized with 
 *       Api_motion_twi_init() before resuming motion sensor operations.
 *
 * @warning Calling this function while TWI communication is in progress may result
 *          in undefined behavior. Ensure all pending transactions are complete.
 *
 * @return motion_error_t  MOTION_SUCCESS on successful uninitialization
 *
 * @see Api_motion_twi_init() to reinitialize TWI after waking from sleep
 */
motion_error_t Api_motion_twi_unInit(void)
{
	nrf_drv_twi_disable(&motion_twi_instance_p);
	nrf_drv_twi_uninit(&motion_twi_instance_p);
	
	LOG_API_IMU("TWI uninitialized for sleep mode\r\n");
	
	return MOTION_SUCCESS;
}

/**
 * @brief Initialize motion sensor with configuration
 * @param[in] p_config Pointer to motion sensor configuration structure
 * @return Motion sensor error code
 * @details This function initializes the LIS2DH12 motion sensor with the provided
 *          configuration. It sets up I2C communication, verifies device ID, configures
 *          registers for motion detection, and enables GPIO interrupt handling.
 */
motion_error_t Api_motion_init_with_config(const motion_config_t *p_config)
{
	motion_error_t motion_result_ui32;
	ret_code_t motion_err_code_ui32;
	uint8_t motion_device_id_ui8;
	uint8_t motion_reg_value_ui8;

	if (p_config == NULL) {
		return MOTION_ERROR_INVALID_PARAM;
	}

	motion_config_g = *p_config;

	motion_result_ui32 = Api_motion_twi_init();
	
	if (motion_result_ui32 == MOTION_ERROR_I2C) {
		return MOTION_ERROR_I2C;
	}

	nrf_delay_ms(MOTION_POWERUP_DELAY_MS);

	motion_result_ui32 = motion_read_register(LIS2DH12_WHO_AM_I, &motion_device_id_ui8);
	if (motion_result_ui32 != MOTION_SUCCESS) {
		LOG_API_IMU("Failed to read device ID\r\n");
		return motion_result_ui32;
	}

	if (motion_device_id_ui8 != LIS2DH12_WHO_AM_I_VALUE) {
		LOG_API_IMU("Wrong device ID: 0x%02X, expected: 0x%02X\r\n", motion_device_id_ui8, LIS2DH12_WHO_AM_I_VALUE);
		return MOTION_ERROR_DEVICE_NOT_FOUND;
	}

	LOG_API_IMU("LIS2DH12 device detected, ID: 0x%02X\r\n", motion_device_id_ui8);

	motion_result_ui32 = motion_write_register(LIS2DH12_CTRL_REG5, LIS2DH12_BOOT);
	if (motion_result_ui32 != MOTION_SUCCESS) {
		return motion_result_ui32;
	}
	
	nrf_delay_ms(5);

	motion_reg_value_ui8 = LIS2DH12_BDU | LIS2DH12_HR;
	switch (motion_config_g.full_scale) {
		case MOTION_FS_2G:
			motion_reg_value_ui8 |= LIS2DH12_FS_2G;
			break;
		case MOTION_FS_4G:
			motion_reg_value_ui8 |= LIS2DH12_FS_4G;
			break;
		case MOTION_FS_8G:
			motion_reg_value_ui8 |= LIS2DH12_FS_8G;
			break;
		case MOTION_FS_16G:
			motion_reg_value_ui8 |= LIS2DH12_FS_16G;
			break;
		default:
			motion_reg_value_ui8 |= LIS2DH12_FS_2G;
			break;
	}

	motion_result_ui32 = motion_write_register(LIS2DH12_CTRL_REG4, motion_reg_value_ui8);
	if (motion_result_ui32 != MOTION_SUCCESS) {
		return motion_result_ui32;
	}

	motion_result_ui32 = motion_write_register(LIS2DH12_CTRL_REG2, LIS2DH12_HPIS1);
	if (motion_result_ui32 != MOTION_SUCCESS) {
		return motion_result_ui32;
	}

	motion_result_ui32 = motion_configure_interrupts();
	if (motion_result_ui32 != MOTION_SUCCESS) {
		return motion_result_ui32;
	}

	motion_reg_value_ui8 = 0;
	switch (motion_config_g.output_data_rate) {
		case MOTION_ODR_1HZ:
			motion_reg_value_ui8 = LIS2DH12_ODR_1HZ;
			break;
		case MOTION_ODR_10HZ:
			motion_reg_value_ui8 = LIS2DH12_ODR_10HZ;
			break;
		case MOTION_ODR_25HZ:
			motion_reg_value_ui8 = LIS2DH12_ODR_25HZ;
			break;
		case MOTION_ODR_50HZ:
			motion_reg_value_ui8 = LIS2DH12_ODR_50HZ;
			break;
		case MOTION_ODR_100HZ:
			motion_reg_value_ui8 = LIS2DH12_ODR_100HZ;
			break;
		case MOTION_ODR_200HZ:
			motion_reg_value_ui8 = LIS2DH12_ODR_200HZ;
			break;
		case MOTION_ODR_400HZ:
			motion_reg_value_ui8 = LIS2DH12_ODR_400HZ;
			break;
		default:
			motion_reg_value_ui8 = LIS2DH12_ODR_100HZ;
			break;
	}

	if (motion_config_g.enable_xyz_axes) {
		motion_reg_value_ui8 |= LIS2DH12_XYZ_EN;
	}

	motion_result_ui32 = motion_write_register(LIS2DH12_CTRL_REG1, motion_reg_value_ui8);
	if (motion_result_ui32 != MOTION_SUCCESS) {
		return motion_result_ui32;
	}

	nrf_delay_ms(10);

	if (!nrf_drv_gpiote_is_init()) {
		motion_err_code_ui32 = nrf_drv_gpiote_init();
		if (motion_err_code_ui32 != NRF_SUCCESS) {
			LOG_API_IMU("GPIOTE initialization failed: %d\r\n", motion_err_code_ui32);
			return MOTION_ERROR_I2C;
		}
	}

	const nrf_drv_gpiote_in_config_t motion_in_config_g = {
		.sense = NRF_GPIOTE_POLARITY_LOTOHI,
		.pull = NRF_GPIO_PIN_PULLDOWN,
		.is_watcher = false,
		.hi_accuracy = true
	};

	motion_err_code_ui32 = nrf_drv_gpiote_in_init(P_MOTION_INT, &motion_in_config_g, motion_interrupt_handler);
	if (motion_err_code_ui32 != NRF_SUCCESS) {
		LOG_API_IMU("GPIOTE pin initialization failed: %d\r\n", motion_err_code_ui32);
		return MOTION_ERROR_I2C;
	}

	nrf_drv_gpiote_in_event_enable(P_MOTION_INT, true);

	Api_motion_clear_interrupt();

	motion_initialized_b = true;
	motion_enabled_b = true;

	LOG_API_IMU("LIS2DH12 motion sensor initialized successfully\r\n");
	LOG_API_IMU("Configuration - ODR: %d, Full Scale: %d, Threshold: %d\r\n", 
		motion_config_g.output_data_rate, 
		motion_config_g.full_scale, 
		motion_config_g.motion_threshold);

	return MOTION_SUCCESS;
}

/**
 * @brief Initialize motion sensor with default configuration
 * @details This function initializes the LIS2DH12 motion sensor with
 *          predefined default settings suitable for most applications.
 *          Uses 100Hz ODR, ±2g full scale, and threshold of 8 for good
 *          motion sensitivity while maintaining low power consumption.
 */
motion_error_t Api_motion_init(motion_config_t *motion_config_struct)
{	
	motion_error_t motion_result_ui32 = Api_motion_init_with_config(motion_config_struct);
	if (motion_result_ui32 != MOTION_SUCCESS) {
		LOG_API_IMU("Motion sensor initialization failed with error: %d\r\n", motion_result_ui32);
		return MOTION_ERROR_NOT_INITIALIZED;
	} else {
		LOG_API_IMU("Motion sensor initialized with default settings\r\n");
		
		// Update sensor configuration - motion sensor is now mounted and active
		s_sensor_config.motion_sensor_mounted |= LIS2DH12;
	}

	return MOTION_SUCCESS;
}

/**
 * @brief Deinitialize motion sensor and free resources
 * @return Motion sensor error code
 * @details This function disables GPIO interrupts, uninitializes GPIOTE,
 *          powers down the motion sensor, and releases I2C resources.
 *          Should be called when motion sensor is no longer needed.
 */
motion_error_t Api_motion_deinit(void)
{
	if (!motion_initialized_b) {
		return MOTION_ERROR_NOT_INITIALIZED;
	}

	nrf_drv_gpiote_in_event_disable(P_MOTION_INT);
	nrf_drv_gpiote_in_uninit(P_MOTION_INT);

	motion_write_register(LIS2DH12_CTRL_REG1, LIS2DH12_ODR_POWERDOWN);

	nrf_drv_twi_disable(&motion_twi_instance_p);
	nrf_drv_twi_uninit(&motion_twi_instance_p);

	motion_initialized_b = false;
	motion_enabled_b = false;

	LOG_API_IMU("LIS2DH12 motion sensor deinitialized\r\n");

	return MOTION_SUCCESS;
}

/**
 * @brief Enable motion detection interrupt
 * @return Motion sensor error code
 * @details This function enables the GPIO interrupt for motion detection.
 *          After calling this function, motion events will trigger the
 *          interrupt handler and set the global motion flag.
 */
motion_error_t Api_motion_enable_interrupt(void)
{
	if (!motion_initialized_b) {
		return MOTION_ERROR_NOT_INITIALIZED;
	}

	nrf_drv_gpiote_in_event_enable(P_MOTION_INT, true);
	motion_interrupt_enabled_b = true;
	
	LOG_API_IMU("Motion interrupt enabled\r\n");
	return MOTION_SUCCESS;
}

/**
 * @brief Disable motion detection interrupt
 * @return Motion sensor error code
 * @details This function disables the GPIO interrupt for motion detection.
 *          After calling this function, motion events will not trigger
 *          interrupts, but polling methods can still be used.
 */
motion_error_t Api_motion_disable_interrupt(void)
{
	if (!motion_initialized_b) {
		return MOTION_ERROR_NOT_INITIALIZED;
	}

	nrf_drv_gpiote_in_event_disable(P_MOTION_INT);
	motion_interrupt_enabled_b = false;
	
	LOG_API_IMU("Motion interrupt disabled\r\n");
	return MOTION_SUCCESS;
}

/**
 * @brief Check if motion is detected
 * @return true if motion detected, false otherwise
 * @details This function checks the global motion interrupt flag that is
 *          set by the interrupt handler when motion is detected.
 *          Use motion_clear_interrupt() to clear the flag after handling.
 */
bool Api_motion_is_detected(void)
{
	return motion_interrupt_flag_b;
}

/**
 * @brief Clear motion detection interrupt flag and hardware interrupt
 * @return Motion sensor error code
 * @details This function clears both the global motion interrupt flag and
 *          reads the INT1_SOURCE register to clear the hardware interrupt.
 *          Should be called after handling a motion detection event.
 */
motion_error_t Api_motion_clear_interrupt(void)
{
	motion_error_t motion_result_ui32;
	uint8_t motion_dummy_ui8;

	if (!motion_initialized_b) {
		return MOTION_ERROR_NOT_INITIALIZED;
	}

	motion_interrupt_flag_b = false;

	motion_result_ui32 = motion_read_register(LIS2DH12_INT1_SOURCE, &motion_dummy_ui8);
	if (motion_result_ui32 == MOTION_SUCCESS) {
		LOG_API_IMU("Interrupt cleared, INT1_SOURCE: 0x%02X\r\n", motion_dummy_ui8);
	}

	return motion_result_ui32;
}

/**
 * @brief Enable motion detection and interrupt handling
 * 
 * This function enables motion detection by configuring the motion interrupt.
 * If motion detection is already enabled, the function returns immediately
 * with success status. Upon successful enable, it clears any pending 
 * interrupts and sets the motion enabled flag.
 * 
 * @return motion_error_t Motion operation result
 * @retval MOTION_SUCCESS Motion detection enabled successfully
 * @retval Other Error code from motion_enable_interrupt()
 */
motion_error_t Api_motion_enable(void)
{
	motion_error_t motion_ret_ui32 = MOTION_SUCCESS;
	
	if (motion_enabled_b == true) {
		return MOTION_SUCCESS;
	}
	
	motion_ret_ui32 = Api_motion_enable_interrupt();
	if (motion_ret_ui32 == MOTION_SUCCESS) {
		Api_motion_clear_interrupt();
		motion_enabled_b = true;
	}
	
	return motion_ret_ui32;
}

/**
 * @brief Disable motion detection and interrupt handling
 * 
 * This function disables motion detection by stopping the motion interrupt.
 * Upon successful disable, it clears the motion enabled flag to indicate
 * that motion detection is no longer active.
 * 
 * @return motion_error_t Motion operation result
 * @retval MOTION_SUCCESS Motion detection disabled successfully
 * @retval Other Error code from motion_disable_interrupt()
 */
motion_error_t Api_motion_disable(void)
{
	motion_error_t motion_ret_ui32 = MOTION_SUCCESS;
	
	motion_ret_ui32 = Api_motion_disable_interrupt();
	if (motion_ret_ui32 == MOTION_SUCCESS) {
		motion_enabled_b = false;
	}
	
	return motion_ret_ui32;
}


/**
 * @brief Read current acceleration data from sensor
 * @param[out] p_data Pointer to store acceleration data
 * @return Motion sensor error code
 * @details This function reads the current X, Y, Z acceleration values from
 *          the LIS2DH12 sensor. The data is automatically scaled based on the
 *          configured full scale range and converted to 12-bit resolution.
 */
motion_error_t Api_motion_read_data(motion_data_t *p_data)
{
	motion_error_t motion_result_ui32;
	uint8_t motion_raw_data_aui8[6];
	uint8_t motion_status_ui8;

	if (!motion_initialized_b || p_data == NULL) {
		return MOTION_ERROR_INVALID_PARAM;
	}

	motion_result_ui32 = motion_read_register(LIS2DH12_STATUS_REG, &motion_status_ui8);
	if (motion_result_ui32 != MOTION_SUCCESS) {
		return motion_result_ui32;
	}

	if ((motion_status_ui8 & 0x08) == 0) {
		LOG_API_IMU("No new data available, status: 0x%02X\r\n", motion_status_ui8);
	}

	motion_result_ui32 = motion_read_multiple_registers(LIS2DH12_OUT_X_L, motion_raw_data_aui8, 6);
	if (motion_result_ui32 != MOTION_SUCCESS) {
		return motion_result_ui32;
	}

	p_data->x_axis = (int16_t)((motion_raw_data_aui8[1] << 8) | motion_raw_data_aui8[0]);
	p_data->y_axis = (int16_t)((motion_raw_data_aui8[3] << 8) | motion_raw_data_aui8[2]);
	p_data->z_axis = (int16_t)((motion_raw_data_aui8[5] << 8) | motion_raw_data_aui8[4]);

	p_data->x_axis >>= 4;
	p_data->y_axis >>= 4;
	p_data->z_axis >>= 4;

	LOG_API_IMU("Accel data: X=%d, Y=%d, Z=%d\r\n", p_data->x_axis, p_data->y_axis, p_data->z_axis);

	return MOTION_SUCCESS;
}

/**
 * @brief Set motion detection threshold
 * @param[in] threshold Threshold value (0-127, depends on full scale)
 * @return Motion sensor error code
 * @details This function sets the motion detection threshold in the INT1_THS register.
 *          Higher values make the sensor less sensitive to motion.
 *          The actual threshold depends on the configured full scale range.
 */
motion_error_t Api_motion_set_threshold(uint8_t threshold)
{
	motion_error_t motion_result_ui32;

	if (!motion_initialized_b) {
		return MOTION_ERROR_NOT_INITIALIZED;
	}

	motion_config_g.motion_threshold = threshold;
	motion_result_ui32 = motion_write_register(LIS2DH12_INT1_THS, threshold);
	
	if (motion_result_ui32 == MOTION_SUCCESS) {
		LOG_API_IMU("Motion threshold set to: %d\r\n", threshold);
	}

	return motion_result_ui32;
}

/**
 * @brief Get current motion detection threshold
 * @param[out] p_threshold Pointer to store threshold value
 * @return Motion sensor error code
 * @details This function reads the current motion detection threshold
 *          from the INT1_THS register and stores it in the provided pointer.
 */
motion_error_t Api_motion_get_threshold(uint8_t *p_threshold)
{
	motion_error_t motion_result_ui32;

	if (!motion_initialized_b || p_threshold == NULL) {
		return MOTION_ERROR_INVALID_PARAM;
	}

	motion_result_ui32 = motion_read_register(LIS2DH12_INT1_THS, p_threshold);
	
	return motion_result_ui32;
}

/**
 * @brief Power down motion sensor to save power
 * @return Motion sensor error code
 * @details This function puts the LIS2DH12 sensor into power-down mode
 *          by setting the ODR to 0 in CTRL_REG1. This significantly
 *          reduces power consumption but disables motion detection.
 */
motion_error_t Api_motion_power_down(void)
{
	motion_error_t motion_result_ui32;

	if (!motion_initialized_b) {
		return MOTION_ERROR_NOT_INITIALIZED;
	}

	motion_result_ui32 = motion_write_register(LIS2DH12_CTRL_REG1, LIS2DH12_ODR_POWERDOWN);
	
	if (motion_result_ui32 == MOTION_SUCCESS) {
		LOG_API_IMU("Motion sensor powered down\r\n");
	}

	return motion_result_ui32;
}

/**
 * @brief Power up motion sensor from power-down mode
 * @return Motion sensor error code
 * @details This function restores the LIS2DH12 sensor from power-down mode
 *          by restoring the previously configured ODR and enabling the
 *          configured axes. Motion detection will resume after power-up.
 */
motion_error_t Api_motion_power_up(void)
{
	motion_error_t motion_result_ui32;
	uint8_t motion_reg_value_ui8;

	if (!motion_initialized_b) {
		return MOTION_ERROR_NOT_INITIALIZED;
	}

	motion_reg_value_ui8 = 0;
	switch (motion_config_g.output_data_rate) {
		case MOTION_ODR_1HZ:
			motion_reg_value_ui8 = LIS2DH12_ODR_1HZ;
			break;
		case MOTION_ODR_10HZ:
			motion_reg_value_ui8 = LIS2DH12_ODR_10HZ;
			break;
		case MOTION_ODR_25HZ:
			motion_reg_value_ui8 = LIS2DH12_ODR_25HZ;
			break;
		case MOTION_ODR_50HZ:
			motion_reg_value_ui8 = LIS2DH12_ODR_50HZ;
			break;
		case MOTION_ODR_100HZ:
			motion_reg_value_ui8 = LIS2DH12_ODR_100HZ;
			break;
		case MOTION_ODR_200HZ:
			motion_reg_value_ui8 = LIS2DH12_ODR_200HZ;
			break;
		case MOTION_ODR_400HZ:
			motion_reg_value_ui8 = LIS2DH12_ODR_400HZ;
			break;
		default:
			motion_reg_value_ui8 = LIS2DH12_ODR_100HZ;
			break;
	}

	if (motion_config_g.enable_xyz_axes) {
		motion_reg_value_ui8 |= LIS2DH12_XYZ_EN;
	}

	motion_result_ui32 = motion_write_register(LIS2DH12_CTRL_REG1, motion_reg_value_ui8);
	
	if (motion_result_ui32 == MOTION_SUCCESS) {
		LOG_API_IMU("Motion sensor powered up\r\n");
		nrf_delay_ms(10);
	}

	return motion_result_ui32;
}

/**
 * @brief Get device identification value
 * @param[out] p_device_id Pointer to store device ID (should be 0x33)
 * @return Motion sensor error code
 * @details This function reads the WHO_AM_I register to verify the device
 *          identity. For LIS2DH12, this should return 0x33. Can be used
 *          to verify proper I2C communication and device presence.
 */
motion_error_t Api_motion_get_device_id(uint8_t *p_device_id)
{
	motion_error_t motion_result_ui32;

	if (!motion_initialized_b || p_device_id == NULL) {
		return MOTION_ERROR_INVALID_PARAM;
	}

	motion_result_ui32 = motion_read_register(LIS2DH12_WHO_AM_I, p_device_id);
	
	return motion_result_ui32;
}

/**
 * @brief Poll INT1_SOURCE register to check motion detection status
 * @param[out] p_int_source Pointer to store INT1_SOURCE register value
 * @return Motion sensor error code
 * @details This function reads the INT1_SOURCE register which contains
 *          motion detection status bits. Useful for polling-based motion detection
 *          when interrupts are not working properly.
 */
motion_error_t Api_motion_poll_interrupt_source(uint8_t *p_int_source)
{
	motion_error_t motion_result_ui32;

	if (!motion_initialized_b || p_int_source == NULL) {
		return MOTION_ERROR_INVALID_PARAM;
	}

	motion_result_ui32 = motion_read_register(LIS2DH12_INT1_SOURCE, p_int_source);
	
	if (motion_result_ui32 == MOTION_SUCCESS) {
		LOG_API_IMU("INT1_SOURCE: 0x%02X\r\n", *p_int_source);
	}
	
	return motion_result_ui32;
}

/**
 * @brief Check if motion is detected using polling method
 * @return true if motion detected via polling, false otherwise
 * @details This function polls the INT1_SOURCE register and checks the IA bit
 *          to determine if motion has been detected. This is an alternative
 *          to interrupt-based motion detection for testing purposes.
 */
bool Api_motion_check_motion_by_polling(void)
{
	uint8_t motion_int_source_ui8;
	motion_error_t motion_result_ui32;

	if (!motion_initialized_b) {
		return false;
	}

	motion_result_ui32 = motion_read_register(LIS2DH12_INT1_SOURCE, &motion_int_source_ui8);
	if (motion_result_ui32 != MOTION_SUCCESS) {
		return false;
	}

	// Check if any axis triggered motion detection
	// IA (bit 6): Interrupt active
	// ZH/ZL (bit 5/4): Z high/low event
	// YH/YL (bit 3/2): Y high/low event 
	// XH/XL (bit 1/0): X high/low event
	if (motion_int_source_ui8 & 0x40) {  // IA bit check
		LOG_API_IMU("Motion detected by polling - INT1_SOURCE: 0x%02X\r\n", motion_int_source_ui8);
		return true;
	}
	
	return false;
}

/**
 * @brief Poll STATUS register to check sensor status
 * @param[out] p_status Pointer to store STATUS register value
 * @return Motion sensor error code
 * @details This function reads the STATUS register which contains information
 *          about data availability and sensor status.
 */
motion_error_t Api_motion_poll_status_register(uint8_t *p_status)
{
	motion_error_t motion_result_ui32;

	if (!motion_initialized_b || p_status == NULL) {
		return MOTION_ERROR_INVALID_PARAM;
	}

	motion_result_ui32 = motion_read_register(LIS2DH12_STATUS_REG, p_status);
	
	if (motion_result_ui32 == MOTION_SUCCESS) {
		LOG_API_IMU("STATUS_REG: 0x%02X\r\n", *p_status);
	}
	
	return motion_result_ui32;
}

/**
 * @brief Check if new acceleration data is available
 * @return true if new data is available, false otherwise
 * @details This function polls the STATUS register and checks the ZYXDA bit
 *          to determine if new acceleration data is ready to be read.
 */
bool Api_motion_check_new_data_available(void)
{
	uint8_t motion_status_ui8;
	motion_error_t motion_result_ui32;

	if (!motion_initialized_b) {
		return false;
	}

	motion_result_ui32 = motion_read_register(LIS2DH12_STATUS_REG, &motion_status_ui8);
	if (motion_result_ui32 != MOTION_SUCCESS) {
		return false;
	}

	// ZYXDA bit (bit 3): new data available for X, Y, Z
	return (motion_status_ui8 & 0x08) != 0;
}

