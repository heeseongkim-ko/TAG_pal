
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

#define MOTION_I2C_TIMEOUT_MS		100

static const nrf_drv_twi_t motion_twi_instance_p = NRF_DRV_TWI_INSTANCE(1);
static motion_config_t motion_config_g;
static bool motion_initialized_b = false;
static bool motion_enabled_b = false;
static bool motion_interrupt_enabled_b = false;
extern sensor_config_t s_sensor_config;

volatile bool motion_interrupt_flag_b = false;

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

motion_error_t Api_motion_twi_unInit(void)
{
	nrf_drv_twi_disable(&motion_twi_instance_p);
	nrf_drv_twi_uninit(&motion_twi_instance_p);
	
	return MOTION_SUCCESS;
}

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

	motion_result_ui32 = motion_write_register(LIS2DH12_INT1_THS, motion_config_g.motion_threshold);	
	if (motion_result_ui32 != MOTION_SUCCESS) {
		return motion_result_ui32;
	}

	nrf_delay_ms(10);

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

bool Api_motion_is_detected(void)
{
	return motion_interrupt_flag_b;
}

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

motion_error_t Api_motion_disable(void)
{
	motion_error_t motion_ret_ui32 = MOTION_SUCCESS;
	
	motion_ret_ui32 = Api_motion_disable_interrupt();
	if (motion_ret_ui32 == MOTION_SUCCESS) {
		motion_enabled_b = false;
	}
	
	return motion_ret_ui32;
}

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

motion_error_t Api_motion_set_full_scale(uint8_t full_scale)
{
	motion_error_t motion_result_ui32;
	uint8_t l_reg_value_ui8;

	if (!motion_initialized_b) {
		return MOTION_ERROR_NOT_INITIALIZED;
	}
	// Set register value with BDU and HR bits, then add full scale
	l_reg_value_ui8 = LIS2DH12_BDU | LIS2DH12_HR;
	switch (full_scale) {
		case MOTION_FS_2G:
			l_reg_value_ui8 |= LIS2DH12_FS_2G;
			break;
		case MOTION_FS_4G:
			l_reg_value_ui8 |= LIS2DH12_FS_4G;
			break;
		case MOTION_FS_8G:
			l_reg_value_ui8 |= LIS2DH12_FS_8G;
			break;
		case MOTION_FS_16G:
			l_reg_value_ui8 |= LIS2DH12_FS_16G;
			break;
		default:
			l_reg_value_ui8 |= LIS2DH12_FS_2G;
			break;
	}

	// Write new value to register
	motion_result_ui32 = motion_write_register(LIS2DH12_CTRL_REG4, l_reg_value_ui8);
	
	if (motion_result_ui32 == MOTION_SUCCESS) {
		LOG_API_IMU("Motion full scale set to: %d\r\n", full_scale);
	}

	return motion_result_ui32;
}

motion_error_t Api_motion_get_threshold(uint8_t *p_threshold)
{
	motion_error_t motion_result_ui32;

	if (!motion_initialized_b || p_threshold == NULL) {
		return MOTION_ERROR_INVALID_PARAM;
	}

	motion_result_ui32 = motion_read_register(LIS2DH12_INT1_THS, p_threshold);
	
	return motion_result_ui32;
}

motion_error_t Api_motion_get_device_id(uint8_t *p_device_id)
{
	motion_error_t motion_result_ui32;

	if (!motion_initialized_b || p_device_id == NULL) {
		return MOTION_ERROR_INVALID_PARAM;
	}

	motion_result_ui32 = motion_read_register(LIS2DH12_WHO_AM_I, p_device_id);
	
	return motion_result_ui32;
}

bool Api_motion_check_motion_by_polling(void)
{
	uint8_t motion_int_source_ui8;
	motion_error_t motion_result_ui32;

	if (!motion_initialized_b) {
		return false;
	}
	
	if (nrf_gpio_pin_read(P_MOTION_INT) == 0)
	{
		return false;
	}

	motion_result_ui32 = motion_read_register(LIS2DH12_INT1_SOURCE, &motion_int_source_ui8);
	if (motion_result_ui32 != MOTION_SUCCESS) {
		return false;
	}
	
	if (motion_int_source_ui8 & 0x40) {  // IA bit check
		LOG_API_IMU("Motion detected by polling - INT1_SOURCE: 0x%02X\r\n", motion_int_source_ui8);
		return true;
	}
	
	return false;
}

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

