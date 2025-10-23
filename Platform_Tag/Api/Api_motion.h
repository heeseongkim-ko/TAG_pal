/**
 * @file Api_motion.h
 * @brief LIS2DH12 motion sensor API header file
 * @details This file contains the API functions and definitions for interfacing
 *          with the LIS2DH12 3-axis MEMS accelerometer. It provides motion detection,
 *          acceleration data reading, and interrupt handling capabilities.
 * @author Tag Platform Team
 * @version 1.0
 * @date 2025-01-01
 */

#ifndef API_MOTION_H
#define API_MOTION_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

//=============================================================================
// CONFIGURATION DEFINES
//=============================================================================

/**
 * @brief Enable/disable motion sensor module
 * Set to 1 to enable motion sensor functionality, 0 to disable
 */
#ifndef MOTION_ENABLED
#define MOTION_ENABLED					1
#endif

/**
 * @brief Motion sensor I2C instance
 * Select which I2C instance to use for motion sensor communication
 */
#ifndef MOTION_I2C_INSTANCE
#define MOTION_I2C_INSTANCE				0
#endif

/**
 * @brief Default motion sensor I2C address
 * LIS2DH12_I2C_ADDR_LOW (0x18) if SDO/SA0 tied to GND
 * LIS2DH12_I2C_ADDR_HIGH (0x19) if SDO/SA0 tied to VDD
 */
#ifndef MOTION_DEFAULT_I2C_ADDR
#define MOTION_DEFAULT_I2C_ADDR			0x19
#endif

/**
 * @brief Default motion detection threshold
 * Range: 0-127, depends on full scale setting
 * Higher value = less sensitive
 */
#ifndef MOTION_DEFAULT_THRESHOLD
#define MOTION_DEFAULT_THRESHOLD		0x08	// Reduced for better sensitivity
#endif

/**
 * @brief Default motion detection duration
 * Range: 0-127, in ODR periods
 * Higher value = motion must persist longer
 */
#ifndef MOTION_DEFAULT_DURATION
#define MOTION_DEFAULT_DURATION			0x00	// No duration filter
#endif

/**
 * @brief Default output data rate
 * Available options:
 * - MOTION_ODR_POWERDOWN
 * - MOTION_ODR_1HZ
 * - MOTION_ODR_10HZ
 * - MOTION_ODR_25HZ
 * - MOTION_ODR_50HZ
 * - MOTION_ODR_100HZ
 * - MOTION_ODR_200HZ
 * - MOTION_ODR_400HZ
 */
#ifndef MOTION_DEFAULT_ODR
#define MOTION_DEFAULT_ODR				MOTION_ODR_1HZ//MOTION_ODR_100HZ
#endif

/**
 * @brief Default full scale range
 * Available options:
 * - MOTION_FS_2G
 * - MOTION_FS_4G
 * - MOTION_FS_8G
 * - MOTION_FS_16G
 */
#ifndef MOTION_DEFAULT_FULL_SCALE
#define MOTION_DEFAULT_FULL_SCALE		MOTION_FS_2G
#endif

/**
 * @brief Enable all axes by default
 */
#ifndef MOTION_DEFAULT_ENABLE_XYZ
#define MOTION_DEFAULT_ENABLE_XYZ		true
#endif

/**
 * @brief Motion sensor interrupt priority
 */
#ifndef MOTION_INTERRUPT_PRIORITY
#define MOTION_INTERRUPT_PRIORITY		APP_IRQ_PRIORITY_HIGH
#endif

/**
 * @brief I2C communication timeout in milliseconds
 */
#ifndef MOTION_I2C_TIMEOUT_MS
#define MOTION_I2C_TIMEOUT_MS			100
#endif

/**
 * @brief Enable motion sensor debugging
 */
#ifndef MOTION_DEBUG_ENABLED
#define MOTION_DEBUG_ENABLED			1
#endif

/**
 * @brief Motion sensor power-up delay in milliseconds
 */
#ifndef MOTION_POWERUP_DELAY_MS
#define MOTION_POWERUP_DELAY_MS			20	// Increased delay
#endif

//=============================================================================
// HARDWARE REGISTER DEFINES
//=============================================================================

/* LIS2DH12 I2C addresses */
#define LIS2DH12_I2C_ADDR_LOW		0x18	/**< SDO/SA0 tied to GND */
#define LIS2DH12_I2C_ADDR_HIGH		0x19	/**< SDO/SA0 tied to VDD */

/* LIS2DH12 Register addresses */
#define LIS2DH12_WHO_AM_I			0x0F	/**< Device identification register */
#define LIS2DH12_CTRL_REG1			0x20	/**< Control register 1 */
#define LIS2DH12_CTRL_REG2			0x21	/**< Control register 2 */
#define LIS2DH12_CTRL_REG3			0x22	/**< Control register 3 */
#define LIS2DH12_CTRL_REG4			0x23	/**< Control register 4 */
#define LIS2DH12_CTRL_REG5			0x24	/**< Control register 5 */
#define LIS2DH12_CTRL_REG6			0x25	/**< Control register 6 */
#define LIS2DH12_STATUS_REG			0x27	/**< Status register */
#define LIS2DH12_OUT_X_L			0x28	/**< X-axis LSB */
#define LIS2DH12_OUT_X_H			0x29	/**< X-axis MSB */
#define LIS2DH12_OUT_Y_L			0x2A	/**< Y-axis LSB */
#define LIS2DH12_OUT_Y_H			0x2B	/**< Y-axis MSB */
#define LIS2DH12_OUT_Z_L			0x2C	/**< Z-axis LSB */
#define LIS2DH12_OUT_Z_H			0x2D	/**< Z-axis MSB */
#define LIS2DH12_INT1_CFG			0x30	/**< Interrupt 1 configuration */
#define LIS2DH12_INT1_SOURCE		0x31	/**< Interrupt 1 source */
#define LIS2DH12_INT1_THS			0x32	/**< Interrupt 1 threshold */
#define LIS2DH12_INT1_DURATION		0x33	/**< Interrupt 1 duration */

/* WHO_AM_I value */
#define LIS2DH12_WHO_AM_I_VALUE		0x33	/**< Expected device ID */

/* CTRL_REG1 bit masks */
#define LIS2DH12_ODR_POWERDOWN		0x00	/**< Power-down mode */
#define LIS2DH12_ODR_1HZ			0x10	/**< 1 Hz output data rate */
#define LIS2DH12_ODR_10HZ			0x20	/**< 10 Hz output data rate */
#define LIS2DH12_ODR_25HZ			0x30	/**< 25 Hz output data rate */
#define LIS2DH12_ODR_50HZ			0x40	/**< 50 Hz output data rate */
#define LIS2DH12_ODR_100HZ			0x50	/**< 100 Hz output data rate */
#define LIS2DH12_ODR_200HZ			0x60	/**< 200 Hz output data rate */
#define LIS2DH12_ODR_400HZ			0x70	/**< 400 Hz output data rate */
#define LIS2DH12_XYZ_EN				0x07	/**< Enable X, Y, Z axes */

/* CTRL_REG2 bit masks */
#define LIS2DH12_HPM_NORMAL			0x00	/**< Normal mode (reset reading) */
#define LIS2DH12_HPM_REF			0x40	/**< Reference signal for filtering */
#define LIS2DH12_HPCF_MASK			0x30	/**< High-pass filter cutoff frequency */
#define LIS2DH12_FDS				0x08	/**< Filtered data selection */
#define LIS2DH12_HPCLICK			0x04	/**< High-pass filter enabled for click function */
#define LIS2DH12_HPIS2				0x02	/**< High-pass filter enabled for interrupt 2 */
#define LIS2DH12_HPIS1				0x01	/**< High-pass filter enabled for interrupt 1 */

/* CTRL_REG3 bit masks */
#define LIS2DH12_I1_CLICK			0x80	/**< CLICK interrupt on INT1 */
#define LIS2DH12_I1_AOI1			0x40	/**< AOI1 interrupt on INT1 */
#define LIS2DH12_I1_AOI2			0x20	/**< AOI2 interrupt on INT1 */
#define LIS2DH12_I1_DRDY1			0x10	/**< Data ready 1 on INT1 */
#define LIS2DH12_I1_DRDY2			0x08	/**< Data ready 2 on INT1 */
#define LIS2DH12_I1_WTM				0x04	/**< FIFO watermark on INT1 */
#define LIS2DH12_I1_OVERRUN			0x02	/**< FIFO overrun on INT1 */

/* CTRL_REG4 bit masks */
#define LIS2DH12_BDU				0x80	/**< Block data update */
#define LIS2DH12_BLE				0x40	/**< Big/little endian data selection */
#define LIS2DH12_FS_2G				0x00	/**< ±2g full scale */
#define LIS2DH12_FS_4G				0x10	/**< ±4g full scale */
#define LIS2DH12_FS_8G				0x20	/**< ±8g full scale */
#define LIS2DH12_FS_16G				0x30	/**< ±16g full scale */
#define LIS2DH12_HR					0x08	/**< High resolution enable */
#define LIS2DH12_ST					0x06	/**< Self-test enable */

/* CTRL_REG5 bit masks */
#define LIS2DH12_BOOT				0x80	/**< Reboot memory content */
#define LIS2DH12_FIFO_EN			0x40	/**< FIFO enable */
#define LIS2DH12_LIR_INT1			0x08	/**< Latch interrupt request on INT1 */
#define LIS2DH12_D4D_INT1			0x04	/**< 4D enable on INT1 */

/* CTRL_REG6 bit masks */
#define LIS2DH12_I2_CLICK			0x80	/**< CLICK interrupt on INT2 */
#define LIS2DH12_I2_INT1			0x40	/**< INT1 function on INT2 */
#define LIS2DH12_BOOT_I2			0x10	/**< Boot on INT2 */
#define LIS2DH12_H_LACTIVE			0x02	/**< Interrupt active high/low */

/* INT1_CFG bit masks */
#define LIS2DH12_AOI				0x80	/**< And/Or combination of interrupt events */
#define LIS2DH12_6D					0x40	/**< 6-direction detection function enabled */
#define LIS2DH12_ZHIE				0x20	/**< Enable interrupt on Z-axis high event */
#define LIS2DH12_ZLIE				0x10	/**< Enable interrupt on Z-axis low event */
#define LIS2DH12_YHIE				0x08	/**< Enable interrupt on Y-axis high event */
#define LIS2DH12_YLIE				0x04	/**< Enable interrupt on Y-axis low event */
#define LIS2DH12_XHIE				0x02	/**< Enable interrupt on X-axis high event */
#define LIS2DH12_XLIE				0x01	/**< Enable interrupt on X-axis low event */

//=============================================================================
// ENUMERATIONS
//=============================================================================

/**
 * @brief Motion sensor error codes
 */
typedef enum {
	MOTION_SUCCESS = 0,				/**< Operation completed successfully */
	MOTION_ERROR_I2C,				/**< I2C communication error */
	MOTION_ERROR_DEVICE_NOT_FOUND,	/**< Device not found or wrong device ID */
	MOTION_ERROR_INVALID_PARAM,		/**< Invalid parameter passed to function */
	MOTION_ERROR_TIMEOUT,			/**< Communication timeout */
	MOTION_ERROR_NOT_INITIALIZED	/**< Device not initialized */
} motion_error_t;

/**
 * @brief Motion sensor output data rate
 */
typedef enum {
	MOTION_ODR_POWERDOWN = 0,		/**< Power-down mode */
	MOTION_ODR_1HZ,					/**< 1 Hz output data rate */
	MOTION_ODR_10HZ,				/**< 10 Hz output data rate */
	MOTION_ODR_25HZ,				/**< 25 Hz output data rate */
	MOTION_ODR_50HZ,				/**< 50 Hz output data rate */
	MOTION_ODR_100HZ,				/**< 100 Hz output data rate */
	MOTION_ODR_200HZ,				/**< 200 Hz output data rate */
	MOTION_ODR_400HZ				/**< 400 Hz output data rate */
} motion_odr_t;

/**
 * @brief Motion sensor full scale range
 */
typedef enum {
	MOTION_FS_2G = 0,				/**< ±2g full scale range */
	MOTION_FS_4G,					/**< ±4g full scale range */
	MOTION_FS_8G,					/**< ±8g full scale range */
	MOTION_FS_16G					/**< ±16g full scale range */
} motion_full_scale_t;

//=============================================================================
// STRUCTURES
//=============================================================================

/**
 * @brief Motion sensor configuration structure
 */
typedef struct {
	uint8_t i2c_address;			/**< I2C slave address (0x18 or 0x19) */
	motion_odr_t output_data_rate;	/**< Output data rate setting */
	motion_full_scale_t full_scale;	/**< Full scale range setting */
	uint8_t motion_threshold;		/**< Motion detection threshold (0-127) */
	uint8_t motion_duration;		/**< Motion detection duration (0-127) */
	bool enable_xyz_axes;			/**< Enable X, Y, Z axes */
} motion_config_t;

/**
 * @brief Motion sensor data structure
 */
typedef struct {
	int16_t x_axis;					/**< X-axis acceleration data */
	int16_t y_axis;					/**< Y-axis acceleration data */
	int16_t z_axis;					/**< Z-axis acceleration data */
} motion_data_t;


//=============================================================================
// FUNCTION PROTOTYPES
//=============================================================================
// Header
/**
 * @brief Initialize TWI for motion sensor
 * @return Motion sensor error code
 */
motion_error_t Api_motion_twi_init(void);

/**
 * @brief Uninitialize TWI for motion sensor (for sleep mode)
 * @return Motion sensor error code
 */
motion_error_t Api_motion_twi_unInit(void);

/**
 * @brief Initialize motion sensor with configuration
 * @param[in] p_config Pointer to motion sensor configuration structure
 * @return Motion sensor error code
 */
motion_error_t Api_motion_init_with_config(const motion_config_t *p_config);

/**
 * @brief Deinitialize motion sensor and free resources
 * @return Motion sensor error code
 */
motion_error_t Api_motion_deinit(void);

/**
 * @brief Enable motion detection interrupt
 * @return Motion sensor error code
 */
motion_error_t Api_motion_enable_interrupt(void);

/**
 * @brief Disable motion detection interrupt
 * @return Motion sensor error code
 */
motion_error_t Api_motion_disable_interrupt(void);

/**
 * @brief Check if motion is detected
 * @return true if motion detected, false otherwise
 */
bool Api_motion_is_detected(void);

/**
 * @brief Clear motion detection interrupt flag and hardware interrupt
 * @return Motion sensor error code
 */
motion_error_t Api_motion_clear_interrupt(void);

/**
 * @brief Read current acceleration data from sensor
 * @param[out] p_data Pointer to store acceleration data
 * @return Motion sensor error code
 */
motion_error_t Api_motion_read_data(motion_data_t *p_data);

/**
 * @brief Set motion detection threshold
 * @param[in] threshold Threshold value (0-127, depends on full scale)
 * @return Motion sensor error code
 */
motion_error_t Api_motion_set_threshold(uint8_t threshold);

/**
 * @brief Get current motion detection threshold
 * @param[out] p_threshold Pointer to store threshold value
 * @return Motion sensor error code
 */
motion_error_t Api_motion_get_threshold(uint8_t *p_threshold);

/**
 * @brief Power down motion sensor to save power
 * @return Motion sensor error code
 */
motion_error_t Api_motion_power_down(void);

/**
 * @brief Power up motion sensor from power-down mode
 * @return Motion sensor error code
 */
motion_error_t Api_motion_power_up(void);

/**
 * @brief Get device identification value
 * @param[out] p_device_id Pointer to store device ID (should be 0x33)
 * @return Motion sensor error code
 */
motion_error_t Api_motion_get_device_id(uint8_t *p_device_id);

/**
 * @brief Initialize motion sensor with default configuration
 * @details This function initializes the LIS2DH12 motion sensor with
 *          predefined default settings suitable for most applications.
 *          Uses 100Hz ODR, ±2g full scale, and threshold of 8 for good
 *          motion sensitivity while maintaining low power consumption.
 */
motion_error_t Api_motion_init(motion_config_t *motion_config_struct);

/**
 * @brief Poll INT1_SOURCE register to check motion detection status
 * @param[out] p_int_source Pointer to store INT1_SOURCE register value
 * @return Motion sensor error code
 * @details This function reads the INT1_SOURCE register which contains
 *          motion detection status bits. Useful for polling-based motion detection
 *          when interrupts are not working properly.
 */
motion_error_t Api_motion_poll_interrupt_source(uint8_t *p_int_source);

/**
 * @brief Check if motion is detected using polling method
 * @return true if motion detected via polling, false otherwise
 * @details This function polls the INT1_SOURCE register and checks the IA bit
 *          to determine if motion has been detected. This is an alternative
 *          to interrupt-based motion detection for testing purposes.
 */
bool Api_motion_check_motion_by_polling(void);

/**
 * @brief Poll STATUS register to check sensor status
 * @param[out] p_status Pointer to store STATUS register value
 * @return Motion sensor error code
 * @details This function reads the STATUS register which contains information
 *          about data availability and sensor status.
 */
motion_error_t Api_motion_poll_status_register(uint8_t *p_status);

/**
 * @brief Check if new acceleration data is available
 * @return true if new data is available, false otherwise
 * @details This function polls the STATUS register and checks the ZYXDA bit
 *          to determine if new acceleration data is ready to be read.
 */
bool Api_motion_check_new_data_available(void);

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
motion_error_t Api_motion_enable(void);

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
motion_error_t Api_motion_disable(void);

#ifdef __cplusplus
}
#endif

#endif // API_MOTION_H