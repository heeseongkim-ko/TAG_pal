
#ifndef API_PORT_H
#define API_PORT_H

#include <stdint.h>
#include <stdbool.h>
#include "port.h"

#ifdef __cplusplus
extern "C" {
#endif

#define	P_NFC_SCL		NRF_GPIO_PIN_MAP(0,27)
#define	P_NFC_SDA		NRF_GPIO_PIN_MAP(0,31)
#define	P_NFC_FD		NRF_GPIO_PIN_MAP(0,17)
#define	P_NFC_PW		NRF_GPIO_PIN_MAP(0,20)

#define P_LED_RED   	NRF_GPIO_PIN_MAP(0,7)
#define P_LED_GREEN 	NRF_GPIO_PIN_MAP(0,23)

#define P_ADC_RDIV  NRF_GPIO_PIN_MAP(0,30)
#define P_ADC_MEAS  NRF_GPIO_PIN_MAP(0,2)

/**
 * @brief Motion sensor I2C SCL pin configuration
 * Default: P1.04
 */
#ifndef P_MOTION_I2C_SCL
#define P_MOTION_I2C_SCL				NRF_GPIO_PIN_MAP(1,4)
#endif

/**
 * @brief Motion sensor I2C SDA pin configuration
 * Default: P0.24
 */
#ifndef P_MOTION_I2C_SDA
#define P_MOTION_I2C_SDA				NRF_GPIO_PIN_MAP(0,24)
#endif

/**
 * @brief Motion sensor interrupt pin
 * Default: P0.16 as INT1 pin
 */
#ifndef P_MOTION_INT
#define P_MOTION_INT					NRF_GPIO_PIN_MAP(0, 16)
#endif

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* API_PORT_H */

