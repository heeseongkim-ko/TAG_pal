/**
 * @file    Drv_uwb_spi.h
 * @brief   UWB SPI Driver Interface
 */

#ifndef DRV_UWB_SPI_H
#define DRV_UWB_SPI_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize SPI for UWB
 * @param fastRate true for fast rate, false for slow rate
 */
void Drv_uwb_spi_init(bool fastRate);

/**
 * @brief Uninitialize SPI for UWB
 */
void Drv_uwb_spi_uninit(void);

API_UWB_STATUS_e Drv_uwb_spi_fastrate(void);

API_UWB_STATUS_e Drv_uwb_spi_slowrate(void);

#endif // DRV_UWB_SPI_H
