/**
 * @file    Drv_uwb_spi.c
 * @brief   UWB SPI Driver Implementation
 * 
 * @details Provides SPI initialization and de-initialization for UWB module.
 *          Moved from deca_spi.c for proper layer separation.
 */

#include "Drv_uwb_internal.h"
#include "Drv_uwb_spi.h"
#include "deca_spi.h"
#include "port.h"
#include <nrf_delay.h>
#include <nrf_drv_gpiote.h>
#include <nrf_drv_spi.h>
#include "Api_failsafe.h"

spi_handle_t drv_uwb_spi1_handler;
spi_handle_t *drv_uwb_SpiHandler_p = &drv_uwb_spi1_handler;

dw_t drv_uwb_s1 = 
{
	.irqPin  = DW3000_IRQn_Pin,
	.rstPin  = DW3000_RESET_Pin,
	.wkupPin = DW3000_WAKEUP_Pin,
	.csPin   = DW3000_CS_Pin,
	.pSpi    = &drv_uwb_spi1_handler,
};

const dw_t *drv_uwb_spi1 = &drv_uwb_s1;

/**
 * @brief Initialize SPI for UWB
 * 
 * @details Configures nRF52840 SPI peripheral for DW3000 communication.
 *          
 * @param fastRate true for 32MHz (fast), false for 4MHz (slow)
 */
void Drv_uwb_spi_init(bool fastRate)
{
	uint32_t ret;
	nrf_drv_spi_t *spi_inst;
	nrf_drv_spi_config_t *spi_config;
	spi_handle_t *pSPI1_handler = drv_uwb_spi1->pSpi;

	pSPI1_handler->frequency_slow = NRF_SPIM_FREQ_4M;
	pSPI1_handler->frequency_fast = NRF_SPIM_FREQ_8M;//NRF_SPIM_FREQ_32M;
	pSPI1_handler->lock = DW_HAL_NODE_UNLOCKED;

	spi_inst = &pSPI1_handler->spi_inst;
	spi_config = &pSPI1_handler->spi_config;

	spi_inst->inst_idx = SPI2_INSTANCE_INDEX;
	spi_inst->use_easy_dma = SPI2_USE_EASY_DMA;
	spi_inst->u.spim.p_reg = NRF_SPIM2;
	spi_inst->u.spim.drv_inst_idx = NRFX_SPIM2_INST_IDX;

	spi_config->sck_pin = DW3000_CLK_Pin;
	spi_config->mosi_pin = DW3000_MOSI_Pin;
	spi_config->miso_pin = DW3000_MISO_Pin;
	spi_config->ss_pin = NRF_DRV_SPI_PIN_NOT_USED;
	spi_config->irq_priority = (APP_IRQ_PRIORITY_MID - 2);
	spi_config->orc = 0xFF;
	spi_config->frequency = fastRate ? pSPI1_handler->frequency_fast : pSPI1_handler->frequency_slow;
	spi_config->mode = NRF_DRV_SPI_MODE_0;
	spi_config->bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;

	nrf_drv_gpiote_out_config_t out_config = NRFX_GPIOTE_CONFIG_OUT_TASK_TOGGLE(NRF_GPIOTE_INITIAL_VALUE_HIGH);
	nrf_drv_gpiote_out_init(DW3000_CS_Pin, &out_config);
#if  0
	APP_ERROR_CHECK(nrf_drv_spi_init(spi_inst, spi_config, NULL, NULL));
#else
	ret = nrf_drv_spi_init(spi_inst, spi_config, NULL, NULL);
	if  (ret == NRF_SUCCESS)
	{		
		Api_failsafe_set_uwb_fail(UWB_FAIL_NONE);
	}
	else
	{
		Api_failsafe_set_uwb_fail(UWB_FAIL_SPI_03);
	}
#endif

	nrf_gpio_cfg(spi_config->sck_pin,
		NRF_GPIO_PIN_DIR_OUTPUT,
		NRF_GPIO_PIN_INPUT_CONNECT,
		NRF_GPIO_PIN_NOPULL,
		NRF_GPIO_PIN_H0H1,
		NRF_GPIO_PIN_NOSENSE);
	nrf_gpio_cfg(spi_config->mosi_pin,
		NRF_GPIO_PIN_DIR_OUTPUT,
		NRF_GPIO_PIN_INPUT_DISCONNECT,
		NRF_GPIO_PIN_NOPULL,
		NRF_GPIO_PIN_H0H1,
		NRF_GPIO_PIN_NOSENSE);

	update_spi_handler(drv_uwb_SpiHandler_p, drv_uwb_spi1->csPin, drv_uwb_spi1->irqPin);
}

/**
 * @brief Uninitialize SPI for UWB
 * 
 * @details Disables SPI peripheral and deinitializes CS pin.
 */
void Drv_uwb_spi_uninit(void)
{
	nrf_drv_spi_uninit(&drv_uwb_SpiHandler_p->spi_inst);
}

API_UWB_STATUS_e Drv_uwb_spi_fastrate(void)
{
	uint32_t ret;
	API_UWB_STATUS_e status = API_UWB_STATUS_SUCCESS;
	
	nrf_drv_spi_uninit(&drv_uwb_SpiHandler_p->spi_inst);
	
	drv_uwb_SpiHandler_p->spi_config.frequency = drv_uwb_SpiHandler_p->frequency_fast;

	ret = nrf_drv_spi_init(&drv_uwb_SpiHandler_p->spi_inst, 
	                                  &drv_uwb_SpiHandler_p->spi_config, NULL, NULL);
	if  (ret == NRF_SUCCESS)
	{		
		Api_failsafe_set_uwb_fail(UWB_FAIL_NONE);
	}
	else
	{
		Api_failsafe_set_uwb_fail(UWB_FAIL_SPI_02);
		status = API_UWB_STATUS_FAIL;
	}

	#if  0
	nrf_gpio_cfg(drv_uwb_SpiHandler_p->spi_config.sck_pin,
		NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_CONNECT,
		NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
	nrf_gpio_cfg(drv_uwb_SpiHandler_p->spi_config.mosi_pin,
		NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
		NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
	#endif
	
	LOG_API_UWB("%s\r\n", __func__);
	
	return status;
}

API_UWB_STATUS_e Drv_uwb_spi_slowrate(void)
{
	uint32_t ret;
	API_UWB_STATUS_e status = API_UWB_STATUS_SUCCESS;
	
	nrf_drv_spi_uninit(&drv_uwb_SpiHandler_p->spi_inst);
	
	drv_uwb_SpiHandler_p->spi_config.frequency = drv_uwb_SpiHandler_p->frequency_slow;

	ret = nrf_drv_spi_init(&drv_uwb_SpiHandler_p->spi_inst, 
	                                  &drv_uwb_SpiHandler_p->spi_config, NULL, NULL);
	if  (ret == NRF_SUCCESS)
	{		
		Api_failsafe_set_uwb_fail(UWB_FAIL_NONE);
	}
	else
	{
		Api_failsafe_set_uwb_fail(UWB_FAIL_SPI_01);
		status = API_UWB_STATUS_FAIL;
	}

	LOG_API_UWB("%s\r\n", __func__);

	return status;
}

