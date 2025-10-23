/**
 * @file Api_sleep_peripheral.c
 * @brief Peripheral disable implementation for low power sleep mode
 */

#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "nrf_ppi.h"
#include "nrf_timer.h"
#include "nrf_pwm.h"
#include "nrf_saadc.h"
#include "nrf_uart.h"
#include "nrf_uarte.h"
#include "nrf_spi.h"
#include "nrf_spim.h"
#include "nrf_spis.h"
#include "nrf_twi.h"
#include "nrf_twim.h"
#include "nrf_twis.h"
#include "nrf_qdec.h"
#include "nrf_comp.h"
#include "nrf_lpcomp.h"
#include "nrf_wdt.h"
#include "nrf_clock.h"

#include "Api_sleep_peripheral.h"
#include "Api_motion.h"
#include "Api_battery.h"
#include "Api_nfc.h"
#include "Api_uwb.h"

/**
 * @brief Disable all UART/UARTE instances before sleep
 */
void Api_sleep_peripheral_uart_disable_all(void)
{
	// UART0
	if (NRF_UART0->ENABLE != UART_ENABLE_ENABLE_Disabled)
	{
		NRF_UART0->TASKS_STOPRX = 1;
		NRF_UART0->TASKS_STOPTX = 1;
		NRF_UART0->ENABLE = UART_ENABLE_ENABLE_Disabled;
	}

	// UARTE0
	if (NRF_UARTE0->ENABLE != UARTE_ENABLE_ENABLE_Disabled)
	{
		NRF_UARTE0->TASKS_STOPRX = 1;
		NRF_UARTE0->TASKS_STOPTX = 1;
		NRF_UARTE0->ENABLE = UARTE_ENABLE_ENABLE_Disabled;
	}

	// UARTE1 (if available on nRF52840)
#ifdef NRF_UARTE1
	if (NRF_UARTE1->ENABLE != UARTE_ENABLE_ENABLE_Disabled)
	{
		NRF_UARTE1->TASKS_STOPRX = 1;
		NRF_UARTE1->TASKS_STOPTX = 1;
		NRF_UARTE1->ENABLE = UARTE_ENABLE_ENABLE_Disabled;
	}
#endif
}

/**
 * @brief Disable all SPI/SPIM/SPIS instances before sleep
 */
void Api_sleep_peripheral_spi_disable_all(void)
{
	// SPI0
	if (NRF_SPI0->ENABLE != SPI_ENABLE_ENABLE_Disabled)
	{
		NRF_SPI0->ENABLE = SPI_ENABLE_ENABLE_Disabled;
	}

	// SPI1
	if (NRF_SPI1->ENABLE != SPI_ENABLE_ENABLE_Disabled)
	{
		NRF_SPI1->ENABLE = SPI_ENABLE_ENABLE_Disabled;
	}

	// SPI2
	if (NRF_SPI2->ENABLE != SPI_ENABLE_ENABLE_Disabled)
	{
		NRF_SPI2->ENABLE = SPI_ENABLE_ENABLE_Disabled;
	}

	// SPIM0
	if (NRF_SPIM0->ENABLE != SPIM_ENABLE_ENABLE_Disabled)
	{
		NRF_SPIM0->TASKS_STOP = 1;
		NRF_SPIM0->ENABLE = SPIM_ENABLE_ENABLE_Disabled;
	}

	// SPIM1
	if (NRF_SPIM1->ENABLE != SPIM_ENABLE_ENABLE_Disabled)
	{
		NRF_SPIM1->TASKS_STOP = 1;
		NRF_SPIM1->ENABLE = SPIM_ENABLE_ENABLE_Disabled;
	}

	// SPIM2
	if (NRF_SPIM2->ENABLE != SPIM_ENABLE_ENABLE_Disabled)
	{
		NRF_SPIM2->TASKS_STOP = 1;
		NRF_SPIM2->ENABLE = SPIM_ENABLE_ENABLE_Disabled;
	}

	// SPIM3
	if (NRF_SPIM3->ENABLE != SPIM_ENABLE_ENABLE_Disabled)
	{
		NRF_SPIM3->TASKS_STOP = 1;
		NRF_SPIM3->ENABLE = SPIM_ENABLE_ENABLE_Disabled;
	}

	// SPIS0
	if (NRF_SPIS0->ENABLE != SPIS_ENABLE_ENABLE_Disabled)
	{
		NRF_SPIS0->TASKS_RELEASE = 1;
		NRF_SPIS0->ENABLE = SPIS_ENABLE_ENABLE_Disabled;
	}

	// SPIS1
	if (NRF_SPIS1->ENABLE != SPIS_ENABLE_ENABLE_Disabled)
	{
		NRF_SPIS1->TASKS_RELEASE = 1;
		NRF_SPIS1->ENABLE = SPIS_ENABLE_ENABLE_Disabled;
	}

	// SPIS2
	if (NRF_SPIS2->ENABLE != SPIS_ENABLE_ENABLE_Disabled)
	{
		NRF_SPIS2->TASKS_RELEASE = 1;
		NRF_SPIS2->ENABLE = SPIS_ENABLE_ENABLE_Disabled;
	}
}

/**
 * @brief Disable all TWI/TWIM/TWIS (I2C) instances before sleep
 */
void Api_sleep_peripheral_twi_disable_all(void)
{
	// TWI0
	if (NRF_TWI0->ENABLE != TWI_ENABLE_ENABLE_Disabled)
	{
		NRF_TWI0->TASKS_STOP = 1;
		NRF_TWI0->ENABLE = TWI_ENABLE_ENABLE_Disabled;
	}

	// TWI1
	if (NRF_TWI1->ENABLE != TWI_ENABLE_ENABLE_Disabled)
	{
		NRF_TWI1->TASKS_STOP = 1;
		NRF_TWI1->ENABLE = TWI_ENABLE_ENABLE_Disabled;
	}

	// TWIM0
	if (NRF_TWIM0->ENABLE != TWIM_ENABLE_ENABLE_Disabled)
	{
		NRF_TWIM0->TASKS_STOP = 1;
		NRF_TWIM0->ENABLE = TWIM_ENABLE_ENABLE_Disabled;
	}

	// TWIM1
	if (NRF_TWIM1->ENABLE != TWIM_ENABLE_ENABLE_Disabled)
	{
		NRF_TWIM1->TASKS_STOP = 1;
		NRF_TWIM1->ENABLE = TWIM_ENABLE_ENABLE_Disabled;
	}

	// TWIS0
	if (NRF_TWIS0->ENABLE != TWIS_ENABLE_ENABLE_Disabled)
	{
		NRF_TWIS0->TASKS_STOP = 1;
		NRF_TWIS0->ENABLE = TWIS_ENABLE_ENABLE_Disabled;
	}

	// TWIS1
	if (NRF_TWIS1->ENABLE != TWIS_ENABLE_ENABLE_Disabled)
	{
		NRF_TWIS1->TASKS_STOP = 1;
		NRF_TWIS1->ENABLE = TWIS_ENABLE_ENABLE_Disabled;
	}
}

/**
 * @brief Disable all TIMER instances before sleep
 */
void Api_sleep_peripheral_timer_disable_all(void)
{
	// TIMER0
	NRF_TIMER0->TASKS_STOP = 1;
	NRF_TIMER0->TASKS_CLEAR = 1;

	// TIMER1
	NRF_TIMER1->TASKS_STOP = 1;
	NRF_TIMER1->TASKS_CLEAR = 1;

	// TIMER2
	NRF_TIMER2->TASKS_STOP = 1;
	NRF_TIMER2->TASKS_CLEAR = 1;

	// TIMER3
	NRF_TIMER3->TASKS_STOP = 1;
	NRF_TIMER3->TASKS_CLEAR = 1;

	// TIMER4
	NRF_TIMER4->TASKS_STOP = 1;
	NRF_TIMER4->TASKS_CLEAR = 1;
}

/**
 * @brief Disable all PWM instances before sleep
 */
void Api_sleep_peripheral_pwm_disable_all(void)
{
	// PWM0
	if (NRF_PWM0->ENABLE != PWM_ENABLE_ENABLE_Disabled)
	{
		NRF_PWM0->TASKS_STOP = 1;
		NRF_PWM0->ENABLE = PWM_ENABLE_ENABLE_Disabled;
	}

	// PWM1
	if (NRF_PWM1->ENABLE != PWM_ENABLE_ENABLE_Disabled)
	{
		NRF_PWM1->TASKS_STOP = 1;
		NRF_PWM1->ENABLE = PWM_ENABLE_ENABLE_Disabled;
	}

	// PWM2
	if (NRF_PWM2->ENABLE != PWM_ENABLE_ENABLE_Disabled)
	{
		NRF_PWM2->TASKS_STOP = 1;
		NRF_PWM2->ENABLE = PWM_ENABLE_ENABLE_Disabled;
	}

	// PWM3
	if (NRF_PWM3->ENABLE != PWM_ENABLE_ENABLE_Disabled)
	{
		NRF_PWM3->TASKS_STOP = 1;
		NRF_PWM3->ENABLE = PWM_ENABLE_ENABLE_Disabled;
	}
}

/**
 * @brief Disable SAADC before sleep
 */
void Api_sleep_peripheral_saadc_disable(void)
{
	if (NRF_SAADC->ENABLE != SAADC_ENABLE_ENABLE_Disabled)
	{
		NRF_SAADC->TASKS_STOP = 1;
		NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Disabled;
	}
}

/**
 * @brief Disable other peripherals before sleep
 */
void Api_sleep_peripheral_others_disable(void)
{
	// QDEC
	if (NRF_QDEC->ENABLE != QDEC_ENABLE_ENABLE_Disabled)
	{
		NRF_QDEC->TASKS_STOP = 1;
		NRF_QDEC->ENABLE = QDEC_ENABLE_ENABLE_Disabled;
	}

	// COMP
	if (NRF_COMP->ENABLE != COMP_ENABLE_ENABLE_Disabled)
	{
		NRF_COMP->TASKS_STOP = 1;
		NRF_COMP->ENABLE = COMP_ENABLE_ENABLE_Disabled;
	}

	// LPCOMP
	if (NRF_LPCOMP->ENABLE != LPCOMP_ENABLE_ENABLE_Disabled)
	{
		NRF_LPCOMP->TASKS_STOP = 1;
		NRF_LPCOMP->ENABLE = LPCOMP_ENABLE_ENABLE_Disabled;
	}

	// GPIOTE - Clear all channel configurations
	for (uint8_t Api_sleep_peripheral_channel_ui8 = 0; Api_sleep_peripheral_channel_ui8 < GPIOTE_CH_NUM; Api_sleep_peripheral_channel_ui8++)
	{
		NRF_GPIOTE->CONFIG[Api_sleep_peripheral_channel_ui8] = 0;
	}

	// PPI - Disable all channels
	NRF_PPI->CHEN = 0;
	for (uint8_t Api_sleep_peripheral_ppi_ui8 = 0; Api_sleep_peripheral_ppi_ui8 < PPI_CH_NUM; Api_sleep_peripheral_ppi_ui8++)
	{
		NRF_PPI->CH[Api_sleep_peripheral_ppi_ui8].EEP = 0;
		NRF_PPI->CH[Api_sleep_peripheral_ppi_ui8].TEP = 0;
	}
}

/**
 * @brief Release high frequency clock if not needed
 */
void Api_sleep_peripheral_hfclk_release(void)
{
#ifndef SOFTDEVICE_PRESENT
	// Release HFCLK if SoftDevice is not present
	nrf_clock_hfclk_release();
#endif
}

/**
 * @brief Main function to disable all peripherals before entering sleep mode
 */
void Api_sleep_peripheral_disable_all(void)
{
	// Disable communication peripherals
	//Api_sleep_peripheral_uart_disable_all();
	Api_sleep_peripheral_spi_disable_all();
	Api_sleep_peripheral_twi_disable_all();

	// Disable timing peripherals
	//Api_sleep_peripheral_timer_disable_all();
	Api_sleep_peripheral_pwm_disable_all();

	// Disable analog peripherals
	Api_sleep_peripheral_saadc_disable();

	// Disable other peripherals
	Api_sleep_peripheral_others_disable();

	// Release high frequency clock
	//Api_sleep_peripheral_hfclk_release();

	// Small delay to ensure all disable operations are completed
//	for (volatile uint32_t Api_sleep_peripheral_delay_ui32 = 0; Api_sleep_peripheral_delay_ui32 < 1000; Api_sleep_peripheral_delay_ui32++)
//	{
//		__NOP();
//	}
}

/**
 * @brief Re-enable peripherals after waking up from sleep mode
 *
 * This function re-initializes all peripherals that were disabled during 
 * sleep mode to reduce power consumption. It restores functionality of:
 * - UART logging interface
 * - Motion sensor I2C (LIS2DH12)
 * - Battery monitoring
 * - NFC I2C interface (NT3H2211)
 * - UWB SPI interface (DW3210)
 *
 * @note This function should be called immediately after the system wakes up
 *       from sleep mode to restore full peripheral functionality.
 *
 * @return void
 */
void Api_sleep_peripheral_enable_after_wakeup(void)
{
	Api_motion_twi_init();
	Api_nfc_twi_init();
	Api_battery_init();
	Api_uwb_spi_init(false);
}

void Api_sleep_peripheral_disable_before_sleep(void)
{
	Api_motion_twi_unInit();
	Api_nfc_twi_unInit();
	Api_battery_unInit();
	Api_uwb_spi_uninit();
}

