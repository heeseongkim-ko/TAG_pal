/**
 * @file Api_nfc.h
 * @brief High-level NFC API for Unified SDK
 * 
 * Provides unified interface for NFC operations using NTAG I2C Plus
 */

#ifndef TEIA_DEFINE_H
#define TEIA_DEFINE_H

#ifdef __cplusplus
extern "C" {
#endif

#define TEIA_API_EXAMPLE

#define DW3000_IRQn_Pin   NRF_GPIO_PIN_MAP(1,2)
#define DW3000_RESET_Pin  NRF_GPIO_PIN_MAP(0,25)
#define DW3000_WAKEUP_Pin NRF_GPIO_PIN_MAP(1,3)
#define DW3000_IRQ2n_Pin  ARDUINO_6_PIN
#define DW3000_CS_Pin   NRF_GPIO_PIN_MAP(1,6)
#define DW3000_CLK_Pin  NRF_GPIO_PIN_MAP(0,3) // DWM3000 shield SPIM1 sck connected to DW3000
#define DW3000_MOSI_Pin NRF_GPIO_PIN_MAP(0,8) // DWM3000 shield SPIM1 mosi connected to DW3000
#define DW3000_MISO_Pin NRF_GPIO_PIN_MAP(0,29) // DWM3000 shield SPIM1 miso connected to DW3000

#define	NT3H2x_SCL		NRF_GPIO_PIN_MAP(0,27)
#define	NT3H2x_SDA		NRF_GPIO_PIN_MAP(0,31)
//#define	NT3H2x_SCL		NRF_GPIO_PIN_MAP(1,4)
//#define	NT3H2x_SDA		NRF_GPIO_PIN_MAP(0,24)
#define	NT3H2x_FD		NRF_GPIO_PIN_MAP(0,17)


#ifdef __cplusplus
}
#endif

#endif // TEIA_DEFINE_H