// Copied and organized from test/def_pin.h

#ifndef DEF_PIN_H_INCLUDED
#define DEF_PIN_H_INCLUDED

#include "nrf_gpio.h"

#if 0  //dwm3001 (reference)
//************************************************************************led
#define LED_RED   NRF_GPIO_PIN_MAP(0,14)
#define LED_GREEN NRF_GPIO_PIN_MAP(0,22)
#define RED 1
#define GREEN 2
#define ORANGE 3

//************************************************************************uart
#define UART_TX_PIN_NUMBER NRF_GPIO_PIN_MAP(0,19)
#define UART_RX_PIN_NUMBER NRF_GPIO_PIN_MAP(0,15)

//************************************************************************spi
#define DW3000_CLK_Pin   NRF_GPIO_PIN_MAP(0,  3)
#define DW3000_MISO_Pin  NRF_GPIO_PIN_MAP(0, 29)
#define DW3000_MOSI_Pin  NRF_GPIO_PIN_MAP(0,  8)
#define DW3000_CS_Pin    NRF_GPIO_PIN_MAP(1,  6)
#define DW3000_WKUP_Pin  NRF_GPIO_PIN_MAP(1, 19)
#define DW3000_IRQ_Pin   NRF_GPIO_PIN_MAP(1,  2)
#define DW3000_RST_Pin   NRF_GPIO_PIN_MAP(0, 25)
#endif

#if 1
//************************************************************************led
#define LED_RED_TEIA   NRF_GPIO_PIN_MAP(0,7)
#define LED_GREEN_TEIA NRF_GPIO_PIN_MAP(0,23)
#define RED_TEIA 1
#define GREEN_TEIA 2
#define ORANGE_TEIA 3
#define BLACK_TEIA 4

//************************************************************************uart
#define UART_TX_PIN_NUMBER_TEIA NRF_GPIO_PIN_MAP(0,26)
#define UART_RX_PIN_NUMBER_TEIA NRF_GPIO_PIN_MAP(0,19)
#define UART_HWFC_TEIA false
#define UART_RX_BUF_SIZE_TEIA 256
#define UART_TX_BUF_SIZE_TEIA 256

//************************************************************************spi
#define DW3000_CLK_Pin_TEIA   NRF_GPIO_PIN_MAP(0,  3)
#define DW3000_MISO_Pin_TEIA  NRF_GPIO_PIN_MAP(0, 29)
#define DW3000_MOSI_Pin_TEIA  NRF_GPIO_PIN_MAP(0,  8)
#define DW3000_CS_Pin_TEIA    NRF_GPIO_PIN_MAP(1,  6)
#define DW3000_WKUP_Pin_TEIA  NRF_GPIO_PIN_MAP(1, 3)
#define DW3000_IRQ_Pin_TEIA   NRF_GPIO_PIN_MAP(1,  2)
#define DW3000_RST_Pin_TEIA   NRF_GPIO_PIN_MAP(0, 25)

//************************************************************************adc
#define ADC_RDIV_TEIA  NRF_GPIO_PIN_MAP(0,30)
#define ADC_MEAS_TEIA  NRF_GPIO_PIN_MAP(0,2)

//************************************************************************I2C
#define I2C_SCL_TEIA   NRF_GPIO_PIN_MAP(0,27)
#define I2C_SDA_TEIA   NRF_GPIO_PIN_MAP(0,31)

//************************************************************************irq
#define ACC_IRQ_TEIA   NRF_GPIO_PIN_MAP(0,16)
#define NFC_FD_TEIA    NRF_GPIO_PIN_MAP(0,17)
#endif

#endif  /* DEF_PIN_H_INCLUDED */


