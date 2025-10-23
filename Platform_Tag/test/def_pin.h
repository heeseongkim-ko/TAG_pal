
#ifndef DEF_PIN_H_INCLUDED
#define DEF_PIN_H_INCLUDED

#include "nrf_gpio.h"


#if 0  //dwm3001
//************************************************************************led

#define LED_RED   NRF_GPIO_PIN_MAP(0,14)
#define LED_GREEN NRF_GPIO_PIN_MAP(0,22)
#define RED 1
#define GREEN 2
#define ORANGE 3

//************************************************************************uart

#define UART_TX_PIN_NUMBER NRF_GPIO_PIN_MAP(0,19) //debug
#define UART_RX_PIN_NUMBER NRF_GPIO_PIN_MAP(0,15) //debug
#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256 
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED

//************************************************************************spi

#define DW3000_CLK_Pin              NRF_GPIO_PIN_MAP(0,  3)  // used as DW3000_CLK_Pin
#define DW3000_MISO_Pin             NRF_GPIO_PIN_MAP(0, 29)  // used as DW3000_MISO_Pin
#define DW3000_MOSI_Pin              NRF_GPIO_PIN_MAP(0,  8)  // used as DW3000_MOSI_Pin
#define DW3000_CS_Pin              NRF_GPIO_PIN_MAP(1,  6)  // used as DW3000_CS_Pin
#define DW3000_WKUP_Pin               NRF_GPIO_PIN_MAP(1, 19)  // used as DW3000_WKUP_Pin
#define DW3000_IRQ_Pin              NRF_GPIO_PIN_MAP(1,  2)  // used as DW3000_IRQ_Pin
#define DW3000_RST_Pin               NRF_GPIO_PIN_MAP(0, 25)  // used as DW3000_RST_Pin
//************************************************************************spi

//************************************************************************spi

//************************************************************************spi

//************************************************************************uwb
#endif 
#if 1
//************************************************************************led

#define LED_RED   NRF_GPIO_PIN_MAP(0,7)
#define LED_GREEN NRF_GPIO_PIN_MAP(0,23)
#define RED 1
#define GREEN 2
#define ORANGE 3
#define BLACK 4

//************************************************************************uart

#define UART_TX_PIN_NUMBER NRF_GPIO_PIN_MAP(0,26) //debug
#define UART_RX_PIN_NUMBER NRF_GPIO_PIN_MAP(0,19) //debug
#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256 
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED

//************************************************************************spi

#define DW3000_CLK_Pin              NRF_GPIO_PIN_MAP(0,  3)  // used as DW3000_CLK_Pin
#define DW3000_MISO_Pin             NRF_GPIO_PIN_MAP(0, 29)  // used as DW3000_MISO_Pin
#define DW3000_MOSI_Pin              NRF_GPIO_PIN_MAP(0,  8)  // used as DW3000_MOSI_Pin
#define DW3000_CS_Pin              NRF_GPIO_PIN_MAP(1,  6)  // used as DW3000_CS_Pin
#define DW3000_WKUP_Pin               NRF_GPIO_PIN_MAP(1, 3)  // used as DW3000_WKUP_Pin//(1, 19)
#define DW3000_IRQ_Pin              NRF_GPIO_PIN_MAP(1,  2)  // used as DW3000_IRQ_Pin
#define DW3000_RST_Pin               NRF_GPIO_PIN_MAP(0, 25)  // used as DW3000_RST_Pin
//************************************************************************adc
#define    ADC_RDIV     NRF_GPIO_PIN_MAP(0,30)
#define    ADC_MEAS     NRF_GPIO_PIN_MAP(0,2)
//************************************************************************I2C
#define    I2C_SCL     NRF_GPIO_PIN_MAP(0,27)//NRF_GPIO_PIN_MAP(1,04)
#define    I2C_SDA     NRF_GPIO_PIN_MAP(0,31)//NRF_GPIO_PIN_MAP(0,24)


//#define    I2C_SCL_NFC     NRF_GPIO_PIN_MAP(0,27)//NRF_GPIO_PIN_MAP(1,04)
//#define    I2C_SDA_NFC     NRF_GPIO_PIN_MAP(0,31)//NRF_GPIO_PIN_MAP(0,24)

#define    ACC_IRQ     NRF_GPIO_PIN_MAP(0,16)//acc intterupt
#define    NFC_FD      NRF_GPIO_PIN_MAP(0,17)//nfc intterupt 
//************************************************************************spi

//************************************************************************uwb
#endif 


#endif  /* DEF_PIN_H_INCLUDED */

