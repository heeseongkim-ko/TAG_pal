#ifndef APP_TEIA_ROUTINES_H
#define APP_TEIA_ROUTINES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// GPIO initialization functions
void APP_TEIA_GPIO_Init(void);
void APP_TEIA_LED_Init(void);
void APP_TEIA_UART_Pins_Init(void);
void APP_TEIA_SPI_Pins_Init(void);
void APP_TEIA_ADC_Pins_Init(void);
void APP_TEIA_I2C_Pins_Init(void);
void APP_TEIA_IRQ_Pins_Init(void);

// LED control functions
// LED IDs: RED_TEIA, GREEN_TEIA, ORANGE_TEIA, BLACK_TEIA
void APP_TEIA_LED_Set(uint8_t led_id, bool state);
void APP_TEIA_LED_Toggle(uint8_t led_id);

// Power management functions
void APP_TEIA_Configure_RAM_Retention(void);

#ifdef __cplusplus
}
#endif

#endif // APP_TEIA_ROUTINES_H
