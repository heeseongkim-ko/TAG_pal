// APP_TEIA_ROUTINES.c
// Provides GPIO initialization and control functions

#include <stdbool.h>
#include <stdint.h>

#include "APP_TEIA_ROUTINES.h"
#include "def_pin.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_power.h"

// GPIO initialization - initialize all pins
void APP_TEIA_GPIO_Init(void)
{
    // Initialize LED pins
    APP_TEIA_LED_Init();
    
    // Initialize UART pins
    APP_TEIA_UART_Pins_Init();
    
    // Initialize SPI pins
    APP_TEIA_SPI_Pins_Init();
    
    // Initialize ADC pins
    APP_TEIA_ADC_Pins_Init();
    
    // Initialize I2C pins
    APP_TEIA_I2C_Pins_Init();
    
    // Initialize IRQ pins
    APP_TEIA_IRQ_Pins_Init();
}

// LED pin initialization
void APP_TEIA_LED_Init(void)
{
    // Configure LED pins as outputs
    nrf_gpio_cfg_output(LED_RED_TEIA);
    nrf_gpio_cfg_output(LED_GREEN_TEIA);
    
    // Initial state is OFF (LEDs are active HIGH)
    nrf_gpio_pin_set(LED_RED_TEIA);    // LED OFF (HIGH = OFF)
    nrf_gpio_pin_set(LED_GREEN_TEIA);  // LED OFF (HIGH = OFF)
}

// UART pin initialization
void APP_TEIA_UART_Pins_Init(void)
{
    // Manually configure UART pins to ensure they are set correctly
    nrf_gpio_cfg_output(UART_TX_PIN_NUMBER_TEIA);  // TX as output
    nrf_gpio_cfg_input(UART_RX_PIN_NUMBER_TEIA, NRF_GPIO_PIN_PULLUP);  // RX as input with pullup
    
    // Set initial TX state to high (idle state)
    nrf_gpio_pin_set(UART_TX_PIN_NUMBER_TEIA);
}

// SPI pin initialization
void APP_TEIA_SPI_Pins_Init(void)
{
    // Configure SPI CLK pin as output
    nrf_gpio_cfg_output(DW3000_CLK_Pin_TEIA);
    
    // Configure SPI MOSI pin as output
    nrf_gpio_cfg_output(DW3000_MOSI_Pin_TEIA);
    
    // Configure SPI MISO pin as input (no pull-up)
    nrf_gpio_cfg_input(DW3000_MISO_Pin_TEIA, NRF_GPIO_PIN_NOPULL);
    
    // Configure SPI CS pin as output (initial state is HIGH)
    nrf_gpio_cfg_output(DW3000_CS_Pin_TEIA);
    nrf_gpio_pin_set(DW3000_CS_Pin_TEIA);
    
    // Configure DW3000 Wake-up pin as output (initial state is LOW)
    nrf_gpio_cfg_output(DW3000_WKUP_Pin_TEIA);
    nrf_gpio_pin_clear(DW3000_WKUP_Pin_TEIA);
    
    // Configure DW3000 IRQ pin as input (pull-up enabled)
    nrf_gpio_cfg_input(DW3000_IRQ_Pin_TEIA, NRF_GPIO_PIN_PULLUP);
    
    // Configure DW3000 Reset pin as output (initial state is HIGH)
    nrf_gpio_cfg_output(DW3000_RST_Pin_TEIA);
    nrf_gpio_pin_set(DW3000_RST_Pin_TEIA);
}

// ADC pin initialization
void APP_TEIA_ADC_Pins_Init(void)
{
    nrf_gpio_cfg_output(ADC_RDIV_TEIA);
    nrf_gpio_pin_set(ADC_RDIV_TEIA);

    nrf_gpio_cfg_input(ADC_MEAS_TEIA,NRF_GPIO_PIN_PULLUP);
}

// I2C pin initialization
void APP_TEIA_I2C_Pins_Init(void)
{
    // Configure I2C SCL pin as input (pull-up enabled)
    nrf_gpio_cfg_input(I2C_SCL_TEIA, NRF_GPIO_PIN_PULLUP);
    
    // Configure I2C SDA pin as input (pull-up enabled)
    nrf_gpio_cfg_input(I2C_SDA_TEIA, NRF_GPIO_PIN_PULLUP);
}

// IRQ pin initialization
void APP_TEIA_IRQ_Pins_Init(void)
{
    // Configure ACC IRQ pin as input (pull-up enabled)
    nrf_gpio_cfg_input(ACC_IRQ_TEIA, NRF_GPIO_PIN_PULLUP);
    
    // Configure NFC FD pin as input (pull-up enabled)
    nrf_gpio_cfg_input(NFC_FD_TEIA, NRF_GPIO_PIN_PULLUP);
}

// LED control functions
void APP_TEIA_LED_Set(uint8_t led_id, bool state)
{
    switch (led_id)
    {
        case RED_TEIA:
            if (state)
                nrf_gpio_pin_clear(LED_RED_TEIA);    // LED ON (LOW = ON)
            else
                nrf_gpio_pin_set(LED_RED_TEIA);      // LED OFF (HIGH = OFF)
            break;
            
        case GREEN_TEIA:
            if (state)
                nrf_gpio_pin_clear(LED_GREEN_TEIA);  // LED ON (LOW = ON)
            else
                nrf_gpio_pin_set(LED_GREEN_TEIA);    // LED OFF (HIGH = OFF)
            break;
            
        case ORANGE_TEIA:
            // ORANGE is both RED and GREEN turned on simultaneously
            if (state)
            {
                nrf_gpio_pin_clear(LED_RED_TEIA);    // LED ON (LOW = ON)
                nrf_gpio_pin_clear(LED_GREEN_TEIA);  // LED ON (LOW = ON)
            }
            else
            {
                nrf_gpio_pin_set(LED_RED_TEIA);      // LED OFF (HIGH = OFF)
                nrf_gpio_pin_set(LED_GREEN_TEIA);    // LED OFF (HIGH = OFF)
            }
            break;
            
        default:
            break;
    }
}

void APP_TEIA_LED_Toggle(uint8_t led_id)
{
    switch (led_id)
    {
        case RED_TEIA:
            nrf_gpio_pin_toggle(LED_RED_TEIA);
            break;
            
        case GREEN_TEIA:
            nrf_gpio_pin_toggle(LED_GREEN_TEIA);
            break;
            
        case ORANGE_TEIA:
            // ORANGE is both RED and GREEN toggled simultaneously
            nrf_gpio_pin_toggle(LED_RED_TEIA);
            nrf_gpio_pin_toggle(LED_GREEN_TEIA);
            break;
            
        default:
            break;
    }
}



// RAM power management using official Nordic SDK
void APP_TEIA_Configure_RAM_Retention(void)
{
    // Configure nRF52 RAM retention parameters for System On 64kB RAM retention
    // Using official Nordic SDK register access (safe way)
    
    // Enable RAM0-7 power (64kB total)
    // Note: This is the safe way to configure RAM power in nRF52
    NRF_POWER->RAM[0].POWER = 1;  // RAM0 ON
    NRF_POWER->RAM[1].POWER = 1;  // RAM1 ON
    NRF_POWER->RAM[2].POWER = 1;  // RAM2 ON
    NRF_POWER->RAM[3].POWER = 1;  // RAM3 ON
    NRF_POWER->RAM[4].POWER = 1;  // RAM4 ON
    NRF_POWER->RAM[5].POWER = 1;  // RAM5 ON
    NRF_POWER->RAM[6].POWER = 1;  // RAM6 ON
    NRF_POWER->RAM[7].POWER = 1;  // RAM7 ON
    
    // Note: Retention settings are handled automatically by the SDK
    // when entering/exiting sleep modes
}
