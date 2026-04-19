#include "bsp_led.h"
#include "main.h"

// Board LEDs LD3..LD6 on PD3..PD6. Pin init is done by MX_GPIO_Init (CubeMX);
// this module just maps logical LED names to GPIO pins and provides on/off/toggle.
static const uint16_t s_led_pins[BSP_LED_COUNT] = {
    [BSP_LED_GREEN]  = GPIO_PIN_12,
    [BSP_LED_ORANGE] = GPIO_PIN_13,
    [BSP_LED_RED]    = GPIO_PIN_14,
    [BSP_LED_BLUE]   = GPIO_PIN_15,
};

void bsp_led_init(void)
{
}

void bsp_led_on(bsp_led_t led)     { HAL_GPIO_WritePin(GPIOD, s_led_pins[led], GPIO_PIN_SET); }
void bsp_led_off(bsp_led_t led)    { HAL_GPIO_WritePin(GPIOD, s_led_pins[led], GPIO_PIN_RESET); }
void bsp_led_toggle(bsp_led_t led) { HAL_GPIO_TogglePin(GPIOD, s_led_pins[led]); }
