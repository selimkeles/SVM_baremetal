#include "bsp_button.h"
#include "bsp_led.h"
#include "inverter.h"
#include "main.h"

void bsp_button_init(void)
{
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0) {
        inverter_toggle_enable();
        bsp_led_toggle(BSP_LED_GREEN);
        bsp_led_toggle(BSP_LED_ORANGE);
        bsp_led_toggle(BSP_LED_RED);
        bsp_led_toggle(BSP_LED_BLUE);
    }
}
