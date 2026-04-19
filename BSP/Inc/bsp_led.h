#ifndef BSP_INC_BSP_LED_H_
#define BSP_INC_BSP_LED_H_

typedef enum {
    BSP_LED_GREEN = 0,
    BSP_LED_ORANGE,
    BSP_LED_RED,
    BSP_LED_BLUE,
    BSP_LED_COUNT
} bsp_led_t;

void bsp_led_init(void);
void bsp_led_on(bsp_led_t led);
void bsp_led_off(bsp_led_t led);
void bsp_led_toggle(bsp_led_t led);

#endif
