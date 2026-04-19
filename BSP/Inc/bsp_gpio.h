#ifndef BSP_INC_BSP_GPIO_H_
#define BSP_INC_BSP_GPIO_H_

/*
 * Gate signal pin map (TIM1, AF1):
 *   U:  PE9  (CH1)  / PE8  (CH1N)
 *   V:  PE11 (CH2)  / PB0  (CH2N)
 *   W:  PE13 (CH3)  / PB1  (CH3N)
 *
 * Misc:
 *   PA0  : user button (EXTI0, rising edge)
 *   PC1  : ADC1_IN11 (frequency potentiometer)
 *   PD12-15: status LEDs
 */

void bsp_gpio_init(void);

#endif
