#ifndef BSP_INC_BSP_PWM_H_
#define BSP_INC_BSP_PWM_H_

#include <stdbool.h>
#include <stdint.h>

void bsp_pwm_init(void);
void bsp_pwm_start(void);
void bsp_pwm_set_ccr(uint32_t ccr_a, uint32_t ccr_b, uint32_t ccr_c);
void bsp_pwm_enable_output(bool enabled);
void bsp_pwm_trigger_comg(void);

#endif
