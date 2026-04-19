#include "bsp_pwm.h"
#include "tim.h"

void bsp_pwm_init(void)
{
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
}

void bsp_pwm_start(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

void bsp_pwm_set_ccr(uint32_t ccr_a, uint32_t ccr_b, uint32_t ccr_c)
{
    TIM1->CCR1 = ccr_a;
    TIM1->CCR2 = ccr_b;
    TIM1->CCR3 = ccr_c;
}

void bsp_pwm_enable_output(bool enabled)
{
    if (enabled) {
        __HAL_TIM_MOE_ENABLE(&htim1);
    } else {
        __HAL_TIM_MOE_DISABLE(&htim1);
    }
}

void bsp_pwm_trigger_comg(void)
{
    TIM1->EGR |= TIM_EGR_COMG;
}
