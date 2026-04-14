#include "bsp_adc.h"
#include "adc.h"

void bsp_adc_init(void)
{
    HAL_ADC_Start(&hadc1);
}

void bsp_adc_start(void)
{
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

uint16_t bsp_adc_read(void)
{
    return (uint16_t)ADC1->DR;
}
