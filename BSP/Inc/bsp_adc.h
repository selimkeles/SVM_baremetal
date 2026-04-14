#ifndef BSP_INC_BSP_ADC_H_
#define BSP_INC_BSP_ADC_H_

#include <stdint.h>

void     bsp_adc_init(void);
void     bsp_adc_start(void);
uint16_t bsp_adc_read(void);

#endif
