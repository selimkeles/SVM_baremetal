#ifndef APP_INC_VF_CONTROL_H_
#define APP_INC_VF_CONTROL_H_

#include <stdint.h>

typedef struct {
    float theta_deg;
    float m;
    int   freq_hz;
} vf_state_t;

void vf_state_init(vf_state_t *s);
void vf_step(vf_state_t *s, uint16_t adc_raw, float carrier_hz);

#endif
