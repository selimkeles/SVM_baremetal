#ifndef APP_INC_SVPWM_H_
#define APP_INC_SVPWM_H_

#include <stdint.h>

typedef struct {
    uint32_t ccr_a;
    uint32_t ccr_b;
    uint32_t ccr_c;
} svpwm_dwell_t;

void svpwm_compute(float theta_deg, float m, uint32_t ts, svpwm_dwell_t *out);

#endif
