#include "vf_control.h"

void vf_state_init(vf_state_t *s)
{
    s->theta_deg = 0.0f;
    s->m         = 1.0f;
    s->freq_hz   = 0;
}

void vf_step(vf_state_t *s, uint16_t adc_raw, float carrier_hz)
{
    int freq = adc_raw / 50;
    if (freq < 1) freq = 1;

    float m = (float)freq / 50.0f;
    if (m > 1.0f) m = 1.0f;

    s->freq_hz = freq;
    s->m       = m;
    s->theta_deg += ((float)freq * 360.0f) / carrier_hz;
    if (s->theta_deg >= 360.0f) s->theta_deg -= 360.0f;
}
