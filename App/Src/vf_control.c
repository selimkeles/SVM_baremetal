#include "vf_control.h"

void vf_state_init(vf_state_t *s)
{
    s->theta_deg = 0.0f;
    s->m         = 0.0f;
    s->freq_hz   = 0;
}

void vf_step(vf_state_t *s, int freq_hz, float carrier_hz)
{
    if (freq_hz <= 0) {
        s->freq_hz = 0;
        s->m       = 0.0f;
        return;  /* hold theta; output zero modulation */
    }

    float m = (float)freq_hz / 50.0f;
    if (m > 1.0f) m = 1.0f;

    s->freq_hz    = freq_hz;
    s->m          = m;
    s->theta_deg += ((float)freq_hz * 360.0f) / carrier_hz;
    if (s->theta_deg >= 360.0f) s->theta_deg -= 360.0f;
}
