#include "svpwm.h"
#include <math.h>

#define SVPWM_PI 3.14159265358979323846f

void svpwm_compute(float theta_deg, float m, uint32_t ts, svpwm_dwell_t *out)
{
    int sector = (int)(theta_deg / 60.0f) + 1;
    if (sector < 1) sector = 1;
    if (sector > 6) sector = 6;

    float theta_rad = theta_deg * (SVPWM_PI / 180.0f);
    float ts_m      = (float)ts * m;

    uint32_t t1 = (uint32_t)(ts_m * sinf(( SVPWM_PI / 3.0f) *  (float)sector        - theta_rad));
    uint32_t t2 = (uint32_t)(ts_m * sinf((-SVPWM_PI / 3.0f) * ((float)sector - 1.0f) + theta_rad));
    uint32_t tz = (t1 + t2 > ts) ? 0u : (ts - t1 - t2);
    uint32_t half_tz = tz / 2u;

    switch (sector) {
    case 1:
        out->ccr_a = half_tz + t1 + t2;
        out->ccr_b = half_tz + t2;
        out->ccr_c = half_tz;
        break;
    case 2:
        out->ccr_a = half_tz + t1;
        out->ccr_b = half_tz + t1 + t2;
        out->ccr_c = half_tz;
        break;
    case 3:
        out->ccr_a = half_tz;
        out->ccr_b = half_tz + t1 + t2;
        out->ccr_c = half_tz + t2;
        break;
    case 4:
        out->ccr_a = half_tz;
        out->ccr_b = half_tz + t1;
        out->ccr_c = half_tz + t1 + t2;
        break;
    case 5:
        out->ccr_a = half_tz + t2;
        out->ccr_b = half_tz;
        out->ccr_c = half_tz + t1 + t2;
        break;
    case 6:
    default:
        out->ccr_a = half_tz + t1 + t2;
        out->ccr_b = half_tz;
        out->ccr_c = half_tz + t1;
        break;
    }
}
