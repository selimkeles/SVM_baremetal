#include "inverter.h"
#include "svpwm.h"
#include "vf_control.h"
#include "bsp_pwm.h"

#define INVERTER_CARRIER_HZ 20000.0f
#define INVERTER_TS_COUNTS  8400u

static vf_state_t s_vf;
static bool       s_enabled;

void inverter_init(void)
{
    vf_state_init(&s_vf);
    s_enabled = false;
    bsp_pwm_enable_output(false);
}

void inverter_on_pwm_update(uint16_t adc_raw)
{
    vf_step(&s_vf, adc_raw, INVERTER_CARRIER_HZ);

    svpwm_dwell_t d;
    svpwm_compute(s_vf.theta_deg, s_vf.m, INVERTER_TS_COUNTS, &d);

    bsp_pwm_set_ccr(d.ccr_a, d.ccr_b, d.ccr_c);
}

void inverter_enable(bool enabled)
{
    s_enabled = enabled;
    bsp_pwm_enable_output(enabled);
}

void inverter_toggle_enable(void)
{
    inverter_enable(!s_enabled);
}

bool inverter_is_enabled(void)      { return s_enabled; }
int  inverter_get_freq_hz(void)     { return s_vf.freq_hz; }
float inverter_get_theta_deg(void)  { return s_vf.theta_deg; }
float inverter_get_modulation(void) { return s_vf.m; }

void inverter_set_freq_hz(int hz)
{
    // Setter stub for future USB CLI. Open-loop freq is driven by ADC today;
    // once the CLI lands, swap vf_step() input source behind a flag.
    (void)hz;
}
