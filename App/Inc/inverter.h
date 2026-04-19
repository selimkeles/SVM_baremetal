#ifndef APP_INC_INVERTER_H_
#define APP_INC_INVERTER_H_

#include <stdbool.h>
#include <stdint.h>

void inverter_init(void);
void inverter_on_pwm_update(void);
void inverter_enable(bool enabled);
void inverter_toggle_enable(void);
bool inverter_is_enabled(void);

void  inverter_set_freq_hz(int hz);
int   inverter_get_freq_hz(void);
float inverter_get_theta_deg(void);
float inverter_get_modulation(void);

#endif
