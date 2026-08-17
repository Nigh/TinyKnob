#include "motor.h"
#include "pins.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

static uint pwm_wrap;

static uint16_t duty_to_level(float d) {
	if(d < 0.f)
		d = 0.f;
	if(d > 1.f)
		d = 1.f;
	return (uint16_t)(d * (float)pwm_wrap);
}

void motor_set_duty(float a, float b, float c) {
	pwm_set_gpio_level(PIN_PWM_A, duty_to_level(a));
	pwm_set_gpio_level(PIN_PWM_B, duty_to_level(b));
	pwm_set_gpio_level(PIN_PWM_C, duty_to_level(c));
}

void motor_setup(void) {
	gpio_set_function(PIN_PWM_A, GPIO_FUNC_PWM);
	gpio_set_function(PIN_PWM_B, GPIO_FUNC_PWM);
	gpio_set_function(PIN_PWM_C, GPIO_FUNC_PWM);

	uint sa = pwm_gpio_to_slice_num(PIN_PWM_A);
	uint sb = pwm_gpio_to_slice_num(PIN_PWM_B);
	uint sc = pwm_gpio_to_slice_num(PIN_PWM_C);

	uint32_t sys = clock_get_hz(clk_sys);
	pwm_wrap = sys / PWM_HZ;
	if(pwm_wrap < 2)
		pwm_wrap = 2;
	pwm_wrap -= 1;

	pwm_config cfg = pwm_get_default_config();
	pwm_config_set_clkdiv(&cfg, 1.f);
	pwm_config_set_wrap(&cfg, (uint16_t)pwm_wrap);
	pwm_init(sa, &cfg, false);
	pwm_init(sb, &cfg, false);
	pwm_init(sc, &cfg, false);

	motor_set_duty(0.5f, 0.5f, 0.5f);
	pwm_set_mask_enabled((1u << sa) | (1u << sb) | (1u << sc));
}
