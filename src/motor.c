#include "motor.h"
#include "pins.h"
#include "mt6701.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include <math.h>

#define PI 3.14159265f
#define RAD_PER_COUNT (2.f * PI / 16384.f)
#define SQRT3_2 0.86602540378f

static uint pwm_wrap;
static uint pwm_slice_a;

static volatile uint8_t mode;
static volatile int32_t pos;
static volatile int8_t dir = 1;
static volatile uint32_t crc_fail;
static float offset_rad;
static float align_theta_m;
static int32_t pulse_pos0;

static bool have_enc;
static uint16_t prev_raw;
static uint32_t mode_tick;
static uint32_t hold_sum;
static uint32_t hold_n;
static float ud_cmd;
static float uq_cmd;
static bool te_from_encoder;

static uint32_t ms_to_ticks(uint32_t ms) {
	return (PWM_HZ * ms) / 1000u;
}

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

void motor_get_state(motor_state_t* s) {
	s->mode = mode;
	s->pos = pos;
	s->offset_mrad = (int32_t)(offset_rad * 1000.f);
	s->dir = dir;
	s->crc_fail = crc_fail;
}

static void ingest_encoder(uint16_t raw) {
	if(!have_enc) {
		prev_raw = raw;
		pos = (int32_t)raw;
		have_enc = true;
		return;
	}
	int32_t d = (int32_t)raw - (int32_t)prev_raw;
	if(d > 8192)
		d -= 16384;
	if(d < -8192)
		d += 16384;
	pos += d;
	prev_raw = raw;
}

static void apply_voltage(float ud, float uq, float te) {
	float mag = sqrtf(ud * ud + uq * uq);
	if(mag > 1.f) {
		ud /= mag;
		uq /= mag;
	}
	float c = cosf(te);
	float s = sinf(te);
	float ualpha = ud * c - uq * s;
	float ubeta = ud * s + uq * c;
	float va = ualpha;
	float vb = -0.5f * ualpha + SQRT3_2 * ubeta;
	float vc = -0.5f * ualpha - SQRT3_2 * ubeta;
	motor_set_duty(0.5f + 0.5f * va, 0.5f + 0.5f * vb, 0.5f + 0.5f * vc);
}

static void apply_cmd(void) {
	float te = 0.f;
	if(te_from_encoder) {
		float th_m = (float)pos * RAD_PER_COUNT;
		te = (float)dir * th_m * (float)MOTOR_POLE_PAIRS + offset_rad;
	}
	float uq = te_from_encoder ? (float)dir * uq_cmd : uq_cmd;
	apply_voltage(ud_cmd, uq, te);
}

static void enter_mode(uint8_t next) {
	mode = next;
	mode_tick = 0;
}

static void motor_isr(void) {
	mt6701_sample_t s;
	if(mt6701_try_read(&s)) {
		if(s.crc_ok)
			ingest_encoder(s.angle);
		else
			crc_fail++;
	}
	mt6701_start_read();

	mode_tick++;
	switch(mode) {
		case MOTOR_ALIGN_RAMP: {
			uint32_t ramp = ms_to_ticks(ALIGN_RAMP_MS);
			if(ramp == 0)
				ramp = 1;
			ud_cmd = ALIGN_UD * (float)mode_tick / (float)ramp;
			uq_cmd = 0.f;
			te_from_encoder = false;
			if(mode_tick >= ramp) {
				ud_cmd = ALIGN_UD;
				hold_sum = 0;
				hold_n = 0;
				enter_mode(MOTOR_ALIGN_HOLD);
			}
			break;
		}
		case MOTOR_ALIGN_HOLD:
			ud_cmd = ALIGN_UD;
			uq_cmd = 0.f;
			te_from_encoder = false;
			if(have_enc) {
				hold_sum += (uint32_t)prev_raw;
				hold_n++;
			}
			if(mode_tick >= ms_to_ticks(ALIGN_HOLD_MS)) {
				if(hold_n == 0) {
					ud_cmd = 0.f;
					enter_mode(MOTOR_FAULT);
					break;
				}
				float avg = (float)(hold_sum / hold_n);
				align_theta_m = avg * RAD_PER_COUNT;
				dir = 1;
				offset_rad = -(float)dir * align_theta_m * (float)MOTOR_POLE_PAIRS;
				pulse_pos0 = pos;
				enter_mode(MOTOR_DIR_PULSE);
			}
			break;
		case MOTOR_DIR_PULSE:
			ud_cmd = ALIGN_UD;
			uq_cmd = DIR_PULSE_UQ;
			te_from_encoder = false;
			if(mode_tick >= ms_to_ticks(DIR_PULSE_MS)) {
				if(have_enc && (pos < pulse_pos0 - 20)) {
					dir = -1;
					offset_rad = -(float)dir * align_theta_m * (float)MOTOR_POLE_PAIRS;
				}
				enter_mode(MOTOR_ALIGN_DOWN);
			}
			break;
		case MOTOR_ALIGN_DOWN: {
			uint32_t ramp = ms_to_ticks(ALIGN_RAMP_MS);
			if(ramp == 0)
				ramp = 1;
			float k = 1.f - (float)mode_tick / (float)ramp;
			if(k < 0.f)
				k = 0.f;
			ud_cmd = ALIGN_UD * k;
			uq_cmd = 0.f;
			te_from_encoder = true;
			if(mode_tick >= ramp) {
				ud_cmd = 0.f;
				uq_cmd = 0.f;
				enter_mode(MOTOR_IDLE);
			}
			break;
		}
		case MOTOR_IDLE:
			ud_cmd = 0.f;
			uq_cmd = 0.f;
			te_from_encoder = true;
			break;
		case MOTOR_FAULT:
			ud_cmd = 0.f;
			uq_cmd = 0.f;
			te_from_encoder = false;
			motor_set_duty(0.5f, 0.5f, 0.5f);
			return;
		default:
			break;
	}
	apply_cmd();
}

static void pwm_wrap_isr(void) {
	if(pwm_get_irq_status_mask() & (1u << pwm_slice_a)) {
		pwm_clear_irq(pwm_slice_a);
		motor_isr();
	}
}

void motor_setup(void) {
	gpio_set_function(PIN_PWM_A, GPIO_FUNC_PWM);
	gpio_set_function(PIN_PWM_B, GPIO_FUNC_PWM);
	gpio_set_function(PIN_PWM_C, GPIO_FUNC_PWM);

	pwm_slice_a = pwm_gpio_to_slice_num(PIN_PWM_A);
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
	pwm_init(pwm_slice_a, &cfg, false);
	pwm_init(sb, &cfg, false);
	pwm_init(sc, &cfg, false);

	motor_set_duty(0.5f, 0.5f, 0.5f);
	mode = MOTOR_ALIGN_RAMP;
	mode_tick = 0;
	te_from_encoder = false;

	pwm_clear_irq(pwm_slice_a);
	pwm_set_irq_enabled(pwm_slice_a, true);
	irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_wrap_isr);
	irq_set_enabled(PWM_IRQ_WRAP, true);

	pwm_set_mask_enabled((1u << pwm_slice_a) | (1u << sb) | (1u << sc));
	mt6701_start_read();
}
