#include "motor.h"
#include "pins.h"
#include "mt6701.h"
#include "platform.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include <math.h>

#define RAD_PER_COUNT (2.f * 3.14159265f / 16384.f)
#define TWO_PI 6.2831853f
#define SQRT3_2 0.86602540378f
#define COUNTS_PER_REV 16384
#define SIN_LUT_N 1024
#define SIN_LUT_MASK (SIN_LUT_N - 1)
#define PWM_MIN_MAG2 (PWM_MIN_MAG * PWM_MIN_MAG)
#define SPRING_VEL_ALPHA (TWO_PI * SPRING_VEL_LP_HZ / (float)PWM_HZ)
#define SPIN_VEL_ALPHA (TWO_PI * SPIN_VEL_LP_HZ / (float)PWM_HZ)

static float sin_lut[SIN_LUT_N];

static uint pwm_wrap;
static uint pwm_slice_a;
static uint pwm_slice_b;
static uint pwm_slice_c;
static uint32_t pwm_enable_mask;

static volatile uint8_t mode = MOTOR_IDLE;
static volatile bool aligned;
static volatile int32_t pos;
static volatile int8_t dir = 1;
static volatile uint32_t crc_fail;
static volatile uint32_t crc_ok;
static volatile uint32_t last_raw;
static volatile uint32_t last_pio;
static volatile bool last_crc_ok;
static float offset_rad;
static float align_theta_m;
static int32_t pulse_pos0;
static int32_t home_pos;
static bool test_fwd;
static int32_t move_start;
static int32_t move_target;
static int32_t rest_pos;
static float spring_k = SPRING_K;
static int32_t pos_last;
static float vel_filt;

static bool have_enc;
static uint16_t prev_raw;
static uint32_t mode_tick;
static int64_t hold_sum;
static uint32_t hold_n;
static int32_t drag_pos0;
static uint32_t crc_fail_run;
static float ud_cmd;
static float uq_cmd;
static float forced_te;
static bool te_from_encoder;
static bool enc_ok_this;

static uint32_t ms_to_ticks(uint32_t ms) {
	return (PWM_HZ * ms) / 1000u;
}

static float clampf(float x, float lo, float hi) {
	if(x < lo)
		return lo;
	if(x > hi)
		return hi;
	return x;
}

static int32_t spring_wrap(int32_t d) {
	d %= COUNTS_PER_REV;
	if(d < 0)
		d += COUNTS_PER_REV;
	if(d > COUNTS_PER_REV / 2)
		d -= COUNTS_PER_REV;
	return d;
}

static float spring_uq(int32_t d_counts, float vel, float k) {
	float err = (float)spring_wrap(d_counts) * RAD_PER_COUNT;
	return clampf(k * err - SPRING_D * vel, -SPRING_UQ_MAX, SPRING_UQ_MAX);
}

static bool spring_selfcheck(void) {
	if(spring_wrap(100) != 100)
		return false;
	if(spring_wrap(-100) != -100)
		return false;
	if(spring_wrap(COUNTS_PER_REV - 100) != -100)
		return false;
	if(spring_uq(2000, 0.f, SPRING_K) <= 0.f)
		return false;
	if(spring_uq(-2000, 0.f, SPRING_K) >= 0.f)
		return false;
	// 1 count through the LPF must not eat ~10° of restoring Uq
	float v = SPRING_VEL_ALPHA * RAD_PER_COUNT * (float)PWM_HZ;
	float u10 = SPRING_K * (10.f * 3.14159265f / 180.f);
	if(SPRING_D * v >= u10 * 0.5f)
		return false;
	if(SPRING_UQ_MAX <= DIR_PULSE_UQ)
		return false;
	return true;
}

static float vel_update(float alpha) {
	float vel_raw = (float)(pos - pos_last) * RAD_PER_COUNT * (float)PWM_HZ;
	pos_last = pos;
	vel_filt += alpha * (vel_raw - vel_filt);
	return vel_filt;
}

static float spin_uq(float vel) {
	float absv = vel < 0.f ? -vel : vel;
	if(absv < SPIN_W_REST)
		return 0.f;
	float signv = vel < 0.f ? -1.f : 1.f;
	float over = absv - SPIN_W_MAX;
	if(over < 0.f)
		over = 0.f;
	float u = (SPIN_KV - SPIN_B) * vel - SPIN_B_CAP * over * signv;
	return clampf(u, -SPIN_UQ_MAX, SPIN_UQ_MAX);
}

static bool spin_selfcheck(void) {
	if(spin_uq(0.f) != 0.f)
		return false;
	if(spin_uq(SPIN_W_REST * 0.5f) != 0.f)
		return false;
	if(SPIN_B <= 0.f || SPIN_B >= SPIN_KV)
		return false;
	float net = SPIN_KV - SPIN_B;
	if(net >= 0.025f)
		return false;
	if(spin_uq(2.f) <= 0.f || spin_uq(-2.f) >= 0.f)
		return false;
	float u2 = spin_uq(2.f);
	if(u2 < net * 2.f - 0.002f || u2 > net * 2.f + 0.002f)
		return false;
	if(spin_uq(SPIN_W_MAX * 0.5f) <= 0.f)
		return false;
	if(SPIN_W_MAX <= SPIN_W_REST)
		return false;
	return true;
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

// ponytail: no INLx — cannot Hi-Z. IDLE holds INHx low (LS on). Mid PWM switches hard and wastes watts.
static void motor_brake_low(void) {
	ud_cmd = 0.f;
	uq_cmd = 0.f;
	forced_te = 0.f;
	te_from_encoder = false;
	motor_set_duty(0.f, 0.f, 0.f);
}

void motor_get_state(motor_state_t* s) {
	s->mode = mode;
	s->pos = pos;
	s->offset_mrad = (int32_t)(offset_rad * 1000.f);
	s->dir = dir;
	s->crc_fail = crc_fail;
	s->crc_ok = crc_ok;
	s->raw = last_raw;
	s->pio_word = last_pio;
	s->last_crc_ok = last_crc_ok;
}

void motor_set_spring_k(float k) {
	if(k < 0.f)
		k = 0.f;
	if(k > 8.f)
		k = 8.f;
	spring_k = k;
}

void motor_set_rest_to_current(void) {
	rest_pos = pos;
}

float motor_get_spring_k(void) {
	return spring_k;
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

static void sincos_lut(float te, float* s_out, float* c_out) {
	// Map te → [0,1) cycles without fmodf (soft-float heavy).
	float turns = te * (1.f / TWO_PI);
	turns -= (float)(int32_t)turns;
	if(turns < 0.f)
		turns += 1.f;
	uint32_t i = (uint32_t)(turns * (float)SIN_LUT_N) & SIN_LUT_MASK;
	*s_out = sin_lut[i];
	*c_out = sin_lut[(i + (SIN_LUT_N / 4)) & SIN_LUT_MASK];
}

static void apply_voltage(float ud, float uq, float te) {
	float mag2 = ud * ud + uq * uq;
	// ponytail: 0% duty is LS brake. SPIN needs tiny PWM to follow BEMF; spring keeps the deadzone.
	if(mode == MOTOR_SPIN) {
		if(mag2 < 1.0e-12f) {
			motor_set_duty(0.f, 0.f, 0.f);
			return;
		}
	} else if(mag2 < PWM_MIN_MAG2) {
		motor_set_duty(0.f, 0.f, 0.f);
		return;
	}
	if(mag2 > 1.f) {
		float inv = 1.f / sqrtf(mag2);
		ud *= inv;
		uq *= inv;
	}
	float s, c;
	sincos_lut(te, &s, &c);
	float ualpha = ud * c - uq * s;
	float ubeta = ud * s + uq * c;
	float va = ualpha;
	float vb = -0.5f * ualpha + SQRT3_2 * ubeta;
	float vc = -0.5f * ualpha - SQRT3_2 * ubeta;
	motor_set_duty(0.5f + 0.5f * va, 0.5f + 0.5f * vb, 0.5f + 0.5f * vc);
}

static void apply_cmd(void) {
	float te = forced_te;
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

static void start_move(int32_t target, uint8_t next) {
	move_start = pos;
	move_target = target;
	enter_mode(next);
}

static bool is_aligning(void) {
	return mode >= MOTOR_ALIGN_RAMP && mode <= MOTOR_ALIGN_DOWN;
}

static bool follow_move(void) {
	uint32_t move = ms_to_ticks(TEST_MOVE_MS);
	uint32_t hold = ms_to_ticks(TEST_HOLD_MS);
	if(move == 0)
		move = 1;
	float t = (mode_tick < move) ? ((float)mode_tick / (float)move) : 1.f;
	// 5th-order smoothstep: s=10t³−15t⁴+6t⁵, ds/dt=30t²(1−t)²
	float s;
	float dsdt;
	if(t <= 0.f) {
		s = 0.f;
		dsdt = 0.f;
	} else if(t >= 1.f) {
		s = 1.f;
		dsdt = 0.f;
	} else {
		float t2 = t * t;
		float t3 = t2 * t;
		float t4 = t3 * t;
		float t5 = t4 * t;
		float u = 1.f - t;
		s = 10.f * t3 - 15.f * t4 + 6.f * t5;
		dsdt = 30.f * t2 * u * u;
	}
	int32_t span = move_target - move_start;
	int32_t target = move_start + (int32_t)((float)span * s);
	float err = (float)(target - pos) * RAD_PER_COUNT;
	float omega = (float)span * RAD_PER_COUNT * dsdt * (float)PWM_HZ / (float)move;
	ud_cmd = 0.f;
	uq_cmd = clampf(err * POS_KP + TEST_KV * omega, -TEST_UQ_MAX, TEST_UQ_MAX);
	te_from_encoder = true;
	return mode_tick >= move + hold;
}

static void trip_fault(void) {
	aligned = false;
	motor_brake_low();
	enter_mode(MOTOR_FAULT);
}

static void motor_isr(void) {
	enc_ok_this = false;
	mt6701_sample_t s;
	if(mt6701_try_read(&s)) {
		last_raw = s.raw;
		last_pio = s.pio_word;
		last_crc_ok = s.crc_ok;
		if(s.crc_ok) {
			ingest_encoder(s.angle);
			crc_ok++;
			crc_fail_run = 0;
			enc_ok_this = true;
		} else {
			crc_fail++;
			crc_fail_run++;
			if(mode != MOTOR_IDLE && mode != MOTOR_FAULT && crc_fail_run >= CRC_FAIL_TRIP) {
				trip_fault();
				mt6701_start_read();
				return;
			}
		}
	}
	mt6701_start_read();

	if(mode == MOTOR_IDLE || mode == MOTOR_FAULT) {
		// Keep INHx low; wrap IRQ still runs for SSI. No mid-PWM switching.
		return;
	}

	mode_tick++;
	switch(mode) {
		case MOTOR_ALIGN_RAMP: {
			uint32_t ramp = ms_to_ticks(ALIGN_RAMP_MS);
			if(ramp == 0)
				ramp = 1;
			ud_cmd = ALIGN_UD * (float)mode_tick / (float)ramp;
			uq_cmd = 0.f;
			forced_te = 0.f;
			te_from_encoder = false;
			if(mode_tick >= ramp) {
				ud_cmd = ALIGN_UD;
				hold_sum = 0;
				hold_n = 0;
				enter_mode(MOTOR_ALIGN_HOLD);
			}
			break;
		}
		case MOTOR_ALIGN_HOLD: {
			// ponytail: static Ud loses to cogging; drag d-axis 2 elec revs then hold. Current-sense ident later.
			uint32_t drag = ms_to_ticks(ALIGN_DRAG_MS);
			uint32_t hold = ms_to_ticks(ALIGN_HOLD_MS);
			if(drag == 0)
				drag = 1;
			ud_cmd = ALIGN_UD;
			uq_cmd = 0.f;
			te_from_encoder = false;
			if(mode_tick == 1)
				drag_pos0 = pos;
			if(mode_tick <= drag) {
				forced_te = ALIGN_DRAG_ELEC_REVS * (2.f * 3.14159265f) *
						(float)mode_tick / (float)drag;
				if(mode_tick == drag) {
					int32_t dp = pos - drag_pos0;
					if(dp < 0)
						dp = -dp;
					if(dp < (int32_t)ALIGN_DRAG_MIN_COUNTS) {
						trip_fault();
						return;
					}
				}
			} else {
				forced_te = 0.f;
				if(enc_ok_this) {
					hold_sum += pos;
					hold_n++;
				}
			}
			if(mode_tick >= drag + hold) {
				if(hold_n < 8) {
					trip_fault();
					return;
				}
				float avg = (float)hold_sum / (float)hold_n;
				align_theta_m = avg * RAD_PER_COUNT;
				dir = 1;
				offset_rad = -(float)dir * align_theta_m * (float)MOTOR_POLE_PAIRS;
				pulse_pos0 = pos;
				enter_mode(MOTOR_DIR_PULSE);
			}
			break;
		}
		case MOTOR_DIR_PULSE:
			ud_cmd = 0.f;
			uq_cmd = DIR_PULSE_UQ;
			forced_te = 0.f;
			te_from_encoder = false;
			if(mode_tick >= ms_to_ticks(DIR_PULSE_MS)) {
				int32_t dp = pos - pulse_pos0;
				if(dp < -(int32_t)DIR_PULSE_COUNTS)
					dir = -1;
				else if(dp > (int32_t)DIR_PULSE_COUNTS)
					dir = 1;
				else
					dir = -1; // no motion: treat as reversed phase sequence
				offset_rad = -(float)dir * align_theta_m * (float)MOTOR_POLE_PAIRS;
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
			forced_te = 0.f;
			te_from_encoder = false;
			if(mode_tick >= ramp) {
				home_pos = pos;
				rest_pos = pos;
				aligned = true;
				motor_brake_low();
				enter_mode(MOTOR_IDLE);
			}
			break;
		}
		case MOTOR_TEST:
			if(follow_move()) {
				test_fwd = !test_fwd;
				start_move(test_fwd ? home_pos + COUNTS_PER_REV : home_pos, MOTOR_TEST);
			}
			break;
		case MOTOR_SPRING: {
			float vel = vel_update(SPRING_VEL_ALPHA);
			ud_cmd = 0.f;
			uq_cmd = spring_uq(rest_pos - pos, vel, spring_k);
			te_from_encoder = true;
			break;
		}
		case MOTOR_SPIN: {
			ud_cmd = 0.f;
			uq_cmd = spin_uq(vel_update(SPIN_VEL_ALPHA));
			te_from_encoder = true;
			break;
		}
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

void motor_start(void) {
	if(is_aligning())
		return;
	aligned = false;
	crc_fail_run = 0;
	have_enc = false;
	te_from_encoder = false;
	forced_te = 0.f;
	ud_cmd = 0.f;
	uq_cmd = 0.f;
	// Ensure PWM counters are running before first FOC update
	pwm_set_mask_enabled(pwm_enable_mask);
	enter_mode(MOTOR_ALIGN_RAMP);
}

void motor_cmd_stop(void) {
	if(is_aligning()) {
		aligned = false;
	}
	irq_set_enabled(PWM_IRQ_WRAP, false);
	motor_brake_low();
	enter_mode(MOTOR_IDLE);
	irq_set_enabled(PWM_IRQ_WRAP, true);
}

bool motor_cmd_spring(void) {
	if(!aligned || mode == MOTOR_FAULT || is_aligning())
		return false;
	irq_set_enabled(PWM_IRQ_WRAP, false);
	rest_pos = pos;
	pos_last = pos;
	vel_filt = 0.f;
	enter_mode(MOTOR_SPRING);
	irq_set_enabled(PWM_IRQ_WRAP, true);
	return true;
}

bool motor_cmd_spin(void) {
	if(!aligned || mode == MOTOR_FAULT || is_aligning())
		return false;
	irq_set_enabled(PWM_IRQ_WRAP, false);
	pos_last = pos;
	vel_filt = 0.f;
	enter_mode(MOTOR_SPIN);
	irq_set_enabled(PWM_IRQ_WRAP, true);
	return true;
}

bool motor_cmd_test(void) {
	if(!aligned || mode == MOTOR_FAULT || is_aligning())
		return false;
	irq_set_enabled(PWM_IRQ_WRAP, false);
	home_pos = pos;
	test_fwd = true;
	start_move(home_pos + COUNTS_PER_REV, MOTOR_TEST);
	irq_set_enabled(PWM_IRQ_WRAP, true);
	return true;
}

void motor_setup(void) {
	// ponytail: 1024-pt sin LUT (~4KB); cos = lut[i+N/4]. Ceiling: ~0.35°; upgrade = lerp.
	for(uint32_t i = 0; i < SIN_LUT_N; i++)
		sin_lut[i] = sinf((float)i * (TWO_PI / (float)SIN_LUT_N));
	if(!spring_selfcheck())
		LOG_RAW("spring selfcheck FAIL\n");
	if(!spin_selfcheck())
		LOG_RAW("spin selfcheck FAIL\n");

	gpio_init(PIN_DRV_EN);
	gpio_set_dir(PIN_DRV_EN, GPIO_OUT);
	gpio_put(PIN_DRV_EN, 1); // nSLEEP/EN, active high

	gpio_set_function(PIN_PWM_A, GPIO_FUNC_PWM);
	gpio_set_function(PIN_PWM_B, GPIO_FUNC_PWM);
	gpio_set_function(PIN_PWM_C, GPIO_FUNC_PWM);

	pwm_slice_a = pwm_gpio_to_slice_num(PIN_PWM_A);
	pwm_slice_b = pwm_gpio_to_slice_num(PIN_PWM_B);
	pwm_slice_c = pwm_gpio_to_slice_num(PIN_PWM_C);
	pwm_enable_mask = (1u << pwm_slice_a) | (1u << pwm_slice_b) | (1u << pwm_slice_c);

	uint32_t sys = clock_get_hz(clk_sys);
	pwm_wrap = sys / PWM_HZ;
	if(pwm_wrap < 2)
		pwm_wrap = 2;
	pwm_wrap -= 1;

	pwm_config cfg = pwm_get_default_config();
	pwm_config_set_clkdiv(&cfg, 1.f);
	pwm_config_set_wrap(&cfg, (uint16_t)pwm_wrap);
	pwm_init(pwm_slice_a, &cfg, false);
	pwm_init(pwm_slice_b, &cfg, false);
	pwm_init(pwm_slice_c, &cfg, false);

	motor_brake_low();
	mode = MOTOR_IDLE;
	mode_tick = 0;

	pwm_clear_irq(pwm_slice_a);
	pwm_set_irq_enabled(pwm_slice_a, true);
	irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_wrap_isr);
	irq_set_enabled(PWM_IRQ_WRAP, true);

	// Enable counters for SSI pacing; outputs stay low at 0% duty (no FET switching)
	pwm_set_mask_enabled(pwm_enable_mask);
	mt6701_start_read();
}
