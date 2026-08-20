#include "motor.h"
#include "pins.h"
#include "mt6701.h"
#include "platform.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
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
#define ADC_TO_V (CS_VREF / 4095.f)
#define INV_CS_GAIN (1.f / CS_GAIN_V_PER_A)
#define CUR_DT_S ((float)CUR_LOOP_DIV / (float)PWM_HZ)
#define CS_IQ_ALPHA (TWO_PI * CS_IQ_LP_HZ * (float)CUR_LOOP_DIV / (float)PWM_HZ)

// DRV8316 §8.3.11.2 eqs (5)–(7): FOC three-shunt CSA cross-coupling correction.
static const float CSA_C00 = 0.995832f, CSA_C01 = -0.028199f, CSA_C02 = -0.014988f;
static const float CSA_C10 = 0.037737f, CSA_C11 = 1.007723f, CSA_C12 = -0.033757f;
static const float CSA_C20 = 0.009226f, CSA_C21 = 0.029805f, CSA_C22 = 1.003268f;

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
static uint8_t stress_phase;
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
static volatile float duty_a_cached;
static volatile float duty_b_cached;
static volatile float duty_c_cached;
static volatile int32_t track_pos;
static bool enc_ok_this;

static float v_off_a, v_off_b, v_off_c;
static float ia, ib, ic;
static float id_meas, iq_meas;
static float iq_ref_cached;
static float vbus = 12.f;
static float id_i, iq_i;
static float ud_hold, uq_hold;
static uint8_t cur_div;

static void trip_fault(void);
static bool overcurrent(void);
static uint16_t duty_to_level(float d);

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

static float adc_volt(uint input) {
	adc_select_input(input);
	return (float)adc_read() * ADC_TO_V;
}

static void cur_pi_reset(void) {
	id_i = 0.f;
	iq_i = 0.f;
	ud_hold = 0.f;
	uq_hold = 0.f;
	iq_ref_cached = 0.f;
	cur_div = 0;
}

static void csa_correct(float ia_s, float ib_s, float ic_s, float* ia_o, float* ib_o, float* ic_o) {
	*ia_o = CSA_C00 * ia_s + CSA_C01 * ib_s + CSA_C02 * ic_s;
	*ib_o = CSA_C10 * ia_s + CSA_C11 * ib_s + CSA_C12 * ic_s;
	*ic_o = CSA_C20 * ia_s + CSA_C21 * ib_s + CSA_C22 * ic_s;
}

// RP2040 edge PWM: OUT high while ctr < level → LS on (INHx low) when ctr >= level.
// Wait until mid of common low window so all three SOx are valid.
static void wait_lowside_sample_window(void) {
	uint16_t la = duty_to_level(duty_a_cached);
	uint16_t lb = duty_to_level(duty_b_cached);
	uint16_t lc = duty_to_level(duty_c_cached);
	uint16_t hi = la;
	if(lb > hi)
		hi = lb;
	if(lc > hi)
		hi = lc;
	uint16_t span = (uint16_t)(pwm_wrap - hi);
	uint16_t target = (uint16_t)(hi + (span >> 1));
	if(target <= hi && hi < (uint16_t)pwm_wrap)
		target = (uint16_t)(hi + 1u);
	uint32_t guard = (uint32_t)pwm_wrap + 8u;
	while(pwm_get_counter(pwm_slice_a) < target && guard--)
		tight_loop_contents();
}

static void map_phase_currents(float xa, float xb, float xc) {
	xa *= CS_SIGN;
	xb *= CS_SIGN;
	xc *= CS_SIGN;
#if CS_PHASE_ORD == 1
	ia = xa;
	ib = xc;
	ic = xb;
#elif CS_PHASE_ORD == 2
	ia = xb;
	ib = xa;
	ic = xc;
#elif CS_PHASE_ORD == 3
	ia = xb;
	ib = xc;
	ic = xa;
#elif CS_PHASE_ORD == 4
	ia = xc;
	ib = xa;
	ic = xb;
#elif CS_PHASE_ORD == 5
	ia = xc;
	ib = xb;
	ic = xa;
#else
	ia = xa;
	ib = xb;
	ic = xc;
#endif
	// Kill zero-sequence (CSA CM / matched offset error); leftover αβ DC still needs good v_off.
	float cm = (ia + ib + ic) * (1.f / 3.f);
	ia -= cm;
	ib -= cm;
	ic -= cm;
}

// ponytail: sample mid-low after wrap IRQ. Ceiling: busy-wait in ISR (~0–25µs);
// upgrade = phase-correct PWM or DMA triggered from compare.
static void sample_phase_currents(void) {
	wait_lowside_sample_window();
	float va = adc_volt(0);
	float vb = adc_volt(1);
	float vc = adc_volt(2);
	float ia_s = (va - v_off_a) * INV_CS_GAIN;
	float ib_s = (vb - v_off_b) * INV_CS_GAIN;
	float ic_s = (vc - v_off_c) * INV_CS_GAIN;
	float xa, xb, xc;
	csa_correct(ia_s, ib_s, ic_s, &xa, &xb, &xc);
	map_phase_currents(xa, xb, xc);
}

static void sample_vbus_once(void) {
	float vs = adc_volt(3);
	vbus = vs / CS_VBUS_DIV;
	if(vbus < 1.f)
		vbus = 1.f;
}

// Must match runtime path: mid-low window at 50% duty. Unsynced avg left αβ DC → ~2×fe in Id/Iq.
static void calibrate_csa_offset(void) {
	motor_set_duty(0.5f, 0.5f, 0.5f);
	float sa = 0.f, sb = 0.f, sc = 0.f;
	for(uint32_t i = 0; i < 256u; i++) {
		wait_lowside_sample_window();
		sa += adc_volt(0);
		sb += adc_volt(1);
		sc += adc_volt(2);
	}
	v_off_a = sa * (1.f / 256.f);
	v_off_b = sb * (1.f / 256.f);
	v_off_c = sc * (1.f / 256.f);
	id_meas = 0.f;
	iq_meas = 0.f;
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

// (spin_iq_cmd removed — SPIN stays voltage flywheel even with CUR_LOOP_CTRL)

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
	if(SPIN_W_MAX <= SPIN_W_REST)
		return false;
	return true;
}

static bool csa_selfcheck(void) {
	// Coefficient smoke + SOX↔I + TI matrix null-sum on equal sensed currents.
	if(CSA_C00 < 0.99f || CSA_C00 > 1.01f)
		return false;
	if(CSA_C11 < 1.00f || CSA_C11 > 1.02f)
		return false;
	float i = 1.f;
	float v = CS_VREF * 0.5f + CS_GAIN_V_PER_A * i;
	float back = (v - CS_VREF * 0.5f) / CS_GAIN_V_PER_A;
	if(back < 0.99f || back > 1.01f)
		return false;
	float ia_o, ib_o, ic_o;
	csa_correct(0.5f, 0.5f, 0.5f, &ia_o, &ib_o, &ic_o);
	float sum = ia_o + ib_o + ic_o;
	if(sum < 1.4f || sum > 1.6f)
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
	duty_a_cached = a;
	duty_b_cached = b;
	duty_c_cached = c;
	pwm_set_gpio_level(PIN_PWM_A, duty_to_level(a));
	pwm_set_gpio_level(PIN_PWM_B, duty_to_level(b));
	pwm_set_gpio_level(PIN_PWM_C, duty_to_level(c));
}

// ponytail: no INLx Hi-Z. Idle = 50% zero-voltage PWM (~+0.01 W vs LS brake on this board).
static void motor_brake_low(void) {
	ud_cmd = 0.f;
	uq_cmd = 0.f;
	forced_te = 0.f;
	te_from_encoder = false;
	cur_pi_reset();
	motor_set_duty(0.5f, 0.5f, 0.5f);
}

void motor_get_state(motor_state_t* s) {
	s->mode = mode;
	s->pos = pos;
	s->angle_mrad = (int32_t)((float)pos * RAD_PER_COUNT * 1000.f);
	s->offset_mrad = (int32_t)(offset_rad * 1000.f);
	s->dir = dir;
	s->duty_a_q15 = (int16_t)(duty_a_cached * 32767.f);
	s->duty_b_q15 = (int16_t)(duty_b_cached * 32767.f);
	s->duty_c_q15 = (int16_t)(duty_c_cached * 32767.f);
	s->crc_fail = crc_fail;
	s->crc_ok = crc_ok;
	s->raw = last_raw;
	s->pio_word = last_pio;
	s->last_crc_ok = last_crc_ok;
	s->id_a = id_meas;
	s->iq_a = iq_meas;
	s->iq_ref_a = iq_ref_cached;
	s->vbus_v = vbus;
	s->ud_out = ud_hold;
	s->uq_out = uq_hold;
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
#if !(CUR_LOOP_EN && CUR_LOOP_CTRL)
	if(mode == MOTOR_SPIN) {
		if(mag2 < 1.0e-12f) {
			motor_set_duty(0.5f, 0.5f, 0.5f);
			return;
		}
	} else
#endif
	if(mag2 < PWM_MIN_MAG2) {
		motor_set_duty(0.5f, 0.5f, 0.5f);
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

#if CUR_LOOP_EN
static void sense_park(float te) {
	te += CS_TE_OFF;
	float s, c;
	sincos_lut(te, &s, &c);
	float i_alpha = ia;
	float i_beta = (ia + 2.f * ib) * (1.f / 1.73205080757f);
	float id = i_alpha * c + i_beta * s;
	float iq = -i_alpha * s + i_beta * c;
	float a = CS_IQ_ALPHA;
	if(a > 1.f)
		a = 1.f;
	id_meas += a * (id - id_meas);
	iq_meas += a * (iq - iq_meas);
}

// Park/PI in (te+CS_TE_OFF); SVPWM in align te. TE_OFF was tuned so Iq@θ+φ tracks Uq@θ —
// applying u at θ+φ breaks that plant (GOTO regress). Ceiling: residual φ mix on Iq≈0.
#if CUR_LOOP_CTRL
static void apply_current_svpwm(float te, float id_ref, float iq_ref) {
	float te_i = te + CS_TE_OFF;
	float s, c;
	sincos_lut(te_i, &s, &c);
	float i_alpha = ia;
	float i_beta = (ia + 2.f * ib) * (1.f / 1.73205080757f);
	float id = i_alpha * c + i_beta * s;
	float iq = -i_alpha * s + i_beta * c;
	float a = CS_IQ_ALPHA;
	if(a > 1.f)
		a = 1.f;
	id_meas += a * (id - id_meas);
	iq_meas += a * (iq - iq_meas);

	float ed = id_ref - id_meas;
	float eq = iq_ref - iq_meas;
	id_i = clampf(id_i + CUR_KI * ed * CUR_DT_S, -CUR_U_MAX, CUR_U_MAX);
	iq_i = clampf(iq_i + CUR_KI * eq * CUR_DT_S, -CUR_U_MAX, CUR_U_MAX);
	float ud = clampf(CUR_KP * ed + id_i, -CUR_U_MAX, CUR_U_MAX);
	float uq = clampf(CUR_KP * eq + iq_i, -CUR_U_MAX, CUR_U_MAX);
	ud_hold = ud;
	uq_hold = uq;

	sincos_lut(te, &s, &c);
	float mag2 = ud * ud + uq * uq;
	if(mag2 < PWM_MIN_MAG2) {
		motor_set_duty(0.5f, 0.5f, 0.5f);
		return;
	}
	if(mag2 > 1.f) {
		float inv = 1.f / sqrtf(mag2);
		ud *= inv;
		uq *= inv;
	}
	float ualpha = ud * c - uq * s;
	float ubeta = ud * s + uq * c;
	motor_set_duty(0.5f + 0.5f * ualpha,
			0.5f + 0.5f * (-0.5f * ualpha + SQRT3_2 * ubeta),
			0.5f + 0.5f * (-0.5f * ualpha - SQRT3_2 * ubeta));
}

static void apply_hold_svpwm(float te) {
	apply_voltage(ud_hold, uq_hold, te);
}
#endif
#endif

static void apply_cmd(void) {
	float te = forced_te;
	if(te_from_encoder) {
		float th_m = (float)pos * RAD_PER_COUNT;
		te = (float)dir * th_m * (float)MOTOR_POLE_PAIRS + offset_rad;
	}
#if CUR_LOOP_EN
	if(te_from_encoder) {
		float uq = (float)dir * uq_cmd;
		float iq_ref = uq * IQ_CMD_A;
		iq_ref_cached = iq_ref;
		// SPIN: voltage flywheel (current PI + TE_OFF mix makes cogging worse).
		bool spin_volt = (mode == MOTOR_SPIN);
#if CUR_LOOP_CTRL
		if(!spin_volt) {
			if(++cur_div >= CUR_LOOP_DIV) {
				cur_div = 0;
				sample_phase_currents();
				if(overcurrent()) {
					trip_fault();
					return;
				}
				apply_current_svpwm(te, 0.f, iq_ref);
				return;
			}
			apply_hold_svpwm(te);
			return;
		}
#endif
		if(++cur_div >= CUR_LOOP_DIV) {
			cur_div = 0;
			sample_phase_currents();
			if(overcurrent()) {
				trip_fault();
				return;
			}
			sense_park(te);
		}
		ud_hold = ud_cmd;
		uq_hold = uq;
		apply_voltage(ud_cmd, uq, te);
		return;
	}
	iq_ref_cached = 0.f;
	apply_voltage(ud_cmd, uq_cmd, te);
#else
	float uq = te_from_encoder ? (float)dir * uq_cmd : uq_cmd;
	apply_voltage(ud_cmd, uq, te);
#endif
}

static void enter_mode(uint8_t next) {
	mode = next;
	mode_tick = 0;
}

static float smoothstep5(float t) {
	if(t <= 0.f)
		return 0.f;
	if(t >= 1.f)
		return 1.f;
	float t2 = t * t;
	float t3 = t2 * t;
	float t4 = t3 * t;
	float t5 = t4 * t;
	return 10.f * t3 - 15.f * t4 + 6.f * t5;
}

static void stress_advance(uint8_t next) {
	stress_phase = next;
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

static bool overcurrent(void) {
	float aa = ia < 0.f ? -ia : ia;
	float bb = ib < 0.f ? -ib : ib;
	float cc = ic < 0.f ? -ic : ic;
	return aa > I_TRIP_A || bb > I_TRIP_A || cc > I_TRIP_A;
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
					dir = -1;
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
		case MOTOR_POS: {
			float err = (float)(track_pos - pos) * RAD_PER_COUNT;
			float vel = vel_update(SPRING_VEL_ALPHA);
			ud_cmd = 0.f;
			uq_cmd = clampf(err * POS_KP - SPRING_D * vel, -TEST_UQ_MAX, TEST_UQ_MAX);
			te_from_encoder = true;
			break;
		}
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
		case MOTOR_STRESS: {
			ud_cmd = 0.f;
			te_from_encoder = true;
			uint32_t ramp = ms_to_ticks(STRESS_RAMP_MS);
			uint32_t run = ms_to_ticks(STRESS_RUN_MS);
			uint32_t stop = ms_to_ticks(STRESS_STOP_MS);
			if(ramp == 0)
				ramp = 1;
			if(run == 0)
				run = 1;
			if(stop == 0)
				stop = 1;
			switch(stress_phase) {
				case 0: {
					float ss = smoothstep5((float)mode_tick / (float)ramp);
					uq_cmd = STRESS_UQ_MAX * ss;
					if(mode_tick >= ramp)
						stress_advance(1);
					break;
				}
				case 1:
					uq_cmd = STRESS_UQ_MAX;
					if(mode_tick >= run)
						stress_advance(2);
					break;
				case 2: {
					float ss = smoothstep5((float)mode_tick / (float)ramp);
					uq_cmd = STRESS_UQ_MAX * (1.f - ss);
					if(mode_tick >= ramp)
						stress_advance(3);
					break;
				}
				case 3:
					uq_cmd = 0.f;
					if(mode_tick >= stop)
						stress_advance(4);
					break;
				case 4: {
					float ss = smoothstep5((float)mode_tick / (float)ramp);
					uq_cmd = -STRESS_UQ_MAX * ss;
					if(mode_tick >= ramp)
						stress_advance(5);
					break;
				}
				case 5:
					uq_cmd = -STRESS_UQ_MAX;
					if(mode_tick >= run)
						stress_advance(6);
					break;
				case 6: {
					float ss = smoothstep5((float)mode_tick / (float)ramp);
					uq_cmd = -STRESS_UQ_MAX * (1.f - ss);
					if(mode_tick >= ramp)
						stress_advance(7);
					break;
				}
				case 7:
					uq_cmd = 0.f;
					if(mode_tick >= stop)
						stress_advance(0);
					break;
				default:
					stress_advance(0);
					uq_cmd = 0.f;
					break;
			}
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
	cur_pi_reset();
	pwm_set_mask_enabled(pwm_enable_mask);
#if CUR_LOOP_EN
	// IRQ off: synced offset at 50% before Ud injects current.
	irq_set_enabled(PWM_IRQ_WRAP, false);
	calibrate_csa_offset();
	irq_set_enabled(PWM_IRQ_WRAP, true);
#endif
	enter_mode(MOTOR_ALIGN_RAMP);
}

void motor_cmd_stop(void) {
	if(is_aligning()) {
		aligned = false;
	}
	irq_set_enabled(PWM_IRQ_WRAP, false);
	motor_brake_low();
#if CUR_LOOP_EN
	calibrate_csa_offset();
#endif
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
	cur_pi_reset();
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
	cur_pi_reset();
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
	cur_pi_reset();
	start_move(home_pos + COUNTS_PER_REV, MOTOR_TEST);
	irq_set_enabled(PWM_IRQ_WRAP, true);
	return true;
}

bool motor_cmd_stress(void) {
	if(!aligned || mode == MOTOR_FAULT || is_aligning())
		return false;
	irq_set_enabled(PWM_IRQ_WRAP, false);
	stress_phase = 0;
	cur_pi_reset();
	enter_mode(MOTOR_STRESS);
	irq_set_enabled(PWM_IRQ_WRAP, true);
	return true;
}

bool motor_cmd_goto(int32_t angle_mrad) {
	if(!aligned || mode == MOTOR_FAULT || is_aligning())
		return false;
	int32_t target = (int32_t)((float)angle_mrad / (RAD_PER_COUNT * 1000.f));
	irq_set_enabled(PWM_IRQ_WRAP, false);
	track_pos = target;
	if(mode != MOTOR_POS) {
		pos_last = pos;
		vel_filt = 0.f;
		cur_pi_reset();
		enter_mode(MOTOR_POS);
	}
	irq_set_enabled(PWM_IRQ_WRAP, true);
	return true;
}

static bool pos_selfcheck(void) {
	int32_t mrad = (int32_t)((float)COUNTS_PER_REV * RAD_PER_COUNT * 1000.f);
	int32_t back = (int32_t)((float)mrad / (RAD_PER_COUNT * 1000.f));
	int32_t err = back - (int32_t)COUNTS_PER_REV;
	if(err < 0)
		err = -err;
	return err <= 2;
}

static bool stress_selfcheck(void) {
	if(smoothstep5(0.f) != 0.f || smoothstep5(1.f) != 1.f)
		return false;
	if(smoothstep5(0.5f) < 0.4f || smoothstep5(0.5f) > 0.6f)
		return false;
	if(STRESS_UQ_MAX <= 0.f || STRESS_UQ_MAX > 1.f)
		return false;
	if(STRESS_RAMP_MS == 0 || STRESS_RUN_MS == 0 || STRESS_STOP_MS == 0)
		return false;
	return true;
}

void motor_setup(void) {
	for(uint32_t i = 0; i < SIN_LUT_N; i++)
		sin_lut[i] = sinf((float)i * (TWO_PI / (float)SIN_LUT_N));
	if(!spring_selfcheck())
		LOG_RAW("spring selfcheck FAIL\n");
	if(!spin_selfcheck())
		LOG_RAW("spin selfcheck FAIL\n");
	if(!pos_selfcheck())
		LOG_RAW("pos selfcheck FAIL\n");
	if(!stress_selfcheck())
		LOG_RAW("stress selfcheck FAIL\n");
	if(!csa_selfcheck())
		LOG_RAW("csa selfcheck FAIL\n");
#if CUR_LOOP_EN
#if CUR_LOOP_CTRL
	LOG_RAW("cur loop CTRL div=%u (~%u Hz) sign=%.0f ord=%d te_off=%.2f\n",
			(unsigned)CUR_LOOP_DIV, (unsigned)(PWM_HZ / CUR_LOOP_DIV), (double)CS_SIGN,
			(int)CS_PHASE_ORD, (double)CS_TE_OFF);
#else
	LOG_RAW("cur loop SENSE-ONLY div=%u (~%u Hz) sign=%.0f ord=%d te_off=%.2f\n",
			(unsigned)CUR_LOOP_DIV, (unsigned)(PWM_HZ / CUR_LOOP_DIV), (double)CS_SIGN,
			(int)CS_PHASE_ORD, (double)CS_TE_OFF);
#endif
	adc_init();
	adc_gpio_init(PIN_CSA);
	adc_gpio_init(PIN_CSB);
	adc_gpio_init(PIN_CSC);
	adc_gpio_init(PIN_VBUS);
#else
	LOG_RAW("cur loop OFF (light ISR)\n");
#endif

	gpio_init(PIN_DRV_EN);
	gpio_set_dir(PIN_DRV_EN, GPIO_OUT);
	gpio_put(PIN_DRV_EN, 1);

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

	pwm_set_mask_enabled(pwm_enable_mask);
#if CUR_LOOP_EN
	calibrate_csa_offset();
	sample_vbus_once();
	LOG_RAW("csa offset %.3f %.3f %.3f V vbus=%.1f\n", v_off_a, v_off_b, v_off_c, vbus);
#endif

	pwm_clear_irq(pwm_slice_a);
	pwm_set_irq_enabled(pwm_slice_a, true);
	irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_wrap_isr);
	irq_set_enabled(PWM_IRQ_WRAP, true);

	mt6701_start_read();
}
