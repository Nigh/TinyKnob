#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "stm32g4xx.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_tim.h"
#include "tusb.h"
#include "cog_lut_default.h"

#define PWM_HZ 20000u
#define PWM_TOP 3600u
#define MOTOR_POLE_PAIRS 11u
#define TEST_TICKS (PWM_HZ * 4u)
#define PAUSE_TICKS (PWM_HZ / 4u)
#define PHASE_STEP ((uint32_t)(((uint64_t)MOTOR_POLE_PAIRS << 32) / TEST_TICKS))
#define ENCODER_STALL_COUNTS 128
#define ENCODER_STALL_MS 500u
#define CRC_FAIL_TRIP 20u
#define SIN_N 256u
#define TWO_PI 6.28318530718f
#define ADC_SAMPLE_POINT ((PWM_TOP * 94u) / 100u)
#define ADC_TO_V (3.3f / 4095.f)
#define CS_GAIN_V_PER_A 0.3f
#define CS_SIGN (-1.f)
#define I_TRIP_A 4.f
#define DIR_PULSE_TICKS (PWM_HZ * 600u / 1000u)
#define DIR_PULSE_UQ 0.18f
#define DIR_PULSE_COUNTS 80
#define POS_KP 0.6f
#define TEST_KV 0.07f
#define TEST_UQ_MAX 0.65f
#define TEST_MOVE_TICKS (PWM_HZ * 1400u / 1000u)
#define TEST_HOLD_TICKS (PWM_HZ * 200u / 1000u)
#define CUR_KP 0.25f
#define CUR_KI 400.f
#define CUR_KAW 1000.f
#define CUR_U_MAX 0.60f
#define POS_D 0.05f
#define SPRING_K 0.6f
#define SPRING_UQ_MAX 0.35f
#define SPRING_WALL_UQ 0.85f
#define SPRING_WALL_BLEND_RAD 0.45f
#define SPRING_WALL_D 0.15f
#define SPRING_D_UQ_MAX 0.12f
#define SPRING_NEUTRAL_RAD 0.00262f
#define SPRING_SETTLE_TICKS (PWM_HZ * 2u)
#define SPIN_KV 0.018f
#define SPIN_B 0.000075f
#define SPIN_B_HIGH 0.0015f
#define SPIN_W_REST 0.35f
#define SPIN_W_MAX 22.f
#define SPIN_B_CAP 0.30f
#define SPIN_UQ_MAX 0.50f
#define STRESS_UQ_MAX 0.65f
#define STRESS_RAMP_TICKS (PWM_HZ / 2u)
#define STRESS_RUN_TICKS (PWM_HZ * 3u)
#define STRESS_STOP_TICKS PWM_HZ
#define GEAR_TEETH 24u
#define GEAR_UQ 0.30f
#define GEAR_PEAK_FRAC 0.86f
#define GEAR_CLICK_UQ 0.08f
#define GEAR_CLICK_TICKS 1u
#define GEAR_D 0.0008f
#define GEAR_D_UQ_MAX 0.03f
#define GEAR_CAPTURE_MIN_TICKS (PWM_HZ * 250u / 1000u)
#define GEAR_CAPTURE_STABLE_TICKS (PWM_HZ * 120u / 1000u)
#define GEAR_CAPTURE_TIMEOUT_TICKS (PWM_HZ * 1200u / 1000u)
#define GEAR_CAPTURE_W_MAX 0.10f
#define SPIN_IQ_CAP 0.20f

typedef enum {
	STATE_DISABLED,
	STATE_VERIFY,
	STATE_CALIBRATE,
	STATE_ALIGN,
	STATE_DIR_PULSE,
	STATE_READY,
	STATE_TEST_FWD,
	STATE_TEST_REV,
	STATE_SPRING,
	STATE_SPIN,
	STATE_POS,
	STATE_STRESS,
	STATE_GEAR,
	STATE_FAULT,
} test_state_t;

#define DFU_REQUEST_MAGIC 0x44465531u
__attribute__((section(".noinit.dfu"))) static volatile uint32_t dfu_request;

static volatile uint32_t milliseconds;
static volatile test_state_t state;
static volatile uint32_t state_ticks;
static volatile bool usb_online;
static float sin_lut[SIN_N];
static uint32_t enc_raw24;
static int32_t enc_pos;
static uint16_t enc_prev;
static uint32_t crc_ok;
static uint32_t crc_fail;
static uint32_t crc_fail_run;
static uint8_t verify_good;
static bool enc_have;
static int32_t motion_pos0;
static uint32_t motion_ms0;
static char fault_reason[24];
static bool ack_pending;
static uint8_t ack_cmd;
static uint8_t ack_status;
static bool diag_after_ack;
static bool diag_pending;
static int16_t duty_a_q15, duty_b_q15, duty_c_q15;
static volatile uint16_t adc_dma[8];
static volatile bool adc_sample_ready;
static volatile uint32_t adc_sequence;
static float adc_offset[3];
static uint32_t adc_offset_sum[3], adc_offset_n;
static float ia, ib, ic, id_meas, iq_meas, vbus;
static uint32_t electrical_offset_phase;
static int8_t motor_dir = 1;
static int32_t align_pos, pulse_pos;
static int32_t home_pos, move_start, move_target;
static bool test_forward;
static volatile float iq_ref;
static float id_integral, iq_integral, ud_out, uq_out, uq_feedforward;
static bool current_control;
static bool aligned;
static int32_t rest_pos, track_pos;
static bool rest_hold, gear_capture;
static uint32_t gear_capture_stable;
static float spring_k = SPRING_K;
static float spring_cog_scale = 0.1f, spin_cog_scale = 0.65f;
static float track_velocity, velocity;
static uint16_t velocity_prev;
static bool velocity_have;
static uint8_t stress_phase;
static float gear_center, gear_prev_progress, gear_cog_center;
static uint32_t gear_click_ticks;

static void delay_cycles(uint32_t cycles);
static void process_adc_sample(void);

static void outputs_off(void) {
	LL_TIM_DisableAllOutputs(TIM1);
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_4);
}

static void set_duty(float a, float b, float c) {
	if(a < 0.f) a = 0.f; else if(a > 1.f) a = 1.f;
	if(b < 0.f) b = 0.f; else if(b > 1.f) b = 1.f;
	if(c < 0.f) c = 0.f; else if(c > 1.f) c = 1.f;
	duty_a_q15 = (int16_t)(a * 32767.f);
	duty_b_q15 = (int16_t)(b * 32767.f);
	duty_c_q15 = (int16_t)(c * 32767.f);
	LL_TIM_OC_SetCompareCH1(TIM1, (uint32_t)(a * PWM_TOP));
	LL_TIM_OC_SetCompareCH2(TIM1, (uint32_t)(b * PWM_TOP));
	LL_TIM_OC_SetCompareCH3(TIM1, (uint32_t)(c * PWM_TOP));
}

static void apply_voltage(float ud, float uq, uint32_t phase) {
	uint8_t i = (uint8_t)(phase >> 24);
	float s = sin_lut[i];
	float c = sin_lut[(uint8_t)(i + SIN_N / 4u)];
	float alpha = ud * c - uq * s;
	float beta = ud * s + uq * c;
	float va = alpha;
	float vb = -0.5f * alpha + 0.86602540378f * beta;
	float vc = -0.5f * alpha - 0.86602540378f * beta;
	set_duty(0.5f + 0.5f * va, 0.5f + 0.5f * vb, 0.5f + 0.5f * vc);
}

static void outputs_on(void) {
	set_duty(0.5f, 0.5f, 0.5f);
	LL_TIM_SetCounter(TIM1, 0);
	LL_TIM_ClearFlag_UPDATE(TIM1);
	LL_TIM_EnableCounter(TIM1);
	LL_TIM_EnableAllOutputs(TIM1);
	LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_4);
}

static void enter_fault(char const *reason) {
	outputs_off();
	aligned = false;
	current_control = false;
	state = STATE_FAULT;
	strncpy(fault_reason, reason, sizeof(fault_reason) - 1u);
	fault_reason[sizeof(fault_reason) - 1u] = 0;
}

static float clampf(float x, float lo, float hi) {
	return x < lo ? lo : x > hi ? hi : x;
}

static float smoothstep5(float t) {
	t = clampf(t, 0.f, 1.f);
	float t2 = t * t, t3 = t2 * t;
	return 10.f * t3 - 15.f * t3 * t + 6.f * t3 * t2;
}

static uint32_t phase_from_rad(float radians) {
	return (uint32_t)(int64_t)(radians * (4294967296.0f / TWO_PI));
}

static void current_reset(void) {
	iq_ref = 0.f; id_integral = 0.f; iq_integral = 0.f; ud_out = 0.f; uq_out = 0.f; uq_feedforward = 0.f;
}

static uint32_t encoder_electrical_phase(void) {
	int64_t delta = (int64_t)motor_dir * enc_pos * MOTOR_POLE_PAIRS * 262144ll;
	return electrical_offset_phase + (uint32_t)delta;
}

static float cog_command(int32_t position) {
	int32_t wrapped = position % 16384;
	if(wrapped < 0) wrapped += 16384;
	uint32_t i0 = (uint32_t)wrapped / 16u, i1 = (i0 + 1u) & 1023u;
	float fraction = (float)(wrapped & 15) * (1.f / 16.f);
	float value = COG_LUT_FLASH[i0] + (COG_LUT_FLASH[i1] - COG_LUT_FLASH[i0]) * fraction;
	float scale = state == STATE_GEAR ? 1.f : state == STATE_SPRING ? spring_cog_scale : spin_cog_scale;
	return scale * value;
}

static void apply_encoder_voltage(float uq) {
	current_control = false;
	iq_ref = 0.f;
	float cog = cog_command(enc_pos);
	if(state == STATE_GEAR) cog -= gear_cog_center;
	uq_out = (float)motor_dir * (uq + cog);
	apply_voltage(0.f, uq_out, encoder_electrical_phase());
}

static float spring_command(int32_t error_counts) {
	const float pi = 3.14159265f;
	float error = (float)error_counts * (TWO_PI / 16384.f), ae = fabsf(error);
	float spring_error = 0.f, neutral = 0.f;
	if(ae > SPRING_NEUTRAL_RAD) {
		spring_error = copysignf(ae - SPRING_NEUTRAL_RAD, error);
		neutral = clampf((ae - SPRING_NEUTRAL_RAD) / SPRING_NEUTRAL_RAD, 0.f, 1.f);
	}
	float envelope = SPRING_UQ_MAX, damping_gain = 0.f;
	float lo = pi - SPRING_WALL_BLEND_RAD, hi = pi + SPRING_WALL_BLEND_RAD;
	if(ae >= hi) { envelope = SPRING_WALL_UQ; damping_gain = SPRING_WALL_D; }
	else if(ae > lo) {
		float t = (ae - lo) / (hi - lo);
		envelope += t * (SPRING_WALL_UQ - SPRING_UQ_MAX);
		damping_gain = t * SPRING_WALL_D;
	}
	float damping = clampf(damping_gain * neutral * velocity, -SPRING_D_UQ_MAX, SPRING_D_UQ_MAX);
	return clampf(spring_k * spring_error - damping, -envelope, envelope);
}

static float spin_command(void) {
	float av = fabsf(velocity);
	if(av < SPIN_W_REST) return 0.f;
	float sign = velocity < 0.f ? -1.f : 1.f;
	float ramp = clampf((av - SPIN_W_REST) / SPIN_W_REST, 0.f, 1.f);
	float frac = clampf(av / SPIN_W_MAX, 0.f, 1.f);
	float damping = SPIN_B + (SPIN_B_HIGH - SPIN_B) * frac * frac;
	float uq = (SPIN_KV - damping) * velocity * ramp;
	if(av > SPIN_W_MAX) uq -= sign * clampf((av - SPIN_W_MAX) / SPIN_W_MAX, 0.f, 1.f) * SPIN_B_CAP;
	return clampf(uq, -SPIN_UQ_MAX, SPIN_UQ_MAX);
}

static float gear_command(void) {
	float tooth = 16384.f / (float)GEAR_TEETH;
	float x = ((float)enc_pos - gear_center) / tooth;
	int32_t crossed = (int32_t)x;
	if(crossed) { gear_center += (float)crossed * tooth; x = ((float)enc_pos - gear_center) / tooth; gear_prev_progress = 0.f; }
	float progress = fabsf(x) - floorf(fabsf(x));
	if(progress >= GEAR_PEAK_FRAC && gear_prev_progress < GEAR_PEAK_FRAC) gear_click_ticks = GEAR_CLICK_TICKS;
	gear_prev_progress = progress;
	float mag = 0.f;
	if(progress < GEAR_PEAK_FRAC) { float rise = progress / GEAR_PEAK_FRAC; mag = GEAR_UQ * rise * rise * rise; }
	float sign = x < 0.f ? -1.f : 1.f;
	float click = gear_click_ticks ? sign * GEAR_CLICK_UQ : 0.f;
	if(gear_click_ticks) gear_click_ticks--;
	float damping = clampf(GEAR_D * velocity, -GEAR_D_UQ_MAX, GEAR_D_UQ_MAX);
	return clampf(-sign * mag + click - damping, -GEAR_UQ - GEAR_CLICK_UQ - GEAR_D_UQ_MAX, GEAR_UQ + GEAR_CLICK_UQ + GEAR_D_UQ_MAX);
}

void SysTick_Handler(void) { milliseconds++; }
void USB_HP_IRQHandler(void) { tud_int_handler(0); }
void USB_LP_IRQHandler(void) { tud_int_handler(0); }
void DMA1_Channel1_IRQHandler(void) {
	if(LL_DMA_IsActiveFlag_TC1(DMA1)) {
		LL_DMA_ClearFlag_TC1(DMA1);
		adc_sample_ready = true;
		adc_sequence++;
		process_adc_sample();
	}
	if(LL_DMA_IsActiveFlag_TE1(DMA1)) LL_DMA_ClearFlag_TE1(DMA1);
}

void TIM1_UP_TIM16_IRQHandler(void) {
	if(!LL_TIM_IsActiveFlag_UPDATE(TIM1)) return;
	LL_TIM_ClearFlag_UPDATE(TIM1);
	state_ticks++;
	switch(state) {
	case STATE_ALIGN:
		apply_voltage(0.08f, 0.f, 0);
		if(state_ticks >= PWM_HZ / 2u) {
			align_pos = enc_pos;
			pulse_pos = enc_pos;
			state_ticks = 0;
			state = STATE_DIR_PULSE;
		}
		break;
	case STATE_DIR_PULSE:
		apply_voltage(0.f, DIR_PULSE_UQ, 0);
		if(state_ticks >= DIR_PULSE_TICKS) {
			int32_t moved = enc_pos - pulse_pos;
			if(moved < 0) { moved = -moved; motor_dir = -1; }
			else motor_dir = 1;
			if(moved < DIR_PULSE_COUNTS) { enter_fault("align_motion"); break; }
			int64_t phase = (int64_t)motor_dir * align_pos * MOTOR_POLE_PAIRS * 262144ll;
			electrical_offset_phase = (uint32_t)-phase;
			outputs_off(); aligned = true; state_ticks = 0; state = STATE_READY;
		}
		break;
	case STATE_TEST_FWD:
	case STATE_TEST_REV: {
		uint32_t move = TEST_MOVE_TICKS;
		float t = state_ticks < move ? (float)state_ticks / (float)move : 1.f;
		float s5 = smoothstep5(t);
		float dsdt = t > 0.f && t < 1.f ? 30.f * t * t * (1.f - t) * (1.f - t) : 0.f;
		int32_t span = move_target - move_start;
		int32_t target = move_start + (int32_t)((float)span * s5);
		float err = (float)(target - enc_pos) * (TWO_PI / 16384.f);
		float omega = (float)span * (TWO_PI / 16384.f) * dsdt * (float)PWM_HZ / (float)move;
		float outer = clampf(err * POS_KP + TEST_KV * omega, -TEST_UQ_MAX, TEST_UQ_MAX);
		iq_ref = (float)motor_dir * outer;
		current_control = true;
		if(state_ticks >= move + TEST_HOLD_TICKS) {
			test_forward = !test_forward;
			move_start = enc_pos;
			move_target = test_forward ? home_pos + 16384 : home_pos;
			state = test_forward ? STATE_TEST_FWD : STATE_TEST_REV;
			state_ticks = 0;
		}
		break;
	}
	case STATE_POS: {
		float error = (float)(track_pos - enc_pos) * (TWO_PI / 16384.f);
		float outer = clampf(POS_KP * error + POS_D * (track_velocity - velocity), -TEST_UQ_MAX, TEST_UQ_MAX);
		iq_ref = (float)motor_dir * outer; current_control = true;
		break;
	}
	case STATE_SPRING:
		if(state_ticks < SPRING_SETTLE_TICKS) {
			current_control = false; iq_ref = 0.f; uq_out = 0.f; set_duty(0.5f, 0.5f, 0.5f); rest_pos = enc_pos;
		} else apply_encoder_voltage(spring_command(rest_pos - enc_pos));
		break;
	case STATE_SPIN: {
		float av = fabsf(velocity);
		if(av > SPIN_W_MAX) {
			float sign = velocity < 0.f ? -1.f : 1.f;
			float brake = -sign * clampf((av - SPIN_W_MAX) / SPIN_W_MAX, 0.f, 1.f) * SPIN_IQ_CAP;
			iq_ref = (float)motor_dir * (brake + cog_command(enc_pos));
			uq_feedforward = (float)motor_dir * spin_command();
			current_control = true;
		} else apply_encoder_voltage(spin_command());
		break;
	}
	case STATE_STRESS: {
		uint32_t duration = (stress_phase == 1 || stress_phase == 5) ? STRESS_RUN_TICKS :
			(stress_phase == 3 || stress_phase == 7) ? STRESS_STOP_TICKS : STRESS_RAMP_TICKS;
		float t = duration ? (float)state_ticks / (float)duration : 1.f;
		float ss = smoothstep5(t), uq = 0.f;
		if(stress_phase == 0) uq = STRESS_UQ_MAX * ss;
		else if(stress_phase == 1) uq = STRESS_UQ_MAX;
		else if(stress_phase == 2) uq = STRESS_UQ_MAX * (1.f - ss);
		else if(stress_phase == 4) uq = -STRESS_UQ_MAX * ss;
		else if(stress_phase == 5) uq = -STRESS_UQ_MAX;
		else if(stress_phase == 6) uq = -STRESS_UQ_MAX * (1.f - ss);
		apply_encoder_voltage(uq);
		if(state_ticks >= duration) { stress_phase = (stress_phase + 1u) & 7u; state_ticks = 0; }
		break;
	}
	case STATE_GEAR:
		if(gear_capture) {
			current_control = false; iq_ref = 0.f; uq_out = 0.f; set_duty(0.5f, 0.5f, 0.5f); gear_center = (float)enc_pos;
			if(state_ticks >= GEAR_CAPTURE_MIN_TICKS && fabsf(velocity) <= GEAR_CAPTURE_W_MAX) gear_capture_stable++;
			else gear_capture_stable = 0;
			if(gear_capture_stable >= GEAR_CAPTURE_STABLE_TICKS || state_ticks >= GEAR_CAPTURE_TIMEOUT_TICKS) {
				gear_capture = false; gear_center = (float)enc_pos; gear_cog_center = cog_command(enc_pos);
				gear_prev_progress = 0.f; gear_click_ticks = 0;
			}
		} else apply_encoder_voltage(gear_command());
		break;
	default:
		break;
	}
}

static void safe_early_init(void) {
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_4);
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
}

static void reset_bootloader_state(void) {
	__disable_irq();
	SCB->VTOR = FLASH_BASE;
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;
	SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
	for(uint32_t i = 0; i < 8u; i++) {
		NVIC->ICER[i] = 0xffffffffu;
		NVIC->ICPR[i] = 0xffffffffu;
	}
	__DSB();
	__ISB();

	LL_RCC_HSI_Enable();
	while(!LL_RCC_HSI_IsReady()) {}
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
	while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI) {}
	LL_RCC_PLL_Disable();
	while(LL_RCC_PLL_IsReady()) {}
	LL_RCC_HSE_Disable();
	while(LL_RCC_HSE_IsReady()) {}

	LL_AHB1_GRP1_ForceReset(LL_AHB1_GRP1_PERIPH_DMA1 | LL_AHB1_GRP1_PERIPH_DMAMUX1);
	LL_AHB1_GRP1_ReleaseReset(LL_AHB1_GRP1_PERIPH_DMA1 | LL_AHB1_GRP1_PERIPH_DMAMUX1);
	LL_AHB2_GRP1_ForceReset(LL_AHB2_GRP1_PERIPH_ADC12);
	LL_AHB2_GRP1_ReleaseReset(LL_AHB2_GRP1_PERIPH_ADC12);
	LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_USB);
	LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_USB);
	LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_TIM1);
	LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_TIM1);
}

static void clock_init(void) {
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
	LL_PWR_EnableRange1BoostMode();
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
	LL_RCC_HSE_Enable();
	while(!LL_RCC_HSE_IsReady()) {}
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_2, 72, LL_RCC_PLLR_DIV_2);
	LL_RCC_PLL_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_2, 72, LL_RCC_PLLQ_DIV_6);
	LL_RCC_PLL_Enable();
	while(!LL_RCC_PLL_IsReady()) {}
	LL_RCC_PLL_EnableDomain_SYS();
	LL_RCC_PLL_EnableDomain_48M();
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
	while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {}
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
	SystemCoreClock = 144000000u;
	LL_RCC_SetUSBClockSource(LL_RCC_USB_CLKSOURCE_PLL);
	SysTick_Config(SystemCoreClock / 1000u);
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void gpio_init(void) {
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA | LL_AHB2_GRP1_PERIPH_GPIOC);
	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_4 | LL_GPIO_PIN_6);
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_4 | LL_GPIO_PIN_6, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_8, LL_GPIO_AF_6);
	LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_9, LL_GPIO_AF_6);
	LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_10, LL_GPIO_AF_6);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_8, LL_GPIO_SPEED_FREQ_VERY_HIGH);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_VERY_HIGH);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_VERY_HIGH);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_6, LL_GPIO_PULL_UP);
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4 | LL_GPIO_PIN_5);
}

static void timer_init(void) {
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
	LL_TIM_SetPrescaler(TIM1, 0);
	LL_TIM_SetAutoReload(TIM1, PWM_TOP - 1u);
	LL_TIM_SetCounterMode(TIM1, LL_TIM_COUNTERMODE_CENTER_UP_DOWN);
	LL_TIM_SetRepetitionCounter(TIM1, 1u);
	LL_TIM_EnableARRPreload(TIM1);
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH4);
	LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM2);
	LL_TIM_OC_SetCompareCH4(TIM1, ADC_SAMPLE_POINT);
	LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_OC4);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3);
	LL_TIM_GenerateEvent_UPDATE(TIM1);
	LL_TIM_ClearFlag_UPDATE(TIM1);
	LL_TIM_EnableIT_UPDATE(TIM1);
	NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 3);
	NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
	LL_TIM_EnableCounter(TIM1);
}

static void adc_dma_init(void) {
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);
	LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSOURCE_SYSCLK);
	for(uint32_t pin = LL_GPIO_PIN_0; pin <= LL_GPIO_PIN_3; pin <<= 1)
		LL_GPIO_SetPinMode(GPIOA, pin, LL_GPIO_MODE_ANALOG);
	LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_SYNC_PCLK_DIV4);
	LL_ADC_DisableDeepPowerDown(ADC1);
	LL_ADC_EnableInternalRegulator(ADC1);
	delay_cycles(SystemCoreClock / 50000u);
	LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
	while(LL_ADC_IsCalibrationOnGoing(ADC1)) {}
	LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS);
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_2);
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_3);
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_4, LL_ADC_CHANNEL_4);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_12CYCLES_5);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_12CYCLES_5);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_12CYCLES_5);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_12CYCLES_5);
	LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_EXT_TIM1_TRGO2);
	LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
	LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
	LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_ADC1);
	LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
		LL_DMA_MODE_CIRCULAR | LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT |
		LL_DMA_PDATAALIGN_HALFWORD | LL_DMA_MDATAALIGN_HALFWORD | LL_DMA_PRIORITY_HIGH);
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1,
		LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA), (uint32_t)adc_dma,
		LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 8);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);
	NVIC_SetPriority(DMA1_Channel1_IRQn, 2);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
	LL_ADC_Enable(ADC1);
	while(!LL_ADC_IsActiveFlag_ADRDY(ADC1)) {}
	LL_ADC_REG_StartConversion(ADC1);
}

static void usb_init(void) {
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USB);
	NVIC_SetPriority(USB_HP_IRQn, 1);
	NVIC_SetPriority(USB_LP_IRQn, 1);
	NVIC_EnableIRQ(USB_HP_IRQn);
	NVIC_EnableIRQ(USB_LP_IRQn);
	tusb_init();
}

static void delay_cycles(uint32_t cycles) {
	uint32_t start = DWT->CYCCNT;
	while((uint32_t)(DWT->CYCCNT - start) < cycles) {}
}

static uint8_t crc6(uint32_t data18) {
	uint8_t crc = 0;
	for(int i = 0; i < 18; i++) {
		uint8_t bit = (uint8_t)(((data18 >> (17 - i)) & 1u) ^ ((crc >> 5) & 1u));
		crc = (uint8_t)((crc << 1) & 0x3fu);
		if(bit) crc ^= 0x03u;
	}
	return crc;
}

static bool encoder_read(void) {
	uint32_t word = 0;
	uint32_t half = SystemCoreClock / 250000u;
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);
	delay_cycles(half * 2u);
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
	delay_cycles(half);
	for(uint8_t i = 0; i < 23; i++) {
		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);
		delay_cycles(half);
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
		word = (word << 1) | (LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_6) ? 1u : 0u);
		delay_cycles(half);
	}
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);
	delay_cycles(half * 2u);
	word = (word << 1) | (LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_6) ? 1u : 0u);
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
	enc_raw24 = word;
	if(crc6(word >> 6) != (word & 0x3fu)) {
		crc_fail++; crc_fail_run++;
		if(crc_fail_run >= CRC_FAIL_TRIP && state != STATE_DISABLED && state != STATE_FAULT) enter_fault("encoder_crc");
		return false;
	}
	crc_ok++; crc_fail_run = 0;
	uint16_t raw = (uint16_t)((word >> 10) & 0x3fffu);
	if(enc_have) {
		int32_t d = (int32_t)raw - enc_prev;
		if(d > 8192) d -= 16384;
		if(d < -8192) d += 16384;
		enc_pos += d;
	} else { enc_pos = raw; enc_have = true; }
	if(velocity_have) {
		int32_t vd = (int32_t)raw - velocity_prev;
		if(vd > 8192) vd -= 16384;
		if(vd < -8192) vd += 16384;
		float instant = (float)vd * (TWO_PI / 16384.f) * 1000.f;
		float alpha = state == STATE_SPRING ? 0.31415927f : state == STATE_GEAR ? 0.50265482f : 0.75398224f;
		velocity += clampf(alpha, 0.f, 1.f) * (instant - velocity);
	} else velocity_have = true;
	velocity_prev = raw;
	enc_prev = raw;
	return true;
}

static int16_t to_i16(float value) {
	if(value > 32767.f) return 32767;
	if(value < -32768.f) return -32768;
	return (int16_t)value;
}

static void process_adc_sample(void) {
	if(!adc_sample_ready) return;
	adc_sample_ready = false;
	/* DMA TC fires after two four-channel scans: use the newest complete scan. */
	uint16_t raw[4] = {adc_dma[4], adc_dma[5], adc_dma[6], adc_dma[7]};
	if(state == STATE_CALIBRATE) {
		for(uint32_t i = 0; i < 3; i++) adc_offset_sum[i] += raw[i];
		if(++adc_offset_n >= 256u) {
			for(uint32_t i = 0; i < 3; i++) adc_offset[i] = (float)adc_offset_sum[i] / 256.f;
			state = STATE_ALIGN; state_ticks = 0;
		}
		return;
	}
	if(adc_offset_n < 256u) return;
	float sa = ((float)raw[0] - adc_offset[0]) * ADC_TO_V / CS_GAIN_V_PER_A;
	float sb = ((float)raw[1] - adc_offset[1]) * ADC_TO_V / CS_GAIN_V_PER_A;
	float sc = ((float)raw[2] - adc_offset[2]) * ADC_TO_V / CS_GAIN_V_PER_A;
	/* DRV8316 datasheet three-shunt cross-coupling correction. */
	float ca = 0.995832f * sa - 0.028199f * sb - 0.014988f * sc;
	float cb = 0.037737f * sa + 1.007723f * sb - 0.033757f * sc;
	float cc = 0.009226f * sa + 0.029805f * sb + 1.003268f * sc;
	/* Match the RP2350 validated CS_SIGN=-1, CS_PHASE_ORD=4 (CAB). */
	ia = -cc; ib = -ca; ic = -cb;
	float common = (ia + ib + ic) * (1.f / 3.f);
	ia -= common; ib -= common; ic -= common;
	vbus = (float)raw[3] * ADC_TO_V * 10.f;
	uint32_t park_phase = encoder_electrical_phase() + phase_from_rad(-0.70f);
	uint8_t index = (uint8_t)(park_phase >> 24);
	float sn = sin_lut[index], cs = sin_lut[(uint8_t)(index + 64u)];
	float alpha = ia;
	float beta = (ia + 2.f * ib) * 0.57735026919f;
	float id = alpha * cs + beta * sn;
	float iq = -alpha * sn + beta * cs;
	const float lp = 0.04712389f; /* 150 Hz at 20 kHz sampling. */
	id_meas += lp * (id - id_meas);
	iq_meas += lp * (iq - iq_meas);
	if(fabsf(ia) > I_TRIP_A || fabsf(ib) > I_TRIP_A || fabsf(ic) > I_TRIP_A) {
		enter_fault("overcurrent"); return;
	}
	if(current_control) {
		const float dt = 2.f / (float)PWM_HZ;
		float ed = -id, eq = iq_ref - iq;
		id_integral += CUR_KI * ed * dt;
		iq_integral += CUR_KI * eq * dt;
		float ud_raw = CUR_KP * ed + id_integral;
		float uq_raw = CUR_KP * eq + iq_integral + uq_feedforward;
		float mag2 = ud_raw * ud_raw + uq_raw * uq_raw;
		ud_out = ud_raw; uq_out = uq_raw;
		if(mag2 > CUR_U_MAX * CUR_U_MAX) {
			float scale = CUR_U_MAX / sqrtf(mag2);
			ud_out *= scale; uq_out *= scale;
		}
		id_integral = clampf(id_integral + CUR_KAW * (ud_out - ud_raw) * dt, -CUR_U_MAX, CUR_U_MAX);
		iq_integral = clampf(iq_integral + CUR_KAW * (uq_out - uq_raw) * dt, -CUR_U_MAX, CUR_U_MAX);
		apply_voltage(ud_out, uq_out, encoder_electrical_phase());
	}
}

static uint8_t protocol_mode(void) {
	if(state == STATE_FAULT) return 8;
	if(state == STATE_SPRING) return 6;
	if(state == STATE_SPIN) return 7;
	if(state == STATE_POS) return 9;
	if(state == STATE_STRESS) return 10;
	if(state == STATE_GEAR) return 12;
	if(state == STATE_TEST_FWD || state == STATE_TEST_REV)
		return 5;
	if(state == STATE_DIR_PULSE) return 3;
	if(state == STATE_VERIFY || state == STATE_CALIBRATE || state == STATE_ALIGN)
		return 2;
	return 0;
}

static bool vendor_ack(uint8_t cmd, uint8_t status) {
	if(!tud_vendor_mounted() || tud_vendor_write_available() < 3) return false;
	uint8_t packet[3] = {0x5a, cmd, status};
	tud_vendor_write(packet, sizeof(packet));
	tud_vendor_write_flush();
	return true;
}

static uint8_t fault_code(void) {
	if(strcmp(fault_reason, "crc_selfcheck") == 0) return 1;
	if(strcmp(fault_reason, "encoder_crc") == 0) return 2;
	if(strcmp(fault_reason, "encoder_stall") == 0) return 3;
	if(strcmp(fault_reason, "overcurrent") == 0) return 4;
	if(strcmp(fault_reason, "align_motion") == 0) return 5;
	if(strcmp(fault_reason, "adc_stale") == 0) return 6;
	return 0;
}

static void put_u32_le(uint8_t *p, uint32_t value) {
	p[0] = (uint8_t)value; p[1] = (uint8_t)(value >> 8);
	p[2] = (uint8_t)(value >> 16); p[3] = (uint8_t)(value >> 24);
}

static bool vendor_diag(void) {
	if(!tud_vendor_mounted() || tud_vendor_write_available() < 19) return false;
	uint8_t p[19] = {0x5c, 1, protocol_mode(), fault_code()};
	p[4] = (uint8_t)(enc_raw24 >> 16); p[5] = (uint8_t)(enc_raw24 >> 8); p[6] = (uint8_t)enc_raw24;
	put_u32_le(&p[7], crc_ok); put_u32_le(&p[11], crc_fail); put_u32_le(&p[15], crc_fail_run);
	tud_vendor_write(p, sizeof(p));
	tud_vendor_write_flush();
	return true;
}

__attribute__((noreturn)) static void jump_system_dfu(void) {
	typedef void (*entry_fn_t)(void);
	const uint32_t system_memory = 0x1fff0000u;
	uint32_t stack = *(uint32_t *)system_memory;
	entry_fn_t entry = (entry_fn_t)*(uint32_t *)(system_memory + 4u);
	__disable_irq();
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;
	SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
	for(uint32_t i = 0; i < 8u; i++) {
		NVIC->ICER[i] = 0xffffffffu;
		NVIC->ICPR[i] = 0xffffffffu;
	}
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
	LL_SYSCFG_SetRemapMemory(LL_SYSCFG_REMAP_SYSTEMFLASH);
	SCB->VTOR = 0;
	__set_CONTROL(0);
	__set_MSP(stack);
	__DSB();
	__ISB();
	__enable_irq();
	entry();
	for(;;) {}
}

__attribute__((noreturn)) static void request_system_dfu(void) {
	outputs_off();
	tud_disconnect();
	delay_cycles(SystemCoreClock / 100u);
	dfu_request = DFU_REQUEST_MAGIC;
	__DSB();
	NVIC_SystemReset();
	for(;;) {}
}

static bool mode_allowed(void) { return aligned && state != STATE_FAULT && state != STATE_VERIFY && state != STATE_CALIBRATE && state != STATE_ALIGN && state != STATE_DIR_PULSE; }

static int32_t get_i32_le(uint8_t const *p) {
	return (int32_t)((uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24);
}

static uint8_t vendor_command(uint8_t const *packet, uint32_t n) {
	uint8_t cmd = packet[0];
	if(cmd == 0x02) {
		outputs_off(); current_control = false; current_reset(); state = aligned ? STATE_READY : STATE_DISABLED; fault_reason[0] = 0;
		return 1;
	}
	if(cmd == 0x01) {
		outputs_off(); aligned = false; current_control = false; current_reset(); verify_good = 0; fault_reason[0] = 0;
		adc_offset_n = 0; memset(adc_offset_sum, 0, sizeof(adc_offset_sum)); state = STATE_VERIFY;
		return 1;
	}
	if(cmd == 0x30) return 1;
	if(!mode_allowed()) return 0;
	if(cmd == 0x03) {
		bool capture = !rest_hold;
		if(capture) rest_pos = enc_pos;
		rest_hold = false; state_ticks = capture ? 0u : SPRING_SETTLE_TICKS;
		state = STATE_SPRING; outputs_on(); return 1;
	}
	if(cmd == 0x04) { state_ticks = 0; state = STATE_SPIN; outputs_on(); return 1; }
	if(cmd == 0x05) {
		home_pos = enc_pos; move_start = enc_pos; move_target = enc_pos + 16384;
		test_forward = true; current_reset(); current_control = true; state = STATE_TEST_FWD; state_ticks = 0;
		motion_pos0 = enc_pos; motion_ms0 = milliseconds; outputs_on(); return 1;
	}
	if(cmd == 0x06 && n >= 5u) {
		track_pos = (int32_t)((float)get_i32_le(&packet[1]) / ((TWO_PI / 16384.f) * 1000.f));
		track_velocity = n >= 9u ? (float)get_i32_le(&packet[5]) / 1000.f : 0.f;
		current_reset(); current_control = true; state = STATE_POS; state_ticks = 0; outputs_on(); return 1;
	}
	if(cmd == 0x07) { stress_phase = 0; state_ticks = 0; state = STATE_STRESS; outputs_on(); return 1; }
	if(cmd == 0x0a) {
		state = STATE_GEAR; gear_center = (float)enc_pos; gear_cog_center = cog_command(enc_pos); gear_prev_progress = 0.f;
		gear_click_ticks = 0; gear_capture = true; gear_capture_stable = 0; state_ticks = 0;
		outputs_on(); return 1;
	}
	if(cmd == 0x20) { if(n >= 2u) spring_k = clampf((float)packet[1] / 10.f, 0.f, 8.f); return 1; }
	if(cmd == 0x21) { rest_pos = enc_pos; rest_hold = true; return 1; }
	if(cmd == 0x22 && n >= 3u) {
		float scale = clampf((float)((uint16_t)packet[1] | (uint16_t)packet[2] << 8) / 1000.f, 0.f, 2.f);
		if(state == STATE_SPRING) spring_cog_scale = scale;
		else if(state == STATE_SPIN) spin_cog_scale = scale;
		else return 0;
		return 1;
	}
	return 0;
}

static void vendor_telem(void) {
	static uint16_t seq;
	if(!tud_vendor_mounted() || tud_vendor_write_available() < 25) return;
	int32_t angle_mrad = (int32_t)(((int64_t)enc_pos * 6283) / 16384);
	int16_t uq_q15 = to_i16(uq_out * 32767.f);
	int8_t dir = motor_dir;
	uint8_t p[25] = {0};
	p[0] = 0xa5; p[1] = protocol_mode();
	p[2] = (uint8_t)angle_mrad; p[3] = (uint8_t)(angle_mrad >> 8);
	p[4] = (uint8_t)(angle_mrad >> 16); p[5] = (uint8_t)(angle_mrad >> 24);
	p[6] = (uint8_t)duty_a_q15; p[7] = (uint8_t)(duty_a_q15 >> 8);
	p[8] = (uint8_t)duty_b_q15; p[9] = (uint8_t)(duty_b_q15 >> 8);
	p[10] = (uint8_t)duty_c_q15; p[11] = (uint8_t)(duty_c_q15 >> 8);
	p[12] = (uint8_t)seq; p[13] = (uint8_t)(seq >> 8);
	int16_t id_ma = to_i16(id_meas * 1000.f), iq_ma = to_i16(iq_meas * 1000.f);
	uint16_t vbus_mv = vbus <= 0.f ? 0u : vbus >= 65.535f ? 65535u : (uint16_t)(vbus * 1000.f);
	p[14] = (uint8_t)id_ma; p[15] = (uint8_t)(id_ma >> 8);
	p[16] = (uint8_t)iq_ma; p[17] = (uint8_t)(iq_ma >> 8);
	int16_t iq_ref_ma = to_i16(iq_ref * 1000.f);
	p[18] = (uint8_t)iq_ref_ma; p[19] = (uint8_t)(iq_ref_ma >> 8);
	p[20] = (uint8_t)vbus_mv; p[21] = (uint8_t)(vbus_mv >> 8);
	p[22] = (uint8_t)uq_q15; p[23] = (uint8_t)(uq_q15 >> 8); p[24] = (uint8_t)dir;
	seq++;
	tud_vendor_write(p, sizeof(p));
	tud_vendor_write_flush();
}

static void vendor_task(void) {
	if(ack_pending) {
		if(vendor_ack(ack_cmd, ack_status)) {
			ack_pending = false;
			if(diag_after_ack) { diag_after_ack = false; diag_pending = true; }
		}
		return;
	}
	if(diag_pending) {
		if(vendor_diag()) diag_pending = false;
		return;
	}
	if(tud_vendor_available()) {
		uint8_t packet[64];
		uint32_t n = tud_vendor_read(packet, sizeof(packet));
		if(n) tud_vendor_write_clear();
		if(n && packet[0] == 0x7f) request_system_dfu();
		if(n) {
			ack_cmd = packet[0];
			ack_status = vendor_command(packet, n);
			if(ack_cmd == 0x06 && ack_status) { ack_pending = false; return; }
			diag_after_ack = ack_cmd == 0x30 && ack_status;
			ack_pending = true;
		}
		return;
	}
	static uint32_t last_ms;
	uint32_t now = milliseconds;
	if(now != last_ms) { last_ms = now; vendor_telem(); }
}

void tud_mount_cb(void) { usb_online = true; }
void tud_umount_cb(void) { usb_online = false; outputs_off(); state = STATE_DISABLED; }
void tud_suspend_cb(bool remote_wakeup_en) { (void)remote_wakeup_en; usb_online = false; outputs_off(); state = STATE_DISABLED; }
void tud_resume_cb(void) { usb_online = true; }

int main(void) {
	if(dfu_request == DFU_REQUEST_MAGIC) {
		dfu_request = 0;
		__DSB();
		jump_system_dfu();
	}
	reset_bootloader_state();
	safe_early_init();
	clock_init();
	gpio_init();
	outputs_off();
	for(uint32_t i = 0; i < SIN_N; i++) sin_lut[i] = sinf((float)i * TWO_PI / SIN_N);
	timer_init();
	adc_dma_init();
	usb_init();
	__enable_irq();
	if(crc6(0) != 0 || crc6(1) != 0x03 || crc6(0x3ffffu) != 0x0e)
		enter_fault("crc_selfcheck");
	uint32_t sample_ms = 0;
	uint32_t led_ms = 0;
	uint32_t adc_seen = adc_sequence;
	uint8_t adc_stale_ms = 0;
	for(;;) {
		tud_task();
		vendor_task();
		uint32_t now = milliseconds;
		if(now != sample_ms) {
			sample_ms = now;
			uint32_t sequence = adc_sequence;
			if(sequence != adc_seen) { adc_seen = sequence; adc_stale_ms = 0; }
			else if(LL_GPIO_IsOutputPinSet(GPIOC, LL_GPIO_PIN_4) && ++adc_stale_ms >= 2u) enter_fault("adc_stale");
			bool good = encoder_read();
			if(state == STATE_VERIFY && good && ++verify_good >= 8u) {
				state = STATE_CALIBRATE; state_ticks = 0; outputs_on();
			}
			if((state == STATE_TEST_FWD || state == STATE_TEST_REV) && state_ticks < TEST_MOVE_TICKS &&
				now - motion_ms0 >= ENCODER_STALL_MS) {
				int32_t moved = enc_pos - motion_pos0; if(moved < 0) moved = -moved;
				if(moved < ENCODER_STALL_COUNTS) enter_fault("encoder_stall");
				else { motion_pos0 = enc_pos; motion_ms0 = now; }
			}
		}
		if(now - led_ms >= 125u) {
			led_ms = now;
			bool on = state == STATE_FAULT ? ((now / 125u) & 1u) : state == STATE_READY || state == STATE_TEST_FWD || state == STATE_TEST_REV;
			if(on) LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_6); else LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_6);
		}
		if(!usb_online && state != STATE_DISABLED) { outputs_off(); state = STATE_DISABLED; }
		__WFI();
	}
}
