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

typedef enum {
	STATE_DISABLED,
	STATE_VERIFY,
	STATE_ALIGN,
	STATE_READY,
	STATE_TEST_ALIGN,
	STATE_TEST_FWD,
	STATE_TEST_PAUSE_FWD,
	STATE_TEST_REV,
	STATE_TEST_PAUSE_REV,
	STATE_FAULT,
} test_state_t;

static volatile uint32_t milliseconds;
static volatile test_state_t state;
static volatile uint32_t state_ticks;
static volatile uint32_t phase_q32;
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
static volatile uint16_t adc_dma[4];
static volatile bool adc_sample_ready;

static void delay_cycles(uint32_t cycles);

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
	state = STATE_FAULT;
	strncpy(fault_reason, reason, sizeof(fault_reason) - 1u);
	fault_reason[sizeof(fault_reason) - 1u] = 0;
}

void SysTick_Handler(void) { milliseconds++; }
void USB_HP_IRQHandler(void) { tud_int_handler(0); }
void USB_LP_IRQHandler(void) { tud_int_handler(0); }
void DMA1_Channel1_IRQHandler(void) {
	if(LL_DMA_IsActiveFlag_TC1(DMA1)) {
		LL_DMA_ClearFlag_TC1(DMA1);
		adc_sample_ready = true;
	}
	if(LL_DMA_IsActiveFlag_TE1(DMA1)) LL_DMA_ClearFlag_TE1(DMA1);
}

void TIM1_UP_TIM16_IRQHandler(void) {
	if(!LL_TIM_IsActiveFlag_UPDATE(TIM1)) return;
	LL_TIM_ClearFlag_UPDATE(TIM1);
	state_ticks++;
	switch(state) {
	case STATE_ALIGN:
	case STATE_TEST_ALIGN:
		apply_voltage(0.08f, 0.f, 0);
		if(state_ticks >= PWM_HZ / 2u) {
			outputs_off();
			state_ticks = 0;
			state = state == STATE_ALIGN ? STATE_READY : STATE_TEST_FWD;
			if(state == STATE_TEST_FWD) {
				phase_q32 = 0;
				motion_pos0 = enc_pos;
				motion_ms0 = milliseconds;
				outputs_on();
			}
		}
		break;
	case STATE_TEST_FWD:
		phase_q32 += PHASE_STEP;
		apply_voltage(0.f, 0.12f, phase_q32);
		if(state_ticks >= TEST_TICKS) { state = STATE_TEST_PAUSE_FWD; state_ticks = 0; set_duty(.5f,.5f,.5f); }
		break;
	case STATE_TEST_PAUSE_FWD:
		if(state_ticks >= PAUSE_TICKS) { state = STATE_TEST_REV; state_ticks = 0; motion_pos0 = enc_pos; motion_ms0 = milliseconds; }
		break;
	case STATE_TEST_REV:
		phase_q32 -= PHASE_STEP;
		apply_voltage(0.f, 0.12f, phase_q32);
		if(state_ticks >= TEST_TICKS) { state = STATE_TEST_PAUSE_REV; state_ticks = 0; set_duty(.5f,.5f,.5f); }
		break;
	case STATE_TEST_PAUSE_REV:
		if(state_ticks >= PAUSE_TICKS) { state = STATE_TEST_FWD; state_ticks = 0; motion_pos0 = enc_pos; motion_ms0 = milliseconds; }
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
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 4);
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
	enc_prev = raw;
	return true;
}

static uint8_t protocol_mode(void) {
	if(state == STATE_FAULT) return 8;
	if(state == STATE_TEST_FWD || state == STATE_TEST_PAUSE_FWD ||
			state == STATE_TEST_REV || state == STATE_TEST_PAUSE_REV)
		return 5;
	if(state == STATE_VERIFY || state == STATE_ALIGN || state == STATE_TEST_ALIGN)
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

__attribute__((noreturn)) static void enter_system_dfu(void) {
	typedef void (*entry_fn_t)(void);
	const uint32_t system_memory = 0x1fff0000u;
	outputs_off();
	tud_disconnect();
	delay_cycles(SystemCoreClock / 100u);
	__disable_irq();
	SysTick->CTRL = 0;
	for(uint32_t i = 0; i < 8; i++) {
		NVIC->ICER[i] = 0xffffffffu;
		NVIC->ICPR[i] = 0xffffffffu;
	}
	LL_RCC_HSI_Enable();
	while(!LL_RCC_HSI_IsReady()) {}
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
	while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI) {}
	LL_RCC_PLL_Disable();
	SCB->VTOR = system_memory;
	__set_MSP(*(uint32_t *)system_memory);
	((entry_fn_t)*(uint32_t *)(system_memory + 4u))();
	for(;;) {}
}

static uint8_t vendor_command(uint8_t cmd) {
	if(cmd == 0x02) {
		outputs_off(); state = STATE_DISABLED; fault_reason[0] = 0;
		return 1;
	}
	if(cmd == 0x01) {
		outputs_off(); verify_good = 0; fault_reason[0] = 0; state = STATE_VERIFY;
		return 1;
	}
	if(cmd == 0x05) {
		if(state != STATE_READY) return 0;
		state = STATE_TEST_ALIGN; state_ticks = 0; outputs_on();
		return 1;
	}
	if(cmd == 0x30) return 1;
	return 0;
}

static void vendor_telem(void) {
	static uint16_t seq;
	if(!tud_vendor_mounted() || tud_vendor_write_available() < 25) return;
	int32_t angle_mrad = (int32_t)(((int64_t)enc_pos * 6283) / 16384);
	int16_t uq_q15 = state == STATE_TEST_FWD ? 3932 : state == STATE_TEST_REV ? -3932 : 0;
	int8_t dir = state == STATE_TEST_REV ? -1 : 1;
	uint8_t p[25] = {0};
	p[0] = 0xa5; p[1] = protocol_mode();
	p[2] = (uint8_t)angle_mrad; p[3] = (uint8_t)(angle_mrad >> 8);
	p[4] = (uint8_t)(angle_mrad >> 16); p[5] = (uint8_t)(angle_mrad >> 24);
	p[6] = (uint8_t)duty_a_q15; p[7] = (uint8_t)(duty_a_q15 >> 8);
	p[8] = (uint8_t)duty_b_q15; p[9] = (uint8_t)(duty_b_q15 >> 8);
	p[10] = (uint8_t)duty_c_q15; p[11] = (uint8_t)(duty_c_q15 >> 8);
	p[12] = (uint8_t)seq; p[13] = (uint8_t)(seq >> 8);
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
		if(n && packet[0] == 0x7f) enter_system_dfu();
		if(n) {
			ack_cmd = packet[0];
			ack_status = vendor_command(ack_cmd);
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
	safe_early_init();
	clock_init();
	gpio_init();
	outputs_off();
	timer_init();
	adc_dma_init();
	for(uint32_t i = 0; i < SIN_N; i++) sin_lut[i] = sinf((float)i * TWO_PI / SIN_N);
	usb_init();
	if(crc6(0) != 0 || crc6(1) != 0x03 || crc6(0x3ffffu) != 0x0e)
		enter_fault("crc_selfcheck");
	uint32_t sample_ms = 0;
	uint32_t led_ms = 0;
	for(;;) {
		tud_task();
		vendor_task();
		uint32_t now = milliseconds;
		if(now != sample_ms) {
			sample_ms = now;
			bool good = encoder_read();
			if(state == STATE_VERIFY && good && ++verify_good >= 8u) { state = STATE_ALIGN; state_ticks = 0; outputs_on();  }
			if((state == STATE_TEST_FWD || state == STATE_TEST_REV) && now - motion_ms0 >= ENCODER_STALL_MS) {
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
