#ifndef _PINS_H_
#define _PINS_H_

// RP2040-Zero — TinyKnob

#define PIN_PWM_A 2
#define PIN_PWM_B 4
#define PIN_PWM_C 6
#define PIN_DRV_EN 8

#define PIN_MT6701_CSN 9
#define PIN_MT6701_CLK 10
#define PIN_MT6701_DO 11

// ADC: CSA/CSB/CSC + 0.1×VCC (DRV8316 SOx / bus divider)
#define PIN_CSA 26
#define PIN_CSB 27
#define PIN_CSC 28
#define PIN_VBUS 29

#define MOTOR_POLE_PAIRS 11

#define PWM_HZ 20000u
#define ALIGN_UD 0.75f
#define ALIGN_RAMP_MS 200u
#define ALIGN_DRAG_MS 2500u
#define ALIGN_DRAG_ELEC_REVS 2.f
#define ALIGN_DRAG_MIN_COUNTS 400
#define ALIGN_HOLD_MS 500u
#define DIR_PULSE_MS 600u
#define DIR_PULSE_UQ 0.18f
#define DIR_PULSE_COUNTS 80
#define PWM_MIN_MAG 0.012f
#define POS_KP 0.6f
#define TEST_MOVE_MS 1400u
#define TEST_HOLD_MS 200u
#define TEST_UQ_MAX 0.65f
// ponytail: open-loop Kv (V/(rad/s)); retune if TEST lags (Uq_ff ≈ lag_rad * POS_KP).
#define TEST_KV 0.07f
// ponytail: K matches POS_KP (known to move). Uq max > DIR_PULSE (cogging), < TEST.
// 20 kHz raw Δpos is 1ct≈7.7 rad/s — D must see LPF'd vel. Ceiling: lag on fast flicks; upgrade = encoder PLL.
#define SPRING_K 0.6f
#define SPRING_D 0.05f
#define SPRING_UQ_MAX 0.35f
#define SPRING_VEL_LP_HZ 50.f
// ponytail: CUR_LOOP_EN=1 enables CSA sample+Park telem. CUR_LOOP_CTRL=1 also closes Id/Iq PI.
// CTRL=0 = sense-only (voltage outer loops; use to inspect Id/Iq without shake).
#define CUR_LOOP_EN 1
#define CUR_LOOP_CTRL 1
#define CUR_LOOP_DIV 4u
// SPIN: always voltage flywheel (SPIN_KV/B + cap). Other modes use current loop when CTRL=1.
#define SPIN_KV 0.020f
#define SPIN_B 0.002f
#define SPIN_W_REST 0.10f
#define SPIN_W_MAX 25.f
#define SPIN_B_CAP 0.08f
#define SPIN_UQ_MAX 0.50f
#define SPIN_VEL_LP_HZ 120.f
// Stress: +full 3s / coast 1s / -full 3s / coast 1s; Uq smoothstep on start & stop.
#define STRESS_UQ_MAX 0.65f
#define STRESS_RUN_MS 3000u
#define STRESS_STOP_MS 1000u
#define STRESS_RAMP_MS 500u
#define CRC_FAIL_TRIP 200u

// DRV8316 CSA: SOX = VREF/2 ± GAIN·I. GAIN pin: AGND=0.15, Hi-Z=0.3, 47k→AVDD=0.6, AVDD=1.2.
#define CS_VREF 3.3f
#define CS_GAIN_V_PER_A 0.3f
#define CS_VBUS_DIV 0.1f
// Sense tuning (sense-only or closed loop): flip sign / permute ABC / shift Park angle if |Id|≫|Iq|.
#define CS_SIGN (-1.f)
#define CS_PHASE_ORD 4 /* 0=ABC 1=ACB 2=BAC 3=BCA 4=CAB 5=CBA */
// Must be 0 while using foc_sense_check sign(Iq)↔Uq; ≠0 parks currents off the voltage axis.
#define CS_TE_OFF (-0.70f) /* tuned vs foc_sense_check; keep 0 while sweeping ORD */
#define CS_IQ_LP_HZ 150.f
// Outer uq_cmd ∈ [-1,1] → Iq_ref (A). Retune with SPRING_K / feel.
#define IQ_CMD_A 1.0f
#define I_TRIP_A 4.0f
// ponytail: PI at CUR_LOOP_DIV rate. Soft on 4015; raise after SOX RC + GAIN match.
#define CUR_KP 0.25f
#define CUR_KI 400.f
#define CUR_U_MAX 0.95f

#endif
