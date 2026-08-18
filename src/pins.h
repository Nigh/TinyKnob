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
// ponytail: voltage-mode SPIN cannot be a true freewheel. Kv must stay < ke or a
// touch self-spins / hunt when held. Ceiling: draggy coast. Full feel needs Iq≈0
// current loop on CSA/CSB/CSC (not wired). Uq=0 is LS brake (no INLx Hi-Z).
#define SPIN_KV 0.020f
#define SPIN_B 0.002f
#define SPIN_W_REST 0.10f
#define SPIN_W_MAX 25.f
#define SPIN_B_CAP 0.08f
#define SPIN_UQ_MAX 0.50f
#define SPIN_VEL_LP_HZ 120.f
#define CRC_FAIL_TRIP 200u

// ponytail: CSA/CSB/CSC not wired; ADC Iq loop later (required for real SPIN freewheel).
// Do not assign GPIOs until the analog traces exist.
// #define PIN_CSA
// #define PIN_CSB
// #define PIN_CSC

#endif
