#ifndef _PINS_H_
#define _PINS_H_

// RP2040-Zero — TinyKnob

#define PIN_PWM_A 2
#define PIN_PWM_B 4
#define PIN_PWM_C 6

#define PIN_MT6701_CSN 9
#define PIN_MT6701_CLK 10
#define PIN_MT6701_DO 11

#define MOTOR_POLE_PAIRS 11

#define PWM_HZ 20000u
#define ALIGN_UD 0.12f
#define ALIGN_RAMP_MS 50u
#define ALIGN_HOLD_MS 300u
#define DIR_PULSE_MS 20u
#define DIR_PULSE_UQ 0.08f
#define UQ_MAX 0.20f
#define MOVE_REV_MS 2000u
#define POS_KP 1.5f
#define SPRING_K 0.8f
#define SPRING_D 0.02f

// ponytail: CSA/CSB/CSC not wired; ADC current loop later. Do not assign GPIOs until the analog traces exist.
// #define PIN_CSA
// #define PIN_CSB
// #define PIN_CSC

#endif
