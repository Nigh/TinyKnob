#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <stdint.h>
#include <stdbool.h>

enum {
	MOTOR_IDLE = 0,
	MOTOR_ALIGN_RAMP,
	MOTOR_ALIGN_HOLD,
	MOTOR_DIR_PULSE,
	MOTOR_ALIGN_DOWN,
	MOTOR_TEST,
	MOTOR_SPRING,
	MOTOR_SPIN, // voltage spin_uq (+ cog); CUR_LOOP_CTRL PI reserved for DMA sample
	MOTOR_FAULT,
	MOTOR_POS, // track absolute angle_mrad (streaming setpoint)
	MOTOR_STRESS, // full-speed fwd/rev burn-in with smooth Uq ramps
	MOTOR_COG_CAL, // slow +1 rev; fill cogging LUT then IDLE
};

typedef struct {
	uint8_t mode;
	int32_t pos;
	int32_t angle_mrad;
	int32_t offset_mrad;
	int8_t dir;
	int16_t duty_a_q15;
	int16_t duty_b_q15;
	int16_t duty_c_q15;
	uint32_t crc_fail;
	uint32_t crc_ok;
	uint32_t raw;
	uint32_t pio_word;
	bool last_crc_ok;
	float id_a;
	float iq_a;
	float iq_ref_a;
	float vbus_v;
	float ud_out;
	float uq_out;
	bool cog_en;
	bool aligned; // false until START align completes
} motor_state_t;

void motor_setup(void);
void motor_start(void);
void motor_cmd_stop(void);
bool motor_cmd_spring(void);
bool motor_cmd_spin(void);
bool motor_cmd_test(void);
bool motor_cmd_stress(void);
bool motor_cmd_cog_cal(void); // slow sweep → RAM LUT override
void motor_cmd_cog_clear(void); // restore flash LUT (drop RAM override)
void motor_cog_dump(void); // CDC: print active LUT
bool motor_cmd_goto(int32_t angle_mrad); // set/track absolute mech angle (telem units)
void motor_set_duty(float a, float b, float c);
void motor_get_state(motor_state_t* s);
void motor_set_spring_k(float k);
void motor_set_rest_to_current(void);
float motor_get_spring_k(void);

#endif
