#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <stdint.h>

enum {
	MOTOR_ALIGN_RAMP = 0,
	MOTOR_ALIGN_HOLD,
	MOTOR_DIR_PULSE,
	MOTOR_ALIGN_DOWN,
	MOTOR_MOVE_FWD,
	MOTOR_MOVE_REV,
	MOTOR_IDLE,
	MOTOR_FAULT,
};

typedef struct {
	uint8_t mode;
	int32_t pos;
	int32_t offset_mrad;
	int8_t dir;
	uint32_t crc_fail;
} motor_state_t;

void motor_setup(void);
void motor_set_duty(float a, float b, float c);
void motor_get_state(motor_state_t* s);

#endif
