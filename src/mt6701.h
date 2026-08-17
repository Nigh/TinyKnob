#ifndef _MT6701_H_
#define _MT6701_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
	uint16_t angle;
	uint8_t mg;
	bool crc_ok;
} mt6701_sample_t;

void mt6701_setup(void);
void mt6701_start_read(void);
bool mt6701_try_read(mt6701_sample_t* out);
bool mt6701_crc_selfcheck(void);

#endif
