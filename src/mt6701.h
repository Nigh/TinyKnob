#ifndef _MT6701_H_
#define _MT6701_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
	uint32_t raw;
	uint32_t pio_word;
	uint16_t angle;
	uint8_t mg;
	bool crc_ok;
} mt6701_sample_t;

void mt6701_setup(void);
void mt6701_start_read(void);
bool mt6701_try_read(mt6701_sample_t* out);
bool mt6701_crc_selfcheck(void);
uint8_t mt6701_crc6(uint32_t data18);
void mt6701_log_brief(uint32_t pio_word);
void mt6701_log_dump(uint32_t pio_word);

#endif
