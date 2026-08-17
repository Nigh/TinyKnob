#include "mt6701.h"
#include "pins.h"
#include "platform.h"
#include "hardware/pio.h"
#include "mt6701.pio.h"

#define MT6701_PIO pio0
#define MT6701_SM 1

static volatile bool ssi_busy;

static uint8_t mt6701_crc6(uint32_t data18) {
	uint8_t crc = 0;
	for(int i = 0; i < 18; i++) {
		uint8_t bit = (data18 >> (17 - i)) & 1u;
		crc = (uint8_t)((crc << 1) | bit);
		if(crc & 0x40)
			crc ^= 0x43;
	}
	return crc & 0x3F;
}

bool mt6701_crc_selfcheck(void) {
	// LFSR x^6+x+1, MSB first; vectors from the same implementation
	if(mt6701_crc6(0) != 0)
		return false;
	if(mt6701_crc6(1) != 1)
		return false;
	if(mt6701_crc6(0x15555) != 0x28)
		return false;
	if(mt6701_crc6(0x3FFFF) != 0x3B)
		return false;
	return true;
}

static void mt6701_decode(uint32_t word24, mt6701_sample_t* out) {
	word24 &= 0xFFFFFFu;
	out->angle = (uint16_t)((word24 >> 10) & 0x3FFFu);
	out->mg = (uint8_t)((word24 >> 6) & 0xFu);
	uint8_t crc = (uint8_t)(word24 & 0x3Fu);
	out->crc_ok = (mt6701_crc6(word24 >> 6) == crc);
}

void mt6701_setup(void) {
	if(!mt6701_crc_selfcheck())
		LOG_RAW("MT6701 CRC selfcheck FAIL\n");
	uint offset = pio_add_program(MT6701_PIO, &mt6701_ssi_program);
	mt6701_ssi_program_init(MT6701_PIO, MT6701_SM, offset, PIN_MT6701_CSN, PIN_MT6701_CLK, PIN_MT6701_DO);
	ssi_busy = false;
}

void mt6701_start_read(void) {
	if(ssi_busy)
		return;
	ssi_busy = true;
	pio_sm_put(MT6701_PIO, MT6701_SM, 0);
}

bool mt6701_try_read(mt6701_sample_t* out) {
	if(pio_sm_is_rx_fifo_empty(MT6701_PIO, MT6701_SM))
		return false;
	uint32_t word = pio_sm_get(MT6701_PIO, MT6701_SM);
	ssi_busy = false;
	mt6701_decode(word, out);
	return true;
}
