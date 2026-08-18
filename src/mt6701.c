#include "mt6701.h"
#include "pins.h"
#include "platform.h"
#include "usb_func.h"
#include "hardware/pio.h"
#include "mt6701.pio.h"
#include <stdio.h>
#include <stdarg.h>

// pio1: keep WS2812 alone on pio0 SM0
#define MT6701_PIO pio1
#define MT6701_SM 0

static volatile bool ssi_busy;

// MagnTek: CRC over D[13:0]|Mg[3:0], poly X^6+X+1, MSB first.
uint8_t mt6701_crc6(uint32_t data18) {
	uint8_t crc = 0;
	for(int i = 0; i < 18; i++) {
		uint8_t bit = ((data18 >> (17 - i)) & 1u) ^ ((crc >> 5) & 1u);
		crc = (uint8_t)((crc << 1) & 0x3F);
		if(bit)
			crc ^= 0x03;
	}
	return crc;
}

bool mt6701_crc_selfcheck(void) {
	if(mt6701_crc6(0) != 0)
		return false;
	if(mt6701_crc6(1) != 0x03)
		return false;
	if(mt6701_crc6(0x15555) != 0x3B)
		return false;
	if(mt6701_crc6(0x3FFFF) != 0x0E)
		return false;
	// known good SSI frame from hardware DUMP
	if(mt6701_crc6(0x7CB827 >> 6) != 0x27)
		return false;
	return true;
}

static uint32_t rev24(uint32_t w) {
	uint32_t r = 0;
	for(int i = 0; i < 24; i++) {
		r <<= 1;
		r |= (w >> i) & 1u;
	}
	return r;
}

static bool fill_from_word(uint32_t word24, mt6701_sample_t* out) {
	word24 &= 0xFFFFFFu;
	out->raw = word24;
	out->angle = (uint16_t)((word24 >> 10) & 0x3FFFu);
	out->mg = (uint8_t)((word24 >> 6) & 0xFu);
	uint8_t crc = (uint8_t)(word24 & 0x3Fu);
	out->crc_ok = (mt6701_crc6(word24 >> 6) == crc);
	return out->crc_ok;
}

static void mt6701_decode(uint32_t word, mt6701_sample_t* out) {
	fill_from_word(word & 0xFFFFFFu, out);
}

static void log_wait(const char* fmt, ...) {
	char buf[128];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	cdc_log_print_wait(buf);
}

void mt6701_log_brief(uint32_t pio_word) {
	uint32_t r = pio_word & 0xFFFFFFu;
	uint16_t ang = (uint16_t)((r >> 10) & 0x3FFFu);
	uint8_t crc = (uint8_t)(r & 0x3Fu);
	uint8_t calc = mt6701_crc6(r >> 6);
	LOG_RAW("ssi %06lX a=%u c=%02X k=%02X ok=%d\n",
			(unsigned long)r, ang, crc, calc, crc == calc);
}

void mt6701_log_dump(uint32_t pio_word) {
	uint32_t r = pio_word & 0xFFFFFFu;
	char bits[25];
	for(int i = 0; i < 24; i++)
		bits[i] = (r & (1u << (23 - i))) ? '1' : '0';
	bits[24] = 0;

	uint16_t ang = (uint16_t)((r >> 10) & 0x3FFFu);
	uint8_t mg = (uint8_t)((r >> 6) & 0xFu);
	uint8_t crc = (uint8_t)(r & 0x3Fu);
	uint8_t calc = mt6701_crc6(r >> 6);

	log_wait("DUMP pio=%08lX raw=%06lX\n", (unsigned long)pio_word, (unsigned long)r);
	log_wait("DUMP bits=%s\n", bits);
	log_wait("DUMP ang=%u mg=%u crc=%02X calc=%02X ok=%d\n",
			 ang, mg, crc, calc, crc == calc);

	for(int sh = -2; sh <= 2; sh++) {
		uint32_t x = (sh >= 0) ? ((r >> sh) & 0xFFFFFFu) : ((r << (uint32_t)(-sh)) & 0xFFFFFFu);
		uint16_t a = (uint16_t)((x >> 10) & 0x3FFFu);
		uint8_t c = (uint8_t)(x & 0x3Fu);
		uint8_t k = mt6701_crc6(x >> 6);
		log_wait("DUMP sh=%+d %06lX a=%u c=%02X k=%02X ok=%d\n",
				 sh, (unsigned long)x, a, c, k, c == k);
	}
	{
		uint32_t x = (pio_word >> 8) & 0xFFFFFFu;
		uint8_t c = (uint8_t)(x & 0x3Fu);
		uint8_t k = mt6701_crc6(x >> 6);
		log_wait("DUMP >>8 %06lX a=%u c=%02X k=%02X ok=%d\n",
				 (unsigned long)x, (unsigned)((x >> 10) & 0x3FFFu), c, k, c == k);
	}
	{
		uint32_t x = rev24(r);
		uint8_t c = (uint8_t)(x & 0x3Fu);
		uint8_t k = mt6701_crc6(x >> 6);
		log_wait("DUMP rev %06lX a=%u c=%02X k=%02X ok=%d\n",
				 (unsigned long)x, (unsigned)((x >> 10) & 0x3FFFu), c, k, c == k);
	}
	log_wait("DUMP end\n");
}

void mt6701_setup(void) {
	if(!mt6701_crc_selfcheck()) {
		LOG_RAW("MT6701 CRC selfcheck FAIL\n");
	} else {
		LOG_RAW("MT6701 CRC selfcheck OK\n");
	}
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
	out->pio_word = word;
	mt6701_decode(word, out);
	return true;
}
