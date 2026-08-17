
#include "hardware/pio.h"
#include "ws2812/ws2812.pio.h"

#define IS_RGBW false
#define NUM_PIXELS 1
#define WS2812_PIN 16

void ws2812_setpixel(uint32_t pixel_grb) {
	pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

void ws2812_setup(void) {
	PIO pio = pio0;
	uint offset = pio_add_program(pio, &ws2812_program);
	ws2812_program_init(pio, 0, offset, WS2812_PIN, 800000, IS_RGBW);
}
