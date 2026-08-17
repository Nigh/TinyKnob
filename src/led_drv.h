
#ifndef _LED_DRV_H_
#define _LED_DRV_H_
#include <stdint.h>
void ws2812_setup(void);
void ws2812_setpixel(uint32_t pixel_grb);

#endif
