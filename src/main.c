
#include "pico/stdlib.h"
#include "hardware/sync.h"

#include "scheduler/uevent.h"
#include "scheduler/scheduler.h"

#include "platform.h"
#include "led_drv.h"

#include "tusb_config.h"

#include "pico/sync.h"
#include "pico/float.h"
#include "pico/bootrom.h"

critical_section_t scheduler_lock;
static __inline void CRITICAL_REGION_INIT(void) {
	critical_section_init(&scheduler_lock);
}
static __inline void CRITICAL_REGION_ENTER(void) {
	critical_section_enter_blocking(&scheduler_lock);
}
static __inline void CRITICAL_REGION_EXIT(void) {
	critical_section_exit(&scheduler_lock);
}

bool timer_4hz_callback(struct repeating_timer* t) {
	LOG_RAW("At %lld us:\n", time_us_64());
	uevt_bc_e(UEVT_TIMER_4HZ);
	return true;
}

#define U32RGB(r, g, b) (((uint32_t)(r) << 8) | ((uint32_t)(g) << 16) | (uint32_t)(b))

void led_blink_routine(void) {
	static uint8_t _tick = 0;
	_tick += 1;
	if(_tick & 0x1) {
		if(usb_mounted) {
			ws2812_setpixel(U32RGB(4, 14, 4));
		} else {
			ws2812_setpixel(U32RGB(20, 20, 2));
		}
	} else {
		ws2812_setpixel(U32RGB(0, 0, 0));
	}
}

void main_handler(uevt_t* evt) {
	switch(evt->evt_id) {
		case UEVT_TIMER_4HZ:
			led_blink_routine();
			break;
	}
}

void uevt_log(char* str) {
	LOG_RAW("%s\n", str);
}

const char printHex[] = "0123456789ABCDEF";
void hid_receive(uint8_t const* buffer, uint16_t bufsize) {
	char str[16 * 2 + 1];
	str[32] = 0;
	for(uint16_t i = 0; i < 16; i++) {
		str[i * 2] = printHex[buffer[i] >> 4];
		str[i * 2 + 1] = printHex[buffer[i] & 0xF];
	}
	// print first 32 bytes
	LOG_RAW("HID[%d]:%s\n", bufsize, str);

	uint8_t echo[64];
	// echo back with every byte + 1
	for (uint16_t i = 0; i < bufsize; i++) {
		echo[i] = buffer[i] + 1;
	}
	hid_send(echo, bufsize);
}

static char serial_fifo[16];
static uint8_t serial_wp = 0;
uint8_t serial_got(const char* str) {
	uint8_t len = strlen(str);
	for(uint8_t i = 1; i <= len; i++) {
		if(serial_fifo[serial_wp + (0x10 - i) & 0xF] != str[len - i]) {
			return 0;
		}
	}
	return 1;
}
void serial_receive(uint8_t const* buffer, uint16_t bufsize) {
	for(uint16_t i = 0; i < bufsize; i++) {
		if((*buffer == 0x0A) || (*buffer == 0x0D)) {
			if(serial_got("UPLOAD")) {
				ws2812_setpixel(U32RGB(20, 0, 20));
				reset_usb_boot(0, 0);
			}
		} else {
			serial_fifo[serial_wp++ & 0xF] = *buffer++;
		}
	}
}

#include "hardware/xosc.h"
extern void cdc_task(void);
int main() {
	xosc_init();

	CRITICAL_REGION_INIT();
	app_sched_init();
	user_event_init();
	user_event_handler_regist(main_handler);

	ws2812_setup();
	struct repeating_timer timer;
	add_repeating_timer_us(249978ul, timer_4hz_callback, NULL, &timer);
	tusb_init();
	cdc_log_init();
	while(true) {
		app_sched_execute();
		tud_task();
		cdc_task();
		__wfi();
	}
}
