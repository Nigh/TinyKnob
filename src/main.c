
#include "pico/stdlib.h"
#include "hardware/sync.h"
#include <string.h>

#include "scheduler/uevent.h"
#include "scheduler/scheduler.h"

#include "platform.h"
#include "led_drv.h"
#include "mt6701.h"
#include "motor.h"

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
	(void)t;
	// IRQ: only queue the event — never touch TinyUSB / LOG_RAW here.
	uevt_bc_e(UEVT_TIMER_4HZ);
	return true;
}

#define U32RGB(r, g, b) (((uint32_t)(r) << 8) | ((uint32_t)(g) << 16) | (uint32_t)(b))

void led_blink_routine(void) {
	static uint8_t _tick = 0;
	motor_state_t st;
	motor_get_state(&st);
	_tick += 1;
	if((_tick & 0x1) == 0) {
		ws2812_setpixel(U32RGB(0, 0, 0));
		return;
	}
	if(st.mode == MOTOR_FAULT) {
		ws2812_setpixel(U32RGB(32, 0, 0));
	} else if(usb_mounted) {
		ws2812_setpixel(U32RGB(4, 14, 4));
	} else {
		ws2812_setpixel(U32RGB(20, 20, 2));
	}
}

void main_handler(uevt_t* evt) {
	switch(evt->evt_id) {
		case UEVT_TIMER_4HZ:
			led_blink_routine();
			{
				static uint8_t tick;
				motor_state_t st;
				motor_get_state(&st);
				tick++;
				// 4 Hz during align and TEST; else ~1 Hz
				bool fast = (st.mode >= MOTOR_ALIGN_RAMP && st.mode <= MOTOR_ALIGN_DOWN) ||
						st.mode == MOTOR_TEST;
				if(fast || (tick & 3) == 0) {
					LOG_RAW("m%u p%ld o%lu f%lu d%d\n",
							st.mode, (long)st.pos,
							(unsigned long)st.crc_ok, (unsigned long)st.crc_fail,
							st.dir);
					mt6701_log_brief(st.pio_word);
				}
			}
			break;
	}
}

void uevt_log(char* str) {
	LOG_RAW("%s\n", str);
}

const char printHex[] = "0123456789ABCDEF";
#define HID_CMD_GET_STATE 0x10
#define HID_CMD_SET_K 0x20
#define HID_CMD_SET_REST 0x21

void hid_receive(uint8_t const* buffer, uint16_t bufsize) {
	char str[16 * 2 + 1];
	str[32] = 0;
	for(uint16_t i = 0; i < 16; i++) {
		str[i * 2] = printHex[buffer[i] >> 4];
		str[i * 2 + 1] = printHex[buffer[i] & 0xF];
	}
	LOG_RAW("HID[%d]:%s\n", bufsize, str);

	if(bufsize < 1)
		return;

	uint8_t out[64];
	memset(out, 0, sizeof(out));
	switch(buffer[0]) {
		case HID_CMD_GET_STATE: {
			motor_state_t st;
			motor_get_state(&st);
			out[0] = HID_CMD_GET_STATE;
			out[1] = st.mode;
			out[2] = (uint8_t)st.dir;
			out[3] = (uint8_t)st.pos;
			out[4] = (uint8_t)(st.pos >> 8);
			out[5] = (uint8_t)(st.pos >> 16);
			out[6] = (uint8_t)(st.pos >> 24);
			out[7] = (uint8_t)st.offset_mrad;
			out[8] = (uint8_t)(st.offset_mrad >> 8);
			out[9] = (uint8_t)(st.offset_mrad >> 16);
			out[10] = (uint8_t)(st.offset_mrad >> 24);
			out[11] = (uint8_t)(motor_get_spring_k() * 10.f);
			hid_send(out, bufsize);
			return;
		}
		case HID_CMD_SET_K:
			if(bufsize >= 2)
				motor_set_spring_k((float)buffer[1] / 10.f);
			out[0] = HID_CMD_SET_K;
			hid_send(out, bufsize);
			return;
		case HID_CMD_SET_REST:
			motor_set_rest_to_current();
			out[0] = HID_CMD_SET_REST;
			hid_send(out, bufsize);
			return;
		default: {
			for(uint16_t i = 0; i < bufsize; i++)
				out[i] = buffer[i] + 1;
			hid_send(out, bufsize);
			return;
		}
	}
}

static char serial_line[40];
static uint8_t serial_line_len;

static void serial_handle_line(const char* line) {
	if(strcmp(line, "UPLOAD") == 0) {
		ws2812_setpixel(U32RGB(20, 0, 20));
		reset_usb_boot(0, 0);
		return;
	}
	if(strcmp(line, "START") == 0) {
		LOG_RAW("START: align\n");
		motor_start();
		return;
	}
	if(strcmp(line, "SPRING") == 0) {
		if(motor_cmd_spring()) {
			LOG_RAW("SPRING\n");
		} else {
			LOG_RAW("need START\n");
		}
		return;
	}
	if(strcmp(line, "TEST") == 0) {
		if(motor_cmd_test()) {
			LOG_RAW("TEST\n");
		} else {
			LOG_RAW("need START\n");
		}
		return;
	}
	if(strcmp(line, "STOP") == 0) {
		LOG_RAW("STOP\n");
		motor_cmd_stop();
		return;
	}
	if(strcmp(line, "DUMP") == 0) {
		motor_state_t st;
		motor_get_state(&st);
		mt6701_log_dump(st.pio_word);
	}
}

void serial_receive(uint8_t const* buffer, uint16_t bufsize) {
	for(uint16_t i = 0; i < bufsize; i++) {
		uint8_t c = buffer[i];
		if(c == '\n' || c == '\r') {
			if(serial_line_len > 0) {
				serial_line[serial_line_len] = 0;
				serial_handle_line(serial_line);
				serial_line_len = 0;
			}
			continue;
		}
		if(serial_line_len + 1u < sizeof(serial_line))
			serial_line[serial_line_len++] = (char)c;
		else
			serial_line_len = 0; // overrun: drop line
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
	mt6701_setup();
	motor_setup();
	struct repeating_timer timer;
	add_repeating_timer_us(249978ul, timer_4hz_callback, NULL, &timer);
	tusb_init();
	cdc_log_init();
	LOG_RAW("idle: START SPRING TEST STOP DUMP + newline\n");
	while(true) {
		app_sched_execute();
		tud_task();
		cdc_task();
		__wfi();
	}
}
