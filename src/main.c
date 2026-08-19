
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

// LED: align=orange blink, idle=green, spring=blue, spin=cyan, test=white blink, fault=red.
// UPLOAD purple is set in serial_handle_line before bootrom.
static void led_status(void) {
	static uint8_t tick;
	motor_state_t st;
	motor_get_state(&st);
	tick++;
	bool on = (tick & 1u) != 0;

	if(st.mode == MOTOR_FAULT) {
		ws2812_setpixel(U32RGB(40, 0, 0));
		return;
	}
	if(st.mode >= MOTOR_ALIGN_RAMP && st.mode <= MOTOR_ALIGN_DOWN) {
		ws2812_setpixel(on ? U32RGB(36, 14, 0) : U32RGB(0, 0, 0));
		return;
	}
	if(st.mode == MOTOR_TEST) {
		ws2812_setpixel(on ? U32RGB(24, 24, 24) : U32RGB(0, 0, 0));
		return;
	}
	if(st.mode == MOTOR_SPRING) {
		ws2812_setpixel(U32RGB(0, 8, 32));
		return;
	}
	if(st.mode == MOTOR_SPIN) {
		ws2812_setpixel(U32RGB(0, 24, 20));
		return;
	}
	// IDLE (armed) and anything else
	ws2812_setpixel(U32RGB(0, 22, 0));
}

void main_handler(uevt_t* evt) {
	switch(evt->evt_id) {
		case UEVT_TIMER_4HZ:
			led_status();
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

enum {
	TK_CMD_START = 0x01,
	TK_CMD_STOP = 0x02,
	TK_CMD_SPRING = 0x03,
	TK_CMD_SPIN = 0x04,
	TK_CMD_TEST = 0x05,
	TK_CMD_SET_K = 0x20,
	TK_CMD_SET_REST = 0x21,
};

int vendor_cmd(uint8_t const* buffer, uint16_t bufsize) {
	if(bufsize < 1)
		return -1;

	char str[16 * 2 + 1];
	uint16_t dump_n = bufsize < 16 ? bufsize : 16;
	for(uint16_t i = 0; i < dump_n; i++) {
		str[i * 2] = printHex[buffer[i] >> 4];
		str[i * 2 + 1] = printHex[buffer[i] & 0xF];
	}
	str[dump_n * 2] = 0;
	LOG_RAW("VND[%d]:%s\n", bufsize, str);

	switch(buffer[0]) {
		case TK_CMD_START:
			motor_start();
			return 1;
		case TK_CMD_STOP:
			motor_cmd_stop();
			return 1;
		case TK_CMD_SPRING:
			return motor_cmd_spring() ? 1 : 0;
		case TK_CMD_SPIN:
			return motor_cmd_spin() ? 1 : 0;
		case TK_CMD_TEST:
			return motor_cmd_test() ? 1 : 0;
		case TK_CMD_SET_K:
			if(bufsize >= 2)
				motor_set_spring_k((float)buffer[1] / 10.f);
			return 1;
		case TK_CMD_SET_REST:
			motor_set_rest_to_current();
			return 1;
		default:
			return -1;
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
	if(strcmp(line, "SPIN") == 0) {
		if(motor_cmd_spin()) {
			LOG_RAW("SPIN\n");
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
	LOG_RAW("boot: auto START; Bulk Vendor + CDC SPRING SPIN TEST STOP DUMP UPLOAD\n");
	motor_start();
	while(true) {
		app_sched_execute();
		tud_task();
		cdc_task();
		vendor_task();
		__wfi();
	}
}
