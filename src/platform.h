
#ifndef _PLATFORM_H_
#define _PLATFORM_H_
#include <stdint.h>
#include <stdbool.h>
#include "tusb.h"
#include "usb_func.h"
#include "scheduler/uevent.h"

#if G_LOG_ENABLED == 1
extern char log_cache[128];
extern char log_buffer[512];
extern uint16_t log_ptr;
extern int16_t log_length;
	#define LOG_RAW(...) \
		do { \
			log_length = sprintf(log_cache, __VA_ARGS__) + 1; \
			if(log_length > 1) { \
				if(log_ptr + log_length >= 512) { \
					log_ptr = 0; \
				} \
				memcpy(log_buffer + log_ptr, log_cache, log_length); \
				cdc_log_print(log_buffer + log_ptr); \
				log_ptr += log_length; \
			} \
		} while(0);
#else
	#define LOG_RAW(...)
#endif
#include <stdarg.h>
void remote_log(const char* format, ...);

typedef union {
	uint8_t raw[64];
	uint8_t _[64];
} data_raw_t;

// REGEXP: #define UEVT_\S+_BASE
#define UEVT_APP_BASE (0xAE00)
#define UEVT_APP_POWERUP (UEVT_APP_BASE | 0x01)
#define UEVT_APP_BOOT (UEVT_APP_BASE | 0x0B)
#define UEVT_APP_SETUP (UEVT_APP_BASE | 0x02)
#define UEVT_APP_START (UEVT_APP_BASE | 0x03)
#define UEVT_APP_BEFORE_SLEEP (UEVT_APP_BASE | 0x0E)
#define UEVT_APP_SLEEP (UEVT_APP_BASE | 0x0C)
#define UEVT_APP_WAKE (UEVT_APP_BASE | 0x0D)

#define UEVT_RTC_BASE (0x0000)
#define UEVT_TIMER_4HZ (UEVT_RTC_BASE | 0x10)
#define UEVT_TIMER_100HZ (UEVT_RTC_BASE | 0x11)

#define UEVT_ADC_BASE (0x0200)
#define UEVT_ADC_TEMPERATURE_RESULT (UEVT_ADC_BASE | 0x01)

#endif
