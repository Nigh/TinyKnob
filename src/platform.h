#ifndef _PLATFORM_H_
#define _PLATFORM_H_
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "tusb.h"
#include "usb_func.h"
#include "scheduler/uevent.h"

#if G_LOG_ENABLED == 1
extern char log_cache[128];
	#define LOG_RAW(...) \
		do { \
			int _n = snprintf(log_cache, sizeof(log_cache), __VA_ARGS__); \
			if(_n > 0) \
				cdc_log_enqueue(log_cache, (uint16_t)_n); \
		} while(0)
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
