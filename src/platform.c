#include "platform.h"
#include "hardware/gpio.h"
#include "usb_func.h"

char log_cache[128] = { 0 };
char log_buffer[512] = { 0 };
uint16_t log_ptr = 0;
int16_t log_length = 0;

#include <stdarg.h>
void remote_log(const char* format, ...) {
#if G_LOG_ENABLED == 1
	va_list args;
	va_start(args, format);
	log_length = sprintf(log_cache, format, args) + 1;
	if(log_length > 1) {
		if(log_ptr + log_length >= 512) {
			log_ptr = 0;
		}
		memcpy(log_buffer + log_ptr, log_cache, log_length);
		cdc_log_print(log_buffer + log_ptr);
		log_ptr += log_length;
	}
	va_end(args);
#endif
}
