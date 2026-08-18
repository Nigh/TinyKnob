#include "platform.h"
#include "hardware/gpio.h"
#include "usb_func.h"

char log_cache[128] = { 0 };

#include <stdarg.h>
void remote_log(const char* format, ...) {
#if G_LOG_ENABLED == 1
	va_list args;
	va_start(args, format);
	int n = vsnprintf(log_cache, sizeof(log_cache), format, args);
	va_end(args);
	if(n > 0)
		cdc_log_enqueue(log_cache, (uint16_t)n);
#else
	(void)format;
#endif
}
