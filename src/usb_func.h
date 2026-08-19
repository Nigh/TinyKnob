#ifndef _USB_FUNC_H_
#define _USB_FUNC_H_
#include <stdint.h>
#include <stdbool.h>

extern volatile bool usb_mounted;
void cdc_log_init(void);
void cdc_task(void);
void cdc_log_enqueue(const void* data, uint16_t len);
void cdc_log_print(char* str);
void cdc_log_print_wait(char* str);
void vendor_task(void);
/* Return 1=ok, 0=rejected, -1=unknown (no ACK). */
int vendor_cmd(uint8_t const* buffer, uint16_t bufsize);

#endif
