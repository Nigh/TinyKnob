#include "tusb.h"
#include "tusb_config.h"
#include "hardware/sync.h"

#include "string.h"

volatile bool usb_mounted = false;

void serial_receive(uint8_t const* buffer, uint16_t bufsize);
void hid_receive(uint8_t const* buffer, uint16_t bufsize);

//--------------------------------------------------------------------+
// CDC log ring — enqueue from any context; drain only from main loop
//--------------------------------------------------------------------+

#define CDC_LOG_RING 1024u

static uint8_t log_ring[CDC_LOG_RING];
static volatile uint16_t log_w;
static volatile uint16_t log_r;
static volatile uint32_t log_drop;

static uint16_t ring_used(void) {
	return (uint16_t)((log_w - log_r) & (CDC_LOG_RING - 1));
}

static uint16_t ring_free(void) {
	return (uint16_t)(CDC_LOG_RING - 1u - ring_used());
}

void cdc_log_init(void) {
	log_w = 0;
	log_r = 0;
	log_drop = 0;
}

void cdc_log_enqueue(const void* data, uint16_t len) {
	if(len == 0 || data == NULL)
		return;
	uint32_t irq = save_and_disable_interrupts();
	if(ring_free() < len) {
		log_drop++;
		restore_interrupts(irq);
		return;
	}
	const uint8_t* p = (const uint8_t*)data;
	for(uint16_t i = 0; i < len; i++) {
		log_ring[log_w] = p[i];
		log_w = (uint16_t)((log_w + 1u) & (CDC_LOG_RING - 1));
	}
	restore_interrupts(irq);
}

static void cdc_log_drain(void) {
	if(!usb_mounted || !tud_cdc_n_connected(0))
		return;
	while(log_r != log_w) {
		uint32_t avail = tud_cdc_n_write_available(0);
		if(avail == 0) {
			tud_cdc_n_write_flush(0);
			break;
		}
		uint16_t used = ring_used();
		uint16_t chunk = used;
		if(chunk > avail)
			chunk = (uint16_t)avail;
		uint16_t to_end = (uint16_t)(CDC_LOG_RING - log_r);
		if(chunk > to_end)
			chunk = to_end;
		tud_cdc_n_write(0, &log_ring[log_r], chunk);
		log_r = (uint16_t)((log_r + chunk) & (CDC_LOG_RING - 1));
	}
	if(tud_cdc_n_write_available(0) < CFG_TUD_CDC_TX_BUFSIZE)
		tud_cdc_n_write_flush(0);
}

void cdc_task(void) {
	uint8_t itf;
	for(itf = 0; itf < CFG_TUD_CDC; itf++) {
		if(tud_cdc_n_available(itf)) {
			uint8_t buf[64];
			uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));
			serial_receive(buf, count);
		}
	}
	cdc_log_drain();
}

void cdc_log_print(char* str) {
	if(str == NULL)
		return;
	cdc_log_enqueue(str, (uint16_t)strlen(str));
}

// Blocking write for rare dumps — waits until CDC has room so lines are not truncated.
void cdc_log_print_wait(char* str) {
	if(!usb_mounted || str == NULL)
		return;
	uint16_t len = (uint16_t)strlen(str);
	uint16_t off = 0;
	while(off < len) {
		tud_task();
		cdc_log_drain();
		uint32_t avail = tud_cdc_n_write_available(0);
		if(avail == 0) {
			tud_cdc_n_write_flush(0);
			continue;
		}
		uint16_t n = (uint16_t)(len - off);
		if(n > avail)
			n = (uint16_t)avail;
		tud_cdc_n_write(0, str + off, n);
		off = (uint16_t)(off + n);
	}
	tud_cdc_n_write_flush(0);
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

void tud_mount_cb(void) {
	usb_mounted = true;
}

void tud_umount_cb(void) {
	usb_mounted = false;
}

void tud_suspend_cb(bool remote_wakeup_en) {
	(void)remote_wakeup_en;
}

void tud_resume_cb(void) {
	usb_mounted = tud_mounted() ? true : false;
}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) {
	(void)itf;
	(void)report_id;
	(void)report_type;
	(void)buffer;
	(void)reqlen;
	return 0;
}

__attribute__((weak)) void serial_receive(uint8_t const* buffer, uint16_t bufsize) {
	(void)buffer;
	(void)bufsize;
}

__attribute__((weak)) void hid_receive(uint8_t const* buffer, uint16_t bufsize) {
	(void)buffer;
	(void)bufsize;
}

void hid_send(uint8_t const* buffer, uint16_t bufsize) {
	if(!usb_mounted)
		return;
	tud_hid_report(0, buffer, bufsize);
}

void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) {
	(void)itf;
	(void)report_id;
	(void)report_type;
	hid_receive(buffer, bufsize);
}
