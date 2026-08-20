#include "tusb.h"
#include "tusb_config.h"
#include "hardware/sync.h"
#include "pico/time.h"

#include "string.h"
#include "motor.h"
#include "usb_func.h"

volatile bool usb_mounted = false;

void serial_receive(uint8_t const* buffer, uint16_t bufsize);

#define TELEM_MAGIC 0xA5
#define ACK_MAGIC 0x5A
#define TELEM_PERIOD_US 1000u /* ~1 kHz */

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
// Vendor Bulk — commands OUT, telemetry IN (see docs/usb-protocol.md)
//--------------------------------------------------------------------+

__attribute__((weak)) void serial_receive(uint8_t const* buffer, uint16_t bufsize) {
	(void)buffer;
	(void)bufsize;
}

__attribute__((weak)) int vendor_cmd(uint8_t const* buffer, uint16_t bufsize) {
	(void)buffer;
	(void)bufsize;
	return -1;
}

static void vendor_send_ack(uint8_t cmd, uint8_t status) {
	if(!usb_mounted || !tud_vendor_n_mounted(0))
		return;
	if(tud_vendor_n_write_available(0) < 3)
		return;
	uint8_t ack[3] = {ACK_MAGIC, cmd, status};
	tud_vendor_n_write(0, ack, 3);
	tud_vendor_n_write_flush(0);
}

static int16_t amp_to_ma(float a) {
	float ma = a * 1000.f;
	if(ma > 32767.f)
		return 32767;
	if(ma < -32768.f)
		return -32768;
	return (int16_t)ma;
}

static uint16_t volt_to_mv(float v) {
	float mv = v * 1000.f;
	if(mv < 0.f)
		return 0;
	if(mv > 65535.f)
		return 65535;
	return (uint16_t)mv;
}

static void vendor_send_telem(void) {
	if(!usb_mounted || !tud_vendor_n_mounted(0))
		return;
	if(tud_vendor_n_write_available(0) < 24)
		return;

	static uint16_t seq;
	motor_state_t st;
	motor_get_state(&st);

	int16_t id_ma = amp_to_ma(st.id_a);
	int16_t iq_ma = amp_to_ma(st.iq_a);
	int16_t iq_ref_ma = amp_to_ma(st.iq_ref_a);
	uint16_t vbus_mv = volt_to_mv(st.vbus_v);
	float uq = st.uq_out;
	if(uq > 1.f)
		uq = 1.f;
	if(uq < -1.f)
		uq = -1.f;
	int16_t uq_q15 = (int16_t)(uq * 32767.f);

	uint8_t pkt[24];
	pkt[0] = TELEM_MAGIC;
	pkt[1] = st.mode;
	pkt[2] = (uint8_t)st.angle_mrad;
	pkt[3] = (uint8_t)(st.angle_mrad >> 8);
	pkt[4] = (uint8_t)(st.angle_mrad >> 16);
	pkt[5] = (uint8_t)(st.angle_mrad >> 24);
	pkt[6] = (uint8_t)st.duty_a_q15;
	pkt[7] = (uint8_t)(st.duty_a_q15 >> 8);
	pkt[8] = (uint8_t)st.duty_b_q15;
	pkt[9] = (uint8_t)(st.duty_b_q15 >> 8);
	pkt[10] = (uint8_t)st.duty_c_q15;
	pkt[11] = (uint8_t)(st.duty_c_q15 >> 8);
	pkt[12] = (uint8_t)seq;
	pkt[13] = (uint8_t)(seq >> 8);
	pkt[14] = (uint8_t)id_ma;
	pkt[15] = (uint8_t)(id_ma >> 8);
	pkt[16] = (uint8_t)iq_ma;
	pkt[17] = (uint8_t)(iq_ma >> 8);
	pkt[18] = (uint8_t)iq_ref_ma;
	pkt[19] = (uint8_t)(iq_ref_ma >> 8);
	pkt[20] = (uint8_t)vbus_mv;
	pkt[21] = (uint8_t)(vbus_mv >> 8);
	pkt[22] = (uint8_t)uq_q15;
	pkt[23] = (uint8_t)(uq_q15 >> 8);
	seq++;

	tud_vendor_n_write(0, pkt, 24);
	tud_vendor_n_write_flush(0);
}

void vendor_task(void) {
	if(!usb_mounted || !tud_vendor_n_mounted(0))
		return;

	while(tud_vendor_n_available(0)) {
		uint8_t buf[64];
		uint32_t n = tud_vendor_n_read(0, buf, sizeof(buf));
		if(n == 0)
			break;
		int rc = vendor_cmd(buf, (uint16_t)n);
		if(rc >= 0) {
			// ponytail: successful GOTO is high-rate; skip ACK so Bulk IN stays telem.
			if(!(buf[0] == 0x06 && rc == 1))
				vendor_send_ack(buf[0], (uint8_t)rc);
		}
	}

	static uint32_t last_us;
	uint32_t now = time_us_32();
	if((uint32_t)(now - last_us) >= TELEM_PERIOD_US) {
		last_us = now;
		vendor_send_telem();
	}
}
