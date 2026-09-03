#include "stm32g4xx.h"
#include "tusb.h"

#define USB_VID 0xACDC
#define USB_PID 0x4011
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN)

static tusb_desc_device_t const device_desc = {
	.bLength = sizeof(tusb_desc_device_t),
	.bDescriptorType = TUSB_DESC_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
	.idVendor = USB_VID,
	.idProduct = USB_PID,
	.bcdDevice = 0x0101,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

static uint8_t const config_desc[] = {
	TUD_CONFIG_DESCRIPTOR(1, 1, 0, CONFIG_TOTAL_LEN, 0, 100),
	TUD_VENDOR_DESCRIPTOR(0, 4, 0x01, 0x81, 64),
};

uint8_t const *tud_descriptor_device_cb(void) { return (uint8_t const *)&device_desc; }
uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
	(void)index;
	return config_desc;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
	(void)langid;
	static uint16_t desc[33];
	char serial[25];
	char const *text = 0;
	uint8_t count;
	if(index == 0) {
		desc[1] = 0x0409;
		count = 1;
	} else {
		if(index == 1) text = "HelloWorks";
		else if(index == 2) text = "TinyRoller STM32G4";
		else if(index == 3) {
			static char const hex[] = "0123456789ABCDEF";
			uint32_t const *uid = (uint32_t const *)UID_BASE;
			for(uint8_t i = 0; i < 12; i++) {
				uint8_t byte = (uint8_t)(uid[i / 4] >> ((i & 3u) * 8u));
				serial[i * 2] = hex[byte >> 4];
				serial[i * 2 + 1] = hex[byte & 0x0f];
			}
			serial[24] = 0;
			text = serial;
		} else if(index == 4) text = "TinyRoller Vendor";
		else return 0;
		for(count = 0; text[count] && count < 32; count++) desc[1 + count] = (uint8_t)text[count];
	}
	desc[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * count + 2));
	return desc;
}
