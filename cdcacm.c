/************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *   Copyright (c) 2010 libopencm3 project (Gareth McMullin)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * LICENSE NOTE FOR EXTERNAL LIBOPENCM3 LIBRARY:
 *
 *   The PX4 development team considers libopencm3 to be
 *   still GPL, not LGPL licensed, as it is unclear if
 *   each and every author agreed to the LGPS -> GPL change.
 *
 ***********************************************************************/

/**
 * @file cdcacm.c
 * @author Gareth McMullin <gareth@blacksphere.co.nz>
 */

#include <stdlib.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>

#include "bl.h"


/* Provide the stings for the Index 1-n as a requested index of 0 is used for the supported langages
 *  and is hard coded in the usb lib. The array below is indexed by requested index-1, therefore 
 *  element[0] maps to requested index 1
 */
static const char *usb_strings[] = { 
	"3D Robotics", /* Maps to Index 1 Index */
	USBDEVICESTRING,
	"0",
};
#define NUM_USB_STRINGS (sizeof(usb_strings)/sizeof(usb_strings[0]))

static usbd_device *usbd_dev;

/* Buffer to be used for control requests. */
static uint8_t usbd_control_buffer[128];

static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,	/**< Specifies he descriptor type */
	.bcdUSB = 0x0200,					/**< The USB interface version, binary coded (2.0) */
	.bDeviceClass = USB_CLASS_CDC,		/**< USB device class, CDC in this case */
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x26AC,					/**< Vendor ID (VID) */
	.idProduct = USBPRODUCTID,			/**< Product ID (PID) */
	.bcdDevice = 0x0101,				/**< Product version. Set to 1.01 (0x0101) to agree with NuttX */
	.iManufacturer = 1,					/**< Use string with index 1 for the manufacturer string ("3D Robotics") */
	.iProduct = 2,						/**< Use string with index 2 for the product string (USBDEVICESTRING define) */
	.iSerialNumber = 3,					/**< Use string with index 3 for the serial number string (empty) */
	.bNumConfigurations = 1,			/**< Number of configurations (one) */
};

/*
 * This notification endpoint isn't implemented. According to CDC spec it's
 * optional, but its absence causes a NULL pointer dereference in the
 * Linux cdc_acm driver.
 */
static const struct usb_endpoint_descriptor comm_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x83,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 16,
	.bInterval = 255,
}};

static const struct usb_endpoint_descriptor data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x01,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x82,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}};

static const struct {
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
	.header = {
		.bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
		.bcdCDC = 0x0110,
	},
	.call_mgmt = {
		.bFunctionLength =
			sizeof(struct usb_cdc_call_management_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
		.bmCapabilities = 0,
		.bDataInterface = 1,
	},
	.acm = {
		.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_ACM,
		.bmCapabilities = 0,
	},
	.cdc_union = {
		.bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_UNION,
		.bControlInterface = 0,
		.bSubordinateInterface0 = 1,
	 }
};

static const struct usb_interface_descriptor comm_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_CDC,
	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
	.iInterface = 0,

	.endpoint = comm_endp,

	.extra = &cdcacm_functional_descriptors,
	.extralen = sizeof(cdcacm_functional_descriptors)
}};

static const struct usb_interface_descriptor data_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_DATA,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = data_endp,
}};

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = comm_iface,
}, {
	.num_altsetting = 1,
	.altsetting = data_iface,
}};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 2,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0xFA, /* Request 500 mA power (0xFA=250, get doubled in protocol) */

	.interface = ifaces,
};

static int cdcacm_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
		uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)buf;
	(void)usbd_dev;

	switch (req->bRequest) {
	case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
		/*
		 * This Linux cdc_acm driver requires this to be implemented
		 * even though it's optional in the CDC spec, and we don't
		 * advertise it in the ACM functional descriptor.
		 */
		return 1;
		}
	case USB_CDC_REQ_SET_LINE_CODING:
		if (*len < sizeof(struct usb_cdc_line_coding))
			return 0;

		return 1;
	}
	return 0;
}

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;

	char buf[64];
	unsigned i;
	unsigned len = usbd_ep_read_packet(usbd_dev, 0x01, buf, sizeof(buf));

	for (i = 0; i < len; i++)
		buf_put(buf[i]);
}

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);
	usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				cdcacm_control_request);
}

void cdc_init(void)
{

#if defined(STM32F4)

	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);
	rcc_peripheral_enable_clock(&RCC_AHB2ENR, RCC_AHB2ENR_OTGFSEN);

	/* Configure to use the Alternate IO Functions USB DP,DM and VBUS */

	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO9 | GPIO11 | GPIO12);

	usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config, usb_strings, NUM_USB_STRINGS,
					usbd_control_buffer, sizeof(usbd_control_buffer));

#elif defined(STM32F1)
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
	gpio_set(GPIOA, GPIO8);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);

	usbd_dev = usbd_init(&stm32f103_usb_driver, &dev, &config, usb_strings, NUM_USB_STRINGS,
					usbd_control_buffer, sizeof(usbd_control_buffer));
#endif

	usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);
}

static void cdc_disconnect(void)
{
        if (usbd_dev) {
            usbd_disconnect(usbd_dev, true);
            usbd_dev = NULL;
        }
}

void
otg_fs_isr(void)
{
	usbd_poll(usbd_dev);
}

void
cinit(void *config)
{
	cdc_init();
#if defined(STM32F4)
	nvic_enable_irq(NVIC_OTG_FS_IRQ);
#endif
}

void
cfini()
{
	cdc_disconnect();
#if defined(STM32F4)
	nvic_disable_irq(NVIC_OTG_FS_IRQ);
#endif
}

int
cin(void)
{
#if defined(STM32F1)
	usbd_poll(usbd_dev);
#endif
	return buf_get();
}

void
cout(uint8_t *buf, unsigned count)
{
	while (count) {
		unsigned len = (count > 64) ? 64 : count;
		unsigned sent;

		sent = usbd_ep_write_packet(usbd_dev, 0x82, buf, len);

		count -= sent;
		buf += sent;
	}
}
