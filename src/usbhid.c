#include <libopencm3/cm3/scb.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/usb/usbstd.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>


enum
{
	USB_HID_IN_ENDPOINT_ADDRESS	= 0x81,
	USB_HID_OUT_ENDPOINT_ADDRESS	= 0x1,
	USB_HID_PACKET_SIZE		= 64,
	USB_HID_POLLING_INTERVAL_MS	= 64,
	USB_HID_INTERFACE_NUMBER	= 0,
};


/* usb descriptors */

static const struct usb_device_descriptor usb_device_descriptor =
{
	.bLength		=	USB_DT_DEVICE_SIZE,
	.bDescriptorType	=	USB_DT_DEVICE,
	.bcdUSB			=	0x200,
	.bDeviceClass		=	0,
	.bDeviceSubClass	=	0,
	.bDeviceProtocol	=	0,
	.bMaxPacketSize0	=	64,
	.idVendor		=	0x1ad4,
	.idProduct		=	0xa000,
	/* if this is not 0x0100, some debuggers fail to identify the debug probe
	 * as a cmsis-dap compliant debug probe */
	.bcdDevice		=	0x0100,
	.iManufacturer		=	1,
	.iProduct		=	2,
	.iSerialNumber		=	0,
	.bNumConfigurations	=	1,
};


static const struct usb_endpoint_descriptor usb_hid_endpoints[] = 
{
	{
		.bLength			=	USB_DT_ENDPOINT_SIZE,
		.bDescriptorType		=	USB_DT_ENDPOINT,
		.bEndpointAddress		=	USB_HID_IN_ENDPOINT_ADDRESS,
		.bmAttributes			=	USB_ENDPOINT_ATTR_INTERRUPT,
		.wMaxPacketSize			=	USB_HID_PACKET_SIZE,
		.bInterval			=	USB_HID_POLLING_INTERVAL_MS,
	},
	{
		.bLength			=	USB_DT_ENDPOINT_SIZE,
		.bDescriptorType		=	USB_DT_ENDPOINT,
		.bEndpointAddress		=	USB_HID_OUT_ENDPOINT_ADDRESS,
		.bmAttributes			=	USB_ENDPOINT_ATTR_INTERRUPT,
		.wMaxPacketSize			=	USB_HID_PACKET_SIZE,
		.bInterval			=	USB_HID_POLLING_INTERVAL_MS,
	},
};

static const uint8_t usb_hid_report_descriptor[] =
{
	0x06, 0x00, 0xFF,  // Usage Page (Vendor Defined 0xFF00)
	0x09, 0x01,        // Usage (0x01)
	0xA1, 0x01,        // Collection (Application)
	0x15, 0x00,        //   Logical Minimum (0)
	0x26, 0xFF, 0x00,  //   Logical Maximum (255)
	0x75, 0x08,        //   Report Size (8)
	0x95, 0x40,        //   Report Count (64)
	0x09, 0x01,        //   Usage (0x01)
	0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
	0x95, 0x40,        //   Report Count (64)
	0x09, 0x01,        //   Usage (0x01)
	0x91, 0x02,        //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
	0x95, 0x01,        //   Report Count (1)
	0x09, 0x01,        //   Usage (0x01)
	0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
	0xC0,              // End Collection
};

static const struct __attribute__((packed))
{
	struct	usb_hid_descriptor	h;
	struct __attribute__((packed))
	{
		uint8_t		bDescriptorType;
		uint16_t	wDescriptorLength;
	};
}
usb_hid_descriptor = 
{
	.h =
	{
		.bLength		=	sizeof usb_hid_descriptor,
		.bDescriptorType	=	USB_DT_HID,
		.bcdHID			=	0x100,
		.bCountryCode		=	0,
		.bNumDescriptors	=	1,	/* just 1 report descriptor */
	},
	{
		.bDescriptorType	=	USB_DT_REPORT,
		.wDescriptorLength	=	sizeof usb_hid_report_descriptor,
	},
};


static const struct usb_interface_descriptor hid_interface =
{
	.bLength		=	USB_DT_INTERFACE_SIZE,
	.bDescriptorType	=	USB_DT_INTERFACE,
	.bInterfaceNumber	=	USB_HID_INTERFACE_NUMBER,
	.bAlternateSetting	=	0,
	.bNumEndpoints		=	2,
	.bInterfaceClass	=	USB_CLASS_HID,
	.bInterfaceSubClass	=	0,
	.bInterfaceProtocol	=	0,
	.iInterface		=	0,
	.endpoint		=	usb_hid_endpoints,
	.extra			=	& usb_hid_descriptor,
	.extralen		=	sizeof usb_hid_descriptor,
};


static const struct usb_interface usb_interfaces[] =
{
	{
		.num_altsetting	=	1,
		.altsetting	=	& hid_interface,
	},
};

static const struct usb_config_descriptor usb_config_descriptor =
{
	.bLength		=	USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType	=	USB_DT_CONFIGURATION,
	/* the wTotalLength field below is automatically computed on-the-fly
	 * by the libopencm3 library before sending the configuration descriptor
	 * to the host; it is not updated here, so this data structure can be
	 * defined as 'const' */
	/* .wTotalLength	= xxx*/
	.bNumInterfaces		=	1,
	.bConfigurationValue	=	1,
	.iConfiguration		=	0,
	.bmAttributes		=	USB_CONFIG_ATTR_DEFAULT,
	.bMaxPower		=	50,	/* in 2 mA units */
	.interface		=	usb_interfaces,
};

static const char * usb_strings[] =
{
	"shopov instruments",
	"vx CMSIS-DAP debug probe",
};
static uint8_t usb_control_buffer[128];


static int usbd_hid_control_callback(usbd_device *usbd_dev,
		struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
		usbd_control_complete_callback *complete)
{
	if (req->bmRequestType != (USB_REQ_TYPE_IN | USB_REQ_TYPE_INTERFACE)
			|| req->bRequest != USB_REQ_GET_DESCRIPTOR
			|| req->wValue != (USB_DT_REPORT << 8)
			|| req->wIndex != USB_HID_INTERFACE_NUMBER)
		return USBD_REQ_NEXT_CALLBACK;

	buf[0] = (uint8_t *) usb_hid_report_descriptor;
	len[0] = sizeof usb_hid_report_descriptor;
	return USBD_REQ_HANDLED;
}

static void usbd_hid_set_config_callback(usbd_device * usbd_dev, uint16_t wValue)
{
	usbd_ep_setup(usbd_dev, USB_HID_IN_ENDPOINT_ADDRESS, USB_ENDPOINT_ATTR_INTERRUPT, USB_HID_PACKET_SIZE, 0);
	usbd_ep_setup(usbd_dev, USB_HID_OUT_ENDPOINT_ADDRESS, USB_ENDPOINT_ATTR_INTERRUPT, USB_HID_PACKET_SIZE, 0);
	usbd_register_control_callback(usbd_dev,
			USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
			USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
			usbd_hid_control_callback);
}

int main(void)
{
	usbd_device *usbd_dev;
	SCB_VTOR = 0x3000;
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
	usbd_dev = usbd_init(& st_usbfs_v1_usb_driver, & usb_device_descriptor, & usb_config_descriptor,
			usb_strings, sizeof usb_strings / sizeof * usb_strings,
			usb_control_buffer, sizeof usb_control_buffer);
	usbd_register_set_config_callback(usbd_dev, usbd_hid_set_config_callback);
	while (1)
		usbd_poll(usbd_dev);
}

