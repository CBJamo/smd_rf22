#ifndef _usb_spec_h
#define _usb_spec_h

/*
 * Data Direction
 */
#define USB_DIR_MASK			( 0x01 << 7 )
#define USB_DIR_OUT			( 0x00 << 7 )
#define USB_DIR_IN			( 0x01 << 7 )

/*
 * Transfer Types
 */
#define USB_TYPE_MASK			( 0x03 << 5 )
#define USB_TYPE_STANDARD		( 0x00 << 5 )
#define USB_TYPE_CLASS			( 0x01 << 5 )
#define USB_TYPE_VENDOR			( 0x02 << 5 )
#define USB_TYPE_RESERVED		( 0x03 << 5 )

/*
 * Standard Descriptors
 */
#define USB_DT_DEVICE                   0x01
#define USB_DT_CONFIG                   0x02
#define USB_DT_STRING                   0x03
#define USB_DT_INTERFACE                0x04
#define USB_DT_ENDPOINT                 0x05
#define USB_DT_DEVICE_QUALIFIER         0x06
#define USB_DT_OTHER_SPEED_CONFIG       0x07
#define USB_DT_INTERFACE_POWER          0x08

/*
 * A few common USB classes
 * Defined by USB-IF
 */
#define USB_CLASS_COMM			0x02
#define USB_CLASS_CDC_DATA		0x0A
#define USB_CLASS_WIRELESS_CONTROLLER	0xE0
#define USB_CLASS_MISC			0xEF
#define USB_CLASS_VENDOR_SPEC		0xFF

#define USB_SUBCLASS_VENDOR_SPEC	0xFF

#define USB_PROTOCOL_VENDOR_SPEC	0xFF

/*
 * bmAttributes
 */
#define USB_CONFIG_ATT_ONE		( 1 << 7 )
#define USB_CONFIG_ATT_SELFPOWER	( 1 << 6 )
#define USB_CONFIG_ATT_WAKEUP		( 1 << 5 )
#define USB_CONFIG_ATT_BATTERY		( 1 << 4 )

#define USB_ENDPOINT_NUMBER_MASK	0x0F
#define USB_ENDPOINT_DIR_MASK		0x80

#define USB_ENDPOINT_XFERTYPE_MASK      0x03
#define USB_ENDPOINT_XFER_CONTROL	0x00
#define USB_ENDPOINT_XFER_ISOC		0x01
#define USB_ENDPOINT_XFER_BULK		0x02
#define USB_ENDPOINT_XFER_INT		0x03
#define USB_ENDPOINT_MAX_ADJUSTABLE	0x80

/*
 * USB Language ID
 * Defined by USB-IF
 */
#define USB_LANGID_NONE			0x0000
#define USB_LANGID_EN_US		0x0409

union standard_usb_descriptor
{
	uint8_t raw[0];
	struct standard_usb_generic_descriptor
	{
		uint8_t bLength;
		uint8_t bDescriptorType;
	} __attribute__((packed)) generic;
	struct standard_usb_device_descriptor
	{
		uint8_t bLength;
		uint8_t bDescriptorType;
		uint16_t bcdUSB;
		uint8_t bDeviceClass;
		uint8_t bDeviceSubClass;
		uint8_t bDeviceProtocol;
		uint8_t bMaxPacketSize0;
		uint16_t idVendor;
		uint16_t idProduct;
		uint16_t bcdDevice;
		uint8_t iManufacturer;
		uint8_t iProduct;
		uint8_t iSerialNumber;
		uint8_t bNumConfigurations;
	} __attribute__((packed)) device;
	struct standard_usb_device_qualifier_descriptor
	{
		uint8_t bLength;
		uint8_t bDescriptorType;
		uint16_t bcdUSB;
		uint8_t bDeviceClass;
		uint8_t bDeviceSubClass;
		uint8_t bDeviceProtocol;
		uint8_t bMaxPacketSize0;
		uint8_t bNumConfigurations;
		uint8_t bReserved;
	} __attribute__((packed)) device_qualifier;
	struct standard_usb_configuration_descriptor
	{
		uint8_t bLength;
		uint8_t bDescriptorType;
		uint16_t wTotalLength;
		uint8_t bNumInterfaces;
		uint8_t bConfigurationValue;
		uint8_t iConfiguration;
		uint8_t bmAttributes;
		uint8_t bMaxPower;
	} __attribute__((packed)) configuration;
	struct standard_usb_interface_descriptor
	{
		uint8_t bLength;
		uint8_t bDescriptorType;
		uint8_t bInterfaceNumber;
		uint8_t bAlternateSetting;
		uint8_t bNumEndpoints;
		uint8_t bInterfaceClass;
		uint8_t bInterfaceSubClass;
		uint8_t bInterfaceProtocol;
		uint8_t iInterface;
	} __attribute__((packed)) interface;
	struct standard_usb_endpoint_descriptor
	{
		uint8_t bLength;
		uint8_t bDescriptorType;
		uint8_t bEndpointAddress;
		uint8_t bmAttributes;
		uint16_t wMaxPacketSize;
		uint8_t bInterval;
	} __attribute__((packed)) endpoint;
	struct standard_usb_string_descriptor
	{
		uint8_t bLength;
		uint8_t bDescriptorType;
		int16_t bString[];
	} __attribute__((packed)) string;
};

struct usb_control_request
{
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
} __attribute__((packed));

enum bRequestTypes
{
	GET_STATUS = 0x00,
	CLEAR_FEATURE = 0x01,
	SET_FEATURE = 0x03,
	SET_ADDRESS = 0x05,
	GET_DESCRIPTOR = 0x06,
	SET_DESCRIPTOR = 0x07,
	GET_CONFIGURATION = 0x08,
	SET_CONFIGURATION = 0x09,
	GET_INTERFACE = 0x0A,
	SET_INTERFACE = 0x11,
	SYNCH_FRAME = 0x12,
};

#endif /* _usb_spec_h */
