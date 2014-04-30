#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "usb.h"
#include "usb_spec.h"
#include "pll.h"
#include "eeprom.h"
#include "debug.h"

#define VENDOR				L"SDSM&T CSR"
#define PRODUCT				L"RFM22B Radio"
#define SERIAL				L"0001"

#define VENDOR_ID			0x03EB // Atmel
#define PRODUCT_ID			0x8888 // Arbitrary Unused PID

#define STRID_LANGID			0
#define STRID_MANUFACTURER		1
#define STRID_PRODUCT			2
#define STRID_SERIAL			3

#define EP_TYPE_CONTROL			0x00
#define EP_TYPE_BULK_IN			0x81
#define EP_TYPE_BULK_OUT		0x80
#define EP_TYPE_INTERRUPT_IN		0xC1
#define EP_TYPE_INTERRUPT_OUT		0xC0
#define EP_TYPE_ISOCHRONOUS_IN		0x41
#define EP_TYPE_ISOCHRONOUS_OUT		0x40

#define EP_SINGLE_BUFFER		0x02
#define EP_DOUBLE_BUFFER		0x06
#define EP_SIZE( s )	( ( s ) == 64 ? 0x30 :	\
			( ( s ) == 32 ? 0x20 :	\
			( ( s ) == 16 ? 0x10 :	\
					0x00 ) ) ) // I'm not so sure this matches the data sheet...

#define GET_REGISTER			0x01
#define SET_REGISTER			0x02
#define SET_REGISTER_TEMP		0x03

/*
 * Static Data
 */
static const struct
{
	struct standard_usb_device_descriptor device;
	struct standard_usb_configuration_descriptor config;
	struct standard_usb_interface_descriptor main_iface;
	struct standard_usb_endpoint_descriptor bulk_in_endpt;
	struct standard_usb_endpoint_descriptor bulk_out_endpt;
	struct standard_usb_string_descriptor str_langid;
	int16_t str_langid_bString[2];
	struct standard_usb_string_descriptor str_manufacturer;
	int16_t str_manufacturer_bString[sizeof( VENDOR ) / sizeof( VENDOR[0] )];
	struct standard_usb_string_descriptor str_product;
	int16_t str_product_bString[sizeof( PRODUCT ) / sizeof( PRODUCT[0] )];
	struct standard_usb_string_descriptor str_serial;
	int16_t str_serial_bString[sizeof( SERIAL ) / sizeof( SERIAL[0] )];
} __attribute__((packed)) descriptor =
{
	.device =
	{
		.bLength = sizeof( descriptor.device ),
		.bDescriptorType = USB_DT_DEVICE,
		.bcdUSB = ( 2 << 8 ) | 0, // 2.0 in BCD (we are a full-speed USB 2.0 device)
		.bDeviceClass = USB_CLASS_VENDOR_SPEC,
		.bDeviceSubClass = USB_SUBCLASS_VENDOR_SPEC,
		.bDeviceProtocol = USB_PROTOCOL_VENDOR_SPEC,
		.bMaxPacketSize0 = 64, // Max for USB 2.0 spec
		.idVendor = VENDOR_ID,
		.idProduct = PRODUCT_ID,
		.bcdDevice = ( 1 << 8 ) | 0, // 1.0 in BCD
		.iManufacturer = STRID_MANUFACTURER,
		.iProduct = STRID_PRODUCT,
		.iSerialNumber = STRID_SERIAL,
		.bNumConfigurations = 1,
	},
	.config =
	{
		.bLength = sizeof( descriptor.config ),
		.bDescriptorType = USB_DT_CONFIG,
		.wTotalLength = (uint16_t)&descriptor.str_langid - (uint16_t)&descriptor.config,
		.bNumInterfaces = 1,
		.bConfigurationValue = 1,
		.iConfiguration = 0,
		.bmAttributes = USB_CONFIG_ATT_ONE,
		.bMaxPower = 50, // TODO: max TX power draw over 2
	},
	.main_iface =
	{
		.bLength = sizeof( descriptor.main_iface ),
		.bDescriptorType = USB_DT_INTERFACE,
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		.bNumEndpoints = 2,
		.bInterfaceClass = USB_CLASS_VENDOR_SPEC,
		.bInterfaceSubClass = USB_SUBCLASS_VENDOR_SPEC,
		.bInterfaceProtocol = USB_PROTOCOL_VENDOR_SPEC,
		.iInterface = 0,
	},
	.bulk_in_endpt =
	{
		.bLength = sizeof( descriptor.bulk_in_endpt ),
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 2 | USB_DIR_IN,
		.bmAttributes = USB_ENDPOINT_XFER_BULK,
		.wMaxPacketSize = 64, // Max for USB 2.0 spec
		.bInterval = 255, // Ignored for bulk EP
	},
	.bulk_out_endpt =
	{
		.bLength = sizeof( descriptor.bulk_out_endpt ),
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 3 | USB_DIR_OUT,
		.bmAttributes = USB_ENDPOINT_XFER_BULK,
		.wMaxPacketSize = 64, // Max for USB 2.0 spec
		.bInterval = 255, // Ignored for bulk EP
	},
	.str_langid =
	{
		.bLength = sizeof( descriptor.str_langid_bString ),
		.bDescriptorType = USB_DT_STRING,
	},
	.str_langid_bString = { USB_LANGID_EN_US },
	.str_manufacturer =
	{
		.bLength = sizeof( descriptor.str_manufacturer_bString ),
		.bDescriptorType = USB_DT_STRING,
	},
	.str_manufacturer_bString = VENDOR,
	.str_product =
	{
		.bLength = sizeof( descriptor.str_product_bString ),
		.bDescriptorType = USB_DT_STRING,
	},
	.str_product_bString = PRODUCT,
	.str_serial =
	{
		.bLength = sizeof( descriptor.str_serial_bString ),
		.bDescriptorType = USB_DT_STRING,
	},
	.str_serial_bString = SERIAL,
};

uint8_t usb_packet[USB_PACKET_SIZE];
static volatile uint16_t _usb_packet_pos;
static volatile uint8_t _usb_packet_ready;

/*
 * Static Function Prototypes
 */
static void usb_send_control( uint8_t *dat, uint16_t len );
static uint16_t usb_recv_control( uint8_t *dat );
static void read_reg_default( const uint8_t idx, uint8_t *data, const uint8_t len );
static void write_reg_default( const uint8_t idx, const uint8_t *data, const uint8_t len );
static void write_reg_temp_default( const uint8_t idx, const uint8_t *data, const uint8_t len );

void EP0_ISR( void );
void BULK_IN_ISR( void );
void BULK_OUT_ISR( void );

/*
 * Global Varibles
 */
static volatile uint8_t usb_configuration = 0;
void (* volatile usb_read_reg)(const uint8_t, uint8_t *, const uint8_t) = &read_reg_default;
void (* volatile usb_write_reg)(const uint8_t, const uint8_t *, const uint8_t) = &write_reg_default;
void (* volatile usb_write_reg_temp)(const uint8_t, const uint8_t *, const uint8_t) = &write_reg_temp_default;

/*
 * Function Definitions
 */
void usb_init( void )
{
	usb_configuration = 0;
	_usb_packet_pos = 0;
	_usb_packet_ready = 0;
	UHWCON |= ( 1 << UVREGE ); // Enable USB pad regulator
	USBCON |= ( 1 << USBE ) | ( 1 << FRZCLK ); // Enable USB controller and freeze USB clock
	USBCON |= ( 1 << OTGPADE ); // Enable VBUS pad
	USBCON &= ~( 1 << FRZCLK ); // Un-freeze USB clock
	UDCON &= ~( 1 << DETACH ) & ~( 1 << LSM ); // Attach and enable full-speed mode
	UDIEN = ( 1 << EORSTE ); // Enable interrupt on End of Reset
}

void usb_disconnect( void )
{
	UDIEN &= ~( ( 1 << EORSTE ) | ( 1 << SOFE ) ); // Disable interrupt on End of Reset and Start of Frame
	UDCON |= ( 1 << DETACH ); // Detach device from system
	USBCON |= ( 1 << FRZCLK ); // Freeze USB clock
	USBCON &= ~( 1 << USBE ); // Disable USB controller
	UHWCON &= ~( 1 << UVREGE ); // Disable USB pad regulator
	usb_configuration = 0;
	_usb_packet_pos = 0;
	_usb_packet_ready = 0;
}

ISR(USB_GEN_vect)
{
	const uint8_t intbits = UDINT;
	UDINT = 0;

        if( intbits & ( 1 << EORSTI ) ) // End of Reset Interrupt
	{
		UENUM = 0; // Select endpoint 0
		UECONX = ( 1 << EPEN ); // Enable the endpoint
		UECFG0X = EP_TYPE_CONTROL; // Set Control Type
		UECFG1X = EP_SIZE( descriptor.device.bMaxPacketSize0 ) | EP_SINGLE_BUFFER;
		while( !( UESTA0X & ( 1 << CFGOK ) ) ); // Wait for config OK
		UERST = ( 1 << 0 ); // Reset endpoint 0
		UERST = 0; // Clear the reset

		UEIENX = ( 1 << RXSTPE ); // Enable interrupt on setup receive
		usb_configuration = 0;
        }
}

ISR(USB_COM_vect)
{
	const uint8_t num = UENUM;
	if( UEINT & ( 1 << 0 ) )
	{
		UENUM = 0;
		EP0_ISR( );
	}
	if( UEINT * ( 1 << descriptor.bulk_in_endpt.bEndpointAddress ) )
	{
		UENUM = ( descriptor.bulk_in_endpt.bEndpointAddress & 0x03 );
		BULK_IN_ISR( );
	}
	if( UEINT * ( 1 << descriptor.bulk_out_endpt.bEndpointAddress ) )
	{
		UENUM = ( descriptor.bulk_out_endpt.bEndpointAddress & 0x03 );
		BULK_OUT_ISR( );
	}
	UENUM = num;
}

void EP0_ISR( void )
{
	const uint8_t intbits = UEINTX;

	if( intbits & ( 1 << RXSTPI ) )
	{
		struct usb_control_request req;
		uint8_t *req_raw = (uint8_t *)&req;

		// RXSTPI says that we have a complete, valid setup request.
		// USB spec says this is exactly 8 bytes.
		req_raw[0] = UEDATX;
		req_raw[1] = UEDATX;
		req_raw[2] = UEDATX;
		req_raw[3] = UEDATX;
		req_raw[4] = UEDATX;
		req_raw[5] = UEDATX;
		req_raw[6] = UEDATX;
		req_raw[7] = UEDATX;

		// We have two directions: IN and OUT
		switch( ( req.bmRequestType & USB_DIR_MASK ) )
		{
		case USB_DIR_IN: // Data To Host
			// Ack the setup request and prepare for IN token
			UEINTX &= ~( ( 1 << RXSTPI ) | ( 1 << TXINI ) );

			// We have two types: standard and vendor
			switch( req.bmRequestType & USB_TYPE_MASK )
			{
			case USB_TYPE_STANDARD:
				switch( req.bRequest )
				{
				case GET_DESCRIPTOR:
					switch( req.wValue >> 8 )
					{
						case USB_DT_DEVICE:
							if( USB_LANGID_NONE != req.wIndex || descriptor.device.bLength > req.wLength )
								goto fail;
							usb_send_control( (uint8_t *)&descriptor.device, descriptor.device.bLength );
							break;
						case USB_DT_CONFIG:
							if( USB_LANGID_NONE != req.wIndex )
								goto fail;
							else if( descriptor.config.wTotalLength <= req.wLength )
								usb_send_control( (uint8_t *)&descriptor.config, descriptor.config.wTotalLength );
							else if( descriptor.config.bLength <= req.wLength )
								usb_send_control( (uint8_t *)&descriptor.config, descriptor.config.bLength );
							else
								goto fail;
							break;
						case USB_DT_STRING:
							switch( req.wValue & 0xFF ) // Quintuple-nested switch. Do I get an award?
							{
							case STRID_LANGID:
								if( USB_LANGID_NONE != req.wIndex || descriptor.str_langid.bLength > req.wLength )
									goto fail;
								usb_send_control( (uint8_t *)&descriptor.str_langid, descriptor.str_langid.bLength );
								break;
							case STRID_MANUFACTURER:
								if( USB_LANGID_EN_US != req.wIndex || descriptor.str_manufacturer.bLength > req.wLength )
									goto fail;
								usb_send_control( (uint8_t *)&descriptor.str_manufacturer, descriptor.str_manufacturer.bLength );
								break;
							case STRID_PRODUCT:
								if( USB_LANGID_EN_US != req.wIndex || descriptor.str_product.bLength > req.wLength )
									goto fail;
								usb_send_control( (uint8_t *)&descriptor.str_product, descriptor.str_product.bLength );
								break;
							case STRID_SERIAL:
								if( USB_LANGID_EN_US != req.wIndex || descriptor.str_serial.bLength > req.wLength )
									goto fail;
								usb_send_control( (uint8_t *)&descriptor.str_serial, descriptor.str_serial.bLength );
								break;
							default:
								goto fail;
								break;
							}
							break;
						case USB_DT_DEVICE_QUALIFIER: // Per usb-spec, we STALL because we only support USB 2.0 full-speed
						default:
							goto fail;
							break;
					}
					break;
				case GET_CONFIGURATION:
					if( req.wLength < 1 )
						goto fail;
					usb_send_control( (uint8_t *)&usb_configuration, 1 );
					break;
				case GET_STATUS:
					if( req.wLength < 2 )
						goto fail;
					else
					{
						uint16_t status = 0; // TODO: self-powered, remote wake-up
						usb_send_control( (uint8_t *)&status, 2 );
					}
					break;
				default:
					goto fail;
					break;
				}
				break;
			case USB_TYPE_VENDOR:
				switch( req.bRequest )
				{
				case GET_REGISTER:
					if( req.wLength > 16 )
						goto fail;
					else
					{
						uint8_t data[16] = { 0x42, 0x42, 0x42, 0x42, 0x42 };
						usb_read_reg( req.wValue, data, req.wLength );
						usb_send_control( data, req.wLength );
					}
					break;
				default:
					goto fail;
					break;
				}
				break;
			default:
				goto fail;
				break;
			}
			break;
		case USB_DIR_OUT: // Data From Host
			// Ack the setup request
			UEINTX &= ~( 1 << RXSTPI );

			// We have two types: standard and vendor
			switch( req.bmRequestType & USB_TYPE_MASK )
			{
			case USB_TYPE_STANDARD:
				switch( req.bRequest )
				{
				case SET_ADDRESS:
					if( 0 != usb_recv_control( 0 ) )
						goto fail;
					while( !( UEINTX & ( 1 << TXINI ) ) );
					UDADDR = req.wValue | ( 1 << ADDEN );
					break;
				case SET_CONFIGURATION:
					if( descriptor.config.bConfigurationValue != req.wValue )
						goto fail;
					if( 0 != usb_recv_control( 0 ) )
						goto fail;
					usb_configuration = req.wValue;

					// Configure Bulk In Endpt
					UENUM = ( descriptor.bulk_in_endpt.bEndpointAddress & 0x03 ); // Select endpoint
					UECONX = ( 1 << EPEN ); // Enable endpoint
					UECFG0X = ( descriptor.bulk_in_endpt.bmAttributes << 6 ) | ( descriptor.bulk_in_endpt.bEndpointAddress >> 7 );
					UECFG1X = EP_SIZE( descriptor.bulk_in_endpt.wMaxPacketSize ) | EP_DOUBLE_BUFFER; // Set size/bank config
					while( !( UESTA0X & ( 1 << CFGOK ) ) ); // Wait for config OK
					UERST = ( 1 << ( descriptor.bulk_in_endpt.bEndpointAddress & 0x07 ) ); // Reset endpoint

					// Configure Bulk Out Endpt
					UENUM = ( descriptor.bulk_out_endpt.bEndpointAddress & 0x03 ); // Select endpoint
					UECONX = ( 1 << EPEN ); // Enable endpoint
					UECFG0X = ( descriptor.bulk_out_endpt.bmAttributes << 6 ) | ( descriptor.bulk_out_endpt.bEndpointAddress >> 7 );
					UECFG1X = EP_SIZE( descriptor.bulk_out_endpt.wMaxPacketSize ) | EP_DOUBLE_BUFFER; // Set size/bank config
					UEIENX |= ( 1 << RXOUTE ); // Interrupt when we get a packet
					while( !( UESTA0X & ( 1 << CFGOK ) ) ); // Wait for config OK
					UERST = ( 1 << ( descriptor.bulk_out_endpt.bEndpointAddress & 0x07 ) ); // Reset endpoint

					UERST = 0; // Clear reset

					// USB ready
					PORTD |= ( 1 << 5 );

					break;
				default:
					goto fail;
					break;
				}
				break;
			case USB_TYPE_VENDOR:
				switch( req.bRequest )
				{
				case SET_REGISTER:
					if( req.wLength > 16 )
						goto fail;
					else
					{
						uint8_t data[16];
						if( req.wLength != usb_recv_control( data ) )
							goto fail;
						usb_write_reg( req.wValue, data, req.wLength );
					}
					break;
				case SET_REGISTER_TEMP:
					if( req.wLength > 16 )
						goto fail;
					else
					{
						uint8_t data[16];
						if( req.wLength != usb_recv_control( data ) )
							goto fail;
						usb_write_reg_temp( req.wValue, data, req.wLength );
					}
					break;
				default:
					goto fail;
					break;
				}
				break;
			default:
				goto fail;
				break;
			}
			break;
		default:
		fail:
			UECONX |= ( 1 << STALLRQ );
			break;
		}
	}
}

void BULK_IN_ISR( void )
{
}

void BULK_OUT_ISR( void )
{
	const uint8_t intbits = UEINTX;

	if( intbits & ( 1 << RXOUTI ) )
	{
		uint16_t len = ( UEBCHX << 8 ) | UEBCLX;
		uint16_t i;

		PORTD &= ~( 1 << 5 ); // RX in progress

		UEINTX &= ~( 1 << RXOUTI ); // Ack

		if( _usb_packet_pos + len > USB_PACKET_SIZE )
		{
			// REALLY REALLY BAD!
			PORTB &= ~( 1 << 0 ); // Both LEDs on during fatal error
			dbg_str( "USB PKT ERR" );
			while( 1 );
		}
		for( i = 0; i < len; i++ )
			usb_packet[_usb_packet_pos++] = UEDATX;
		if( len != descriptor.bulk_out_endpt.wMaxPacketSize )
		{
			_usb_packet_ready = 1;
			UEIENX &= ~( 1 << RXOUTE ); // Disable RX interrupt
		}

		UEINTX &= ~( 1 << FIFOCON ); // Release Buffer
	}
}

uint8_t usb_ready( void )
{
	return ( usb_configuration == descriptor.config.bConfigurationValue );
}

static void usb_send_control( uint8_t *dat, uint16_t len )
{
	uint8_t intbits;
	uint8_t n;

	while( 1 )
	{
		// Wait for token
		do
		{
			intbits = UEINTX;
		} while( !( intbits & ( ( 1 << TXINI ) | ( 1 << RXOUTI ) ) ) );

		// If we got an OUT token, host has ACK'd the data
		if( intbits & ( 1 << RXOUTI ) )
			break;

		// send as much IN data as we can
		if( (int16_t)len >= 0 )
		{
			n = len < descriptor.device.bMaxPacketSize0 ? len : descriptor.device.bMaxPacketSize0;
			while( n-- )
				UEDATX = *dat++;
			len -= descriptor.device.bMaxPacketSize0;
		}

		// Send this frame
		UEINTX &= ~( 1 << TXINI );
	}

	UEINTX &= ~( 1 << RXOUTI );
}

static uint16_t usb_recv_control( uint8_t *dat )
{
	uint8_t intbits;
	uint16_t n;
	uint16_t len = 0;

	// This was TOTALLY left out of the data sheet...
	UEINTX &= ~( 1 << NAKINI );

	while( 1 )
	{
		// Wait for OUT token
		do
		{
			intbits = UEINTX;
		} while( !( intbits & (  ( 1 << NAKINI ) | ( 1 << RXOUTI ) ) ) );

		// If we get a NAK, we should transition to status phase
		if( intbits & ( 1 << NAKINI ) )
			break;

		// Get frame length
		n = ( UEBCHX << 8 ) | UEBCLX;
		if( !n )
		{
			// ZLP, we're done
			UEINTX = ~( 1 << RXOUTI );
			break;
		}

		// Read the frame
		len += n;
		while( n-- )
		{
			if( dat )
				*dat++ = UEDATX;
			else
				UEDATX;
		}

		// Ack this frame
		UEINTX = ~( 1 << RXOUTI );
	}

	UEINTX &= ~( 1 << TXINI );

	return len;
}

void usb_send_bulk( uint8_t *dat, uint16_t len )
{
	uint8_t intbits;
	uint8_t n;

	if( !usb_ready( ) )
		return;

	UENUM = ( descriptor.bulk_in_endpt.bEndpointAddress & 0x03 );

	while( (int16_t)len >= 0 )
	{
		// Wait for token
		do
		{
			intbits = UEINTX;
		} while( !( intbits & ( 1 << TXINI ) ) );

		// Ack the token
		UEINTX &= ~( 1 << TXINI );

		// send as much IN data as we can
		n = len < descriptor.bulk_in_endpt.wMaxPacketSize ? len : descriptor.bulk_in_endpt.wMaxPacketSize;
		while( n-- )
			UEDATX = *dat++;
		len -= descriptor.bulk_in_endpt.wMaxPacketSize;

		// Send this frame
		UEINTX &= ~( 1 << FIFOCON );
	}
}

void usb_send_bulk_ring( uint8_t *buff, uint16_t len, uint16_t pos, const uint16_t buff_len )
{
	uint8_t intbits;
	uint8_t n;

	if( !usb_ready( ) )
		return;

	UENUM = ( descriptor.bulk_in_endpt.bEndpointAddress & 0x03 );

	while( (int16_t)len >= 0 )
	{
		// Wait for token
		do
		{
			intbits = UEINTX;
		} while( !( intbits & ( 1 << TXINI ) ) );

		// Ack the token
		UEINTX &= ~( 1 << TXINI );

		// send as much IN data as we can
		n = len < descriptor.bulk_in_endpt.wMaxPacketSize ? len : descriptor.bulk_in_endpt.wMaxPacketSize;
		while( n-- )
		{
			UEDATX = buff[pos++];
			pos %= buff_len;
		}
		len -= descriptor.bulk_in_endpt.wMaxPacketSize;

		// Send this frame
		UEINTX &= ~( 1 << FIFOCON );
	}
}

inline uint8_t usb_packet_ready( void )
{
	return ( usb_ready( ) && _usb_packet_ready );
}

void usb_packet_done( void )
{
	if( !usb_ready( ) )
		return;

	_usb_packet_pos = 0;
	_usb_packet_ready = 0;
	UENUM = ( descriptor.bulk_out_endpt.bEndpointAddress & 0x03 );
	UEIENX |= ( 1 << RXOUTE ); // Interrupt when we get a packet
	PORTD |= ( 1 << 5 );
}

static void read_reg_default( const uint8_t idx, uint8_t *data, const uint8_t len )
{
}

static void write_reg_default( const uint8_t idx, const uint8_t *data, const uint8_t len )
{
}

static void write_reg_temp_default( const uint8_t idx, const uint8_t *data, const uint8_t len )
{
}

