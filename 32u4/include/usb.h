#ifndef _usb_h
#define _usb_h

#define USB_PACKET_SIZE 260

void usb_init( void );
void usb_disconnect( void );
uint8_t usb_ready( void );

/*
 * for uC->PC
 *
 * Should never be called in an interrupt
 */
void usb_send_bulk( uint8_t *dat, uint16_t len );
void usb_send_bulk_ring( uint8_t *buff, uint16_t len, uint16_t pos, const uint16_t buff_len );

/*
 * For PC->uC
 *
 * Should never be called in an interrupt
 */
inline uint8_t usb_packet_ready( void );
void usb_packet_done( void );
extern uint8_t usb_packet[USB_PACKET_SIZE];

/*
 * Callbacks for register changes
 */
extern void (* volatile usb_read_reg)( const uint8_t, uint8_t *, const uint8_t );
extern void (* volatile usb_write_reg)( const uint8_t, const uint8_t *, const uint8_t );
extern void (* volatile usb_write_reg_temp)( const uint8_t, const uint8_t *, const uint8_t );

#endif /* _usb_h */
