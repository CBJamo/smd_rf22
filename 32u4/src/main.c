#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "usb.h"
#include "pll.h"
#include "spi.h"
#include "rf22.h"
#include "debug.h"

#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))

int main( void )
{
	CPU_PRESCALE( 0 );

	// Both LEDs are outputs
	DDRB |= ( 1 << 0 );
	DDRD |= ( 1 << 5 );
	DDRF |= ( 1 << 7 );

	// Both LEDs ON
	PORTB &= ~( 1 << 0 );
	PORTD &= ~( 1 << 5 );
	PORTF &= ~( 1 << 7 );

	// Reset
	usb_disconnect( );
	pll_stop( );

	// Setup callbacks
	usb_read_reg = &rf22_read_reg;
	usb_write_reg = &rf22_write_reg;
	usb_write_reg_temp = &rf22_write_reg_temp;

	// Initialize the things
	pll_start( );
	usb_init( );
	spi_init( );
	rf22_init( );

	PORTF |= ( 1 << 7 );

	// Enable interrupts (buffers will now fill)
	sei( );

	while( 1 )
	{
		if( usb_packet_ready( ) )
		{
			switch( rf22_send_bulk( usb_packet ) )
			{
			case 0: // Finished
				usb_packet_done( );
				//_delay_ms( 30 );
			case -1: // In Progress
			case -2: // RX Busy
			default:
				break;
			}
		}
		if( rf22_packet_ready( ) )
		{
			usb_send_bulk_ring( (uint8_t *)rf22_ring_buff, rf22_ring_buff[( rf22_ring_buff_head + 4 ) % RING_BUFF_SIZE] + 5, rf22_ring_buff_head, RING_BUFF_SIZE );
			rf22_packet_done( );
		}
	}

	return 0;
}

