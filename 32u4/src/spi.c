#include <avr/io.h>

#include "spi.h"

void spi_init( void )
// Initialize pins for spi communication
{
	// Define the following pins as output
	DDR_SPI |= ( ( 1 << DD_MOSI ) | ( 1 << DD_SCK ) );
	DDR_SS |= ( 1 << DD_SS );

	// Initially raise the SS pin
	PORT_SS |= ( 1 << DD_SS );

	SPCR = ( ( 1 << SPE ) |		// SPI Enable
		( 0 << SPIE ) |		// SPI Interupt Enable
		( 0 << DORD ) |		// Data Order (0:MSB first / 1:LSB first)
		( 1 << MSTR ) |		// Master/Slave select
		( 0 << SPR1 ) | ( 0 << SPR0 ) |	// SPI Clock Rate
		( 0 << CPOL ) |		// Clock Polarity (0:SCK low / 1:SCK hi when idle)
		( 0 << CPHA ) );	// Clock Phase (0:leading / 1:trailing edge sampling)

	SPSR = ( 1 << SPI2X );		// Double Clock Rate
}

void spi_transfer_sync( const uint8_t *dataout, uint8_t *datain, uint8_t len )
// Shift full array through target device
{
	PORT_SS &= ~( 1 << DD_SS );
	while( len-- )
	{
		SPDR = *dataout++;
		while( !( SPSR & ( 1 << SPIF ) ) );
		*datain++ = SPDR;
	}
	PORT_SS |= ( 1 << DD_SS );
}

void spi_transmit_sync( const uint8_t *dataout, uint8_t len )
// Shift full array to target device without receiving any byte
{
	PORT_SS &= ~( 1 << DD_SS );
	while( len-- )
	{
		SPDR = *dataout++;
		while( !( SPSR & ( 1 << SPIF ) ) );
	}
	PORT_SS |= ( 1 << DD_SS );
}

inline uint8_t spi_fast_shift( const uint8_t data )
// Clocks only one byte to target device and returns the received one
{
	SPDR = data;
	while( !( SPSR & ( 1 << SPIF ) ) );
	return SPDR;
}

