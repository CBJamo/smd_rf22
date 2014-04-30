#ifndef _spi_h
#define _spi_h

#include <avr/io.h>

#define PORT_SPI        PORTB
#define PORT_SS         PORTD
#define DDR_SPI         DDRB
#define DDR_SS          DDRD
#define DD_MISO         DDB3
#define DD_MOSI         DDB2
#define DD_SS           DDD7
#define DD_SCK          DDB1

void spi_init( void );
void spi_transfer_sync( const uint8_t *dataout, uint8_t *datain, uint8_t len );
void spi_transmit_sync( const uint8_t *dataout, uint8_t len );
inline uint8_t spi_fast_shift( uint8_t data );

#endif //spi_h
